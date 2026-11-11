"""Stop times and distances of the owner's automatic runs, from a session's wheel journal (velocity_channel.md 7.3).

Reads <session>/sources/wheel_feedback.jsonl (or <session>/wheel_feedback.jsonl) and
<session>/configuration/manual_runtime.json, during a recording or afterwards. Every AUTO run of the wheel owner is
one trial: the commanded and the measured speed while steady, for the axle and for each wheel (the speed conversion
check: commanded 0.10 m/s should read back as 0.10 m/s on both wheels), then how it stopped:

- software_zero: the command ramped down while the owner stayed in AUTO (the source sent zero);
- source_lost: the owner left AUTO because its bridge or test source went away (kill -9) or fell silent;
- estop: the wheels slowed while still commanded and the owner then reported WHEELS_STOPPED_WHILE_COMMANDED (the
  vehicle's emergency stop acts on the driver); the stop counts from the first feedback below 90 % of the steady
  speed;
- feedback_lost: the wheel feedback ended while moving (USB adapter pulled, or the owner killed): stop time and
  distance are unknown here and come from the video;
- operator, or the owner's own stop reason otherwise.

Stop time runs from the trigger to the first feedback sample with both wheels at zero. Distance and heading change
come from the owner's wheel preview (axle distance and yaw it integrated), interpolated at the trigger. Values
come from the 10 Hz wheel feedback: about 0.1 s and, at 0.10 m/s, about 1 cm resolution.
"""
import argparse
import datetime
import json
import math
from pathlib import Path
import re

SCHEMA = 'wc_auto_stop_report_v1'
SLOWING = 0.9            # estop trigger: feedback below this share of the steady speed
STEADY_WINDOW_S = 1.0    # the steady speed is measured over the last second of the command plateau
SOURCE_LOST = ('BRIDGE_DISCONNECTED', 'DECISION_STALE')
OPERATOR = ('USER_STOPPED_AUTO', 'USER_STOP', 'MANUAL_TAKEOVER', 'UI_NOT_FRESH', 'UI_CLOSED', 'SOFTWARE_ESTOP',
            'USER_HAND_PUSH_REQUEST')
LABEL = re.compile(r'[A-Za-z0-9_.-]{1,64}')


def session_files(session):
    session = Path(session)
    for journal in (session / 'sources/wheel_feedback.jsonl', session / 'wheel_feedback.jsonl'):
        if journal.is_file():
            return journal, session / 'configuration/manual_runtime.json'
    raise FileNotFoundError('no wheel_feedback.jsonl in %s or its sources/' % session)


def read_journal(path):
    """The records of a wheel journal; a line still being written at the end is skipped."""
    records = []
    with Path(path).open(encoding='utf-8') as stream:
        for line in stream:
            try:
                records.append(json.loads(line))
            except ValueError:
                if line.endswith('\n'):
                    raise
    return records


def command_wheels(request_hex, wheel, scale):
    """(left, right) wheel speeds, forward positive, of one FC10 2088h speed write (inverse of the owner's encoding)."""
    data = bytes.fromhex(request_hex)
    words = [int.from_bytes(data[7 + 2 * i:9 + 2 * i], 'big', signed=True) for i in range(2)]
    return tuple(words[i] / (wheel[sign] * scale) * 2 * math.pi * wheel['wheel_radius_m'] / 60
                 for i, sign in enumerate(('left_sign', 'right_sign')))


def command_linear(request_hex, wheel, scale):
    """Axle speed of one FC10 2088h speed write."""
    return sum(command_wheels(request_hex, wheel, scale)) / 2


def timeline(records, wheel, scale):
    feedback, writes, states, failures = [], [], [], []
    rpm_to_m_s = 2 * math.pi * wheel['wheel_radius_m'] / 60
    for record in records:
        event = record.get('event')
        if event == 'history_preview':
            feedback.append({'t': record['receive_monotonic_ns'] * 1e-9, 'wall_ns': record.get('stamp_ns'),
                             'v': record['linear_velocity_m_s'], 'd': record['signed_distance_m'],
                             'yaw': record['yaw_rad'],
                             'left': record['left_wheel_rpm_candidate'] * rpm_to_m_s,
                             'right': record['right_wheel_rpm_candidate'] * rpm_to_m_s,
                             'zero': record['left_raw_signed'] == 0 and record['right_raw_signed'] == 0})
        elif event == 'wheel_io_complete' and record.get('request_hex', '')[2:4] == '10':
            left, right = command_wheels(record['request_hex'], wheel, scale)
            writes.append((record['exchange_started_monotonic_ns'] * 1e-9, (left + right) / 2, (left, right)))
        elif event == 'auto_state':
            states.append((record['journal_enqueued_monotonic_ns'] * 1e-9, record['state'], record.get('reason'),
                           record.get('immediate_reason')))
        elif event in ('wheel_io_failed', 'manual_fault', 'capture_failed'):
            failures.append((record.get('journal_enqueued_monotonic_ns', 0) * 1e-9,
                             record.get('error') or record.get('reason')))
    return feedback, writes, states, failures


def _interpolate(samples, t, key):
    before = [s for s in samples if s['t'] <= t]
    after = [s for s in samples if s['t'] >= t]
    if not before:
        return samples[0][key] if samples else None
    if not after:
        return before[-1][key]
    a, b = before[-1], after[0]
    if b['t'] == a['t']:
        return a[key]
    return a[key] + (b[key] - a[key]) * (t - a['t']) / (b['t'] - a['t'])


def _runs(states):
    """(start, end, end state, end reason) of every AUTO run; end None while still in AUTO."""
    runs, start = [], None
    for t, state, reason, immediate in states:
        if state == 'AUTO' and start is None:
            start = t
        elif state != 'AUTO' and start is not None:
            runs.append((start, t, state, immediate or reason))
            start = None
    if start is not None:
        runs.append((start, None, None, None))
    return runs


def _median(values):
    values = sorted(values)
    return values[len(values) // 2] if values else None


def trial(feedback, writes, failures, run):
    start, end, end_state, end_reason = run
    commands = [(t, v, w) for t, v, w in writes if t >= start and (end is None or t <= end)]   # written in AUTO
    plateau = max((v for _, v, _ in commands), default=0.0)
    row = {'auto_start_mono_s': round(start, 3), 'commanded_m_s': round(plateau, 4), 'kind': None,
           'owner_exit_reason': end_reason}
    if plateau <= 0:
        row['kind'] = 'never_moved'
        return row
    reached, wheels = next((t, w) for t, v, w in commands if v >= plateau - 1e-6)
    ramp_down = next((t for t, v, _ in commands if t > reached and v < plateau - 1e-6), None)
    moving = [s for s in feedback if s['t'] >= reached + 0.3]   # after the ramp and the driver's own lag
    failure = next(((t, reason) for t, reason in failures if t >= reached and (end is None or t <= end)), None)
    if ramp_down is not None and (end is None or ramp_down < end):
        kind, trigger = 'software_zero', ramp_down
    elif end is not None and end_reason in SOURCE_LOST:
        kind, trigger = 'source_lost', end
    elif end is not None and end_reason == 'WHEELS_STOPPED_WHILE_COMMANDED':
        kind = 'estop'
        base = _median([s['v'] for s in moving if s['t'] <= end and s['v'] >= 0.5 * plateau])
        trigger = next((s['t'] for s in moving if base and s['v'] < SLOWING * base), end)
    elif failure is not None:
        # The serial line failed while in AUTO (USB adapter pulled): the owner faults without a state record.
        kind = 'feedback_lost'
        last = [s['t'] for s in moving if s['t'] <= failure[0]]
        trigger = last[-1] if last else reached
        row['failure'] = failure[1]
    elif end is not None:
        kind, trigger = ('operator' if end_reason in OPERATOR else str(end_reason)), end
    else:
        row['kind'] = 'running'
        return row
    steady = [s for s in moving if trigger - STEADY_WINDOW_S <= s['t'] < trigger]
    measured = _median([s['v'] for s in steady])
    row.update(kind=kind, measured_m_s=None if measured is None else round(measured, 4),
               speed_ratio=None if not measured else round(measured / plateau, 3))
    for side, command in zip(('left', 'right'), wheels):
        # One wheel at 80 % and the other at 120 % averages to 100 % on the axle (second review, item 6).
        value = _median([s[side] for s in steady])
        row[side + '_ratio'] = None if value is None or abs(command) < 1e-6 else round(value / command, 3)
    stop = None if kind == 'feedback_lost' else next((s for s in feedback if s['t'] >= trigger and s['zero']), None)
    row['trigger_mono_s'] = round(trigger, 3)
    wall = next((s['wall_ns'] for s in feedback if s['t'] >= trigger and s.get('wall_ns')), None)
    if wall:
        row['trigger_local_time'] = datetime.datetime.fromtimestamp(wall * 1e-9).strftime('%H:%M:%S.%f')[:-3]
    if kind == 'feedback_lost' or stop is None:
        row['stop'] = 'UNKNOWN: no zero feedback after the trigger (use the video)'
        return row
    turn = _interpolate(feedback, trigger, 'yaw')
    row.update(stop_time_s=round(stop['t'] - trigger, 3),
               stop_distance_m=round(stop['d'] - _interpolate(feedback, trigger, 'd'), 4),
               heading_change_deg=round(math.degrees((stop['yaw'] - turn + math.pi) % (2 * math.pi) - math.pi), 2))
    return row


def report(session, *, since_mono_s=None):
    journal, config_path = session_files(session)
    config = json.loads(config_path.read_text(encoding='utf-8'))
    wheel = config['wheel_candidate']
    scale = config['manual_controls'].get('command_rpm_to_register_scale', 8.9)
    feedback, writes, states, failures = timeline(read_journal(journal), wheel, scale)
    runs = [run for run in _runs(states) if since_mono_s is None or run[0] >= since_mono_s]
    return {'schema': SCHEMA, 'session': str(Path(session)), 'journal': str(journal),
            'session_id': config.get('session_id'), 'wheel_candidate': {key: wheel.get(key) for key in (
                'wheel_radius_m', 'track_width_m', 'register_to_wheel_rpm', 'left_sign', 'right_sign')},
            'command_rpm_to_register_scale': scale, 'trials': [trial(feedback, writes, failures, run) for run in runs],
            'resolution': 'wheel feedback at about 10 Hz: about 0.1 s and, at 0.10 m/s, about 1 cm'}


def lines(result):
    out = []
    for index, row in enumerate(result['trials'], 1):
        text = '%d. %s  指令 %.3f m/s' % (index, row['kind'], row['commanded_m_s'])
        if row.get('measured_m_s') is not None:
            text += '，反馈 %.3f m/s（%.0f%%' % (row['measured_m_s'], 100 * row['speed_ratio'])
            if row.get('left_ratio') is not None and row.get('right_ratio') is not None:
                text += '，左轮 %.0f%%、右轮 %.0f%%' % (100 * row['left_ratio'], 100 * row['right_ratio'])
            text += '）'
        if row.get('trigger_local_time'):
            text += '，%s 起停' % row['trigger_local_time']
        if 'stop_time_s' in row:
            text += '，停车 %.2f s、%.3f m、航向变 %.1f°' % (row['stop_time_s'], row['stop_distance_m'],
                                                    row['heading_change_deg'])
        elif row.get('stop'):
            text += '，停车时间和距离不知道：看视频'
        if row.get('owner_exit_reason'):
            text += '（轮控所有者：%s）' % row['owner_exit_reason']
        out.append(text)
    return out or ['没有自动运行']


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--session', type=Path, help='recording or live session directory; default: the newest '
                        'recording of --project-root still in the capture staging area')
    parser.add_argument('--last', type=int, default=0, help='only the last N trials')
    parser.add_argument('--label', help='also write reports/auto_stop_tests/<UTC>_<label>/report.json')
    parser.add_argument('--project-root', type=Path, help='the checkout: finds the session, and its storage policy '
                        'places --label reports')
    args = parser.parse_args(argv)
    session = args.session
    if session is None:
        if args.project_root is None:
            raise SystemExit('give --session, or --project-root for the newest recording')
        from wc_runtime.mapping_auto import recording_sessions
        sessions = recording_sessions(args.project_root)
        if not sessions:
            raise SystemExit('no recording of %s in the capture staging area: give --session <recording directory>'
                             % args.project_root)
        session = sessions[0][0]
        print('session %s (%s)' % (session, sessions[0][2]), flush=True)
    result = report(session)
    if args.last > 0:
        result['trials'] = result['trials'][-args.last:]
    print('\n'.join(lines(result)), flush=True)
    if args.label:
        if not LABEL.fullmatch(args.label) or args.project_root is None:
            raise SystemExit('--label needs 1-64 of A-Z a-z 0-9 _ . - and --project-root')
        print(write_report(args.project_root, args.label, result), flush=True)
    return 0


def write_report(project_root, label, result):
    from wc_runtime.storage_policy import StoragePolicy
    policy = StoragePolicy(project_root)
    policy.check()
    stamp = datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%SZ')
    folder = Path(policy.resolve('reports/auto_stop_tests/%s_%s' % (stamp, label)))
    folder.mkdir(parents=True, exist_ok=False)
    path = folder / 'report.json'
    path.write_text(json.dumps(result, indent=1, ensure_ascii=False, allow_nan=False) + '\n', encoding='utf-8')
    return path


if __name__ == '__main__':
    raise SystemExit(main())
