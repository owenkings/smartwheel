"""Reviewed evidence for the automatic velocity channel (docs/architecture/velocity_channel.md, section 8).

The file records what was measured on this chair before automatic control may run at a stage, and the speed limits
the user approved for that stage. It is a human-reviewed record, not a manufacturer attestation: the hashes only
stop a different revision of the evidence being used by accident. ``load_evidence`` rejects anything incomplete;
the template in ``config/wheel_auto_drive_evidence.template.json`` must always be rejected.

Speeds go up one step at a time (user decision of 2026-11-10): linear 0.10, 0.20, 0.30 m/s and angular 0.30, 0.40,
0.50 rad/s. On the stand and in the first runs of each stage the first step is the limit; the next linear step needs
braking rows (ground test) or a passed supervised run (closed loop) at the step below.

``hold_to_run_switch`` records whether the hold-to-run switch is installed. Not installed is the user's decision of
2026-11-09 23:50 (the first empty-chair runs use the vehicle's emergency stop with a person beside the chair); it has
to be written out, and then the ``dead_man`` stop test and braking path are not required.

What each stage needs follows the coordinator's plan of 2026-11-10 14:40 for the first empty-chair closed loop at
0.10 m/s (velocity_channel.md 8.1):

- STAND_TEST, before automatic control runs on the stand: with the existing manual channel, the emergency stop (and
  no wheel motion after a reset with zero commanded) and the USB adapter pulled, 3 trials each.
- GROUND_TEST adds ``auto_stand_checks``: the test command source at 0.10 m/s on the stand, with the wheel feedback
  within 10 % of the command in every trial (the speed conversion), and at least one emergency stop during
  automatic control after which the wheels did not move on the reset (``estop_trials``, ``moved_after_reset``).
- SUPERVISED_CLOSED_LOOP adds braking rows on the ground at the approved top speed, 3 trials each: software zero,
  test source killed, emergency stop (again with ``moved_after_reset`` false), and USB adapter pulled.

A driver that does not stop by itself when the USB adapter is pulled may be recorded as such (``driver_stopped``
false) with the decision, by the user or the coordinator, only while the approved linear limit is 0.10 m/s: the
person beside the chair presses the emergency stop (coordinator 2026-11-10 14:40). Then the communication-loss
braking row is not required. Up to the same speed ``communication_timeout_s`` may be null (not known, for example
when 2000h cannot be read); then a driver that stopped by itself must have stopped within 1.5 s, a later stop counts
as not stopped (second review of 2026-11-10, item 6). Above that speed the timeout is needed and every line-loss
stop must come within it plus 0.5 s.
The killed owner and the cut Orin power are optional now; when recorded as verified they are checked as before.
"""
import hashlib
import json
import math
from pathlib import Path
import re
import struct

from .protocol import FeedbackError, checked_frame, read_request

EVIDENCE_SCHEMA = 'wc_auto_drive_evidence_v1'
STAGES = ('STAND_TEST', 'GROUND_TEST', 'SUPERVISED_CLOSED_LOOP')
# Section 5: no layer of the chain may exceed these, whatever a file says.
HARD_LIMITS = {'max_linear_m_s': 0.30, 'max_angular_rad_s': 0.50, 'max_wheel_rpm': 27.0,
               'max_linear_accel_m_s2': 0.30, 'max_linear_decel_m_s2': 0.60, 'max_angular_accel_rad_s2': 0.80}
LINEAR_STEPS_M_S = (0.10, 0.20, 0.30)
ANGULAR_STEPS_RAD_S = (0.30, 0.40, 0.50)
STOP_TESTS = ('estop', 'dead_man', 'owner_killed', 'usb_unplugged', 'orin_power_cut', 'software_zero',
              'source_killed', 'auto_stand_checks')
LOSS_TESTS = ('owner_killed', 'usb_unplugged', 'orin_power_cut')   # stop by the driver's communication timeout
LOSS_STOP_MARGIN_S = 0.5   # section 8.1, level 2: stopped within the communication timeout plus 0.5 s
UNKNOWN_TIMEOUT_STOP_S = 1.5   # communication timeout not known: a later stop counts as the driver not stopping
REQUIRED_STOP_TESTS = {
    # The existing manual channel on the stand (WASD at 0.10 m/s), before the automatic channel runs.
    'STAND_TEST': ('estop', 'dead_man', 'usb_unplugged'),
    # The test command source at 0.10 m/s on the stand.
    'GROUND_TEST': ('estop', 'dead_man', 'usb_unplugged', 'auto_stand_checks'),
    # On the ground the braking rows carry the trials.
    'SUPERVISED_CLOSED_LOOP': ('estop', 'dead_man', 'usb_unplugged', 'auto_stand_checks')}
# Up to this approved speed a driver that ignores the lost line, or an unknown communication timeout, is accepted.
NOT_STOPPED_MAX_LINEAR_M_S = 0.10
DECIDED_BY = ('user', 'coordinator')
SPEED_RATIO_TOLERANCE = 0.10        # wheel feedback within 10 % of the command on the stand
# source_killed: the owner brakes in software when its source is gone, as for software zero.
BRAKING_LIMITS_M = {'software_zero': 0.18, 'source_killed': 0.18, 'communication_loss': 0.30, 'estop': 0.30,
                    'dead_man': 0.30}
# 3 trials per path (coordinator 2026-11-10). The spread may be 20 % of the mean or 2 cm, whichever is larger: at
# 0.10 m/s the distances are a few centimetres and the 10 Hz wheel feedback resolves about 1 cm.
BRAKING_TRIALS, BRAKING_SPREAD, BRAKING_SPREAD_FLOOR_M, BRAKING_HEADING_DEG = 3, 0.20, 0.02, 5.0
REQUIRED_READBACK = ('communication_timeout',)   # when communication_timeout_s is given
_SHA256 = re.compile(r'[0-9a-f]{64}')


def _number(value, name, low, high):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) \
            or not low <= value <= high:
        raise FeedbackError('%s must be a finite number in [%s, %s]' % (name, low, high))
    return float(value)


def _integer(value, name, low, high):
    if type(value) is not int or not low <= value <= high:
        raise FeedbackError('%s must be an integer in [%s, %s]' % (name, low, high))
    return value


def _text(value, name, limit=200):
    if not isinstance(value, str) or not value.strip() or len(value) > limit:
        raise FeedbackError(name + ' must be a short non-empty string')
    return value


def _step_cap(steps, done):
    """One step above the highest step already passed (``done``), the first step when nothing passed yet."""
    if done is None:
        return steps[0]
    for step in steps:
        if step > done + 1e-9:
            return step
    return steps[-1]


def allowed_roots(project_root):
    """Evidence sources live in this checkout or in the configured data root."""
    roots = [Path(project_root).resolve()]
    try:
        from wc_runtime.storage_policy import StoragePolicy
        storage = StoragePolicy(project_root)
        if storage.enabled:
            roots.append(Path(storage.archive_root).resolve())
    except Exception:  # noqa: BLE001 - an unreadable storage policy only removes the data root option
        pass
    return roots


def check_source(item, project_root, roots):
    if not isinstance(item, dict) or set(item) - {'kind', 'path', 'sha256', 'note'} or \
            not isinstance(item.get('path'), str) or not _SHA256.fullmatch(str(item.get('sha256'))):
        raise FeedbackError('each source needs a path and a lowercase sha256')
    path = Path(item['path'])
    path = (Path(project_root) / path if not path.is_absolute() else path).resolve()
    if not any(path.is_relative_to(root) for root in roots) or not path.is_file() or path.stat().st_size > 200_000_000:
        raise FeedbackError('evidence source must be a bounded file in the checkout or the data root: ' + item['path'])
    if hashlib.sha256(path.read_bytes()).hexdigest() != item['sha256']:
        raise FeedbackError('evidence source content hash mismatch: ' + item['path'])
    return str(path)


def _sources(value, name, project_root, roots):
    if not isinstance(value, list) or not value:
        raise FeedbackError(name + ' needs its recordings or videos as sources')
    return [check_source(item, project_root, roots) for item in value]


def check_readback(item):
    """One read-only FC03 check of a driver parameter, written like manual_hardware's state checks."""
    if not isinstance(item, dict) or set(item) - {'purpose', 'request_hex', 'word_index', 'mask', 'equals', 'note'}:
        raise FeedbackError('readback check needs purpose, request_hex, word_index, mask and equals')
    purpose = _text(item.get('purpose'), 'readback purpose')
    try:
        request = checked_frame(bytes.fromhex(str(item.get('request_hex'))))
    except (ValueError, FeedbackError) as error:
        raise FeedbackError('readback request must be a valid RTU frame') from error
    if len(request) != 8 or request[:2] != b'\x01\x03':
        raise FeedbackError('readback permits only slave 1 FC03')
    _, _, address, count = struct.unpack('>BBHH', request[:-2])
    if not 1 <= count <= 8 or request != read_request(1, address, count):
        raise FeedbackError('readback range is invalid')
    from .driver_readback import whitelisted
    whitelisted(request)   # the owner reads these at session start, only from the read-only whitelist
    index = _integer(item.get('word_index'), 'readback word_index', 0, count - 1)
    mask = _integer(item.get('mask'), 'readback mask', 1, 65535)
    equals = _integer(item.get('equals'), 'readback equals', 0, 65535)
    if equals & ~mask:
        raise FeedbackError('readback expected value lies outside its mask')
    return {'purpose': purpose, 'request': request, 'word_index': index, 'mask': mask, 'equals': equals}


def check_hold_to_run(value):
    """Installed or not, written out; not installed needs the user's decision in words, by whom and when."""
    if not isinstance(value, dict) or set(value) - {'installed', 'decision', 'decided_by', 'decided_at', 'note'} or \
            type(value.get('installed')) is not bool:
        raise FeedbackError('hold_to_run_switch must say installed true or false')
    result = {'installed': value['installed'], 'decision': value.get('decision')}
    if not value['installed']:
        result['decision'] = _text(value.get('decision'), 'hold_to_run_switch.decision', 400)
        if value.get('decided_by') != 'user':
            raise FeedbackError('hold_to_run_switch not installed must be decided_by the user')
        result['decided_at'] = _text(value.get('decided_at'), 'hold_to_run_switch.decided_at')
    return result


def check_limits(value, stage):
    if not isinstance(value, dict) or set(value) != set(HARD_LIMITS):
        raise FeedbackError('approved_limits must give exactly ' + ', '.join(sorted(HARD_LIMITS)))
    return {name: _number(value[name], 'approved_limits.' + name, 1e-6, HARD_LIMITS[name]) for name in HARD_LIMITS}


def driver_ignored_loss(value):
    """True when a loss test records that the driver did not stop by itself (``driver_stopped`` false)."""
    return isinstance(value, dict) and value.get('driver_stopped', True) is False


def check_stop_test(name, value, project_root, roots, communication_timeout_s, max_linear_m_s):
    if not isinstance(value, dict) or value.get('verified') is not True:
        raise FeedbackError('stop test %s is not verified' % name)
    trials = _integer(value.get('trials'), name + '.trials', 3, 1000)
    if name in LOSS_TESTS and driver_ignored_loss(value):
        # Coordinator 2026-11-10 14:40: at 0.10 m/s the chair may run on a driver that keeps going when the line is
        # lost; the person beside it presses the emergency stop, and the driver parameters are changed later.
        if max_linear_m_s > NOT_STOPPED_MAX_LINEAR_M_S + 1e-9:
            raise FeedbackError('stop test %s: a driver that did not stop by itself is accepted only up to %.2f m/s'
                                % (name, NOT_STOPPED_MAX_LINEAR_M_S))
        _text(value.get('decision'), name + '.decision', 400)
        if value.get('decided_by') not in DECIDED_BY:
            raise FeedbackError('%s.decided_by must be one of %s' % (name, ', '.join(DECIDED_BY)))
        _text(value.get('decided_at'), name + '.decided_at')
        if value.get('max_stop_time_s') is not None:
            raise FeedbackError('%s: the driver did not stop by itself, so max_stop_time_s must be null' % name)
    elif name in LOSS_TESTS:
        if value.get('driver_stopped', True) is not True:
            raise FeedbackError(name + '.driver_stopped must be true or false')
        stop = _number(value.get('max_stop_time_s'), name + '.max_stop_time_s', 0.0, 10.0)
        if communication_timeout_s is None:
            if stop > UNKNOWN_TIMEOUT_STOP_S + 1e-9:
                raise FeedbackError('stop test %s took %.2f s with the communication timeout not known: a stop after '
                                    '%.1f s counts as not stopped (driver_stopped false, with the decision)'
                                    % (name, stop, UNKNOWN_TIMEOUT_STOP_S))
        elif stop > communication_timeout_s + LOSS_STOP_MARGIN_S + 1e-9:
            raise FeedbackError('stop test %s took %.2f s, more than the communication timeout plus %.1f s (%.2f s)'
                                % (name, stop, LOSS_STOP_MARGIN_S, communication_timeout_s + LOSS_STOP_MARGIN_S))
    elif value.get('max_stop_time_s') is not None:
        _number(value['max_stop_time_s'], name + '.max_stop_time_s', 0.0, 10.0)
    if name == 'estop' and value.get('moved_after_reset') is not False:
        raise FeedbackError('estop.moved_after_reset must be false: after a reset with zero commanded the wheels '
                            'must not move')
    if name == 'auto_stand_checks':
        ratios = value.get('speed_ratios')
        if not isinstance(ratios, list) or len(ratios) != trials:
            raise FeedbackError('auto_stand_checks.speed_ratios needs one feedback/command ratio per trial')
        for ratio in ratios:
            ratio = _number(ratio, 'auto_stand_checks.speed_ratios', 0.0, 10.0)
            if abs(ratio - 1.0) > SPEED_RATIO_TOLERANCE + 1e-9:
                raise FeedbackError('speed conversion: wheel feedback %.0f %% of the command, outside %.0f %%'
                                    % (100 * ratio, 100 * SPEED_RATIO_TOLERANCE))
        # The emergency stop during automatic control (field step 5.3): the stop the wheel feedback rule must see.
        _integer(value.get('estop_trials'), 'auto_stand_checks.estop_trials', 1, 1000)
        if value.get('moved_after_reset') is not False:
            raise FeedbackError('auto_stand_checks.moved_after_reset must be false: after the emergency stop during '
                                'automatic control the wheels must not move on the reset')
    return _sources(value.get('sources'), 'stop test ' + name, project_root, roots)


def check_braking(rows, paths, project_root, roots):
    """Section 8 pass lines for every row; returns the highest measured speed per path."""
    if not isinstance(rows, list):
        raise FeedbackError('braking must be a list of measured rows')
    measured = {path: 0.0 for path in paths}
    for row in rows:
        if not isinstance(row, dict) or row.get('path') not in BRAKING_LIMITS_M:
            raise FeedbackError('braking row needs a path in ' + ', '.join(BRAKING_LIMITS_M))
        speed = _number(row.get('speed_m_s'), 'braking.speed_m_s', 0.01, HARD_LIMITS['max_linear_m_s'])
        distances = row.get('distances_m')
        if not isinstance(distances, list) or len(distances) < BRAKING_TRIALS:
            raise FeedbackError('braking row needs at least %d distances' % BRAKING_TRIALS)
        values = [_number(d, 'braking distance', 0.0, 5.0) for d in distances]
        mean = sum(values) / len(values)
        if max(values) > BRAKING_LIMITS_M[row['path']]:
            raise FeedbackError('braking distance above the pass line for ' + row['path'])
        if max(values) - min(values) > max(BRAKING_SPREAD * mean, BRAKING_SPREAD_FLOOR_M) + 1e-9:
            raise FeedbackError('braking distances scatter more than 20%% of the mean and more than %.2f m for %s'
                                % (BRAKING_SPREAD_FLOOR_M, row['path']))
        if _number(row.get('max_heading_change_deg'), 'braking heading change', 0.0, 180.0) > BRAKING_HEADING_DEG:
            raise FeedbackError('heading changed more than 5 degrees while braking on ' + row['path'])
        if row['path'] == 'estop' and row.get('moved_after_reset') is not False:
            raise FeedbackError('estop braking row needs moved_after_reset false: after the emergency stop on the '
                                'ground the chair must not move on the reset')
        _sources(row.get('sources'), 'braking row', project_root, roots)
        if row['path'] in measured:
            measured[row['path']] = max(measured[row['path']], speed)
    return measured


def check_closed_loop_runs(rows, project_root, roots):
    """Supervised runs already passed: (highest linear, highest angular) among them, or (None, None)."""
    if rows is None:
        return None, None
    if not isinstance(rows, list):
        raise FeedbackError('closed_loop_runs must be a list')
    linear = angular = None
    for row in rows:
        if not isinstance(row, dict) or set(row) - {'max_linear_m_s', 'max_angular_rad_s', 'passed', 'reviewed_at',
                                                    'sources', 'note'} or type(row.get('passed')) is not bool:
            raise FeedbackError('closed loop run needs max_linear_m_s, max_angular_rad_s, passed, reviewed_at, sources')
        v = _number(row.get('max_linear_m_s'), 'closed loop max_linear_m_s', 0.01, HARD_LIMITS['max_linear_m_s'])
        w = _number(row.get('max_angular_rad_s'), 'closed loop max_angular_rad_s', 0.01, HARD_LIMITS['max_angular_rad_s'])
        _text(row.get('reviewed_at'), 'closed loop reviewed_at')
        _sources(row.get('sources'), 'closed loop run', project_root, roots)
        if row['passed']:
            linear = v if linear is None else max(linear, v)
            angular = w if angular is None else max(angular, w)
    return linear, angular


def load_evidence(value, project_root, expected_device_id):
    """Validated content of an evidence document (already parsed JSON); raises FeedbackError otherwise."""
    from wc_runtime.device_bindings import identity_token
    if not isinstance(value, dict) or value.get('schema') != EVIDENCE_SCHEMA:
        raise FeedbackError('automatic drive evidence must use ' + EVIDENCE_SCHEMA)
    if value.get('state') != 'REVIEWED':
        raise FeedbackError('automatic drive evidence is not reviewed')
    stage = value.get('stage')
    if stage not in STAGES:
        raise FeedbackError('evidence stage must be one of ' + ', '.join(STAGES))
    try:
        device = identity_token(value.get('device_id'), prefix='ZLAC8030D-')
        expected = identity_token(expected_device_id, prefix='ZLAC8030D-')
    except ValueError as error:
        raise FeedbackError(str(error)) from error
    if device != expected:
        raise FeedbackError('evidence identifies another wheel controller')
    firmware = _text(value.get('firmware_id'), 'firmware_id')
    _text(value.get('reviewer'), 'reviewer')
    _text(value.get('reviewed_at'), 'reviewed_at')
    hold = check_hold_to_run(value.get('hold_to_run_switch'))
    timeout = value.get('communication_timeout_s')   # null: not known, accepted up to NOT_STOPPED_MAX_LINEAR_M_S
    if timeout is not None or 'communication_timeout_s' not in value:
        timeout = _number(timeout, 'communication_timeout_s', 0.01, 5.0)
    limits = check_limits(value.get('approved_limits'), stage)
    if timeout is None and limits['max_linear_m_s'] > NOT_STOPPED_MAX_LINEAR_M_S + 1e-9:
        raise FeedbackError('communication_timeout_s is required above %.2f m/s' % NOT_STOPPED_MAX_LINEAR_M_S)
    readback = value.get('driver_readback')
    if not isinstance(readback, list) or not readback:
        raise FeedbackError('driver parameter readback checks are required')
    checks = [check_readback(item) for item in readback]
    missing = set(REQUIRED_READBACK if timeout is not None else ()) - {item['purpose'] for item in checks}
    if missing:
        raise FeedbackError('driver readback must include ' + ', '.join(sorted(missing)))
    roots = allowed_roots(project_root)
    tests = value.get('stop_tests')
    if not isinstance(tests, dict) or set(tests) - set(STOP_TESTS):
        raise FeedbackError('stop_tests must use the names ' + ', '.join(STOP_TESTS))
    required = [name for name in REQUIRED_STOP_TESTS[stage] if hold['installed'] or name != 'dead_man']
    for name in STOP_TESTS:
        # Tests a stage does not need are optional; one recorded as verified is checked like a required one.
        if name in required or (isinstance(tests.get(name), dict) and tests[name].get('verified') is True):
            check_stop_test(name, tests.get(name), project_root, roots, timeout, limits['max_linear_m_s'])
    paths = [path for path in BRAKING_LIMITS_M if (hold['installed'] or path != 'dead_man') and
             not (path == 'communication_loss' and driver_ignored_loss(tests.get('usb_unplugged')))]
    measured = check_braking(value.get('braking', []), paths, project_root, roots)
    if stage == 'SUPERVISED_CLOSED_LOOP':
        short = sorted(path for path in paths if measured[path] < limits['max_linear_m_s'] - 1e-9)
        if short:
            raise FeedbackError('braking not measured at the approved top speed for ' + ', '.join(short))
        done_linear, done_angular = check_closed_loop_runs(value.get('closed_loop_runs'), project_root, roots)
    else:
        if value.get('closed_loop_runs'):
            raise FeedbackError('closed_loop_runs belong to the SUPERVISED_CLOSED_LOOP stage')
        done_angular = None   # stand and ground tests stay at the first angular step
        ground = [step for step in LINEAR_STEPS_M_S if all(measured[path] >= step - 1e-9 for path in paths)]
        done_linear = max(ground) if stage == 'GROUND_TEST' and ground else None
    caps = {'max_linear_m_s': _step_cap(LINEAR_STEPS_M_S, done_linear),
            'max_angular_rad_s': _step_cap(ANGULAR_STEPS_RAD_S, done_angular)}
    for name, cap in caps.items():
        if limits[name] > cap + 1e-9:
            raise FeedbackError('approved_limits.%s %.2f is above the next speed step %.2f for this evidence'
                                % (name, limits[name], cap))
    raw = json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False).encode('utf-8')
    return {'stage': stage, 'device_id': device, 'firmware_id': firmware, 'limits': limits, 'readback': checks,
            'hold_to_run_switch': hold, 'communication_timeout_s': timeout, 'speed_caps': caps,
            'evidence_sha256': hashlib.sha256(raw).hexdigest()}


def _reject_constant(value):
    raise FeedbackError('non-finite JSON constant ' + value)


def wheel_device_id(project_root):
    """device_id of the checkout's selected wheel_feedback_current.json, the one the wheel owner checks against."""
    from wc_runtime.project_config import selected_config_path
    path = selected_config_path(Path(project_root), 'wheel_feedback_current.json')
    return json.loads(path.read_text(encoding='utf-8'))['device_id']


def read_evidence_file(path, project_root, expected_device_id):
    path = Path(path)
    path = (Path(project_root) / path if not path.is_absolute() else path)
    if not path.is_file() or path.stat().st_size > 1_000_000:
        raise FeedbackError('automatic drive evidence file missing or oversized: ' + str(path))
    try:
        value = json.loads(path.read_text(encoding='utf-8'), parse_constant=_reject_constant)
    except ValueError as error:
        raise FeedbackError('automatic drive evidence is not valid JSON') from error
    return load_evidence(value, project_root, expected_device_id)


def main(argv=None):
    """Check an evidence file as the wheel owner does at session start; reads files only."""
    import argparse
    parser = argparse.ArgumentParser(description='Check an automatic drive evidence file as the wheel owner does at '
                                     'session start (velocity_channel.md 8.1). Reads files only.')
    parser.add_argument('--project-root', type=Path, required=True)
    parser.add_argument('--evidence', type=Path, required=True,
                        help='as in auto_controls.evidence: relative to the checkout, or absolute')
    parser.add_argument('--device-id', help="default: device_id of the checkout's wheel_feedback_current.json")
    args = parser.parse_args(argv)
    root = args.project_root.resolve()
    try:
        device = args.device_id or wheel_device_id(root)
        result = read_evidence_file(args.evidence, root, device)
    except (FeedbackError, OSError, ValueError, KeyError) as error:
        print('REJECTED: %s' % error, flush=True)
        return 1
    limits = result['limits']
    print('PASS: stage %s, device %s, limits %.2f m/s %.2f rad/s (next step caps %.2f m/s %.2f rad/s), '
          'readback %s, communication timeout %s s, hold-to-run switch %s, evidence sha256 %s'
          % (result['stage'], result['device_id'], limits['max_linear_m_s'], limits['max_angular_rad_s'],
             result['speed_caps']['max_linear_m_s'], result['speed_caps']['max_angular_rad_s'],
             ', '.join(check['purpose'] for check in result['readback']), result['communication_timeout_s'],
             'installed' if result['hold_to_run_switch']['installed'] else 'not installed',
             result['evidence_sha256'][:12]), flush=True)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
