"""Write a few per-wheel ZLAC8030D parameters into the driver's RAM, on site, with the user's consent at the time.

Coordinator relay 2026-11-12 item 2, user decision BOARD 11-12 11:05 (3). Field flow: the read-only
``driver_readback --label wheels_compare`` first; then write only items whose left and right values differ, or
items with a stated reason.

Registers: FC06 to slave 1 for the registers in WRITABLE only, the per-wheel items of window A's 11-12 00:25 list in
the layout of the ZLAC8015D RS485 manual V1.04 (not verified for the 8030D). Left out on purpose: the hall offset
angle (each motor's own commutation setting: left and right may differ by design, and a wrong value can make a motor
run rough or run away) and every other register. 2010h (save to EEPROM) is never written. The 8015D manual keeps
parameter writes in RAM until 2010h, so switching the driver off would restore the stored values; that is not
verified for the 8030D, so ``revert`` writes the earlier values back instead of relying on it.

Values: ``--copy left-to-right`` (or ``right-to-left``) gives the target wheel the other wheel's value for every chosen
item that differs in the read before the write. ``--set right.velocity_loop_ki=VALUE`` writes a given raw word within
SET_RATIO of the value read before and needs ``--reason``. ``--plan-from <readback result.json>`` prints the plan these
options make from a readback and stops, without opening the port.

A write run: today's approval WC_DRIVER_WRITE_APPROVAL=APPROVED_BY_USER_<YYYYMMDD>; the wheel serial lease (refused
while the panel or a capture holds the port); both wheels standing still (the feedback read 20ABh-20ACh within
STATIONARY_WORDS); the parameter blocks read; the plan printed and confirmed by typing yes; one FC06 write per
register, each answered by its exact echo (the first failure stops the writes, nothing is retried); the blocks read
again, where every answered write must read back and nothing else may have changed. After a transport failure the
line is not used again: read back with driver_readback. Every run, also a refused or unconfirmed one, writes
reports/driver_params/<UTC>_<label>/result.json. ``revert --run <folder>`` writes the values read before that run
back in the same way, only where a register still holds what that run wrote.
"""
import argparse
import datetime
import json
import os
from pathlib import Path
import re
import select
import struct
import time

from . import driver_readback as dr
from .feedback_transport import FeedbackSerialLease, QueryFailure, validate_config
from .protocol import FeedbackError, checked_frame, crc16, integer, read_request

SCHEMA = 'wc_driver_params_v1'
SLAVE = dr.SLAVE
APPROVAL_ENV = 'WC_DRIVER_WRITE_APPROVAL'
EEPROM_SAVE = 0x2010
# item: (name in the readback's left/right table, left register, right register)
WRITABLE = {
    'rated_current': ('rated current', 0x2033, 0x2063),
    'maximum_current': ('maximum current', 0x2034, 0x2064),
    'velocity_smoothing': ('velocity smoothing factor', 0x2037, 0x2067),
    'velocity_loop_kp': ('velocity loop Kp', 0x203C, 0x206C),
    'velocity_loop_ki': ('velocity loop Ki', 0x203D, 0x206D),
    'acceleration_time': ('acceleration time', 0x2080, 0x2081),
    'deceleration_time': ('deceleration time', 0x2082, 0x2083),
}
REGISTERS = {register: (item, wheel) for item, (_, left, right) in WRITABLE.items()
             for wheel, register in (('left', left), ('right', right))}
PARAMETER_READS = ((0x2030, 17, 'left motor parameters 2030h-2040h'),
                   (0x2060, 17, 'right motor parameters 2060h-2070h'),
                   (0x2080, 8, 'acceleration, deceleration, quick stop and torque rate times, left then right'))
SPEED_READ = (0x20AB, 2, 'actual speed, left and right (the wheel owner feedback read)')
ALLOWED_READS = {(start, count) for start, count, _ in (*PARAMETER_READS, SPEED_READ)}
STATIONARY_WORDS = 3          # the wheel owner's standing-still rule (mapping_auto.STATIONARY_WORDS)
SET_RATIO = (0.5, 1.5)        # a given value stays within these times the value read before
LABEL = dr.LABEL
SET_TEXT = re.compile(r'(left|right)\.([a-z_]+)=(\d{1,5})')


def write_request(register, value):
    """The FC06 frame for one writable register; anything else raises FeedbackError."""
    if register == EEPROM_SAVE or register not in REGISTERS:
        raise FeedbackError('%04Xh is not a writable parameter' % register)
    integer(value, 'register value', 0, 0xFFFF)
    payload = struct.pack('>BBHH', SLAVE, 6, register, value)
    return payload + crc16(payload)


def checked_write(request):
    """(register, value) of an exact FC06 frame for a writable register; anything else raises FeedbackError."""
    frame = checked_frame(request)
    if len(frame) != 8 or frame[0] != SLAVE or frame[1] != 6:
        raise FeedbackError('only FC06 writes to slave 1 are sent')
    register, value = struct.unpack('>HH', frame[2:6])
    if frame != write_request(register, value):
        raise FeedbackError('write outside the parameter whitelist')
    return register, value


def write_exchange(fd, request, timeout_s, *, clock=time.monotonic):
    """One FC06 transaction: (answer, None) for the exact echo, (answer, code) for a Modbus exception answer.

    Anything else raises QueryFailure; the caller must not retry or use the line again.
    """
    checked_write(request)
    if select.select([fd], [], [], .002)[0]:
        raise QueryFailure('unsolicited bytes before the write; nothing sent', os.read(fd, 256))
    deadline = clock() + timeout_s
    if not select.select([], [fd], [], max(0, deadline - clock()))[1]:
        raise QueryFailure('serial write readiness timeout; nothing sent')
    sent = os.write(fd, request)
    if sent != len(request):
        raise QueryFailure('partial FC06 transmission; no retry', b'', sent)
    response, expected = bytearray(), len(request)
    while len(response) < expected:
        remaining = deadline - clock()
        if remaining <= 0 or not select.select([fd], [], [], remaining)[0]:
            raise QueryFailure('response timeout; no retry', response, sent)
        chunk = os.read(fd, 256)
        if not chunk:
            raise QueryFailure('serial disconnected; no reconnect', response, sent)
        response.extend(chunk)
        if len(response) >= 2 and response[1] == 0x86:
            expected = 5
        if len(response) > expected:
            raise QueryFailure('response overflow or overlapping traffic', response, sent)
    response = bytes(response)
    try:
        checked_frame(response)
    except FeedbackError as error:
        raise QueryFailure(str(error), response, sent) from error
    if response[0] != SLAVE:
        raise QueryFailure('response slave mismatch', response, sent)
    if response[1] == 0x86:
        return response, response[2]
    if response != request:
        raise QueryFailure('FC06 answer is not the exact echo', response, sent)
    return response, None


def read_words(lease, reads, timeout_s, rows, *, sleep=time.sleep):
    """Read ``reads`` with driver_readback.read_all; (words by register, None or what went wrong)."""
    entries = [(start, count, meaning, read_request(SLAVE, start, count)) for start, count, meaning in reads]
    error = dr.read_all(lease, entries, timeout_s, rows, sleep=sleep, allowed=ALLOWED_READS)
    words = {}
    for row in rows:
        if row['status'] == 'WORDS':
            start = int(row['start_register'][:-1], 16)
            words.update({start + index: word for index, word in enumerate(row['words_u16'])})
        elif error is None:
            error = '%s: %s' % (row['start_register'], row['status'])
    return words, error


def signed(word):
    return word - 0x10000 if word >= 0x8000 else word


def parse_set(text):
    """(wheel, item, value) of 'right.velocity_loop_ki=120'."""
    match = SET_TEXT.fullmatch(text)
    if not match or match.group(2) not in WRITABLE:
        raise FeedbackError('--set takes left|right.<item>=<raw word>, items: ' + ', '.join(WRITABLE))
    return match.group(1), match.group(2), integer(int(match.group(3)), 'register value', 0, 0xFFFF)


def plan_writes(before, *, copy=None, items=None, sets=(), reason=None):
    """[{item, wheel, register, before, value, why}] in WRITABLE order; FeedbackError for anything not allowed.

    ``before``: the words read just before, by register.
    """
    chosen = list(WRITABLE) if items is None else list(items)
    if set(chosen) - set(WRITABLE):
        raise FeedbackError('unknown items: ' + ', '.join(sorted(set(chosen) - set(WRITABLE))))
    planned = {}
    if copy is not None:
        if copy not in ('left-to-right', 'right-to-left'):
            raise FeedbackError('--copy is left-to-right or right-to-left')
        source, target = copy.split('-to-')
        for item in chosen:
            _, left, right = WRITABLE[item]
            register = {'left': left, 'right': right}
            if before[register[source]] != before[register[target]]:
                planned[register[target]] = dict(item=item, wheel=target, register='%04Xh' % register[target],
                                                 before=before[register[target]], value=before[register[source]],
                                                 why='copy from the %s wheel' % source)
    if sets and not (isinstance(reason, str) and reason.strip()):
        raise FeedbackError('--set needs --reason')
    for text in sets:
        wheel, item, value = parse_set(text)
        register = WRITABLE[item][1 if wheel == 'left' else 2]
        if register in planned:
            raise FeedbackError('%04Xh would be written twice' % register)
        old = before[register]
        if value == old:
            continue
        if old == 0 or not SET_RATIO[0] <= value / old <= SET_RATIO[1]:
            raise FeedbackError('%s.%s=%d: a given value stays within %.1f-%.1f times the value read before (%d)'
                                % (wheel, item, value, SET_RATIO[0], SET_RATIO[1], old))
        planned[register] = dict(item=item, wheel=wheel, register='%04Xh' % register, before=old, value=value,
                                 why=reason.strip())
    order = [register for _, left, right in WRITABLE.values() for register in (left, right)]
    return [planned[register] for register in order if register in planned]


def revert_plan(previous, before, run_name='that run'):
    """Writes putting back the values read before ``previous`` (a write run's result), where a register still holds
    what that run wrote; FeedbackError when a register holds something else."""
    if previous.get('schema') != SCHEMA or previous.get('mode') != 'write':
        raise FeedbackError('--run must be a driver_params write run')
    plan, conflicts = [], []
    for row in previous.get('writes') or []:
        if not row.get('transmitted_bytes') and row.get('status') != 'IN_PROGRESS':   # never sent
            continue
        register = int(row['register'][:-1], 16)
        if register not in REGISTERS:
            raise FeedbackError('%s is not a writable parameter' % row['register'])
        now = before[register]
        if now == row['value'] and now != row['before']:
            plan.append(dict(item=row['item'], wheel=row['wheel'], register=row['register'], before=now,
                             value=row['before'], why='back to the value before ' + run_name))
        elif now != row['before']:
            conflicts.append('%s is %d; %s wrote %d over %d' % (row['register'], now, run_name, row['value'],
                                                                  row['before']))
    if conflicts:
        raise FeedbackError('not reverting, changed since: ' + '; '.join(conflicts))
    return plan


def write_all(lease, plan, timeout_s, rows, *, sleep=time.sleep):
    """One FC06 write per planned register in order, appending one row each to rows (so an interrupted run still
    records what was sent). Returns None or the error that stopped the writes."""
    error = None
    for index, step in enumerate(plan):
        register = int(step['register'][:-1], 16)
        request = write_request(register, step['value'])
        row = dict(step, request_hex=request.hex(), status='NOT_SENT', transmitted_bytes=0)
        rows.append(row)
        if error is not None:
            continue
        if index:
            sleep(dr.REQUEST_GAP_S)
        row['status'] = 'IN_PROGRESS'
        started = time.monotonic_ns()
        try:
            response, code = write_exchange(lease.fd, request, timeout_s)
        except QueryFailure as failure:
            lease.failed = True
            error = '%s: %s' % (step['register'], failure)
            row.update(status='TRANSPORT_FAILED', error=str(failure), response_hex=failure.response.hex(),
                       transmitted_bytes=failure.transmitted_bytes)
            continue
        row.update(response_hex=response.hex(), transmitted_bytes=len(request),
                   exchange_started_monotonic_ns=started, exchange_completed_monotonic_ns=time.monotonic_ns())
        if code is None:
            row['status'] = 'ACKED'
        else:
            row.update(status='MODBUS_EXCEPTION', exception_code=code)
            error = '%s: Modbus exception %d' % (step['register'], code)
    return error


def verify(before, after, writes):
    """What the read after the writes shows wrongly: an answered write not read back, or another word changed.

    Only called when every sent write got a complete answer (an echo or a Modbus exception).
    """
    expected = {register: word for register, word in before.items()
                if any(start <= register < start + count for start, count, _ in PARAMETER_READS)}
    for row in writes:
        if row['status'] == 'ACKED':
            expected[int(row['register'][:-1], 16)] = row['value']
    return ['%04Xh reads %s, expected %d' % (register, after.get(register), word)
            for register, word in sorted(expected.items()) if after.get(register) != word]


def readback_words(path):
    """Parameter words of a driver_readback result.json (for --plan-from)."""
    path = Path(path)
    result = json.loads((path/'result.json' if path.is_dir() else path).read_text(encoding='utf-8'))
    if result.get('schema') != dr.SCHEMA:
        raise FeedbackError('--plan-from takes a driver_readback result.json')
    words = {}
    for row in result.get('reads') or []:
        if row.get('status') == 'WORDS':
            start = int(row['start_register'][:-1], 16)
            words.update({start + index: word for index, word in enumerate(row['words_u16'])})
    missing = [register for register in REGISTERS if register not in words]
    if missing:
        raise FeedbackError('the readback lacks ' + ', '.join('%04Xh' % register for register in missing))
    return words, result.get('device_id')


def approval_problem(environ, today):
    """None with today's explicit user approval, otherwise what is missing."""
    expected = 'APPROVED_BY_USER_' + today.strftime('%Y%m%d')
    if environ.get(APPROVAL_ENV) != expected:
        return 'driver parameter writes need %s=%s (the user agrees on site, at the time)' % (APPROVAL_ENV, expected)
    return None


def report_folder(project_root, label):
    from wc_runtime.storage_policy import StoragePolicy
    policy = StoragePolicy(project_root)
    policy.check()
    stamp = datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%SZ')
    folder = Path(policy.resolve('reports/driver_params/%s_%s' % (stamp, label)))
    folder.mkdir(parents=True, exist_ok=False)
    return folder


def note_owner(lease, mode):
    """The lease's owner note names the feedback query; say what this run may send instead."""
    lease.lock.seek(0)
    lease.lock.truncate()
    json.dump({'pid': os.getpid(), 'serial': lease.config['hardware_serial'],
               'purpose': 'driver_params_%s_fc03_and_fc06_ram_only' % mode,
               'writable_registers': ['%04Xh' % register for register in sorted(REGISTERS)],
               **lease.host_configuration}, lease.lock)
    lease.lock.flush()


def print_plan(plan):
    print('plan (raw words; FC06 into the driver RAM, never 2010h):', flush=True)
    for step in plan:
        print('  %-5s %-26s %s  %6d -> %-6d %s' % (step['wheel'], WRITABLE[step['item']][0], step['register'],
                                                    step['before'], step['value'], step['why']), flush=True)


def ask(prompt):
    try:
        return input(prompt).strip() == 'yes'
    except EOFError:
        return False


def _utc():
    return datetime.datetime.now(datetime.timezone.utc).isoformat(timespec='milliseconds')


def parse(argv):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    modes = parser.add_subparsers(dest='mode', required=True)
    write = modes.add_parser('write', help='copy or set whitelisted per-wheel parameters')
    revert = modes.add_parser('revert', help='write back the values read before a write run')
    for sub in (write, revert):
        sub.add_argument('--project-root', type=Path, required=True)
        sub.add_argument('--config', type=Path, help='wheel feedback transport config; default: the project selection')
    write.add_argument('--label', required=True, help='what this write is for, e.g. right_like_left')
    write.add_argument('--copy', choices=('left-to-right', 'right-to-left'),
                       help='give one wheel the other wheel\'s value where they differ')
    write.add_argument('--items', nargs='+', choices=list(WRITABLE), help='limit --copy to these items')
    write.add_argument('--set', dest='sets', action='append', default=[], metavar='WHEEL.ITEM=WORD',
                       help='a given raw word, within %.1f-%.1f times the value read before' % SET_RATIO)
    write.add_argument('--reason', help='why the --set values (required with --set)')
    write.add_argument('--plan-from', type=Path, metavar='READBACK',
                       help='print the plan from a driver_readback result.json (or its folder) and stop')
    revert.add_argument('--run', type=Path, required=True, help='the write run folder or its result.json')
    args = parser.parse_args(argv)
    if args.mode == 'write':
        if not LABEL.fullmatch(args.label):
            parser.error('--label: 1-64 of A-Z a-z 0-9 _ . -')
        if args.copy is None and not args.sets:
            parser.error('write needs --copy or --set')
        if args.items and args.copy is None:
            parser.error('--items limits --copy')
        if args.sets and not (args.reason or '').strip():
            parser.error('--set needs --reason')
        try:
            [parse_set(text) for text in args.sets]
        except FeedbackError as error:
            parser.error(str(error))
    return args


def main(argv=None, *, environ=None, today=None, confirm=ask):
    args = parse(argv)
    environ = os.environ if environ is None else environ
    root = args.project_root.resolve()
    if args.config is None:
        from wc_runtime.project_config import selected_config_path
        config_path = selected_config_path(root, 'wheel_feedback_current.json')
    else:
        config_path = args.config
    config = validate_config(json.loads(Path(config_path).read_text(encoding='utf-8')))
    previous, run_name = None, None
    if args.mode == 'write':
        options = dict(copy=args.copy, items=args.items, sets=args.sets, reason=args.reason)
        label = args.label
        if args.plan_from is not None:
            try:
                words, device = readback_words(args.plan_from)
                if device != config['device_id']:
                    raise FeedbackError('the readback is of %s, not %s' % (device, config['device_id']))
                plan = plan_writes(words, **options)
            except FeedbackError as refusal:
                print('no plan: %s' % refusal, flush=True)
                return 2
            if plan:
                print_plan(plan)
            print('planned from %s: %d writes; nothing sent' % (args.plan_from, len(plan)), flush=True)
            return 0
    else:
        run_folder = args.run.parent if args.run.name == 'result.json' else args.run
        previous = json.loads((run_folder/'result.json').read_text(encoding='utf-8'))
        if previous.get('schema') != SCHEMA or previous.get('mode') != 'write':
            raise FeedbackError('--run must be a driver_params write run')
        if previous.get('device_id') != config['device_id']:
            raise FeedbackError('that run wrote %s, not %s' % (previous.get('device_id'), config['device_id']))
        run_name, options = run_folder.name, dict(run=str(run_folder))
        label = ('revert_' + str(previous.get('label')))[:64]
    problem = approval_problem(environ, today or datetime.date.today())
    if problem:
        print(problem, flush=True)
        return 2
    folder = report_folder(root, label)
    record = {'schema': SCHEMA, 'mode': args.mode, 'label': label, 'started_utc': _utc(),
              'device_id': config['device_id'], 'slave': SLAVE, 'approval': environ[APPROVAL_ENV],
              'register_source': dr.REGISTER_SOURCE, 'values': 'raw 16-bit words; units and meaning unconfirmed',
              'request': options, 'timeout_s': config['timeout_s'], 'config': str(config_path),
              'writable_registers': {'%04Xh' % register: '%s %s' % REGISTERS[register][::-1]
                                     for register in sorted(REGISTERS)},
              'eeprom_save_written': False, 'plan': [], 'confirmed': False, 'writes': []}
    from wc_runtime.project_paths import runtime_root
    lease = FeedbackSerialLease(config, runtime_root(root))
    try:
        try:
            lease.open()  # never transmits; any failure here means nothing was sent
        except Exception as refusal:  # noqa: BLE001 - busy port, identity or permission: all refusals
            record.update(status='PORT_REFUSED', error=str(refusal))
        else:
            note_owner(lease, args.mode)
            record['host_serial'] = lease.host_configuration
            record['before'] = {'reads': []}
            before, error = read_words(lease, (*PARAMETER_READS, SPEED_READ), config['timeout_s'],
                                       record['before']['reads'])
            if error:
                record.update(status='READ_FAILED', error=error)
            else:
                record['speed_words'] = [signed(before[0x20AB]), signed(before[0x20AC])]
                if max(map(abs, record['speed_words'])) > STATIONARY_WORDS:
                    record.update(status='NOT_STANDING_STILL',
                                  error='wheel speed words %s; stop the wheels first' % record['speed_words'])
                else:
                    try:
                        plan = (plan_writes(before, **options) if previous is None
                                else revert_plan(previous, before, run_name))
                    except FeedbackError as refusal:
                        record.update(status='PLAN_REFUSED', error=str(refusal))
                    else:
                        record['plan'] = plan
                        if not plan:
                            record.update(status='NOTHING_TO_WRITE', error=None)
                        else:
                            print_plan(plan)
                            record['confirmed'] = bool(confirm(
                                'Write these %d registers into the driver RAM now? Type yes: ' % len(plan)))
                            if not record['confirmed']:
                                record.update(status='NOT_CONFIRMED', error=None)
                            else:
                                error = write_all(lease, plan, config['timeout_s'], record['writes'])
                                if lease.failed:
                                    record.update(status='WRITE_FAILED', error=error + '; line not used again, '
                                                  'read back with driver_readback')
                                else:
                                    record['after'] = {'reads': []}
                                    after, read_error = read_words(lease, PARAMETER_READS, config['timeout_s'],
                                                                   record['after']['reads'])
                                    record['verification'] = (['read after failed: ' + read_error] if read_error
                                                              else verify(before, after, record['writes']))
                                    record.update(status='WRITE_FAILED' if error else
                                                  'VERIFY_FAILED' if record['verification'] else 'COMPLETE',
                                                  error=error)
    finally:
        lease.close()
        record['finished_utc'] = _utc()
        record.setdefault('status', 'INTERRUPTED')
        sent = [row for row in (record.get('before', {}).get('reads', []) + record.get('after', {}).get('reads', []))
                if row.get('transmitted_bytes') or row['status'] == 'IN_PROGRESS']
        writes = [row for row in record['writes'] if row['transmitted_bytes'] or row['status'] == 'IN_PROGRESS']
        record['function_codes_sent'] = ([3] if sent else []) + ([6] if writes else [])
        record['parameter_writes_sent'] = len(writes)
        (folder/'result.json').write_text(json.dumps(record, indent=1, ensure_ascii=False, allow_nan=False)+'\n',
                                          encoding='utf-8')
    for row in record['writes']:
        print('%s  %s  %d -> %d' % (row['register'], row['status'], row['before'], row['value']), flush=True)
    for problem in record.get('verification') or []:
        print('read back: ' + problem, flush=True)
    if record.get('verification') == []:
        print('read back: every write reads back, nothing else changed', flush=True)
    print('%s: %s' % (record['status'], folder/'result.json'), flush=True)
    if record.get('error'):
        print(record['error'], flush=True)
    if args.mode == 'write' and record['parameter_writes_sent']:
        print('undo: %s=%s python3 -m wc_motion.driver_params revert --project-root %s --run %s'
              % (APPROVAL_ENV, environ[APPROVAL_ENV], root, folder), flush=True)
    return 0 if record['status'] in ('COMPLETE', 'NOTHING_TO_WRITE') else 1


if __name__ == '__main__':
    raise SystemExit(main())
