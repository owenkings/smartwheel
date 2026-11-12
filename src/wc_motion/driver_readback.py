"""Read-only FC03 readback of ZLAC8030D driver registers (velocity_channel.md 7.3 step 7).

Only Modbus FC03 read requests to slave 1 are sent, and only for the fixed list
REGISTERS; this module has no write function code, and every request is checked
against the list before it is written. The serial port is taken through
FeedbackSerialLease, the lease of the wheel owner and feedback captures (one
lock shared by all checkouts, no other process may have the port open, then
TIOCEXCL), so the tool refuses while the panel or a capture holds the port.

REGISTERS comes from the ZLAC8015D RS485 manual V1.04 and is not verified for
the 8030D. Values are kept as raw 16-bit words, without units. A Modbus
exception answer is recorded and the next register is read; a timeout, CRC
error, unexpected byte or disconnect ends the run without a retry.

Field use needs the user's approval on that day:
WC_DRIVER_READBACK_APPROVAL=APPROVED_BY_USER_<YYYYMMDD>. Every approved run
writes reports/driver_readback/<UTC>_<label>/result.json under the storage
policy, also when the port was refused or a read failed. To read 2003h once per
switch combination, run with --registers 2003 and a different --label each time.
The list also holds the left and right motor parameter blocks (2030h, 2043h, 2060h,
2073h), the per-wheel control times 2080h-2087h, the status words 20A1h-20A4h and
the actual currents 20ADh-20AEh. After a run the tool prints each per-wheel item
left against right and stores the same table as left_right in result.json.
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

from .feedback_transport import FeedbackSerialLease, QueryFailure, validate_config
from .protocol import FeedbackError, checked_frame, parse_exchange, read_request

SCHEMA = 'wc_driver_readback_v1'
SLAVE = 1
APPROVAL_ENV = 'WC_DRIVER_READBACK_APPROVAL'
# (first register, count, meaning in the ZLAC8015D RS485 manual V1.04 as given
# in velocity_channel.md section 10; None where the design gives none).
REGISTERS = (
    (0x2000, 1, 'communication loss time'),
    (0x2003, 1, 'input status'),
    (0x2007, 1, None),
    (0x2008, 1, 'maximum speed'),
    (0x2011, 1, 'quick stop mode'),
    (0x2016, 3, 'input levels and functions, 2016h-2018h'),
    (0x2084, 1, 'one of the acceleration, deceleration and quick stop times 2080h-2085h'),
    (0x2085, 1, 'one of the acceleration, deceleration and quick stop times 2080h-2085h'),
    (0x20A0, 1, 'software version'),
    (0x20A5, 1, 'fault code'),
    (0x20A6, 1, 'fault code'),
    # Left and right motor parameters, for comparing the two wheels (coordinator 2026-11-11, item 2). The blocks
    # skip 2041h-2042h and 2071h-2072h, which the 8015D manual does not list.
    (0x2030, 17, 'left motor 2030h-2040h: encoder lines, hall offset, overload factor, rated and maximum current, '
                 'overload time, position following error, velocity smoothing, current loop Kp Ki, feedforward and '
                 'torque smoothing, velocity loop Kp Ki Kf, position loop Kp Kf'),
    (0x2043, 8, 'left motor 2043h-204Ah: initial velocity (velocity, position mode), poles, over-temperature, '
                'velocity observer 1-4'),
    (0x2060, 17, 'right motor 2060h-2070h, same layout as 2030h-2040h'),
    (0x2073, 8, 'right motor 2073h-207Ah, same layout as 2043h-204Ah'),
    (0x2080, 8, 'acceleration, deceleration, quick stop deceleration time and torque rate, left then right'),
    (0x20A1, 4, 'DC voltage, motor state, hall state, motor temperature (high byte left, low byte right)'),
    (0x20AD, 2, 'actual torque (current), left and right'),
)
# (name, left register, right register); a register given as (address, 'high'|'low') is one byte of a word.
WHEEL_PAIRS = tuple(
    [(name, 0x2030 + offset, 0x2060 + offset) for offset, name in (
        (0x00, 'encoder lines'), (0x01, 'hall offset angle'), (0x02, 'overload factor'), (0x03, 'rated current'),
        (0x04, 'maximum current'), (0x05, 'overload protection time'), (0x06, 'position following error threshold'),
        (0x07, 'velocity smoothing factor'), (0x08, 'current loop Kp'), (0x09, 'current loop Ki'),
        (0x0A, 'feedforward output smoothing'), (0x0B, 'torque output smoothing'), (0x0C, 'velocity loop Kp'),
        (0x0D, 'velocity loop Ki'), (0x0E, 'velocity loop Kf'), (0x0F, 'position loop Kp'), (0x10, 'position loop Kf'),
        (0x13, 'initial velocity, velocity mode'), (0x14, 'initial velocity, position mode'), (0x15, 'motor poles'),
        (0x16, 'over-temperature threshold'), (0x17, 'velocity observer 1'), (0x18, 'velocity observer 2'),
        (0x19, 'velocity observer 3'), (0x1A, 'velocity observer 4'))] +
    [('acceleration time', 0x2080, 0x2081), ('deceleration time', 0x2082, 0x2083),
     ('quick stop deceleration time', 0x2084, 0x2085), ('torque rate', 0x2086, 0x2087),
     ('motor state', (0x20A2, 'high'), (0x20A2, 'low')), ('hall state', (0x20A3, 'high'), (0x20A3, 'low')),
     ('motor temperature', (0x20A4, 'high'), (0x20A4, 'low')), ('actual torque (current)', 0x20AD, 0x20AE)])
REQUEST_GAP_S = .05  # bus idle time between two reads
LABEL = re.compile(r'[A-Za-z0-9_.-]{1,64}')
REGISTER_SOURCE = 'ZLAC8015D RS485 manual V1.04; not verified for the ZLAC8030D'


def plan(selected=None):
    """[(start, count, meaning, request)] in list order; selected limits it to these start registers."""
    if selected is not None:
        unknown = set(selected) - {start for start, _, _ in REGISTERS}
        if unknown:
            raise FeedbackError('not in the read-only whitelist: ' +
                                ', '.join('%04Xh' % start for start in sorted(unknown)))
    return [(start, count, meaning, read_request(SLAVE, start, count)) for start, count, meaning in REGISTERS
            if selected is None or start in selected]


def whitelisted(request, allowed=None):
    """(start, count) of an exact whitelisted FC03 request; anything else raises FeedbackError.

    ``allowed``: other (start, count) reads instead of REGISTERS (driver_params' own read list).
    """
    frame = checked_frame(request)
    if len(frame) != 8 or frame[0] != SLAVE or frame[1] != 3:
        raise FeedbackError('only FC03 reads of slave 1 are sent')
    start, count = struct.unpack('>HH', frame[2:6])
    reads = {(first, n) for first, n, _ in REGISTERS} if allowed is None else set(allowed)
    if (start, count) not in reads or frame != read_request(SLAVE, start, count):
        raise FeedbackError('request outside the read-only whitelist')
    return start, count


def exchange(fd, request, timeout_s, *, clock=time.monotonic, allowed=None):
    """One bounded FC03 transaction: (response, words, None) or (response, None, exception code).

    A Modbus exception answer is a complete answer. Anything other than the
    exact answer raises QueryFailure, and the caller must not retry.
    """
    _, count = whitelisted(request, allowed)
    response, sent = bytearray(), 0
    if select.select([fd], [], [], .002)[0]:
        response.extend(os.read(fd, 256))
        raise QueryFailure('unsolicited bytes before the request; nothing sent', response)
    deadline = clock()+timeout_s
    if not select.select([], [fd], [], max(0, deadline-clock()))[1]:
        raise QueryFailure('serial write readiness timeout; nothing sent')
    sent = os.write(fd, request)
    if sent != len(request):
        raise QueryFailure('partial FC03 transmission; no retry', response, sent)
    expected = 5+2*count
    while len(response) < expected:
        remaining = deadline-clock()
        if remaining <= 0 or not select.select([fd], [], [], remaining)[0]:
            raise QueryFailure('response timeout; no retry', response, sent)
        chunk = os.read(fd, 256)
        if not chunk:
            raise QueryFailure('serial disconnected; no reconnect', response, sent)
        response.extend(chunk)
        if len(response) >= 2 and response[1] == 0x83:
            expected = 5
        if len(response) > expected:
            raise QueryFailure('response overflow or overlapping traffic', response, sent)
    response = bytes(response)
    try:
        if response[1] == 0x83:
            checked_frame(response)
            if response[0] != SLAVE:
                raise FeedbackError('response slave mismatch')
            return response, None, response[2]
        return response, list(parse_exchange(request, response)[2]), None
    except FeedbackError as error:
        raise QueryFailure(str(error), response, sent) from error


def read_all(lease, entries, timeout_s, reads, *, clock=time.monotonic, sleep=time.sleep, allowed=None):
    """Read entries over an open lease in order, appending one row each to reads.

    Returns None or the first transport failure; the entries after it stay NOT_READ.
    ``allowed`` replaces the whitelist as in ``whitelisted``.
    """
    error = None
    for start, count, meaning, request in entries:
        row = {'start_register': '%04Xh' % start, 'count': count, 'meaning_in_8015d_manual': meaning,
               'request_hex': request.hex(), 'status': 'NOT_READ'}
        reads.append(row)
        if error is not None:
            continue
        if len(reads) > 1:
            sleep(REQUEST_GAP_S)
        row['status'] = 'IN_PROGRESS'
        started = time.monotonic_ns()
        try:
            response, words, code = exchange(lease.fd, request, timeout_s, clock=clock, allowed=allowed)
        except QueryFailure as failure:
            lease.failed = True
            error = '%04Xh: %s' % (start, failure)
            row.update(status='TRANSPORT_FAILED', error=str(failure), response_hex=failure.response.hex(),
                       transmitted_bytes=failure.transmitted_bytes)
            continue
        row.update(response_hex=response.hex(), transmitted_bytes=len(request),
                   exchange_started_monotonic_ns=started, exchange_completed_monotonic_ns=time.monotonic_ns())
        if code is None:
            row.update(status='WORDS', words_u16=words, words_hex=['%04X' % word for word in words])
        else:
            row.update(status='MODBUS_EXCEPTION', exception_code=code)
    return error


def wheel_pairs(reads):
    """Left and right value of every per-wheel item that was read: [{name, left, right, equal}] (raw words)."""
    words = {}
    for row in reads:
        if row.get('status') == 'WORDS':
            start = int(row['start_register'][:-1], 16)
            for index, word in enumerate(row['words_u16']):
                words[start + index] = word

    def value(register):
        if isinstance(register, tuple):
            address, part = register
            if address not in words:
                return None
            return words[address] >> 8 if part == 'high' else words[address] & 0xff
        return words.get(register)

    def label(register):
        return '%04Xh %s byte' % register if isinstance(register, tuple) else '%04Xh' % register

    out = []
    for name, left, right in WHEEL_PAIRS:
        a, b = value(left), value(right)
        if a is None or b is None:
            continue
        out.append({'name': name, 'left_register': label(left), 'right_register': label(right),
                    'left': a, 'right': b, 'equal': a == b})
    return out


def approval_problem(environ, today):
    """None with today's explicit user approval, otherwise what is missing."""
    expected = 'APPROVED_BY_USER_' + today.strftime('%Y%m%d')
    if environ.get(APPROVAL_ENV) != expected:
        return 'driver readback needs %s=%s (the user approves on the day)' % (APPROVAL_ENV, expected)
    return None


def report_folder(project_root, label):
    from wc_runtime.storage_policy import StoragePolicy
    policy = StoragePolicy(project_root)
    policy.check()
    stamp = datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%SZ')
    folder = Path(policy.resolve('reports/driver_readback/%s_%s' % (stamp, label)))
    folder.mkdir(parents=True, exist_ok=False)
    return folder


def note_owner(lease, entries):
    """The lease's owner note names the feedback query; say what this run sends instead."""
    lease.lock.seek(0)
    lease.lock.truncate()
    json.dump({'pid': os.getpid(), 'serial': lease.config['hardware_serial'], 'purpose': 'driver_readback_fc03_only',
               'request_hex': [entry[3].hex() for entry in entries], **lease.host_configuration}, lease.lock)
    lease.lock.flush()


def _utc():
    return datetime.datetime.now(datetime.timezone.utc).isoformat(timespec='milliseconds')


def main(argv=None, *, environ=None, today=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--project-root', type=Path, required=True)
    parser.add_argument('--label', required=True, help='what was set while reading, e.g. estop_pressed_blue_up')
    parser.add_argument('--registers', nargs='+', metavar='HEX',
                        help='only these whitelisted start registers, e.g. 2003; default: all')
    parser.add_argument('--config', type=Path, help='wheel feedback transport config; default: the project selection')
    parser.add_argument('--dry-run', action='store_true',
                        help='print the requests and stop: no approval needed, the port is not opened')
    args = parser.parse_args(argv)
    if not LABEL.fullmatch(args.label):
        raise SystemExit('--label: 1-64 of A-Z a-z 0-9 _ . -')
    try:
        selected = None if args.registers is None else {int(value, 16) for value in args.registers}
    except ValueError:
        raise SystemExit('--registers takes hex start registers such as 2003') from None
    entries = plan(selected)
    root = args.project_root.resolve()
    if args.config is None:
        from wc_runtime.project_config import selected_config_path
        config_path = selected_config_path(root, 'wheel_feedback_current.json')
    else:
        config_path = args.config
    config = validate_config(json.loads(Path(config_path).read_text(encoding='utf-8')))
    for start, count, meaning, request in entries:
        print('%04Xh x%d  %s  %s' % (start, count, request.hex(' '), meaning or ''), flush=True)
    if args.dry_run:
        print('dry run: %d FC03 reads of slave 1 on %s; nothing sent' % (len(entries), config['device_id']), flush=True)
        return 0
    problem = approval_problem(os.environ if environ is None else environ, today or datetime.date.today())
    if problem:
        print(problem, flush=True)
        return 2
    folder = report_folder(root, args.label)
    record = {'schema': SCHEMA, 'label': args.label, 'started_utc': _utc(), 'device_id': config['device_id'],
              'slave': SLAVE, 'approval': (os.environ if environ is None else environ)[APPROVAL_ENV],
              'register_source': REGISTER_SOURCE, 'values': 'raw 16-bit words; units and meaning unconfirmed',
              'timeout_s': config['timeout_s'], 'config': str(config_path), 'control_transmissions': 0}
    from wc_runtime.project_paths import runtime_root
    lease = FeedbackSerialLease(config, runtime_root(root))
    reads = record['reads'] = []
    try:
        try:
            lease.open()  # never transmits; any failure here means nothing was sent
        except Exception as refusal:  # noqa: BLE001 - busy port, identity or permission: all refusals
            record.update(status='PORT_REFUSED', error=str(refusal))
        else:
            note_owner(lease, entries)
            record['host_serial'] = lease.host_configuration
            error = read_all(lease, entries, config['timeout_s'], reads)
            record.update(status='TRANSPORT_FAILED' if error else 'COMPLETE', error=error)
            record['left_right'] = wheel_pairs(reads)
    finally:
        lease.close()
        record['finished_utc'] = _utc()
        record.setdefault('status', 'INTERRUPTED')
        # A read still IN_PROGRESS was interrupted and may have been sent.
        record['function_codes_sent'] = [3] if any(row.get('transmitted_bytes') or row['status'] == 'IN_PROGRESS'
                                                   for row in reads) else []
        (folder/'result.json').write_text(json.dumps(record, indent=1, ensure_ascii=False, allow_nan=False)+'\n',
                                          encoding='utf-8')
    for row in record['reads']:
        value = (' '.join('0x%s(%d)' % (text, word) for text, word in zip(row['words_hex'], row['words_u16']))
                 if row['status'] == 'WORDS' else 'exception %d' % row['exception_code']
                 if row['status'] == 'MODBUS_EXCEPTION' else row['status'])
        if row.get('exchange_completed_monotonic_ns') is not None:
            # The wheel owner's own readback at session start waits at most 0.1 s per read.
            elapsed = row['exchange_completed_monotonic_ns'] - row['exchange_started_monotonic_ns']
            value += '  %.1f ms' % (elapsed / 1e6)
        print('%s  %s' % (row['start_register'], value), flush=True)
    pairs = record.get('left_right') or []
    if pairs:
        print('left / right (raw words; = same, != different):', flush=True)
        for pair in pairs:
            print('  %-36s %-15s %6d  %-2s %6d  %s' % (pair['name'], pair['left_register'], pair['left'],
                                                    '=' if pair['equal'] else '!=', pair['right'],
                                                    pair['right_register']), flush=True)
        print('%d of %d per-wheel items differ' % (sum(not pair['equal'] for pair in pairs), len(pairs)), flush=True)
    print('%s: %s' % (record['status'], folder/'result.json'), flush=True)
    if record.get('error'):
        print(record['error'], flush=True)
    return 0 if record['status'] == 'COMPLETE' else 1


if __name__ == '__main__':
    raise SystemExit(main())
