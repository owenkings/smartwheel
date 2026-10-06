"""Exclusive, allowlisted 8030D feedback polling; no motor-control API.

Only FC03 slave 1 / register 0x20AB / two registers may be transmitted.
Transport response validity does not establish speed units or measurement time.
Startup/exit never transmit zero speed, enable, mode, fault or brake commands.
"""

import argparse
from collections import deque
import copy
import array
import errno
import hashlib
import json
import math
import os
from pathlib import Path
import select
import shutil
import signal
import stat
import subprocess
import sys
import threading
import time
import uuid

from wc_imu.ros_node import verify_device_identity, require_unoccupied
from .protocol import FeedbackError, parse_exchange, read_request


QUERY = read_request(1, 0x20AB, 2)
SERIAL = '0000000014'
# At the observed four events per 10 Hz read, 1024 event slots correspond to
# 25.6 s by count alone. The independent byte limit can fill sooner; neither
# budget guarantees tolerable disk latency or changes any source age limit.
JOURNAL_PENDING_EVENTS = 1024
JOURNAL_PENDING_BYTES = 4 * 1024 * 1024
JOURNAL_RECORD_BYTES = 65536
JOURNAL_CLOSE_TIMEOUT_S = 10.0
JOURNAL_BATCH_EVENTS = 64
JOURNAL_BATCH_BYTES = 65536
JOURNAL_BATCH_WAIT_S = .05


def bounded(value, name, low, high):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or not low <= value <= high:
        raise FeedbackError(f'{name} must be finite in [{low}, {high}]')
    return value


def validate_config(config):
    if config.get('schema') != 'wc_8030d_feedback_transport_v1' or config.get('read_request_reviewed') is not True or not config.get('request_evidence'):
        raise FeedbackError('explicit reviewed FC03 request evidence required')
    if config.get('request_hex') != QUERY.hex() or config.get('baud') != 115200:
        raise FeedbackError('this transport permits only the reviewed 115200 baud FC03 query')
    if config.get('hardware_serial') != SERIAL or config.get('device_id') != 'ZLAC8030D-'+SERIAL:
        raise FeedbackError('reviewed unique 8030D device identity required')
    if config.get('device') != '/dev/smartwheel_zlac8030' or config.get('expected_by_id') != '/dev/serial/by-id/usb-1a86_USB_Single_Serial_'+SERIAL+'-if00':
        raise FeedbackError('reviewed serial alias and by-id required; no numbered-port fallback')
    if config.get('usb_vid') != '1a86' or config.get('usb_pid') != '55d3':
        raise FeedbackError('reviewed USB VID/PID required')
    bounded(config.get('timeout_s'), 'transaction timeout', .01, .5)
    bounded(config.get('startup_settle_s', 1.0), 'startup settle', 0, 2.0)
    return config


def verify_identity(config):
    actual, identity = verify_device_identity(config['device'], config['expected_by_id'], config['hardware_serial'])
    result = subprocess.run(['udevadm', 'info', '--query=property', '--name', str(actual)],
                            capture_output=True, text=True, check=True, timeout=5)
    values = dict(line.split('=', 1) for line in result.stdout.splitlines() if '=' in line)
    expected = {'ID_SERIAL_SHORT': config['hardware_serial'], 'ID_VENDOR_ID': config['usb_vid'], 'ID_MODEL_ID': config['usb_pid']}
    if any(values.get(key) != value for key, value in expected.items()):
        raise FeedbackError('serial udev identity differs from reviewed VID/PID/serial')
    return actual, identity


def require_only_self(device):
    executable = shutil.which('fuser')
    if executable is None:
        raise FeedbackError('fuser unavailable for post-open ownership check')
    result = subprocess.run([executable, str(device)], capture_output=True, text=True, timeout=5, check=False)
    try:
        pids = {int(value) for value in result.stdout.split()}
    except ValueError as error:
        raise FeedbackError('cannot parse post-open serial ownership') from error
    if result.returncode != 0 or pids != {os.getpid()} or result.stderr.strip() not in ('', str(device)+':'):
        raise FeedbackError('serial owner appeared during open; no request transmitted')


def raw_attributes(previous, termios_module):
    t = termios_module
    attrs = copy.deepcopy(previous)
    for name in ('IGNBRK', 'BRKINT', 'PARMRK', 'ISTRIP', 'INLCR', 'IGNCR', 'ICRNL', 'IXON', 'IXOFF', 'IXANY'):
        attrs[0] &= ~getattr(t, name, 0)
    attrs[1] &= ~t.OPOST
    for name in ('CSIZE', 'PARENB', 'CSTOPB', 'CRTSCTS', 'HUPCL'):
        attrs[2] &= ~getattr(t, name, 0)
    attrs[2] |= t.CS8 | t.CLOCAL | t.CREAD
    for name in ('ECHO', 'ECHONL', 'ICANON', 'ISIG', 'IEXTEN'):
        attrs[3] &= ~getattr(t, name, 0)
    attrs[4] = attrs[5] = t.B115200
    attrs[6][t.VMIN] = attrs[6][t.VTIME] = 0
    return attrs


def validate_raw_attributes(attrs, termios_module):
    """Check effective requirements, allowing the kernel to normalize CBAUD.

    tcsetattr may encode the requested input/output speeds into c_cflag as
    well. The whole flags word is therefore not compared with its input value.
    """
    t = termios_module
    if attrs[4] != t.B115200 or attrs[5] != t.B115200:
        raise FeedbackError('host serial baud readback is not 115200')
    if attrs[2] & t.CSIZE != t.CS8 or attrs[2] & (t.CLOCAL | t.CREAD) != t.CLOCAL | t.CREAD:
        raise FeedbackError('host serial readback is not local 8-bit receive mode')
    prohibited = sum(getattr(t, key, 0) for key in ('PARENB', 'CSTOPB', 'CRTSCTS', 'HUPCL'))
    if attrs[2] & prohibited:
        raise FeedbackError('host serial parity/stop/flow/HUPCL readback is invalid')
    input_flags = sum(getattr(t, key, 0) for key in ('IGNBRK', 'BRKINT', 'PARMRK', 'ISTRIP',
        'INLCR', 'IGNCR', 'ICRNL', 'IXON', 'IXOFF', 'IXANY'))
    local_flags = sum(getattr(t, key, 0) for key in ('ECHO', 'ECHONL', 'ICANON', 'ISIG', 'IEXTEN'))
    if attrs[0] & input_flags or attrs[1] & t.OPOST or attrs[3] & local_flags:
        raise FeedbackError('host serial readback retains non-raw processing')
    if any(attrs[6][index] not in (0, b'\x00') for index in (t.VMIN, t.VTIME)):
        raise FeedbackError('host serial readback has blocking character timers')


class QueryFailure(FeedbackError):
    def __init__(self, message, response=b'', transmitted_bytes=0):
        super().__init__(message)
        self.response = bytes(response)
        self.transmitted_bytes = transmitted_bytes


class FeedbackSerialLease:
    """One owner across worktrees, fuser + identity checks + TIOCEXCL.

    Host tty configuration is 115200/8N1/raw/no flow/no HUPCL. No modem-control
    ioctls, retries, automatic reconnect or implicit device transmissions.
    """
    def __init__(self, config, run_root, *, identity_checker=None, occupancy_checker=None, exclusive_checker=None):
        self.config = validate_config(config)
        self.run_root = Path(run_root)
        self.identity_checker = identity_checker or verify_identity
        self.occupancy_checker = occupancy_checker or require_unoccupied
        self.exclusive_checker = exclusive_checker or require_only_self
        self.fd = self.lock = None
        self.device = None
        self.failed = False
        self.host_configuration = None

    def open(self):
        import fcntl
        import termios
        if self.fd is not None or not self.run_root.is_absolute():
            raise FeedbackError('closed lease and absolute shared runtime root required')
        device, identity = self.identity_checker(self.config)
        self.device = device
        locks = self.run_root.resolve()/'locks'
        locks.mkdir(parents=True, exist_ok=True)
        path = locks/f'serial-{os.major(identity.st_rdev)}-{os.minor(identity.st_rdev)}.lock'
        descriptor = os.open(path, os.O_CREAT | os.O_RDWR | os.O_CLOEXEC | os.O_NOFOLLOW, 0o600)
        self.lock = os.fdopen(descriptor, 'a+', encoding='utf-8')
        try:
            if not stat.S_ISREG(os.fstat(self.lock.fileno()).st_mode):
                raise FeedbackError('serial ownership lock is not a regular file')
            fcntl.flock(self.lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            self.occupancy_checker(device)
            checked_device, checked = self.identity_checker(self.config)
            if checked_device != device or (identity.st_rdev, identity.st_ino) != (checked.st_rdev, checked.st_ino):
                raise FeedbackError('serial identity changed during preflight')
            self.fd = os.open(str(device), os.O_RDWR | os.O_NONBLOCK | os.O_NOCTTY | os.O_CLOEXEC | os.O_NOFOLLOW)
            opened = os.fstat(self.fd)
            if not stat.S_ISCHR(opened.st_mode) or (opened.st_rdev, opened.st_ino) != (checked.st_rdev, checked.st_ino):
                raise FeedbackError('opened serial identity mismatch')
            fcntl.ioctl(self.fd, termios.TIOCEXCL)
            self.exclusive_checker(device)
            before = termios.tcgetattr(self.fd)
            desired = raw_attributes(before, termios)
            termios.tcsetattr(self.fd, termios.TCSANOW, desired)
            after = termios.tcgetattr(self.fd)
            validate_raw_attributes(after, termios)
            self.host_configuration = {'before_flags_and_speeds': before[:6], 'after_flags_and_speeds': after[:6],
                                       'device': str(device), 'access': 'O_RDWR_FC03_ONLY'}
            # Observe line states only. Never assert DTR/RTS or change direction.
            bits = array.array('i', [0])
            try:
                fcntl.ioctl(self.fd, termios.TIOCMGET, bits, True)
                self.host_configuration['modem_lines_observed'] = {
                    'raw': bits[0], 'dtr': bool(bits[0] & termios.TIOCM_DTR),
                    'rts': bool(bits[0] & termios.TIOCM_RTS)}
            except OSError as error:
                if error.errno not in (errno.ENOTTY, errno.EINVAL):
                    raise
                self.host_configuration['modem_lines_observed'] = 'UNSUPPORTED'
            # USB bridge settling is an explicit host parameter, not a device
            # reset or protocol response timeout; no bytes are read/discarded.
            delay = self.config.get('startup_settle_s', 1.0)
            self.host_configuration['startup_settle_s'] = delay
            time.sleep(delay)
            self.lock.seek(0)
            self.lock.truncate()
            json.dump({'pid': os.getpid(), 'serial': SERIAL, 'request_hex': QUERY.hex(), **self.host_configuration}, self.lock)
            self.lock.flush()
            return self
        except BaseException:
            self.close()
            raise

    def exchange(self, request, timeout_s):
        """Exactly one bounded transaction. Any failure permanently closes use."""
        if self.fd is None or self.failed or bytes(request) != QUERY:
            raise QueryFailure('closed/failed lease or non-allowlisted request')
        bounded(timeout_s, 'transaction timeout', .01, .5)
        response = bytearray()
        sent = 0
        try:
            # Existing/unexpected input cannot be associated with this request.
            # Do not silently flush it and then label a late response as current.
            readable, _, _ = select.select([self.fd], [], [], .002)
            if readable:
                response.extend(os.read(self.fd, 256))
                raise QueryFailure('unsolicited bytes before query', response)
            deadline = time.monotonic()+timeout_s
            _, writable, _ = select.select([], [self.fd], [], max(0, deadline-time.monotonic()))
            if not writable:
                raise QueryFailure('request write readiness timeout')
            sent = os.write(self.fd, QUERY)
            if sent != len(QUERY):
                raise QueryFailure('partial FC03 transmission; no retry', response, sent)
            expected = 9
            while len(response) < expected:
                remaining = deadline-time.monotonic()
                if remaining <= 0:
                    raise QueryFailure('response timeout; no retry', response, sent)
                readable, _, _ = select.select([self.fd], [], [], remaining)
                if not readable:
                    raise QueryFailure('response timeout; no retry', response, sent)
                chunk = os.read(self.fd, 256)
                if not chunk:
                    raise QueryFailure('serial disconnected; no reconnect', response, sent)
                response.extend(chunk)
                if len(response) >= 2 and response[1] == 0x83:
                    expected = 5
                if len(response) > expected:
                    raise QueryFailure('response overflow / overlapping traffic', response, sent)
            parse_exchange(QUERY, response)
            return bytes(response)
        except BaseException as error:
            self.failed = True
            if isinstance(error, (KeyboardInterrupt, SystemExit)):
                raise
            if isinstance(error, QueryFailure):
                raise
            raise QueryFailure(str(error), response, sent) from error

    def close(self):
        if self.fd is not None:
            os.close(self.fd)
            self.fd = None
        if self.lock is not None:
            self.lock.close()
            self.lock = None


class SyncJournal:
    """The existing default: write and flush each event on the calling thread."""
    def __init__(self, output):
        self.output = output
        self.written_events = 0
        self.final_fsync_complete = False

    def check_health(self):
        if self.output.closed:
            raise FeedbackError('JOURNAL_CLOSED')

    def __call__(self, record):
        self.check_health()
        self.output.write(json.dumps(record, allow_nan=False)+'\n')
        self.output.flush()
        self.written_events += 1

    def close(self):
        try:
            self.output.flush(); os.fsync(self.output.fileno())
            self.final_fsync_complete = True
        finally:
            self.output.close()

    def status(self):
        return {'mode': 'synchronous', 'written_events': self.written_events,
                'final_fsync_complete': self.final_fsync_complete}


class AsyncJournal:
    """Opt-in FIFO writer; the worker owns only the journal, never serial/ROS.

    Accepted application events are bounded including the in-flight batch.
    Only this worker coalesces FIFO records, up to 64 events/64 KiB per write
    and flush, with at most 50 ms of intentional aggregation wait per batch.
    Enqueue does not promise persistence. Normal close drains, flushes and
    fsyncs; a timeout never closes a descriptor still owned by a blocked worker.
    """
    def __init__(self, output, *, max_events=JOURNAL_PENDING_EVENTS,
                 max_bytes=JOURNAL_PENDING_BYTES, close_timeout_s=JOURNAL_CLOSE_TIMEOUT_S):
        if type(max_events) is not int or max_events < 1 or type(max_bytes) is not int or max_bytes < 1:
            raise FeedbackError('positive integer async journal bounds required')
        bounded(close_timeout_s, 'journal close timeout', .001, 90)
        self.output, self.max_events, self.max_bytes = output, max_events, max_bytes
        self.close_timeout_s = close_timeout_s
        self.condition = threading.Condition()
        self.queue = deque()  # Capacity includes both queued and in-flight events.
        self.closing = self.worker_exited = self.final_fsync_complete = False
        self.error = None
        self.enqueued_events = self.written_events = self.pending_events = self.pending_bytes = 0
        self.rejected_events = self.high_water_events = self.high_water_bytes = 0
        self.write_count = self.last_write_ns = self.max_write_ns = self.fsync_duration_ns = 0
        self.completed_batches = self.max_batch_events = self.max_batch_bytes = 0
        self.thread = threading.Thread(target=self._run, name='wheel-feedback-journal', daemon=True)
        self.thread.start()

    def _fail_locked(self, reason):
        self.error = self.error or str(reason)
        self.closing = True
        self.condition.notify_all()

    def check_health(self):
        with self.condition:
            if self.error or self.closing:
                raise FeedbackError(self.error or 'ASYNC_JOURNAL_CLOSED')

    def __call__(self, record):
        # Freeze data before handing it to another thread. No caller-owned
        # object or source timestamp is mutated, and no filesystem call occurs.
        with self.condition:
            if self.error or self.closing:
                raise FeedbackError(self.error or 'ASYNC_JOURNAL_CLOSED')
            try:
                line = json.dumps({**record, 'journal_event_index': self.enqueued_events+1,
                                   'journal_enqueued_monotonic_ns': time.monotonic_ns()}, allow_nan=False)+'\n'
                size = len(line.encode('utf-8'))
            except Exception as error:
                self._fail_locked('ASYNC_JOURNAL_ENCODE_FAILED: '+str(error))
                raise FeedbackError(self.error) from error
            if size > JOURNAL_RECORD_BYTES or self.pending_events >= self.max_events or self.pending_bytes+size > self.max_bytes:
                self.rejected_events += 1
                self._fail_locked('ASYNC_JOURNAL_RECORD_TOO_LARGE' if size > JOURNAL_RECORD_BYTES else 'ASYNC_JOURNAL_QUEUE_FULL')
                raise FeedbackError(self.error)
            self.queue.append((line, size))
            self.enqueued_events += 1
            self.pending_events += 1; self.pending_bytes += size
            self.high_water_events = max(self.high_water_events, self.pending_events)
            self.high_water_bytes = max(self.high_water_bytes, self.pending_bytes)
            self.condition.notify_all()

    def status(self):
        with self.condition:
            return {'mode': 'async', 'enqueued_events': self.enqueued_events, 'written_events': self.written_events,
                    'close_timeout_s': self.close_timeout_s,
                    'pending_events': self.pending_events, 'pending_bytes': self.pending_bytes,
                    'rejected_events': self.rejected_events, 'high_water_events': self.high_water_events,
                    'high_water_bytes': self.high_water_bytes, 'max_pending_events': self.max_events,
                    'max_pending_bytes': self.max_bytes, 'error': self.error, 'worker_exited': self.worker_exited,
                    'write_flush_count': self.write_count, 'last_write_flush_ns': self.last_write_ns,
                    'max_write_flush_ns': self.max_write_ns, 'final_fsync_duration_ns': self.fsync_duration_ns,
                    'completed_event_batches': self.completed_batches,
                    'max_event_batch_events': self.max_batch_events, 'max_event_batch_bytes': self.max_batch_bytes,
                    'batch_limits': {'events': JOURNAL_BATCH_EVENTS, 'bytes': JOURNAL_BATCH_BYTES,
                                     'aggregation_wait_s': JOURNAL_BATCH_WAIT_S},
                    'final_fsync_complete': self.final_fsync_complete,
                    'persistence': 'Queued/flushed events are not claimed durable before successful final fsync.',
                    'counts_exclude_writer_finalization_record': True}

    def _write(self, line):
        started = time.monotonic_ns()
        try:
            if self.output.write(line) != len(line):
                raise OSError('journal short text write')
            self.output.flush()
        finally:
            duration = time.monotonic_ns()-started
            with self.condition:
                self.write_count += 1
                self.last_write_ns = duration
                self.max_write_ns = max(self.max_write_ns, duration)

    def _run(self):
        try:
            while True:
                with self.condition:
                    while not self.queue and not self.closing:
                        self.condition.wait()
                    if not self.queue:
                        break
                    lines, size = [], 0
                    deadline = time.monotonic()+JOURNAL_BATCH_WAIT_S
                    while True:
                        while self.queue and len(lines) < JOURNAL_BATCH_EVENTS:
                            if lines and size+self.queue[0][1] > JOURNAL_BATCH_BYTES:
                                break
                            line, length = self.queue.popleft()
                            lines.append(line); size += length
                        byte_full = self.queue and size+self.queue[0][1] > JOURNAL_BATCH_BYTES
                        remaining = deadline-time.monotonic()
                        if len(lines) >= JOURNAL_BATCH_EVENTS or byte_full or self.closing or remaining <= 0:
                            break
                        self.condition.wait(remaining)  # Releases the lock; producers never wait on I/O.
                    self.max_batch_events = max(self.max_batch_events, len(lines))
                    self.max_batch_bytes = max(self.max_batch_bytes, size)
                self._write(''.join(lines))
                with self.condition:
                    self.completed_batches += 1
                    self.written_events += len(lines)
                    self.pending_events -= len(lines); self.pending_bytes -= size
            self._write(json.dumps({'event': 'journal_finalization', **self.status(),
                'note': 'This record precedes final fsync; process result must confirm final_fsync_complete.'}, allow_nan=False)+'\n')
            started = time.monotonic_ns()
            try:
                self.output.flush(); os.fsync(self.output.fileno())
                with self.condition:
                    self.final_fsync_complete = True
            finally:
                with self.condition:
                    self.fsync_duration_ns = time.monotonic_ns()-started
        except Exception as error:
            with self.condition:
                self._fail_locked('ASYNC_JOURNAL_WRITE_FAILED: '+str(error))
        finally:
            # Only the writer closes its stream. A main-thread timeout must
            # leave a kernel-blocked writer's handle alone until process exit.
            try:
                self.output.close()
            except Exception as error:
                with self.condition:
                    self._fail_locked('ASYNC_JOURNAL_CLOSE_FAILED: '+str(error))
            with self.condition:
                self.worker_exited = True
                self.condition.notify_all()

    def close(self):
        with self.condition:
            self.closing = True
            self.condition.notify_all()
        self.thread.join(self.close_timeout_s)
        with self.condition:
            if self.thread.is_alive():
                self._fail_locked('ASYNC_JOURNAL_CLOSE_TIMEOUT')
            if self.error:
                raise FeedbackError(self.error)
            if not self.final_fsync_complete:
                raise FeedbackError('ASYNC_JOURNAL_NOT_SYNCED')


def create_journal(path, asynchronous=False, *, close_timeout_s=JOURNAL_CLOSE_TIMEOUT_S):
    output = Path(path).open('x', encoding='utf-8')
    try:
        return AsyncJournal(output, close_timeout_s=close_timeout_s) if asynchronous else SyncJournal(output)
    except BaseException:
        output.close()
        raise


class FeedbackPoller:
    def __init__(self, config, lease, journal, *, epoch=None):
        self.config = validate_config(config)
        self.lease, self.journal = lease, journal
        self.epoch = epoch or str(uuid.uuid4())
        self.sequence = 0
        self.failed = False

    def sample(self):
        if self.failed:
            raise FeedbackError('failed stream may not retry; start a reviewed new session')
        base = {'schema': 'wc_wheel_feedback_v1', 'device_id': self.config['device_id'],
                'stream_epoch': self.epoch, 'sequence': self.sequence,
                'request_hex': QUERY.hex(), 'request_unix_ns': time.time_ns(),
                'request_monotonic_ns': time.monotonic_ns(), 'time_valid': False,
                'time_source': 'arrival_only', 'uncertainty_ns': None,
                'protocol_units_state': 'UNVALIDATED', 'formal_odometry_eligible': False}
        response = b''
        exchange_started = None
        try:
            self.check_journal()
            self.journal({'event': 'request_planned', **base})
            self.check_journal()
            exchange_started = time.monotonic_ns()
            response = self.lease.exchange(QUERY, self.config['timeout_s'])
            exchange_completed = time.monotonic_ns()
            _, _, words = parse_exchange(QUERY, response)
            record = {**base, 'stamp_ns': time.time_ns(), 'receive_monotonic_ns': time.monotonic_ns(),
                      'exchange_started_monotonic_ns': exchange_started,
                      'exchange_completed_monotonic_ns': exchange_completed,
                      'response_hex': response.hex(), 'response_sha256': hashlib.sha256(response).hexdigest(),
                      'status': 'RESPONSE_VALID', 'transmitted_bytes': len(QUERY),
                      'register_words_u16': list(words), 'control_transmissions': 0}
            self.check_journal()
            self.journal({'event': 'transaction_complete', **record})
            self.check_journal()
            self.sequence += 1
            return record
        except BaseException as error:
            self.failed = True
            try:
                self.check_journal()
                self.journal({'event': 'transaction_failed', **base, 'stamp_ns': time.time_ns(),
                    'receive_monotonic_ns': time.monotonic_ns(), 'status': 'ERROR', 'error': str(error),
                    'exchange_started_monotonic_ns': exchange_started,
                    'response_hex': getattr(error, 'response', response).hex(),
                    'transmitted_bytes': getattr(error, 'transmitted_bytes', len(QUERY) if response else
                                                  (0 if exchange_started is None else None)), 'control_transmissions': 0})
            except Exception:
                pass  # Preserve the original failure; an unhealthy writer cannot log another event.
            raise

    def check_journal(self):
        check = getattr(self.journal, 'check_health', None)
        if check is not None:
            check()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--summary-path', type=Path, help='Post-fsync source completion evidence')
    parser.add_argument('--run-root', type=Path, required=True)
    parser.add_argument('--samples', type=int, default=10, help='0 together with --max-duration 0 means until stopped')
    parser.add_argument('--rate-hz', type=float, default=10)
    parser.add_argument('--max-duration', type=float, default=300,
                        help='Acquisition limit, default 300, up to 7215; 0 together with --samples 0 means until stopped')
    parser.add_argument('--allow-read-queries', action='store_true')
    parser.add_argument('--publish-ros', action='store_true')
    parser.add_argument('--require-recorder', action='store_true', help='Wait for raw feedback subscription before serial open')
    parser.add_argument('--async-journal', action='store_true',
                        help='Opt-in bounded journal writer; FIFO events, final drain and fsync on close')
    parser.add_argument('--preview-history-calibration', action='store_true',
                        help='Opt-in UNVALIDATED historical wheel speed/distance preview; never formal odometry')
    parser.add_argument('--continuous-preview', action='store_true',
                        help='With historical preview, skip uncovered data gaps and resume on real samples')
    parser.add_argument('--preview-config', type=Path)
    parser.add_argument('--journal-close-timeout-s', type=float, default=JOURNAL_CLOSE_TIMEOUT_S)
    args = parser.parse_args(argv)
    if not args.allow_read_queries:
        raise FeedbackError('read queries require explicit --allow-read-queries')
    bounded(args.max_duration, 'max_duration', 0, 7215)
    until_stopped = args.max_duration == 0 and args.samples == 0
    if not until_stopped:
        bounded(args.max_duration, 'max_duration', 1, 7215)
        bounded(args.samples, 'sample count', 1, int(args.max_duration * 50))
    bounded(args.rate_hz, 'rate_hz', 1, 50)
    bounded(args.journal_close_timeout_s, 'journal close timeout', .001, 90)
    if args.samples/args.rate_hz > args.max_duration:
        raise FeedbackError('capture schedule exceeds the explicit acquisition duration')
    config = validate_config(json.loads(args.config.read_text(encoding='utf-8')))
    if args.preview_config and not args.preview_history_calibration:
        raise FeedbackError('--preview-config requires explicit --preview-history-calibration')
    if args.continuous_preview and not args.preview_history_calibration:
        raise FeedbackError('--continuous-preview requires explicit --preview-history-calibration')
    preview = None
    if args.preview_history_calibration:
        from .history_preview import HistoryPreview, assign_preview_odometry
        preview_path = args.preview_config or args.config.with_name('wheel_history_calibration.json')
        preview = HistoryPreview(json.loads(preview_path.read_text(encoding='utf-8')),
                                 continuous=args.continuous_preview)
    node = ros = publisher = preview_publisher = preview_diagnostics = None
    stopped = False

    def stop(signum, frame):
        nonlocal stopped
        stopped = True

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    lease = FeedbackSerialLease(config, args.run_root)
    result = 0
    completed = 0
    journal = create_journal(args.output, args.async_journal, close_timeout_s=args.journal_close_timeout_s)
    def safe_journal(record):
        nonlocal result
        try:
            journal(record)
        except Exception as error:
            result = 1
            print('feedback journal failed: '+str(error), file=sys.stderr, flush=True)
    try:
        if args.publish_ros:
            import rclpy as ros
            from std_msgs.msg import String
            from rclpy.qos import QoSProfile, ReliabilityPolicy
            from rclpy.signals import SignalHandlerOptions
            ros.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
            node = ros.create_node('wc_8030d_feedback_reader')
            publisher = node.create_publisher(String, '/wc_mapping/wheel/feedback_raw',
                QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE))
            if preview is not None:
                from nav_msgs.msg import Odometry
                preview_publisher = node.create_publisher(Odometry, '/wc_mapping/wheel/odom_preview', 20)
                preview_diagnostics = node.create_publisher(String, '/wc_mapping/wheel/preview_diagnostics', 20)
        if args.require_recorder:
            if publisher is None:
                raise FeedbackError('--require-recorder requires --publish-ros')
            deadline = time.monotonic()+10.
            while publisher.get_subscription_count() < 1:
                if stopped or time.monotonic() > deadline:
                    raise FeedbackError('recording subscriber discovery timeout')
                ros.spin_once(node, timeout_sec=.05)
        lease.open()
        journal({'event': 'lease_open', 'control_transmissions': 0, 'host_serial': lease.host_configuration})
        poller = FeedbackPoller(config, lease, journal)
        started = time.monotonic()
        while until_stopped or completed < args.samples:
            if stopped or (not until_stopped and time.monotonic()-started > args.max_duration):
                break
            sample_started = time.monotonic()
            record = poller.sample()
            poller.check_journal()
            completed += 1
            if publisher is not None:
                publisher.publish(String(data=json.dumps(record, allow_nan=False)))
            if preview is not None:
                candidate = preview.update(record)
                journal({'event': 'history_preview', **candidate})
                if preview_publisher is not None:
                    preview_publisher.publish(assign_preview_odometry(candidate, Odometry()))
                    preview_diagnostics.publish(String(data=json.dumps(candidate, allow_nan=False)))
            if node is not None:
                ros.spin_once(node, timeout_sec=0)
            deadline = sample_started+1/args.rate_hz  # No catch-up bursts after a delayed response.
            while not stopped and time.monotonic() < deadline:
                poller.check_journal()
                time.sleep(max(0, min(.02, deadline-time.monotonic())))
        if not stopped and not until_stopped and completed != args.samples:
            raise FeedbackError('capture reached wall-time bound before requested samples completed')
        journal({'event': 'capture_complete', 'completed': completed, 'requested': args.samples,
            'until_stopped': until_stopped,
            'interrupted': stopped, 'control_transmissions': 0, 'formal_odometry_eligible': False})
    except Exception as error:
        print("feedback capture failed: "+str(error), file=sys.stderr, flush=True)
        result = 1
        if preview is not None:
            preview.block(str(error))
            blocked = {'state': 'BLOCKED', 'classification': 'UNVALIDATED', 'reason': str(error),
                       'time_valid': False, 'formal_odometry_eligible': False, 'publishes_tf': False}
            safe_journal({'event': 'history_preview_blocked', **blocked})
            if preview_diagnostics is not None:
                preview_diagnostics.publish(String(data=json.dumps(blocked, allow_nan=False)))
        safe_journal({'event': 'capture_failed', 'completed': completed, 'error': str(error), 'control_transmissions': 0})
    finally:
        try:
            lease.close()
            if preview is not None:
                ended = {'state': 'BLOCKED' if preview.blocked else 'STOPPED', 'classification': 'UNVALIDATED',
                    'reason': preview.blocked or 'CAPTURE_FINISHED', 'completed': completed,
                    'time_valid': False, 'formal_odometry_eligible': False, 'publishes_tf': False}
                safe_journal({'event': 'history_preview_end', **ended})
                if preview_diagnostics is not None:
                    preview_diagnostics.publish(String(data=json.dumps(ended, allow_nan=False)))
            safe_journal({'event': 'lease_closed', 'control_transmissions': 0})
            if args.require_recorder and publisher is not None:
                from rclpy.duration import Duration
                acknowledged = publisher.wait_for_all_acked(Duration(seconds=5.0))
                safe_journal({'event': 'publication_ack', 'acknowledged': bool(acknowledged),
                              'timeout_s': 5.0})
                if not acknowledged:
                    raise FeedbackError('wheel reliable publication acknowledgement timeout')
        except Exception as error:
            result = 1
            print('feedback shutdown failed: '+str(error), file=sys.stderr, flush=True)
        try:
            journal.close()
        except Exception as error:
            result = 1
            print('feedback journal close failed: '+str(error), file=sys.stderr, flush=True)
        if node is not None:
            node.destroy_node()
        if ros is not None and ros.ok():
            ros.shutdown()
    summary = {'state': 'RAW_FEEDBACK_CAPTURED' if result == 0 else 'FAILED',
        'completed': completed, 'formal_odometry_eligible': False, 'control_transmissions': 0,
        'journal': journal.status()}
    if args.summary_path is not None:
        from wc_runtime.source_archive import atomic_json, digest
        summary.update(closed_normally=result==0, synchronized=result==0 and journal.status().get('final_fsync_complete') is True,
                       events_sha256=digest(args.output))
        atomic_json(args.summary_path, summary)
    print(json.dumps(summary))
    return result


if __name__ == '__main__':
    raise SystemExit(main())
