"""Bounded receive-only serial capture. No serial writes or tty reconfiguration.

This requires the host serial port to be configured already at the explicitly
specified baud in raw mode. A silent Modbus slave need not publish any bytes:
capture silence is a valid result, never synthesized zero wheel feedback.
"""

import argparse
import hashlib
import json
import os
from pathlib import Path
import select
import stat
import time

from wc_imu.ros_node import verify_device_identity, require_unoccupied
from .protocol import FeedbackError


class PassiveSerialLease:
    def __init__(self, *, alias, expected_by_id, hardware_serial, run_root, baud,
                 identity_checker=verify_device_identity, occupancy_checker=require_unoccupied):
        self.alias, self.expected_by_id, self.hardware_serial = alias, expected_by_id, hardware_serial
        self.run_root, self.baud = Path(run_root), baud
        self.identity_checker, self.occupancy_checker = identity_checker, occupancy_checker
        self.fd = self.lock = None
        self.device = None

    def open(self):
        import fcntl
        import termios
        if self.fd is not None or not self.run_root.is_absolute():
            raise FeedbackError('closed lease and absolute shared run_root required')
        speed = getattr(termios, 'B' + str(self.baud), None)
        if isinstance(self.baud, bool) or speed is None:
            raise FeedbackError('explicit supported host serial baud required')
        self.device, identity = self.identity_checker(self.alias, self.expected_by_id, self.hardware_serial)
        lock_dir = self.run_root.resolve() / 'locks'
        lock_dir.mkdir(parents=True, exist_ok=True)
        lock_path = lock_dir / f'serial-{os.major(identity.st_rdev)}-{os.minor(identity.st_rdev)}.lock'
        if lock_path.is_symlink():
            raise FeedbackError('serial lock may not be a symlink')
        self.lock = lock_path.open('a+', encoding='utf-8')
        try:
            fcntl.flock(self.lock.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
            self.occupancy_checker(self.device)
            device, checked = self.identity_checker(self.alias, self.expected_by_id, self.hardware_serial)
            if device != self.device or checked.st_rdev != identity.st_rdev or checked.st_ino != identity.st_ino:
                raise FeedbackError('serial identity changed during ownership check')
            self.fd = os.open(str(self.device), os.O_RDONLY | os.O_NONBLOCK | os.O_NOCTTY | os.O_CLOEXEC)
            actual = os.fstat(self.fd)
            if not stat.S_ISCHR(actual.st_mode) or actual.st_rdev != checked.st_rdev or actual.st_ino != checked.st_ino:
                raise FeedbackError('opened serial identity mismatch')
            fcntl.ioctl(self.fd, termios.TIOCEXCL)
            attrs = termios.tcgetattr(self.fd)
            prohibited = getattr(termios, 'CRTSCTS', 0) | getattr(termios, 'HUPCL', 0) | termios.PARENB | termios.CSTOPB
            if attrs[4] != speed or attrs[5] != speed or attrs[2] & prohibited or attrs[2] & termios.CSIZE != termios.CS8:
                raise FeedbackError('existing tty must already be configured as requested 8N1/no-flow/no-HUPCL')
            if attrs[3] & (termios.ICANON | termios.ECHO | termios.ISIG) or attrs[0] & (termios.IXON | termios.IXOFF | termios.ICRNL | termios.INLCR):
                raise FeedbackError('existing tty must already be raw; capture will not change settings')
            self.lock.seek(0)
            self.lock.truncate()
            json.dump({'pid': os.getpid(), 'access': 'O_RDONLY', 'baud': self.baud,
                       'device': str(device), 'serial': self.hardware_serial}, self.lock)
            self.lock.flush()
            return self
        except BaseException:
            self.close()
            raise

    def read(self, maximum=4096):
        if self.fd is None or not 1 <= maximum <= 65536:
            raise FeedbackError('bounded read from open lease required')
        readable, _, _ = select.select([self.fd], [], [], .05)
        if not readable:
            return b''
        try:
            chunk = os.read(self.fd, maximum)
        except BlockingIOError:
            return b''
        if not chunk:
            raise FeedbackError('serial disconnected; no automatic reconnect')
        return chunk

    def close(self):
        if self.fd is not None:
            os.close(self.fd)
            self.fd = None
        if self.lock is not None:
            self.lock.close()
            self.lock = None


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--device', required=True)
    parser.add_argument('--expected-by-id', required=True)
    parser.add_argument('--hardware-serial', required=True)
    parser.add_argument('--run-root', required=True)
    parser.add_argument('--baud', type=int, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--duration', type=float, default=10.)
    parser.add_argument('--max-bytes', type=int, default=10_000_000)
    args = parser.parse_args(argv)
    if not 0 < args.duration <= 300 or not 0 < args.max_bytes <= 100_000_000:
        raise FeedbackError('bounded duration<=300s and max-bytes<=100MB required')
    lease = PassiveSerialLease(alias=args.device, expected_by_id=args.expected_by_id,
                               hardware_serial=args.hardware_serial, run_root=args.run_root, baud=args.baud)
    count = chunks = 0
    try:
        # Exclusive creation prevents overwriting recordings, even on failure.
        with args.output.open('x', encoding='utf-8') as output:
            lease.open()
            deadline = time.monotonic() + args.duration
            while time.monotonic() < deadline:
                data = lease.read()
                if not data:
                    continue
                if count + len(data) > args.max_bytes:
                    raise FeedbackError('bounded capture size reached')
                record = {'schema': 'wc_wheel_serial_chunk_v1', 'sequence': chunks,
                          'device_id': args.hardware_serial, 'receive_ns': time.time_ns(),
                          'receive_monotonic_ns': time.monotonic_ns(), 'time_valid': False,
                          'time_source': 'arrival_only', 'raw_hex': data.hex(),
                          'sha256': hashlib.sha256(data).hexdigest()}
                output.write(json.dumps(record, allow_nan=False) + '\n')
                output.flush()
                count += len(data)
                chunks += 1
            output.flush()
            os.fsync(output.fileno())
    finally:
        lease.close()
    print(json.dumps({'state': 'RAW_CAPTURE_ONLY', 'bytes': count, 'chunks': chunks,
                      'wheel_odometry_published': False, 'control_transmissions': 0}))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
