"""Linux PTY verification only; no physical motor or serial adapter is opened."""

import os
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'src'))

pytestmark = pytest.mark.skipif(sys.platform != 'linux', reason='requires Linux PTY; no hardware')


def test_passive_lease_reads_without_transmission_and_excludes_second_owner(tmp_path):
    import fcntl
    import pty
    import select
    import termios
    import tty
    from wc_motion.passive_serial import PassiveSerialLease
    master, slave = pty.openpty()
    device = Path(os.ttyname(slave))
    tty.setraw(slave)
    attrs = termios.tcgetattr(slave)
    attrs[2] &= ~(getattr(termios, 'HUPCL', 0) | getattr(termios, 'CRTSCTS', 0))
    attrs[4] = attrs[5] = termios.B115200
    termios.tcsetattr(slave, termios.TCSANOW, attrs)
    # Linux normalizes c_cflag speed bits in tcsetattr. Compare against the
    # actual kernel state immediately before the receive-only lease opens.
    attrs = termios.tcgetattr(slave)
    identity = lambda *args: (device, device.stat())
    occupied = []
    config = dict(alias=str(device), expected_by_id='/dev/serial/by-id/TEST_ONLY', hardware_serial='TEST_ONLY',
                  run_root=str(tmp_path), baud=115200, identity_checker=identity,
                  occupancy_checker=lambda path: occupied.append(path))
    lease = PassiveSerialLease(**config)
    other = PassiveSerialLease(**config)
    try:
        lease.open()
        with pytest.raises(BlockingIOError):
            other.open()
        assert lease.read() == b''
        payload = bytes.fromhex('01 03 02 00 00 B8 44')
        os.write(master, payload)  # Writes to the synthetic PTY, not any hardware.
        assert lease.read() == payload
        assert not select.select([master], [], [], 0)[0]  # No query/echo transmitted.
        assert occupied == [device]
        assert termios.tcgetattr(slave) == attrs
    finally:
        lease.close()
        other.close()
        # Test owns the PTY; release exclusivity before closing its handles.
        fcntl.ioctl(slave, termios.TIOCNXCL)
        os.close(master)
        os.close(slave)


def test_existing_wrong_baud_rejected_without_changing_tty(tmp_path):
    import fcntl
    import pty
    import termios
    import tty
    from wc_motion.passive_serial import PassiveSerialLease
    from wc_motion.protocol import FeedbackError
    master, slave = pty.openpty()
    device = Path(os.ttyname(slave))
    tty.setraw(slave)
    attrs = termios.tcgetattr(slave)
    attrs[4] = attrs[5] = termios.B9600
    termios.tcsetattr(slave, termios.TCSANOW, attrs)
    attrs = termios.tcgetattr(slave)
    lease = PassiveSerialLease(alias=str(device), expected_by_id='/dev/serial/by-id/TEST_ONLY', hardware_serial='TEST_ONLY',
                               run_root=str(tmp_path), baud=115200,
                               identity_checker=lambda *args: (device, device.stat()), occupancy_checker=lambda path: None)
    try:
        with pytest.raises(FeedbackError, match='already be configured'):
            lease.open()
        assert lease.fd is None and lease.lock is None
        assert termios.tcgetattr(slave) == attrs
    finally:
        lease.close()
        fcntl.ioctl(slave, termios.TIOCNXCL)
        os.close(master)
        os.close(slave)
