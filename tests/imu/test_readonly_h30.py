"""Read-only serial boundary and H30 ROS-field semantics; never open real hardware."""

import copy
import json
import math
import os
from pathlib import Path
import stat
import struct
from types import SimpleNamespace as NS

import pytest

from wc_imu.ros_node import (ImuAcquisitionError, ReadOnlySerialLease, assign_ros_frame,
                            frame_record, raw_460800_attributes, require_unoccupied,
                            verify_device_identity)
from wc_sensors.h30 import H30Parser, h30_checksum


def _decoded(orientation=None):
    payload = b'\x10\x0c'+struct.pack('<iii', 0, 0, 9_810_000)
    payload += b'\x20\x0c'+struct.pack('<iii', 180_000_000, 0, 0)
    payload += b'\x51\x04'+struct.pack('<I', 123456)
    if orientation is not None:
        payload += b'\x41\x10'+struct.pack('<iiii', *orientation)
    packet = b'\x59\x53'+struct.pack('<H', 9)+bytes([len(payload)])+payload
    packet += h30_checksum(packet[2:])
    parser = H30Parser()
    assert parser.feed(packet[:7]) == []
    return parser.feed(packet[7:])[0]


def _record(frame, sequence=1):
    return frame_record(frame, session_id='SYN-session', sensor_id='SYN-H30', stream_epoch='SYN-boot',
                        frame_sequence=sequence, host_receive_ns=1_789_000_000_123_456_789, host_monotonic_ns=1234)


def _ros_frame():
    return NS(header=NS(stamp=NS(sec=0, nanosec=0), frame_id=''),
              imu=NS(header=None, orientation=NS(x=0., y=0., z=0., w=0.),
                     angular_velocity=NS(x=0., y=0., z=0.), linear_acceleration=NS(x=0., y=0., z=0.)))


def test_raw_packet_association_and_unknown_time_survive_ros_conversion():
    first, second = _record(_decoded(), 1), _record(_decoded(), 2)
    assert first['host_receive_ns'] == second['host_receive_ns']
    assert first['frame_sequence'] != second['frame_sequence']  # Same receive call can contain multiple packets.
    message = assign_ros_frame(first, _ros_frame())
    assert message.raw_packet == _decoded().raw_packet
    assert message.header.stamp.sec == 1_789_000_000 and message.header.stamp.nanosec == 123_456_789
    assert message.imu.header is message.header
    assert message.time_source == 'arrival_only'
    assert not message.common_time_valid and not message.uncertainty_valid
    assert message.sample_timestamp_valid and message.sample_timestamp_raw == 123456
    assert not message.dataready_timestamp_valid
    assert message.device_timestamp_unit == 'UNKNOWN_us_or_100us'
    assert message.coordinate_convention == 'H30_NATIVE_UNVALIDATED'


def test_absent_orientation_is_minus_one_not_fake_identity_and_units_correct():
    message = assign_ros_frame(_record(_decoded()), _ros_frame())
    assert message.orientation_valid is False
    assert message.imu.orientation_covariance == [-1.] + [0.]*8
    assert message.imu.orientation.w == 0.  # Inert default, explicitly invalid; no fabricated identity.
    assert message.angular_velocity_valid and message.linear_acceleration_valid
    assert message.imu.angular_velocity.x == pytest.approx(math.pi)
    assert message.imu.linear_acceleration.z == pytest.approx(9.81)
    assert message.imu.angular_velocity_covariance == [0.]*9  # Standard unknown covariance semantics.


def test_measured_quaternion_is_preserved_normalized_and_invalid_quaternion_rejected():
    valid = assign_ros_frame(_record(_decoded([1_000_000, 0, 0, 0])), _ros_frame())
    assert valid.orientation_valid and valid.imu.orientation.w == 1.
    invalid = assign_ros_frame(_record(_decoded([0, 0, 0, 0])), _ros_frame())
    assert invalid.orientation_valid is False and invalid.imu.orientation_covariance[0] == -1.


def test_raw_configuration_disables_all_flow_and_hangup_flags_without_mutating_input():
    import termios as t
    initial = [0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, t.B9600, t.B9600, [0]*32]
    copy_before = copy.deepcopy(initial)
    output = raw_460800_attributes(initial, t)
    assert initial == copy_before
    assert output[4] == output[5] == t.B460800
    assert not output[0] & (t.IXON | t.IXOFF | getattr(t, 'IXANY', 0))
    assert not output[2] & (t.HUPCL | getattr(t, 'CRTSCTS', 0))
    assert output[2] & t.CREAD and output[2] & t.CLOCAL
    assert output[2] & t.CSIZE == t.CS8
    assert output[6][t.VMIN] == output[6][t.VTIME] == 0


def test_lease_uses_readonly_flags_exclusive_ioctl_and_never_transmits(monkeypatch, tmp_path):
    import fcntl
    import termios as t
    from wc_runtime import project_paths
    locks = tmp_path/'shared_locks'; locks.mkdir()
    monkeypatch.setattr(project_paths, 'shared_lock_root', lambda: locks)
    calls = []
    identity = NS(st_mode=stat.S_IFCHR, st_rdev=os.makedev(166, 1), st_ino=123)
    state = {'attrs': [0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, t.B9600, t.B9600, [0]*32]}
    def opened(path, flags):
        assert path == '/dev/ttyACM1'
        assert flags & os.O_ACCMODE == os.O_RDONLY
        assert flags & os.O_NOCTTY and flags & os.O_NONBLOCK and flags & os.O_CLOEXEC
        calls.append('readonly_open')
        return 9001
    monkeypatch.setattr(os, 'open', opened)
    monkeypatch.setattr(os, 'fstat', lambda fd: identity)
    monkeypatch.setattr(os, 'close', lambda fd: calls.append('close'))
    monkeypatch.setattr(os, 'write', lambda *a: pytest.fail('serial transmit path invoked'))
    monkeypatch.setattr(fcntl, 'ioctl', lambda fd, request: calls.append(('ioctl', request)))
    monkeypatch.setattr(t, 'tcgetattr', lambda fd: copy.deepcopy(state['attrs']))
    def setattrs(fd, when, attrs):
        assert when == t.TCSANOW
        state['attrs'] = copy.deepcopy(attrs)
        calls.append('raw_460800')
    monkeypatch.setattr(t, 'tcsetattr', setattrs)
    monkeypatch.setattr(t, 'tcflush', lambda *a: pytest.fail('input flush loses authoritative data'))
    monkeypatch.setattr(t, 'tcsendbreak', lambda *a: pytest.fail('serial break is not read-only'))
    lease = ReadOnlySerialLease('/dev/smartwheel_h30_imu', '/dev/serial/by-id/SYN', tmp_path,
        'SYN', identity_checker=lambda *a: (Path('/dev/ttyACM1'), identity),
        occupancy_checker=lambda *a: calls.append('ownership_check'))
    lease.open()
    lease.close()
    assert calls == ['ownership_check', 'readonly_open', ('ioctl', t.TIOCEXCL), 'raw_460800', 'close']
    assert lease.fd is None and lease.lock_file is None
    metadata = json.loads(next(locks.iterdir()).read_text())
    assert metadata['access'] == 'O_RDONLY' and metadata['baud'] == 460800


def test_existing_os_lock_prevents_any_device_open(monkeypatch, tmp_path):
    import fcntl
    from wc_runtime import project_paths
    identity = NS(st_mode=stat.S_IFCHR, st_rdev=os.makedev(166, 1), st_ino=123)
    locks = tmp_path/'locks'
    locks.mkdir()
    monkeypatch.setattr(project_paths, 'shared_lock_root', lambda: locks)
    owner = (locks/'serial-166-1.lock').open('a+')
    try:
        fcntl.flock(owner.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        monkeypatch.setattr(os, 'open', lambda *a: pytest.fail('contended lock must prevent device open'))
        for clone in ('clone-A', 'clone-B'):
            lease = ReadOnlySerialLease('/dev/x', '/dev/serial/by-id/SYN', tmp_path/clone, 'SYN',
                 identity_checker=lambda *a: (Path('/dev/ttyACM1'), identity), occupancy_checker=lambda *a: None)
            with pytest.raises(BlockingIOError):
                lease.open()
            assert lease.fd is None and lease.lock_file is None
    finally:
        owner.close()


def test_occupied_or_unknown_ownership_blocks(monkeypatch):
    import wc_imu.ros_node as module
    monkeypatch.setattr(module.shutil, 'which', lambda name: '/usr/bin/fuser')
    monkeypatch.setattr(module.subprocess, 'run', lambda *a, **kw: NS(returncode=0, stdout='1234', stderr=''))
    with pytest.raises(ImuAcquisitionError, match='already owned'):
        require_unoccupied('/dev/SYN')
    monkeypatch.setattr(module.subprocess, 'run', lambda *a, **kw: NS(returncode=1, stdout='', stderr='Permission denied'))
    with pytest.raises(ImuAcquisitionError, match='incomplete'):
        require_unoccupied('/dev/SYN')


def test_installed_fuser_detects_owned_and_closed_test_file(tmp_path):
    # A real OS query against an owned regular fixture, never a sensor handle.
    path=tmp_path/'fuser-fixture'
    with path.open('w'):
        with pytest.raises(ImuAcquisitionError,match='already owned'):
            require_unoccupied(path)
    require_unoccupied(path)


def test_identity_mismatch_rejected_before_device_open(monkeypatch):
    original_resolve = Path.resolve
    def resolve(path, strict=False):
        if str(path) == '/dev/smartwheel_h30_imu':
            return Path('/dev/ttyACM0')
        if str(path).startswith('/dev/serial/by-id/'):
            return Path('/dev/ttyACM1')
        return original_resolve(path, strict=strict)
    monkeypatch.setattr(Path, 'resolve', resolve)
    with pytest.raises(ImuAcquisitionError, match='mismatch'):
        verify_device_identity('/dev/smartwheel_h30_imu', '/dev/serial/by-id/usb-SYN-serial', 'SYN')


def test_read_is_bounded_and_disconnect_never_reconnects(monkeypatch):
    import wc_imu.ros_node as module
    lease = ReadOnlySerialLease('/dev/x', '/dev/y', '/tmp/SYN', 'SYN')
    lease.fd = 9001
    monkeypatch.setattr(module.select, 'select', lambda *a: ([9001], [], []))
    monkeypatch.setattr(os, 'read', lambda fd, size: b'')
    monkeypatch.setattr(os, 'open', lambda *a: pytest.fail('disconnect must not trigger automatic reconnect'))
    with pytest.raises(ImuAcquisitionError, match='disconnect'):
        lease.read_available()
    with pytest.raises(ImuAcquisitionError, match='bounded'):
        lease.read_available(999999)
    lease.fd = None
