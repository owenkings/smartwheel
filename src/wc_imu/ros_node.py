"""H30 read-only acquisition and ROS 2 observation publishing.

Only os.read touches serial bytes. No command writes, break, flush, DTR/RTS
modem-control ioctl, automatic reconnect, IMU reset, or TF broadcast exists.
The host tty is configured raw/460800 with software/hardware flow control and
HUPCL disabled. Raw frames remain authoritative; measurement timing is unknown.
"""

import argparse
import copy
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
import time
import uuid

from wc_sensors.h30 import H30Parser


class ImuAcquisitionError(RuntimeError):
    pass


def verify_device_identity(alias, expected_by_id, hardware_serial):
    alias, by_id = Path(alias), Path(expected_by_id)
    if not alias.is_absolute() or not by_id.is_absolute() or by_id.parent != Path('/dev/serial/by-id'):
        raise ImuAcquisitionError('explicit absolute alias and /dev/serial/by-id path required')
    if not hardware_serial or hardware_serial not in by_id.name:
        raise ImuAcquisitionError('expected by-id does not contain configured hardware serial')
    actual, expected = alias.resolve(strict=True), by_id.resolve(strict=True)
    if actual != expected:
        raise ImuAcquisitionError('IMU alias/by-id realpath identity mismatch')
    identity = actual.stat()
    if not stat.S_ISCHR(identity.st_mode):
        raise ImuAcquisitionError('IMU target is not a character device')
    return actual, identity


def require_unoccupied(device):
    executable = shutil.which('fuser')
    if executable is None:
        raise ImuAcquisitionError('fuser unavailable: cannot verify current serial ownership')
    # Installed psmisc fuser does not accept the generic '--' terminator.
    # The caller supplies an identity-checked absolute path, never an option.
    if not Path(device).is_absolute():
        raise ImuAcquisitionError('ownership query requires an absolute device path')
    result = subprocess.run([executable, str(device)], capture_output=True, text=True, timeout=5, check=False)
    if result.returncode == 0:
        raise ImuAcquisitionError('serial device already owned by process(es): '+result.stdout.strip())
    if result.returncode != 1 or result.stdout.strip() or result.stderr.strip():
        raise ImuAcquisitionError('serial ownership check incomplete: '+result.stderr.strip())


def raw_460800_attributes(previous, termios_module):
    """Pure host-configuration function, separately testable without any serial fd."""
    t = termios_module
    if not hasattr(t, 'B460800'):
        raise ImuAcquisitionError('target termios does not expose B460800')
    value = copy.deepcopy(previous)
    for name in ('IGNBRK', 'BRKINT', 'PARMRK', 'ISTRIP', 'INLCR', 'IGNCR', 'ICRNL', 'IXON', 'IXOFF', 'IXANY'):
        value[0] &= ~getattr(t, name, 0)
    value[1] &= ~t.OPOST
    for name in ('CSIZE', 'PARENB', 'CSTOPB', 'CRTSCTS', 'HUPCL'):
        value[2] &= ~getattr(t, name, 0)
    value[2] |= t.CS8 | t.CLOCAL | t.CREAD
    for name in ('ECHO', 'ECHONL', 'ICANON', 'ISIG', 'IEXTEN'):
        value[3] &= ~getattr(t, name, 0)
    value[4] = value[5] = t.B460800
    value[6][t.VMIN] = value[6][t.VTIME] = 0
    return value


class ReadOnlySerialLease:
    """One shared RUN_ROOT OS lock plus tty exclusivity and identity checks."""

    def __init__(self, alias, expected_by_id, run_root, hardware_serial, *, identity_checker=None, occupancy_checker=None):
        self.alias, self.by_id = str(alias), str(expected_by_id)
        self.run_root, self.hardware_serial = Path(run_root), str(hardware_serial)
        self.identity_checker = identity_checker or verify_device_identity
        self.occupancy_checker = occupancy_checker or require_unoccupied
        self.fd, self.lock_file, self.device = None, None, None

    def open(self):
        import fcntl
        import termios
        if self.fd is not None:
            raise ImuAcquisitionError('serial lease already open')
        if not self.run_root.is_absolute():
            raise ImuAcquisitionError('shared RUN_ROOT must be an explicit absolute path')
        self.device, identity = self.identity_checker(self.alias, self.by_id, self.hardware_serial)
        locks = self.run_root.resolve() / 'locks'
        locks.mkdir(parents=True, exist_ok=True)
        lock_path = locks / f'serial-h30-{os.major(identity.st_rdev)}-{os.minor(identity.st_rdev)}.lock'
        if lock_path.is_symlink():
            raise ImuAcquisitionError('resource lock may not be a symlink')
        self.lock_file = lock_path.open('a+', encoding='utf-8')
        try:
            fcntl.flock(self.lock_file.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
            self.lock_file.seek(0)
            self.lock_file.truncate()
            json.dump({'pid': os.getpid(), 'resource': str(self.device), 'by_id': self.by_id,
                       'serial': self.hardware_serial, 'access': 'O_RDONLY', 'baud': 460800}, self.lock_file)
            self.lock_file.flush()
            self.occupancy_checker(self.device)
            actual, checked = self.identity_checker(self.alias, self.by_id, self.hardware_serial)
            if actual != self.device or checked.st_rdev != identity.st_rdev:
                raise ImuAcquisitionError('serial identity changed during preflight')
            self.fd = os.open(str(self.device), os.O_RDONLY | os.O_NONBLOCK | os.O_NOCTTY | os.O_CLOEXEC)
            opened = os.fstat(self.fd)
            if not stat.S_ISCHR(opened.st_mode) or opened.st_rdev != checked.st_rdev or opened.st_ino != checked.st_ino:
                raise ImuAcquisitionError('opened serial descriptor identity mismatch')
            fcntl.ioctl(self.fd, termios.TIOCEXCL)
            attributes = raw_460800_attributes(termios.tcgetattr(self.fd), termios)
            termios.tcsetattr(self.fd, termios.TCSANOW, attributes)
            # Confirm the host accepted the requested speed and flow-control mask.
            observed = termios.tcgetattr(self.fd)
            if observed[4] != termios.B460800 or observed[5] != termios.B460800 or \
                    observed[2] & (getattr(termios, 'HUPCL', 0) | getattr(termios, 'CRTSCTS', 0)):
                raise ImuAcquisitionError('host serial settings failed verification')
            return self
        except BaseException:
            self.close()
            raise

    def read_available(self, max_bytes=4096):
        if self.fd is None or not 1 <= max_bytes <= 65536:
            raise ImuAcquisitionError('closed serial lease or invalid bounded read size')
        ready, _, _ = select.select([self.fd], [], [], 0)
        if not ready:
            return b''
        try:
            data = os.read(self.fd, max_bytes)
        except BlockingIOError:
            return b''
        if not data:
            raise ImuAcquisitionError('serial EOF/disconnect; automatic reconnect is disabled')
        return data

    def close(self):
        # HUPCL stays disabled; restoring an original HUPCL bit would request a
        # hangup on close. No modem line toggles are explicitly issued here.
        if self.fd is not None:
            os.close(self.fd)
            self.fd = None
        if self.lock_file is not None:
            self.lock_file.close()  # OS releases flock; never delete a live lock.
            self.lock_file = None


def frame_record(frame, *, session_id, sensor_id, stream_epoch, frame_sequence,
                 host_receive_ns, host_monotonic_ns):
    for value in (frame_sequence, host_receive_ns, host_monotonic_ns):
        if isinstance(value, bool) or not isinstance(value, int) or value < 0:
            raise ImuAcquisitionError('nonnegative integer sequence and host ns required')
    if not all(isinstance(value, str) and value for value in (session_id, sensor_id, stream_epoch)):
        raise ImuAcquisitionError('nonempty stable frame identity required')
    imu = frame.imu_dict(quaternion_norm_tolerance=0.001)
    if imu['orientation_valid']:
        imu['orientation'] = tuple(component / imu['orientation_norm'] for component in imu['orientation'])
    sample, ready = imu['sample_timestamp_raw'], imu['dataready_timestamp_raw']
    return {'session_id': session_id, 'sensor_id': sensor_id, 'stream_epoch': stream_epoch,
            'frame_sequence': frame_sequence, 'tid': frame.tid, 'raw_packet': frame.raw_packet,
            'packet_sha256': hashlib.sha256(frame.raw_packet).hexdigest(), 'imu': imu,
            'sample_timestamp_valid': sample is not None, 'sample_timestamp_raw': sample,
            'dataready_timestamp_valid': ready is not None, 'dataready_timestamp_raw': ready,
            'host_receive_ns': host_receive_ns, 'host_monotonic_ns': host_monotonic_ns,
            'device_timestamp_unit': imu['device_timestamp_unit'], 'coordinate_convention': 'H30_NATIVE_UNVALIDATED',
            'time_source': 'arrival_only', 'common_time_valid': False, 'common_time_ns': None,
            'uncertainty_valid': False, 'uncertainty_ns': None,
            'diagnostic_flags': ['ARRIVAL_ONLY', 'DEVICE_TIME_UNIT_UNRESOLVED', 'IMU_MOUNT_EXTRINSIC_UNVALIDATED',
                                 'NATIVE_ORIENTATION_REFERENCE_UNVALIDATED', 'COVARIANCE_UNKNOWN']}


def assign_ros_frame(record, frame_message, *, frame_id='imu_h30_native'):
    """Populate a strong H30Frame; no time-only sidecar association is needed."""
    seconds, nanos = divmod(record['host_receive_ns'], 1_000_000_000)
    if not -(2**31) <= seconds < 2**31:
        raise ImuAcquisitionError('arrival timestamp outside ROS Time sec range')
    frame_message.header.stamp.sec, frame_message.header.stamp.nanosec = seconds, nanos
    frame_message.header.frame_id = frame_id
    for field in ('session_id', 'sensor_id', 'stream_epoch', 'frame_sequence', 'tid',
                  'sample_timestamp_valid', 'dataready_timestamp_valid', 'device_timestamp_unit',
                  'coordinate_convention', 'time_source', 'host_monotonic_ns', 'common_time_valid', 'uncertainty_valid'):
        setattr(frame_message, field, record[field])
    frame_message.raw_packet = record['raw_packet']
    frame_message.sample_timestamp_raw = record['sample_timestamp_raw'] if record['sample_timestamp_valid'] else 0
    frame_message.dataready_timestamp_raw = record['dataready_timestamp_raw'] if record['dataready_timestamp_valid'] else 0
    frame_message.host_receive_time = frame_message.header.stamp
    frame_message.common_time_ns, frame_message.uncertainty_ns = 0, 0  # Unknown flags above are authoritative.
    frame_message.diagnostic_flags = record['diagnostic_flags']
    imu, values = frame_message.imu, record['imu']
    imu.header = frame_message.header
    frame_message.orientation_valid = values['orientation_valid']
    frame_message.angular_velocity_valid = values['field_validity']['angular_velocity']
    frame_message.linear_acceleration_valid = values['field_validity']['linear_acceleration']
    if values['orientation_valid']:
        imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w = values['orientation']
    for vector_name in ('angular_velocity', 'linear_acceleration'):
        vector = values[vector_name]
        if vector is not None:
            destination = getattr(imu, vector_name)
            destination.x, destination.y, destination.z = vector
    for field in ('orientation_covariance', 'angular_velocity_covariance', 'linear_acceleration_covariance'):
        setattr(imu, field, values[field])
    return frame_message


def poll_period_seconds(value):
    """Allow only explicit reviewed poll periods; no automatic cadence change."""
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value not in (2.5, 5.0, 10.0):
        raise ImuAcquisitionError('poll-period-ms must be exactly 2.5, 5 or 10')
    return value / 1000.0


def read_with_host_arrival(serial):
    """One read-completion time pair per byte batch, not per-device sample time."""
    data = serial.read_available()
    return data, time.time_ns(), time.monotonic_ns()


def parser():
    result = argparse.ArgumentParser(description=__doc__)
    result.add_argument('--device', default='/dev/smartwheel_h30_imu')
    result.add_argument('--expected-by-id', required=True)
    result.add_argument('--hardware-serial', default='0000000015')
    result.add_argument('--sensor-id', default='H30-0000000015')
    result.add_argument('--session-id', required=True)
    result.add_argument('--journal-dir', type=Path, help='Optional source-side packets, read batches and synchronized summary')
    result.add_argument('--require-recorder', action='store_true', help='Wait for source subscription before serial open')
    result.add_argument('--run-root', type=Path, required=True)
    result.add_argument('--duration', type=float, default=60.0, help='0 means until stopped; otherwise a positive duration up to 12h')
    result.add_argument('--stale-timeout', type=float, default=2.0,
                        help='seconds without a valid frame before exit; 0 keeps waiting for data until stopped')
    result.add_argument('--poll-period-ms', type=float, choices=(2.5, 5.0, 10.0), default=10.0,
                        help='explicit host serial poll period; arrival time remains unvalidated measurement time')
    return result


def main(argv=None):
    arguments, ros_arguments = parser().parse_known_args(argv)
    if (not math.isfinite(arguments.duration) or not 0 <= arguments.duration <= 43200 or
            not math.isfinite(arguments.stale_timeout) or arguments.stale_timeout < 0):
        raise ImuAcquisitionError('duration must be 0 (until stopped) or positive up to 12h; stale-timeout must be finite nonnegative')
    poll_seconds = poll_period_seconds(arguments.poll_period_ms)
    import rclpy
    from rclpy.duration import Duration
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.signals import SignalHandlerOptions
    from sensor_msgs.msg import Imu
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
    from wc_interfaces.msg import H30Frame

    stopped = False

    def request_stop(signum, frame):
        # Never interrupt append -> publish or shut down the ROS context here.
        nonlocal stopped
        stopped = True

    class H30Node(Node):
        def __init__(self):
            super().__init__('wc_h30_readonly')
            if self.get_parameter('use_sim_time').value:
                raise ImuAcquisitionError('live serial acquisition cannot use simulated clock')
            self.parser = H30Parser()
            self.sequence, self.latest, self.error, self.done = 0, None, None, False
            self.stream_epoch = str(uuid.uuid4())
            self.started, self.last_receive = time.monotonic_ns(), None
            self.bytes_received = 0
            self.batch_sequence = 0
            self.journal = None
            self.serial = None
            if arguments.journal_dir is not None:
                from wc_runtime.source_archive import SourceJournal
                self.journal = SourceJournal(arguments.journal_dir, arguments.sensor_id)
            frame_qos = qos_profile_sensor_data
            if arguments.require_recorder:
                from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
                frame_qos = QoSProfile(depth=2048,reliability=ReliabilityPolicy.RELIABLE,
                                       durability=DurabilityPolicy.VOLATILE)
            self.frame_pub = self.create_publisher(H30Frame, '/wc_mapping/imu/source_frame', frame_qos)
            self.imu_pub = self.create_publisher(Imu, '/wc_mapping/imu/data_raw', qos_profile_sensor_data)
            self.diag_pub = self.create_publisher(DiagnosticArray, '/wc_mapping/imu/diagnostics', 4)
            try:
                if arguments.require_recorder:
                    deadline = time.monotonic()+10.
                    while self.frame_pub.get_subscription_count() < 1:
                        if stopped:
                            raise ImuAcquisitionError('stopped before recording subscriber discovery')
                        if time.monotonic() > deadline or not rclpy.ok():
                            raise ImuAcquisitionError('recording subscriber discovery timeout')
                        time.sleep(.05)
                if stopped:
                    raise ImuAcquisitionError('stopped before serial acquisition')
                self.serial = ReadOnlySerialLease(arguments.device, arguments.expected_by_id, arguments.run_root,
                                                   arguments.hardware_serial).open()
                self.create_timer(poll_seconds, self.acquire)
                self.create_timer(.25, self.diagnostics)
            except BaseException as error:
                self.error = str(error)
                try:
                    self.close()
                finally:
                    self.destroy_node()
                raise

        def acquire(self):
            if self.done or stopped:
                return
            try:
                data, arrival, now = read_with_host_arrival(self.serial)
                if data:
                    self.bytes_received += len(data)
                    self.batch_sequence += 1
                    if self.journal:
                        self.journal.append({'event': 'byte_batch', 'batch_id': self.batch_sequence,
                            'host_receive_ns': arrival, 'host_monotonic_ns': now,
                            'bytes_hex': data.hex(), 'sha256': hashlib.sha256(data).hexdigest()})
                    for decoded in self.parser.feed(data):
                        self.sequence += 1
                        record = frame_record(decoded, session_id=arguments.session_id, sensor_id=arguments.sensor_id,
                            stream_epoch=self.stream_epoch, frame_sequence=self.sequence,
                            host_receive_ns=arrival, host_monotonic_ns=now)
                        output = assign_ros_frame(record, H30Frame())
                        if self.journal:
                            self.journal.append({'event': 'frame', 'sensor_id': arguments.sensor_id,
                                'stream_epoch': self.stream_epoch, 'sequence': self.sequence,
                                'batch_id': self.batch_sequence, 'packet_sha256': record['packet_sha256'],
                                'host_receive_ns': arrival, 'host_monotonic_ns': now})
                        self.frame_pub.publish(output)
                        self.imu_pub.publish(output.imu)
                        self.latest, self.last_receive = record, now
                if arguments.stale_timeout > 0 and now - (self.last_receive if self.last_receive is not None else self.started) > arguments.stale_timeout * 1e9:
                    raise ImuAcquisitionError('no fresh checksum-valid H30 frame within timeout')
                if arguments.duration > 0 and now - self.started >= arguments.duration * 1e9:
                    self.done = True
            except (OSError, ValueError, RuntimeError) as error:
                self.error, self.done = str(error), True
                self.get_logger().error(self.error)
                self.diagnostics()

        def diagnostics(self):
            message = DiagnosticArray()
            message.header.stamp = self.get_clock().now().to_msg()
            message.header.frame_id = 'imu_h30_native'
            info = dict(session_id=arguments.session_id, sensor_id=arguments.sensor_id, stream_epoch=self.stream_epoch,
                        frame_sequence=self.sequence, bytes_received=self.bytes_received, source='live_readonly',
                        parser=self.parser.stats, time_source='arrival_only', common_time_valid=False,
                        uncertainty_valid=False, mount_extrinsic_valid=False, error=self.error,
                        requested_poll_period_ms=arguments.poll_period_ms,
                        stale_timeout_s=arguments.stale_timeout, silence_exit_enabled=arguments.stale_timeout > 0,
                        source_age_s=None if self.last_receive is None else max(0, time.monotonic_ns()-self.last_receive)/1e9,
                        waiting_for_first_frame=self.last_receive is None,
                        raw_recording_topic='/wc_mapping/imu/source_frame')
            if self.latest is not None:
                info.update(tid=self.latest['tid'], packet_sha256=self.latest['packet_sha256'],
                            host_receive_ns=self.latest['host_receive_ns'], host_monotonic_ns=self.latest['host_monotonic_ns'],
                            field_validity=self.latest['imu']['field_validity'])
            status = DiagnosticStatus(name='h30_readonly', hardware_id=arguments.hardware_serial,
                level=DiagnosticStatus.ERROR if self.error else DiagnosticStatus.WARN,
                message=self.error or 'Native IMU observations; time and mounting extrinsic unvalidated')
            status.values = [KeyValue(key=key, value=json.dumps(value, ensure_ascii=False)) for key, value in info.items()]
            message.status = [status]
            self.diag_pub.publish(message)

        def close(self):
            self.parser.reset()
            acquired = self.serial is not None
            if acquired:
                try:
                    self.serial.close()
                except Exception as error:
                    self.error = '; '.join(filter(None, (self.error, 'IMU_SERIAL_CLOSE_FAILED: '+str(error))))
            # The recorder stays alive during source shutdown. Keep the context
            # and publisher alive until every already-published reliable frame
            # is acknowledged, with a bounded wait. This is transport evidence;
            # the capture byte audit still decides persistence completeness.
            if arguments.require_recorder and acquired:
                try:
                    if not self.frame_pub.wait_for_all_acked(Duration(seconds=3.0)):
                        raise ImuAcquisitionError('IMU_RELIABLE_ACK_TIMEOUT: 3 seconds')
                except Exception as error:
                    self.error = '; '.join(filter(None, (self.error, 'IMU_RELIABLE_ACK_FAILED: '+str(error))))
                    self.get_logger().error(self.error)
            if self.journal:
                self.journal.close(source_error=self.error)

    node = None
    code = 0
    previous_handlers = {}
    initialized = False
    try:
        for name in ('SIGINT', 'SIGTERM', 'SIGHUP'):
            signum = getattr(signal, name, None)
            if signum is not None and signum not in previous_handlers:
                previous_handlers[signum] = signal.getsignal(signum)
                signal.signal(signum, request_stop)
        rclpy.init(args=ros_arguments, signal_handler_options=SignalHandlerOptions.NO)
        initialized = True
        node = H30Node()
        while rclpy.ok() and not node.done and not stopped:
            rclpy.spin_once(node, timeout_sec=.1)
        if not rclpy.ok():
            node.error = node.error or 'ROS_CONTEXT_SHUTDOWN_BEFORE_SOURCE_CLOSE'
    finally:
        try:
            if node is not None:
                try:
                    node.close()
                    code = 2 if node.error else 0
                finally:
                    node.destroy_node()
        finally:
            try:
                if initialized and rclpy.ok():
                    rclpy.shutdown()
            finally:
                for signum, previous in previous_handlers.items():
                    signal.signal(signum, previous)
    return code


if __name__ == '__main__':
    raise SystemExit(main())
