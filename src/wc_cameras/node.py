"""Publish one real USB slot; optional source BGR archive precedes preview throttle.

Run one instance per slot under the common runtime supervisor. Direction-to-port
assignment and its confirmation source come from configuration. Image timestamps are
host read completion (arrival_only), never exposure time or synchronized time.
"""
import argparse
import json
import os
from pathlib import Path
import re
import signal
import sys
import time

from .capture import CaptureProcess
from .config import CameraError, ROLES, load_config, number, topic


def camera_stream_epoch(session_id,stream_epoch):
    """Validate the worker's actual identity; never synthesize a replacement."""
    if (not isinstance(session_id,str) or not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_.-]{0,119}',session_id)
            or not isinstance(stream_epoch,str) or not stream_epoch.startswith(session_id+'-')
            or not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_.-]{0,119}',stream_epoch[len(session_id)+1:])):
        raise CameraError('invalid actual camera stream epoch for readiness')
    return stream_epoch


def camera_readiness_record(session_id, role, worker_pid, frame, first_monotonic_ns, identity,
                            *, stream_epoch, error=None):
    """Native capture observation, independent of preview publication throttle."""
    stream_epoch=camera_stream_epoch(session_id,stream_epoch)
    if (role not in ROLES or type(worker_pid) is not int or worker_pid<=0 or not isinstance(frame,dict)
            or not isinstance(identity,dict) or type(first_monotonic_ns) is not int or first_monotonic_ns<=0
            or type(frame.get('host_monotonic_ns')) is not int or frame['host_monotonic_ns']<first_monotonic_ns
            or type(frame.get('capture_sequence')) is not int or frame['capture_sequence']<1):
        raise CameraError('invalid native camera readiness observation')
    return dict(schema_version=1,session_id=session_id,logical_source='camera_'+role,source_id=role,
        stream_epoch=stream_epoch,worker_pid=worker_pid,ready=not error,error=error,
        first_valid_monotonic_ns=first_monotonic_ns,last_valid_monotonic_ns=frame['host_monotonic_ns'],
        valid_count=frame['capture_sequence'],expected_transport_identity=identity,
        count_basis='actual native capture sequence; preview may skip frames',
        persistence_verified=False,physical_synchronization_verified=False)


class FrameTracker:
    """Hardware-free freshness and publication accounting."""
    def __init__(self, started_ns, timeouts):
        self.started_ns, self.timeouts = started_ns, timeouts
        self.ready_ns = None
        self.latest = None
        self.published_count = 0
        self.last_published_capture = 0
        self.skipped_total = 0

    def accept(self, frame):
        if self.latest is not None and frame['capture_sequence'] <= self.latest['capture_sequence']:
            raise CameraError('capture sequence did not advance')
        if not frame['host_monotonic_ns'] or not frame['host_arrival_ns']:
            raise CameraError('capture arrival timestamps are missing')
        self.latest = frame

    def health(self, now_ns):
        if self.ready_ns is None:
            return ('ERROR', 'camera open/identity deadline exceeded') if now_ns-self.started_ns > self.timeouts['open_s']*1e9 else ('STARTING', 'opening camera')
        reference = self.latest['host_monotonic_ns'] if self.latest else self.ready_ns
        age_ns = now_ns-reference
        if age_ns < 0:
            return 'ERROR', 'host monotonic timestamp moved backwards'
        if age_ns > self.timeouts['read_s']*1e9:
            return 'ERROR', 'camera read deadline exceeded; capture child will be stopped'
        if age_ns > self.timeouts['stale_s']*1e9:
            return 'STALE', 'no fresh decoded frame'
        return ('ACTIVE', 'decoded frame available') if self.latest else ('STARTING', 'waiting for first decoded frame')

    def published(self, frame, now_ns):
        sequence = frame['capture_sequence']
        if sequence <= self.last_published_capture:
            raise CameraError('cannot republish an old frame with a new timestamp')
        elapsed = now_ns-frame['host_monotonic_ns']
        if elapsed < 0:
            raise CameraError('negative arrival-to-publish elapsed time')
        skipped = sequence-self.last_published_capture-1
        self.skipped_total += skipped
        self.last_published_capture = sequence
        self.published_count += 1
        return {'capture_sequence': sequence, 'published_sequence': self.published_count,
                'host_arrival_ns': frame['host_arrival_ns'], 'host_monotonic_ns': frame['host_monotonic_ns'],
                'host_read_elapsed_ns': frame['host_read_elapsed_ns'],
                'host_arrival_to_publish_elapsed_ns': elapsed,
                'skipped_capture_frames_since_previous_image': skipped,
                'skipped_capture_frames_total': self.skipped_total,
                'skipped_capture_frames_meaning': 'latest-frame publication throttle/queue replacement, not measured device losses',
                'device_dropped_frames': 'UNKNOWN', 'measurement_latency_ns': 'UNKNOWN'}


def fill_image(message, frame, frame_id):
    seconds, nanoseconds = divmod(frame['host_arrival_ns'], 1_000_000_000)
    if not 0 <= seconds < 2**31:
        raise CameraError('host arrival timestamp is outside ROS Time range')
    if len(frame['data']) != frame['width']*frame['height']*3:
        raise CameraError('BGR8 payload/shape mismatch')
    message.header.stamp.sec, message.header.stamp.nanosec = seconds, nanoseconds
    message.header.frame_id = frame_id
    message.height, message.width = frame['height'], frame['width']
    message.encoding, message.is_bigendian = 'bgr8', False
    message.step = frame['width']*3
    message.data = frame['data']
    return message


def check_capture_events(events, alive, exit_code):
    """Check even when a bounded session is ending; never hide a final error."""
    errors = [str(event.get('error', 'unknown capture error')) for event in events if event.get('event') == 'ERROR']
    if errors:
        raise CameraError('; '.join(errors))
    if not alive:
        raise CameraError('capture child exited unexpectedly, code='+str(exit_code))


def finish_capture(node):
    """Latch cleanup failure before main computes its return code."""
    try:
        result = node.capture.close()
    except Exception as error:
        result = node.capture.cleanup_result or {'status': 'FAIL', 'errors': [str(error)]}
        result = {**result, 'status': 'FAIL', 'close_error': str(error)}
    if result['status'] != 'PASS':
        node.failed = True
        detail = 'capture cleanup failed: '+json.dumps(result, allow_nan=False, separators=(',', ':'))
        node.latest_error = (node.latest_error+'; '+detail).lstrip('; ')
    return result


def exit_if_capture_unreaped(node, *, immediate_exit=os._exit):
    """After logging and ROS cleanup, fail without multiprocessing's final join.

    CPython's multiprocessing exit hook joins every active child without a
    timeout, including a daemon it has just terminated. A child still alive
    after our TERM/KILL deadlines would therefore hang this CLI on SystemExit.
    Do not discard its process record, unlink its lock or pretend it is gone.
    The supervisor receives exit 1 and the logged residual PID/state; the child
    retains its device/lock until the kernel actually releases those resources.
    """
    if node is not None:
        result = node.capture.cleanup_result
        if result is not None and result.get('alive_after_cleanup') is True:
            immediate_exit(1)


def set_diagnostic_level(message, state):
    # Humble's generated byte field requires its byte-valued constants, not int.
    from diagnostic_msgs.msg import DiagnosticStatus
    message.level = DiagnosticStatus.ERROR if state == 'ERROR' else DiagnosticStatus.WARN


def diagnostic_level_number(value):
    """Convert a ROS byte field into an explicit JSON integer without guessing."""
    if isinstance(value, (bytes, bytearray)) and len(value) == 1:
        return value[0]
    if type(value) is int and 0 <= value <= 255:
        return value
    raise CameraError('DiagnosticStatus.level must be one byte or a bounded integer')


def parser():
    result = argparse.ArgumentParser(description=__doc__)
    result.add_argument('--config', type=Path, required=True)
    result.add_argument('--role', choices=ROLES, required=True, help='preview direction; USB assignment and confirmation are read from config')
    result.add_argument('--profile', help='explicit profile override; no automatic quality fallback')
    result.add_argument('--session-id', required=True)
    result.add_argument('--archive-dir', type=Path, help='Lossless decoded BGR8 archive before preview rotation')
    result.add_argument('--run-root', type=Path, required=True)
    result.add_argument('--duration', type=float, default=60, help='0 means until stopped; otherwise 0.5..43200 seconds')
    result.add_argument('--check-config', action='store_true', help='validate and print the selected config; no ROS or device access')
    return result


def camera_description(config, config_hash, camera, profile_name):
    """Shared startup/diagnostic identity; user direction confirmation is not calibration."""
    role = camera['role']
    return {'logical_slot': role, 'slot_label': 'USB3.'+str(camera['port']),
            'topic': topic(role), 'frame_id': 'camera_usb3_'+str(camera['port']),
            'mapping_status': config['mapping_status'], 'mapping_note': config['mapping_note'],
            'physical_direction': role if config['mapping_status'] == 'USER_CONFIRMED' else 'UNKNOWN',
            'rotation_status': config['rotation_status'], 'rotation_degrees': camera['rotate_deg'],
            'profile': profile_name, 'requested': config['profiles'][profile_name], 'config_sha256': config_hash,
            'time_source': 'arrival_only', 'common_time_valid': False,
            'image_storage': 'DISABLED', 'camera_info': 'UNAVAILABLE', 'tf_published': False}


def main(argv=None):
    args = parser().parse_args(argv)
    number(args.duration, 'duration', 0, 43200)
    if 0 < args.duration < .5:
        raise CameraError('duration must be 0 (until stopped) or in [0.5, 43200] seconds')
    if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_.-]{0,119}', args.session_id):
        raise CameraError('session-id must be a bounded simple identifier')
    if not args.run_root.is_absolute():
        raise CameraError('run-root must be an explicit shared absolute path')
    config, config_hash = load_config(args.config)
    profile_name = args.profile or config['default_profile']
    if profile_name not in config['profiles']:
        raise CameraError('unknown profile: '+profile_name)
    profile = config['profiles'][profile_name]
    camera = next(row for row in config['cameras'] if row['role'] == args.role)
    description = camera_description(config, config_hash, camera, profile_name)
    if args.archive_dir is not None:
        description['image_storage'] = 'DECODED_BGR8_LOSSLESS_SOURCE_ARCHIVE'
        description['archive_path'] = str(args.archive_dir)
    if args.check_config:
        print(json.dumps(description, allow_nan=False, sort_keys=True))
        return 0
    if sys.platform != 'linux':
        raise CameraError('real camera node requires Linux V4L2; use --check-config for local static validation')

    # Import ROS only after CLI/config validation; offline tests need no ROS/CV2.
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from sensor_msgs.msg import Image
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

    class CameraNode(Node):
        def __init__(self):
            super().__init__('camera_'+args.role, namespace='/wc_mapping/cameras')
            self.capture = CaptureProcess(camera, profile, args.run_root, args.session_id,
                                          **({'archive_root': args.archive_dir} if args.archive_dir else {}))
            self.started_ns = time.monotonic_ns()
            self.tracker = FrameTracker(self.started_ns, config['timeouts'])
            self.actual, self.identity, self.version = None, None, None
            self.epoch = None  # Only the successful worker READY event can establish a source epoch.
            self.done, self.failed, self.stop_requested = False, False, False
            self.next_publish_ns = self.started_ns
            self.last_diag_ns = 0
            self.last_accounting = {}
            self.capture_child_errors = 0
            self.latest_error = ''
            self.first_capture_monotonic_ns = None
            self.last_ready_write_ns = 0
            qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
            self.image = self.create_publisher(Image, topic(args.role), qos)
            self.diag = self.create_publisher(DiagnosticArray, '/wc_mapping/cameras/'+args.role+'/diagnostics', 10)
            self.common_diag = self.create_publisher(DiagnosticArray, '/wc_mapping/diagnostics', 10)
            self.timer = self.create_timer(.02, self.tick)

        def diagnostic(self, state, reason, now_ns):
            message = DiagnosticArray()
            message.header.stamp = self.get_clock().now().to_msg()
            row = DiagnosticStatus()
            row.name = '/wc_mapping/cameras/'+args.role
            row.hardware_id = 'usb3_'+str(camera['port'])+'/0bda:5858/'+camera['serial']
            set_diagnostic_level(row, state)  # Arrival time and missing calibration remain explicit after direction confirmation.
            row.message = state+': '+reason
            details = {**description, 'session_id': args.session_id, 'stream_epoch': self.epoch,
                       'state': state, 'actual': self.actual, 'device': self.identity,
                       'opencv_version': self.version, 'published_frames': self.tracker.published_count,
                       'capture_child_errors': self.capture_child_errors, 'failure_latched': self.failed,
                       'capture_sequence_latest_seen': self.tracker.latest['capture_sequence'] if self.tracker.latest else None,
                       'latest_frame_age_ns': now_ns-self.tracker.latest['host_monotonic_ns'] if self.tracker.latest else None,
                       'shape': [self.tracker.latest['height'], self.tracker.latest['width'], 3] if self.tracker.latest else None,
                       'host_elapsed_is_not_measurement_latency': True,
                       'device_timestamp': 'UNKNOWN', 'synchronized_with_other_cameras': False,
                       **self.last_accounting}
            for key, value in details.items():
                pair = KeyValue()
                pair.key = key
                pair.value = json.dumps(value, allow_nan=False, separators=(',', ':'))
                row.values.append(pair)
            message.status = [row]
            self.diag.publish(message)
            self.common_diag.publish(message)
            self.last_diag_ns = now_ns

        def fail(self, error, now_ns):
            self.failed, self.done, self.latest_error = True, True, str(error)
            try:
                self.write_readiness(now_ns,force=True)
            except Exception as marker_error:
                self.latest_error += '; readiness marker: '+str(marker_error)
            self.get_logger().error(self.latest_error)
            self.diagnostic('ERROR', self.latest_error, now_ns)

        def write_readiness(self, now_ns, force=False):
            if (args.archive_dir is None or self.tracker.latest is None or self.identity is None
                    or not force and now_ns-self.last_ready_write_ns<250_000_000):
                return
            from wc_runtime.source_archive import atomic_json
            ready_root = args.archive_dir.parents[2]/'ready'
            if ready_root.is_symlink():
                raise CameraError('camera readiness directory may not be linked')
            ready_root.mkdir(exist_ok=True)
            record = camera_readiness_record(args.session_id,args.role,self.capture.process.pid,
                self.tracker.latest,self.first_capture_monotonic_ns,self.identity,
                stream_epoch=self.epoch,error=self.latest_error or None)
            atomic_json(ready_root/('camera_'+args.role+'.json'),record)
            self.last_ready_write_ns=now_ns

        def tick(self):
            if self.done:
                return
            now_ns = time.monotonic_ns()
            try:
                events = self.capture.events()
                self.capture_child_errors += sum(event.get('event') == 'ERROR' for event in events)
                check_capture_events(events, self.capture.process.is_alive(), self.capture.process.exitcode)
                for event in events:
                    if event['event'] != 'READY' or self.tracker.ready_ns is not None:
                        raise CameraError('unexpected capture child event')
                    self.epoch = camera_stream_epoch(args.session_id,event.get('stream_epoch'))
                    self.tracker.ready_ns = now_ns
                    self.actual, self.identity, self.version = event['actual'], event['device'], event['opencv_version']
                    self.get_logger().info(json.dumps({'event': 'CAMERA_READY', **description,
                                                       'actual': self.actual, 'device': self.identity}, allow_nan=False))
                if self.stop_requested or (args.duration > 0 and now_ns-self.started_ns >= args.duration*1e9):
                    if not self.stop_requested and self.tracker.published_count == 0:
                        raise CameraError('session duration expired before any camera image was published')
                    self.done = True
                    self.diagnostic('STOPPING', 'bounded session complete or stop requested', now_ns)
                    return
                after = self.tracker.latest['capture_sequence'] if self.tracker.latest else 0
                frame = self.capture.shared.snapshot(after)
                if frame is not None:
                    self.tracker.accept(frame)
                    if self.first_capture_monotonic_ns is None:
                        self.first_capture_monotonic_ns = frame['host_monotonic_ns']
                now_ns = time.monotonic_ns()  # Snapshot can contain a frame newer than the start of this tick.
                self.write_readiness(now_ns)
                if not self.capture.process.is_alive():
                    raise CameraError('capture child exited unexpectedly, code='+str(self.capture.process.exitcode))
                health, reason = self.tracker.health(now_ns)
                if health == 'ERROR':
                    raise CameraError(reason)
                latest = self.tracker.latest
                if (health == 'ACTIVE' and latest is not None and latest['capture_sequence'] > self.tracker.last_published_capture
                        and now_ns >= self.next_publish_ns):
                    message = fill_image(Image(), latest, description['frame_id'])
                    accounting = self.tracker.published(latest, time.monotonic_ns())
                    self.image.publish(message)
                    period_ns = 1e9/profile['publish_hz']
                    self.next_publish_ns += (int((now_ns-self.next_publish_ns)/period_ns)+1)*period_ns
                    self.last_accounting = accounting
                    self.diagnostic(health, reason, now_ns)
                elif now_ns-self.last_diag_ns >= .5e9:
                    self.diagnostic(health, reason, now_ns)
            except Exception as error:
                self.fail(error, now_ns)

    node = None
    previous = {}
    rclpy.init(args=[])
    try:
        node = CameraNode()
        for signum in (signal.SIGINT, signal.SIGTERM):
            previous[signum] = signal.getsignal(signum)
            signal.signal(signum, lambda *_: setattr(node, 'stop_requested', True))
        node.capture.start()
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=.1)
    finally:
        try:
            if node is not None:
                cleanup = finish_capture(node)
                print(json.dumps({'event': 'CAMERA_STOPPED', 'logical_slot': args.role,
                                  'published_frames': node.tracker.published_count,
                                  'failure_latched': node.failed, 'error': node.latest_error,
                                  'cleanup': cleanup,
                                  'images_saved': False}), flush=True)
                node.destroy_node()
        finally:
            try:
                for signum, handler in previous.items():
                    signal.signal(signum, handler)
                if rclpy.ok():
                    rclpy.shutdown()
            finally:
                exit_if_capture_unreaped(node)
    return 1 if node is None or node.failed else 0


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (CameraError, OSError, ValueError) as error:
        print('camera: '+str(error), file=sys.stderr, flush=True)
        raise SystemExit(1)
