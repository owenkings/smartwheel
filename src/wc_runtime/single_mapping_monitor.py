"""Read-only single-lidar experiment monitor. No hardware startup or TF publisher."""
import argparse
from collections import deque, OrderedDict
import copy
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
import os
from pathlib import Path
import re
import signal
import threading
import time
from urllib.parse import urlsplit
import zipfile

import numpy as np

from wc_sensors.pointcloud import decode_pointcloud2


MAX_DISPLAY_POINTS = 50000
MAX_RAW_POINTS = 2000000
MAX_CLOUD_BYTES = 128000000
MAX_TRAJECTORY_POINTS = 2000
ASSETS = Path(__file__).parent / 'single_mapping_assets'


def _json_bytes(value):
    return json.dumps(value, ensure_ascii=False, allow_nan=False, separators=(',', ':')).encode('utf-8')


def _no_links(path):
    if '..' in path.parts or any(p.is_symlink() or (hasattr(p, 'is_junction') and p.is_junction())
                               for p in (path, *path.parents)):
        raise ValueError('Monitor paths must not traverse links, junctions or parent components')


def _stamp(message):
    value = message.header.stamp
    seconds, nanos = int(value.sec), int(value.nanosec)
    if seconds < 0 or not 0 <= nanos < 1000000000:
        raise ValueError('Invalid ROS source timestamp')
    return seconds * 1000000000 + nanos


def _vector(value, size):
    if len(value) != size or any(isinstance(x, bool) or not isinstance(x, (int, float, np.number)) or
                               not math.isfinite(x) for x in value):
        raise ValueError('Expected a finite numeric vector')
    return [float(x) for x in value]


class MappingState:
    """Thread-safe bounded snapshots and owned recording files; ROS-independent."""
    def __init__(self, session_id, side, output_root, *, clock=time.monotonic, wall_clock=time.time_ns):
        if not isinstance(session_id, str) or re.fullmatch(r'[A-Za-z0-9_][A-Za-z0-9_.-]{0,95}', session_id) is None:
            raise ValueError('Explicit safe session id required')
        if side not in ('left', 'right'):
            raise ValueError('Explicit left or right side required')
        output = Path(output_root).absolute()
        _no_links(output)
        output.mkdir(parents=True, exist_ok=False)
        self.output = output
        self.session_id, self.side = session_id, side
        self.clock, self.wall_clock = clock, wall_clock
        self.lock = threading.RLock()
        self.started_mono, self.started_ns = clock(), wall_clock()
        self.last_written_mono = -math.inf
        self.last = {name: None for name in ('odom', 'odom_info', 'cloud_map')}
        self.counts = {name: 0 for name in self.last}
        self.frames = {name: None for name in self.last}
        self.source_stamps = {name: None for name in self.last}
        self.received_ns = {name: None for name in self.last}
        self.failure = None
        self.stopped = False
        self.tracking_lost = None
        self.lost_count = 0
        self.total_lost = 0
        self.segment = 0
        self.trajectory = deque(maxlen=MAX_TRAJECTORY_POINTS)
        self.pending_poses = OrderedDict()
        self.tracking_by_stamp = OrderedDict()
        self.latest_pose = None
        self.info = {}
        self.map_revision = 0
        self.map_snapshot = {'available': False, 'revision': 0, 'frame_id': None, 'units': 'm',
                             'raw_point_count': None, 'finite_point_count': None,
                             'display_point_count': 0, 'points': [], 'bounds': None}
        self.events = (output / 'events.jsonl').open('x', encoding='utf-8', buffering=1)
        self.odometry = (output / 'odometry.jsonl').open('x', encoding='utf-8', buffering=1)
        self.trajectory_log = (output / 'trajectory.jsonl').open('x', encoding='utf-8', buffering=1)
        self.status_log = (output / 'status.jsonl').open('x', encoding='utf-8', buffering=1)
        self.publish_status(force=True)

    def _event(self, kind, **values):
        self.events.write(_json_bytes({'event': kind, 'host_receive_ns': self.wall_clock(), **values}).decode() + '\n')

    def _receive(self, stream, stamp_ns, frame_id):
        if type(stamp_ns) is not int or stamp_ns < 0:
            raise ValueError('Source timestamps must be nonnegative integer nanoseconds')
        previous = self.source_stamps[stream]
        if previous is not None and stamp_ns < previous:
            raise ValueError(stream + ' source timestamp moved backward')
        self.counts[stream] += 1
        self.last[stream] = self.clock()
        self.received_ns[stream] = self.wall_clock()
        self.source_stamps[stream] = stamp_ns
        self.frames[stream] = frame_id

    def receive_odom(self, frame_id, child_frame_id, stamp_ns, position, orientation):
        with self.lock:
            if self.failure or self.stopped:
                return
            if frame_id != 'single_' + self.side + '_odom' or child_frame_id != 'lidar_' + self.side:
                raise ValueError('Odometry frame must be single_SIDE_odom -> lidar_SIDE')
            position, orientation = _vector(position, 3), _vector(orientation, 4)
            quaternion_norm = math.sqrt(sum(x*x for x in orientation))
            pose_valid = quaternion_norm > 1e-12
            # Native icp_odometry publishes an all-zero quaternion while lost.
            # Preserve that explicit null pose; OdomInfo owns the lost-count gate.
            if pose_valid and abs(quaternion_norm - 1.0) > .01:
                raise ValueError('Odometry quaternion is not unit length')
            self._receive('odom', stamp_ns, frame_id)
            tracking = self.tracking_by_stamp.get(stamp_ns)
            pose = {'source_stamp_ns': stamp_ns, 'host_receive_ns': self.received_ns['odom'],
                    'frame_id': frame_id, 'child_frame_id': child_frame_id,
                    'position': position, 'orientation_xyzw': orientation,
                    'pose_valid': pose_valid,
                    'segment': tracking[1] if tracking is not None else None,
                    'tracking_lost': tracking[0] if tracking is not None else None,
                    'tracking_confirmation': 'same_source_stamp' if tracking is not None else 'awaiting_matching_odom_info'}
            self.latest_pose = pose
            self.odometry.write(_json_bytes(pose).decode() + '\n')
            self.pending_poses[stamp_ns] = pose
            while len(self.pending_poses) > 128:
                self.pending_poses.popitem(last=False)
            if tracking is not None:
                self._confirm_pose(stamp_ns, *tracking)

    def _confirm_pose(self, stamp_ns, lost, segment):
        pose = self.pending_poses.pop(stamp_ns, None)
        if pose is None:
            return
        pose.update(tracking_lost=lost, segment=segment, tracking_confirmation='same_source_stamp')
        if not lost and pose['pose_valid'] and (not self.trajectory or stamp_ns > self.trajectory[-1]['source_stamp_ns']):
            point = {'xyz': pose['position'], 'segment': segment, 'source_stamp_ns': stamp_ns}
            self.trajectory.append(point)
            self.trajectory_log.write(_json_bytes(point).decode() + '\n')

    def receive_info(self, frame_id, stamp_ns, lost, details=None):
        with self.lock:
            if self.failure or self.stopped:
                return
            if type(lost) is not bool:
                raise ValueError('OdomInfo.lost must be explicit bool')
            if frame_id != 'single_' + self.side + '_odom':
                raise ValueError('OdomInfo frame must be single_SIDE_odom')
            self._receive('odom_info', stamp_ns, frame_id)
            self.info = copy.deepcopy(details or {})
            if lost:
                if self.tracking_lost is not True:
                    self.segment += 1
                self.lost_count += 1
                self.total_lost += 1
                self._event('ODOMETRY_LOST', consecutive=self.lost_count, segment=self.segment,
                            source_stamp_ns=stamp_ns)
            else:
                if self.tracking_lost is True:
                    self._event('ODOMETRY_RECOVERED_NEW_SEGMENT', segment=self.segment, source_stamp_ns=stamp_ns)
                self.lost_count = 0
            self.tracking_lost = lost
            self.tracking_by_stamp[stamp_ns] = (lost, self.segment)
            while len(self.tracking_by_stamp) > 128:
                self.tracking_by_stamp.popitem(last=False)
            self._confirm_pose(stamp_ns, lost, self.segment)
            if self.lost_count >= 5:
                self.fail('ODOMETRY_LOST_FIVE_CONSECUTIVE', 'OdomInfo reported five consecutive lost samples')

    def receive_map(self, frame_id, stamp_ns, xyz):
        if frame_id != 'single_' + self.side + '_map':
            raise ValueError('Cloud map must be in single_SIDE_map')
        points = np.asarray(xyz)
        if points.ndim != 2 or points.shape[1] != 3 or len(points) > MAX_RAW_POINTS or points.dtype.kind not in 'fiu':
            raise ValueError('Map must be a bounded numeric Nx3 array')
        points = np.array(points, dtype=np.float64, order='C', copy=True)
        finite_mask = np.isfinite(points).all(axis=1)
        finite_points = points[finite_mask]
        finite_count = len(finite_points)
        if finite_count:
            bounds = {'min': finite_points.min(axis=0).tolist(), 'max': finite_points.max(axis=0).tolist()}
            indices = np.linspace(0, finite_count - 1, min(finite_count, MAX_DISPLAY_POINTS), dtype=np.int64)
            display = finite_points[indices].tolist()
        else:
            bounds, display = None, []
        with self.lock:
            if self.failure or self.stopped:
                return
            # Check source identity/time before replacing the previous usable snapshot.
            if type(stamp_ns) is not int or stamp_ns < 0 or (self.source_stamps['cloud_map'] is not None and stamp_ns < self.source_stamps['cloud_map']):
                raise ValueError('Invalid or backward map source timestamp')
            target = self.output / 'latest_map.npz'
            temporary = self.output / '.latest_map.npz.partial'
            _no_links(target); _no_links(temporary)
            with temporary.open('wb') as stream:
                np.savez(stream, xyz=points, frame_id=np.asarray(frame_id), session_id=np.asarray(self.session_id),
                         side=np.asarray(self.side), source_stamp_ns=np.int64(stamp_ns),
                         host_receive_ns=np.int64(self.wall_clock()))
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temporary, target)
            self._receive('cloud_map', stamp_ns, frame_id)
            self.map_revision += 1
            self.map_snapshot = {'available': True, 'revision': self.map_revision,
                'frame_id': frame_id, 'units': 'm', 'raw_point_count': len(points),
                'finite_point_count': finite_count, 'excluded_nonfinite_display_count': len(points)-finite_count,
                'display_point_count': len(display), 'points': display, 'bounds': bounds,
                'source_stamp_ns': stamp_ns, 'host_receive_ns': self.received_ns['cloud_map'],
                'display_sampling': 'evenly_spaced_indices_of_finite_original_rows',
                'raw_snapshot_file': 'latest_map.npz', 'raw_rows_preserved': True}

    def fail(self, code, detail):
        with self.lock:
            if self.failure is None:
                self.failure = {'code': code, 'detail': str(detail)[:2000], 'host_receive_ns': self.wall_clock()}
                self._event('FAILED', **self.failure)
                self.publish_status(force=True)

    def tick(self):
        with self.lock:
            if self.failure or self.stopped:
                return False
            now = self.clock()
            for stream in ('odom', 'odom_info'):
                if self.last[stream] is not None and now - self.last[stream] > 3.0:
                    self.fail('STREAM_TIMEOUT', stream + ' has not arrived for more than 3 seconds')
                    return False
            if now - self.started_mono > 30.0:
                missing = [stream for stream, count in self.counts.items() if count == 0]
                if missing:
                    self.fail('INITIAL_STREAM_TIMEOUT', 'No initial messages from: ' + ', '.join(missing))
                    return False
            self.publish_status()
            return True

    def status_snapshot(self):
        with self.lock:
            now = self.clock()
            if self.failure:
                state = 'FAILED'
            elif self.stopped:
                state = 'STOPPED'
            elif not any(self.counts.values()):
                state = 'UNKNOWN'
            elif all(self.counts.values()) and self.tracking_lost is False:
                state = 'RUNNING'
            elif self.tracking_lost:
                state = 'TRACKING_LOST'
            else:
                state = 'INITIALIZING'
            map_meta = {key: value for key, value in self.map_snapshot.items() if key != 'points'}
            ages = {name: None if value is None else max(0., now-value) for name, value in self.last.items()}
            return copy.deepcopy({'schema_version': 1, 'session_id': self.session_id, 'status': state,
                'experiment': 'SINGLE_LIDAR_3D_MAPPING_EXPERIMENT', 'source_mode': 'real',
                'sensor_mode': 'single_' + self.side, 'side': self.side, 'units': 'm',
                'calibration_validated': False, 'ground_reference_validated': False,
                'navigation_validated': False, 'started_ns': self.started_ns,
                'updated_ns': self.wall_clock(), 'elapsed_s': max(0., now-self.started_mono),
                'counts': self.counts, 'last_host_receive_ns': self.received_ns,
                'last_source_stamp_ns': self.source_stamps, 'age_s': ages, 'frames': self.frames,
                'tracking_lost': self.tracking_lost, 'consecutive_lost': self.lost_count,
                'total_lost': self.total_lost, 'trajectory_segment': self.segment,
                'odometry': self.latest_pose, 'odom_info': self.info,
                'trajectory': list(self.trajectory), 'trajectory_frame': 'single_' + self.side + '_odom',
                'trajectory_capacity': MAX_TRAJECTORY_POINTS, 'map': map_meta, 'failure': self.failure,
                'cloud_map_stale': None if ages['cloud_map'] is None else ages['cloud_map'] > 3,
                'topics': {name: '/wc_mapping/single_' + self.side + '/' + name for name in self.last},
                'limitations': ['3D geometry experiment; no validated horizontal ground reference.',
                    'Map and odometry trajectory have different frames; trajectory is not overlaid without a validated transform.',
                    'Cloud map may remain unchanged while stationary; its age alone does not stop the session.',
                    'Host receive times are observations, not measurement synchronization.'],
                'qos_requested': {'reliability': 'RELIABLE', 'durability': 'VOLATILE'}})

    def map_bytes(self):
        with self.lock:
            return _json_bytes(self.map_snapshot)

    def publish_status(self, force=False):
        with self.lock:
            now = self.clock()
            if not force and now-self.last_written_mono < 1.0:
                return
            value = self.status_snapshot()
            encoded = _json_bytes(value)
            temporary = self.output / '.status.json.partial'
            _no_links(temporary); _no_links(self.output / 'status.json')
            with temporary.open('wb') as stream:
                stream.write(encoded + b'\n'); stream.flush(); os.fsync(stream.fileno())
            os.replace(temporary, self.output / 'status.json')
            # Keep log lines compact; complete odometry is in its dedicated JSONL.
            log_value = {key: val for key, val in value.items() if key not in ('trajectory', 'limitations')}
            self.status_log.write(_json_bytes(log_value).decode() + '\n')
            self.last_written_mono = now

    def close(self, reason='requested_stop'):
        with self.lock:
            if self.events.closed:
                return
            self.stopped = True
            self._event('MONITOR_STOPPED', reason=reason)
            self.publish_status(force=True)
            for stream in (self.events, self.odometry, self.trajectory_log, self.status_log):
                stream.flush(); stream.close()


class OfflineState:
    """Load only saved monitor files, with no ROS import, subscription or writes."""
    def __init__(self, output_root):
        root = Path(output_root).absolute()
        _no_links(root)
        status_path, map_path = root/'status.json', root/'latest_map.npz'
        _no_links(status_path); _no_links(map_path)
        if not root.is_dir() or not status_path.is_file() or status_path.stat().st_size > 5000000:
            raise ValueError('Existing bounded monitor status.json required')
        status = json.loads(status_path.read_text(encoding='utf-8'))
        _json_bytes(status)
        if status.get('schema_version') != 1 or status.get('source_mode') != 'real' or \
                status.get('side') not in ('left', 'right') or status.get('sensor_mode') != 'single_'+status['side'] or \
                re.fullmatch(r'[A-Za-z0-9_][A-Za-z0-9_.-]{0,95}', str(status.get('session_id', ''))) is None or \
                status.get('experiment') != 'SINGLE_LIDAR_3D_MAPPING_EXPERIMENT' or \
                status.get('calibration_validated') is not False or status.get('navigation_validated') is not False:
            raise ValueError('Saved status is not an identified single-lidar experiment')
        self.status = copy.deepcopy(status)
        self.status.update(offline=True, display_mode='OFFLINE_SNAPSHOT',
                           offline_note='Saved files only; counters and ages are from the saved snapshot, not current sensor health')
        payload = {'available': False, 'revision': 0, 'frame_id': None, 'units': 'm',
                   'raw_point_count': None, 'finite_point_count': None, 'display_point_count': 0, 'points': [], 'bounds': None}
        if map_path.exists():
            if not map_path.is_file() or map_path.stat().st_size > MAX_CLOUD_BYTES:
                raise ValueError('Saved map exceeds the monitor byte budget')
            with zipfile.ZipFile(map_path) as archive:
                if sum(entry.file_size for entry in archive.infolist()) > MAX_CLOUD_BYTES:
                    raise ValueError('Expanded saved map exceeds the monitor byte budget')
            with np.load(map_path, allow_pickle=False) as saved:
                xyz = saved['xyz']
                frame = str(saved['frame_id'].item())
                stamp = int(saved['source_stamp_ns'].item())
                if str(saved['session_id'].item()) != status['session_id'] or str(saved['side'].item()) != status['side']:
                    raise ValueError('Saved map does not belong to the recorded session and sensor side')
                if frame != 'single_'+status['side']+'_map' or xyz.ndim != 2 or xyz.shape[1] != 3 or \
                        len(xyz) > MAX_RAW_POINTS or xyz.dtype.kind not in 'fiu':
                    raise ValueError('Saved map frame or XYZ layout mismatch')
                finite = xyz[np.isfinite(xyz).all(axis=1)]
                indices = np.linspace(0, len(finite)-1, min(len(finite), MAX_DISPLAY_POINTS), dtype=np.int64)
                payload.update(available=True, revision=status['map']['revision'], frame_id=frame,
                    source_stamp_ns=stamp, raw_point_count=len(xyz), finite_point_count=len(finite),
                    display_point_count=len(indices), points=finite[indices].tolist(),
                    bounds=None if not len(finite) else {'min': finite.min(axis=0).tolist(), 'max': finite.max(axis=0).tolist()})
        elif status.get('map', {}).get('available'):
            raise ValueError('Saved status references a missing map file')
        self.payload = _json_bytes(payload)

    def status_snapshot(self):
        return copy.deepcopy(self.status)

    def map_bytes(self):
        return self.payload


class MonitorServer(ThreadingHTTPServer):
    allow_reuse_address = False
    daemon_threads = True
    block_on_close = False

    def __init__(self, state, port):
        if type(port) is not int or not 0 <= port <= 65535:
            raise ValueError('Invalid monitor port')
        self.state = state
        self.slots = threading.BoundedSemaphore(8)
        super().__init__(('127.0.0.1', port), MonitorHandler)

    def get_request(self):
        connection, address = super().get_request()
        connection.settimeout(2.0)
        return connection, address

    def process_request(self, request, client_address):
        if not self.slots.acquire(blocking=False):
            self.shutdown_request(request)
            return
        try:
            super().process_request(request, client_address)
        except BaseException:
            self.slots.release()
            raise

    def process_request_thread(self, request, client_address):
        try:
            super().process_request_thread(request, client_address)
        finally:
            self.slots.release()


class MonitorHandler(BaseHTTPRequestHandler):
    protocol_version = 'HTTP/1.0'

    def log_message(self, *_):
        pass

    def _send(self, status, body, content_type='application/json; charset=utf-8'):
        self.send_response(status)
        self.send_header('Content-Type', content_type)
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')
        self.send_header('X-Content-Type-Options', 'nosniff')
        self.send_header('Content-Security-Policy', "default-src 'self'; script-src 'self'; style-src 'self'; connect-src 'self'; img-src 'self' data:; frame-ancestors 'none'")
        self.end_headers()
        self.wfile.write(body)

    def _host_ok(self):
        hosts = self.headers.get_all('Host') or []
        if hosts != ['127.0.0.1:' + str(self.server.server_port)]:
            self._send(403, b'{"error":"loopback Host required"}')
            return False
        return True

    def do_GET(self):
        if not self._host_ok():
            return
        if len(self.path) > 2048:
            self._send(414, b'{"error":"path too long"}')
            return
        parsed = urlsplit(self.path)
        if parsed.scheme or parsed.netloc or parsed.query or parsed.fragment:
            self._send(400, b'{"error":"exact local endpoint required"}')
            return
        if parsed.path == '/api/status':
            self._send(200, _json_bytes(self.server.state.status_snapshot()))
        elif parsed.path == '/api/map':
            self._send(200, self.server.state.map_bytes())
        elif parsed.path in ('/', '/index.html', '/viewer.js', '/viewer.css'):
            name = 'index.html' if parsed.path == '/' else parsed.path[1:]
            kind = {'.html': 'text/html; charset=utf-8', '.js': 'text/javascript; charset=utf-8', '.css': 'text/css; charset=utf-8'}[Path(name).suffix]
            self._send(200, (ASSETS/name).read_bytes(), kind)
        else:
            self._send(404, b'{"error":"not found"}')

    def do_POST(self):
        if self._host_ok():
            self._send(405, b'{"error":"read-only monitor"}')


def run(arguments):
    state = MappingState(arguments.session_id, arguments.side, arguments.output_root)
    stop = threading.Event()
    reason = ['requested_stop']
    server = server_thread = node = rclpy_module = None
    previous_handlers = {}
    for signum in (signal.SIGINT, signal.SIGTERM):
        previous_handlers[signum] = signal.getsignal(signum)
        def request_stop(value, _frame):
            reason[0] = 'signal_' + str(value); stop.set()
        signal.signal(signum, request_stop)
    try:
        import rclpy
        from nav_msgs.msg import Odometry
        from rtabmap_msgs.msg import OdomInfo
        from sensor_msgs.msg import PointCloud2
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        from rclpy.signals import SignalHandlerOptions
        rclpy_module = rclpy
        rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
        node = rclpy.create_node('single_' + arguments.side + '_mapping_monitor')
        def guarded(callback):
            def receive(message):
                try:
                    callback(message)
                except Exception as error:
                    state.fail('INVALID_SOURCE_MESSAGE', type(error).__name__ + ': ' + str(error))
            return receive
        def odom(message):
            p, q = message.pose.pose.position, message.pose.pose.orientation
            state.receive_odom(message.header.frame_id, message.child_frame_id, _stamp(message),
                               [p.x, p.y, p.z], [q.x, q.y, q.z, q.w])
        def info(message):
            details = {}
            for field in ('matches', 'inliers', 'features', 'icp_inliers_ratio', 'time_estimation', 'time_particle_filtering'):
                value = getattr(message, field, None)
                details[field] = value if isinstance(value, (int, float)) and math.isfinite(value) else None
            state.receive_info(message.header.frame_id, _stamp(message), message.lost, details)
        def cloud(message):
            if int(message.width)*int(message.height) > MAX_RAW_POINTS or len(message.data) > MAX_CLOUD_BYTES:
                raise ValueError('Native map exceeds monitor raw point/byte budget')
            state.receive_map(message.header.frame_id, _stamp(message), decode_pointcloud2(message))
        qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)
        map_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)
        prefix = '/wc_mapping/single_' + arguments.side + '/'
        subscriptions = [node.create_subscription(Odometry, prefix+'odom', guarded(odom), qos),
                         node.create_subscription(OdomInfo, prefix+'odom_info', guarded(info), qos),
                         node.create_subscription(PointCloud2, prefix+'cloud_map', guarded(cloud), map_qos)]
        server = MonitorServer(state, arguments.port)
        server_thread = threading.Thread(target=server.serve_forever, kwargs={'poll_interval': .1}, daemon=True)
        server_thread.start()
        print(json.dumps({'status': 'MONITOR_LISTENING', 'url': 'http://127.0.0.1:'+str(server.server_port)+'/',
                          'session_id': arguments.session_id}), flush=True)
        while not stop.is_set() and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=.1)
            if not state.tick():
                break
        if not stop.is_set() and not rclpy.ok() and state.failure is None:
            state.fail('ROS_CONTEXT_ENDED', 'ROS context stopped without a monitor stop request')
    except Exception as error:
        state.fail('MONITOR_RUNTIME_ERROR', type(error).__name__ + ': ' + str(error))
    finally:
        if server is not None:
            if server_thread is not None and server_thread.is_alive():
                server.shutdown()
            server.server_close()
        if server_thread is not None:
            server_thread.join(timeout=2)
        if node is not None:
            node.destroy_node()
        if rclpy_module is not None and rclpy_module.ok():
            rclpy_module.shutdown()
        state.close(reason[0])
        for signum, previous in previous_handlers.items():
            signal.signal(signum, previous)
    return 2 if state.failure else 0


def serve_offline(output_root, port):
    state = OfflineState(output_root)
    server = MonitorServer(state, port)
    stop = threading.Event()
    previous_handlers = {}
    for signum in (signal.SIGINT, signal.SIGTERM):
        previous_handlers[signum] = signal.getsignal(signum)
        signal.signal(signum, lambda _signum, _frame: stop.set())
    thread = threading.Thread(target=server.serve_forever, kwargs={'poll_interval': .1}, daemon=True)
    thread.start()
    try:
        print(json.dumps({'status': 'OFFLINE_MONITOR_LISTENING',
                          'url': 'http://127.0.0.1:'+str(server.server_port)+'/',
                          'session_id': state.status['session_id']}), flush=True)
        while not stop.wait(.2):
            pass
    finally:
        server.shutdown(); server.server_close(); thread.join(timeout=2)
        for signum, previous in previous_handlers.items():
            signal.signal(signum, previous)
    return 0


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session-id')
    parser.add_argument('--side', choices=('left', 'right'))
    parser.add_argument('--output-root', type=Path)
    parser.add_argument('--offline', type=Path)
    parser.add_argument('--port', type=int, default=8770)
    arguments = parser.parse_args(argv)
    if not 1 <= arguments.port <= 65535:
        parser.error('--port must be in 1..65535')
    if arguments.offline is not None:
        if any(value is not None for value in (arguments.session_id, arguments.side, arguments.output_root)):
            parser.error('--offline cannot be combined with live session arguments')
    elif any(value is None for value in (arguments.session_id, arguments.side, arguments.output_root)):
        parser.error('live mode requires --session-id, --side and --output-root')
    try:
        return serve_offline(arguments.offline, arguments.port) if arguments.offline else run(arguments)
    except Exception as error:
        print(json.dumps({'status': 'MONITOR_START_FAILED', 'error': type(error).__name__, 'detail': str(error)}), flush=True)
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
