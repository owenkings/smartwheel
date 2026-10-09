"""Session-checked, BEST_EFFORT capture preview; subscriptions only.

Two RViz windows retain each lidar's native frame and show the two cameras
on that side. This process never starts a driver, opens a sensor, publishes TF,
commands motion, or gates raw recording. Closing it closes only its windows.
"""
import argparse
import copy
import hashlib
import json
import math
import re
import signal
import sys
import time
import uuid
from pathlib import Path


ROLES = ('left_front', 'right_front', 'left_side', 'right_side')
SIDES = ('left', 'right')
PREVIEW_STALE_NS = 1_000_000_000
RENDER_LOG_READ_BYTES = 64*1024
RENDER_ENVIRONMENT_KEYS = ('LIBGL_ALWAYS_SOFTWARE', 'GALLIUM_DRIVER', '__GLX_VENDOR_LIBRARY_NAME',
                           'MESA_LOADER_DRIVER_OVERRIDE', '__NV_PRIME_RENDER_OFFLOAD',
                           '__NV_PRIME_RENDER_OFFLOAD_PROVIDER', 'QT_OPENGL', 'QT_XCB_GL_INTEGRATION')


def rendering_environment(environment, renderer='software'):
    """Select Mesa only for the owned views, without changing source environments.

    The selected backend is an explicit launch request. A GL version or live
    subscription does not prove that backend selection worked or pixels appear.
    """
    if renderer not in ('software', 'system'):
        raise ValueError('preview renderer must be software or system')
    env = dict(environment)
    removed = []
    if renderer == 'software':
        # This is the same Mesa/llvmpipe selection used by the saved-map viewer.
        # Inherited PRIME/loader overrides must not defeat this explicit choice.
        for key in ('MESA_LOADER_DRIVER_OVERRIDE', '__NV_PRIME_RENDER_OFFLOAD',
                    '__NV_PRIME_RENDER_OFFLOAD_PROVIDER'):
            if key in env:
                removed.append(key); env.pop(key)
        env.update(LIBGL_ALWAYS_SOFTWARE='1', GALLIUM_DRIVER='llvmpipe', __GLX_VENDOR_LIBRARY_NAME='mesa')
    return env, dict(selected_backend=renderer, applied_to='owned preview RViz processes only',
        environment={key: env[key] for key in RENDER_ENVIRONMENT_KEYS if key in env},
        removed_inherited_overrides=removed, actual_backend_verified=False, visible_pixels_verified=False)


class RenderLogMonitor:
    """Bounded incremental observations; errors are sticky even while RViz lives."""
    ERROR_SIGNATURES = ('failed to create an opengl context', 'failed to create opengl context',
        'glxbaddrawable', 'libgl error: failed to create drawable', 'renderingapierror',
        'could not initialize opengl', "couldn't find any suitable glx", 'no matching fbconfig')

    def __init__(self, path):
        self.path = Path(path)
        self.offset, self.carry = 0, ''
        self.errors = []
        self.initialization_evidence = None
        self.actual_renderer_evidence = None
        self.observed_backend = 'UNKNOWN'
        self.unread_bytes = 0

    def _error(self, message):
        message = message[:2048]
        if message not in self.errors and len(self.errors) < 8:
            self.errors.append(message)

    def observe(self):
        try:
            if self.path.is_symlink():
                raise ValueError('render log was redirected by a symlink')
            if self.path.exists():
                size = self.path.stat().st_size
                if size < self.offset:
                    raise ValueError('render log shrank during observation')
                with self.path.open('rb') as stream:
                    stream.seek(self.offset)
                    data = stream.read(RENDER_LOG_READ_BYTES)
                self.offset += len(data)
                self.unread_bytes = max(0, size-self.offset)
                text = self.carry+data.decode('utf-8', errors='replace')
                for line in text.splitlines():
                    lower = line.lower()
                    if any(signature in lower for signature in self.ERROR_SIGNATURES):
                        self._error(line.strip())
                    if 'opengl version:' in lower or 'actual ogre renderer:' in lower or 'native rviz ready;' in lower:
                        self.initialization_evidence = line.strip()[:2048]
                    # Stock RViz commonly reports only the GL version. Do not
                    # infer NVIDIA or Mesa from a version number or requested env.
                    if any(marker in lower for marker in ('actual ogre renderer:', 'gl_renderer', 'opengl device:')):
                        self.actual_renderer_evidence = line.strip()[:2048]
                        self.observed_backend = ('MESA_SOFTWARE' if 'llvmpipe' in lower or 'softpipe' in lower
                                                 else ('NVIDIA' if 'nvidia' in lower else 'OTHER_REPORTED'))
                self.carry = text[-4096:]
        except (OSError, ValueError) as error:
            self._error('render log observation failed: '+str(error))
        return dict(status='FAILED' if self.errors else
                    ('INITIALIZATION_REPORTED' if self.initialization_evidence else 'WAITING_LOG'),
            log_path=str(self.path), bytes_observed=self.offset, unread_bytes=self.unread_bytes,
            initialization_evidence=self.initialization_evidence,
            observed_backend=self.observed_backend, actual_renderer_evidence=self.actual_renderer_evidence,
            actual_backend_verified=self.actual_renderer_evidence is not None,
            visible_pixels_verified=False, errors=list(self.errors))


def _json(path):
    path = Path(path)
    if path.is_symlink() or not path.is_file() or path.stat().st_size > 2*1024*1024:
        raise ValueError('missing, linked or oversized preview session metadata: '+str(path))
    return json.loads(path.read_text(encoding='utf-8'))


def preview_plan(session_root, session_id, *, preview_hz=3., cloud_source='raw'):
    """Pure metadata validation. Geometry calibration and controls are unused."""
    if not isinstance(session_id, str) or not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_.-]{0,119}', session_id):
        raise ValueError('bounded session identifier required')
    if type(preview_hz) not in (int, float) or not math.isfinite(preview_hz) or not .5 <= preview_hz <= 5:
        raise ValueError('preview_hz must be in [0.5, 5]')
    if cloud_source not in ('raw', 'filtered'):
        raise ValueError('preview cloud source must be raw or filtered')
    root = Path(session_root).absolute()
    if any(p.is_symlink() for p in (root, *root.parents)) or not root.is_dir():
        raise ValueError('preview requires an existing unlinked capture directory')
    manifest = _json(root/'capture_manifest.json')
    ready = _json(root/'recorder_ready.json')
    if manifest.get('session_id') != session_id or manifest.get('status') != 'RECORDING':
        raise ValueError('preview requires the matching active capture manifest')
    selection = _json(root/'configuration/source_selection.json')
    selected = selection.get('selected_sources')
    if not isinstance(selected, list) or not {'lidar_left', 'lidar_right'} & set(selected):
        raise ValueError('native preview requires at least one explicitly selected lidar source')
    started_wall = manifest.get('started_wall_ns')
    started_mono = manifest.get('started_monotonic_ns')
    if type(started_wall) is not int or type(started_mono) is not int or min(started_wall, started_mono) <= 0:
        raise ValueError('capture start timestamps required for stale-session rejection')
    # Existing recorder receipts omit session_id. A fresh ready timestamp in
    # this newly created capture directory binds that receipt to its manifest.
    ready_mono = ready.get('host_monotonic_ns')
    if ready.get('session_id', session_id) != session_id or ready.get('ready') is not True or \
            type(ready_mono) is not int or ready_mono < started_mono:
        raise ValueError('preview must start after the matching fresh raw recorder is ready')
    namespace = '/wc_capture_preview/s_'+hashlib.sha256(session_id.encode()).hexdigest()[:16]
    sources = {}
    suffix = '_filtered' if cloud_source == 'filtered' else ''
    for side in SIDES:
        if 'lidar_'+side not in selected:
            continue
        path = root/'configuration/lidar'/(side+'.yaml')
        if path.is_symlink() or not path.is_file() or path.stat().st_size > 65536:
            raise ValueError('missing frozen lidar identity YAML: '+side)
        values = {}
        for name in ('expected_serial', 'frame_id'):
            matches = re.findall(r'^\s*'+name+r':\s*([A-Za-z0-9_]+)\s*$', path.read_text(encoding='utf-8'), re.M)
            if len(matches) != 1:
                raise ValueError('ambiguous frozen lidar '+name)
            values[name] = matches[0]
        if values['frame_id'] != 'lidar_'+side:
            raise ValueError('native lidar frame differs from reviewed capture contract')
        sources[side] = dict(sensor_id=values['expected_serial'], frame_id=values['frame_id'],
            input_topic='/wc_mapping/lidar_'+side+'/source_frame'+suffix,
            output_topic=namespace+'/lidar_'+side+'/points')
    cameras = {}
    camera_roles = [role for role in ROLES if 'camera_'+role in selected]
    if camera_roles:
        config = _json(root/'configuration/cameras.json')
        for role in camera_roles:
            rows = [row for row in config.get('cameras', []) if row.get('role') == role]
            if len(rows) != 1 or type(rows[0].get('port')) is not int or not 1 <= rows[0]['port'] <= 4:
                raise ValueError('explicit frozen camera role/port required: '+role)
            cameras[role] = dict(frame_id='camera_usb3_'+str(rows[0]['port']),
                input_topic='/wc_mapping/cameras/'+role+'/image_raw',
                diagnostic_topic='/wc_mapping/cameras/'+role+'/diagnostics',
                output_topic=namespace+'/cameras/'+role+'/image_raw')
    return dict(session_id=session_id, session_root=str(root), namespace=namespace,
                preview_hz=float(preview_hz), cloud_source=cloud_source, lidars=sources, cameras=cameras,
                started_wall_ns=started_wall, started_monotonic_ns=started_mono,
                hardware_started=False, controls_enabled=False, tf_published=False,
                recording_affected_by_preview_exit=False)


def _stamp(value):
    return int(value.sec)*1_000_000_000+int(value.nanosec)


def empty_preview_cloud(message):
    """Clear only a derived display; preserve native frame/fields and source stamp.

    An empty organized row has zero width/data/row_step while retaining the
    original point layout. This is not a source frame or a new measurement.
    """
    if type(message.point_step) is not int or message.point_step <= 0:
        raise ValueError('preview clear requires the original valid point layout')
    result = type(message)()
    result.header = copy.deepcopy(message.header)
    result.fields = copy.deepcopy(message.fields)
    result.height, result.width = 1, 0
    result.is_bigendian = message.is_bigendian
    result.point_step, result.row_step = message.point_step, 0
    result.data, result.is_dense = b'', True
    return result


class PreviewCache:
    """One latest sample per stream, independent of recorder queues/history."""
    def __init__(self, plan):
        self.plan = plan
        self.latest, self.keys, self.camera_sessions, self.published = {}, {}, {}, {}
        self.last_received, self.cleared = {}, {}
        self.rejected = {}

    def reject(self, reason):
        self.rejected[reason] = self.rejected.get(reason, 0)+1
        return False

    def lidar(self, side, message, now_ns):
        expected = self.plan['lidars'][side]
        if message.session_id != self.plan['session_id'] or message.sensor_id != expected['sensor_id'] or message.side != side:
            return self.reject('LIDAR_SESSION_OR_IDENTITY_MISMATCH')
        if message.header.frame_id != expected['frame_id'] or message.cloud.header.frame_id != expected['frame_id']:
            return self.reject('LIDAR_NATIVE_FRAME_MISMATCH')
        stamp = _stamp(message.host_receive_time)
        if stamp < self.plan['started_wall_ns'] or message.host_monotonic_ns < self.plan['started_monotonic_ns']:
            return self.reject('LIDAR_BEFORE_CAPTURE_START')
        if _stamp(message.header.stamp) != stamp or _stamp(message.cloud.header.stamp) != stamp:
            return self.reject('LIDAR_STAMP_MISMATCH')
        key = (message.stream_epoch, int(message.frame_sequence))
        previous = self.keys.get(side)
        if previous and previous[0] == key[0] and key[1] <= previous[1]:
            return self.reject('LIDAR_SEQUENCE_NOT_NEW')
        if not key[0] or key[1] < 0:
            return self.reject('LIDAR_SEQUENCE_INVALID')
        self.keys[side] = key
        self.latest[side] = (message.cloud, now_ns, key)
        self.last_received[side] = now_ns
        self.cleared.pop(side, None)
        return True

    def camera_diagnostic(self, role, message):
        matches = []
        for row in message.status:
            if row.name != '/wc_mapping/cameras/'+role:
                continue
            try:
                values = {item.key: json.loads(item.value) for item in row.values}
            except (ValueError, TypeError):
                continue
            if values.get('session_id') == self.plan['session_id'] and \
                    isinstance(values.get('stream_epoch'), str) and \
                    values['stream_epoch'].startswith(self.plan['session_id']+'-') and \
                    values.get('state') == 'ACTIVE':
                matches.append(values['stream_epoch'])
        if len(matches) != 1:
            self.camera_sessions.pop(role, None)
            self.latest.pop(role, None)
            return self.reject('CAMERA_SESSION_NOT_ACTIVE')
        self.camera_sessions[role] = matches[0]
        return True

    def camera(self, role, message, now_ns):
        if role not in self.camera_sessions or message.header.frame_id != self.plan['cameras'][role]['frame_id']:
            return self.reject('CAMERA_SESSION_OR_FRAME_MISMATCH')
        stamp = _stamp(message.header.stamp)
        if stamp < self.plan['started_wall_ns']:
            return self.reject('CAMERA_BEFORE_CAPTURE_START')
        key = (self.camera_sessions[role], stamp)
        previous = self.keys.get(role)
        if previous and key[1] <= previous[1]:
            return self.reject('CAMERA_STAMP_NOT_NEW')
        self.keys[role] = key
        self.latest[role] = (message, now_ns, key)
        self.last_received[role] = now_ns
        return True

    def source_status(self, now_ns):
        """Accepted callback arrival freshness, independent of capture health."""
        result = {}
        streams = [('lidar_'+side, side, 'lidar') for side in self.plan['lidars']]
        streams += [('camera_'+role, role, 'camera') for role in self.plan['cameras']]
        for logical, name, kind in streams:
            received = self.last_received.get(name)
            age = now_ns-received if received is not None else None
            fresh = received is not None and 0 <= age <= PREVIEW_STALE_NS and name in self.latest
            result[logical] = dict(status='WAITING' if received is None else ('FRESH' if fresh else 'STALE'),
                last_received_monotonic_ns=received, age_ns=age,
                age_s=age/1_000_000_000 if age is not None else None,
                stale_after_ns=PREVIEW_STALE_NS, kind=kind,
                time_basis='accepted preview callback host monotonic arrival; not measurement-time synchronization')
        return result

    def pending(self, now_ns):
        for name, (message, received, key) in list(self.latest.items()):
            if 0 <= now_ns-received <= PREVIEW_STALE_NS and self.published.get(name) != key:
                self.published[name] = key
                yield name, message

    def stale_lidar_clears(self, now_ns):
        """Once per stale displayed sample; never republish old data as fresh."""
        for side in self.plan['lidars']:
            if side not in self.latest or side not in self.published:
                continue
            message, received, key = self.latest[side]
            if now_ns-received > PREVIEW_STALE_NS and self.cleared.get(side) != key:
                cleared = empty_preview_cloud(message)
                self.cleared[side] = key
                yield side, cleared


def rviz_configuration(plan, side):
    """Build only native point/image displays, with explicit BEST_EFFORT QoS."""
    if side not in SIDES:
        raise ValueError('explicit lidar side required')
    def topic(value):
        return {'Value': value, 'Depth': 1, 'History Policy': 'Keep Last',
                'Reliability Policy': 'Best Effort', 'Durability Policy': 'Volatile'}
    displays = [{'Class': 'rviz_default_plugins/PointCloud2', 'Name': side+' native '+plan['cloud_source']+' preview',
        'Enabled': True, 'Alpha': 1, 'Color Transformer': 'AxisColor', 'Axis': 'X',
        'Position Transformer': 'XYZ', 'Style': 'Points', 'Size (Pixels)': 3,
        'Decay Time': 0, 'Topic': topic(plan['lidars'][side]['output_topic'])}]
    for role in (side+'_front', side+'_side'):
        if role in plan['cameras']:
            displays.append({'Class': 'rviz_default_plugins/Image', 'Name': role+' camera preview',
                             'Enabled': True, 'Topic': topic(plan['cameras'][role]['output_topic'])})
    return {'Panels': [{'Class': 'rviz_common/Displays', 'Name': 'Displays'},
                       {'Class': 'rviz_common/Views', 'Name': 'Views'}],
        'Visualization Manager': {'Class': '', 'Enabled': True, 'Name': side+' independent native preview',
            'Global Options': {'Fixed Frame': plan['lidars'][side]['frame_id'],
                               'Background Color': '24; 29; 36', 'Frame Rate': 10},
            'Displays': displays,
            'Tools': [{'Class': 'rviz_default_plugins/Interact'}, {'Class': 'rviz_default_plugins/MoveCamera'}],
            'Views': {'Current': {'Class': 'rviz_default_plugins/Orbit', 'Distance': 5,
                'Focal Point': {'X': 2, 'Y': 0, 'Z': 0}, 'Pitch': .3, 'Yaw': 3.14}}},
        'Window Geometry': {'Width': 960, 'Height': 800, 'X': 0 if side == 'left' else 960, 'Y': 0}}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session-root', type=Path, required=True)
    parser.add_argument('--session-id', required=True)
    parser.add_argument('--preview-hz', type=float, default=3.)
    parser.add_argument('--cloud-source', choices=('raw', 'filtered'), default='raw')
    parser.add_argument('--layout', choices=('separate','unified'), default='separate')
    parser.add_argument('--renderer', choices=('software', 'system'), default='software',
                        help='Owned RViz only: Mesa software avoids the observed native GL context failure')
    args = parser.parse_args(argv)
    plan = preview_plan(args.session_root, args.session_id, preview_hz=args.preview_hz, cloud_source=args.cloud_source)
    plan['layout'] = args.layout
    plan['recording_affected_by_preview_exit'] = args.layout == 'unified'
    plan['controls_enabled'] = args.layout == 'unified' and _json(args.session_root/'capture_manifest.json').get('manual_drive') is True
    if sys.platform != 'linux':
        raise RuntimeError('live subscription preview requires the target Linux ROS desktop')
    from .sensor_viewer import OwnedRviz, desktop_environment, confirm_display, write_report
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from sensor_msgs.msg import PointCloud2, Image
    from wc_interfaces.msg import SourceFrame
    from diagnostic_msgs.msg import DiagnosticArray
    root = Path(plan['session_root'])
    epoch = uuid.uuid4().hex
    output = root/'preview'/epoch
    output.mkdir(parents=True)
    status_path = root/'capture_preview_status.json'
    status = dict(plan, preview_epoch=epoch, status='STARTING', windows=[], errors=[],
                  published_counts={}, stale_clear_counts={})
    stopped = [False]
    handlers = {sig: signal.signal(sig, lambda *_: stopped.__setitem__(0, True)) for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP)}
    windows, node, timer, ros_started = [], None, None, False
    cache = PreviewCache(plan)
    reported_states = {}
    render_monitors = {side: RenderLogMonitor(output/(side+'.log'))
                       for side in (('unified',) if args.layout == 'unified' else plan['lidars'])}
    reported_render_states = {}
    def update_source_status(now_ns):
        sources = cache.source_status(now_ns)
        changes = {name: value for name, value in sources.items()
                   if reported_states.get(name) != value['status']}
        status['source_status'] = sources
        if changes:
            print(json.dumps(dict(event='PREVIEW_SOURCE_STATE_CHANGED', session_id=args.session_id,
                sources=changes, recording_affected=False), ensure_ascii=False), flush=True)
            reported_states.update({name: value['status'] for name, value in changes.items()})
    def write_status():
        write_report(output/'status.json', status)
        write_report(status_path, status)
    def update_render_status(*, fail_on_error=True):
        observations = {side: monitor.observe() for side, monitor in render_monitors.items()}
        status.setdefault('rendering', {})['windows'] = observations
        status['rendering']['actual_backend_verified'] = all(
            row['actual_backend_verified'] for row in observations.values())
        changed = {side: row for side, row in observations.items()
                   if reported_render_states.get(side) != row['status']}
        if changed:
            print(json.dumps(dict(event='PREVIEW_RENDER_STATE_CHANGED', session_id=args.session_id,
                selected_backend=status['rendering'].get('selected_backend'), windows=changed,
                recording_affected=False), ensure_ascii=False), flush=True)
            reported_render_states.update({side: row['status'] for side, row in changed.items()})
        failures = [side for side, row in observations.items() if row['status'] == 'FAILED']
        if failures:
            message = 'preview rendering failed: '+', '.join(failures)+'; inspect per-window render evidence'
            if message not in status['errors']:
                status['errors'].append(message)
            status.update(status='FAILED', stop_reason='PREVIEW_RENDERING_FAILURE')
            if fail_on_error:
                write_status()
                raise RuntimeError(message)
    update_source_status(time.monotonic_ns())
    try:
        env, render_selection = rendering_environment(desktop_environment(), args.renderer)
        status['rendering'] = render_selection
        confirm_display(env)
        rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO); ros_started = True
        node = rclpy.create_node('wc_capture_preview_'+epoch)
        qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST, reliability=ReliabilityPolicy.BEST_EFFORT)
        publishers = {}
        for side, source in plan['lidars'].items():
            publishers[side] = node.create_publisher(PointCloud2, source['output_topic'], qos)
            node.create_subscription(SourceFrame, source['input_topic'],
                lambda message, side=side: cache.lidar(side, message, time.monotonic_ns()), qos)
        for role, source in plan['cameras'].items():
            publishers[role] = node.create_publisher(Image, source['output_topic'], qos)
            node.create_subscription(Image, source['input_topic'],
                lambda message, role=role: cache.camera(role, message, time.monotonic_ns()), qos)
            node.create_subscription(DiagnosticArray, source['diagnostic_topic'],
                lambda message, role=role: cache.camera_diagnostic(role, message), qos)
        def publish_latest():
            now_ns = time.monotonic_ns()
            for name, message in cache.pending(now_ns):
                publishers[name].publish(message)
                status['published_counts'][name] = status['published_counts'].get(name, 0)+1
            for name, message in cache.stale_lidar_clears(now_ns):
                publishers[name].publish(message)
                status['stale_clear_counts'][name] = status['stale_clear_counts'].get(name, 0)+1
            update_source_status(now_ns)
        timer = node.create_timer(1/plan['preview_hz'], publish_latest)
        for side in plan['lidars']:
            config = output/(side+'.rviz')
            # JSON is valid YAML; RViz reads it without adding a YAML dependency.
            write_report(config, rviz_configuration(plan, side))
            if args.layout == 'separate':
                command = ['rviz2', '-d', str(config), '--ros-args',
                           '-r', '__node:=capture_preview_'+side+'_'+epoch]
                window = OwnedRviz(side, command, env, root, output/(side+'.log'))
                windows.append(window); window.start()
        if args.layout == 'unified':
            from .project_paths import project_root
            executable = project_root()/'install/main/wc_bringup/lib/wc_bringup/unified_capture_rviz'
            if not executable.is_file():
                raise RuntimeError('unified capture RViz executable is not installed')
            selected = 'both' if len(plan['lidars']) == 2 else next(iter(plan['lidars']))
            command = [str(executable),'--lidars',selected,'--session-root',str(root),'--session-id',args.session_id]
            for side in plan['lidars']:
                command += ['--'+side+'-config',str(output/(side+'.rviz'))]
            window = OwnedRviz('unified',command,env,root,output/'unified.log')
            windows.append(window); window.start()
        # Subscriptions and a launched RViz do not certify a rendered window.
        # In particular OpenGL version output can precede a context failure.
        status.update(status='SUBSCRIBING', windows=[window.metadata() for window in windows])
        update_render_status()
        write_status()
        write_report(root/'capture_preview_ready.json', dict(session_id=args.session_id, preview_epoch=epoch,
            ready=True, ready_scope='SUBSCRIPTION_BRIDGE', rendering_verified=False,
            selected_backend=args.renderer, subscriptions_only=True, hardware_started=False, controls_enabled=plan['controls_enabled']))
        next_status = 0.
        while not stopped[0]:
            rclpy.spin_once(node, timeout_sec=.05)
            update_render_status()
            closed = [(window.side, window.poll()) for window in windows if window.poll() is not None]
            if closed:
                status['stop_reason'] = 'PREVIEW_WINDOW_CLOSED'
                if any(code != 0 for _, code in closed):
                    raise RuntimeError('preview RViz exited with an error: '+str(closed))
                break
            now = time.monotonic()
            if now >= next_status:
                current = _json(root/'capture_manifest.json')
                if current.get('session_id') != args.session_id or current.get('status') != 'RECORDING':
                    status['stop_reason'] = 'CAPTURE_SESSION_ENDED'; break
                status['rejected_messages'] = dict(cache.rejected)
                update_source_status(time.monotonic_ns())
                write_status(); next_status = now+1
        status.setdefault('stop_reason', 'PARENT_OR_USER_SIGNAL')
    except Exception as error:
        if str(error) not in status['errors']:
            status['errors'].append(str(error))
    finally:
        for window in windows:
            try: window.request_stop()
            except Exception as error: status['errors'].append('preview stop: '+str(error))
        status['window_cleanup'] = []
        for window in windows:
            try:
                result = window.finish(); status['window_cleanup'].append(result)
                if result['status'] != 'PASS': status['errors'].append('preview cleanup: '+str(result))
            except Exception as error: status['errors'].append('preview cleanup: '+str(error))
        # Context errors may only reach stderr when RViz receives its stop
        # signal. Preserve that failure even if its process exited with zero.
        try:
            update_render_status(fail_on_error=False)
        except Exception as error:
            status['errors'].append('preview rendering observation cleanup: '+str(error))
        if node is not None:
            try: node.destroy_node()
            except Exception as error: status['errors'].append('preview node cleanup: '+str(error))
        if ros_started and rclpy.ok():
            try: rclpy.shutdown()
            except Exception as error: status['errors'].append('preview ROS cleanup: '+str(error))
        for sig, previous in handlers.items(): signal.signal(sig, previous)
        status['status'] = 'FAILED' if status['errors'] else 'CLOSED'
        status['rejected_messages'] = dict(cache.rejected)
        try:
            update_source_status(time.monotonic_ns())
            write_status()
            write_report(root/'capture_preview_ready.json', dict(session_id=args.session_id, preview_epoch=epoch,
                ready=False, status=status['status'], ready_scope='SUBSCRIPTION_BRIDGE',
                rendering_verified=False, selected_backend=args.renderer,
                subscriptions_only=True, hardware_started=False))
        except OSError as error:
            print('preview status persistence failed: '+str(error), file=sys.stderr)
            return 2
    return 2 if status['errors'] else 0


if __name__ == '__main__':
    raise SystemExit(main())
