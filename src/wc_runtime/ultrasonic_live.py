"""Live FD07 ultrasonic ranges: the capture's read-only transactions plus sensor_msgs/Range and a status topic.

Window A, 2026-11-13 (coordinator relay A3 item 3; user decision 11-12 22:05: the four ultrasonic probes as one more
brake). Off unless asked for: mapping_live.json "ultrasonic_enabled" (default false) starts it in the panel's live
session; ``python3 -s -m wc_runtime.ultrasonic_live --allow-read-queries ...`` runs it alone (field check).

Device access is ultrasonic_capture's: the same identity check, lease and FC03 / register 0x0001 request, polled
address by address with the configured inter-sensor interval. Each transaction is
  - journaled (<output>/events.jsonl: the capture's row fields, then one source_finalization row) and published as
    that JSON row on /wc_mapping/ultrasonic/feedback_raw (std_msgs/String; the session's bag records it);
  - when valid, also published as sensor_msgs/Range on /wc_mapping/ultrasonic/<probe>/range: frame
    ultrasonic_<probe>, ULTRASOUND, the configured field of view and limits, range = raw * raw_unit_m; +inf for a
    no-echo value or beyond max_range_m, -inf below min_range_m (REP 117); stamp = the reply's arrival (host wall
    clock, the row's host_receive_ns).
A failed transaction is journaled, published raw and counted, without a Range, so its probe's age grows for
consumers; polling goes on. The process stops on a signal, after --cycles or --duration-s, or when the device lease
or the journal fails.

After every cycle a JSON status goes out on /wc_mapping/ultrasonic/status (latched): per probe the last status, raw
value, range and age, counts, the cycle time, this process's CPU, the geometry and what is not verified yet (address
to probe, raw units, limits, no-echo value, tape check of the geometry). "verified" stays false until all of them are
done: consumers must not brake on these ranges before that.

Geometry (``probe_transforms``, CAD_NOMINAL): each dual-hole module's CAD outward face centre relative to the
same-side lidar's CAD face centre (config/hardware_setup.json mechanical_reference), in the lidar's nominal axes
(the M frame's), with the nominal view direction (front +x, left +y, right -y). ``--publish-tf lidar`` sends
lidar_<side> -> ultrasonic_<probe> on /tf_static, ``--publish-tf base_link`` base_link (axle) -> ultrasonic_<probe>
through the CAD alone (axle_from_mounting_M), not through the right lidar's calibrated mount, which hardware_setup.json
marks as 8.6 cm forward of the symmetric housing. The panel session uses base_link (window D's base_link is the axle);
the mapping app and its RViz keep their own /wc_mapping/app/tf_static, so nothing there sees these frames.

``--check-binding`` prints where the ultrasonic CH340 (bound by USB port: it has no serial) and the wheel adapter
(bound by serial) are plugged in against the configuration; ``--write-local-binding`` then writes the port seen now
into config/local/project/ultrasonic_capture.json (the field override; the repository file stays as it is).
"""
import argparse
from collections import deque
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import uuid

import numpy as np

PROBES = ('left_front', 'right_front', 'left_side', 'right_side')
VIEWS = {'front': 0.0, 'left': math.pi / 2, 'right': -math.pi / 2}
RAW_TOPIC = '/wc_mapping/ultrasonic/feedback_raw'
STATUS_TOPIC = '/wc_mapping/ultrasonic/status'
CHECKS = {'address_mapping_status': ('UNVERIFIED', 'VERIFIED'), 'raw_unit_status': ('UNVERIFIED', 'VERIFIED'),
          'no_echo_status': ('UNMEASURED', 'MEASURED'), 'range_limits_status': ('UNMEASURED', 'MEASURED'),
          'geometry_tape_check': ('NOT_DONE', 'DONE')}


def range_topic(name):
    return '/wc_mapping/ultrasonic/%s/range' % name


def frame_id(name):
    return 'ultrasonic_' + name


TOPICS = (RAW_TOPIC, STATUS_TOPIC) + tuple(range_topic(name) for name in PROBES)


def _number(value, low, high, name, *, low_open=True):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) \
            or value > high or (value <= low if low_open else value < low):
        raise ValueError('%s must be a number within %s%g, %g]' % (name, '(' if low_open else '[', low, high))
    return float(value)


def validate_live_config(value):
    """The live configuration (config/ultrasonic_live.json), checked; returned unchanged."""
    if not isinstance(value, dict) or value.get('schema_version') != 1:
        raise ValueError('ultrasonic live configuration must be an object with schema_version 1')
    probes = value.get('probes')
    if not isinstance(probes, list) or len(probes) != len(PROBES):
        raise ValueError('ultrasonic live configuration needs exactly four probes')
    names, addresses = set(), set()
    for probe in probes:
        if not isinstance(probe, dict) or set(probe) != {'name', 'address', 'lidar', 'view', 'cad_point'}:
            raise ValueError('each probe needs exactly name, address, lidar, view and cad_point')
        if probe['name'] not in PROBES or type(probe['address']) is not int or probe['address'] not in (1, 2, 3, 4) \
                or probe['lidar'] not in ('left', 'right') or probe['view'] not in VIEWS \
                or not isinstance(probe['cad_point'], str) or not probe['cad_point']:
            raise ValueError('invalid probe entry: %r' % (probe,))
        names.add(probe['name'])
        addresses.add(probe['address'])
    if names != set(PROBES) or addresses != {1, 2, 3, 4}:
        raise ValueError('the four probes need distinct names and addresses 1..4')
    for key, allowed in CHECKS.items():
        if value.get(key) not in allowed:
            raise ValueError('%s must be one of %s' % (key, ', '.join(allowed)))
    if value.get('geometry_status') != 'CAD_NOMINAL':
        raise ValueError('geometry_status must be CAD_NOMINAL')
    _number(value.get('raw_unit_m'), 0.0, 0.1, 'raw_unit_m')
    _number(value.get('field_of_view_rad'), 0.0, math.pi, 'field_of_view_rad')
    low = _number(value.get('min_range_m'), 0.0, 20.0, 'min_range_m')
    high = _number(value.get('max_range_m'), 0.0, 20.0, 'max_range_m')
    if low >= high:
        raise ValueError('min_range_m must be below max_range_m')
    _number(value.get('status_period_s'), 0.2, 10.0, 'status_period_s', low_open=False)
    echo = value.get('no_echo_raw')
    if not isinstance(echo, list) or any(type(v) is not int or not 0 <= v <= 65535 for v in echo):
        raise ValueError('no_echo_raw must be a list of raw register values')
    return value


def not_verified(cfg):
    """The checks still open (empty when every one is done)."""
    return [key for key, allowed in CHECKS.items() if cfg[key] != allowed[1]]


def range_value(row, cfg):
    """Range (m) of one transaction row by REP 117 (+inf no echo or beyond max, -inf below min); None if invalid."""
    if row.get('status') != 'VALID_RESPONSE':
        return None
    raw = row['response_value_u16']
    if raw in cfg['no_echo_raw']:
        return math.inf
    value = raw * cfg['raw_unit_m']
    if value > cfg['max_range_m']:
        return math.inf
    if value < cfg['min_range_m']:
        return -math.inf
    return value


def _rz(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def _quaternion(rotation):
    from scipy.spatial.transform import Rotation
    return [float(v) for v in Rotation.from_matrix(rotation).as_quat()]


def probe_transforms(cfg, hardware_setup):
    """Per probe: the CAD_NOMINAL transforms lidar_<side> -> ultrasonic_<probe> and base_link -> ultrasonic_<probe>.

    ``T_lidar_probe``: the module's CAD outward face centre minus the same-side lidar's, in the lidar's nominal axes
    (M axes), and the view yaw. ``T_base_probe``: base_link (axle) -> probe through the CAD alone. When the hardware
    setup resolves the lidar's mount, ``via_lidar_mount_minus_cad_m`` is how far the probe would move if it were
    hung on that mount instead.
    """
    reference = hardware_setup['mechanical_reference']
    if reference.get('unit') != 'mm' or reference.get('frame_id') != 'mounting_M':
        raise ValueError('mechanical reference must be the mounting_M frame in millimetres')
    points = {point['id']: point for point in reference['reference_points']}
    axle = next(t for t in hardware_setup['data_transforms'] if t['id'] == 'axle_from_mounting_M')
    if axle['parent_frame'] != 'axle' or axle['child_frame'] != 'mounting_M' or axle['direction'] != 'child_to_parent':
        raise ValueError('axle_from_mounting_M must map mounting_M into the axle frame')
    r_axle_m = np.asarray(axle['rotation']['value_matrix'], dtype=float)
    t_axle_m = np.asarray(axle['translation']['value_m'], dtype=float)
    mounts = {}
    try:
        from .hardware_setup import resolve_hardware_setup
        mounts = resolve_hardware_setup(hardware_setup).get('mounts', {})
    except Exception:
        mounts = {}
    out = {}
    for probe in cfg['probes']:
        lidar = points['lidar_%s_mechanical_reference' % probe['lidar']]
        cad = points[probe['cad_point']]
        for point in (lidar, cad):
            if point.get('status') != 'CAD_NOMINAL' or point.get('point_kind') != 'CAD_OUTWARD_FACE_CENTER':
                raise ValueError('%s is not a CAD_NOMINAL outward face centre' % point['id'])
        view = np.asarray(cad['nominal_view_direction_M'], dtype=float)
        yaw = VIEWS[probe['view']]
        if not np.allclose(view, _rz(yaw)[:, 0], atol=1e-9):
            raise ValueError('%s: configured view %s differs from the CAD view direction %s' % (
                probe['name'], probe['view'], view.tolist()))
        p_lidar = np.asarray(lidar['position_mm'], dtype=float) / 1000.0
        p_probe = np.asarray(cad['position_mm'], dtype=float) / 1000.0
        offset = p_probe - p_lidar
        rotation = r_axle_m @ _rz(yaw)
        base = t_axle_m + r_axle_m @ p_probe
        entry = {'status': 'CAD_NOMINAL', 'address': probe['address'], 'frame_id': frame_id(probe['name']),
                 'lidar_frame': 'lidar_' + probe['lidar'], 'cad_point': probe['cad_point'],
                 'T_lidar_probe': {'translation_m': [round(float(v), 6) for v in offset],
                                   'yaw_rad': yaw, 'rotation_xyzw': _quaternion(_rz(yaw))},
                 'T_base_probe': {'parent': 'base_link', 'translation_m': [round(float(v), 6) for v in base],
                                  'yaw_rad': float(math.atan2(rotation[1, 0], rotation[0, 0])),
                                  'rotation_xyzw': _quaternion(rotation)}}
        mount = mounts.get(probe['lidar'])
        if mount is not None:
            via = np.asarray(mount['t_axle_lidar_m'], dtype=float) + np.asarray(mount['R_axle_lidar'], dtype=float) @ offset
            entry['via_lidar_mount_minus_cad_m'] = [round(float(v), 4) for v in via - base]
        out[probe['name']] = entry
    return out


class StatusBook:
    """What the status topic reports: per probe the last transaction, counts and age; cycle times; CPU."""

    def __init__(self, cfg, transforms, *, config_sha256=None, cpu_clock=time.process_time):
        self.cfg, self.transforms, self.config_sha256 = cfg, transforms, config_sha256
        self.cpu_clock = cpu_clock
        self.started_cpu, self.started_ns = cpu_clock(), time.monotonic_ns()
        self.window = deque(maxlen=64)                    # (monotonic ns, cpu s) per cycle
        self.window.append((self.started_ns, self.started_cpu))
        self.cycles = deque(maxlen=200)
        self.transactions = 0
        self.probes = {p['name']: {'address': p['address'], 'topic': range_topic(p['name']),
                                   'frame_id': frame_id(p['name']), 'status': None, 'raw': None,
                                   'range_m': None, 'received_wall_ns': None, 'received_monotonic_ns': None,
                                   'counts': {}, 'consecutive_failures': 0} for p in cfg['probes']}

    def record(self, name, row, value):
        probe = self.probes[name]
        self.transactions += 1
        status = row.get('status', 'UNKNOWN')
        probe['counts'][status] = probe['counts'].get(status, 0) + 1
        probe['status'] = status
        probe['received_wall_ns'], probe['received_monotonic_ns'] = row['host_receive_ns'], row['host_monotonic_ns']
        if status == 'VALID_RESPONSE':
            probe['raw'], probe['range_m'], probe['consecutive_failures'] = row['response_value_u16'], value, 0
        else:
            probe['consecutive_failures'] += 1

    def cycle(self, started_ns, ended_ns):
        self.cycles.append((ended_ns - started_ns) / 1e9)
        self.window.append((ended_ns, self.cpu_clock()))

    def message(self, now_ns=None):
        now_ns = time.monotonic_ns() if now_ns is None else now_ns
        open_checks = not_verified(self.cfg)
        cycles = np.array(self.cycles) if self.cycles else np.zeros(0)
        (t0, c0), (t1, c1) = self.window[0], self.window[-1]
        recent = 100.0 * (c1 - c0) / ((t1 - t0) / 1e9) if t1 > t0 else None
        total = (now_ns - self.started_ns) / 1e9
        probes = {}
        for name, probe in self.probes.items():
            value = probe['range_m']
            probes[name] = dict({k: v for k, v in probe.items() if k != 'range_m'},
                                range_m=None if value is None else (value if math.isfinite(value) else
                                                                     ('+inf' if value > 0 else '-inf')),
                                age_s=None if probe['received_monotonic_ns'] is None else
                                round((now_ns - probe['received_monotonic_ns']) / 1e9, 3))
        failing = [name for name, probe in self.probes.items() if probe['status'] != 'VALID_RESPONSE']
        return {'schema_version': 1, 'stamp_wall_ns': time.time_ns(),
                'state': 'RUNNING' if not failing else 'DEGRADED', 'failing_probes': failing,
                'verified': not open_checks, 'not_verified': open_checks,
                'transactions': self.transactions, 'cycles': int(len(self.cycles)),
                'cycle_s': None if not len(cycles) else {'last': round(float(cycles[-1]), 3),
                                                         'median': round(float(np.median(cycles)), 3),
                                                         'max': round(float(cycles.max()), 3)},
                'cpu_percent_recent': None if recent is None else round(recent, 2),
                'cpu_percent_since_start': round(100.0 * (self.cpu_clock() - self.started_cpu) / total, 2)
                if total > 0 else None,
                'limits': {k: self.cfg[k] for k in ('raw_unit_m', 'field_of_view_rad', 'min_range_m', 'max_range_m',
                                                    'no_echo_raw')},
                'geometry': self.transforms, 'config_sha256': self.config_sha256, 'probes': probes}


def _stamp(message_stamp, wall_ns):
    message_stamp.sec, message_stamp.nanosec = divmod(int(wall_ns), 1_000_000_000)


def binding_report(root):
    """Where the ultrasonic CH340 and the wheel adapter are plugged in, against the selected configurations."""
    from .project_config import selected_config_path
    from .ultrasonic_capture import validate_config

    def properties(device):
        return dict(row.split('=', 1) for row in subprocess.check_output(
            ['udevadm', 'info', '--query=property', '--name=' + str(device)], text=True, timeout=5).splitlines()
            if '=' in row)

    root = Path(root)
    path = selected_config_path(root, 'ultrasonic_capture.json')
    cfg = validate_config(json.loads(path.read_text(encoding='utf-8')))
    ch340 = []
    for node in sorted(Path('/dev').glob('ttyUSB*')):
        info = properties(node)
        if info.get('ID_VENDOR_ID') == cfg['usb_vid'] and info.get('ID_MODEL_ID') == cfg['usb_pid']:
            ch340.append({'node': str(node), 'usb_path': info.get('ID_PATH')})
    report = {'ultrasonic': {'config_file': str(path), 'configured_usb_path': cfg['expected_usb_path'],
                             'adapters_found': ch340}}
    if len(ch340) == 1:
        report['ultrasonic']['usb_path_now'] = ch340[0]['usb_path']
        report['ultrasonic']['matches'] = ch340[0]['usb_path'] == cfg['expected_usb_path']
    else:
        report['ultrasonic']['matches'] = False
        report['ultrasonic']['problem'] = 'expected exactly one %s:%s adapter, found %d' % (
            cfg['usb_vid'], cfg['usb_pid'], len(ch340))
    wheel_path = selected_config_path(root, 'wheel_feedback_current.json')
    wheel = json.loads(wheel_path.read_text(encoding='utf-8'))
    try:
        node = Path(wheel['device']).resolve(strict=True)
        info = properties(node)
        report['wheel'] = {'config_file': str(wheel_path), 'node': str(node), 'usb_path_now': info.get('ID_PATH'),
                           'serial_now': info.get('ID_SERIAL_SHORT'), 'configured_serial': wheel.get('hardware_serial'),
                           'matches': info.get('ID_SERIAL_SHORT') == wheel.get('hardware_serial'),
                           'binding': 'serial number: any port works'}
    except (OSError, KeyError, subprocess.SubprocessError) as error:
        report['wheel'] = {'config_file': str(wheel_path), 'matches': False, 'problem': str(error)}
    return report


def write_local_binding(root, report, *, replace=False):
    """config/local/project/ultrasonic_capture.json: the selected configuration with the port seen now."""
    from .project_config import selected_config_path
    from .ultrasonic_capture import validate_config
    seen = report['ultrasonic']
    if len(seen.get('adapters_found', [])) != 1:
        raise RuntimeError('ULTRASONIC_BINDING_AMBIGUOUS: ' + seen.get('problem', ''))
    root = Path(root)
    target = root / 'config/local/project/ultrasonic_capture.json'
    if target.exists() and not replace:
        raise RuntimeError('ULTRASONIC_LOCAL_BINDING_EXISTS: %s (add --replace)' % target)
    cfg = json.loads(selected_config_path(root, 'ultrasonic_capture.json').read_text(encoding='utf-8'))
    cfg['expected_usb_path'] = seen['usb_path_now']
    cfg['evidence'] = ('%s field binding by wc_runtime.ultrasonic_live --write-local-binding: the only %s:%s adapter '
                       'is at %s (udev ID_PATH). Repository default and earlier evidence: see '
                       'config/ultrasonic_capture.json.' % (time.strftime('%Y-%m-%d %H:%M'), cfg['usb_vid'],
                                                            cfg['usb_pid'], seen['usb_path_now']))
    validate_config(cfg)
    target.parent.mkdir(parents=True, exist_ok=True)
    data = (json.dumps(cfg, ensure_ascii=False, indent=2) + '\n').encode('utf-8')
    with tempfile.NamedTemporaryFile(dir=target.parent, prefix='.ultrasonic_capture.', delete=False) as stream:
        stream.write(data)
        stream.flush()
        os.fsync(stream.fileno())
        temporary = Path(stream.name)
    os.replace(temporary, target)
    return target


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--project-root', type=Path, default=Path(__file__).resolve().parents[2])
    parser.add_argument('--capture-config', type=Path, help='ultrasonic_capture.json (default: the selected one)')
    parser.add_argument('--live-config', type=Path, help='ultrasonic_live.json (default: the selected one)')
    parser.add_argument('--hardware-setup', type=Path, help='hardware_setup.json (default: the selected one)')
    parser.add_argument('--output', type=Path, help='new directory for the raw transaction journal')
    parser.add_argument('--run-root', type=Path)
    parser.add_argument('--session', default='ultrasonic_live')
    parser.add_argument('--allow-read-queries', action='store_true')
    parser.add_argument('--no-ros', action='store_true', help='journal and stdout only')
    parser.add_argument('--publish-tf', choices=('off', 'lidar', 'base_link'), default='off')
    parser.add_argument('--cycles', type=int, default=0, help='stop after this many cycles (0: until stopped)')
    parser.add_argument('--duration-s', type=float, default=0.0, help='stop after this long (0: until stopped)')
    parser.add_argument('--check-binding', action='store_true')
    parser.add_argument('--write-local-binding', action='store_true')
    parser.add_argument('--replace', action='store_true')
    parser.add_argument('--print-transforms', action='store_true')
    args = parser.parse_args(argv)
    from .project_config import selected_config_path
    root = args.project_root
    if args.check_binding or args.write_local_binding:
        report = binding_report(root)
        if args.write_local_binding and not report['ultrasonic']['matches']:
            report['written'] = str(write_local_binding(root, report, replace=args.replace))
        print(json.dumps(report, ensure_ascii=False, indent=1))
        ok = report['wheel'].get('matches') and (report['ultrasonic']['matches'] or 'written' in report)
        return 0 if ok else 1
    live_path = args.live_config or selected_config_path(root, 'ultrasonic_live.json')
    live_raw = live_path.read_bytes()
    live = validate_live_config(json.loads(live_raw.decode('utf-8')))
    setup = json.loads((args.hardware_setup or selected_config_path(root, 'hardware_setup.json')).read_text(
        encoding='utf-8'))
    transforms = probe_transforms(live, setup)
    if args.print_transforms:
        print(json.dumps(transforms, indent=1))
        return 0
    if not args.allow_read_queries:
        raise ValueError('explicit read-query selection required')
    if args.output is None or args.run_root is None:
        raise ValueError('--output and --run-root are required to poll the probes')
    if args.cycles < 0 or args.duration_s < 0:
        raise ValueError('--cycles and --duration-s must not be negative')
    from .source_archive import SourceJournal, atomic_json
    from .ultrasonic_capture import DistanceLease, observed_identity, validate_config
    capture = validate_config(json.loads((args.capture_config or selected_config_path(
        root, 'ultrasonic_capture.json')).read_text(encoding='utf-8')))
    stopped = False

    def stop(*_):
        nonlocal stopped
        stopped = True
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, stop)
    by_address = {p['address']: p['name'] for p in live['probes']}
    book = StatusBook(live, transforms, config_sha256=hashlib.sha256(live_raw).hexdigest())
    journal = SourceJournal(args.output, 'ultrasonic')
    lease = DistanceLease(capture, args.run_root)
    ros = node = raw_pub = status_pub = None
    range_pubs, error, epoch, sequence, published = {}, None, str(uuid.uuid4()), 0, 0
    started = time.monotonic()
    try:
        if not args.no_ros:
            import rclpy as ros
            from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
            from rclpy.signals import SignalHandlerOptions
            from sensor_msgs.msg import Range
            from std_msgs.msg import String
            ros.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
            node = ros.create_node('wc_ultrasonic_live')
            raw_pub = node.create_publisher(String, RAW_TOPIC, 32)
            latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
            status_pub = node.create_publisher(String, STATUS_TOPIC, latched)
            range_pubs = {name: node.create_publisher(Range, range_topic(name), 10) for name in by_address.values()}
            if args.publish_tf != 'off':
                from geometry_msgs.msg import TransformStamped
                from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
                messages = []
                for name, entry in transforms.items():
                    message = TransformStamped()
                    message.header.stamp = node.get_clock().now().to_msg()
                    key = 'T_lidar_probe' if args.publish_tf == 'lidar' else 'T_base_probe'
                    message.header.frame_id = entry['lidar_frame'] if args.publish_tf == 'lidar' else 'base_link'
                    message.child_frame_id = entry['frame_id']
                    t, q = message.transform.translation, message.transform.rotation
                    t.x, t.y, t.z = entry[key]['translation_m']
                    q.x, q.y, q.z, q.w = entry[key]['rotation_xyzw']
                    messages.append(message)
                tf_static = StaticTransformBroadcaster(node)    # kept alive with the node until the end
                tf_static.sendTransform(messages)
        lease.open()
        atomic_json(args.output / 'identity.json', observed_identity(capture, lease, args.session))
        cycles = 0
        while not stopped:
            cycle_start = time.monotonic_ns()
            for address in capture['addresses']:
                if stopped:
                    break
                name = by_address[address]
                row = dict(lease.exchange(address), event='transaction', sensor_id='ultrasonic',
                           stream_epoch=epoch, sequence=sequence, session_id=args.session,
                           physical_role_status='UNVERIFIED', time_source='arrival_only', probe=name)
                journal.append(row)
                sequence += 1
                value = range_value(row, live)
                book.record(name, row, value)
                if raw_pub is not None:
                    raw_pub.publish(String(data=json.dumps(row)))
                    published += 1
                    if value is not None:
                        message = Range()
                        _stamp(message.header.stamp, row['host_receive_ns'])
                        message.header.frame_id = frame_id(name)
                        message.radiation_type = Range.ULTRASOUND
                        message.field_of_view = float(live['field_of_view_rad'])
                        message.min_range = float(live['min_range_m'])
                        message.max_range = float(live['max_range_m'])
                        message.range = float(value)
                        range_pubs[name].publish(message)
                deadline = time.monotonic() + capture['inter_sensor_delay_s']
                while not stopped and time.monotonic() < deadline:
                    if node is not None:
                        ros.spin_once(node, timeout_sec=min(0.02, max(0.0, deadline - time.monotonic())))
                    else:
                        time.sleep(max(0.0, min(0.02, deadline - time.monotonic())))
            book.cycle(cycle_start, time.monotonic_ns())
            cycles += 1
            status = book.message()
            if status_pub is not None:
                status_pub.publish(String(data=json.dumps(status)))
            if args.no_ros:
                print(json.dumps({k: status[k] for k in ('state', 'cycle_s', 'cpu_percent_recent')} | {
                    'raw': {n: p['raw'] for n, p in status['probes'].items()}}), flush=True)
            if (args.cycles and cycles >= args.cycles) or (args.duration_s and time.monotonic() - started >= args.duration_s):
                break
    except Exception as exc:
        error = str(exc)
    finally:
        cleanup_errors = []
        try:
            lease.close()
        except Exception as exc:
            cleanup_errors.append('lease close: ' + str(exc))
        final = book.message()
        try:
            journal.append({'event': 'source_finalization', 'session_id': args.session, 'stream_epoch': epoch,
                            'completed_transactions': sequence, 'published_transactions': published,
                            'lease_closed': not any(x.startswith('lease close:') for x in cleanup_errors),
                            'cleanup_errors': cleanup_errors, 'cycles': final['cycles'],
                            'cycle_s': final['cycle_s'], 'cpu_percent_since_start': final['cpu_percent_since_start'],
                            'counts': {n: p['counts'] for n, p in final['probes'].items()}})
        except Exception as exc:
            cleanup_errors.append('finalization journal: ' + str(exc))
        if cleanup_errors:
            error = '; '.join(([error] if error else []) + cleanup_errors)
        try:
            journal.close(source_error=error)
        finally:
            if node is not None:
                node.destroy_node()
            if ros is not None and ros.ok():
                ros.shutdown()
    if error:
        print(error, flush=True)
    return 2 if error else 0


if __name__ == '__main__':
    raise SystemExit(main())
