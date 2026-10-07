#!/usr/bin/env python3
"""Native RTAB A/B from derived frontend CDR; no devices or original bag replay.

Run only under root scheduling on the verified target. Each cell gets a new
database. No trajectory estimation or cloud filtering is implemented here.
"""
import argparse
import copy
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
from .resource_audit import snapshot as resource_snapshot,delta as resource_delta


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def digest(path):
    result = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            result.update(block)
    return result.hexdigest()


def write(path, value):
    with Path(path).open('x', encoding='utf-8') as stream:
        json.dump(value, stream, ensure_ascii=False, allow_nan=False, indent=2)
        stream.write('\n')


def stamp(message):
    return int(message.header.stamp.sec)*10**9 + int(message.header.stamp.nanosec)


def load_index(directory):
    result = {}
    with (directory/'frontend/index.jsonl').open(encoding='utf-8') as stream:
        for line in stream:
            row = json.loads(line)
            key = row['stamp_ns']
            require(type(key) is int and key > 0 and key not in result, 'Duplicate/invalid frontend stamp')
            for field in ('cloud_cdr_file', 'odom_cdr_file'):
                filename = Path(row[field])
                require(filename.name == str(filename) and filename.suffix == '.cdr', 'Invalid frontend CDR name')
            require(row['frame_id'] == row['odom_child'] == 'mapping_reference' and
                    row['odom_parent'] == 'mapping_odom', 'Unexpected frontend frames')
            result[key] = row
    require(result, 'Empty frontend stream')
    return result


def subsample(stamps, hz, limit):
    interval = int(math.ceil(1e9/hz))
    selected = []
    for key in sorted(stamps):
        if not selected or key-selected[-1] >= interval:
            selected.append(key)
            if limit and len(selected) >= limit:
                break
    require(selected, 'No common native input')
    return selected


def load_spec(root, directory, config, *, storage_root=None):
    path = root/'src/wc_bringup/launch/mapping_app.launch.py'
    spec = importlib.util.spec_from_file_location('route1_production_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    data_root = root if storage_root is None else Path(storage_root)
    result = module.build_spec(directory, project_root=data_root, odometry_source='wheel_imu', runtime_config=config)
    require(len(result['nodes']) == 1 and result['nodes'][0]['package'] == 'rtabmap_slam'
            and result['nodes'][0]['executable'] == 'rtabmap', 'Production launch contains unexpected nodes')
    parameters = result['nodes'][0]['parameters'][0]
    expected = {'RGBD/NeighborLinkRefining': 'false', 'RGBD/ProximityByTime': 'false',
                'RGBD/ProximityBySpace': 'false', 'Rtabmap/LoopThr': '1', 'RGBD/AggressiveLoopThr': '1'}
    require(all(parameters.get(key) == value for key, value in expected.items()), 'Native loop/refinement must remain disabled')
    parameters['use_sim_time'] = True  # Replay clock; per-instance rate is set by run_cell.
    return result


def with_native_detection_rate(spec, input_hz):
    """Set only this new offline instance's rate; preserve the loaded spec/config."""
    require(math.isfinite(input_hz) and 0 < input_hz <= 10,
            'Offline native DetectionRate must be finite and in (0,10] Hz')
    result = copy.deepcopy(spec)
    parameters = result['nodes'][0]['parameters'][0]
    key = 'Rtabmap/DetectionRate'
    explicit = key in parameters
    provenance = {'parameter': key, 'previous_was_explicit': explicit,
                  'previous_explicit_value': copy.deepcopy(parameters.get(key)),
                  'previous_value_source': 'LOADED_OFFLINE_SPEC' if explicit else 'UNSPECIFIED_BACKEND_DEFAULT',
                  'requested_hz': float(input_hz), 'applied_value': str(input_hz),
                  'effective_hz': None, 'scope': 'NEW_OFFLINE_NATIVE_INSTANCE_ONLY'}
    parameters[key] = provenance['applied_value']
    return result, provenance


def verify_native_detection_rate_readback(report, detection_hz):
    """Retain failed readbacks in JSON-safe evidence before rejecting a mismatch."""
    provenance = report['native_detection_rate']
    finite = math.isfinite(detection_hz)
    report['effective_native_detection_rate_hz'] = detection_hz if finite else None
    provenance['effective_hz'] = detection_hz if finite else None
    provenance['readback_value'] = str(detection_hz)
    matches = finite and detection_hz > 0 and math.isclose(
        detection_hz, provenance['requested_hz'], rel_tol=1e-9, abs_tol=1e-12)
    provenance['readback_matches_request'] = matches
    require(matches, 'Native DetectionRate readback differs from the requested offline rate: '
            +str(detection_hz)+' vs '+str(provenance['requested_hz']))


def run_cell(root, frontend, output, name, rows, selected, args, rclpy, *, storage_root=None):
    resources_before=resource_snapshot()
    import yaml
    from ament_index_python.packages import get_package_prefix
    from rclpy.serialization import deserialize_message
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from rcl_interfaces.srv import GetParameters
    from sensor_msgs.msg import PointCloud2
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import TransformStamped
    from tf2_msgs.msg import TFMessage
    from rosgraph_msgs.msg import Clock
    from rtabmap_msgs.msg import Info, MapGraph
    from wc_runtime.map_viewer import inspect_source, export_database, sqlite_read

    source = frontend/name
    directory = output/name
    directory.mkdir()
    (directory/'slam').mkdir()
    config = json.loads((source/'runtime_config.json').read_text(encoding='utf-8'))
    require(config.get('odometry_source') == 'wheel_imu', 'Native A/B requires wheel_imu authority')
    write(directory/'runtime_config.json', config)
    spec = load_spec(root, directory, config, storage_root=storage_root)
    spec, detection_rate = with_native_detection_rate(spec, args.input_hz)
    write(directory/'native_spec.json', spec)
    native = spec['nodes'][0]
    parameters = directory/'native_params.yaml'
    parameters.write_text(yaml.safe_dump({'/**': {'ros__parameters': native['parameters'][0]}}, sort_keys=False), encoding='utf-8')
    executable = Path(get_package_prefix('rtabmap_slam'))/'lib/rtabmap_slam/rtabmap'
    require(executable.is_file(), 'Native RTAB executable not found')
    command = [sys.executable, '-m', 'wc_runtime.component', '--parent', str(os.getpid()),
               '--sigint-grace-s', '120', '--', str(executable), '--ros-args', '--params-file', str(parameters),
               '-r', '__node:='+native['name'], '-r', '__ns:='+native['namespace'], *native['ros_arguments']]
    for old, new in native['remappings']:
        command += ['-r', old+':='+new]
    report = {'status': 'FAIL', 'validation_level': 'REAL_BAG_DERIVED_NATIVE_REPLAY', 'cell': name,
              'hardware_started': False, 'control_topics_published': False, 'native_command': command,
              'native_spec': spec, 'input_frames_available': len(rows), 'selected_frames': len(selected),
              'selected_source_span_s': (selected[-1]-selected[0])*1e-9,
              'input_hz_max': args.input_hz, 'wall_min_interval_s': args.wall_interval,
              'published_pairs': 0, 'acknowledged_pairs': 0, 'processed_info': [], 'graphs': [],
              'offline_override': {'use_sim_time': True,
                                   'Rtabmap/DetectionRate': detection_rate['applied_value']},
              'native_detection_rate': detection_rate,
              'comparison_scope': 'Same recorded source inputs; estimator/filter/input-rate factors are frozen in each cell config.',
              'limitation': 'Common source-time subsample, not full frontend frame replay or accuracy acceptance.'}
    node = rclpy.create_node('route1_native_replay_'+name)
    child, log = None, None
    acknowledged = set()
    started = time.monotonic()

    def spin_until(predicate, timeout, reason):
        deadline = time.monotonic()+timeout
        while not predicate():
            if child is not None:
                require(child.poll() is None, 'Owned native node exited early')
            require(time.monotonic() < deadline, reason)
            rclpy.spin_once(node, timeout_sec=min(.05, max(0., deadline-time.monotonic())))

    def info(message):
        key = stamp(message)
        row = {'stamp_ns': key}
        for field in ('ref_id', 'loop_closure_id', 'proximity_detection_id', 'landmark_id'):
            if hasattr(message, field):
                row[field] = int(getattr(message, field))
        report['processed_info'].append(row)
        acknowledged.add(key)

    def graph(message):
        report['graphs'].append({'stamp_ns': stamp(message), 'nodes': len(message.poses_id),
                                'ids': list(message.poses_id),
                                'positions': [[p.position.x, p.position.y, p.position.z] for p in message.poses]})

    try:
        until = time.monotonic()+2.
        while time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=.05)
        foreign = [(n, ns) for n, ns in node.get_node_names_and_namespaces() if n != node.get_name()]
        require(not foreign, 'Isolated ROS domain occupied: '+repr(foreign))
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        observed = QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT)
        cloud_pub = node.create_publisher(PointCloud2, '/wc_mapping/app/scan_cloud', qos)
        odom_pub = node.create_publisher(Odometry, '/wc_mapping/app/odom', qos)
        tf_pub = node.create_publisher(TFMessage, '/wc_mapping/app/tf', qos)
        clock_pub = node.create_publisher(Clock, '/clock', qos)
        subscriptions = [node.create_subscription(Info, '/wc_mapping/app/info', info, observed),
                         node.create_subscription(MapGraph, '/wc_mapping/app/mapGraph', graph, observed)]
        log = (directory/'native.log').open('xb')
        child = subprocess.Popen(command, cwd=root, env=os.environ.copy(), stdin=subprocess.DEVNULL,
                                 stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        spin_until(lambda: cloud_pub.get_subscription_count() >= 1 and odom_pub.get_subscription_count() >= 1,
                   30., 'Native scan/odom subscribers not ready')
        # Read back the explicitly requested rate from this isolated offline
        # instance; never infer that writing the YAML made the setting effective.
        client = node.create_client(GetParameters, '/wc_mapping/app/rtabmap/get_parameters')
        spin_until(client.service_is_ready, 10., 'Native parameter service unavailable')
        request = GetParameters.Request(names=['Rtabmap/DetectionRate'])
        future = client.call_async(request)
        spin_until(future.done, 10., 'Native DetectionRate query timed out')
        response = future.result()
        require(response is not None and len(response.values) == 1, 'Native DetectionRate missing')
        value = response.values[0]
        require(value.type in (2, 3, 4), 'Native DetectionRate parameter is undeclared or has unexpected type')
        detection_hz = float(value.string_value if value.type == 4 else value.double_value if value.type == 3 else value.integer_value)
        verify_native_detection_rate_readback(report, detection_hz)
        last_wall = None
        for key in selected:
            row = rows[key]
            cdrs = {}
            for kind in ('cloud', 'odom'):
                path = source/'frontend'/row[kind+'_cdr_file']
                require(not path.is_symlink(), 'Refusing linked frontend CDR')
                payload = path.read_bytes()
                require(hashlib.sha256(payload).hexdigest() == row[kind+'_cdr_sha256'], 'Derived CDR hash mismatch')
                cdrs[kind] = payload
            cloud, odom = deserialize_message(cdrs['cloud'], PointCloud2), deserialize_message(cdrs['odom'], Odometry)
            require(stamp(cloud) == stamp(odom) == key and cloud.header.frame_id == odom.child_frame_id == 'mapping_reference'
                    and odom.header.frame_id == 'mapping_odom', 'Derived CDR stamp/frame mismatch')
            if last_wall is not None:
                spin_until(lambda: time.monotonic()-last_wall >= args.wall_interval,
                           args.wall_interval+2., 'Wall pacing failed')
            identity = TransformStamped()
            identity.header = copy.deepcopy(odom.header)
            identity.child_frame_id = 'prior_odom'
            identity.transform.rotation.w = 1.
            pose_tf = TransformStamped()
            pose_tf.header = copy.deepcopy(odom.header)
            pose_tf.header.frame_id = 'prior_odom'
            pose_tf.child_frame_id = odom.child_frame_id
            p = odom.pose.pose.position
            pose_tf.transform.translation.x, pose_tf.transform.translation.y, pose_tf.transform.translation.z = p.x, p.y, p.z
            pose_tf.transform.rotation = copy.deepcopy(odom.pose.pose.orientation)
            clock_pub.publish(Clock(clock=odom.header.stamp))
            tf_pub.publish(TFMessage(transforms=[identity, pose_tf]))
            odom_pub.publish(odom)
            cloud_pub.publish(cloud)
            last_wall = time.monotonic()
            report['published_pairs'] += 1
            spin_until(lambda: key in acknowledged, args.frame_timeout,
                       'No native Info acknowledgement for original stamp '+str(key))
            report['acknowledged_pairs'] += 1
            if report['acknowledged_pairs'] % 20 == 0:
                print(name, report['acknowledged_pairs'], '/', len(selected), flush=True)
        until = time.monotonic()+.5
        while time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=.05)
        require(report['graphs'], 'Native graph was never received')
        require(not any(row.get('loop_closure_id', 0) > 0 or row.get('proximity_detection_id', 0) > 0
                        for row in report['processed_info']), 'Unexpected loop closure/proximity correction')
        report['status'] = 'REPLAY_COMPLETE_REQUIRES_CLOSED_DATABASE'
    except BaseException as error:
        report['error'] = type(error).__name__+': '+str(error)
    finally:
        if child is not None:
            if child.poll() is None:
                child.send_signal(signal.SIGINT)
            try:
                child.wait(timeout=135.)
            except subprocess.TimeoutExpired:
                # Wrapper owns all descendants; keep it alive rather than orphaning
                # a database writer. Its cleanup already has bounded escalation.
                report['cleanup_error'] = 'Owned component wrapper did not reap in 135 seconds'
                report['unreaped_owner_pid'] = child.pid
            report['native_exit_code'] = child.poll()
            if report['native_exit_code'] != 0:
                report['status'] = 'FAIL'
        node.destroy_node()
        if log is not None:
            log.close()
    try:
        require(report['status'] == 'REPLAY_COMPLETE_REQUIRES_CLOSED_DATABASE', 'Replay/normal close failed')
        log_text = (directory/'native.log').read_text(errors='replace')
        report['native_warning_error_lines'] = [line for line in log_text.splitlines()
            if '[WARN]' in line or '[ERROR]' in line or '[FATAL]' in line]
        require('Saving database/long-term memory...done!' in log_text, 'Native normal-close database barrier missing')
        database = directory/'slam/rtabmap.db'
        def database_statistics(connection):
            return {'nodes': connection.execute('SELECT COUNT(*) FROM Node').fetchone()[0],
                    'links_by_type': connection.execute('SELECT type,COUNT(*) FROM Link GROUP BY type').fetchall(),
                    'node_stamp_bounds': connection.execute('SELECT MIN(stamp),MAX(stamp) FROM Node').fetchone()}
        report['database'], report['database_fingerprint'] = sqlite_read(database, database_statistics)
        require(report['database']['nodes'] > 0, 'Native closed database has no nodes')
        from .map_quality import inspect_map_quality
        report['map_quality']=inspect_map_quality(database,policy=config.get('map_quality_policy'))
        write(directory/'map_quality.json',report['map_quality'])
        require(report['map_quality']['file_integrity']['status']=='PASS', 'Native database file integrity check failed')
        # Static prefixes remain useful and exportable. Qualification is a
        # separate explicit result, never promoted by a successful file export.
        report['moving_map_qualification']=report['map_quality']['movement_map_qualification']['status']
        map_source = {'path': directory, 'database': database, 'geometry': None}
        # Inspection receives the actual RAM/disk source path. Export's
        # project_root is its subprocess cwd/code environment, not a data guard.
        exported = inspect_source(map_source)
        export_database(map_source, exported, directory, project_root=root)
        report['map_export'] = exported
        report['status'] = 'NATIVE_REPLAY_AND_EXPORT_COMPLETE_REQUIRES_VISUAL_REVIEW'
    except BaseException as error:
        report['status'] = 'FAIL'
        report['database_or_export_error'] = type(error).__name__+': '+str(error)
    report['wall_elapsed_s'] = time.monotonic()-started
    report['resources']=resource_delta(resources_before,resource_snapshot())
    write(directory/'result.json', report)
    require(report['status'] == 'NATIVE_REPLAY_AND_EXPORT_COMPLETE_REQUIRES_VISUAL_REVIEW',
            name+' failed; see '+str(directory/'result.json'))
    return report



