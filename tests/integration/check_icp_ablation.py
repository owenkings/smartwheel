#!/usr/bin/env python3
"""Bounded native ICP ablation, synthetic input only; never starts a graph/driver.

Usage: source the project ROS environment, then run with --session-config PATH
--output /home/nvidia/wheelchair/reports/ablation/NEW.json. Domain 87 must be empty.
Each variant consumes the same first 12 archived bundles in timestamp order;
wall-clock pacing waits for actual Odometry plus OdomInfo, never simulates Hz.
"""
import argparse
import ast
import fcntl
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import uuid

ROOT = Path('/home/nvidia/wheelchair')


def require(ok, reason):
    if not ok:
        raise RuntimeError(reason)


def digest(data):
    return hashlib.sha256(data).hexdigest()


def finite_number(value):
    import math
    return float(value) if math.isfinite(value) else None


def owned_stop(child, pidfd):
    sent = []
    for signum, timeout in ((signal.SIGINT, 5), (signal.SIGTERM, 3), (signal.SIGKILL, 2)):
        if child.poll() is not None:
            break
        try:
            if pidfd is None:
                child.send_signal(signum)  # Only our still-unreaped direct child.
            else:
                signal.pidfd_send_signal(pidfd, signum)
            sent.append(signal.Signals(signum).name)
        except ProcessLookupError:
            pass
        try:
            child.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            continue
    require(child.poll() is not None, 'owned ICP process failed bounded cleanup')
    return {'signals': sent, 'exit_code': child.returncode}


def load_inputs(config_path):
    import numpy as np
    config_path = config_path.resolve(strict=True)
    require(config_path.is_relative_to(ROOT/'data/synthetic'), 'config must be project synthetic data')
    before = {str(config_path): digest(config_path.read_bytes())}
    config = json.loads(config_path.read_bytes())
    require(config.get('source_mode') == 'synthetic' and config.get('sensor_mode') == 'dual', 'synthetic dual input required')
    session = Path(config['session_root']).resolve(strict=True)
    require(session.is_relative_to(ROOT/'data/synthetic'), 'synthetic session path outside project')
    raw = session/'raw_observations'
    require(not raw.is_symlink() and raw.is_dir(), 'raw archive directory missing/symlink')
    paths = list(raw.glob('*.json'))
    require(12 <= len(paths) <= 10000 and all(path.stem.isdecimal() for path in paths), 'need numbered, bounded raw bundle archive')
    indexed = []
    for path in sorted(paths, key=lambda value: int(value.stem))[:12]:
        require(not path.is_symlink() and path.stat().st_size <= 40_000_000, 'invalid raw file')
        data = path.read_bytes()
        item = json.loads(data)
        indexed.append((item['t_ref_ns'], path, item, digest(data)))
    require(len(indexed) >= 12, 'need at least 12 closed archived dual bundles')
    frames, stamps, keys = [], set(), set()
    for stamp, path, item, checksum in sorted(indexed, key=lambda value: value[0])[:12]:
        require(item.get('source_mode') == 'synthetic' and item.get('sensor_mode') == 'dual', 'real/single raw rejected')
        require(item.get('session_id') == config['session_id'], 'raw session mismatch')
        require(type(stamp) is int and 0 < stamp < (2**31)*10**9 and stamp not in stamps, 'invalid/repeated ROS stamp')
        stamps.add(stamp)
        sides = {}
        require(len(item['observations']) == 2, 'exactly two source observations required')
        for source in item['observations']:
            side = source['side']
            require(side in ('left', 'right') and side not in sides, 'duplicate/missing side')
            require(source['sensor_id'] == config['sensor_ids'][side], 'sensor ID mismatch')
            require(source['raw_key'] not in keys, 'repeated raw observation')
            keys.add(source['raw_key'])
            points, transform = np.asarray(source['points'], dtype=np.float64), np.asarray(source['T_ref_sensor'], dtype=np.float64)
            require(points.ndim == 2 and points.shape[1] == 3 and 20 <= len(points) <= 100000, 'invalid points')
            require(np.isfinite(points).all() and transform.shape == (4, 4) and np.isfinite(transform).all(), 'nonfinite geometry')
            require(np.allclose(transform[3], [0, 0, 0, 1]) and np.allclose(transform[:3, :3].T@transform[:3, :3], np.eye(3), atol=1e-5)
                    and np.isclose(np.linalg.det(transform[:3, :3]), 1, atol=1e-5), 'invalid rigid transform')
            sides[side] = np.asarray(points@transform[:3, :3].T+transform[:3, 3], dtype='<f4')
        sides['dual'] = np.concatenate([sides['left'], sides['right']])
        require(len({digest(value.tobytes()) for value in sides.values()}) == 3, 'ablation inputs must differ')
        before[str(path)] = checksum
        frames.append({'stamp': stamp, 'path': str(path), 'sides': sides,
                       'counts': {side: len(sides[side]) for side in ('left', 'right')}})
    return config, frames, before


def actual_parameters():
    path = ROOT/'src/wc_bringup/launch/processing.launch.py'
    source = path.read_bytes()
    assignments = [n for n in ast.walk(ast.parse(source)) if isinstance(n, ast.Assign)
                   and any(isinstance(t, ast.Name) and t.id == 'common' for t in n.targets)]
    require(len(assignments) == 1 and isinstance(assignments[0].value, ast.Dict), 'cannot verify actual ICP parameter dictionary')
    value = assignments[0].value
    parameters = {ast.literal_eval(key): (False if ast.literal_eval(key) == 'use_sim_time' else ast.literal_eval(item))
                  for key, item in zip(value.keys, value.values)}
    require(parameters.get('frame_id') == 'rig_link' and parameters.get('odom_frame_id') == 'odom'
            and parameters.get('qos') == 1 and parameters.get('Icp/PointToPlane') == 'true', 'ICP contract changed')
    return parameters, digest(source)


def run_variant(node, executable, variant, frames, parameters, logs, token, report):
    import numpy as np
    import rclpy
    from builtin_interfaces.msg import Time
    from nav_msgs.msg import Odometry
    from rtabmap_msgs.msg import OdomInfo
    from sensor_msgs.msg import PointCloud2
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from wc_fusion.ros_node import cloud_message
    topic = '/wc_private_ablation_'+token+'/'+variant
    odom, info, unexpected = {}, {}, []
    allowed = set()
    def receive(destination, message):
        stamp = message.header.stamp.sec*10**9+message.header.stamp.nanosec
        if stamp not in allowed or stamp in destination:
            unexpected.append(stamp)
        else:
            destination[stamp] = message
    qos = QoSProfile(depth=4, reliability=ReliabilityPolicy.RELIABLE)
    publisher = node.create_publisher(PointCloud2, topic+'/cloud', qos)
    subscriptions = [node.create_subscription(Odometry, topic+'/odom', lambda m: receive(odom, m), qos),
                     node.create_subscription(OdomInfo, topic+'/info', lambda m: receive(info, m), qos)]
    command = [str(executable), '--ros-args', '-r', '__node:=ablation_'+variant, '-r', '__ns:='+topic]
    for key, value in parameters.items():
        rendered = json.dumps(value) if isinstance(value, (str, bool)) else str(value)
        command += ['-p', key+':='+rendered]
    for old, new in [('scan_cloud', 'cloud'), ('scan', 'unused_scan'), ('odom', 'odom'), ('odom_info', 'info'),
                     ('/tf', 'tf'), ('/tf_static', 'tf_static')]:
        command += ['-r', old+':='+topic+'/'+new]
    child, pidfd = None, None
    report.update(command=command, frames=[], status='RUNNING')
    with (logs/(variant+'.log')).open('x') as log:
        try:
            child = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, env=os.environ.copy())
            pidfd = os.pidfd_open(child.pid)
            def wait(predicate, timeout=15):
                end = time.monotonic()+timeout
                while time.monotonic() < end:
                    require(child.poll() is None, 'native ICP exited unexpectedly')
                    require(not unexpected, 'duplicate/unrequested result stamp: '+str(unexpected))
                    if predicate():
                        return
                    rclpy.spin_once(node, timeout_sec=.02)
                raise TimeoutError('native '+variant+' response/discovery timeout')
            wait(lambda: publisher.get_subscription_count() == 1 and node.count_publishers(topic+'/odom') == 1
                 and node.count_publishers(topic+'/info') == 1)
            for index, frame in enumerate(frames):
                stamp = frame['stamp']
                points = frame['sides'][variant]
                message = cloud_message(points, Time(sec=stamp//10**9, nanosec=stamp % 10**9))
                entry = {'stamp_ns': stamp, 'source_point_counts': frame['counts'], 'sent_count': len(points),
                         'sent_sha256': digest(bytes(message.data)), 'matched': False}
                report['frames'].append(entry)
                require(entry['sent_sha256'] == digest(points.tobytes()), 'serialized actual input changed')
                allowed.add(stamp)
                publisher.publish(message)
                wait(lambda: stamp in odom and stamp in info)
                actual, status = odom[stamp], info[stamp]
                pose = actual.pose.pose
                xyz = [pose.position.x, pose.position.y, pose.position.z]
                quaternion = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
                entry.update(matched=True, lost=bool(status.lost), correspondences=int(status.icp_correspondences),
                             icp_inliers_ratio=finite_number(status.icp_inliers_ratio),
                             xyz=[finite_number(v) for v in xyz], quaternion_xyzw=[finite_number(v) for v in quaternion])
                entry['valid'] = bool(not status.lost and np.isfinite(xyz+quaternion).all()
                                      and .999 <= np.linalg.norm(quaternion) <= 1.001
                                      and (index == 0 or status.icp_correspondences >= 20))
                require(actual.header.frame_id == 'odom' and actual.child_frame_id == 'rig_link', 'native output frames differ')
            report['valid_after_initialization'] = sum(entry['valid'] for entry in report['frames'][1:])
            require(report['valid_after_initialization'] >= 10, 'fewer than ten valid matched native results after initialization')
            poses = np.asarray([entry['xyz'] for entry in report['frames'] if all(v is not None for v in entry['xyz'])])
            report.update(status='PASS', matched=len(odom), lost=sum(entry['lost'] for entry in report['frames']),
                          path_length_m=float(np.linalg.norm(np.diff(poses, axis=0), axis=1).sum()),
                          net_displacement_m=float(np.linalg.norm(poses[-1]-poses[0])), accuracy='NOT_ASSESSED_NO_TRUTH')
        finally:
            report['observed_info'] = [{'stamp_ns': stamp, 'lost': bool(value.lost),
                                        'correspondences': int(value.icp_correspondences)} for stamp, value in info.items()]
            report['observed_odom_stamps'] = list(odom)
            if child is not None:
                report['cleanup'] = owned_stop(child, pidfd)
            if pidfd is not None:
                os.close(pidfd)
            for subscription in subscriptions:
                node.destroy_subscription(subscription)
            node.destroy_publisher(publisher)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session-config', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    require(os.uname().machine == 'aarch64' and Path.cwd().resolve() == ROOT, 'run on target in project root')
    config, frames, before = load_inputs(args.session_config)
    parameters, launch_sha = actual_parameters()
    output = args.output.absolute()
    require(output.parent.resolve() == output.parent and output.is_relative_to(ROOT/'reports/ablation')
            and '..' not in output.parts, 'new report must be in project reports/ablation')
    output.parent.mkdir(parents=True, exist_ok=True)
    logs = output.with_suffix('.logs')
    report = {'schema_version': 1, 'status': 'FAIL', 'source_mode': 'synthetic', 'session_id': config['session_id'],
              'domain_id': 87, 'parameters': parameters, 'processing_launch_sha256': launch_sha, 'variants': {},
              'scope': 'NATIVE_ICP_ABLATION_ONLY; no formal single-source mapping, graph or physical validation', 'input_sha256': before}
    with output.open('x') as destination, (output.parent/'domain87.lock').open('a+') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        logs.mkdir(exist_ok=False)
        os.environ.update(ROS_DOMAIN_ID='87', ROS_LOCALHOST_ONLY='1')
        import rclpy
        from ament_index_python.packages import get_package_prefix
        executable = Path(get_package_prefix('rtabmap_odom'))/'lib/rtabmap_odom/icp_odometry'
        require(executable.is_file(), 'actual installed icp_odometry executable missing')
        rclpy.init()
        token = uuid.uuid4().hex[:12]
        node = rclpy.create_node('icp_ablation_observer_'+token)
        try:
            for variant in ('left', 'right', 'dual'):
                end = time.monotonic()+3
                while time.monotonic() < end:
                    rclpy.spin_once(node, timeout_sec=.05)
                require(node.get_node_names() == [node.get_name()], 'domain 87 not empty; refusing shared-domain test')
                result = report['variants'].setdefault(variant, {})
                run_variant(node, executable, variant, frames, parameters, logs, token, result)
            for index, frame in enumerate(frames):
                evidence = [report['variants'][mode]['frames'][index] for mode in ('left', 'right', 'dual')]
                require(len({entry['sent_sha256'] for entry in evidence}) == 3, 'ablation did not alter actual input')
                require(evidence[2]['sent_count'] == evidence[0]['sent_count']+evidence[1]['sent_count'], 'dual is not both inputs')
            report['status'] = 'PASS'
        except Exception as error:
            report['error'] = type(error).__name__+': '+str(error)
        finally:
            report['input_sha256_after'] = {path: digest(Path(path).read_bytes()) if Path(path).is_file() else 'MISSING' for path in before}
            report['raw_files_unchanged'] = before == report['input_sha256_after']
            if not report['raw_files_unchanged']:
                report['status'] = 'FAIL'
            node.destroy_node()
            rclpy.shutdown()
            json.dump(report, destination, indent=2, allow_nan=False)
            destination.flush()
            os.fsync(destination.fileno())
    print(json.dumps({'status': report['status'], 'output': str(output)}))
    return 0 if report['status'] == 'PASS' else 1


if __name__ == '__main__':
    raise SystemExit(main())
