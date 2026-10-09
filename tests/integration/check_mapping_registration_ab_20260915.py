#!/usr/bin/env python3
"""Bounded native ICP experiment on immutable recorded scan/prior pairs; no devices."""
import argparse
from array import array
import copy
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import signal
import sqlite3
import subprocess
import sys
import time

import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation


def digest(path):
    result = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024*1024), b''): result.update(block)
    return result.hexdigest()


def write(path, value):
    with path.open('x', encoding='utf-8') as stream:
        json.dump(value, stream, ensure_ascii=False, allow_nan=False, indent=2)
        stream.write('\n')


def stamp(message):
    return message.header.stamp.sec*10**9+message.header.stamp.nanosec


def matrix(pose):
    p = pose.position if hasattr(pose, 'position') else pose.translation
    q = pose.orientation if hasattr(pose, 'orientation') else pose.rotation
    quaternion = [q.x, q.y, q.z, q.w]
    if not np.isfinite(quaternion).all() or abs(np.linalg.norm(quaternion)-1) > 1e-5:
        return None
    result = np.eye(4)
    result[:3, :3] = Rotation.from_quat(quaternion).as_matrix()
    result[:3, 3] = [p.x, p.y, p.z]
    if not np.isfinite(result).all(): return None
    return result


def pose_delta(a, b):
    delta = np.linalg.inv(a)@b
    return {'translation_m': float(np.linalg.norm(delta[:3, 3])),
            'rotation_deg': float(np.rad2deg(Rotation.from_matrix(delta[:3, :3]).magnitude()))}


def reduced(points, voxel=.03, maximum=20000):
    points = np.asarray(points, dtype=float)
    points = points[np.isfinite(points).all(axis=1)]
    _, indices = np.unique(np.floor(points/voxel).astype(np.int64), axis=0, return_index=True)
    points = points[np.sort(indices)]
    if len(points) > maximum: points = points[np.linspace(0, len(points)-1, maximum, dtype=int)]
    return points


def transformed(points, pose):
    return points@pose[:3, :3].T+pose[:3, 3]


def summary(values):
    values = np.asarray(values, dtype=float)
    values = values[np.isfinite(values)]
    if not len(values): return None
    return {'count': len(values), 'min': float(values.min()), 'median': float(np.median(values)),
            'p95': float(np.quantile(values, .95)), 'max': float(values.max())}


def residual(source, target):
    """Independent nearest-surface diagnostic, not native ICP's objective value."""
    if min(len(source), len(target)) < 21: return {'available': False}
    tree = cKDTree(target)
    _, neighborhoods = tree.query(target, k=20, workers=1)
    groups = target[neighborhoods]
    centered = groups-groups.mean(axis=1, keepdims=True)
    eigenvalues, eigenvectors = np.linalg.eigh(np.einsum('nki,nkj->nij', centered, centered)/20)
    normals = eigenvectors[:, :, 0]
    distances, neighbors = tree.query(source, k=1, workers=1)
    planar = eigenvalues[:, 0]/np.maximum(eigenvalues.sum(axis=1), 1e-12) < .05
    mask = (distances <= .25)&planar[neighbors]
    signed = np.einsum('ij,ij->i', source[mask]-target[neighbors[mask]], normals[neighbors[mask]])
    return {'available': True, 'source_points': len(source), 'target_points': len(target),
            'correspondence_radius_m': .25, 'planarity_eigenvalue_fraction_max': .05,
            'matched_points': int(mask.sum()), 'matched_ratio': float(mask.mean()),
            'nearest_distance_all_m': summary(distances), 'point_to_plane_abs_m': summary(abs(signed)),
            'point_to_plane_signed_m': summary(signed)}


def structure_metrics(records, clouds):
    """Compare identical frame pairs; report overlap alongside residual selection."""
    rows = []
    # A fixed five-frame interval avoids comparing unrelated global surfaces.
    for index in range(5, len(records), 5):
        a, b = records[index-5], records[index]
        if a.get('icp_pose') is None or b.get('icp_pose') is None: continue
        first, second = clouds[a['stamp_ns']], clouds[b['stamp_ns']]
        baseline_relative = np.linalg.inv(a['baseline_pose'])@np.asarray(b['baseline_pose'])
        icp_relative = np.linalg.inv(a['icp_pose'])@np.asarray(b['icp_pose'])
        rows.append({'first_stamp_ns': a['stamp_ns'], 'second_stamp_ns': b['stamp_ns'],
                     'interval_s': (b['stamp_ns']-a['stamp_ns'])*1e-9,
                     'baseline': residual(transformed(second, baseline_relative), first),
                     'icp': residual(transformed(second, icp_relative), first)})
    # Long-baseline repetitions of the first static view, never use an inferred
    # floor at world origin as a supposed step-height measurement.
    static = []
    if records:
        first = records[0]
        for row in records[1:]:
            delta = pose_delta(np.asarray(first['baseline_pose']), np.asarray(row['baseline_pose']))
            if delta['translation_m'] > .02 or delta['rotation_deg'] > 1: break
            if row.get('icp_pose') is None or first.get('icp_pose') is None: continue
            if row['index'] % 5: continue
            static.append({'stamp_ns': row['stamp_ns'], 'baseline_motion': delta,
                'baseline': residual(transformed(clouds[row['stamp_ns']], np.asarray(row['baseline_pose'])), clouds[first['stamp_ns']]),
                'icp': residual(transformed(clouds[row['stamp_ns']], np.asarray(row['icp_pose'])), clouds[first['stamp_ns']])})
    return {'five_frame_pairs': rows, 'initial_static_candidate_repetitions': static,
            'limitation': 'Nearest-surface overlap diagnostic, not surveyed thickness; small residual with low overlap is not improvement.'}


def load_pairs(source, start_s, count):
    from rclpy.serialization import deserialize_message
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import PointCloud2
    priors = {}
    bags = sorted((source/'bag').glob('*.db3'))
    signatures = {str(p): [p.stat().st_size, p.stat().st_mtime_ns] for p in bags}
    for path in bags:
        with sqlite3.connect(path.as_uri()+'?mode=ro', uri=True) as db:
            for raw, in db.execute('SELECT m.data FROM messages m JOIN topics t ON t.id=m.topic_id WHERE t.name=? ORDER BY m.timestamp,m.id', ('/wc_mapping/app/prior_odom',)):
                message = deserialize_message(raw, Odometry)
                key = stamp(message)
                if key in priors: raise ValueError('Duplicate recorded prior timestamp')
                if message.header.frame_id != 'prior_odom' or message.child_frame_id != 'mapping_reference' or matrix(message.pose.pose) is None:
                    raise ValueError('Invalid recorded prior identity/pose')
                priors[key] = (message, hashlib.sha256(raw).hexdigest())
    beginning = min(priors)
    selected = [key for key in sorted(priors) if (key-beginning)*1e-9 >= start_s][:count]
    if len(selected) != count: raise ValueError('Insufficient consecutive recorded priors')
    needed = set(selected); clouds = {}
    for path in bags:
        with sqlite3.connect(path.as_uri()+'?mode=ro', uri=True) as db:
            for raw, in db.execute('SELECT m.data FROM messages m JOIN topics t ON t.id=m.topic_id WHERE t.name=? ORDER BY m.timestamp,m.id', ('/wc_mapping/app/scan_cloud',)):
                message = deserialize_message(raw, PointCloud2)
                key = stamp(message)
                if key not in needed: continue
                if key in clouds or message.header.frame_id != 'mapping_reference': raise ValueError('Cloud duplicate/frame mismatch')
                clouds[key] = (message, hashlib.sha256(raw).hexdigest())
    if set(clouds) != needed: raise ValueError('Selected prior/cloud timestamps do not completely match')
    if max(np.diff(selected))*1e-9 > 1: raise ValueError('Selected frames contain >1s input gap; do not hide missing overlap')
    return selected, priors, clouds, signatures, beginning


def source_raw_clouds(source, selected, clouds, config, root):
    """Bind same source frame, then apply the frozen rigid rotation to raw XYZ."""
    from rclpy.serialization import deserialize_message, serialize_message
    from wc_interfaces.msg import SourceFrame
    from wc_sensors.pointcloud import decode_pointcloud2, validate_pointcloud2
    from wc_runtime.mapping_prior import _field_values
    if config['mode'] not in ('left', 'right'): raise ValueError('Raw experiment requires one explicit lidar')
    path = root/'tests/integration/check_mapping_filter_ab_20260915.py'
    spec = importlib.util.spec_from_file_location('registration_source_pair_contract', path)
    module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    side = config['mode']; needed = set(selected); sources = {'raw': {}, 'filtered': {}}
    for bag in sorted((source/'bag').glob('*.db3')):
        with sqlite3.connect(bag.as_uri()+'?mode=ro', uri=True) as db:
            for variant, suffix in [('raw', ''), ('filtered', '_filtered')]:
                topic = '/wc_mapping/lidar_'+side+'/source_frame'+suffix
                for raw, in db.execute('SELECT m.data FROM messages m JOIN topics t ON t.id=m.topic_id WHERE t.name=? ORDER BY m.timestamp,m.id', (topic,)):
                    message = deserialize_message(raw, SourceFrame); key = stamp(message)
                    if key not in needed: continue
                    if key in sources[variant]: raise ValueError('Duplicate original source stamp')
                    sources[variant][key] = (message, hashlib.sha256(raw).hexdigest())
    if any(set(rows) != needed for rows in sources.values()): raise ValueError('Selected raw/filtered source pair missing')
    rotation = np.asarray(json.loads((source/'prior/initialization.json').read_text())['R_reference_base'])
    def rotate(message, reference_header):
        result = copy.deepcopy(message); layout = validate_pointcloud2(message)
        points = decode_pointcloud2(message); finite = np.isfinite(points).all(axis=1)
        points[finite] = points[finite]@rotation.T
        storage = bytearray(message.data)
        for axis, name in enumerate(('x', 'y', 'z')):
            view = _field_values(result, layout, name, storage)
            values = view.copy().reshape(-1); values[finite] = points[finite, axis]
            view[:] = values.reshape(view.shape)
        result.data = array('B', storage); result.header = copy.deepcopy(reference_header)
        return result
    output = {}; bindings = []
    for key in selected:
        raw, raw_hash = sources['raw'][key]; filtered, filtered_hash = sources['filtered'][key]
        binding = module.verify_pair(raw, filtered, source.name)
        if raw.side != side or raw.sensor_id != config['sensor_ids'][side]: raise ValueError('Source sensor/config mismatch')
        reference = clouds[key][0]; check = rotate(filtered.cloud, reference.header)
        if bytes(check.data) != bytes(reference.data): raise ValueError('Frozen rotated filtered bytes differ from recorded scan_cloud')
        candidate = rotate(raw.cloud, reference.header)
        candidate_hash = hashlib.sha256(serialize_message(candidate)).hexdigest()
        output[key] = (candidate, candidate_hash)
        raw_xyz = decode_pointcloud2(raw.cloud); filtered_xyz = decode_pointcloud2(filtered.cloud)
        common = np.isfinite(raw_xyz).all(axis=1)&np.isfinite(filtered_xyz).all(axis=1)
        distance = np.linalg.norm(raw_xyz, axis=1)
        bounded = common&(distance >= .3)&(distance < 8)
        edges = module.boundary_mask(raw_xyz, common)
        displacement = np.linalg.norm(filtered_xyz-raw_xyz, axis=1)
        distribution = {'raw_finite': int(np.isfinite(raw_xyz).all(axis=1).sum()),
            'common_finite': int(common.sum()), 'bounded_common_0p3_to_8m': int(bounded.sum()),
            'raw_radial_distance_m': summary(distance), 'raw_filtered_displacement_m': summary(displacement[bounded])}
        if edges is not None:
            boundary = bounded&edges; interior = bounded&~edges
            distribution.update(boundary_points=int(boundary.sum()), interior_points=int(interior.sum()),
                boundary_fraction_of_bounded_common=float(boundary.sum()/max(1, bounded.sum())),
                boundary_displacement_m=summary(displacement[boundary]), interior_displacement_m=summary(displacement[interior]))
        bindings.append({**binding, 'stamp_ns': key, 'raw_source_cdr_sha256': raw_hash,
            'filtered_source_cdr_sha256': filtered_hash, 'recorded_filtered_scan_cdr_sha256': clouds[key][1],
            'published_rotated_raw_scan_cdr_sha256': candidate_hash, 'filtered_transform_matches_recorded_bytes': True,
            'input_distribution': distribution})
    return output, {'pair_verifier_sha256': digest(path), 'R_reference_base': rotation.tolist(), 'frames': bindings}


def guess_audit(records):
    previous = None; translation = []; angle = []
    for row in records:
        if previous is not None and row['native_guess'] is not None:
            expected = np.linalg.inv(previous)@np.asarray(row['baseline_pose'])
            error = pose_delta(expected, np.asarray(row['native_guess']))
            translation.append(error['translation_m']); angle.append(error['rotation_deg'])
        if not row['lost']: previous = np.asarray(row['baseline_pose'])
    return {'reference': 'inverse(last successful prior pose) * current prior pose; includes accumulated guesses on loss',
            'translation_error_m': summary(translation), 'rotation_error_deg': summary(angle)}


def restrict_range(clouds, minimum, maximum):
    """Diagnostic copied messages only: retain layout, mark excluded XYZ invalid."""
    from rclpy.serialization import serialize_message
    from wc_runtime.mapping_prior import _field_values
    from wc_sensors.pointcloud import decode_pointcloud2, validate_pointcloud2
    output = {}; counts = []
    for key, (message, checksum) in clouds.items():
        points = decode_pointcloud2(message); distance = np.linalg.norm(points, axis=1)
        finite = np.isfinite(points).all(axis=1)
        excluded = finite&((distance < minimum)|(distance > maximum))
        result = copy.deepcopy(message); layout = validate_pointcloud2(message); storage = bytearray(message.data)
        for name in ('x', 'y', 'z'):
            view = _field_values(result, layout, name, storage)
            values = view.copy().reshape(-1); values[excluded] = np.nan; view[:] = values.reshape(view.shape)
        result.data = array('B', storage); result.is_dense = False
        after = hashlib.sha256(serialize_message(result)).hexdigest()
        output[key] = (result, after)
        counts.append({'stamp_ns': key, 'before_cloud_cdr_sha256': checksum, 'after_cloud_cdr_sha256': after,
                       'finite_before': int(finite.sum()), 'excluded': int(excluded.sum()),
                       'finite_after': int((finite&~excluded).sum())})
    return output, counts


def direction_sweep(first, second, relative_pose):
    """Small relative-yaw sign hypothesis with common source support and gates."""
    first = reduced(first)
    second = np.asarray(second, dtype=float)
    second = second[np.isfinite(second).all(axis=1)]
    tree = cKDTree(first)
    _, neighbors = tree.query(first, k=20, workers=1)
    groups = first[neighbors]; groups -= groups.mean(axis=1, keepdims=True)
    eigenvalues, eigenvectors = np.linalg.eigh(np.einsum('nki,nkj->nij', groups, groups)/20)
    normals = eigenvectors[:, :, 0]
    planar = eigenvalues[:, 0]/np.maximum(eigenvalues.sum(axis=1), 1e-12) < .05
    original_rpy = Rotation.from_matrix(relative_pose[:3, :3]).as_euler('xyz')
    candidates = []; common = np.ones(len(second), dtype=bool)
    for factor in (-1.5, -1., -.5, 0., .5, 1., 1.5):
        rpy = original_rpy.copy(); rpy[2] *= factor
        pose = relative_pose.copy(); pose[:3, :3] = Rotation.from_euler('xyz', rpy).as_matrix()
        points = transformed(second, pose); distance, match = tree.query(points, k=1, workers=1)
        gate = (distance <= .25)&planar[match]
        common &= gate
        candidates.append((factor, points, distance, match, gate))
    baseline = next(row for row in candidates if row[0] == 1.)
    fixed_match = baseline[3][common]
    rows = []
    for factor, points, distance, match, gate in candidates:
        nearest_signed = np.einsum('ij,ij->i', points[common]-first[match[common]], normals[match[common]])
        fixed_delta = points[common]-first[fixed_match]
        fixed_signed = np.einsum('ij,ij->i', fixed_delta, normals[fixed_match])
        rows.append({'yaw_factor': factor, 'hypothesis_relative_yaw_deg': float(np.rad2deg(original_rpy[2]*factor)),
            'hypothesis_matched_fraction_before_common_mask': float(gate.mean()),
            'common_nearest_point_to_point_m': summary(distance[common]),
            'common_nearest_point_to_plane_abs_m': summary(abs(nearest_signed)),
            'fixed_baseline_matches_point_to_point_m': summary(np.linalg.norm(fixed_delta, axis=1)),
            'fixed_baseline_matches_point_to_plane_abs_m': summary(abs(fixed_signed))})
    return {'original_relative_rpy_deg': np.rad2deg(original_rpy).tolist(),
        'unchanged_relative_translation_m': relative_pose[:3, 3].tolist(),
        'source_finite_points': len(second), 'common_source_points': int(common.sum()),
        'common_source_fraction': float(common.mean()), 'target_voxel_m': .03, 'normal_k': 20,
        'correspondence_radius_m': .25, 'planarity_eigenvalue_fraction_max': .05, 'curve': rows,
        'interpretation': 'Only relative Euler yaw changed; translation/roll/pitch fixed. Common source intersection across all hypotheses; fixed baseline matches favor original associations.'}


def main():
    import fcntl
    import yaml
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import PointCloud2
    from geometry_msgs.msg import TransformStamped
    from tf2_msgs.msg import TFMessage
    from rosgraph_msgs.msg import Clock
    from rtabmap_msgs.msg import OdomInfo
    from wc_runtime.cli import ROOT, target
    from wc_runtime.prepare_picker_input import project_path
    from wc_sensors.pointcloud import decode_pointcloud2
    target()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session-root', type=Path, required=True)
    parser.add_argument('--run-name', required=True)
    parser.add_argument('--start-s', type=float, default=10.)
    parser.add_argument('--count', type=int, default=100)
    parser.add_argument('--prior-jsonl', type=Path)
    parser.add_argument('--cloud-variant', choices=('recorded_filtered', 'source_raw'), default='recorded_filtered')
    parser.add_argument('--diagnostic-range-min-m', type=float)
    parser.add_argument('--diagnostic-range-max-m', type=float)
    args = parser.parse_args()
    if not args.run_name.startswith('registration_') or not args.run_name.replace('_', '').isalnum():
        raise ValueError('Use a new registration_ identifier')
    if not 1 <= args.count <= 100 or not 0 <= args.start_s < 600: raise ValueError('Invalid bounded frame selection')
    range_filter = args.diagnostic_range_min_m is not None or args.diagnostic_range_max_m is not None
    if range_filter and (args.diagnostic_range_min_m is None or args.diagnostic_range_max_m is None or
                        not 0 <= args.diagnostic_range_min_m < args.diagnostic_range_max_m <= 10):
        raise ValueError('Diagnostic range requires two finite bounds between 0 and 10m')
    if os.environ.get('ROS_DOMAIN_ID') != '87' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise RuntimeError('Requires isolated localhost domain 87')
    source = project_path(ROOT, args.session_root)
    out = project_path(ROOT, ROOT/'reports/map_geometry_fix_20260915'/args.run_name)
    out.mkdir(exist_ok=False)
    lock = project_path(ROOT, ROOT/'.phase1_runtime/locks/domain-87-geometry-review.lock').open('a')
    fcntl.flock(lock, fcntl.LOCK_EX|fcntl.LOCK_NB)
    metadata = {}
    for name in ('session.json', 'runtime_config.json', 'hardware_setup.json', 'wheel_candidate.json', 'prior/initialization.json'):
        path = project_path(ROOT, source/name)
        raw = path.read_bytes(); metadata[name] = hashlib.sha256(raw).hexdigest()
        destination = out/'frozen_source'/name; destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_bytes(raw)
    config = json.loads((out/'frozen_source/runtime_config.json').read_text())
    if config.get('session_id') != source.name or config.get('source_mode') != 'real': raise ValueError('Source session identity mismatch')
    baseline_manifest = project_path(ROOT, ROOT/'reports/map_geometry_fix_20260915/baseline/source_manifest.json')
    if not baseline_manifest.is_file(): raise ValueError('Root shared input SHA baseline missing')
    (out/'shared_input_manifest.json').write_bytes(baseline_manifest.read_bytes())
    baseline = json.loads(baseline_manifest.read_text())['source_file_manifest']
    for name, checksum in metadata.items():
        if name in baseline and baseline[name]['sha256'] != checksum: raise ValueError('Metadata differs from root shared baseline')
    selected, priors, clouds, signatures, beginning = load_pairs(source, args.start_s, args.count)
    for path, signature in signatures.items():
        item = baseline[str(Path(path).relative_to(source))]
        if signature != [item['bytes'], item['mtime_ns']]: raise ValueError('Bag stat differs from root shared baseline')
    raw_bindings = None
    if args.cloud_variant == 'source_raw':
        clouds, raw_bindings = source_raw_clouds(source, selected, clouds, config, ROOT)
        write(out/'raw_source_bindings.json', raw_bindings)
    if range_filter:
        clouds, range_counts = restrict_range(clouds, args.diagnostic_range_min_m, args.diagnostic_range_max_m)
        write(out/'diagnostic_range_counts.json', range_counts)
    poses = {key: matrix(priors[key][0].pose.pose) for key in selected}
    override = None
    if args.prior_jsonl:
        path = project_path(ROOT, args.prior_jsonl)
        raw = path.read_bytes(); override = {'path': str(path), 'sha256': hashlib.sha256(raw).hexdigest()}
        improved = {}
        for line in raw.decode().splitlines():
            row = json.loads(line); key = row['stamp_ns']; pose = np.asarray(row['T_prior_reference'], dtype=float)
            if key in improved or pose.shape != (4, 4) or not np.isfinite(pose).all() or not np.allclose(pose[3], [0, 0, 0, 1]):
                raise ValueError('Invalid improved prior')
            if not np.allclose(pose[:3, :3].T@pose[:3, :3], np.eye(3), atol=1e-6) or np.linalg.det(pose[:3, :3]) < .999:
                raise ValueError('Improved prior rotation is invalid')
            improved[key] = pose
        if any(key not in improved for key in selected): raise ValueError('Improved prior lacks selected timestamps')
        poses = {key: improved[key] for key in selected}
        (out/'improved_prior.jsonl').write_bytes(raw)
    initial_inverse = np.linalg.inv(poses[selected[0]])
    baseline_poses = {key: initial_inverse@poses[key] for key in selected}
    (out/'slam').mkdir()  # build_spec validates a fresh destination; no SLAM process is launched.
    launch_path = ROOT/'src/wc_bringup/launch/mapping_app.launch.py'
    spec = importlib.util.spec_from_file_location('registration_native_spec', launch_path)
    launch = importlib.util.module_from_spec(spec); spec.loader.exec_module(launch)
    native = launch.build_spec(out, odometry_source='icp')['nodes'][0]
    assert native['executable'] == 'icp_odometry'
    params = copy.deepcopy(native['parameters'][0]); params['use_sim_time'] = True
    parameter_path = out/'native_params.yaml'
    parameter_path.write_text(yaml.safe_dump({'/**': {'ros__parameters': params}}, sort_keys=False))
    command = [sys.executable, '-s', '-m', 'wc_runtime.component', '--parent', str(os.getpid()), '--sigint-grace-s', '10', '--',
        '/opt/ros/humble/lib/rtabmap_odom/icp_odometry', '--ros-args', '--params-file', str(parameter_path),
        '-r', '__node:=geometry_icp_registration', '-r', '__ns:=/wc_mapping/app', '--log-level', 'warn']
    for a, b in native['remappings']: command += ['-r', a+':='+b]
    report = {'status': 'RUNNING', 'validation_level': 'REAL_BAG_NATIVE_ICP_AB', 'source_session': source.name,
        'source_root': str(source), 'source_metadata_sha256': metadata, 'source_bag_stat_before': signatures,
        'shared_input_manifest_sha256': digest(baseline_manifest), 'source_launch_sha256': digest(launch_path),
        'experiment_script_sha256': digest(Path(__file__)), 'cloud_variant': args.cloud_variant,
        'diagnostic_range_m': [args.diagnostic_range_min_m, args.diagnostic_range_max_m] if range_filter else None,
        'hardware_started': False, 'control_commands_sent': False, 'gui_started': False, 'slam_started': False,
        'domain': 87, 'localhost_only': True, 'original_timestamps_preserved': True,
        'input_cloud': ('Recorded scan_cloud: host-filtered and fixed-reference transformed; not raw sensor cloud.'
            if raw_bindings is None else 'Same-stamp SourceFrame raw.cloud before host filtering, verified against filtered identity/hash and rotated by frozen reference.'),
        'native_parameters': params, 'native_command': command, 'source_covariance_weights': config['wheel_imu_covariance'],
        'experimental_changes': ['Explicit ICP branch instead of wheel_imu authority', 'use_sim_time with original scan stamps',
            'Original prior relative to selected first frame used consistently for prior TF and baseline comparison'],
        'prior_override': override, 'first_selected_source_time_s': (selected[0]-beginning)*1e-9,
        'selected_span_s': (selected[-1]-selected[0])*1e-9, 'selected_count': len(selected), 'records': [],
        'limitations': ['No motion or plane-angle ground truth', 'No loop closure or graph optimization',
            'Native correspondence thresholds unchanged; original five-consecutive-loss stop retained',
            'Independent residuals are overlap diagnostics, not native ICP objective values or surveyed thickness']}
    write(out/'selection.json', [{'stamp_ns': k, 'cloud_sha256': clouds[k][1], 'prior_sha256': priors[k][1]} for k in selected])
    rclpy.init(args=[]); node = rclpy.create_node('geometry_registration_replay')
    child = None; log = None
    native_info = {}; native_odom = {}; xyz = {}
    try:
        for _ in range(20): rclpy.spin_once(node, timeout_sec=.05)
        foreign = [(n, ns) for n, ns in node.get_node_names_and_namespaces() if n != node.get_name()]
        if foreign: raise RuntimeError('Domain 87 occupied: '+repr(foreign))
        qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE)
        cloud_pub = node.create_publisher(PointCloud2, '/wc_mapping/app/scan_cloud', qos)
        tf_pub = node.create_publisher(TFMessage, '/wc_mapping/app/tf', qos)
        clock_pub = node.create_publisher(Clock, '/clock', qos)
        subscriptions = [node.create_subscription(OdomInfo, '/wc_mapping/app/odom_info', lambda m: native_info.__setitem__(stamp(m), m), qos),
                         node.create_subscription(Odometry, '/wc_mapping/app/odom', lambda m: native_odom.__setitem__(stamp(m), m), qos)]
        log = (out/'native.log').open('x')
        child = subprocess.Popen(command, stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        deadline = time.monotonic()+20
        while time.monotonic() < deadline:
            if child.poll() is not None: raise RuntimeError('Native ICP exited during startup')
            clock_pub.publish(Clock(clock=clouds[selected[0]][0].header.stamp))
            rclpy.spin_once(node, timeout_sec=.05)
            if cloud_pub.get_subscription_count() and tf_pub.get_subscription_count() and node.count_publishers('/wc_mapping/app/odom_info'): break
        else: raise RuntimeError('Native ICP subscriptions not ready')
        began = time.monotonic(); consecutive_lost = 0
        for index, key in enumerate(selected):
            if time.monotonic()-began > 480: raise RuntimeError('Bounded replay deadline exceeded')
            cloud = clouds[key][0]; pose = baseline_poses[key]
            transform = TransformStamped(); transform.header = copy.deepcopy(cloud.header)
            transform.header.frame_id = 'prior_odom'; transform.child_frame_id = 'mapping_reference'
            transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z = map(float, pose[:3, 3])
            q = Rotation.from_matrix(pose[:3, :3]).as_quat()
            transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w = map(float, q)
            clock_pub.publish(Clock(clock=cloud.header.stamp)); tf_pub.publish(TFMessage(transforms=[transform]))
            until = time.monotonic()+.04
            while time.monotonic() < until: rclpy.spin_once(node, timeout_sec=.005)
            sent = time.monotonic(); cloud_pub.publish(cloud)
            deadline = sent+5
            while key not in native_info or (not native_info[key].lost and key not in native_odom):
                if child.poll() is not None: raise RuntimeError('Native ICP exited during replay')
                if time.monotonic() >= deadline: raise RuntimeError('No matching native output for stamp '+str(key))
                rclpy.spin_once(node, timeout_sec=.02)
            info = native_info[key]; odom = native_odom.get(key)
            result_pose = matrix(odom.pose.pose) if odom is not None and not info.lost else None
            if not info.lost and result_pose is None: raise RuntimeError('Native successful odom pose malformed')
            points = decode_pointcloud2(cloud); finite = np.isfinite(points).all(axis=1)
            ranges = np.linalg.norm(points[finite], axis=1)
            row = {'index': index, 'stamp_ns': key, 'source_time_s': (key-beginning)*1e-9,
                'source_points': cloud.width*cloud.height, 'lost': bool(info.lost),
                'finite_points': int(finite.sum()), 'input_range_m': summary(ranges),
                'input_fraction_beyond_6m': float(np.mean(ranges > 6)),
                'input_fraction_beyond_8m': float(np.mean(ranges > 8)),
                'wall_response_s': time.monotonic()-sent, 'native_time_estimation_s': float(info.time_estimation),
                'native_interval_s': float(info.interval), 'native_icp_inliers_ratio': float(info.icp_inliers_ratio),
                'native_icp_correspondences': int(info.icp_correspondences),
                'native_icp_translation_m': float(info.icp_translation), 'native_icp_rotation_rad': float(info.icp_rotation),
                'native_structural_complexity': float(info.icp_structural_complexity),
                'native_structural_distribution': float(info.icp_structural_distribution),
                'native_guess': matrix(info.guess).tolist() if matrix(info.guess) is not None else None,
                'native_covariance': list(info.covariance), 'baseline_pose': pose.tolist(),
                'icp_pose': result_pose.tolist() if result_pose is not None else None,
                'cumulative_pose_correction': pose_delta(pose, result_pose) if result_pose is not None else None}
            report['records'].append(row); xyz[key] = reduced(points)
            consecutive_lost = consecutive_lost+1 if info.lost else 0
            if index % 10 == 0: print(json.dumps({'index': index, 'source_time_s': row['source_time_s'], 'lost': bool(info.lost)}), flush=True)
            if consecutive_lost >= 5:
                report['status'] = 'FAILED_NATIVE_TRACKING'; report['stop_reason'] = 'ICP_CONSECUTIVE_FIVE_LOST'; break
        else: report['status'] = 'COMPLETED_REQUIRES_GEOMETRY_REVIEW'
    except Exception as error:
        report['status'] = 'FAILED_EXPERIMENT'; report['error'] = repr(error)
    finally:
        if child is not None:
            if child.poll() is None: child.send_signal(signal.SIGINT)
            try: child.wait(timeout=20)
            except subprocess.TimeoutExpired:
                report['status'] = 'FAILED_CLEANUP'; report['unreaped_owner_pid'] = child.pid
            report['native_exit_code'] = child.returncode
        node.destroy_node(); rclpy.shutdown()
        if log: log.close()
        lock.close()
    report['processed_count'] = len(report['records'])
    report['native_guess_audit'] = guess_audit(report['records'])
    report['lost_count'] = sum(row['lost'] for row in report['records'])
    report['response_time_s'] = summary([row['wall_response_s'] for row in report['records']])
    report['native_inlier_ratio'] = summary([row['native_icp_inliers_ratio'] for row in report['records'] if not row['lost']])
    report['cumulative_correction_translation_m'] = summary([row['cumulative_pose_correction']['translation_m'] for row in report['records'] if row['cumulative_pose_correction']])
    report['cumulative_correction_rotation_deg'] = summary([row['cumulative_pose_correction']['rotation_deg'] for row in report['records'] if row['cumulative_pose_correction']])
    report['structure_metrics'] = structure_metrics(report['records'], xyz)
    report['source_bag_stat_after'] = {path: [Path(path).stat().st_size, Path(path).stat().st_mtime_ns] for path in signatures}
    report['source_bag_stat_unchanged'] = report['source_bag_stat_after'] == signatures
    report['source_metadata_sha256_after'] = {name: digest(source/name) for name in metadata}
    report['source_metadata_unchanged'] = report['source_metadata_sha256_after'] == metadata
    if not report['source_bag_stat_unchanged'] or not report['source_metadata_unchanged']:
        report['status'] = 'FAILED_SOURCE_INTEGRITY'
    write(out/'result.json', report)
    print(json.dumps({k: report[k] for k in ('status', 'processed_count', 'lost_count', 'native_inlier_ratio',
          'cumulative_correction_translation_m', 'cumulative_correction_rotation_deg', 'response_time_s')}, allow_nan=False), flush=True)
    return 0 if report['status'] == 'COMPLETED_REQUIRES_GEOMETRY_REVIEW' else 2


if __name__ == '__main__':
    sys.exit(main())
