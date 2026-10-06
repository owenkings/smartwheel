#!/usr/bin/env python3
"""Replay immutable source packets through the production input/prior, without ROS nodes.

Four cells isolate motion-model and cloud-filter changes. This exports frontend
CDR and poses, not an RTAB-Map result or an accuracy acceptance. ROS is imported
only for message classes/serialization; rclpy.init, publishers and devices are
never used. Run on the verified target only under the root agent's scheduling.
"""
import argparse
from collections import Counter, deque
from contextlib import ExitStack
import copy
import hashlib
import json
import os
from pathlib import Path
import sqlite3
import sys
import time

for _thread_variable in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ[_thread_variable] = '1'

import numpy as np
from scipy.spatial.transform import Rotation


CELLS = (('se3_no_filter', 'se3_gyro', False),
         ('se3_filter', 'se3_gyro', True),
         ('planar_no_filter', 'planar_ekf', False),
         ('planar_filter', 'planar_ekf', True))
ORDER = {'wheel': 0, 'imu': 1, 'source': 2}


def require(condition, message):
    if not condition:
        raise ValueError(message)


def digest(path):
    result = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            result.update(block)
    return result.hexdigest()


def write_json(path, value):
    with Path(path).open('x', encoding='utf-8') as stream:
        json.dump(value, stream, ensure_ascii=False, allow_nan=False, indent=2)
        stream.write('\n')


def append_json(stream, value):
    stream.write(json.dumps(value, ensure_ascii=False, allow_nan=False, separators=(',', ':')) + '\n')


def stamp(value):
    return int(value.sec) * 10**9 + int(value.nanosec)


def stats(values):
    a = np.asarray(values, dtype=float)
    if not a.size:
        return None
    require(np.isfinite(a).all(), 'Non-finite diagnostic statistic')
    return {'count': len(a), 'min': float(a.min()), 'median': float(np.median(a)),
            'p05': float(np.quantile(a, .05)), 'p95': float(np.quantile(a, .95)), 'max': float(a.max())}


def signatures(paths):
    return {str(p): {'bytes': p.stat().st_size, 'mtime_ns': p.stat().st_mtime_ns} for p in paths}


def checked_output(root, source, output):
    path = output if output.is_absolute() else root / output
    require('..' not in path.parts, 'Output must not contain parent traversal')
    for parent in (path, *path.parents):
        require(not parent.is_symlink(), 'Output must not follow symlinks')
    path = path.absolute()
    require(path.is_relative_to(root) and path != root, 'Output must be a new project subdirectory')
    require(not path.is_relative_to(source) and not source.is_relative_to(path), 'Output overlaps original session')
    require(not path.exists(), 'Output already exists; choose a new experiment directory')
    return path


class RecordedPackets:
    """Keep bounded indexes in memory; source CDR stays in read-only SQLite."""
    def __init__(self, source, config, *, max_events=1_000_000):
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
        self.source, self.decode = source, deserialize_message
        self.connections, self.types, self.events = {}, {}, []
        self.stack = ExitStack()
        self.paths = sorted((source / 'bag').glob('*.db3'))
        require(self.paths, 'Original session has no SQLite bag parts')
        self.before = signatures(self.paths)
        self.recorded_priors = {}
        self.counts = Counter()
        self.sides = ('left', 'right') if config['mode'] == 'all' else (config['mode'],)
        suffix = '' if config.get('cloud_source', 'filtered') == 'raw' else '_filtered'
        topics = {config['prior_template']['wheel_topic']: 'wheel',
                  config['prior_template']['imu_topic']: 'imu'}
        topics.update({'/wc_mapping/lidar_' + side + '/source_frame' + suffix: 'source' for side in self.sides})
        try:
            for path in self.paths:
                db = sqlite3.connect(path.as_uri() + '?mode=ro', uri=True)
                self.stack.callback(db.close)
                db.execute('PRAGMA query_only=ON')
                self.connections[path.name] = db
                definitions = {i: (name, kind) for i, name, kind in db.execute('SELECT id,name,type FROM topics')}
                for topic_id, (topic, kind) in definitions.items():
                    if topic not in topics and topic != '/wc_mapping/app/prior_odom':
                        continue
                    expected = {'source': 'wc_interfaces/msg/SourceFrame',
                                'imu': 'wc_interfaces/msg/H30Frame', 'wheel': 'std_msgs/msg/String'}
                    category = topics.get(topic, 'recorded_prior')
                    require(kind == expected.get(category, 'nav_msgs/msg/Odometry'), 'Unexpected original message type: ' + topic)
                    self.types[kind] = get_message(kind)
                    for message_id, recorded, cdr in db.execute(
                            'SELECT id,timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp,id', (topic_id,)):
                        message = self.decode(cdr, self.types[kind])
                        self.counts[topic] += 1
                        if category == 'recorded_prior':
                            key = stamp(message.header.stamp)
                            require(key not in self.recorded_priors, 'Duplicate recorded prior timestamp')
                            p, q = message.pose.pose.position, message.pose.pose.orientation
                            require(message.header.frame_id == 'prior_odom' and message.child_frame_id == 'mapping_reference',
                                    'Unexpected recorded prior frames')
                            pose = np.eye(4)
                            pose[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
                            pose[:3, 3] = [p.x, p.y, p.z]
                            self.recorded_priors[key] = pose
                            continue
                        if category == 'wheel':
                            data = json.loads(message.data)
                            key, mono, sequence = data['stamp_ns'], data['receive_monotonic_ns'], data['sequence']
                        else:
                            key, mono, sequence = stamp(message.host_receive_time), int(message.host_monotonic_ns), int(message.frame_sequence)
                        self.events.append({'stamp_ns': key, 'monotonic_ns': mono, 'sequence': sequence,
                            'category': category, 'side': getattr(message, 'side', ''), 'topic': topic, 'type': kind,
                            'bag': path.name, 'message_id': message_id, 'bag_received_ns': recorded,
                            'cdr_sha256': hashlib.sha256(cdr).hexdigest()})
                        require(len(self.events) <= max_events, 'Diagnostic event-index capacity exceeded; do not silently truncate')
            self.events.sort(key=lambda row: (row['stamp_ns'], ORDER[row['category']], row['sequence'],
                                               row['side'], row['bag'], row['message_id']))
            require(all(any(row['category'] == kind for row in self.events) for kind in ORDER), 'Source/IMU/wheel packets required')
            require({row['side'] for row in self.events if row['category'] == 'source'} == set(self.sides),
                    'All requested lidar sides must be present; no single-side substitution')
        except BaseException:
            self.close()
            raise

    def get(self, entry):
        row = self.connections[entry['bag']].execute('SELECT data FROM messages WHERE id=?', (entry['message_id'],)).fetchone()
        require(row is not None and hashlib.sha256(row[0]).hexdigest() == entry['cdr_sha256'], 'Original CDR changed during replay')
        return self.decode(row[0], self.types[entry['type']])

    def close(self):
        self.stack.close()


def settle(owner, now_ns, predicate):
    deadline = time.monotonic() + 30.
    while True:
        owner.poll_io(now_ns, clock=lambda: now_ns)
        owner.ensure_active()
        if predicate():
            return
        if time.monotonic() >= deadline:
            raise TimeoutError('Offline archive did not drain in 30s; diagnostic failed, no gate relaxed')
        time.sleep(.001)


def authoritative_odometry(cloud, pose, report, prior):
    """Serialize production pose/twist; use the actual authority adapter.

    This mirrors only the ROS field assignment in mapping_prior.main, not an
    estimator or covariance calculation. The experiment does not create nodes.
    """
    from nav_msgs.msg import Odometry
    from wc_runtime.mapping_odometry import adapt_prior_odometry
    result = Odometry()
    result.header.stamp = copy.deepcopy(cloud.header.stamp)
    result.header.frame_id = prior.config['guess_frame_id']
    result.child_frame_id = prior.config['reference_frame']
    p, q = pose[:3, 3], Rotation.from_matrix(pose[:3, :3]).as_quat()
    result.pose.pose.position.x, result.pose.pose.position.y, result.pose.pose.position.z = map(float, p)
    result.pose.pose.orientation.x, result.pose.pose.orientation.y, result.pose.pose.orientation.z, result.pose.pose.orientation.w = map(float, q)
    result.twist.twist.linear.x, result.twist.twist.linear.y, result.twist.twist.linear.z = report['linear_velocity_reference_m_s']
    result.twist.twist.angular.x, result.twist.twist.angular.y, result.twist.twist.angular.z = report['angular_velocity_reference_rad_s']
    for index in (0, 7, 14, 21, 28, 35):
        result.pose.covariance[index] = result.twist.covariance[index] = 1e6
    result = adapt_prior_odometry(result, prior.config['wheel_imu_covariance'])
    if prior.planar:
        result.pose.covariance = report['pose_covariance']
        result.twist.covariance = report['twist_covariance']
    return result


def replay_cell(root, output, runtime, candidate, packets, name, model, filtering, source_limit):
    from rclpy.serialization import serialize_message
    from sensor_msgs.msg import PointCloud2, PointField
    from wc_sensors.pointcloud import decode_pointcloud2
    from wc_runtime.mapping_input import MappingInput
    from wc_runtime.mapping_shutdown import PERSISTENCE_CLOSE_S
    from wc_runtime import mapping_prior as production
    from wc_runtime.mapping_filter import resolve_cloud_filter

    directory = output / name
    directory.mkdir()
    stream_dir = directory / 'frontend'
    stream_dir.mkdir()
    config = copy.deepcopy(runtime)
    config.update(mapping_enabled=True, motion_model=model)
    config['planar_ekf'] = copy.deepcopy(candidate.get('planar_ekf', {}))
    # Archived for a later native stage only; MappingInput/MotionPrior never
    # consume the RTAB grid profile in this frontend factorial experiment.
    config['map_profile'] = copy.deepcopy(candidate.get('map_profile')) if model == 'planar_ekf' else None
    config['cloud_filter'] = copy.deepcopy(candidate.get('cloud_filter', {}))
    config['cloud_filter']['enabled'] = filtering
    config['cloud_filter'] = resolve_cloud_filter(config)
    config['prior_template'].update(motion_model=model, planar_ekf=config['planar_ekf'])
    # No bias seeds or calibration are inferred from the complete recording.
    require(config['prior_template'].get('confirmed_gyro_bias') is None,
            'Baseline unexpectedly contains a confirmed gyro bias; review separately')
    write_json(directory / 'runtime_config.json', config)
    mono = min(entry['monotonic_ns'] for entry in packets.events)
    # Match mapping_controller.plan's actual persistence allowance. The library
    # default is for direct callers; it is shorter than observed target eMMC
    # final checkpoints and is not the production mapping entry's close budget.
    owner = MappingInput(config, root, directory / 'input', started_ns=mono,
                         close_timeout_s=PERSISTENCE_CLOSE_S)
    queued, forwarded = deque(), {}
    counters = {'input_clouds': 0, 'warmup_clouds_dropped': 0, 'forwarded_clouds': 0}
    event_counts, source_count = Counter(), 0
    prior = None
    completed = False
    begin = time.monotonic()
    try:
        settle(owner, mono, lambda: owner.transforms is not None)
        prior_config = json.loads((directory / 'prior_config.json').read_text(encoding='utf-8'))
        prior = production.MotionPrior(prior_config, config['session_id'])
        require(prior.config.get('motion_model', 'se3_gyro') == model, 'Production estimator did not retain requested motion model')

        def receive_cloud(message):
            counters['input_clouds'] += 1
            production.enqueue_cloud(prior, queued, counters, mono * 1e-9, message)

        with (stream_dir / 'index.jsonl').open('x', encoding='utf-8') as index:
            for entry in packets.events:
                mono = max(mono, entry['monotonic_ns'])
                message = packets.get(entry)
                category = entry['category']
                event_counts[category] += 1
                if category == 'wheel':
                    prior.add_wheel(json.loads(message.data))
                elif category == 'imu':
                    require(message.angular_velocity_valid and message.linear_acceleration_valid and not message.uncertainty_valid,
                            'Invalid H30 measurement flags')
                    require(message.header.frame_id == message.imu.header.frame_id == 'imu_h30_native', 'Unexpected H30 frame')
                    a, g = message.imu.linear_acceleration, message.imu.angular_velocity
                    prior.add_imu(stamp_ns=stamp(message.host_receive_time), monotonic_ns=int(message.host_monotonic_ns),
                        acceleration=[a.x, a.y, a.z], angular_velocity=[g.x, g.y, g.z], sensor_id=message.sensor_id,
                        session_id=message.session_id, stream_epoch=message.stream_epoch, sequence=int(message.frame_sequence),
                        coordinate_convention=message.coordinate_convention, time_source=message.time_source,
                        common_time_valid=message.common_time_valid)
                else:
                    source_count += 1
                    owner.receive_source(message, mono, serialize_message, receive_cloud, clock=lambda: mono,
                                         cloud_factory=PointCloud2, field_factory=PointField)
                    # Preserve actual gates/pairing/filtering; remove random disk
                    # scheduling as a difference between the four experiment cells.
                    settle(owner, mono, lambda: not owner.pending)
                while queued:
                    prepared = production.next_prepared_cloud(prior, queued, counters, mono * 1e-9)
                    if prepared is None:
                        break
                    cloud, pose, report = prepared
                    key = report['stamp_ns']
                    require(key not in forwarded and stamp(cloud.header.stamp) == key, 'Duplicate/redated derived scan')
                    require(np.isfinite(pose).all(), 'Non-finite production pose')
                    points = decode_pointcloud2(cloud)
                    data = serialize_message(cloud)
                    filename = str(key) + '.scan.cdr'
                    with (stream_dir / filename).open('xb') as stream:
                        stream.write(data)
                    odom = authoritative_odometry(cloud, pose, report, prior)
                    odom_data, odom_filename = serialize_message(odom), str(key) + '.odom.cdr'
                    with (stream_dir / odom_filename).open('xb') as stream:
                        stream.write(odom_data)
                    row = {'stamp_ns': key, 'cloud_cdr_file': filename,
                        'cloud_type': 'sensor_msgs/msg/PointCloud2', 'cloud_topic': prior.config['output_cloud_topic'],
                        'cloud_cdr_sha256': hashlib.sha256(data).hexdigest(),
                        'cloud_data_sha256': hashlib.sha256(bytes(cloud.data)).hexdigest(),
                        'frame_id': cloud.header.frame_id, 'pose_parent': prior.config['guess_frame_id'],
                        'pose_child': prior.config['reference_frame'], 'T_prior_reference': pose.tolist(),
                        'odom_cdr_file': odom_filename, 'odom_type': 'nav_msgs/msg/Odometry',
                        'odom_topic': '/wc_mapping/app/odom', 'odom_parent': odom.header.frame_id,
                        'odom_child': odom.child_frame_id, 'odom_cdr_sha256': hashlib.sha256(odom_data).hexdigest(),
                        'pose_covariance': list(odom.pose.covariance), 'twist_covariance': list(odom.twist.covariance),
                        'linear_velocity_reference_m_s': report['linear_velocity_reference_m_s'],
                        'angular_velocity_reference_rad_s': report['angular_velocity_reference_rad_s'],
                        'input_pair_index_file': '../input/pairs.jsonl', 'input_pair_reference_stamp_ns': key,
                        'point_rows': len(points), 'finite_points': int(np.isfinite(points).all(axis=1).sum()),
                        'sample_report': report}
                    append_json(index, row)
                    # The actual pose checkpoint API is required for causal history retirement.
                    prior.record_forwarded(key, pose)
                    queued.popleft()
                    counters['forwarded_clouds'] += 1
                    forwarded[key] = {key2: value for key2, value in row.items() if key2 != 'sample_report'}
                if source_limit and source_count >= source_limit:
                    break
        completed = True
    finally:
        close_started = time.monotonic()
        try:
            owner.close()
        finally:
            # A timed-out checkpoint deliberately cannot write a final STOPPED
            # status. Preserve the actual failure and in-memory I/O state here
            # instead of leaving only its earlier STARTING status and a generic
            # "frontend failed" exception. This is diagnostic, never recovery.
            close_diagnostic = {'replay_loop_completed': completed,
                'close_elapsed_s': time.monotonic()-close_started,
                'production_close_timeout_s': PERSISTENCE_CLOSE_S,
                'owner_failure_reason': owner.failure_reason,
                'prior_failure': prior.failure if prior is not None else None,
                'forwarded_frames': len(forwarded), 'packet_counts': dict(event_counts),
                'input_status_after_close_attempt': owner.status(time.monotonic_ns())}
            write_json(directory / 'close_diagnostic.json', close_diagnostic)
    require(completed and owner.failure_reason is None and prior is not None and prior.failure is None,
            'Production frontend failed: '+json.dumps({'loop_completed': completed,
                'input': owner.failure_reason, 'prior': prior.failure if prior is not None else 'not constructed'},
                ensure_ascii=False))
    status = json.loads((directory / 'input/status.json').read_text(encoding='utf-8'))
    require(status['state'] == 'STOPPED', 'Input archive did not close normally')
    pairs = {}
    with (directory / 'input/pairs.jsonl').open(encoding='utf-8') as stream:
        for line in stream:
            row = json.loads(line)
            key = row['reference_stamp_ns']
            require(key not in pairs, 'Duplicate archived pair time')
            pairs[key] = row
    write_json(directory / 'prior_initialization.json', prior.initialization)
    result = {'status': 'FRONTEND_REPLAY_COMPLETE', 'motion_model': model, 'filter_enabled': filtering,
        'wall_elapsed_s': time.monotonic() - begin, 'packet_counts': dict(event_counts), 'counters': counters,
        'pending_clouds_at_input_end': len(queued), 'pending_cloud_stamps_ns': [stamp(msg.header.stamp) for _, msg in queued],
        'input_status': status, 'motion_coverage': prior.coverage_report(), 'bias': prior.bias_report(),
        'initialization': prior.initialization, 'pair_count': len(pairs), 'output_frames': len(forwarded),
        'native_rtabmap': {'status': 'NOT_RUN', 'nodes': None},
        'coordinate_warning': 'Planar Z/roll/pitch constraints are model assumptions, not measured accuracy.'}
    write_json(directory / 'summary.json', result)
    return {'summary': result, 'frames': forwarded, 'pairs': pairs, 'directory': directory}


def pose_statistics(frames, selected):
    poses = [np.asarray(frames[key]['T_prior_reference']) for key in selected]
    if not poses:
        return {'frames': 0}
    position = np.asarray([p[:3, 3] for p in poses])
    euler = Rotation.from_matrix(np.asarray([p[:3, :3] for p in poses])).as_euler('xyz', degrees=True)
    return {'frames': len(poses), 'first_stamp_ns': selected[0], 'last_stamp_ns': selected[-1],
        'span_s': (selected[-1] - selected[0]) * 1e-9,
        'z_m': stats(position[:, 2]), 'roll_deg': stats(euler[:, 0]), 'pitch_deg': stats(euler[:, 1]),
        'xyz_extent_m': np.ptp(position, axis=0).tolist(),
        'path_length_m': float(np.linalg.norm(np.diff(position, axis=0), axis=1).sum()),
        'interpretation': 'Trajectory/model diagnostics only; no surveyed pose error or ATE.'}


def roi_statistics(path, source_id, cells, common):
    """Optional independent, fixed pixel support and plane; never fit to output."""
    if path is None:
        return {'available': False, 'reason': 'NO_INDEPENDENT_FIXED_PLANE_PIXEL_ROI', 'ground_thickness_m': None}
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import PointCloud2
    from wc_sensors.pointcloud import decode_pointcloud2
    roi = json.loads(path.read_text(encoding='utf-8'))
    require(roi.get('schema_version') == 1 and roi.get('kind') == 'independent_fixed_plane_roi', 'Invalid ROI schema')
    require(roi.get('source_session_id') == source_id and roi.get('independent') is True, 'ROI must declare independent source identity')
    require(isinstance(roi.get('selection_evidence'), str) and roi['selection_evidence'].strip(), 'ROI needs independent selection evidence')
    normal = np.asarray(roi['normal_in_prior_frame'], dtype=float)
    offset = float(roi['plane_offset_m'])
    require(normal.shape == (3,) and np.isfinite(normal).all() and abs(np.linalg.norm(normal)-1) < 1e-6
            and np.isfinite(offset), 'ROI fixed plane must be finite with unit normal')
    residuals = {name: [] for name in cells}
    counts = {name: Counter() for name in cells}
    missed, seen = [], set()
    for selection in roi['frames']:
        key = selection['stamp_ns']
        require(type(key) is int and key not in seen, 'Duplicate/noninteger ROI frame stamp')
        seen.add(key)
        if key not in common:
            missed.append(key)
            continue
        indices = np.asarray(selection['row_indices'])
        require(indices.ndim == 1 and len(indices) and indices.dtype.kind in 'iu'
                and len(np.unique(indices)) == len(indices) and int(indices.min()) >= 0, 'Invalid fixed ROI indices')
        selected = {}
        for name, cell in cells.items():
            row = cell['frames'][key]
            file = cell['directory'] / 'frontend' / row['cloud_cdr_file']
            require(digest(file) == row['cloud_cdr_sha256'], 'Exported scan changed before ROI audit')
            points = decode_pointcloud2(deserialize_message(file.read_bytes(), PointCloud2))
            require(int(indices.max()) < len(points), 'ROI exceeds original point-row layout')
            pose = np.asarray(row['T_prior_reference'])
            selected[name] = points[indices] @ pose[:3, :3].T + pose[:3, 3]
            counts[name]['selected_rows'] += len(indices)
            counts[name]['retained_finite_rows'] += int(np.isfinite(selected[name]).all(axis=1).sum())
        common_mask = np.logical_and.reduce([np.isfinite(points).all(axis=1) for points in selected.values()])
        for name, points in selected.items():
            residuals[name].extend((points[common_mask] @ normal + offset).tolist())
            counts[name]['common_retained_rows'] += int(common_mask.sum())
    return {'available': True, 'roi_sha256': digest(path), 'selection_evidence': roi['selection_evidence'],
        'independence': 'CALLER_DECLARED; this tool does not invent or verify physical ground truth',
        'missing_common_frame_stamps': missed, 'selection_frames': len(seen),
        'cells': {name: {'coverage': dict(counts[name]), 'signed_fixed_plane_distance_m': stats(values),
            'p95_minus_p05_m': float(np.quantile(values, .95)-np.quantile(values, .05)) if values else None}
            for name, values in residuals.items()},
        'limitation': 'Same frozen pixel set and same fixed plane; no refit, RANSAC, residual trimming, or enforced flat cloud.'}


def compare(cells, recorded_priors):
    common = sorted(set.intersection(*(set(cell['frames']) for cell in cells.values())))
    require(common, 'No common production-forwarded scan; replay is not a usable native A/B input')
    all_stamps = set.union(*(set(cell['frames']) for cell in cells.values()))
    summary = {'selected_cells': list(cells), 'common_forwarded_frames': len(common),
               'common_frame_stamps_ns': common, 'union_forwarded_frames': len(all_stamps), 'cells': {}}
    for name, cell in cells.items():
        frames = cell['frames']
        keys = sorted(frames)
        summary['cells'][name] = {'all_forwarded': pose_statistics(frames, keys),
            'common_frames': pose_statistics(frames, common),
            'not_in_selected_cell_common': len(set(keys)-set(common)),
            'finite_points': stats([row['finite_points'] for row in frames.values()]),
            'pair_delta_ns': stats([row.get('pair_delta_ns', max(s['source_stamp_ns'] for s in row['sources'])-
                min(s['source_stamp_ns'] for s in row['sources'])) for row in cell['pairs'].values()])}
    # Every shared scan must bind to the same source identity, rows and time.
    for key in common:
        bindings = []
        for cell in cells.values():
            require(key in cell['pairs'], 'Forwarded scan has no actual input archive pair')
            bindings.append([(row['side'], row['raw_key'], row['source_stamp_ns']) for row in cell['pairs'][key]['sources']])
        require(all(value == bindings[0] for value in bindings[1:]), 'Common scan has different source pairing')
    baseline = cells['se3_no_filter']['frames']
    shared = sorted(set(baseline) & set(recorded_priors))
    position, angle = [], []
    for key in shared:
        old, new = recorded_priors[key], np.asarray(baseline[key]['T_prior_reference'])
        position.append(float(np.linalg.norm(old[:3, 3]-new[:3, 3])))
        angle.append(float(np.rad2deg(Rotation.from_matrix(old[:3, :3].T @ new[:3, :3]).magnitude())))
    summary['recorded_prior_comparison'] = {'common_frames': len(shared), 'position_difference_m': stats(position),
        'rotation_difference_deg': stats(angle),
        'interpretation': 'Deterministic source-time replay is not recorded ROS callback order; differences must be reviewed, not called accuracy.'}
    return summary, common


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--project-root', type=Path, default=Path(__file__).resolve().parents[2])
    parser.add_argument('--source', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--candidate-config', type=Path, required=True, help='Reviewed config; import cloud_filter/planar_ekf, archive map_profile for later native stage')
    parser.add_argument('--source-manifest', type=Path, required=True)
    parser.add_argument('--verify-source-hashes', action='store_true', help='Rehash shared originals once; otherwise verify shared manifest sizes/mtimes and every consumed CDR')
    parser.add_argument('--cloud', choices=('raw', 'filtered'), help='Explicit separate experiment; never switch representation between cells')
    parser.add_argument('--source-record-limit', type=int, default=0, help='Diagnostic prefix only; 0 means complete recording')
    parser.add_argument('--cells', nargs='+', choices=[row[0] for row in CELLS],
                        default=[row[0] for row in CELLS], help='Baseline is required; start with se3_no_filter planar_filter if desired')
    parser.add_argument('--independent-roi', type=Path)
    args = parser.parse_args(argv)
    root, source = args.project_root.resolve(), args.source.resolve()
    require(source.is_dir() and source.is_relative_to(root), 'Source must be an existing project session')
    require(args.source_record_limit >= 0, 'Negative source-record limit')
    require(len(set(args.cells)) == len(args.cells) and 'se3_no_filter' in args.cells,
            'Cells must be unique and include se3_no_filter')
    output = checked_output(root, source, args.output)
    sys.path.insert(0, str(root / 'src'))
    runtime = json.loads((source / 'runtime_config.json').read_text(encoding='utf-8'))
    candidate = json.loads(args.candidate_config.read_text(encoding='utf-8'))
    require(runtime['session_id'] == source.name and runtime.get('continuous_mapping') is True, 'Unexpected source session/profile')
    require(runtime.get('odometry_source') == 'wheel_imu', 'This A/B baseline requires the recorded wheel_imu source')
    if args.cloud:
        runtime['cloud_source'] = args.cloud
    else:
        runtime.setdefault('cloud_source', 'filtered')
    manifest = json.loads(args.source_manifest.read_text(encoding='utf-8'))
    require(Path(manifest['source']).resolve() == source, 'Shared manifest source mismatch')
    original_paths = []
    for relative, evidence in manifest['source_file_manifest'].items():
        path = source / relative
        require('..' not in Path(relative).parts and path.resolve().is_relative_to(source), 'Unsafe source manifest path')
        require(path.stat().st_size == evidence['bytes'] and path.stat().st_mtime_ns == evidence['mtime_ns'], 'Original manifest size/mtime mismatch: ' + relative)
        if args.verify_source_hashes:
            require(digest(path) == evidence['sha256'], 'Original SHA256 mismatch: ' + relative)
        original_paths.append(path)
    before = signatures(original_paths)
    output.mkdir(parents=True)
    write_json(output / 'shared_source_manifest.json', manifest)
    write_json(output / 'candidate_config.json', candidate)
    source_files = [root/'src/wc_runtime'/name for name in
        ('mapping_prior.py', 'mapping_planar.py', 'mapping_filter.py', 'mapping_input.py', 'single_mapping_input.py',
         'mapping_odometry.py', 'mapping_bias.py', 'mapping_profile.py')]
    write_json(output / 'implementation_manifest.json', {
        'tool_sha256': digest(Path(__file__)), 'source_files': {str(p.relative_to(root)): digest(p) for p in source_files if p.is_file()},
        'source_manifest_sha256': digest(args.source_manifest), 'original_files_rehashed': args.verify_source_hashes})
    packets, cells = None, {}
    try:
        packets = RecordedPackets(source, runtime)
        with (output / 'original_packet_index.jsonl').open('x', encoding='utf-8') as stream:
            for entry in packets.events:
                append_json(stream, entry)
        for name, model, filtering in CELLS:
            if name not in args.cells:
                continue
            print('REPLAY', name, flush=True)
            cells[name] = replay_cell(root, output, runtime, candidate, packets, name, model, filtering,
                                      args.source_record_limit)
        comparisons, common = compare(cells, packets.recorded_priors)
        database = source / 'slam/rtabmap.db'
        with sqlite3.connect(database.as_uri() + '?mode=ro', uri=True) as db:
            db.execute('PRAGMA query_only=ON')
            recorded_nodes = db.execute('SELECT COUNT(*) FROM Node').fetchone()[0]
        require(signatures(original_paths) == before and signatures(packets.paths) == packets.before, 'Original files changed')
        result = {'schema_version': 1, 'status': 'FRONTEND_AB_COMPLETE_NATIVE_RTABMAP_NOT_RUN', 'verification_level': 'REAL_BAG_FRONTEND',
            'source_session_id': runtime['session_id'], 'cloud_source': runtime['cloud_source'],
            'hardware_accessed': False, 'ros_nodes_started': False, 'originals_unchanged_by_stat': True,
            'original_recorded_nodes': recorded_nodes, 'source_packet_counts': dict(packets.counts),
            'source_record_limit': args.source_record_limit, 'complete_recording_requested': args.source_record_limit == 0,
            'packet_order': 'original source stamp; wheel then IMU then lidar at ties, preserving per-source sequence; not recorded callback timing',
            'archive_schedule': 'actual MappingInput, drain archive after each lidar callback; no live scheduling/disk-throughput claim',
            'comparisons': comparisons, 'roi': roi_statistics(args.independent_roi, runtime['session_id'], cells, common),
            'limitations': ['No native SLAM/loop closure/map export performed.', 'No surveyed trajectory or wall/floor truth inferred.',
                'Planar Z/roll/pitch are imposed assumptions, never accuracy evidence.',
                'Original arrival stamps remain unchanged; no invented exposure/per-point timing.',
                'Historical missing image-layout metadata must remain missing; neighborhood-filter skips must be reported.',
                'Coverage and retained pixel count must accompany any residual/thickness comparison.']}
        write_json(output / 'result.json', result)
        print(json.dumps({'status': result['status'], 'output': str(output), 'common_frames': len(common)}, ensure_ascii=False))
    except BaseException as error:
        write_json(output / 'failure.json', {'status': 'FAILED', 'error': type(error).__name__ + ': ' + str(error),
            'completed_cells': list(cells), 'originals_unchanged_by_stat': signatures(original_paths) == before,
            'hardware_accessed': False, 'ros_nodes_started': False})
        raise
    finally:
        if packets is not None:
            packets.close()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
