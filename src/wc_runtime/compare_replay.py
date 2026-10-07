"""Reusable immutable SQLite packet replay through production input and prior.

Promoted from the audited 20260916 experiment. No ROS node or hardware is
created. All real wheel/IMU events are consumed; only cloud inputs are gated.
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


from .source_time import ORDER, event_key, SourceClock
from .resource_audit import snapshot as resource_snapshot,delta as resource_delta


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
        self.paths = sorted(Path(config.get('_offline_bag_path', source / 'bag')).glob('*.db3'))
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
            self.clock = SourceClock(self.events)
            self.events = [self.clock.canonicalize(row) for row in self.events]
            self.events.sort(key=event_key)
            require(all(any(row['category'] == kind for row in self.events) for kind in ORDER), 'Source/IMU/wheel packets required')
            require({row['side'] for row in self.events if row['category'] == 'source'} == set(self.sides),
                    'All requested lidar sides must be present; no single-side substitution')
        except BaseException:
            self.close()
            raise

    def get(self, entry):
        row = self.connections[entry['bag']].execute('SELECT data FROM messages WHERE id=?', (entry['message_id'],)).fetchone()
        require(row is not None and hashlib.sha256(row[0]).hexdigest() == entry['cdr_sha256'], 'Original CDR changed during replay')
        message = self.decode(row[0], self.types[entry['type']])
        key=entry['stamp_ns']
        if entry['category']=='wheel':
            value=json.loads(message.data)
            value['original_stamp_ns']=value['stamp_ns']; value['stamp_ns']=key
            message.data=json.dumps(value,allow_nan=False,separators=(',',':'))
        else:
            original=entry['original_source_stamp_ns']
            subordinate=message.cloud.header.stamp if entry['category']=='source' else message.imu.header.stamp
            require(stamp(message.header.stamp)==stamp(message.host_receive_time)==stamp(subordinate)==original,
                    'Original source timestamp metadata inconsistent; never repair original input')
            require(message.common_time_valid is False and message.time_source=='arrival_only',
                    'Canonical offline clock requires explicitly arrival-only source metadata')
            if entry['category']=='source':
                require(message.common_time_ns==original,'Original lidar arrival metadata inconsistent')
                message.common_time_ns=key # Arrival-only mirror, common_time_valid stays False.
            else:
                require(message.common_time_ns==0,'Original H30 common-time metadata must remain unknown zero')
            for value in (message.header.stamp,message.host_receive_time,
                          subordinate):
                value.sec,value.nanosec=divmod(key,10**9)
        return message

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


def replay_cell(root, output, runtime, candidate, packets, name, estimator, filtering, source_limit, input_rate_hz, *, storage_root=None, prior_factory=None):
    resources_before=resource_snapshot()
    from rclpy.serialization import serialize_message
    from sensor_msgs.msg import PointCloud2, PointField
    from wc_sensors.pointcloud import decode_pointcloud2
    from wc_runtime.mapping_input import MappingInput
    from wc_runtime.mapping_shutdown import PERSISTENCE_CLOSE_S
    from wc_runtime import mapping_prior as production
    from wc_runtime.mapping_filter import resolve_cloud_filter
    from wc_runtime.mapping_planar import validate_planar_config

    directory = output / name
    directory.mkdir()
    stream_dir = directory / 'frontend'
    stream_dir.mkdir()
    config = copy.deepcopy(runtime)
    model = 'planar_ekf'
    config.update(mapping_enabled=True, motion_model=model, wheel_imu_estimator=estimator,
                  offline_experiment=True, input_rate_hz=input_rate_hz)
    config['planar_ekf'] = validate_planar_config(candidate.get('planar_ekf', {}))
    # Archived for a later native stage only; MappingInput/MotionPrior never
    # consume the RTAB grid profile in this frontend factorial experiment.
    config['map_profile'] = copy.deepcopy(candidate.get('map_profile')) if model == 'planar_ekf' else None
    config['cloud_filter'] = copy.deepcopy(candidate.get('cloud_filter', {}))
    config['cloud_filter']['enabled'] = filtering
    config['cloud_filter'] = resolve_cloud_filter(config)
    config['prior_template'].update(motion_model=model, estimator=estimator, planar_ekf=config['planar_ekf'])
    # Every cell uses the same explicit frozen calibration. Future-data bias
    # candidates never update or seed the comparison.
    write_json(directory / 'runtime_config.json', config)
    mono = min(entry['monotonic_ns'] for entry in packets.events)
    # Match mapping_controller.plan's actual persistence allowance. The library
    # default is for direct callers; it is shorter than observed target eMMC
    # final checkpoints and is not the production mapping entry's close budget.
    # The caller validates an optional RAM workspace. This changes only output
    # containment; production algorithm imports still come from this code tree.
    data_root = root if storage_root is None else Path(storage_root)
    owner = MappingInput(config, data_root, directory / 'input', started_ns=mono,
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
        prior_config['estimator'] = estimator
        # Offline refinement may provide a validated, session-scoped adapter.
        # The normal compare/live behavior retains the production prior exactly.
        factory = production.MotionPrior if prior_factory is None else prior_factory
        prior = factory(prior_config, config['session_id'], clock=lambda: mono*1e-9)
        write_json(directory / 'resolved_estimator_config.json', prior.config)
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
                    if prior_factory is not None:
                        report['source_pose_samples'] = {
                            str(offset): prior.pose_at(key+int(offset)).tolist()
                            for offset in report.get('source_offsets_ns', [0])}
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
    result = {'status': 'FRONTEND_REPLAY_COMPLETE', 'motion_model': model, 'estimator': estimator,
        'input_rate_hz': input_rate_hz, 'filter_enabled': filtering,
        'input_rate_policy': ('ALL_VALID_PAIRS_NOT_THROTTLED' if input_rate_hz == 0
                              else 'MINIMUM_SOURCE_AND_PAIR_INTERVAL'),
        'wall_elapsed_s': time.monotonic() - begin, 'packet_counts': dict(event_counts), 'counters': counters,
        'pending_clouds_at_input_end': len(queued), 'pending_cloud_stamps_ns': [stamp(msg.header.stamp) for _, msg in queued],
        'input_status': status, 'motion_coverage': prior.coverage_report(), 'bias': prior.bias_report(),
        'input_status_clock_scope': {
            'last_publication_source_age_ns_valid_for_replay': False,
            'reason': 'close snapshot subtracts historical source monotonic from current host clock',
            'trajectory_clock': 'controlled source event clock; close does not publish'},
        'initialization': prior.initialization, 'pair_count': len(pairs), 'output_frames': len(forwarded),
        'native_rtabmap': {'status': 'NOT_RUN', 'nodes': None},
        'coordinate_warning': 'Planar Z/roll/pitch constraints are model assumptions, not measured accuracy.'}
    result['resources']=resource_delta(resources_before,resource_snapshot())
    write_json(directory / 'summary.json', result)
    return {'summary': result, 'frames': forwarded, 'pairs': pairs, 'directory': directory}



