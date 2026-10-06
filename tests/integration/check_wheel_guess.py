#!/usr/bin/env python3
"""Observe optional synthetic filtered-cloud + wheel-guess ROS integration.

Start this read-only observer first, wait for WHEEL_GUESS_OBSERVER_READY, then
start the prepared synthetic mapping session. It publishes nothing and performs
no device, graph-close, control or reset operations. Exit 0 requires the entire
configured finite source, actual native OdomInfo guesses, dual-source tracking,
and private/public TF separation. This is not real encoder or vendor-filter QA.
"""

import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import time

import numpy as np
from scipy.spatial.transform import Rotation


def stamp(header):
    return int(header.stamp.sec) * 1_000_000_000 + int(header.stamp.nanosec)


def transform_matrix(value):
    q = np.asarray([value.rotation.x, value.rotation.y, value.rotation.z, value.rotation.w], dtype=float)
    xyz = np.asarray([value.translation.x, value.translation.y, value.translation.z], dtype=float)
    if not np.isfinite(q).all() or not np.isfinite(xyz).all() or abs(np.linalg.norm(q) - 1) > 1e-5:
        raise ValueError('nonfinite or invalid transform quaternion')
    pose = np.eye(4)
    pose[:3, :3] = Rotation.from_quat(q).as_matrix()
    pose[:3, 3] = xyz
    return pose


def pose_matrix(value):
    from types import SimpleNamespace
    return transform_matrix(SimpleNamespace(translation=value.position, rotation=value.orientation))


class Recorder:
    def __init__(self, config):
        self.config = config
        self.raw = {}
        self.filtered = {}
        self.wheel = {}
        self.wheel_diag = {}
        self.private = {}
        self.info = {}
        self.tracking = {}
        self.bundles = {}
        self.public = {}
        self.public_edges = Counter()
        self.counts = Counter()
        self.errors = []
        self.endpoint_snapshots = []
        self.private_tf_evidence = None
        self.private_tf_evidence_error = None
        self.private_tf_evidence_path = None
        self.private_tf_evidence_sha256 = None

    def store(self, destination, key, value):
        if len(destination) >= 1024 and key not in destination:
            raise ValueError('observer bounded cache full')
        if key in destination and json.dumps(destination[key], sort_keys=True) != json.dumps(value, sort_keys=True):
            raise ValueError('same observation identity changed content')
        destination[key] = value

    def callback(self, name, handler):
        def receive(message):
            self.counts[name] += 1
            try:
                handler(message)
            except Exception as error:
                if len(self.errors) < 100:
                    self.errors.append({'callback': name, 'error': str(error)})
        return receive

    def source(self, message, filtered=False):
        if message.session_id != self.config['session_id']:
            return
        if message.side not in ('left', 'right'):
            raise ValueError('unexpected source side')
        data = {'stamp_ns': stamp(message.header), 'common_time_ns': int(message.common_time_ns),
                'frame_sequence': int(message.frame_sequence), 'side': message.side,
                'count': int(message.cloud.width) * int(message.cloud.height),
                'cloud_sha256': hashlib.sha256(bytes(message.cloud.data)).hexdigest(),
                'config_hash': message.source_config_hash, 'flags': list(message.diagnostic_flags),
                'source_id': message.sensor_id, 'epoch': message.stream_epoch,
                'host_receive_ns': stamp(type('Header', (), {'stamp': message.host_receive_time})()),
                'time_source': message.time_source, 'time_valid': bool(message.common_time_valid),
                'units': message.units, 'coordinate_convention': message.coordinate_convention}
        if not filtered:
            cloud = message.cloud
            if cloud.height != 1 or cloud.row_step != cloud.width * cloud.point_step:
                raise ValueError('fixture expects one contiguous cloud row')
            rows = np.frombuffer(bytes(cloud.data), dtype=np.uint8).reshape(int(cloud.width), int(cloud.point_step))
            data['every_second_point_sha256'] = hashlib.sha256(rows[::2].tobytes()).hexdigest()
        key = f'{message.side}:{message.frame_sequence}'
        self.store(self.filtered if filtered else self.raw, key, data)

    def wheel_message(self, message):
        data = {'stamp_ns': stamp(message.header), 'parent': message.header.frame_id, 'child': message.child_frame_id,
                'pose': pose_matrix(message.pose.pose).tolist(), 'linear_x': float(message.twist.twist.linear.x),
                'angular_z': float(message.twist.twist.angular.z)}
        self.store(self.wheel, data['stamp_ns'], data)

    def wheel_diagnostic(self, message):
        value = json.loads(message.data)
        if value.get('state') == 'FEEDBACK':
            self.store(self.wheel_diag, value['stamp_ns'], value)

    def private_tf(self, message):
        for value in message.transforms:
            if (value.header.frame_id, value.child_frame_id) != ('wheel_odom', 'rig_link'):
                raise ValueError('unexpected private TF edge')
            self.store(self.private, stamp(value.header), transform_matrix(value.transform).tolist())

    def public_tf(self, message):
        for value in message.transforms:
            edge = value.header.frame_id + '->' + value.child_frame_id
            self.public_edges[edge] += 1
            if edge == 'odom->rig_link':
                self.store(self.public, stamp(value.header), transform_matrix(value.transform).tolist())

    def odom_info(self, message):
        self.store(self.info, stamp(message.header), {'guess': transform_matrix(message.guess).tolist(),
                   'lost': bool(message.lost), 'icp_correspondences': int(message.icp_correspondences)})

    def assessment(self, message):
        if message.session_id != self.config['session_id']:
            return
        self.store(self.tracking, stamp(message.header), {'bundle_id': int(message.bundle_id),
                   'accepted': bool(message.accepted), 'state': message.tracking_state,
                   'left': int(message.left_solver_input_count), 'right': int(message.right_solver_input_count),
                   'native_pose': pose_matrix(message.odometry.pose.pose).tolist()})

    def bundle(self, message):
        if message.session_id != self.config['session_id']:
            return
        codes = Counter(int(value) for value in message.source_codes)
        self.store(self.bundles, stamp(message.header), {'bundle_id': int(message.bundle_id),
                   'left': int(message.left_retained_count), 'right': int(message.right_retained_count),
                   'left_time_ns': int(message.left_time_ns), 'right_time_ns': int(message.right_time_ns),
                   'left_raw_key': message.left_raw_key, 'right_raw_key': message.right_raw_key,
                   'source_codes': dict(codes), 'dual_valid': bool(message.dual_valid),
                   'compensation_mode': message.compensation_mode,
                   'cloud_count': int(message.cloud.width) * int(message.cloud.height)})

    def discover(self, node):
        snapshot = {}
        for topic in ('/wc_mapping/icp/odom_info', '/wc_mapping/icp_guess_tf', '/tf'):
            snapshot[topic] = [{'node': endpoint.node_name, 'namespace': endpoint.node_namespace,
                               'gid': bytes(endpoint.endpoint_gid).hex()}
                              for endpoint in node.get_publishers_info_by_topic(topic)]
        if not self.endpoint_snapshots or snapshot != self.endpoint_snapshots[-1]:
            self.endpoint_snapshots.append(snapshot)


def compare_guesses(private, info, translation_tolerance_m=1e-5, rotation_tolerance_rad=1e-5):
    """Actual native guess is current rig pose relative to previous rig pose."""
    rows = []
    stamps = sorted(private)
    for previous_stamp, current_stamp in zip(stamps, stamps[1:]):
        if current_stamp not in info:
            rows.append({'stamp_ns': current_stamp, 'status': 'MISSING_NATIVE_GUESS'})
            continue
        expected = np.linalg.inv(np.asarray(private[previous_stamp])) @ np.asarray(private[current_stamp])
        observed = np.asarray(info[current_stamp]['guess'])
        translation = float(np.linalg.norm(observed[:3, 3] - expected[:3, 3]))
        rotation = float(Rotation.from_matrix(expected[:3, :3].T @ observed[:3, :3]).magnitude())
        rows.append({'stamp_ns': current_stamp, 'previous_stamp_ns': previous_stamp,
                     'translation_error_m': translation, 'rotation_error_rad': rotation,
                     'expected_guess': expected.tolist(), 'native_guess': observed.tolist(),
                     'status': 'PASS' if translation <= translation_tolerance_m and rotation <= rotation_tolerance_rad else 'FAIL'})
    return rows


def check_private_tf_attribution(evidence, private, *, session_id, expected_count):
    """Attribute actual received TF messages, never merely discovered endpoints.

    An unused TransformBroadcaster still appears in DDS discovery. A publisher
    is an actual sender only when its GID occurs in native MessageInfo records.
    Every native record must match this observer's timestamp and transform.
    """
    errors, comparisons = [], []
    if not isinstance(evidence, dict):
        return {'status': 'FAIL', 'errors': ['NATIVE_PRIVATE_TF_EVIDENCE_MISSING'], 'comparisons': []}
    if evidence.get('schema_version') != 1 or evidence.get('test') != 'native_tf_ownership_probe':
        errors.append('NATIVE_PRIVATE_TF_SCHEMA_MISMATCH')
    if evidence.get('status') != 'PASS' or evidence.get('errors') or evidence.get('interrupted') is not False:
        errors.append('NATIVE_PRIVATE_TF_PROBE_NOT_PASS')
    if evidence.get('session_id') != session_id:
        errors.append('NATIVE_PRIVATE_TF_SESSION_MISMATCH')
    if evidence.get('wheel_private_mode') is not True or evidence.get('topic') != '/wc_mapping/icp_guess_tf':
        errors.append('NATIVE_PRIVATE_TF_TOPIC_OR_MODE_MISMATCH')
    expected_scope = 'Actual observed /wc_mapping/icp_guess_tf messages only; no claim about unobserved publishers or TF static topic.'
    if evidence.get('scope') != expected_scope:
        errors.append('NATIVE_PRIVATE_TF_SCOPE_MISMATCH')
    edges = evidence.get('edges', {})
    if not isinstance(edges, dict) or set(edges) != {'wheel_odom->rig_link'}:
        errors.append('NATIVE_PRIVATE_TF_EDGE_MISMATCH')
        edges = {}
    edge = edges.get('wheel_odom->rig_link', {})
    if not isinstance(edge, dict):
        errors.append('NATIVE_PRIVATE_TF_EDGE_RECORD_INVALID')
        edge = {}
    observations = edge.get('observations', [])
    if not isinstance(observations, list):
        observations = []
        errors.append('NATIVE_PRIVATE_TF_RECORDS_INVALID')
    if len(observations) != expected_count or edge.get('transform_count') != len(observations):
        errors.append('NATIVE_PRIVATE_TF_INCOMPLETE_COUNT')
    gid_size = evidence.get('rmw_gid_storage_size')
    if type(gid_size) is not int or not 16 <= gid_size <= 32:
        errors.append('NATIVE_PRIVATE_TF_GID_SIZE_INVALID')
        gid_size = 24
    senders, stamps = Counter(), set()
    for row in observations:
        if not isinstance(row, dict):
            errors.append('NATIVE_PRIVATE_TF_RECORD_INVALID');continue
        stamp_ns, gid = row.get('stamp_ns'), row.get('publisher_gid')
        if type(stamp_ns) is not int or stamp_ns <= 0 or stamp_ns in stamps:
            errors.append('NATIVE_PRIVATE_TF_DUPLICATE_OR_INVALID_STAMP');continue
        stamps.add(stamp_ns)
        if not isinstance(gid, str) or len(gid) != gid_size * 2 or not set(gid) <= set('0123456789abcdef') or set(gid) == {'0'}:
            errors.append('NATIVE_PRIVATE_TF_MESSAGE_GID_INVALID');continue
        senders[gid] += 1
        try:
            values = np.asarray(row['transform_xyz_xyzw'], dtype=float)
            if values.shape != (7,) or not np.isfinite(values).all() or abs(np.linalg.norm(values[3:]) - 1) > 1e-5:
                raise ValueError('invalid native pose')
            native = np.eye(4)
            native[:3, :3] = Rotation.from_quat(values[3:]).as_matrix()
            native[:3, 3] = values[:3]
            if stamp_ns not in private:
                raise ValueError('native stamp absent from Python private observations')
            difference = float(np.max(np.abs(native - np.asarray(private[stamp_ns]))))
            passed = difference <= 1e-10
            comparisons.append({'stamp_ns': stamp_ns, 'publisher_gid': gid,
                                'maximum_matrix_difference': difference, 'status': 'PASS' if passed else 'FAIL'})
            if not passed:errors.append('NATIVE_PRIVATE_TF_VALUE_MISMATCH')
        except (KeyError, TypeError, ValueError) as error:
            errors.append('NATIVE_PRIVATE_TF_SAMPLE_MISMATCH:' + str(error))
    if stamps != set(private) or len(private) != expected_count:
        errors.append('NATIVE_PRIVATE_TF_STAMP_COVERAGE_MISMATCH')
    if len(senders) != 1 or dict(senders) != edge.get('publisher_gids'):
        errors.append('NATIVE_PRIVATE_TF_SENDER_NOT_UNIQUE_OR_COUNT_MISMATCH')
    endpoint_history = evidence.get('endpoint_history', [])
    if not isinstance(endpoint_history, list):
        endpoint_history = []
        errors.append('NATIVE_PRIVATE_TF_ENDPOINT_HISTORY_INVALID')
    identities = []
    for gid, count in senders.items():
        nodes = {endpoint.get('node') for endpoint in endpoint_history if isinstance(endpoint, dict)
                 and endpoint.get('publisher_gid') == gid
                 and endpoint.get('node') != '_NODE_NAMESPACE_UNKNOWN_/_NODE_NAME_UNKNOWN_'}
        valid = nodes == {'/wc_dual_fusion'}
        identities.append({'publisher_gid': gid, 'actual_transform_count': count,
                           'resolved_nodes': sorted(str(node) for node in nodes), 'status': 'PASS' if valid else 'FAIL'})
        if not valid:errors.append('NATIVE_PRIVATE_TF_ACTUAL_SENDER_NOT_FUSION')
    unused = [endpoint for endpoint in endpoint_history if isinstance(endpoint, dict) and endpoint.get('publisher_gid') not in senders]
    return {'status': 'PASS' if not errors else 'FAIL', 'errors': errors, 'comparisons': comparisons,
            'actual_senders': identities, 'discovered_endpoints_without_observed_transforms': unused,
            'attribution': 'Native rclcpp MessageInfo publisher GID matched to DDS endpoint history; discovery alone is not transmission evidence'}


def build_report(recorder):
    r, config = recorder, recorder.config
    count = int(config['synthetic']['frame_count'])
    checks, source_rows, source_errors = {}, [], []
    expected_keys = {f'{side}:{i}' for side in ('left', 'right') for i in range(1, count + 1)}
    for key in sorted(expected_keys):
        raw, filtered = r.raw.get(key), r.filtered.get(key)
        if not raw or not filtered:
            source_errors.append({'key': key, 'error': 'MISSING_RAW_OR_FILTERED'})
            continue
        flags = set(filtered['flags'])
        errors = []
        if filtered['count'] != (raw['count'] + 1) // 2 or not 0 < filtered['count'] < raw['count']:
            errors.append('FILTER_NOT_ACTUALLY_REDUCING_POINTS')
        if filtered['cloud_sha256'] != raw['every_second_point_sha256']:
            errors.append('FILTERED_CLOUD_NOT_EXACT_RAW_SUBSAMPLE')
        if not {'raw_source_config_hash=' + raw['config_hash'], 'before_host_filter_cloud_sha256=' + raw['cloud_sha256']}.issubset(flags):
            errors.append('RAW_HASH_REFERENCE_MISMATCH')
        if any(raw[field] != filtered[field] for field in ('stamp_ns', 'common_time_ns', 'source_id', 'epoch', 'host_receive_ns')):
            errors.append('RAW_FILTERED_IDENTITY_MISMATCH')
        if raw['stamp_ns'] != raw['common_time_ns'] or raw['stamp_ns'] > raw['host_receive_ns'] or not raw['time_valid']:
            errors.append('INVALID_OR_FUTURE_SOURCE_TIME')
        if raw['source_id'] != config['sensor_ids'][raw['side']] or raw['units'] != 'm' or raw['coordinate_convention'] != 'FLU' or raw['time_source'] != 'synthetic':
            errors.append('SOURCE_IDENTITY_OR_UNITS_INVALID')
        source_rows.append({'key': key, 'stamp_ns': raw['stamp_ns'], 'raw_count': raw['count'],
                            'filtered_count': filtered['count'], 'status': 'FAIL' if errors else 'PASS'})
        if errors:source_errors.append({'key': key, 'errors': errors})
    checks['filtered_branch_changes_points_with_raw_hash'] = not source_errors and set(r.raw) == set(r.filtered) == expected_keys
    expected_stamps = {value['stamp_ns'] for key, value in r.raw.items() if key.startswith('left:')}
    guess_rows = compare_guesses(r.private, r.info)
    checks['native_guesses_match_private_wheel_deltas'] = len(guess_rows) >= count - 1 and all(row['status'] == 'PASS' for row in guess_rows)
    checks['complete_native_chain'] = len(expected_stamps) == count and expected_stamps == set(r.private) == set(r.info) == set(r.tracking) == set(r.bundles) == set(r.public)
    wheel_errors = []
    for t_ref in sorted(expected_stamps):
        for offset in (-20_000_000, -10_000_000):
            wheel_stamp = t_ref + offset
            wheel, diag = r.wheel.get(wheel_stamp), r.wheel_diag.get(wheel_stamp)
            if not wheel or not diag:
                wheel_errors.append({'stamp_ns': wheel_stamp, 'error': 'MISSING_WHEEL_OR_DIAGNOSTIC'})
                continue
            pose = np.asarray(wheel['pose'])
            valid = (wheel['parent'] == 'wheel_odom' and wheel['child'] == 'axle_link'
                     and diag.get('time_valid') is True and diag.get('stamp_ns') == wheel_stamp
                     and diag.get('stream_epoch') == 'SYNTHETIC_WHEEL_BOOT'
                     and diag.get('fixture') == 'not_real_encoder_feedback'
                     and diag.get('time_source') == 'external_common_time'
                     and np.allclose(pose[:3, :3], np.eye(3), atol=1e-12)
                     and abs(pose[1, 3]) <= 1e-12 and abs(pose[2, 3]) <= 1e-12
                     and abs(wheel['angular_z']) <= 1e-12 and abs(wheel['linear_x']) <= .2)
            if not valid:wheel_errors.append({'stamp_ns': wheel_stamp, 'error': 'WHEEL_FIXTURE_CONTRACT'})
    checks['explicit_synthetic_diffdrive_wheel_inputs'] = not wheel_errors and len(r.wheel) == len(r.wheel_diag) == count * 2
    dual_errors = []
    for stamp_ns, tracking in r.tracking.items():
        bundle = r.bundles.get(stamp_ns)
        if not bundle:
            dual_errors.append({'stamp_ns': stamp_ns, 'error': 'BUNDLE_MISSING'});continue
        for side in ('left', 'right'):
            candidates = [value for key, value in r.filtered.items() if key.startswith(side + ':') and value['stamp_ns'] == bundle[side + '_time_ns']]
            if len(candidates) != 1 or tracking[side] <= 0 or tracking[side] != bundle[side] or bundle[side] != candidates[0]['count']:
                dual_errors.append({'stamp_ns': stamp_ns, 'side': side, 'error': 'FILTERED_SOLVER_CONTRIBUTION_MISMATCH'})
        if not tracking['accepted'] or not bundle['dual_valid'] or r.info.get(stamp_ns, {}).get('lost', True):
            dual_errors.append({'stamp_ns': stamp_ns, 'error': 'DUAL_TRACKING_NOT_ACCEPTED'})
        if bundle['source_codes'].get(1, 0) != bundle['left'] or bundle['source_codes'].get(2, 0) != bundle['right']:
            dual_errors.append({'stamp_ns': stamp_ns, 'error': 'CLOUD_SOURCE_LABEL_MISMATCH'})
    checks['both_filtered_sources_reach_same_native_tracking'] = not dual_errors and len(r.tracking) == count
    checks['private_wheel_tf_absent_from_public_tf'] = r.public_edges.get('wheel_odom->rig_link', 0) == 0 and not any('wheel_odom' in edge for edge in r.public_edges)
    relay_errors = [stamp_ns for stamp_ns, tracking in r.tracking.items()
                    if stamp_ns not in r.public or not np.allclose(r.public[stamp_ns], tracking['native_pose'], rtol=0, atol=1e-10)]
    checks['public_tf_is_native_pose_relay'] = not relay_errors and len(r.public) == count
    observed_native = [e for snapshot in r.endpoint_snapshots for e in snapshot['/wc_mapping/icp/odom_info']]
    private_attribution = check_private_tf_attribution(r.private_tf_evidence, r.private,
        session_id=config['session_id'], expected_count=count)
    if r.private_tf_evidence_error:
        private_attribution['errors'].append(r.private_tf_evidence_error)
        private_attribution['status'] = 'FAIL'
    checks['native_and_private_endpoint_identity'] = bool(observed_native) and \
        all(e['node'] == 'wc_icp_odometry' for e in observed_native) and private_attribution['status'] == 'PASS'
    checks['observer_has_no_callback_errors'] = not r.errors
    return {'schema_version': 1, 'test': 'synthetic_filtered_dual_native_wheel_guess',
            'status': 'PASS' if all(checks.values()) else 'FAIL', 'session_id': config['session_id'],
            'source_mode': 'synthetic', 'real_encoder_verified': False, 'vendor_filter_quality_verified': False,
            'weighted_wheel_constraint_verified': False, 'checks': checks,
            'expected_bundles': count, 'message_counts': dict(r.counts),
            'unique_counts': {name: len(getattr(r, name)) for name in ('raw', 'filtered', 'wheel', 'wheel_diag', 'private', 'info', 'tracking', 'bundles', 'public')},
            'guess_numerical_tolerance': {'translation_m': 1e-5, 'rotation_rad': 1e-5, 'meaning': 'float representation check, not physical accuracy'},
            'guess_comparisons': guess_rows, 'source_comparisons': source_rows,
            'errors': r.errors, 'source_errors': source_errors, 'wheel_errors': wheel_errors,
            'dual_errors': dual_errors, 'public_relay_mismatched_stamps': relay_errors,
            'public_tf_edges': dict(r.public_edges), 'endpoint_snapshots': r.endpoint_snapshots,
            'private_tf_attribution': private_attribution,
            'private_tf_evidence': {'path': r.private_tf_evidence_path, 'sha256': r.private_tf_evidence_sha256}}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session-config', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--duration', type=float, default=100.)
    parser.add_argument('--private-tf-evidence', type=Path,
                        help='Native tf_ownership_probe JSON from wheel_private_mode=true; required for sender attribution PASS')
    args, ros_args = parser.parse_known_args(argv)
    if not 1 <= args.duration <= 600:
        raise ValueError('bounded observer duration required')
    config = json.loads(args.session_config.read_text(encoding='utf-8'))
    if config.get('source_mode') != 'synthetic' or not config.get('synthetic', {}).get('with_wheel_filter_fixture'):
        raise ValueError('observer requires explicitly prepared synthetic wheel/filter fixture')
    if args.output.exists():
        raise FileExistsError('will not overwrite existing evidence')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    recorder = Recorder(config)
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
    from tf2_msgs.msg import TFMessage
    from rtabmap_msgs.msg import OdomInfo
    from wc_interfaces.msg import SourceFrame, TrackingAssessment, FusionBundle
    rclpy.init(args=ros_args)
    node = rclpy.create_node('wc_wheel_guess_observer')
    reliable = QoSProfile(depth=256, reliability=ReliabilityPolicy.RELIABLE)
    best_effort = QoSProfile(depth=512, reliability=ReliabilityPolicy.BEST_EFFORT)
    durable = QoSProfile(depth=256, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    subscriptions = []
    for side in ('left', 'right'):
        subscriptions.append(node.create_subscription(SourceFrame, '/wc_mapping/lidar_' + side + '/source_frame',
                             recorder.callback('raw_' + side, recorder.source), reliable))
        subscriptions.append(node.create_subscription(SourceFrame, '/wc_mapping/lidar_' + side + '/source_frame_filtered',
                             recorder.callback('filtered_' + side, lambda msg: recorder.source(msg, True)), reliable))
    for topic, name, kind, handler, qos in (
        ('/wc_mapping/wheel/odom', 'wheel', Odometry, recorder.wheel_message, reliable),
        ('/wc_mapping/wheel/diagnostics', 'wheel_diag', String, recorder.wheel_diagnostic, reliable),
        ('/wc_mapping/icp_guess_tf', 'private_tf', TFMessage, recorder.private_tf, reliable),
        ('/wc_mapping/icp/odom_info', 'info', OdomInfo, recorder.odom_info, best_effort),
        ('/wc_mapping/tracking/status', 'tracking', TrackingAssessment, recorder.assessment, reliable),
        ('/wc_mapping/fusion/bundle', 'bundle', FusionBundle, recorder.bundle, reliable),
        ('/tf', 'public_tf', TFMessage, recorder.public_tf, best_effort),
        ('/tf_static', 'public_static_tf', TFMessage, recorder.public_tf, durable),
    ):
        subscriptions.append(node.create_subscription(kind, topic, recorder.callback(name, handler), qos))
    print('WHEEL_GUESS_OBSERVER_READY ' + config['session_id'], flush=True)
    deadline, next_discovery = time.monotonic() + args.duration, 0.
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.02)
            if time.monotonic() >= next_discovery:
                recorder.discover(node)
                next_discovery = time.monotonic() + .5
    except KeyboardInterrupt:
        recorder.errors.append({'error': 'OBSERVER_INTERRUPTED'})
    finally:
        recorder.discover(node)
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
    if args.private_tf_evidence:
        recorder.private_tf_evidence_path = str(args.private_tf_evidence)
        try:
            evidence_path = args.private_tf_evidence
            if not evidence_path.is_absolute() or any(part == '..' for part in evidence_path.parts):
                raise ValueError('native private evidence path must be absolute without traversal')
            if any(part.is_symlink() for part in (evidence_path, *evidence_path.parents)):
                raise ValueError('native private evidence path must not traverse symlinks')
            if evidence_path.stat().st_size > 64 * 1024 * 1024:
                raise ValueError('native private evidence exceeds 64MiB')
            payload = evidence_path.read_bytes()
            recorder.private_tf_evidence_sha256 = hashlib.sha256(payload).hexdigest()
            recorder.private_tf_evidence = json.loads(payload)
        except (OSError, ValueError, TypeError) as error:
            recorder.private_tf_evidence_error = 'PRIVATE_EVIDENCE_READ_FAILED:' + str(error)
    report = build_report(recorder)
    report['session_config_sha256'] = hashlib.sha256(args.session_config.read_bytes()).hexdigest()
    report['observer_source_sha256'] = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    with args.output.open('x', encoding='utf-8') as output:
        json.dump(report, output, indent=2, allow_nan=False)
    print(json.dumps({'status': report['status'], 'checks': report['checks'], 'output': str(args.output)}), flush=True)
    return 0 if report['status'] == 'PASS' else 1


if __name__ == '__main__':
    raise SystemExit(main())
