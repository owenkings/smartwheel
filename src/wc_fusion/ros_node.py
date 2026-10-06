"""ROS 2 bridge: source envelopes -> one ICP input -> accepted dual graph input.

RTAB-Map computes odom->rig_link. In wheel mode this node relays that native
pose to public TF; wheel guesses stay on a private TF topic.
An exact nanosecond stamp indexes a locally unique in-flight bundle. Odom and
OdomInfo must both match before acceptance; no rounded-time lookup is used.
"""
import argparse
from collections import deque, OrderedDict
import json
from pathlib import Path
import time
import uuid

import numpy as np
from scipy.spatial.transform import Rotation

from .core import DualFusion, FusionError, Gate, Policy
from .archive import DurableArchive, ArchiveError
from .cache import LatestCacheWriter
from .graph_delivery import GraphDelivery, GraphDeliveryError
from wc_sensors.pointcloud import decode_pointcloud2


def stamp_ns(stamp):
    if not 0 <= stamp.nanosec < 1_000_000_000:
        raise FusionError('NONCANONICAL_ROS_STAMP')
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def pose_matrix(pose):
    quaternion = np.array([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
    if not np.isfinite(quaternion).all() or abs(np.linalg.norm(quaternion)-1) > 1e-4:
        raise FusionError('INVALID_ODOM_QUATERNION')
    result = np.eye(4)
    result[:3, :3] = Rotation.from_quat(quaternion).as_matrix()
    result[:3, 3] = [pose.position.x, pose.position.y, pose.position.z]
    return result


def assign_transform(message, matrix):
    message.translation.x, message.translation.y, message.translation.z = map(float, matrix[:3, 3])
    q = Rotation.from_matrix(matrix[:3, :3]).as_quat()
    message.rotation.x, message.rotation.y, message.rotation.z, message.rotation.w = map(float, q)


def cloud_message(points, stamp, frame_id='rig_link'):
    from sensor_msgs.msg import PointCloud2, PointField
    msg = PointCloud2()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height, msg.width = 1, len(points)
    msg.fields = [PointField(name=axis, offset=4*i, datatype=PointField.FLOAT32, count=1)
                  for i, axis in enumerate(('x', 'y', 'z'))]
    msg.is_bigendian = False
    msg.point_step, msg.row_step = 12, 12*len(points)
    msg.data = np.asarray(points, dtype='<f4').tobytes()
    msg.is_dense = bool(np.isfinite(points).all())
    return msg


def json_value(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, (np.integer, np.floating)):
        return value.item()
    raise TypeError(type(value).__name__)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session-config', required=True, type=Path)
    parser.add_argument('--graph-ack-required', action='store_true', help='Require native graph processing acknowledgements; full map launch enables this')
    args, ros_args = parser.parse_known_args(argv)
    config = json.loads(args.session_config.read_text())
    source_mode = config['source_mode']
    session_id = config['session_id']
    root = Path(config['session_root']).resolve()
    if not root.is_relative_to(Path('/home/nvidia/wheelchair').resolve()):
        raise FusionError('SESSION_PATH_OUTSIDE_PROJECT')
    root.mkdir(parents=True, exist_ok=True)
    raw_dir = root / 'raw_observations'
    raw_dir.mkdir(exist_ok=True)
    policy = Policy(**config.get('fusion_policy', {}))
    fusion = DualFusion(session_id, config['sensor_ids'], config['calibration'],
                        config['clock_model_ids'], policy=policy, source_mode=source_mode,
                        quality_profile=config.get('quality_profile'))
    wheel_config = config.get('motion_prior', {})
    if wheel_config.get('enabled'):
        from .wheel_prior import WheelMotionHistory
        fusion.history = WheelMotionHistory(policy, wheel_config, source_mode)
    elif source_mode == 'real' and config.get('encoder_required', True):
        raise FusionError('ENCODER_REQUIRED_CONFIGURE_MOTION_PRIOR')
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
    from sensor_msgs.msg import PointCloud2
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
    from geometry_msgs.msg import TransformStamped
    from tf2_msgs.msg import TFMessage
    from rtabmap_msgs.msg import OdomInfo
    from wc_interfaces.msg import SourceFrame, FusionBundle, TrackingAssessment, AcceptedBundle, MapStatus, GraphSnapshot
    from rclpy.serialization import serialize_message, deserialize_message
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
    from std_srvs.srv import Trigger
    from wc_interfaces.srv import Resume

    class FusionNode(Node):
        def __init__(self):
            super().__init__('wc_dual_fusion', parameter_overrides=[rclpy.parameter.Parameter(
                'use_sim_time', value=config.get('execution_mode') == 'replay')])
            self.fusion = fusion
            self.gate = Gate(session_id, max_pending=policy.max_queue,
                             timeout_ns=config.get('icp_timeout_ns', 2_000_000_000),
                             max_speed_mps=min(policy.max_motion_speed_mps, policy.velocity_bound_mps),
                             max_angular_radps=min(policy.max_motion_angular_radps, policy.angular_bound_radps))
            self.odom_epoch = str(uuid.uuid4())
            self.by_stamp = {}
            self.archive = DurableArchive(capacity=policy.max_queue)
            self.cache = LatestCacheWriter([root/'frontend_status.json', root/'last_tracking.json'])
            self.graph_delivery = GraphDelivery(session_id,source_mode,fusion.calibration_id,config['clock_model_ids'],
                graph_epoch=config.get('graph_epoch')) if args.graph_ack_required else None
            self.archive_pending = {}
            self.icp_ready = deque()
            self.source_callback_seconds = deque(maxlen=256)
            self.tracking_callback_seconds = deque(maxlen=256)
            self.watchdog_callback_seconds = deque(maxlen=256)
            self.source_delivery_age_seconds = deque(maxlen=256)
            self.first_failure = None
            self.runtime_stage_seconds = {}
            self.archive_timings = deque(maxlen=64)
            self.odom_cache, self.info_cache = {}, {}
            self.last_observations = {}
            self.stable_observations = {'left': deque(maxlen=5), 'right': deque(maxlen=5)}
            self.intentional_pause = False
            self.wheel_bridge = None
            self.wheel_join = OrderedDict()
            self.wheel_last_stamp = 0
            self.wheel_guesses = {}
            self.native_publisher_gid = None
            durable = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.bundle_pub = self.create_publisher(FusionBundle, '/wc_mapping/fusion/bundle', 4)
            estimator_qos = QoSProfile(depth=8, reliability=ReliabilityPolicy.RELIABLE)
            self.cloud_pub = self.create_publisher(PointCloud2, '/wc_mapping/fusion/points', estimator_qos)
            self.accepted_pub = self.create_publisher(AcceptedBundle, '/wc_mapping/accepted', 8)
            self.assessment_pub = self.create_publisher(TrackingAssessment, '/wc_mapping/tracking/status', 4)
            # Overall map status belongs to the graph/map coordinator. This is
            # front-end status only, so there are no competing publishers.
            self.status_pub = self.create_publisher(MapStatus, '/wc_mapping/frontend/status', durable)
            if self.graph_delivery is not None:
                self.create_subscription(GraphSnapshot, '/wc_mapping/graph/snapshot', self.graph_ack, durable)
            self.diagnostics_pub = self.create_publisher(DiagnosticArray, '/wc_mapping/diagnostics', 4)
            source_qos = estimator_qos if (source_mode == 'synthetic' and
                config.get('synthetic_source_reliability') == 'reliable') else qos_profile_sensor_data
            from .source_join import RawFilteredJoin
            self.filtered_input = config.get('cloud_input', 'xtcfg_filtered' if source_mode == 'real' else 'raw') == 'xtcfg_filtered'
            if config.get('cloud_input', 'raw') not in ('raw', 'xtcfg_filtered'):
                raise FusionError('UNKNOWN_CLOUD_INPUT')
            self.source_join = RawFilteredJoin(capacity=policy.max_queue*4, timeout_ns=policy.max_age_ns)
            for side in ('left', 'right'):
                if self.filtered_input:
                    self.create_subscription(SourceFrame, '/wc_mapping/lidar_'+side+'/source_frame',
                        lambda msg: self.source_branch('raw', msg), source_qos)
                    self.create_subscription(SourceFrame, '/wc_mapping/lidar_'+side+'/source_frame_filtered',
                        lambda msg: self.source_branch('filtered', msg), source_qos)
                else:
                    self.create_subscription(SourceFrame, '/wc_mapping/lidar_'+side+'/source_frame', self.source, source_qos)
            icp_qos=estimator_qos
            self.create_subscription(Odometry, '/wc_mapping/odom', self.odom, icp_qos)
            self.create_subscription(OdomInfo, '/wc_mapping/icp/odom_info', self.info, icp_qos)
            if wheel_config.get('enabled'):
                from .wheel_tf import WheelTfBridge
                self.wheel_bridge = WheelTfBridge(policy, wheel_config, source_mode)
                self.private_tf_pub = self.create_publisher(TFMessage, self.wheel_bridge.private_tf_topic, 100)
                self.public_tf_pub = self.create_publisher(TFMessage, '/tf', 100)
                self.create_subscription(Odometry, '/wc_mapping/wheel/odom', self.wheel_odom, estimator_qos)
                self.create_subscription(String, '/wc_mapping/wheel/diagnostics', self.wheel_diagnostic, estimator_qos)
            self.create_timer(0.1, self.watchdog)
            self.create_service(Trigger, '/wc_mapping/pause_frontend', self.pause)
            self.create_service(Resume, '/wc_mapping/resume_frontend', self.resume)

        def fail(self, reason):
            if self.first_failure is None:
                now = time.monotonic_ns()
                self.first_failure = dict(reason=str(reason), monotonic_ns=now,
                    last_seen_age_s={side:(now-value)/1e9 for side,value in fusion.last_seen_monotonic_ns.items()},
                    source_callback_seconds=list(self.source_callback_seconds),
                    tracking_callback_seconds=list(self.tracking_callback_seconds),
                    watchdog_callback_seconds=list(self.watchdog_callback_seconds),
                    source_delivery_age_seconds=list(self.source_delivery_age_seconds),
                    runtime_stage_seconds={k:dict(v) for k,v in self.runtime_stage_seconds.items()})
                self.first_failure['archive_timings'] = list(self.archive_timings)
                self.first_failure['graph_delivery'] = self.graph_delivery.status() if self.graph_delivery else None
                self.first_failure['icp_ready_count'] = len(self.icp_ready)
            self.fusion.latch(reason)
            self.gate.pause(reason)
            self.by_stamp.clear()
            self.archive_pending.clear()
            self.icp_ready.clear()
            self.odom_cache.clear()
            self.info_cache.clear()
            self.get_logger().error(reason)

        def source_branch(self, kind, msg):
            try:
                pair = self.source_join.push(kind, msg, time.monotonic_ns())
                if pair is not None:
                    self.source(pair[1], raw_message=pair[0])
            except (ValueError, TypeError) as exc:
                self.fail(str(exc))

        def source(self, msg, raw_message=None):
            now = time.monotonic_ns()
            if config.get('execution_mode') != 'replay':
                self.source_delivery_age_seconds.append((now-int(msg.host_monotonic_ns))/1e9)
            if msg.side not in ('left','right'):
                self.fail('SOURCE_SIDE_INVALID')
                return
            if source_mode == 'real' and config.get('time_models'):
                from wc_sensors.clock_adapter import apply_clock_model
                try:
                    msg = apply_clock_model(msg, config['time_models'][msg.side],
                                            config['sensor_ids'][msg.side], config['clock_model_ids'][msg.side])
                except (ValueError,KeyError) as exc:
                    self.fail('CLOCK_MODEL_REJECTED:'+str(exc))
                    return
            self.last_observations[msg.side] = (now, msg)
            if msg.side in self.stable_observations:
                history = self.stable_observations[msg.side]
                key = (msg.stream_epoch, msg.frame_sequence, msg.common_time_ns)
                expected_time_source = 'synthetic' if source_mode == 'synthetic' else 'validated_model'
                valid = (msg.session_id == session_id and msg.sensor_id == config['sensor_ids'][msg.side]
                         and msg.clock_model_id == config['clock_model_ids'][msg.side]
                         and msg.common_time_valid and msg.uncertainty_valid
                         and msg.coordinate_convention == 'FLU' and msg.units == 'm'
                         and msg.time_source == expected_time_source
                         and (config.get('execution_mode') == 'replay' or 0 <= now-msg.host_monotonic_ns <= policy.max_age_ns))
                increasing = not history or (key[0] == history[-1][1][0]
                    and key[1] > history[-1][1][1] and key[2] > history[-1][1][2])
                if not valid or not increasing:
                    history.clear()
                if valid and increasing:
                    history.append((now, key, True))
            if self.intentional_pause or self.fusion.latched_reason:
                return
            callback_started = time.monotonic()
            try:
                # Historical monotonic values belong to the recorded boot.
                # Replay freshness measures current delivery, while the full
                # SourceFrame retains the recorded value in the bag.
                received = now if config.get('execution_mode') == 'replay' else int(msg.host_monotonic_ns)
                frame = dict(session_id=msg.session_id, side=msg.side, sensor_id=msg.sensor_id,
                             stream_epoch=msg.stream_epoch, frame_sequence=int(msg.frame_sequence),
                             points=decode_pointcloud2(msg.cloud), frame_id=msg.cloud.header.frame_id,
                             common_time_ns=int(msg.common_time_ns), clock_model_id=msg.clock_model_id,
                             time_valid=msg.common_time_valid,
                             uncertainty_ns=int(msg.uncertainty_ns) if msg.uncertainty_valid else None,
                             time_source=msg.time_source, host_monotonic_ns=received,
                             source_config_hash=msg.source_config_hash)
                if raw_message is not None:
                    frame.update(representation='host_filtered', original_points=decode_pointcloud2(raw_message.cloud),
                                 original_config_hash=raw_message.source_config_hash,
                                 filter_config_hash=msg.source_config_hash)
                if msg.coordinate_convention != 'FLU' or msg.units != 'm':
                    raise FusionError('AXIS_OR_UNITS_UNVERIFIED')
                b = fusion.push(frame, now)
                if b is None:
                    return
                if self.wheel_bridge is not None:
                    self.wheel_guesses[b['t_ref_ns']] = self.wheel_bridge.prepare_guess(b['t_ref_ns'],
                        now_ns=self.get_clock().now().nanoseconds, now_monotonic_ns=now)
                stamp = msg.header.stamp.__class__()
                stamp.sec, stamp.nanosec = divmod(b['t_ref_ns'], 1_000_000_000)
                if not -(2**31) <= stamp.sec < 2**31:
                    raise FusionError('ROS_STAMP_RANGE')
                if (b['t_ref_ns'] in self.by_stamp or
                        any(item[0]['t_ref_ns'] == b['t_ref_ns'] for item in self.archive_pending.values()) or
                        any(item[0]['t_ref_ns'] == b['t_ref_ns'] for item in self.icp_ready)):
                    raise FusionError('AMBIGUOUS_BUNDLE_STAMP')
                if len(self.archive_pending)+len(self.icp_ready)+len(self.by_stamp) >= policy.max_queue:
                    raise FusionError('ICP_ARCHIVE_PIPELINE_FULL')
                output = FusionBundle()
                output.header.stamp, output.header.frame_id = stamp, 'rig_link'
                output.session_id, output.bundle_id = session_id, b['bundle_id']
                output.source_mode, output.sensor_mode = source_mode, 'dual'
                output.calibration_id = b['calibration_id']
                output.left_clock_model_id = b['clock_model_ids']['left']
                output.right_clock_model_id = b['clock_model_ids']['right']
                for side, observation in zip(('left', 'right'), b['observations']):
                    setattr(output, side+'_raw_key', observation['raw_key'])
                    setattr(output, side+'_time_ns', observation['stamp_ns'])
                    assign_transform(getattr(output, 't_ref_'+side+'_sensor'), observation['T_ref_sensor'])
                    assign_transform(getattr(output, 't_ref_rig_at_'+side+'_time'), observation['T_ref_rig_at_source'])
                    setattr(output, side+'_retained_count', b['source_counts'][side])
                output.compensation_mode = b['compensation_mode']
                output.temporal_error_bound_valid = True
                output.temporal_error_bound_m = b['temporal_error_bound_m']
                output.source_codes = b['source_codes'].tolist()
                output.cloud = cloud_message(b['points'], stamp)
                output.calibration_valid = output.time_valid = output.dual_valid = True
                # Only valid ray returns go into mapping sidecars; complete
                # unfiltered SourceFrames remain in the separate raw recording.
                observations = [{k:v for k,v in o.items() if k != 'raw_points'} for o in b['observations']]
                payload = json.dumps(dict(
                    schema_version=1, session_id=session_id, source_mode=source_mode,
                    sensor_mode='dual', bundle_id=b['bundle_id'], t_ref_ns=b['t_ref_ns'],
                    calibration_id=b['calibration_id'], time_model_id=config['time_model_id'],
                    observations=observations), default=json_value, allow_nan=False).encode('utf-8')
                key = str(b['bundle_id'])
                self.archive.submit(key, raw_dir / f"{b['bundle_id']}.json", payload)
                self.archive_pending[key] = (b, output, now)
            except (ValueError, TypeError, OSError, ArchiveError) as exc:
                self.fail(str(exc))
            finally:
                self.source_callback_seconds.append(time.monotonic()-callback_started)

        def measured(self, stage, call):
            started = time.monotonic()
            try:
                return call()
            finally:
                elapsed = time.monotonic()-started
                row = self.runtime_stage_seconds.setdefault(stage, dict(count=0, last=0.0, max=0.0))
                row.update(count=row['count']+1, last=elapsed, max=max(row['max'], elapsed))

        def cache_json(self, path, data):
            self.cache.submit(path, json.dumps(data, default=json_value, allow_nan=False).encode('utf-8'))

        def graph_ack(self, message):
            try:
                if self.graph_delivery.failure is None:
                    self.graph_delivery.acknowledge(message)
            except GraphDeliveryError as error:
                self.fail(str(error))

        def deliver_graph(self, now):
            if self.graph_delivery is not None and self.graph_delivery.failure is None:
                try:
                    payload = self.graph_delivery.next_payload(now)
                    if payload is not None:
                        self.measured('accepted_graph_publish', lambda: self.accepted_pub.publish(deserialize_message(payload, AcceptedBundle)))
                except (GraphDeliveryError,ValueError,TypeError) as error:
                    self.fail(str(error))

        def publish_archived(self, now):
            # Only fully persisted raw observations may enter the estimator.
            # Poll runs on this node's executor, so publication and exact-stamp
            # joins never race a ROS callback or a quality-gate transition.
            for result in self.archive.poll():
                self.archive_timings.append(dict(bundle_id=result.key, timing_ns=dict(result.timing_ns), error=result.error))
                record = self.archive_pending.pop(result.key, None)
                if result.error is not None:
                    self.fail('RAW_ARCHIVE_FAILED:' + str(result.error))
                    continue
                if record is None or self.intentional_pause or fusion.latched_reason:
                    continue
                self.icp_ready.append(record)
            # Native odometry has an asynchronous processing stage: reliable
            # DDS does not guarantee it processes a burst released by fsync.
            # One exact-stamp result pair releases the next durable FIFO entry.
            # Never resend an ICP observation or silently skip an expired one.
            if any(not 0 <= now-item[2] <= policy.max_age_ns for item in self.icp_ready):
                self.fail('RAW_ARCHIVE_AGE_EXCEEDED')
                return
            if self.by_stamp or not self.icp_ready:
                return
            b, output, submitted = self.icp_ready.popleft()
            try:
                self.gate.submit(b, now)
                self.by_stamp[b['t_ref_ns']] = (b, output)
                if self.wheel_bridge is not None:
                    from .wheel_tf import assign_transform as assign_wheel_transform
                    guess = self.wheel_bridge.register_guess(self.wheel_guesses.pop(b['t_ref_ns']))
                    self.private_tf_pub.publish(TFMessage(transforms=[assign_wheel_transform(guess, TransformStamped())]))
                self.measured('bundle_publish', lambda: self.bundle_pub.publish(output))
                self.measured('cloud_publish', lambda: self.cloud_pub.publish(output.cloud))
            except (ValueError, TypeError, OSError) as exc:
                self.fail(str(exc))

        def wheel_odom(self, msg):
            try:
                self.wheel_pair('odom', stamp_ns(msg.header.stamp), msg)
            except (ValueError, TypeError) as exc:
                self.fail(str(exc))

        def wheel_diagnostic(self, msg):
            try:
                data = json.loads(msg.data)
                if data.get('state') in ('STALE', 'BLOCKED') and self.wheel_last_stamp:
                    raise FusionError('WHEEL_FEEDBACK_'+data['state'])
                if data.get('stamp_ns') is not None and data.get('state') == 'FEEDBACK':
                    self.wheel_pair('diagnostic', data['stamp_ns'], data)
            except (ValueError, TypeError) as exc:
                self.fail(str(exc))

        def wheel_pair(self, kind, stamp, value):
            if stamp <= self.wheel_last_stamp or self.fusion.latched_reason:
                return
            now = time.monotonic_ns()
            if len(self.wheel_join) >= 32 and stamp not in self.wheel_join:
                raise FusionError('WHEEL_DIAGNOSTIC_JOIN_FULL')
            row = self.wheel_join.setdefault(stamp, {'received': now})
            row[kind] = value
            if 'odom' not in row or 'diagnostic' not in row:
                return
            self.wheel_bridge.observe_wheel(row['odom'], row['diagnostic'],
                now_ns=self.get_clock().now().nanoseconds, now_monotonic_ns=now)
            fusion.history.add_wheel(stamp, pose_matrix(row['odom'].pose.pose), row['diagnostic']['stream_epoch'])
            self.wheel_last_stamp = stamp
            del self.wheel_join[stamp]

        def odom(self, msg, metadata=None):
            key = stamp_ns(msg.header.stamp)
            if key in self.by_stamp:
                if self.wheel_bridge is not None:
                    # This is endpoint identity checking within the isolated
                    # project domain, not DDS cryptographic authentication.
                    endpoints = self.get_publishers_info_by_topic('/wc_mapping/odom')
                    if len(endpoints) != 1 or endpoints[0].node_name != 'wc_icp_odometry':
                        self.fail('NATIVE_ICP_PUBLISHER_NOT_UNIQUE');return
                    identity = bytes(endpoints[0].endpoint_gid)
                    observed_value = getattr(metadata, 'publisher_gid', None)
                    observed = bytes(observed_value) if observed_value is not None else None
                    if (observed is not None and identity != observed) or (self.native_publisher_gid is not None and identity != self.native_publisher_gid):
                        self.fail('NATIVE_ICP_PUBLISHER_CHANGED');return
                    self.native_publisher_gid = identity
                self.odom_cache[key] = msg
                self.complete(key)

        def info(self, msg):
            key = stamp_ns(msg.header.stamp)
            if key in self.by_stamp:
                self.info_cache[key] = msg
                self.complete(key)

        def complete(self, key):
            if key not in self.odom_cache or key not in self.info_cache:
                return
            now = time.monotonic_ns()
            odom, info = self.odom_cache.pop(key), self.info_cache.pop(key)
            b, ros_bundle = self.by_stamp[key]
            callback_started = time.monotonic()
            try:
                if odom.header.frame_id != 'odom' or odom.child_frame_id != 'rig_link':
                    raise FusionError('ODOM_FRAME_MISMATCH')
                pose = pose_matrix(odom.pose.pose)
                # Available native fields were verified on RTAB-Map ROS 0.23.7.
                initialized = self.gate.accepted == 0
                has_metrics = info.icp_correspondences > 0
                if has_metrics and (not np.isfinite(info.icp_inliers_ratio)
                    or not 0 <= info.icp_inliers_ratio <= 1
                    or not np.isfinite(info.icp_structural_complexity)
                    or info.icp_structural_complexity < 0):
                    raise FusionError('INVALID_NATIVE_ICP_DIAGNOSTICS')
                native_valid = not info.lost and (initialized or info.icp_correspondences >= config.get('min_icp_correspondences', 20))
                if not initialized and has_metrics and info.icp_structural_complexity < config.get('min_icp_structural_complexity', 0.02):
                    native_valid = False
                result = dict(session_id=session_id, bundle_id=b['bundle_id'], stamp_ns=key,
                              odom_epoch=self.odom_epoch, tracking_valid=native_valid,
                              T_odom_rig=pose, actual_source_participation=b['source_counts'],
                              residual=None, inliers=int(info.icp_correspondences) if has_metrics else None,
                              structural_complexity=float(info.icp_structural_complexity) if has_metrics else None,
                              inliers_ratio=float(info.icp_inliers_ratio) if has_metrics else None)
                if self.graph_delivery is not None:
                    self.graph_delivery.ensure_capacity()
                accepted = self.gate.assess(result, now)
                if self.wheel_bridge is not None:
                    from .wheel_tf import assign_transform as assign_wheel_transform
                    native_pose = self.wheel_bridge.relay_native_odometry(odom)
                    self.public_tf_pub.publish(TFMessage(transforms=[assign_wheel_transform(native_pose, TransformStamped())]))
                fusion.history.add(key, pose, self.odom_epoch)
                assessment = TrackingAssessment()
                assessment.header = ros_bundle.header
                assessment.session_id, assessment.bundle_id = session_id, b['bundle_id']
                assessment.odom_epoch, assessment.tracking_state = self.odom_epoch, 'INITIALIZED' if initialized else 'TRACKING'
                assessment.accepted = True
                assessment.left_solver_input_count, assessment.right_solver_input_count = b['source_counts']['left'], b['source_counts']['right']
                assessment.inliers_available = has_metrics
                assessment.inliers = max(0, info.icp_correspondences)
                assessment.residual_available = False
                assessment.degeneracy_available = has_metrics
                assessment.degeneracy_metric = float(info.icp_structural_complexity) if has_metrics else 0.0
                assessment.odometry = odom
                # Operational cache only. Accepted poses are persisted by the
                # graph owner; this file is never a restart/continuity authority.
                self.cache_json(root / 'last_tracking.json', result)
                self.assessment_pub.publish(assessment)
                envelope = AcceptedBundle(bundle=ros_bundle, tracking=assessment)
                if self.graph_delivery is None:
                    self.accepted_pub.publish(envelope)
                else:
                    self.graph_delivery.submit(int(b['bundle_id']), (ros_bundle.left_raw_key,ros_bundle.right_raw_key),
                                               self.odom_epoch, bytes(serialize_message(envelope)))
                    self.deliver_graph(now)
                self.by_stamp.pop(key, None)
            except (ValueError, TypeError, OSError) as exc:
                self.fail(str(exc))
            finally:
                self.tracking_callback_seconds.append(time.monotonic()-callback_started)

        def watchdog(self):
            callback_started = time.monotonic()
            now = time.monotonic_ns()
            if not self.intentional_pause and not fusion.latched_reason:
                try:
                    self.source_join.expire(now)
                    if any(now-row['received'] > policy.max_age_ns for row in self.wheel_join.values()):
                        raise FusionError('WHEEL_DIAGNOSTIC_JOIN_TIMEOUT')
                except ValueError as exc:
                    self.fail(str(exc))
            if not self.intentional_pause:
                fresh = fusion.check_freshness(now)
                tracked = self.gate.expire(now)
                if not fresh and not self.gate.paused_reason:
                    self.fail(fusion.latched_reason)
                if not tracked and not fusion.latched_reason:
                    self.fail(self.gate.paused_reason)
            self.publish_archived(now)
            # Drain observations accepted before an input pause. Their original
            # measurement stamps never change; delivery cannot validate new data.
            self.deliver_graph(time.monotonic_ns())
            status = MapStatus()
            status.header.stamp = self.get_clock().now().to_msg()
            status.header.frame_id = 'rig_link'
            status.session_id, status.source_mode, status.sensor_mode = session_id, source_mode, 'dual'
            reasons = [x for x in (fusion.latched_reason, self.gate.paused_reason) if x]
            status.state = 'PAUSED_INVALID' if reasons else ('MAPPING_DUAL' if self.gate.accepted else 'PREFLIGHT')
            status.pause_reasons = list(dict.fromkeys(reasons))
            status.left_fresh = now - self.last_observations.get('left', (0,None))[0] <= policy.max_age_ns
            status.right_fresh = now - self.last_observations.get('right', (0,None))[0] <= policy.max_age_ns
            status.calibration_valid = not bool(reasons)
            status.time_valid = status.left_fresh and status.right_fresh and not bool(reasons)
            status.tracking_valid = self.gate.accepted > 0 and not reasons
            status.latest_accepted_time_ns = self.gate.last_stamp_ns or 0
            self.measured('frontend_status_publish', lambda: self.status_pub.publish(status))
            diag = DiagnosticStatus(name='dual_fusion', hardware_id='|'.join(config['sensor_ids'].values()),
                                    level=DiagnosticStatus.ERROR if reasons else DiagnosticStatus.OK,
                                    message=status.state)
            diag.values = [KeyValue(key=k, value=str(v)) for k,v in dict(
                accepted=self.gate.accepted, left_queue=len(fusion.queues['left']),
                right_queue=len(fusion.queues['right']), pending=len(self.gate.pending),
                raw_archive_pending=self.archive.pending_count, raw_archive_failure=self.archive.failure,
                icp_ready=len(self.icp_ready),icp_inflight=len(self.by_stamp),
                disposable_cache_error=self.cache.error, disposable_cache_pending=self.cache.pending_count,
                graph_delivery=self.graph_delivery.status() if self.graph_delivery else 'ICP_ONLY_NO_GRAPH_ACK',
                cloud_input='xtcfg_filtered' if self.filtered_input else 'raw',
                wheel_prediction=fusion.history.status() if wheel_config.get('enabled') else 'DISABLED',
                wheel_icp_bridge=self.wheel_bridge.status() if self.wheel_bridge else 'DISABLED',
                source_mode=source_mode, reasons=status.pause_reasons).items()]
            self.measured('diagnostics_publish', lambda: self.diagnostics_pub.publish(DiagnosticArray(header=status.header, status=[diag])))
            # This file is an ephemeral UI/status cache. Durable reconstruction
            # uses the acknowledged raw files and graph database, never this
            # timer output; do not fsync the eMMC ten times per second here.
            self.measured('status_cache_submit', lambda: self.cache_json(root / 'frontend_status.json', dict(state=status.state, source_mode=source_mode,
                session_id=session_id, accepted=self.gate.accepted, pause_reasons=status.pause_reasons,
                wheel_icp_bridge=self.wheel_bridge.status() if self.wheel_bridge else None,
                raw_archive_pending=self.archive.pending_count,
                icp_ready=len(self.icp_ready),icp_inflight=len(self.by_stamp),
                source_callback_seconds=list(self.source_callback_seconds),
                tracking_callback_seconds=list(self.tracking_callback_seconds),
                watchdog_callback_seconds=list(self.watchdog_callback_seconds),
                source_delivery_age_seconds=list(self.source_delivery_age_seconds),
                first_failure=self.first_failure,
                runtime_stage_seconds={k:dict(v) for k,v in self.runtime_stage_seconds.items()},
                archive_timings=list(self.archive_timings),
                graph_delivery=self.graph_delivery.status() if self.graph_delivery else None,
                disposable_cache_error=self.cache.error,
                latest_accepted_time_ns=status.latest_accepted_time_ns)))
            self.watchdog_callback_seconds.append(time.monotonic()-callback_started)

        def pause(self, request, response):
            self.intentional_pause = True
            self.fail('EXPLICIT_PAUSE')
            response.success, response.message = True, 'Map input paused; recording remains independent.'
            return response

        def resume(self, request, response):
            # Require five independent fresh preflight timer observations and
            # the caller's explicit same-epoch continuity validation.
            now = time.monotonic_ns()
            stable = all(side in self.last_observations and now-self.last_observations[side][0] < policy.max_age_ns
                         and len(self.stable_observations[side]) == 5
                         and now-self.stable_observations[side][0][0] <= 5*policy.max_age_ns
                         and all(item[2] for item in self.stable_observations[side])
                         and len({item[1][0] for item in self.stable_observations[side]}) == 1
                         for side in ('left','right'))
            response.accepted = False
            if self.wheel_bridge is not None:
                response.reasons = ['WHEEL_PRIOR_CONTINUITY_REQUIRES_NEW_SESSION']
                return response
            if self.graph_delivery is not None and self.graph_delivery.failure:
                response.reasons = ['GRAPH_DELIVERY_FAILED_NEW_SESSION_REQUIRED']
                return response
            if request.session_id != session_id:
                response.reasons = ['SESSION_MISMATCH']
                return response
            try:
                for side in ('left','right'):
                    msg = self.last_observations[side][1]
                    if not msg.common_time_valid or not msg.uncertainty_valid or msg.sensor_id != config['sensor_ids'][side]:
                        raise FusionError('RESUME_SOURCE_UNVALIDATED')
                if request.calibration_id != fusion.calibration_id or {
                    'left': request.left_clock_model_id, 'right':request.right_clock_model_id} != fusion.clock_model_ids:
                    raise FusionError('RESUME_CONFIG_MISMATCH')
                self.gate.resume(explicit=request.explicit_request, stable=stable,
                                 continuity_verified=request.continuity_verified, odom_epoch=request.odom_epoch)
                fusion.resume(explicit=request.explicit_request, stable=stable,
                              continuity_verified=request.continuity_verified,
                              calibration_id=request.calibration_id,
                              clock_model_ids={'left':request.left_clock_model_id,'right':request.right_clock_model_id})
                self.by_stamp.clear()
                self.source_join.clear()
                self.archive_pending.clear()
                self.odom_cache.clear()
                self.info_cache.clear()
                self.intentional_pause = False
                response.accepted = True
            except (ValueError, KeyError) as exc:
                response.reasons = [str(exc)]
            return response

    rclpy.init(args=ros_args)
    node = FusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.archive.close(timeout=2.0)
        finally:
            try:
                node.cache.close(timeout=2.0)
            finally:
                node.destroy_node()
                if rclpy.ok():rclpy.shutdown()


if __name__ == '__main__':
    main()
