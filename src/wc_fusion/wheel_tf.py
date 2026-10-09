"""Private wheel TF guesses and a sole public relay of native ICP poses.

Only enabled wheel-prior mode uses this adapter. Configure native ICP with
publish_tf=false, guess_frame_id=wheel_odom, frame_id=rig_link and private /tf,
/tf_static remaps. Public odom->rig_link always copies native ICP output; this
module never derives that public pose from wheel motion.
"""

from collections import OrderedDict
from dataclasses import dataclass

import numpy as np
from scipy.spatial.transform import Rotation

from .core import FusionError, MotionHistory, integer_ns, matrix


@dataclass(frozen=True)
class TransformRecord:
    stamp_ns: int
    parent_frame: str
    child_frame: str
    transform: np.ndarray
    source: str


def positive_ns(value, name, allow_zero=False):
    value = integer_ns(value)
    if value < (0 if allow_zero else 1):
        raise FusionError(name + '_INVALID')
    return value


def ros_pose_matrix(pose):
    values = np.asarray([pose.position.x, pose.position.y, pose.position.z,
                         pose.orientation.x, pose.orientation.y, pose.orientation.z,
                         pose.orientation.w], dtype=float)
    if not np.isfinite(values).all():
        raise FusionError('TF_POSE_NONFINITE')
    if abs(np.linalg.norm(values[3:]) - 1.) > 1e-6:
        raise FusionError('TF_POSE_QUATERNION_INVALID')
    result = np.eye(4)
    result[:3, :3] = Rotation.from_quat(values[3:]).as_matrix()
    result[:3, 3] = values[:3]
    return result


def message_stamp(message):
    sec = integer_ns(message.header.stamp.sec)
    nsec = integer_ns(message.header.stamp.nanosec)
    if not -(2**31) <= sec < 2**31 or not 0 <= nsec < 1_000_000_000:
        raise FusionError('TF_ROS_STAMP_INVALID')
    return positive_ns(sec * 1_000_000_000 + nsec, 'STAMP')


class WheelTfBridge:
    """Bounded state with exact wheel/diagnostic and ICP input/output joining.

    Call observe_wheel() only after receiving its matching diagnostics; the ROS
    caller must bound that join queue. guess_for_cloud() is called immediately
    before publishing the one allowed native ICP input. It returns a transform
    for the exact cloud stamp, using the same bounded pose prediction as fusion.
    """

    private_tf_topic = '/wc_mapping/icp_guess_tf'
    private_static_tf_topic = '/wc_mapping/icp_guess_tf_static'

    def __init__(self, policy, config, source_mode):
        expected = 'SYNTHETIC' if source_mode == 'synthetic' else 'VERIFIED'
        if source_mode not in ('synthetic', 'real') or config.get('coordinate_status') != expected or config.get('time_status') != expected:
            raise FusionError('WHEEL_TF_COORDINATES_OR_TIME_UNVERIFIED')
        self.T_axle_rig = np.linalg.inv(matrix(config.get('T_rig_axle')))
        self.policy = policy
        self.history = MotionHistory(policy)
        self.max_uncertainty_ns = positive_ns(config.get('max_time_uncertainty_ns'), 'WHEEL_TIME_UNCERTAINTY')
        self.epoch = self.sequence = self.latest_receive_monotonic_ns = None
        self.blocked = None
        self.prepared = OrderedDict()
        self.pending = OrderedDict()
        self.last_prepared_ns = self.last_registered_ns = self.last_relayed_ns = None

    def _active(self):
        if self.blocked:
            raise FusionError('WHEEL_TF_BLOCKED:' + self.blocked)

    def _fresh(self, now_ns, now_monotonic_ns):
        now_ns = positive_ns(now_ns, 'NOW')
        now_monotonic_ns = positive_ns(now_monotonic_ns, 'MONOTONIC', allow_zero=True)
        if self.latest_receive_monotonic_ns is None or not self.history.poses:
            raise FusionError('WHEEL_TF_NO_OBSERVATIONS')
        age = now_monotonic_ns - self.latest_receive_monotonic_ns
        measurement_age = now_ns - self.history.poses[-1][0]
        if not 0 <= age <= self.policy.max_age_ns or not 0 <= measurement_age <= self.policy.max_age_ns:
            raise FusionError('WHEEL_TF_STALE_OR_CLOCK_JUMP')

    def observe_wheel(self, message, diagnostic, *, now_ns, now_monotonic_ns):
        self._active()
        try:
            stamp = message_stamp(message)
            now_ns = positive_ns(now_ns, 'NOW')
            mono = positive_ns(now_monotonic_ns, 'MONOTONIC', allow_zero=True)
            if message.header.frame_id != 'wheel_odom' or message.child_frame_id != 'axle_link':
                raise FusionError('WHEEL_TF_FRAME_MISMATCH')
            if diagnostic.get('schema') != 'wc_wheel_diagnostics_v1' or diagnostic.get('time_valid') is not True or diagnostic.get('state') != 'FEEDBACK':
                raise FusionError('WHEEL_TF_DIAGNOSTIC_INVALID')
            if diagnostic.get('time_source') not in ('common_measurement', 'external_common_time') or positive_ns(diagnostic.get('stamp_ns'), 'DIAGNOSTIC_STAMP') != stamp:
                raise FusionError('WHEEL_TF_COMMON_TIME_MISMATCH')
            uncertainty = positive_ns(diagnostic.get('uncertainty_ns'), 'UNCERTAINTY', allow_zero=True)
            if uncertainty > self.max_uncertainty_ns or not 0 <= now_ns - stamp <= self.policy.max_age_ns:
                raise FusionError('WHEEL_TF_TIME_BUDGET_EXCEEDED')
            epoch = diagnostic.get('stream_epoch')
            sequence = positive_ns(diagnostic.get('sequence'), 'SEQUENCE', allow_zero=True)
            if not isinstance(epoch, str) or not epoch:
                raise FusionError('WHEEL_TF_EPOCH_REQUIRED')
            if self.epoch is not None and (epoch != self.epoch or sequence <= self.sequence):
                raise FusionError('WHEEL_TF_EPOCH_OR_SEQUENCE_RESET')
            if self.latest_receive_monotonic_ns is not None and mono < self.latest_receive_monotonic_ns:
                raise FusionError('WHEEL_TF_MONOTONIC_CLOCK_RESET')
            pose = ros_pose_matrix(message.pose.pose) @ self.T_axle_rig
            if self.history.poses:
                before_stamp, before = self.history.poses[-1]
                if stamp <= before_stamp or stamp - before_stamp > self.policy.max_age_ns:
                    raise FusionError('WHEEL_TF_TIME_GAP_OR_RESET')
                dt = (stamp - before_stamp) * 1e-9
                if np.linalg.norm(pose[:3, 3] - before[:3, 3]) / dt > min(self.policy.velocity_bound_mps, self.policy.max_motion_speed_mps):
                    raise FusionError('WHEEL_TF_MOTION_SPEED_EXCEEDED')
                angle = Rotation.from_matrix(before[:3, :3].T @ pose[:3, :3]).magnitude()
                if angle / dt > min(self.policy.angular_bound_radps, self.policy.max_motion_angular_radps):
                    raise FusionError('WHEEL_TF_MOTION_ROTATION_EXCEEDED')
            self.history.add(stamp, pose, epoch)
            self.epoch, self.sequence = epoch, sequence
            self.latest_receive_monotonic_ns = mono
            return TransformRecord(stamp, 'wheel_odom', 'rig_link', pose.copy(), 'validated_wheel_pose_private_only')
        except (ValueError, TypeError, AttributeError) as error:
            self.blocked = str(error)
            raise

    def prepare_guess(self, stamp_ns, *, now_ns, now_monotonic_ns):
        """Freeze at fusion time, before potentially slow durable raw archiving."""
        self._active()
        self._fresh(now_ns, now_monotonic_ns)
        stamp_ns = positive_ns(stamp_ns, 'ICP_INPUT_STAMP')
        if self.last_prepared_ns is not None and stamp_ns <= self.last_prepared_ns:
            raise FusionError('WHEEL_TF_ICP_INPUT_NOT_INCREASING')
        if len(self.prepared) + len(self.pending) >= self.policy.max_queue:
            raise FusionError('WHEEL_TF_PREPARED_QUEUE_FULL')
        if not 0 <= now_ns - stamp_ns <= self.policy.max_age_ns:
            raise FusionError('WHEEL_TF_ICP_INPUT_STALE_OR_FUTURE')
        pose = self.history.predict(stamp_ns)
        self.prepared[stamp_ns] = (self.epoch, pose.copy())
        self.last_prepared_ns = stamp_ns
        return TransformRecord(stamp_ns, 'wheel_odom', 'rig_link', pose, 'wheel_prediction_private_only')

    def register_guess(self, record):
        """Register frozen guess just before native input; no resampling in time.

        Root enforces its raw-archive/input-age deadline separately. This record
        retains the real historical cloud timestamp even if archiving was slow.
        """
        self._active()
        stamp = positive_ns(record.stamp_ns, 'ICP_INPUT_STAMP')
        if self.pending:
            raise FusionError('WHEEL_TF_ICP_INPUT_ALREADY_IN_FLIGHT')
        if not self.prepared or stamp != next(iter(self.prepared)):
            raise FusionError('WHEEL_TF_UNPREPARED_OR_REORDERED_GUESS')
        epoch, pose = self.prepared[stamp]
        if epoch != self.epoch or record.parent_frame != 'wheel_odom' or record.child_frame != 'rig_link' or \
                record.source != 'wheel_prediction_private_only' or not np.array_equal(matrix(record.transform), pose):
            raise FusionError('WHEEL_TF_FROZEN_GUESS_CHANGED')
        if self.last_registered_ns is not None and stamp <= self.last_registered_ns:
            raise FusionError('WHEEL_TF_ICP_INPUT_NOT_INCREASING')
        del self.prepared[stamp]
        self.pending[stamp] = epoch
        self.last_registered_ns = stamp
        return TransformRecord(stamp, 'wheel_odom', 'rig_link', pose.copy(), record.source)

    def guess_for_cloud(self, stamp_ns, *, now_ns, now_monotonic_ns):
        """Convenience for synchronous pipelines with no intervening raw archive."""
        return self.register_guess(self.prepare_guess(stamp_ns, now_ns=now_ns, now_monotonic_ns=now_monotonic_ns))

    def relay_native_odometry(self, message):
        """Copy an output from the caller's authenticated native ICP subscription.

        DDS publisher GID authorization is the ROS caller's responsibility. This
        method validates exact expected stamp, frames and pose. It never feeds
        this output back into wheel prediction or emits wheel pose on public TF.
        """
        self._active()
        stamp = message_stamp(message)
        if message.header.frame_id != 'odom' or message.child_frame_id != 'rig_link':
            raise FusionError('NATIVE_ICP_TF_FRAME_MISMATCH')
        if stamp not in self.pending or self.pending[stamp] != self.epoch:
            raise FusionError('NATIVE_ICP_TF_UNEXPECTED_STAMP')
        if self.last_relayed_ns is not None and stamp <= self.last_relayed_ns:
            raise FusionError('NATIVE_ICP_TF_DUPLICATE_OR_REVERSED')
        pose = ros_pose_matrix(message.pose.pose)
        del self.pending[stamp]
        self.last_relayed_ns = stamp
        return TransformRecord(stamp, 'odom', 'rig_link', pose, 'native_icp_pose_relay')

    def status(self):
        return {'public_tf_owner': 'native_icp_pose_adapter', 'public_tf_edge': ['odom', 'rig_link'],
                'private_tf_edge': ['wheel_odom', 'rig_link'], 'private_tf_topic': self.private_tf_topic,
                'native_publish_tf': False, 'wheel_samples': len(self.history.poses),
                'epoch': self.epoch, 'sequence': self.sequence, 'blocked': self.blocked,
                'prepared_stamps': list(self.prepared),
                'pending_native_stamps': list(self.pending), 'last_relayed_ns': self.last_relayed_ns,
                'wheel_weighted_constraint': False}


def assign_transform(record, message):
    """Populate geometry_msgs/TransformStamped, without importing ROS in tests."""
    stamp = positive_ns(record.stamp_ns, 'TF_STAMP')
    sec, nsec = divmod(stamp, 1_000_000_000)
    if sec >= 2**31:
        raise FusionError('TF_STAMP_OUTSIDE_ROS_TIME')
    pose = matrix(record.transform)
    quaternion = Rotation.from_matrix(pose[:3, :3]).as_quat()
    message.header.stamp.sec, message.header.stamp.nanosec = sec, nsec
    message.header.frame_id, message.child_frame_id = record.parent_frame, record.child_frame
    message.transform.translation.x, message.transform.translation.y, message.transform.translation.z = pose[:3, 3]
    message.transform.rotation.x, message.transform.rotation.y, message.transform.rotation.z, message.transform.rotation.w = quaternion
    return message
