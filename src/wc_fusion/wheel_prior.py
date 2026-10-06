"""Wheel pose prediction for dual-frame compensation, with no TF publication.

This is explicitly a prediction input, not a weighted wheel/ICP optimizer.
Wheel poses locate axle_link; the existing fusion reference is rig_link.
"""
import numpy as np
from scipy.spatial.transform import Rotation
from .core import FusionError, MotionHistory, matrix, integer_ns


class WheelMotionHistory:
    mode = 'wheel_odometry_prediction'
    requires_prediction = True

    def __init__(self, policy, config, source_mode):
        expected = 'SYNTHETIC' if source_mode == 'synthetic' else 'VERIFIED'
        if config.get('coordinate_status') != expected or config.get('time_status') != expected:
            raise FusionError('WHEEL_COORDINATES_OR_TIME_UNVERIFIED')
        self.T_rig_axle = matrix(config.get('T_rig_axle'))
        self.T_axle_rig = np.linalg.inv(self.T_rig_axle)
        self.policy = policy
        self.history = MotionHistory(policy)
        self.sample_count = 0
        self.last_completed_icp = None

    def add_wheel(self, stamp_ns, T_wheel_axle, epoch='wheel_session'):
        stamp_ns = integer_ns(stamp_ns)
        pose = matrix(T_wheel_axle) @ self.T_axle_rig
        if self.history.poses:
            previous_stamp, previous = self.history.poses[-1]
            if stamp_ns <= previous_stamp:
                raise FusionError('WHEEL_TIME_NOT_INCREASING')
            dt = (stamp_ns-previous_stamp)*1e-9
            speed = np.linalg.norm(pose[:3,3]-previous[:3,3])/dt
            rate = Rotation.from_matrix(previous[:3,:3].T @ pose[:3,:3]).magnitude()/dt
            if speed > min(self.policy.max_motion_speed_mps, self.policy.velocity_bound_mps):
                raise FusionError('WHEEL_POSE_SPEED_OR_RESET')
            if rate > min(self.policy.max_motion_angular_radps, self.policy.angular_bound_radps):
                raise FusionError('WHEEL_POSE_ROTATION_OR_RESET')
        self.history.add(stamp_ns, pose, epoch)
        self.sample_count += 1

    def add(self, stamp_ns, pose, epoch):
        # ICP remains an independent output. It must not overwrite the wheel
        # history and silently turn an enabled wheel predictor into ICP-only.
        self.last_completed_icp = (integer_ns(stamp_ns), matrix(pose), epoch)

    def predict(self, stamp_ns):
        if self.sample_count < 2:
            raise FusionError('WHEEL_PREDICTION_NEEDS_TWO_SAMPLES')
        return self.history.predict(stamp_ns)

    def status(self):
        return {'mode': self.mode, 'samples': self.sample_count,
                'latest_stamp_ns': self.history.poses[-1][0] if self.history.poses else None,
                'weighted_estimator_constraint': False, 'publishes_tf': False}
