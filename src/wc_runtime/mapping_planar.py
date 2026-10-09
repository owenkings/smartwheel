"""Pure five-state planar EKF, no ROS, devices or inferred stationary truth.

State is [axle x, axle y, yaw, forward velocity, yaw rate]. Independent wheel
and gyro measurements update a nonlinear constant-turn-rate motion model.
Noise values are explicit experimental weights, not measured accuracy.
"""
import copy
import math
import re

import numpy as np


DEFAULT_PLANAR = {
    'status': 'UNVALIDATED_MODEL_WEIGHTS',
    'wheel_v_variance': .0025, 'wheel_w_variance': .0025,
    'gyro_w_variance': .0004,
    'linear_acceleration_variance': .25, 'angular_acceleration_variance': .25,
    'initial_pose_variance': [1e-6, 1e-6, 1e-6],
    'initial_velocity_variance': [.04, .04],
    'innovation_gate_sigma': 5.0,
}

NATIVE_CONSTRAINT_VARIANCE = 1e-6


def native_covariances(pose, twist):
    """Regularize only constrained ROS dimensions; keep EKF covariance intact.

    Native consumers may invert a full 6x6 covariance. A planar model has no
    stochastic z/roll/pitch state, and lateral twist is a deterministic lever
    arm function of yaw rate. Positive model residuals make those full matrices
    invertible without replacing the observable dynamic blocks. This numerical
    constraint variance is not a measured sensor accuracy.
    """
    pose, twist = np.asarray(pose).copy(), np.asarray(twist).copy()
    pose[[2, 3, 4], [2, 3, 4]] += NATIVE_CONSTRAINT_VARIANCE
    twist[[1, 2, 3, 4], [1, 2, 3, 4]] += NATIVE_CONSTRAINT_VARIANCE
    return pose, twist


def validate_planar_config(value=None):
    if value is None:
        value = {}
    if not isinstance(value, dict) or set(value)-set(DEFAULT_PLANAR):
        raise ValueError('unknown planar_ekf configuration fields')
    result = {**copy.deepcopy(DEFAULT_PLANAR), **copy.deepcopy(value)}
    if result['status'] != 'UNVALIDATED_MODEL_WEIGHTS':
        raise ValueError('planar_ekf requires UNVALIDATED_MODEL_WEIGHTS')
    for key, values in result.items():
        if key == 'status':
            continue
        count = {'initial_pose_variance': 3, 'initial_velocity_variance': 2}.get(key)
        if count is not None:
            if not isinstance(values, (list, tuple)) or len(values) != count:
                raise ValueError(key+' has incorrect dimensions')
            values = list(values)
        else:
            values = [values]
        if any(isinstance(v, bool) or not isinstance(v, (int, float)) or
               not math.isfinite(v) or not 0 < v < 1e4 for v in values):
            raise ValueError(key+' must contain finite positive model variances below 10000')
        result[key] = [float(v) for v in values] if count else float(values[0])
    return result


def validate_confirmed_bias(value, *, expected_sensor_id=None):
    """Validate a declared external calibration, not prove the declaration true."""
    if value is None:
        return None
    required = {'status', 'sensor_id', 'bias_native_rad_s', 'evidence_id', 'evidence_sha256', 'source'}
    optional = {'evidence_note', 'coordinate_convention'}
    if not isinstance(value, dict) or not required <= set(value) or set(value)-required-optional:
        raise ValueError('confirmed_gyro_bias requires explicit bias and evidence provenance')
    result = copy.deepcopy(value)
    from .device_bindings import identity_token
    identity_token(result['sensor_id'], prefix='H30-')
    if result['status'] != 'INDEPENDENTLY_CONFIRMED' or (expected_sensor_id is not None and result['sensor_id'] != expected_sensor_id):
        raise ValueError('confirmed_gyro_bias declaration/sensor mismatch')
    if result['source'] not in ('external_stationarity_calibration', 'operator_visual_confirmation'):
        raise ValueError('confirmed_gyro_bias source cannot be wheel-only or automatic prior inference')
    result.setdefault('coordinate_convention', 'H30_NATIVE_UNVALIDATED')
    if result['coordinate_convention'] != 'H30_NATIVE_UNVALIDATED':
        raise ValueError('confirmed_gyro_bias must use native H30 axes')
    a = np.asarray(result['bias_native_rad_s'])
    if a.shape != (3,) or a.dtype.kind not in 'iuf' or not np.isfinite(a).all() or np.linalg.norm(a) > .02:
        raise ValueError('confirmed native bias must be finite, three-axis and norm <= 0.02 rad/s')
    for key in ('evidence_id', 'evidence_sha256'):
        if not isinstance(result[key], str) or not result[key].strip() or len(result[key]) > 256:
            raise ValueError('confirmed_gyro_bias missing '+key)
    if not re.fullmatch(r'[0-9a-fA-F]{64}', result['evidence_sha256']):
        raise ValueError('confirmed_gyro_bias requires an evidence SHA-256')
    if 'evidence_note' in result and not isinstance(result['evidence_note'], str):
        raise ValueError('confirmed_gyro_bias evidence_note must be text')
    result['bias_native_rad_s'] = a.astype(float).tolist()
    result['evidence_sha256'] = result['evidence_sha256'].lower()
    return result


class PlanarEKF:
    """Copyable filter state. Caller owns chronological, single-use observations.

    Predict uses an exact constant-turn arc and its analytic state Jacobian.
    Q models independent constant accelerations on each source-event interval;
    acceleration variances have (m/s²)² and (rad/s²)² units. Query-only partial
    predictions never become anchors, avoiding query-dependent discretization.
    """
    def __init__(self, config, axle_xy=(0., 0.)):
        self.config = validate_planar_config(config)
        self.x = np.array([*axle_xy, 0., 0., 0.], dtype=float)
        self.P = np.diag(self.config['initial_pose_variance'] + self.config['initial_velocity_variance'])
        self.counts = {'wheel': 0, 'gyro': 0}
        self.last_sequence = {'wheel': -1, 'imu': -1}
        self.last_innovation = {}
        self.rejected_counts = {'wheel': 0, 'gyro': 0}
        self.last_update = {}
        self.gap_intervals = 0

    def copy(self):
        return copy.deepcopy(self)

    @staticmethod
    def transition(x, dt):
        angle = x[4]*dt/2
        if abs(angle) < 1e-4:
            sinc = 1-angle*angle/6+angle**4/120
            dsinc = -angle/3+angle**3/30-angle**5/840
        else:
            sinc = math.sin(angle)/angle
            dsinc = (angle*math.cos(angle)-math.sin(angle))/(angle*angle)
        a = dt*sinc
        da = dt*dt*.5*dsinc
        c, s = math.cos(x[2]+angle), math.sin(x[2]+angle)
        dx, dy = x[3]*a*c, x[3]*a*s
        result = x.copy()
        result[:3] += [dx, dy, x[4]*dt]
        F = np.eye(5)
        F[0, 2], F[1, 2] = -dy, dx
        F[0, 3], F[1, 3] = a*c, a*s
        F[0, 4] = x[3]*(da*c-a*s*dt/2)
        F[1, 4] = x[3]*(da*s+a*c*dt/2)
        F[2, 4] = dt
        return result, F

    def decorrelate_gap(self):
        # A new velocity observation cannot move a frozen pose through an
        # unobserved interval by its old pose/velocity cross-covariance.
        self.P[:3, 3:] = 0.
        self.P[3:, :3] = 0.

    def predict(self, dt, *, observed=True):
        if not math.isfinite(dt) or dt < 0:
            raise ValueError('nonnegative finite prediction interval required')
        if dt == 0:
            return
        qv, qw = self.config['linear_acceleration_variance'], self.config['angular_acceleration_variance']
        if not observed:
            self.decorrelate_gap()
            self.P += np.diag([qv*dt**4/4, qv*dt**4/4, qw*dt**4/4, qv*dt**2, qw*dt**2])
            self.gap_intervals += 1
            return
        x, F = self.transition(self.x, dt)
        # Mid-interval body acceleration, including heading sensitivity.
        G = np.zeros((5, 2))
        G[:3, 0] = .5*dt*F[:3, 3]
        G[:3, 1] = .5*dt*F[:3, 4]
        G[3, 0] = G[4, 1] = dt
        self.P = F @ self.P @ F.T + G @ np.diag([qv, qw]) @ G.T
        self.P = (self.P+self.P.T)*.5
        self.x = x

    def _update(self, z, H, R, source):
        innovation = np.asarray(z)-H @ self.x
        S = H @ self.P @ H.T+R
        nis = float(innovation @ np.linalg.solve(S, innovation))
        threshold = self.config['innovation_gate_sigma']**2
        accepted = math.isfinite(nis) and nis <= threshold
        self.last_innovation[source] = innovation.tolist()
        self.last_update[source] = {'innovation': innovation.tolist(),
            'innovation_covariance': S.tolist(), 'nis': nis,
            'threshold_nis': threshold, 'candidate_sigma': self.config['innovation_gate_sigma'],
            'threshold_status': 'EXPERIMENTAL_NOT_CALIBRATED', 'accepted': accepted,
            'reason': 'ACCEPTED' if accepted else 'MAHALANOBIS_GATE_REJECTED'}
        if not accepted:
            self.rejected_counts[source] += 1
            return False
        K = np.linalg.solve(S, H @ self.P).T
        self.x += K @ innovation
        residual = np.eye(5)-K @ H
        # Joseph form preserves numerical PSD with finite model noise.
        self.P = residual @ self.P @ residual.T+K @ R @ K.T
        self.P = (self.P+self.P.T)*.5
        self.counts[source] += 1
        self.last_innovation[source] = innovation.tolist()
        return True

    def wheel(self, sequence, velocity, yaw_rate):
        if sequence <= self.last_sequence['wheel']:
            return
        H = np.zeros((2, 5)); H[0, 3] = H[1, 4] = 1.
        if any(not math.isfinite(v) for v in (velocity, yaw_rate)):
            raise ValueError('wheel measurements must be finite')
        accepted = self._update([velocity, yaw_rate], H, np.diag([self.config['wheel_v_variance'],
                                                     self.config['wheel_w_variance']]), 'wheel')
        self.last_sequence['wheel'] = sequence
        self.last_update['wheel']['sequence'] = sequence
        return accepted

    def gyro(self, sequence, yaw_rate):
        if sequence <= self.last_sequence['imu']:
            return
        H = np.zeros((1, 5)); H[0, 4] = 1.
        if not math.isfinite(yaw_rate):
            raise ValueError('gyro measurement must be finite')
        accepted = self._update([yaw_rate], H, np.array([[self.config['gyro_w_variance']]]), 'gyro')
        self.last_sequence['imu'] = sequence
        self.last_update['gyro']['sequence'] = sequence
        return accepted

    def output(self, t_reference_axle):
        t = np.asarray(t_reference_axle, dtype=float)
        c, s = math.cos(self.x[2]), math.sin(self.x[2])
        R = np.array([[c, -s, 0.], [s, c, 0.], [0., 0., 1.]])
        pose = np.eye(4); pose[:3, :3] = R
        pose[:2, 3] = self.x[:2]-(R @ t)[:2]
        angular = np.array([0., 0., self.x[4]])
        linear = np.array([self.x[3], 0., 0.])-np.cross(angular, t)
        # Pose covariance in the world frame; twist in the child reference.
        J = np.zeros((6, 5)); J[0, 0] = J[1, 1] = J[5, 2] = 1.
        J[:2, 2] = -(R @ np.array([-t[1], t[0], 0.]))[:2]
        B = np.zeros((6, 5)); B[0, 3] = B[5, 4] = 1.
        B[0, 4], B[1, 4] = t[1], -t[0]
        return pose, linear, angular, J @ self.P @ J.T, B @ self.P @ B.T

    def report(self):
        return {'estimator': 'five_state', 'state_order': ['axle_x_m', 'axle_y_m', 'yaw_rad', 'vx_m_s', 'wz_rad_s'],
                'state': self.x.tolist(), 'covariance': self.P.tolist(),
                'measurement_updates': dict(self.counts), 'last_sequence': dict(self.last_sequence),
                'measurement_rejections': dict(self.rejected_counts),
                'last_update': copy.deepcopy(self.last_update),
                'last_innovation': copy.deepcopy(self.last_innovation), 'gap_intervals': self.gap_intervals,
                'noise_status': self.config['status'],
                'bias_learning': 'none; only explicit versioned independently declared calibration',
                'constraint': 'planar axle motion; not measured level or forced planar point cloud'}
