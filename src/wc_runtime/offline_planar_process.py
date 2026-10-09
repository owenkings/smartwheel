"""Explicit continuous process-noise PSD for offline planar experiments.

The existing acceleration VARIANCE parameters are untouched. PSD has different
physical units and must be independently specified for this experiment.
"""
import copy
import math
import numpy as np
from .mapping_planar import PlanarEKF


MODEL = 'continuous_white_acceleration'
KEYS = {'model', 'status', 'linear_acceleration_psd_m2_s3', 'angular_acceleration_psd_rad2_s3'}


def validate_process_noise(value):
    if value is None:
        return None
    if not isinstance(value, dict) or set(value) != KEYS:
        raise ValueError('explicit process noise model/status and both PSD values required')
    if value['model'] != MODEL or value['status'] != 'EXPERIMENTAL_NOT_CALIBRATED':
        raise ValueError('unknown or falsely calibrated offline process noise model')
    result = copy.deepcopy(value)
    for key in ('linear_acceleration_psd_m2_s3', 'angular_acceleration_psd_rad2_s3'):
        v = result[key]
        if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) or v <= 0:
            raise ValueError('process-noise PSD must be finite and strictly positive: '+key)
        result[key] = float(v)
    return result


class ContinuousPlanarEKF(PlanarEKF):
    """Same state, measurement update and gate; explicit continuous PSD Q."""
    def __init__(self, config, axle_xy=(0., 0.), *, process_noise):
        self.process_noise = validate_process_noise(process_noise)
        if self.process_noise is None:
            raise ValueError('ContinuousPlanarEKF requires explicit PSD model')
        self._continuous_gap_s = 0.0
        super().__init__(config, axle_xy)

    @classmethod
    def from_state(cls, state, *, process_noise):
        if type(state) is not PlanarEKF:
            raise ValueError('PSD replacement requires a fresh five-state PlanarEKF anchor')
        result = cls(state.config, state.x[:2], process_noise=process_noise)
        # No predict/update is repeated. Preserve all state/covariance/sequence/
        # counters and diagnostics established by the existing initialization.
        for name, value in state.__dict__.items():
            setattr(result, name, copy.deepcopy(value))
        result.process_noise = validate_process_noise(process_noise)
        return result

    def process_noise_report(self):
        return {**copy.deepcopy(self.process_noise),
            'discretization': '3-point Gauss-Legendre integral of propagated continuous velocity noise along constant-turn arc',
            'state_order': ['axle_x_m', 'axle_y_m', 'yaw_rad', 'vx_m_s', 'wz_rad_s'],
            'units': {'linear_acceleration_psd_m2_s3': 'm^2/s^3', 'angular_acceleration_psd_rad2_s3': 'rad^2/s^3'},
            'legacy_variance_parameters_reinterpreted': False,
            'legacy_acceleration_variances_used_for_Q': False,
            'measurement_noise_and_innovation_gate_changed': False,
            'formal_calibration': False, 'automatic_live_application': False,
            'gap_policy': 'hold pose; decorrelate pose/velocity; cumulative-gap diagonal uncertainty budget; no integration across gap',
            'gap_covariance_model': 'heuristic isotropic pose budget q*T_gap^3/3 and velocity budget q*T_gap; consecutive missing intervals use differences of the cumulative budget',
            'gap_model_boundary': 'uncertainty budget only; does not reconstruct unobserved motion or propagate initial velocity uncertainty into gap pose',
            'current_continuous_gap_s': self._continuous_gap_s}

    def predict(self, dt, *, observed=True):
        if not math.isfinite(dt) or dt < 0:
            raise ValueError('nonnegative finite prediction interval required')
        if dt == 0:
            return
        qv = self.process_noise['linear_acceleration_psd_m2_s3']
        qw = self.process_noise['angular_acceleration_psd_rad2_s3']
        if not observed:
            # Preserve the existing missing-data contract. Cross-covariance
            # must not make the next observation move a pose across the gap.
            self.decorrelate_gap()
            # A source still arriving during a gap splits the missing period.
            # Accumulate the pose uncertainty over the whole gap, instead of
            # summing dt^3 (which would underestimate it by n^2 for n pieces).
            previous_gap = self._continuous_gap_s
            self._continuous_gap_s += dt
            cubic_increment = (self._continuous_gap_s**3-previous_gap**3)/3
            self.P += np.diag([qv*cubic_increment, qv*cubic_increment, qw*cubic_increment, qv*dt, qw*dt])
            self.gap_intervals += 1
            return
        self._continuous_gap_s = 0.0
        start = self.x.copy()
        x, F = self.transition(start, dt)
        Q = np.zeros((5, 5))
        D = np.diag([qv, qw])
        for fraction, weight in ((.5-math.sqrt(15)/10, 5/18), (.5, 4/9), (.5+math.sqrt(15)/10, 5/18)):
            at = self.transition(start, dt*fraction)[0]
            _, remaining = self.transition(at, dt*(1-fraction))
            J = remaining[:, [3, 4]]
            Q += weight*dt*(J @ D @ J.T)
        self.P = F @ self.P @ F.T + Q
        self.P = (self.P+self.P.T)*.5
        self.x = x

    def report(self):
        return {**super().report(), 'process_noise': self.process_noise_report()}
