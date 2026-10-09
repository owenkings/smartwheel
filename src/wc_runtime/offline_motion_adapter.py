"""Session-scoped offline calibration adapter; original measurements stay intact."""
from bisect import bisect_left, bisect_right
import copy
import numpy as np
from .mapping_prior import MotionPrior
from .measurement import prepare_gyro
from .offline_motion_calibration import validate_for_session
from .offline_planar_process import ContinuousPlanarEKF, validate_process_noise


def corrected_wheel_observation(velocity, yaw_rate, config, scale, speed_coefficient):
    values = np.asarray([velocity, yaw_rate, scale, speed_coefficient], dtype=float)
    if not np.isfinite(values).all() or scale <= 0:
        raise ValueError('finite wheel inputs and positive scale required')
    J = np.array([[1., 0.], [speed_coefficient, scale]])
    original = np.array([velocity, yaw_rate])
    raw_covariance = np.diag([config['wheel_v_variance'], config['wheel_w_variance']])
    return J @ original, J @ raw_covariance @ J.T


class OfflineCorrectedPrior(MotionPrior):
    """Uses the original event/history/filter implementation with fixed corrections.

    This class is never instantiated by a live entry. Static validation and
    motion fitting use separate windows, but are not independent physical truth.
    It does not pass a screened candidate off as INDEPENDENTLY_CONFIRMED.
    """
    def __init__(self, config, session_id, *, candidate, candidate_sha256, clock, process_noise=None):
        estimator = config.get('estimator', 'five_state')
        if estimator not in ('five_state', 'robot_localization') or config.get('motion_model') != 'planar_ekf':
            raise ValueError('offline correction requires a supported planar EKF')
        if estimator != 'five_state' and process_noise is not None:
            raise ValueError('five-state continuous process noise cannot be applied to robot_localization')
        if config.get('offline_experiment') is not True:
            raise ValueError('offline correction is prohibited outside explicit offline replay')
        if config.get('confirmed_gyro_bias') is not None:
            raise ValueError('do not apply offline bias on top of confirmed bias')
        if config.get('offline_motion_correction') is not None:
            raise ValueError('do not apply a candidate on top of an existing correction')
        if not isinstance(candidate_sha256, str) or len(candidate_sha256) != 64 or any(c not in '0123456789abcdef' for c in candidate_sha256):
            raise ValueError('candidate file SHA-256 is required')
        validate_for_session(candidate, session_id=session_id, imu_device_id=config['imu_sensor_id'],
                             wheel_device_id=config['wheel_device_id'])
        self.offline_candidate = copy.deepcopy(candidate)
        self.offline_bias = np.asarray(candidate['bias_native_rad_s'], dtype=float)
        self.offline_scale = float(candidate['wheel_yaw_scale'])
        self.offline_speed_coefficient = float(candidate['wheel_yaw_speed_coefficient'])
        self._offline_rotation_checked = False
        self.offline_process_noise = validate_process_noise(process_noise)
        self.offline_evidence = {
            'status': 'OFFLINE_CANDIDATE_APPLIED', 'candidate_sha256': candidate_sha256,
            'candidate_id': candidate.get('candidate_id'),
            'bias_native_rad_s': self.offline_bias.tolist(), 'wheel_yaw_scale': self.offline_scale,
            'wheel_yaw_speed_coefficient': self.offline_speed_coefficient,
            'physical_calibration': False, 'automatic_live_application': False,
            'covariance_policy': 'original model R propagated with full J R J^T; no retuning',
            'relative_fit_warning': 'wheel correction referenced gyro in training; not independent absolute truth'}
        super().__init__(config, session_id, clock=clock)
        self.config['offline_motion_correction'] = copy.deepcopy(self.offline_evidence)

    def _initialize_from_mount(self):
        ready = super()._initialize_from_mount()
        if ready:
            self.initialization.update(gyro_bias_candidate_rad_s=self.offline_bias.tolist(),
                gyro_bias_status='OFFLINE_CANDIDATE_APPLIED',
                offline_motion_correction=copy.deepcopy(self.offline_evidence))
            if self.offline_process_noise is not None:
                horizon, state = self.planar_anchor
                state = ContinuousPlanarEKF.from_state(state, process_noise=self.offline_process_noise)
                self.planar_anchor = (horizon, state)
                self.initialization['offline_process_noise'] = state.process_noise_report()
        return ready

    def _planar_observe(self, state, stamp, *, initial=False, grouped=None):
        # Selection deliberately matches MotionPrior: original source sequences,
        # wheel before gyro at equal stamps; queries never create observations.
        if not self._offline_rotation_checked:
            expected = np.asarray(self.offline_candidate['provenance']['R_reference_imu'])
            if not np.allclose(self.R_reference_imu, expected, atol=1e-9, rtol=0):
                raise ValueError('offline candidate IMU rotation differs from actual replay')
            self._offline_rotation_checked = True
        for kind, rows in (('wheel', self.wheel), ('imu', self.imu)):
            if grouped is not None:
                selected = grouped[kind].get(stamp, ())
            else:
                stamps = [r['stamp'] for r in rows]
                if initial:
                    selected = rows[max(0, bisect_right(stamps, stamp)-1):bisect_right(stamps, stamp)]
                else:
                    selected = rows[bisect_left(stamps, stamp):bisect_right(stamps, stamp)]
            for row in selected:
                if kind == 'wheel':
                    if row['sequence'] <= state.last_sequence['wheel']:
                        continue
                    z, R = corrected_wheel_observation(row['v'], row['wheel_yaw'], state.config,
                                                      self.offline_scale, self.offline_speed_coefficient)
                    if self.config['estimator'] == 'robot_localization':
                        updated = state.wheel_covariance(row['sequence'], z[0], z[1], R)
                    else:
                        H = np.zeros((2, 5)); H[0, 3] = H[1, 4] = 1.
                        updated = state._update(z, H, R, 'wheel')
                        state.last_sequence['wheel'] = row['sequence']
                    state.last_update['wheel'].update(sequence=row['sequence'], stamp_ns=stamp,
                        stream_epoch=row['epoch'], raw_measurement=[row['v'], row['wheel_yaw']],
                        measurement=z.tolist(), measurement_covariance=R.tolist(), measurement_frame='axle',
                        offline_candidate_sha256=self.offline_evidence['candidate_sha256'])
                else:
                    # Native-axis subtraction occurs once, before installation rotation.
                    prepared = prepare_gyro(row['gyro'], self.offline_bias, self.R_reference_imu)
                    prepared['bias_provenance'] = copy.deepcopy(self.offline_evidence)
                    prepared['bias_status'] = 'OFFLINE_CANDIDATE_APPLIED'
                    updated = state.gyro(row['sequence'], prepared['target_rad_s'][2])
                    if updated is not None:
                        state.last_update['gyro'].update(stamp_ns=stamp, stream_epoch=row['epoch'],
                            measurement=[prepared['target_rad_s'][2]], measurement_frame='configured_axle_z',
                            preparation=prepared)

    def sample_report(self, stamp_ns, pose):
        report = super().sample_report(stamp_ns, pose)
        report.update(gyro_bias_applied_native_rad_s=self.offline_bias.tolist(),
                      offline_motion_correction=copy.deepcopy(self.offline_evidence))
        return report

    def bias_report(self):
        return {'status': 'OFFLINE_CANDIDATE_APPLIED', 'bias_estimated': True,
                'formal_calibration': False, 'applied': copy.deepcopy(self.offline_evidence),
                'background_monitor_only': super().bias_report()}


def prior_factory(candidate, candidate_sha256, process_noise=None):
    fixed = copy.deepcopy(candidate)
    fixed_process = validate_process_noise(process_noise)
    def build(config, session_id, *, clock):
        configured = copy.deepcopy(config)
        return OfflineCorrectedPrior(configured, session_id, candidate=fixed,
                                     candidate_sha256=candidate_sha256, clock=clock,
                                     process_noise=fixed_process)
    return build
