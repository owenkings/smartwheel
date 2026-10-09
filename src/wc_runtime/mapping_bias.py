"""Causal native-IMU bias candidates; no ROS, device writes or frame changes.

Wheel/IMU statistics are screening evidence, never proof of true stillness.
Candidates affect no motion until independently corroborated. All limits below
are explicit experimental screening limits, not a calibrated sensor model.
"""
from bisect import bisect_right
from collections import deque
import copy
from dataclasses import asdict, dataclass
import math

import numpy as np


def _stamp(value):
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise ValueError('source stamp must be a nonnegative integer')
    return value


def _vector(value, size):
    result = np.asarray(value, dtype=float)
    if result.shape != (size,) or not np.isfinite(result).all():
        raise ValueError('finite native-axis vector required')
    return result


@dataclass(frozen=True)
class BiasPolicy:
    window_s: float = 5.0
    min_samples: int = 200
    max_samples: int = 4000
    max_imu_gap_s: float = 0.10
    max_wheel_age_s: float = 0.20
    max_wheel_speed_m_s: float = 0.002
    max_gyro_sample_rad_s: float = 0.015
    max_gyro_mean_rad_s: float = 0.0015
    max_gyro_axis_std_rad_s: float = 0.004
    gravity_m_s2: float = 9.80665
    max_accel_norm_error_m_s2: float = 0.25
    max_accel_axis_std_m_s2: float = 0.10
    max_accel_direction_p95_deg: float = 0.50
    robust_windows: int = 5
    min_confirmed_windows: int = 2
    max_bias_norm_rad_s: float = 0.0015
    max_update_norm_rad_s: float = 0.00025
    max_pending_windows: int = 16
    max_versions: int = 2048

    def __post_init__(self):
        for name, value in asdict(self).items():
            if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value) or value <= 0:
                raise ValueError('bias policy limits must be finite and positive: '+name)
        for name in ('min_samples', 'max_samples', 'robust_windows', 'min_confirmed_windows',
                     'max_pending_windows', 'max_versions'):
            if not isinstance(getattr(self, name), int):
                raise ValueError('bias policy sample/history limits must be integers')
        if self.min_samples < 3 or self.max_samples < self.min_samples or self.min_confirmed_windows > self.robust_windows:
            raise ValueError('inconsistent bias policy sample/window limits')


@dataclass(frozen=True)
class BiasVersion:
    version: int
    effective_stamp_ns: int
    bias_native_rad_s: tuple
    evidence_ids: tuple


@dataclass(frozen=True)
class StationaryCalibrationPolicy:
    """Candidate 10+10+10 source-time protocol; thresholds need field acceptance."""
    warmup_s: float = 10.
    calibration_s: float = 10.
    holdout_s: float = 10.
    min_rate_hz: float = 40. # Existing 200 samples / 5 s screening density.
    min_phase_samples: int = 200
    max_phase_samples: int = 4000
    max_gap_s: float = .10
    max_bias_norm_rad_s: float = .0015
    max_gyro_sample_rad_s: float = .015
    max_gyro_mean_rad_s: float = .0015
    max_holdout_mean_rad_s: float = .0005
    max_axis_std_rad_s: float = .004
    max_wheel_speed_m_s: float = .002
    max_wheel_age_s: float = .20
    max_accel_axis_std_m_s2: float = .10
    max_accel_norm_error_m_s2: float = .25
    max_accel_direction_p95_deg: float = .50
    gravity_m_s2: float = 9.80665

    def __post_init__(self):
        for key, value in asdict(self).items():
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
                raise ValueError('positive finite stationary-calibration limit required: '+key)
        if type(self.min_phase_samples) is not int or type(self.max_phase_samples) is not int or self.max_phase_samples<self.min_phase_samples:
            raise ValueError('valid integer stationary phase sample bounds required')


def calibrate_stationary(rows, independent_evidence, policy=None):
    """Estimate only on middle 10 s, validate unseen last 10 s; never autoapply.

    Rows are original IMU source events with stamp_ns, sequence, gyro_native,
    acceleration_native, wheel_velocity_m_s and wheel_age_s. Physical stillness
    must cover all stages in a separate, explicitly attested evidence record.
    """
    policy = policy if isinstance(policy, StationaryCalibrationPolicy) else StationaryCalibrationPolicy(**(policy or {}))
    rows = list(rows)
    if len(rows) < 3:
        raise ValueError('insufficient original calibration events')
    stamps = np.asarray([_stamp(r['stamp_ns']) for r in rows], dtype=np.int64)
    if (np.diff(stamps) < 0).any() or len({r['sequence'] for r in rows}) != len(rows):
        raise ValueError('calibration requires ordered, distinct original sequences')
    if len({r.get('stream_epoch','unspecified') for r in rows}) != 1:
        raise ValueError('calibration cannot span a source stream restart')
    start = int(stamps[0])
    boundaries = start+np.rint(np.cumsum([0., policy.warmup_s, policy.calibration_s, policy.holdout_s])*1e9).astype(np.int64)
    end = int(boundaries[-1])
    evidence = independent_evidence
    if not isinstance(evidence, dict) or evidence.get('physically_stationary') is not True or \
            evidence.get('source') not in ('operator_visual_confirmation', 'external_stationarity_calibration') or \
            not isinstance(evidence.get('evidence_id'), str) or not evidence['evidence_id'].strip() or \
            not isinstance(evidence.get('evidence_sha256'), str) or len(evidence['evidence_sha256']) != 64 or \
            _stamp(evidence.get('start_stamp_ns')) > start or _stamp(evidence.get('end_stamp_ns')) < end:
        raise ValueError('independent physical-stationarity evidence must cover entire 10+10+10 protocol')
    if int(stamps[-1]) < end:
        raise ValueError('recording does not cover entire warmup/calibration/holdout duration')
    retained = [r for r in rows if r['stamp_ns'] <= end]
    gyro = np.asarray([_vector(r['gyro_native'], 3) for r in retained])
    accel = np.asarray([_vector(r['acceleration_native'], 3) for r in retained])
    times = np.asarray([r['stamp_ns'] for r in retained], dtype=np.int64)
    reasons = []
    if np.diff(times).max()*1e-9 > policy.max_gap_s:
        reasons.append('SOURCE_GAP')
    if np.max(np.linalg.norm(gyro,axis=1))>policy.max_gyro_sample_rad_s:
        reasons.append('GYRO_SAMPLE_NORM')
    if any(not math.isfinite(r['wheel_velocity_m_s']) or abs(r['wheel_velocity_m_s']) > policy.max_wheel_speed_m_s or
           not math.isfinite(r['wheel_age_s']) or not 0 <= r['wheel_age_s'] <= policy.max_wheel_age_s for r in retained):
        reasons.append('WHEEL_STATIONARY_SCREENING_OR_COVERAGE')
    phases = {}
    for i, name in enumerate(('warmup', 'calibration', 'holdout')):
        mask = (times >= boundaries[i]) & (times < boundaries[i+1])
        count = int(mask.sum())
        duration = (boundaries[i+1]-boundaries[i])*1e-9
        if count < max(policy.min_phase_samples,math.ceil(policy.min_rate_hz*duration)):
            reasons.append(name.upper()+'_INSUFFICIENT_SAMPLES')
        if count>policy.max_phase_samples:
            reasons.append(name.upper()+'_SAMPLE_CAPACITY_EXCEEDED')
        if count < 3:
            raise ValueError('insufficient samples in '+name)
        phases[name] = {'start_stamp_ns': int(boundaries[i]), 'end_stamp_ns': int(boundaries[i+1]),
                        'sample_count': count, 'mask': mask}
        if np.std(accel[mask], axis=0).max() > policy.max_accel_axis_std_m_s2 or \
                np.max(np.abs(np.linalg.norm(accel[mask], axis=1)-policy.gravity_m_s2)) > policy.max_accel_norm_error_m_s2:
            reasons.append(name.upper()+'_ACCEL_SCREENING')
        mean_accel=accel[mask].mean(axis=0); mean_norm=np.linalg.norm(mean_accel)
        accel_norms=np.linalg.norm(accel[mask],axis=1)
        direction_p95=(float(np.rad2deg(np.quantile(np.arccos(np.clip(
            (accel[mask]/accel_norms[:,None])@(mean_accel/mean_norm),-1,1)),.95)))
            if mean_norm>1e-12 and np.all(accel_norms>1e-12) else None)
        phases[name]['accel_direction_p95_deg']=direction_p95
        if direction_p95 is None or direction_p95>policy.max_accel_direction_p95_deg:
            reasons.append(name.upper()+'_ACCEL_DIRECTION_SCREENING')
        if np.std(gyro[mask], axis=0).max() > policy.max_axis_std_rad_s:
            reasons.append(name.upper()+'_GYRO_STD')
        if name!='warmup' and np.linalg.norm(gyro[mask].mean(axis=0))>policy.max_gyro_mean_rad_s:
            reasons.append(name.upper()+'_GYRO_MEAN')
    calibration = gyro[phases['calibration']['mask']]
    bias = calibration.mean(axis=0)
    residual = gyro[phases['holdout']['mask']]-bias
    if np.linalg.norm(bias) > policy.max_bias_norm_rad_s:
        reasons.append('BIAS_NORM')
    if np.linalg.norm(residual.mean(axis=0)) > policy.max_holdout_mean_rad_s:
        reasons.append('HOLDOUT_MEAN_RESIDUAL')
    for phase in phases.values():
        del phase['mask']
    return {'schema_version': 1, 'protocol': 'SOURCE_TIME_10_10_10',
            'status': 'HOLDOUT_PASSED' if not reasons else 'REJECTED', 'rejection_reasons': reasons,
            'bias_native_rad_s': bias.tolist(), 'calibration_covariance_rad2_s2': np.cov(calibration.T).tolist(),
            'holdout_residual_mean_rad_s': residual.mean(axis=0).tolist(),
            'holdout_residual_std_rad_s': np.std(residual, axis=0, ddof=1).tolist(),
            'phases': phases, 'policy': asdict(policy), 'policy_status': 'EXPERIMENTAL_NOT_CALIBRATED',
            'policy_sources':{'stationary_screening':'existing BiasPolicy limits; sample density 200/5s',
                'holdout_mean_limit':'new 0.0005 rad/s experimental candidate; needs field validation',
                'protocol_duration':'explicit 10 s warmup + 10 s estimation + 10 s unseen holdout'},
            'independent_evidence': copy.deepcopy(evidence), 'automatic_stationarity_proven': False,
            'applied_to_running_session': False, 'bias_axes': 'H30_NATIVE_UNVALIDATED'}


class CausalBiasEstimator:
    """Observe screened windows; confirm them with independent motion evidence.

    ``observe`` does not delay startup and never changes the accepted bias.
    ``confirm_stationary`` may accept a bounded update only after independently
    observed evidence covers entire windows. It must not be based on wheel/IMU
    prior or a map graph that simply copied that prior.

    Integrators must split at ``change_stamps`` and use ``bias_at(begin)`` per
    interval. Prune only after the integrator has anchored all older history.
    """
    def __init__(self, policy=None, *, confirmed_bias=None):
        self.policy = policy if isinstance(policy, BiasPolicy) else BiasPolicy(**(policy or {}))
        self.rows = deque()
        self.candidates = deque(maxlen=self.policy.max_pending_windows)
        self.confirmed = deque(maxlen=self.policy.robust_windows)
        self.versions = [BiasVersion(0, 0, (0., 0., 0.), ())]
        self.last_stamp_ns = None
        self.retained_from_ns = 0
        self.last_reason = 'NO_OBSERVATIONS_UNESTIMATED_ZERO'
        self.candidate_count = self.rejected_windows = self.gap_resets = 0
        self._next_candidate_id = 1
        from .mapping_planar import validate_confirmed_bias
        self.initial_confirmed_bias = validate_confirmed_bias(confirmed_bias)
        if self.initial_confirmed_bias is not None:
            value = self.initial_confirmed_bias
            self.versions = [BiasVersion(1, 0, tuple(value['bias_native_rad_s']), (value['evidence_id'],))]
            self.last_reason = 'EXPLICIT_EXTERNAL_CALIBRATION_DECLARATION'

    def bias_at(self, stamp_ns):
        stamp_ns = _stamp(stamp_ns)
        if stamp_ns < self.retained_from_ns:
            raise ValueError('bias query predates explicitly retained history')
        index = bisect_right([v.effective_stamp_ns for v in self.versions], stamp_ns)-1
        return np.asarray(self.versions[index].bias_native_rad_s).copy()

    def bias_metadata_at(self,stamp_ns):
        self.bias_at(stamp_ns) # Same retained-history and source-time checks.
        index=bisect_right([v.effective_stamp_ns for v in self.versions],stamp_ns)-1
        version=self.versions[index]
        return {'version':version.version,'effective_stamp_ns':version.effective_stamp_ns,
                'evidence_ids':list(version.evidence_ids),
                'calibration_status':'UNKNOWN_UNESTIMATED_ZERO_CANDIDATE' if version.version==0 else 'INDEPENDENT_EVIDENCE_DECLARED',
                'physical_accuracy_validated':False}

    def change_stamps(self, begin_ns, end_ns):
        _stamp(begin_ns); _stamp(end_ns)
        if end_ns < begin_ns or begin_ns < self.retained_from_ns:
            raise ValueError('invalid or retired bias interval')
        return [v.effective_stamp_ns for v in self.versions if begin_ns < v.effective_stamp_ns < end_ns]

    def prune_before(self, anchor_stamp_ns):
        anchor_stamp_ns = _stamp(anchor_stamp_ns)
        if anchor_stamp_ns < self.retained_from_ns or self.last_stamp_ns is None or anchor_stamp_ns > self.last_stamp_ns:
            raise ValueError('bias history retirement must follow an observed integrator anchor')
        index = bisect_right([v.effective_stamp_ns for v in self.versions], anchor_stamp_ns)-1
        del self.versions[:index]
        self.retained_from_ns = anchor_stamp_ns

    def _reset(self, reason):
        self.rows.clear()
        self.last_reason = reason

    def reset_observations(self, reason='SOURCE_COVERAGE_GAP'):
        """Discard only a partial fit window after validated source discontinuity."""
        self._reset(str(reason))
        self.gap_resets += 1

    def observe(self, stamp_ns, gyro_native_rad_s, accel_native_m_s2,
                wheel_stamp_ns, wheel_speeds_m_s):
        stamp_ns = _stamp(stamp_ns)
        gyro, accel = _vector(gyro_native_rad_s, 3), _vector(accel_native_m_s2, 3)
        wheel = _vector(wheel_speeds_m_s, 2)
        wheel_stamp_ns = _stamp(wheel_stamp_ns)
        # Distinct validated H30 packets can share one host receive timestamp.
        # Upstream owns identity/sequence checks; retain every genuine sample
        # without manufacturing extra time or dropping half of a burst.
        if self.last_stamp_ns is not None and stamp_ns < self.last_stamp_ns:
            raise ValueError('bias observations must have nondecreasing source time')
        if wheel_stamp_ns > stamp_ns:
            raise ValueError('future wheel observation cannot establish stationary evidence')
        if self.last_stamp_ns is not None and stamp_ns-self.last_stamp_ns > self.policy.max_imu_gap_s*1e9:
            self._reset('IMU_GAP_WINDOW_RESET'); self.gap_resets += 1
        self.last_stamp_ns = stamp_ns
        reason = None
        if stamp_ns-wheel_stamp_ns > self.policy.max_wheel_age_s*1e9:
            reason = 'WHEEL_COVERAGE_MISSING'
        elif np.max(np.abs(wheel)) > self.policy.max_wheel_speed_m_s:
            reason = 'WHEEL_MOTION'
        elif np.linalg.norm(gyro) > self.policy.max_gyro_sample_rad_s:
            reason = 'GYRO_MOTION_OR_OUTLIER'
        elif abs(np.linalg.norm(accel)-self.policy.gravity_m_s2) > self.policy.max_accel_norm_error_m_s2:
            reason = 'ACCELERATION_NOT_GRAVITY_LIKE'
        if reason:
            self._reset(reason)
            return None
        self.rows.append((stamp_ns, gyro.copy(), accel.copy()))
        if len(self.rows) > self.policy.max_samples:
            self._reset('WINDOW_SAMPLE_CAPACITY_NO_ESTIMATE')
            return None
        if len(self.rows) < self.policy.min_samples or stamp_ns-self.rows[0][0] < self.policy.window_s*1e9:
            self.last_reason = 'COLLECTING_BACKGROUND_WINDOW'
            return None
        gyros = np.asarray([row[1] for row in self.rows])
        accels = np.asarray([row[2] for row in self.rows])
        mean, amean = gyros.mean(axis=0), accels.mean(axis=0)
        gstd, astd = gyros.std(axis=0), accels.std(axis=0)
        units = accels/np.linalg.norm(accels, axis=1)[:, None]
        mean_accel_norm = float(np.linalg.norm(amean))
        # Finite individual samples can cancel. Such a window has no defined
        # gravity direction; rejecting its estimate must not poison status JSON.
        direction_p95 = (float(np.rad2deg(np.quantile(np.arccos(np.clip(
            units@(amean/mean_accel_norm), -1, 1)), .95))) if mean_accel_norm > 1e-12 else None)
        passed = (direction_p95 is not None and np.linalg.norm(mean) <= self.policy.max_gyro_mean_rad_s and
                  np.max(gstd) <= self.policy.max_gyro_axis_std_rad_s and
                  np.max(astd) <= self.policy.max_accel_axis_std_m_s2 and
                  direction_p95 <= self.policy.max_accel_direction_p95_deg)
        candidate = {'candidate_id': self._next_candidate_id, 'start_stamp_ns': self.rows[0][0],
                     'end_stamp_ns': stamp_ns, 'sample_count': len(self.rows),
                     'bias_native_rad_s': mean.tolist(), 'gyro_axis_std_rad_s': gstd.tolist(),
                     'mean_acceleration_native_m_s2': amean.tolist(), 'accel_axis_std_m_s2': astd.tolist(),
                     'accel_direction_p95_deg': direction_p95,
                     'stationary_truth_available': False, 'screening_passed': bool(passed)}
        self._next_candidate_id += 1
        self.rows.clear()
        if passed:
            self.candidates.append(candidate); self.candidate_count += 1
            self.last_reason = 'CANDIDATE_ONLY_AWAITING_INDEPENDENT_STATIONARITY'
        else:
            self.rejected_windows += 1
            self.last_reason = 'WINDOW_STATISTICS_REJECTED'
        return copy.deepcopy(candidate)

    def confirm_stationary(self, start_stamp_ns, end_stamp_ns, observed_stamp_ns, *,
                           evidence_id, source, independent):
        """Cover complete pending windows with causally received external evidence.

        ``source`` identifies a real registration result/measurement, not just
        a topic name. The caller is responsible for its quality and independence.
        False/absent evidence does not change the bias or mapping availability.
        """
        start, end, observed = map(_stamp, (start_stamp_ns, end_stamp_ns, observed_stamp_ns))
        if not isinstance(evidence_id, str) or not evidence_id or not isinstance(source, str) or not source:
            raise ValueError('stationarity evidence needs a source and unique identifier')
        if start > end or observed < end or self.last_stamp_ns is None or observed < self.last_stamp_ns:
            raise ValueError('stationarity evidence cannot be future-dated or retroactively applied')
        if independent is not True or source in ('wheel_imu_prior', 'copied_prior_graph', 'wheel_zero'):
            self.last_reason = 'NONINDEPENDENT_EVIDENCE_NO_UPDATE'
            return None
        if any(evidence_id == row['evidence_id'] for row in self.confirmed):
            self.last_reason = 'DUPLICATE_EVIDENCE_NO_UPDATE'
            return None
        matching = [row for row in self.candidates if start <= row['start_stamp_ns'] and row['end_stamp_ns'] <= end]
        if not matching:
            self.last_reason = 'NO_COMPLETE_SCREENED_WINDOW_IN_EVIDENCE'
            return None
        existing = {row['candidate']['candidate_id'] for row in self.confirmed}
        for candidate in matching:
            if candidate['candidate_id'] not in existing:
                self.confirmed.append({'candidate': candidate, 'evidence_id': evidence_id, 'source': source})
        consumed = {row['candidate_id'] for row in matching}
        self.candidates = deque((row for row in self.candidates if row['candidate_id'] not in consumed),
                                maxlen=self.policy.max_pending_windows)
        # One independent confirmation may cover multiple nonoverlapping windows.
        if len(self.confirmed) < self.policy.min_confirmed_windows:
            self.last_reason = 'MORE_CONFIRMED_WINDOWS_REQUIRED'
            return None
        target = np.median([row['candidate']['bias_native_rad_s'] for row in self.confirmed], axis=0)
        if np.linalg.norm(target) > self.policy.max_bias_norm_rad_s:
            self.last_reason = 'ROBUST_BIAS_MAGNITUDE_REJECTED'
            return None
        current = np.asarray(self.versions[-1].bias_native_rad_s)
        delta = target-current
        norm = float(np.linalg.norm(delta))
        if norm < 1e-12:
            self.last_reason = 'NO_BIAS_CHANGE'
            return None
        if len(self.versions) >= self.policy.max_versions:
            self.last_reason = 'HISTORY_CAPACITY_NO_UPDATE_UNTIL_ANCHORED'
            return None
        applied = current+delta*min(1., self.policy.max_update_norm_rad_s/norm)
        effective = max(observed, self.versions[-1].effective_stamp_ns+1)
        version = BiasVersion(self.versions[-1].version+1, effective, tuple(applied),
                              tuple(dict.fromkeys(row['evidence_id'] for row in self.confirmed)))
        self.versions.append(version)
        self.last_reason = 'INDEPENDENTLY_CORROBORATED_BOUNDED_UPDATE'
        return version

    def status(self):
        return {'status': self.last_reason, 'bias_estimated': self.versions[-1].version > 0,
                'accepted_version': asdict(self.versions[-1]), 'candidate_windows': self.candidate_count,
                'rejected_windows': self.rejected_windows, 'gap_resets': self.gap_resets,
                'pending_window_count': len(self.candidates), 'retained_versions': len(self.versions),
                'retained_from_ns': self.retained_from_ns, 'policy': asdict(self.policy),
                'policy_validation': 'EXPERIMENTAL_SCREENING_NOT_STATIONARY_TRUTH',
                'initial_confirmed_bias': copy.deepcopy(self.initial_confirmed_bias),
                'external_evidence_verified_by_estimator': False,
                'calibration_status':'UNKNOWN_UNESTIMATED_ZERO_CANDIDATE' if self.versions[-1].version==0 else 'INDEPENDENT_EVIDENCE_DECLARED',
                'gravity_correction_applied': False, 'startup_blocked': False}
