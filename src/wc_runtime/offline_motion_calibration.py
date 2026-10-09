"""Auditable, session-bound offline motion consistency calibration.

This module never opens devices or writes configuration.  A validated candidate
is a same-recording consistency model, not an absolute wheel/IMU calibration.
"""
import copy
import hashlib
import json
import math
import re

import numpy as np


SCHEMA = 'wc_offline_motion_calibration_candidate_v1'
VALIDATED = 'VALIDATED_FOR_OFFLINE_REPLAY'
DEFAULT_POLICY = {
    'warmup_min_s': 10.0,
    'bias_window_min_s': 8.0,
    'bias_min_samples': 500,
    'max_imu_gap_s': 0.1,
    'max_wheel_gap_s': 0.5,
    'stationary_max_v_m_s': 0.002,
    'stationary_max_w_rad_s': 0.01,
    'stationary_gyro_std_max_rad_s': 0.004,
    'stationary_accel_std_max_m_s2': 0.1,
    'gravity_m_s2': 9.80665,
    'gravity_norm_error_max_m_s2': 0.25,
    'bias_norm_max_rad_s': 0.0015,
    'validation_bias_residual_norm_max_rad_s': 0.00025,
    'bin_s': 0.1,
    'min_packets_per_bin': 5,
    'min_bins': 200,
    'turn_threshold_rad_s': 0.1,
    'min_bins_each_turn_direction': 30,
    'max_design_condition_number': 50.0,
    'robust_mad_sigma_floor_rad_s': 0.001,
    'robust_reject_sigma': 4.5,
    'robust_huber_sigma': 1.5,
    'robust_max_rejected_fraction': 0.1,
    'scale_min': 0.5,
    'scale_max': 1.5,
    'speed_coefficient_max_abs_per_m': 0.1,
    'holdout_rmse_max_rad_s': 0.03,
    'holdout_rmse_max_fraction_of_baseline': 0.9,
    'direction_rmse_max_fraction_of_baseline': 1.0,
    'direction_sign_agreement_min': 0.99,
    'covariance_floor_rad2_s2': 1e-6,
}


class CalibrationError(ValueError):
    pass


def _candidate_digest(candidate):
    # Freeze diagnostics and covariance as well as coefficients and provenance.
    content = {k:v for k,v in candidate.items() if k != 'candidate_id'}
    return hashlib.sha256(json.dumps(content, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()).hexdigest()


def _finite(value, name):
    try:
        out = np.asarray(value, dtype=float)
    except (ValueError, TypeError) as exc:
        raise CalibrationError(name + ': not numeric') from exc
    if not np.isfinite(out).all():
        raise CalibrationError(name + ': non-finite value')
    return out


def _provenance(value):
    p = copy.deepcopy(value)
    for k in ('session_id', 'imu_device_id', 'wheel_device_id'):
        if not isinstance(p.get(k), str) or not p[k]:
            raise CalibrationError('missing provenance ' + k)
    sources = p.get('sources')
    if not isinstance(sources, list) or not sources:
        raise CalibrationError('nonempty source evidence hashes required')
    for source in sources:
        if not isinstance(source, dict) or not isinstance(source.get('path'), str) or not source['path']:
            raise CalibrationError('source evidence path required')
        if not re.fullmatch('[0-9a-f]{64}', source.get('sha256', '')):
            raise CalibrationError('source evidence SHA-256 required')
    R = _finite(p.get('R_reference_imu'), 'R_reference_imu')
    if R.shape != (3, 3) or not np.allclose(R @ R.T, np.eye(3), atol=1e-8) or not math.isclose(float(np.linalg.det(R)), 1., abs_tol=1e-8):
        raise CalibrationError('R_reference_imu must be a proper rotation')
    return p, R


def _windows(value, policy):
    names = ('warmup', 'bias_train', 'bias_validation', 'wheel_train', 'wheel_validation')
    if set(value) != set(names):
        raise CalibrationError('explicit warmup, bias train/validation and wheel train/validation windows required')
    out = {}
    for name in names:
        v = _finite(value[name], name)
        if v.shape != (2,) or v[1] <= v[0]:
            raise CalibrationError('invalid half-open window: ' + name)
        out[name] = v.tolist()
    # All five windows are independent, not merely the two regression windows.
    for i, a in enumerate(names):
        for b in names[i+1:]:
            if max(out[a][0], out[b][0]) < min(out[a][1], out[b][1]):
                raise CalibrationError('overlapping windows: ' + a + ' / ' + b)
    if out['warmup'][1] > out['bias_train'][0] or out['warmup'][1]-out['warmup'][0] < policy['warmup_min_s']:
        raise CalibrationError('warmup must precede bias estimation and have sufficient duration')
    if out['bias_validation'][0] <= out['bias_train'][1]:
        raise CalibrationError('independent later bias validation required')
    for name in ('bias_train', 'bias_validation'):
        if out[name][1]-out[name][0] < policy['bias_window_min_s']:
            raise CalibrationError('bias window too short')
    return out


def _samples(rows, kind, p):
    fields = ('t_s', 'gyro_native_rad_s', 'accel_native_m_s2') if kind == 'imu' else ('t_s', 'v_m_s', 'w_rad_s')
    out = []
    for row in rows:
        if row.get('device_id') != p[kind+'_device_id'] or row.get('session_id') != p['session_id']:
            raise CalibrationError(kind + ': cross-device or cross-session sample')
        if row.get('calibration_applied') or row.get('gyro_bias_applied') or row.get('wheel_yaw_corrected'):
            raise CalibrationError(kind + ': raw samples required; correction already applied')
        try:
            if kind == 'imu':
                g = _finite(row[fields[1]], 'raw gyro')
                a = _finite(row[fields[2]], 'raw acceleration')
                if g.shape != (3,) or a.shape != (3,):
                    raise CalibrationError('gyro/acceleration must have three axes')
                values = [float(row['t_s']), *g, *a]
            else:
                values = [row[field] for field in fields]
        except (KeyError, TypeError, ValueError) as exc:
            raise CalibrationError(kind + ': invalid sample') from exc
        out.append(_finite(values, kind + ' sample'))
    arr = np.asarray(out)
    if len(arr) < 2:
        raise CalibrationError(kind + ': insufficient samples')
    dt = np.diff(arr[:, 0])
    if np.any(dt < 0) or (kind == 'wheel' and np.any(dt == 0)):
        raise CalibrationError(kind + ': time sequence not ordered')
    return arr


def _interp(t, values, query, gap):
    query = np.asarray(query)
    k = np.clip(np.searchsorted(t, query, side='right'), 1, len(t)-1)
    ok = (query >= t[0]) & (query <= t[-1]) & ((t[k]-t[k-1]) <= gap)
    out = np.column_stack([np.interp(query, t, values[:, j]) for j in range(values.shape[1])])
    out[~ok] = np.nan
    return out


def _static_window(I, W, bounds, policy):
    a, b = bounds
    q = I[(I[:, 0] >= a) & (I[:, 0] < b)]
    problems = []
    if len(q) < policy['bias_min_samples']:
        return None, ['STATIC_TOO_FEW_SAMPLES']
    dt = np.diff(np.r_[a, np.unique(q[:, 0]), b])
    if np.max(dt) > policy['max_imu_gap_s']:
        problems.append('STATIC_IMU_COVERAGE_GAP')
    speeds = _interp(W[:, 0], W[:, 1:3], q[:, 0], policy['max_wheel_gap_s'])
    if not np.isfinite(speeds).all():
        problems.append('STATIC_WHEEL_COVERAGE_MISSING')
    elif np.max(abs(speeds[:, 0])) > policy['stationary_max_v_m_s'] or np.max(abs(speeds[:, 1])) > policy['stationary_max_w_rad_s']:
        problems.append('STATIC_WHEELS_MOVING')
    gs, am, ast = q[:, 1:4].std(axis=0), q[:, 4:7].mean(axis=0), q[:, 4:7].std(axis=0)
    if np.max(gs) > policy['stationary_gyro_std_max_rad_s']:
        problems.append('STATIC_GYRO_UNSTABLE')
    if np.max(ast) > policy['stationary_accel_std_max_m_s2'] or abs(np.linalg.norm(am)-policy['gravity_m_s2']) > policy['gravity_norm_error_max_m_s2']:
        problems.append('STATIC_ACCEL_UNSTABLE')
    return {'sample_count': len(q), 'window_s': list(bounds), 'gyro_mean_rad_s': q[:, 1:4].mean(axis=0).tolist(), 'gyro_std_rad_s': gs.tolist(), 'accel_mean_m_s2': am.tolist(), 'accel_std_m_s2': ast.tolist(), 'independent_static_truth': False, '_samples': q}, problems


def _paired_bins(I, W, bounds, bias, R, policy):
    a, b = bounds
    starts = np.arange(a, b-policy['bin_s']+1e-8, policy['bin_s'])
    centers = starts+policy['bin_s']*.5
    speed = _interp(W[:, 0], W[:, 1:3], centers, policy['max_wheel_gap_s'])
    gyro = np.full(len(starts), np.nan)
    count = np.zeros(len(starts), dtype=int)
    for i, start in enumerate(starts):
        j, k = np.searchsorted(I[:, 0], [start, start+policy['bin_s']], side='left')
        if k-j < policy['min_packets_per_bin']:
            continue
        if np.max(np.diff(np.r_[start, np.unique(I[j:k, 0]), start+policy['bin_s']])) > policy['max_imu_gap_s']:
            continue
        gyro[i] = float((R @ (I[j:k, 1:4].mean(axis=0)-bias))[2])
        count[i] = k-j
    ok = np.isfinite(gyro) & np.isfinite(speed).all(axis=1)
    return np.column_stack([centers[ok], speed[ok], gyro[ok]]), {'requested_bins': len(starts), 'retained_bins': int(ok.sum()), 'missing_bins': int((~ok).sum()), 'imu_packet_count': int(count[ok].sum())}


def _excitation(q, policy):
    if len(q) < policy['min_bins']:
        return ['TOO_FEW_MOTION_BINS']
    X = q[:, [2, 1]]  # wheel angular rate, then forward speed; no intercept
    errors = []
    if np.linalg.matrix_rank(X) != 2 or np.linalg.cond(X) > policy['max_design_condition_number']:
        errors.append('WEAK_OR_COLLINEAR_EXCITATION')
    for sign in (-1, 1):
        if np.sum(q[:, 2]*sign > policy['turn_threshold_rad_s']) < policy['min_bins_each_turn_direction']:
            errors.append('INSUFFICIENT_BOTH_TURN_DIRECTIONS')
            break
    return errors


def _robust_fit(q, policy):
    X, y = q[:, [2, 1]], q[:, 3]
    beta = np.linalg.lstsq(X, y, rcond=None)[0]
    for _ in range(12):
        residual = y-X @ beta
        center = np.median(residual)
        scale = max(1.4826*np.median(abs(residual-center)), policy['robust_mad_sigma_floor_rad_s'])
        dist = abs(residual-center)
        weights = np.minimum(1., policy['robust_huber_sigma']*scale/np.maximum(dist, 1e-15))
        beta = np.linalg.lstsq(X*np.sqrt(weights[:, None]), y*np.sqrt(weights), rcond=None)[0]
    residual = y-X @ beta
    center = np.median(residual)
    scale = max(1.4826*np.median(abs(residual-center)), policy['robust_mad_sigma_floor_rad_s'])
    keep = abs(residual-center) <= policy['robust_reject_sigma']*scale
    beta = np.linalg.lstsq(X[keep], y[keep], rcond=None)[0]
    return beta, keep, scale


def _metrics(q, beta, policy):
    before = q[:, 2]-q[:, 3]
    corrected = q[:, [2, 1]] @ beta
    after = corrected-q[:, 3]
    def values(mask):
        if not np.any(mask):
            return {'samples': 0}
        return {'samples': int(mask.sum()), 'before_rmse_rad_s': float(np.sqrt(np.mean(before[mask]**2))), 'after_rmse_rad_s': float(np.sqrt(np.mean(after[mask]**2))), 'after_mean_rad_s': float(after[mask].mean()), 'after_p95_abs_rad_s': float(np.percentile(abs(after[mask]), 95)), 'corrected_reference_sign_agreement': float(np.mean(np.sign(corrected[mask]) == np.sign(q[mask, 3])))}
    return {'all': values(np.ones(len(q), bool)), 'positive_turn': values(q[:, 2] > policy['turn_threshold_rad_s']), 'negative_turn': values(q[:, 2] < -policy['turn_threshold_rad_s'])}


def select_windows(imu_samples, wheel_samples, *, provenance, policy=None):
    """Select disjoint windows using stationary screening, not fitted residuals.

    Requires a stable beginning and a separate stable end around a motion span.
    No session-specific times. The caller must freeze the returned windows in
    the experiment manifest before fitting or comparing candidate maps.
    """
    p, _ = _provenance(provenance)
    pol = dict(DEFAULT_POLICY)
    if policy:
        if set(policy)-set(pol):
            raise CalibrationError('unknown policy keys')
        pol.update(policy)
    I, W = _samples(imu_samples, 'imu', p), _samples(wheel_samples, 'wheel', p)
    lo = math.ceil(max(I[0, 0], W[0, 0]))
    hi = math.floor(min(I[-1, 0], W[-1, 0]))
    small = dict(pol, bias_min_samples=50)
    good = []
    for a in range(lo, hi):
        _, errors = _static_window(I, W, [a, a+1], small)
        good.append(not errors)
    edges = np.diff(np.r_[False, good, False].astype(int))
    runs = [(lo+int(a), lo+int(b)) for a, b in zip(np.where(edges == 1)[0], np.where(edges == -1)[0])]
    needed = pol['warmup_min_s']+max(20., pol['bias_window_min_s'])+2.
    heads = [(a, b) for a, b in runs if a <= lo+3 and b-a >= needed]
    tails = [(a, b) for a, b in runs if b >= hi-3 and b-a >= pol['bias_window_min_s']+4]
    if not heads or not tails or heads[0] == tails[-1]:
        raise CalibrationError('reliable separate initial and final stationary segments unavailable')
    head, tail = heads[0], tails[-1]
    motion_a, motion_b = head[1]+1., tail[0]-1.
    if motion_b-motion_a < 2*pol['min_bins']*pol['bin_s']:
        raise CalibrationError('motion span too short for independent holdout')
    split = round((motion_a+motion_b)*.5, 1)
    warm_end = head[0]+pol['warmup_min_s']
    tail_a = tail[0]+2.
    windows = {'warmup': [float(head[0]), warm_end],
        'bias_train': [warm_end, min(warm_end+20., head[1]-1.)],
        'bias_validation': [tail_a, min(tail_a+40., tail[1]-2.)],
        'wheel_train': [motion_a, split], 'wheel_validation': [split, motion_b]}
    return _windows(windows, pol)


def build_candidate(imu_samples, wheel_samples, *, provenance, windows, policy=None):
    """Build a candidate without changing supplied inputs or any configuration.

    Samples use t_s relative to a documented common arrival-time origin. Windows
    are half-open.  All identities and all numeric samples fail closed. Quality
    failures produce REJECTED with reasons, never a silently relaxed threshold.
    """
    p, R = _provenance(provenance)
    pol = dict(DEFAULT_POLICY)
    if policy:
        unknown = set(policy)-set(pol)
        if unknown:
            raise CalibrationError('unknown policy keys: '+str(sorted(unknown)))
        pol.update(policy)
    for key, value in pol.items():
        if not math.isfinite(float(value)) or float(value) <= 0:
            raise CalibrationError('policy must contain finite positive values: '+key)
    win = _windows(windows, pol)
    I, W = _samples(imu_samples, 'imu', p), _samples(wheel_samples, 'wheel', p)
    out = {'schema': SCHEMA, 'status': 'REJECTED', 'formal_calibration': False, 'automatic_apply': False,
           'scope': 'same-session offline wheel/gyro consistency; not absolute radius/track calibration',
           'provenance': p, 'windows': win, 'policy': pol, 'validation': {'passed': False, 'reasons': [], 'independent_geometric_truth': False},
           'bias_native_rad_s': None, 'wheel_yaw_scale': None, 'wheel_yaw_speed_coefficient': None,
           'equation': 'corrected_wheel_w = a * raw_wheel_w + b * raw_forward_v; gyro bias subtracted in native axes before R_reference_imu',
           'intercept_rad_s': 0., 'intercept_policy': 'FIXED_ZERO_AFTER_INDEPENDENT_STATIC_BIAS',
           'parameter_units': {'bias_native_rad_s': 'rad/s', 'wheel_yaw_scale': 'dimensionless', 'wheel_yaw_speed_coefficient': '1/m'}}
    reasons = out['validation']['reasons']
    warm, errors = _static_window(I, W, win['warmup'], pol)
    reasons.extend('WARMUP_'+e for e in errors)
    train, errors = _static_window(I, W, win['bias_train'], pol)
    reasons.extend('BIAS_TRAIN_'+e for e in errors)
    validation, errors = _static_window(I, W, win['bias_validation'], pol)
    reasons.extend('BIAS_VALIDATION_'+e for e in errors)
    if reasons:
        return out
    raw = train['_samples'][:, 1:4]
    med = np.median(raw, axis=0)
    scale = np.maximum(1.4826*np.median(abs(raw-med), axis=0), 0.0002)
    keep = np.all(abs(raw-med) <= 5.*scale, axis=1)
    if np.mean(~keep) > pol['robust_max_rejected_fraction'] or keep.sum() < pol['bias_min_samples']:
        reasons.append('BIAS_TRAIN_EXCESSIVE_OUTLIERS')
        return out
    bias = raw[keep].mean(axis=0)
    if np.linalg.norm(bias) > pol['bias_norm_max_rad_s']:
        reasons.append('BIAS_NORM_EXCEEDED')
    residual = validation['_samples'][:, 1:4]-bias
    residual_mean = residual.mean(axis=0)
    if np.linalg.norm(residual_mean) > pol['validation_bias_residual_norm_max_rad_s']:
        reasons.append('INDEPENDENT_STATIC_BIAS_VALIDATION_FAILED')
    out['bias_native_rad_s'] = bias.tolist()
    out['bias_validation'] = {'warmup': {k:v for k,v in warm.items() if k != '_samples'},
        'train': {k:v for k,v in train.items() if k != '_samples'},
        'validation': {k:v for k,v in validation.items() if k != '_samples'},
        'train_rejected_samples': int((~keep).sum()), 'validation_residual_mean_rad_s': residual_mean.tolist(),
        'validation_residual_norm_rad_s': float(np.linalg.norm(residual_mean)), 'validation_samples_trimmed': False,
        'gyro_noise_variance_native_rad2_s2': raw[keep].var(axis=0, ddof=1).tolist()}
    if reasons:
        return out
    tr, tc = _paired_bins(I, W, win['wheel_train'], bias, R, pol)
    va, vc = _paired_bins(I, W, win['wheel_validation'], bias, R, pol)
    out['coverage'] = {'training': tc, 'holdout': vc}
    reasons.extend('TRAIN_'+e for e in _excitation(tr, pol))
    reasons.extend('HOLDOUT_'+e for e in _excitation(va, pol))
    # Missing bins cannot silently disappear from candidate acceptance.
    if tc['missing_bins'] or vc['missing_bins']:
        reasons.append('MOTION_COVERAGE_MISSING_BINS')
    if reasons:
        return out
    beta, accepted, robust_scale = _robust_fit(tr, pol)
    if np.mean(~accepted) > pol['robust_max_rejected_fraction']:
        reasons.append('EXCESSIVE_MOTION_OUTLIERS')
    reasons.extend('FILTERED_TRAIN_'+e for e in _excitation(tr[accepted], pol))
    if not pol['scale_min'] <= beta[0] <= pol['scale_max'] or abs(beta[1]) > pol['speed_coefficient_max_abs_per_m']:
        reasons.append('COEFFICIENT_OUT_OF_CANDIDATE_BOUNDS')
    out['wheel_yaw_scale'], out['wheel_yaw_speed_coefficient'] = map(float, beta)
    out['robust_fit'] = {'method': 'Huber IRLS then 4.5 MAD cutoff, fixed zero intercept', 'mad_sigma_rad_s': robust_scale,
        'training_rejected_bins': int((~accepted).sum()), 'training_rejected_t_s': tr[~accepted, 0].tolist(),
        'training_design_condition_number': float(np.linalg.cond(tr[accepted][:, [2, 1]])),
        'holdout_residual_filtering': False}
    out['training_metrics'] = _metrics(tr, beta, pol)
    out['holdout_metrics'] = _metrics(va, beta, pol)
    m = out['holdout_metrics']['all']
    if m['after_rmse_rad_s'] > pol['holdout_rmse_max_rad_s']:
        reasons.append('HOLDOUT_ABSOLUTE_RMSE_EXCEEDED')
    if m['after_rmse_rad_s'] > m['before_rmse_rad_s']*pol['holdout_rmse_max_fraction_of_baseline']:
        reasons.append('HOLDOUT_INSUFFICIENT_IMPROVEMENT')
    for name in ('positive_turn', 'negative_turn'):
        d = out['holdout_metrics'][name]
        if d['after_rmse_rad_s'] > d['before_rmse_rad_s']*pol['direction_rmse_max_fraction_of_baseline']:
            reasons.append(name.upper()+'_RMSE_WORSE')
        if d['corrected_reference_sign_agreement'] < pol['direction_sign_agreement_min']:
            reasons.append(name.upper()+'_SIGN_DISAGREEMENT')
    # Covariance is estimated only from training. Do not shrink by packet count:
    # arrival batches are correlated. Both sensors get the FULL relative-error
    # second moment, not an unidentifiable split of that variance.
    rows = I[(I[:, 0] >= win['wheel_train'][0]) & (I[:, 0] < win['wheel_train'][1])]
    speeds = _interp(W[:, 0], W[:, 1:3], rows[:, 0], pol['max_wheel_gap_s'])
    target = (rows[:, 1:4]-bias) @ R[2]
    highrate_residual = target-(beta[0]*speeds[:, 1]+beta[1]*speeds[:, 0])
    highrate_residual = highrate_residual[np.isfinite(highrate_residual)]
    covariance = np.cov(raw[keep].T)
    static_yaw_var = float(R[2] @ covariance @ R[2])
    relative_mse = float(np.mean(highrate_residual**2))
    noise = max(relative_mse, static_yaw_var, pol['covariance_floor_rad2_s2'])
    out['covariance_candidates'] = {'status': 'EXPERIMENTAL_FROM_TRAINING_RELATIVE_RESIDUAL_NOT_SENSOR_TRUTH',
        'gyro_w_variance': noise, 'wheel_w_variance': noise, 'stationary_gyro_yaw_variance': static_yaw_var,
        'training_highrate_relative_second_moment': relative_mse, 'training_highrate_samples': len(highrate_residual),
        'holdout_binned_relative_second_moment': m['after_rmse_rad_s']**2,
        'unit': 'rad^2/s^2', 'variance_allocation': 'each sensor assigned full observed training relative-error second moment; individual noise not identifiable',
        'observation_domain': 'CORRECTED_GYRO_YAW_AND_CORRECTED_WHEEL_YAW; not raw wheel covariance; do not multiply these values by a squared again',
        'application_policy': 'REPORT_ONLY_IN_COEFFICIENT_COMPARISON; changing noise is a separate experiment factor',
        'innovation_gate_sigma_unchanged': True, 'not_independently_validated_as_individual_sensor_covariance': True,
        'batch_correlation_warning': 'same-time packets are correlated; no division by nominal rate or batch count'}
    if not reasons:
        out['status'] = VALIDATED
        out['validation']['passed'] = True
    out['candidate_id'] = _candidate_digest(out)
    return out


def validate_for_session(candidate, *, session_id, imu_device_id, wheel_device_id):
    if candidate.get('schema') != SCHEMA or candidate.get('status') != VALIDATED or candidate.get('validation', {}).get('passed') is not True:
        raise CalibrationError('candidate has not passed offline validation')
    if candidate['validation'].get('reasons') != []:
        raise CalibrationError('candidate retains validation failures')
    p, _ = _provenance(candidate.get('provenance', {}))
    if (session_id, imu_device_id, wheel_device_id) != (p['session_id'], p['imu_device_id'], p['wheel_device_id']):
        raise CalibrationError('candidate belongs to a different session or device')
    b = _finite(candidate.get('bias_native_rad_s'), 'candidate gyro bias')
    if b.shape != (3,):
        raise CalibrationError('candidate bias shape')
    coeff = _finite([candidate.get('wheel_yaw_scale'), candidate.get('wheel_yaw_speed_coefficient')], 'candidate wheel coefficients')
    pol = candidate.get('policy', {})
    if set(pol) != set(DEFAULT_POLICY) or any(not math.isfinite(float(v)) or float(v) <= 0 for v in pol.values()):
        raise CalibrationError('invalid candidate policy')
    if not pol['scale_min'] <= coeff[0] <= pol['scale_max'] or abs(coeff[1]) > pol['speed_coefficient_max_abs_per_m'] or np.linalg.norm(b) > pol['bias_norm_max_rad_s']:
        raise CalibrationError('candidate coefficients exceed validation bounds')
    _windows(candidate.get('windows', {}), pol)
    # Catch editing of source hashes, identities, windows or fitted coefficients.
    digest = _candidate_digest(candidate)
    if candidate.get('candidate_id') != digest:
        raise CalibrationError('candidate content hash mismatch')
    return True


def correct_measurement(sample, *, kind, candidate):
    """Return a new same-session sample with raw values retained; fail on reuse.

    Caller must explicitly choose to use corrected fields. No production config
    is changed and no caller's arrays/dicts are mutated.
    """
    if kind not in ('imu', 'wheel'):
        raise CalibrationError('kind must be imu or wheel')
    p = candidate.get('provenance', {})
    validate_for_session(candidate, session_id=sample.get('session_id'), imu_device_id=sample.get('device_id') if kind == 'imu' else p.get('imu_device_id'), wheel_device_id=sample.get('device_id') if kind == 'wheel' else p.get('wheel_device_id'))
    if sample.get('calibration_applied') or sample.get('gyro_bias_applied') or sample.get('wheel_yaw_corrected'):
        raise CalibrationError('calibration already applied; refusing double correction')
    out = copy.deepcopy(sample)
    if kind == 'imu':
        raw = _finite(sample.get('gyro_native_rad_s'), 'raw gyro')
        if raw.shape != (3,):
            raise CalibrationError('gyro must have three axes')
        corrected = raw-np.asarray(candidate['bias_native_rad_s'])
        out['gyro_corrected_native_rad_s'] = corrected.tolist()
        out['gyro_corrected_reference_rad_s'] = (np.asarray(p['R_reference_imu']) @ corrected).tolist()
        out['gyro_bias_applied'] = True
    else:
        v, w = _finite([sample.get('v_m_s'), sample.get('w_rad_s')], 'raw wheel')
        out['w_corrected_rad_s'] = candidate['wheel_yaw_scale']*w+candidate['wheel_yaw_speed_coefficient']*v
        out['wheel_yaw_corrected'] = True
    out['calibration_applied'] = candidate['candidate_id']
    return out
