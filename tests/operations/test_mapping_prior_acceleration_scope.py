"""Acceleration eligibility is separate from gyro delivery; synthetic only."""
import json

import numpy as np
import pytest

from wc_runtime.mapping_prior import MotionPrior, PriorError
from test_mapping_prior import config, imu, wheel


def make_prior(*, planar=True, continuous=True, initialized=True):
    cfg = config()
    cfg.update(motion_model='planar_ekf' if planar else 'se3_gyro',
               continuous_mapping=continuous, policy={'history_s': 5.})
    p = MotionPrior(cfg, 'synthetic', clock=lambda: 1.)
    if initialized:
        p.add_wheel(wheel(0, 1_000_000_000))
        imu(p, 0, 1_000_000_000)
        assert p.initialization is not None
    return p


@pytest.mark.parametrize('continuous', [False, True])
def test_initialized_planar_retains_high_acceleration_and_same_gyro_trajectory(continuous):
    actual, baseline = make_prior(continuous=continuous), make_prior(continuous=continuous)
    for i in range(1, 5):
        stamp = 1_000_000_000+i*10_000_000
        for p in (actual, baseline):
            p.add_wheel(wheel(i, stamp, -100, 100))
        imu(actual, i, stamp, accel=(20., 20., 20.), gyro=(0., 0., .01))
        imu(baseline, i, stamp, gyro=(0., 0., .01))
    assert actual.failure is None
    assert np.array_equal(actual.imu[-1]['accel'], [20., 20., 20.])
    assert np.array_equal(actual.imu[-1]['gyro'], [0., 0., .01])
    a, b = actual._planar_state_at(stamp), baseline._planar_state_at(stamp)
    assert np.array_equal(a.x, b.x) and np.array_equal(a.P, b.P)
    assert a.counts['gyro'] == b.counts['gyro'] == 4
    report = actual.coverage_report()
    assert report['motion_coverage_complete'] is True
    assert report['coverage_gap_count'] == 0
    diag = report['imu_acceleration_diagnostics']
    assert diag['total_bound_exceeded_samples'] == 4
    assert diag['acceleration_fused_into_planar_motion'] is False
    last = diag['recent_examples'][-1]
    assert last['sequence'] == 4 and last['stamp_ns'] == stamp
    assert last['receive_monotonic_ns'] == stamp
    assert last['sensor_id'] == actual.config['imu_sensor_id']
    assert last['session_id'] == 'synthetic' and last['stream_epoch'] == 'imu-fixture'
    assert last['acceleration_native_m_s2'] == [20., 20., 20.]
    assert last['gyro_native_rad_s'] == [0., 0., .01]
    assert last['acceleration_norm_m_s2'] == pytest.approx(np.sqrt(1200.))
    assert last['gyro_norm_rad_s'] == .01
    assert last['max_accel_m_s2'] == 30. and last['max_gyro_rad_s'] == 3.
    json.dumps(report, allow_nan=False)


@pytest.mark.parametrize('accel,norm,recorded_sequence', [
    ([-35.192792, -9.729773, .770541], 36.521156, 15688),
    ([-33.741192, -5.197448, 2.236759], 34.212346, 15689),
])
def test_recorded_acceleration_outliers_keep_independent_valid_gyro(accel, norm, recorded_sequence):
    # Actual acceleration values from v7_core_20261006_130530, packets shown
    # above. Gyro here is synthetic; this test makes no physical-cause claim.
    actual, baseline = make_prior(), make_prior()
    for p in (actual, baseline):
        p.add_wheel(wheel(1, 1_010_000_000))
    imu(actual, 1, 1_010_000_000, accel=accel, gyro=(0., 0., .01))
    imu(baseline, 1, 1_010_000_000, gyro=(0., 0., .01))
    for p in (actual, baseline):
        p.add_wheel(wheel(2, 1_020_000_000))
        imu(p, 2, 1_020_000_000)
    a, b = actual._planar_state_at(1_020_000_000), baseline._planar_state_at(1_020_000_000)
    assert np.array_equal(a.x, b.x) and np.array_equal(a.P, b.P)
    diag = actual.coverage_report()['imu_acceleration_diagnostics']
    assert diag['total_bound_exceeded_samples'] == 1
    assert diag['recent_examples'][0]['acceleration_native_m_s2'] == accel
    assert diag['recent_examples'][0]['acceleration_norm_m_s2'] == pytest.approx(norm, abs=2e-6)


def test_bad_acceleration_cannot_bridge_stationary_bias_candidate(monkeypatch):
    p = make_prior()
    assert p.bias_estimator.rows
    called = []
    original = p.bias_estimator.observe
    monkeypatch.setattr(p.bias_estimator, 'observe', lambda *a, **k: called.append(a))
    p.add_wheel(wheel(1, 1_010_000_000))
    imu(p, 1, 1_010_000_000, accel=(0., 0., 31.))
    assert called == [] and not p.bias_estimator.rows
    assert p.bias_estimator.last_reason == 'ACCELERATION_BOUND_EXCEEDED_NOT_FUSED_PLANAR'
    # Ordinary next packets start a fresh partial window; no bias is applied.
    monkeypatch.setattr(p.bias_estimator, 'observe', original)
    p.add_wheel(wheel(2, 1_020_000_000))
    imu(p, 2, 1_020_000_000)
    assert len(p.bias_estimator.rows) == 1
    assert np.array_equal(p.bias_estimator.bias_at(1_020_000_000), [0., 0., 0.])


@pytest.mark.parametrize('planar,continuous,initialized', [
    (True, True, False), (True, False, False),
    (False, True, False), (False, True, True), (False, False, False),
])
def test_startup_and_se3_acceleration_bound_remain_strict(planar, continuous, initialized):
    p = make_prior(planar=planar, continuous=continuous, initialized=initialized)
    with pytest.raises(PriorError, match='SI magnitude exceeds'):
        imu(p, 1, 1_010_000_000, accel=(0., 0., 31.))
    assert p.failure is not None
    assert p.imu_accel_bound_count == 0


@pytest.mark.parametrize('accel', [(0., 0., 9.80665), (0., 0., 31.)])
def test_gyro_bound_remains_strict_even_if_acceleration_not_fused(accel):
    p = make_prior()
    with pytest.raises(PriorError, match='SI magnitude exceeds'):
        imu(p, 1, 1_010_000_000, gyro=(0., 0., 3.01), accel=accel)
    assert p.last_seen['imu']['sequence'] == 0
    assert p.imu_accel_bound_count == 0


@pytest.mark.parametrize('accel', [(0., 0., float('nan')), (0., 0., float('inf')), (0., 31.)])
def test_nonfinite_or_malformed_acceleration_is_not_an_eligible_diagnostic(accel):
    p = make_prior()
    with pytest.raises(PriorError):
        imu(p, 1, 1_010_000_000, accel=accel)
    assert p.imu_accel_bound_count == 0


@pytest.mark.parametrize('extra', [
    {'sensor_id': 'wrong'}, {'session_id': 'wrong'}, {'stream_epoch': 'new-epoch'},
    {'sequence': 0}, {'stamp_ns': 999_000_000}, {'monotonic_ns': 999_000_000},
])
def test_bad_acceleration_never_bypasses_identity_sequence_or_time_checks(extra):
    p = make_prior()
    with pytest.raises(PriorError):
        imu(p, 1, 1_010_000_000, accel=(0., 0., 31.), **extra)
    assert p.last_seen['imu']['sequence'] == 0
    assert p.imu_accel_bound_count == 0


def test_diagnostic_examples_are_bounded_but_total_counts_all_packets():
    p = make_prior()
    for i in range(1, 81):
        stamp = 1_000_000_000+i*10_000_000
        p.add_wheel(wheel(i, stamp))
        imu(p, i, stamp, accel=(0., 0., 31.))
    report = p.coverage_report()['imu_acceleration_diagnostics']
    assert report['total_bound_exceeded_samples'] == 80
    assert len(report['recent_examples']) == report['example_capacity'] == 64
    assert report['recent_examples'][0]['sequence'] == 17
    assert report['recent_examples'][-1]['sequence'] == 80
    json.dumps(report, allow_nan=False)


def test_at_bound_remains_ordinary_bias_observation(monkeypatch):
    p = make_prior()
    observed = []
    monkeypatch.setattr(p, '_observe_bias', lambda row: observed.append(row))
    p.add_wheel(wheel(1, 1_010_000_000))
    imu(p, 1, 1_010_000_000, accel=(0., 0., 30.))
    assert len(observed) == 1 and p.imu_accel_bound_count == 0
