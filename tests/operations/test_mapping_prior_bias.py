"""Background bias integration, including delayed evidence and immutable poses."""
import numpy as np
import json
import pytest
from scipy.spatial.transform import Rotation

from wc_runtime.mapping_bias import CausalBiasEstimator
from wc_runtime.mapping_prior import integrate_body_twist
from test_mapping_continuous_prior import continuous
from test_mapping_prior import imu, wheel


def feed(prior, first, last, *, gyro=(0, 0, .0006), raw=0):
    for index in range(first, last+1):
        stamp = 1_000_000_000+index*10_000_000
        prior.add_wheel(wheel(index, stamp, -raw, raw))
        imu(prior, index, stamp, gyro=gyro)


def short_windows(prior):
    prior.bias_estimator = CausalBiasEstimator({'window_s': .2, 'min_samples': 21})


def test_screened_slow_rotation_never_silently_changes_pose_or_blocks_startup():
    prior = continuous(history_s=5.)
    short_windows(prior)
    feed(prior, 0, 60)
    assert prior.initialization['stamp_ns'] == 1_000_000_000
    assert prior.bias_estimator.candidate_count >= 2
    assert prior.bias_report()['automatic_confirmation_enabled'] is False
    assert prior.bias_report()['bias_estimated'] is False
    yaw = Rotation.from_matrix(prior.pose_at(1_600_000_000)[:3, :3]).as_rotvec()[2]
    assert yaw == pytest.approx(.0006*.6, abs=1e-12)
    assert prior.confirm_bias_stationarity(1_000_000_000, 1_600_000_000, 1_600_000_000,
        evidence_id='wheel-only', source='wheel_zero', independent=True) is None
    assert np.array_equal(prior.bias_estimator.bias_at(1_600_000_000), np.zeros(3))


def test_late_confirmation_preserves_published_history_and_splits_between_imu_samples():
    prior = continuous(history_s=5.)
    short_windows(prior)
    feed(prior, 0, 60)
    saved = prior.pose_at(1_550_000_000)
    prior.record_forwarded(1_550_000_000, saved)
    before = prior.pose_at(1_600_000_000)
    # Actual evidence can arrive between IMU observations. Changes start then.
    version = prior.confirm_bias_stationarity(1_000_000_000, 1_500_000_000, 1_605_000_000,
        evidence_id='synthetic-lidar', source='synthetic_independent_measurement', independent=True)
    assert version.effective_stamp_ns == 1_605_000_000
    assert np.array_equal(prior.pose_at(1_550_000_000), saved)
    assert np.array_equal(prior.pose_at(1_600_000_000), before)
    feed(prior, 61, 62)
    bias = np.asarray(version.bias_native_rad_s)
    expected = integrate_body_twist(before, np.zeros(3), np.array([0, 0, .0006]), .005)
    expected = integrate_body_twist(expected, np.zeros(3), np.array([0, 0, .0006])-bias, .015)
    assert np.allclose(prior.pose_at(1_620_000_000), expected, atol=1e-13)
    report = prior.sample_report(1_620_000_000, expected)
    assert np.allclose(report['gyro_bias_applied_native_rad_s'], bias)


def test_retirement_keeps_applied_bias_and_missing_wheel_only_resets_candidate():
    prior = continuous(history_s=.3)
    short_windows(prior)
    feed(prior, 0, 60)
    prior.confirm_bias_stationarity(1_000_000_000, 1_600_000_000, 1_600_000_000,
        evidence_id='synthetic-lidar', source='synthetic_independent_measurement', independent=True)
    feed(prior, 61, 130)
    anchor = prior.checkpoints[0][0]
    assert prior.bias_estimator.retained_from_ns == anchor
    assert len(prior.bias_estimator.versions) == 1
    bias = prior.bias_estimator.bias_at(2_300_000_000)
    expected_yaw = .0006*.6+(.0006-bias[2])*.7
    assert Rotation.from_matrix(prior.pose_at(2_300_000_000)[:3, :3]).as_rotvec()[2] == pytest.approx(expected_yaw)
    for index in range(131, 171):
        imu(prior, index, 1_000_000_000+index*10_000_000, gyro=(0, 0, .0006))
    assert prior.failure is None
    assert prior.bias_report()['status'] == 'WHEEL_COVERAGE_MISSING'
    assert np.array_equal(prior.bias_estimator.bias_at(2_700_000_000), bias)


def test_undefined_background_gravity_direction_rejects_estimate_without_poisoning_journal():
    prior = continuous()
    # All packets satisfy the source contract, while this window is unusable
    # for a bias fit. It must not create NaN and fail the strict status writer.
    for index in range(200):
        stamp = 1_000_000_000+index*26_000_000
        prior.add_wheel(wheel(index, stamp))
        imu(prior, index, stamp, accel=(0, 0, 9.80665 if index%2 else -9.80665))
    report = prior.bias_report()
    assert report['latest_candidate']['screening_passed'] is False
    assert report['latest_candidate']['accel_direction_p95_deg'] is None
    json.dumps(report, allow_nan=False)
    assert prior.failure is None and report['bias_estimated'] is False
    assert np.array_equal(prior.pose_at(stamp), np.eye(4))
