"""Meaningful causal/history and observability checks; no ROS or hardware."""
from dataclasses import FrozenInstanceError

import numpy as np
import pytest

from wc_runtime.mapping_bias import BiasPolicy, CausalBiasEstimator


def estimator(**values):
    defaults = dict(window_s=.2, min_samples=21, max_imu_gap_s=.02,
                    min_confirmed_windows=2, robust_windows=3)
    defaults.update(values)
    return CausalBiasEstimator(defaults)


def window(obj, start, bias=(.0006, -.0001, .0004), **values):
    result = None
    for i in range(21):
        stamp = start+i*10_000_000
        noise = np.sin(i*np.pi/5)*.00005
        result = obj.observe(stamp, np.asarray(bias)+[noise, -noise, 0],
                             values.get('accel', [0, 0, 9.80665]),
                             stamp-values.get('wheel_age_ns', 0), values.get('wheel', [0, 0]))
    return result


def confirm(obj, first, last, identifier, **values):
    return obj.confirm_stationary(first, last, values.pop('observed', last),
                                  evidence_id=identifier, source=values.pop('source', 'synthetic_independent_lidar'),
                                  independent=values.pop('independent', True), **values)


def test_immediate_moving_start_has_no_wait_or_estimation():
    obj = estimator()
    assert np.array_equal(obj.bias_at(0), np.zeros(3))
    for i in range(200):
        obj.observe(i*10_000_000, [0, 0, .2], [2., 0, 9.80665], i*10_000_000, [.3, .3])
    assert obj.candidate_count == 0 and not obj.status()['startup_blocked']
    assert obj.status()['bias_estimated'] is False


def test_slow_real_rotation_can_pass_screening_but_never_becomes_bias_without_independent_evidence():
    obj = estimator()
    for i in range(5):
        assert window(obj, i*210_000_000, bias=(0, 0, .001))['screening_passed']
    assert obj.candidate_count == 5
    assert len(obj.versions) == 1 and np.array_equal(obj.bias_at(1_000_000_000), [0, 0, 0])
    assert confirm(obj, 0, obj.last_stamp_ns, 'wheel-derived', source='wheel_imu_prior') is None
    assert confirm(obj, 0, obj.last_stamp_ns, 'not-independent', independent=False) is None
    assert len(obj.versions) == 1


def test_partial_evidence_does_not_cover_complete_window():
    obj = estimator(min_confirmed_windows=1)
    window(obj, 0)
    assert confirm(obj, 10_000_000, 200_000_000, 'partial') is None
    assert len(obj.versions) == 1


def test_confirmed_update_is_clipped_and_old_history_does_not_change():
    obj = estimator()
    window(obj, 0)
    assert confirm(obj, 0, 200_000_000, 'first') is None
    saved = obj.bias_at(200_000_000)
    window(obj, 210_000_000)
    version = confirm(obj, 210_000_000, 410_000_000, 'second')
    assert version.effective_stamp_ns == 410_000_000
    assert np.allclose(np.linalg.norm(version.bias_native_rad_s), obj.policy.max_update_norm_rad_s)
    assert np.array_equal(obj.bias_at(200_000_000), saved)
    assert np.array_equal(obj.bias_at(409_999_999), [0, 0, 0])
    assert np.allclose(obj.bias_at(410_000_000), version.bias_native_rad_s)
    assert obj.change_stamps(0, 500_000_000) == [410_000_000]
    assert obj.change_stamps(410_000_000, 500_000_000) == []
    with pytest.raises(FrozenInstanceError):
        version.effective_stamp_ns = 0
    query = obj.bias_at(410_000_000); query[0] = 100
    assert obj.bias_at(410_000_000)[0] < 1


def test_delayed_external_confirmation_applies_at_receipt_without_backdating():
    obj = estimator(min_confirmed_windows=1)
    window(obj, 0)
    obj.observe(210_000_000, [0, 0, .2], [0, 0, 9.80665], 210_000_000, [.2, .2])
    version = confirm(obj, 0, 200_000_000, 'delayed', observed=210_000_000)
    assert version.effective_stamp_ns == 210_000_000
    assert np.array_equal(obj.bias_at(209_999_999), [0, 0, 0])


def test_reusing_consumed_window_with_new_evidence_cannot_ratchet_bias():
    obj = estimator(min_confirmed_windows=1)
    window(obj, 0)
    version = confirm(obj, 0, 200_000_000, 'once')
    assert version
    assert confirm(obj, 0, 200_000_000, 'once') is None
    assert confirm(obj, 0, 200_000_000, 'new-id-same-window') is None
    assert len(obj.versions) == 2


def test_gap_and_stale_wheel_reset_window_and_never_estimate_across_silence():
    obj = estimator()
    for i in range(15):
        obj.observe(i*10_000_000, [0, 0, .0005], [0, 0, 9.80665], i*10_000_000, [0, 0])
    assert obj.observe(10_000_000_000, [0, 0, .0005], [0, 0, 9.80665], 0, [0, 0]) is None
    assert not obj.rows and obj.gap_resets == 1
    candidate = window(obj, 10_010_000_000)
    assert candidate['start_stamp_ns'] == 10_010_000_000
    assert len(obj.versions) == 1


def test_accel_direction_change_rejects_window_even_with_gravity_norm_and_zero_wheels():
    obj = estimator()
    result = None
    for i in range(21):
        angle = (-1 if i % 2 else 1)*.02
        accel = [9.80665*np.sin(angle), 0, 9.80665*np.cos(angle)]
        result = obj.observe(i*10_000_000, [0, 0, .0005], accel, i*10_000_000, [0, 0])
    assert result['screening_passed'] is False
    assert obj.candidate_count == 0


@pytest.mark.parametrize('values', [dict(wheel=[.01, 0]), dict(accel=[0, 0, 10.2])])
def test_motion_and_linear_acceleration_prevent_candidate(values):
    obj = estimator()
    assert window(obj, 0, **values) is None
    assert not obj.candidates


def test_robust_multiple_windows_resist_one_mean_outlier():
    obj = estimator(min_confirmed_windows=3, max_update_norm_rad_s=.0015)
    for i, bias in enumerate(((.0005, 0, 0), (.0014, 0, 0), (.0006, 0, 0))):
        window(obj, i*210_000_000, bias=bias)
        version = confirm(obj, i*210_000_000, i*210_000_000+200_000_000, 'independent-'+str(i))
    assert np.allclose(version.bias_native_rad_s, [.0006, 0, 0], atol=1e-12)


def test_capacity_stops_updates_without_discarding_required_history_then_anchor_frees_space():
    obj = estimator(min_confirmed_windows=1, max_versions=2)
    window(obj, 0)
    confirm(obj, 0, 200_000_000, 'first')
    window(obj, 210_000_000)
    assert confirm(obj, 210_000_000, 410_000_000, 'full') is None
    assert obj.last_reason == 'HISTORY_CAPACITY_NO_UPDATE_UNTIL_ANCHORED'
    saved = obj.bias_at(410_000_000)
    obj.prune_before(410_000_000)
    assert len(obj.versions) == 1
    assert np.array_equal(saved, obj.bias_at(410_000_000))
    with pytest.raises(ValueError, match='predates'):
        obj.bias_at(0)
    window(obj, 420_000_000)
    assert confirm(obj, 420_000_000, 620_000_000, 'after-prune')


def test_source_rows_and_unconfirmed_candidates_are_bounded():
    obj = estimator(max_pending_windows=3)
    for i in range(12):
        window(obj, i*210_000_000)
    assert len(obj.candidates) == 3 and not obj.rows
    assert len(obj.versions) == 1


def test_bad_time_and_nonfinite_data_are_rejected_without_creating_versions():
    obj = estimator()
    window(obj, 0)
    with pytest.raises(ValueError, match='source time'):
        obj.observe(199_000_000, [0, 0, 0], [0, 0, 9.8], 199_000_000, [0, 0])
    with pytest.raises(ValueError, match='future wheel'):
        obj.observe(210_000_000, [0, 0, 0], [0, 0, 9.8], 220_000_000, [0, 0])
    with pytest.raises(ValueError, match='finite'):
        obj.observe(210_000_000, [float('nan'), 0, 0], [0, 0, 9.8], 210_000_000, [0, 0])
    with pytest.raises(ValueError, match='retroactively'):
        confirm(obj, 0, 200_000_000, 'time-reverse', observed=199_000_000)
    assert len(obj.versions) == 1


def test_distinct_validated_packets_sharing_host_timestamp_are_all_observed():
    obj = estimator(min_samples=42)
    for i in range(21):
        timestamp = i*10_000_000
        obj.observe(timestamp, [.0001, 0, 0], [0, 0, 9.80665], timestamp, [0, 0])
        result = obj.observe(timestamp, [.0009, 0, 0], [0, 0, 9.80665], timestamp, [0, 0])
    assert result['sample_count'] == 42
    assert np.allclose(result['bias_native_rad_s'], [.0005, 0, 0])
    assert result['end_stamp_ns']-result['start_stamp_ns'] == 200_000_000


@pytest.mark.parametrize('values', [dict(window_s=float('inf')), dict(min_samples=True),
                                    dict(min_samples=2), dict(robust_windows=1)])
def test_invalid_policy(values):
    with pytest.raises(ValueError):
        BiasPolicy(**values)


def test_ab_integration_splits_at_bias_boundary_and_never_integrates_missing_interval():
    import importlib.util
    from pathlib import Path
    from scipy.spatial.transform import Rotation
    path = Path(__file__).parents[1]/'integration/check_mapping_bias_ab_20260915.py'
    spec = importlib.util.spec_from_file_location('bias_ab_check', path)
    ab = importlib.util.module_from_spec(spec); spec.loader.exec_module(ab)
    rows = {'imu': [], 'wheel': []}
    for i in range(3):
        rows['imu'].append({'stamp': i*1_000_000_000, 'gyro': np.array([0, 0, .001]), 'gap_before': i == 2})
        rows['wheel'].append({'stamp': i*1_000_000_000, 'speeds': np.zeros(2), 'v': 0., 'gap_before': i == 2})
    initial = {'stamp_ns': 0, 'R_reference_imu': np.eye(3), 'T_reference_axle': np.eye(4)}
    profiles = {'zero': [], 'causal': [(500_000_000, np.array([0, 0, .001]))]}
    times, poses, observed, _, segments = ab.reintegrate(rows, {'max_imu_age_s': 2, 'max_wheel_age_s': 2}, initial, profiles, [])
    assert np.array_equal(times, [0, 500_000_000, 1_000_000_000, 2_000_000_000])
    assert np.array_equal(observed, [True, True, False])
    assert segments == [{'start_stamp_ns': 0, 'end_stamp_ns': 1_000_000_000, 'duration_s': 1.0}]
    assert np.isclose(Rotation.from_matrix(poses['zero'][-1, :3, :3]).magnitude(), .001)
    assert np.isclose(Rotation.from_matrix(poses['causal'][-1, :3, :3]).magnitude(), .0005)
    assert np.array_equal(poses['causal'][-1], poses['causal'][-2])
