"""Synthetic planar EKF/covariance/causal-history tests; no hardware accuracy claim."""
import copy
import json
import math

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from wc_runtime.mapping_planar import (PlanarEKF, validate_planar_config, validate_confirmed_bias,
                                       native_covariances, NATIVE_CONSTRAINT_VARIANCE)
from wc_runtime.mapping_prior import (MotionPrior, NotReady, DroppedCloud, PriorError, prepare_cloud,
                                      adapt_motion_odometry)
from wc_runtime.mapping_bias import CausalBiasEstimator
from test_mapping_prior import config, imu, wheel, cloud


def planar(**policy):
    cfg = config()
    cfg.update(continuous_mapping=True, motion_model='planar_ekf', policy=policy)
    return MotionPrior(cfg, 'synthetic', clock=lambda: 1000.)


def feed(p, first=0, last=100, *, raw=100, gyro=(0., 0., .1), origin=1_000_000_000):
    for i in range(first, last+1):
        stamp = origin+i*10_000_000
        p.add_wheel(wheel(i, stamp, -raw, raw))
        imu(p, i, stamp, gyro=gyro)


def test_exact_arc_jacobian_matches_finite_differences_including_zero_turn():
    for rate in (0., 1e-9, -.5, 1.2):
        x = np.array([.3, -.8, .4, .6, rate])
        out, F = PlanarEKF.transition(x, .37)
        numerical = np.zeros((5, 5))
        for j in range(5):
            delta = np.eye(5)[j]*1e-6
            numerical[:, j] = (PlanarEKF.transition(x+delta, .37)[0]-
                                PlanarEKF.transition(x-delta, .37)[0])/2e-6
        assert np.allclose(F, numerical, atol=2e-10)
        if rate == 0:
            assert np.allclose(out[:2]-x[:2], .6*.37*np.array([math.cos(.4), math.sin(.4)]))


def test_independent_measurements_joseph_covariance_and_no_reused_sample():
    # This test isolates Joseph arithmetic. Candidate-gate rejection is tested
    # separately; these intentionally contradictory measurements exceed 5σ.
    f = PlanarEKF({'innovation_gate_sigma':100.})
    p0 = f.P[4, 4]
    f.wheel(10, .4, .2)
    f.gyro(20, -.1)
    expected_variance = 1/(1/p0+1/.0025+1/.0004)
    expected_rate = expected_variance*(.2/.0025-.1/.0004)
    assert f.x[4] == pytest.approx(expected_rate)
    assert f.P[4, 4] == pytest.approx(expected_variance)
    before = f.x.copy(), f.P.copy()
    f.wheel(10, 999., 999.); f.gyro(20, 999.)
    assert np.array_equal(f.x, before[0]) and np.array_equal(f.P, before[1])
    assert f.counts == {'wheel': 1, 'gyro': 1}
    f.predict(.2)
    assert f.P[0, 3] != 0 and f.P[2, 4] != 0
    assert np.linalg.eigvalsh(f.P).min() >= -1e-14


def test_nonzero_wheel_yaw_is_an_actual_measurement_and_roll_pitch_cannot_tilt_pose():
    p = planar(history_s=5.)
    for i in range(101):
        t = 1_000_000_000+i*10_000_000
        p.add_wheel(wheel(i, t, 80, 80))  # Opposite physical wheel speeds: turn.
        imu(p, i, t, gyro=(.7, -.4, 0.))
    pose = p.pose_at(t)
    state = p._planar_state_at(t)
    assert abs(state.x[4]) > .005 and abs(state.x[2]) > .005
    assert np.array_equal(pose[2], [0., 0., 1., 0.])
    assert np.array_equal(pose[:2, 2], [0., 0.])
    report = p.sample_report(t, pose)
    assert report['planar_ekf']['measurement_updates'] == {'wheel': 100, 'gyro': 0}
    assert report['planar_ekf']['measurement_rejections']['gyro'] == 100
    assert report['planar_ekf']['last_update']['gyro']['reason'] == 'MAHALANOBIS_GATE_REJECTED'
    json.dumps(report, allow_nan=False)


def test_installed_imu_axes_project_to_axle_z_and_lidar_lever_arm_is_exact():
    cfg = config(); cfg.update(continuous_mapping=True, motion_model='planar_ekf', policy={'history_s': 5.})
    mount = Rotation.from_euler('xyz', [.3, -.2, .4]).as_matrix()
    T = np.eye(4); T[:3, :3] = mount; T[:3, 3] = mount @ np.array([-.4, .2, -.5])
    cfg['T_base_axle'] = T.tolist(); cfg['R_base_imu'] = mount.tolist()
    p = MotionPrior(cfg, 'synthetic')
    feed(p, raw=0, gyro=(0., 0., .2))
    assert np.allclose(p.R_reference_imu, np.eye(3))
    t = 2_000_000_000; state = p._planar_state_at(t); pose = p.pose_at(t)
    assert state.x[4] > 0
    lever = p.T_reference_axle[:3, 3]
    expected = state.x[:2]-(pose[:3, :3] @ lever)[:2]
    assert np.allclose(pose[:2, 3], expected)
    assert np.allclose(pose[:2, 3], lever[:2]-(pose[:3, :3] @ lever)[:2], atol=1e-14)
    _, linear, angular, pc, tc = state.output(lever)
    assert np.allclose(linear, -np.cross(angular, lever))
    assert np.linalg.eigvalsh(pc).min() >= -1e-14
    assert tc[0, 5] == pytest.approx(lever[1]*state.P[4, 4])


def test_native_covariances_are_invertible_preserving_dynamic_observable_blocks():
    f = PlanarEKF({}); f.wheel(0, .4, .1); f.gyro(0, .2); f.predict(.15)
    _, _, _, pc, tc = f.output([-.4, .2, -.5])
    original = f.P.copy()
    pose, twist = native_covariances(pc, tc)
    for array in (pose, twist):
        assert np.linalg.eigvalsh(array).min() > 0
        assert np.allclose(array @ np.linalg.inv(array), np.eye(6), atol=1e-10)
    assert np.array_equal(pose[np.ix_([0, 1, 5], [0, 1, 5])], pc[np.ix_([0, 1, 5], [0, 1, 5])])
    assert np.array_equal(twist[np.ix_([0, 5], [0, 5])], tc[np.ix_([0, 5], [0, 5])])
    assert pose[2, 2] == NATIVE_CONSTRAINT_VARIANCE
    assert twist[1, 1]-tc[1, 1] == pytest.approx(NATIVE_CONSTRAINT_VARIANCE)
    assert np.array_equal(f.P, original)


def test_authority_adapter_retains_fused_twist_pose_and_dynamic_covariance():
    from test_mapping_odometry import original, weights
    p = planar(); feed(p, last=20, gyro=(.2, -.3, .1))
    stamp = 1_200_000_000
    report = p.sample_report(stamp, p.pose_at(stamp))
    msg = original()
    msg.pose.covariance = report['pose_covariance']
    msg.twist.covariance = report['twist_covariance']
    for field, values in ((msg.twist.twist.linear, report['linear_velocity_reference_m_s']),
                          (msg.twist.twist.angular, report['angular_velocity_reference_rad_s'])):
        field.x, field.y, field.z = values
    cfg = {**p.config, 'wheel_imu_covariance': weights()}
    out = adapt_motion_odometry(msg, cfg)
    assert out.header.frame_id == 'mapping_odom'
    assert out.pose.pose == msg.pose.pose and out.twist.twist == msg.twist.twist
    assert out.pose.covariance == report['pose_covariance']
    assert out.twist.covariance == report['twist_covariance']
    assert out.twist.twist.angular.x == out.twist.twist.angular.y == 0
    assert 0 < out.twist.twist.angular.z < .1


def test_historical_queries_scan_checkpoints_and_cache_do_not_change_ekf():
    a, b = planar(history_s=5.), planar(history_s=5.)
    feed(a); feed(b)
    for t in [1_030_000_001, 1_750_100_000, 1_500_000_000, 1_900_000_099, 1_100_000_000]:
        pose = a.pose_at(t)
        assert np.allclose(pose, b.pose_at(t), atol=1e-15)
    for t in [1_150_100_000, 1_350_900_000, 1_800_000_011]:
        a.record_forwarded(t, a.pose_at(t))
    final_a, final_b = a._planar_state_at(2_000_000_000), b._planar_state_at(2_000_000_000)
    assert np.array_equal(final_a.x, final_b.x)
    assert np.array_equal(final_a.P, final_b.P)
    saved = a.pose_at(1_900_000_000)
    feed(a, 101, 110, raw=-250, gyro=(0, 0, -.8))
    assert np.array_equal(a.pose_at(1_900_000_000), saved)


def test_same_timestamp_distinct_imu_sequences_once_each_and_no_endpoint_reinterpretation():
    p = planar(history_s=5.)
    p.add_wheel(wheel(0, 1_000_000_000)); imu(p, 0, 1_000_000_000)
    p.add_wheel(wheel(1, 1_020_000_000)); imu(p, 1, 1_020_000_000, gyro=(0, 0, .1))
    before = p.pose_at(1_020_000_000)
    imu(p, 2, 1_020_000_000, gyro=(0, 0, .2))
    assert np.array_equal(p.pose_at(1_020_000_000), before)
    p.add_wheel(wheel(2, 1_040_000_000)); imu(p, 3, 1_040_000_000)
    state = p._planar_state_at(1_040_000_000)
    assert state.counts['wheel'] == 2
    assert state.counts['gyro']+state.rejected_counts['gyro'] == 3
    assert state.last_sequence['imu'] == 2


def test_different_callback_arrival_order_preserves_source_order_fusion():
    a, b = planar(history_s=5.), planar(history_s=5.)
    feed(a, last=0); feed(b, last=0)
    for i in range(1, 61):
        stamp = 1_000_000_000+i*10_000_000
        a.add_wheel(wheel(i, stamp, -100, 90))
        imu(a, i, stamp, gyro=(0., 0., .05))
        imu(b, i, stamp, gyro=(0., 0., .05))
        # A query behind the slow stream must not commit a partial update.
        with pytest.raises(NotReady): b.pose_at(stamp)
        b.add_wheel(wheel(i, stamp, -100, 90))
    sa, sb = a._planar_state_at(stamp), b._planar_state_at(stamp)
    assert np.array_equal(sa.x, sb.x) and np.array_equal(sa.P, sb.P)


def test_confirmed_bias_version_preserves_history_and_only_changes_new_measurements():
    p = planar(history_s=5.)
    p.bias_estimator = CausalBiasEstimator({'window_s': .2, 'min_samples': 21})
    feed(p, last=60, raw=0, gyro=(0., 0., .0006))
    historical = p.pose_at(1_550_000_000)
    p.record_forwarded(1_550_000_000, historical)
    frontier = p._planar_state_at(1_600_000_000)
    version = p.confirm_bias_stationarity(1_000_000_000, 1_500_000_000, 1_605_000_000,
        evidence_id='synthetic-independent', source='synthetic_stationarity', independent=True)
    assert version.effective_stamp_ns == 1_605_000_000
    assert np.array_equal(p.pose_at(1_550_000_000), historical)
    assert np.array_equal(p._planar_state_at(1_600_000_000).x, frontier.x)
    feed(p, first=61, last=62, raw=0, gyro=(0., 0., .0006))
    after = p._planar_state_at(1_620_000_000)
    assert after.counts == {'wheel': 62, 'gyro': 62}
    assert after.x[4] < frontier.x[4]
    assert np.array_equal(p.pose_at(1_550_000_000), historical)


def test_gap_has_no_fake_displacement_then_real_observations_resume():
    p = planar(history_s=8.)
    feed(p, last=20, gyro=(0, 0, .1))
    before = p.pose_at(1_200_000_000)
    feed(p, 21, 25, origin=4_000_000_000)
    assert np.array_equal(p.pose_at(4_210_000_000), before)
    with pytest.raises(DroppedCloud):
        p.require_observed_cloud_time(3_000_000_000)
    assert not np.allclose(p.pose_at(4_250_000_000), before)
    assert p.failure is None


def test_bounded_history_while_clouds_or_wheel_pause_and_query_order_independence():
    short, full = planar(history_s=.3), planar(history_s=10.)
    for i in range(201):
        feed(short, i, i); feed(full, i, i)
        if i > 10 and i%3 == 0:
            short.pose_at(1_000_000_000+i*10_000_000-3_333_333)
    t = 3_000_000_000
    assert np.allclose(short.pose_at(t), full.pose_at(t), atol=1e-13)
    assert np.allclose(short._planar_state_at(t).P, full._planar_state_at(t).P, atol=1e-13)
    frozen = short.pose_at(t)
    for i in range(201, 301):
        imu(short, i, 1_000_000_000+i*10_000_000, gyro=(0, 0, .1))
    assert len(short.imu) < 40 and len(short.wheel) <= 2
    assert len(short.planar_cache) < 40
    p0 = short.planar_anchor[1].output(short.T_reference_axle[:3, 3])[0]
    assert np.array_equal(p0, frozen)
    with pytest.raises(NotReady):
        short.pose_at(4_000_000_000)
    assert short.failure is None


def confirmed(value=(0., 0., .001)):
    return {'status': 'INDEPENDENTLY_CONFIRMED', 'sensor_id': 'H30-0000000015',
            'bias_native_rad_s': list(value), 'evidence_id': 'synthetic-operator-confirmed-fixture',
            'evidence_sha256': 'a'*64, 'source': 'operator_visual_confirmation',
            'evidence_note': 'Synthetic input, not a real calibration.'}


def test_explicit_confirmed_native_bias_is_usable_at_moving_start_without_static_wait():
    cfg = config(); cfg.update(continuous_mapping=True, motion_model='planar_ekf', confirmed_gyro_bias=confirmed())
    cfg['confirmed_gyro_bias']['sensor_id'] = cfg['imu_sensor_id']
    p = MotionPrior(cfg, 'synthetic'); p.enable_startup_recovery(0.)
    feed(p, raw=100, gyro=(0., 0., .001))
    pose = p.pose_at(2_000_000_000)
    assert pose[0, 3] > .1 and pose[1, 3] == 0
    assert np.array_equal(pose[:3, :3], np.eye(3))
    assert p.startup_deadline is None and p.initialization['stamp_ns'] == 1_000_000_000
    assert p.bias_report()['accepted_version']['bias_native_rad_s'] == (0., 0., .001)
    assert p.bias_report()['accepted_version']['version'] == 1
    assert p.bias_report()['accepted_version']['effective_stamp_ns'] == 0
    assert p.bias_report()['external_evidence_verified_by_estimator'] is False
    assert p.initialization['gyro_bias_candidate_rad_s'] == [0., 0., .001]
    assert p.initialization['gyro_bias_status'] == 'EXPLICIT_EXTERNAL_CALIBRATION_DECLARATION'
    report = p.sample_report(2_000_000_000, pose)
    assert report['gyro_bias_applied_native_rad_s'] == [0., 0., .001]
    assert report['angular_velocity_reference_rad_s'] == [0., 0., 0.]
    assert report['linear_velocity_reference_m_s'][0] == p._planar_state_at(2_000_000_000).x[3]
    assert np.linalg.eigvalsh(np.reshape(report['pose_covariance'], (6, 6))).min() > 0
    assert np.linalg.eigvalsh(np.reshape(report['twist_covariance'], (6, 6))).min() > 0
    json.dumps(p.bias_report(), allow_nan=False)


@pytest.mark.parametrize('value', [0, {'wheel_v_variance': 0}, {'gyro_w_variance': float('nan')},
                                 {'initial_pose_variance': [1., 1.]}, {'unknown': 3}])
def test_planar_config_rejects_invalid_noise(value):
    with pytest.raises(ValueError): validate_planar_config(value)


def test_bias_config_never_accepts_wheel_zero_as_confirmed_stationarity():
    c = confirmed(); c['source'] = 'wheel_zero'
    with pytest.raises(ValueError): validate_confirmed_bias(c)
    c = confirmed(); c['bias_native_rad_s'] = [0, 0, float('nan')]
    with pytest.raises(ValueError): validate_confirmed_bias(c)
    assert validate_confirmed_bias(None) is None
