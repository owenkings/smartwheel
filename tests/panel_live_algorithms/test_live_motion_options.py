"""Live options are exercised with synthetic measurements, never device access."""
import copy
import json
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'operations'))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'runtime'))
from test_mapping_prior import config, imu, wheel, cloud, authority_weights
from test_offline_motion_adapter import candidate
from test_offline_planar_registration import scene, xyz
from wc_runtime.mapping_prior import MotionPrior, PriorError, prepare_cloud
from wc_runtime.live_motion_options import validate_live_options, LiveGeometry
from wc_runtime.offline_motion_adapter import OfflineCorrectedPrior
from wc_runtime.offline_planar_process import ContinuousPlanarEKF
from wc_runtime.offline_planar_registration import se2


def settings(**changes):
    value = config()
    value.update(continuous_mapping=True, motion_model='planar_ekf', estimator='five_state',
                 wheel_device_id=wheel(0, 1)['device_id'], policy={'history_s': 5.},
                 odometry_source='wheel_imu', wheel_imu_covariance=authority_weights())
    value.update(changes)
    return value


def declaration(value, *, bias=(.0003, -.0002, .0008), scale=.91, coefficient=-.014):
    return {'schema_version': 1, 'scope': 'device', 'status': 'EXPLICIT_DEVICE_CALIBRATION_DECLARATION',
            'imu_sensor_id': value['imu_sensor_id'], 'wheel_device_id': value['wheel_device_id'],
            'R_reference_imu': np.eye(3).tolist(), 'wheel_yaw_scale': scale,
            'wheel_yaw_speed_coefficient': coefficient, 'evidence_id': 'synthetic-wheel-reference',
            'evidence_sha256': 'a'*64,
            'confirmed_gyro_bias': {'status': 'INDEPENDENTLY_CONFIRMED', 'sensor_id': value['imu_sensor_id'],
                'bias_native_rad_s': list(bias), 'evidence_id': 'synthetic-external-static',
                'evidence_sha256': 'b'*64, 'source': 'operator_visual_confirmation'}}


def feed(prior, bias=(.0003, -.0002, .0008), queries=False):
    for i in range(61):
        stamp = 1_000_000_000 + i*10_000_000
        prior.add_wheel(wheel(i, stamp, -100, 110))
        imu(prior, i, stamp, gyro=np.array([0., 0., .025]) + bias)
        if queries and i > 2:
            prior.pose_at(stamp-5_000_000)
    return stamp


@pytest.mark.parametrize('option,value', [('geometry', True), ('process_noise', 'white_acceleration')])
def test_official_rejects_unimplemented_combinations(option, value):
    with pytest.raises(ValueError):
        validate_live_options(settings(estimator='robot_localization', **{option: value}))


def test_motion_on_requires_explicit_device_calibration_and_no_offline_conversion():
    c = settings(motion_correction=True)
    with pytest.raises(ValueError, match='设备校正'):
        validate_live_options(c)
    c['live_motion_calibration'] = candidate(c)
    with pytest.raises(ValueError):
        validate_live_options(c)


def test_device_mismatch_and_rotation_mismatch_fail_without_guessing():
    c = settings(motion_correction=True)
    c['live_motion_calibration'] = declaration(c)
    c['live_motion_calibration']['imu_sensor_id'] = 'H30-other-device'
    with pytest.raises(ValueError, match='身份'):
        validate_live_options(c)
    c['live_motion_calibration'] = declaration(c)
    c['live_motion_calibration']['R_reference_imu'] = [[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]]
    p = MotionPrior(c, 'synthetic')
    with pytest.raises(PriorError, match='rotation differs'):
        feed(p)


def test_live_and_offline_correction_math_match_for_same_parameters():
    c = settings(motion_correction=True)
    c['live_motion_calibration'] = declaration(c)
    live = MotionPrior(c, 'synthetic')
    offline_config = settings(offline_experiment=True)
    offline = OfflineCorrectedPrior(offline_config, 'synthetic',
        candidate=candidate(offline_config, bias=(.0003, -.0002, .0008), a=.91, b=-.014),
        candidate_sha256='c'*64, clock=lambda: 1000.)
    stamp = feed(live, queries=True)
    feed(offline)
    actual, reference = live._planar_state_at(stamp), offline._planar_state_at(stamp)
    np.testing.assert_allclose(actual.x, reference.x, atol=1e-13, rtol=0)
    np.testing.assert_allclose(actual.P, reference.P, atol=1e-13, rtol=0)
    assert actual.last_update['wheel']['measurement_covariance'][0][1] != 0
    assert live.bias_report()['automatic_confirmation_enabled'] is False
    np.testing.assert_allclose(actual.last_update['gyro']['measurement'], [.025], atol=1e-14)


def test_duplicate_same_bias_is_once_but_conflicting_bias_is_rejected():
    c = settings(motion_correction=True)
    c['live_motion_calibration'] = declaration(c)
    c['confirmed_gyro_bias'] = copy.deepcopy(c['live_motion_calibration']['confirmed_gyro_bias'])
    p = MotionPrior(c, 'synthetic')
    stamp = feed(p)
    np.testing.assert_allclose(p._planar_state_at(stamp).last_update['gyro']['measurement'], [.025], atol=1e-14)
    c['confirmed_gyro_bias']['bias_native_rad_s'][2] += .001
    with pytest.raises(ValueError, match='double correction'):
        validate_live_options(c)


def test_psd_switch_changes_actual_Q_without_reinterpreting_variance():
    c = settings(process_noise='white_acceleration')
    c['planar_ekf'] = {'linear_acceleration_variance': 99., 'angular_acceleration_variance': 88.}
    a, b = MotionPrior(c, 'synthetic'), MotionPrior(settings(process_noise='white_acceleration'), 'synthetic')
    stamp = feed(a, queries=True)
    feed(b)
    assert isinstance(a.planar_anchor[1], ContinuousPlanarEKF)
    np.testing.assert_allclose(a._planar_state_at(stamp).P, b._planar_state_at(stamp).P, atol=1e-13, rtol=0)
    legacy = MotionPrior(settings(), 'synthetic')
    feed(legacy)
    assert not np.allclose(legacy._planar_state_at(stamp).P, a._planar_state_at(stamp).P)
    assert a.config['continuous_process_noise']['linear_acceleration_psd_m2_s3'] == .25


def test_geometry_modifies_actual_prepared_pose_and_preserves_point_fields():
    p = MotionPrior(settings(geometry=True), 'synthetic')
    feed(p)
    points = xyz(scene())
    first = cloud(points, stamp=1_100_000_000)
    first_out, first_pose, first_report = prepare_cloud(first, p)
    p.record_forwarded(1_100_000_000, first_pose)
    second = cloud(points, stamp=1_400_000_000)
    output, pose, report = prepare_cloud(second, p)
    registration = report['live_geometry']['registration']
    assert registration['accepted'], registration['rejection_reasons']
    assert not np.allclose(pose, p.pose_at(1_400_000_000))
    assert np.allclose(pose, np.array(report['T_prior_reference']))
    assert np.allclose(p.pose_at(1_400_000_000), report['T_uncorrected_prior_reference'])
    assert report['live_geometry']['ekf_measurement_feedback'] is False
    assert report['live_geometry']['published_pose_corrected'] is True
    assert bytes(output.data)[-7:] == b'padding'
    assert output.width == second.width and output.fields == second.fields
    assert report['covariance_status'].startswith('UNKNOWN_GEOMETRY')
    json.dumps(report, allow_nan=False)


def test_geometry_degenerate_scene_keeps_prior_increment_and_memory_is_bounded():
    stage = LiveGeometry()
    x = np.linspace(.4, 5, 1000)
    points = xyz(np.vstack([np.column_stack([x, x*0+1]), np.column_stack([x, x*0-1])]))
    report = {'pose_covariance': np.eye(6).reshape(-1).tolist(), 'twist_covariance': np.eye(6).reshape(-1).tolist()}
    stage.correct(1_000_000_000, points, np.zeros(len(points), dtype=int), np.eye(4), report)
    prior = se2(.1, 0, 0)
    _, pose, observed = stage.correct(1_300_000_000, points, np.zeros(len(points), dtype=int), prior, report)
    assert not observed['live_geometry']['registration']['accepted']
    np.testing.assert_allclose(pose, prior, atol=1e-12)
    assert stage.corrections.maxlen == 128
    assert stage.engine.history.maxlen == stage.engine.policy.submap_keyframes
