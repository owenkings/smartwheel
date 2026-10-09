"""Gravity leveling geometry and rejection tests; no hardware or timing claims."""

import json

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from wc_calibration.core import CalibrationError
from wc_calibration.gravity_level import GravityLevelPolicy, estimate_gravity_level


def static_samples(up=(0., 0., 1.), count=200):
    return np.tile(np.asarray(up) * 9.80665, (count, 1))


@pytest.mark.parametrize("roll,pitch", [(0., 0.), (.3, -.2), (-.4, .35), (2.8, .2)])
def test_levels_known_gravity_and_preserves_horizontal_forward(roll, pitch):
    expected = Rotation.from_euler("xyz", [roll, pitch, 0.]).as_matrix()
    up_left = expected.T @ [0., 0., 1.]
    result = estimate_gravity_level(static_samples(up_left), np.eye(3), angular_velocity=np.zeros((200, 3)))
    actual = np.asarray(result["R_level_left"])
    assert np.allclose(actual, expected, atol=1e-12)
    assert np.allclose(actual @ up_left, [0., 0., 1.], atol=1e-12)
    forward = actual @ [1., 0., 0.]
    assert forward[0] > 0 and abs(forward[1]) < 1e-12
    assert np.allclose(actual.T @ actual, np.eye(3), atol=1e-12)
    assert np.linalg.det(actual) == pytest.approx(1.)
    assert result["status"] == "CANDIDATE" and result["live_eligible"] is False
    assert result["mounting_rotation_validated"] is False
    assert result["time_model_validated"] is False
    json.dumps(result, allow_nan=False)


def test_mount_rotation_is_applied_in_imu_to_left_direction_without_mutating_inputs():
    expected = Rotation.from_euler("xyz", [-.2, .4, 0.]).as_matrix()
    mount = Rotation.from_euler("xyz", [.1, -.3, .2]).as_matrix()
    up_left = expected.T @ [0., 0., 1.]
    acceleration = static_samples(mount.T @ up_left)
    before, mount_before = acceleration.copy(), mount.copy()
    result = estimate_gravity_level(acceleration, mount)
    actual = np.asarray(result["R_level_left"])
    assert np.allclose(actual, expected, atol=1e-12)
    assert np.allclose(result["up_unit_in_left"], up_left, atol=1e-12)
    assert np.array_equal(acceleration, before) and np.array_equal(mount, mount_before)
    assert result["statistics"]["max_gyro_rad_s"] is None
    assert result["statistics"]["angular_velocity_available"] is False


def test_rotation_levels_ground_without_changing_distances_or_origin():
    true_rotation = Rotation.from_euler("xyz", [.22, -.31, 0.]).as_matrix()
    up = true_rotation.T @ [0., 0., 1.]
    result = estimate_gravity_level(static_samples(up), np.eye(3))
    rotation = np.asarray(result["R_level_left"])
    ground = np.array([[1., -.8, -.73], [2., .5, -.73], [3., 1.2, -.73], [.4, .8, -.73]])
    raw = ground @ true_rotation
    leveled = raw @ rotation.T
    assert np.allclose(leveled, ground, atol=1e-12)
    assert np.allclose(rotation @ np.zeros(3), np.zeros(3))
    assert np.allclose(np.linalg.norm(raw - raw[0], axis=1), np.linalg.norm(leveled - leveled[0], axis=1))
    assert result["origin"] == "left_lidar_origin_unchanged"
    assert "T_left_imu" not in result and "T_left_right" not in result


@pytest.mark.parametrize("bad", [[], [[0., 0., 9.8]], np.ones((100, 2)), static_samples(count=99),
                                  np.zeros((100, 3)), np.full((100, 3), 1e-12),
                                  np.full((100, 3), np.nan), np.full((100, 3), np.inf),
                                  np.full((100, 3), 1e308), np.ones((100, 3), dtype=bool),
                                  [["0", "0", "9.80665"]] * 100])
def test_invalid_acceleration_cannot_produce_transform(bad):
    with pytest.raises(CalibrationError):
        estimate_gravity_level(bad, np.eye(3))


@pytest.mark.parametrize("bad", [np.eye(4), np.diag([1., -1., 1.]), np.eye(3) * 2,
                                  [[1., .1, 0.], [0., 1., 0.], [0., 0., 1.]],
                                  np.full((3, 3), np.nan), np.ones((3, 3), dtype=bool)])
def test_invalid_mounting_transform_cannot_be_silently_repaired(bad):
    with pytest.raises(CalibrationError, match="R_left_imu"):
        estimate_gravity_level(static_samples(), bad)


def test_motion_checks_are_independent_and_policy_is_persisted():
    with pytest.raises(CalibrationError, match="ACCELERATION_NOT_NEAR_GRAVITY"):
        estimate_gravity_level(static_samples() * 1.2, np.eye(3))
    vibration = static_samples()
    vibration[:, 2] += np.tile([.4, -.4], 100)
    with pytest.raises(CalibrationError, match="ACCELERATION_DISPERSION_TOO_HIGH"):
        estimate_gravity_level(vibration, np.eye(3))
    tilt_samples = np.tile([[0., np.sin(.08), np.cos(.08)], [0., -np.sin(.08), np.cos(.08)]], (100, 1)) * 9.80665
    with pytest.raises(CalibrationError, match="GRAVITY_DIRECTION_UNSTABLE"):
        estimate_gravity_level(tilt_samples, np.eye(3), policy={"max_accel_rms_m_s2": 2.})
    rates = np.zeros((200, 3))
    rates[50, 0] = .06
    with pytest.raises(CalibrationError, match="ANGULAR_MOTION_OR_GYRO_BIAS_TOO_HIGH"):
        estimate_gravity_level(static_samples(), np.eye(3), angular_velocity=rates)
    policy = GravityLevelPolicy(max_gyro_rad_s=.04)
    result = estimate_gravity_level(static_samples(), np.eye(3), angular_velocity=np.zeros((200, 3)), policy=policy)
    assert result["policy"]["max_gyro_rad_s"] == .04
    assert result["statistics"]["max_gyro_rad_s"] == 0.


@pytest.mark.parametrize("gyro", [np.zeros((100, 3)), np.zeros((200, 2)), np.full((200, 3), np.nan)])
def test_corresponding_gyro_must_be_complete_and_finite(gyro):
    with pytest.raises(CalibrationError):
        estimate_gravity_level(static_samples(), np.eye(3), angular_velocity=gyro)


def test_vertical_forward_cannot_manufacture_heading():
    with pytest.raises(CalibrationError, match="FORWARD_HORIZONTAL_PROJECTION_UNOBSERVABLE"):
        estimate_gravity_level(static_samples([1., 0., 0.]), np.eye(3))


@pytest.mark.parametrize("policy", [{"min_samples": True}, {"min_samples": 2}, {"max_samples": 99},
                                     {"max_gyro_rad_s": float("nan")}, {"gravity_m_s2": 0},
                                     {"max_accel_rms_m_s2": -1}, {"max_direction_p95_deg": 181},
                                     {"min_forward_projection_norm": 1}, {"unknown": 1}, "default"])
def test_invalid_policy_is_rejected(policy):
    with pytest.raises(CalibrationError):
        estimate_gravity_level(static_samples(), np.eye(3), policy=policy)


def test_statistics_describe_noise_and_cannot_claim_constant_acceleration_is_observed():
    rng = np.random.default_rng(17)
    acceleration = static_samples() + rng.normal(0., .005, (200, 3))
    result = estimate_gravity_level(acceleration, np.eye(3))
    assert 0 < result["statistics"]["acceleration_rms_about_mean_m_s2"] < .02
    assert 0 < result["statistics"]["direction_p95_deg"] < .2
    assert any("constant linear acceleration" in value for value in result["limits"])
    assert any("duration" in value for value in result["limits"])
