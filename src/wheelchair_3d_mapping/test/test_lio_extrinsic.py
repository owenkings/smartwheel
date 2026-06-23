"""Unit tests for the LiDAR<-IMU extrinsic math used to configure FAST-LIO.

FAST-LIO extrinsic_T/R = LiDAR pose in the IMU body frame:
    T_il = inv(T_base_imu) @ T_base_lidar
These tests synthesize known base->imu and base->lidar transforms and assert the
derived T_il translation/rotation are correct. Pure numpy, no ROS needed.
"""
import numpy as np


def rpy_to_R(roll, pitch, yaw):
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return Rz @ Ry @ Rx


def T_from_xyz_rpy(xyz, rpy):
    T = np.eye(4)
    T[:3, :3] = rpy_to_R(*rpy)
    T[:3, 3] = xyz
    return T


def lidar_in_imu(T_base_imu, T_base_lidar):
    return np.linalg.inv(T_base_imu) @ T_base_lidar


def test_pure_translation_extrinsic():
    # imu at 0.45 m up, lidar at (0.45,0.24,0.65), both axis-aligned.
    T_bi = T_from_xyz_rpy([0.0, 0.0, 0.45], [0.0, 0.0, 0.0])
    T_bl = T_from_xyz_rpy([0.45, 0.24, 0.65], [0.0, 0.0, 0.0])
    T_il = lidar_in_imu(T_bi, T_bl)
    np.testing.assert_allclose(T_il[:3, 3], [0.45, 0.24, 0.20], atol=1e-9)
    np.testing.assert_allclose(T_il[:3, :3], np.eye(3), atol=1e-9)


def test_pitch_extrinsic_matches_known_values():
    # Reproduces the project's documented extrinsic: lidar pitched -0.0873 rad.
    T_bi = T_from_xyz_rpy([0.0, 0.0, 0.45], [0.0, 0.0, 0.0])
    T_bl = T_from_xyz_rpy([0.45, 0.24, 0.65], [0.0, -0.0873, 0.0])
    T_il = lidar_in_imu(T_bi, T_bl)
    np.testing.assert_allclose(T_il[:3, 3], [0.45, 0.24, 0.20], atol=1e-6)
    # R should be a pure pitch (rotation about Y) of -0.0873 rad.
    expected_R = rpy_to_R(0.0, -0.0873, 0.0)
    np.testing.assert_allclose(T_il[:3, :3], expected_R, atol=1e-6)
    # spot-check the well-known cos/sin entries
    assert abs(T_il[0, 0] - np.cos(-0.0873)) < 1e-6
    assert abs(T_il[0, 2] - np.sin(-0.0873)) < 1e-6


def test_imu_rotated_is_inverted_correctly():
    # If the IMU itself is yawed, T_il must account for inv(R_imu).
    T_bi = T_from_xyz_rpy([0.0, 0.0, 0.45], [0.0, 0.0, np.pi / 2])
    T_bl = T_from_xyz_rpy([1.0, 0.0, 0.45], [0.0, 0.0, 0.0])
    T_il = lidar_in_imu(T_bi, T_bl)
    # base->lidar offset (1,0,0) expressed in a +90deg-yaw IMU frame becomes (0,-1,0).
    np.testing.assert_allclose(T_il[:3, 3], [0.0, -1.0, 0.0], atol=1e-9)


def test_roundtrip_identity():
    T = T_from_xyz_rpy([0.1, -0.2, 0.3], [0.05, -0.0873, 0.2])
    T_il = lidar_in_imu(T, T)
    np.testing.assert_allclose(T_il, np.eye(4), atol=1e-9)
