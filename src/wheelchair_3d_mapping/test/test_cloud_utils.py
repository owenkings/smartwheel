"""Minimal tests for cloud_utils geometry helpers (no ROS required)."""
import numpy as np

from wheelchair_3d_mapping import cloud_utils


def test_quat_identity():
    R = cloud_utils.quat_to_rotation(0.0, 0.0, 0.0, 1.0)
    assert np.allclose(R, np.eye(3))


def test_quat_yaw_90():
    # +90 deg about z maps +x -> +y
    R = cloud_utils.quat_to_rotation(0.0, 0.0, 0.7071068, 0.7071068)
    assert np.allclose(R @ np.array([1.0, 0.0, 0.0]), [0.0, 1.0, 0.0], atol=1e-5)


def test_apply_transform_translation():
    mat = np.eye(4)
    mat[:3, 3] = [1.0, 2.0, 3.0]
    out = cloud_utils.apply_transform(np.array([[0.0, 0.0, 0.0]]), mat)
    assert np.allclose(out, [[1.0, 2.0, 3.0]])


def test_filter_by_range():
    xyz = np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [100.0, 0.0, 0.0]])
    out, _ = cloud_utils.filter_by_range(xyz, None, 0.5, 20.0)
    assert out.shape[0] == 1
    assert np.allclose(out[0], [1.0, 0.0, 0.0])


def test_voxel_downsample_reduces():
    xyz = np.array([[0.0, 0.0, 0.0], [0.01, 0.0, 0.0], [5.0, 5.0, 5.0]])
    out, _ = cloud_utils.voxel_downsample(xyz, None, 0.1)
    assert out.shape[0] == 2
