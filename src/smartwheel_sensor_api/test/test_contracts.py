import numpy as np
import pytest

from smartwheel_sensor_api.contracts import BackendMode, TimestampMonitor, guard_real_backend
from smartwheel_sensor_api.pointcloud import (
    apply_transform,
    convert_points_to_metres,
    voxel_downsample,
)


def test_millimetres_are_converted_to_metres():
    points = np.array([[1000.0, -500.0, 250.0]])
    np.testing.assert_allclose(convert_points_to_metres(points, "mm"), [[1.0, -0.5, 0.25]])


def test_unknown_point_unit_is_rejected():
    with pytest.raises(ValueError, match="unsupported point unit"):
        convert_points_to_metres(np.zeros((1, 3)), "cm")


def test_timestamp_monitor_rejects_regression():
    monitor = TimestampMonitor()
    monitor.observe(1.0)
    monitor.observe(1.0)
    with pytest.raises(ValueError, match="timestamp regression"):
        monitor.observe(0.9)


def test_real_backend_is_fail_closed():
    with pytest.raises(RuntimeError, match="hardware_enabled=false"):
        guard_real_backend("real", hardware_enabled=False, adapter_ready=True)
    with pytest.raises(NotImplementedError, match="NOT_IMPLEMENTED"):
        guard_real_backend("real", hardware_enabled=True, adapter_ready=False)
    assert guard_real_backend("mock", False, False) is BackendMode.MOCK


def test_transform_and_voxel_filter():
    points = np.array([[0.0, 0.0, 1.0], [0.01, 0.0, 1.0], [2.0, 0.0, 1.0]])
    matrix = np.eye(4)
    matrix[:3, 3] = [1.0, 2.0, 3.0]
    moved = apply_transform(points, matrix)
    np.testing.assert_allclose(moved[0], [1.0, 2.0, 4.0])
    filtered, _ = voxel_downsample(points, 0.1)
    assert filtered.shape == (2, 3)

