import numpy as np

from dual_lidar_fusion.pairing import TimestampPairer
from smartwheel_sensor_api import apply_transform
from smartwheel_sensor_api.pointcloud import transform_matrix


def test_frames_pair_only_inside_tolerance():
    pairer = TimestampPairer[str](tolerance_sec=0.02)
    assert pairer.add_left(1.000, "left") is None
    pair = pairer.add_right(1.015, "right")
    assert pair is not None
    assert pair[0].value == "left"
    assert pair[1].value == "right"


def test_stale_frames_are_dropped_not_joined_to_latest():
    pairer = TimestampPairer[str](tolerance_sec=0.01)
    pairer.add_left(1.0, "old-left")
    assert pairer.add_right(2.0, "right") is None
    assert pairer.dropped == 1


def test_extrinsic_transform_is_applied_in_base_frame():
    matrix = transform_matrix([1.0, 2.0, 0.5], [0.0, 0.0, np.pi / 2.0])
    transformed = apply_transform(np.array([[1.0, 0.0, 0.0]]), matrix)
    np.testing.assert_allclose(transformed, [[1.0, 3.0, 0.5]], atol=1e-9)

