import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from wheelchair_perception.pointcloud_to_laserscan_node import (  # noqa: E402
    PointCloudToLaserScanNode,
    ScanProjectionConfig,
    project_points_to_scan,
)


def test_project_points_to_scan_uses_nearest_range_per_angle_bin():
    config = ScanProjectionConfig(
        angle_min=-math.pi / 4,
        angle_max=math.pi / 4,
        angle_increment=math.pi / 8,
        range_min=0.05,
        range_max=5.0,
        z_min=-0.1,
        z_max=1.0,
    )
    ranges = project_points_to_scan(
        [
            (2.0, 0.0, 0.0),
            (1.0, 0.0, 0.0),
            (1.0, 1.0, 0.0),
            (1.0, -1.0, 2.0),
            (-1.0, 0.0, 0.0),
        ],
        config,
    )

    center_index = int(round((0.0 - config.angle_min) / config.angle_increment))
    right_index = int(round((math.pi / 4 - config.angle_min) / config.angle_increment))
    assert ranges[center_index] == 1.0
    assert ranges[right_index] == pytest_approx(math.sqrt(2.0))
    assert all(math.isinf(value) for index, value in enumerate(ranges) if index not in (center_index, right_index))


def test_scan_projection_uses_expected_laserscan_beam_count():
    config = ScanProjectionConfig()

    assert config.beam_count == 241
    assert config.realized_angle_max <= config.angle_max
    assert config.realized_angle_max + config.angle_increment > config.angle_max


def test_pointcloud_subscription_accepts_best_effort_sensor_qos():
    import rclpy
    from rclpy.context import Context
    from rclpy.qos import ReliabilityPolicy

    context = Context()
    rclpy.init(context=context)
    node = PointCloudToLaserScanNode(context=context)
    try:
        assert node.cloud_sub.qos_profile.reliability == ReliabilityPolicy.BEST_EFFORT
    finally:
        node.destroy_node()
        rclpy.shutdown(context=context)


def pytest_approx(value):
    import pytest

    return pytest.approx(value, rel=1e-3)


def test_restamp_output_defaults_false_and_preserves_capture_stamp():
    """D024: /scan must keep the cloud's capture timestamp by default."""
    import rclpy
    from rclpy.context import Context
    from std_msgs.msg import Header

    context = Context()
    rclpy.init(context=context)
    node = PointCloudToLaserScanNode(context=context)
    try:
        assert node.restamp_output is False
        header = Header()
        header.frame_id = "base_link"
        header.stamp.sec = 1234
        header.stamp.nanosec = 567
        scan = node._make_scan(header, project_points_to_scan([], node.config))
        # capture stamp preserved, not overwritten with wall clock "now"
        assert scan.header.stamp.sec == 1234
        assert scan.header.stamp.nanosec == 567
    finally:
        node.destroy_node()
        rclpy.shutdown(context=context)


def test_make_scan_falls_back_to_now_when_source_stamp_absent():
    """When source stamp is 0/0, fall back to clock so downstream still gets a stamp."""
    import rclpy
    from rclpy.context import Context
    from std_msgs.msg import Header

    context = Context()
    rclpy.init(context=context)
    node = PointCloudToLaserScanNode(context=context)
    try:
        header = Header()
        header.frame_id = "base_link"
        header.stamp.sec = 0
        header.stamp.nanosec = 0
        scan = node._make_scan(header, project_points_to_scan([], node.config))
        assert scan.header.stamp.sec != 0 or scan.header.stamp.nanosec != 0
    finally:
        node.destroy_node()
        rclpy.shutdown(context=context)
