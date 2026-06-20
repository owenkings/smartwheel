"""Unit tests for dual_lidar_cloud_fusion_node audit fixes.

Covers:
  * D034/D168 - /points_merged.header.stamp comes from the source frame
    acquisition time, with wall-clock fallback only when all source stamps
    are zero.
  * D179 - invalid / placeholder points ((0,0,0) and NaN) are removed when a
    PointCloud2 is read and range-filtered.

These tests require ROS 2 (sensor_msgs / sensor_msgs_py / rclpy) to be sourced;
they are skipped gracefully if the environment is not available.
"""
import numpy as np
import pytest

rclpy = pytest.importorskip("rclpy")
pytest.importorskip("sensor_msgs")
pytest.importorskip("sensor_msgs_py")

from builtin_interfaces.msg import Time
from std_msgs.msg import Header

from wheelchair_3d_mapping import cloud_utils
from wheelchair_3d_mapping.dual_lidar_cloud_fusion_node import DualLidarCloudFusionNode


@pytest.fixture(scope="module")
def ros_init():
    rclpy.init()
    yield
    rclpy.shutdown()


def _make_cloud(stamp_sec, stamp_nanosec, xyz, frame_id="xtm60_left_link"):
    header = Header()
    header.stamp = Time(sec=int(stamp_sec), nanosec=int(stamp_nanosec))
    header.frame_id = frame_id
    return cloud_utils.make_xyzi_cloud(header, np.asarray(xyz, dtype=np.float64), None)


class _CapturePub:
    """Stand-in publisher that records the last published message."""

    def __init__(self):
        self.last = None

    def publish(self, msg):
        self.last = msg


def _make_node():
    node = DualLidarCloudFusionNode()
    # Replace the real publishers so the test does not depend on a running
    # ROS graph and so we can inspect the published header.
    node.pub = _CapturePub()
    node.status_pub = None
    return node


# --------------------------------------------------------------------------
# D034/D168: output stamp comes from the source frame
# --------------------------------------------------------------------------

def test_merged_stamp_uses_source_frame_stamp(ros_init):
    node = _make_node()
    try:
        # Single-lidar (left only) fresh state with a known source stamp.
        node.left.xyz = np.array([[1.0, 0.0, 0.0], [2.0, 0.0, 0.0]])
        node.left.inten = None
        node.left.stamp = Time(sec=123, nanosec=456)
        node.left.recv_time = __import__("time").monotonic()

        node._publish_merged()

        assert node.pub.last is not None
        assert node.pub.last.header.stamp.sec == 123
        assert node.pub.last.header.stamp.nanosec == 456
    finally:
        node.destroy_node()


def test_merged_stamp_picks_newer_of_two_lidars(ros_init):
    node = _make_node()
    try:
        now = __import__("time").monotonic()
        node.left.xyz = np.array([[1.0, 0.0, 0.0]])
        node.left.inten = None
        node.left.stamp = Time(sec=100, nanosec=0)
        node.left.recv_time = now

        node.right.xyz = np.array([[0.0, 1.0, 0.0]])
        node.right.inten = None
        node.right.stamp = Time(sec=105, nanosec=0)
        node.right.recv_time = now

        node._publish_merged()

        # Newer (larger) stamp must win.
        assert node.pub.last.header.stamp.sec == 105
        assert node.pub.last.header.stamp.nanosec == 0
    finally:
        node.destroy_node()


def test_merged_stamp_falls_back_to_wall_clock_when_all_zero(ros_init):
    node = _make_node()
    try:
        node.left.xyz = np.array([[1.0, 0.0, 0.0]])
        node.left.inten = None
        node.left.stamp = Time(sec=0, nanosec=0)
        node.left.recv_time = __import__("time").monotonic()

        node._publish_merged()

        stamp = node.pub.last.header.stamp
        # Wall-clock fallback => non-zero stamp from the node clock.
        assert not (stamp.sec == 0 and stamp.nanosec == 0)
    finally:
        node.destroy_node()


def test_merged_frame_id_is_target_frame(ros_init):
    node = _make_node()
    try:
        node.left.xyz = np.array([[1.0, 0.0, 0.0]])
        node.left.inten = None
        node.left.stamp = Time(sec=5, nanosec=5)
        node.left.recv_time = __import__("time").monotonic()

        node._publish_merged()

        assert node.pub.last.header.frame_id == node.target_frame
    finally:
        node.destroy_node()


# --------------------------------------------------------------------------
# D179: invalid / placeholder points are removed end-to-end through a cloud
# --------------------------------------------------------------------------

def test_cloud_with_invalid_points_is_cleaned(ros_init):
    # Build a real PointCloud2 containing a placeholder origin point, a NaN
    # point and two valid points, then run it through the same read + range
    # filter the fusion node uses.
    xyz = np.array(
        [
            [0.0, 0.0, 0.0],     # placeholder origin -> dropped (D179)
            [np.nan, 1.0, 0.0],  # NaN -> dropped
            [1.0, 0.0, 0.0],     # valid
            [3.0, 0.0, 0.0],     # valid
        ]
    )
    msg = _make_cloud(10, 0, xyz)

    read_xyz, _ = cloud_utils.read_xyz_intensity(msg)
    filtered, _ = cloud_utils.filter_by_range(read_xyz, None, 0.05, 20.0)

    assert filtered.shape[0] == 2
    # No remaining point is at the origin and all are finite.
    assert np.all(np.isfinite(filtered))
    assert np.all(np.linalg.norm(filtered, axis=1) > 0.0)
