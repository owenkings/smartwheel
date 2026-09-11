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
        self.count = 0

    def publish(self, msg):
        self.last = msg
        self.count += 1


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


def _set_motion_pair(node, delta_ns=50000000):
    node.motion_compensation = True
    node.motion_fixed_frame = "odom"
    node.require_synchronized_pair = True
    now_ns = node.get_clock().now().nanoseconds - 10000000
    for state, stamp_ns, xyz, intensity in (
        (node.left, now_ns - delta_ns, [2.0, 0.0, 0.0], 101.0),
        (node.right, now_ns, [1.95, 0.0, 0.0], 202.0),
    ):
        state.stamp = Time(sec=stamp_ns // 1000000000, nanosec=stamp_ns % 1000000000)
        state.xyz = np.array([xyz])
        state.inten = np.array([intensity])
        state.recv_time = __import__("time").monotonic()


def test_two_time_compensation_preserves_intensity_and_original_stamps(ros_init):
    from geometry_msgs.msg import TransformStamped

    node = _make_node()
    try:
        _set_motion_pair(node)
        original_left = (node.left.stamp.sec, node.left.stamp.nanosec)
        original_right = (node.right.stamp.sec, node.right.stamp.nanosec)
        calls = []

        class FullTF:
            def lookup_transform_full(self, **kwargs):
                calls.append(kwargs)
                tf = TransformStamped()
                tf.transform.rotation.w = 1.0
                # Synthetic robot moves +x at 1 m/s. World-fixed point shifts
                # -0.05 m in the current base frame across a 50 ms capture gap.
                tf.transform.translation.x = -(
                    kwargs["target_time"].nanoseconds - kwargs["source_time"].nanoseconds
                ) * 1.0e-9
                return tf

        node.tf_buffer = FullTF()
        node._publish_merged()
        assert node.pub.count == 1
        xyz, intensity = cloud_utils.read_xyz_intensity(node.pub.last)
        np.testing.assert_allclose(xyz[:, 0], [1.95, 1.95], atol=1.0e-6)
        np.testing.assert_array_equal(intensity, [101.0, 202.0])
        assert len(calls) == 2
        assert all(call["fixed_frame"] == "odom" for call in calls)
        assert all(call["source_frame"] == call["target_frame"] == "base_link" for call in calls)
        assert (node.left.stamp.sec, node.left.stamp.nanosec) == original_left
        assert (node.right.stamp.sec, node.right.stamp.nanosec) == original_right
        assert (node.pub.last.header.stamp.sec, node.pub.last.header.stamp.nanosec) == original_right
        assert node._last_motion_compensated
        node._publish_merged()
        assert node.pub.count == 1  # Timer must not manufacture repeated scans.
    finally:
        node.destroy_node()


def test_missing_two_time_tf_rejects_without_consuming_pair(ros_init):
    node = _make_node()
    try:
        _set_motion_pair(node)
        node._lookup_motion = lambda *_: None
        node._publish_merged()
        assert node.pub.last is None
        assert node.left.last_published_stamp_ns is None
        assert "two-time TF" in node._last_rejection_reason
        # The same still-fresh pair can recover once exact-time TF arrives.
        node._lookup_motion = lambda *_: np.eye(4)
        node._publish_merged()
        assert node.pub.count == 1
    finally:
        node.destroy_node()


def test_two_time_buffer_compensates_rotation(ros_init):
    from geometry_msgs.msg import TransformStamped

    node = _make_node()
    try:
        _set_motion_pair(node)
        yaw = 0.1
        expected = np.array([2.0 * np.cos(yaw), -2.0 * np.sin(yaw), 0.0])
        node.right.xyz = expected.reshape(1, 3)
        for stamp, angle in ((node.left.stamp, 0.0), (node.right.stamp, yaw)):
            tf = TransformStamped()
            tf.header.frame_id = "odom"
            tf.child_frame_id = "base_link"
            tf.header.stamp = stamp
            tf.transform.rotation.z = np.sin(angle / 2.0)
            tf.transform.rotation.w = np.cos(angle / 2.0)
            node.tf_buffer.set_transform(tf, "offline_test")
        node._publish_merged()
        assert node.pub.count == 1
        xyz, _ = cloud_utils.read_xyz_intensity(node.pub.last)
        np.testing.assert_allclose(xyz, [expected, expected], atol=1.0e-6)
    finally:
        node.destroy_node()


def test_pending_pair_survives_new_input_until_exact_time_tf_arrives(ros_init):
    node = _make_node()
    try:
        _set_motion_pair(node)
        expected_stamp = (node.right.stamp.sec, node.right.stamp.nanosec)
        node._lookup_motion = lambda *_: None
        node._publish_merged()
        assert node._pending_motion_pair is not None
        # Simulate the next callback replacing the newest states while the
        # delayed trajectory can now transform the previously selected pair.
        _set_motion_pair(node, delta_ns=40000000)
        node._lookup_motion = lambda *_: np.eye(4)
        node._publish_merged()
        assert node.pub.count == 1
        assert (node.pub.last.header.stamp.sec, node.pub.last.header.stamp.nanosec) == expected_stamp
        assert node._pending_motion_pair is None
    finally:
        node.destroy_node()


@pytest.mark.parametrize("bad_stamp", ["zero", "future", "stale", "bad_nsec"])
def test_motion_compensation_rejects_invalid_clock_stamps(ros_init, bad_stamp):
    node = _make_node()
    try:
        _set_motion_pair(node)
        node._lookup_motion = lambda *_: np.eye(4)
        if bad_stamp == "zero":
            node.left.stamp = Time()
        elif bad_stamp == "future":
            node.left.stamp.sec += 1
            node.right.stamp.sec += 1
        elif bad_stamp == "stale":
            node.left.stamp.sec -= 2
            node.right.stamp.sec -= 2
        else:
            node.left.stamp.nanosec = 1000000000
        node._publish_merged()
        assert node.pub.last is None
    finally:
        node.destroy_node()


def test_motion_compensation_rejects_large_pair_gap_even_if_pair_flag_off(ros_init):
    node = _make_node()
    try:
        _set_motion_pair(node, delta_ns=100000000)
        node.require_synchronized_pair = False
        node._lookup_motion = lambda *_: np.eye(4)
        node._publish_merged()
        assert node.pub.last is None
        assert "exceeds" in node._last_rejection_reason
    finally:
        node.destroy_node()


@pytest.mark.parametrize("stamp_ns", [0, 9999999999, 10000000000])
def test_motion_input_rejects_zero_duplicate_and_backwards(stamp_ns, ros_init):
    node = _make_node()
    try:
        node.motion_compensation = True
        node.left.last_input_stamp_ns = 10000000000
        node.left.xyz = np.array([[1.0, 0.0, 0.0]])
        msg = _make_cloud(stamp_ns // 1000000000, stamp_ns % 1000000000, [[1, 0, 0]])
        node._on_cloud(msg, node.left)
        assert node.left.xyz is None
        assert node.left.last_input_stamp_ns == 10000000000
        assert node._timestamp_rejection_count == 1
    finally:
        node.destroy_node()
