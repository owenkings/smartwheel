"""Unit tests for lio_cloud_adapter_node (Task 3, FAST-LIO input adapter).

Covers:
  * (a) NaN / (0,0,0) placeholder point rejection (req 2.5).
  * (b) add_zero_time_field appends a 'time'=0 field when enabled and not when
        disabled (req 2.3).
  * (c) restamp_to_now overwrites the header stamp with "now" while preserving
        frame_id (req 2.6).
  * (d) output frame_id is preserved and XYZI field layout / values are correct
        (req 2.1).

These tests require ROS 2 (sensor_msgs / sensor_msgs_py / rclpy) to be sourced;
they are skipped gracefully if the environment is not available. Synthetic
PointCloud2 messages are constructed in-process so no hardware/radar is needed.
"""
import numpy as np
import pytest

rclpy = pytest.importorskip("rclpy")
pytest.importorskip("sensor_msgs")
pytest.importorskip("sensor_msgs_py")

from builtin_interfaces.msg import Time
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

from wheelchair_3d_mapping import cloud_utils
from wheelchair_3d_mapping.lio_cloud_adapter_node import LioCloudAdapterNode


@pytest.fixture(scope="module")
def ros_init():
    rclpy.init()
    yield
    rclpy.shutdown()


def _make_cloud(xyz, inten=None, frame_id="xtm60_left_link", stamp_sec=10, stamp_nanosec=0):
    header = Header()
    header.stamp = Time(sec=int(stamp_sec), nanosec=int(stamp_nanosec))
    header.frame_id = frame_id
    xyz = np.asarray(xyz, dtype=np.float64)
    if inten is not None:
        inten = np.asarray(inten, dtype=np.float32)
    return cloud_utils.make_xyzi_cloud(header, xyz, inten)


def _make_node(**param_overrides):
    node = LioCloudAdapterNode()
    for key, value in param_overrides.items():
        setattr(node, key, value)
    return node


def _read_xyz(msg):
    arr = point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=False)
    return np.column_stack((arr["x"], arr["y"], arr["z"])).astype(np.float64)


# --------------------------------------------------------------------------
# (a) NaN / (0,0,0) placeholder rejection
# --------------------------------------------------------------------------

def test_invalid_points_are_rejected(ros_init):
    node = _make_node()
    try:
        xyz = np.array(
            [
                [0.0, 0.0, 0.0],     # placeholder origin -> dropped
                [np.nan, 1.0, 0.0],  # NaN -> dropped
                [1.0, 0.0, 0.0],     # valid
                [3.0, 0.0, 0.0],     # valid
            ]
        )
        out = node.adapt(_make_cloud(xyz))
        assert out.width == 2
        pts = _read_xyz(out)
        assert np.all(np.isfinite(pts))
        assert np.all(np.linalg.norm(pts, axis=1) > 0.0)
    finally:
        node.destroy_node()


# --------------------------------------------------------------------------
# (b) add_zero_time_field behaviour
# --------------------------------------------------------------------------

def test_time_field_absent_by_default(ros_init):
    node = _make_node()
    try:
        out = node.adapt(_make_cloud([[1.0, 0.0, 0.0]]))
        assert "time" not in cloud_utils.field_names(out)
        assert cloud_utils.field_names(out) == ["x", "y", "z", "intensity"]
        assert out.point_step == 16
    finally:
        node.destroy_node()


def test_time_field_added_when_enabled(ros_init):
    node = _make_node(add_zero_time_field=True)
    try:
        out = node.adapt(_make_cloud([[1.0, 0.0, 0.0], [2.0, 0.0, 0.0]]))
        names = cloud_utils.field_names(out)
        # FAST-LIO velodyne format: x,y,z,intensity,time,ring (Task 7 fix).
        assert "time" in names
        assert "ring" in names
        assert out.point_step == 24
        arr = point_cloud2.read_points(out, field_names=("time", "ring"), skip_nans=False)
        # per-point time ramps 0..0.1s; last point > 0, monotonic non-decreasing
        times = np.asarray(arr["time"], dtype=np.float32)
        assert times[-1] > 0.0
        assert np.all(np.diff(times) >= -1e-9)
        # ring is a valid index (0)
        assert np.all(np.asarray(arr["ring"]).astype(int) == 0)
    finally:
        node.destroy_node()


# --------------------------------------------------------------------------
# (c) restamp_to_now behaviour
# --------------------------------------------------------------------------

def test_restamp_disabled_preserves_source_stamp(ros_init):
    node = _make_node()
    try:
        out = node.adapt(_make_cloud([[1.0, 0.0, 0.0]], stamp_sec=123, stamp_nanosec=456))
        assert out.header.stamp.sec == 123
        assert out.header.stamp.nanosec == 456
    finally:
        node.destroy_node()


def test_restamp_to_now_overwrites_stamp(ros_init):
    node = _make_node(restamp_to_now=True)
    try:
        src = _make_cloud([[1.0, 0.0, 0.0]], stamp_sec=0, stamp_nanosec=0)
        out = node.adapt(src)
        # "now" from the node clock must be non-zero and frame_id preserved.
        assert not (out.header.stamp.sec == 0 and out.header.stamp.nanosec == 0)
        assert out.header.frame_id == "xtm60_left_link"
        # The source message header must not be mutated.
        assert src.header.stamp.sec == 0 and src.header.stamp.nanosec == 0
    finally:
        node.destroy_node()


# --------------------------------------------------------------------------
# (d) frame_id preserved + XYZI values correct
# --------------------------------------------------------------------------

def test_frame_id_preserved_and_xyzi_correct(ros_init):
    node = _make_node()
    try:
        xyz = [[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]]
        inten = [11.0, 22.0]
        out = node.adapt(_make_cloud(xyz, inten=inten, frame_id="xtm60_left_link"))
        assert out.header.frame_id == "xtm60_left_link"
        assert cloud_utils.field_names(out) == ["x", "y", "z", "intensity"]
        arr = point_cloud2.read_points(
            out, field_names=("x", "y", "z", "intensity"), skip_nans=False
        )
        got_xyz = np.column_stack((arr["x"], arr["y"], arr["z"])).astype(np.float64)
        got_i = np.asarray(arr["intensity"], dtype=np.float32)
        np.testing.assert_allclose(got_xyz, np.asarray(xyz), rtol=1e-6)
        np.testing.assert_allclose(got_i, np.asarray(inten, dtype=np.float32), rtol=1e-6)
    finally:
        node.destroy_node()
