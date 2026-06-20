"""Unit tests for ground_plane_calibrator_node task-10 behaviour.

Covers (Requirements 2.5, 2.6):
  * Diagnostic JSON on /calibration/ground_plane includes
    height / pitch / inlier_ratio / valid.
  * Failure fallback: when a fit is invalid (ground not visible / low inlier
    ratio) but a previous valid transform exists, the node keeps re-broadcasting
    the last valid TF and never publishes invalid extrinsics.
  * The two recalibration modes:
      - recalibrate_period_sec == 0  -> calibrate once after settle_frames, lock.
      - recalibrate_period_sec  > 0  -> periodic re-estimation.

These tests require ROS 2 (sensor_msgs / sensor_msgs_py / rclpy) to be sourced;
they are skipped gracefully if the environment is not available.
"""
import json

import numpy as np
import pytest

rclpy = pytest.importorskip("rclpy")
pytest.importorskip("sensor_msgs")
pytest.importorskip("sensor_msgs_py")

from builtin_interfaces.msg import Time
from std_msgs.msg import Header

from wheelchair_3d_mapping import cloud_utils
from wheelchair_3d_mapping.ground_plane_calibrator_node import (
    GroundPlaneCalibratorNode,
)


@pytest.fixture(scope="module")
def ros_init():
    rclpy.init()
    yield
    rclpy.shutdown()


# --------------------------------------------------------------------------
# Synthetic cloud helpers
# --------------------------------------------------------------------------

def _make_cloud(xyz, frame_id="xtm60_left_link", stamp_sec=1, stamp_nanosec=0):
    header = Header()
    header.stamp = Time(sec=int(stamp_sec), nanosec=int(stamp_nanosec))
    header.frame_id = frame_id
    return cloud_utils.make_xyzi_cloud(header, np.asarray(xyz, dtype=np.float64), None)


def _ground_cloud(height=0.45, n=400, noise=0.002, seed=0):
    """A near-horizontal ground plane in the sensor frame.

    Sensor convention: y = up.  A flat floor below the radar at distance
    ``height`` therefore lives at y == -height with the plane normal along y.
    Points spread in the x (left) and z (forward) directions.
    """
    rng = np.random.default_rng(seed)
    x = rng.uniform(-2.0, 2.0, size=n)
    z = rng.uniform(0.5, 4.0, size=n)
    y = np.full(n, -height) + rng.normal(0.0, noise, size=n)
    return np.column_stack((x, y, z))


def _wall_cloud(n=400, noise=0.002, seed=1):
    """A vertical wall (normal along sensor z = forward), NOT a ground plane."""
    rng = np.random.default_rng(seed)
    x = rng.uniform(-2.0, 2.0, size=n)
    y = rng.uniform(-1.0, 1.0, size=n)
    z = np.full(n, 2.0) + rng.normal(0.0, noise, size=n)
    return np.column_stack((x, y, z))


class _CaptureBroadcaster:
    """Stand-in StaticTransformBroadcaster recording every sendTransform."""

    def __init__(self):
        self.sent = []

    def sendTransform(self, tf):
        self.sent.append(tf)


class _CapturePub:
    def __init__(self):
        self.messages = []

    @property
    def last(self):
        return self.messages[-1] if self.messages else None

    def publish(self, msg):
        self.messages.append(msg)


def _make_node(**overrides):
    node = GroundPlaneCalibratorNode()
    node._tf_broadcaster = _CaptureBroadcaster()
    node._diag_pub = _CapturePub()
    for k, v in overrides.items():
        setattr(node, k, v)
    return node


# --------------------------------------------------------------------------
# req 2.6: diagnostic JSON content
# --------------------------------------------------------------------------

def test_diagnostic_json_has_required_keys(ros_init):
    node = _make_node(_settle_frames=1)
    try:
        node._on_cloud(_make_cloud(_ground_cloud(height=0.45)))
        assert node._diag_pub.last is not None
        payload = json.loads(node._diag_pub.last.data)
        for key in ("height", "pitch_deg", "inlier_ratio", "valid"):
            assert key in payload, f"missing diagnostic key: {key}"
        assert payload["valid"] is True
        assert payload["height"] == pytest.approx(0.45, abs=0.05)
    finally:
        node.destroy_node()


# --------------------------------------------------------------------------
# req 2.5: failure fallback keeps last valid TF, never publishes invalid
# --------------------------------------------------------------------------

def test_invalid_fit_retains_last_valid_tf(ros_init):
    # Periodic mode so calibration re-attempts every frame.
    node = _make_node(_recalib_period=0.0, _settle_frames=1)
    try:
        # First: a valid ground frame -> publishes a TF.
        node._on_cloud(_make_cloud(_ground_cloud(height=0.40)))
        assert node._calibrated is True
        assert len(node._tf_broadcaster.sent) == 1
        good_tf = node._tf_broadcaster.sent[-1]
        good_z = good_tf.transform.translation.z
        assert good_z == pytest.approx(0.40, abs=0.05)

        # Force a recalibration attempt with an invalid (wall) cloud by
        # switching to periodic mode and resetting the lock.
        node._recalib_period = 0.0001
        node._last_calib_time = 0.0
        node._calibrated = True  # already locked, but periodic path is used
        import time as _t
        _t.sleep(0.01)
        node._on_cloud(_make_cloud(_wall_cloud()))

        # The diagnostic must report invalid, and the re-broadcast TF must equal
        # the last valid one (never an invalid extrinsic).
        payload = json.loads(node._diag_pub.last.data)
        assert payload["valid"] is False
        # A TF was re-broadcast (fallback) and it matches the last valid z.
        assert node._tf_broadcaster.sent[-1].transform.translation.z == pytest.approx(
            good_z, abs=1e-9
        )
    finally:
        node.destroy_node()


def test_invalid_fit_with_no_prior_valid_publishes_no_tf(ros_init):
    node = _make_node(_recalib_period=0.0, _settle_frames=1)
    try:
        node._on_cloud(_make_cloud(_wall_cloud()))
        # No valid calibration ever -> never broadcast a TF.
        assert node._calibrated is False
        assert len(node._tf_broadcaster.sent) == 0
        payload = json.loads(node._diag_pub.last.data)
        assert payload["valid"] is False
    finally:
        node.destroy_node()


# --------------------------------------------------------------------------
# Recalibration modes
# --------------------------------------------------------------------------

def test_oneshot_mode_locks_after_settle_frames(ros_init):
    node = _make_node(_recalib_period=0.0, _settle_frames=3)
    try:
        # First two frames are within the settle window -> no calibration yet.
        node._on_cloud(_make_cloud(_ground_cloud(height=0.45)))
        node._on_cloud(_make_cloud(_ground_cloud(height=0.45)))
        assert node._calibrated is False
        assert len(node._tf_broadcaster.sent) == 0

        # Third frame reaches settle_frames -> calibrates and locks.
        node._on_cloud(_make_cloud(_ground_cloud(height=0.45)))
        assert node._calibrated is True
        assert len(node._tf_broadcaster.sent) == 1

        # Subsequent frames are ignored (locked); no extra TFs published.
        node._on_cloud(_make_cloud(_ground_cloud(height=0.45)))
        node._on_cloud(_make_cloud(_ground_cloud(height=0.45)))
        assert len(node._tf_broadcaster.sent) == 1
    finally:
        node.destroy_node()


def test_periodic_mode_recalibrates_after_period(ros_init):
    import time as _t

    # Period must be comfortably larger than a single RANSAC fit (~tens of ms at
    # the 400-iteration default) so the "within period" frame reliably lands
    # inside the window regardless of machine speed.
    node = _make_node(_recalib_period=0.5, _settle_frames=1)
    try:
        node._on_cloud(_make_cloud(_ground_cloud(height=0.45)))
        assert len(node._tf_broadcaster.sent) == 1

        # Within the period -> the next frame is skipped.
        node._on_cloud(_make_cloud(_ground_cloud(height=0.45)))
        assert len(node._tf_broadcaster.sent) == 1

        # After the period elapses -> recalibrates and publishes again.
        _t.sleep(0.6)
        node._on_cloud(_make_cloud(_ground_cloud(height=0.50)))
        assert len(node._tf_broadcaster.sent) == 2
        assert node._tf_broadcaster.sent[-1].transform.translation.z == pytest.approx(
            0.50, abs=0.05
        )
    finally:
        node.destroy_node()


# --------------------------------------------------------------------------
# Fixed / manual-override mode (USER-DECIDED override of req 2.7)
# --------------------------------------------------------------------------

def _make_fixed_node(fixed_height=0.50, fixed_pitch_deg=0.0):
    """Construct the node in fixed-height mode via parameter overrides.

    In fixed mode the node does NOT subscribe to the cloud and schedules a
    one-shot timer to publish the static TF + diagnostic. The timer is not
    spun in the test, so we swap in capture stubs and invoke the one-shot
    callback directly to assert the startup publish happens WITHOUT any cloud.
    """
    from rclpy.parameter import Parameter

    node = GroundPlaneCalibratorNode(
        parameter_overrides=[
            Parameter("fixed_height", Parameter.Type.DOUBLE, float(fixed_height)),
            Parameter("fixed_pitch_deg", Parameter.Type.DOUBLE, float(fixed_pitch_deg)),
        ]
    )
    node._tf_broadcaster = _CaptureBroadcaster()
    node._diag_pub = _CapturePub()
    return node


def test_fixed_height_publishes_tf_without_any_cloud(ros_init):
    """req 2.7 manual override: fixed_height=0.50 publishes a TF with z==0.50
    and a valid "fixed" diagnostic, with NO RANSAC fit and NO cloud fed."""
    node = _make_fixed_node(fixed_height=0.50, fixed_pitch_deg=0.0)
    try:
        assert node._fixed_mode is True
        # No cloud has been delivered. Trigger the one-shot startup publish.
        node._fixed_publish_once()

        # Exactly one static TF, translation.z == 0.50 (the user-fixed height).
        assert len(node._tf_broadcaster.sent) == 1
        tf = node._tf_broadcaster.sent[-1]
        assert tf.header.frame_id == "base_link"
        assert tf.child_frame_id == "xtm60_left_link"
        assert tf.transform.translation.x == pytest.approx(0.45)
        assert tf.transform.translation.y == pytest.approx(0.24)
        assert tf.transform.translation.z == pytest.approx(0.50, abs=1e-9)

        # Diagnostic reports the fixed mode, valid=true, inlier_ratio null.
        payload = json.loads(node._diag_pub.last.data)
        assert payload["mode"] == "fixed"
        assert payload["valid"] is True
        assert payload["height"] == pytest.approx(0.50, abs=1e-9)
        assert payload["inlier_ratio"] is None
    finally:
        node.destroy_node()


def test_fixed_mode_does_not_subscribe_or_run_ransac(ros_init):
    """In fixed mode the node must not run the per-frame RANSAC lock path:
    there is no cloud subscription, so a stray cloud cannot trigger a fit."""
    node = _make_fixed_node(fixed_height=0.50)
    try:
        # No subscription was created in fixed mode (auto-cal path skipped).
        # rclpy stores the node's own subscriptions on the private list.
        assert len(list(node.subscriptions)) == 0
        # frame_count never advances because _on_cloud is never wired.
        assert node._frame_count == 0
        node._fixed_publish_once()
        payload = json.loads(node._diag_pub.last.data)
        assert payload["mode"] == "fixed"
    finally:
        node.destroy_node()


def test_default_mode_is_not_fixed(ros_init):
    """fixed_height==0.0 (default) keeps the full auto-cal path intact."""
    node = GroundPlaneCalibratorNode()
    try:
        assert node._fixed_mode is False
        assert node._fixed_height == pytest.approx(0.0)
    finally:
        node.destroy_node()
