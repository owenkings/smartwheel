"""Numerical correctness tests for the ground-plane RANSAC / geometry (Task 12).

These exercise the calibration math against *synthetic* clouds with a known
height and tilt plus noise/outliers, verifying (Requirements 2.1, 2.2, 2.3, 2.5):

  * RANSAC-estimated height (|d|) and pitch are within tolerance for a plane at
    a known height + known tilt.                              (req 2.1/2.2/2.3)
  * The inlier mask / ratio is computed correctly (ground inliers counted,
    far outliers excluded).                                   (req 2.1)
  * Near-vertical normal detection accepts a ground plane and rejects a wall
    (normal not close to the sensor vertical axis).           (req 2.1)
  * Low inlier-ratio clouds are flagged ``valid == False`` so no invalid
    extrinsic is published.                                   (req 2.5)

The geometry/RANSAC routines are pure-numpy, but they live in the node module
whose top-level imports require ROS 2; the tests are therefore skipped
gracefully when rclpy / tf2_ros / sensor_msgs are not sourced.
"""
import json
import math

import numpy as np
import pytest

rclpy = pytest.importorskip("rclpy")
pytest.importorskip("tf2_ros")
pytest.importorskip("sensor_msgs")
pytest.importorskip("sensor_msgs_py")

from builtin_interfaces.msg import Time
from std_msgs.msg import Header

from wheelchair_3d_mapping import cloud_utils
from wheelchair_3d_mapping.ground_plane_calibrator_node import (
    GroundPlaneCalibratorNode,
    matrix_to_quaternion,
    ransac_fit_plane,
    signed_pitch_correction,
)


# ==========================================================================
# Synthetic cloud generators
# ==========================================================================
# Sensor convention (verified on hardware): z = forward, y = up, x = left.
# A ground plane therefore has its normal along ±y. A tilt of `pitch_deg`
# rotates the up-normal toward +z (forward), giving normal (0, cosθ, sinθ).

def _tilted_ground(height, pitch_deg=0.0, n=500, noise=0.002, seed=0):
    """Synthetic ground plane at perpendicular distance `height`, tilted by
    `pitch_deg` (rotation that levels to base-link pitch), plus Gaussian noise
    along the normal."""
    theta = math.radians(pitch_deg)
    normal = np.array([0.0, math.cos(theta), math.sin(theta)])
    normal = normal / np.linalg.norm(normal)
    # Foot of perpendicular from sensor origin onto the plane (ground is below
    # the radar -> negative y for the level case).
    p0 = -height * normal
    # Two in-plane basis vectors orthogonal to the normal.
    a = np.array([1.0, 0.0, 0.0])
    a = a - a.dot(normal) * normal
    a = a / np.linalg.norm(a)
    b = np.cross(normal, a)
    rng = np.random.default_rng(seed)
    s = rng.uniform(-2.0, 2.0, size=n)
    t = rng.uniform(0.5, 4.0, size=n)
    pts = p0 + s[:, None] * a + t[:, None] * b
    pts = pts + rng.normal(0.0, noise, size=(n, 1)) * normal
    return pts


def _wall(n=500, noise=0.002, seed=1):
    """A vertical wall: normal along sensor +z (forward) -> NOT a ground plane."""
    rng = np.random.default_rng(seed)
    x = rng.uniform(-2.0, 2.0, size=n)
    y = rng.uniform(-1.0, 1.0, size=n)
    z = np.full(n, 2.0) + rng.normal(0.0, noise, size=n)
    return np.column_stack((x, y, z))


def _ground_with_outliers(height=0.45, n_ground=30, n_outlier=1200, seed=2):
    """Small ground plane drowned in scattered outliers so the best-plane
    inlier ratio falls below the calibration threshold."""
    ground = _tilted_ground(height, pitch_deg=0.0, n=n_ground, noise=0.002, seed=seed)
    rng = np.random.default_rng(seed + 100)
    outliers = rng.uniform(-5.0, 5.0, size=(n_outlier, 3))
    return np.vstack((ground, outliers))


def _flat_plane(height, n=500, noise=0.002, seed=0):
    """A perfectly level horizontal plane at perpendicular distance ``height``
    BELOW the sensor (sensor convention y = up, so the plane lives at y == -height
    with its normal along +y).  Spread over x (left) and z (forward)."""
    rng = np.random.default_rng(seed)
    x = rng.uniform(-2.0, 2.0, size=n)
    z = rng.uniform(0.5, 4.0, size=n)
    y = np.full(n, -height) + rng.normal(0.0, noise, size=n)
    return np.column_stack((x, y, z))


def _make_cloud(xyz, frame_id="xtm60_left_link", stamp_sec=1, stamp_nanosec=0):
    header = Header()
    header.stamp = Time(sec=int(stamp_sec), nanosec=int(stamp_nanosec))
    header.frame_id = frame_id
    return cloud_utils.make_xyzi_cloud(header, np.asarray(xyz, dtype=np.float64), None)


# ==========================================================================
# Pure-function tests: ransac_fit_plane / signed_pitch_correction
# ==========================================================================

def test_height_estimate_level_plane():
    pts = _tilted_ground(height=0.45, pitch_deg=0.0, seed=0)
    result = ransac_fit_plane(pts, n_iterations=200, distance_thresh=0.02,
                              rng=np.random.default_rng(0))
    assert result is not None
    _, d, _ = result
    assert abs(d) == pytest.approx(0.45, abs=0.01)


def test_height_estimate_unchanged_by_tilt():
    # Perpendicular distance |d| must remain the height even when the plane is
    # tilted (it is the foot-of-perpendicular distance, not a vertical drop).
    pts = _tilted_ground(height=0.60, pitch_deg=12.0, seed=3)
    result = ransac_fit_plane(pts, n_iterations=200, distance_thresh=0.02,
                              rng=np.random.default_rng(1))
    assert result is not None
    _, d, _ = result
    assert abs(d) == pytest.approx(0.60, abs=0.02)


def test_pitch_estimate_level_is_zero():
    pts = _tilted_ground(height=0.45, pitch_deg=0.0, seed=0)
    normal, _, _ = ransac_fit_plane(pts, n_iterations=200, distance_thresh=0.02,
                                    rng=np.random.default_rng(0))
    pitch = signed_pitch_correction(normal)
    assert math.degrees(abs(pitch)) == pytest.approx(0.0, abs=1.0)


@pytest.mark.parametrize("tilt_deg", [5.0, 10.0, 18.0])
def test_pitch_estimate_matches_known_tilt(tilt_deg):
    pts = _tilted_ground(height=0.45, pitch_deg=tilt_deg, seed=7)
    normal, _, _ = ransac_fit_plane(pts, n_iterations=300, distance_thresh=0.02,
                                    rng=np.random.default_rng(2))
    pitch = signed_pitch_correction(normal)
    assert math.degrees(abs(pitch)) == pytest.approx(tilt_deg, abs=1.5)


def test_inlier_mask_counts_ground_excludes_outliers():
    height = 0.45
    n_ground, n_outlier = 400, 200
    ground = _tilted_ground(height, pitch_deg=0.0, n=n_ground, noise=0.002, seed=4)
    rng = np.random.default_rng(5)
    # Outliers placed well away from the plane (|y - (-height)| >> threshold).
    outliers = np.column_stack((
        rng.uniform(-2.0, 2.0, n_outlier),
        rng.uniform(1.0, 3.0, n_outlier),     # far above the floor
        rng.uniform(0.5, 4.0, n_outlier),
    ))
    pts = np.vstack((ground, outliers))
    _, _, mask = ransac_fit_plane(pts, n_iterations=300, distance_thresh=0.02,
                                  rng=np.random.default_rng(6))
    inlier_ratio = float(mask.sum()) / pts.shape[0]
    # The ground points should be (almost) entirely captured and the far
    # outliers excluded -> ratio close to n_ground / total.
    assert inlier_ratio == pytest.approx(n_ground / pts.shape[0], abs=0.05)
    # The captured inliers must overwhelmingly be the ground points (first n_ground).
    assert int(mask[:n_ground].sum()) >= int(0.95 * n_ground)
    assert int(mask[n_ground:].sum()) == 0


def test_ransac_returns_none_for_too_few_points():
    assert ransac_fit_plane(np.zeros((2, 3)), n_iterations=10) is None


# ==========================================================================
# Task-14 / Property-3 fix: two-plane selection + vertical-constrained RANSAC
# ==========================================================================
# Reproduces the indoor scene from auto_test/20260619_143746_ground_calib:
# a NEAR horizontal surface (~8 cm below, MORE points: seat/desk/platform) and
# the TRUE FLOOR (~0.57 m below, FEWER points).  The old "most inliers" RANSAC
# locked onto the near surface (height ~0.08 m).  The fix must select the FAR
# floor.  (req 2.2/2.3)

def _two_horizontal_planes(near_h=0.08, far_h=0.57, n_near=2580, n_far=1746,
                           noise=0.003, seed=11):
    """Two level horizontal planes below the sensor: a near, denser one and a
    far, sparser one (the true floor)."""
    near = _flat_plane(near_h, n=n_near, noise=noise, seed=seed)
    far = _flat_plane(far_h, n=n_far, noise=noise, seed=seed + 1)
    return np.vstack((near, far)), near.shape[0]


def test_selects_far_floor_over_nearer_denser_plane():
    # The near plane has MORE points but the fix's farthest-down prior must pick
    # the far floor (req 2.2/2.3, Property 3).
    pts, _ = _two_horizontal_planes(near_h=0.08, far_h=0.57)
    result = ransac_fit_plane(
        pts,
        n_iterations=400,
        distance_thresh=0.02,
        rng=np.random.default_rng(0),
        vertical_axis=np.array([0.0, 1.0, 0.0]),
        vertical_tol_rad=math.radians(25.0),
        prefer_farthest=True,
        ground_min_inlier_frac=0.5,
    )
    assert result is not None
    _, d, _ = result
    assert abs(d) == pytest.approx(0.57, abs=0.03)


def _tilted_fragment(abs_d, drop, pitch_deg, n=1200, noise=0.003, seed=21):
    """A sparse, TILTED near-vertical fragment whose plane offset ``|d|`` is
    INFLATED by tilt-extrapolation relative to its inliers' actual vertical
    depth.

    Mirrors the right-radar over-shoot scene (Task-14, req 2.2/2.3): the plane
    is tilted by ``pitch_deg`` so its normal is (0, cosθ, sinθ); its points are
    centred at vertical depth ``drop`` below the sensor, but because the patch is
    offset forward (+z) the foot-of-perpendicular offset ``|d|`` reads much
    larger (``abs_d`` ≫ ``drop``).  The consistency gate must reject this patch.
    """
    theta = math.radians(pitch_deg)
    normal = np.array([0.0, math.cos(theta), math.sin(theta)])
    normal = normal / np.linalg.norm(normal)
    rng = np.random.default_rng(seed)
    # Plane: normal·p + d = 0 with |d| = abs_d (plane below the sensor origin).
    d = abs_d
    # Sample y around -drop (the inliers' actual vertical depth) and solve z on
    # the plane:  cosθ*y + sinθ*z + d = 0  ->  z = -(cosθ*y + d)/sinθ.
    y = -drop + rng.normal(0.0, noise, size=n)
    if abs(math.sin(theta)) < 1e-6:
        z = rng.uniform(0.2, 2.0, size=n)
    else:
        z = -(math.cos(theta) * y + d) / math.sin(theta)
    x = rng.uniform(-1.0, 1.0, size=n)
    pts = np.column_stack((x, y, z))
    # Add noise along the normal to give the patch thickness.
    pts = pts + rng.normal(0.0, noise, size=(n, 1)) * normal
    return pts


def test_consistency_gate_rejects_tilted_extrapolated_fragment():
    # Right-radar scene (Task-14, req 2.2/2.3, Property 3): a LEVEL true floor at
    # depth ~0.55 m with MORE inliers competes with a sparse TILTED fragment
    # whose extrapolated |d| ~0.84 m but whose inliers' actual vertical depth is
    # only ~0.42 m.  Under the (now 15°) vertical tolerance the tilted fragment
    # is still admitted as "near-vertical", and the plain farthest-down prior
    # would pick its inflated |d| (the 0.749 m over-shoot bug).  The
    # extrapolation-consistency gate must REJECT the tilted fragment (|d|≫drop)
    # and select the level floor (|d|≈drop).
    floor = _flat_plane(0.55, n=2350, noise=0.003, seed=31)
    tilted = _tilted_fragment(abs_d=0.84, drop=0.42, pitch_deg=12.0,
                              n=1200, noise=0.003, seed=32)
    pts = np.vstack((floor, tilted))
    result = ransac_fit_plane(
        pts,
        n_iterations=600,
        distance_thresh=0.03,
        rng=np.random.default_rng(7),
        vertical_axis=np.array([0.0, 1.0, 0.0]),
        vertical_tol_rad=math.radians(15.0),
        prefer_farthest=True,
        ground_min_inlier_frac=0.5,
        ground_extrapolation_tol=0.10,
    )
    assert result is not None
    _, d, _ = result
    # Must select the level floor (~0.55 m), NOT the tilted fragment's inflated
    # |d| (~0.84 m).  The 0.749 m-class over-shoot must never be returned.
    assert abs(d) == pytest.approx(0.55, abs=0.05)
    assert abs(d) < 0.70


def test_most_inliers_default_picks_nearer_denser_plane():
    # Sanity check that WITHOUT the prior the classic behaviour selects the
    # nearer denser surface (this is the documented bug the fix corrects).
    pts, _ = _two_horizontal_planes(near_h=0.08, far_h=0.57)
    result = ransac_fit_plane(
        pts, n_iterations=400, distance_thresh=0.02,
        rng=np.random.default_rng(0),
    )
    assert result is not None
    _, d, _ = result
    assert abs(d) == pytest.approx(0.08, abs=0.03)


def test_most_inliers_gate_is_noop_without_vertical_constraint():
    # The extrapolation-consistency gate added to the most-inliers branch must
    # be a NO-OP when the vertical constraint is OFF: with no vertical axis the
    # recorded depth == |d|, so |d|-drop == 0 <= tol for every candidate and the
    # densest plane still wins exactly as before (backward compatibility for the
    # plain most-inliers path).  (LEFT-radar tune; req 2.2/2.3)
    pts, _ = _two_horizontal_planes(near_h=0.08, far_h=0.57)
    result = ransac_fit_plane(
        pts, n_iterations=400, distance_thresh=0.02,
        rng=np.random.default_rng(0),
        ground_extrapolation_tol=0.10,   # gate present but inert (no vert axis)
    )
    assert result is not None
    _, d, _ = result
    assert abs(d) == pytest.approx(0.08, abs=0.03)


def test_left_radar_most_inliers_gate_selects_level_floor_over_tilted_shard():
    # LEFT-radar ground-lock tune (auto_test/20260620_160536_ground_calib_left_
    # tune; req 2.2/2.3, Property 3).  The LEFT scene stacks a near surface
    # (~0.13 m, removed offline by ground_min_drop) and, below it, a DENSE level
    # true floor (~0.50 m) competing with a sparse TILTED shard whose |d| is
    # inflated by tilt-extrapolation (~0.63 m).  With prefer_farthest=False the
    # node uses the most-inliers branch; the NEW extrapolation-consistency gate
    # on that branch must reject the tilted shard (|d|≫drop) so the dense level
    # floor (|d|≈drop, more inliers) wins — landing in the tape band instead of
    # over-shooting.  Without the gate, the tilted shard could be picked when it
    # happens to gather more inliers, drifting |d| above the band.
    floor = _flat_plane(0.50, n=900, noise=0.003, seed=41)
    shard = _tilted_fragment(abs_d=0.63, drop=0.46, pitch_deg=14.0,
                             n=1100, noise=0.003, seed=42)
    pts = np.vstack((floor, shard))
    result = ransac_fit_plane(
        pts,
        n_iterations=600,
        distance_thresh=0.03,
        rng=np.random.default_rng(9),
        vertical_axis=np.array([0.0, 1.0, 0.0]),
        vertical_tol_rad=math.radians(15.0),
        prefer_farthest=False,            # LEFT config: most-inliers + gate
        ground_extrapolation_tol=0.05,    # LEFT config value
    )
    assert result is not None
    _, d, _ = result
    # Must select the level floor (~0.50 m), NOT the tilted shard's inflated
    # |d| (~0.63 m).  Landing in the tape band 0.48-0.55 m.
    assert abs(d) == pytest.approx(0.50, abs=0.05)
    assert abs(d) < 0.60


def test_vertical_constraint_rejects_wall_candidate():
    # A scene with a vertical wall (more points) plus a real floor: the
    # in-loop vertical constraint must reject the wall and return the floor.
    floor = _flat_plane(0.55, n=600, noise=0.002, seed=3)
    wall = _wall(n=1500, noise=0.002, seed=4)
    pts = np.vstack((floor, wall))
    result = ransac_fit_plane(
        pts,
        n_iterations=400,
        distance_thresh=0.02,
        rng=np.random.default_rng(1),
        vertical_axis=np.array([0.0, 1.0, 0.0]),
        vertical_tol_rad=math.radians(25.0),
        prefer_farthest=True,
    )
    assert result is not None
    normal, d, _ = result
    # Returned plane must be the near-vertical floor, not the wall.
    angle_to_y = math.degrees(math.acos(abs(float(np.dot(normal, [0.0, 1.0, 0.0])))))
    assert angle_to_y < 25.0
    assert abs(d) == pytest.approx(0.55, abs=0.03)


def test_wall_only_scene_returns_none_under_vertical_constraint():
    # Only a wall is visible: no near-vertical candidate exists, so the
    # constrained RANSAC must return None (caller keeps last valid TF, req 2.5).
    result = ransac_fit_plane(
        _wall(n=800),
        n_iterations=300,
        distance_thresh=0.02,
        rng=np.random.default_rng(2),
        vertical_axis=np.array([0.0, 1.0, 0.0]),
        vertical_tol_rad=math.radians(25.0),
        prefer_farthest=True,
    )
    assert result is None


def test_matrix_to_quaternion_identity_and_yaw90():
    qx, qy, qz, qw = matrix_to_quaternion(np.eye(3))
    assert (qx, qy, qz, qw) == pytest.approx((0.0, 0.0, 0.0, 1.0), abs=1e-9)
    # +90 deg about Z.
    rz = np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])
    qx, qy, qz, qw = matrix_to_quaternion(rz)
    assert (qx, qy, qz) == pytest.approx((0.0, 0.0, math.sqrt(0.5)), abs=1e-6)
    assert qw == pytest.approx(math.sqrt(0.5), abs=1e-6)


# ==========================================================================
# Node-level tests: near-vertical detection and low-inlier invalidation
# ==========================================================================

@pytest.fixture(scope="module")
def ros_init():
    rclpy.init()
    yield
    rclpy.shutdown()


class _CaptureBroadcaster:
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


def test_near_vertical_ground_accepted(ros_init):
    node = _make_node(_settle_frames=1)
    try:
        node._on_cloud(_make_cloud(_tilted_ground(height=0.45, pitch_deg=8.0)))
        payload = json.loads(node._diag_pub.last.data)
        assert payload["valid"] is True
        assert payload["height"] == pytest.approx(0.45, abs=0.05)
        assert payload["pitch_deg"] == pytest.approx(8.0, abs=2.0)
        assert payload["inlier_ratio"] >= node._min_inlier_ratio
    finally:
        node.destroy_node()


def test_wall_rejected_as_not_vertical(ros_init):
    node = _make_node(_settle_frames=1)
    try:
        node._on_cloud(_make_cloud(_wall()))
        payload = json.loads(node._diag_pub.last.data)
        # A wall's normal (~forward) is far from the sensor vertical axis. With
        # the Task-14 fix the vertical-normal constraint runs INSIDE the RANSAC
        # loop, so a wall yields no near-vertical candidate at all -> invalid,
        # no TF published (req 2.2/2.3, Property 3).
        assert payload["valid"] is False
        assert len(node._tf_broadcaster.sent) == 0
    finally:
        node.destroy_node()


def test_low_inlier_ratio_flagged_invalid(ros_init):
    node = _make_node(_settle_frames=1, _min_inlier_ratio=0.15)
    try:
        node._on_cloud(_make_cloud(_ground_with_outliers(height=0.45)))
        payload = json.loads(node._diag_pub.last.data)
        # Best plane is the small ground patch but its inlier ratio is below the
        # threshold -> calibration must be flagged invalid (req 2.5).
        assert payload["inlier_ratio"] < node._min_inlier_ratio
        assert payload["valid"] is False
        assert node._calibrated is False
        assert len(node._tf_broadcaster.sent) == 0
    finally:
        node.destroy_node()
