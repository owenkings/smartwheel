"""Ground-plane auto-calibration node for the XT-M60 ToF radar.

Subscribes to ``/xtm60/left/points`` (sensor_msgs/PointCloud2 in
``xtm60_left_link`` frame), runs a pure-numpy RANSAC to fit the dominant
ground plane, and derives the radar installation height (z) and pitch from the
fit result.

Sensor coordinate convention (verified on hardware):
  z = forward, y = up, x = left.
So the ground-plane normal in sensor frame should be close to ±y.

Outputs
-------
- TF ``base_link → xtm60_left_link`` via StaticTransformBroadcaster, owning the
  full transform: translation = (x_offset, y_offset, height) and rotation =
  yaw(param) ⊕ verified sensor-convention rotation ⊕ pitch(calibrated). This node
  is the SOLE owner of that TF edge — the URDF ``xtm60_left_fixed_joint`` was
  removed so there is no double-publish (design "方案 A").
- ``/calibration/ground_plane`` (std_msgs/String, JSON) with
  height / pitch_deg / inlier_ratio / valid on each calibration attempt.

Requirements: 2.1, 2.2, 2.3, 2.4, 2.7
"""
from __future__ import annotations

import json
import math
import time
from dataclasses import dataclass
from typing import Optional

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from tf2_ros import StaticTransformBroadcaster

from wheelchair_3d_mapping import cloud_utils


# ---------------------------------------------------------------------------
# Data model
# ---------------------------------------------------------------------------

@dataclass
class PlaneFitResult:
    """Result of a RANSAC + PCA ground-plane fit.

    Attributes
    ----------
    normal : np.ndarray, shape (3,)
        Unit normal vector of the fitted plane in the sensor frame.
    d : float
        Plane offset so that  normal · p + d = 0  for any inlier point p.
    height : float
        Perpendicular distance from the sensor origin to the plane: ``|d|``
        (valid because normal is already unit-length).
    pitch_rad : float
        Angle between the plane normal and the sensor vertical axis (y),
        i.e.  arccos(|normal · [0,1,0]|). Also the pitch correction needed.
    inlier_ratio : float
        Fraction of input points that lie within ``ransac_distance_thresh``
        of the fitted plane.  Range [0, 1].
    valid : bool
        True when ``inlier_ratio >= min_inlier_ratio`` AND
        ``pitch_rad < vertical_normal_tol_deg`` (in radians).
    """
    normal: np.ndarray
    d: float
    height: float
    pitch_rad: float
    inlier_ratio: float
    valid: bool


# ---------------------------------------------------------------------------
# Verified sensor → base orientation convention
# ---------------------------------------------------------------------------
# XT-M60 raw point convention VERIFIED on hardware (.diag_frame.py):
#   z = forward, y = UP, x = LEFT.
# The URDF used rpy=(pi/2, 0, pi/2) for xtm60_left_fixed_joint, whose rotation
# matrix maps a sensor-frame point p_s into base_link as:
#   x_base = z_sensor (forward), y_base = x_sensor (left), z_base = y_sensor (up).
# That fixed convention is reproduced here as R_CONV so the calibrator can OWN
# the full base_link→xtm60_left_link edge (URDF joint removed, design 方案 A).
#   R_CONV @ (x,y,z)_sensor = (z, x, y)_base
R_CONV = np.array(
    [
        [0.0, 0.0, 1.0],
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
    ]
)


def _rot_y(theta: float) -> np.ndarray:
    """Rotation matrix about the (base_link) Y axis — i.e. pitch."""
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def _rot_z(theta: float) -> np.ndarray:
    """Rotation matrix about the (base_link) Z axis — i.e. yaw."""
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def matrix_to_quaternion(m: np.ndarray):
    """Convert a 3x3 rotation matrix to a (x, y, z, w) quaternion.

    Uses the numerically stable branch method (Shepperd / Eigen style).
    """
    t = float(np.trace(m))
    if t > 0.0:
        s = math.sqrt(t + 1.0) * 2.0
        w = 0.25 * s
        x = (m[2, 1] - m[1, 2]) / s
        y = (m[0, 2] - m[2, 0]) / s
        z = (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
        w = (m[2, 1] - m[1, 2]) / s
        x = 0.25 * s
        y = (m[0, 1] + m[1, 0]) / s
        z = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
        w = (m[0, 2] - m[2, 0]) / s
        x = (m[0, 1] + m[1, 0]) / s
        y = 0.25 * s
        z = (m[1, 2] + m[2, 1]) / s
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
        w = (m[1, 0] - m[0, 1]) / s
        x = (m[0, 2] + m[2, 0]) / s
        y = (m[1, 2] + m[2, 1]) / s
        z = 0.25 * s
    q = np.array([x, y, z, w])
    n = np.linalg.norm(q)
    if n < 1e-12:
        return 0.0, 0.0, 0.0, 1.0
    q = q / n
    return float(q[0]), float(q[1]), float(q[2]), float(q[3])


def signed_pitch_correction(normal_sensor: np.ndarray) -> float:
    """Signed pitch (rotation about base_link Y) that levels the ground plane.

    The plane normal is first oriented "up" in the sensor frame (sensor +y),
    then expressed in base_link via R_CONV.  The pitch correction is the angle
    about base_link Y that drives the normal's X component to zero so the ground
    becomes horizontal (normal → +Z) in base_link.

    Returns the correction angle in radians.
    """
    n = np.asarray(normal_sensor, dtype=np.float64)
    norm = np.linalg.norm(n)
    if norm < 1e-12:
        return 0.0
    n = n / norm
    # Orient "up": sensor vertical axis is +y.
    if n[1] < 0.0:
        n = -n
    # Express in base_link:  R_CONV @ (x,y,z) = (z, x, y).
    nb = R_CONV @ n  # (nb_x, nb_y, nb_z) = (n_z, n_x, n_y)
    # Pitch about base Y to send nb_x → 0 while keeping nb_z > 0.
    return math.atan2(-nb[0], nb[2])


# ---------------------------------------------------------------------------
# Pure-numpy RANSAC plane fitting
# ---------------------------------------------------------------------------

def _fit_plane_3pts(p0: np.ndarray, p1: np.ndarray, p2: np.ndarray):
    """Fit a plane through three 3-D points.

    Returns (normal, d) where ``normal`` is a unit vector and d is the
    plane offset such that  normal · p + d = 0.
    Returns None if the three points are collinear (cross-product ≈ 0).
    """
    v1 = p1 - p0
    v2 = p2 - p0
    n = np.cross(v1, v2)
    norm = np.linalg.norm(n)
    if norm < 1e-9:
        return None
    n = n / norm
    d = -float(np.dot(n, p0))
    return n, d


def _pca_refine(pts: np.ndarray):
    """Refine a plane normal via PCA on the inlier point set.

    The plane normal is the right-singular vector corresponding to the
    *smallest* singular value (i.e. last column of Vt).

    Returns (normal, d) or None if not enough points.
    """
    if pts.shape[0] < 3:
        return None
    centroid = pts.mean(axis=0)
    centered = pts - centroid
    # SVD: U S Vt;  columns of V (rows of Vt) are principal directions.
    # The last row of Vt is the direction of *minimum* variance → plane normal.
    _, _, vt = np.linalg.svd(centered, full_matrices=False)
    n = vt[-1]  # last row
    n = n / np.linalg.norm(n)
    d = -float(np.dot(n, centroid))
    return n, d


def ransac_fit_plane(
    pts: np.ndarray,
    n_iterations: int = 100,
    distance_thresh: float = 0.03,
    rng: Optional[np.random.Generator] = None,
    vertical_axis: Optional[np.ndarray] = None,
    vertical_tol_rad: Optional[float] = None,
    prefer_farthest: bool = False,
    ground_min_inlier_frac: float = 0.5,
    ground_extrapolation_tol: float = 0.10,
) -> Optional[tuple]:
    """Fit a plane to *pts* (N, 3) using RANSAC + PCA refinement.

    Parameters
    ----------
    pts : np.ndarray, shape (N, 3)
        Input point cloud (NaN-free).
    n_iterations : int
        Number of RANSAC trials.
    distance_thresh : float
        Inlier distance threshold (metres).
    rng : np.random.Generator, optional
        Random number generator (for reproducibility in tests).
    vertical_axis : np.ndarray, shape (3,), optional
        Sensor vertical axis (e.g. ``[0, 1, 0]`` for this radar where y = up).
        When supplied together with ``vertical_tol_rad``, a candidate plane is
        only accepted if its normal lies within ``vertical_tol_rad`` of this
        axis.  This rejects walls / tilted surfaces DURING selection rather than
        after, which is the core of the Task-14 / Property-3 fix (req 2.2/2.3):
        the old "most inliers wins" selection locked onto a near-horizontal
        seat/desk/platform (~8 cm below the radar, more points) instead of the
        true floor (~0.57 m below, fewer points).
    vertical_tol_rad : float, optional
        Maximum angle (radians) between a candidate normal and ``vertical_axis``
        for the candidate to be accepted.  Ignored unless ``vertical_axis`` is
        given.
    prefer_farthest : bool
        When True, among all accepted near-vertical candidates that carry enough
        inliers (see ``ground_min_inlier_frac``) the plane with the LARGEST
        ``|d|`` (farthest *down* from the sensor origin) is chosen instead of the
        one with the most inliers.  This is the "ground = lowest horizontal
        plane" prior (Task-14 fix, req 2.2/2.3, Property 3).  When False the
        classic "most inliers" plane is returned (backward compatible).
    ground_min_inlier_frac : float
        Eligibility floor for the farthest-down prior, expressed as a fraction
        of the densest near-vertical candidate's inlier count.  A candidate is
        only allowed to win on distance if its inlier count is at least
        ``ground_min_inlier_frac * best_vertical_count``.  This keeps the choice
        principled — a sparse noise plane far below cannot win — while still
        letting a real floor with fewer points than a nearer surface be picked.
    ground_extrapolation_tol : float
        Extrapolation-consistency gate (metres) for the farthest-down prior
        (Task-14 RIGHT-radar over-shoot fix, req 2.2/2.3, Property 3).  A
        candidate may only win on ``|d|`` if that offset is consistent with the
        ACTUAL vertical depth of its own inliers:
        ``|abs_d - drop| <= ground_extrapolation_tol`` where
        ``drop = |median(inliers · vertical_axis)|``.  A LEVEL far floor has
        ``|d| ≈ drop`` (passes), whereas a sparse TILTED fragment inflates ``|d|``
        by tilt-extrapolation (``|d|`` ≫ ``drop``) and is rejected.  This stops
        the loose vertical tolerance from selecting a tilted near-vertical
        fragment whose extrapolated |d| over-shoots the true radar height (the
        right-radar 0.749 m vs tape-measured 0.50–0.60 m bug).  If the gate
        empties the eligible set it falls back to the eligibility set (no crash).
        Only used in the ``prefer_farthest`` branch.

    Returns
    -------
    (normal, d, inlier_mask) or None if fitting fails.
    """
    N = pts.shape[0]
    if N < 3:
        return None

    if rng is None:
        rng = np.random.default_rng()

    constrain_vertical = vertical_axis is not None and vertical_tol_rad is not None
    if constrain_vertical:
        axis = np.asarray(vertical_axis, dtype=np.float64)
        axis = axis / np.linalg.norm(axis)
        cos_tol = math.cos(float(vertical_tol_rad))

    # Collected candidates: (inlier_count, |d|, normal, d, drop).  We keep only
    # scalar summaries (not the full N-length masks) to stay memory-light; the
    # mask for the winning plane is recomputed once below.  ``drop`` is the
    # ACTUAL vertical depth of the candidate's inliers along the vertical axis,
    # used by the extrapolation-consistency gate (Task-14 right-radar fix).
    candidates = []  # list of (count:int, abs_d:float, n:np.ndarray, d:float, drop:float)
    best_count = 0

    for _ in range(n_iterations):
        # Random sample of 3 distinct indices
        idx = rng.choice(N, size=3, replace=False)
        result = _fit_plane_3pts(pts[idx[0]], pts[idx[1]], pts[idx[2]])
        if result is None:
            continue
        n, d = result

        # Vertical-normal constraint applied INSIDE the RANSAC loop so that
        # walls / tilted surfaces are never even considered (req 2.2/2.3).
        if constrain_vertical:
            # |n·axis| because the plane normal sign is arbitrary.
            if abs(float(np.dot(n, axis))) < cos_tol:
                continue

        # Count inliers
        dist = np.abs(pts @ n + d)
        mask = dist < distance_thresh
        count = int(mask.sum())
        if count < 3:
            continue
        # Record the inliers' ACTUAL vertical depth (|median projection onto the
        # vertical axis|).  For a level plane this ≈ |d|; for a tilt-extrapolated
        # fragment |d| over-shoots this (Task-14 right-radar over-shoot fix).
        if constrain_vertical:
            drop = abs(float(np.median(pts[mask] @ axis)))
        else:
            drop = abs(float(d))
        candidates.append((count, abs(float(d)), n, float(d), drop))
        if count > best_count:
            best_count = count

    if not candidates:
        return None

    if prefer_farthest:
        # "Ground = lowest / farthest-down horizontal plane" prior.  Among the
        # near-vertical candidates that carry enough inliers, pick the one whose
        # plane sits farthest below the sensor (largest |d|).  The inlier floor
        # stops a sparse noise plane from winning purely on distance.
        count_floor = max(3, int(math.ceil(ground_min_inlier_frac * best_count)))
        eligible = [c for c in candidates if c[0] >= count_floor]
        if not eligible:
            eligible = candidates
        # Extrapolation-consistency gate (Task-14 right-radar over-shoot fix,
        # req 2.2/2.3, Property 3): keep only candidates whose |d| is consistent
        # with their inliers' actual vertical depth.  This rejects sparse TILTED
        # fragments (|d| inflated by tilt-extrapolation, e.g. |d|≈0.84 m vs drop
        # ≈0.42 m, ~19° tilt) while keeping LEVEL far floors (|d|≈drop).  Falls
        # back to the eligibility set if the gate empties it (never crash).
        consistent = [
            c for c in eligible if abs(c[1] - c[4]) <= ground_extrapolation_tol
        ]
        if consistent:
            eligible = consistent
        chosen = max(eligible, key=lambda c: c[1])  # largest |d|
    else:
        # Most-inliers selection, but FIRST restricted to candidates whose |d|
        # is consistent with their inliers' actual vertical depth — the SAME
        # extrapolation-consistency gate used by the farthest-down branch
        # (Task-tune LEFT-radar over-shoot fix, req 2.2/2.3, Property 3).
        #   - When the vertical constraint is OFF, drop == |d| (see the loop
        #     above), so abs(c[1]-c[4]) == 0 <= tol always: the gate is a NO-OP
        #     and this is identical to classic most-inliers (backward compatible
        #     with test_most_inliers_default_picks_nearer_denser_plane).
        #   - When the vertical constraint is ON (the calibrator's ground mode),
        #     this rejects TILTED fragments whose |d| is inflated/deflated by
        #     tilt-extrapolation, so the densest LEVEL plane (the true floor)
        #     wins instead of a tilted shard. Combined with ground_min_drop
        #     (which excludes the near seat/desk surface so inlier_ratio is
        #     computed against near-floor points) this lands the LEFT radar in
        #     its tape-measured 0.48-0.55 m band, where the farthest-down prior
        #     over-shot to ~0.63 m (a sparse lower cluster).
        eligible = [
            c for c in candidates if abs(c[1] - c[4]) <= ground_extrapolation_tol
        ]
        if not eligible:
            eligible = candidates
        chosen = max(eligible, key=lambda c: c[0])

    n_sel, d_sel = chosen[2], chosen[3]

    # PCA refinement on the chosen plane's inlier set
    sel_dist = np.abs(pts @ n_sel + d_sel)
    sel_mask = sel_dist < distance_thresh
    inlier_pts = pts[sel_mask]
    refined = _pca_refine(inlier_pts)
    if refined is None:
        return None
    n_ref, d_ref = refined

    # Recompute inlier mask with refined normal
    dist_ref = np.abs(pts @ n_ref + d_ref)
    final_mask = dist_ref < distance_thresh
    return n_ref, d_ref, final_mask


# ---------------------------------------------------------------------------
# ROS 2 node
# ---------------------------------------------------------------------------

class GroundPlaneCalibratorNode(Node):
    """Calibrate radar height and pitch by RANSAC ground-plane fitting."""

    def __init__(self, **kwargs):
        super().__init__("ground_plane_calibrator", **kwargs)

        # --- Parameters -------------------------------------------------------
        self.declare_parameter("input_topic", "/xtm60/left/points")
        self.declare_parameter("target_frame", "base_link")
        self.declare_parameter("radar_frame", "xtm60_left_link")
        # x/y/yaw are the fixed geometric part formerly carried by the URDF
        # xtm60_left_fixed_joint (xyz=[0.45, 0.24, *], yaw=0).  z (height) and
        # pitch are now calibrated from the ground plane (req 2.2/2.3/2.7).
        self.declare_parameter("x_offset", 0.45)
        self.declare_parameter("y_offset", 0.24)
        self.declare_parameter("yaw", 0.0)
        # --- Fixed / manual-override mode (USER-DECIDED) ----------------------
        # USER-DECIDED MANUAL OVERRIDE of req 2.7 (auto height/pitch calibration).
        # The LEFT-radar auto ground-plane calibration never locks in the target
        # environment (sparse/noisy floor + a ~0.13 m near surface; 0/240 samples
        # valid). Because it never locks, this node never publishes the
        # base_link→xtm60_left_link TF, the fusion node skips every frame
        # (D178 fail-safe: no TF → skip), /points_merged stays empty and RViz
        # shows no point cloud. The user has decided to FIX the LEFT radar height
        # and DISABLE auto height tuning. This is an INTENTIONAL, user-approved
        # deviation from req 2.7 (documented as a manual override, NOT a silent
        # requirements change).
        #
        # When ``fixed_height > 0.0`` the node SKIPS RANSAC entirely and publishes
        # base_link→xtm60_left_link ONCE at startup (so the TF exists even before/
        # without any radar data), unconditionally restoring the point cloud.
        # ``fixed_height == 0.0`` (the default) leaves the full auto-cal path
        # untouched — used by the right radar and any auto-cal deployment.
        self.declare_parameter("fixed_height", 0.0)
        self.declare_parameter("fixed_pitch_deg", 0.0)
        self.declare_parameter("ransac_distance_thresh", 0.03)
        # Task-14 RIGHT-radar over-shoot fix (req 2.2/2.3, Property 3): raise the
        # default from 100 to 400 iterations.  Offline sweep on the right-radar
        # raw frames (auto_test/.../iter_sensitivity.py) showed 100 iters
        # under-samples the dense LEVEL floor — single-frame |d| median 0.692 m
        # (14/50 in 0.50–0.60 band) — so a noisy frame could still lock a bad
        # value even WITH the extrapolation gate.  400 iters → median 0.537 m,
        # 45/50 in band, making the single-frame settle-lock reliable.
        self.declare_parameter("ransac_iterations", 400)
        # Task-14 / Property-3 fix (req 2.2/2.3): the inlier ratio is now
        # computed relative to the VERTICAL-CONSTRAINED candidate subset (only
        # near-horizontal points participate), so the ratio is meaningful again
        # and the floor can be raised.  0.30 = the chosen ground plane must hold
        # at least 30% of the near-horizontal points it competes within.
        self.declare_parameter("min_inlier_ratio", 0.30)
        # Task-14 RIGHT-radar over-shoot fix (req 2.2/2.3, Property 3): tighten
        # the near-vertical tolerance from 25° to 15°.  The loose 25° band let a
        # sparse TILTED (~19°) fragment qualify as "near-vertical"; its
        # extrapolated |d|≈0.84 m over-shot the true radar height (tape-measured
        # 0.50–0.60 m, reported 0.749 m).  15° still admits real floor tilt
        # (6–8°) while excluding the tilted fragment.
        self.declare_parameter("vertical_normal_tol_deg", 15.0)
        self.declare_parameter("recalibrate_period_sec", 0.0)
        self.declare_parameter("settle_frames", 5)
        # Task-14 / Property-3 fix (req 2.2/2.3): prefer the lowest/farthest-down
        # near-vertical plane (the true floor) over the one with the most points
        # (often a nearer seat/desk/platform).  Eligibility floor keeps the
        # choice principled (no sparse noise plane wins on distance alone).
        self.declare_parameter("prefer_farthest_ground", True)
        self.declare_parameter("ground_min_inlier_frac", 0.5)
        # Task-14 RIGHT-radar over-shoot fix (req 2.2/2.3, Property 3):
        # extrapolation-consistency gate.  In the farthest-down branch a
        # candidate may only win on |d| if |d| matches its inliers' actual
        # vertical depth within this tolerance (metres).  Rejects tilt-
        # extrapolated fragments (|d|≈0.84 m vs actual depth ≈0.42 m) while
        # keeping level far floors.  0.10 validated on the right-radar raw frames
        # (auto_test/.../consistency_test.py: |d| med 0.513 m, in band).
        self.declare_parameter("ground_extrapolation_tol", 0.10)
        # Optional: restrict the ground fit to points at least this far below the
        # sensor along the vertical axis (sensor -y).  0.0 disables the subset
        # restriction (the vertical-constraint + farthest-down prior is the
        # primary fix; this is an additional guard for cluttered scenes).
        self.declare_parameter("ground_min_drop", 0.0)

        self._input_topic = self.get_parameter("input_topic").value
        self._target_frame = self.get_parameter("target_frame").value
        self._radar_frame = self.get_parameter("radar_frame").value
        self._x_offset = float(self.get_parameter("x_offset").value)
        self._y_offset = float(self.get_parameter("y_offset").value)
        self._yaw = float(self.get_parameter("yaw").value)
        # Fixed / manual-override mode (user-decided override of req 2.7).
        self._fixed_height = float(self.get_parameter("fixed_height").value)
        self._fixed_pitch_rad = math.radians(
            float(self.get_parameter("fixed_pitch_deg").value)
        )
        self._fixed_mode = self._fixed_height > 0.0
        self._ransac_thresh = float(self.get_parameter("ransac_distance_thresh").value)
        self._ransac_iters = int(self.get_parameter("ransac_iterations").value)
        self._min_inlier_ratio = float(self.get_parameter("min_inlier_ratio").value)
        self._vert_tol_rad = math.radians(
            float(self.get_parameter("vertical_normal_tol_deg").value)
        )
        self._recalib_period = float(self.get_parameter("recalibrate_period_sec").value)
        self._settle_frames = int(self.get_parameter("settle_frames").value)
        self._prefer_farthest = bool(self.get_parameter("prefer_farthest_ground").value)
        self._ground_min_inlier_frac = float(
            self.get_parameter("ground_min_inlier_frac").value
        )
        self._ground_extrapolation_tol = float(
            self.get_parameter("ground_extrapolation_tol").value
        )
        self._ground_min_drop = float(self.get_parameter("ground_min_drop").value)

        # --- Internal state ---------------------------------------------------
        self._frame_count = 0           # frames received so far
        self._calibrated = False        # True once first valid result locked
        self._last_valid: Optional[PlaneFitResult] = None
        self._last_tf: Optional[TransformStamped] = None  # last valid TF published
        self._last_calib_time = 0.0     # wall-clock time of last calibration
        self._warn_log: dict = {}       # key → last warning time (throttle)

        # --- TF broadcaster ---------------------------------------------------
        # Task 9 will complete the full transform computation.  Here we create
        # the broadcaster so the stub _publish_tf can use it.
        self._tf_broadcaster = StaticTransformBroadcaster(self)

        # --- Publishers -------------------------------------------------------
        self._diag_pub = self.create_publisher(
            String, "/calibration/ground_plane", 10
        )

        # --- Fixed / manual-override mode short-circuit -----------------------
        # USER-DECIDED MANUAL OVERRIDE of req 2.7. When a fixed height is set we
        # do NOT run RANSAC at all: no per-frame lock path, hence no lock-failure
        # / missing-TF path. We publish the static TF + a "fixed" diagnostic ONCE
        # at startup, regardless of whether any cloud has arrived, so the
        # base_link→xtm60_left_link edge exists even before/without radar data
        # (this is what unconditionally restores the /points_merged point cloud).
        # We also skip subscribing to the cloud so RANSAC can never run.
        if self._fixed_mode:
            self.get_logger().warning(
                f"[ground_plane_calibrator] FIXED-HEIGHT MODE (user-decided manual "
                f"override of req 2.7): RANSAC DISABLED. Publishing static TF "
                f"{self._target_frame}->{self._radar_frame} at "
                f"xyz=({self._x_offset:.3f},{self._y_offset:.3f},{self._fixed_height:.3f}) "
                f"fixed_pitch={math.degrees(self._fixed_pitch_rad):+.2f}° yaw={math.degrees(self._yaw):+.2f}°"
            )
            # One-shot timer publishes the static TF + "fixed" diagnostic right
            # after startup (regardless of whether any cloud has arrived). A
            # latched StaticTransformBroadcaster keeps the edge alive thereafter.
            # We do NOT subscribe to the cloud in fixed mode, so RANSAC / the
            # per-frame lock path can never run.
            self._fixed_timer = self.create_timer(0.0, self._fixed_publish_once)
            return

        # --- Subscriber (auto-cal path only) ----------------------------------
        self.create_subscription(
            PointCloud2,
            self._input_topic,
            self._on_cloud,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            f"ground_plane_calibrator: listening on '{self._input_topic}' "
            f"target_frame='{self._target_frame}' radar_frame='{self._radar_frame}' "
            f"ransac_thresh={self._ransac_thresh} iters={self._ransac_iters} "
            f"min_inlier_ratio={self._min_inlier_ratio} "
            f"vert_tol_deg={math.degrees(self._vert_tol_rad):.1f} "
            f"settle_frames={self._settle_frames} "
            f"recalib_period={self._recalib_period}s"
        )

    # ------------------------------------------------------------------
    # Subscription callback
    # ------------------------------------------------------------------

    def _on_cloud(self, msg: PointCloud2):
        self._frame_count += 1

        # --- Decide whether to attempt calibration this frame ------------------
        # recalibrate_period_sec == 0: calibrate once, using first settle_frames
        # frames; lock thereafter.
        # recalibrate_period_sec > 0: re-calibrate periodically (every N sec).
        now = time.monotonic()
        if self._recalib_period <= 0.0:
            if self._calibrated:
                return  # already locked
            if self._frame_count < self._settle_frames:
                self.get_logger().debug(
                    f"settling ({self._frame_count}/{self._settle_frames})"
                )
                return
        else:
            if (now - self._last_calib_time) < self._recalib_period:
                return

        # --- Extract XYZ from PointCloud2 -------------------------------------
        xyz, _ = cloud_utils.read_xyz_intensity(msg)
        if xyz.shape[0] < 3:
            self._warn(
                "too_few_pts",
                f"[ground_plane_calibrator] too few points ({xyz.shape[0]}) for RANSAC",
            )
            return

        # Sensor convention: y = up → ground lies BELOW the sensor (y < 0) and a
        # ground-plane normal is ≈ ±y.
        y_axis = np.array([0.0, 1.0, 0.0])

        # --- Optional ground subset restriction (Task-14 fix, req 2.2/2.3) ----
        # Additionally / optionally restrict the fit to points at least
        # ``ground_min_drop`` below the sensor.  This is a guard for cluttered
        # scenes; the vertical-constraint + farthest-down prior below is the
        # primary fix and works with ground_min_drop == 0.
        fit_pts = xyz
        if self._ground_min_drop > 0.0:
            below = xyz @ y_axis < -self._ground_min_drop
            if int(below.sum()) >= 3:
                fit_pts = xyz[below]

        # --- RANSAC fit -------------------------------------------------------
        # Task-14 / Property-3 fix (req 2.2/2.3): constrain candidate planes to
        # near-vertical normals INSIDE the RANSAC loop (rejects walls/tilted
        # surfaces during selection), and prefer the lowest/farthest-down plane
        # (the true floor) over the densest one (often a nearer seat/desk).
        result = ransac_fit_plane(
            fit_pts,
            n_iterations=self._ransac_iters,
            distance_thresh=self._ransac_thresh,
            vertical_axis=y_axis,
            vertical_tol_rad=self._vert_tol_rad,
            prefer_farthest=self._prefer_farthest,
            ground_min_inlier_frac=self._ground_min_inlier_frac,
            ground_extrapolation_tol=self._ground_extrapolation_tol,
        )
        if result is None:
            # No near-vertical candidate found — e.g. only a wall is visible, or
            # the ground is out of view.  req 2.5 / Property 5: retain the last
            # valid transform (never drop or invalidate good extrinsics) and
            # throttle-warn.
            if self._last_valid is not None and self._last_tf is not None:
                self._last_tf.header.stamp = self.get_clock().now().to_msg()
                self._tf_broadcaster.sendTransform(self._last_tf)
            self._warn(
                "ransac_fail",
                "[ground_plane_calibrator] RANSAC failed to find any near-vertical "
                "plane (no ground visible? wall-only view?)",
            )
            self._publish_invalid()
            return

        n_vec, d_val, inlier_mask = result

        # --- Inlier ratio relative to the candidate subset (Task-14 fix) ------
        # The old ratio (inliers / N over the WHOLE cloud) was dominated by
        # walls/clutter so min_inlier_ratio had no discriminating power (every
        # plane scored ~0.16–0.18).  Compute the ratio against the "could-be
        # ground" subset — points BELOW the sensor (y < 0) — so it actually
        # measures how dominant the chosen plane is among ground candidates
        # (req 2.2/2.3, report problem #1.3).  Numerator counts only inliers that
        # are themselves below the sensor, keeping the ratio in [0, 1].
        below_mask = fit_pts @ y_axis < 0.0
        denom = int(below_mask.sum())
        if denom < 3:
            denom = fit_pts.shape[0]
            inlier_count = int(inlier_mask.sum())
        else:
            inlier_count = int((inlier_mask & below_mask).sum())
        inlier_ratio = float(inlier_count) / float(denom)

        # --- Ground check: normal should be close to sensor +y/−y axis --------
        # Sensor convention: y = up → ground normal ≈ ±y → angle with y < tol.
        # The candidate loop already enforces this, but PCA refinement can shift
        # the normal slightly, so we re-check here as a safety net.
        cos_angle = float(np.clip(abs(np.dot(n_vec, y_axis)), 0.0, 1.0))
        pitch_rad = float(np.arccos(cos_angle))

        is_vertical = pitch_rad < self._vert_tol_rad
        is_enough_inliers = inlier_ratio >= self._min_inlier_ratio
        valid = is_vertical and is_enough_inliers

        height = abs(d_val)  # sensor origin → ground perpendicular distance

        fit = PlaneFitResult(
            normal=n_vec,
            d=d_val,
            height=height,
            pitch_rad=pitch_rad,
            inlier_ratio=inlier_ratio,
            valid=valid,
        )

        # --- Handle result ----------------------------------------------------
        if valid:
            self._last_valid = fit
            self._calibrated = True
            self._last_calib_time = now
            self.get_logger().info(
                f"[ground_plane_calibrator] valid calibration: "
                f"height={height:.4f}m pitch={math.degrees(pitch_rad):.2f}° "
                f"inlier_ratio={inlier_ratio:.3f}"
            )
            self._publish_tf(fit)
        else:
            if self._last_valid is not None:
                # req 2.5: NEVER publish invalid extrinsics. Keep the last valid
                # transform live. StaticTransformBroadcaster latches, but in
                # periodic recalibration mode we explicitly re-broadcast the
                # retained TF so it is never overwritten by an invalid fit.
                if self._last_tf is not None:
                    self._last_tf.header.stamp = self.get_clock().now().to_msg()
                    self._tf_broadcaster.sendTransform(self._last_tf)
                self._warn(
                    "invalid_fit",
                    f"[ground_plane_calibrator] invalid fit "
                    f"(inlier_ratio={inlier_ratio:.3f} valid={is_enough_inliers}, "
                    f"pitch={math.degrees(pitch_rad):.1f}° vertical={is_vertical}); "
                    f"retaining last valid transform",
                )
            else:
                self._warn(
                    "no_valid_yet",
                    f"[ground_plane_calibrator] no valid calibration yet "
                    f"(inlier_ratio={inlier_ratio:.3f}, pitch={math.degrees(pitch_rad):.1f}°)",
                )

        # Always publish diagnostic JSON
        self._publish_diag(fit)

    # ------------------------------------------------------------------
    # TF computation and broadcast (single owner of base_link→radar edge)
    # ------------------------------------------------------------------

    def _publish_tf(self, result: PlaneFitResult):
        """Publish the full ``base_link → xtm60_left_link`` transform.

        translation = (x_offset, y_offset, height)   # x/y from params, z=calibrated
        rotation    = R_z(yaw) ⊕ R_y(pitch) ⊕ R_CONV  # yaw(param) · pitch(calib) · verified convention

        This node is the sole owner of this TF edge (URDF fixed joint removed),
        so there is no double-publish (design 方案 A).
        """
        pitch_corr = signed_pitch_correction(result.normal)
        rot = _rot_z(self._yaw) @ _rot_y(pitch_corr) @ R_CONV
        qx, qy, qz, qw = matrix_to_quaternion(rot)

        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = self._target_frame
        tf.child_frame_id = self._radar_frame
        tf.transform.translation.x = self._x_offset
        tf.transform.translation.y = self._y_offset
        tf.transform.translation.z = float(result.height)
        tf.transform.rotation.x = qx
        tf.transform.rotation.y = qy
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw

        self._tf_broadcaster.sendTransform(tf)
        # Retain the last valid TF so we can keep it live during invalid fits
        # (req 2.5 failure fallback, esp. for periodic recalibration mode).
        self._last_tf = tf

        self.get_logger().info(
            f"[ground_plane_calibrator] published TF "
            f"{self._target_frame}->{self._radar_frame}: "
            f"xyz=({self._x_offset:.3f},{self._y_offset:.3f},{result.height:.4f}) "
            f"pitch_corr={math.degrees(pitch_corr):+.2f}° "
            f"quat=({qx:.4f},{qy:.4f},{qz:.4f},{qw:.4f}) "
            f"inlier_ratio={result.inlier_ratio:.3f}"
        )

    # ------------------------------------------------------------------
    # Fixed / manual-override mode (USER-DECIDED override of req 2.7)
    # ------------------------------------------------------------------

    def _fixed_publish_once(self):
        """One-shot timer callback: publish the fixed TF + diagnostic once.

        Cancels its own timer so it fires exactly once at startup.
        """
        timer = getattr(self, "_fixed_timer", None)
        if timer is not None:
            timer.cancel()
        self._publish_fixed_tf()
        self._publish_fixed_diag()

    def _publish_fixed_tf(self):
        """Publish the static ``base_link → xtm60_left_link`` TF in fixed mode.

        USER-DECIDED MANUAL OVERRIDE of req 2.7. No RANSAC is run. The transform
        uses the user-supplied fixed height and pitch:

            translation = (x_offset, y_offset, fixed_height)
            rotation    = R_z(yaw) · R_y(fixed_pitch) · R_CONV

        (same R_CONV / _rot_y / _rot_z / matrix_to_quaternion as the auto path).
        This node remains the SOLE owner of that TF edge (design 方案 A); the
        URDF joint is NOT re-added. Published once at startup so the edge exists
        even before/without radar data, unconditionally restoring /points_merged.
        """
        rot = _rot_z(self._yaw) @ _rot_y(self._fixed_pitch_rad) @ R_CONV
        qx, qy, qz, qw = matrix_to_quaternion(rot)

        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = self._target_frame
        tf.child_frame_id = self._radar_frame
        tf.transform.translation.x = self._x_offset
        tf.transform.translation.y = self._y_offset
        tf.transform.translation.z = float(self._fixed_height)
        tf.transform.rotation.x = qx
        tf.transform.rotation.y = qy
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw

        self._tf_broadcaster.sendTransform(tf)
        self._last_tf = tf
        self._calibrated = True  # nothing more to do; TF is locked

        self.get_logger().info(
            f"[ground_plane_calibrator] FIXED TF "
            f"{self._target_frame}->{self._radar_frame}: "
            f"xyz=({self._x_offset:.3f},{self._y_offset:.3f},{self._fixed_height:.4f}) "
            f"fixed_pitch={math.degrees(self._fixed_pitch_rad):+.2f}° "
            f"quat=({qx:.4f},{qy:.4f},{qz:.4f},{qw:.4f})"
        )

    def _publish_fixed_diag(self):
        """Publish the one-shot ``/calibration/ground_plane`` JSON in fixed mode.

        Reports the fixed height/pitch with ``mode="fixed"``, ``valid=true`` and
        ``inlier_ratio=null`` (no RANSAC was run). USER-DECIDED override of
        req 2.7.
        """
        payload = {
            "height": round(self._fixed_height, 4),
            "pitch_deg": round(math.degrees(self._fixed_pitch_rad), 3),
            "inlier_ratio": None,
            "mode": "fixed",
            "valid": True,
        }
        self._diag_pub.publish(String(data=json.dumps(payload)))

    # ------------------------------------------------------------------
    # Diagnostic helpers
    # ------------------------------------------------------------------

    def _publish_diag(self, result: PlaneFitResult):
        payload = {
            "height": round(result.height, 4),
            "pitch_deg": round(math.degrees(result.pitch_rad), 3),
            "inlier_ratio": round(result.inlier_ratio, 4),
            "valid": bool(result.valid),
        }
        self._diag_pub.publish(String(data=json.dumps(payload)))

    def _publish_invalid(self):
        """Publish an invalid/zero diagnostic when RANSAC fails outright."""
        payload = {
            "height": 0.0,
            "pitch_deg": 0.0,
            "inlier_ratio": 0.0,
            "valid": False,
        }
        self._diag_pub.publish(String(data=json.dumps(payload)))

    def _warn(self, key: str, msg: str, period: float = 5.0):
        """Throttled logger.warning to avoid log spam."""
        now = time.monotonic()
        if now - self._warn_log.get(key, 0.0) >= period:
            self.get_logger().warning(msg)
            self._warn_log[key] = now


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = GroundPlaneCalibratorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
