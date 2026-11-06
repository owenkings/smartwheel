"""Bounded scan-to-map correction of an odometry-predicted sensor pose.

Point-to-line ICP of the scan endpoints against occupied cells of the map
being built, regularized by a Gaussian prior on the odometry prediction
(Gauss-Newton on the MAP objective). Correspondences beyond a shrinking
distance are dropped instead of scored low; in simulation the correlative
matcher of slam_toolbox, which scores every candidate pose, accumulated about
10 degrees of heading lag with the ~115 degree forward sensor.

Two safeguards came from simulation run 20261105T052035Z, where heading stayed
within about 0.2 degrees but translation slipped twice, by 0.46 m and 0.5 m,
while turning in place near corners of a sparse map:

* a correspondence is used only if the scan's local line normal agrees with
  the map line normal, so points on one wall cannot pull along another wall;
* the correction is applied only along directions the scan constrains
  (eigenvalues of the scan information above a floor). Other directions,
  such as along a corridor, keep the odometry prediction exactly.

A correction is applied only with enough correspondences, a small residual and
a bounded size; otherwise the prediction is kept and the reason counted.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass
import math

import numpy as np
from scipy.spatial import cKDTree

from ..geometry import _number

INTEGER_FIELDS = ("iterations", "min_correspondences", "normal_neighbors", "max_points", "scan_normal_window")


@dataclass(frozen=True)
class IcpConfig:
    prior_sigma_xy_m: float = 0.05
    prior_sigma_yaw_rad: float = math.radians(1.0)
    range_noise_m: float = 0.02
    correlation_points: float = 5.0   # endpoints on one wall are not independent
    max_correspondence_m: float = 0.20
    final_correspondence_m: float = 0.08
    huber_m: float = 0.03
    iterations: int = 12
    min_correspondences: int = 30
    max_rms_m: float = 0.04
    max_correction_xy_m: float = 0.15
    max_correction_yaw_rad: float = math.radians(3.0)
    normal_neighbors: int = 6
    max_line_flatness: float = 0.3
    min_normal_agreement: float = 0.82          # |cos| of the angle between scan and map normals
    scan_normal_window: int = 2                 # neighbours on each side along the scan
    scan_normal_max_gap_m: float = 0.25
    # Information floor per direction, in fully weighted points. With the
    # normal check removing cross-wall pulls, this only guards directions that
    # a handful of points constrain; coupled y/yaw directions of a single wall
    # seen over a short range spread are typically 20+ points.
    min_direction_points: float = 8.0
    max_points: int = 240

    def __post_init__(self):
        for name, value in asdict(self).items():
            if name in INTEGER_FIELDS:
                if type(value) is not int or value < (1 if name == "scan_normal_window" else 2):
                    raise ValueError(name + " must be a positive integer")
            else:
                object.__setattr__(self, name, _number(value, name, positive=True))
        if self.final_correspondence_m > self.max_correspondence_m:
            raise ValueError("final correspondence distance cannot exceed the initial one")
        if self.min_normal_agreement > 1.0:
            raise ValueError("min_normal_agreement is a cosine and cannot exceed 1")

    @property
    def point_weight(self):
        return 1.0 / (self.range_noise_m ** 2 * self.correlation_points)

    def as_dict(self):
        return asdict(self)


def _rotation(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s], [s, c]])


def map_points(grid, center_xy, radius_m):
    """Occupied cell centres of a LogOddsGrid within ``radius_m`` (square window)."""
    res = grid.resolution
    half = int(math.ceil(_number(radius_m, "radius", positive=True) / res))
    col = int(math.floor((center_xy[0] - grid.origin[0]) / res))
    row = int(math.floor((center_xy[1] - grid.origin[1]) / res))
    r0, r1 = max(0, row - half), min(grid.cells, row + half + 1)
    c0, c1 = max(0, col - half), min(grid.cells, col + half + 1)
    if r0 >= r1 or c0 >= c1:
        return np.zeros((0, 2))
    rows, cols = np.nonzero(grid.logodds[r0:r1, c0:c1] >= grid.occupied_at)
    return np.column_stack([grid.origin[0] + (cols + c0 + 0.5) * res, grid.origin[1] + (rows + r0 + 0.5) * res])


def scan_points(angles, ranges, range_min, range_max, max_points):
    """Finite endpoints in the sensor frame, in beam order, evenly thinned."""
    angles = np.asarray(angles, dtype=float)
    ranges = np.asarray(ranges, dtype=float)
    with np.errstate(invalid="ignore"):
        ok = np.isfinite(ranges) & (ranges >= range_min) & (ranges <= range_max)
    a, r = angles[ok], ranges[ok]
    if len(r) > max_points:
        keep = np.linspace(0, len(r) - 1, max_points).round().astype(int)
        a, r = a[keep], r[keep]
    return np.column_stack([r * np.cos(a), r * np.sin(a)])


def scan_normals(points, window, max_gap_m, max_flatness):
    """Line normals of beam-ordered scan points from short neighbourhoods; NaN if none."""
    n = len(points)
    normals = np.full((n, 2), np.nan)
    if n < 2 * window + 1:
        return normals
    index = np.clip(np.arange(n)[:, None] + np.arange(-window, window + 1)[None, :], 0, n - 1)
    neighbourhood = points[index]
    gaps = np.linalg.norm(np.diff(neighbourhood, axis=1), axis=2)
    interior = (np.arange(n) >= window) & (np.arange(n) < n - window)
    contiguous = interior & (gaps.max(axis=1) <= max_gap_m)
    local = neighbourhood - neighbourhood.mean(axis=1, keepdims=True)
    covariance = np.einsum("nki,nkj->nij", local, local) / neighbourhood.shape[1]
    values, vectors = np.linalg.eigh(covariance)
    line = values[:, 0] <= max_flatness * np.maximum(values[:, 1], 1e-12)
    good = contiguous & line
    normals[good] = vectors[good, :, 0]
    return normals


def correct_pose(map_xy, prior_pose, points_sensor, config=None):
    """Return a dict with ``status``, ``pose`` (corrected or prior) and diagnostics."""
    config = config or IcpConfig()
    prior = np.array([_number(v, "prior pose") for v in prior_pose])
    report = {"status": None, "pose": prior.tolist(), "delta": [0.0, 0.0, 0.0], "correspondences": 0,
              "rms_m": None, "scan_information_eigenvalues": None, "constrained_directions": 0}
    map_xy = np.asarray(map_xy, dtype=float).reshape(-1, 2)
    pts = np.asarray(points_sensor, dtype=float).reshape(-1, 2)
    if len(map_xy) < config.normal_neighbors * 2:
        report["status"] = "SKIPPED_EMPTY_MAP"
        return report
    if len(pts) < config.min_correspondences:
        report["status"] = "SKIPPED_FEW_POINTS"
        return report
    normals_sensor = scan_normals(pts, config.scan_normal_window, config.scan_normal_max_gap_m,
                                  config.max_line_flatness)
    usable = np.isfinite(normals_sensor[:, 0])
    pts, normals_sensor = pts[usable], normals_sensor[usable]
    if len(pts) < config.min_correspondences:
        report["status"] = "SKIPPED_FEW_LINE_POINTS"
        return report
    tree = cKDTree(map_xy)
    k = min(config.normal_neighbors, len(map_xy))
    weight = config.point_weight
    prior_info = np.diag([1.0 / config.prior_sigma_xy_m ** 2] * 2 + [1.0 / config.prior_sigma_yaw_rad ** 2])

    def linearize(state, threshold):
        rot = _rotation(state[2])
        world = pts @ rot.T + state[:2]
        distance, index = tree.query(world, distance_upper_bound=threshold)
        valid = np.isfinite(distance)
        if valid.sum() < config.min_correspondences:
            return None
        q = map_xy[index[valid]]
        _, neighbors = tree.query(q, k=k)
        local = map_xy[neighbors] - map_xy[neighbors].mean(axis=1, keepdims=True)
        covariance = np.einsum("nki,nkj->nij", local, local) / k
        values, vectors = np.linalg.eigh(covariance)
        normal = vectors[:, :, 0]
        scan_normal_world = normals_sensor[valid] @ rot.T
        agree = np.abs(np.einsum("ni,ni->n", normal, scan_normal_world)) >= config.min_normal_agreement
        keep = (values[:, 0] <= config.max_line_flatness * np.maximum(values[:, 1], 1e-12)) & agree
        if keep.sum() < config.min_correspondences:
            return None
        normal = normal[keep]
        p_world, q, p_sensor = world[valid][keep], q[keep], pts[valid][keep]
        residual = np.einsum("ni,ni->n", normal, p_world - q)
        c, s = math.cos(state[2]), math.sin(state[2])
        d_rot = np.column_stack([-s * p_sensor[:, 0] - c * p_sensor[:, 1], c * p_sensor[:, 0] - s * p_sensor[:, 1]])
        jacobian = np.column_stack([normal, np.einsum("ni,ni->n", normal, d_rot)])
        absolute = np.abs(residual)
        huber = np.where(absolute <= config.huber_m, 1.0, config.huber_m / np.maximum(absolute, 1e-12))
        return residual, jacobian, huber

    x = prior.copy()
    for iteration in range(config.iterations):
        fraction = iteration / max(1, config.iterations - 1)
        threshold = config.max_correspondence_m + fraction * (config.final_correspondence_m - config.max_correspondence_m)
        linear = linearize(x, threshold)
        if linear is None:
            report["status"] = "REJECTED_FEW_CORRESPONDENCES"
            return report
        residual, jacobian, huber = linear
        w = weight * huber
        hessian = jacobian.T @ (jacobian * w[:, None]) + prior_info
        error = x - prior
        error[2] = math.atan2(math.sin(error[2]), math.cos(error[2]))
        gradient = jacobian.T @ (w * residual) + prior_info @ error
        step = -np.linalg.solve(hessian, gradient)
        x = x + step
        if np.abs(step[:2]).max() < 1e-4 and abs(step[2]) < 1e-5:
            break
    linear = linearize(x, config.final_correspondence_m)
    if linear is None:
        report["status"] = "REJECTED_FEW_CORRESPONDENCES"
        return report
    residual, jacobian, huber = linear
    information = jacobian.T @ (jacobian * (weight * huber)[:, None])
    values, vectors = np.linalg.eigh(information)
    constrained = vectors[:, values >= config.min_direction_points * weight]
    delta = x - prior
    delta[2] = math.atan2(math.sin(delta[2]), math.cos(delta[2]))
    projected = constrained @ (constrained.T @ delta)   # remove unconstrained components
    report.update(scan_information_eigenvalues=values.tolist(), constrained_directions=int(constrained.shape[1]),
                  candidate_delta=delta.tolist(), projected_delta=projected.tolist())
    if constrained.shape[1] == 0:
        report["status"] = "SKIPPED_DEGENERATE"
        return report
    final_pose = prior + projected
    final = linearize(final_pose, config.final_correspondence_m)
    if final is None:
        report["status"] = "REJECTED_FEW_CORRESPONDENCES"
        return report
    residual = final[0]
    report.update(correspondences=int(len(residual)), rms_m=float(np.sqrt(np.mean(residual ** 2))))
    if report["rms_m"] > config.max_rms_m:
        report["status"] = "REJECTED_RESIDUAL"
    elif math.hypot(projected[0], projected[1]) > config.max_correction_xy_m or abs(projected[2]) > config.max_correction_yaw_rad:
        report["status"] = "REJECTED_LARGE_CORRECTION"
    else:
        report.update(status="CORRECTED", delta=projected.tolist(),
                      pose=[float(final_pose[0]), float(final_pose[1]),
                            math.atan2(math.sin(final_pose[2]), math.cos(final_pose[2]))])
    return report
