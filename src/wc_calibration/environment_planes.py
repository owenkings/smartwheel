"""Offline automatic planes from an organised SDK FLU point cloud, in metres.

The equation is ``normal @ point + d_m == 0``. Normals point towards the
sensor, so d_m is positive. Pixel indices always refer to the original
row-major cloud. These experimental gates are not an accuracy certificate.
No hardware, network, or runtime configuration is accessed by this module.
"""
from __future__ import annotations

from copy import deepcopy
import math
from typing import Any

import numpy as np
from scipy import ndimage
from scipy.spatial import ConvexHull, QhullError


ENVIRONMENT_PLANE_POLICY = {
    "label": "EXPERIMENTAL_NOT_ACCURACY_PROOF",
    "coordinate_frame": "SDK_FLU",
    "plane_equation": "normal.dot(point) + d_m = 0; d_m > 0",
    "minimum_range_m": 0.25,
    "maximum_range_m": 12.0,
    "edge_absolute_jump_m": 0.08,
    "edge_relative_jump": 0.04,
    "edge_dilation_pixels": 1,
    "invalid_hole_edge_minimum_pixels": 4,
    "interior_erosion_pixels": 1,
    "diagnostic_region_dilation_pixels": 2,
    "ransac_distance_m": 0.025,
    "ransac_trials": 320,
    "ransac_batch_size": 40,
    "maximum_models": 8,
    "maximum_components_per_model": 3,
    "maximum_planes": 8,
    "support_assignment": "second pass over original valid non-edge pixels; nearest absolute residual within recovered connected proposal support",
    "duplicate_maximum_normal_angle_deg": 3.0,
    "duplicate_maximum_offset_m": 0.025,
    "duplicate_minimum_support_overlap": 0.60,
    "duplicate_support_overlap_definition": "intersection divided by smaller original connected support",
    "assignment_iteration_bound": "at most initial qualified proposal count plus one; failed proposals only leave",
    "maximum_points": 20000,
    "minimum_component_points": 100,
    "minimum_interior_points": 80,
    "minimum_span_m": [0.45, 0.30],
    "minimum_hull_area_m2": 0.15,
    "maximum_aspect_ratio": 10.0,
    "maximum_rmse_m": 0.020,
    "maximum_p95_m": 0.040,
    "maximum_unexplained_region_p95_m": 0.075,
    "maximum_unexplained_region_outlier_fraction": 0.20,
    "minimum_neighbor_plane_normal_angle_deg": 10.0,
    "region_gate_definition": "retain every raw region residual; gate unexplained points after separating boundary pixels supported by another qualified, differently oriented plane",
    "span_definition": "5th-to-95th percentile along TLS in-plane principal axes",
    "area_definition": "convex hull of accepted interior projected into fitted plane",
    "connectivity": "4-connected original image pixels",
    "temporal": {
        "minimum_frames": 3,
        "maximum_evaluated_frames": 64,
        "minimum_valid_fraction": 0.80,
        "minimum_frame_pass_fraction": 0.90,
        "maximum_rmse_m": 0.025,
        "maximum_p95_m": 0.050,
        "maximum_reference_p95_m": 0.060,
        "maximum_normal_angle_deg": 2.0,
        "maximum_offset_delta_m": 0.040,
        "maximum_offset_span_m": 0.050,
        "minimum_span_ratio_to_reference": 0.60,
        "support": "fixed original interior pixel indices; no per-frame RANSAC or trimming",
        "independent_validation": False,
        "interpretation": "repeatability within one capture only; not independent holdout evidence",
    },
}

_CROSS = ndimage.generate_binary_structure(2, 1)


def _stats(values: np.ndarray) -> dict[str, float | None]:
    if not len(values):
        return {"rmse_m": None, "p95_m": None, "maximum_m": None}
    return {
        "rmse_m": float(np.sqrt(np.mean(np.square(values)))),
        "p95_m": float(np.quantile(np.abs(values), 0.95)),
        "maximum_m": float(np.max(np.abs(values))),
    }


def _fit_tls(points: np.ndarray) -> tuple[np.ndarray, float, np.ndarray, np.ndarray]:
    center = np.mean(points, axis=0)
    # eigh is bounded in memory and computes just a 3 x 3 covariance matrix.
    centered = points - center
    _, axes = np.linalg.eigh(centered.T @ centered / len(points))
    normal = axes[:, 0].copy()
    if float(normal @ center) > 0.0:
        normal *= -1.0
    return normal, -float(normal @ center), center, axes[:, [2, 1]]


def _valid_points(points: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    finite = np.isfinite(points).all(axis=-1)
    ranges = np.full(finite.shape, np.nan, dtype=float)
    ranges[finite] = np.linalg.norm(points[finite], axis=-1)
    valid = finite & (ranges >= ENVIRONMENT_PLANE_POLICY["minimum_range_m"])
    valid &= ranges <= ENVIRONMENT_PLANE_POLICY["maximum_range_m"]
    return valid, finite, ranges


def _edges(valid: np.ndarray, ranges: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    raw = np.zeros(valid.shape, dtype=bool)
    missing_labels, _ = ndimage.label(~valid, structure=_CROSS)
    missing_sizes = np.bincount(missing_labels.ravel())
    large_missing = (missing_labels > 0) & (missing_sizes[missing_labels] >= ENVIRONMENT_PLANE_POLICY["invalid_hole_edge_minimum_pixels"])
    for axis in (0, 1):
        lo = [slice(None), slice(None)]
        hi = [slice(None), slice(None)]
        lo[axis], hi[axis] = slice(None, -1), slice(1, None)
        lo, hi = tuple(lo), tuple(hi)
        both = valid[lo] & valid[hi]
        threshold = ENVIRONMENT_PLANE_POLICY["edge_absolute_jump_m"]
        threshold = threshold + ENVIRONMENT_PLANE_POLICY["edge_relative_jump"] * np.minimum(ranges[lo], ranges[hi])
        jump = both & (np.abs(ranges[lo] - ranges[hi]) > threshold)
        # A single absent return is not evidence of a physical depth edge.
        # Large missing regions still get conservative boundary exclusion.
        invalid_boundary = (valid[lo] & large_missing[hi]) | (valid[hi] & large_missing[lo])
        marked = jump | invalid_boundary
        raw[lo] |= marked & valid[lo]
        raw[hi] |= marked & valid[hi]
    expanded = ndimage.binary_dilation(raw, structure=_CROSS,
        iterations=ENVIRONMENT_PLANE_POLICY["edge_dilation_pixels"])
    return raw, expanded & valid


def _local_triplets(mask: np.ndarray, rng: np.random.Generator, limit: int) -> np.ndarray:
    """Spatial proposals augment ordinary random three-point RANSAC proposals."""
    h, w = mask.shape
    candidates = []
    for step in (3, 6):
        if h <= step or w <= step:
            continue
        anchors = mask[:-step, :-step] & mask[:-step, step:] & mask[step:, :-step]
        y, x = np.nonzero(anchors)
        if len(y):
            if len(y) > limit:
                take = rng.choice(len(y), limit, replace=False)
                y, x = y[take], x[take]
            a = y * w + x
            candidates.append(np.stack((a, a + step, a + step * w), axis=1))
    if not candidates:
        return np.empty((0, 3), dtype=np.int64)
    result = np.concatenate(candidates)
    if len(result) > limit:
        result = result[rng.choice(len(result), limit, replace=False)]
    return result


def _ransac(points: np.ndarray, available: np.ndarray,
            rng: np.random.Generator) -> tuple[np.ndarray, float] | None:
    ids = np.flatnonzero(available.ravel())
    if len(ids) < ENVIRONMENT_PLANE_POLICY["minimum_component_points"]:
        return None
    trials = ENVIRONMENT_PLANE_POLICY["ransac_trials"]
    local = _local_triplets(available, rng, trials // 3)
    # Duplicated indices yield zero-area proposals and are explicitly discarded.
    random = ids[rng.integers(0, len(ids), size=(trials - len(local), 3))]
    triples = np.concatenate((random, local), axis=0)
    a, b, c = (points[triples[:, i]] for i in range(3))
    normals = np.cross(b - a, c - a)
    lengths = np.linalg.norm(normals, axis=1)
    good = lengths > 1e-8
    normals = normals[good] / lengths[good, None]
    offsets = -np.einsum("ij,ij->i", normals, a[good])
    best = None
    best_key = (-1, -math.inf)
    subset = points[ids]
    batch_size = ENVIRONMENT_PLANE_POLICY["ransac_batch_size"]
    for start in range(0, len(normals), batch_size):
        batch = normals[start:start + batch_size]
        residual = np.abs(np.einsum("ij,kj->ik", subset, batch, optimize=False)
                          + offsets[start:start + len(batch)])
        mask = residual <= ENVIRONMENT_PLANE_POLICY["ransac_distance_m"]
        counts = mask.sum(axis=0)
        for column in np.flatnonzero(counts >= best_key[0]):
            count = int(counts[column])
            if count < 3:
                continue
            mse = float(np.mean(residual[mask[:, column], column] ** 2))
            key = (count, -mse)
            if key > best_key:
                best_key = key
                best = (batch[column].copy(), float(offsets[start + column]))
    if best is None:
        return None
    normal, d = best
    support = np.abs(subset @ normal + d) <= ENVIRONMENT_PLANE_POLICY["ransac_distance_m"]
    for _ in range(3):
        if np.count_nonzero(support) < 3:
            break
        normal, d, _, _ = _fit_tls(subset[support])
        updated = np.abs(subset @ normal + d) <= ENVIRONMENT_PLANE_POLICY["ransac_distance_m"]
        if np.array_equal(updated, support):
            break
        support = updated
    return normal, d


def _temporal(points: np.ndarray, indices: np.ndarray, normal: np.ndarray, d: float,
              center: np.ndarray, frames: np.ndarray | None,
              frame_error: str | None) -> dict[str, Any]:
    policy = ENVIRONMENT_PLANE_POLICY["temporal"]
    result: dict[str, Any] = {
        "status": "NOT_EVALUATED", "reasons": [], "provided_frame_count": 0,
        "evaluated_frame_count": 0, "support_point_count": int(len(indices)),
        "fixed_pixel_support": True, "independent_validation": False,
        "passing_frame_fraction": None, "offset_span_m": None,
        "maximum_normal_angle_deg": None, "frames": [],
        "interpretation": policy["interpretation"],
    }
    if frame_error:
        result.update(status="REJECTED", reasons=[frame_error])
        return result
    if frames is None or not len(frames):
        result["reasons"] = ["NO_TEMPORAL_FRAMES"]
        return result
    result["provided_frame_count"] = int(len(frames))
    take = np.unique(np.linspace(0, len(frames) - 1,
        min(len(frames), policy["maximum_evaluated_frames"]), dtype=int))
    details = []
    offsets, angles = [], []
    _, _, _, reference_axes = _fit_tls(points[indices])
    reference_coords = (points[indices] - center) @ reference_axes
    reference_spans = np.quantile(reference_coords, 0.95, axis=0) - np.quantile(reference_coords, 0.05, axis=0)
    for frame_index in take:
        fixed = frames[frame_index, indices]
        valid, _, _ = _valid_points(fixed)
        sample = fixed[valid]
        reasons = []
        fraction = float(np.mean(valid))
        row: dict[str, Any] = {
            "frame_index": int(frame_index), "valid_point_count": int(len(sample)),
            "valid_fraction": fraction, "status": "REJECTED", "reasons": reasons,
            "rmse_m": None, "p95_m": None, "reference_plane_p95_m": None,
            "normal_angle_deg": None, "offset_delta_m": None,
            "centroid_shift_m": None, "span_m": None, "minimum_span_ratio_to_reference": None,
        }
        if fraction < policy["minimum_valid_fraction"]:
            reasons.append("FIXED_SUPPORT_MISSING_POINTS")
        if len(sample) < ENVIRONMENT_PLANE_POLICY["minimum_interior_points"]:
            reasons.append("TOO_FEW_FIXED_SUPPORT_POINTS")
        if len(sample) >= 3:
            n_frame, d_frame, c_frame, _ = _fit_tls(sample)
            if float(n_frame @ normal) < 0.0:
                n_frame, d_frame = -n_frame, -d_frame
            stats = _stats(sample @ n_frame + d_frame)
            angle = math.degrees(math.acos(float(np.clip(n_frame @ normal, -1.0, 1.0))))
            delta = float(d_frame - d)
            reference_p95 = float(np.quantile(np.abs(sample @ normal + d), 0.95))
            projected = (sample - c_frame) @ reference_axes
            spans = np.quantile(projected, 0.95, axis=0) - np.quantile(projected, 0.05, axis=0)
            span_ratio = float(np.min(spans / np.maximum(reference_spans, 1e-9)))
            row.update(rmse_m=stats["rmse_m"], p95_m=stats["p95_m"],
                reference_plane_p95_m=reference_p95, normal_angle_deg=angle,
                offset_delta_m=delta, centroid_shift_m=float(np.linalg.norm(c_frame - center)),
                span_m=np.sort(spans)[::-1].tolist(), minimum_span_ratio_to_reference=span_ratio)
            offsets.append(d_frame)
            angles.append(angle)
            if stats["rmse_m"] > policy["maximum_rmse_m"]:
                reasons.append("FIXED_SUPPORT_RMSE_EXCEEDED")
            if stats["p95_m"] > policy["maximum_p95_m"]:
                reasons.append("FIXED_SUPPORT_P95_EXCEEDED")
            if reference_p95 > policy["maximum_reference_p95_m"]:
                reasons.append("REFERENCE_PLANE_P95_EXCEEDED")
            if angle > policy["maximum_normal_angle_deg"]:
                reasons.append("NORMAL_NOT_STABLE")
            if abs(delta) > policy["maximum_offset_delta_m"]:
                reasons.append("OFFSET_NOT_STABLE")
            if span_ratio < policy["minimum_span_ratio_to_reference"]:
                reasons.append("FIXED_SUPPORT_SPATIAL_COVERAGE_COLLAPSED")
        row["status"] = "PASS" if not reasons else "REJECTED"
        details.append(row)
    result["frames"] = details
    result["evaluated_frame_count"] = len(details)
    result["passing_frame_fraction"] = sum(row["status"] == "PASS" for row in details) / len(details)
    result["offset_span_m"] = float(np.ptp(offsets)) if offsets else None
    result["maximum_normal_angle_deg"] = max(angles) if angles else None
    if len(frames) < policy["minimum_frames"]:
        result["reasons"] = ["INSUFFICIENT_TEMPORAL_FRAMES"]
        return result
    reasons = []
    if result["passing_frame_fraction"] < policy["minimum_frame_pass_fraction"]:
        reasons.append("LOW_TEMPORAL_FRAME_PASS_FRACTION")
    if not offsets or result["offset_span_m"] > policy["maximum_offset_span_m"]:
        reasons.append("TEMPORAL_OFFSET_SPAN_EXCEEDED")
    result.update(status="PASS" if not reasons else "REJECTED", reasons=reasons)
    return result


def _component_fit(points: np.ndarray, component: np.ndarray, valid: np.ndarray,
                   edge: np.ndarray, identifier: str, group: str,
                   frames: np.ndarray | None, frame_error: str | None) -> dict[str, Any]:
    missing_labels, _ = ndimage.label(~valid, structure=_CROSS)
    missing_sizes = np.bincount(missing_labels.ravel())
    small_missing = (missing_labels > 0) & (missing_sizes[missing_labels] < ENVIRONMENT_PLANE_POLICY["invalid_hole_edge_minimum_pixels"])
    interior = ndimage.binary_erosion(component | small_missing, structure=_CROSS,
        iterations=ENVIRONMENT_PLANE_POLICY["interior_erosion_pixels"], border_value=0) & component
    core_ids = np.flatnonzero(interior.ravel())
    region = ndimage.binary_dilation(component, structure=_CROSS,
        iterations=ENVIRONMENT_PLANE_POLICY["diagnostic_region_dilation_pixels"]) & valid
    region_ids = np.flatnonzero(region.ravel())
    reasons = []
    result: dict[str, Any] = {
        "id": identifier, "coplanar_group": group, "status": "REJECTED",
        "normal": None, "d_m": None, "centroid": None, "span_m": [0.0, 0.0],
        "rmse_m": None, "p95_m": None, "hull_area_m2": 0.0,
        "point_count": int(len(core_ids)), "indices": core_ids.tolist(),
        "component_point_count": int(np.count_nonzero(component)),
        "all_region_point_count": int(len(region_ids)),
        "all_region_rmse_m": None, "all_region_p95_m": None,
        "all_region_outlier_count": None, "all_region_outlier_fraction": None,
        "neighbor_plane_explained_point_count": 0,
        "unexplained_region_point_count": int(len(region_ids)),
        "unexplained_region_p95_m": None, "unexplained_region_outlier_fraction": None,
        "edge_points_in_region": int(np.count_nonzero(region & edge)),
        "erosion_removed_point_count": int(np.count_nonzero(component) - len(core_ids)),
        "quality": {"status": "REJECTED", "reasons": reasons}, "reasons": reasons,
        "temporal": {"status": "NOT_EVALUATED", "reasons": ["NO_GEOMETRIC_FIT"],
                     "independent_validation": False},
        "_region_indices": region_ids,
    }
    if len(core_ids) < ENVIRONMENT_PLANE_POLICY["minimum_interior_points"]:
        reasons.append("TOO_FEW_INTERIOR_POINTS")
    if len(core_ids) < 3:
        return result
    normal, d, center, axes = _fit_tls(points[core_ids])
    residual = points[core_ids] @ normal + d
    projected = (points[core_ids] - center) @ axes
    spans = np.sort(np.quantile(projected, 0.95, axis=0)
                    - np.quantile(projected, 0.05, axis=0))[::-1]
    stats = _stats(residual)
    raw_residual = points[region_ids] @ normal + d
    raw_stats = _stats(raw_residual)
    outliers = int(np.count_nonzero(np.abs(raw_residual) > ENVIRONMENT_PLANE_POLICY["ransac_distance_m"]))
    outlier_fraction = outliers / len(region_ids)
    try:
        area = float(ConvexHull(projected).volume)
    except QhullError:
        area = 0.0
    result.update(normal=normal.tolist(), d_m=float(d), centroid=center.tolist(),
        span_m=spans.tolist(), rmse_m=stats["rmse_m"], p95_m=stats["p95_m"],
        hull_area_m2=area, all_region_rmse_m=raw_stats["rmse_m"],
        all_region_p95_m=raw_stats["p95_m"], all_region_outlier_count=outliers,
        all_region_outlier_fraction=float(outlier_fraction))
    if d <= 1e-6:
        reasons.append("PLANE_ORIENTATION_UNDEFINED_AT_SENSOR")
    if np.any(spans < np.asarray(ENVIRONMENT_PLANE_POLICY["minimum_span_m"])):
        reasons.append("INSUFFICIENT_TWO_DIMENSIONAL_SPAN")
    if area < ENVIRONMENT_PLANE_POLICY["minimum_hull_area_m2"]:
        reasons.append("INSUFFICIENT_PLANAR_AREA")
    if spans[1] <= 1e-9 or spans[0] / spans[1] > ENVIRONMENT_PLANE_POLICY["maximum_aspect_ratio"]:
        reasons.append("STRIP_LIKE_SUPPORT")
    if stats["rmse_m"] > ENVIRONMENT_PLANE_POLICY["maximum_rmse_m"]:
        reasons.append("INTERIOR_RMSE_EXCEEDED")
    if stats["p95_m"] > ENVIRONMENT_PLANE_POLICY["maximum_p95_m"]:
        reasons.append("INTERIOR_P95_EXCEEDED")
    result["quality"]["status"] = "PASS" if not reasons else "REJECTED"
    if not reasons:
        result["temporal"] = _temporal(points, core_ids, normal, d, center, frames, frame_error)
        temporal_status = result["temporal"]["status"]
        result["status"] = "CANDIDATE" if temporal_status == "NOT_EVALUATED" else temporal_status
        if temporal_status == "REJECTED":
            # Keep geometry and temporal decisions separate, without aliasing reasons.
            result["reasons"] = ["TEMPORAL_REPEATABILITY_REJECTED"]
    return result


def _finalize_region_quality(candidates: list[dict[str, Any]], points: np.ndarray) -> None:
    """Do not mistake a real wall/floor junction for contamination of a plane.

    Raw expanded-region residuals remain untouched. A boundary pixel can be
    explained only by another large, geometrically qualified plane with a
    substantially different normal. Unknown outliers and parallel offsets
    remain in the gate. Rejected donor planes are withdrawn to a fixed point.
    """
    base = {i for i, p in enumerate(candidates) if p["quality"]["status"] == "PASS"}
    donors = {i for i in base if candidates[i]["temporal"]["status"] != "REJECTED"
              and candidates[i]["status"] != "REJECTED"}
    threshold = ENVIRONMENT_PLANE_POLICY["ransac_distance_m"]
    cosine_limit = math.cos(math.radians(ENVIRONMENT_PLANE_POLICY["minimum_neighbor_plane_normal_angle_deg"]))

    def measure(index: int, eligible: set[int]) -> tuple[dict[str, Any], list[str]]:
        plane = candidates[index]
        xyz = points[plane["_region_indices"]]
        normal = np.asarray(plane["normal"])
        residual = np.abs(xyz @ normal + plane["d_m"])
        explained = np.zeros(len(xyz), dtype=bool)
        for other_index in sorted(eligible):
            if other_index == index:
                continue
            other = candidates[other_index]
            other_normal = np.asarray(other["normal"])
            if abs(float(normal @ other_normal)) > cosine_limit:
                continue
            on_other = np.abs(xyz @ other_normal + other["d_m"]) <= threshold
            explained |= (residual > threshold) & on_other
        remaining = residual[~explained]
        p95 = float(np.quantile(remaining, 0.95)) if len(remaining) else None
        fraction = float(np.mean(remaining > threshold)) if len(remaining) else None
        metrics = {
            "neighbor_plane_explained_point_count": int(np.count_nonzero(explained)),
            "unexplained_region_point_count": int(len(remaining)),
            "unexplained_region_p95_m": p95,
            "unexplained_region_outlier_fraction": fraction,
        }
        reasons = []
        if p95 is None or p95 > ENVIRONMENT_PLANE_POLICY["maximum_unexplained_region_p95_m"]:
            reasons.append("UNEXPLAINED_REGION_P95_EXCEEDED")
        if fraction is None or fraction > ENVIRONMENT_PLANE_POLICY["maximum_unexplained_region_outlier_fraction"]:
            reasons.append("UNEXPLAINED_REGION_OUTLIER_FRACTION_EXCEEDED")
        return metrics, reasons

    for _ in range(len(donors) + 1):
        retained = {i for i in donors if not measure(i, donors)[1]}
        if retained == donors:
            break
        donors = retained
    for index, plane in enumerate(candidates):
        if index in base:
            metrics, reasons = measure(index, donors)
            plane.update(metrics)
            if reasons:
                plane["quality"]["status"] = "REJECTED"
                plane["quality"]["reasons"].extend(reasons)
                plane["status"] = "REJECTED"
                plane["reasons"] = list(dict.fromkeys(plane["reasons"] + reasons))
        plane.pop("_region_indices", None)


def _deduplicate_proposals(entries: list[dict[str, Any]]) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    """Suppress unresolved near-identical models, never average separate faces.

    Normal/offset proximity alone is insufficient: original image supports
    must overlap substantially as well. Larger original interiors win, with
    deterministic discovery order breaking ties. Disconnected coplanar patches
    remain distinct and retain their shared coplanar_group.
    """
    order = sorted(range(len(entries)), key=lambda i: (-entries[i]["fit"]["point_count"], i))
    kept: list[int] = []
    suppressed = []
    cos_limit = math.cos(math.radians(ENVIRONMENT_PLANE_POLICY["duplicate_maximum_normal_angle_deg"]))
    for index in order:
        candidate = entries[index]
        fit = candidate["fit"]
        normal = np.asarray(fit["normal"])
        duplicate = None
        for prior_index in kept:
            prior = entries[prior_index]
            prior_fit = prior["fit"]
            dot = float(normal @ np.asarray(prior_fit["normal"]))
            if dot < cos_limit:
                continue
            if abs(fit["d_m"] - prior_fit["d_m"]) > ENVIRONMENT_PLANE_POLICY["duplicate_maximum_offset_m"]:
                continue
            overlap = int(np.count_nonzero(candidate["mask"] & prior["mask"]))
            denominator = min(np.count_nonzero(candidate["mask"]), np.count_nonzero(prior["mask"]))
            fraction = overlap / denominator
            if fraction >= ENVIRONMENT_PLANE_POLICY["duplicate_minimum_support_overlap"]:
                duplicate = (prior_index, fraction)
                break
        if duplicate is None:
            kept.append(index)
        else:
            prior_index, fraction = duplicate
            suppressed.append({
                "id": fit["id"], "retained_id": entries[prior_index]["fit"]["id"],
                "support_overlap_fraction": float(fraction),
                "reason": "NEAR_IDENTICAL_MODEL_WITH_OVERLAPPING_ORIGINAL_SUPPORT",
            })
    # Discovery order also provides a deterministic tie rule at intersections.
    return [entries[i] for i in sorted(kept)], suppressed


def _exclusive_refit(points: np.ndarray, entries: list[dict[str, Any]], valid: np.ndarray,
                     edge: np.ndarray, first_owner: np.ndarray,
                     frames: np.ndarray | None, frame_error: str | None) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    """Recover supports before quality fitting, then retain mutually exclusive interiors.

    Each pixel is assigned only once, to the closest active plane, subject to
    the unchanged residual gate and that proposal's ORIGINAL connected support.
    Failed planes cannot permanently reserve points: remove their proposals
    and reassign from the same original data. No new proposals are generated.
    """
    eligible = (valid & ~edge).ravel()
    active = list(range(len(entries)))
    archived = []
    final = []
    rounds = 0
    ambiguous_count = 0
    total_small_components = total_small_points = 0
    for _ in range(len(entries) + 1):
        if not active:
            break
        rounds += 1
        residuals = np.full((len(points), len(active)), np.inf)
        for column, entry_index in enumerate(active):
            entry = entries[entry_index]
            ids = np.flatnonzero(eligible & entry["mask"].ravel())
            fit = entry["fit"]
            residuals[ids, column] = np.abs(points[ids] @ np.asarray(fit["normal"]) + fit["d_m"])
        inside = residuals <= ENVIRONMENT_PLANE_POLICY["ransac_distance_m"]
        ambiguous_count = int(np.count_nonzero(inside.sum(axis=1) > 1))
        owners = np.argmin(residuals, axis=1)
        owners[~np.any(inside, axis=1)] = -1
        current = []
        round_small_components = round_small_points = 0
        for column, entry_index in enumerate(active):
            entry = entries[entry_index]
            mask = (owners == column).reshape(valid.shape)
            labels, count = ndimage.label(mask, structure=_CROSS)
            sizes = np.bincount(labels.ravel())
            order = np.argsort(-sizes[1:], kind="stable") + 1
            selected = []
            for position, label in enumerate(order):
                if sizes[label] < ENVIRONMENT_PLANE_POLICY["minimum_component_points"] or position >= ENVIRONMENT_PLANE_POLICY["maximum_components_per_model"]:
                    round_small_components += 1
                    round_small_points += int(sizes[label])
                    continue
                selected.append(label)
            # Preserve a concrete rejection diagnostic when exclusive ownership
            # fragments an otherwise qualified raw proposal into small pieces.
            if not selected:
                selected = [int(order[0])] if len(order) else [0]
            for part, label in enumerate(selected):
                component = labels == label if label else np.zeros(valid.shape, dtype=bool)
                identifier = entry["fit"]["id"] if part == 0 else f"{entry['fit']['id']}_part_{part + 1:02d}"
                plane = _component_fit(points, component, valid, edge, identifier,
                    entry["fit"]["coplanar_group"], frames, frame_error)
                ids = np.asarray(plane["indices"], dtype=int)
                old_owner = first_owner[ids]
                plane.update({
                    "proposal_model_index": entry["model_index"] + 1,
                    "original_support_point_count": int(np.count_nonzero(entry["mask"])),
                    "original_interior_point_count": int(entry["fit"]["point_count"]),
                    "proposal_consumed_point_count": entry["consumed_count"],
                    "recovered_interior_from_earlier_models_point_count": int(np.count_nonzero((old_owner >= 0) & (old_owner < entry["model_index"]))),
                    "support_assignment": ENVIRONMENT_PLANE_POLICY["support_assignment"],
                    "_entry_index": entry_index,
                })
                if np.count_nonzero(component) < ENVIRONMENT_PLANE_POLICY["minimum_component_points"]:
                    plane["quality"]["reasons"].append("TOO_FEW_EXCLUSIVE_COMPONENT_POINTS")
                    plane["quality"]["status"] = "REJECTED"
                    plane["status"] = "REJECTED"
                    plane["reasons"] = list(dict.fromkeys(plane["reasons"] + ["TOO_FEW_EXCLUSIVE_COMPONENT_POINTS"]))
                current.append(plane)
        _finalize_region_quality(current, points)
        surviving = {p["_entry_index"] for p in current if p["status"] != "REJECTED"}
        total_small_components, total_small_points = round_small_components, round_small_points
        if surviving == set(active):
            final = current
            break
        # Archive only proposals that completely failed, then let survivors
        # recover any pixels the failed models had temporarily owned.
        archived.extend(p for p in current if p["_entry_index"] not in surviving)
        active = [i for i in active if i in surviving]
    candidates = archived + final
    for plane in candidates:
        plane.pop("_entry_index", None)
    return candidates, {
        "reassignment_rounds": rounds,
        "overlapping_original_support_pixel_count": ambiguous_count,
        "qualified_proposals_before_assignment": len(entries),
        "surviving_proposals_after_assignment": len(active),
        "ignored_small_or_excess_component_count": total_small_components,
        "ignored_small_or_excess_component_points": total_small_points,
    }


def extract_planes(cloud: Any, *, width: int, height: int,
                   frames: Any = None, seed: int = 0) -> dict[str, Any]:
    """Extract up to eight large planar patches without manually marked pixels.

    ``cloud`` is N x 3 and ``frames`` optionally K x N x 3, with N=width*height.
    Missing points are NaNs. ``cloud`` may be an externally computed temporal
    representative, but extraction never pools repeated frames as new evidence.
    ``planes`` includes geometry candidates and repeatability-passing patches;
    inspect both quality and temporal status. Absence of frames never passes
    temporal checks. All returned values are strict-JSON compatible.
    """
    result: dict[str, Any] = {
        "schema_version": "environment_planes.v2", "policy": deepcopy(ENVIRONMENT_PLANE_POLICY),
        "status": "REJECTED", "reasons": [], "seed": None, "width": None, "height": None,
        "preprocessing": {}, "planes": [], "rejected_planes": [],
        "invalid_indices": [], "range_filtered_indices": [], "raw_edge_indices": [],
        "edge_indices": [], "unassigned_indices": [], "diagnostics": {},
    }
    try:
        w, h = int(width), int(height)
        if isinstance(width, bool) or isinstance(height, bool) or w != width or h != height or w < 3 or h < 3:
            raise ValueError
        seed_value = int(seed)
        if seed_value != seed or seed_value < 0:
            raise ValueError
    except (TypeError, ValueError, OverflowError):
        result["reasons"] = ["INVALID_WIDTH_HEIGHT_OR_SEED"]
        return result
    result.update(width=w, height=h, seed=seed_value)
    if w * h > ENVIRONMENT_PLANE_POLICY["maximum_points"]:
        result["reasons"] = ["POINT_COUNT_EXCEEDS_BOUNDED_EXTRACTION_POLICY"]
        return result
    try:
        points = np.asarray(cloud, dtype=float)
    except (TypeError, ValueError):
        result["reasons"] = ["INVALID_POINT_ARRAY"]
        return result
    if points.shape != (w * h, 3):
        result["reasons"] = ["POINT_ARRAY_MUST_MATCH_WIDTH_TIMES_HEIGHT_BY_3"]
        return result
    sequence = None
    frame_error = None
    if frames is not None:
        try:
            sequence = np.asarray(frames, dtype=float)
            if sequence.ndim != 3 or sequence.shape[1:] != points.shape:
                sequence, frame_error = None, "INVALID_TEMPORAL_FRAME_SHAPE"
        except (TypeError, ValueError):
            frame_error = "INVALID_TEMPORAL_FRAME_ARRAY"
    valid_1d, finite_1d, ranges_1d = _valid_points(points)
    valid, ranges = valid_1d.reshape(h, w), ranges_1d.reshape(h, w)
    raw_edge, edge = _edges(valid, ranges)
    available = valid & ~edge
    original_eligible = available.copy()
    invalid_ids = np.flatnonzero(~finite_1d)
    range_ids = np.flatnonzero(finite_1d & ~valid_1d)
    result.update(invalid_indices=invalid_ids.tolist(), range_filtered_indices=range_ids.tolist(),
        raw_edge_indices=np.flatnonzero(raw_edge.ravel()).tolist(),
        edge_indices=np.flatnonzero(edge.ravel()).tolist())
    result["preprocessing"] = {
        "input_point_count": int(len(points)), "invalid_point_count": int(len(invalid_ids)),
        "range_filtered_point_count": int(len(range_ids)),
        "valid_point_count": int(np.count_nonzero(valid)),
        "raw_edge_point_count": int(np.count_nonzero(raw_edge)),
        "edge_point_count": int(np.count_nonzero(edge)),
        "eligible_point_count": int(np.count_nonzero(available)),
        "provided_frame_count": int(len(sequence)) if sequence is not None else 0,
        "frame_error": frame_error,
    }
    if frame_error:
        result["reasons"].append(frame_error)
    rng = np.random.default_rng(seed_value)
    ignored_component_count = 0
    ignored_component_points = 0
    models_examined = 0
    candidate_number = 0
    assigned = np.zeros(len(points), dtype=bool)
    first_owner = np.full(len(points), -1, dtype=int)
    proposals = []
    for model_index in range(ENVIRONMENT_PLANE_POLICY["maximum_models"]):
        model = _ransac(points, available, rng)
        if model is None:
            break
        models_examined += 1
        normal, d = model
        residual = np.full(len(points), np.inf)
        ids = np.flatnonzero(available.ravel())
        residual[ids] = np.abs(points[ids] @ normal + d)
        consensus = available & (residual.reshape(h, w) <= ENVIRONMENT_PLANE_POLICY["ransac_distance_m"])
        count = int(np.count_nonzero(consensus))
        if count < ENVIRONMENT_PLANE_POLICY["minimum_component_points"]:
            break
        proposals.append({"normal": normal, "d_m": d, "consumed_count": count})
        first_owner[consensus.ravel()] = model_index
        # Consumption is exclusively a bounded proposal-discovery mechanism.
        # It must never define the support used by final quality fitting.
        available[consensus] = False
    raw_rejected = []
    entries = []
    original_ids = np.flatnonzero(original_eligible.ravel())
    for model_index, proposal in enumerate(proposals):
        residual = np.full(len(points), np.inf)
        residual[original_ids] = np.abs(points[original_ids] @ proposal["normal"] + proposal["d_m"])
        recovered = original_eligible & (residual.reshape(h, w) <= ENVIRONMENT_PLANE_POLICY["ransac_distance_m"])
        labels, component_count = ndimage.label(recovered, structure=_CROSS)
        sizes = np.bincount(labels.ravel())
        order = np.argsort(-sizes[1:], kind="stable") + 1
        for position, label in enumerate(order):
            size = int(sizes[label])
            if size < ENVIRONMENT_PLANE_POLICY["minimum_component_points"] or position >= ENVIRONMENT_PLANE_POLICY["maximum_components_per_model"]:
                ignored_component_count += 1
                ignored_component_points += size
                continue
            candidate_number += 1
            plane = _component_fit(points, labels == label, valid, edge,
                f"plane_{candidate_number:03d}", f"model_{model_index + 1:03d}", None, None)
            plane.pop("_region_indices", None)
            if plane["quality"]["status"] != "PASS":
                plane["support_assignment_stage"] = "ORIGINAL_SUPPORT_GEOMETRY_REJECTED"
                raw_rejected.append(plane)
            else:
                entries.append({"fit": plane, "mask": labels == label,
                    "model_index": model_index, "consumed_count": proposal["consumed_count"]})
    entries, suppressed = _deduplicate_proposals(entries)
    candidates, assignment_diagnostics = _exclusive_refit(points, entries, valid, edge,
        first_owner, sequence, frame_error)
    candidates = raw_rejected + candidates
    ignored_component_count += assignment_diagnostics["ignored_small_or_excess_component_count"]
    ignored_component_points += assignment_diagnostics["ignored_small_or_excess_component_points"]
    for plane in candidates:
        if plane["status"] != "REJECTED" and len(result["planes"]) >= ENVIRONMENT_PLANE_POLICY["maximum_planes"]:
            plane["status"] = "REJECTED"
            plane["reasons"] = plane["reasons"] + ["EXTRACTION_PLANE_LIMIT_REACHED"]
        if plane["status"] == "REJECTED":
            result["rejected_planes"].append(plane)
        else:
            result["planes"].append(plane)
            assigned[np.asarray(plane["indices"], dtype=int)] = True
    result["unassigned_indices"] = np.flatnonzero(valid_1d & ~assigned).tolist()
    # Nearest-plane residual covers ALL valid input pixels, including pixels
    # excluded by edges, RANSAC, connectivity, erosion, and temporal rejection.
    nearest = None
    if result["planes"]:
        nearest = np.full(np.count_nonzero(valid_1d), np.inf)
        for plane in result["planes"]:
            residual = np.abs(points[valid_1d] @ np.asarray(plane["normal"]) + plane["d_m"])
            nearest = np.minimum(nearest, residual)
    global_stats = _stats(nearest if nearest is not None else np.empty(0))
    result["diagnostics"] = {
        "models_examined": models_examined, "candidate_count": candidate_number,
        "support_assignment": ENVIRONMENT_PLANE_POLICY["support_assignment"],
        "reassignment_rounds": assignment_diagnostics["reassignment_rounds"],
        "overlapping_original_support_pixel_count": assignment_diagnostics["overlapping_original_support_pixel_count"],
        "qualified_proposals_before_assignment": assignment_diagnostics["qualified_proposals_before_assignment"],
        "surviving_proposals_after_assignment": assignment_diagnostics["surviving_proposals_after_assignment"],
        "duplicate_proposals_suppressed": suppressed,
        "ignored_small_or_excess_component_count": ignored_component_count,
        "ignored_small_or_excess_component_points": ignored_component_points,
        "accepted_interior_point_count": int(np.count_nonzero(assigned)),
        "unassigned_valid_point_count": len(result["unassigned_indices"]),
        "all_valid_nearest_plane_rmse_m": global_stats["rmse_m"],
        "all_valid_nearest_plane_p95_m": global_stats["p95_m"],
        "all_valid_nearest_plane_outlier_count": int(np.count_nonzero(nearest > ENVIRONMENT_PLANE_POLICY["ransac_distance_m"])) if nearest is not None else None,
        "all_valid_nearest_plane_outlier_fraction": float(np.mean(nearest > ENVIRONMENT_PLANE_POLICY["ransac_distance_m"])) if nearest is not None and len(nearest) else None,
        "independent_validation": False,
        "normal_diversity_is_solver_responsibility": True,
        "warning": "inlier residuals alone are not accuracy proof; inspect excluded pixels and all-region residuals",
    }
    if not result["planes"]:
        result["reasons"].append("NO_QUALIFIED_LARGE_PLANAR_PATCHES")
    elif frame_error:
        result["status"] = "REJECTED"
    elif all(plane["temporal"]["status"] == "PASS" for plane in result["planes"]):
        result["status"] = "PASS"
    else:
        result["status"] = "CANDIDATE"
        result["reasons"].append("TEMPORAL_REPEATABILITY_NOT_FULLY_EVALUATED")
    return result
