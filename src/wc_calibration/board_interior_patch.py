"""Controlled board-interior support, without claiming to see a complete board.

An inclusive image rectangle is an operator's same-face identity assumption.
It is not a point correspondence, a corner measurement, or proof of target
identity. Every finite raw ROI point participates in its diagnostic TLS fit.
Fixed fitting support is selected only by one-pixel ROI erosion, finite data,
and the existing depth-edge detector; residuals never choose or trim points.
"""
from __future__ import annotations

from copy import deepcopy
import math
from typing import Any, Mapping

import numpy as np
from scipy import ndimage

from .environment_planes import ENVIRONMENT_PLANE_POLICY, _temporal, _valid_points
from .small_board_diagnostic import _depth_edges, _residual_stats, _tls
from .small_board_planes import CONTROLLED_BOARD_POLICY, _support_areas


INTERIOR_PATCH_POLICY = deepcopy(CONTROLLED_BOARD_POLICY)
INTERIOR_PATCH_POLICY.update({
    "label": "EXPERIMENTAL_OPERATOR_CONTROLLED_BOARD_INTERIOR_NOT_ACCURACY_PROOF",
    "selection": "inclusive operator ROI wholly inside the same physical board face; no complete-board claim",
    "roi_format": "[x_min, y_min, x_max, y_max], inclusive integer original pixels",
    "interior_erosion_pixels": 1,
    "interior_support": "one-pixel ROI erosion intersect finite pixels, excluding existing diagnostic depth edges",
    "fitting": "TLS over every finite point in each declared support; no residual trimming or RANSAC",
    "identity_assumption_basis": "OPERATOR_ROI_WHOLELY_INSIDE_SAME_BOARD_FACE",
    "context_residual_gating": "outside-ROI ring and whole-image residuals are context only, never board accuracy gates",
    "maximum_points": 20000,
    "board_completeness_verified": False,
})
_CROSS = ndimage.generate_binary_structure(2, 1)


def _safe(value: Any) -> Any:
    if isinstance(value, Mapping):
        return {str(k): _safe(v) for k, v in value.items()}
    if isinstance(value, np.ndarray):
        return _safe(value.tolist())
    if isinstance(value, (tuple, list)):
        return [_safe(v) for v in value]
    if isinstance(value, (np.bool_, bool)):
        return bool(value)
    if isinstance(value, (np.integer, int)):
        return int(value)
    if isinstance(value, (np.floating, float)):
        return float(value) if math.isfinite(float(value)) else None
    if isinstance(value, str) or value is None:
        return value
    return str(value)


def _residual_record(points: np.ndarray, ids: np.ndarray,
                     normal: np.ndarray, d: float) -> dict[str, Any]:
    finite = np.isfinite(points[ids]).all(axis=1)
    finite_ids = ids[finite]
    signed = points[finite_ids] @ normal + d
    return {"original_indices": ids.tolist(), "finite_indices": finite_ids.tolist(),
            "invalid_indices": ids[~finite].tolist(), "signed_residuals_m": signed.tolist(),
            "absolute_residuals_m": np.abs(signed).tolist(), "statistics": _residual_stats(signed),
            "residual_trimming_performed": False}


def controlled_interior_patch(cloud: Any, *, frames: Any, width: int, height: int,
                              roi_xyxy: Any, operator_confirmed: bool) -> dict[str, Any]:
    """Return a compatible offline plane only after all fixed-support gates pass.

    ``roi_xyxy`` contains inclusive integer bounds in the original organized
    image. It may touch image bounds: the operator is declaring an interior
    patch, never declaring that all physical board edges are visible. Unconfirmed
    observations still receive diagnostics, but never return qualified planes.
    """
    reasons, geometry_reasons = [], []
    assumption = {"basis": INTERIOR_PATCH_POLICY["identity_assumption_basis"],
        "operator_confirmed": operator_confirmed is True,
        "automatic_identity_verified": False, "board_completeness_verified": False,
        "statement": "Every pixel in the coarse rectangle is asserted to lie on the same physical board face."}
    diagnostic: dict[str, Any] = {
        "status": "REJECTED", "reasons": reasons, "width": None, "height": None,
        "roi_xyxy": None, "roi_format": INTERIOR_PATCH_POLICY["roi_format"],
        "coordinate_frame": "SDK_FLU", "units": "m", "identity_assumption": assumption,
        "physical_target_identity_verified": False, "board_completeness_verified": False,
        "residual_trimming_performed": False, "raw_roi_indices": [], "raw_roi_finite_indices": [],
        "raw_roi_invalid_indices": [], "raw_roi_points_m": [], "raw_roi_fit": None,
        "interior_indices": [], "interior_points_m": [], "interior_fit": None,
        "roi_erosion_excluded_indices": [], "roi_depth_edge_excluded_indices": [],
        "roi_invalid_excluded_indices": [], "raw_depth_edge_indices": [], "depth_edge_indices": [],
        "outside_ring_indices": [], "invalid_indices": [],
        "raw_roi_residual_to_interior_plane": None,
        "outside_ring_residual_to_interior_plane": None,
        "whole_image_residual_to_interior_plane": None,
        "context_is_board_accuracy_gate": False,
        "temporal": {"status": "NOT_EVALUATED", "reasons": ["NO_INTERIOR_FIT"],
                     "independent_validation": False},
        "policy": deepcopy(INTERIOR_PATCH_POLICY),
    }
    result: dict[str, Any] = {"schema_version": "controlled_board_interior_patch.v1",
        "status": "REJECTED", "reasons": reasons, "planes": [], "rejected_planes": [],
        "diagnostic": diagnostic, "physical_target_identity_verified": False,
        "board_completeness_verified": False, "identity_assumption": assumption,
        "absolute_accuracy_estimated": False, "independent_validation": False,
        "extrinsic_calibration_evaluated": False, "live_eligible": False}

    def fail(reason: str, *, geometry: bool = False) -> None:
        if reason not in reasons:
            reasons.append(reason)
        if geometry and reason not in geometry_reasons:
            geometry_reasons.append(reason)

    if operator_confirmed is not True:
        fail("OPERATOR_ROI_SAME_BOARD_FACE_NOT_CONFIRMED")
    try:
        if (not isinstance(width, (int, np.integer)) or isinstance(width, bool)
                or not isinstance(height, (int, np.integer)) or isinstance(height, bool)
                or width < 3 or height < 3 or width*height > INTERIOR_PATCH_POLICY["maximum_points"]):
            raise ValueError("INVALID_OR_EXCESSIVE_IMAGE_DIMENSIONS")
        w, h = int(width), int(height)
        xyz = np.asarray(cloud, dtype=float)
        if xyz.shape != (w*h, 3):
            raise ValueError("INVALID_ORGANIZED_CLOUD_SHAPE")
        if np.any(np.abs(xyz[np.isfinite(xyz)]) > INTERIOR_PATCH_POLICY["maximum_absolute_coordinate_m"]):
            raise ValueError("COORDINATES_OUTSIDE_DECLARED_NUMERICAL_BOUND")
        roi = np.asarray(roi_xyxy)
        if roi.shape != (4,) or roi.dtype.kind not in "iu" or roi.dtype.kind == "b":
            raise ValueError("ROI_REQUIRES_FOUR_INCLUSIVE_INTEGER_PIXEL_BOUNDS")
        x0, y0, x1, y1 = [int(v) for v in roi]
        if not (0 <= x0 < x1 < w and 0 <= y0 < y1 < h):
            raise ValueError("ROI_BOUNDS_OUTSIDE_IMAGE_OR_EMPTY")
    except (TypeError, ValueError, OverflowError) as error:
        fail(str(error) or "INVALID_PATCH_INPUT", geometry=True)
        return _safe(result)
    diagnostic.update(width=w, height=h, roi_xyxy=[x0, y0, x1, y1])
    finite = np.isfinite(xyz).all(axis=1)
    diagnostic["invalid_indices"] = np.flatnonzero(~finite).tolist()
    roi_mask = np.zeros((h, w), dtype=bool)
    roi_mask[y0:y1+1, x0:x1+1] = True
    eroded = ndimage.binary_erosion(roi_mask, structure=_CROSS, iterations=1, border_value=0)
    raw_edge, edge = _depth_edges(xyz, finite, h, w)
    interior_mask = eroded & finite.reshape(h, w) & ~edge
    raw_ids, ids = np.flatnonzero(roi_mask), np.flatnonzero(interior_mask)
    raw_finite_ids = raw_ids[finite[raw_ids]]
    ring = ndimage.binary_dilation(roi_mask, structure=np.ones((3, 3), bool), iterations=1) & ~roi_mask
    ring_ids = np.flatnonzero(ring)
    diagnostic.update(raw_roi_indices=raw_ids.tolist(), raw_roi_finite_indices=raw_finite_ids.tolist(),
        raw_roi_invalid_indices=raw_ids[~finite[raw_ids]].tolist(), raw_roi_points_m=xyz[raw_ids],
        interior_indices=ids.tolist(), interior_points_m=xyz[ids],
        roi_erosion_excluded_indices=np.flatnonzero(roi_mask & ~eroded).tolist(),
        roi_depth_edge_excluded_indices=np.flatnonzero(roi_mask & edge).tolist(),
        roi_invalid_excluded_indices=raw_ids[~finite[raw_ids]].tolist(),
        raw_depth_edge_indices=np.flatnonzero(raw_edge).tolist(), depth_edge_indices=np.flatnonzero(edge).tolist(),
        outside_ring_indices=ring_ids.tolist())
    raw_fit, fit = _tls(xyz[raw_finite_ids]), _tls(xyz[ids])
    diagnostic.update(raw_roi_fit=raw_fit, interior_fit=fit)
    if len(ids) < INTERIOR_PATCH_POLICY["minimum_interior_points"]:
        fail("TOO_FEW_FIXED_INTERIOR_POINTS", geometry=True)
    if fit is None:
        fail("NO_INTERIOR_FULL_POINT_TLS", geometry=True)
        return _safe(result)
    plane = {key: fit[key] for key in ("normal", "d_m", "centroid", "span_m", "point_count", "rmse_m", "p95_m")}
    plane.update(id="controlled_interior_patch_001", status="REJECTED", indices=ids.tolist(),
        physical_target_identity_verified=False, board_completeness_verified=False,
        identity_assumption=assumption, residual_trimming_performed=False,
        raw_candidate_indices=raw_ids.tolist(), policy=deepcopy(INTERIOR_PATCH_POLICY))
    n, d, center = np.asarray(fit["normal"]), float(fit["d_m"]), np.asarray(fit["centroid"])
    _, axes = np.linalg.eigh((xyz[ids]-center).T @ (xyz[ids]-center) / len(ids))
    area, hull = _support_areas(xyz[ids], ids, w, h, center, axes[:, [2, 1]])
    diagnostic.update(projected_mesh_area_m2=area, hull_area_m2=hull)
    plane.update(projected_mesh_area_m2=area, hull_area_m2=hull)
    roi_residual = _residual_record(xyz, raw_ids, n, d)
    diagnostic["raw_roi_residual_to_interior_plane"] = roi_residual
    diagnostic["outside_ring_residual_to_interior_plane"] = _residual_record(xyz, ring_ids, n, d)
    diagnostic["whole_image_residual_to_interior_plane"] = _residual_record(xyz, np.arange(w*h), n, d)
    raw_signed = np.asarray(roi_residual["signed_residuals_m"])
    outlier_count = int(np.count_nonzero(np.abs(raw_signed) > INTERIOR_PATCH_POLICY["raw_candidate_outlier_distance_m"]))
    outlier_fraction = outlier_count / len(raw_signed) if len(raw_signed) else 1.0
    plane.update(all_candidate_residual_to_interior_plane=roi_residual["statistics"],
        raw_candidate_outlier_count=outlier_count, raw_candidate_outlier_fraction=outlier_fraction)
    diagnostic.update(raw_roi_outlier_count=outlier_count, raw_roi_outlier_fraction=outlier_fraction)
    if np.any(np.asarray(fit["span_m"]) < np.asarray(INTERIOR_PATCH_POLICY["minimum_span_m"])):
        fail("INSUFFICIENT_TWO_DIMENSIONAL_INTERIOR_SPAN", geometry=True)
    if area < INTERIOR_PATCH_POLICY["minimum_projected_mesh_area_m2"]:
        fail("INSUFFICIENT_PROJECTED_INTERNAL_MESH_AREA", geometry=True)
    if d <= 1e-6 or abs(float(np.linalg.norm(n))-1) > 1e-6:
        fail("INVALID_SENSOR_FACING_UNIT_PLANE", geometry=True)
    if fit["rmse_m"] > INTERIOR_PATCH_POLICY["maximum_interior_rmse_m"]:
        fail("INTERIOR_RMSE_EXCEEDED", geometry=True)
    if fit["p95_m"] > INTERIOR_PATCH_POLICY["maximum_interior_p95_m"]:
        fail("INTERIOR_P95_EXCEEDED", geometry=True)
    if roi_residual["statistics"]["p95_m"] is None or roi_residual["statistics"]["p95_m"] > INTERIOR_PATCH_POLICY["maximum_raw_candidate_p95_m"]:
        fail("RAW_ROI_P95_EXCEEDED", geometry=True)
    if outlier_fraction > INTERIOR_PATCH_POLICY["maximum_raw_candidate_outlier_fraction"]:
        fail("RAW_ROI_OUTLIER_FRACTION_EXCEEDED", geometry=True)

    sequence, frame_error = None, None
    if frames is not None:
        try:
            sequence = np.asarray(frames, dtype=float)
            if sequence.ndim != 3 or sequence.shape[1:] != xyz.shape:
                raise ValueError("INVALID_TEMPORAL_FRAME_SHAPE")
            if np.isinf(sequence).any():
                raise ValueError("INFINITE_TEMPORAL_FRAME_COORDINATES")
            if np.any(np.abs(sequence[np.isfinite(sequence)]) > INTERIOR_PATCH_POLICY["maximum_absolute_coordinate_m"]):
                raise ValueError("FRAME_COORDINATES_OUTSIDE_DECLARED_NUMERICAL_BOUND")
        except (TypeError, ValueError, OverflowError) as error:
            sequence, frame_error = None, str(error) or "INVALID_TEMPORAL_FRAMES"
    temporal = _temporal(xyz, ids, n, d, center, sequence, frame_error)
    temporal["support_original_indices"] = ids.tolist()
    temporal["residual_trimming_performed"] = False
    temporal["d_parameter_variation_is_physical_motion_proof"] = False
    temporal["implementation"] = "environment_planes._temporal; unchanged policy; fixed original interior pixels"
    for row in temporal["frames"]:
        valid, frame_finite, _ = _valid_points(sequence[row["frame_index"], ids])
        row["invalid_original_pixel_indices"] = ids[~frame_finite].tolist()
        row["range_filtered_original_pixel_indices"] = ids[frame_finite & ~valid].tolist()
        row["d_parameter_delta_m"] = row.pop("offset_delta_m")
        row["residual_trimming_performed"] = False
    temporal["d_parameter_span_m"] = temporal.pop("offset_span_m")
    diagnostic["temporal"] = temporal
    plane["temporal"] = temporal
    if temporal["status"] != "PASS":
        fail("FIXED_PIXEL_TEMPORAL_GATES_NOT_PASSED")
        for reason in temporal["reasons"]:
            fail("TEMPORAL:" + reason)
    plane["quality"] = {"status": "PASS" if not geometry_reasons else "REJECTED",
        "geometrically_qualified": not geometry_reasons, "reasons": geometry_reasons,
        "policy": deepcopy(INTERIOR_PATCH_POLICY)}
    plane["reasons"] = reasons
    diagnostic["status"] = "DIAGNOSTIC_AVAILABLE"
    if not reasons:
        plane["status"], result["status"] = "PASS", "PASS"
        result["planes"] = [plane]
    else:
        result["rejected_planes"] = [plane]
    return _safe(result)
