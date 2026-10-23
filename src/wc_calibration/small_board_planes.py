"""Fail-closed controlled-board plane inputs, without estimating extrinsics.

Gates are predeclared engineering starting points, not accuracy certificates.
The frozen foreground diagnostic must be enriched in memory with original
``interior_points``, ``raw_candidate_points`` and ``fixed_interior_frames``.
Their order must match the diagnostic's original pixel indices. No image,
sensor, network, configuration or file is accessed by this module.
"""
from __future__ import annotations

from copy import deepcopy
import math
from typing import Any, Mapping

import numpy as np
from scipy.spatial import ConvexHull, QhullError

from .environment_planes import ENVIRONMENT_PLANE_POLICY, _temporal, _valid_points
from .small_board_diagnostic import _residual_stats, _tls


CONTROLLED_BOARD_POLICY = {
    "label": "EXPERIMENTAL_CONTROLLED_50CM_BOARD_GATES_NOT_ACCURACY_PROOF",
    "minimum_interior_points": 80,
    "minimum_span_m": [0.25, 0.25],
    "minimum_projected_mesh_area_m2": 0.08,
    "area_definition": "sum of projected areas of adjacent original-image triangles with all three vertices in the fixed interior; convex hull is diagnostic only",
    "maximum_interior_rmse_m": 0.020,
    "maximum_interior_p95_m": 0.040,
    "maximum_raw_candidate_p95_m": 0.075,
    "raw_candidate_outlier_distance_m": 0.050,
    "maximum_raw_candidate_outlier_fraction": 0.20,
    "selection": "DIAGNOSTIC_AVAILABLE; one central foreground; full candidate bounds strictly inside image; no partial fallback",
    "maximum_absolute_coordinate_m": 100.0,
    "source_metric_consistency_tolerance": 1e-6,
    "identity_assumption_basis": "OPERATOR_SAME_SINGLE_WOOD_BOARD_FACE",
    "identity": "operator-controlled same physical face is an assumption; automatic selection never proves target identity",
    "temporal": deepcopy(ENVIRONMENT_PLANE_POLICY["temporal"]),
    "temporal_implementation": "frozen environment_planes._temporal; original fixed pixels; reference in-plane axes",
    "absolute_accuracy_estimated": False,
}
_MEMORY_KEYS = {"interior_points", "raw_candidate_points", "fixed_interior_frames"}


def _json_safe(value: Any) -> Any:
    if isinstance(value, Mapping):
        return {str(k): _json_safe(v) for k, v in value.items() if k not in _MEMORY_KEYS}
    if isinstance(value, np.ndarray):
        return _json_safe(value.tolist())
    if isinstance(value, (list, tuple)):
        return [_json_safe(v) for v in value]
    if isinstance(value, (np.bool_, bool)):
        return bool(value)
    if isinstance(value, (np.integer, int)):
        return int(value)
    if isinstance(value, (np.floating, float)):
        return float(value) if math.isfinite(float(value)) else None
    if value is None or isinstance(value, str):
        return value
    return str(value)


def _indices(value: Any, count: int) -> np.ndarray:
    a = np.asarray(value)
    if a.ndim != 1 or a.dtype.kind not in "iu" or a.dtype.kind == "b":
        raise ValueError("INVALID_ORIGINAL_PIXEL_INDICES")
    a = a.astype(np.int64)
    if len(np.unique(a)) != len(a) or np.any(a < 0) or np.any(a >= count):
        raise ValueError("DUPLICATE_OR_OUT_OF_BOUNDS_PIXEL_INDICES")
    return a


def _points(value: Any, count: int) -> np.ndarray:
    a = np.asarray(value, dtype=float)
    if a.shape != (count, 3) or not np.isfinite(a).all():
        raise ValueError("MISSING_OR_INVALID_ORIGINAL_SUPPORT_COORDINATES")
    if np.any(np.abs(a) > CONTROLLED_BOARD_POLICY["maximum_absolute_coordinate_m"]):
        raise ValueError("COORDINATES_OUTSIDE_DECLARED_NUMERICAL_BOUND")
    return a


def _fit_consistent(source: Any, computed: dict[str, Any] | None) -> bool:
    if not isinstance(source, Mapping) or computed is None:
        return False
    try:
        if source.get("residual_trimming_performed") is not False:
            return False
        if source["point_count"] != computed["point_count"]:
            return False
        for key in ("normal", "centroid", "span_m", "d_m", "rmse_m", "p95_m"):
            a, b = np.asarray(source[key], dtype=float), np.asarray(computed[key], dtype=float)
            if a.shape != b.shape or not np.isfinite(a).all() or not np.allclose(a, b,
                    atol=CONTROLLED_BOARD_POLICY["source_metric_consistency_tolerance"], rtol=0):
                return False
        return True
    except (KeyError, TypeError, ValueError):
        return False


def _support_areas(points: np.ndarray, ids: np.ndarray, width: int, height: int,
                   center: np.ndarray, axes: np.ndarray) -> tuple[float, float]:
    projected = (points - center) @ axes
    try:
        hull = float(ConvexHull(projected).volume)
    except QhullError:
        hull = 0.0
    coordinates = np.zeros((height * width, 2))
    coordinates[ids] = projected
    coordinates = coordinates.reshape(height, width, 2)
    mask = np.zeros(height * width, dtype=bool)
    mask[ids] = True
    mask = mask.reshape(height, width)
    total = 0.0
    triangles = [
        (mask[:-1, :-1] & mask[:-1, 1:] & mask[1:, :-1], coordinates[:-1, :-1], coordinates[:-1, 1:], coordinates[1:, :-1]),
        (mask[1:, 1:] & mask[:-1, 1:] & mask[1:, :-1], coordinates[1:, 1:], coordinates[:-1, 1:], coordinates[1:, :-1]),
    ]
    for valid, a, b, c in triangles:
        ab, ac = b-a, c-a
        area = .5 * np.abs(ab[..., 0]*ac[..., 1] - ab[..., 1]*ac[..., 0])
        total += float(np.sum(area[valid]))
    return total, hull


def controlled_board_planes(diagnostic: Any) -> dict[str, Any]:
    """Convert one complete, stable observation into one solver plane input.

    This does not solve calibration or establish physical identity. Missing
    coordinates or fixed-frame evidence are rejected; source ``EVALUATED`` is
    never treated as a temporal PASS. All source diagnostics are preserved,
    except the three potentially large in-memory coordinate arrays.
    """
    result: dict[str, Any] = {
        "schema_version": "controlled_board_planes.v1", "status": "REJECTED",
        "purpose": "CONTROLLED_BOARD_LOCAL_PLANE_INPUT_ONLY", "reasons": [],
        "policy": deepcopy(CONTROLLED_BOARD_POLICY), "planes": [], "rejected_planes": [],
        "physical_target_identity_verified": False, "extrinsic_calibration_evaluated": False,
        "absolute_accuracy_estimated": False, "independent_validation": False,
        "source_diagnostic": _json_safe(diagnostic), "identity_assumption": None,
    }
    plane: dict[str, Any] = {
        "id": "controlled_board_001", "status": "REJECTED", "normal": None,
        "d_m": None, "centroid": None, "span_m": [0., 0.], "point_count": 0,
        "rmse_m": None, "p95_m": None, "indices": [],
        "projected_mesh_area_m2": None, "hull_area_m2": None,
        "all_candidate_residual_to_interior_plane": None,
        "raw_candidate_outlier_count": None, "raw_candidate_outlier_fraction": None,
        "quality": {"status": "REJECTED", "geometrically_qualified": False, "reasons": []},
        "temporal": {"status": "NOT_EVALUATED", "reasons": ["MISSING_FIXED_FRAME_EVIDENCE"],
                     "independent_validation": False},
        "physical_target_identity_verified": False,
        "identity_assumption": None, "reasons": [],
    }
    reasons, geometry_reasons = [], []

    def fail(reason: str) -> None:
        if reason not in reasons:
            reasons.append(reason)

    def geometry_fail(reason: str) -> None:
        if reason not in geometry_reasons:
            geometry_reasons.append(reason)
        fail(reason)

    if not isinstance(diagnostic, Mapping):
        fail("INVALID_DIAGNOSTIC_OBJECT")
    else:
        if diagnostic.get("status") != "DIAGNOSTIC_AVAILABLE":
            fail("SOURCE_DIAGNOSTIC_NOT_AVAILABLE")
        source_reasons = diagnostic.get("reasons", [])
        if isinstance(source_reasons, (list, tuple)):
            for reason in source_reasons:
                fail("SOURCE:" + str(reason))
        else:
            fail("INVALID_SOURCE_REASON_ARRAY")
        selection = diagnostic.get("selection", {})
        if not isinstance(selection, Mapping) or selection.get("status") != "CANDIDATE_SELECTED_IDENTITY_UNVERIFIED":
            fail("COMPLETE_CENTRAL_SELECTION_REQUIRED_NO_PARTIAL_FALLBACK")
        if not isinstance(selection, Mapping):
            selection = {}
        candidates = selection.get("candidates", [])
        if not isinstance(candidates, (list, tuple)):
            candidates = []
            fail("INVALID_SOURCE_CANDIDATE_ARRAY")
        central = [c for c in candidates if isinstance(c, Mapping) and c.get("central") is True]
        if len(central) != 1 or central[0].get("clean_component_id") != selection.get("clean_component_id") or central[0].get("raw_component_id") != selection.get("raw_component_id"):
            fail("UNIQUE_CENTRAL_FOREGROUND_NOT_ESTABLISHED")
        assumption = diagnostic.get("identity_assumption")
        if not isinstance(assumption, Mapping) or assumption.get("basis") != CONTROLLED_BOARD_POLICY["identity_assumption_basis"] or assumption.get("automatic_identity_verified") is not False:
            fail("EXPLICIT_OPERATOR_SAME_BOARD_FACE_ASSUMPTION_REQUIRED")
        result["identity_assumption"] = _json_safe(assumption)
        plane["identity_assumption"] = _json_safe(assumption)
        try:
            w, h = int(diagnostic["width"]), int(diagnostic["height"])
            if w != diagnostic["width"] or h != diagnostic["height"] or w < 3 or h < 3 or w*h > 20000:
                raise ValueError("INVALID_IMAGE_DIMENSIONS")
            ids = _indices(diagnostic.get("interior_indices"), w*h)
            raw_ids = _indices(diagnostic.get("raw_candidate_indices"), w*h)
            xyz = _points(diagnostic.get("interior_points"), len(ids))
            raw_xyz = _points(diagnostic.get("raw_candidate_points"), len(raw_ids))
            plane["indices"] = ids.tolist()
            if not set(ids).issubset(set(raw_ids)):
                raise ValueError("INTERIOR_INDICES_NOT_SUBSET_OF_RAW_CANDIDATE")
            positions = {int(index): position for position, index in enumerate(raw_ids)}
            if not np.allclose(xyz, raw_xyz[[positions[int(index)] for index in ids]], atol=1e-9, rtol=0):
                raise ValueError("RAW_AND_INTERIOR_ORIGINAL_COORDINATES_DISAGREE")
            if not len(raw_ids):
                raise ValueError("EMPTY_RAW_CANDIDATE")
            rows, cols = raw_ids//w, raw_ids % w
            actual_bounds = [int(cols.min()), int(rows.min()), int(cols.max()), int(rows.max())]
            if selection.get("image_bounds_xyxy") != actual_bounds:
                fail("SOURCE_IMAGE_BOUNDS_DISAGREE_WITH_ORIGINAL_INDICES")
            if actual_bounds[0] <= 0 or actual_bounds[1] <= 0 or actual_bounds[2] >= w-1 or actual_bounds[3] >= h-1:
                fail("CANDIDATE_FOUR_IMAGE_BOUNDS_MUST_BE_COMPLETE")
            raw_fit, fit = _tls(raw_xyz), _tls(xyz)
            if not _fit_consistent(diagnostic.get("interior_fit"), fit):
                geometry_fail("INTERIOR_FULL_POINT_TLS_DISAGREES_WITH_SOURCE_OR_UNVERIFIED_TRIMMING")
            if not _fit_consistent(diagnostic.get("raw_candidate_fit"), raw_fit):
                geometry_fail("RAW_CANDIDATE_FULL_POINT_TLS_DISAGREES_WITH_SOURCE")
            if fit is None:
                raise ValueError("TOO_FEW_INTERNAL_POINTS_FOR_TLS")
            for key in ("normal", "d_m", "centroid", "span_m", "point_count", "rmse_m", "p95_m"):
                plane[key] = fit[key]
            plane["residual_trimming_performed"] = False
            center = np.asarray(fit["centroid"])
            _, axes = np.linalg.eigh((xyz-center).T @ (xyz-center) / len(xyz))
            area, hull = _support_areas(xyz, ids, w, h, center, axes[:, [2, 1]])
            plane.update(projected_mesh_area_m2=area, hull_area_m2=hull)
            raw_residuals = raw_xyz @ np.asarray(fit["normal"]) + fit["d_m"]
            raw_stats = _residual_stats(raw_residuals)
            outliers = int(np.count_nonzero(np.abs(raw_residuals) > CONTROLLED_BOARD_POLICY["raw_candidate_outlier_distance_m"]))
            fraction = outliers / len(raw_xyz)
            plane.update(all_candidate_residual_to_interior_plane=raw_stats,
                raw_candidate_outlier_count=outliers, raw_candidate_outlier_fraction=float(fraction),
                raw_candidate_indices=raw_ids.tolist(),
                candidate_envelope_diagnostic=_json_safe({
                    "indices": diagnostic.get("candidate_envelope_indices", []),
                    "all_finite_residual_to_interior_plane": diagnostic.get("all_envelope_residual_to_interior_plane"),
                    "invalid_indices": diagnostic.get("invalid_inside_candidate_envelope_indices", []),
                    "excluded_by_x_band_indices": diagnostic.get("excluded_by_x_band_inside_candidate_envelope_indices", []),
                    "interpretation": "may include background; not a board-plane accuracy metric"}))
            if len(xyz) < CONTROLLED_BOARD_POLICY["minimum_interior_points"]:
                geometry_fail("TOO_FEW_FIXED_INTERIOR_POINTS")
            if np.any(np.asarray(fit["span_m"]) < np.asarray(CONTROLLED_BOARD_POLICY["minimum_span_m"])):
                geometry_fail("INSUFFICIENT_TWO_DIMENSIONAL_INTERIOR_SPAN")
            if area < CONTROLLED_BOARD_POLICY["minimum_projected_mesh_area_m2"]:
                geometry_fail("INSUFFICIENT_PROJECTED_INTERNAL_MESH_AREA")
            if fit["d_m"] <= 1e-6 or abs(np.linalg.norm(fit["normal"])-1) > 1e-6:
                geometry_fail("INVALID_SENSOR_FACING_UNIT_PLANE")
            if fit["rmse_m"] > CONTROLLED_BOARD_POLICY["maximum_interior_rmse_m"]:
                geometry_fail("INTERIOR_RMSE_EXCEEDED")
            if fit["p95_m"] > CONTROLLED_BOARD_POLICY["maximum_interior_p95_m"]:
                geometry_fail("INTERIOR_P95_EXCEEDED")
            if raw_stats["p95_m"] > CONTROLLED_BOARD_POLICY["maximum_raw_candidate_p95_m"]:
                geometry_fail("RAW_CANDIDATE_P95_EXCEEDED")
            if fraction > CONTROLLED_BOARD_POLICY["maximum_raw_candidate_outlier_fraction"]:
                geometry_fail("RAW_CANDIDATE_OUTLIER_FRACTION_EXCEEDED")
            try:
                fixed = np.asarray(diagnostic.get("fixed_interior_frames"), dtype=float)
                if fixed.ndim != 3 or fixed.shape[1:] != xyz.shape:
                    raise ValueError("INVALID_FIXED_INTERIOR_FRAME_ARRAY")
                if np.any(np.abs(fixed[np.isfinite(fixed)]) > CONTROLLED_BOARD_POLICY["maximum_absolute_coordinate_m"]):
                    raise ValueError("FIXED_FRAME_COORDINATES_OUTSIDE_NUMERICAL_BOUND")
                source_temporal = diagnostic.get("temporal", {})
                if not isinstance(source_temporal, Mapping):
                    source_temporal = {}
                if source_temporal.get("status") != "EVALUATED" or source_temporal.get("provided_frame_count") != len(fixed):
                    fail("SOURCE_TEMPORAL_EVIDENCE_DISAGREES_WITH_FIXED_FRAME_INPUT")
                temporal = _temporal(xyz, np.arange(len(xyz)), np.asarray(fit["normal"]),
                    fit["d_m"], center, fixed, None)
                temporal["d_parameter_variation_is_physical_motion_proof"] = False
                temporal["implementation"] = CONTROLLED_BOARD_POLICY["temporal_implementation"]
                for frame in temporal["frames"]:
                    valid, finite, _ = _valid_points(fixed[frame["frame_index"]])
                    frame["invalid_original_pixel_indices"] = ids[~finite].tolist()
                    frame["range_filtered_original_pixel_indices"] = ids[finite & ~valid].tolist()
                    frame["d_parameter_delta_m"] = frame.pop("offset_delta_m")
                    frame["residual_trimming_performed"] = False
                temporal["d_parameter_span_m"] = temporal.pop("offset_span_m")
                plane["temporal"] = temporal
                if temporal["status"] != "PASS":
                    fail("FIXED_PIXEL_TEMPORAL_GATES_NOT_PASSED")
                    for reason in temporal["reasons"]:
                        fail("TEMPORAL:" + reason)
            except (TypeError, ValueError) as error:
                fail(str(error) or "INVALID_FIXED_INTERIOR_FRAME_EVIDENCE")
        except (KeyError, TypeError, ValueError, OverflowError, np.linalg.LinAlgError) as error:
            geometry_fail(str(error) or "INVALID_OR_MISSING_ORIGINAL_COORDINATE_EVIDENCE")
    geometrically_qualified = not geometry_reasons and plane["normal"] is not None
    plane["quality"] = {"status": "PASS" if geometrically_qualified else "REJECTED",
        "geometrically_qualified": geometrically_qualified,
        "reasons": geometry_reasons, "policy": deepcopy(CONTROLLED_BOARD_POLICY)}
    plane["reasons"] = reasons
    result["reasons"] = reasons
    if not reasons:
        plane["status"] = "PASS"
        result["status"] = "PASS"
        result["planes"] = [_json_safe(plane)]
    else:
        result["rejected_planes"] = [_json_safe(plane)]
    return result
