"""Isolated, offline diagnostics of one central foreground planar observation.

This module never identifies a physical board or estimates extrinsics. All
coordinates are SDK FLU metres and all indices are original row-major pixels.
TLS uses every finite point in the stated support, without residual trimming.
"""
from __future__ import annotations

from copy import deepcopy
import json
import math
from typing import Any, Mapping

import numpy as np
from scipy import ndimage


SMALL_BOARD_DIAGNOSTIC_POLICY = {
    "label": "EXPERIMENTAL_LOCAL_OBSERVATION_DIAGNOSTIC_NOT_ACCURACY_PROOF",
    "forward_x_band_m": [0.6, 1.6],
    "allowed_selection_x_band_bounds_m": [0.25, 12.0],
    "selection_parameters": "explicit caller distance band and optional coarse image ROI; provenance is recorded and does not verify target identity",
    "selection_truncation": "continuous support beyond a distance band, or candidate contact with a coarse ROI boundary, is UNKNOWN; no synthetic board boundary",
    "minimum_clean_component_points": 100,
    "minimum_interior_points": 80,
    "central_image_fraction_bounds": [[0.25, 0.75], [0.20, 0.80]],
    "compatible_span_5_95_m": [0.20, 0.80],
    "edge_absolute_range_jump_m": 0.08,
    "edge_relative_range_jump": 0.03,
    "edge_dilation_pixels": 1,
    "interior_erosion_pixels": 1,
    "maximum_points": 20000,
    "maximum_evaluated_frames": 64,
    "minimum_frame_valid_fraction": 0.80,
    "warning_rmse_m": 0.025,
    "warning_p95_m": 0.050,
    "warning_reference_p95_m": 0.060,
    "warning_normal_angle_deg": 2.0,
    "warning_absolute_d_parameter_delta_m": 0.040,
    "fitting": "all finite support points, TLS, no RANSAC and no residual trimming",
    "span_definition": "5th-to-95th percentile and full extent along TLS in-plane principal axes",
    "target_identity": "a unique compatible central foreground component is not verified physical board identity",
    "partial_fallback": "one unique large off-center foreground may be inspected locally, always UNKNOWN and never identity-verified",
    "envelope_definition": "raw candidate image bounding box; may contain background and missing points",
    "d_parameter_interpretation": "parameter variation only; not proof of physical target or sensor movement",
}
_CROSS = ndimage.generate_binary_structure(2, 1)


def _residual_stats(values: np.ndarray) -> dict[str, Any]:
    if not len(values):
        return {"point_count": 0, "rmse_m": None, "p95_m": None,
                "maximum_absolute_m": None, "signed_mean_m": None,
                "signed_minimum_m": None, "signed_maximum_m": None}
    return {"point_count": int(len(values)),
            "rmse_m": float(np.sqrt(np.mean(values ** 2))),
            "p95_m": float(np.quantile(np.abs(values), 0.95)),
            "maximum_absolute_m": float(np.max(np.abs(values))),
            "signed_mean_m": float(np.mean(values)),
            "signed_minimum_m": float(np.min(values)),
            "signed_maximum_m": float(np.max(values))}


def _tls(points: np.ndarray) -> dict[str, Any] | None:
    if len(points) < 3:
        return None
    center = np.mean(points, axis=0)
    delta = points - center
    eigenvalues, axes = np.linalg.eigh(delta.T @ delta / len(points))
    normal = axes[:, 0].copy()
    if float(normal @ center) > 0:
        normal *= -1.0
    d = -float(normal @ center)
    coordinates = delta @ axes[:, [2, 1]]
    spans = np.sort(np.quantile(coordinates, .95, axis=0)
                    - np.quantile(coordinates, .05, axis=0))[::-1]
    full_spans = np.sort(np.ptp(coordinates, axis=0))[::-1]
    result = {"normal": normal.tolist(), "d_m": d, "centroid": center.tolist(),
              "point_count": int(len(points)), "span_m": spans.tolist(),
              "full_span_m": full_spans.tolist(),
              "covariance_eigenvalues_m2": np.maximum(eigenvalues, 0).tolist(),
              "equation": "normal.dot(point) + d_m = 0",
              "residual_trimming_performed": False}
    result.update(_residual_stats(points @ normal + d))
    result["orientation_defined"] = d > 1e-6
    return result


def _depth_edges(points: np.ndarray, finite: np.ndarray, height: int, width: int) -> tuple[np.ndarray, np.ndarray]:
    valid = finite.reshape(height, width)
    ranges = np.full(len(points), np.nan)
    ranges[finite] = np.linalg.norm(points[finite], axis=1)
    ranges = ranges.reshape(height, width)
    raw = np.zeros(valid.shape, bool)
    for axis in (0, 1):
        lo = [slice(None), slice(None)]
        hi = [slice(None), slice(None)]
        lo[axis], hi[axis] = slice(None, -1), slice(1, None)
        lo, hi = tuple(lo), tuple(hi)
        with np.errstate(invalid="ignore"):
            threshold = SMALL_BOARD_DIAGNOSTIC_POLICY["edge_absolute_range_jump_m"] + SMALL_BOARD_DIAGNOSTIC_POLICY["edge_relative_range_jump"] * np.minimum(ranges[lo], ranges[hi])
            jump = valid[lo] & valid[hi] & (np.abs(ranges[lo] - ranges[hi]) > threshold)
        missing_boundary = valid[lo] != valid[hi]
        raw[lo] |= (jump | missing_boundary) & valid[lo]
        raw[hi] |= (jump | missing_boundary) & valid[hi]
    expanded = ndimage.binary_dilation(raw, structure=_CROSS,
        iterations=SMALL_BOARD_DIAGNOSTIC_POLICY["edge_dilation_pixels"])
    return raw, expanded & valid


def _temporal(frames: np.ndarray | None, frame_error: str | None, indices: np.ndarray,
              reference: dict[str, Any] | None,
              selection_x_band_m: tuple[float, float] = (.6, 1.6)) -> dict[str, Any]:
    result: dict[str, Any] = {
        "status": "NOT_EVALUATED", "reasons": [], "provided_frame_count": 0,
        "evaluated_frame_count": 0, "fixed_pixel_support": True,
        "support_indices": indices.tolist(), "independent_validation": False,
        "d_parameter_variation_is_physical_motion_proof": False,
        "d_parameter_span_m": None, "maximum_normal_angle_deg": None,
        "maximum_reference_plane_p95_m": None, "invalid_frame_indices": [],
        "warning_frame_indices": [], "frames": [],
        "interpretation": SMALL_BOARD_DIAGNOSTIC_POLICY["d_parameter_interpretation"],
    }
    if frame_error:
        result.update(status="REJECTED", reasons=[frame_error])
        return result
    if frames is None or not len(frames):
        result["reasons"] = ["NO_TEMPORAL_FRAMES"]
        return result
    result["provided_frame_count"] = int(len(frames))
    if reference is None or len(indices) < 3:
        result["reasons"] = ["NO_FIXED_INTERIOR_REFERENCE"]
        return result
    take = np.unique(np.linspace(0, len(frames)-1,
        min(len(frames), SMALL_BOARD_DIAGNOSTIC_POLICY["maximum_evaluated_frames"]), dtype=int))
    normal = np.asarray(reference["normal"])
    d = reference["d_m"]
    parameters, angles, reference_p95 = [], [], []
    for frame_index in take:
        fixed = frames[frame_index, indices]
        finite = np.isfinite(fixed).all(axis=1)
        xyz = fixed[finite]
        fit = _tls(xyz)
        reasons = []
        row: dict[str, Any] = {
            "frame_index": int(frame_index), "status": "VALID", "reasons": reasons,
            "finite_point_count": int(len(xyz)), "valid_fraction": float(np.mean(finite)),
            "invalid_pixel_indices": indices[~finite].tolist(),
            "finite_points_outside_selection_x_band": int(np.count_nonzero((xyz[:, 0] < selection_x_band_m[0]) | (xyz[:, 0] > selection_x_band_m[1]))),
            "fit": fit, "normal_angle_deg": None, "d_parameter_delta_m": None,
            "reference_plane_residual": None,
            "residual_trimming_performed": False,
        }
        if len(xyz) < SMALL_BOARD_DIAGNOSTIC_POLICY["minimum_interior_points"] or row["valid_fraction"] < SMALL_BOARD_DIAGNOSTIC_POLICY["minimum_frame_valid_fraction"]:
            reasons.append("FIXED_SUPPORT_MISSING_OR_TOO_FEW_FINITE_POINTS")
            row["status"] = "INVALID"
        if fit is not None:
            n_frame = np.asarray(fit["normal"])
            if float(n_frame @ normal) < 0:
                n_frame *= -1
                fit["normal"] = n_frame.tolist()
                fit["d_m"] *= -1
            angle = math.degrees(math.acos(float(np.clip(n_frame @ normal, -1, 1))))
            delta = float(fit["d_m"] - d)
            stats = _residual_stats(xyz @ normal + d)
            row.update(normal_angle_deg=angle, d_parameter_delta_m=delta,
                reference_plane_residual=stats)
            parameters.append(fit["d_m"])
            angles.append(angle)
            reference_p95.append(stats["p95_m"])
            if fit["rmse_m"] > SMALL_BOARD_DIAGNOSTIC_POLICY["warning_rmse_m"]:
                reasons.append("FIXED_SUPPORT_RMSE_WARNING")
            if fit["p95_m"] > SMALL_BOARD_DIAGNOSTIC_POLICY["warning_p95_m"]:
                reasons.append("FIXED_SUPPORT_P95_WARNING")
            if stats["p95_m"] > SMALL_BOARD_DIAGNOSTIC_POLICY["warning_reference_p95_m"]:
                reasons.append("REFERENCE_PLANE_RESIDUAL_WARNING")
            if angle > SMALL_BOARD_DIAGNOSTIC_POLICY["warning_normal_angle_deg"]:
                reasons.append("NORMAL_PARAMETER_VARIATION_WARNING")
            if abs(delta) > SMALL_BOARD_DIAGNOSTIC_POLICY["warning_absolute_d_parameter_delta_m"]:
                reasons.append("D_PARAMETER_VARIATION_WARNING_NOT_MOTION_PROOF")
        elif "FIXED_SUPPORT_MISSING_OR_TOO_FEW_FINITE_POINTS" not in reasons:
            reasons.append("NO_FINITE_PLANE_FIT")
            row["status"] = "INVALID"
        if reasons and row["status"] != "INVALID":
            row["status"] = "WARNING"
        if row["status"] == "INVALID":
            result["invalid_frame_indices"].append(int(frame_index))
        elif row["status"] == "WARNING":
            result["warning_frame_indices"].append(int(frame_index))
        result["frames"].append(row)
    result.update(status="EVALUATED", evaluated_frame_count=len(take),
        d_parameter_span_m=float(np.ptp(parameters)) if parameters else None,
        maximum_normal_angle_deg=max(angles) if angles else None,
        maximum_reference_plane_p95_m=max(reference_p95) if reference_p95 else None)
    return result


def _selection_parameters(x_band: Any, roi: Any, provenance: Any,
                          width: int, height: int) -> dict[str, Any]:
    try:
        band = np.asarray(x_band)
        if band.shape != (2,) or band.dtype.kind not in "fiu" or band.dtype.kind == "b":
            raise ValueError
        band = band.astype(float)
        lower, upper = SMALL_BOARD_DIAGNOSTIC_POLICY["allowed_selection_x_band_bounds_m"]
        if not np.isfinite(band).all() or not lower <= band[0] < band[1] <= upper:
            raise ValueError
    except (TypeError, ValueError, OverflowError):
        raise ValueError("INVALID_SELECTION_X_BAND") from None
    bounds = None
    if roi is not None:
        try:
            a = np.asarray(roi)
            if a.shape != (4,) or a.dtype.kind not in "iu" or a.dtype.kind == "b":
                raise ValueError
            x0, y0, x1, y1 = (int(v) for v in a)
            if not (0 <= x0 < x1 < width and 0 <= y0 < y1 < height):
                raise ValueError
            bounds = [x0, y0, x1, y1]
        except (TypeError, ValueError, OverflowError):
            raise ValueError("INVALID_SELECTION_ROI") from None
    if provenance is None:
        provenance = {"basis": "BUILT_IN_DEFAULT_SELECTION" if bounds is None and np.array_equal(band, [.6, 1.6])
                      else "EXPLICIT_CALLER_SELECTION_PARAMETERS_SOURCE_UNSPECIFIED"}
    elif not isinstance(provenance, (str, Mapping)) or (isinstance(provenance, str) and not provenance.strip()):
        raise ValueError("INVALID_SELECTION_PROVENANCE")
    try:
        # Normalize only strict JSON data. Provenance is evidence about the
        # caller's choice, never evidence of automatic physical identity.
        provenance = json.loads(json.dumps(provenance, allow_nan=False))
    except (TypeError, ValueError, OverflowError):
        raise ValueError("INVALID_SELECTION_PROVENANCE") from None
    return {"x_band_m": band.tolist(), "roi_xyxy": bounds,
            "provenance": provenance, "automatic_target_identity_verified": False}


def _continuous_band_boundary(candidate: np.ndarray, finite: np.ndarray,
                              foreground: np.ndarray, raw_edges: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Retain artificial distance-cut borders without fitting a plane to decide."""
    inside, outside = np.zeros(candidate.shape, bool), np.zeros(candidate.shape, bool)
    for axis in (0, 1):
        lo, hi = [slice(None), slice(None)], [slice(None), slice(None)]
        lo[axis], hi[axis] = slice(None, -1), slice(1, None)
        lo, hi = tuple(lo), tuple(hi)
        for a, b in ((lo, hi), (hi, lo)):
            crossing = candidate[a] & finite[b] & ~foreground[b] & ~raw_edges[a] & ~raw_edges[b]
            inside[a] |= crossing
            outside[b] |= crossing
    return inside, outside


def diagnose_small_board(cloud: Any, *, frames: Any, width: int, height: int,
                         selection_x_band_m: Any = (.6, 1.6),
                         selection_roi_xyxy: Any = None,
                         selection_provenance: Any = None) -> dict[str, Any]:
    """Diagnose a unique central foreground observation, without board identity.

    ``cloud`` is N x 3, ``frames`` is optionally K x N x 3, N=width*height.
    Selection defaults to X in [.6, 1.6] m, original image connectivity and range
    edges. Explicit distance/ROI choices and their provenance are recorded.
    ROI bounds are inclusive original pixel coordinates. Artificial truncation
    remains UNKNOWN, including connected foreground outside the selected ROI.
    Residuals from raw candidate pixels AND its complete image envelope remain
    visible, including finite points excluded by selection and missing pixels.
    """
    result: dict[str, Any] = {
        "schema_version": "small_board_diagnostic.v1", "status": "REJECTED", "reasons": [],
        "purpose": "LOCAL_PLANE_OBSERVATION_DIAGNOSTIC_ONLY",
        "coordinate_frame": "SDK_FLU", "units": "m", "width": None, "height": None,
        "policy": deepcopy(SMALL_BOARD_DIAGNOSTIC_POLICY),
        "selection_parameters": None,
        "physical_target_identity_verified": False, "extrinsic_calibration_evaluated": False,
        "preprocessing": {}, "selection": {"status": "NOT_SELECTED", "candidates": []},
        "raw_candidate_indices": [], "interior_indices": [], "candidate_envelope_indices": [],
        "edge_indices": [], "raw_edge_indices": [], "invalid_indices": [],
        "invalid_inside_candidate_envelope_indices": [],
        "excluded_by_x_band_inside_candidate_envelope_indices": [],
        "edge_points_inside_candidate_indices": [],
        "selection_band_boundary_indices": [], "selection_band_excluded_adjacent_indices": [],
        "selection_roi_boundary_indices": [], "outside_roi_same_component_indices": [],
        "full_connected_foreground_indices": [], "full_connected_foreground_fit": None,
        "full_connected_foreground_residual_to_interior_plane": None,
        "outside_roi_same_component_residual_to_interior_plane": None,
        "raw_candidate_fit": None, "interior_fit": None,
        "all_candidate_residual_to_interior_plane": None,
        "all_envelope_residual_to_interior_plane": None,
        "temporal": {"status": "NOT_EVALUATED", "independent_validation": False},
        "limitations": ["Automatic selection does not establish physical board identity.",
            "Repeated frames are not independent calibration validation.",
            "Plane parameter variation does not prove physical movement.",
            "The candidate envelope may contain background; its residuals are diagnostic."],
    }
    try:
        w, h = int(width), int(height)
        if isinstance(width, bool) or isinstance(height, bool) or w != width or h != height or w < 3 or h < 3:
            raise ValueError
        xyz = np.asarray(cloud, dtype=float)
    except (TypeError, ValueError, OverflowError):
        result["reasons"] = ["INVALID_CLOUD_OR_IMAGE_DIMENSIONS"]
        return result
    result.update(width=w, height=h)
    if w*h > SMALL_BOARD_DIAGNOSTIC_POLICY["maximum_points"] or xyz.shape != (w*h, 3):
        result["reasons"] = ["INVALID_OR_EXCESSIVE_ORGANIZED_POINT_ARRAY"]
        return result
    try:
        parameters = _selection_parameters(selection_x_band_m, selection_roi_xyxy,
                                            selection_provenance, w, h)
    except ValueError as error:
        result["reasons"] = [str(error)]
        return result
    result["selection_parameters"] = parameters
    result["policy"]["forward_x_band_m"] = parameters["x_band_m"]
    band_min, band_max = parameters["x_band_m"]
    roi_mask = np.ones((h, w), dtype=bool)
    roi_boundary = np.zeros((h, w), dtype=bool)
    if parameters["roi_xyxy"] is not None:
        x0, y0, x1, y1 = parameters["roi_xyxy"]
        roi_mask[:] = False
        roi_mask[y0:y1+1, x0:x1+1] = True
        roi_boundary[y0, x0:x1+1] = roi_boundary[y1, x0:x1+1] = True
        roi_boundary[y0:y1+1, x0] = roi_boundary[y0:y1+1, x1] = True
    sequence, frame_error = None, None
    if frames is not None:
        try:
            sequence = np.asarray(frames, dtype=float)
            if sequence.ndim != 3 or sequence.shape[1:] != xyz.shape:
                sequence, frame_error = None, "INVALID_TEMPORAL_FRAME_SHAPE"
        except (TypeError, ValueError):
            frame_error = "INVALID_TEMPORAL_FRAME_ARRAY"
    finite = np.isfinite(xyz).all(axis=1)
    foreground = finite & (xyz[:, 0] >= band_min) & (xyz[:, 0] <= band_max)
    raw_edges, edges = _depth_edges(xyz, finite, h, w)
    foreground = foreground.reshape(h, w)
    raw_labels, raw_count = ndimage.label(foreground, structure=_CROSS)
    clean = foreground & roi_mask & ~edges
    labels, clean_count = ndimage.label(clean, structure=_CROSS)
    sizes = np.bincount(labels.ravel())
    result.update(invalid_indices=np.flatnonzero(~finite).tolist(),
        raw_edge_indices=np.flatnonzero(raw_edges).tolist(), edge_indices=np.flatnonzero(edges).tolist())
    result["preprocessing"] = {"input_point_count": int(len(xyz)),
        "finite_point_count": int(np.count_nonzero(finite)),
        "foreground_point_count": int(np.count_nonzero(foreground)),
        "roi_foreground_point_count": int(np.count_nonzero(foreground & roi_mask)),
        "clean_foreground_point_count": int(np.count_nonzero(clean)),
        "raw_foreground_component_count": int(raw_count), "clean_component_count": int(clean_count),
        "provided_frame_count": int(len(sequence)) if sequence is not None else 0,
        "frame_error": frame_error}
    large_labels = np.flatnonzero(sizes[1:] >= SMALL_BOARD_DIAGNOSTIC_POLICY["minimum_clean_component_points"]) + 1
    central = []
    for label in large_labels:
        indices = np.flatnonzero(labels.ravel() == label)
        rows, cols = indices // w, indices % w
        row_center, col_center = float(np.median(rows)/(h-1)), float(np.median(cols)/(w-1))
        is_central = .25 <= col_center <= .75 and .20 <= row_center <= .80
        parent = int(raw_labels.ravel()[indices[0]])
        candidate = {"clean_component_id": int(label), "raw_component_id": parent,
            "clean_point_count": int(len(indices)),
            "image_median_fraction_xy": [col_center, row_center], "central": is_central,
            "clean_image_bounds_xyxy": [int(cols.min()), int(rows.min()), int(cols.max()), int(rows.max())],
            "full_connected_foreground_point_count": int(np.count_nonzero(raw_labels == parent)),
            "outside_roi_same_component_point_count": int(np.count_nonzero((raw_labels == parent) & ~roi_mask))}
        result["selection"]["candidates"].append(candidate)
        if is_central:
            central.append((int(label), parent))
    off_center_partial = False
    if len(central) != 1:
        if not central and len(large_labels) == 1:
            # Retain an inspectable visible patch without relaxing the central
            # selection rule or turning a cropped patch into a verified target.
            candidate = result["selection"]["candidates"][0]
            selected_label, raw_label = candidate["clean_component_id"], candidate["raw_component_id"]
            off_center_partial = True
        else:
            result["status"] = "UNKNOWN" if large_labels.size else "REJECTED"
            result["reasons"] = ["MULTIPLE_CENTRAL_FOREGROUND_COMPONENTS" if len(central) > 1 else "NO_UNIQUE_LARGE_CENTRAL_FOREGROUND_COMPONENT"]
            return result
    else:
        selected_label, raw_label = central[0]
    siblings = [c for c in result["selection"]["candidates"] if c["raw_component_id"] == raw_label]
    if len(siblings) != 1:
        result["status"] = "UNKNOWN"
        result["reasons"] = ["CONNECTED_FOREGROUND_HAS_MULTIPLE_LARGE_DEPTH_SEPARATED_COMPONENTS"]
        return result
    full_candidate_mask = raw_labels == raw_label
    candidate_mask = full_candidate_mask & roi_mask
    interior = ndimage.binary_erosion(labels == selected_label, structure=_CROSS,
        iterations=SMALL_BOARD_DIAGNOSTIC_POLICY["interior_erosion_pixels"], border_value=0)
    raw_ids, interior_ids = np.flatnonzero(candidate_mask), np.flatnonzero(interior)
    full_ids = np.flatnonzero(full_candidate_mask)
    outside_roi_ids = np.flatnonzero(full_candidate_mask & ~roi_mask)
    cut_inside, cut_outside = _continuous_band_boundary(candidate_mask,
        finite.reshape(h, w), foreground, raw_edges)
    roi_contact_ids = np.flatnonzero(candidate_mask & roi_boundary)
    rows, cols = raw_ids // w, raw_ids % w
    envelope = np.zeros((h, w), bool)
    envelope[rows.min():rows.max()+1, cols.min():cols.max()+1] = True
    envelope_ids = np.flatnonzero(envelope)
    envelope_finite = envelope_ids[finite[envelope_ids]]
    raw_fit, interior_fit = _tls(xyz[raw_ids]), _tls(xyz[interior_ids])
    result.update(raw_candidate_indices=raw_ids.tolist(), interior_indices=interior_ids.tolist(),
        full_connected_foreground_indices=full_ids.tolist(),
        full_connected_foreground_fit=_tls(xyz[full_ids]),
        outside_roi_same_component_indices=outside_roi_ids.tolist(),
        selection_band_boundary_indices=np.flatnonzero(cut_inside).tolist(),
        selection_band_excluded_adjacent_indices=np.flatnonzero(cut_outside).tolist(),
        selection_roi_boundary_indices=roi_contact_ids.tolist(),
        candidate_envelope_indices=envelope_ids.tolist(), raw_candidate_fit=raw_fit, interior_fit=interior_fit,
        invalid_inside_candidate_envelope_indices=envelope_ids[~finite[envelope_ids]].tolist(),
        excluded_by_x_band_inside_candidate_envelope_indices=envelope_ids[finite[envelope_ids] & ~foreground.ravel()[envelope_ids]].tolist(),
        edge_points_inside_candidate_indices=np.flatnonzero(candidate_mask & edges).tolist())
    result["selection"].update(status="PARTIAL_CANDIDATE_DIAGNOSTIC_IDENTITY_UNVERIFIED" if off_center_partial else "CANDIDATE_SELECTED_IDENTITY_UNVERIFIED",
        raw_component_id=raw_label, clean_component_id=selected_label,
        image_bounds_xyxy=[int(cols.min()), int(rows.min()), int(cols.max()), int(rows.max())])
    reasons = ["UNIQUE_FOREGROUND_OUTSIDE_CENTRAL_SELECTION_REGION"] if off_center_partial else []
    if np.any(cut_inside):
        reasons.append("CANDIDATE_CONTINUES_BEYOND_SELECTION_X_BAND")
    if len(roi_contact_ids):
        reasons.append("RAW_CANDIDATE_TOUCHES_SELECTION_ROI_BOUNDARY")
    if len(outside_roi_ids):
        reasons.append("CONNECTED_FOREGROUND_CONTINUES_OUTSIDE_SELECTION_ROI")
    if np.any(np.asarray(raw_fit["span_m"]) < .20) or np.any(np.asarray(raw_fit["span_m"]) > .80):
        reasons.append("RAW_CANDIDATE_SIZE_NOT_COMPATIBLE_WITH_SMALL_BOARD")
    if cols.min() == 0 or rows.min() == 0 or cols.max() == w-1 or rows.max() == h-1:
        reasons.append("RAW_CANDIDATE_CLIPPED_BY_IMAGE_BOUNDARY")
    if len(interior_ids) < SMALL_BOARD_DIAGNOSTIC_POLICY["minimum_interior_points"]:
        reasons.append("INSUFFICIENT_FIXED_INTERIOR_SUPPORT")
    if frame_error:
        reasons.append(frame_error)
    if interior_fit is not None:
        n = np.asarray(interior_fit["normal"])
        d = interior_fit["d_m"]
        result["all_candidate_residual_to_interior_plane"] = _residual_stats(xyz[raw_ids] @ n + d)
        result["all_envelope_residual_to_interior_plane"] = _residual_stats(xyz[envelope_finite] @ n + d)
        result["full_connected_foreground_residual_to_interior_plane"] = _residual_stats(xyz[full_ids] @ n + d)
        result["outside_roi_same_component_residual_to_interior_plane"] = _residual_stats(xyz[outside_roi_ids] @ n + d)
    result["temporal"] = _temporal(sequence, frame_error, interior_ids, interior_fit,
        (band_min, band_max))
    result["status"] = "UNKNOWN" if reasons else "DIAGNOSTIC_AVAILABLE"
    result["reasons"] = reasons
    return result
