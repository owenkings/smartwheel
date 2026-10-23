"""Offline geometric registration of independently extracted environment planes.

Plane convention is n @ p + d = 0, with unit n facing the sensor and d > 0.
The estimated transform is p_left = R @ p_right + t. Consequently n_left =
R @ n_right and d_right = d_left + n_left @ t. No board-pose or orthogonal-wall
assumption is made. Nothing in this module accesses hardware or enables a live
extrinsic. Thresholds are experimental consistency gates, not accuracy claims.
"""
from __future__ import annotations

import itertools
import math
from collections import Counter
from typing import Any, Mapping, Sequence

import numpy as np


POLICY = {
    "label": "EXPERIMENTAL_GEOMETRIC_CONSISTENCY_GATES_NOT_ABSOLUTE_ACCURACY",
    "minimum_points": 80,
    "minimum_span_m": [0.20, 0.20],
    "maximum_plane_rmse_m": 0.025,
    "maximum_plane_p95_m": 0.050,
    "maximum_input_scene_extent_m": 1000.0,
    "initial_matching_angle_deg": 20.0,
    "initial_matching_offset_m": 0.35,
    "maximum_normal_angle_deg": 3.0,
    "maximum_offset_m": 0.05,
    "maximum_normal_condition": 20.0,
    "minimum_normal_rank": 3,
    "normal_residual_scale_deg": 2.0,
    "offset_residual_scale_m": 0.03,
    "robust_loss": "soft_l1",
    "capture_weighting": "each capture has equal total residual weight",
    "maximum_seed_triples": 12000,
    "maximum_distinct_match_hypotheses": 256,
    "ambiguity_rotation_separation_deg": 0.5,
    "ambiguity_translation_separation_m": 0.01,
    "ambiguity_score_tolerance": 0.25,
    "holdout_used_for_fitting_or_model_selection": False,
    "minimum_independent_holdout_captures": 1,
    "same_side_plane_normal_convention_required": True,
    "confidence_intervals": "NOT_ESTIMATED",
}


def _empty_report() -> dict[str, Any]:
    return {
        "status": "REJECTED", "reasons": [], "warnings": [],
        "transform_direction": "p_left = R_right_to_left @ p_right + t_right_to_left_m",
        "plane_equation": "n @ p + d_m = 0; unit normal faces sensor; d_m > 0",
        "transform_right_to_left": None,
        "rotation_right_to_left": None, "translation_right_to_left_m": None,
        "origin_distance_m": None, "matches": [], "unmatched_planes": [],
        "conditioning": {}, "train_summary": {}, "holdout_summary": {},
        "validation": {"independent_holdout_performed": False,
                       "holdout_used_for_fitting": False,
                       "absolute_accuracy_certified": False},
        "temporal_quality_missing": [], "temporal_validation": {},
        "uncertainty": {"status": "NOT_ESTIMATED", "bootstrap_performed": False,
                        "confidence_intervals": None},
        "search_parameters": dict(POLICY), "initial_transform": None,
        "initial_transform_is_calibrated_truth": False,
        "live_eligible": False, "icp_allowed": False,
    }


def _angle(a: np.ndarray, b: np.ndarray) -> float:
    return math.degrees(math.acos(float(np.clip(a @ b, -1.0, 1.0))))


def _rotation_angle(r: np.ndarray) -> float:
    return math.degrees(math.acos(float(np.clip((np.trace(r) - 1) / 2, -1, 1))))


def _transform(value: Any) -> tuple[np.ndarray, np.ndarray, str]:
    if value is None:
        # External shell front-centre distance is a mechanical initial estimate,
        # not the measured distance between the SDK point-cloud origins.
        return np.eye(3), np.array([0.0, -0.4575, 0.0]), "MECHANICAL_INITIAL_ESTIMATE_ONLY"
    source = "CALLER_SUPPLIED_SEARCH_INITIAL_TRANSFORM"
    if isinstance(value, Mapping):
        if "matrix" in value:
            value = value["matrix"]
        elif "transform_right_to_left" in value:
            value = value["transform_right_to_left"]
        else:
            rotation = value.get("rotation_right_to_left", value.get("R"))
            translation = value.get("translation_right_to_left_m", value.get("t"))
            if rotation is None or translation is None:
                raise ValueError("INITIAL_TRANSFORM_REQUIRES_MATRIX_OR_R_AND_T")
            r, t = np.asarray(rotation, dtype=float), np.asarray(translation, dtype=float)
            if r.shape != (3, 3) or t.shape != (3,):
                raise ValueError("INITIAL_TRANSFORM_INVALID_SHAPE")
            matrix = np.eye(4)
            matrix[:3, :3], matrix[:3, 3] = r, t
            value = matrix
    matrix = np.asarray(value, dtype=float)
    if matrix.shape != (4, 4):
        raise ValueError("INITIAL_TRANSFORM_MUST_BE_4_BY_4")
    if not np.isfinite(matrix).all():
        raise ValueError("INITIAL_TRANSFORM_NONFINITE")
    if not np.allclose(matrix[3], [0, 0, 0, 1], atol=1e-8, rtol=0):
        raise ValueError("INITIAL_TRANSFORM_INVALID_HOMOGENEOUS_ROW")
    r = matrix[:3, :3].copy()
    if not np.allclose(r.T @ r, np.eye(3), atol=1e-6, rtol=0):
        raise ValueError("INITIAL_ROTATION_NOT_ORTHONORMAL")
    if abs(float(np.linalg.det(r)) - 1) > 1e-6:
        raise ValueError("INITIAL_ROTATION_NOT_SO3_OR_REFLECTION")
    return r, matrix[:3, 3].copy(), source


def _matrix(r: np.ndarray, t: np.ndarray) -> list[list[float]]:
    matrix = np.eye(4)
    matrix[:3, :3], matrix[:3, 3] = r, t
    return matrix.tolist()


def _read_plane(raw: Mapping[str, Any], fallback: int) -> dict[str, Any]:
    if not isinstance(raw, Mapping):
        raise ValueError("INVALID_PLANE_OBJECT")
    plane_id = raw.get("id", str(fallback))
    if not isinstance(plane_id, (str, int)) or isinstance(plane_id, bool):
        raise ValueError("INVALID_PLANE_ID")
    n = np.asarray(raw.get("normal"), dtype=float)
    c = np.asarray(raw.get("centroid"), dtype=float)
    span = np.asarray(raw.get("span_m"), dtype=float)
    d, rmse, p95 = float(raw["d_m"]), float(raw["rmse_m"]), float(raw["p95_m"])
    count = raw["point_count"]
    if n.shape != (3,) or c.shape != (3,) or span.shape != (2,):
        raise ValueError("INVALID_PLANE_SHAPE")
    if not all(np.isfinite(a).all() for a in (n, c, span)) or not all(
            math.isfinite(a) for a in (d, rmse, p95)):
        raise ValueError("NONFINITE_PLANE")
    if abs(float(np.linalg.norm(n)) - 1) > 1e-6:
        raise ValueError("PLANE_NORMAL_MUST_BE_UNIT_LENGTH")
    if d <= 0:
        raise ValueError("PLANE_NORMAL_MUST_FACE_SENSOR_WITH_POSITIVE_D")
    if abs(float(n @ c) + d) > 1e-4:
        raise ValueError("PLANE_CENTROID_EQUATION_INCONSISTENT")
    if (d > POLICY["maximum_input_scene_extent_m"] or
            np.any(np.abs(c) > POLICY["maximum_input_scene_extent_m"]) or
            np.any(span > 2 * POLICY["maximum_input_scene_extent_m"])):
        raise ValueError("PLANE_OUTSIDE_DECLARED_NUMERICAL_SCENE_EXTENT")
    if not isinstance(count, (int, np.integer)) or isinstance(count, bool) or count < 3:
        raise ValueError("INVALID_POINT_COUNT")
    if rmse < 0 or p95 < 0 or np.any(span <= 0):
        raise ValueError("NEGATIVE_OR_ZERO_PLANE_QUALITY")
    quality = raw.get("quality", {})
    quality_status = quality.get("status") if isinstance(quality, Mapping) else None
    status = raw.get("status", quality_status)
    rejected = status in {"REJECTED", "FAIL", "FAILED"} or quality_status in {
        "REJECTED", "FAIL", "FAILED"}
    accepted_status = status in {"PASS", "CANDIDATE"} or quality_status == "PASS"
    if not accepted_status and not rejected:
        raise ValueError("PLANE_MISSING_EXPLICIT_QUALITY_STATUS")
    eligible = (not rejected and accepted_status and count >= POLICY["minimum_points"]
                and all(span >= POLICY["minimum_span_m"])
                and rmse <= POLICY["maximum_plane_rmse_m"]
                and p95 <= POLICY["maximum_plane_p95_m"])
    temporal = raw.get("temporal", {})
    temporal_status = temporal.get("status") if isinstance(temporal, Mapping) else None
    return {"id": str(plane_id), "n": n, "d": d, "c": c, "span": span,
            "rmse": rmse, "p95": p95, "count": int(count), "eligible": eligible,
            "temporal_status": temporal_status}


def _parse(captures: Any) -> list[dict[str, Any]]:
    if not isinstance(captures, (list, tuple)) or not captures:
        raise ValueError("NO_CAPTURES")
    result, seen, sensor_pair, recording_roles, source_modes = [], set(), None, {}, set()
    for index, raw in enumerate(captures):
        if not isinstance(raw, Mapping):
            raise ValueError(f"CAPTURE_{index}:INVALID_CAPTURE")
        cid, role = raw.get("capture_id"), raw.get("role")
        if not isinstance(cid, str) or not cid.strip():
            raise ValueError(f"CAPTURE_{index}:MISSING_CAPTURE_ID")
        if cid in seen:
            raise ValueError(f"{cid}:DUPLICATE_CAPTURE_OR_TRAIN_HOLDOUT_LEAKAGE")
        seen.add(cid)
        if role not in {"train", "holdout"}:
            raise ValueError(f"{cid}:ROLE_MUST_BE_TRAIN_OR_HOLDOUT")
        ids = raw.get("sensor_ids")
        if isinstance(ids, Mapping):
            ids = (ids.get("left"), ids.get("right"))
        if not isinstance(ids, (list, tuple)) or len(ids) != 2 or not all(
                isinstance(v, str) and v.strip() for v in ids) or ids[0] == ids[1]:
            raise ValueError(f"{cid}:INVALID_OR_IDENTICAL_SENSOR_IDS")
        ids = tuple(ids)
        if sensor_pair is not None and ids != sensor_pair:
            raise ValueError(f"{cid}:SENSOR_IDS_OR_LEFT_RIGHT_ORDER_CHANGED")
        sensor_pair = ids
        # When provenance is supplied, a recording is one acquisition. Splitting
        # it into roles must not masquerade as independently captured validation.
        recording_id = raw.get("recording_id", raw.get("source_capture_id"))
        if recording_id is not None:
            if not isinstance(recording_id, str) or not recording_id.strip():
                raise ValueError(f"{cid}:INVALID_RECORDING_ID")
            previous = recording_roles.setdefault(recording_id, role)
            if previous != role:
                raise ValueError(f"{cid}:RECORDING_USED_IN_TRAIN_AND_HOLDOUT")
        mode = raw.get("source_mode")
        if mode not in {None, "real", "synthetic"}:
            raise ValueError(f"{cid}:SOURCE_MODE_MUST_BE_REAL_OR_SYNTHETIC")
        source_modes.add(mode)
        if len(source_modes) > 1:
            raise ValueError(f"{cid}:MIXED_OR_MISSING_SOURCE_MODES")
        capture = {"capture_id": cid, "role": role, "sensor_ids": ids, "source_mode": mode}
        for side, sensor_id in zip(("left", "right"), ids):
            extraction = raw.get(side)
            if not isinstance(extraction, Mapping) or not isinstance(extraction.get("planes"), list):
                raise ValueError(f"{cid}:{side}:MISSING_EXTRACTED_PLANES")
            if extraction.get("sensor_id", sensor_id) != sensor_id:
                raise ValueError(f"{cid}:{side}:EXTRACTION_SENSOR_ID_MISMATCH")
            planes = []
            for pi, plane in enumerate(extraction["planes"]):
                try:
                    planes.append(_read_plane(plane, pi))
                except (ValueError, TypeError, KeyError) as exc:
                    raise ValueError(f"{cid}:{side}:PLANE_{pi}:{exc}") from exc
            if len({p["id"] for p in planes}) != len(planes):
                raise ValueError(f"{cid}:{side}:DUPLICATE_PLANE_ID")
            capture[side] = [p for p in planes if p["eligible"]]
            capture[side + "_rejected"] = [p["id"] for p in planes if not p["eligible"]]
        result.append(capture)
    if not any(c["role"] == "train" for c in result):
        raise ValueError("NO_TRAIN_CAPTURES")
    return result


def _conditioning(matches: Sequence[tuple]) -> dict[str, Any]:
    if not matches:
        return {"rank": 0, "condition": None, "singular_values": [], "passed": False}
    counts = Counter(c["capture_id"] for c, _, _ in matches)
    # Equal total contribution per capture also governs conditioning diagnostics.
    normals = np.stack([l["n"] / math.sqrt(counts[c["capture_id"]]) for c, l, _ in matches])
    s = np.linalg.svd(normals, compute_uv=False)
    rank = int(np.count_nonzero(s > max(1e-8, s[0] * 1e-6)))
    condition = float(s[0] / s[-1]) if len(s) == 3 and rank == 3 else None
    return {"rank": rank, "condition": condition, "singular_values": s.tolist(),
            "passed": rank == 3 and condition is not None and
            condition <= POLICY["maximum_normal_condition"]}


def _pair_metrics(l: dict, r: dict, rotation: np.ndarray, t: np.ndarray) -> tuple[float, float]:
    return _angle(l["n"], rotation @ r["n"]), float(l["d"] + l["n"] @ t - r["d"])


def _allowed(capture: dict, rotation: np.ndarray, t: np.ndarray,
             angle_gate: float, offset_gate: float) -> set[tuple[int, int]]:
    return {(i, j) for i, l in enumerate(capture["left"])
            for j, r in enumerate(capture["right"])
            if (lambda a, e: a <= angle_gate and abs(e) <= offset_gate)(
                *_pair_metrics(l, r, rotation, t))}


def _assignment(capture: dict, rotation: np.ndarray, t: np.ndarray,
                allowed: set[tuple[int, int]], linear_sum_assignment: Any,
                *, banned: tuple[int, int] | None = None) -> tuple[list[tuple], float]:
    nl, nr = len(capture["left"]), len(capture["right"])
    if not nl or not nr or not allowed:
        return [], 0.0
    # Dummy columns let every left plane remain unmatched. A sufficiently large
    # penalty first maximizes cardinality, then minimizes geometric mismatch.
    costs = np.full((nl, nr + nl), 1e9)
    costs[:, nr:] = 1e5
    for i, j in allowed:
        if (i, j) == banned:
            continue
        a, e = _pair_metrics(capture["left"][i], capture["right"][j], rotation, t)
        costs[i, j] = (a / POLICY["normal_residual_scale_deg"]) ** 2 + (
            e / POLICY["offset_residual_scale_m"]) ** 2
    rows, cols = linear_sum_assignment(costs)
    chosen = [(int(i), int(j)) for i, j in zip(rows, cols) if j < nr and costs[i, j] < 1e5]
    matches = [(capture, capture["left"][i], capture["right"][j]) for i, j in chosen]
    return matches, float(sum(costs[i, j] for i, j in chosen))


def _signature(matches: Sequence[tuple]) -> tuple:
    return tuple(sorted((c["capture_id"], l["id"], r["id"]) for c, l, r in matches))


def _seed(matches: Sequence[tuple]) -> tuple[np.ndarray, np.ndarray] | None:
    if not _conditioning(matches)["passed"]:
        return None
    left = np.stack([l["n"] for _, l, _ in matches])
    right = np.stack([r["n"] for _, _, r in matches])
    if np.linalg.matrix_rank(right, tol=1e-6) < 3:
        return None
    u, _, vt = np.linalg.svd(left.T @ right)
    correction = np.eye(3)
    correction[-1, -1] = 1.0 if np.linalg.det(u @ vt) >= 0 else -1.0
    rotation = u @ correction @ vt
    offsets = np.array([r["d"] - l["d"] for _, l, r in matches])
    translation = np.linalg.lstsq(left, offsets, rcond=None)[0]
    # A reflection cannot be an acceptable rigid-body transform. Correcting its
    # determinant alone is insufficient: the normal residual gate rejects it.
    if any(a > POLICY["maximum_normal_angle_deg"] or abs(e) > POLICY["maximum_offset_m"]
           for _, l, r in matches for a, e in [_pair_metrics(l, r, rotation, translation)]):
        return None
    return rotation, translation


def _refine(matches: Sequence[tuple], rotation: np.ndarray, translation: np.ndarray,
            least_squares: Any, Rotation: Any) -> tuple[np.ndarray, np.ndarray, bool]:
    counts = Counter(c["capture_id"] for c, _, _ in matches)
    normal_scale = math.radians(POLICY["normal_residual_scale_deg"])
    offset_scale = POLICY["offset_residual_scale_m"]
    loss_weights = np.repeat([1 / counts[c["capture_id"]] for c, _, _ in matches], 4)

    def residual(x: np.ndarray) -> np.ndarray:
        rmat, t = Rotation.from_rotvec(x[:3]).as_matrix(), x[3:]
        output = []
        for capture, left, right in matches:
            output.extend((left["n"] - rmat @ right["n"]) / normal_scale)
            output.append((left["d"] + left["n"] @ t - right["d"]) / offset_scale)
        return np.asarray(output)

    def capture_weighted_soft_l1(z: np.ndarray) -> np.ndarray:
        # Weight the robust *loss*, not its input: the robust rejection scale
        # stays identical for captures with different numbers of plane pairs.
        root = np.sqrt(1 + z)
        return np.vstack((2 * (root - 1) * loss_weights,
                          loss_weights / root,
                          -0.5 * loss_weights / root ** 3))

    x0 = np.r_[Rotation.from_matrix(rotation).as_rotvec(), translation]
    solved = least_squares(residual, x0, loss=capture_weighted_soft_l1, f_scale=1.0,
                           max_nfev=400, xtol=1e-11, ftol=1e-11, gtol=1e-11)
    return (Rotation.from_rotvec(solved.x[:3]).as_matrix(), solved.x[3:].copy(),
            bool(solved.success and np.isfinite(solved.x).all()))


def _metrics(matches: Sequence[tuple], rotation: np.ndarray, translation: np.ndarray) -> list[dict]:
    result = []
    for capture, left, right in matches:
        angle, offset = _pair_metrics(left, right, rotation, translation)
        result.append({"capture_id": capture["capture_id"], "role": capture["role"],
                       "left_plane_id": left["id"], "right_plane_id": right["id"],
                       "normal_angle_deg": angle, "offset_m": offset,
                       "left_points": left["count"], "right_points": right["count"],
                       "left_temporal_status": left["temporal_status"],
                       "right_temporal_status": right["temporal_status"],
                       "passed": angle <= POLICY["maximum_normal_angle_deg"] and
                       abs(offset) <= POLICY["maximum_offset_m"]})
    return result


def _summary(metrics: Sequence[dict]) -> dict:
    if not metrics:
        return {"matched_plane_pairs": 0, "independent_capture_count": 0, "passed": None}
    ids = sorted({m["capture_id"] for m in metrics})
    angle_squared, offset_squared = [], []
    for cid in ids:
        group = [m for m in metrics if m["capture_id"] == cid]
        angle_squared.append(float(np.mean([m["normal_angle_deg"] ** 2 for m in group])))
        offset_squared.append(float(np.mean([m["offset_m"] ** 2 for m in group])))
    return {"matched_plane_pairs": len(metrics), "independent_capture_count": len(ids),
            "capture_ids": ids,
            "normal_angle_rmse_deg": math.sqrt(float(np.mean(angle_squared))),
            "offset_rmse_m": math.sqrt(float(np.mean(offset_squared))),
            "maximum_normal_angle_deg": max(m["normal_angle_deg"] for m in metrics),
            "maximum_abs_offset_m": max(abs(m["offset_m"]) for m in metrics),
            "passed": all(m["passed"] for m in metrics)}


def _score(metrics: Sequence[dict]) -> float:
    summary = _summary(metrics)
    return ((summary.get("normal_angle_rmse_deg", math.inf) /
             POLICY["normal_residual_scale_deg"]) ** 2 +
            (summary.get("offset_rmse_m", math.inf) /
             POLICY["offset_residual_scale_m"]) ** 2)


def _temporal_report(metrics: Sequence[dict]) -> tuple[list[dict], dict]:
    missing = []
    for m in metrics:
        for side in ("left", "right"):
            if m[side + "_temporal_status"] != "PASS":
                missing.append({"capture_id": m["capture_id"], "role": m["role"],
                                "side": side, "plane_id": m[side + "_plane_id"],
                                "status": m[side + "_temporal_status"] or "MISSING"})
    return missing, {"status": "PASS" if metrics and not missing else "NOT_VERIFIED",
                     "all_used_planes_pass": bool(metrics and not missing),
                     "requirement": "Every used plane needs independent frame stability PASS."}


def solve_environment(captures: Sequence[Mapping[str, Any]], *,
                      initial_transform: Any = None) -> dict[str, Any]:
    """Estimate and independently check a strictly offline right-to-left candidate.

    Each capture contains capture_id, role (train or holdout), sensor_ids
    ({left: id, right: id} or [left_id, right_id]), and left/right extraction
    dictionaries with planes. At least three independent normal directions are
    required over train data; all three may be in one capture. Holdouts never
    choose correspondence hypotheses, fit parameters, or select a train model.

    The optional initial transform confines correspondence search; its values
    are never residual priors or substituted for unobservable fitted values.
    """
    out = _empty_report()
    try:
        initial_r, initial_t, source = _transform(initial_transform)
        parsed = _parse(captures)
    except (ValueError, TypeError, KeyError, OverflowError, np.linalg.LinAlgError) as exc:
        out["reasons"].append(str(exc))
        return out
    out["initial_transform"] = _matrix(initial_r, initial_t)
    out["search_parameters"]["initial_transform_source"] = source
    out["search_parameters"]["prior_in_optimization"] = False
    out["search_parameters"]["search_completeness"] = "NOT_YET_EVALUATED"
    if initial_transform is None:
        out["warnings"].append("457.5 mm is a shell-based mechanical initial estimate, not a verified SDK-origin baseline.")
    out["warnings"].append("Geometric plane correspondence assumes the same physical surface and the same visible half-space; symmetry can be unidentifiable.")
    train = [c for c in parsed if c["role"] == "train"]
    holdout = [c for c in parsed if c["role"] == "holdout"]
    source_mode = parsed[0]["source_mode"]
    out["validation"]["source_mode"] = source_mode or "UNSPECIFIED"
    out["sensor_ids"] = {"left": parsed[0]["sensor_ids"][0], "right": parsed[0]["sensor_ids"][1]}
    try:
        from scipy.optimize import least_squares, linear_sum_assignment
        from scipy.spatial.transform import Rotation
    except ImportError:
        out["reasons"].append("SCIPY_REQUIRED_FOR_ROBUST_SO3_JOINT_OPTIMIZATION")
        return out

    allowed_by_id, edges = {}, []
    for capture in train:
        allowed = _allowed(capture, initial_r, initial_t,
                           POLICY["initial_matching_angle_deg"], POLICY["initial_matching_offset_m"])
        allowed_by_id[capture["capture_id"]] = allowed
        edges.extend((capture, capture["left"][i], capture["right"][j]) for i, j in sorted(allowed))
    out["search_parameters"]["initial_pair_count"] = len(edges)
    if len(edges) < 3:
        out["reasons"].append("INSUFFICIENT_COMMON_PLANES_INSIDE_DECLARED_INITIAL_SEARCH_WINDOW")
        return out
    potential_triples = math.comb(len(edges), 3)
    out["search_parameters"]["potential_seed_triples"] = potential_triples
    if potential_triples > POLICY["maximum_seed_triples"]:
        out["reasons"].append("SEARCH_TOO_LARGE_TO_EXCLUDE_AMBIGUITY_NARROW_INITIAL_TRANSFORM_OR_FILTER_PLANES")
        out["search_parameters"]["search_completeness"] = "REJECTED_BEFORE_INCOMPLETE_SEARCH"
        return out

    # Enumerate all three-direction hypotheses inside the declared window.
    # No randomly sampled unexamined alternatives can silently pass ambiguity.
    hypotheses, signatures = [], set()
    for triple in itertools.combinations(edges, 3):
        left_keys = [(c["capture_id"], l["id"]) for c, l, _ in triple]
        right_keys = [(c["capture_id"], r["id"]) for c, _, r in triple]
        if len(set(left_keys)) != 3 or len(set(right_keys)) != 3:
            continue
        seed = _seed(triple)
        if seed is None:
            continue
        rotation, translation = seed
        matches = []
        for capture in train:
            tight = _allowed(capture, rotation, translation,
                             POLICY["maximum_normal_angle_deg"], POLICY["maximum_offset_m"])
            tight &= allowed_by_id[capture["capture_id"]]
            matched, _ = _assignment(capture, rotation, translation, tight, linear_sum_assignment)
            matches.extend(matched)
        signature = _signature(matches)
        if signature in signatures or not _conditioning(matches)["passed"]:
            continue
        signatures.add(signature)
        if len(signatures) > POLICY["maximum_distinct_match_hypotheses"]:
            out["reasons"].append("TOO_MANY_CORRESPONDENCE_HYPOTHESES_AMBIGUITY_SEARCH_INCOMPLETE")
            out["search_parameters"]["search_completeness"] = "INCOMPLETE_REJECTED"
            return out
        success = True
        for _ in range(6):
            rotation, translation, success = _refine(matches, rotation, translation, least_squares, Rotation)
            if not success:
                break
            updated = []
            for capture in train:
                tight = _allowed(capture, rotation, translation,
                                 POLICY["maximum_normal_angle_deg"], POLICY["maximum_offset_m"])
                tight &= allowed_by_id[capture["capture_id"]]
                matched, _ = _assignment(capture, rotation, translation, tight, linear_sum_assignment)
                updated.extend(matched)
            if _signature(updated) == _signature(matches):
                break
            if not _conditioning(updated)["passed"]:
                success = False
                break
            matches = updated
        if not success or not matches:
            continue
        metrics = _metrics(matches, rotation, translation)
        if not all(m["passed"] for m in metrics) or not _conditioning(matches)["passed"]:
            continue
        if len({c["capture_id"] for c, _, _ in matches}) != len(train):
            continue
        # Refit if the sixth association update changed the active match set.
        rotation, translation, success = _refine(matches, rotation, translation, least_squares, Rotation)
        metrics = _metrics(matches, rotation, translation)
        if success and all(m["passed"] for m in metrics):
            hypotheses.append({"r": rotation, "t": translation, "matches": matches,
                               "metrics": metrics, "score": _score(metrics)})
    out["search_parameters"]["search_completeness"] = "ALL_DECLARED_SEED_TRIPLES_EXAMINED"
    out["search_parameters"]["distinct_match_hypotheses"] = len(signatures)
    out["search_parameters"]["passing_train_hypotheses"] = len(hypotheses)
    if not hypotheses:
        out["reasons"].append("NO_CONSISTENT_FULL_RANK_TRAIN_CORRESPONDENCE_SOLUTION")
        out["conditioning"] = _conditioning(edges)
        return out
    hypotheses.sort(key=lambda h: (-len(h["matches"]), h["score"]))
    best = hypotheses[0]
    alternatives = []
    for hypothesis in hypotheses[1:]:
        if len(hypothesis["matches"]) < len(best["matches"]):
            continue
        rotation_distance = _rotation_angle(hypothesis["r"] @ best["r"].T)
        translation_distance = float(np.linalg.norm(hypothesis["t"] - best["t"]))
        if (rotation_distance > POLICY["ambiguity_rotation_separation_deg"] or
                translation_distance > POLICY["ambiguity_translation_separation_m"]) and (
                hypothesis["score"] <= best["score"] + POLICY["ambiguity_score_tolerance"]):
            alternatives.append({"rotation_distance_deg": rotation_distance,
                                 "translation_distance_m": translation_distance,
                                 "train_score": hypothesis["score"],
                                 "transform_right_to_left": _matrix(hypothesis["r"], hypothesis["t"]),
                                 "matches": list(_signature(hypothesis["matches"]))})
    out["ambiguity"] = {"status": "REJECTED" if alternatives else "NO_COMPETING_SOLUTION_IN_DECLARED_SEARCH",
                        "best_train_score": best["score"], "alternatives": alternatives,
                        "global_uniqueness_proven": False}
    if alternatives:
        out["reasons"].append("AMBIGUOUS_DISTINCT_TRAIN_TRANSFORMS_HOLDOUT_NOT_USED_TO_CHOOSE")
        return out
    rotation, translation = best["r"], best["t"]
    # Even an unchanged transform can hide indistinguishable physical-plane
    # assignments (for example duplicated coplanar patches). Do not interpret
    # deterministic index ordering as evidence for the correspondence.
    ambiguous_associations = []
    for capture in train:
        allowed = _allowed(capture, rotation, translation,
                           POLICY["maximum_normal_angle_deg"], POLICY["maximum_offset_m"])
        allowed &= allowed_by_id[capture["capture_id"]]
        matched, total_cost = _assignment(capture, rotation, translation, allowed, linear_sum_assignment)
        index_l = {p["id"]: i for i, p in enumerate(capture["left"])}
        index_r = {p["id"]: i for i, p in enumerate(capture["right"])}
        for _, left, right in matched:
            alternative, alternative_cost = _assignment(
                capture, rotation, translation, allowed, linear_sum_assignment,
                banned=(index_l[left["id"]], index_r[right["id"]]))
            if len(alternative) == len(matched) and alternative_cost <= total_cost + (
                    POLICY["ambiguity_score_tolerance"] * len(matched)):
                ambiguous_associations.append(capture["capture_id"])
                break
    if ambiguous_associations:
        out["ambiguity"]["status"] = "REJECTED_AMBIGUOUS_PLANE_IDENTITIES"
        out["ambiguity"]["ambiguous_capture_ids"] = ambiguous_associations
        out["reasons"].append("AMBIGUOUS_TRAIN_PLANE_CORRESPONDENCE_MERGE_COPLANAR_PATCHES_OR_ADD_DISTINCT_GEOMETRY")
        return out
    out["transform_right_to_left"] = _matrix(rotation, translation)
    out["rotation_right_to_left"] = rotation.tolist()
    out["translation_right_to_left_m"] = translation.tolist()
    out["origin_distance_m"] = float(np.linalg.norm(translation))
    out["conditioning"] = _conditioning(best["matches"])
    train_metrics = best["metrics"]
    out["train_summary"] = _summary(train_metrics)

    # Holdout begins only after the train model has been irreversibly selected.
    # It is associated in a wider window and then checked with strict gates;
    # the strict gate is not used to silently discard validation failures.
    holdout_matches, holdout_failures = [], []
    for capture in holdout:
        allowed = _allowed(capture, rotation, translation,
                           POLICY["initial_matching_angle_deg"], POLICY["initial_matching_offset_m"])
        matched, total_cost = _assignment(capture, rotation, translation, allowed, linear_sum_assignment)
        if not matched:
            holdout_failures.append(f"{capture['capture_id']}:NO_COMMON_HOLDOUT_PLANES")
            continue
        holdout_matches.extend(matched)
        # Alternative one-to-one associations under the fixed model do not
        # create another train model; they invalidate holdout interpretation.
        index_l = {p["id"]: i for i, p in enumerate(capture["left"])}
        index_r = {p["id"]: i for i, p in enumerate(capture["right"])}
        for _, left, right in matched:
            banned = (index_l[left["id"]], index_r[right["id"]])
            alternative, alternative_cost = _assignment(capture, rotation, translation, allowed,
                                                        linear_sum_assignment, banned=banned)
            if len(alternative) == len(matched) and alternative_cost <= total_cost + (
                    POLICY["ambiguity_score_tolerance"] * len(matched)):
                holdout_failures.append(f"{capture['capture_id']}:AMBIGUOUS_HOLDOUT_PLANE_CORRESPONDENCE")
                break
    holdout_metrics = _metrics(holdout_matches, rotation, translation)
    out["matches"] = train_metrics + holdout_metrics
    out["holdout_summary"] = _summary(holdout_metrics)
    out["holdout_summary"]["conditioning"] = _conditioning(holdout_matches)
    for capture in parsed:
        used_l = {m["left_plane_id"] for m in out["matches"] if m["capture_id"] == capture["capture_id"]}
        used_r = {m["right_plane_id"] for m in out["matches"] if m["capture_id"] == capture["capture_id"]}
        out["unmatched_planes"].append({"capture_id": capture["capture_id"], "role": capture["role"],
            "left": [p["id"] for p in capture["left"] if p["id"] not in used_l],
            "right": [p["id"] for p in capture["right"] if p["id"] not in used_r],
            "left_quality_rejected": capture["left_rejected"],
            "right_quality_rejected": capture["right_rejected"]})
    missing, temporal = _temporal_report(out["matches"])
    out["temporal_quality_missing"], out["temporal_validation"] = missing, temporal
    out["validation"]["independent_holdout_performed"] = bool(holdout)
    out["validation"]["independent_holdout_capture_ids"] = [c["capture_id"] for c in holdout]
    out["validation"]["holdout_geometric_failures"] = holdout_failures
    if holdout and not all(m["passed"] for m in holdout_metrics):
        holdout_failures.append("HOLDOUT_NORMAL_OR_OFFSET_RESIDUAL_LIMIT_EXCEEDED")
    if holdout and not _conditioning(holdout_matches)["passed"]:
        holdout_failures.append("HOLDOUT_DOES_NOT_INDEPENDENTLY_COVER_THREE_WELL_CONDITIONED_DIRECTIONS")
    if holdout_failures:
        out["reasons"].extend(holdout_failures)
        out["validation"]["holdout_passed"] = False
        return out
    out["status"] = "CANDIDATE"
    out["validation"]["holdout_passed"] = True if holdout else None
    if not holdout:
        out["warnings"].append("NO_INDEPENDENT_HOLDOUT: geometry is a fitting candidate only.")
    if missing:
        out["warnings"].append("TEMPORAL_QUALITY_NOT_VERIFIED: repeated-frame stability is required before offline validation status.")
    if source_mode != "real":
        out["warnings"].append("SOURCE_IS_SYNTHETIC_OR_UNSPECIFIED: consistency checks cannot certify a real-capture validation.")
    if holdout and temporal["all_used_planes_pass"] and source_mode == "real":
        out["status"] = "OFFLINE_VALIDATED_CANDIDATE"
    return out
