"""Offline, same-face, multiple-pose plane calibration (all distances in metres).

T_left_right maps right-lidar points into the left lidar.  Candidate thresholds
are engineering starting points, never a claim of measured accuracy.  This
module neither accesses sensors nor writes or enables runtime extrinsics.
"""
from __future__ import annotations

import math
from collections import defaultdict
from typing import Any, Mapping, Sequence

import numpy as np


PLANE_POLICY = {
    "label": "EXPERIMENTAL_CANDIDATE_GATES_NOT_ACCURACY_PROOF",
    "minimum_points": 60,
    "ransac_distance_m": 0.03,
    "ransac_trials": 320,
    "minimum_inlier_fraction": 0.75,
    "maximum_all_p95_m": 0.06,
    "minimum_span_m": [0.10, 0.10],
    "span_definition": "5th-to-95th percentile of inlier plane coordinates",
}
SOLVE_POLICY = {
    "label": "EXPERIMENTAL_CANDIDATE_GATES_NOT_ACCURACY_PROOF",
    "minimum_independent_train_captures": 3,
    "minimum_independent_holdout_captures": 3,
    "maximum_normal_condition": 20.0,
    "maximum_normal_angle_deg": 3.0,
    "maximum_offset_m": 0.05,
    "same_face_confirmation_required": True,
    "pose_weighting": "one equal-weight representative per capture_id",
    "holdout_used_for_fitting": False,
}


def _fit_svd(points: np.ndarray) -> tuple[np.ndarray, float, np.ndarray, np.ndarray]:
    centroid = np.mean(points, axis=0)
    _, _, vt = np.linalg.svd(points - centroid, full_matrices=False)
    normal = vt[-1].copy()
    # The physical same-face constraint in solve_planes relies on this sign.
    if float(normal @ centroid) > 0.0:
        normal = -normal
    return normal, -float(normal @ centroid), centroid, vt[:2]


def fit_plane(points: Sequence[Sequence[float]], *, seed: int = 0) -> dict[str, Any]:
    """Robust plane diagnostic, including rejected fits and original point IDs.

    Non-finite coordinates are rejected rather than silently changing the ROI.
    Residuals named ``all_*`` include every finite input point.  Inlier-only
    residuals must not conceal background or a second plane in the selection.
    """
    result: dict[str, Any] = {
        "normal": None, "d_m": None, "centroid": None,
        "point_count": 0, "inlier_count": 0, "inlier_fraction": 0.0,
        "rmse_m": None, "p95_m": None, "all_rmse_m": None,
        "all_p95_m": None, "span_m": [0.0, 0.0], "inlier_indices": [],
        "status": "REJECTED", "reasons": [], "policy": dict(PLANE_POLICY),
    }
    reasons = result["reasons"]
    try:
        xyz = np.asarray(points, dtype=np.float64)
    except (TypeError, ValueError):
        reasons.append("INVALID_POINT_ARRAY")
        return result
    if xyz.ndim != 2 or xyz.shape[1] != 3:
        reasons.append("POINT_ARRAY_MUST_BE_N_BY_3")
        return result
    result["point_count"] = int(len(xyz))
    if not np.isfinite(xyz).all():
        reasons.append("NONFINITE_POINTS")
        return result
    if len(xyz) < PLANE_POLICY["minimum_points"]:
        reasons.append("TOO_FEW_POINTS")
    if len(xyz) < 3:
        return result

    rng = np.random.default_rng(seed)
    sample = xyz
    if len(xyz) > 4000:
        sample = xyz[rng.choice(len(xyz), 4000, replace=False)]
    threshold = PLANE_POLICY["ransac_distance_m"]
    best = None
    best_key = (-1, -math.inf)
    for _ in range(PLANE_POLICY["ransac_trials"]):
        a, b, c = sample[rng.choice(len(sample), 3, replace=False)]
        normal = np.cross(b - a, c - a)
        length = float(np.linalg.norm(normal))
        if length <= 1e-10:
            continue
        normal /= length
        residual = np.abs(sample @ normal - float(a @ normal))
        inside = residual <= threshold
        count = int(np.count_nonzero(inside))
        key = (count, -float(np.mean(residual[inside] ** 2)))
        if key > best_key:
            best_key = key
            best = (normal.copy(), -float(a @ normal))

    if best is None:
        normal, d, _, _ = _fit_svd(xyz)
    else:
        normal, d = best
    mask = np.abs(xyz @ normal + d) <= threshold
    # Re-estimate with all ROI points, even when RANSAC was subsampled.
    for _ in range(5):
        if np.count_nonzero(mask) < 3:
            break
        normal, d, centroid, axes = _fit_svd(xyz[mask])
        updated = np.abs(xyz @ normal + d) <= threshold
        if np.array_equal(mask, updated):
            break
        mask = updated
    if np.count_nonzero(mask) < 3:
        reasons.append("NO_SUPPORTED_PLANE")
        return result
    normal, d, centroid, axes = _fit_svd(xyz[mask])
    residual = np.abs(xyz @ normal + d)
    inlier_residual = residual[mask]
    coords = (xyz[mask] - centroid) @ axes.T
    spans = np.quantile(coords, 0.95, axis=0) - np.quantile(coords, 0.05, axis=0)
    spans = np.sort(spans)[::-1]
    count = int(np.count_nonzero(mask))
    fraction = count / len(xyz)
    result.update({
        "normal": normal.tolist(), "d_m": float(d), "centroid": centroid.tolist(),
        "inlier_count": count, "inlier_fraction": float(fraction),
        "rmse_m": float(np.sqrt(np.mean(inlier_residual ** 2))),
        "p95_m": float(np.quantile(inlier_residual, 0.95)),
        "all_rmse_m": float(np.sqrt(np.mean(residual ** 2))),
        "all_p95_m": float(np.quantile(residual, 0.95)),
        "span_m": spans.tolist(), "inlier_indices": np.flatnonzero(mask).tolist(),
    })
    if count < PLANE_POLICY["minimum_points"]:
        reasons.append("TOO_FEW_INLIERS")
    if fraction < PLANE_POLICY["minimum_inlier_fraction"]:
        reasons.append("LOW_INLIER_FRACTION")
    if any(float(x) < minimum for x, minimum in zip(spans, PLANE_POLICY["minimum_span_m"])):
        reasons.append("INSUFFICIENT_TWO_DIMENSIONAL_SPAN")
    if result["all_p95_m"] > PLANE_POLICY["maximum_all_p95_m"]:
        reasons.append("ALL_POINT_P95_LIMIT_EXCEEDED")
    if abs(float(normal @ centroid)) < 1e-9:
        reasons.append("PLANE_ORIENTATION_UNDEFINED_AT_SENSOR")
    result["reasons"] = list(dict.fromkeys(reasons))
    result["status"] = "PASS" if not result["reasons"] else "REJECTED"
    return result


def _read_fit(fit: Mapping[str, Any]) -> tuple[np.ndarray, float, np.ndarray]:
    if not isinstance(fit, Mapping):
        raise ValueError("INVALID_PLANE_OBJECT")
    if fit.get("status") != "PASS":
        raise ValueError("PLANE_FIT_NOT_PASS")
    normal = np.asarray(fit["normal"], dtype=np.float64)
    centroid = np.asarray(fit["centroid"], dtype=np.float64)
    d = float(fit["d_m"])
    if normal.shape != (3,) or centroid.shape != (3,):
        raise ValueError("INVALID_PLANE_SHAPE")
    if not np.isfinite(normal).all() or not np.isfinite(centroid).all() or not math.isfinite(d):
        raise ValueError("NONFINITE_PLANE")
    length = float(np.linalg.norm(normal))
    if length <= 1e-12:
        raise ValueError("ZERO_PLANE_NORMAL")
    normal, d = normal / length, d / length
    if abs(float(normal @ centroid) + d) > 1e-5:
        raise ValueError("PLANE_CENTROID_EQUATION_INCONSISTENT")
    direction = float(normal @ centroid)
    if abs(direction) < 1e-9:
        raise ValueError("PLANE_ORIENTATION_UNDEFINED_AT_SENSOR")
    if direction > 0:
        normal, d = -normal, -d
    return normal, d, centroid


def _observability(normals: np.ndarray) -> dict[str, Any]:
    if len(normals) == 0:
        return {"rank": 0, "condition": None, "singular_values": []}
    singular = np.linalg.svd(normals, compute_uv=False)
    rank = int(np.count_nonzero(singular > 1e-8))
    condition = float(singular[0] / singular[-1]) if len(singular) == 3 and rank == 3 else None
    return {"rank": rank, "condition": condition, "singular_values": singular.tolist()}


def _angle(a: np.ndarray, b: np.ndarray) -> float:
    return math.degrees(math.acos(float(np.clip(a @ b, -1.0, 1.0))))


def _summary(metrics: list[dict[str, Any]], role: str) -> dict[str, Any]:
    selected = [m for m in metrics if m["role"] == role]
    if not selected:
        return {"observation_count": 0, "independent_capture_count": 0,
                "normal_angle_rmse_deg": None, "maximum_normal_angle_deg": None,
                "offset_rmse_m": None, "maximum_abs_offset_m": None, "passed": None}
    # Every capture receives the same total weight, including repeated frames.
    grouped: dict[str, list[dict[str, Any]]] = defaultdict(list)
    for metric in selected:
        grouped[metric["capture_id"]].append(metric)
    angle_ms = [np.mean([m["normal_angle_deg"] ** 2 for m in group]) for group in grouped.values()]
    offset_ms = [np.mean([m["offset_m"] ** 2 for m in group]) for group in grouped.values()]
    return {
        "observation_count": len(selected), "independent_capture_count": len(grouped),
        "normal_angle_rmse_deg": float(np.sqrt(np.mean(angle_ms))),
        "maximum_normal_angle_deg": max(m["normal_angle_deg"] for m in selected),
        "offset_rmse_m": float(np.sqrt(np.mean(offset_ms))),
        "maximum_abs_offset_m": max(abs(m["offset_m"]) for m in selected),
        "passed": all(m["passed"] for m in selected),
    }


def solve_planes(observations: Sequence[Mapping[str, Any]]) -> dict[str, Any]:
    """Estimate an offline candidate from independent same-face board poses.

    The caller must confirm that each left/right ROI observes the same physical
    board face.  With this physical half-space constraint, normals directed
    towards the respective lidar origins have the same physical orientation.
    Unconfirmed opposite-face data are rejected: they must not be resolved by
    silently choosing an arbitrary sign or an assumed mounting baseline.

    Multiple ROIs/frames sharing a capture_id are *one* independent pose.  The
    first observation is its fitting representative; every observation still
    undergoes residual checks.  Holdouts never participate in fitting.
    """
    out: dict[str, Any] = {
        "status": "REJECTED", "initial_T_left_right": None,
        "rotation_left_right": None, "translation_left_right_m": None,
        "transform_direction": "p_left = R_left_right @ p_right + t_left_right",
        "per_pose_metrics": [], "train_summary": {}, "holdout_summary": {},
        "observability": {"rank": 0, "condition": None},
        "reasons": [], "warnings": [], "policy": dict(SOLVE_POLICY),
        "live_eligible": False, "icp_allowed": False,
        "independent_validation_performed": False,
    }
    reasons = out["reasons"]
    parsed = []
    roles: dict[str, set[str]] = defaultdict(set)
    seen_ids: set[str] = set()
    if not isinstance(observations, (list, tuple)) or not observations:
        reasons.append("NO_OBSERVATIONS")
        return out
    for index, observation in enumerate(observations):
        if not isinstance(observation, Mapping):
            reasons.append(f"OBSERVATION_{index}:INVALID_OBSERVATION")
            continue
        oid = str(observation.get("id", index))
        capture_id = observation.get("capture_id")
        role = observation.get("role")
        if not isinstance(capture_id, str) or not capture_id.strip():
            reasons.append(f"{oid}:MISSING_CAPTURE_ID")
            continue
        if role not in ("train", "holdout"):
            reasons.append(f"{oid}:INVALID_ROLE")
            continue
        roles[capture_id].add(role)
        if oid in seen_ids:
            reasons.append(f"{oid}:DUPLICATE_OBSERVATION_ID")
        seen_ids.add(oid)
        if observation.get("same_face_confirmed") is not True:
            reasons.append(f"{oid}:SAME_FACE_CONFIRMATION_REQUIRED")
            continue
        try:
            left = _read_fit(observation["left"])
            right = _read_fit(observation["right"])
        except (KeyError, TypeError, ValueError, np.linalg.LinAlgError) as exc:
            reasons.append(f"{oid}:INVALID_PLANE:{exc}")
            continue
        parsed.append({"id": oid, "capture_id": capture_id, "role": role,
                       "left": left, "right": right})
    for capture_id, used_roles in roles.items():
        if len(used_roles) > 1:
            reasons.append(f"{capture_id}:CAPTURE_ROLE_LEAKAGE")
    # Invalid supplied evidence cannot be silently dropped to obtain a pass.
    if reasons:
        return out

    grouped: dict[tuple[str, str], list[dict[str, Any]]] = defaultdict(list)
    for observation in parsed:
        grouped[(observation["role"], observation["capture_id"])].append(observation)
    for (_, capture_id), group in grouped.items():
        reference = group[0]
        for other in group[1:]:
            for side in ("left", "right"):
                n0, d0, _ = reference[side]
                n1, d1, _ = other[side]
                if (_angle(n0, n1) > SOLVE_POLICY["maximum_normal_angle_deg"] or
                        abs(d0 - d1) > SOLVE_POLICY["maximum_offset_m"]):
                    reasons.append(f"{capture_id}:INCONSISTENT_REPEATED_POSE")
    representatives = [group[0] for group in grouped.values()]
    train = [o for o in representatives if o["role"] == "train"]
    holdout = [o for o in representatives if o["role"] == "holdout"]
    left_normals = np.asarray([o["left"][0] for o in train]).reshape((-1, 3))
    right_normals = np.asarray([o["right"][0] for o in train]).reshape((-1, 3))
    left_obs = _observability(left_normals)
    right_obs = _observability(right_normals)
    valid_conditions = [v["condition"] for v in (left_obs, right_obs) if v["condition"] is not None]
    out["observability"] = {
        "rank": min(left_obs["rank"], right_obs["rank"]),
        "condition": max(valid_conditions) if len(valid_conditions) == 2 else None,
        "left": left_obs, "right": right_obs,
        "independent_train_captures": [o["capture_id"] for o in train],
        "repeated_observation_count": len(parsed) - len(representatives),
        "representative_ids": [o["id"] for o in train],
    }
    if len(train) < SOLVE_POLICY["minimum_independent_train_captures"]:
        reasons.append("TOO_FEW_INDEPENDENT_TRAIN_CAPTURES")
    if min(left_obs["rank"], right_obs["rank"]) < 3:
        reasons.append("TRAIN_NORMALS_RANK_DEFICIENT")
    if (left_obs["condition"] is not None and right_obs["condition"] is not None and
            max(left_obs["condition"], right_obs["condition"]) > SOLVE_POLICY["maximum_normal_condition"]):
        reasons.append("TRAIN_NORMALS_ILL_CONDITIONED")
    if reasons:
        return out

    # Kabsch maps the right normals to the left normals.  det(R) must be +1.
    u, _, vt = np.linalg.svd(right_normals.T @ left_normals)
    correction = np.eye(3)
    correction[2, 2] = 1.0 if np.linalg.det(vt.T @ u.T) > 0.0 else -1.0
    rotation = vt.T @ correction @ u.T
    rhs = np.asarray([o["right"][1] - o["left"][1] for o in train])
    translation, _, _, _ = np.linalg.lstsq(left_normals, rhs, rcond=None)
    transform = np.eye(4)
    transform[:3, :3] = rotation
    transform[:3, 3] = translation
    out["initial_T_left_right"] = transform.tolist()
    out["rotation_left_right"] = rotation.tolist()
    out["translation_left_right_m"] = translation.tolist()
    out["rotation_determinant"] = float(np.linalg.det(rotation))
    if abs(float(np.linalg.det(rotation)) - 1.0) > 1e-8:
        reasons.append("IMPROPER_ROTATION")
    for observation in parsed:
        nl, dl, _ = observation["left"]
        nr, dr, _ = observation["right"]
        angle = _angle(nl, rotation @ nr)
        offset = float(nl @ translation + dl - dr)
        passed = angle <= SOLVE_POLICY["maximum_normal_angle_deg"] and abs(offset) <= SOLVE_POLICY["maximum_offset_m"]
        out["per_pose_metrics"].append({
            "id": observation["id"], "capture_id": observation["capture_id"],
            "role": observation["role"], "normal_angle_deg": angle,
            "offset_m": offset, "absolute_offset_m": abs(offset), "passed": passed,
        })
        if not passed:
            reasons.append(f"{observation['id']}:{observation['role'].upper()}_PLANE_RESIDUAL_LIMIT_EXCEEDED")
    out["train_summary"] = _summary(out["per_pose_metrics"], "train")
    out["holdout_summary"] = _summary(out["per_pose_metrics"], "holdout")
    hl = _observability(np.asarray([o["left"][0] for o in holdout]).reshape((-1, 3)))
    hr = _observability(np.asarray([o["right"][0] for o in holdout]).reshape((-1, 3)))
    out["holdout_summary"]["observability"] = {"left": hl, "right": hr}
    holdout_condition_good = (hl["condition"] is not None and hr["condition"] is not None and
                              max(hl["condition"], hr["condition"]) <= SOLVE_POLICY["maximum_normal_condition"])
    out["independent_validation_performed"] = bool(
        len(holdout) >= SOLVE_POLICY["minimum_independent_holdout_captures"] and
        hl["rank"] == 3 and hr["rank"] == 3 and holdout_condition_good
    )
    if not out["independent_validation_performed"]:
        out["warnings"].append("INDEPENDENT_HOLDOUT_VALIDATION_NOT_PERFORMED")
    out["holdout_summary"]["validation_performed"] = out["independent_validation_performed"]
    out["reasons"] = list(dict.fromkeys(reasons))
    out["status"] = "CANDIDATE" if not reasons else "REJECTED"
    out["icp_allowed"] = out["status"] == "CANDIDATE"
    return out
