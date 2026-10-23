"""Bounded, pose-separated point-to-plane ICP for offline board calibration.

Different board captures are never concatenated before correspondence search.
Only training captures contribute to the normal equations; holdouts are used
after fitting.  This is an offline candidate generator, not an activation API.
"""
from __future__ import annotations

import math
from typing import Any, Mapping, Sequence

import numpy as np
from scipy.spatial import cKDTree

from .plane_calibration import SOLVE_POLICY, _angle, _read_fit, _summary, solve_planes


ICP_POLICY = {
    "label": "EXPERIMENTAL_CANDIDATE_GATES_NOT_ACCURACY_PROOF",
    "stages_m": [0.12, 0.08, 0.05],
    "iterations_per_stage": 10,
    "huber_delta_m": 0.02,
    "minimum_correspondences_per_pose": 30,
    "minimum_overlap_per_pose": 0.20,
    "maximum_points_per_pose_per_side": 1500,
    "maximum_jtj_condition": 1e8,
    "maximum_total_translation_change_m": 0.10,
    "maximum_total_rotation_change_deg": 5.0,
    "maximum_holdout_offset_regression_m": 0.01,
    "maximum_holdout_angle_regression_deg": 0.5,
    "pose_weighting": "each independent capture contributes total weight one",
    "correspondence_scope": "same observation and capture only",
    "holdout_used_for_fitting": False,
    "translation_change_definition": "Euclidean difference of extrinsic translation vectors",
}


def _rotation_exp(vector: np.ndarray) -> np.ndarray:
    theta = float(np.linalg.norm(vector))
    skew = np.array([[0.0, -vector[2], vector[1]],
                     [vector[2], 0.0, -vector[0]],
                     [-vector[1], vector[0], 0.0]])
    if theta < 1e-10:
        return np.eye(3) + skew + 0.5 * (skew @ skew)
    return np.eye(3) + (math.sin(theta) / theta) * skew + ((1.0 - math.cos(theta)) / theta**2) * (skew @ skew)


def _change(transform: np.ndarray, initial: np.ndarray) -> dict[str, float]:
    angle = math.degrees(math.acos(float(np.clip((np.trace(transform[:3, :3] @ initial[:3, :3].T) - 1.0) / 2.0, -1.0, 1.0))))
    return {"translation_m": float(np.linalg.norm(transform[:3, 3] - initial[:3, 3])),
            "rotation_deg": angle}


def _read_cloud(value: Any) -> np.ndarray:
    xyz = np.asarray(value, dtype=np.float64)
    if xyz.ndim != 2 or xyz.shape[1] != 3 or not np.isfinite(xyz).all():
        raise ValueError("CLOUD_MUST_BE_FINITE_N_BY_3")
    if len(xyz) < ICP_POLICY["minimum_correspondences_per_pose"]:
        raise ValueError("TOO_FEW_ROI_POINTS")
    limit = ICP_POLICY["maximum_points_per_pose_per_side"]
    if len(xyz) > limit:
        xyz = xyz[np.linspace(0, len(xyz) - 1, limit, dtype=int)]
    return xyz


def _correspondences(pose: Mapping[str, Any], transform: np.ndarray, gate: float):
    moved = pose["right_points"] @ transform[:3, :3].T + transform[:3, 3]
    distance, indices = pose["tree"].query(moved, k=1, workers=1)
    mask = distance <= gate
    matched = pose["left_points"][indices[mask]]
    return moved[mask], matched, distance[mask], float(np.mean(mask))


def _metrics(poses: list[dict[str, Any]], transform: np.ndarray) -> list[dict[str, Any]]:
    metrics = []
    gate = ICP_POLICY["stages_m"][-1]
    for pose in poses:
        nl, dl, _ = pose["left_fit"]
        nr, dr, _ = pose["right_fit"]
        angle = _angle(nl, transform[:3, :3] @ nr)
        offset = float(nl @ transform[:3, 3] + dl - dr)
        moved, matched, distance, overlap = _correspondences(pose, transform, gate)
        plane_residual = (moved - matched) @ nl
        geometry_passed = (angle <= SOLVE_POLICY["maximum_normal_angle_deg"] and
                           abs(offset) <= SOLVE_POLICY["maximum_offset_m"])
        metrics.append({
            "id": pose["id"], "capture_id": pose["capture_id"], "role": pose["role"],
            "normal_angle_deg": angle, "offset_m": offset, "absolute_offset_m": abs(offset),
            "passed": geometry_passed, "nn_gate_m": gate,
            "correspondence_count": int(len(moved)), "source_point_count": len(pose["right_points"]),
            "target_point_count": len(pose["left_points"]), "overlap": overlap,
            "nearest_neighbor_rmse_m": float(np.sqrt(np.mean(distance**2))) if len(distance) else None,
            "point_to_plane_rmse_m": float(np.sqrt(np.mean(plane_residual**2))) if len(distance) else None,
            "cloud_overlap_passed": bool(len(moved) >= ICP_POLICY["minimum_correspondences_per_pose"] and
                                         overlap >= ICP_POLICY["minimum_overlap_per_pose"]),
        })
    return metrics


def _summary_with_clouds(metrics: list[dict[str, Any]], role: str) -> dict[str, Any]:
    summary = _summary(metrics, role)
    selected = [m for m in metrics if m["role"] == role]
    summary["minimum_overlap"] = min((m["overlap"] for m in selected), default=None)
    summary["all_cloud_overlap_passed"] = all(m["cloud_overlap_passed"] for m in selected) if selected else None
    # Each independent capture receives equal weight, including its repeated ROIs.
    captures = sorted(set(m["capture_id"] for m in selected))
    ms = []
    for capture in captures:
        residuals = [m["point_to_plane_rmse_m"] for m in selected
                     if m["capture_id"] == capture and m["point_to_plane_rmse_m"] is not None]
        if residuals:
            ms.append(float(np.mean(np.square(residuals))))
    summary["point_to_plane_rmse_m"] = float(np.sqrt(np.mean(ms))) if ms else None
    return summary


def refine_planes_icp(
    observations: Sequence[Mapping[str, Any]],
    clouds: Mapping[str, Mapping[str, Any]],
    initial_T_left_right: Sequence[Sequence[float]],
) -> dict[str, Any]:
    """Refine a passing plane candidate using same-pose ROI correspondences.

    Cloud arrays must contain only the selected board ROI's plane inliers in
    the original lidar coordinates.  Sampling and iteration are deterministic.
    Results always remain candidates, including successful independent tests.
    """
    out: dict[str, Any] = {
        "status": "REJECTED", "refined_T_left_right": None,
        "initial_T_left_right": None, "initial_per_pose_metrics": [],
        "refined_per_pose_metrics": [], "initial_train_summary": {},
        "refined_train_summary": {}, "initial_holdout_summary": {},
        "refined_holdout_summary": {}, "iteration_history": [], "change": None,
        "reasons": [], "warnings": [], "policy": dict(ICP_POLICY),
        "live_eligible": False, "independent_validation_performed": False,
        "independent_validation_passed": False,
    }
    reasons = out["reasons"]
    solved = solve_planes(observations)
    out["observability"] = solved["observability"]
    out["warnings"] = list(solved["warnings"])
    if solved["status"] != "CANDIDATE":
        reasons.append("PLANE_INITIALIZATION_NOT_CANDIDATE")
        reasons.extend(solved["reasons"])
        return out
    try:
        initial = np.asarray(initial_T_left_right, dtype=np.float64)
        if initial.shape != (4, 4) or not np.isfinite(initial).all():
            raise ValueError("TRANSFORM_MUST_BE_FINITE_4_BY_4")
        if not np.allclose(initial[3], [0, 0, 0, 1], atol=1e-9, rtol=0):
            raise ValueError("INVALID_HOMOGENEOUS_ROW")
        rotation = initial[:3, :3]
        if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6, rtol=0) or abs(np.linalg.det(rotation) - 1.0) > 1e-6:
            raise ValueError("INVALID_PROPER_ROTATION")
    except (TypeError, ValueError) as exc:
        reasons.append(f"INVALID_INITIAL_TRANSFORM:{exc}")
        return out
    out["initial_T_left_right"] = initial.tolist()
    poses = []
    for observation in observations:
        oid = str(observation.get("id"))
        try:
            left = _read_cloud(clouds[oid]["left"])
            right = _read_cloud(clouds[oid]["right"])
            left_fit = _read_fit(observation["left"])
            right_fit = _read_fit(observation["right"])
        except (KeyError, TypeError, ValueError) as exc:
            reasons.append(f"{oid}:INVALID_ROI_CLOUD:{exc}")
            continue
        # Check the declared inlier cloud: an unfiltered background ROI cannot
        # quietly become the ICP target after a good separate plane was fit.
        for side, points, fit in (("left", left, left_fit), ("right", right, right_fit)):
            normal, d, _ = fit
            if float(np.quantile(np.abs(points @ normal + d), 0.95)) > 0.035:
                reasons.append(f"{oid}:{side}:ROI_CLOUD_NOT_PLANE_INLIERS")
        poses.append({"id": oid, "role": observation["role"], "capture_id": observation["capture_id"],
                      "left_points": left, "right_points": right, "tree": cKDTree(left),
                      "left_fit": left_fit, "right_fit": right_fit})
    if reasons:
        return out
    out["initial_per_pose_metrics"] = _metrics(poses, initial)
    out["initial_train_summary"] = _summary_with_clouds(out["initial_per_pose_metrics"], "train")
    out["initial_holdout_summary"] = _summary_with_clouds(out["initial_per_pose_metrics"], "holdout")
    if any(not m["passed"] for m in out["initial_per_pose_metrics"]):
        reasons.append("PROVIDED_INITIAL_TRANSFORM_FAILS_PLANE_GATES")
        return out
    # One representative per capture is consistent with solve_planes and makes
    # repeated frames unable to dominate either translation or rotation.
    train = []
    seen_captures = set()
    for pose in poses:
        if pose["role"] == "train" and pose["capture_id"] not in seen_captures:
            train.append(pose)
            seen_captures.add(pose["capture_id"])
    transform = initial.copy()
    stopped = False
    for gate in ICP_POLICY["stages_m"]:
        for iteration in range(ICP_POLICY["iterations_per_stage"]):
            hessian = np.zeros((6, 6))
            gradient = np.zeros(6)
            fixed = []
            for pose in train:
                moved, matched, _, overlap = _correspondences(pose, transform, gate)
                if len(moved) < ICP_POLICY["minimum_correspondences_per_pose"] or overlap < ICP_POLICY["minimum_overlap_per_pose"]:
                    reasons.append(f"{pose['id']}:INSUFFICIENT_ICP_OVERLAP_AT_{gate:.2f}M")
                    stopped = True
                    break
                normal = pose["left_fit"][0]
                residual = (moved - matched) @ normal
                jacobian = np.column_stack([np.cross(moved, normal), np.broadcast_to(normal, moved.shape)])
                weight = np.minimum(1.0, ICP_POLICY["huber_delta_m"] / np.maximum(np.abs(residual), 1e-12))
                weight /= len(moved)
                hessian += jacobian.T @ (weight[:, None] * jacobian)
                gradient += jacobian.T @ (weight * residual)
                fixed.append((moved, matched, normal, weight))
            if stopped:
                break
            eigenvalues = np.linalg.eigvalsh(hessian)
            condition = (float(eigenvalues[-1] / eigenvalues[0]) if eigenvalues[0] > 1e-12 else None)
            if condition is None or condition > ICP_POLICY["maximum_jtj_condition"]:
                reasons.append("ICP_NORMAL_EQUATIONS_ILL_CONDITIONED")
                out["iteration_history"].append({"gate_m": gate, "iteration": iteration,
                                                  "jtj_condition": condition, "eigenvalues": eigenvalues.tolist()})
                stopped = True
                break
            step = -np.linalg.solve(hessian, gradient)
            current_cost = sum(float(np.sum(w * (((p - q) @ n) ** 2))) for p, q, n, w in fixed)
            accepted = None
            accepted_scale = None
            for scale in (1.0, 0.5, 0.25, 0.125):
                delta_rotation = _rotation_exp(scale * step[:3])
                delta_translation = scale * step[3:]
                candidate = transform.copy()
                candidate[:3, :3] = delta_rotation @ transform[:3, :3]
                candidate[:3, 3] = delta_rotation @ transform[:3, 3] + delta_translation
                candidate_cost = sum(float(np.sum(w * (((p @ delta_rotation.T + delta_translation - q) @ n) ** 2)))
                                     for p, q, n, w in fixed)
                if candidate_cost <= current_cost + 1e-15:
                    accepted, accepted_scale = candidate, scale
                    break
            if accepted is None:
                # No improving Gauss-Newton step at these correspondences.
                out["iteration_history"].append({"gate_m": gate, "iteration": iteration,
                                                  "jtj_condition": condition, "converged": True,
                                                  "reason": "NO_IMPROVING_STEP"})
                break
            change = _change(accepted, initial)
            if (change["translation_m"] > ICP_POLICY["maximum_total_translation_change_m"] or
                    change["rotation_deg"] > ICP_POLICY["maximum_total_rotation_change_deg"]):
                reasons.append("ICP_TOTAL_CHANGE_LIMIT_EXCEEDED")
                out["iteration_history"].append({"gate_m": gate, "iteration": iteration,
                                                  "jtj_condition": condition, "proposed_change": change})
                stopped = True
                break
            transform = accepted
            out["iteration_history"].append({"gate_m": gate, "iteration": iteration,
                                              "jtj_condition": condition, "step_scale": accepted_scale,
                                              "fixed_correspondence_cost": current_cost, "change": change})
            if float(np.linalg.norm(accepted_scale * step)) < 1e-7:
                break
        if stopped:
            break
    out["refined_T_left_right"] = transform.tolist()
    out["change"] = _change(transform, initial)
    out["refined_per_pose_metrics"] = _metrics(poses, transform)
    out["refined_train_summary"] = _summary_with_clouds(out["refined_per_pose_metrics"], "train")
    out["refined_holdout_summary"] = _summary_with_clouds(out["refined_per_pose_metrics"], "holdout")
    initial_metrics = {m["id"]: m for m in out["initial_per_pose_metrics"]}
    for metric in out["refined_per_pose_metrics"]:
        if not metric["passed"]:
            reasons.append(f"{metric['id']}:REFINED_PLANE_RESIDUAL_LIMIT_EXCEEDED")
        if not metric["cloud_overlap_passed"]:
            reasons.append(f"{metric['id']}:REFINED_CLOUD_OVERLAP_LIMIT_EXCEEDED")
        if metric["role"] == "holdout":
            before = initial_metrics[metric["id"]]
            if (abs(metric["offset_m"]) > abs(before["offset_m"]) + ICP_POLICY["maximum_holdout_offset_regression_m"] or
                    metric["normal_angle_deg"] > before["normal_angle_deg"] + ICP_POLICY["maximum_holdout_angle_regression_deg"]):
                reasons.append(f"{metric['id']}:HOLDOUT_REGRESSION")
    holdout_metrics = [m for m in out["refined_per_pose_metrics"] if m["role"] == "holdout"]
    out["independent_validation_performed"] = bool(solved["independent_validation_performed"])
    out["independent_validation_passed"] = bool(
        out["independent_validation_performed"] and holdout_metrics and
        all(m["passed"] and m["cloud_overlap_passed"] for m in holdout_metrics) and
        not any("HOLDOUT_REGRESSION" in reason for reason in reasons)
    )
    out["reasons"] = list(dict.fromkeys(reasons))
    out["status"] = "CANDIDATE" if not reasons else "REJECTED"
    return out
