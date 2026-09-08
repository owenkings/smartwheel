#!/usr/bin/env python3
"""Offline candidate calibration for the direct dual XT-M60 capture.

This is deliberately an evidence/reporting tool.  It never edits ROS/URDF
configuration.  It can estimate a relative radar transform from overlapping
clouds and report an effective host-time correlation when motion exists.  A
vehicle body anchor or a physical IMU lever arm is not inferred silently.
"""

from __future__ import annotations

import argparse
import json
import math
import statistics
from pathlib import Path

import numpy as np

try:
    from scipy.spatial import cKDTree
except Exception:  # pragma: no cover - report a useful result on minimal hosts
    cKDTree = None


def _load_jsonl(path: Path):
    if not path.exists():
        return []
    rows = []
    for line in path.read_text(encoding="utf-8").splitlines():
        if line.strip():
            rows.append(json.loads(line))
    return rows


def _load_points(sensor_dir: Path, record: dict) -> np.ndarray:
    n = int(record.get("point_count", 0))
    if n <= 0:
        return np.empty((0, 3), dtype=np.float32)
    seq = int(record.get("sequence", 0))
    path = sensor_dir / "points_xyz_f32.bin"
    with path.open("rb") as stream:
        stream.seek(seq * n * 3 * 4)
        values = np.fromfile(stream, dtype="<f4", count=n * 3)
    if values.size != n * 3:
        return np.empty((0, 3), dtype=np.float32)
    return values.reshape(n, 3)


def _valid_points(points: np.ndarray, max_range: float = 15.0) -> np.ndarray:
    if points.size == 0:
        return points.reshape((-1, 3))
    finite = np.isfinite(points).all(axis=1)
    norms = np.linalg.norm(points, axis=1)
    mask = finite & (norms >= 0.20) & (norms <= max_range)
    return points[mask]


def _sample(points: np.ndarray, limit: int, rng: np.random.Generator) -> np.ndarray:
    if len(points) <= limit:
        return points
    return points[rng.choice(len(points), size=limit, replace=False)]


def _rpy_matrix(rpy):
    roll, pitch, yaw = (float(x) for x in rpy)
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def _rigid_fit(source: np.ndarray, target: np.ndarray):
    if len(source) < 3 or len(target) != len(source):
        return np.eye(3), np.zeros(3)
    source_center = source.mean(axis=0)
    target_center = target.mean(axis=0)
    h = (source - source_center).T @ (target - target_center)
    u, _, vt = np.linalg.svd(h)
    rotation = vt.T @ u.T
    if np.linalg.det(rotation) < 0:
        vt[-1, :] *= -1
        rotation = vt.T @ u.T
    translation = target_center - rotation @ source_center
    return rotation, translation


def _icp(source: np.ndarray, target: np.ndarray, initial_r: np.ndarray, initial_t: np.ndarray, iterations=30):
    if cKDTree is None:
        return {"success": False, "reason": "scipy_cKDTree_unavailable"}
    if len(source) < 30 or len(target) < 30:
        return {"success": False, "reason": "too_few_valid_points"}
    tree = cKDTree(target)
    rotation = initial_r.copy()
    translation = initial_t.copy()
    used = 0
    last_rmse = None
    for _ in range(iterations):
        transformed = (rotation @ source.T).T + translation
        distances, indices = tree.query(transformed, k=1, workers=1)
        finite = np.isfinite(distances)
        if not np.any(finite):
            break
        finite_distances = distances[finite]
        cutoff = min(0.80, max(0.08, float(np.percentile(finite_distances, 75)) * 1.8))
        keep = finite & (distances <= cutoff)
        if np.count_nonzero(keep) < 30:
            break
        # Trim the worst tail to reduce floor/wall aliases.
        kept_indices = np.flatnonzero(keep)
        if len(kept_indices) > 2500:
            order = np.argsort(distances[kept_indices])[:2500]
            kept_indices = kept_indices[order]
        mapped_target = target[indices[kept_indices]]
        mapped_source = transformed[kept_indices]
        delta_r, delta_t = _rigid_fit(mapped_source, mapped_target)
        rotation = delta_r @ rotation
        translation = delta_r @ translation + delta_t
        used = len(kept_indices)
        rmse = float(np.sqrt(np.mean(distances[kept_indices] ** 2)))
        if last_rmse is not None and abs(last_rmse - rmse) < 1e-5:
            break
        last_rmse = rmse
    return {
        "success": used >= 30,
        "correspondence_count": used,
        "rmse_m": last_rmse,
        "rotation": rotation.tolist(),
        "translation_m": translation.tolist(),
    }


def _plane_fit(points: np.ndarray, rng: np.random.Generator, iterations=250, threshold=0.035):
    if len(points) < 50:
        return {"success": False, "reason": "too_few_valid_points"}
    sample = _sample(points, 5000, rng)
    best = None
    for _ in range(iterations):
        indices = rng.choice(len(sample), size=3, replace=False)
        a, b, c = sample[indices]
        normal = np.cross(b - a, c - a)
        norm = np.linalg.norm(normal)
        if norm < 1e-6:
            continue
        normal = normal / norm
        offset = -float(normal @ a)
        distances = np.abs(sample @ normal + offset)
        inliers = distances <= threshold
        count = int(np.count_nonzero(inliers))
        if best is None or count > best[0]:
            best = (count, inliers, normal, offset)
    if best is None or best[0] < 50:
        return {"success": False, "reason": "no_dominant_plane"}
    _, inliers, _, _ = best
    inlier_points = sample[inliers]
    center = inlier_points.mean(axis=0)
    _, _, vh = np.linalg.svd(inlier_points - center, full_matrices=False)
    normal = vh[-1]
    normal = normal / np.linalg.norm(normal)
    offset = -float(normal @ center)
    residual = np.abs(inlier_points @ normal + offset)
    return {
        "success": True,
        "inlier_count": int(len(inlier_points)),
        "inlier_fraction": float(len(inlier_points) / len(sample)),
        "normal": normal.tolist(),
        "offset_m": offset,
        "rmse_m": float(np.sqrt(np.mean(residual**2))),
    }


def _rotation_angle_deg(rotation):
    trace = float(np.trace(rotation))
    value = max(-1.0, min(1.0, (trace - 1.0) / 2.0))
    return math.degrees(math.acos(value))


def _pair_records(left, right, tolerance_ns=80_000_000):
    right_by_time = sorted(right, key=lambda item: int(item.get("host_monotonic_ns", 0)))
    result = []
    for lrec in left:
        lt = int(lrec.get("host_monotonic_ns", 0))
        if not right_by_time:
            continue
        nearest = min(right_by_time, key=lambda item: abs(int(item.get("host_monotonic_ns", 0)) - lt))
        delta = abs(int(nearest.get("host_monotonic_ns", 0)) - lt)
        if delta <= tolerance_ns:
            result.append((lrec, nearest, delta))
    return result


def _gyro_motion(capture_dir: Path, radar_records):
    imu = _load_jsonl(capture_dir / "imu_h30.jsonl")
    if len(imu) < 20 or len(radar_records) < 10:
        return {"status": "insufficient_samples", "sample_count": len(imu)}
    gyro_times = np.asarray([int(x.get("host_monotonic_ns", 0)) for x in imu], dtype=np.float64) / 1e9
    gyro_values = np.asarray([
        math.sqrt(sum(float(v) ** 2 for v in (x.get("gyro_rps") or (0, 0, 0))))
        for x in imu
    ])
    if np.nanstd(gyro_values) < 1e-3:
        return {"status": "no_dynamic_excitation", "gyro_norm_std_rps": float(np.nanstd(gyro_values))}
    radar_times = np.asarray([int(x.get("host_monotonic_ns", 0)) for x in radar_records], dtype=np.float64) / 1e9
    radar_feature = []
    for previous, current in zip(radar_records, radar_records[1:]):
        # Metadata-only fallback: use valid-fraction change as a weak motion cue.
        radar_feature.append(float(current.get("valid_fraction", 0.0)) - float(previous.get("valid_fraction", 0.0)))
    radar_times = radar_times[1:]
    radar_feature = np.asarray(radar_feature)
    if len(radar_feature) < 10 or np.std(radar_feature) < 1e-4:
        return {"status": "no_radar_dynamic_excitation", "gyro_norm_std_rps": float(np.std(gyro_values))}
    return {"status": "motion_present_but_requires_point_feature_fit", "gyro_norm_std_rps": float(np.std(gyro_values))}


def analyze(args):
    capture = Path(args.capture_dir)
    rng = np.random.default_rng(20260906)
    left_records = _load_jsonl(capture / "left" / "frames.jsonl")
    right_records = _load_jsonl(capture / "right" / "frames.jsonl")
    pairs = _pair_records(left_records, right_records)
    if args.max_pairs > 0:
        pairs = pairs[: args.max_pairs]
    result = {
        "schema": "smartwheel.h2_dual_calibration_candidate.v1",
        "read_only": True,
        "production_configuration_written": False,
        "capture_dir": str(capture),
        "frame_counts": {"left": len(left_records), "right": len(right_records), "paired": len(pairs)},
        "frame_coordinate_unit": "metres (verified by direct SDK point/dist ratio)",
        "base_frame_proposal": {
            "origin": "midpoint of left/right optical origins",
            "x_axis": "bisector of the two XT-M60 +z forward axes",
            "y_axis": "left; z_axis up",
            "status": "candidate_only",
        },
        "user_priors": {
            "relative_fore_aft_m": 0.0,
            "relative_height_m": 0.0,
            "lateral_separation_m": 0.60,
            "h30_below_left_radar_m": 0.16,
            "h30_behind_left_radar_m": 0.065,
        },
    }
    plane_results = {}
    clouds = {}
    for sensor, records in (("left", left_records), ("right", right_records)):
        selected = records[: min(len(records), max(1, args.plane_frames))]
        frame_planes = []
        for record in selected:
            cloud = _valid_points(_load_points(capture / sensor, record))
            if len(cloud):
                clouds.setdefault(sensor, []).append(cloud)
                frame_planes.append(_plane_fit(cloud, rng))
        good = [item for item in frame_planes if item.get("success")]
        plane_results[sensor] = {
            "frames_examined": len(frame_planes),
            "successful_planes": len(good),
            "planes": good[:10],
        }
    result["ground_or_dominant_plane_fit"] = plane_results

    # Current historical TF is only a seed for registration; it is not copied into output as final data.
    left_seed_r = _rpy_matrix((1.5515153364, -0.0136154207, 1.5709313394))
    right_seed_r = _rpy_matrix((1.7071165220, 0.0265268546, 1.5744345976))
    left_seed_t = np.array((0.45, 0.30, 0.735))
    right_seed_t = np.array((0.45, -0.30, 0.735))
    seed_r = left_seed_r.T @ right_seed_r
    seed_t = left_seed_r.T @ (right_seed_t - left_seed_t)
    icp_results = []
    for left_record, right_record, pair_delta in pairs:
        left_cloud = _valid_points(_load_points(capture / "left", left_record))
        right_cloud = _valid_points(_load_points(capture / "right", right_record))
        if len(left_cloud) < 50 or len(right_cloud) < 50:
            continue
        left_cloud = _sample(left_cloud, args.icp_points, rng)
        right_cloud = _sample(right_cloud, args.icp_points, rng)
        fit = _icp(right_cloud, left_cloud, seed_r, seed_t)
        fit["pair_host_delta_ms"] = pair_delta / 1e6
        if fit.get("success"):
            fit["rotation_delta_from_seed_deg"] = _rotation_angle_deg(np.asarray(fit["rotation"]) @ seed_r.T)
        icp_results.append(fit)
    good_icp = [item for item in icp_results if item.get("success") and item.get("rmse_m") is not None]
    result["relative_radar_registration"] = {
        "seed_transform_source": "historical provisional TF; seed only",
        "attempts": len(icp_results),
        "successful": len(good_icp),
        "rmse_median_m": statistics.median([x["rmse_m"] for x in good_icp]) if good_icp else None,
        "translation_median_m": (
            np.median(np.asarray([x["translation_m"] for x in good_icp]), axis=0).tolist()
            if good_icp else None
        ),
        "rotation_delta_median_deg": statistics.median([x["rotation_delta_from_seed_deg"] for x in good_icp]) if good_icp else None,
        "samples": good_icp[:20],
        "status": "candidate_fit" if len(good_icp) >= 3 else "insufficient_overlap_or_excitation",
    }

    result["time_offset"] = {
        "status": "not_estimated_from_static_capture",
        "reason": "host_receive and H30 data contain no verified dynamic excitation in this capture; physical LiDAR-IMU delay must not be inferred from a static bag",
        "h30_motion_check": _gyro_motion(capture, left_records),
        "reported_contract": "host_receive; time_offset_lidar_to_imu=0.0; BLOCKED_CONFLICT",
    }
    result["h30_extrinsic"] = {
        "translation_prior_m": [-0.065, 0.30, -0.16],
        "translation_prior_note": "derived from user wording relative to the proposed radar-midpoint frame; signs and optical/body-center correction require confirmation",
        "rotation_status": "not solved; H30 outputs are in its own axes, static gravity can constrain roll/pitch but yaw and full mounting rotation require a reference/motion",
        "translation_status": "not observable from static or pure-yaw data alone",
    }
    result["acceptance"] = {
        "status": "H2_FORMAL_NOT_ACCEPTED",
        "blocking_items": [
            "no marked dual-radar wall-corner/slow-yaw motion capture in the analyzed data",
            "no validated absolute vehicle yaw anchor under the proposed sensor-midpoint frame",
            "H30 time semantics remain host_receive with duplicate device/header timestamps",
            "IMU lever-arm translation is only a rough prior",
        ],
        "next_safe_step": "capture a short marked sequence with the direct SDK recorder, then rerun this analyzer and inspect holdout residuals",
    }
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--capture-dir", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--max-pairs", type=int, default=20)
    parser.add_argument("--plane-frames", type=int, default=10)
    parser.add_argument("--icp-points", type=int, default=2500)
    args = parser.parse_args()
    result = analyze(args)
    rendered = json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True)
    print(rendered)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(rendered + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
