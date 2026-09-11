"""Read-only multi-frame floor candidates; a visible plane is not floor proof.

XT-M60 SDK axes are +x left, +y up, +z forward. For the fitted unit normal
pointing up, n.p + d = 0, d > 0 is the perpendicular sensor-origin height.
Range to an arbitrary ground pixel is NOT height, and a single floor cannot
identify yaw, x/y translation, IMU lever arm or timing.
"""

import math

import numpy as np


def fit_floor_plane(points, *, iterations=300, threshold_m=0.03,
                    vertical_tolerance_deg=25.0, height_bounds_m=(0.3, 1.3)):
    """RANSAC floor candidate, constrained by SDK up and below-sensor sign."""
    points = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    points = points[np.isfinite(points).all(axis=1)]
    if len(points) < 30:
        return None
    rng = np.random.default_rng(20260910)
    if len(points) > 4000:
        points = points[rng.choice(len(points), 4000, replace=False)]
    cosine_limit = math.cos(math.radians(vertical_tolerance_deg))
    best_mask = None
    best_count = 0
    for _ in range(iterations):
        a, b, c = points[rng.choice(len(points), 3, replace=False)]
        normal = np.cross(b - a, c - a)
        norm = np.linalg.norm(normal)
        if norm < 1e-9:
            continue
        normal /= norm
        if normal[1] < 0:
            normal = -normal
        d = -float(normal @ a)
        if normal[1] < cosine_limit or not height_bounds_m[0] <= d <= height_bounds_m[1]:
            continue
        mask = np.abs(points @ normal + d) <= threshold_m
        count = int(mask.sum())
        if count > best_count:
            best_count, best_mask = count, mask
    if best_mask is None or best_count < 30:
        return None
    inliers = points[best_mask]
    center = inliers.mean(axis=0)
    _, _, vh = np.linalg.svd(inliers - center, full_matrices=False)
    normal = vh[-1]
    if normal[1] < 0:
        normal = -normal
    d = -float(normal @ center)
    # Refinement can move outside the RANSAC constraints: recheck it.
    if normal[1] < cosine_limit or not height_bounds_m[0] <= d <= height_bounds_m[1]:
        return None
    residual = np.abs(points @ normal + d)
    mask = residual <= threshold_m
    inliers = points[mask]
    planar_projection = (inliers - center) @ vh[:2].T
    planar_span = np.percentile(planar_projection, 95, axis=0) - np.percentile(planar_projection, 5, axis=0)
    return {
        "normal_xyz": normal.tolist(), "d": d,
        "distance_from_sensor_m": d,
        "plane_y_at_sensor_xz_m": -d / float(normal[1]),
        "inlier_count": int(mask.sum()),
        "inlier_ratio": float(mask.mean()),
        "residual_mean_m": float(residual[mask].mean()),
        "residual_p95_m": float(np.percentile(residual[mask], 95)),
        "inlier_centroid_xyz_m": inliers.mean(axis=0).tolist(),
        "in_plane_p05_p95_span_m": planar_span.tolist(),
    }


def evaluate_floor_frames(frames, *, stamps=None, frame_ids=None,
                          floor_roi_confirmed=False, stationary_level_confirmed=False,
                          min_frames=20, min_duration_sec=1.5,
                          min_inlier_ratio=0.20, max_residual_p95_m=0.025,
                          max_height_spread_m=0.025, max_normal_spread_deg=1.0,
                          **fit_options):
    """Return evidence and a candidate only when geometry AND context pass.

    Even a stable desk/table can satisfy geometry. Explicit floor and stationary
    level setup confirmation is mandatory and does not constitute 6DoF approval.
    """
    fits = [fit_floor_plane(frame, **fit_options) for frame in frames]
    accepted = [plane for plane in fits if plane is not None
                and plane["inlier_count"] >= 100
                and plane["inlier_ratio"] >= min_inlier_ratio
                and plane["residual_p95_m"] <= max_residual_p95_m
                and min(plane["in_plane_p05_p95_span_m"]) >= 0.20]
    heights = np.asarray([plane["d"] for plane in accepted])
    normals = np.asarray([plane["normal_xyz"] for plane in accepted])
    height_spread = normal_spread = None
    normal = None
    if len(accepted):
        normal = normals.mean(axis=0)
        normal /= np.linalg.norm(normal)
        height_spread = float(np.percentile(heights, 95) - np.percentile(heights, 5))
        angles = np.degrees(np.arccos(np.clip(normals @ normal, -1.0, 1.0)))
        normal_spread = float(np.percentile(angles, 95))
    valid_stamps = (stamps is not None and len(stamps) == len(frames)
                    and len(stamps) >= 2 and np.isfinite(stamps).all()
                    and np.min(stamps) > 0 and np.all(np.diff(stamps) > 0))
    checks = {
        "enough_frames": len(accepted) >= min_frames,
        "accepted_fraction_at_least_80pct": len(accepted) >= 0.8 * max(1, len(frames)),
        "monotonic_source_timestamps": bool(valid_stamps),
        "sufficient_duration": bool(valid_stamps and stamps[-1] - stamps[0] >= min_duration_sec),
        "single_sensor_frame": (frame_ids is not None and len(frame_ids) == len(frames)
                                and len(set(frame_ids)) == 1 and bool(frame_ids[0])),
        "height_stable": height_spread is not None and height_spread <= max_height_spread_m,
        "normal_stable": normal_spread is not None and normal_spread <= max_normal_spread_deg,
        "floor_roi_confirmed": bool(floor_roi_confirmed),
        "stationary_level_confirmed": bool(stationary_level_confirmed),
    }
    eligible = all(checks.values())
    return {
        "status": "CANDIDATE_REVIEW_REQUIRED" if eligible else "REJECTED_OR_CONTEXT_UNCONFIRMED",
        "read_only": True, "automatic_apply_allowed": False,
        "formal_runtime_eligible": False,
        "frames": len(frames), "accepted_frames": len(accepted),
        "checks": checks, "per_frame_planes": fits,
        "height_median_m": float(np.median(heights)) if len(heights) else None,
        "sensor_to_floor_distance_m": float(np.median(heights)) if len(heights) else None,
        "base_to_sensor_z_m": None,
        "base_floor_offset_m": None,
        "height_p05_p95_spread_m": height_spread,
        "normal_xyz": normal.tolist() if normal is not None else None,
        "normal_p95_deviation_deg": normal_spread,
        "unobservable": ["yaw", "x", "y", "lidar_imu_lever_arm", "time_offset"],
        "caveat": "Height is distance from SDK origin to selected plane, NOT automatically base-to-sensor TF z. Floor identity, level base, and base-origin floor offset require independent confirmation.",
    }
