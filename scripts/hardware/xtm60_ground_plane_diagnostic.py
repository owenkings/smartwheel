#!/usr/bin/env python3
"""Fit a floor plane from bounded XT-M60 PointCloud2 input to verify axes."""

import argparse
import json
import math
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2

from xtm60_cloud_diagnostic import _points_array
from wheelchair_3d_mapping.ground_plane_candidate import evaluate_floor_frames, fit_floor_plane


class CloudCollector(Node):
    def __init__(self, topic, frame_limit, save_raw=False):
        super().__init__("xtm60_ground_plane_diagnostic")
        self.frame_limit = frame_limit
        self.frames = []
        self.frame_ids = set()
        self.source_frames = []
        self.stamps = []
        self.save_raw = save_raw
        self.raw_frames = []
        self.create_subscription(PointCloud2, topic, self.on_cloud, qos_profile_sensor_data)

    def on_cloud(self, message):
        if len(self.frames) >= self.frame_limit:
            return
        points = _points_array(message)
        if not all(axis in points.dtype.names for axis in ("x", "y", "z")):
            return
        xyz = np.column_stack((points["x"], points["y"], points["z"])).astype(
            np.float64, copy=False
        )
        valid = np.all(np.isfinite(xyz), axis=1)
        ranges = np.linalg.norm(xyz, axis=1)
        valid &= (ranges >= 0.3) & (ranges <= 12.0)
        xyz = xyz[valid]
        if xyz.size:
            self.frames.append(xyz)
            self.frame_ids.add(message.header.frame_id)
            self.source_frames.append(message.header.frame_id)
            self.stamps.append(message.header.stamp.sec + message.header.stamp.nanosec * 1e-9)
            if self.save_raw:
                self.raw_frames.append(points.copy())


def fit_horizontal_plane(points, iterations, threshold_m, vertical_tolerance_deg):
    rng = np.random.default_rng(20260722)
    cosine_limit = math.cos(math.radians(vertical_tolerance_deg))
    best_mask = None
    best_count = 0
    for _ in range(iterations):
        indices = rng.choice(points.shape[0], size=3, replace=False)
        p0, p1, p2 = points[indices]
        normal = np.cross(p1 - p0, p2 - p0)
        norm = np.linalg.norm(normal)
        if norm < 1e-9:
            continue
        normal /= norm
        if abs(normal[1]) < cosine_limit:
            continue
        distance = np.abs(points @ normal - np.dot(p0, normal))
        mask = distance <= threshold_m
        count = int(np.count_nonzero(mask))
        if count > best_count:
            best_count = count
            best_mask = mask
    if best_mask is None:
        return None

    inliers = points[best_mask]
    centroid = np.mean(inliers, axis=0)
    _, _, vh = np.linalg.svd(inliers - centroid, full_matrices=False)
    normal = vh[-1]
    normal /= np.linalg.norm(normal)
    if normal[1] < 0:
        normal = -normal
    d = -float(np.dot(normal, centroid))
    residuals = np.abs(points @ normal + d)
    refined_mask = residuals <= threshold_m
    refined = points[refined_mask]
    return {
        "normal_xyz": [float(value) for value in normal],
        "d": d,
        "distance_from_sensor_m": abs(d),
        "plane_y_at_sensor_xz_m": -d / float(normal[1]),
        "inlier_count": int(refined.shape[0]),
        "inlier_ratio": float(refined.shape[0] / points.shape[0]),
        "residual_mean_m": float(np.mean(residuals[refined_mask])),
        "residual_p95_m": float(np.percentile(residuals[refined_mask], 95)),
        "inlier_centroid_xyz_m": [float(value) for value in np.mean(refined, axis=0)],
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", default="/xtm60/left/points")
    parser.add_argument("--frame-count", type=int, default=30)
    parser.add_argument("--timeout-sec", type=float, default=10.0)
    parser.add_argument("--max-points", type=int, default=100000)
    parser.add_argument("--iterations", type=int, default=300)
    parser.add_argument("--threshold-m", type=float, default=0.03)
    parser.add_argument("--vertical-tolerance-deg", type=float, default=25.0)
    parser.add_argument("--floor-roi-confirmed", action="store_true",
                        help="Operator confirms visible candidate surface is the floor, not a desk/ceiling")
    parser.add_argument("--stationary-level-confirmed", action="store_true",
                        help="Operator confirms chassis is stationary on a level floor")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--raw-output", type=Path,
                        help="Optional NPZ of original structured XYZ/intensity frames and source stamps")
    args = parser.parse_args()

    rclpy.init()
    node = CloudCollector(args.topic, args.frame_count, save_raw=args.raw_output is not None)
    started = time.monotonic()
    try:
        while (
            len(node.frames) < args.frame_count
            and time.monotonic() - started < args.timeout_sec
        ):
            rclpy.spin_once(node, timeout_sec=0.2)
        frame_ids = sorted(node.frame_ids)
        frames = list(node.frames)
        stamps = list(node.stamps)
        source_frames = list(node.source_frames)
        raw_frames = list(node.raw_frames)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    if args.raw_output is not None:
        args.raw_output.parent.mkdir(parents=True, exist_ok=True)
        np.savez_compressed(args.raw_output, stamps=np.asarray(stamps),
                            frame_ids=np.asarray(source_frames),
                            **{f"frame_{i:04d}": frame for i, frame in enumerate(raw_frames)})

    points = np.concatenate(frames) if frames else np.empty((0, 3), dtype=np.float64)
    if points.shape[0] > args.max_points:
        rng = np.random.default_rng(20260722)
        points = points[rng.choice(points.shape[0], args.max_points, replace=False)]
    plane = (
        fit_floor_plane(points, iterations=args.iterations, threshold_m=args.threshold_m,
                        vertical_tolerance_deg=args.vertical_tolerance_deg)
        if points.shape[0] >= 3
        else None
    )
    positive_z_fraction = float(np.mean(points[:, 2] > 0)) if points.size else 0.0
    result = {
        "schema": "smartwheel.xtm60_ground_plane_diagnostic.v2",
        "read_only": True,
        "topic": args.topic,
        "frame_ids": frame_ids,
        "frames_collected": len(frames),
        "points_used": int(points.shape[0]),
        "ransac": {
            "iterations": args.iterations,
            "threshold_m": args.threshold_m,
            "vertical_tolerance_deg": args.vertical_tolerance_deg,
        },
        "plane": plane,
        "quality": evaluate_floor_frames(
            frames, stamps=stamps, frame_ids=source_frames,
            floor_roi_confirmed=args.floor_roi_confirmed,
            stationary_level_confirmed=args.stationary_level_confirmed,
            iterations=args.iterations, threshold_m=args.threshold_m,
            vertical_tolerance_deg=args.vertical_tolerance_deg),
        "positive_z_fraction": positive_z_fraction,
    }
    result["checks"] = {
        "frames_present": len(frames) > 0,
        "horizontal_plane_found": plane is not None,
        "floor_below_sensor_if_y_is_up": (
            plane is not None and plane["plane_y_at_sensor_xz_m"] < 0
        ),
        "floor_distance_plausible": (
            plane is not None and 0.2 <= plane["distance_from_sensor_m"] <= 1.5
        ),
        "forward_axis_is_positive_z": positive_z_fraction >= 0.99,
    }
    rendered = json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True)
    print(rendered)
    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered + "\n", encoding="utf-8")
    return 0 if result["quality"]["status"] == "CANDIDATE_REVIEW_REQUIRED" else 2


if __name__ == "__main__":
    raise SystemExit(main())
