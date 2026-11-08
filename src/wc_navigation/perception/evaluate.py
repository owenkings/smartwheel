"""Offline check of an obstacle filter on a frozen recording; no ROS nodes, no devices.

False positives without ground truth: the recording's own future path. A static
obstacle cannot lie where the wheelchair drove in the next few seconds, so every
obstacle point inside the footprint swept along the next ``horizon_s`` of the
recorded axle path (outline shrunk by ``shrink_m``) is either an artefact or a
moving object that left in time. The same count is made for the 161-bin /scan
that the shadow runs use today, so old and new inputs are compared on equal
terms. Points within ``near_m`` of the swept path are counted separately (what a
slowdown zone would react to).

The left frontend's axle path is the time base; right-lidar frames are placed at
the left path interpolated to their own capture stamps (same recording clock).
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import uuid

import numpy as np

from . import frontend_frames
from .consensus import RecentFrames, Witness, move, planar_relative
from .obstacle_cloud import ObstacleFilter, body_outline, filter_frame, grow_convex, inside_convex

SCAN_ANGLE_MIN, SCAN_INCREMENT, SCAN_BINS = -math.pi / 3, math.pi / 240, 161


def planar(T):
    T = np.asarray(T, dtype=float)
    return float(T[0, 3]), float(T[1, 3]), math.atan2(T[1, 0], T[0, 0])


class AxlePath:
    """Recorded planar axle poses with interpolation (yaw along the shortest arc)."""

    def __init__(self, stamps_ns, lidar_poses, lever_xy):
        self.t = (np.asarray(stamps_ns, dtype=np.int64) - int(stamps_ns[0])) * 1e-9
        self.t0_ns = int(stamps_ns[0])
        poses = np.asarray(lidar_poses, dtype=float)
        lx, ly = float(lever_xy[0]), float(lever_xy[1])
        c, s = np.cos(poses[:, 2]), np.sin(poses[:, 2])
        self.poses = np.column_stack([poses[:, 0] - c * lx + s * ly, poses[:, 1] - s * lx - c * ly, poses[:, 2]])
        steps = np.hypot(*np.diff(self.poses[:, :2], axis=0).T)
        self.distance = np.concatenate([[0.0], np.cumsum(steps)])

    def at(self, t):
        t = np.atleast_1d(np.asarray(t, dtype=float))
        k = np.clip(np.searchsorted(self.t, t, side="right") - 1, 0, len(self.t) - 2)
        a = np.clip((t - self.t[k]) / np.maximum(self.t[k + 1] - self.t[k], 1e-9), 0.0, 1.0)
        p, q = self.poses[k], self.poses[k + 1]
        dyaw = np.arctan2(np.sin(q[:, 2] - p[:, 2]), np.cos(q[:, 2] - p[:, 2]))
        return np.column_stack([p[:, :2] + a[:, None] * (q[:, :2] - p[:, :2]), p[:, 2] + a * dyaw])

    def odometer(self, t):
        return np.interp(t, self.t, self.distance)


def to_world(points_base_xy, pose):
    c, s = math.cos(pose[2]), math.sin(pose[2])
    x, y = points_base_xy[:, 0], points_base_xy[:, 1]
    return np.column_stack([pose[0] + c * x - s * y, pose[1] + s * x + c * y])


def to_base(points_world_xy, pose):
    c, s = math.cos(pose[2]), math.sin(pose[2])
    dx, dy = points_world_xy[:, 0] - pose[0], points_world_xy[:, 1] - pose[1]
    return np.column_stack([c * dx + s * dy, -s * dx + c * dy])


def swept_hits(world_xy, path, t, outline, horizon_s, step_s=0.1, min_travel_m=0.05):
    """Mask of points inside the outline at any recorded pose in (t, t + horizon], once the chair has moved."""
    hit = np.zeros(len(world_xy), dtype=bool)
    if not len(world_xy):
        return hit
    start = path.odometer(t)
    for tj in np.arange(t + step_s, t + horizon_s + 1e-9, step_s):
        if tj > path.t[-1]:
            break
        if path.odometer(tj) - start < min_travel_m:
            continue
        hit |= inside_convex(to_base(world_xy, path.at(tj)[0]), outline)
    return hit


def scan_points(points, lever_xy, height_band=(-0.5, 0.5)):
    """The shadow run's /scan returns of one frame as base_link xy (nearest in-band point per 0.75 deg bin)."""
    from ..amcl_replay import build_scan
    ranges, _ = build_scan(points, *height_band)
    ok = np.isfinite(ranges)
    a = SCAN_ANGLE_MIN + SCAN_INCREMENT * np.flatnonzero(ok)
    r = ranges[ok].astype(float)
    return np.column_stack([lever_xy[0] + r * np.cos(a), lever_xy[1] + r * np.sin(a)])


def evaluate(left_cell, right_cell=None, cfg=ObstacleFilter(), footprint=(), radius_m=None, horizon_s=4.0,
             shrink_m=0.03, near_m=0.30, window_s=None, right_lever=None, hashes=None, keep_points=True,
             consensus=True, witness_max_age_s=0.2):
    """Per-frame false-positive counts of the obstacle filter and of the baseline scan."""
    hashes = {} if hashes is None else hashes
    left_cell, left_runtime, _ = frontend_frames.load_runtime(left_cell)
    left_lever = frontend_frames.mount(left_runtime, "left")
    frames = {"left": list(frontend_frames.read_cell(left_cell, hashes=hashes))}
    levers = {"left": left_lever}
    if right_cell is not None:
        right_cell, right_runtime, side = frontend_frames.load_runtime(right_cell)
        if side != "right":
            raise ValueError("--right-cell must be a single-right frontend")
        frames["right"] = list(frontend_frames.read_cell(right_cell, hashes=hashes))
        levers["right"] = (np.asarray(right_lever, dtype=float) if right_lever is not None
                           else frontend_frames.mount(right_runtime, "right"))
    stamps = [f[0] for f in frames["left"]]
    path = AxlePath(stamps, [planar(f[2]) for f in frames["left"]], left_lever[:2])
    body = body_outline(footprint, radius_m)
    outline = grow_convex(body, -shrink_m) if body else []
    side_times = {side: np.asarray([(f[0] - path.t0_ns) * 1e-9 for f in side_frames])
                  for side, side_frames in frames.items()}
    rows, kept_points = [], []
    for side, side_frames in frames.items():
        lever = levers[side]
        other = "right" if side == "left" else "left"
        history = (RecentFrames(cfg.persist_min_age_s, cfg.persist_max_age_s, cfg.cluster_voxel_m)
                   if cfg.persist_min_age_s > 0 else None)
        lead = cfg.persist_max_age_s if history is not None else 0.0
        for k, (stamp, pts, _) in enumerate(side_frames):
            t = (stamp - path.t0_ns) * 1e-9
            if t < path.t[0] - 0.5 or t > path.t[-1]:
                continue
            if window_s and not window_s[0] - lead <= t <= window_s[1]:
                continue
            in_window = not window_s or t >= window_s[0]       # earlier frames only warm the history
            pose = path.at(t)[0]
            witness = None
            if consensus and other in frames:
                j = int(np.argmin(np.abs(side_times[other] - t)))
                if abs(side_times[other][j] - t) <= witness_max_age_s:
                    relative = planar_relative(path.at(side_times[other][j])[0], pose)
                    witness = Witness(move(frames[other][j][1].astype(float) + levers[other], relative),
                                      move(levers[other][None, :], relative)[0])
            recent = history.recent(t, pose) if history is not None else None
            result = filter_frame(pts, lever, cfg, body, witness, recent=recent)
            obstacles, stats = result["points"], result["stats"]
            if history is not None:
                history.add(t, pose, result["candidates"])
            if not in_window:
                continue
            stats["witness"] = witness is not None
            stats["recent"] = recent is not None
            base = obstacles.astype(float) + lever
            world = to_world(base[:, :2], pose)
            hit = swept_hits(world, path, t, outline, horizon_s)
            near = swept_hits(world, path, t, grow_convex(body, near_m), horizon_s) & ~hit if body else hit
            h = base[:, 2] + cfg.axle_height_m
            row = {"side": side, "frame": k, "t": round(t, 3), "voxels": int(len(obstacles)), **stats,
                   "swept_voxels": int(hit.sum()), "near_voxels": int(near.sum()),
                   "swept_heights": [round(float(h[hit].min()), 3), round(float(h[hit].max()), 3)] if hit.any() else None}
            if side == "left":
                scan = scan_points(pts, lever[:2])
                scan_world = to_world(scan, pose)
                scan_hit = swept_hits(scan_world, path, t, outline, horizon_s)
                row["scan_returns"] = int(len(scan))
                row["scan_swept_returns"] = int(scan_hit.sum())
            rows.append(row)
            if keep_points and hit.any():
                r = np.hypot(obstacles[hit, 0], obstacles[hit, 1])
                kept_points.append(np.column_stack([np.full(hit.sum(), 0 if side == "left" else 1), np.full(hit.sum(), t),
                                                    world[hit], h[hit], r]))
    points = np.concatenate(kept_points) if kept_points else np.zeros((0, 6))
    return rows, points, {"levers": {s: v.tolist() for s, v in levers.items()}, "frames": {s: len(f) for s, f in frames.items()},
                          "path_length_m": float(path.distance[-1]), "duration_s": float(path.t[-1])}


def summarize(rows, horizon_s):
    out = {}
    for side in ("left", "right"):
        sel = [r for r in rows if r["side"] == side]
        if not sel:
            continue
        n = len(sel)
        swept = np.asarray([r["swept_voxels"] for r in sel])
        near = np.asarray([r["near_voxels"] for r in sel])
        removed = {k[len("removed_"):]: int(sum(r.get(k, 0) for r in sel)) for k in sel[0] if k.startswith("removed_")}
        out[side] = {"frames": n, "frames_with_swept_obstacle": int((swept > 0).sum()),
                     "frames_with_4plus_swept_voxels": int((swept >= 4).sum()), "swept_voxels_total": int(swept.sum()),
                     "frames_with_near_path_obstacle": int((near > 0).sum()),
                     "mean_voxels_per_frame": float(np.mean([r["voxels"] for r in sel])),
                     "removed_points_by_stage": removed}
        if side == "left":
            scan = np.asarray([r["scan_swept_returns"] for r in sel])
            out["scan_baseline"] = {"frames": n, "frames_with_swept_return": int((scan > 0).sum()),
                                    "frames_with_4plus_swept_returns": int((scan >= 4).sum()),
                                    "swept_returns_total": int(scan.sum())}
    out["horizon_s"] = horizon_s
    return out


def arguments(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--project-root", type=Path, required=True)
    parser.add_argument("--left-cell", type=Path, required=True)
    parser.add_argument("--right-cell", type=Path)
    parser.add_argument("--footprint", nargs="+", type=float, metavar="X Y")
    parser.add_argument("--footprint-source", default="EXAMPLE_NOT_MEASURED")
    parser.add_argument("--assumed-radius", type=float, default=0.55)
    parser.add_argument("--horizon", type=float, default=4.0, help="seconds of future path that must be free")
    parser.add_argument("--window", nargs=2, type=float, metavar=("START", "END"))
    parser.add_argument("--right-lever", nargs=3, type=float, metavar=("X", "Y", "Z"),
                        help="override the right lidar lever arm (default: frozen in the right cell)")
    parser.add_argument("--filter", default="{}", help="JSON object of ObstacleFilter overrides")
    parser.add_argument("--no-consensus", action="store_true",
                        help="do not use the other lidar as witness for unsupported clusters")
    parser.add_argument("--label", default="obstacle_eval")
    args = parser.parse_args(argv)
    if args.footprint is not None:
        if len(args.footprint) < 6 or len(args.footprint) % 2:
            parser.error("--footprint needs at least three x y pairs")
        args.footprint = [args.footprint[k:k + 2] for k in range(0, len(args.footprint), 2)]
    if not 0.5 <= args.horizon <= 20:
        parser.error("--horizon must be within 0.5..20 s")
    args.filter = ObstacleFilter(**json.loads(args.filter))
    return args


def main(argv=None):
    args = arguments(argv)
    from wc_runtime.storage_policy import StoragePolicy
    storage = StoragePolicy(args.project_root)
    storage.check()
    hashes = {}
    rows, points, meta = evaluate(args.left_cell, args.right_cell, args.filter, args.footprint or (),
                                  None if args.footprint else args.assumed_radius, args.horizon,
                                  window_s=args.window, right_lever=args.right_lever, hashes=hashes,
                                  consensus=not args.no_consensus)
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    output = storage.resolve("reports/navigation_obstacles/" + stamp + "_" + args.label + "_" + uuid.uuid4().hex[:6])
    output.mkdir(parents=True, exist_ok=False)
    summary = summarize(rows, args.horizon)
    # index and runtime files of both cells (the per-frame CDR files were checked while reading)
    unchanged = all(hashlib.sha256(Path(p).read_bytes()).hexdigest() == d for p, d in hashes.items()
                    if p.endswith(("index.jsonl", "runtime_config.json")))
    result = {"schema_version": 1, "kind": "OBSTACLE_FILTER_FUTURE_PATH_CHECK", "left_cell": str(args.left_cell),
              "right_cell": str(args.right_cell) if args.right_cell else None, "filter": args.filter.as_dict(),
              "consensus": not args.no_consensus,
              "body_model": ("POLYGON_" + args.footprint_source) if args.footprint else "CIRCLE_ASSUMED_NOT_MEASURED",
              "footprint": args.footprint, "assumed_radius_m": None if args.footprint else args.assumed_radius,
              "window_s": args.window, "meta": meta, "summary": summary, "source_files_hashed": len(hashes),
              "sources_unchanged": unchanged, "hardware_started": False,
              "interpretation": ("points inside the footprint swept along the next horizon of the recorded path; "
                                 "static obstacles cannot be there, so these are artefacts or objects that moved")}
    with (output / "result.json").open("x", encoding="utf-8") as stream:
        json.dump(result, stream, indent=1)
    with (output / "frames.jsonl").open("x", encoding="utf-8") as stream:
        for row in rows:
            stream.write(json.dumps(row) + "\n")
    with (output / "swept_points.npz").open("xb") as stream:
        np.savez_compressed(stream, points=points,
                            columns=np.asarray(["side(0=left,1=right)", "t_s", "x_odom", "y_odom", "h_m", "range_m"]))
    print(json.dumps({"output": str(output), "summary": summary}, indent=1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
