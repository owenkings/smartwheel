"""Offline check of the obstacle tracks on a frozen recording; no ROS nodes, no devices.

Both lidars are filtered as in the shadow runs (``shadow_obstacles.prepare``: witness, optional
persistence check, optional lidar time offset). At every left frame the left points and the right
lidar's nearest frame (within 0.15 s), each placed at its own recorded pose, go into one
``tracks.Tracker`` update in the recording's odometry frame; the map removes walls and mapped furniture.
The messages in ``tracks.jsonl`` carry the measurement time (left frame stamp minus the lidar time offset),
as the live ``/obstacles/tracks`` will.

On the ZL1107 recordings nobody should walk in front of the chair, so every ``moving`` track is a false
alarm or somebody who was there; the summary lists them with times, places and shapes, per segment
(start static, moving, end static) when the recording has ``--static-before``/``--static-after``.
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

from . import frontend_frames, shadow_obstacles
from .obstacle_cloud import WRAP_DELETE_NOT_WIRED, ObstacleFilter, body_outline
from .tracks import CLASSES, StaticMap, Tracker, TrackerConfig


def _planar(T):
    T = np.asarray(T, dtype=float)
    return float(T[0, 3]), float(T[1, 3]), math.atan2(T[1, 0], T[0, 0])


def run(left_cell, right_cell=None, map_yaml=None, cfg=ObstacleFilter(), tracker_cfg=TrackerConfig(),
        footprint=(), radius_m=None, window_s=None, lidar_time_offset_s=0.0, hashes=None, pair_s=0.15,
        odom_to_map=None):
    """(per-update messages, track summaries, meta). ``odom_to_map``: map pose (x, y, yaw) of the recording's
    odometry frame when the map comes from another recording (as ``--map-alignment-json`` in the shadow runs)."""
    hashes = {} if hashes is None else hashes
    left_cell, left_runtime, _ = frontend_frames.load_runtime(left_cell)
    frames = [{"stamp_ns": int(s), "points": p, "odometry_T": T}
              for s, p, T in frontend_frames.read_cell(left_cell, hashes=hashes)]
    sides = ("left", "right") if right_cell is not None else ("left",)
    rec = shadow_obstacles.prepare(frames, left_runtime, sides=sides, right_cell=right_cell, cfg=cfg,
                                   footprint=footprint, radius_m=radius_m, consensus=right_cell is not None,
                                   hashes=hashes, window_s=window_s, lidar_time_offset_s=lidar_time_offset_s)
    stamps = np.asarray([f["stamp_ns"] for f in frames], dtype=np.int64)
    rec["stamps_ns"] = stamps
    left_lever = frontend_frames.mount(left_runtime, "left")
    path = shadow_obstacles._Path(stamps, [_planar(f["odometry_T"]) for f in frames], left_lever[:2])
    offset_ns = int(round(float(lidar_time_offset_s) * 1e9))
    static_map = StaticMap.from_yaml(map_yaml) if map_yaml else None
    tracker = Tracker(tracker_cfg, static_map, odom_to_map)
    times = {s: shadow_obstacles.relative_times(rec, s) for s in shadow_obstacles.cloud_sides(rec)}
    stamps_of = {s: rec["obstacle_%s_stamps_ns" % s] for s in times}
    levers = {s: np.asarray(rec["obstacle_%s_lever" % s], dtype=float) for s in times}

    def world(side, k):
        p = shadow_obstacles.frame_points(rec, side, k).astype(float) + levers[side]
        x, y, yaw = path.at(int(stamps_of[side][k]) - offset_ns)
        c, s = math.cos(yaw), math.sin(yaw)
        return np.column_stack([x + c * p[:, 0] - s * p[:, 1], y + s * p[:, 0] + c * p[:, 1],
                                p[:, 2] + cfg.axle_height_m])

    messages, history = [], {}
    for k, t in enumerate(times["left"]):
        if window_s and not window_s[0] <= t <= window_s[1]:
            continue
        points = [world("left", k)]
        seen = ["left"]
        if "right" in times and len(times["right"]):
            j = int(np.argmin(np.abs(times["right"] - t)))
            if abs(times["right"][j] - t) <= pair_s:
                points.append(world("right", j))
                seen.append("right")
        stamp = int(stamps_of["left"][k]) - offset_ns
        before, after = path.at(stamp - 250_000_000), path.at(stamp + 250_000_000)
        chair_velocity = ((after[0] - before[0]) / 0.5, (after[1] - before[1]) / 0.5)
        # tracks are stamped with the measurement time: recording time of the left frame minus the offset
        measured = float(t) - offset_ns * 1e-9
        tracker.update(measured, np.concatenate(points), source=seen, chair_velocity=chair_velocity)
        message = tracker.message(measured)
        messages.append(message)
        for tr in message["tracks"]:
            h = history.setdefault(tr["id"], {"id": tr["id"], "first_s": round(float(t) - tr["age_s"], 2),
                                              "classes": {}, "positions": [], "max_speed": 0.0, "heights": []})
            h["classes"][tr["class"]] = h["classes"].get(tr["class"], 0) + 1
            h["positions"].append((round(float(t), 2), tr["x"], tr["y"], tr["speed"]))
            h["max_speed"] = max(h["max_speed"], tr["speed"])
            h["heights"].append(tr["height_m"])
    meta = {"frames": {s: int(len(v)) for s, v in times.items()}, "updates": len(messages),
            "obstacle_settings": shadow_obstacles.settings(rec)}
    return messages, history, meta


def summarize(history, segments, update_period_s=0.2):
    """Per class: tracks and seconds; moving tracks listed with where and when, per segment."""
    def segment(t):
        for name, a, b in segments:
            if a <= t < b:
                return name
        return "all"
    out = {"classes": {}, "moving_tracks": []}
    for cls in CLASSES:
        ids = [h for h in history.values() if h["classes"].get(cls)]
        out["classes"][cls] = {"tracks": len(ids),
                               "seconds": round(update_period_s * sum(h["classes"][cls] for h in ids), 1)}
    for h in history.values():
        n = h["classes"].get("moving", 0)
        if not n:
            continue
        pos = h["positions"]
        heights = np.asarray(h["heights"], dtype=float)
        out["moving_tracks"].append({
            "id": h["id"], "segment": segment(pos[0][0]), "from_s": pos[0][0], "to_s": pos[-1][0],
            "moving_s": round(update_period_s * n, 1), "max_speed_mps": round(h["max_speed"], 2),
            "start_xy": [pos[0][1], pos[0][2]], "end_xy": [pos[-1][1], pos[-1][2]],
            "path_m": round(float(sum(math.hypot(b[1] - a[1], b[2] - a[2]) for a, b in zip(pos, pos[1:]))), 2),
            "height_m": [round(float(heights[:, 0].min()), 2), round(float(heights[:, 1].max()), 2)],
            "classes": h["classes"]})
    by_segment = {}
    for m in out["moving_tracks"]:
        by_segment.setdefault(m["segment"], 0)
        by_segment[m["segment"]] += 1
    out["moving_tracks_by_segment"] = by_segment
    return out


def arguments(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--project-root", type=Path, required=True)
    parser.add_argument("--left-cell", type=Path, required=True)
    parser.add_argument("--right-cell", type=Path)
    parser.add_argument("--map", type=Path, help="trinary map YAML (in the recording's odometry frame unless "
                                                  "--map-alignment-json places the recording in it)")
    parser.add_argument("--map-alignment-json", type=Path,
                        help="alignment check result with best_alignment (dx_m, dy_m, yaw_deg): the map pose of the "
                             "recording's odometry frame, as in the shadow runs' fixed localization")
    parser.add_argument("--footprint", nargs="+", type=float, metavar="X Y")
    parser.add_argument("--footprint-source", default="EXAMPLE_NOT_MEASURED")
    parser.add_argument("--assumed-radius", type=float, default=0.55)
    parser.add_argument("--window", nargs=2, type=float, metavar=("START", "END"))
    parser.add_argument("--filter", default="{}", help="JSON object of ObstacleFilter overrides")
    parser.add_argument("--tracker", default="{}", help="JSON object of TrackerConfig overrides")
    parser.add_argument("--lidar-time-offset-ms", type=float, default=0.0, metavar="MS")
    parser.add_argument("--static-before", type=float, help="recording seconds before which the chair stands")
    parser.add_argument("--static-after", type=float, help="recording seconds after which the chair stands")
    parser.add_argument("--label", default="obstacle_tracks")
    args = parser.parse_args(argv)
    if args.footprint is not None:
        if len(args.footprint) < 6 or len(args.footprint) % 2:
            parser.error("--footprint needs at least three x y pairs")
        args.footprint = [args.footprint[k:k + 2] for k in range(0, len(args.footprint), 2)]
    if not math.isfinite(args.lidar_time_offset_ms) or abs(args.lidar_time_offset_ms) > 250.0:
        parser.error("--lidar-time-offset-ms must be finite and within +-250")
    try:
        args.filter = ObstacleFilter(**json.loads(args.filter))
        args.tracker = TrackerConfig(**json.loads(args.tracker))
    except (TypeError, ValueError) as exc:
        parser.error(str(exc))
    if args.filter.wrap_delete:
        parser.error("--filter: " + WRAP_DELETE_NOT_WIRED)
    return args


def main(argv=None):
    args = arguments(argv)
    from wc_runtime.storage_policy import StoragePolicy
    storage = StoragePolicy(args.project_root)
    storage.check()
    hashes = {}
    alignment = None
    if args.map_alignment_json is not None:
        raw = args.map_alignment_json.read_bytes()
        best = json.loads(raw)["best_alignment"]
        alignment = {"x_m": float(best["dx_m"]), "y_m": float(best["dy_m"]), "yaw_deg": float(best["yaw_deg"]),
                     "source": str(args.map_alignment_json), "sha256": hashlib.sha256(raw).hexdigest()}
    odom_to_map = (alignment["x_m"], alignment["y_m"], math.radians(alignment["yaw_deg"])) if alignment else None
    messages, history, meta = run(args.left_cell, args.right_cell, args.map, args.filter, args.tracker,
                                  args.footprint or (), None if args.footprint else args.assumed_radius,
                                  args.window, args.lidar_time_offset_ms * 1e-3, hashes, odom_to_map=odom_to_map)
    segments = []
    if args.static_before is not None:
        segments.append(("start_static", -1e9, args.static_before))
    lo = args.static_before if args.static_before is not None else -1e9
    hi = args.static_after if args.static_after is not None else 1e9
    segments.append(("moving", lo, hi))
    if args.static_after is not None:
        segments.append(("end_static", args.static_after, 1e9))
    stamps = np.asarray([m["stamp"] for m in messages], dtype=float)
    period = float(np.median(np.diff(stamps))) if len(stamps) > 1 else 0.2
    summary = summarize(history, segments, update_period_s=period)
    summary["update_period_s"] = round(period, 3)
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    output = storage.resolve("reports/navigation_obstacles/" + stamp + "_" + args.label + "_" + uuid.uuid4().hex[:6])
    output.mkdir(parents=True, exist_ok=False)
    map_sha = hashlib.sha256(Path(args.map).read_bytes()).hexdigest() if args.map else None
    result = {"schema_version": 1, "kind": "OBSTACLE_TRACKS_OFFLINE", "left_cell": str(args.left_cell),
              "right_cell": str(args.right_cell) if args.right_cell else None,
              "map_yaml": str(args.map) if args.map else None, "map_yaml_sha256": map_sha,
              "map_alignment": alignment,
              "filter": args.filter.as_dict(), "tracker": args.tracker.as_dict(),
              "lidar_time_offset_ms": args.lidar_time_offset_ms, "window_s": args.window,
              "stamp_basis": "tracks.jsonl stamps: seconds after the first left frame's header stamp, minus the "
                             "lidar time offset (measurement time); windows, segments and the summary use the "
                             "recording time without the offset",
              "segments": [[n, a, b] for n, a, b in segments],
              "body_model": ("POLYGON_" + args.footprint_source) if args.footprint else "CIRCLE_ASSUMED_NOT_MEASURED",
              "meta": {k: v for k, v in meta.items() if k != "obstacle_settings"}, "summary": summary,
              "source_files_hashed": len(hashes), "hardware_started": False,
              "interpretation": "moving tracks on a recording without walking people are false alarms or "
                                "people who were there; check them against the recording"}
    with (output / "result.json").open("x", encoding="utf-8") as stream:
        json.dump(result, stream, indent=1)
    with (output / "tracks.jsonl").open("x", encoding="utf-8") as stream:
        for message in messages:
            stream.write(json.dumps(message) + "\n")
    print(json.dumps({"output": str(output), "summary": {k: v for k, v in summary.items() if k != "moving_tracks"},
                      "moving_tracks": len(summary["moving_tracks"])}, indent=1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
