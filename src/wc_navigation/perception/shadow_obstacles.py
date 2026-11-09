"""Obstacle clouds for shadow runs: filtered once from the frozen frontends, replayed at recorded times.

``prepare`` runs inside ``shadow_cli.prepare_recording`` when ``--obstacle-input`` is
not ``scan``. It filters every frame of the requested lidars (left: the frames the
shadow run already loaded and hash-checked; right: a second frozen cell, hashes
added to the run's source check), using the other lidar's nearest frame as the
witness for unsupported clusters, and returns arrays for ``recording.npz``:

``obstacle_<side>_stamps_ns`` (n), ``obstacle_<side>_offsets`` (n + 1),
``obstacle_<side>_points`` (M x 3 float32, lidar frame), ``obstacle_<side>_lever`` (3),
``obstacle_<side>_clear_offsets`` (n + 1), ``obstacle_<side>_clear_points`` (K x 3
float16 clearing-ray ends, lidar frame), optionally ``obstacle_left_scan_ranges``
(n x 161 float32, the filtered scan), ``obstacle_settings`` (JSON text).

``replay_node`` publishes them on ``/obstacles/<side>/points`` and
``/obstacles/<side>/clearing`` in ``obstacles_lidar_<side>``, and the filtered scan on
``/obstacles/left/scan`` in ``laser``.
"""
from __future__ import annotations

import json
import math

import numpy as np

from . import frontend_frames
from .consensus import RecentFrames, Witness, move, planar_relative
from .nav2_obstacles import CLEAR_TOPICS, CLOUD_FRAMES, CLOUD_TOPICS, FILTERED_SCAN_TOPIC, SIDES
from .obstacle_cloud import SCAN_BINS, STAGES, ObstacleFilter, body_outline, filter_frame

SYMMETRIC_RIGHT_NOTE = ("right lever arm mirrored from the left mount (CAD-symmetric housing) instead of the "
                        "frozen 2026-10-26 field extrinsic, which puts the right lidar about 8.6 cm further forward")


def _planar(T):
    T = np.asarray(T, dtype=float)
    return float(T[0, 3]), float(T[1, 3]), math.atan2(T[1, 0], T[0, 0])


class _Path:
    """Planar axle poses of the left cell (the shadow run's motion source), interpolated by stamp."""

    def __init__(self, stamps_ns, lidar_poses, lever_xy):
        self.stamps = np.asarray(stamps_ns, dtype=np.int64)
        p = np.asarray(lidar_poses, dtype=float)
        c, s = np.cos(p[:, 2]), np.sin(p[:, 2])
        lx, ly = float(lever_xy[0]), float(lever_xy[1])
        self.poses = np.column_stack([p[:, 0] - c * lx + s * ly, p[:, 1] - s * lx - c * ly, p[:, 2]])
        self.t = (self.stamps - self.stamps[0]) * 1e-9

    def at(self, stamp_ns):
        t = (int(stamp_ns) - int(self.stamps[0])) * 1e-9
        k = int(np.clip(np.searchsorted(self.t, t, side="right") - 1, 0, len(self.t) - 2))
        a = float(np.clip((t - self.t[k]) / max(self.t[k + 1] - self.t[k], 1e-9), 0.0, 1.0))
        p, q = self.poses[k], self.poses[k + 1]
        dyaw = math.atan2(math.sin(q[2] - p[2]), math.cos(q[2] - p[2]))
        return (p[0] + a * (q[0] - p[0]), p[1] + a * (q[1] - p[1]), p[2] + a * dyaw)


def symmetric_right_lever(left_lever):
    left = np.asarray(left_lever, dtype=float)
    return np.array([left[0], -left[1], left[2]])


def prepare(left_frames, left_runtime, sides=("left", "right"), right_cell=None, cfg=ObstacleFilter(),
            footprint=(), radius_m=None, consensus=True, right_lever=None, witness_max_age_s=0.2, hashes=None,
            window_s=None, scan=False, lidar_time_offset_s=0.0):
    """Filtered obstacle clouds of the requested lidars, packed for recording.npz.

    ``window_s`` (recording seconds from the first left frame) limits the work to
    the frames a shadow run will publish, plus one second on each side. ``scan``
    also stores the left lidar's filtered 161-bin scan (``obstacle_left_scan_ranges``).
    ``lidar_time_offset_s``: the lidar stamps are late by this against the odometry,
    so every frame of both lidars is placed at the recorded pose of stamp - offset
    (witness and persistence geometry; the replay stamps the messages that much
    earlier). Stamps in the recording stay as recorded.
    """
    offset_s = float(lidar_time_offset_s)
    if not math.isfinite(offset_s) or abs(offset_s) > 0.25:
        raise ValueError("lidar time offset must be finite and within +-0.25 s")
    offset_ns = int(round(offset_s * 1e9))
    if scan and "left" not in sides:
        raise ValueError("the filtered scan comes from the left lidar")
    sides = tuple(s for s in SIDES if s in sides)
    if not sides:
        raise ValueError("at least one lidar side is required")
    left_lever = frontend_frames.mount(left_runtime, "left")
    frames = {"left": [(int(f["stamp_ns"]), np.asarray(f["points"], dtype=np.float32), f["odometry_T"])
                       for f in left_frames]}
    levers = {"left": left_lever}
    lever_basis = {"left": "FROZEN_FRONTEND_MOUNT"}
    if "right" in sides or (consensus and right_cell is not None):
        if right_cell is None:
            raise ValueError("the right lidar needs --obstacle-right-cell (a frozen single-right frontend)")
        cell, runtime, side = frontend_frames.load_runtime(right_cell)
        if side != "right":
            raise ValueError("--obstacle-right-cell must be a frozen single-right frontend")
        start, end = frames["left"][0][0] - 10 ** 9, frames["left"][-1][0] + 10 ** 9
        frames["right"] = list(frontend_frames.read_cell(cell, hashes=hashes, start_ns=start, end_ns=end))
        if right_lever == "symmetric":
            levers["right"] = symmetric_right_lever(left_lever)
            lever_basis["right"] = "SYMMETRIC_TO_LEFT_MOUNT"
        elif right_lever is not None:
            levers["right"] = np.asarray(right_lever, dtype=float).reshape(3)
            lever_basis["right"] = "EXPLICIT_OVERRIDE"
        else:
            levers["right"] = frontend_frames.mount(runtime, "right")
            lever_basis["right"] = "FROZEN_FRONTEND_MOUNT_FIELD_EXTRINSIC_20261026"
    path = _Path([f[0] for f in frames["left"]], [_planar(f[2]) for f in frames["left"]], left_lever[:2])
    body = body_outline(footprint, radius_m)
    stamps_of = {s: np.asarray([f[0] for f in v], dtype=np.int64) for s, v in frames.items()}
    out, totals = {}, {}
    for side in sides:
        other = "right" if side == "left" else "left"
        chunks, offsets, stamps = [], [0], []
        clear_chunks, clear_offsets = [], [0]
        scans = []
        with_scan = scan and side == "left"
        total = {"frames": 0, "frames_with_witness": 0, "frames_with_recent": 0}
        history = (RecentFrames(cfg.persist_min_age_s, cfg.persist_max_age_s, cfg.cluster_voxel_m)
                   if cfg.persist_min_age_s > 0 else None)
        for stamp, pts, _ in frames[side]:
            if stamp < path.stamps[0] - 10 ** 9 or stamp > path.stamps[-1]:
                continue
            store = True
            if window_s is not None:
                t = (stamp - int(path.stamps[0])) * 1e-9
                # the persistence check looks back persist_max_age_s: those frames only warm the history
                lead = cfg.persist_max_age_s if history is not None else 0.0
                if not window_s[0] - 1.0 - lead <= t <= window_s[1] + 1.0:
                    continue
                store = t >= window_s[0] - 1.0
            witness = None
            if consensus and other in frames:
                j = int(np.argmin(np.abs(stamps_of[other] - stamp)))
                if abs(int(stamps_of[other][j]) - stamp) <= witness_max_age_s * 1e9:
                    relative = planar_relative(path.at(int(stamps_of[other][j]) - offset_ns), path.at(stamp - offset_ns))
                    witness = Witness(move(frames[other][j][1].astype(float) + levers[other], relative),
                                      move(levers[other][None, :], relative)[0])
            recent = None
            if history is not None:
                pose, t_s = path.at(stamp - offset_ns), (stamp - offset_ns) * 1e-9
                recent = history.recent(t_s, pose)
            result = filter_frame(pts, levers[side], cfg, body, witness, clearing=True, scan=with_scan,
                                  recent=recent)
            if history is not None:
                history.add(t_s, pose, result["candidates"])
            if not store:
                continue
            if history is not None:
                total["frames_with_recent"] += int(recent is not None)
            kept, stats, rays = result["points"], result["stats"], result["rays"]
            if with_scan:
                scans.append(result["scan"])
            chunks.append(kept)
            offsets.append(offsets[-1] + len(kept))
            clear_chunks.append(rays.astype(np.float16))      # ray ends only: 1 cm steps are plenty
            clear_offsets.append(clear_offsets[-1] + len(rays))
            stamps.append(stamp)
            total["frames"] += 1
            total["frames_with_witness"] += int(witness is not None)
            for key, value in stats.items():
                if isinstance(value, (int, np.integer)) and not isinstance(value, bool):
                    total[key] = total.get(key, 0) + int(value)
        out["obstacle_%s_stamps_ns" % side] = np.asarray(stamps, dtype=np.int64)
        out["obstacle_%s_offsets" % side] = np.asarray(offsets, dtype=np.int64)
        out["obstacle_%s_points" % side] = (np.concatenate(chunks).astype(np.float32) if chunks
                                            else np.zeros((0, 3), dtype=np.float32))
        out["obstacle_%s_lever" % side] = np.asarray(levers[side], dtype=float)
        out["obstacle_%s_clear_offsets" % side] = np.asarray(clear_offsets, dtype=np.int64)
        out["obstacle_%s_clear_points" % side] = (np.concatenate(clear_chunks) if clear_chunks
                                                  else np.zeros((0, 3), dtype=np.float16))
        if with_scan:
            out["obstacle_left_scan_ranges"] = (np.stack(scans).astype(np.float32) if scans
                                                else np.zeros((0, SCAN_BINS), dtype=np.float32))
        totals[side] = total
    settings = {"sides": list(sides), "filter": cfg.as_dict(), "consensus": bool(consensus),
                "witness_max_age_s": witness_max_age_s, "lidar_time_offset_s": offset_s,
                "levers": {s: levers[s].tolist() for s in levers},
                "window_s": list(window_s) if window_s is not None else None,
                "lever_basis": lever_basis, "right_cell": str(right_cell) if right_cell is not None else None,
                "body": [list(p) for p in body], "body_source": "FOOTPRINT" if footprint else "ASSUMED_CIRCLE",
                "topics": {s: CLOUD_TOPICS[s] for s in sides}, "clearing_topics": {s: CLEAR_TOPICS[s] for s in sides},
                "filtered_scan_topic": FILTERED_SCAN_TOPIC if scan else None,
                "frames": {s: CLOUD_FRAMES[s] for s in sides},
                "stage_order": list(STAGES), "totals": totals,
                "right_lever_note": SYMMETRIC_RIGHT_NOTE if lever_basis.get("right") == "SYMMETRIC_TO_LEFT_MOUNT"
                else None}
    out["obstacle_settings"] = np.asarray(json.dumps(settings))
    return out


def settings(recording):
    raw = recording.get("obstacle_settings")
    return json.loads(str(raw)) if raw is not None else None


def cloud_sides(recording):
    return [s for s in SIDES if "obstacle_%s_points" % s in recording]


def frame_points(recording, side, k):
    offsets = recording["obstacle_%s_offsets" % side]
    return recording["obstacle_%s_points" % side][offsets[k]:offsets[k + 1]]


def frame_scan(recording, side, k):
    """Filtered 161-bin scan of frame ``k`` of the left lidar, or None."""
    if side != "left" or "obstacle_left_scan_ranges" not in recording:
        return None
    return recording["obstacle_left_scan_ranges"][k]


def frame_clearing(recording, side, k):
    """Clearing ray ends of frame ``k`` (float32), or an empty array for recordings without them."""
    key = "obstacle_%s_clear_offsets" % side
    if key not in recording:
        return np.zeros((0, 3), dtype=np.float32)
    offsets = recording[key]
    return recording["obstacle_%s_clear_points" % side][offsets[k]:offsets[k + 1]].astype(np.float32)


def relative_times(recording, side):
    """Capture times of one lidar's obstacle frames in the recording's time base (times_s)."""
    return (recording["obstacle_%s_stamps_ns" % side] - int(recording["stamps_ns"][0])).astype(np.float64) * 1e-9


def zone_audit(recording, window, stop_polygon, slow_polygon, max_points):
    """Per lidar frame inside ``window``: obstacle points in the stop zone and the slowdown box (base_link).

    Uses each cloud at its own capture pose, like the scan audit. Returns counts and,
    for frames whose stop zone holds more than ``max_points`` points, the heights and
    ranges of those points so a stop can be traced to its returns.
    """
    from ..autonomy.shadow import inside_polygon
    axle_height = float(((settings(recording) or {}).get("filter") or {}).get("axle_height_m", 0.185))
    out = {}
    for side in cloud_sides(recording):
        times = relative_times(recording, side)
        lever = np.asarray(recording["obstacle_%s_lever" % side], dtype=float)
        sel = np.flatnonzero((times >= window[0] - 1e-9) & (times <= window[1] + 1e-9))
        stop_frames, slow_frames, nearest, examples = 0, 0, [], []
        for k in sel:
            pts = frame_points(recording, side, k).astype(float) + lever
            if not len(pts):
                nearest.append(math.inf)
                continue
            d = np.hypot(pts[:, 0], pts[:, 1])
            nearest.append(float(d.min()))
            stop = inside_polygon(pts[:, :2], stop_polygon)
            slow = inside_polygon(pts[:, :2], slow_polygon)
            stop_frames += int(stop.sum() > max_points)
            slow_frames += int(slow.sum() > max_points)
            if stop.sum() > max_points and len(examples) < 50:
                h = pts[stop, 2] + axle_height
                r = np.hypot(pts[stop, 0] - lever[0], pts[stop, 1] - lever[1])
                examples.append({"time_s": round(float(times[k]), 3), "points": int(stop.sum()),
                                 "height_m": [round(float(h.min()), 3), round(float(h.max()), 3)],
                                 "range_from_lidar_m": [round(float(r.min()), 3), round(float(r.max()), 3)],
                                 "xy_mean": [round(float(pts[stop, 0].mean()), 3), round(float(pts[stop, 1].mean()), 3)]})
        finite = np.asarray([v for v in nearest if math.isfinite(v)])
        out[side] = {"frames": int(len(sel)), "frames_stop_expected": stop_frames, "frames_slow_expected": slow_frames,
                     "nearest_point_to_axle_m": ({"median": float(np.median(finite)), "p10": float(np.percentile(finite, 10)),
                                                  "min": float(finite.min())} if len(finite) else None),
                     "stop_examples": examples}
    return out
