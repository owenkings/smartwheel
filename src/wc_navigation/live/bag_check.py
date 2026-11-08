"""Compare a recorded live-input run with the frozen frontend of the same recording (read-only).

Scans: every frozen frame is matched to the live scan with the same stamp
(within ``match_tolerance_s``); bins are equal when both are missing (+inf) or
differ by at most ``range_tolerance_m``. Odometry: the live axle trajectory is
placed on the frozen one at the first common frame (the two odometry frames
start at their own initialization) and compared at every frozen frame stamp.
Neither side is ground truth; this checks that the live chain reproduces the
offline one.
"""
from __future__ import annotations

import csv
import math
from pathlib import Path

import numpy as np


def read_live_scans(path):
    stamps, ranges = [], []
    with open(path, newline="", encoding="utf-8") as stream:
        reader = csv.reader(stream)
        header = next(reader)
        first = header.index("r000") if "r000" in header else None
        if first is None:
            raise ValueError("the run did not record scan ranges")
        for row in reader:
            stamps.append(float(row[header.index("stamp_s")]))
            ranges.append([math.inf if v in ("inf", "") else float(v) for v in row[first:first + 161]])
    return np.asarray(stamps), np.asarray(ranges, dtype=float).reshape(-1, 161)


def read_live_odometry(path):
    rows = []
    with open(path, newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            rows.append((float(row["stamp_s"]), float(row["x"]), float(row["y"]), float(row["yaw"])))
    data = np.asarray(rows, dtype=float).reshape(-1, 4)
    keep = np.concatenate([[True], np.diff(data[:, 0]) > 0]) if len(data) else np.zeros(0, dtype=bool)
    return data[keep]


def compare_scans(frozen_stamps, frozen_ranges, live_stamps, live_ranges, match_tolerance_s=5e-4,
                  range_tolerance_m=1e-4):
    order = np.argsort(live_stamps)
    live_stamps, live_ranges = live_stamps[order], live_ranges[order]
    matched, equal_frames, differing_bins, max_diff, unmatched = 0, 0, [], 0.0, 0
    offsets = []
    for stamp, ranges in zip(frozen_stamps, frozen_ranges):
        k = int(np.searchsorted(live_stamps, stamp))
        candidates = [j for j in (k - 1, k) if 0 <= j < len(live_stamps)]
        if not candidates:
            unmatched += 1
            continue
        j = min(candidates, key=lambda j: abs(live_stamps[j] - stamp))
        if abs(live_stamps[j] - stamp) > match_tolerance_s:
            unmatched += 1
            continue
        offsets.append(live_stamps[j] - stamp)
        matched += 1
        a, b = np.asarray(ranges, dtype=float), live_ranges[j]
        both_missing = np.isinf(a) & np.isinf(b)
        finite = np.isfinite(a) & np.isfinite(b)
        diff = np.zeros(len(a))
        diff[finite] = np.abs(a[finite] - b[finite])
        equal = both_missing | (finite & (diff <= range_tolerance_m))
        differing_bins.append(int((~equal).sum()))
        if finite.any():
            max_diff = max(max_diff, float(diff[finite].max()))
        equal_frames += int(equal.all())
    differing = np.asarray(differing_bins)
    return {"frozen_frames": int(len(frozen_stamps)), "matched_frames": matched, "unmatched_frames": unmatched,
            "identical_frames": equal_frames, "frames_with_differing_bins": int((differing > 0).sum()),
            "differing_bins_total": int(differing.sum()), "bins_compared": int(161 * matched),
            "max_abs_difference_finite_m": max_diff,
            "stamp_offset_s": {"max_abs": float(np.max(np.abs(offsets))) if offsets else None},
            "tolerances": {"match_s": match_tolerance_s, "range_m": range_tolerance_m}}


def _wrap(angle):
    return np.arctan2(np.sin(angle), np.cos(angle))


def compare_odometry(frozen_stamps, frozen_axle, live):
    """Place the live axle path on the frozen one at the first common frame; differences at frozen stamps."""
    from ..autonomy.shadow_localization import compose_poses
    from ..autonomy.shadow import interpolate_poses
    inside = (frozen_stamps >= live[0, 0]) & (frozen_stamps <= live[-1, 0])
    if inside.sum() < 2:
        return {"status": "NO_OVERLAP"}
    stamps, axle = frozen_stamps[inside], frozen_axle[inside]
    at = interpolate_poses(live[:, 0], live[:, 1:4], stamps)
    x0, y0, a0 = at[0]
    c, s = math.cos(a0), math.sin(a0)
    inverse_first = (-c * x0 - s * y0, s * x0 - c * y0, -a0)
    placed = compose_poses(tuple(axle[0]), compose_poses(inverse_first, at))
    position = np.hypot(placed[:, 0] - axle[:, 0], placed[:, 1] - axle[:, 1])
    heading = np.degrees(np.abs(_wrap(placed[:, 2] - axle[:, 2])))
    path = float(np.sum(np.hypot(*np.diff(axle[:, :2], axis=0).T)))
    stats = lambda v: {"median": round(float(np.median(v)), 4), "p95": round(float(np.quantile(v, 0.95)), 4),
                       "max": round(float(v.max()), 4), "final": round(float(v[-1]), 4)}
    return {"status": "COMPARED", "frames": int(len(stamps)), "path_length_m": round(path, 2),
            "position_difference_m": stats(position), "heading_difference_deg": stats(heading),
            "first_common_stamp_s": float(stamps[0]),
            "basis": "live axle path placed on the frozen one at the first common frame; not ground truth"}


def frozen_frontend(cell):
    """Frozen frame stamps (s), 161-bin scans and axle poses of a single-left frontend cell."""
    from .amcl_cross import load_frames
    from ..amcl_replay import build_scan
    from ..autonomy.replay_mapping import planar
    from ..autonomy.shadow import base_poses_from_lidar
    source, frames, lever, hashes = load_frames(Path(cell))
    stamps = np.array([f["stamp_ns"] * 1e-9 for f in frames])
    ranges = np.array([build_scan(f["points"], -0.5, 0.5)[0] for f in frames], dtype=float)
    lidar = np.array([planar(f["odometry_T"]) for f in frames])
    return stamps, ranges, base_poses_from_lidar(lidar, lever), hashes


def check_run(run_dir, cell, lidar_time_offset_ms=None):
    """``lidar_time_offset_ms``: the run stamped its scans this much earlier than the lidar frames.

    None reads the value the run recorded in run_config.json (runs from before the option: 0).
    """
    import json
    run_dir = Path(run_dir)
    if lidar_time_offset_ms is None:
        recorded = run_dir / "run_config.json"
        config = json.loads(recorded.read_text(encoding="utf-8")) if recorded.is_file() else {}
        lidar_time_offset_ms = float(config.get("lidar_time_offset_ms", 0.0))
    stamps, ranges, axle, hashes = frozen_frontend(cell)
    live_stamps, live_ranges = read_live_scans(run_dir / "scans.csv")
    odometry = read_live_odometry(run_dir / "odom.csv")
    expected = stamps - float(lidar_time_offset_ms) * 1e-3
    result = {"frozen_cell": str(cell), "live_scans": int(len(live_stamps)), "live_odometry_samples": int(len(odometry)),
              "lidar_time_offset_ms": float(lidar_time_offset_ms),
              "scans": compare_scans(expected, ranges, live_stamps, live_ranges),
              "odometry": compare_odometry(stamps, axle, odometry) if len(odometry) >= 2 else {"status": "NO_ODOMETRY"}}
    if len(live_stamps) > 1:
        result["live_scan_rate_hz"] = round(float((len(live_stamps) - 1) / (live_stamps.max() - live_stamps.min())), 2)
    if len(odometry) > 1:
        result["live_odometry_rate_hz"] = round(float((len(odometry) - 1) / (odometry[-1, 0] - odometry[0, 0])), 2)
    return result
