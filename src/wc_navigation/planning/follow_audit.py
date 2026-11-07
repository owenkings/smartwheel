"""Follow-mode audit of a shadow run (no ROS): turning in place, controller stops and speed in narrow spots.

In follow mode the controller gets the driver's own recorded path, which the
real chair drove without touching anything. The audit asks three questions:

1. Turning: how much of the time the driver turns while driving forward the
   controller instead asks to turn in place.
2. Stops: for every controller stop (zero output while the driver moves), was
   anything in the recent scans inside the body outline somewhere along the
   driver's next ``check_m`` of route? If so the scan itself contradicts the
   route (a close or moving object, a near-field or self return, or an outline
   larger than the chair); if not, the controller stopped on its own command
   (its predicted arc or in-place turn), which is the false stop to remove.
3. Narrow spots: commanded speed against the gap between the body and the
   static map along the driver's route.

The scan points are the recording's 161-bin scans of the last ``memory_s``
seconds placed with their own capture poses, an approximation of the local
costmap (marks outside the field of view persist there as well).
"""
from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path

import numpy as np

from ..autonomy.shadow import interpolate_poses, scan_points_base
from .body import Body, grow_outline, point_polygon_distance

GAP_BINS = (0.0, 0.10, 0.20, 0.40, 1.0001)


def _read_samples(path):
    with Path(path).open(encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))

    def column(name):
        return np.array([float(r[name]) if r[name] not in ("", None) else np.nan for r in rows])
    return {"t": column("recording_time_s"), "v_rec": column("recorded_linear"), "w_rec": column("recorded_angular"),
            "moving": np.array([r["recording_moving"] == "1" for r in rows]), "outcome": np.array([r["outcome"] for r in rows]),
            "nav_v": column("nav_linear"), "nav_w": column("nav_angular"), "gated_v": column("gated_linear")}


def world_scan_points(recording, k):
    """Finite returns of frame ``k`` in the map frame (scan frame = lever translation of the axle)."""
    angles = float(recording["angle_min"]) + float(recording["angle_increment"]) * np.arange(recording["ranges"].shape[1])
    local = scan_points_base(angles, recording["ranges"][k], recording["lever_xy"])
    x, y, yaw = recording["base_poses"][k]
    c, s = math.cos(yaw), math.sin(yaw)
    return np.column_stack([x + c * local[:, 0] - s * local[:, 1], y + s * local[:, 0] + c * local[:, 1]])


def route_ahead(times, poses, t, check_m, step_s=0.05, horizon_s=8.0):
    """Recorded poses from ``t`` until ``check_m`` of travel (or ``horizon_s``)."""
    query = np.arange(t, min(t + horizon_s, float(times[-1])) + 1e-9, step_s)
    if not len(query):
        return np.zeros((0, 3))
    route = interpolate_poses(times, poses, query)
    travel = np.concatenate([[0.0], np.cumsum(np.hypot(*np.diff(route[:, :2], axis=0).T))])
    return route[travel <= check_m]


def points_in_body_along(route, points, body, grow):
    """Number of points inside the (grown) outline at any pose of ``route``."""
    if not len(points) or not len(route):
        return 0
    inside = np.zeros(len(points), dtype=bool)
    reach = body.circumscribed_m + grow + 0.05
    for pose in route[::2]:
        near = np.hypot(points[:, 0] - pose[0], points[:, 1] - pose[1]) <= reach
        if not near.any():
            continue
        polygon = body.outline_at(pose, grow) if body.is_polygon else None
        if polygon is None:
            inside[near] |= np.hypot(points[near, 0] - pose[0], points[near, 1] - pose[1]) < body.radius_m + grow
        else:
            inside[near] |= point_polygon_distance(points[near], polygon) <= 0.0
    return int(inside.sum())


def _runs(mask):
    starts = np.flatnonzero(mask & ~np.r_[False, mask[:-1]])
    ends = np.flatnonzero(mask & ~np.r_[mask[1:], False])
    return list(zip(starts, ends))


def audit(run_dir, *, check_m=0.6, memory_s=2.0, grow=0.025, map_geometry=None):
    run = Path(run_dir)
    config = json.loads((run / "run_config.json").read_text(encoding="utf-8"))
    from ..autonomy.nav2_config import NavProfile
    profile = NavProfile(**config["profile"])
    body = Body.from_profile(profile, source=config.get("body_model"))
    with np.load(run / "recording.npz") as data:
        recording = {k: data[k] for k in data.files}
    times, poses = recording["times_s"], recording["base_poses"]
    s = _read_samples(run / "samples.csv")
    moving = s["moving"]
    forward = moving & (s["v_rec"] > 0.10)
    turning_forward = forward & (np.abs(s["w_rec"]) > 0.15)
    rotates = s["outcome"] == "ROTATES_IN_PLACE"
    zero = moving & (s["outcome"] == "CONTROLLER_ZERO")
    stops = []
    for a, b in _runs(zero):
        t = float(s["t"][a])
        frames = np.flatnonzero((times <= t) & (times >= t - memory_s))
        points = np.vstack([world_scan_points(recording, k) for k in frames]) if len(frames) else np.zeros((0, 2))
        route = route_ahead(times, poses, t, check_m)
        hits = points_in_body_along(route, points, body, grow)
        previous = "NONE"
        j = a - 1
        while j >= 0 and s["outcome"][j] in ("CONTROLLER_ZERO", "NAV2_NOT_COMMANDING"):
            j -= 1
        if j >= 0:
            previous = s["outcome"][j]
        x, y, yaw = interpolate_poses(times, poses, t)[0]
        stops.append({"t": round(t, 2), "duration_s": round(float(s["t"][b] - s["t"][a]) + 0.1, 2),
                      "x": round(float(x), 2), "y": round(float(y), 2),
                      "scan_points_in_body_on_driver_route": hits, "previous_outcome": previous,
                      "driver_turn_rate": round(float(s["w_rec"][a]), 3)})
    caused = [p for p in stops if p["scan_points_in_body_on_driver_route"] > 0]
    out = {"run": run.name, "body_model": config.get("body_model"), "planning": config.get("planning"),
           "window_s": config.get("window_s"), "moving_s": round(float(moving.sum()) * 0.1, 1),
           "turn_in_place_while_driver_turns_forward_s": round(float((rotates & turning_forward).sum()) * 0.1, 1),
           "driver_turns_forward_s": round(float(turning_forward.sum()) * 0.1, 1),
           "turn_in_place_while_driver_drives_forward_s": round(float((rotates & forward).sum()) * 0.1, 1),
           "turn_in_place_while_moving_s": round(float((rotates & moving).sum()) * 0.1, 1),
           "controller_stops": len(stops), "controller_stop_s": round(float(zero.sum()) * 0.1, 1),
           "stops_with_scan_points_on_driver_route": len(caused),
           "stops_without_scan_points_on_driver_route": len(stops) - len(caused),
           "stop_s_without_scan_points": round(sum(p["duration_s"] for p in stops if p not in caused), 1),
           "check": {"route_ahead_m": check_m, "scan_memory_s": memory_s, "outline_grown_m": grow},
           "stops": stops}
    if map_geometry is not None:
        out["speed_by_static_gap"] = speed_by_gap(s, times, poses, body, map_geometry, forward)
    return out


def speed_by_gap(s, times, poses, body, geometry, forward):
    """Median commanded and gated speed while the driver drives forward, by the body's gap to the static map."""
    idx = np.flatnonzero(forward)[::5]                      # every 0.5 s
    at = interpolate_poses(times, poses, s["t"][idx])
    gaps = np.array([geometry.gap(p, body) for p in at])
    table = []
    for lo, hi in zip(GAP_BINS[:-1], GAP_BINS[1:]):
        sel = (gaps >= lo) & (gaps < hi)
        k = idx[sel]
        def med(values):
            values = values[np.isfinite(values)]
            return None if not len(values) else round(float(np.median(values)), 3)
        table.append({"gap_m": [lo, round(hi, 2)], "samples": int(sel.sum()),
                      "driver_mps": med(s["v_rec"][k]), "nav_mps": med(s["nav_v"][k]), "gated_mps": med(s["gated_v"][k]),
                      "controller_zero_fraction": (round(float(np.mean(s["outcome"][k] == "CONTROLLER_ZERO")), 3)
                                                   if len(k) else None),
                      "forward_fraction": (round(float(np.mean(s["outcome"][k] == "DRIVES_FORWARD")), 3)
                                           if len(k) else None)})
    return table


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("runs", nargs="+", type=Path, help="shadow run directories (follow mode)")
    parser.add_argument("--output", type=Path, required=True, help="JSON file to write (must not exist)")
    parser.add_argument("--check-m", type=float, default=0.6)
    args = parser.parse_args(argv)
    from ..autonomy.maps import read_back
    from .bench import MapGeometry
    results, geometries = [], {}
    for run in args.runs:
        config = json.loads((run / "run_config.json").read_text(encoding="utf-8"))
        key = config["map_yaml"]
        if key not in geometries:
            _, data, meta = read_back(key)
            geometries[key] = MapGeometry(data, meta["resolution"], meta["origin"])
        results.append(audit(run, check_m=args.check_m, map_geometry=geometries[key]))
    with args.output.open("x", encoding="utf-8") as stream:
        json.dump(results, stream, ensure_ascii=False, indent=1)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
