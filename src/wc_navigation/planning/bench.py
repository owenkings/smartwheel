"""Planner bench scoring: recorded start/goal requests and path metrics against the static map (no ROS).

Requests come from a frozen recording: the recorded axle pose at time t is the
start and the pose ``lead_s`` later the goal, so every planner answers the same
question the navigate shadow run asks. Paths are scored with the true body (no
margin) against the occupied cells of the same map: the gap the body would keep,
whether it would overlap an obstacle or unknown space, how sharply the path
turns, in-place rotations, reversing and how far it strays from the driver's route.
"""
from __future__ import annotations

import math

import numpy as np

from ..geometry import Grid
from ..autonomy.shadow import interpolate_poses, polyline_distance, wrap
from ..autonomy.world import SimWorld

SHARP_WINDOW_M = 0.3          # a turn of more than SHARP_TURN_RAD within this arc needs a radius below ~0.38 m
SHARP_TURN_RAD = math.radians(45.0)
GAP_CAP_M = 1.0


def make_requests(times, poses, window, lead_s, every_s, min_distance_m=1.0, spacing_m=0.1):
    """Start at the recorded pose at t, goal at t + lead (window end at most); driver route in between."""
    t0, t1 = float(window[0]), float(window[1])
    requests = []
    for t in np.arange(t0, t1 - 1.0, float(every_s)):
        goal_t = min(t + float(lead_s), t1)
        start, goal = interpolate_poses(times, poses, [t, goal_t])
        if math.hypot(goal[0] - start[0], goal[1] - start[1]) < min_distance_m:
            continue
        route = interpolate_poses(times, poses, np.arange(t, goal_t, 0.05))
        kept = [route[0]]
        for pose in route[1:]:
            if math.hypot(pose[0] - kept[-1][0], pose[1] - kept[-1][1]) >= spacing_m:
                kept.append(pose)
        kept.append(goal)
        requests.append({"index": len(requests), "t": round(float(t), 3), "goal_t": round(float(goal_t), 3),
                         "start": [float(v) for v in start], "goal": [float(v) for v in goal],
                         "driver": np.asarray(kept, dtype=float)})
    return requests


class MapGeometry:
    """Occupied cells as obstacles (gaps), occupied or unknown as blocked (what navigation must avoid)."""

    def __init__(self, data, resolution, origin):
        data = np.asarray(data)
        origin = (float(origin[0]), float(origin[1]), 0.0)
        self.occupied = SimWorld(Grid(np.where(data == 100, 100, 0).astype(np.int8), resolution, origin))
        self.blocked = SimWorld(Grid(np.where(data == 0, 0, 100).astype(np.int8), resolution, origin))
        self.resolution = float(resolution)

    def gap(self, pose, body):
        return self.occupied.body_clearance(pose, body, GAP_CAP_M)

    def hits_occupied(self, pose, body):
        return self.occupied.body_collides(pose, body)

    def hits_blocked(self, pose, body):
        return self.blocked.body_collides(pose, body)

    def passage_width(self, pose, max_range=3.0):
        """Free width across the heading at ``pose``: blocked distance to the left plus to the right."""
        x, y, yaw = (float(v) for v in pose)
        world = self.blocked
        left = world.raycast((x, y), [yaw + math.pi / 2], max_range)[0]
        right = world.raycast((x, y), [yaw - math.pi / 2], max_range)[0]
        return float(min(left, max_range) + min(right, max_range))


def resample(xy, spacing):
    """Points every ``spacing`` metres of arc length (ends kept) and their arc length."""
    xy = np.asarray(xy, dtype=float).reshape(-1, 2)
    if len(xy) < 2:
        return xy, np.zeros(len(xy))
    step = np.hypot(*np.diff(xy, axis=0).T)
    s = np.concatenate([[0.0], np.cumsum(step)])
    if s[-1] < 1e-9:
        return xy[:1], np.zeros(1)
    query = np.append(np.arange(0.0, s[-1], spacing), s[-1])
    keep = np.concatenate([[True], np.diff(s) > 1e-12])
    return np.column_stack([np.interp(query, s[keep], xy[keep, 0]), np.interp(query, s[keep], xy[keep, 1])]), query


def tangent_headings(xy):
    """Heading of travel at each resampled point (central differences)."""
    if len(xy) < 2:
        return np.zeros(len(xy))
    d = np.gradient(xy, axis=0)
    return np.arctan2(d[:, 1], d[:, 0])


def turn_profile(headings, s, window_m=SHARP_WINDOW_M):
    """Sharp-turn count and the smallest turning radius over arc windows of ``window_m``."""
    if len(headings) < 3 or s[-1] < window_m:
        return 0, math.inf
    unwrapped = np.unwrap(headings)
    k = max(1, int(round(window_m / max(1e-9, s[1] - s[0]))))
    change = np.abs(unwrapped[k:] - unwrapped[:-k])
    sharp = change > SHARP_TURN_RAD
    count = int(np.sum(sharp[1:] & ~sharp[:-1]) + (1 if sharp[:1].any() else 0))
    largest = float(change.max()) if len(change) else 0.0
    return count, (window_m / largest if largest > 1e-9 else math.inf)


def score_path(geometry, body, xy, yaw=None, *, driver_xy=None, spacing_m=0.1):
    """Metrics of one planned path for the true body.

    ``yaw`` (planner orientations) is used for outline-aware planners whose
    paths carry the heading they checked; otherwise the travel direction is
    the heading, as a pursuit controller would drive it.
    """
    xy = np.asarray(xy, dtype=float).reshape(-1, 2)
    out = {"points": int(len(xy))}
    if len(xy) < 2:
        out.update(length_m=0.0)
        return out
    step = np.hypot(*np.diff(xy, axis=0).T)
    out["length_m"] = float(step.sum())
    in_place = 0.0
    reverse = 0.0
    if yaw is not None:
        yaw = np.asarray(yaw, dtype=float)
        turn = np.abs(wrap(np.diff(yaw)))
        in_place = float(turn[step < 0.01].sum())
        travel = np.arctan2(np.diff(xy[:, 1]), np.diff(xy[:, 0]))
        moving = step >= 0.01
        reverse = float(step[moving & (np.cos(travel - yaw[:-1]) < 0.0)].sum())
    points, s = resample(xy, spacing_m)
    headings = tangent_headings(points)
    if yaw is not None:      # the planner's own heading at the nearest raw point
        nearest = np.argmin(np.hypot(points[:, None, 0] - xy[None, :, 0], points[:, None, 1] - xy[None, :, 1]), axis=1)
        headings = yaw[nearest]
    poses = np.column_stack([points, headings])
    gaps = np.array([geometry.gap(p, body) for p in poses])
    overlap_occupied = int(sum(geometry.hits_occupied(p, body) for p in poses[gaps <= 0.0]))
    overlap_blocked = int(sum(geometry.hits_blocked(p, body) for p in poses))
    sharp, radius = turn_profile(tangent_headings(points), s)
    out.update(min_gap_m=float(gaps.min()), p10_gap_m=float(np.percentile(gaps, 10)),
               median_gap_m=float(np.median(gaps)), poses_scored=int(len(poses)),
               poses_overlapping_occupied=overlap_occupied, poses_overlapping_blocked=overlap_blocked,
               fraction_gap_below_0_05_m=float(np.mean(gaps < 0.05)),
               fraction_gap_below_0_10_m=float(np.mean(gaps < 0.10)),
               sharp_turns=sharp, min_turn_radius_m=(None if not math.isfinite(radius) else float(radius)),
               in_place_rotation_rad=in_place, reverse_m=reverse)
    if driver_xy is not None and len(driver_xy):
        deviation = polyline_distance(np.asarray(driver_xy)[:, :2], xy)
        out.update(max_deviation_from_driver_m=float(deviation.max()),
                   mean_deviation_from_driver_m=float(deviation.mean()))
    return out


def request_context(geometry, body, margin_m, request):
    """Per request: narrowest passage the driver used, and whether start/goal fit the padded body."""
    from .body import Body, grow_outline
    driver = request["driver"]
    widths = [geometry.passage_width(p) for p in driver]
    if body.is_polygon:
        padded = Body(outline=tuple(map(tuple, grow_outline(np.asarray(body.outline), margin_m))),
                      source=body.source)
    else:
        padded = Body(radius_m=body.radius_m + margin_m, source=body.source)
    inscribed = Body(radius_m=padded.inscribed_m)
    start, goal = request["start"], request["goal"]
    true_overlap = int(sum(geometry.hits_blocked(p, body) for p in driver))
    padded_overlap = int(sum(geometry.hits_blocked(p, padded) for p in driver))
    return {"min_passage_width_m": float(min(widths)), "driver_min_gap_m": float(min(geometry.gap(p, body) for p in driver)),
            "start_fits_padded_body": not geometry.hits_blocked(start, padded),
            "goal_fits_padded_body": not geometry.hits_blocked(goal, padded),
            "start_fits_inscribed_disc": not geometry.hits_blocked(start, inscribed),
            "goal_fits_inscribed_disc": not geometry.hits_blocked(goal, inscribed),
            "driver_overlaps_true_body": true_overlap, "driver_overlaps_padded_body": padded_overlap,
            "category": route_category(true_overlap, padded_overlap)}


CATEGORIES = ("clear", "tight", "conflict")


def route_category(true_overlap, padded_overlap):
    """How the driver's own route fits the map for this body.

    ``clear``: the body plus the planning margin fits everywhere, so every
    planner should succeed. ``tight``: the bare body fits but the margin does
    not somewhere (the driver passed closer than the margin). ``conflict``: the
    bare body overlaps occupied or unknown cells on the driver's route (a map
    artefact, a door mapped narrower than it is, or a wrong body model); a
    "successful" plan through such a spot overlaps obstacles for the real body.
    """
    if true_overlap:
        return "conflict"
    return "tight" if padded_overlap else "clear"


def _stats(values):
    values = np.asarray([v for v in values if v is not None and math.isfinite(v)], dtype=float)
    if not len(values):
        return None
    return {"count": int(len(values)), "median": float(np.median(values)), "p10": float(np.percentile(values, 10)),
            "p90": float(np.percentile(values, 90)), "max": float(values.max()), "min": float(values.min())}


def summarize(rows, planners, narrow_width_m=1.4):
    """Per planner: all requests, each route category, and routes through a passage below ``narrow_width_m``.

    A "safe" plan has no pose where the true body overlaps occupied or unknown
    cells; "unsafe successes" are plans the real body could not follow.
    """
    out = {}
    for name in planners:
        mine = [r for r in rows if r["planner"] == name]
        groups = {"all": mine, **{c: [r for r in mine if r["context"]["category"] == c] for c in CATEGORIES},
                  "narrow": [r for r in mine if r["context"]["min_passage_width_m"] < narrow_width_m],
                  "narrow_feasible": [r for r in mine if r["context"]["min_passage_width_m"] < narrow_width_m
                                      and r["context"]["category"] != "conflict"]}
        out[name] = {}
        for label, group in groups.items():
            ok = [r for r in group if r["status"] == "SUCCEEDED" and r.get("score", {}).get("length_m", 0) > 0]
            scores = [r["score"] for r in ok]
            safe = [s for s in scores if s["poses_overlapping_blocked"] == 0]
            out[name][label] = {
                "requests": len(group), "succeeded": len(ok),
                "success_fraction": (len(ok) / len(group)) if group else None,
                "safe_succeeded": len(safe), "unsafe_succeeded": len(ok) - len(safe),
                "safe_success_fraction": (len(safe) / len(group)) if group else None,
                "planning_time_s": _stats([r["planning_time_s"] for r in group if r.get("planning_time_s") is not None]),
                "planning_time_s_succeeded": _stats([r["planning_time_s"] for r in ok]),
                "min_gap_m": _stats([s["min_gap_m"] for s in scores]),
                "paths_overlapping_occupied": sum(1 for s in scores if s["poses_overlapping_occupied"] > 0),
                "paths_overlapping_blocked": sum(1 for s in scores if s["poses_overlapping_blocked"] > 0),
                "paths_with_gap_below_0_05_m": sum(1 for s in scores if s["min_gap_m"] < 0.05),
                "sharp_turns_per_path": _stats([s["sharp_turns"] for s in scores]),
                "paths_with_in_place_rotation": sum(1 for s in scores if s["in_place_rotation_rad"] > 0.3),
                "paths_reversing": sum(1 for s in scores if s["reverse_m"] > 0.05),
                "max_deviation_from_driver_m": _stats([s.get("max_deviation_from_driver_m") for s in scores]),
                "length_ratio_to_driver": _stats([s["length_m"] / max(1e-6, r["driver_length_m"]) for s, r in
                                                  zip(scores, ok)]),
            }
    return out
