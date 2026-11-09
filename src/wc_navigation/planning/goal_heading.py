"""Goal heading at arrival: an explicit mission option (``--goal-heading turn_at_goal``). Pure geometry, no ROS.

By default a navigation goal's heading goes to the planner. Hybrid-A* and the
State Lattice plan to that heading, so a goal that faces back the way the chair
came makes a forward-only planner loop around inside a small room, and the
chair (which cannot reverse) can end up where no plan starts.

With ``turn_at_goal`` the mission instead:

1. Picks the arrival heading. Heading-aware planners (``smac_hybrid``,
   ``smac_lattice``) are asked for a path to the goal point at several
   candidate headings. The requested heading is kept when its path is no
   longer than the shortest one plus a small slack; otherwise the candidate
   closest to the requested heading among those within the slack is used, so
   no detour is made for the heading. Heading-free planners (NavFn, Smac 2D,
   Theta*) are asked once; the arrival heading is the direction of the path's
   last 0.5 m.
2. Sends NavigateToPose with that heading.
3. On arrival, turns in place to the requested heading only when the area the
   body outline sweeps while turning (for the measured outline at most the
   0.846 m circle) keeps the margin to the saved map's occupied and unknown
   cells and to the local costmap's obstacle cells. The shorter direction is
   tried first, then the other one. Otherwise the chair stays where it is and
   the heading difference is reported.

The turn uses Nav2's Spin behaviour (its own collision check on the local
costmap stays on; the collision monitor and the gate still apply). The chair
has no rear range sensor: an area behind it that was never in view and is not
in the saved map counts as free, the same limitation as any in-place turn.
"""
from __future__ import annotations

import math

import numpy as np

from .body import wrap

MODES = ("planned", "turn_at_goal")
DEFAULT_MODE = "planned"
HEADING_AWARE_PLANNERS = ("smac_hybrid", "smac_lattice")
CANDIDATES = 12                      # every 30 degrees, the requested heading first
LENGTH_SLACK_FRACTION = 0.03         # a heading costs no detour when its path is within 3 % + 0.10 m
LENGTH_SLACK_M = 0.10
ARRIVAL_BACK_M = 0.5                 # heading-free paths: direction of the last 0.5 m
PLAN_TIMEOUT_S = 10.0                # per candidate request (simulated time)
LETHAL_COST = 100                    # nav_msgs/OccupancyGrid value of a costmap obstacle cell


def candidate_headings(requested, count=CANDIDATES):
    """``count`` headings evenly spaced around the circle, starting with ``requested`` (wrapped)."""
    count = int(count)
    if count < 1 or count > 72:
        raise ValueError("candidate count must be 1..72")
    return [wrap(float(requested) + 2.0 * math.pi * k / count) for k in range(count)]


def path_length(xy):
    xy = np.asarray(xy, dtype=float).reshape(-1, 2)
    return float(np.sum(np.hypot(*np.diff(xy, axis=0).T))) if len(xy) > 1 else 0.0


def arrival_direction(xy, back_m=ARRIVAL_BACK_M):
    """Direction of travel over the path's last ``back_m`` metres (None for a path without length)."""
    xy = np.asarray(xy, dtype=float).reshape(-1, 2)
    if len(xy) < 2:
        return None
    end = xy[-1]
    steps = np.hypot(*np.diff(xy, axis=0).T)
    from_end = np.concatenate([np.cumsum(steps[::-1])[::-1], [0.0]])   # distance along the path to the end
    k = int(np.flatnonzero(from_end >= back_m)[-1]) if np.any(from_end >= back_m) else 0
    dx, dy = end - xy[k]
    if math.hypot(dx, dy) < 1e-3:
        return None
    return math.atan2(dy, dx)


def choose_arrival(requested, results, *, slack_fraction=LENGTH_SLACK_FRACTION, slack_m=LENGTH_SLACK_M):
    """(heading, reason, shortest length) from ``results`` = [(candidate heading, path length or None), ...].

    The requested heading wins whenever its path costs no detour; otherwise the
    allowed candidate closest to it. No path at all keeps the requested heading.
    """
    found = [(float(h), float(length)) for h, length in results if length is not None and math.isfinite(length)]
    if not found:
        return float(requested), "NO_PATH_FOR_ANY_CANDIDATE", None
    shortest = min(length for _, length in found)
    allowed = [h for h, length in found if length <= shortest * (1.0 + slack_fraction) + slack_m]
    best = min(allowed, key=lambda h: abs(wrap(h - float(requested))))
    if abs(wrap(best - float(requested))) < 1e-9:
        return float(requested), "REQUESTED_HEADING_NO_DETOUR", shortest
    return best, "SHORTER_PATH_AT_ANOTHER_HEADING", shortest


def turn_options(current_yaw, requested_yaw):
    """Signed in-place turns that reach ``requested_yaw``: the shorter one first, then the other direction."""
    delta = wrap(float(requested_yaw) - float(current_yaw))
    if abs(delta) < 1e-9:
        return [0.0]
    return [delta, delta - math.copysign(2.0 * math.pi, delta)]


def rotation_samples(pose, angle, body, resolution):
    """Poses turning in place from ``pose`` by the signed ``angle``, and the growth that covers the motion between them.

    No body point moves more than half a cell between samples (the outline's
    farthest point travels ``|angle| * circumscribed``).
    """
    x, y, yaw = (float(v) for v in pose)
    reach = abs(float(angle)) * body.circumscribed_m
    count = max(1, int(math.ceil(reach / (float(resolution) * 0.5))))
    grow = reach / count * 0.5
    return [(x, y, yaw + k / count * float(angle)) for k in range(count + 1)], grow


def sweep_check(world, pose, angle, body, margin):
    """Whether the body turning in place by ``angle`` keeps ``margin`` to ``world`` (autonomy.world.SimWorld).

    Returns (clear, smallest gap in metres over the samples, capped at 0.5; 0 when overlapping).
    """
    poses, grow = rotation_samples(pose, angle, body, world.resolution)
    clear, gap = True, 0.5
    for p in poses:
        gap = min(gap, float(world.body_clearance(p, body, 0.5)))
        if world.body_collides(p, body, float(margin) + grow):
            clear = False
    return clear, round(gap, 3)


def grid_world(data, resolution, origin, *, blocked):
    """SimWorld of the cells ``blocked(data)`` marks solid (origin (x, y[, yaw]); outside counts as solid)."""
    from ..autonomy.world import SimWorld
    from ..geometry import Grid
    solid = np.where(blocked(np.asarray(data)), 100, 0).astype(np.int8)
    ox, oy = float(origin[0]), float(origin[1])
    oyaw = float(origin[2]) if len(origin) > 2 else 0.0
    return SimWorld(Grid(solid, float(resolution), (ox, oy, oyaw)))


def saved_map_blocked(data):
    """Saved trinary map: occupied and unknown cells block a turn."""
    return (data >= 65) | (data < 0)


def costmap_blocked(data):
    """Costmap as published (OccupancyGrid): only obstacle cells (lethal) block; inflation and unknown do not."""
    return data >= LETHAL_COST


def decide_turn(checks, current_yaw, requested_yaw, body, *, margin, tolerance):
    """Whether and how to turn in place at the goal.

    ``checks``: [(name, SimWorld, pose in that world's frame), ...], e.g. the saved
    map in the map frame and the local costmap in the odom frame. Returns a dict
    with ``angle`` (signed turn, None for no turn), ``reason`` and per option the
    clearance found in each world.
    """
    delta = wrap(float(requested_yaw) - float(current_yaw))
    out = {"heading_error_rad": round(delta, 4), "tolerance_rad": float(tolerance), "margin_m": float(margin),
           "angle": None, "options": []}
    if abs(delta) <= float(tolerance):
        out["reason"] = "WITHIN_TOLERANCE"
        return out
    if not checks:
        out["reason"] = "NO_OBSTACLE_DATA"
        return out
    for angle in turn_options(current_yaw, requested_yaw):
        option = {"angle_rad": round(angle, 4), "clear": True, "gaps_m": {}}
        for name, world, pose in checks:
            clear, gap = sweep_check(world, pose, angle, body, margin)
            option["gaps_m"][name] = gap
            option["clear"] = option["clear"] and clear
        out["options"].append(option)
        if option["clear"]:
            out["angle"] = angle
            out["reason"] = "SWEEP_CLEAR_SHORTER" if len(out["options"]) == 1 else "SWEEP_CLEAR_OTHER_DIRECTION"
            return out
    out["reason"] = "SWEEP_BLOCKED"
    return out


def mission_config(planner, body, *, margin_m, candidates=CANDIDATES):
    """``goal_heading`` section of the mission config for ``turn_at_goal`` (``yaw_tolerance_rad`` added at bring-up)."""
    aware = planner in HEADING_AWARE_PLANNERS
    return {"mode": "turn_at_goal", "heading_aware": aware, "candidates": int(candidates) if aware else 1,
            "length_slack_fraction": LENGTH_SLACK_FRACTION, "length_slack_m": LENGTH_SLACK_M,
            "arrival_back_m": ARRIVAL_BACK_M, "plan_timeout_s": PLAN_TIMEOUT_S, "planner_id": "GridBased",
            "sweep_margin_m": float(margin_m), "body": body.as_config()}
