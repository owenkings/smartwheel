"""Shadow run along a frozen recording: pure helpers, no ROS imports.

In a shadow run the recorded wheelchair motion moves ``base_link`` and the
recorded scans feed the costmaps and the collision monitor. Nav2 plans toward
the recorded pose a few seconds ahead and commands as if it were driving; every
stage of the velocity chain is recorded and nothing is sent to a base. The
recording does not react to the commands, so the comparison is open loop: it
shows where the stack agrees with the driver, slows, stops or cannot plan on
real sensor data, not what would happen with Nav2 in control.
"""
from __future__ import annotations

import math

import numpy as np

STAGES = ("nav", "smoothed", "monitored", "gated")       # velocity chain order
OUTCOMES = ("DRIVES_FORWARD", "ROTATES_IN_PLACE", "STOPPED_BY_GATE", "STOPPED_BY_COLLISION_MONITOR",
            "SMOOTHER_ZERO", "CONTROLLER_ZERO", "NAV2_NOT_COMMANDING")
MOVING_LINEAR_MPS = 0.05        # the recording counts as moving above either rate
MOVING_ANGULAR_RADPS = 0.15
ACTIVE_LINEAR_MPS = 0.02        # a forward request above this speed
ACTIVE_ANGULAR_RADPS = 0.05     # a turn counted in the turn-direction comparison
ZERO_ANGULAR_RADPS = 1e-3       # below this the controller output is a stop, not a slow turn


def wrap(angle):
    return np.arctan2(np.sin(angle), np.cos(angle))


def base_poses_from_lidar(lidar_poses, lever_xy):
    """Planar axle poses ``T_lidar * inverse(lever)``; the lever is the lidar origin in the axle frame."""
    poses = np.asarray(lidar_poses, dtype=float)
    lx, ly = (float(v) for v in lever_xy)
    c, s = np.cos(poses[:, 2]), np.sin(poses[:, 2])
    return np.column_stack([poses[:, 0] - c * lx + s * ly, poses[:, 1] - s * lx - c * ly, poses[:, 2]])


def _checked(times, poses):
    times = np.asarray(times, dtype=float)
    poses = np.asarray(poses, dtype=float)
    if times.ndim != 1 or len(times) < 2 or poses.shape != (len(times), 3):
        raise ValueError("at least two poses (x, y, yaw) with one time each are required")
    if not (np.isfinite(times).all() and np.isfinite(poses).all()) or np.any(np.diff(times) <= 0):
        raise ValueError("finite poses at strictly increasing times are required")
    return times, poses


def segment_twist(times, poses):
    """Forward speed and yaw rate over each interval between consecutive poses."""
    times, poses = _checked(times, poses)
    turn = wrap(np.diff(poses[:, 2]))
    heading = poses[:-1, 2] + 0.5 * turn
    d = np.diff(poses[:, :2], axis=0)
    dt = np.diff(times)
    return (d[:, 0] * np.cos(heading) + d[:, 1] * np.sin(heading)) / dt, turn / dt


def interpolate_poses(times, poses, query):
    """Linear interpolation along the shortest yaw arc, clamped at both ends; returns (N, 3)."""
    times, poses = _checked(times, poses)
    query = np.atleast_1d(np.asarray(query, dtype=float))
    k = np.clip(np.searchsorted(times, query, side="right") - 1, 0, len(times) - 2)
    a = np.clip((query - times[k]) / (times[k + 1] - times[k]), 0.0, 1.0)
    p, q = poses[k], poses[k + 1]
    xy = p[:, :2] + a[:, None] * (q[:, :2] - p[:, :2])
    return np.column_stack([xy, wrap(p[:, 2] + a * wrap(q[:, 2] - p[:, 2]))])


def twist_at(times, poses, query, half_window_s=0.3):
    """Centred finite-difference forward speed and yaw rate of the recorded motion."""
    times, poses = _checked(times, poses)
    query = np.atleast_1d(np.asarray(query, dtype=float))
    lo = np.clip(query - half_window_s, times[0], times[-1])
    hi = np.clip(query + half_window_s, times[0], times[-1])
    span = np.maximum(hi - lo, 1e-6)
    a, b = interpolate_poses(times, poses, lo), interpolate_poses(times, poses, hi)
    turn = wrap(b[:, 2] - a[:, 2])
    heading = a[:, 2] + 0.5 * turn
    d = b[:, :2] - a[:, :2]
    return (d[:, 0] * np.cos(heading) + d[:, 1] * np.sin(heading)) / span, turn / span


def motion_window(times, poses, lead_s=5.0):
    """From ``lead_s`` before the first recorded motion to the last frame (recording seconds)."""
    times, poses = _checked(times, poses)
    v, w = segment_twist(times, poses)
    moving = (np.abs(v) > MOVING_LINEAR_MPS) | (np.abs(w) > MOVING_ANGULAR_RADPS)
    if not moving.any():
        raise ValueError("the recording never moves")
    first = float(times[int(np.argmax(moving))])
    return max(float(times[0]), first - float(lead_s)), float(times[-1])


def carrot_goal(times, poses, t, lead_s, end_s):
    """Recorded pose ``lead_s`` after recording time ``t``, at most the window end."""
    return tuple(float(v) for v in interpolate_poses(times, poses, min(float(t) + float(lead_s), float(end_s)))[0])


def path_ahead(times, poses, t, lead_s, end_s, spacing_m=0.1, step_s=0.05):
    """Recorded poses from ``t`` to ``min(t + lead_s, end_s)``, at least ``spacing_m`` apart (ends kept)."""
    stop = max(float(t), min(float(t) + float(lead_s), float(end_s)))
    query = np.append(np.arange(float(t), stop, step_s), stop)
    sampled = interpolate_poses(times, poses, query)
    kept = [sampled[0]]
    for pose in sampled[1:-1]:
        if math.hypot(pose[0] - kept[-1][0], pose[1] - kept[-1][1]) >= spacing_m:
            kept.append(pose)
    if len(sampled) > 1:
        kept.append(sampled[-1])
    return np.asarray(kept)


def offset_to_latest_plan(query_t, positions, plans, max_age_s=2.0):
    """Distance from each recorded position to the newest plan made at most ``max_age_s`` earlier (NaN if none)."""
    plans = sorted(((float(p["t"]), np.asarray(p["xy"], dtype=float).reshape(-1, 2)) for p in plans if len(p["xy"])),
                   key=lambda item: item[0])
    starts = [t for t, _ in plans]
    out = np.full(len(query_t), np.nan)
    for k, (t, xy) in enumerate(zip(np.asarray(query_t, dtype=float), np.asarray(positions, dtype=float))):
        j = int(np.searchsorted(starts, t, side="right")) - 1
        if j >= 0 and t - starts[j] <= max_age_s:
            out[k] = float(polyline_distance(xy, plans[j][1])[0])
    return out


def goal_moved(a, b, min_distance_m, min_yaw_rad):
    if a is None or b is None:
        return True
    return (math.hypot(a[0] - b[0], a[1] - b[1]) > min_distance_m
            or abs(math.atan2(math.sin(a[2] - b[2]), math.cos(a[2] - b[2]))) > min_yaw_rad)


# -- geometry of the recorded scans against the collision-monitor zones ------------------------
def scan_points_base(angles, ranges, lever_xy, range_min=0.30, range_max=8.0):
    """Finite returns in the axle frame; the scan frame is the lever translation of base_link."""
    ranges = np.asarray(ranges, dtype=float)
    angles = np.asarray(angles, dtype=float)
    ok = np.isfinite(ranges) & (ranges >= range_min) & (ranges <= range_max)
    r, a = ranges[ok], angles[ok]
    return np.column_stack([float(lever_xy[0]) + r * np.cos(a), float(lever_xy[1]) + r * np.sin(a)])


def inside_polygon(points, flat_polygon):
    """Even-odd point-in-polygon test; ``flat_polygon`` is [x0, y0, x1, y1, ...]."""
    poly = np.asarray(flat_polygon, dtype=float).reshape(-1, 2)
    points = np.asarray(points, dtype=float).reshape(-1, 2)
    x, y = points[:, 0], points[:, 1]
    inside = np.zeros(len(points), dtype=bool)
    for (x1, y1), (x2, y2) in zip(poly, np.roll(poly, -1, axis=0)):
        if y1 == y2:
            continue
        crosses = (y1 > y) != (y2 > y)
        inside ^= crosses & (x < x1 + (y - y1) * (x2 - x1) / (y2 - y1))
    return inside


def zone_audit(angles, ranges_by_frame, lever_xy, stop_radius_m, slow_polygon, max_points, stop_polygon=None):
    """Per frame: nearest return to the axle, points in the stop zone and in the slowdown box.

    The stop zone is the circle ``stop_radius_m`` or, when given, ``stop_polygon``
    (flat x, y list). A zone acts once it holds more than ``max_points`` points
    (Humble collision monitor). This uses each scan at its own capture pose; the
    running monitor also shifts scans to the current pose, so small differences
    are expected.
    """
    nearest, stop, slow = [], [], []
    for ranges in ranges_by_frame:
        points = scan_points_base(angles, ranges, lever_xy)
        distance = np.hypot(points[:, 0], points[:, 1])
        nearest.append(float(distance.min()) if len(distance) else math.inf)
        if stop_polygon is not None:
            stop.append(int(inside_polygon(points, stop_polygon).sum()) if len(points) else 0)
        else:
            stop.append(int((distance <= stop_radius_m).sum()))
        slow.append(int(inside_polygon(points, slow_polygon).sum()))
    stop, slow = np.asarray(stop), np.asarray(slow)
    return {"nearest_m": np.asarray(nearest), "stop_points": stop, "slow_points": slow,
            "stop_expected": stop > max_points, "slow_expected": slow > max_points}


def near_points(angles, ranges_by_frame, lever_xy, radius_m, polygon=None):
    """Returns within ``radius_m`` of the axle (or inside ``polygon``) as rows (frame, x, y) in base_link."""
    rows = []
    for k, ranges in enumerate(ranges_by_frame):
        points = scan_points_base(angles, ranges, lever_xy)
        if polygon is not None:
            close = points[inside_polygon(points, polygon)] if len(points) else points
        else:
            close = points[np.hypot(points[:, 0], points[:, 1]) <= radius_m]
        rows.extend((k, float(x), float(y)) for x, y in close)
    return rows


# -- static map: what the global planner can use ---------------------------------------------
class StaticPlanningMap:
    """Static-layer approximation of the global costmap, for attributing planner failures.

    Occupied cells, and unknown cells when ``inflate_unknown`` (the navigation
    setting ``inflate_around_unknown``), are inflation sources. A cell is
    plannable when it is not unknown and lies farther than the lethal radius
    from every source, because NavFn treats inscribed and lethal costs as
    obstacles. Live obstacle-layer marking and clearing are not modelled.
    """

    def __init__(self, data, resolution, origin, lethal_radius_m, *, inflate_unknown=True):
        from scipy import ndimage
        self.data = np.asarray(data)
        self.resolution = float(resolution)
        self.origin = (float(origin[0]), float(origin[1]))
        self.lethal_radius_m = float(lethal_radius_m)
        occupied, unknown = self.data == 100, self.data == -1
        far = np.full(self.data.shape, np.inf)
        self.to_occupied = ndimage.distance_transform_edt(~occupied) * self.resolution if occupied.any() else far
        self.to_unknown = ndimage.distance_transform_edt(~unknown) * self.resolution if unknown.any() else far
        distance = np.minimum(self.to_occupied, self.to_unknown) if inflate_unknown else self.to_occupied
        self.plannable = (distance > self.lethal_radius_m) & ~unknown
        self.labels, _ = ndimage.label(self.plannable)          # 4-connected, as NavFn propagates

    @classmethod
    def from_blocked(cls, blocked, resolution, origin):
        """Reachability on a recorded costmap; ``blocked`` marks inscribed, lethal and unknown cost."""
        from scipy import ndimage
        self = cls.__new__(cls)
        blocked = np.asarray(blocked, dtype=bool)
        self.data = np.where(blocked, 100, 0).astype(np.int8)
        self.resolution = float(resolution)
        self.origin = (float(origin[0]), float(origin[1]))
        self.lethal_radius_m = 0.0
        self.to_occupied = self.to_unknown = np.full(blocked.shape, np.inf)
        self.plannable = ~blocked
        self.labels, _ = ndimage.label(self.plannable)
        return self

    def cell(self, x, y):
        col = int(math.floor((x - self.origin[0]) / self.resolution))
        row = int(math.floor((y - self.origin[1]) / self.resolution))
        if 0 <= row < self.data.shape[0] and 0 <= col < self.data.shape[1]:
            return row, col
        return None

    def clearance(self, x, y):
        """Distances (m) from the cell at (x, y) to the nearest occupied and unknown cell."""
        rc = self.cell(x, y)
        return (0.0, 0.0) if rc is None else (float(self.to_occupied[rc]), float(self.to_unknown[rc]))

    def check(self, start_xy, goal_xy, tolerance_m=0.25):
        """NavFn-like reachability: the robot cell is cleared, propagation needs a
        plannable 4-neighbour, the goal may move within a square tolerance window."""
        s, g = self.cell(*start_xy), self.cell(*goal_xy)
        if s is None or g is None:
            return "OUTSIDE_MAP"
        r, c = s
        neighbours = [self.labels[i, j] for i, j in ((r, c), (r - 1, c), (r + 1, c), (r, c - 1), (r, c + 1))
                      if 0 <= i < self.labels.shape[0] and 0 <= j < self.labels.shape[1]]
        start = {int(v) for v in neighbours if v > 0}
        if not start:
            return "START_ENCLOSED"
        k = int(round(tolerance_m / self.resolution))
        window = self.labels[max(0, g[0] - k):g[0] + k + 1, max(0, g[1] - k):g[1] + k + 1]
        return "REACHABLE" if np.isin(window, list(start)).any() else "GOAL_NOT_CONNECTED"


def attribute_planner_failure(conservative, occupied_only, start_xy, goal_xy, tolerance_m=0.25):
    """Name the static-map reason a plan from ``start_xy`` to ``goal_xy`` fails, if there is one.

    ``conservative`` inflates unknown cells (the navigation setting);
    ``occupied_only`` keeps unknown cells untraversable but does not inflate them.
    """
    verdict = conservative.check(start_xy, goal_xy, tolerance_m)
    if verdict == "REACHABLE":
        return "NOT_EXPLAINED_BY_STATIC_MAP"
    if verdict == "OUTSIDE_MAP":
        return "OUTSIDE_MAP"
    if verdict == "START_ENCLOSED":
        to_occupied, _ = conservative.clearance(*start_xy)
        if to_occupied <= conservative.lethal_radius_m:
            return "START_WITHIN_LETHAL_RADIUS_OF_OCCUPIED"
        return "START_WITHIN_LETHAL_RADIUS_OF_UNKNOWN"
    if occupied_only.check(start_xy, goal_xy, tolerance_m) == "REACHABLE":
        return "INFLATED_UNKNOWN_CUTS_THE_ROUTE"
    return "OCCUPIED_CELLS_CUT_THE_ROUTE_OR_GOAL"


def path_map_audit(planning_map, path_xy, pinhole_max_cells=20):
    """Static clearance along the recorded path and unknown pinholes the planner must avoid."""
    from scipy import ndimage
    path_xy = np.asarray(path_xy, dtype=float).reshape(-1, 2)
    clear = np.array([planning_map.clearance(x, y) for x, y in path_xy]).reshape(-1, 2)
    lethal, res = planning_map.lethal_radius_m, planning_map.resolution
    on_path = np.zeros(planning_map.data.shape, dtype=bool)
    for x, y in path_xy:
        rc = planning_map.cell(x, y)
        if rc is not None:
            on_path[rc] = True
    near = ndimage.distance_transform_edt(~on_path) * res <= lethal if on_path.any() else on_path
    labels, count = ndimage.label(planning_map.data == -1, structure=np.ones((3, 3), dtype=bool))
    sizes = np.bincount(labels.ravel(), minlength=count + 1)
    touched = np.unique(labels[near & (labels > 0)])
    pinholes = touched[sizes[touched] <= pinhole_max_cells]
    def low(values):   # +inf when the map has no cell of that kind
        values = values[np.isfinite(values)]
        if not len(values):
            return None
        return {"min": float(values.min()), "p10": float(np.percentile(values, 10)), "median": float(np.median(values))}
    return {"path_samples": int(len(path_xy)),
            "clearance_to_occupied_m": low(clear[:, 0]), "clearance_to_unknown_m": low(clear[:, 1]),
            "fraction_within_lethal_of_occupied": float(np.mean(clear[:, 0] <= lethal)) if len(clear) else None,
            "fraction_within_lethal_of_unknown": float(np.mean(clear[:, 1] <= lethal)) if len(clear) else None,
            "unknown_pinholes_within_lethal_of_path": int(len(pinholes)),
            "pinhole_max_cells": int(pinhole_max_cells), "lethal_radius_m": lethal}


# -- command comparison ------------------------------------------------------------------------
def resample_latest(sample_times, values, query, max_age_s):
    """Latest sample at or before each query time; NaN where none is younger than ``max_age_s``."""
    sample_times = np.asarray(sample_times, dtype=float)
    query = np.asarray(query, dtype=float)
    values = np.asarray(values, dtype=float).reshape(len(sample_times), -1) if len(sample_times) else np.zeros((0, 2))
    out = np.full((len(query), values.shape[1]), np.nan)
    if not len(sample_times):
        return out
    order = np.argsort(sample_times, kind="stable")
    sample_times, values = sample_times[order], values[order]
    k = np.searchsorted(sample_times, query, side="right") - 1
    ok = k >= 0
    ok[ok] = query[ok] - sample_times[k[ok]] <= max_age_s
    out[ok] = values[k[ok]]
    return out


def _passes(command, forward):
    """Whether a stage still carries the request: forward speed, or else any turn."""
    command = np.asarray(command, dtype=float)
    ok = np.isfinite(command).all(axis=1)
    return np.where(forward, ok & (command[:, 0] > ACTIVE_LINEAR_MPS), ok & (np.abs(command[:, 1]) > ZERO_ANGULAR_RADPS))


def attribute(stages):
    """Where the velocity chain ends at each sample; ``stages`` maps STAGES to (N, 2) arrays.

    The controller asks either for forward motion or, with zero forward speed,
    for a turn in place (Regulated Pure Pursuit's rotate-to-heading, limited in
    acceleration around the current yaw rate, so its output can be small). An
    exactly zero output is a stop (collision ahead, goal reached or canceled).
    Each request is followed down the chain to the first stage that zeroes it.
    """
    nav, smoothed, monitored, gated = (np.asarray(stages[s], dtype=float) for s in STAGES)
    finite = np.isfinite(nav).all(axis=1)
    forward = finite & (nav[:, 0] > ACTIVE_LINEAR_MPS)
    turning = finite & ~forward & (np.abs(nav[:, 1]) > ZERO_ANGULAR_RADPS)
    requested = forward | turning
    s_ok, m_ok, g_ok = (_passes(c, forward) for c in (smoothed, monitored, gated))
    outcome = np.full(len(nav), "NAV2_NOT_COMMANDING", dtype=object)
    outcome[finite] = "CONTROLLER_ZERO"
    outcome[requested] = "SMOOTHER_ZERO"
    outcome[requested & s_ok] = "STOPPED_BY_COLLISION_MONITOR"
    outcome[requested & s_ok & m_ok] = "STOPPED_BY_GATE"
    done = requested & s_ok & m_ok & g_ok
    outcome[done & forward] = "DRIVES_FORWARD"
    outcome[done & turning] = "ROTATES_IN_PLACE"
    slowed = forward & s_ok & m_ok & (np.nan_to_num(monitored[:, 0]) < 0.8 * smoothed[:, 0])
    return outcome, slowed


def _fractions(labels, weights_mask):
    total = int(weights_mask.sum())
    if not total:
        return {name: 0.0 for name in OUTCOMES}
    return {name: float(np.sum((labels == name) & weights_mask) / total) for name in OUTCOMES}


def _stats(values):
    values = np.asarray([v for v in np.ravel(values) if math.isfinite(v)], dtype=float)
    if not len(values):
        return None
    return {"count": int(len(values)), "median": float(np.median(values)), "p90": float(np.percentile(values, 90)),
            "max": float(values.max())}


def _intervals(times, mask, min_duration_s=0.0):
    """Contiguous [start, end] intervals where ``mask`` holds."""
    result, start = [], None
    for t, m in zip(times, mask):
        if m and start is None:
            start = t
        elif not m and start is not None:
            result.append([float(start), float(t)])
            start = None
    if start is not None:
        result.append([float(start), float(times[-1])])
    return [i for i in result if i[1] - i[0] >= min_duration_s]


def polyline_distance(points, polyline):
    """Distance from each point to a polyline with at least one vertex."""
    points = np.asarray(points, dtype=float).reshape(-1, 2)
    line = np.asarray(polyline, dtype=float).reshape(-1, 2)
    if len(line) == 1:
        return np.hypot(points[:, 0] - line[0, 0], points[:, 1] - line[0, 1])
    a, b = line[:-1][None], line[1:][None]
    ab = b - a
    p = points[:, None, :]
    t = np.clip(((p - a) * ab).sum(-1) / np.maximum((ab * ab).sum(-1), 1e-12), 0.0, 1.0)
    return np.linalg.norm(p - (a + t[..., None] * ab), axis=-1).min(axis=1)


def plan_agreement(times, poses, plans, horizon_s, step_s=0.2, route_change_m=1.0):
    """Deviation of the recorded path over the next ``horizon_s`` from each plan made at that time.

    ``plans``: dicts with ``t`` (recording seconds) and ``xy`` (map frame). The
    planner keeps the margin and centres in free space, so a few tenths of a
    metre are expected; more than ``route_change_m`` means another route.
    """
    per_plan = []
    for plan in plans:
        xy = np.asarray(plan["xy"], dtype=float).reshape(-1, 2)
        t0 = float(plan["t"])
        if not len(xy) or t0 < times[0] or t0 + step_s > times[-1]:
            continue
        query = np.arange(t0, min(t0 + horizon_s, float(times[-1])) + 1e-9, step_s)
        future = interpolate_poses(times, poses, query)[:, :2]
        d = polyline_distance(future, xy)
        length = float(np.sum(np.hypot(*np.diff(xy, axis=0).T))) if len(xy) > 1 else 0.0
        per_plan.append({"t": t0, "max_deviation_m": float(d.max()), "mean_deviation_m": float(d.mean()),
                         "plan_length_m": length})
    maxima = [p["max_deviation_m"] for p in per_plan]
    return {"plans_compared": len(per_plan), "horizon_s": float(horizon_s),
            "max_deviation_m": _stats(maxima), "mean_deviation_m": _stats([p["mean_deviation_m"] for p in per_plan]),
            "route_change_threshold_m": float(route_change_m),
            "route_changes": [{"t": p["t"], "max_deviation_m": p["max_deviation_m"]}
                              for p in per_plan if p["max_deviation_m"] > route_change_m]}, per_plan


def evaluate(times, poses, window, samples, *, step_s=0.1, max_age_s=0.5):
    """Compare every velocity-chain stage with the recorded motion on a regular time grid.

    ``samples`` maps each of STAGES to ``(t, values)`` with ``t`` in recording
    seconds and ``values`` as (N, 2) linear/angular rows. Returns the table and
    summary metrics; samples while the recording stands still are counted
    separately, because the carrot then may sit behind or beside the chair.
    """
    times, poses = _checked(times, poses)
    grid = np.arange(float(window[0]), float(window[1]) + 1e-9, step_s)
    v_rec, w_rec = twist_at(times, poses, grid)
    stages = {name: resample_latest(*samples[name], grid, max_age_s) if len(samples[name][0])
              else np.full((len(grid), 2), np.nan) for name in STAGES}
    outcome, slowed = attribute(stages)
    moving = (np.abs(v_rec) > MOVING_LINEAR_MPS) | (np.abs(w_rec) > MOVING_ANGULAR_RADPS)
    nav = stages["nav"]
    turning = moving & (np.abs(w_rec) > MOVING_ANGULAR_RADPS) & np.isfinite(nav[:, 1]) & (np.abs(nav[:, 1]) > ACTIVE_ANGULAR_RADPS)
    agree = np.sign(nav[:, 1]) == np.sign(w_rec)
    valid = moving & np.isfinite(nav).all(axis=1)
    correlation = None
    if valid.sum() > 10 and np.std(nav[valid, 1]) > 1e-6 and np.std(w_rec[valid]) > 1e-6:
        correlation = float(np.corrcoef(nav[valid, 1], w_rec[valid])[0, 1])
    gated = stages["gated"]
    commanded = moving & (outcome == "DRIVES_FORWARD")
    forward_driving = v_rec > MOVING_LINEAR_MPS
    summary = {
        "grid_step_s": step_s, "command_max_age_s": max_age_s, "samples": int(len(grid)),
        "moving_samples": int(moving.sum()), "stationary_samples": int((~moving).sum()),
        "outcome_fractions_while_recording_moves": _fractions(outcome, moving),
        "outcome_fractions_while_recording_stands": _fractions(outcome, ~moving),
        "drives_forward_fraction_while_recording_drives_forward": (
            float(np.sum(forward_driving & (outcome == "DRIVES_FORWARD")) / forward_driving.sum())
            if forward_driving.sum() else None),
        "slowed_by_collision_monitor_fraction_while_moving": float(np.sum(slowed & moving) / max(1, moving.sum())),
        "turn_direction_agreement": {
            "samples": int(turning.sum()),
            "fraction": float(np.sum(agree & turning) / turning.sum()) if turning.sum() else None,
            "recorded_turn_threshold_radps": MOVING_ANGULAR_RADPS},
        "yaw_rate_correlation_while_moving": correlation,
        "recorded_linear_mps_while_moving": _stats(v_rec[moving]),
        "gated_linear_mps_while_driving_forward": _stats(gated[commanded, 0]),
        "stop_intervals_collision_monitor": _intervals(grid, moving & (outcome == "STOPPED_BY_COLLISION_MONITOR"), 0.3),
        "stop_intervals_not_commanding": _intervals(grid, moving & (outcome == "NAV2_NOT_COMMANDING"), 0.5),
        "stop_intervals_controller_zero": _intervals(grid, moving & (outcome == "CONTROLLER_ZERO"), 0.3),
        "stop_intervals_gate": _intervals(grid, moving & (outcome == "STOPPED_BY_GATE"), 0.3),
        # Open loop: the recording keeps its own heading, so a turn in place while the driver
        # goes on means the path direction at the lookahead point differs by more than 0.5 rad.
        "rotate_in_place_intervals_while_moving": _intervals(grid, moving & (outcome == "ROTATES_IN_PLACE"), 0.3),
    }
    table = {"t": grid, "v_rec": v_rec, "w_rec": w_rec, "moving": moving, "outcome": outcome, "slowed": slowed,
             **{name: stages[name] for name in STAGES}}
    return table, summary
