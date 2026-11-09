"""How the chair reacts to people: pure computation on obstacle tracks, no ROS (window B).

Window C finds the moving obstacles and publishes ``/obstacles/tracks`` (``wc_obstacle_tracks/1``, see
``perception/tracks.py``: position, velocity and a class per track, odom frame). This module decides what the
chair does about them. The node that wires it into Nav2 publishes only a speed limit (nav2_msgs/SpeedLimit on
the controller's speed limit topic) and keepout masks for both costmaps (Nav2 KeepoutFilter); it never sends
velocities. The collision monitor (window C) stays the last stop and the gate (window D) is unchanged.

People are tracks of class ``moving`` or ``person_candidate``, and tracks that have moved before
(``has_moved``) whatever their class is now. Other unmapped obstacles (``static_unknown``, e.g. a person
standing against a wall) stay ordinary obstacles without the person clearance.

Prediction: moving people at constant velocity over ``horizon_s``, their disc growing with the velocity
uncertainty (``growth_sigma`` x speed_std per second, at least ``min_growth_mps``); standing people keep their
disc. The chair moves along its current arc (linear and angular speed). Gaps are measured from the body
outline to the person's disc.

Speed limit, decided by the person that is worst off:

- yield (stop): within ``yield_horizon_s`` the predicted gap falls below the person's clearance
  (``moving_clearance_m`` 0.5 m, ``standing_clearance_m`` 0.3 m). The limit is ``stop_limit_mps`` (Nav2 reads
  a speed limit of 0 as "no limit", so a stop is the smallest positive limit);
- slow: the current or predicted gap within ``horizon_s`` is below ``slow_near_m`` (1 m): ``slow_near_mps``
  (0.12 m/s); below ``slow_far_m`` (2 m): ``slow_far_mps`` (0.2 m/s);
- otherwise no limit (None).

A yield ends after the condition has been clear for ``release_s``. A yield that lasts ``max_wait_s`` (20 s) is
not held further: the people it waits for are treated as standing (the smaller clearance, no prediction), so
the planner can go around them where there is room and the goal fails where there is not. They are treated
normally again once they are farther than ``slow_far_m``. The chair never reverses or spins to get out of the
way: with an oncoming person who does not stop either, the chair stands still before the gap is below the
clearance, and what happens next is up to the person.

Masks (``keepout_mask``): per costmap an OccupancyGrid of the costmap's own size (the KeepoutFilter refreshes
its whole window only for a mask as large as the costmap, and leaves stale cells otherwise). Lethal (100)
within the clearance of each person's current and predicted discs (``mask_horizon_s``). Around that zone the
costmap's inflation curve is repeated out to the circumscribed radius, because the inflation layer does not
inflate filter cells and the Smac planners skip the footprint check where the centre cell costs less than
the circumscribed cost: within the inscribed radius 99 (or 100 for a heading-free planner that only checks the
centre point), then the decaying inflation cost. Cells away from people are -1, which the filter skips
(0 would turn unknown cells free).
"""
from __future__ import annotations

from dataclasses import dataclass, replace
import math

import numpy as np

PERSON_CLASSES = ("moving", "person_candidate")
NAV2_LETHAL, NAV2_INSCRIBED = 254, 253


@dataclass(frozen=True)
class PeopleConfig:
    horizon_s: float = 3.0
    step_s: float = 0.25
    yield_horizon_s: float = 1.5
    moving_clearance_m: float = 0.5
    standing_clearance_m: float = 0.3
    slow_far_m: float = 2.0
    slow_far_mps: float = 0.20
    slow_near_m: float = 1.0
    slow_near_mps: float = 0.12
    stop_limit_mps: float = 0.001
    min_growth_mps: float = 0.1
    growth_sigma: float = 2.0
    max_wait_s: float = 20.0
    release_s: float = 1.0
    mask_horizon_s: float = 1.5
    stale_after_s: float = 1.0

    def __post_init__(self):
        for name in self.__dataclass_fields__:
            value = float(getattr(self, name))
            if not math.isfinite(value) or value < 0.0:
                raise ValueError("people config %s must be a non-negative finite number" % name)
        if not (0.0 < self.step_s <= self.yield_horizon_s <= self.horizon_s and self.mask_horizon_s <= self.horizon_s):
            raise ValueError("people config needs 0 < step <= yield horizon <= horizon and mask horizon <= horizon")
        if not (self.slow_near_m <= self.slow_far_m and 0.0 < self.stop_limit_mps < self.slow_near_mps <= self.slow_far_mps):
            raise ValueError("people config needs slow_near <= slow_far and 0 < stop limit < slow speeds")
        if self.standing_clearance_m > self.moving_clearance_m:
            raise ValueError("standing clearance must not exceed the moving clearance")


@dataclass(frozen=True)
class Person:
    id: int
    x: float
    y: float
    vx: float
    vy: float
    radius_m: float
    moving: bool
    growth_mps: float = 0.0

    def at(self, t):
        """(x, y, radius) ``t`` seconds ahead."""
        if not self.moving:
            return self.x, self.y, self.radius_m
        return self.x + self.vx * t, self.y + self.vy * t, self.radius_m + self.growth_mps * t

    def as_standing(self):
        return replace(self, vx=0.0, vy=0.0, moving=False, growth_mps=0.0)

    def clearance(self, cfg):
        return cfg.moving_clearance_m if self.moving else cfg.standing_clearance_m


def people_from_message(message, now, cfg=PeopleConfig()):
    """(people, age_s) of a checked ``wc_obstacle_tracks/1`` message, extrapolated from its stamp to ``now``.

    A message older than ``stale_after_s`` gives no people (the age tells the caller why).
    """
    age = float(now) - float(message["stamp"])
    if age > cfg.stale_after_s:
        return [], age
    dt = max(0.0, age)
    people = []
    for e in message["tracks"]:
        if e["class"] not in PERSON_CLASSES and not e["has_moved"]:
            continue
        moving = e["class"] == "moving"
        vx, vy = (float(e["vx"]), float(e["vy"])) if moving else (0.0, 0.0)
        growth = max(cfg.min_growth_mps, cfg.growth_sigma * float(e["speed_std"])) if moving else 0.0
        p = Person(int(e["id"]), float(e["x"]), float(e["y"]), vx, vy, float(e["radius_m"]), moving, growth)
        x, y, r = p.at(dt)
        people.append(replace(p, x=x, y=y, radius_m=r))
    return people, age


def transform_people(people, pose):
    """People in another frame: ``pose`` (x, y, yaw) is the source frame in the target frame (e.g. odom in map)."""
    tx, ty, yaw = (float(v) for v in pose)
    c, s = math.cos(yaw), math.sin(yaw)
    return [replace(p, x=tx + c * p.x - s * p.y, y=ty + s * p.x + c * p.y,
                    vx=c * p.vx - s * p.vy, vy=s * p.vx + c * p.vy) for p in people]


def chair_at(pose, linear, angular, t):
    """Chair pose ``t`` seconds ahead on its current arc."""
    x, y, yaw = (float(v) for v in pose)
    v, w = float(linear), float(angular)
    if abs(w) < 1e-6:
        return x + v * t * math.cos(yaw), y + v * t * math.sin(yaw), yaw
    r = v / w
    return (x + r * (math.sin(yaw + w * t) - math.sin(yaw)), y - r * (math.cos(yaw + w * t) - math.cos(yaw)),
            yaw + w * t)


def body_gap(body, pose, x, y, radius):
    """Gap from the body (planning.body.Body) at ``pose`` to a disc; negative when they overlap."""
    if not body.is_polygon:
        return math.hypot(x - pose[0], y - pose[1]) - body.radius_m - radius
    from .body import point_polygon_distance
    return float(point_polygon_distance((x, y), body.outline_at(pose))[0]) - radius


def assess(people, chair_pose, chair_velocity, body, cfg=PeopleConfig()):
    """Per person: clearance, current gap, smallest predicted gap within the yield horizon and the horizon."""
    times = np.arange(0.0, cfg.horizon_s + 1e-9, cfg.step_s)
    poses = [chair_at(chair_pose, chair_velocity[0], chair_velocity[1], t) for t in times]
    out = []
    for p in people:
        gaps = np.array([body_gap(body, q, *p.at(t)) for q, t in zip(poses, times)])
        within = times <= cfg.yield_horizon_s + 1e-9
        k = int(np.argmin(gaps))
        out.append({"id": p.id, "moving": p.moving, "clearance_m": p.clearance(cfg), "gap_now_m": float(gaps[0]),
                    "gap_yield_m": float(gaps[within].min()), "gap_horizon_m": float(gaps.min()),
                    "t_closest_s": float(times[k])})
    return out


def speed_decision(assessments, cfg=PeopleConfig()):
    """(speed limit in m/s or None, state, deciding assessment) for the current assessments."""
    yielding = [a for a in assessments if a["gap_yield_m"] < a["clearance_m"]]
    if yielding:
        return cfg.stop_limit_mps, "YIELD", min(yielding, key=lambda a: a["gap_yield_m"] - a["clearance_m"])
    if not assessments:
        return None, "CLEAR", None
    nearest = min(assessments, key=lambda a: min(a["gap_now_m"], a["gap_horizon_m"]))
    gap = min(nearest["gap_now_m"], nearest["gap_horizon_m"])
    if gap < cfg.slow_near_m:
        return cfg.slow_near_mps, "SLOW_NEAR", nearest
    if gap < cfg.slow_far_m:
        return cfg.slow_far_mps, "SLOW_FAR", nearest
    return None, "CLEAR", nearest


class Reaction:
    """Speed limit over time: yield with release hold and a maximum wait (see the module docstring)."""

    def __init__(self, cfg=PeopleConfig()):
        self.cfg = cfg
        self.yield_since = None
        self.clear_since = None
        self.demoted = set()

    def update(self, now, people, chair_pose, chair_velocity, body):
        cfg = self.cfg
        now = float(now)
        current = [p.as_standing() if p.id in self.demoted else p for p in people]
        assessments = assess(current, chair_pose, chair_velocity, body, cfg)
        far = {a["id"] for a in assessments if min(a["gap_now_m"], a["gap_horizon_m"]) > cfg.slow_far_m}
        self.demoted -= far | (self.demoted - {p.id for p in people})
        limit, state, who = speed_decision(assessments, cfg)
        if state == "YIELD":
            self.clear_since = None
            if self.yield_since is None:
                self.yield_since = now
            elif now - self.yield_since >= cfg.max_wait_s:
                waited = {a["id"] for a in assessments if a["gap_yield_m"] < a["clearance_m"] and a["moving"]}
                if waited:
                    self.demoted |= waited
                    self.yield_since = None
                    return self.update(now, people, chair_pose, chair_velocity, body)
        elif self.yield_since is not None:
            if self.clear_since is None:
                self.clear_since = now
            if now - self.clear_since < cfg.release_s:
                limit, state = cfg.stop_limit_mps, "YIELD_RELEASING"
            else:
                self.yield_since = self.clear_since = None
        return {"limit_mps": limit, "state": state, "deciding": who,
                "yield_s": None if self.yield_since is None else round(now - self.yield_since, 2),
                "treated_as_standing": sorted(self.demoted), "people": len(people)}


def inflation_cost(distance_m, inscribed_m, cost_scaling_factor):
    """Nav2 InflationLayer cost at ``distance_m`` from an obstacle (253 within the inscribed radius)."""
    if distance_m <= 0.0:
        return float(NAV2_LETHAL)
    if distance_m <= inscribed_m:
        return float(NAV2_INSCRIBED)
    return (NAV2_INSCRIBED - 1) * math.exp(-float(cost_scaling_factor) * (distance_m - inscribed_m))


def keepout_mask(shape, resolution, origin, people, cfg=PeopleConfig(), *, inscribed_m, circumscribed_m,
                 cost_scaling_factor, center_point=False):
    """int8 KeepoutFilter mask (rows y, columns x) of ``shape`` cells at ``resolution`` from ``origin`` (x, y).

    ``inscribed_m``/``circumscribed_m``/``cost_scaling_factor``: the costmap's own footprint radii (padding
    included) and inflation factor. ``center_point``: the costmap's planner checks only the centre cell
    (NavFn, Smac 2D, Theta*), so the inscribed ring is lethal too.
    """
    height, width = (int(v) for v in shape)
    res = float(resolution)
    ox, oy = float(origin[0]), float(origin[1])
    distance = np.full((height, width), np.inf)
    reach = float(circumscribed_m) + res
    mask_times = np.arange(0.0, cfg.mask_horizon_s + 1e-9, cfg.step_s)
    for p in people:
        for t in (mask_times if p.moving else (0.0,)):
            cx, cy, r = p.at(t)
            zone = r + p.clearance(cfg)
            c0 = max(0, int(math.floor((cx - zone - reach - ox) / res)))
            c1 = min(width, int(math.ceil((cx + zone + reach - ox) / res)) + 1)
            r0 = max(0, int(math.floor((cy - zone - reach - oy) / res)))
            r1 = min(height, int(math.ceil((cy + zone + reach - oy) / res)) + 1)
            if c0 >= c1 or r0 >= r1:
                continue
            xs = ox + (np.arange(c0, c1) + 0.5) * res
            ys = oy + (np.arange(r0, r1) + 0.5) * res
            d = np.hypot(xs[None, :] - cx, ys[:, None] - cy) - zone
            distance[r0:r1, c0:c1] = np.minimum(distance[r0:r1, c0:c1], d)
    data = np.full((height, width), -1, dtype=np.int16)
    inner = (distance > 0.0) & (distance <= float(inscribed_m))
    outer = (distance > float(inscribed_m)) & (distance <= reach)
    cost = (NAV2_INSCRIBED - 1) * np.exp(-float(cost_scaling_factor) * (distance[outer] - float(inscribed_m)))
    # OccupancyGrid value v becomes cost round(v * 254 / 100): one step above keeps the cost at or above Nav2's.
    data[outer] = np.clip(np.ceil(cost * 100.0 / NAV2_LETHAL) + 1, 1, 98)
    data[inner] = 100 if center_point else 99
    data[distance <= 0.0] = 100
    return data.astype(np.int8)
