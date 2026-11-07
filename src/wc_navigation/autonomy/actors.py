"""Moving pedestrians ("actors") for the closed-loop simulator. Simulation only.

An actor is a disc (default radius 0.25 m) that walks through waypoints in world
coordinates at a constant speed. Its clock starts when the simulated base gets its
first non-zero motion command (Nav2 bring-up takes a varying amount of simulated
time) plus the actor's ``start_after_s``; before that it stands at its first
waypoint. After the last waypoint it stops (``repeat`` ``once``), starts again at
the first (``loop``) or walks back (``pingpong``); ``pause_s`` is spent at each
waypoint it reaches. Actors do not see walls, so scenarios keep waypoints in free
space.

``yield_gap_m`` 0 (default) is the worst case: the actor walks on regardless of
the chair and can walk into it. With ``yield_gap_m`` > 0 it does not take a step
that brings it closer than that to the chair's body (a pedestrian who stops for
the chair).

Scenario file (``--actors FILE``)::

    {"schema_version": 1, "name": "oncoming",
     "actors": [{"id": "walker", "waypoints": [[-10.5, -0.5], [0.5, -0.5]], "speed_mps": 1.0,
                 "radius_m": 0.25, "start_after_s": 2.0, "pause_s": 0.0, "repeat": "once",
                 "yield_gap_m": 0.0}]}

In the simulator (``SimActors``) the actors' current discs are added to the
world's static discs before every physics step, so the simulated scan, the swept
collision veto and the contact gaps include them. A line goes to ``actors.jsonl``
in the run directory every 0.1 s of simulated time, plus event lines for the first
command and for contacts. ``result_entries`` turns that log into the ``actors``
section of result.json. A chair that drives into an actor is a collision of the
run as before; an actor walking into the standing chair is reported in the
``actors`` section only.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass
import hashlib
import json
import math
from pathlib import Path
import re

import numpy as np

SCHEMA_VERSION = 1
LOG_NAME = "actors.jsonl"
REPEATS = ("once", "loop", "pingpong")
LOG_PERIOD_S = 0.1
CONTACT_GAP_M = 0.005          # gap at or below which an actor touches the body
NEAR_M = (0.3, 0.5, 1.0, 1.5)
_ID = re.compile(r"^[A-Za-z0-9_-]{1,40}$")


def _finite(value, name, low=None, high=None):
    number = float(value)
    if not math.isfinite(number) or (low is not None and number < low) or (high is not None and number > high):
        raise ValueError("actor %s must be a finite number within %s..%s" % (name, low, high))
    return number


@dataclass(frozen=True)
class ActorSpec:
    id: str
    waypoints: tuple
    speed_mps: float = 1.0
    radius_m: float = 0.25
    start_after_s: float = 0.0
    pause_s: float = 0.0
    repeat: str = "once"
    yield_gap_m: float = 0.0

    def __post_init__(self):
        if not isinstance(self.id, str) or not _ID.match(self.id):
            raise ValueError("actor id must be 1-40 letters, digits, '_' or '-'")
        points = tuple((_finite(p[0], "waypoint x", -1000, 1000), _finite(p[1], "waypoint y", -1000, 1000))
                       for p in self.waypoints)
        if not 1 <= len(points) <= 50 or any(len(p) != 2 for p in self.waypoints):
            raise ValueError("actor %s needs 1-50 waypoints [x, y]" % self.id)
        object.__setattr__(self, "waypoints", points)
        object.__setattr__(self, "speed_mps", _finite(self.speed_mps, "speed_mps", 0.0, 2.5))
        object.__setattr__(self, "radius_m", _finite(self.radius_m, "radius_m", 0.1, 0.6))
        object.__setattr__(self, "start_after_s", _finite(self.start_after_s, "start_after_s", 0.0, 3600.0))
        object.__setattr__(self, "pause_s", _finite(self.pause_s, "pause_s", 0.0, 600.0))
        object.__setattr__(self, "yield_gap_m", _finite(self.yield_gap_m, "yield_gap_m", 0.0, 3.0))
        if self.repeat not in REPEATS:
            raise ValueError("actor repeat must be one of " + ", ".join(REPEATS))
        if len(points) > 1 and self.speed_mps <= 0:
            raise ValueError("actor %s has waypoints to walk but no speed" % self.id)

    def as_dict(self):
        value = asdict(self)
        value["waypoints"] = [list(p) for p in self.waypoints]
        return value


def parse_actors(items):
    """Validated ActorSpec list from scenario entries; ids must be unique."""
    if not isinstance(items, (list, tuple)) or not 1 <= len(items) <= 20:
        raise ValueError("a scenario needs 1-20 actors")
    specs = []
    for item in items:
        if not isinstance(item, dict):
            raise ValueError("each actor must be an object")
        unknown = set(item) - set(ActorSpec.__dataclass_fields__)
        if unknown:
            raise ValueError("unknown actor keys: " + ", ".join(sorted(unknown)))
        specs.append(ActorSpec(**{k: (tuple(map(tuple, v)) if k == "waypoints" else v) for k, v in item.items()}))
    if len({s.id for s in specs}) != len(specs):
        raise ValueError("actor ids must be unique")
    return specs


def load_scenario(path):
    """(normalized actor dicts, provenance) of a scenario file."""
    path = Path(path).expanduser().resolve(strict=True)
    raw = path.read_bytes()
    value = json.loads(raw.decode("utf-8"))
    if not isinstance(value, dict) or value.get("schema_version") != SCHEMA_VERSION:
        raise ValueError("actor scenario must be an object with schema_version %d" % SCHEMA_VERSION)
    specs = parse_actors(value.get("actors"))
    provenance = {"path": str(path), "sha256": hashlib.sha256(raw).hexdigest(), "name": str(value.get("name", "")),
                  "actors": len(specs)}
    return [s.as_dict() for s in specs], provenance


def body_gap(pose, body, x, y, radius):
    """Gap (m) from the body (planning.body.Body) at ``pose`` to a disc; negative when they overlap."""
    if not body.is_polygon:
        return math.hypot(x - pose[0], y - pose[1]) - body.radius_m - radius
    from ..planning.body import point_polygon_distance
    polygon = body.outline_at(pose)
    return float(point_polygon_distance((x, y), polygon)[0]) - radius


def check_start(actor_dicts, start_pose, body):
    """Refuse scenarios whose actors stand on the chair's start pose."""
    for spec in parse_actors(actor_dicts):
        x, y = spec.waypoints[0]
        if body_gap(start_pose, body, x, y, spec.radius_m) <= CONTACT_GAP_M:
            raise ValueError("actor %s starts on the chair's start pose" % spec.id)


class Actor:
    """One walking disc: position, current target and state."""

    def __init__(self, spec):
        self.spec = spec
        self.x, self.y = spec.waypoints[0]
        self.target = 1 if len(spec.waypoints) > 1 else None
        self.direction = 1
        self.pause_left = 0.0
        self.state = "waiting"
        self.yield_s = 0.0
        self.walked_m = 0.0

    def _advance_target(self):
        n = len(self.spec.waypoints)
        nxt = self.target + self.direction
        if 0 <= nxt < n:
            self.target = nxt
        elif self.spec.repeat == "loop":
            self.target = 0
        elif self.spec.repeat == "pingpong":
            self.direction = -self.direction
            self.target = self.target + self.direction
        else:
            self.target = None
        self.pause_left = self.spec.pause_s

    def step(self, now, dt, armed_at, chair_pose=None, body=None):
        if self.target is None:
            self.state = "done" if len(self.spec.waypoints) > 1 else "standing"
            return
        if armed_at is None or now < armed_at + self.spec.start_after_s:
            self.state = "waiting"
            return
        if self.pause_left > 0:
            self.pause_left = max(0.0, self.pause_left - dt)
            self.state = "paused"
            return
        tx, ty = self.spec.waypoints[self.target]
        dx, dy = tx - self.x, ty - self.y
        distance = math.hypot(dx, dy)
        length = min(self.spec.speed_mps * dt, distance)
        nx, ny = (self.x + dx / distance * length, self.y + dy / distance * length) if distance > 1e-9 else (tx, ty)
        if self.spec.yield_gap_m > 0 and chair_pose is not None and body is not None:
            r = self.spec.radius_m
            ahead = body_gap(chair_pose, body, nx, ny, r)
            if ahead < self.spec.yield_gap_m and ahead < body_gap(chair_pose, body, self.x, self.y, r):
                self.state = "yielding"
                self.yield_s += dt
                return
        self.x, self.y = nx, ny
        self.walked_m += length
        self.state = "walking"
        if distance - length <= 1e-9:
            self._advance_target()

    def disc(self):
        from .world import CircleObstacle
        return CircleObstacle(self.x, self.y, self.spec.radius_m)


class SimActors:
    """The actors inside the simulator node (see the module docstring)."""

    def __init__(self, specs, body, static_obstacles, log_path=None):
        self.actors = [Actor(s) for s in specs]
        self.body = body
        self.static = tuple(static_obstacles)
        self.armed_at = None
        self.next_log = None
        self.touching = {a.spec.id: False for a in self.actors}
        self.contacts = {a.spec.id: 0 for a in self.actors}
        self.last_collisions = 0
        self.log_path = Path(log_path) if log_path is not None else None
        self._log = None

    @classmethod
    def from_config(cls, config, body, world, log_path=None):
        """None when the world spec has no actors; ``log_path`` defaults to ``actors.jsonl`` in the cwd."""
        items = (config.get("world") or {}).get("actors")
        if not items:
            return None
        check_start(items, tuple(config["start_pose"]), body)
        return cls(parse_actors(items), body, world.obstacles, log_path or Path.cwd() / LOG_NAME)

    def _write(self, record):
        if self.log_path is None:
            return
        if self._log is None:
            self._log = self.log_path.open("a", encoding="utf-8")
        self._log.write(json.dumps(record, ensure_ascii=False, allow_nan=False) + "\n")
        self._log.flush()

    def on_command(self, linear, angular, now):
        if self.armed_at is None and (abs(float(linear)) > 1e-3 or abs(float(angular)) > 1e-3):
            self.armed_at = float(now)
            self._write({"event": "first_command", "t": round(self.armed_at, 3)})

    def discs(self):
        return tuple(a.disc() for a in self.actors)

    def update(self, now, dt, base, world):
        """Advance the actors, put their discs into ``world.obstacles``, record contacts and the log line."""
        pose = tuple(base.pose)
        for actor in self.actors:
            actor.step(now, dt, self.armed_at, pose, self.body)
        world.obstacles = self.static + self.discs()
        moving = abs(base.v) > 0.01 or abs(base.w) > 0.01
        gaps = {}
        for actor in self.actors:
            gap = body_gap(pose, self.body, actor.x, actor.y, actor.spec.radius_m)
            gaps[actor.spec.id] = gap
            touching = gap <= CONTACT_GAP_M
            if touching and not self.touching[actor.spec.id]:
                self.contacts[actor.spec.id] += 1
                self._write({"event": "contact_start", "t": round(now, 3), "actor": actor.spec.id,
                             "chair_moving": bool(moving), "chair_v": round(base.v, 3), "actor_state": actor.state})
            elif not touching and self.touching[actor.spec.id]:
                self._write({"event": "contact_end", "t": round(now, 3), "actor": actor.spec.id})
            self.touching[actor.spec.id] = touching
        if base.collisions > self.last_collisions:
            nearest = min(gaps, key=gaps.get)
            if gaps[nearest] <= 0.03:
                self._write({"event": "chair_blocked_by_actor", "t": round(now, 3), "actor": nearest,
                             "gap_m": round(gaps[nearest], 3)})
            self.last_collisions = base.collisions
        if self.next_log is None or now + 1e-9 >= self.next_log:
            self.next_log = (now if self.next_log is None else self.next_log) + LOG_PERIOD_S
            self._write({"t": round(now, 3), "armed": self.armed_at is not None,
                         "chair": [round(v, 4) for v in pose], "chair_v": round(base.v, 3),
                         "chair_w": round(base.w, 3),
                         "actors": [{"id": a.spec.id, "x": round(a.x, 3), "y": round(a.y, 3),
                                     "gap_m": round(gaps[a.spec.id], 3), "state": a.state} for a in self.actors]})


def scenario_config(path, start_pose, body):
    """(world-spec actors, run-config provenance) for ``--actors``."""
    actors, provenance = load_scenario(path)
    check_start(actors, start_pose, body)
    return actors, provenance


def result_entries(config, output):
    """``{"actors": ...}`` for result.json from ``actors.jsonl``; {} for runs without actors."""
    items = (config.get("world") or {}).get("actors")
    if not items:
        return {}
    path = Path(output) / LOG_NAME
    section = {"scenario": config.get("actor_scenario"), "log": str(path),
               "interpretation": "SIMULATED_DISCS_ON_KNOWN_PATHS: pedestrians are discs that do not react to "
                                 "the chair unless yield_gap_m > 0; no legs, no 3D shape",
               "contact_gap_m": CONTACT_GAP_M}
    if not path.exists():
        section["error"] = "NO_ACTOR_LOG"
        return {"actors": section}
    rows, events = [], []
    for line in path.read_text(encoding="utf-8").splitlines():
        if not line.strip():
            continue
        record = json.loads(line)
        (events if "event" in record else rows).append(record)
    armed = next((e["t"] for e in events if e["event"] == "first_command"), None)
    section["first_command_sim_s"] = armed
    section["log_rows"] = len(rows)
    live = [r for r in rows if r["armed"]]
    per_actor = {}
    for spec in parse_actors(items):
        gaps = np.asarray([next(a["gap_m"] for a in r["actors"] if a["id"] == spec.id) for r in live], dtype=float)
        speeds = np.asarray([abs(r["chair_v"]) for r in live], dtype=float)
        states = [next(a["state"] for a in r["actors"] if a["id"] == spec.id) for r in live]
        entry = {"min_gap_m": round(float(gaps.min()), 3) if len(gaps) else None,
                 "min_gap_sim_s": live[int(np.argmin(gaps))]["t"] if len(gaps) else None,
                 "contacts": sum(1 for e in events if e["event"] == "contact_start" and e["actor"] == spec.id),
                 "contacts_while_chair_moving": sum(1 for e in events if e["event"] == "contact_start"
                                                    and e["actor"] == spec.id and e["chair_moving"]),
                 "chair_blocked_by_actor": sum(1 for e in events if e["event"] == "chair_blocked_by_actor"
                                               and e["actor"] == spec.id),
                 "yielding_s": round(LOG_PERIOD_S * sum(s == "yielding" for s in states), 1)}
        for near in NEAR_M:
            entry["seconds_within_%g_m" % near] = round(LOG_PERIOD_S * int((gaps < near).sum()), 1)
        close = gaps < 1.0
        entry["chair_speed_within_1_m"] = ({"max": round(float(speeds[close].max()), 3),
                                            "median": round(float(np.median(speeds[close])), 3)}
                                           if close.any() else None)
        per_actor[spec.id] = entry
    section["per_actor"] = per_actor
    if live:
        nearest = np.asarray([min(a["gap_m"] for a in r["actors"]) for r in live], dtype=float)
        stopped = np.asarray([abs(r["chair_v"]) < 0.02 and abs(r["chair_w"]) < 0.05 for r in live])
        section["chair_stopped_s"] = {"actor_within_1_5_m": round(LOG_PERIOD_S * int((stopped & (nearest < 1.5)).sum()), 1),
                                      "no_actor_within_1_5_m": round(LOG_PERIOD_S * int((stopped & (nearest >= 1.5)).sum()), 1)}
        section["min_gap_m"] = round(float(nearest.min()), 3)
    section["contacts"] = sum(e["contacts"] for e in per_actor.values())
    section["contacts_while_chair_moving"] = sum(e["contacts_while_chair_moving"] for e in per_actor.values())
    return {"actors": section}
