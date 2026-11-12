"""Front check: scan points ahead of the standing chair that land in the map's free space (总管 2026-11-11 item 2).

On 2026-11-11 the corridor in front of the chair was empty, but the left lidar's 12 MHz phase wrap put points 1-6 m
ahead (surfaces 12.5-18.5 m away seen at their distance minus 12.49 m). NavFn planned around them and the chair turned
in place. This check makes such points visible before "start automatic", and in ``block`` mode the safety gate holds
the start while they are there (GateConfig.front_hold).

A point counts when it is at most ``range_m`` from the laser, in front of base_link, outside the body outline (plus
``body_margin_m``) and on a free map cell farther than ``wall_tolerance_m`` from the nearest occupied cell (the
tolerance keeps a start pose that is off by up to that much from turning the side walls into "free space"). The
verdict uses the median count of the last ``frames`` scans: more than ``max_points`` is "not clear"; no pose is "not
clear" too. Measured 2026-11-11 (D.md 23:40): 25-43 points in the corridor with the phase wrap, 8 at the
validation recording's doorway start facing across the corridor (no wrap there).
"""
from __future__ import annotations

from collections import deque
import math

import numpy as np

TOPIC = "/wc_live/front_check"
SCHEMA = "wc_front_check_v1"
MODES = ("block", "warn", "off")
RANGE_M = 5.0
WALL_TOLERANCE_M = 0.30
BODY_MARGIN_M = 0.05
FRAMES = 10
MAX_POINTS = 10
POSE_MAX_AGE_S = 1.0


def _inside_body(points, footprint, radius_m, margin):
    """Points inside the body outline grown by ``margin`` (polygon: inside, or within margin of an edge)."""
    if not footprint:
        return np.hypot(points[:, 0], points[:, 1]) <= float(radius_m) + margin
    polygon = np.asarray(footprint, dtype=float)
    x, y = points[:, 0], points[:, 1]
    inside = np.zeros(len(points), dtype=bool)
    near = np.zeros(len(points), dtype=bool)
    for i in range(len(polygon)):
        (x1, y1), (x2, y2) = polygon[i], polygon[(i + 1) % len(polygon)]
        crosses = ((y1 > y) != (y2 > y)) & (x < (x2 - x1) * (y - y1) / ((y2 - y1) + 1e-12) + x1)
        inside ^= crosses
        dx, dy = x2 - x1, y2 - y1
        t = np.clip(((x - x1) * dx + (y - y1) * dy) / (dx * dx + dy * dy + 1e-12), 0.0, 1.0)
        near |= np.hypot(x - (x1 + t * dx), y - (y1 + t * dy)) <= margin
    return inside | near


def free_points(scorer, ranges, angle_min, increment, lever, pose, footprint=(), radius_m=0.55, range_m=RANGE_M,
                wall_tolerance_m=WALL_TOLERANCE_M, body_margin_m=BODY_MARGIN_M):
    """(count, nearest range m or None) of one scan; ``pose`` (x, y, yaw) is base_link in the map, ``lever`` the
    laser's (x, y) in base_link (the live chain's laser frame has no rotation against base_link)."""
    ranges = np.asarray(ranges, dtype=float)
    angles = float(angle_min) + float(increment) * np.arange(len(ranges))
    ok = np.isfinite(ranges) & (ranges > 0.0) & (ranges <= float(range_m))
    r, a = ranges[ok], angles[ok]
    base = np.column_stack([r * np.cos(a) + float(lever[0]), r * np.sin(a) + float(lever[1])])
    keep = base[:, 0] > 0.0
    keep &= ~_inside_body(base, footprint, radius_m, body_margin_m)
    c, s = math.cos(pose[2]), math.sin(pose[2])
    world = np.column_stack([pose[0] + c * base[:, 0] - s * base[:, 1], pose[1] + s * base[:, 0] + c * base[:, 1]])
    selected = keep & (scorer.cell_values(world) == 0) & (scorer.lookup(world) > float(wall_tolerance_m))
    count = int(selected.sum())
    return count, (round(float(r[selected].min()), 3) if count else None)


class FrontCheck:
    """Counts of the last ``frames`` scans and the verdict published on TOPIC."""

    def __init__(self, scorer, lever, footprint=(), radius_m=0.55, mode="block", max_points=MAX_POINTS,
                 range_m=RANGE_M, wall_tolerance_m=WALL_TOLERANCE_M, frames=FRAMES):
        if mode not in MODES or mode == "off":
            raise ValueError("front check mode must be block or warn")
        if type(max_points) is not int or max_points < 0:
            raise ValueError("max_points must be a non-negative integer")
        self.scorer, self.lever, self.footprint, self.radius_m = scorer, tuple(lever), list(footprint or ()), radius_m
        self.mode, self.max_points, self.range_m = mode, max_points, float(range_m)
        self.wall_tolerance_m = float(wall_tolerance_m)
        self.counts = deque(maxlen=int(frames))
        self.nearest = deque(maxlen=int(frames))
        self.no_pose = 0

    def add_scan(self, ranges, angle_min, increment, pose):
        """One /scan with the map pose of base_link at that time (None: not localized)."""
        if pose is None:
            self.counts.clear()
            self.nearest.clear()
            self.no_pose += 1
            return None
        count, nearest = free_points(self.scorer, ranges, angle_min, increment, self.lever, pose, self.footprint,
                                     self.radius_m, self.range_m, self.wall_tolerance_m)
        self.counts.append(count)
        self.nearest.append(nearest)
        return count

    def verdict(self, mono_ns, time_s):
        """The message published on TOPIC (a dict for std_msgs/String JSON)."""
        value = {"schema": SCHEMA, "time_s": round(float(time_s), 3), "stamp_mono_ns": int(mono_ns),
                 "mode": self.mode, "max_points": self.max_points, "range_m": self.range_m,
                 "wall_tolerance_m": self.wall_tolerance_m, "frames": len(self.counts)}
        if not self.counts:
            value.update(points=None, last=None, nearest_m=None, clear=False, reason="NO_POSE_OR_SCAN")
            return value
        points = int(np.median(self.counts))
        nearest = [n for n in self.nearest if n is not None]
        clear = points <= self.max_points
        value.update(points=points, last=int(self.counts[-1]), nearest_m=(min(nearest) if nearest else None),
                     clear=clear, reason=None if clear else "FRONT_NOT_CLEAR")
        return value


def parse(text):
    """(clear, reason) of a TOPIC message for the gate; anything malformed is not clear."""
    import json
    try:
        value = json.loads(text)
    except ValueError:
        return False, "FRONT_CHECK_INVALID"
    if not isinstance(value, dict) or value.get("schema") != SCHEMA or type(value.get("clear")) is not bool:
        return False, "FRONT_CHECK_INVALID"
    return value["clear"], ("" if value["clear"] else str(value.get("reason") or "FRONT_NOT_CLEAR"))


def operator_text(value):
    """The status line's part: the median count, the limit and, when not clear, what to do."""
    if not value:
        return "车前空地 未算"
    if value.get("points") is None:
        return "车前空地 没有位姿或扫描"
    text = "车前空地 %d 点" % value["points"]
    if value.get("clear"):
        return text + "（门限 %d）" % value["max_points"]
    nearest = value.get("nearest_m")
    return text + " > 门限 %d%s：不要开始自动" % (value["max_points"], "，最近 %.2f m" % nearest if nearest else "")
