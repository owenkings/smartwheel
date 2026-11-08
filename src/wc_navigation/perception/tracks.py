"""Tracks of obstacles that are not in the map, with position, velocity and a coarse class; pure numpy, no ROS.

Window C finds the moving obstacles; how the chair reacts is window B's (status/C.md 11-08 13:30).

Every update takes the obstacle points of one lidar frame (or of both lidars' latest frames) in the odom
frame with their height above the floor, removes points within ``map_clearance_m`` of an occupied map cell
(walls, door frames, mapped furniture), clusters the rest on a ``cell_m`` grid (8-neighbour connected),
and tracks the clusters with a constant-velocity Kalman filter (nearest-neighbour association inside
``gate_m`` plus the distance the track may have moved). A track is ``tentative`` until it was seen in
``confirm_hits`` updates, ``lost`` after ``lost_after_s`` without a match, and dropped after
``drop_after_s``.

Classes (a hint for the reaction, not the reaction itself):

- ``moving``: in ``moving_confirm_updates`` consecutive updates the track was confirmed, had been seen
  standing on the floor (points below ``person_ground_m``), moved more than ``moving_displacement_m``
  over the last ``moving_window_s`` with a filtered speed between ``moving_speed_mps`` and
  ``max_speed_mps``, had a radius of at most ``moving_max_radius_m`` in every match of that window, was
  not elongated, kept ``moving_min_wall_gap_m`` from the map's walls, and - when the chair's velocity is
  given - did not just ride along with the chair (relative speed of at least ``ego_margin_mps``). Without
  these checks the visible part of a long or wall-side object slides along with the chair's view, and
  pieces of a large unmapped object hop from one to the next, and look like they walk (ZL1107 mapping
  recording). The association gate grows with the track's speed only up to ``max_speed_mps``.
- ``person_candidate``: not moving, but shaped like a standing person: ``person_diameter_m`` across,
  top at least ``person_top_m`` above the floor, points below ``person_ground_m`` (stands on the floor),
  at least ``person_wall_gap_m`` from the map's walls, and not elongated.
- ``static_unknown``: anything else that is not in the map (boxes, an open door leaf against the wall).

Door frames and walls are in the map and never become tracks; an unmapped door leaf touches the wall
and is elongated, so it stays ``static_unknown``. A person standing still against a wall is also
``static_unknown``: still an obstacle, only without the person margin. A floating artefact that both
lidars report at the same place has no points near the floor, so it is neither a ``person_candidate``
nor ``moving``, even when it moves along with the chair.

A track that is not matched any more is ``lost`` after ``lost_after_s`` and published until it is dropped
after ``drop_after_s``: it keeps the class and the velocity of its last match, its position is extrapolated
with that velocity (``x - vx * last_seen_s`` is the last measured position) and its standard deviations
grow. Tentative tracks are never published, not even when they get lost.

``Tracker.message(t)`` gives the ``/obstacles/tracks`` JSON (schema ``wc_obstacle_tracks/1``, layout in
``tracks_message``). ``t`` is the measurement time of the points: the lidar frame's header stamp minus the
lidar time offset (live: ``config/calibration/lidar_time_offset.json``; offline: ``--lidar-time-offset-ms``).
``track_entry`` and ``tracks_message`` build the same message for other producers (window B's simulated
truth tracks); ``check_message`` is the consumer's check.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass
import math

import numpy as np

SCHEMA = "wc_obstacle_tracks/1"
CLASSES = ("moving", "person_candidate", "static_unknown")
STATES = ("confirmed", "lost")
SOURCES = ("perception", "truth")
# keys every track of a message has; consumers ignore keys they do not know
ENTRY_KEYS = ("id", "x", "y", "vx", "vy", "vx_std", "vy_std", "speed", "speed_std", "radius_m", "height_m",
              "points", "age_s", "last_seen_s", "seen_by", "state", "class", "wall_gap_m", "elongation",
              "has_moved")


def _number(value, name, low=None):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(float(value)):
        raise ValueError("track %s must be a finite number" % name)
    if low is not None and float(value) < low:
        raise ValueError("track %s must be at least %g" % (name, low))
    return float(value)


def track_entry(track_id, x, y, vx, vy, vx_std, vy_std, radius_m, cls, state="confirmed", *, age_s=0.0,
                last_seen_s=0.0, height_m=None, points=0, seen_by=(), wall_gap_m=None, elongation=None,
                has_moved=None):
    """One track of the ``/obstacles/tracks`` message, checked and rounded.

    Odom frame, metres, m/s, seconds. ``vx_std``/``vy_std``: standard deviations of the velocity along the
    odom axes; ``speed_std`` is their root sum of squares. ``height_m`` ([low, top] above the floor),
    ``wall_gap_m`` (clearance to the map's walls) and ``elongation`` (plan-view axis ratio) are None when the
    producer does not know them (simulated truth); ``points`` is 0 then. ``has_moved`` defaults to
    ``cls == "moving"``.
    """
    if isinstance(track_id, bool) or not isinstance(track_id, (int, np.integer)) or int(track_id) < 1:
        raise ValueError("track id must be a positive integer")
    if cls not in CLASSES:
        raise ValueError("track class must be one of " + ", ".join(CLASSES))
    if state not in STATES:
        raise ValueError("track state must be one of " + ", ".join(STATES))
    x, y, vx, vy = (_number(v, n) for v, n in ((x, "x"), (y, "y"), (vx, "vx"), (vy, "vy")))
    vx_std, vy_std = _number(vx_std, "vx_std", 0.0), _number(vy_std, "vy_std", 0.0)
    radius_m = _number(radius_m, "radius_m", 0.0)
    age_s, last_seen_s = _number(age_s, "age_s", 0.0), _number(last_seen_s, "last_seen_s", 0.0)
    if last_seen_s > age_s + 1e-6:
        raise ValueError("track last_seen_s must not exceed age_s")
    if height_m is not None:
        low, top = (_number(v, "height_m") for v in height_m)
        if low > top:
            raise ValueError("track height_m must be [low, top]")
        height_m = [round(low, 3), round(top, 3)]
    if isinstance(points, bool) or not isinstance(points, (int, np.integer)) or int(points) < 0:
        raise ValueError("track points must be a non-negative integer")
    seen_by = sorted({str(s) for s in seen_by})
    return {"id": int(track_id), "x": round(x, 3), "y": round(y, 3), "vx": round(vx, 3), "vy": round(vy, 3),
            "vx_std": round(vx_std, 3), "vy_std": round(vy_std, 3), "speed": round(math.hypot(vx, vy), 3),
            "speed_std": round(math.hypot(vx_std, vy_std), 3), "radius_m": round(radius_m, 3),
            "height_m": height_m, "points": int(points), "age_s": round(age_s, 2),
            "last_seen_s": round(last_seen_s, 2), "seen_by": seen_by, "state": state, "class": cls,
            "wall_gap_m": None if wall_gap_m is None else round(_number(wall_gap_m, "wall_gap_m"), 3),
            "elongation": None if elongation is None else round(_number(elongation, "elongation", 0.0), 2),
            "has_moved": bool(cls == "moving" if has_moved is None else has_moved)}


def tracks_message(stamp, entries, frame="odom", source="perception"):
    """The ``/obstacles/tracks`` message (std_msgs/String JSON)::

        {"schema": "wc_obstacle_tracks/1", "stamp": <measurement time, s>, "frame": "odom",
         "source": "perception" | "truth", "tracks": [track_entry(...), ...]}

    ``stamp`` is the time the points were measured (lidar header stamp minus the lidar time offset),
    in the producer's clock (ROS time live and in simulation, recording seconds offline).
    """
    return check_message({"schema": SCHEMA, "stamp": round(float(stamp), 3), "frame": frame, "source": source,
                          "tracks": list(entries)})


def check_message(message):
    """The message itself when it follows ``wc_obstacle_tracks/1``; ValueError naming the first problem."""
    if not isinstance(message, dict) or message.get("schema") != SCHEMA:
        raise ValueError("not a %s message" % SCHEMA)
    _number(message.get("stamp"), "message stamp")
    if not isinstance(message.get("frame"), str) or not message["frame"]:
        raise ValueError("message frame must be a frame id")
    if message.get("source") not in SOURCES:
        raise ValueError("message source must be one of " + ", ".join(SOURCES))
    tracks = message.get("tracks")
    if not isinstance(tracks, list):
        raise ValueError("message tracks must be a list")
    ids = set()
    for entry in tracks:
        if not isinstance(entry, dict):
            raise ValueError("each track must be an object")
        missing = [k for k in ENTRY_KEYS if k not in entry]
        if missing:
            raise ValueError("track lacks " + ", ".join(missing))
        if not isinstance(entry["seen_by"], list) or not isinstance(entry["has_moved"], bool):
            raise ValueError("track seen_by must be a list and has_moved a boolean")
        if entry["height_m"] is not None and (not isinstance(entry["height_m"], list) or len(entry["height_m"]) != 2):
            raise ValueError("track height_m must be [low, top] or null")
        rebuilt = track_entry(entry["id"], entry["x"], entry["y"], entry["vx"], entry["vy"], entry["vx_std"],
                              entry["vy_std"], entry["radius_m"], entry["class"], entry["state"],
                              age_s=entry["age_s"], last_seen_s=entry["last_seen_s"], height_m=entry["height_m"],
                              points=entry["points"], seen_by=entry["seen_by"], wall_gap_m=entry["wall_gap_m"],
                              elongation=entry["elongation"], has_moved=entry["has_moved"])
        for key in ("speed", "speed_std"):
            if abs(_number(entry[key], key) - rebuilt[key]) > 0.002:
                raise ValueError("track %s does not match its components" % key)
        if entry["id"] in ids:
            raise ValueError("track ids must be unique within a message")
        ids.add(entry["id"])
    return message


@dataclass(frozen=True)
class TrackerConfig:
    cell_m: float = 0.10
    min_points: int = 5
    min_height_m: float = 0.05
    max_height_m: float = 1.80
    map_clearance_m: float = 0.15
    gate_m: float = 0.5
    confirm_hits: int = 3
    lost_after_s: float = 1.0
    drop_after_s: float = 2.0
    moving_speed_mps: float = 0.25
    moving_displacement_m: float = 0.30
    moving_window_s: float = 1.0
    moving_confirm_updates: int = 3
    moving_max_radius_m: float = 0.50
    # faster than a person runs in a corridor: caps the association gate's growth; such tracks are not moving
    max_speed_mps: float = 2.5
    moving_min_wall_gap_m: float = 0.10
    ego_margin_mps: float = 0.50
    chair_moving_mps: float = 0.20
    accel_noise_mps2: float = 1.5
    measurement_noise_m: float = 0.05
    person_min_diameter_m: float = 0.20
    person_max_diameter_m: float = 0.80
    person_top_m: float = 1.00
    person_ground_m: float = 0.30
    person_wall_gap_m: float = 0.15
    # the visible front arc of a person is about 3:1 in plan view; a door leaf or a board along a wall 10:1 and more
    elongated_ratio: float = 5.0

    def __post_init__(self):
        for name in self.__dataclass_fields__:
            value = getattr(self, name)
            if not isinstance(value, (int, float)) or not math.isfinite(float(value)) or float(value) <= 0:
                raise ValueError(name + " must be a positive finite number")
        if self.min_height_m >= self.max_height_m or self.person_min_diameter_m >= self.person_max_diameter_m:
            raise ValueError("height window and person diameters must be increasing")
        if self.lost_after_s >= self.drop_after_s:
            raise ValueError("lost_after_s must be shorter than drop_after_s")
        for name in ("confirm_hits", "min_points", "moving_confirm_updates"):
            if int(getattr(self, name)) != getattr(self, name):
                raise ValueError(name + " must be an integer")

    def as_dict(self):
        return asdict(self)


class StaticMap:
    """Distance from world points to the nearest occupied cell of a trinary grid (rows bottom-up)."""

    def __init__(self, data, resolution, origin_xy):
        from scipy.ndimage import distance_transform_edt
        occupied = np.asarray(data) == 100
        if occupied.ndim != 2:
            raise ValueError("2-D trinary grid required")
        self.resolution = float(resolution)
        self.origin = (float(origin_xy[0]), float(origin_xy[1]))
        self.shape = occupied.shape
        self.distance_m = distance_transform_edt(~occupied, sampling=self.resolution)

    @classmethod
    def from_yaml(cls, yaml_path):
        from wc_runtime.map_viewer import inspect_grid
        _, data, meta = inspect_grid(yaml_path)
        return cls(data, meta["resolution"], meta["origin"][:2])

    def distance(self, xy):
        xy = np.asarray(xy, dtype=float).reshape(-1, 2)
        col = np.floor((xy[:, 0] - self.origin[0]) / self.resolution).astype(np.int64)
        row = np.floor((xy[:, 1] - self.origin[1]) / self.resolution).astype(np.int64)
        inside = (col >= 0) & (col < self.shape[1]) & (row >= 0) & (row < self.shape[0])
        out = np.zeros(len(xy))                    # outside the map counts as structure
        out[inside] = self.distance_m[row[inside], col[inside]]
        return out


def clusters(points, cfg=TrackerConfig()):
    """Connected clusters of (x, y, h) points on the ``cell_m`` grid: list of feature dicts."""
    from scipy import ndimage
    p = np.asarray(points, dtype=float).reshape(-1, 3)
    p = p[np.isfinite(p).all(axis=1) & (p[:, 2] >= cfg.min_height_m) & (p[:, 2] <= cfg.max_height_m)]
    if not len(p):
        return []
    keys = np.floor(p[:, :2] / cfg.cell_m).astype(np.int64)
    lo = keys.min(axis=0)
    shape = keys.max(axis=0) - lo + 1
    if int(np.prod(shape)) > 25_000_000:
        raise ValueError("points spread too far for the clustering grid")
    grid = np.zeros(tuple(shape), dtype=bool)
    k = keys - lo
    grid[k[:, 0], k[:, 1]] = True
    labels, count = ndimage.label(grid, structure=np.ones((3, 3), dtype=bool))
    point_label = labels[k[:, 0], k[:, 1]]
    out = []
    for lab in range(1, count + 1):
        q = p[point_label == lab]
        if len(q) < cfg.min_points:
            continue
        centre = q[:, :2].mean(axis=0)
        radius = float(np.hypot(q[:, 0] - centre[0], q[:, 1] - centre[1]).max()) + cfg.cell_m / 2
        if len(q) >= 3:
            spread = np.linalg.eigvalsh(np.cov((q[:, :2] - centre).T))
            ratio = float(math.sqrt(max(spread[1], 1e-12) / max(spread[0], 1e-12)))
        else:
            ratio = 1.0
        out.append({"xy": centre, "radius_m": radius, "points": int(len(q)),
                    "height_m": (float(q[:, 2].min()), float(q[:, 2].max())), "elongation": ratio})
    return out


class Track:
    def __init__(self, track_id, t, cluster, cfg):
        self.id = int(track_id)
        self.x = np.array([cluster["xy"][0], cluster["xy"][1], 0.0, 0.0])
        self.P = np.diag([0.1 ** 2, 0.1 ** 2, 1.0, 1.0])
        self.first_t = self.last_t = self.t = float(t)
        self.hits = 1
        self.cfg = cfg
        # (t, x, y, cluster radius) of the matches within moving_window_s
        self.history = [(float(t), float(self.x[0]), float(self.x[1]), float(cluster["radius_m"]))]
        self.shape = cluster
        self.sources = set()
        # seen standing on the floor at least once (a walking person's legs leave the view near the chair)
        self.grounded = cluster["height_m"][0] <= cfg.person_ground_m
        self.moving_streak = 0                    # consecutive updates with evidence of its own motion
        self.cls = "static_unknown"               # class at the last match; a lost track keeps it

    def predict(self, t):
        dt = float(t) - self.t
        if dt <= 0:
            return
        F = np.eye(4)
        F[0, 2] = F[1, 3] = dt
        a2 = self.cfg.accel_noise_mps2 ** 2
        q = np.array([[dt ** 4 / 4, dt ** 3 / 2], [dt ** 3 / 2, dt ** 2]]) * a2
        Q = np.zeros((4, 4))
        Q[np.ix_([0, 2], [0, 2])] = q
        Q[np.ix_([1, 3], [1, 3])] = q
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + Q
        self.t = float(t)

    def update(self, t, cluster):
        H = np.zeros((2, 4))
        H[0, 0] = H[1, 1] = 1.0
        R = np.eye(2) * self.cfg.measurement_noise_m ** 2
        innovation = np.asarray(cluster["xy"], dtype=float) - H @ self.x
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ innovation
        self.P = (np.eye(4) - K @ H) @ self.P
        self.last_t = float(t)
        self.hits += 1
        self.shape = cluster
        self.grounded = self.grounded or cluster["height_m"][0] <= self.cfg.person_ground_m
        self.history.append((float(t), float(self.x[0]), float(self.x[1]), float(cluster["radius_m"])))
        self.history = [h for h in self.history if t - h[0] <= max(self.cfg.moving_window_s, 0.0) + 1e-9]

    def speed(self):
        return float(math.hypot(self.x[2], self.x[3]))

    def state(self, t):
        if self.hits < self.cfg.confirm_hits:
            return "tentative"                    # also when unseen: never published
        return "lost" if t - self.last_t > self.cfg.lost_after_s else "confirmed"

    def is_moving(self, t):
        if self.state(t) != "confirmed" or len(self.history) < 2 or not self.grounded:
            return False
        t0, x0, y0 = self.history[0][:3]
        t1, x1, y1 = self.history[-1][:3]
        if t1 - t0 < 0.5 * self.cfg.moving_window_s:
            return False
        return self.speed() > self.cfg.moving_speed_mps and math.hypot(x1 - x0, y1 - y0) > self.cfg.moving_displacement_m


class Tracker:
    """Tracks of unmapped obstacles in the odom frame (see the module docstring)."""

    def __init__(self, cfg=TrackerConfig(), static_map=None, odom_to_map=None):
        self.cfg = cfg
        self.map = static_map
        # map pose of the odom frame (x, y, yaw); identity when the map is in the odometry frame
        self.odom_to_map = tuple(odom_to_map) if odom_to_map is not None else (0.0, 0.0, 0.0)
        self.tracks = []
        self.next_id = 1
        self.t = None
        self.ever_moving = set()

    def _to_map(self, xy):
        x0, y0, yaw = self.odom_to_map
        c, s = math.cos(yaw), math.sin(yaw)
        xy = np.asarray(xy, dtype=float).reshape(-1, 2)
        return np.column_stack([x0 + c * xy[:, 0] - s * xy[:, 1], y0 + s * xy[:, 0] + c * xy[:, 1]])

    def candidate_points(self, points):
        """Points (x, y, h in odom) not explained by the map's occupied cells."""
        p = np.asarray(points, dtype=float).reshape(-1, 3)
        if self.map is None or not len(p):
            return p
        return p[self.map.distance(self._to_map(p[:, :2])) > self.cfg.map_clearance_m]

    def update(self, t, points, source="left", chair_velocity=None):
        """One frame of (x, y, h) obstacle points in odom measured at ``t``; returns the current tracks.

        ``source``: the lidar side(s) the points come from (a name or a sequence of names).
        ``chair_velocity`` (vx, vy in odom), when known, rejects tracks that only ride along with the chair.
        """
        t = float(t)
        if self.t is not None and t < self.t - 1e-9:
            raise ValueError("tracker updates must not go back in time")
        self.t = t
        sources = {source} if isinstance(source, str) else set(source)
        found = clusters(self.candidate_points(points), self.cfg)
        for track in self.tracks:
            track.predict(t)
        # greedy nearest-neighbour association, closest pairs first
        pairs = []
        for i, track in enumerate(self.tracks):
            gate = self.cfg.gate_m + min(track.speed(), self.cfg.max_speed_mps) * max(0.0, t - track.last_t)
            for j, c in enumerate(found):
                d = math.hypot(c["xy"][0] - track.x[0], c["xy"][1] - track.x[1])
                if d <= gate:
                    pairs.append((d, i, j))
        used_tracks, used_clusters = set(), set()
        for d, i, j in sorted(pairs):
            if i in used_tracks or j in used_clusters:
                continue
            self.tracks[i].update(t, found[j])
            self.tracks[i].sources.update(sources)
            used_tracks.add(i)
            used_clusters.add(j)
        for j, c in enumerate(found):
            if j not in used_clusters:
                track = Track(self.next_id, t, c, self.cfg)
                track.sources.update(sources)
                self.tracks.append(track)
                self.next_id += 1
        self.tracks = [tr for tr in self.tracks if t - tr.last_t <= self.cfg.drop_after_s]
        for tr in self.tracks:
            if tr.last_t == t:                    # judged only when it was seen in this update
                tr.moving_streak = tr.moving_streak + 1 if self._moves(tr, t, chair_velocity) else 0
                tr.cls = self.classify(tr)
            if tr.moving_streak >= self.cfg.moving_confirm_updates:
                self.ever_moving.add(tr.id)
        return self.current(t)

    def wall_gap(self, track):
        if self.map is None:
            return None
        return float(self.map.distance(self._to_map(np.asarray(track.x[:2])[None, :]))[0]) - track.shape["radius_m"]

    def _moves(self, track, t, chair_velocity):
        """This update's evidence that ``track`` moves by itself (not a view effect of the chair's motion)."""
        if not track.is_moving(t) or track.speed() > self.cfg.max_speed_mps:
            return False
        if max(h[3] for h in track.history) > self.cfg.moving_max_radius_m:
            return False                          # was part of something larger: pieces of it hop as the view changes
        if track.shape["elongation"] >= self.cfg.elongated_ratio:
            return False                          # the visible part of a long object slides along with the view
        gap = self.wall_gap(track)
        if gap is not None and gap < self.cfg.moving_min_wall_gap_m:
            return False                          # clusters on the walls' edges slide the same way
        if chair_velocity is not None and math.hypot(*chair_velocity) > self.cfg.chair_moving_mps:
            relative = math.hypot(track.x[2] - chair_velocity[0], track.x[3] - chair_velocity[1])
            if relative < self.cfg.ego_margin_mps:
                return False                      # rides along with the chair: artefact or view effect
        return True

    def classify(self, track):
        """Class of ``track`` from its latest match (``Track.cls`` keeps it while the track is lost)."""
        if track.moving_streak >= self.cfg.moving_confirm_updates:
            return "moving"
        shape = track.shape
        diameter = 2.0 * shape["radius_m"]
        low, top = shape["height_m"]
        gap = self.wall_gap(track)
        person = (self.cfg.person_min_diameter_m <= diameter <= self.cfg.person_max_diameter_m
                  and top >= self.cfg.person_top_m and low <= self.cfg.person_ground_m
                  and shape["elongation"] < self.cfg.elongated_ratio
                  and (gap is None or gap >= self.cfg.person_wall_gap_m))
        return "person_candidate" if person else "static_unknown"

    def current(self, t):
        """Published tracks (``track_entry`` dicts) as of the update at ``t``; tentative ones are left out."""
        out = []
        for tr in self.tracks:
            state = tr.state(t)
            if state == "tentative":
                continue
            out.append(track_entry(
                tr.id, tr.x[0], tr.x[1], tr.x[2], tr.x[3], math.sqrt(max(float(tr.P[2, 2]), 0.0)),
                math.sqrt(max(float(tr.P[3, 3]), 0.0)), tr.shape["radius_m"], tr.cls, state,
                age_s=max(0.0, t - tr.first_t), last_seen_s=max(0.0, t - tr.last_t),
                height_m=tr.shape["height_m"], points=tr.shape["points"], seen_by=tr.sources,
                wall_gap_m=self.wall_gap(tr),
                elongation=tr.shape["elongation"], has_moved=tr.id in self.ever_moving))
        return out

    def message(self, t, frame="odom"):
        """``tracks_message`` of the update at ``t`` (the points' measurement time)."""
        return tracks_message(t, self.current(float(t)), frame=frame, source="perception")
