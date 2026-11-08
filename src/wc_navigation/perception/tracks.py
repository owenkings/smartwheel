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

- ``moving``: confirmed, seen standing on the floor at least once (points below ``person_ground_m``),
  and over the last ``moving_window_s`` both its filtered speed is above ``moving_speed_mps`` and it
  moved more than ``moving_displacement_m``.
- ``person_candidate``: not moving, but shaped like a standing person: ``person_diameter_m`` across,
  top at least ``person_top_m`` above the floor, points below ``person_ground_m`` (stands on the floor),
  at least ``person_wall_gap_m`` from the map's walls, and not elongated.
- ``static_unknown``: anything else that is not in the map (boxes, an open door leaf against the wall).

Door frames and walls are in the map and never become tracks; an unmapped door leaf touches the wall
and is elongated, so it stays ``static_unknown``. A person standing still against a wall is also
``static_unknown``: still an obstacle, only without the person margin. A floating artefact that both
lidars report at the same place has no points near the floor, so it is neither a ``person_candidate``
nor ``moving``, even when it moves along with the chair.

``Tracker.message(t)`` gives the ``/obstacles/tracks`` JSON (schema ``wc_obstacle_tracks/1``).
"""
from __future__ import annotations

from dataclasses import asdict, dataclass
import math

import numpy as np

SCHEMA = "wc_obstacle_tracks/1"
CLASSES = ("moving", "person_candidate", "static_unknown")


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
    accel_noise_mps2: float = 1.5
    measurement_noise_m: float = 0.05
    person_min_diameter_m: float = 0.20
    person_max_diameter_m: float = 0.80
    person_top_m: float = 1.00
    person_ground_m: float = 0.30
    person_wall_gap_m: float = 0.15
    elongated_ratio: float = 3.0

    def __post_init__(self):
        for name in self.__dataclass_fields__:
            value = getattr(self, name)
            if not isinstance(value, (int, float)) or not math.isfinite(float(value)) or float(value) <= 0:
                raise ValueError(name + " must be a positive finite number")
        if self.min_height_m >= self.max_height_m or self.person_min_diameter_m >= self.person_max_diameter_m:
            raise ValueError("height window and person diameters must be increasing")
        if self.lost_after_s >= self.drop_after_s:
            raise ValueError("lost_after_s must be shorter than drop_after_s")
        if int(self.confirm_hits) != self.confirm_hits or int(self.min_points) != self.min_points:
            raise ValueError("confirm_hits and min_points must be integers")

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
        self.history = [(float(t), float(self.x[0]), float(self.x[1]))]
        self.shape = cluster
        self.sources = set()
        # seen standing on the floor at least once (a walking person's legs leave the view near the chair)
        self.grounded = cluster["height_m"][0] <= cfg.person_ground_m

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
        self.history.append((float(t), float(self.x[0]), float(self.x[1])))
        self.history = [h for h in self.history if t - h[0] <= max(self.cfg.moving_window_s, 0.0) + 1e-9]

    def speed(self):
        return float(math.hypot(self.x[2], self.x[3]))

    def state(self, t):
        if t - self.last_t > self.cfg.lost_after_s:
            return "lost"
        return "confirmed" if self.hits >= self.cfg.confirm_hits else "tentative"

    def is_moving(self, t):
        if self.state(t) != "confirmed" or len(self.history) < 2 or not self.grounded:
            return False
        t0, x0, y0 = self.history[0]
        t1, x1, y1 = self.history[-1]
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

    def update(self, t, points, source="left"):
        """One frame of (x, y, h) obstacle points in odom at time ``t``; returns the current tracks."""
        t = float(t)
        if self.t is not None and t < self.t - 1e-9:
            raise ValueError("tracker updates must not go back in time")
        self.t = t
        found = clusters(self.candidate_points(points), self.cfg)
        for track in self.tracks:
            track.predict(t)
        # greedy nearest-neighbour association, closest pairs first
        pairs = []
        for i, track in enumerate(self.tracks):
            gate = self.cfg.gate_m + track.speed() * max(0.0, t - track.last_t)
            for j, c in enumerate(found):
                d = math.hypot(c["xy"][0] - track.x[0], c["xy"][1] - track.x[1])
                if d <= gate:
                    pairs.append((d, i, j))
        used_tracks, used_clusters = set(), set()
        for d, i, j in sorted(pairs):
            if i in used_tracks or j in used_clusters:
                continue
            self.tracks[i].update(t, found[j])
            self.tracks[i].sources.add(source)
            used_tracks.add(i)
            used_clusters.add(j)
        for j, c in enumerate(found):
            if j not in used_clusters:
                track = Track(self.next_id, t, c, self.cfg)
                track.sources.add(source)
                self.tracks.append(track)
                self.next_id += 1
        self.tracks = [tr for tr in self.tracks if t - tr.last_t <= self.cfg.drop_after_s]
        for tr in self.tracks:
            if tr.is_moving(t):
                self.ever_moving.add(tr.id)
        return self.current(t)

    def classify(self, track, t):
        if track.is_moving(t):
            return "moving"
        shape = track.shape
        diameter = 2.0 * shape["radius_m"]
        low, top = shape["height_m"]
        gap = None
        if self.map is not None:
            gap = float(self.map.distance(self._to_map(np.asarray(track.x[:2])[None, :]))[0]) - shape["radius_m"]
        person = (self.cfg.person_min_diameter_m <= diameter <= self.cfg.person_max_diameter_m
                  and top >= self.cfg.person_top_m and low <= self.cfg.person_ground_m
                  and shape["elongation"] < self.cfg.elongated_ratio
                  and (gap is None or gap >= self.cfg.person_wall_gap_m))
        return "person_candidate" if person else "static_unknown"

    def current(self, t):
        out = []
        for tr in self.tracks:
            state = tr.state(t)
            if state == "tentative":
                continue
            gap = None
            if self.map is not None:
                gap = round(float(self.map.distance(self._to_map(tr.x[None, :2]))[0]) - tr.shape["radius_m"], 3)
            speed_var = float(tr.P[2, 2] + tr.P[3, 3])
            out.append({"id": tr.id, "x": round(float(tr.x[0]), 3), "y": round(float(tr.x[1]), 3),
                        "vx": round(float(tr.x[2]), 3), "vy": round(float(tr.x[3]), 3),
                        "speed": round(tr.speed(), 3), "speed_std": round(math.sqrt(max(speed_var, 0.0)), 3),
                        "radius_m": round(float(tr.shape["radius_m"]), 3),
                        "height_m": [round(v, 3) for v in tr.shape["height_m"]], "points": tr.shape["points"],
                        "age_s": round(t - tr.first_t, 2), "seen_by": sorted(tr.sources), "state": state,
                        "class": self.classify(tr, t), "wall_gap_m": gap,
                        "has_moved": tr.id in self.ever_moving})
        return out

    def message(self, t, frame="odom"):
        return {"schema": SCHEMA, "stamp": round(float(t), 3), "frame": frame, "tracks": self.current(float(t))}
