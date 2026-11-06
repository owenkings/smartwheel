"""Ground-truth worlds and laser ray casting for the closed-loop simulator.

Simulation only. The occupancy grid is the true environment used to generate
synthetic scans and to detect simulated collisions. Unknown cells of a recorded
map are treated as solid, so the simulated robot never enters space the source
map did not observe. Grids use ROS bottom-up row order: ``data[row, column]``.
"""
from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path

import numpy as np
from scipy.ndimage import distance_transform_edt

from ..geometry import Grid, _number


@dataclass(frozen=True)
class CircleObstacle:
    """A solid disc in world coordinates, e.g. an object absent from the map."""
    x: float
    y: float
    radius: float

    def __post_init__(self):
        for name in ("x", "y"):
            object.__setattr__(self, name, _number(getattr(self, name), "obstacle " + name))
        object.__setattr__(self, "radius", _number(self.radius, "obstacle radius", positive=True))


@dataclass(frozen=True)
class LaserModel:
    """Planar scan model of a forward-facing range sensor mounted on base_link.

    Defaults approximate the left XT-M60: horizontal span measured from native
    RoomA1 frames (about +/-57.5 degrees, 160 columns) and the 0.3-8 m range used
    by the offline safety tools. The mount is the configured parallel-axis lever
    arm. None of these values is a validated sensor model.
    """
    mount_x: float = 0.323
    mount_y: float = 0.229
    mount_yaw: float = 0.0
    angle_min: float = -1.0036
    angle_max: float = 1.0036
    beams: int = 160
    range_min: float = 0.3
    range_max: float = 8.0
    noise_std: float = 0.01

    def __post_init__(self):
        for name in ("mount_x", "mount_y", "mount_yaw", "angle_min", "angle_max"):
            object.__setattr__(self, name, _number(getattr(self, name), name))
        object.__setattr__(self, "range_min", _number(self.range_min, "range_min", positive=True))
        object.__setattr__(self, "range_max", _number(self.range_max, "range_max", positive=True))
        object.__setattr__(self, "noise_std", _number(self.noise_std, "noise_std", nonnegative=True))
        if type(self.beams) is not int or not 2 <= self.beams <= 4096:
            raise ValueError("beams must be an integer between 2 and 4096")
        if not -math.pi <= self.angle_min < self.angle_max <= math.pi:
            raise ValueError("scan angles must satisfy -pi <= min < max <= pi")
        if self.range_min >= self.range_max or self.range_max > 50.0:
            raise ValueError("scan range must satisfy 0 < min < max <= 50 m")

    @property
    def angle_increment(self):
        return (self.angle_max - self.angle_min) / (self.beams - 1)

    def angles(self):
        return self.angle_min + self.angle_increment * np.arange(self.beams)

    def as_dict(self):
        return {name: getattr(self, name) for name in self.__dataclass_fields__}


class SimWorld:
    """Solid occupancy plus optional discs; outside the map counts as solid."""

    def __init__(self, grid, obstacles=(), *, name="", source=None):
        if not isinstance(grid, Grid):
            raise ValueError("grid must be a Grid")
        if abs(math.atan2(math.sin(grid.origin[2]), math.cos(grid.origin[2]))) > 1e-9:
            raise ValueError("simulation worlds must be axis aligned (origin yaw = 0)")
        self.grid = grid
        self.name = str(name)
        self.source = source
        self.obstacles = tuple(o if isinstance(o, CircleObstacle) else CircleObstacle(*o) for o in obstacles)
        self.solid = np.asarray(grid.data) != 0
        self.solid.setflags(write=False)
        padded = np.pad(~self.solid, 1, mode="constant", constant_values=False)
        distance = distance_transform_edt(padded, sampling=grid.resolution)[1:-1, 1:-1]
        # Cell-centre EDT minus half a diagonal bounds the distance to solid squares.
        self.clearance_m = np.maximum(0.0, distance - grid.resolution / math.sqrt(2.0))
        self.clearance_m.setflags(write=False)

    @property
    def resolution(self):
        return self.grid.resolution

    def clearance(self, x, y):
        """Conservative clearance from a world point to solid cells and discs."""
        res = self.grid.resolution
        gx = (x - self.grid.origin[0]) / res
        gy = (y - self.grid.origin[1]) / res
        col, row = math.floor(gx), math.floor(gy)
        if not (0 <= col < self.grid.width and 0 <= row < self.grid.height):
            return 0.0
        offset = math.hypot(gx - col - 0.5, gy - row - 0.5) * res
        value = max(0.0, float(self.clearance_m[row, col]) - offset)
        for o in self.obstacles:
            value = min(value, max(0.0, math.hypot(x - o.x, y - o.y) - o.radius))
        return value

    def collides(self, x, y, radius):
        return self.clearance(x, y) + 1e-12 < radius

    def clearance_grid(self):
        """Clearance at every cell centre, including disc obstacles (conservative)."""
        values = np.array(self.clearance_m, dtype=float)
        if self.obstacles:
            res = self.grid.resolution
            rows, cols = np.indices(values.shape)
            cx = self.grid.origin[0] + (cols + 0.5) * res
            cy = self.grid.origin[1] + (rows + 0.5) * res
            for o in self.obstacles:
                values = np.minimum(values, np.maximum(0.0, np.hypot(cx - o.x, cy - o.y) - o.radius))
        return values

    def segment_collides(self, start, end, radius):
        """Check a swept disc between two poses without skipping cells."""
        (x0, y0), (x1, y1) = start, end
        length = math.hypot(x1 - x0, y1 - y0)
        count = max(1, math.ceil(length / (self.grid.resolution * 0.5)))
        penalty = length / count * 0.5
        for k in range(count + 1):
            f = k / count
            if self.clearance(x0 + f * (x1 - x0), y0 + f * (y1 - y0)) + 1e-12 < radius + penalty:
                return True
        return False

    def raycast(self, origin_xy, angles, max_range):
        """Distance along each world-frame angle to the first solid surface.

        Rays are sampled every quarter cell. A diagonal step between two cells
        also hits when both side-adjacent cells are solid, so diagonal walls do
        not leak. Returns ``inf`` where nothing is hit within ``max_range``.
        """
        ox, oy = float(origin_xy[0]), float(origin_xy[1])
        angles = np.asarray(angles, dtype=float)
        res = self.grid.resolution
        step = res * 0.25
        t = step * np.arange(1, int(math.ceil(max_range / step)) + 2)
        cos, sin = np.cos(angles)[:, None], np.sin(angles)[:, None]
        cols = np.floor((ox + cos * t - self.grid.origin[0]) / res).astype(np.int64)
        rows = np.floor((oy + sin * t - self.grid.origin[1]) / res).astype(np.int64)
        width, height = self.grid.width, self.grid.height
        inside = (cols >= 0) & (cols < width) & (rows >= 0) & (rows < height)
        cc, rr = np.clip(cols, 0, width - 1), np.clip(rows, 0, height - 1)
        hit = ~inside | self.solid[rr, cc]
        start_col = math.floor((ox - self.grid.origin[0]) / res)
        start_row = math.floor((oy - self.grid.origin[1]) / res)
        prev_c = np.concatenate([np.full((len(angles), 1), start_col), cols[:, :-1]], axis=1)
        prev_r = np.concatenate([np.full((len(angles), 1), start_row), rows[:, :-1]], axis=1)
        diagonal = (cols != prev_c) & (rows != prev_r)
        pc, pr = np.clip(prev_c, 0, width - 1), np.clip(prev_r, 0, height - 1)
        side_a = self.solid[pr, cc] | ~((cols >= 0) & (cols < width) & (prev_r >= 0) & (prev_r < height))
        side_b = self.solid[rr, pc] | ~((prev_c >= 0) & (prev_c < width) & (rows >= 0) & (rows < height))
        hit |= diagonal & side_a & side_b
        any_hit = hit.any(axis=1)
        first = np.argmax(hit, axis=1)
        ranges = np.where(any_hit, t[first] - step * 0.5, np.inf)
        for o in self.obstacles:
            dx, dy = o.x - ox, o.y - oy
            along = cos[:, 0] * dx + sin[:, 0] * dy
            perpendicular2 = dx * dx + dy * dy - along * along
            disc = o.radius * o.radius - perpendicular2
            with np.errstate(invalid="ignore"):
                entry = along - np.sqrt(np.maximum(disc, 0.0))
            valid = (disc >= 0) & (along + np.sqrt(np.maximum(disc, 0.0)) > 0)
            entry = np.where(entry < 0, 0.0, entry)
            ranges = np.where(valid & (entry < ranges), entry, ranges)
        ranges = np.where(ranges <= max_range, ranges, np.inf)
        return ranges

    def scan(self, pose, laser, rng=None):
        """Return REP-117 ranges: ``-inf`` too close, ``+inf`` no return."""
        x, y, yaw = pose
        c, s = math.cos(yaw), math.sin(yaw)
        sx = x + c * laser.mount_x - s * laser.mount_y
        sy = y + s * laser.mount_x + c * laser.mount_y
        world_angles = yaw + laser.mount_yaw + laser.angles()
        true_ranges = self.raycast((sx, sy), world_angles, laser.range_max + 4 * laser.noise_std + 0.05)
        ranges = true_ranges.copy()
        finite = np.isfinite(ranges)
        if rng is not None and laser.noise_std > 0:
            ranges[finite] += rng.normal(0.0, laser.noise_std, int(finite.sum()))
        out = np.where(ranges > laser.range_max, np.inf, ranges)
        out = np.where(finite & (true_ranges < laser.range_min), -np.inf, out)
        out = np.where(np.isfinite(out) & (out < laser.range_min), laser.range_min, out)
        return out

    def free_area_m2(self):
        return float((~self.solid).sum()) * self.grid.resolution ** 2

    def describe(self):
        return {"name": self.name, "source": self.source, "resolution_m": self.grid.resolution,
                "width_cells": self.grid.width, "height_cells": self.grid.height,
                "origin": list(self.grid.origin), "free_area_m2": self.free_area_m2(),
                "obstacles": [[o.x, o.y, o.radius] for o in self.obstacles],
                "unknown_cells_treated_as_solid": True}


def _rect(data, resolution, x0, y0, x1, y1, value):
    c0, c1 = int(round(x0 / resolution)), int(round(x1 / resolution))
    r0, r1 = int(round(y0 / resolution)), int(round(y1 / resolution))
    data[max(r0, 0):max(r1, 0), max(c0, 0):max(c1, 0)] = value


# Rectangles in metres: (x0, y0, x1, y1). Walls 0.2 m thick; openings sized for
# the assumed 0.55 m circle plus 0.10 m margin (1.8 m door, 2.4 m corridor).
_SYNTHETIC = {
    "two_rooms": {
        "size": (10.0, 7.0),
        "start": (1.2, 1.4, 0.0),
        "walls": [(0.0, 2.6, 1.6, 2.8), (3.4, 2.6, 6.6, 2.8), (8.4, 2.6, 10.0, 2.8),
                  (4.9, 2.8, 5.1, 7.0)],
        "furniture": [(1.0, 5.6, 2.4, 6.4), (6.4, 4.4, 7.4, 5.4), (8.6, 0.2, 9.8, 0.8)],
    },
    "office_floor": {
        "size": (20.0, 12.0),
        "start": (1.4, 6.0, 0.0),
        "walls": [  # corridor y 4.8..7.2; rooms above and below with 1.8 m doors
            (0.0, 7.2, 2.4, 7.4), (4.2, 7.2, 8.8, 7.4), (10.6, 7.2, 15.2, 7.4), (17.0, 7.2, 20.0, 7.4),
            (6.6, 7.4, 6.8, 12.0), (13.2, 7.4, 13.4, 12.0),
            (0.0, 4.6, 4.0, 4.8), (5.8, 4.6, 14.0, 4.8), (15.8, 4.6, 20.0, 4.8),
            (9.9, 0.0, 10.1, 4.6),
        ],
        # Revision 2: the corridor cabinet stands against the lower wall, leaving
        # a 1.9 m passage. Revision 1 had a block in the middle of the corridor
        # that left two 1.0 m gaps, closing the east half to the assumed 1.3 m
        # wide envelope (run 20261105T084454Z explored only the west half).
        "furniture": [(1.0, 9.6, 2.6, 10.4), (9.0, 10.6, 11.0, 11.4), (16.4, 8.6, 17.4, 9.6),
                      (3.0, 1.0, 4.4, 1.8), (13.0, 1.4, 14.2, 3.0), (8.0, 4.8, 8.6, 5.3)],
        "revision": 2,
    },
}


def synthetic_names():
    return sorted(_SYNTHETIC)


def synthetic_world(name, obstacles=(), resolution=0.05):
    """Deterministic floor plan; the outer boundary is a 0.2 m wall."""
    if name not in _SYNTHETIC:
        raise ValueError("unknown synthetic world: " + str(name))
    spec = _SYNTHETIC[name]
    width_m, height_m = spec["size"]
    data = np.zeros((int(round(height_m / resolution)), int(round(width_m / resolution))), dtype=np.int8)
    for rect in ((0, 0, width_m, 0.2), (0, height_m - 0.2, width_m, height_m),
                 (0, 0, 0.2, height_m), (width_m - 0.2, 0, width_m, height_m)):
        _rect(data, resolution, *rect, 100)
    for rect in spec["walls"] + spec["furniture"]:
        _rect(data, resolution, *rect, 100)
    layout = json.dumps({k: spec[k] for k in sorted(spec)}, sort_keys=True).encode()
    world = SimWorld(Grid(data, resolution, (0.0, 0.0, 0.0)), obstacles, name="synthetic:" + name,
                     source={"kind": "synthetic", "name": name, "revision": spec.get("revision", 1),
                             "layout_sha256": hashlib.sha256(layout).hexdigest()})
    return world, tuple(spec["start"])


def _sha256(path):
    h = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1 << 20), b""):
            h.update(block)
    return h.hexdigest()


def map_world(yaml_path, obstacles=()):
    """Load a saved trinary map as ground truth; unknown cells become solid."""
    from wc_runtime.map_viewer import inspect_grid
    yaml_path = Path(yaml_path).expanduser().resolve(strict=True)
    stats, data, meta = inspect_grid(yaml_path)
    grid = Grid(data, meta["resolution"], tuple(meta["origin"]))
    source = {"kind": "map", "yaml": str(yaml_path), "image": stats["image"],
              "yaml_sha256": _sha256(yaml_path), "image_sha256": _sha256(stats["image"])}
    return SimWorld(grid, obstacles, name="map:" + yaml_path.parent.name, source=source)


def load_world(spec):
    """Rebuild a world from its JSON spec; map hashes must still match."""
    obstacles = [CircleObstacle(*o) for o in spec.get("obstacles", [])]
    if spec.get("kind") == "synthetic":
        world = synthetic_world(spec["name"], obstacles)[0]
        if spec.get("layout_sha256") and spec["layout_sha256"] != world.source["layout_sha256"]:
            raise ValueError("synthetic world layout changed since the run was configured: " + spec["name"])
        return world
    if spec.get("kind") == "map":
        world = map_world(spec["yaml"], obstacles)
        for key in ("yaml_sha256", "image_sha256"):
            if spec.get(key) and spec[key] != world.source[key]:
                raise ValueError("world map changed since the run was configured: " + key)
        return world
    raise ValueError("world spec kind must be synthetic or map")


def auto_start(world, radius_m, margin_m):
    """Pick a start in the largest free component, facing its far end."""
    from ..geometry import inflate_grid, demo_endpoints
    grid = Grid(np.where(world.solid, 100, 0).astype(np.int8), world.grid.resolution, world.grid.origin)
    start, goal = demo_endpoints(inflate_grid(grid, radius_m + world.grid.resolution * math.sqrt(2), margin_m))
    return (start[0], start[1], math.atan2(goal[1] - start[1], goal[0] - start[0]))
