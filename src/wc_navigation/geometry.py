"""Pure map geometry and numerical differential-drive simulation.

The circular footprint is an explicit simulation assumption. Nothing in this
module opens hardware, publishes motion, edits a map, or qualifies navigation.
Occupancy arrays use ROS bottom-up row order: ``data[row, column]``.
"""
from collections import deque
from dataclasses import dataclass
import math
from numbers import Real

import numpy as np
from scipy.ndimage import distance_transform_edt


def _number(value, label, *, positive=False, nonnegative=False):
    if isinstance(value, (bool, np.bool_)) or not isinstance(value, Real):
        raise ValueError(label + " must be a finite number")
    value = float(value)
    if not math.isfinite(value) or (positive and value <= 0) or (nonnegative and value < 0):
        raise ValueError(label + " is outside its finite valid range")
    return value


def _point(value, label="point"):
    try:
        if len(value) != 2:
            raise ValueError(label + " must contain x and y")
        return (_number(value[0], label + " x"), _number(value[1], label + " y"))
    except (TypeError, IndexError) as exc:
        raise ValueError(label + " must contain x and y") from exc


def _angle(value):
    return (value + math.pi) % (2 * math.pi) - math.pi


@dataclass(frozen=True)
class Grid:
    data: np.ndarray
    resolution: float
    origin: tuple = (0.0, 0.0, 0.0)

    def __post_init__(self):
        array = np.asarray(self.data)
        if array.ndim != 2 or not array.size or array.size > 20_000_000:
            raise ValueError("occupancy must be a nonempty bounded two-dimensional grid")
        if array.dtype.kind not in "iu" or not np.isin(array, (-1, 0, 100)).all():
            raise ValueError("occupancy cells must be integer -1, 0, or 100")
        array = array.astype(np.int8, copy=True)
        array.setflags(write=False)
        resolution = _number(self.resolution, "resolution", positive=True)
        try:
            if len(self.origin) != 3:
                raise ValueError("origin must contain x, y, and yaw")
            origin = tuple(_number(x, "origin") for x in self.origin)
        except TypeError as exc:
            raise ValueError("origin must contain x, y, and yaw") from exc
        c, s = math.cos(origin[2]), math.sin(origin[2])
        extent_x, extent_y = array.shape[1] * resolution, array.shape[0] * resolution
        for x, y in ((0.0, 0.0), (extent_x, 0.0), (0.0, extent_y), (extent_x, extent_y)):
            if not all(math.isfinite(v) for v in (origin[0] + c * x - s * y,
                                                  origin[1] + s * x + c * y)):
                raise ValueError("grid metric extent is not finite")
        object.__setattr__(self, "data", array)
        object.__setattr__(self, "resolution", resolution)
        object.__setattr__(self, "origin", origin)

    @property
    def width(self):
        return self.data.shape[1]

    @property
    def height(self):
        return self.data.shape[0]

    def world_to_grid(self, point):
        """Return continuous grid coordinates, including points outside the map."""
        x, y = _point(point)
        ox, oy, yaw = self.origin
        c, s = math.cos(yaw), math.sin(yaw)
        dx, dy = x - ox, y - oy
        coordinates = ((c * dx + s * dy) / self.resolution,
                       (-s * dx + c * dy) / self.resolution)
        if not all(math.isfinite(v) for v in coordinates):
            raise ValueError("point-to-grid conversion is outside the finite numeric range")
        return coordinates

    def world_to_cell(self, point):
        """Return ``(column, row)``; finite points outside the map are rejected."""
        gx, gy = self.world_to_grid(point)
        col, row = math.floor(gx), math.floor(gy)
        if not (0 <= col < self.width and 0 <= row < self.height):
            raise ValueError("point is outside the map")
        return col, row

    def cell_to_world(self, cell):
        """Return the world coordinates of a cell center."""
        try:
            col, row = cell
        except (TypeError, ValueError) as exc:
            raise ValueError("cell must contain column and row") from exc
        if any(isinstance(v, (bool, np.bool_)) or not isinstance(v, (int, np.integer))
               for v in (col, row)) or not (0 <= col < self.width and 0 <= row < self.height):
            raise ValueError("cell indices must be integers inside the map")
        x = (int(col) + 0.5) * self.resolution
        y = (int(row) + 0.5) * self.resolution
        ox, oy, yaw = self.origin
        c, s = math.cos(yaw), math.sin(yaw)
        return ox + c * x - s * y, oy + s * x + c * y


@dataclass(frozen=True)
class InflatedGrid:
    grid: Grid
    blocked: np.ndarray
    clearance_m: np.ndarray
    radius_m: float
    margin_m: float

    @property
    def free(self):
        return ~self.blocked

    @property
    def required_clearance_m(self):
        return self.radius_m + self.margin_m

    def world_clearance(self, point):
        """Conservative clearance to occupied/unknown squares and map boundary.

        The center EDT bound is reduced by the distance to that center. This
        remains a lower bound for arbitrary continuous world coordinates.
        Finite outside-map coordinates have zero clearance.
        """
        gx, gy = self.grid.world_to_grid(point)
        col, row = math.floor(gx), math.floor(gy)
        if not (0 <= col < self.grid.width and 0 <= row < self.grid.height):
            return 0.0
        offset = math.hypot(gx - col - 0.5, gy - row - 0.5) * self.grid.resolution
        return max(0.0, float(self.clearance_m[row, col]) - offset)

    def is_free_world(self, point):
        return self.world_clearance(point) + 1e-12 >= self.required_clearance_m

    def segment_is_free(self, start, end, *, extra_margin_m=0.0):
        """Check a swept circular footprint without skipping between samples.

        Samples are at most half a cell apart. Each sampled clearance is also
        reduced by half the sample interval, using the distance function's
        Lipschitz bound to cover the continuous segment between samples.
        ``extra_margin_m`` can bound an integration arc's deviation from its chord.
        """
        start, end = np.asarray(_point(start)), np.asarray(_point(end))
        extra = _number(extra_margin_m, "extra margin", nonnegative=True)
        with np.errstate(over="ignore", invalid="ignore"):
            delta = end - start
        distance = math.hypot(float(delta[0]), float(delta[1]))
        if not math.isfinite(distance):
            raise ValueError("segment length is not finite")
        required = self.required_clearance_m + extra
        if distance <= 1e-15:
            return self.world_clearance(start) + 1e-12 >= required
        count = max(1, math.ceil(distance / (self.grid.resolution * 0.5)))
        if count > 1_000_000:
            raise ValueError("segment exceeds collision-check resource bound")
        penalty = distance / count * 0.5
        for fraction in np.linspace(0.0, 1.0, count + 1):
            point = start + fraction * (end - start)
            if self.world_clearance(point) + 1e-12 < required + penalty:
                return False
        return True


def inflate_grid(grid, radius_m, margin_m=0.0):
    """Treat unknown, occupied, and outside-map cells as obstacles.

    Subtracting half a cell diagonal from center-to-center EDT distance gives
    a conservative distance to obstacle squares. Padding supplies outside-map
    obstacles even for an otherwise entirely free map. No map cells are edited.
    """
    if not isinstance(grid, Grid):
        raise ValueError("grid must be a Grid")
    radius = _number(radius_m, "radius", positive=True)
    margin = _number(margin_m, "margin", nonnegative=True)
    padded_free = np.pad(grid.data == 0, 1, mode="constant", constant_values=False)
    distances = distance_transform_edt(padded_free, sampling=grid.resolution)[1:-1, 1:-1]
    clearance = np.maximum(0.0, distances - grid.resolution / math.sqrt(2.0))
    blocked = (grid.data != 0) | (clearance + 1e-12 < radius + margin)
    clearance.setflags(write=False)
    blocked.setflags(write=False)
    return InflatedGrid(grid, blocked, clearance, radius, margin)


_NEIGHBORS = ((1, 0), (0, 1), (-1, 0), (0, -1),
              (1, 1), (-1, 1), (-1, -1), (1, -1))


def _neighbors(free, col, row):
    height, width = free.shape
    for dx, dy in _NEIGHBORS:
        x, y = col + dx, row + dy
        if 0 <= x < width and 0 <= y < height and free[y, x]:
            if dx and dy and not (free[row, x] and free[y, col]):
                continue
            yield x, y


def connected_components(inflated):
    """Return deterministic components of ``(column,row)`` pairs, largest first."""
    if not isinstance(inflated, InflatedGrid):
        raise ValueError("inflated must be an InflatedGrid")
    free = inflated.free
    visited = np.zeros(free.shape, dtype=bool)
    components = []
    for row, col in np.argwhere(free):
        if visited[row, col]:
            continue
        seed = (int(col), int(row))
        pending = deque([seed])
        visited[row, col] = True
        cells = []
        while pending:
            current = pending.popleft()
            cells.append(current)
            for x, y in _neighbors(free, *current):
                if not visited[y, x]:
                    visited[y, x] = True
                    pending.append((x, y))
        components.append(cells)
    # Stable sort preserves row-major tie-breaking between equal components.
    return sorted(components, key=len, reverse=True)


def _farthest(free, start):
    distances = np.full(free.shape, -1, dtype=np.int32)
    distances[start[1], start[0]] = 0
    pending = deque([start])
    farthest = start
    while pending:
        current = pending.popleft()
        if distances[current[1], current[0]] > distances[farthest[1], farthest[0]]:
            farthest = current
        for x, y in _neighbors(free, *current):
            if distances[y, x] < 0:
                distances[y, x] = distances[current[1], current[0]] + 1
                pending.append((x, y))
    return farthest


def demo_endpoints(inflated):
    """Choose two demonstration points, not an estimate of the actual robot pose."""
    components = connected_components(inflated)
    if not components or len(components[0]) < 2:
        raise ValueError("demonstration needs at least two traversable connected cells")
    free = inflated.free
    start = _farthest(free, components[0][0])
    goal = _farthest(free, start)
    return inflated.grid.cell_to_world(start), inflated.grid.cell_to_world(goal)


def _validated_path(world_points):
    try:
        points = np.asarray([_point(point, "path point") for point in world_points], dtype=float)
    except TypeError as exc:
        raise ValueError("path must contain finite x/y points") from exc
    if points.ndim != 2 or points.shape[1] != 2 or not 1 <= len(points) <= 200_000:
        raise ValueError("path must contain one to 200000 finite x/y points")
    if len(points) > 1:
        with np.errstate(over="ignore", invalid="ignore"):
            difference = np.diff(points, axis=0)
        if not np.isfinite(difference).all():
            raise ValueError("path differences are outside the finite numeric range")
        points = points[np.r_[True, np.hypot(difference[:, 0], difference[:, 1]) > 1e-10]]
    return points


def _approach(current, target, increment):
    return current + max(-increment, min(increment, target - current))


def _target(points, cumulative, position, progress, lookahead):
    vectors = np.diff(points, axis=0)
    lengths = np.diff(cumulative)
    fractions = np.clip(np.einsum("ij,ij->i", position - points[:-1], vectors) / (lengths ** 2), 0, 1)
    projections = points[:-1] + fractions[:, None] * vectors
    nearest = int(np.argmin(np.linalg.norm(projections - position, axis=1)))
    progress = max(progress, float(cumulative[nearest] + fractions[nearest] * lengths[nearest]))
    target_distance = min(float(cumulative[-1]), progress + lookahead)
    index = min(len(lengths) - 1, int(np.searchsorted(cumulative, target_distance, side="right")) - 1)
    fraction = (target_distance - cumulative[index]) / lengths[index]
    return points[index] + fraction * vectors[index], progress


def simulate_path(inflated, world_points, *, start_yaw=None, max_duration_s=180.0,
                  dt=0.05, max_linear_speed=0.25, max_angular_speed=0.6,
                  linear_acceleration=0.25, angular_acceleration=1.0,
                  goal_tolerance=0.12, lookahead_m=0.30):
    """Numerically follow a world-coordinate path with bounded acceleration.

    Returns ``REACHED``, ``BLOCKED`` or ``TIMEOUT``. Poses are
    ``[time_s,x_m,y_m,yaw_rad,v_mps,w_radps]``. A blocked step is never appended
    to the trajectory. The simulation has no hardware execution interface.
    """
    if not isinstance(inflated, InflatedGrid):
        raise ValueError("inflated must be an InflatedGrid")
    points = _validated_path(world_points)
    settings = {name: _number(value, name, positive=True) for name, value in {
        "max_duration_s": max_duration_s, "dt": dt,
        "max_linear_speed": max_linear_speed, "max_angular_speed": max_angular_speed,
        "linear_acceleration": linear_acceleration, "angular_acceleration": angular_acceleration,
        "goal_tolerance": goal_tolerance, "lookahead_m": lookahead_m}.items()}
    if settings["dt"] > 0.5 or math.ceil(settings["max_duration_s"] / settings["dt"]) > 200_000:
        raise ValueError("simulation time step or resource budget is invalid")
    dt = settings["dt"]
    if start_yaw is None:
        direction = points[1] - points[0] if len(points) > 1 else np.array([1.0, 0.0])
        yaw = math.atan2(direction[1], direction[0])
    else:
        yaw = _angle(_number(start_yaw, "start yaw"))
    position = points[0].copy()
    poses = [[0.0, float(position[0]), float(position[1]), yaw, 0.0, 0.0]]
    cumulative = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(points, axis=0), axis=1))]
    result = {
        "status": None, "reason": None, "poses": poses,
        "footprint": {"model": "circle", "radius_m": inflated.radius_m,
                      "margin_m": inflated.margin_m, "source": "EXPLICIT_SIMULATION_ASSUMPTION"},
        "settings": settings, "hardware_started": False, "navigation_validated": False,
        "path_length_m": float(cumulative[-1]), "simulated_distance_m": 0.0,
        "collision_traversals": 0, "blocked_step_rejected": False,
    }

    def finish(status, reason):
        result.update(status=status, reason=reason, elapsed_s=poses[-1][0],
                      final_goal_distance_m=float(np.linalg.norm(position - points[-1])),
                      minimum_clearance_m=min(inflated.world_clearance(row[1:3]) for row in poses))
        return result

    if not inflated.is_free_world(position):
        return finish("BLOCKED", "START_FOOTPRINT_BLOCKED")
    if any(not inflated.segment_is_free(a, b) for a, b in zip(points[:-1], points[1:])):
        return finish("BLOCKED", "PATH_FOOTPRINT_BLOCKED")
    if len(points) == 1:
        return finish("REACHED", "GOAL_REACHED")
    velocity = angular = progress = 0.0
    elapsed = 0.0
    while elapsed < settings["max_duration_s"] - 1e-12:
        step = min(dt, settings["max_duration_s"] - elapsed)
        distance_to_goal = float(np.linalg.norm(points[-1] - position))
        if distance_to_goal <= settings["goal_tolerance"] and abs(velocity) < 1e-8 and abs(angular) < 1e-8:
            return finish("REACHED", "GOAL_REACHED")
        target, progress = _target(points, cumulative, position, progress, settings["lookahead_m"])
        delta = target - position
        error = _angle(math.atan2(delta[1], delta[0]) - yaw)
        if distance_to_goal <= settings["goal_tolerance"]:
            desired_v = desired_w = 0.0
        else:
            desired_w = max(-settings["max_angular_speed"], min(settings["max_angular_speed"], 2.0 * error))
            braking_speed = math.sqrt(2 * settings["linear_acceleration"] * max(
                0.0, distance_to_goal - settings["goal_tolerance"] * 0.5))
            desired_v = min(settings["max_linear_speed"], braking_speed)
            desired_v *= max(0.0, math.cos(error)) if abs(error) <= 0.6 else 0.0
        next_v = _approach(velocity, desired_v, settings["linear_acceleration"] * step)
        next_w = _approach(angular, desired_w, settings["angular_acceleration"] * step)
        # Use midpoint speeds, so finite acceleration also holds during startup
        # and braking rather than instantly applying the next endpoint speed.
        v, w = (velocity + next_v) * 0.5, (angular + next_w) * 0.5
        next_yaw = yaw + w * step
        if abs(w) < 1e-10:
            next_position = position + v * step * np.array([math.cos(yaw), math.sin(yaw)])
        else:
            next_position = position + (v / w) * np.array([
                math.sin(next_yaw) - math.sin(yaw), math.cos(yaw) - math.cos(next_yaw)])
        arc_bound = abs(v * w) * step * step / 8.0
        if not inflated.segment_is_free(position, next_position, extra_margin_m=arc_bound):
            result["blocked_step_rejected"] = True
            return finish("BLOCKED", "SIMULATED_SWEEP_BLOCKED")
        result["simulated_distance_m"] += float(np.linalg.norm(next_position - position))
        position, yaw = next_position, _angle(next_yaw)
        velocity, angular = next_v, next_w
        elapsed += step
        poses.append([float(elapsed), float(position[0]), float(position[1]), yaw, velocity, angular])
    return finish("TIMEOUT", "DURATION_LIMIT")
