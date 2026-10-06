"""Deterministic sparse occupancy reconstruction; never assumes a rig ray origin.

This is a conservative binary evidence grid, not OctoMap/log-odds. Occupied
evidence wins over free evidence within the full revision. Dynamic-object
removal is intentionally not claimed. Every call starts from the raw frames.
"""

from __future__ import annotations

import json
import math
import threading
from typing import Any

import numpy as np


class MapError(ValueError):
    """An input cannot produce a complete, trustworthy snapshot."""


def rigid(value: Any, label: str) -> np.ndarray:
    matrix = np.asarray(value, dtype=float)
    if matrix.shape != (4, 4) or not np.isfinite(matrix).all():
        raise MapError(f"{label}: expected a finite 4x4 transform")
    if not np.allclose(matrix[3], [0, 0, 0, 1], atol=1e-8, rtol=0):
        raise MapError(f"{label}: invalid homogeneous transform")
    rotation = matrix[:3, :3]
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6, rtol=0):
        raise MapError(f"{label}: rotation is not orthonormal")
    if not np.isclose(np.linalg.det(rotation), 1, atol=1e-6, rtol=0):
        raise MapError(f"{label}: rotation is not proper")
    return matrix


def integer(value: Any, label: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise MapError(f"{label}: expected a nonnegative integer")
    return value


def epoch(value: Any, label: str):
    """Preserve opaque ROS epoch UUIDs or explicit offline integer epochs."""
    if isinstance(value, str) and value:
        return value
    return integer(value, label)


def identity(value: Any, label: str) -> str:
    if isinstance(value, bool) or not isinstance(value, (str, int)) or str(value) == "":
        raise MapError(f"{label}: expected a nonempty string or integer")
    # Preserve the distinction between numeric and string node IDs.
    return json.dumps(value, ensure_ascii=False, sort_keys=True)


def raw_identity(value: Any) -> str:
    if isinstance(value, str) and value:
        return value
    if isinstance(value, dict) and value:
        return json.dumps(value, ensure_ascii=False, sort_keys=True, allow_nan=False)
    raise MapError("raw_key must be a nonempty string or structured key")


def _ray_cells(origin: np.ndarray, endpoint: np.ndarray, resolution: float,
               max_cells: int) -> list[tuple[int, int, int]]:
    """Amanatides-Woo traversal; endpoint voxel is never marked free.

    Simultaneous boundary crossings advance all tied axes. Cells touched only
    at an edge/corner receive no free evidence, avoiding a thick clearing ray.
    The sensor's own voxel is omitted because it is not fully observed.
    """
    # Keep IEEE float64 operation order equivalent to the original NumPy DDA,
    # but avoid allocating/masking three-element arrays at every voxel step.
    ox, oy, oz = float(origin[0]), float(origin[1]), float(origin[2])
    ex, ey, ez = float(endpoint[0]), float(endpoint[1]), float(endpoint[2])
    x, y, z = math.floor(ox / resolution), math.floor(oy / resolution), math.floor(oz / resolution)
    fx, fy, fz = math.floor(ex / resolution), math.floor(ey / resolution), math.floor(ez / resolution)
    count_bound = abs(fx - x) + abs(fy - y) + abs(fz - z) + 1
    if count_bound > max_cells:
        raise MapError("ray exceeds max_ray_cells resource bound")
    dx, dy, dz = ex - ox, ey - oy, ez - oz
    sx, sy, sz = (1 if dx > 0 else -1 if dx < 0 else 0), (1 if dy > 0 else -1 if dy < 0 else 0), (1 if dz > 0 else -1 if dz < 0 else 0)
    tx = ((x + (sx > 0)) * resolution - ox) / dx if sx else math.inf
    ty = ((y + (sy > 0)) * resolution - oy) / dy if sy else math.inf
    tz = ((z + (sz > 0)) * resolution - oz) / dz if sz else math.inf
    delta_x = resolution / abs(dx) if sx else math.inf
    delta_y = resolution / abs(dy) if sy else math.inf
    delta_z = resolution / abs(dz) if sz else math.inf
    if math.isnan(tx) or math.isnan(ty) or math.isnan(tz):
        raise MapError("ray traversal did not reach its endpoint")
    cells = []
    for _ in range(count_bound):
        if x == fx and y == fy and z == fz:
            return cells
        next_time = min(tx, ty, tz)
        # All comparisons use the SAME pre-step minimum. Equality also matches
        # np.isclose(inf, inf); the tolerance remains absolute, never relative.
        if tx == next_time or abs(tx - next_time) <= 1e-12:
            x += sx
            tx += delta_x
        if ty == next_time or abs(ty - next_time) <= 1e-12:
            y += sy
            ty += delta_y
        if tz == next_time or abs(tz - next_time) <= 1e-12:
            z += sz
            tz += delta_z
        if x != fx or y != fy or z != fz:
            cells.append((x, y, z))
    raise MapError("ray traversal did not reach its endpoint")


def build_map(graph: dict, observations: dict, config: dict) -> dict:
    """Build a complete graph revision from original sensor-frame endpoints.

    Required config: resolution, ground_z, ground_tolerance,
    collision_min_height, collision_max_height, ground_valid. Heights are
    relative to the horizontal ground_z plane, whose validity is supplied by
    the caller. Defaults are resource bounds, not physical calibration.
    """
    if not isinstance(graph.get("session_id"), str) or not graph["session_id"]:
        raise MapError("graph session_id is required")
    graph_epoch = epoch(graph.get("graph_epoch"), "graph_epoch")
    revision = integer(graph.get("revision"), "graph revision")
    required = ("resolution", "ground_z", "ground_tolerance", "collision_min_height", "collision_max_height")
    try:
        settings = {key: float(config[key]) for key in required}
    except (KeyError, TypeError, ValueError) as exc:
        raise MapError(f"missing/invalid physical map configuration: {exc}") from exc
    if not all(math.isfinite(v) for v in settings.values()):
        raise MapError("map configuration must be finite")
    resolution = settings["resolution"]
    tolerance = settings["ground_tolerance"]
    minimum, maximum = settings["collision_min_height"], settings["collision_max_height"]
    ground = settings["ground_z"]
    if resolution <= 0 or tolerance < 0 or minimum <= tolerance or maximum <= minimum:
        raise MapError("require resolution>0 and 0<=ground_tolerance<collision_min_height<collision_max_height")
    if not isinstance(config.get("ground_valid"), bool):
        raise MapError("ground_valid must be explicitly true or false")
    mode = config.get("sensor_mode", "dual")
    expected_sides = {"dual": {"left", "right"}, "single_left": {"left"}, "single_right": {"right"}}.get(mode)
    if expected_sides is None:
        raise MapError("invalid sensor_mode")
    max_ray_cells = integer(config.get("max_ray_cells", 10000), "max_ray_cells")
    max_grid_cells = integer(config.get("max_grid_cells", 4000000), "max_grid_cells")
    max_points = integer(config.get("max_points", 5000000), "max_points")
    max_voxels = integer(config.get("max_voxels", 4000000), "max_voxels")
    nodes = {}
    for node in graph.get("nodes", []):
        key = identity(node.get("node_id"), "node_id")
        if key in nodes:
            raise MapError("duplicate graph node ID")
        epoch(node.get("odom_epoch"), "node odom_epoch")
        nodes[key] = (node, rigid(node.get("T_map_node"), "T_map_node"))
    if not nodes:
        raise MapError("cannot build an empty graph")
    for edge in graph.get("edges", []):
        if identity(edge.get("from_id"), "edge from_id") not in nodes or identity(edge.get("to_id"), "edge to_id") not in nodes:
            raise MapError("graph edge references a missing node")
        if not isinstance(edge.get("type"), str) or not edge["type"]:
            raise MapError("edge type must retain its actual graph provenance")

    # Store the union during tracing: its cardinality is the global resource
    # count even when several rays/sensors repeat a voxel or occupied wins.
    all_cells: set[tuple[int, int, int]] = set()
    occupied: set[tuple[int, int, int]] = set()
    ground_support: set[tuple[int, int]] = set()
    seen_raw = set()
    mapped = {key: set() for key in nodes}
    sensor_sides: dict[str, str] = {}
    side_sensors: dict[str, str] = {}
    cloud = []
    origins = []
    raw_index = []
    counts = {"left": 0, "right": 0}
    for frame in observations.get("observations", []):
        raw_key = raw_identity(frame.get("raw_key"))
        if raw_key in seen_raw:
            raise MapError("raw frame reused; a raw_key may contribute only once")
        seen_raw.add(raw_key)
        key = identity(frame.get("node_id"), "observation node_id")
        if key not in nodes:
            raise MapError("observation references a missing graph node")
        node, T_map_node = nodes[key]
        if epoch(frame.get("odom_epoch"), "observation odom_epoch") != node["odom_epoch"]:
            raise MapError("observation crosses an odometry epoch")
        if frame.get("session_id", graph["session_id"]) != graph["session_id"] or frame.get("graph_epoch", graph_epoch) != graph_epoch:
            raise MapError("observation crosses session/graph epoch")
        side = frame.get("side")
        sensor = frame.get("sensor_id")
        if side not in expected_sides or not isinstance(sensor, str) or not sensor:
            raise MapError("observation sensor identity/side is invalid")
        if sensor in sensor_sides and sensor_sides[sensor] != side:
            raise MapError("same sensor identity used for both sides")
        if side in side_sensors and side_sensors[side] != sensor:
            raise MapError("sensor identity changed within one snapshot")
        sensor_sides[sensor], side_sensors[side] = side, sensor
        T_map_sensor = T_map_node @ rigid(frame.get("T_node_sensor"), "T_node_sensor")
        points = np.asarray(frame.get("points"), dtype=float)
        if points.ndim != 2 or points.shape[1] != 3 or not len(points) or not np.isfinite(points).all():
            raise MapError("raw points must be a nonempty finite Nx3 array; invalid/no-return data must be excluded explicitly")
        if len(cloud) + len(points) > max_points:
            raise MapError("snapshot exceeds max_points resource bound")
        # Coordinates that overflow voxel integer arithmetic are not valid input.
        transformed = points @ T_map_sensor[:3, :3].T + T_map_sensor[:3, 3]
        origin = T_map_sensor[:3, 3]
        if not np.isfinite(transformed).all() or np.max(np.abs(np.r_[transformed.ravel(), origin])) / resolution > 1e12:
            raise MapError("point coordinates exceed supported voxel range")
        mapped[key].add(side)
        counts[side] += len(points)
        for endpoint in transformed:
            if np.linalg.norm(endpoint - origin) <= 1e-10:
                raise MapError("zero-range return cannot be treated as an obstacle")
            voxel = tuple(int(v) for v in np.floor(endpoint / resolution))
            if len(all_cells) >= max_voxels and voxel not in all_cells:
                raise MapError("snapshot exceeds max_voxels resource bound")
            occupied.add(voxel)
            all_cells.add(voxel)
            ray_cells = _ray_cells(origin, endpoint, resolution, max_ray_cells)
            if len(all_cells) + len(ray_cells) <= max_voxels:
                all_cells.update(ray_cells)  # Fast path cannot exceed the bound.
            else:
                # Near the limit, account only new union members and reject
                # before growth. Repeated rays/occupied-free overlap are free.
                for ray_cell in ray_cells:
                    if ray_cell not in all_cells:
                        if len(all_cells) >= max_voxels:
                            raise MapError("snapshot exceeds max_voxels resource bound")
                        all_cells.add(ray_cell)
            if abs(float(endpoint[2]) - ground) <= tolerance:
                ground_support.add(voxel[:2])
        cloud.extend(transformed.tolist())
        origins.append({"raw_key": frame["raw_key"], "side": side, "origin_map": origin.tolist()})
        raw_index.append({"raw_key": frame["raw_key"], "session_id": graph["session_id"], "graph_epoch": graph_epoch,
                          "node_id": node["node_id"], "odom_epoch": node["odom_epoch"], "sensor_id": sensor,
                          "side": side, "point_count": len(points), "T_node_sensor": frame["T_node_sensor"]})
    for key, sides in mapped.items():
        if sides != expected_sides:
            raise MapError(f"graph node {key} lacks required raw observations: {sorted(expected_sides - sides)}")
    free = all_cells - occupied
    lower = [min(cell[axis] for cell in all_cells) for axis in (0, 1)]
    upper = [max(cell[axis] for cell in all_cells) for axis in (0, 1)]
    width, height = upper[0] - lower[0] + 1, upper[1] - lower[1] + 1
    if width * height > max_grid_cells:
        raise MapError("projected map exceeds max_grid_cells resource bound")
    z_start = math.floor((ground + minimum) / resolution)
    z_end = math.ceil((ground + maximum) / resolution)
    if z_end - z_start > max_ray_cells:
        raise MapError("collision interval exceeds resource bound")
    data = []
    for y in range(lower[1], upper[1] + 1):
        for x in range(lower[0], upper[0] + 1):
            vertical = [(x, y, z) for z in range(z_start, z_end)]
            if any(cell in occupied for cell in vertical):
                data.append(100)
            elif config["ground_valid"] and (x, y) in ground_support and all(cell in free for cell in vertical):
                data.append(0)
            else:
                data.append(-1)
    voxels = [{"index": list(cell), "state": 100 if cell in occupied else 0} for cell in sorted(all_cells)]
    return {"schema_version": 1, "session_id": graph["session_id"], "graph_epoch": graph_epoch, "revision": revision,
            "cloud": cloud, "occupancy": {"format": "sparse_binary_evidence_v1", "resolution": resolution, "voxels": voxels},
            "grid": {"resolution": resolution, "width": width, "height": height,
                     "origin": [lower[0] * resolution, lower[1] * resolution, 0.0], "data": data,
                     "row_order": "y_increasing_x_fastest", "inflated": False},
            "origins": origins, "raw_data_index": raw_index, "source_point_counts": counts,
            "sensor_ids": side_sensors, "sensor_mode": mode, "navigation_validated": False,
            "quality": {"ground_valid": config["ground_valid"], "dynamic_object_removal": "not_implemented",
                        "free_policy": "all_collision_voxels_free_and_same_column_ground_return",
                        "collision_min_height": minimum, "collision_max_height": maximum,
                        "ground_z": ground, "ground_tolerance": tolerance}}


class RevisionGuard:
    """Thread-safe commit guard for a caller-owned bounded rebuild worker.

    request() invalidates in-flight work. A failed/stale build leaves the last
    complete map readable, but status explicitly reports it as stale.
    """

    def __init__(self):
        self._lock = threading.Lock()
        self._generation = 0
        self._requested = None
        self._completed = None
        self._error = None

    def request(self, session_id: str, graph_epoch: int, revision: int) -> int:
        with self._lock:
            self._generation += 1
            self._requested = (session_id, graph_epoch, revision)
            self._error = None
            return self._generation

    def complete(self, token: int, result: dict) -> bool:
        with self._lock:
            if token != self._generation:
                return False
            signature = (result["session_id"], result["graph_epoch"], result["revision"])
            if signature != self._requested:
                raise MapError("completed map does not match requested graph revision")
            self._completed = result
            self._error = None
            return True

    def fail(self, token: int, reason: str) -> None:
        with self._lock:
            if token == self._generation:
                self._error = str(reason)

    def status(self) -> dict:
        with self._lock:
            completed = None if self._completed is None else (self._completed["session_id"], self._completed["graph_epoch"], self._completed["revision"])
            return {"requested": self._requested, "completed": completed,
                    "stale": self._requested != completed, "rebuilding": self._requested != completed and self._error is None,
                    "error": self._error, "snapshot": self._completed}
