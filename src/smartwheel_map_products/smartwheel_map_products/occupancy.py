from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class OccupancyGridData:
    cells: np.ndarray
    resolution: float
    origin_x: float
    origin_y: float

    @property
    def width(self) -> int:
        return int(self.cells.shape[1])

    @property
    def height(self) -> int:
        return int(self.cells.shape[0])


def bresenham(x0: int, y0: int, x1: int, y1: int):
    dx = abs(x1 - x0)
    sx = 1 if x0 < x1 else -1
    dy = -abs(y1 - y0)
    sy = 1 if y0 < y1 else -1
    error = dx + dy
    while True:
        yield x0, y0
        if x0 == x1 and y0 == y1:
            break
        twice = 2 * error
        if twice >= dy:
            error += dy
            x0 += sx
        if twice <= dx:
            error += dx
            y0 += sy


def raycast_occupancy(
    endpoints: np.ndarray,
    sensor_origins: np.ndarray,
    resolution: float = 0.05,
    min_obstacle_z: float = 0.1,
    max_obstacle_z: float = 2.2,
    padding_m: float = 1.0,
) -> OccupancyGridData:
    points = np.asarray(endpoints, dtype=np.float64)
    origins = np.asarray(sensor_origins, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("endpoints must have shape (N, 3)")
    if origins.shape != points.shape:
        raise ValueError("sensor_origins must match endpoints")
    if resolution <= 0.0 or padding_m < 0.0:
        raise ValueError("invalid occupancy grid dimensions")
    valid = (
        np.all(np.isfinite(points), axis=1)
        & np.all(np.isfinite(origins), axis=1)
        & (points[:, 2] >= min_obstacle_z)
        & (points[:, 2] <= max_obstacle_z)
    )
    points = points[valid]
    origins = origins[valid]
    if points.shape[0] == 0:
        raise ValueError("no valid obstacle endpoints for occupancy grid")
    all_xy = np.vstack((points[:, :2], origins[:, :2]))
    minimum = np.floor((all_xy.min(axis=0) - padding_m) / resolution) * resolution
    maximum = np.ceil((all_xy.max(axis=0) + padding_m) / resolution) * resolution
    width, height = np.ceil((maximum - minimum) / resolution).astype(int) + 1
    cells = np.full((int(height), int(width)), -1, dtype=np.int8)

    def index(xy):
        return np.floor((xy - minimum) / resolution + 1e-9).astype(int)

    endpoint_cells = index(points[:, :2])
    origin_cells = index(origins[:, :2])
    ray_cells = np.column_stack((origin_cells, endpoint_cells))
    ray_cells = np.unique(ray_cells, axis=0)
    for ox, oy, ex, ey in ray_cells:
        ray = list(bresenham(int(ox), int(oy), int(ex), int(ey)))
        for x, y in ray[:-1]:
            if 0 <= x < width and 0 <= y < height and cells[y, x] != 100:
                cells[y, x] = 0
        if 0 <= ex < width and 0 <= ey < height:
            cells[ey, ex] = 100
    return OccupancyGridData(cells, float(resolution), float(minimum[0]), float(minimum[1]))
