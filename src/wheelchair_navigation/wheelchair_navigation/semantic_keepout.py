import math
from dataclasses import dataclass
from typing import Iterable, List, Sequence


@dataclass(frozen=True)
class GridGeometry:
    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    origin_yaw: float = 0.0


def world_to_grid(x: float, y: float, geometry: GridGeometry):
    dx = float(x) - geometry.origin_x
    dy = float(y) - geometry.origin_y
    cosine = math.cos(geometry.origin_yaw)
    sine = math.sin(geometry.origin_yaw)
    local_x = cosine * dx + sine * dy
    local_y = -sine * dx + cosine * dy
    return local_x / geometry.resolution, local_y / geometry.resolution


def point_in_polygon(x: float, y: float, polygon: Sequence[Sequence[float]]) -> bool:
    inside = False
    previous = polygon[-1]
    for current in polygon:
        x1, y1 = float(previous[0]), float(previous[1])
        x2, y2 = float(current[0]), float(current[1])
        if (y1 > y) != (y2 > y):
            crossing_x = (x2 - x1) * (y - y1) / (y2 - y1) + x1
            if x < crossing_x:
                inside = not inside
        previous = current
    return inside


def rasterize_keepout_zones(
    zones: Iterable[dict],
    geometry: GridGeometry,
    occupied_value: int = 100,
) -> List[int]:
    if geometry.width <= 0 or geometry.height <= 0 or geometry.resolution <= 0:
        raise ValueError("grid dimensions and resolution must be positive")

    data = [0] * (geometry.width * geometry.height)
    for zone in zones:
        polygon = zone.get("polygon", [])
        if len(polygon) < 3:
            continue
        grid_polygon = [world_to_grid(point[0], point[1], geometry) for point in polygon]
        min_x = max(0, int(math.floor(min(point[0] for point in grid_polygon))))
        max_x = min(
            geometry.width - 1,
            int(math.ceil(max(point[0] for point in grid_polygon))),
        )
        min_y = max(0, int(math.floor(min(point[1] for point in grid_polygon))))
        max_y = min(
            geometry.height - 1,
            int(math.ceil(max(point[1] for point in grid_polygon))),
        )
        if min_x > max_x or min_y > max_y:
            continue
        for row in range(min_y, max_y + 1):
            for column in range(min_x, max_x + 1):
                if point_in_polygon(column + 0.5, row + 0.5, grid_polygon):
                    data[row * geometry.width + column] = int(occupied_value)
    return data
