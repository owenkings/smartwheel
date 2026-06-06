"""Pure coordinate conversions shared by tests and map UI design."""

from typing import Any, Dict, Iterable, List, Sequence, Tuple


Point = Tuple[float, float]


def _map_geometry(map_info: Dict[str, Any]) -> Tuple[int, int, float, float, float]:
    width = int(map_info["width"])
    height = int(map_info["height"])
    resolution = float(map_info["resolution"])
    origin = map_info.get("origin") or {}
    origin_x = float(origin.get("x", 0.0))
    origin_y = float(origin.get("y", 0.0))
    if width <= 0 or height <= 0 or resolution <= 0.0:
        raise ValueError("map width, height, and resolution must be positive")
    return width, height, resolution, origin_x, origin_y


def map_to_canvas(
    x: float,
    y: float,
    map_info: Dict[str, Any],
    canvas_width: float,
    canvas_height: float,
    scale: float,
    offset_x: float,
    offset_y: float,
) -> Point:
    """Convert ROS map coordinates to Canvas coordinates with a flipped y axis."""
    width, height, resolution, origin_x, origin_y = _map_geometry(map_info)
    if canvas_width <= 0 or canvas_height <= 0 or scale <= 0:
        raise ValueError("canvas dimensions and scale must be positive")
    map_x = (float(x) - origin_x) / resolution
    map_y = (float(y) - origin_y) / resolution
    return (
        float(offset_x) + map_x * float(scale),
        float(offset_y) + (height - map_y) * float(scale),
    )


def canvas_to_map(
    px: float,
    py: float,
    map_info: Dict[str, Any],
    canvas_width: float,
    canvas_height: float,
    scale: float,
    offset_x: float,
    offset_y: float,
) -> Point:
    """Convert Canvas coordinates back to continuous ROS map coordinates."""
    _, height, resolution, origin_x, origin_y = _map_geometry(map_info)
    if canvas_width <= 0 or canvas_height <= 0 or scale <= 0:
        raise ValueError("canvas dimensions and scale must be positive")
    map_x = (float(px) - float(offset_x)) / float(scale)
    map_y = height - (float(py) - float(offset_y)) / float(scale)
    return (
        origin_x + map_x * resolution,
        origin_y + map_y * resolution,
    )


def map_point_in_bounds(x: float, y: float, map_info: Dict[str, Any]) -> bool:
    width, height, resolution, origin_x, origin_y = _map_geometry(map_info)
    return (
        origin_x <= float(x) < origin_x + width * resolution
        and origin_y <= float(y) < origin_y + height * resolution
    )


def path_to_canvas_polyline(
    points: Iterable[Sequence[float]],
    map_info: Dict[str, Any],
    canvas_width: float,
    canvas_height: float,
    scale: float,
    offset_x: float,
    offset_y: float,
) -> List[Point]:
    return [
        map_to_canvas(
            float(point[0]),
            float(point[1]),
            map_info,
            canvas_width,
            canvas_height,
            scale,
            offset_x,
            offset_y,
        )
        for point in points
    ]
