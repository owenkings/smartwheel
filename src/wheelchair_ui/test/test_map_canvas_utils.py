import math

from wheelchair_ui.map_canvas_utils import (
    canvas_to_map,
    map_point_in_bounds,
    map_to_canvas,
    path_to_canvas_polyline,
)


def test_origin_zero_resolution_one_maps_to_canvas_edges():
    info = {"width": 10, "height": 8, "resolution": 1.0, "origin": {"x": 0, "y": 0}}

    assert map_to_canvas(0, 0, info, 100, 80, 10, 0, 0) == (0.0, 80.0)
    assert canvas_to_map(0, 80, info, 100, 80, 10, 0, 0) == (0.0, 0.0)


def test_negative_origin_round_trip_is_better_than_half_cell():
    info = {
        "width": 200,
        "height": 160,
        "resolution": 0.05,
        "origin": {"x": -5, "y": -5},
    }
    source = (1.23, -0.87)
    pixel = map_to_canvas(*source, info, 1000, 800, 5, 0, 0)
    result = canvas_to_map(*pixel, info, 1000, 800, 5, 0, 0)

    assert math.dist(source, result) < info["resolution"] / 2


def test_canvas_y_axis_is_opposite_map_y_axis():
    info = {"width": 10, "height": 10, "resolution": 1.0, "origin": {"x": 0, "y": 0}}

    lower = map_to_canvas(2, 2, info, 100, 100, 10, 0, 0)
    upper = map_to_canvas(2, 7, info, 100, 100, 10, 0, 0)

    assert upper[1] < lower[1]


def test_map_bounds_exclude_outer_maximum_edges():
    info = {"width": 4, "height": 3, "resolution": 0.5, "origin": {"x": -1, "y": -2}}

    assert map_point_in_bounds(-1, -2, info)
    assert map_point_in_bounds(0.99, -0.51, info)
    assert not map_point_in_bounds(-1.01, -2, info)
    assert not map_point_in_bounds(1.0, -2, info)
    assert not map_point_in_bounds(-1, -0.5, info)


def test_route_path_converts_every_point_in_order():
    info = {"width": 10, "height": 10, "resolution": 1.0, "origin": {"x": 0, "y": 0}}

    polyline = path_to_canvas_polyline(
        [(0, 0), (2, 3), (5, 8)],
        info,
        100,
        100,
        10,
        0,
        0,
    )

    assert polyline == [(0.0, 100.0), (20.0, 70.0), (50.0, 20.0)]
