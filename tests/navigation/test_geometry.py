"""Geometry and closed-loop numerical checks; no ROS or hardware imports."""
import math

import numpy as np
import pytest

from wc_navigation.geometry import (Grid, connected_components, demo_endpoints,
                                    inflate_grid, simulate_path)


def open_grid(size=80, resolution=0.05, origin=(0.0, 0.0, 0.0)):
    return Grid(np.zeros((size, size), dtype=np.int8), resolution, origin)


def test_rotated_origin_cell_centers_round_trip_and_bottom_up_rows():
    grid = Grid(np.array([[0, 100, -1], [0, 0, 0]], dtype=np.int8), 0.2,
                (10.0, -3.0, math.pi / 2))
    assert grid.cell_to_world((0, 0)) == pytest.approx((9.9, -2.9))
    for row in range(grid.height):
        for col in range(grid.width):
            assert grid.world_to_cell(grid.cell_to_world((col, row))) == (col, row)
    assert grid.data[0, 1] == 100 and grid.data[0, 2] == -1
    with pytest.raises(ValueError, match="outside"):
        grid.world_to_cell((11.0, -3.0))


@pytest.mark.parametrize("values", [[], [[0.0]], [[True]], [[1]], [[float("nan")]], [0, 0]])
def test_malformed_occupancy_is_rejected(values):
    with pytest.raises(ValueError):
        Grid(values, 0.1)


@pytest.mark.parametrize("resolution", [-1, 0, float("nan"), float("inf"), True, "0.1"])
def test_invalid_metric_resolution_is_rejected(resolution):
    with pytest.raises(ValueError):
        Grid([[0]], resolution)


@pytest.mark.parametrize("radius,margin", [(-0.1, 0), (0, 0), (float("nan"), 0),
                                          (0.2, -1), (0.2, float("inf")), (True, 0)])
def test_invalid_footprint_is_rejected(radius, margin):
    with pytest.raises(ValueError):
        inflate_grid(open_grid(), radius, margin)


def test_unknown_and_outside_are_obstacles_without_editing_map():
    data = np.zeros((40, 40), dtype=np.int8)
    data[:, 20] = -1
    grid = Grid(data, 0.1)
    original = grid.data.copy()
    field = inflate_grid(grid, 0.2, 0.05)
    assert field.blocked[:, 20].all()
    assert not field.is_free_world((-0.01, 1.0))
    assert not field.is_free_world(grid.cell_to_world((0, 10)))
    assert field.is_free_world(grid.cell_to_world((10, 10)))
    assert not field.segment_is_free(grid.cell_to_world((10, 10)), grid.cell_to_world((30, 10)))
    np.testing.assert_array_equal(grid.data, original)


def test_footprint_inflation_changes_narrow_corridor_without_shrinking_robot():
    data = np.full((30, 50), 100, dtype=np.int8)
    data[10:19, 2:48] = 0  # Nine cells = 0.9 m; centers have bounded clearance.
    grid = Grid(data, 0.1)
    center = grid.cell_to_world((25, 14))
    small = inflate_grid(grid, 0.30, 0.02)
    large = inflate_grid(grid, 0.43, 0.02)
    assert small.is_free_world(center)
    assert not large.is_free_world(center)
    assert large.radius_m == 0.43 and large.margin_m == 0.02


def test_components_do_not_connect_diagonal_corner_touching_cells():
    grid = Grid(np.array([[0, 100], [100, 0]], dtype=np.int8), 1.0)
    field = inflate_grid(grid, 0.1)
    components = connected_components(field)
    assert [len(c) for c in components] == [1, 1]
    with pytest.raises(ValueError, match="at least two"):
        demo_endpoints(field)


def test_demo_endpoints_are_deterministic_and_use_largest_component():
    data = np.full((30, 40), 100, dtype=np.int8)
    data[2:7, 2:7] = 0
    data[10:27, 12:37] = 0
    field = inflate_grid(Grid(data, 0.1), 0.08)
    first = demo_endpoints(field)
    assert first == demo_endpoints(field)
    largest = set(connected_components(field)[0])
    assert all(field.grid.world_to_cell(p) in largest for p in first)
    assert first[0] != first[1]


def test_swept_circle_rejects_obstacle_between_clear_endpoints():
    data = np.zeros((60, 60), dtype=np.int8)
    data[30, 30] = 100
    field = inflate_grid(Grid(data, 0.1), 0.1)
    start, end = (2.0, 3.05), (4.0, 3.05)
    assert field.is_free_world(start) and field.is_free_world(end)
    assert not field.segment_is_free(start, end)


def test_straight_simulation_reaches_and_obeys_rate_acceleration_limits():
    field = inflate_grid(open_grid(), 0.25, 0.05)
    result = simulate_path(field, [(1.0, 1.0), (3.0, 1.0)], start_yaw=math.pi / 2,
                           max_duration_s=40.0)
    assert result["status"] == "REACHED"
    assert result["final_goal_distance_m"] <= 0.12
    assert not result["hardware_started"] and not result["navigation_validated"]
    assert result["collision_traversals"] == 0
    poses = np.array(result["poses"])
    settings = result["settings"]
    intervals = np.diff(poses[:, 0])
    assert np.abs(poses[:, 4]).max() <= settings["max_linear_speed"] + 1e-12
    assert np.abs(poses[:, 5]).max() <= settings["max_angular_speed"] + 1e-12
    assert np.all(np.abs(np.diff(poses[:, 4])) <= settings["linear_acceleration"] * intervals + 1e-10)
    assert np.all(np.abs(np.diff(poses[:, 5])) <= settings["angular_acceleration"] * intervals + 1e-10)
    assert np.abs(poses[-1, 4:6]).max() < 1e-8
    assert all(field.is_free_world(p[1:3]) for p in poses)
    assert any(abs(p[5]) > 0 and p[4] == 0 for p in poses)


def test_simulation_follows_a_bend_in_a_free_room():
    field = inflate_grid(open_grid(), 0.25, 0.05)
    result = simulate_path(field, [(1.0, 1.0), (2.5, 1.0), (2.5, 2.7)],
                           max_duration_s=40.0)
    assert result["status"] == "REACHED"
    assert result["final_goal_distance_m"] <= 0.12
    assert result["simulated_distance_m"] > 2.5


def test_wide_synthetic_l_corridor_supports_explicit_large_assumed_envelope():
    data = np.full((200, 240), 100, dtype=np.int8)  # Explicit 12 m by 10 m synthetic map.
    data[20:70, 20:220] = 0
    data[20:180, 170:220] = 0
    field = inflate_grid(Grid(data, 0.05), 0.55, 0.10)
    points = [(2.5, 2.25), (9.75, 2.25), (9.75, 7.5)]
    assert not field.segment_is_free(points[0], points[-1])
    result = simulate_path(field, points, max_duration_s=100.0)
    assert result["status"] == "REACHED"
    assert result["simulated_distance_m"] > 11.0
    assert result["footprint"]["radius_m"] == 0.55


def test_controller_corner_cutting_rejects_step_without_appending_collision():
    data = np.full((160, 160), 100, dtype=np.int8)
    data[30:51, 20:121] = 0
    data[30:131, 100:121] = 0
    field = inflate_grid(Grid(data, 0.05), 0.2, 0.03)
    result = simulate_path(field, [(1.5, 2.0), (5.5, 2.0), (5.5, 6.0)],
                           lookahead_m=1.5, max_linear_speed=0.5,
                           max_angular_speed=0.5, max_duration_s=50.0)
    assert result["status"] == "BLOCKED"
    assert result["reason"] == "SIMULATED_SWEEP_BLOCKED"
    assert result["blocked_step_rejected"]
    assert len(result["poses"]) > 10
    assert result["collision_traversals"] == 0
    assert all(field.is_free_world(p[1:3]) for p in result["poses"])


def test_blocked_path_does_not_advance_or_cross_obstacle():
    data = np.zeros((80, 80), dtype=np.int8)
    data[:, 40] = 100
    field = inflate_grid(Grid(data, 0.05), 0.25, 0.05)
    result = simulate_path(field, [(1.0, 2.0), (3.0, 2.0)])
    assert result["status"] == "BLOCKED"
    assert result["reason"] == "PATH_FOOTPRINT_BLOCKED"
    assert len(result["poses"]) == 1 and result["simulated_distance_m"] == 0


def test_finite_outside_start_is_blocked_and_short_budget_times_out():
    field = inflate_grid(open_grid(), 0.25, 0.05)
    result = simulate_path(field, [(-1.0, 1.0), (2.0, 1.0)])
    assert result["status"] == "BLOCKED" and len(result["poses"]) == 1
    result = simulate_path(field, [(1.0, 1.0), (3.0, 1.0)], max_duration_s=0.15)
    assert result["status"] == "TIMEOUT"
    assert result["elapsed_s"] == pytest.approx(0.15)


@pytest.mark.parametrize("points", [[], [[1, float("nan")]], [[1]], [[1, 2, 3]], [["1", 2]], [[True, 2]]])
def test_bad_world_path_points_are_rejected(points):
    with pytest.raises(ValueError):
        simulate_path(inflate_grid(open_grid(), 0.2), points)


def test_single_goal_and_duplicate_waypoints_are_supported():
    field = inflate_grid(open_grid(), 0.2)
    result = simulate_path(field, [(1.0, 1.0), (1.0, 1.0)])
    assert result["status"] == "REACHED" and len(result["poses"]) == 1


@pytest.mark.parametrize("settings", [{"dt": 0}, {"dt": 1.0}, {"max_duration_s": float("nan")},
                                      {"linear_acceleration": -0.2}, {"start_yaw": float("nan")},
                                      {"max_angular_speed": True}])
def test_invalid_simulation_settings_are_rejected(settings):
    with pytest.raises(ValueError):
        simulate_path(inflate_grid(open_grid(), 0.2), [(1.0, 1.0), (2.0, 1.0)], **settings)


def test_extreme_finite_coordinates_fail_as_value_errors_before_indexing():
    with pytest.raises(ValueError, match="extent"):
        Grid(np.zeros((2, 2), dtype=np.int8), 1e308)
    grid = Grid(np.zeros((2, 2), dtype=np.int8), 0.1, (-1e308, 0.0, 0.0))
    with pytest.raises(ValueError, match="finite"):
        grid.world_to_cell((1e308, 0.0))
    field = inflate_grid(open_grid(), 0.2)
    with pytest.raises(ValueError, match="finite"):
        field.segment_is_free((-1e308, 0.0), (1e308, 0.0))
