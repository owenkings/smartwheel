"""Frontier detection and conservative goal selection."""
import math

import numpy as np
import pytest

from wc_navigation.autonomy.frontier import (find_frontiers, reachable_distances, select_goal, trinary,
                                             with_robot_disc_free)
from wc_navigation.geometry import Grid


def corridor(known_until_m=3.0, length_m=8.0, width_m=2.4, resolution=0.05):
    """Free corridor along x with walls; cells beyond ``known_until_m`` unknown."""
    rows, cols = int(round((width_m + 0.4) / resolution)), int(round(length_m / resolution))
    data = np.zeros((rows, cols), dtype=np.int8)
    data[:4, :] = data[-4:, :] = 100
    data[:, :4] = 100
    data[:, int(round(known_until_m / resolution)):] = -1
    return Grid(data, resolution, (0.0, 0.0, 0.0))


def test_trinary_conversion_keeps_uncertain_values_unknown():
    assert trinary([[-1, 0, 25, 50, 65, 100]]).tolist() == [[-1, 0, 0, -1, 100, 100]]


def test_frontier_at_known_unknown_boundary():
    frontiers = find_frontiers(corridor(), min_length_m=0.5)
    assert len(frontiers) == 1
    assert frontiers[0].centroid_xy[0] == pytest.approx(3.0 - 0.025, abs=0.03)
    assert frontiers[0].length_m == pytest.approx(2.4, abs=0.1)


def test_small_frontiers_are_ignored():
    grid = corridor()
    data = np.array(grid.data)
    data[:, 60:] = 100  # close the corridor end: no frontier
    data[30, 30] = -1   # single unknown cell inside free space
    assert find_frontiers(Grid(data, 0.05, grid.origin), min_length_m=0.5) == []


def test_goal_is_reachable_known_free_with_clearance_and_faces_frontier():
    grid = corridor()
    candidate, report = select_goal(grid, (1.0, 1.4), radius_m=0.55, margin_m=0.10)
    assert candidate is not None and report["candidates"] == 1
    gx, gy = candidate.goal_xy
    assert gx < 3.0 - 0.65 + 1e-6  # stays clear of unknown space by the stated clearance
    assert abs(gy - 1.4) < 0.5
    assert abs(candidate.goal_yaw) < math.radians(30)
    assert candidate.path_distance_m == pytest.approx(gx - 1.0, abs=0.2)


def test_goal_prefers_extra_clearance_and_falls_back_in_narrow_space():
    wide = corridor()
    candidate, report = select_goal(wide, (1.0, 1.4), radius_m=0.55, margin_m=0.10, goal_extra_clearance_m=0.15)
    assert candidate.goal_xy[0] <= 3.0 - 0.80 + 1e-6 and report["fallback_goals_at_bare_clearance"] == 0
    narrow = corridor(width_m=1.45)  # centre clearance about 0.72 m: only the bare 0.65 m is available
    candidate, report = select_goal(narrow, (1.0, 0.925), radius_m=0.55, margin_m=0.10, goal_extra_clearance_m=0.15)
    assert candidate is not None and report["fallback_goals_at_bare_clearance"] == 1


def test_frontier_behind_a_wall_is_skipped():
    grid = corridor()
    data = np.array(grid.data)
    data[:, 40:42] = 100  # wall across the corridor at x = 2.0 m
    candidate, report = select_goal(Grid(data, 0.05, grid.origin), (1.0, 1.4), radius_m=0.55, margin_m=0.10)
    assert candidate is None and report["skipped_unreachable"] == 1


def test_robot_near_a_wall_starts_search_from_nearest_clear_cell():
    grid = corridor()
    candidate, report = select_goal(grid, (0.80, 1.4), radius_m=0.55, margin_m=0.10)
    assert candidate is not None and 0 < report["start_cell_snapped_m"] <= 0.6


def test_blacklisted_frontier_is_skipped():
    grid = corridor()
    first, _ = select_goal(grid, (1.0, 1.4), radius_m=0.55, margin_m=0.10)
    again, report = select_goal(grid, (1.0, 1.4), radius_m=0.55, margin_m=0.10,
                                blacklist=[(*first.frontier_centroid_xy, 1.0)])
    assert again is None and report["skipped_blacklisted"] == 1


def test_robot_disc_counts_as_free_but_occupied_cells_stay():
    data = np.full((40, 40), -1, dtype=np.int8)
    data[25:30, 20] = 100
    grid = with_robot_disc_free(Grid(data, 0.05, (0.0, 0.0, 0.0)), (1.0, 1.0), 0.4)
    assert grid.data[20, 20] == 0
    assert (grid.data[25:30, 20] == 100).all()
    assert grid.data[0, 0] == -1


def test_reachable_distances_do_not_cut_corners():
    free = np.ones((3, 3), dtype=bool)
    free[0, 1] = free[1, 0] = False
    distances = reachable_distances(free, (0, 0), 1.0)
    assert np.isinf(distances[1, 1])  # diagonal blocked by both side cells
    free[1, 0] = True
    distances = reachable_distances(free, (0, 0), 1.0)
    assert distances[1, 1] == pytest.approx(2.0)
    assert np.isinf(reachable_distances(free, (1, 0), 1.0)).all()  # start cell blocked


def test_no_frontier_means_exploration_complete():
    grid = corridor(known_until_m=8.0)
    data = np.array(grid.data)
    data[:, -4:] = 100
    candidate, report = select_goal(Grid(data, 0.05, grid.origin), (1.0, 1.4), radius_m=0.55, margin_m=0.10)
    assert candidate is None and report["frontiers"] == 0
