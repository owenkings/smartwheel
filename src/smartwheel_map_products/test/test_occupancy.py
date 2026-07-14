import numpy as np

from smartwheel_map_products.occupancy import raycast_occupancy


def test_raycast_marks_free_occupied_and_unknown():
    endpoints = np.array([[2.0, 0.0, 0.5], [0.0, 2.0, 0.5]])
    origins = np.zeros_like(endpoints)
    grid = raycast_occupancy(endpoints, origins, resolution=0.5, padding_m=0.5)
    values = set(np.unique(grid.cells).tolist())
    assert {-1, 0, 100}.issubset(values)
    assert np.count_nonzero(grid.cells == 0) > 0
    assert np.count_nonzero(grid.cells == 100) == 2


def test_floor_and_ceiling_points_do_not_become_obstacles():
    endpoints = np.array([[1.0, 0.0, 0.0], [2.0, 0.0, 2.8], [3.0, 0.0, 0.5]])
    origins = np.zeros_like(endpoints)
    grid = raycast_occupancy(endpoints, origins, resolution=0.5, min_obstacle_z=0.1, max_obstacle_z=2.0)
    assert np.count_nonzero(grid.cells == 100) == 1

