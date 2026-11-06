"""Odometry-anchored log-odds grid: ray marking, scan semantics and cropping."""
import math

import numpy as np
import pytest

from wc_navigation.autonomy.occupancy import LogOddsGrid


def cell(grid, x, y):
    col = int(math.floor((x - grid.origin[0]) / grid.resolution))
    row = int(math.floor((y - grid.origin[1]) / grid.resolution))
    return row, col


def test_returns_mark_end_cells_occupied_and_rays_free():
    grid = LogOddsGrid(resolution=0.05, size_m=10.0)
    for _ in range(3):
        grid.integrate((0.0, 0.0, 0.0), [-0.05, 0.0, 0.05], [2.0, 2.0, 2.0], 0.3, 8.0)
    data = grid.trinary()
    assert data[cell(grid, 2.0, 0.0)] == 100
    assert data[cell(grid, 1.0, 0.0)] == 0 and data[cell(grid, 1.85, 0.0)] == 0
    assert data[cell(grid, 2.3, 0.0)] == -1      # behind the return stays unknown
    assert data[cell(grid, 0.0, 1.0)] == -1      # outside the beams stays unknown


def test_a_return_never_clears_its_own_end_cell():
    grid = LogOddsGrid(resolution=0.05, size_m=10.0)
    for _ in range(20):
        grid.integrate((0.01, 0.02, 0.3), [0.0], [3.0], 0.3, 8.0)
    end = (0.01 + 3.0 * math.cos(0.3), 0.02 + 3.0 * math.sin(0.3))
    assert grid.trinary()[cell(grid, *end)] == 100
    assert grid.logodds[cell(grid, *end)] == pytest.approx(3.5)  # clamped


def test_no_return_clears_to_range_and_too_close_changes_nothing():
    grid = LogOddsGrid(resolution=0.05, size_m=20.0)
    for _ in range(3):
        report = grid.integrate((0.0, 0.0, 0.0), [0.0, 0.5], [math.inf, -math.inf], 0.3, 5.0)
    data = grid.trinary()
    assert report["no_return"] == 1 and report["finite_returns"] == 0 and report["hit_cells"] == 0
    assert data[cell(grid, 4.5, 0.0)] == 0 and data[cell(grid, 5.5, 0.0)] == -1
    assert data[cell(grid, 0.8 * math.cos(0.5), 0.8 * math.sin(0.5))] == -1
    keep = LogOddsGrid(resolution=0.05, size_m=20.0, clear_no_return=False)
    keep.integrate((0.0, 0.0, 0.0), [0.0], [math.inf], 0.3, 5.0)
    assert not keep.logodds.any()


def test_out_of_range_and_nan_returns_are_ignored():
    grid = LogOddsGrid(resolution=0.05, size_m=20.0)
    report = grid.integrate((0.0, 0.0, 0.0), [0.0, 0.1, 0.2], [math.nan, 0.1, 9.0], 0.3, 8.0)
    assert report["finite_returns"] == 0 and not grid.logodds.any()


def test_cropped_map_covers_observations_with_snapped_blocks():
    grid = LogOddsGrid(resolution=0.05, size_m=30.0)
    assert grid.cropped() is None
    grid.integrate((1.0, -2.0, 1.0), np.linspace(-0.5, 0.5, 21), np.full(21, 3.0), 0.3, 8.0)
    data, origin = grid.cropped(padding_m=1.0, block_m=2.0)
    block = 40
    assert data.shape[0] % block == 0 and data.shape[1] % block == 0
    assert (round((origin[0] - grid.origin[0]) / 0.05) % block, round((origin[1] - grid.origin[1]) / 0.05) % block) == (0, 0)
    observed = np.argwhere(grid.logodds != 0)
    rows = observed[:, 0] - round((origin[1] - grid.origin[1]) / 0.05)
    cols = observed[:, 1] - round((origin[0] - grid.origin[0]) / 0.05)
    assert rows.min() >= 20 and cols.min() >= 20                      # at least 1 m padding
    assert rows.max() < data.shape[0] - 20 and cols.max() < data.shape[1] - 20
    assert origin[2] == 0.0


@pytest.mark.parametrize("kwargs", [{"miss": 0.4}, {"size_m": 500.0}, {"resolution": 0},
                                    {"free_at": -3.0}, {"occupied_at": 4.0}, {"clear_no_return": 1}])
def test_invalid_grid_settings_are_rejected(kwargs):
    with pytest.raises(ValueError):
        LogOddsGrid(**kwargs)


def test_mismatched_or_invalid_scans_are_rejected():
    grid = LogOddsGrid()
    with pytest.raises(ValueError):
        grid.integrate((0.0, 0.0, 0.0), [0.0, 0.1], [1.0], 0.3, 8.0)
    with pytest.raises(ValueError):
        grid.integrate((0.0, 0.0, math.nan), [0.0], [1.0], 0.3, 8.0)
    with pytest.raises(ValueError):
        grid.integrate((0.0, 0.0, 0.0), [0.0], [1.0], 8.0, 0.3)


def test_contested_cells_mostly_seen_through_can_be_classified_free():
    grid = LogOddsGrid(resolution=0.05, size_m=10.0, clear_no_return=False)
    for _ in range(10):                              # the ray passes through (1.0, 0) ten times
        grid.integrate((0.0, 0.0, 0.0), [0.0], [3.0], 0.3, 8.0)
    for _ in range(2):                               # then something stands there twice
        grid.integrate((0.0, 0.0, 0.0), [0.0], [1.0], 0.3, 8.0)
    middle = cell(grid, 1.0, 0.0)
    assert grid.hits[middle] == 2 and grid.misses[middle] == 10
    assert -1.0 < grid.logodds[middle] < 1.0         # order-dependent log-odds: unknown
    assert grid.trinary()[middle] == -1
    assert grid.trinary(contested_free=True)[middle] == 0
    assert grid.trinary(contested_free=True, max_hit_ratio=0.1)[middle] == -1
    wall = cell(grid, 3.0, 0.0)
    assert grid.trinary(contested_free=True)[wall] == 100     # occupied still wins
    sparse = LogOddsGrid(resolution=0.05, size_m=10.0, clear_no_return=False)
    sparse.integrate((0.0, 0.0, 0.0), [0.0], [3.0], 0.3, 8.0)
    sparse.integrate((0.0, 0.0, 0.0), [0.0], [1.0], 0.3, 8.0)
    assert sparse.trinary(contested_free=True)[middle] == -1  # two observations are not enough
    data, _ = grid.cropped(contested_free=True)
    assert (data == 0).sum() > (grid.cropped()[0] == 0).sum()
