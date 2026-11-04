import json
import math

import numpy as np
import pytest

from wc_navigation.geometry import Grid
from wc_navigation.grid_quality import audit_array, compare_grids


def scene(data, resolution=.1, origin=(0., 0., 0.)):
    return Grid(np.asarray(data, dtype=np.int8), resolution, origin)


def test_unknown_and_map_edge_remain_blocked_and_input_unchanged():
    data = np.zeros((30, 40), dtype=np.int8)
    data[:, 19:21] = -1
    grid = Grid(data, .1)
    before = grid.data.tobytes()
    result = audit_array(grid, .35, .05)
    assert result['grid']['unknown_cells'] == 60
    assert result['traversability']['connected_components'] == 2
    assert result['traversability']['free_cells_after_inflation'] < (data == 0).sum()
    assert result['unknown_space'] == result['map_boundary'] == 'BLOCKED'
    assert result['navigation_validated'] is False
    assert grid.data.tobytes() == before
    json.dumps(result, allow_nan=False)


def test_narrow_gap_blocked_under_same_assumed_footprint():
    # Two large rooms joined by a 0.6 m opening.
    data = np.zeros((60, 100), dtype=np.int8)
    data[:, 49:51] = 100
    data[27:33, 49:51] = 0
    grid = Grid(data, .1)
    small = audit_array(grid, .15, .05)
    wheelchair = audit_array(grid, .55, .10)
    assert small['traversability']['connected_components'] == 1
    assert wheelchair['traversability']['connected_components'] == 2
    assert wheelchair['traversability']['largest_component_cells'] < small['traversability']['largest_component_cells']


def test_route_center_free_does_not_imply_circle_or_swept_free():
    data = np.zeros((40, 60), dtype=np.int8)
    data[10:30, 29:31] = 100
    grid = Grid(data, .1)
    route = [[1., 2.], [5., 2.]]
    result = audit_array(grid, .35, .05, trajectory_xy=route)
    route_result = result['trajectory_coverage']
    assert route_result['free_points'] == 2
    assert route_result['circle_clear_points'] == 2
    assert route_result['swept_clear_segments'] == 0
    assert route_result['swept_clear_route_length_m'] == 0


def test_route_outside_unknown_and_occupied_are_distinct():
    data = np.zeros((20, 20), dtype=np.int8)
    data[5, 5] = -1
    data[15, 15] = 100
    grid = Grid(data, .1)
    route = [[-.1, 1.], grid.cell_to_world((5, 5)), grid.cell_to_world((15, 15)), [1., 1.]]
    r = audit_array(grid, .1, 0, trajectory_xy=route)['trajectory_coverage']
    assert (r['outside_map'], r['unknown_points'], r['occupied_points'], r['free_points']) == (1, 1, 1, 1)
    assert r['circle_clear_points'] == 1


def test_low_obstacle_loss_is_visible_in_diagnostic_high_threshold():
    # Synthetic 10 cm object: the conservative 5 cm height map retains it;
    # a 15 cm-height diagnostic map would classify its returns as floor.
    conservative = np.zeros((30, 30), dtype=np.int8)
    conservative[14:16, 14:16] = 100
    diagnostic = np.zeros_like(conservative)
    first, second = Grid(conservative, .1), Grid(diagnostic, .1)
    observations = [first.cell_to_world((14, 14)), first.cell_to_world((15, 15))]
    a = audit_array(first, .2, .05, low_points_xy=observations, wall_points_xy=observations)
    b = audit_array(second, .2, .05, low_points_xy=observations, wall_points_xy=observations)
    assert a['low_band_point_occupancy']['occupied_fraction'] == 1
    assert b['low_band_point_occupancy']['free_fraction'] == 1
    assert a['wall_point_support']['supported_fraction'] == 1
    assert b['wall_point_support']['supported_fraction'] == 0
    compared = compare_grids({'height05': first, 'height15_diagnostic': second}, 'height05')
    assert compared['maps']['height15_diagnostic']['baseline_transitions']['occupied_to_free'] == 4
    assert compared['navigation_validated'] is False


def test_wall_support_uses_world_coordinates_with_rotated_origin():
    data = np.zeros((20, 20), dtype=np.int8)
    data[:, 10] = 100
    grid = Grid(data, .1, (10., -5., math.pi / 2))
    wall = grid.cell_to_world((10, 10))
    far = grid.cell_to_world((2, 10))
    r = audit_array(grid, .1, 0, wall_points_xy=[wall, far])['wall_point_support']
    assert r['supported_points'] == 1
    assert r['supported_fraction'] == .5


def test_differing_origins_compare_common_world_region_not_pixel_indices():
    baseline = np.zeros((4, 8), dtype=np.int8)
    baseline[:, 4] = 100
    shifted = np.zeros((4, 8), dtype=np.int8)
    shifted[:, 2] = 100  # Same world column after origin shifts by 0.2 m.
    a, b = Grid(baseline, .1), Grid(shifted, .1, (.2, 0., 0.))
    result = compare_grids({'original': a, 'shifted': b}, 'original')
    assert result['sampled_cells'] == 24
    assert result['maps']['shifted']['changed_cells'] == 0
    assert result['maps']['shifted']['cells']['occupied'] == 4
    assert result['maps']['shifted']['baseline_transitions']['occupied_to_occupied'] == 4


def test_unknown_to_free_is_reported_and_never_implicitly_certified():
    a = np.full((5, 5), -1, dtype=np.int8)
    b = a.copy(); b[1:4, 1:4] = 0
    result = compare_grids({'old': Grid(a, .1), 'new': Grid(b, .1)}, 'old')
    assert result['maps']['new']['baseline_transitions']['unknown_to_free'] == 9
    assert result['navigation_validated'] is False


@pytest.mark.parametrize('bad', [np.nan, np.inf, -.1, 0])
def test_invalid_radius_rejected(bad):
    with pytest.raises(ValueError):
        audit_array(Grid(np.zeros((10, 10), dtype=np.int8), .1), bad)


@pytest.mark.parametrize('bad', [[[float('nan'), 1]], [[0, 1, 2]], [1, 2]])
def test_malformed_point_inputs_rejected(bad):
    with pytest.raises(ValueError):
        audit_array(Grid(np.zeros((10, 10), dtype=np.int8), .1), trajectory_xy=bad)


def test_empty_optional_points_produce_finite_json():
    r = audit_array(Grid(np.zeros((10, 10), dtype=np.int8), .1), .1, 0,
                    trajectory_xy=[], wall_points_xy=[], low_points_xy=[])
    assert r['trajectory_coverage']['circle_clear_fraction'] is None
    assert r['wall_point_support']['supported_fraction'] is None
    assert r['low_band_point_occupancy']['free_fraction'] is None
    json.dumps(r, allow_nan=False)


def test_incompatible_common_roi_is_rejected():
    a = Grid(np.zeros((10, 10), dtype=np.int8), .1)
    with pytest.raises(ValueError, match='axis-aligned'):
        compare_grids({'a': a, 'rotated': Grid(a.data, .1, (0., 0., .1))}, 'a')
    with pytest.raises(ValueError, match='identical resolution'):
        compare_grids({'a': a, 'different': Grid(a.data, .2)}, 'a')
    with pytest.raises(ValueError, match='overlapping'):
        compare_grids({'a': a, 'far': Grid(a.data, .1, (2., 0., 0.))}, 'a')
