import math

import numpy as np
import pytest

from wc_navigation.geometry import Grid
from wc_navigation.localization_coverage import CoverageAnalyzer


def corridor():
    data = np.zeros((61, 601), dtype=np.int8)
    data[10, :] = 100
    data[50, :] = 100
    return Grid(data, .05, (0., -1.525, 0.))


def side_scan():
    # Twenty distinct observations per wall, all lying on the same flat walls.
    x = np.linspace(-.5, .5, 20)
    points = np.vstack([np.column_stack([x, np.ones(20)]), np.column_stack([x, -np.ones(20)])])
    return np.linalg.norm(points, axis=1), np.arctan2(points[:, 1], points[:, 0])


def test_long_parallel_walls_expose_along_heading_ambiguity_without_navigation_pass():
    grid = corridor(); before = grid.data.copy(); ranges, angles = side_scan()
    report = CoverageAnalyzer(grid).analyze(ranges, angles, (15., 0., 0.))
    assert not report['coverage_insufficient']
    assert report['along_heading_ambiguity_evidence']
    assert report['similar_offsets_m'] == [-2., -1., -.5, .5, 1., 2.]
    assert report['status'] == 'REVIEW_REQUIRED'
    assert report['physical_navigation_validated'] is False
    np.testing.assert_array_equal(grid.data, before)


def test_unknown_endpoints_near_wall_are_missing_coverage_not_supported():
    data = np.zeros((60, 60), dtype=np.int8)
    data[30, :] = 100
    data[29, :] = -1
    analyzer = CoverageAnalyzer(Grid(data, .05))
    ranges = np.ones(20)
    # Endpoint y=1.475, neighboring wall center y=1.525 is only 5cm away.
    report = analyzer.analyze(ranges, np.full(20, math.pi / 2), (1.5, .475, 0.))
    assert report['coverage_insufficient']
    assert report['base']['unknown_fraction'] == 1
    assert report['base']['supported_fraction'] == 0
    assert report['base']['mean_capped_cost_with_missing_penalty_m'] == 1
    assert not report['along_heading_ambiguity_evidence']


def test_outside_and_infinite_missing_returns_are_counted_separately():
    grid = Grid(np.zeros((20, 20), dtype=np.int8), .1)
    report = CoverageAnalyzer(grid).analyze([1., 10., math.inf], [0., 0., 0.], (0.5, .5, 0.))
    assert report['missing_beams_excluded'] == 1
    assert report['base']['observed_endpoints'] == 2
    assert report['base']['outside_fraction'] == .5
    assert report['base']['known_fraction'] == .5
    assert report['base']['known_endpoint_distance_to_occupied_center_m'] is None
    assert report['coverage_insufficient']


def test_all_missing_does_not_claim_no_trigger():
    report = CoverageAnalyzer(corridor()).analyze([math.inf] * 20, [0.] * 20, (15., 0., 0.))
    assert report['base']['observed_endpoints'] == 0
    assert report['base']['known_fraction'] is None
    assert report['coverage_insufficient']


def test_distinct_end_wall_breaks_the_tested_translation_fit():
    data = np.zeros((100, 240), dtype=np.int8)
    data[:, 160] = 100
    angles = np.linspace(-.35, .35, 41)
    ranges = (8.025 - 5.025) / np.cos(angles)
    report = CoverageAnalyzer(Grid(data, .05)).analyze(ranges, angles, (5.025, 2.525, 0.))
    assert report['base']['supported_fraction'] == 1
    assert not report['coverage_insufficient']
    assert not report['along_heading_ambiguity_evidence']
    assert report['status'] == 'NO_TRIGGER_IN_TESTED_HYPOTHESES_UNVALIDATED'


def test_rotated_grid_and_explicit_vehicle_heading_preserve_world_meaning():
    grid = corridor(); ranges, angles = side_scan()
    plain = CoverageAnalyzer(grid).analyze(ranges, angles, (15., 0., 0.))
    rotated = CoverageAnalyzer(Grid(grid.data, grid.resolution, (4., 7., math.pi / 2)))
    # Original grid's world origin is (0,-1.525), so laser grid-local y=1.525.
    report = rotated.analyze(ranges, angles, (4.-1.525, 7.+15., math.pi / 2), heading_yaw_rad=math.pi / 2)
    assert report['similar_offsets_m'] == plain['similar_offsets_m']
    assert report['base']['supported_fraction'] == plain['base']['supported_fraction']


def test_extreme_grid_scale_fails_cleanly_before_integer_cast():
    analyzer = CoverageAnalyzer(Grid(np.zeros((2, 2), dtype=np.int8), 1e-310))
    with pytest.raises(ValueError, match='coordinate overflow'):
        analyzer.analyze([100.], [0.], (0., 0., 0.))


@pytest.mark.parametrize('ranges,angles,pose', [
    ([0.], [0.], (0., 0., 0.)), ([-1.], [0.], (0., 0., 0.)),
    ([float('nan')], [0.], (0., 0., 0.)), ([float('-inf')], [0.], (0., 0., 0.)),
    ([101.], [0.], (0., 0., 0.)), ([1.], [float('inf')], (0., 0., 0.)),
    ([1.], [7.], (0., 0., 0.)), ([1.], [0., 1.], (0., 0., 0.)),
    ([True], [0.], (0., 0., 0.)), ([1.], [0.], (0., 0.)),
    ([1.], [0.], (float('nan'), 0., 0.)),
    ([1.] * 4097, [0.] * 4097, (0., 0., 0.)),
])
def test_invalid_or_unbounded_inputs_rejected(ranges, angles, pose):
    with pytest.raises(ValueError):
        CoverageAnalyzer(corridor()).analyze(ranges, angles, pose)
