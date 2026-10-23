"""Controlled complete-board gates using analytical points, not recordings."""
import copy
import json
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'src'))
from wc_calibration.small_board_diagnostic import diagnose_small_board
from wc_calibration.small_board_planes import controlled_board_planes


def fixture(cropped=False, edge_contamination=False, missing_frames=False):
    w, h = 100, 60
    u, v = np.meshgrid(np.linspace(-.6, .6, w), np.linspace(.4, -.4, h))
    rays = np.stack((np.ones_like(u), u, v), axis=-1).reshape(-1, 3)
    board = (np.abs(rays[:, 1]) <= .25) & (np.abs(rays[:, 2]) <= .25)
    if cropped:
        board = (np.abs(rays[:, 1]) <= .25) & (rays[:, 2] <= -.15)
    cloud = rays * np.where(board, 1., 2.5)[:, None]
    frames = np.repeat(cloud[None], 4, axis=0)
    diagnostic = diagnose_small_board(cloud, frames=frames, width=w, height=h)
    if edge_contamination:
        boundary = sorted(set(diagnostic['raw_candidate_indices']) - set(diagnostic['interior_indices']))
        cloud[boundary, 0] += .15
        frames = np.repeat(cloud[None], 4, axis=0)
        diagnostic = diagnose_small_board(cloud, frames=frames, width=w, height=h)
    diagnostic['interior_points'] = cloud[diagnostic['interior_indices']]
    diagnostic['raw_candidate_points'] = cloud[diagnostic['raw_candidate_indices']]
    diagnostic['fixed_interior_frames'] = frames[:, diagnostic['interior_indices'], :]
    if missing_frames:
        diagnostic.pop('fixed_interior_frames')
    diagnostic['identity_assumption'] = {'basis': 'OPERATOR_SAME_SINGLE_WOOD_BOARD_FACE',
                                         'automatic_identity_verified': False}
    return diagnostic


def run(diagnostic):
    result = controlled_board_planes(diagnostic)
    json.dumps(result, allow_nan=False)
    return result


def test_complete_50cm_board_can_supply_one_local_plane_without_accuracy_claim():
    diagnostic = fixture()
    before = copy.deepcopy(diagnostic)
    result = run(diagnostic)
    assert result['status'] == 'PASS', result['reasons']
    assert len(result['planes']) == 1 and not result['rejected_planes']
    plane = result['planes'][0]
    np.testing.assert_allclose(plane['normal'], [-1, 0, 0], atol=1e-10)
    assert abs(plane['d_m'] - 1.) < 1e-10
    assert plane['projected_mesh_area_m2'] >= .08
    assert plane['temporal']['status'] == 'PASS'
    assert result['physical_target_identity_verified'] is False
    assert result['extrinsic_calibration_evaluated'] is False
    assert result['absolute_accuracy_estimated'] is False
    assert plane['residual_trimming_performed'] is False
    assert plane['identity_assumption']['automatic_identity_verified'] is False
    np.testing.assert_array_equal(diagnostic['fixed_interior_frames'], before['fixed_interior_frames'])
    assert diagnostic['temporal'] == before['temporal']


def test_cropped_fallback_never_becomes_a_solver_plane():
    result = run(fixture(cropped=True))
    assert result['status'] == 'REJECTED' and not result['planes']
    assert 'COMPLETE_CENTRAL_SELECTION_REQUIRED_NO_PARTIAL_FALLBACK' in result['reasons']
    assert 'CANDIDATE_FOUR_IMAGE_BOUNDS_MUST_BE_COMPLETE' in result['reasons']


def test_edge_mixed_returns_are_not_hidden_by_an_accurate_interior():
    result = run(fixture(edge_contamination=True))
    assert result['status'] == 'REJECTED'
    assert 'RAW_CANDIDATE_P95_EXCEEDED' in result['reasons'] or 'RAW_CANDIDATE_OUTLIER_FRACTION_EXCEEDED' in result['reasons']
    assert result['source_diagnostic']['all_candidate_residual_to_interior_plane']['maximum_absolute_m'] >= .14


def test_missing_raw_coordinates_and_fixed_frames_fail_closed():
    missing_points = fixture()
    missing_points.pop('interior_points')
    assert run(missing_points)['status'] == 'REJECTED'
    result = run(fixture(missing_frames=True))
    assert result['status'] == 'REJECTED'
    assert result['rejected_planes'][0]['temporal']['status'] != 'PASS'


def test_evaluated_source_label_cannot_override_recomputed_frame_anomaly():
    diagnostic = fixture()
    diagnostic['fixed_interior_frames'][1:, :, 0] += .12
    assert diagnostic['temporal']['status'] == 'EVALUATED'
    result = run(diagnostic)
    assert result['status'] == 'REJECTED'
    assert result['rejected_planes'][0]['temporal']['status'] == 'REJECTED'
    assert 'FIXED_PIXEL_TEMPORAL_GATES_NOT_PASSED' in result['reasons']


def test_nan_fixed_support_and_in_plane_coverage_collapse_are_rejected():
    diagnostic = fixture()
    diagnostic['fixed_interior_frames'][1:] = np.nan
    result = run(diagnostic)
    assert result['status'] == 'REJECTED'
    assert result['rejected_planes'][0]['temporal']['frames'][1]['invalid_original_pixel_indices']
    diagnostic = fixture()
    center = np.mean(diagnostic['interior_points'], axis=0)
    diagnostic['fixed_interior_frames'][1:] = center
    result = run(diagnostic)
    assert result['status'] == 'REJECTED'
    reasons = result['rejected_planes'][0]['temporal']['frames'][1]['reasons']
    assert 'FIXED_SUPPORT_SPATIAL_COVERAGE_COLLAPSED' in reasons


def test_identity_assumption_and_exact_original_index_coordinates_are_required():
    diagnostic = fixture()
    diagnostic.pop('identity_assumption')
    result = run(diagnostic)
    assert result['status'] == 'REJECTED'
    assert 'EXPLICIT_OPERATOR_SAME_BOARD_FACE_ASSUMPTION_REQUIRED' in result['reasons']
    diagnostic = fixture()
    diagnostic['interior_points'][0, 0] += .01
    result = run(diagnostic)
    assert 'RAW_AND_INTERIOR_ORIGINAL_COORDINATES_DISAGREE' in result['reasons']


def test_temporal_at_least_three_frames_and_reference_axis_span_gate():
    diagnostic = fixture()
    diagnostic['fixed_interior_frames'] = diagnostic['fixed_interior_frames'][:2]
    diagnostic['temporal']['provided_frame_count'] = 2
    result = run(diagnostic)
    assert result['status'] == 'REJECTED'
    assert result['rejected_planes'][0]['temporal']['status'] == 'NOT_EVALUATED'
    diagnostic = fixture()
    fixed = diagnostic['fixed_interior_frames']
    center = np.mean(diagnostic['interior_points'], axis=0)
    fixed[1:, :, 1:] = center[1:] + .4*(fixed[1:, :, 1:] - center[1:])
    result = run(diagnostic)
    assert result['status'] == 'REJECTED'
    assert 'FIXED_SUPPORT_SPATIAL_COVERAGE_COLLAPSED' in result['rejected_planes'][0]['temporal']['frames'][1]['reasons']


def test_visible_hole_cannot_be_filled_by_convex_hull_area():
    w, h = 100, 60
    u, v = np.meshgrid(np.linspace(-.6, .6, w), np.linspace(.4, -.4, h))
    rays = np.stack((np.ones_like(u), u, v), axis=-1).reshape(-1, 3)
    board = (np.abs(rays[:, 1]) <= .25) & (np.abs(rays[:, 2]) <= .25)
    hole = (np.abs(rays[:, 1]) < .12) & (np.abs(rays[:, 2]) < .10)
    cloud = rays * np.where(board & ~hole, 1., 2.5)[:, None]
    frames = np.repeat(cloud[None], 4, axis=0)
    diagnostic = diagnose_small_board(cloud, frames=frames, width=w, height=h)
    diagnostic.update(interior_points=cloud[diagnostic['interior_indices']],
        raw_candidate_points=cloud[diagnostic['raw_candidate_indices']],
        fixed_interior_frames=frames[:, diagnostic['interior_indices'], :],
        identity_assumption={'basis': 'OPERATOR_SAME_SINGLE_WOOD_BOARD_FACE',
                             'automatic_identity_verified': False})
    result = run(diagnostic)
    assert result['status'] == 'REJECTED'
    plane = result['rejected_planes'][0]
    assert plane['hull_area_m2'] >= .08
    assert plane['projected_mesh_area_m2'] < .08
    assert 'INSUFFICIENT_PROJECTED_INTERNAL_MESH_AREA' in result['reasons']
