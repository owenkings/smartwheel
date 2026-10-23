"""Analytical foreground observations, never sensor data or extrinsic truth."""
import json
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'src'))
from wc_calibration.small_board_diagnostic import diagnose_small_board


def board_scene(two=False):
    width, height = 100, 60
    horizontal, vertical = np.meshgrid(np.linspace(-.60, .60, width), np.linspace(.40, -.40, height))
    rays = np.stack((np.ones_like(horizontal), horizontal, vertical), axis=-1).reshape(-1, 3)
    board = (np.abs(rays[:, 1]) <= .25) & (np.abs(rays[:, 2]) <= .25)
    if two:
        board = ((np.abs(rays[:, 1] + .22) <= .14) | (np.abs(rays[:, 1] - .22) <= .14)) & (np.abs(rays[:, 2]) <= .22)
    distance = np.where(board, 1., 2.5)
    cloud = rays * distance[:, None]
    return cloud, width, height, board


def diagnose(cloud, width, height, frames=None):
    result = diagnose_small_board(cloud, frames=frames, width=width, height=height)
    json.dumps(result, allow_nan=False)
    return result


def test_known_planar_board_geometry_and_no_extrinsic_claim():
    cloud, w, h, board = board_scene()
    result = diagnose(cloud, w, h, np.repeat(cloud[None], 3, axis=0))
    assert result['status'] == 'DIAGNOSTIC_AVAILABLE'
    assert result['physical_target_identity_verified'] is False
    assert result['extrinsic_calibration_evaluated'] is False
    np.testing.assert_allclose(result['interior_fit']['normal'], [-1, 0, 0], atol=1e-10)
    assert abs(result['interior_fit']['d_m'] - 1) < 1e-10
    assert result['interior_fit']['rmse_m'] < 1e-12
    assert set(result['interior_indices']) < set(result['raw_candidate_indices'])
    assert all(board[i] for i in result['raw_candidate_indices'])
    assert result['temporal']['fixed_pixel_support'] is True
    assert result['temporal']['independent_validation'] is False
    assert result['temporal']['d_parameter_variation_is_physical_motion_proof'] is False
    assert 'transform' not in result


def test_boundary_and_inside_outliers_remain_visible_in_raw_diagnostics():
    cloud, w, h, board = board_scene()
    clean = diagnose(cloud, w, h)
    ids = np.asarray(clean['raw_candidate_indices'])
    # An edge pixel is excluded from the interior, while the raw plane and
    # residual diagnostic must still contain its analytical 12 cm error.
    edge_id = next(i for i in ids if i not in set(clean['interior_indices']))
    cloud[edge_id, 0] += .12
    interior_id = clean['interior_indices'][len(clean['interior_indices'])//2]
    cloud[interior_id, 0] = 2.2  # outside selection band, still inside its envelope
    result = diagnose(cloud, w, h)
    assert edge_id in result['raw_candidate_indices']
    assert edge_id not in result['interior_indices']
    assert result['all_candidate_residual_to_interior_plane']['maximum_absolute_m'] >= .119
    assert interior_id in result['excluded_by_x_band_inside_candidate_envelope_indices']
    assert result['all_envelope_residual_to_interior_plane']['maximum_absolute_m'] >= 1.19
    assert result['raw_candidate_fit']['residual_trimming_performed'] is False


def test_two_central_foregrounds_are_explicitly_ambiguous():
    cloud, w, h, _ = board_scene(two=True)
    result = diagnose(cloud, w, h)
    assert result['status'] == 'UNKNOWN'
    assert 'MULTIPLE_CENTRAL_FOREGROUND_COMPONENTS' in result['reasons']
    assert result['selection']['status'] == 'NOT_SELECTED'


def test_no_data_and_missing_frames_are_not_passed():
    result = diagnose(np.full((6000, 3), np.nan), 100, 60)
    assert result['status'] == 'REJECTED'
    cloud, w, h, _ = board_scene()
    result = diagnose(cloud, w, h)
    assert result['temporal']['status'] == 'NOT_EVALUATED'
    assert result['temporal']['reasons'] == ['NO_TEMPORAL_FRAMES']


def test_fixed_pixels_retain_frame_outliers_and_report_nan_failure():
    cloud, w, h, _ = board_scene()
    initial = diagnose(cloud, w, h)
    ids = np.asarray(initial['interior_indices'])
    frames = np.repeat(cloud[None], 3, axis=0)
    frames[1, ids[:len(ids)//5], 0] += .20
    frames[2, ids] = np.nan
    result = diagnose(cloud, w, h, frames)
    rows = result['temporal']['frames']
    assert rows[1]['reference_plane_residual']['maximum_absolute_m'] >= .199
    assert rows[1]['fit']['point_count'] == len(ids)
    assert rows[1]['residual_trimming_performed'] is False
    assert rows[1]['status'] == 'WARNING'
    assert rows[2]['status'] == 'INVALID'
    assert rows[2]['invalid_pixel_indices'] == ids.tolist()
    assert rows[2]['fit'] is None
    assert result['temporal']['invalid_frame_indices'] == [2]


def test_cropped_off_center_unique_patch_keeps_visible_diagnostics_unknown():
    w, h = 100, 60
    horizontal, vertical = np.meshgrid(np.linspace(-.6, .6, w), np.linspace(.4, -.4, h))
    rays = np.stack((np.ones_like(horizontal), horizontal, vertical), axis=-1).reshape(-1, 3)
    visible_board = (np.abs(rays[:, 1]) <= .25) & (rays[:, 2] <= -.15)
    cloud = rays * np.where(visible_board, 1., 2.5)[:, None]
    result = diagnose(cloud, w, h)
    assert result['status'] == 'UNKNOWN'
    assert 'RAW_CANDIDATE_CLIPPED_BY_IMAGE_BOUNDARY' in result['reasons']
    assert 'UNIQUE_FOREGROUND_OUTSIDE_CENTRAL_SELECTION_REGION' in result['reasons']
    assert result['raw_candidate_indices'] and result['interior_indices']
    assert result['raw_candidate_fit']['rmse_m'] < 1e-12
    assert result['all_candidate_residual_to_interior_plane']['point_count'] == len(result['raw_candidate_indices'])


def test_malformed_frames_remain_explicit_rejection_without_losing_geometry():
    cloud, w, h, _ = board_scene()
    result = diagnose(cloud, w, h, np.zeros((3, 7, 3)))
    assert result['status'] == 'UNKNOWN'
    assert result['interior_fit'] is not None
    assert result['temporal']['status'] == 'REJECTED'
    assert result['temporal']['reasons'] == ['INVALID_TEMPORAL_FRAME_SHAPE']


def test_explicit_distance_band_recovers_farther_complete_analytical_board():
    _, w, h, _ = board_scene()
    horizontal, vertical = np.meshgrid(np.linspace(-.6, .6, w), np.linspace(.4, -.4, h))
    rays = np.stack((np.ones_like(horizontal), horizontal, vertical), axis=-1).reshape(-1, 3)
    board = (np.abs(1.8*rays[:, 1]) <= .25) & (np.abs(1.8*rays[:, 2]) <= .25)
    cloud = rays * np.where(board, 1.8, 3.)[:, None]
    assert diagnose(cloud, w, h)['status'] == 'REJECTED'
    result = diagnose_small_board(cloud, frames=np.repeat(cloud[None], 3, axis=0),
        width=w, height=h, selection_x_band_m=(.6, 2.),
        selection_provenance={'basis': 'ANALYTICAL_SCENE_OPERATOR_DISTANCE_BAND'})
    json.dumps(result, allow_nan=False)
    assert result['status'] == 'DIAGNOSTIC_AVAILABLE'
    assert abs(result['interior_fit']['d_m'] - 1.8) < 1e-12
    assert result['selection_parameters']['x_band_m'] == [.6, 2.]
    assert result['selection_parameters']['provenance']['basis'] == 'ANALYTICAL_SCENE_OPERATOR_DISTANCE_BAND'
    assert all(f['finite_points_outside_selection_x_band'] == 0 for f in result['temporal']['frames'])
    assert result['selection_band_boundary_indices'] == []


def test_distance_band_cut_is_unknown_even_when_partial_plane_fits_perfectly():
    w, h = 100, 60
    horizontal, vertical = np.meshgrid(np.linspace(-.6, .6, w), np.linspace(.4, -.4, h))
    rays = np.stack((np.ones_like(horizontal), horizontal, vertical), axis=-1).reshape(-1, 3)
    # X = 1.55 + .4 Y crosses the default distance limit within the actual
    # finite board; no physical depth edge exists at the algorithmic cut.
    distance = 1.55 / (1. - .4*rays[:, 1])
    points = rays * distance[:, None]
    board = (np.abs(points[:, 1]) <= .25) & (np.abs(points[:, 2]) <= .25)
    cloud = np.where(board[:, None], points, rays*3.)
    result = diagnose(cloud, w, h)
    assert result['status'] == 'UNKNOWN'
    assert 'CANDIDATE_CONTINUES_BEYOND_SELECTION_X_BAND' in result['reasons']
    assert result['interior_fit']['rmse_m'] < 1e-12
    assert len(result['selection_band_boundary_indices']) > 0
    outside = result['selection_band_excluded_adjacent_indices']
    assert all(board[i] and cloud[i, 0] > 1.6 for i in outside)
    assert set(outside).issubset(result['excluded_by_x_band_inside_candidate_envelope_indices']) is False
    # Outside-band adjacency is retained separately even outside the truncated
    # candidate's bounding box, where the old envelope could not expose it.
    complete = diagnose_small_board(cloud, frames=None, width=w, height=h,
        selection_x_band_m=(.6, 2.))
    assert complete['status'] == 'DIAGNOSTIC_AVAILABLE'
    assert len(complete['raw_candidate_indices']) > len(result['raw_candidate_indices'])


def test_roi_cannot_hide_coplanar_support_box_continuing_to_image_bottom():
    cloud, w, h, board = board_scene()
    rays = cloud / cloud[:, 0, None]
    support = (np.abs(rays[:, 1]) <= .10) & (rays[:, 2] < -.25)
    cloud[support] = rays[support]
    result = diagnose_small_board(cloud, frames=None, width=w, height=h,
        selection_roi_xyxy=[20, 8, 79, 49],
        selection_provenance='Analytical coarse discovery ROI; not a physical board mask')
    json.dumps(result, allow_nan=False)
    assert result['status'] == 'UNKNOWN'
    assert 'RAW_CANDIDATE_TOUCHES_SELECTION_ROI_BOUNDARY' in result['reasons']
    assert 'CONNECTED_FOREGROUND_CONTINUES_OUTSIDE_SELECTION_ROI' in result['reasons']
    assert result['interior_fit']['rmse_m'] < 1e-12
    outside = result['outside_roi_same_component_indices']
    assert outside and all(support[i] for i in outside)
    assert set(result['raw_candidate_indices']).isdisjoint(outside)
    assert set(result['full_connected_foreground_indices']) == set(result['raw_candidate_indices']) | set(outside)
    assert result['outside_roi_same_component_residual_to_interior_plane']['point_count'] == len(outside)
    assert result['full_connected_foreground_residual_to_interior_plane']['point_count'] == np.count_nonzero(board | support)
    assert result['selection_roi_boundary_indices']


def test_coarse_roi_fully_enclosing_a_separate_board_preserves_original_pixels():
    cloud, w, h, board = board_scene()
    result = diagnose_small_board(cloud, frames=None, width=w, height=h,
        selection_roi_xyxy=[20, 8, 79, 52], selection_provenance={'basis': 'ANALYTICAL_COARSE_ROI'})
    assert result['status'] == 'DIAGNOSTIC_AVAILABLE'
    assert set(result['raw_candidate_indices']) == set(np.flatnonzero(board))
    assert result['outside_roi_same_component_indices'] == []
    assert result['selection_roi_boundary_indices'] == []
    assert result['physical_target_identity_verified'] is False


@pytest.mark.parametrize('kwargs,reason', [
    ({'selection_x_band_m': [float('nan'), 2.]}, 'INVALID_SELECTION_X_BAND'),
    ({'selection_x_band_m': [.6, float('inf')]}, 'INVALID_SELECTION_X_BAND'),
    ({'selection_x_band_m': ['.6', '2.0']}, 'INVALID_SELECTION_X_BAND'),
    ({'selection_x_band_m': [2., .6]}, 'INVALID_SELECTION_X_BAND'),
    ({'selection_x_band_m': [.1, 2.]}, 'INVALID_SELECTION_X_BAND'),
    ({'selection_x_band_m': [.6, 13.]}, 'INVALID_SELECTION_X_BAND'),
    ({'selection_x_band_m': [True, False]}, 'INVALID_SELECTION_X_BAND'),
    ({'selection_roi_xyxy': [20, 8, 79, 60]}, 'INVALID_SELECTION_ROI'),
    ({'selection_roi_xyxy': [-1, 8, 79, 52]}, 'INVALID_SELECTION_ROI'),
    ({'selection_roi_xyxy': [79, 8, 20, 52]}, 'INVALID_SELECTION_ROI'),
    ({'selection_roi_xyxy': [20., 8., 79., 52.]}, 'INVALID_SELECTION_ROI'),
    ({'selection_roi_xyxy': ['20', '8', '79', '52']}, 'INVALID_SELECTION_ROI'),
    ({'selection_provenance': {'distance': float('nan')}}, 'INVALID_SELECTION_PROVENANCE'),
])
def test_invalid_selection_parameters_are_rejected_as_strict_json(kwargs, reason):
    cloud, w, h, _ = board_scene()
    result = diagnose_small_board(cloud, frames=None, width=w, height=h, **kwargs)
    json.dumps(result, allow_nan=False)
    assert result['status'] == 'REJECTED'
    assert result['reasons'] == [reason]
