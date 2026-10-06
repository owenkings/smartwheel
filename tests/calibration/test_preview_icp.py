"""Single-scene synthetic geometry tests, never sensor/extrinsic validation evidence."""
import json

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from wc_calibration import preview_icp as preview
from wc_calibration.core import _se3_exp, _transform


def geometry(seed=371, count=1100):
    rng = np.random.default_rng(seed)
    patches = []
    for axis, offset in ((0, 3.0), (1, -.85), (2, -.70)):
        points = np.column_stack((rng.uniform(1.2, 3.0, count), rng.uniform(-.85, 1.0, count),
                                  rng.uniform(-.70, .9, count)))
        points[:, axis] = offset
        patches.append(points)
    # A separate oblique patch breaks repeated axis-aligned geometry.
    a, b = rng.uniform(-.6, .6, (2, count//2))
    patches.append(np.column_stack((2.1+.3*a-.1*b, .4+a, .2+b)))
    return np.concatenate(patches)


def known_transform():
    T = np.eye(4)
    T[:3, :3] = Rotation.from_euler('xyz', [2., -3., 4.], degrees=True).as_matrix()
    T[:3, 3] = [.12, -.6, .03]
    return T


def error(found, expected):
    found = np.asarray(found)
    return np.linalg.norm(found[:3, 3]-expected[:3, 3]), \
        np.degrees(Rotation.from_matrix(found[:3, :3] @ expected[:3, :3].T).magnitude())


def assert_safe_result(result):
    json.dumps(result, allow_nan=False)
    assert result['schema_version'] == 1 and result['kind'] == 'offline_icp_candidate'
    assert result['live_eligible'] is False and result['independent_validation_performed'] is False
    assert result['policy']['scale_fitted'] is False
    T = np.asarray(result['refined_T_left_right'])
    assert np.allclose(T[:3, :3].T@T[:3, :3], np.eye(3), atol=1e-9)
    assert np.isclose(np.linalg.det(T[:3, :3]), 1.)
    assert result['transform_change']['translation_m'] <= .5+1e-10
    assert result['transform_change']['rotation_deg'] <= 20.+1e-10
    assert len(result['history']) <= sum(x['iterations'] for x in result['policy']['levels'])
    for row in result['history']:
        if 'translation_step_m' in row:
            assert row['translation_step_m'] <= .06+1e-12
            assert row['rotation_step_rad'] <= np.radians(3)+1e-12
            assert row['rank'] == 6
            assert row['normal_direction_eigenvalues'][0] >= result['policy']['minimum_normal_direction_eigenvalue']
            assert row['fixed_correspondence_huber_loss_after'] <= row['fixed_correspondence_huber_loss_before']+1e-13


def test_known_rigid_transform_refines_and_preserves_input_rows():
    left = geometry()
    expected = known_transform()
    right = _transform(left, np.linalg.inv(expected))
    before_left, before_right = left.copy(), right.copy()
    initial = _se3_exp(np.array([.018, -.014, .022, .038, -.025, .03]))@expected
    result = preview.refine_preview(left, right, initial)
    assert_safe_result(result)
    assert result['status'] == 'CANDIDATE', result['reasons']
    translation, angle = error(result['refined_T_left_right'], expected)
    assert translation < .008 and angle < .25, (translation, angle)
    assert result['final_metrics']['nn_rmse_m'] < result['initial_metrics']['nn_rmse_m']*.4
    assert np.array_equal(left, before_left) and np.array_equal(right, before_right)
    for pair in result['correspondences'][::47]:
        a, b = pair['left_index'], pair['right_index']
        observed = np.linalg.norm(left[a]-_transform(right[[b]], np.asarray(result['refined_T_left_right']))[0])
        assert abs(observed-pair['distance_m']) < 1e-10
        assert pair['verified'] is False


def test_partial_overlap_and_outliers_remain_a_candidate_not_validation():
    rng = np.random.default_rng(990)
    all_points = geometry(count=1400)
    expected = known_transform()
    left = all_points[rng.choice(len(all_points), int(.82*len(all_points)), replace=False)]
    source = all_points[rng.choice(len(all_points), int(.74*len(all_points)), replace=False)]
    right = _transform(source, np.linalg.inv(expected))
    left += rng.normal(0, .001, left.shape)
    right += rng.normal(0, .001, right.shape)
    left = np.vstack((left, rng.uniform([.7, -2, -1], [4, 2, 1.8], (300, 3))))
    right = np.vstack((right, rng.uniform([.7, -2, -1], [4, 2, 1.8], (500, 3))))
    initial = _se3_exp(np.array([-.015, .012, .016, -.028, .025, -.018]))@expected
    result = preview.refine_preview(left, right, initial)
    assert_safe_result(result)
    assert result['status'] == 'CANDIDATE', result['reasons']
    translation, angle = error(result['refined_T_left_right'], expected)
    assert translation < .015 and angle < .5, (translation, angle)
    assert result['final_metrics']['forward_overlap_fraction'] > .65
    assert result['final_metrics']['reverse_overlap_fraction'] > .65


def test_evaluation_indices_are_original_rows_and_bidirectional_coverage_is_distinct():
    left = geometry(count=150)
    order = np.arange(len(left)-1, -1, -2)
    right = left[order]
    # More distant target-only points lower reverse coverage without lowering forward coverage.
    left = np.vstack((left, [[20., 20., 20.], [21., 21., 21.]]))
    metrics, pairs = preview._nearest_metrics(left, right, np.eye(4))
    assert metrics == preview.evaluate_preview(left, right, np.eye(4))
    assert metrics['forward_overlap_fraction'] == 1.0
    assert metrics['reverse_overlap_fraction'] < 1.0
    assert [p['right_index'] for p in pairs] == list(range(len(right)))
    assert all(p['left_index'] == int(order[p['right_index']]) for p in pairs)
    assert all(p['distance_m'] < 1e-12 and p['verified'] is False for p in pairs)


@pytest.mark.parametrize('kind', ['plane', 'line', 'coincident'])
def test_degenerate_geometry_is_never_promoted(kind):
    rng = np.random.default_rng(321)
    if kind == 'plane':
        left = np.column_stack((rng.uniform(-1, 1, 1200), rng.uniform(-1, 1, 1200), np.zeros(1200)))
    elif kind == 'line':
        left = np.column_stack((np.linspace(-2, 2, 1200), np.zeros(1200), np.zeros(1200)))
    else:
        left = np.tile([1., 1., 1.], (100, 1))
    result = preview.refine_preview(left, left, np.eye(4))
    assert_safe_result(result)
    assert result['status'] == 'DEGENERATE'
    assert result['reasons']
    assert np.array_equal(result['refined_T_left_right'], np.eye(4))


@pytest.mark.parametrize('rotate_scene', [False, True])
@pytest.mark.parametrize('line_direction_offset_m', [0.0, .08])
def test_intersecting_planes_pca_edge_normals_cannot_invent_translation_constraints(rotate_scene, line_direction_offset_m):
    # Keep the crossing strip: removing it hides the original false-rank regression.
    rng = np.random.default_rng(9013)
    a = np.column_stack((np.full(1500, 3.), rng.uniform(-.8, 1., 1500), rng.uniform(-.7, 1.3, 1500)))
    b = np.column_stack((rng.uniform(1., 3., 1500), np.full(1500, -.8), rng.uniform(-.7, 1.3, 1500)))
    left = np.vstack((a, b))
    world_rotation = Rotation.from_euler('xyz', [13., -21., 31.], degrees=True).as_matrix() if rotate_scene else np.eye(3)
    left = left@world_rotation.T
    expected = known_transform()
    expected[:3, 3] = [.1, -.6, .04]
    right = _transform(left, np.linalg.inv(expected))
    initial = expected.copy()
    initial[:3, 3] += world_rotation[:, 2]*line_direction_offset_m
    result = preview.refine_preview(left, right, initial)
    assert_safe_result(result)
    assert result['status'] == 'DEGENERATE'
    assert result['reasons'] == ['SURFACE_NORMAL_DIRECTIONS_DO_NOT_CONSTRAIN_3D_TRANSLATION']
    first = result['history'][0]
    assert first['normal_direction_eigenvalues'][0] < result['policy']['minimum_normal_direction_eigenvalue']
    # No spurious motion along the unsupported intersection direction is accepted.
    assert np.array_equal(result['refined_T_left_right'], initial)


@pytest.mark.parametrize('empty', [[], np.empty((0, 3))])
def test_empty_scene_has_json_safe_absent_residual_metrics(empty):
    result = preview.refine_preview(empty, [[1, 2, 3]], np.eye(4))
    assert_safe_result(result)
    assert result['status'] == 'INSUFFICIENT_OVERLAP'
    assert result['final_metrics']['nn_rmse_m'] is None
    assert result['correspondences'] == []


def test_distant_wrong_initial_transform_aborts_without_unbounded_motion():
    left = geometry(count=100)
    initial = np.eye(4); initial[0, 3] = 20
    result = preview.refine_preview(left, left, initial)
    assert_safe_result(result)
    assert result['status'] == 'INSUFFICIENT_OVERLAP'
    assert result['correspondences'] == []
    assert np.array_equal(result['refined_T_left_right'], initial)


@pytest.mark.parametrize('points', [[[float('nan'), 0, 0]], [[float('inf'), 0, 0]],
    [[1, 2]], [1, 2, 3], [['1', '2', '3']], [[True, False, True]], [[1001., 0, 0]],
    np.zeros((20001, 3))])
def test_invalid_input_points_are_rejected(points):
    with pytest.raises(ValueError):
        preview.refine_preview(points, [[1, 2, 3]], np.eye(4))


@pytest.mark.parametrize('kind', ['scale', 'reflection', 'nan', 'shape', 'bottom_row', 'huge_translation'])
def test_invalid_transform_never_fits_scale_or_reflection(kind):
    T = np.eye(4)
    if kind == 'scale': T[0, 0] = 1.01
    elif kind == 'reflection': T[0, 0] = -1
    elif kind == 'nan': T[0, 3] = np.nan
    elif kind == 'shape': T = np.eye(3)
    elif kind == 'bottom_row': T[3, 0] = .1
    else: T[0, 3] = 1e300
    with pytest.raises(ValueError):
        preview.refine_preview([[1, 2, 3]], [[1, 2, 3]], T)


def test_evaluation_and_optimizer_are_deterministic_for_frozen_input():
    left = geometry(count=150)
    T = known_transform()
    right = _transform(left, np.linalg.inv(T))
    a = preview.refine_preview(left, right, T)
    b = preview.refine_preview(left, right, T)
    assert a == b
    assert_safe_result(a)


def test_exhausted_iteration_budget_is_not_called_convergence(monkeypatch):
    levels = [dict(level, iterations=1) for level in preview.POLICY['levels']]
    monkeypatch.setitem(preview.POLICY, 'levels', levels)
    left = geometry()
    expected = known_transform()
    right = _transform(left, np.linalg.inv(expected))
    initial = _se3_exp(np.array([.018, -.014, .022, .038, -.025, .03]))@expected
    result = preview.refine_preview(left, right, initial)
    assert_safe_result(result)
    assert result['status'] == 'NOT_CONVERGED'
    assert result['reasons'] == ['FINEST_LEVEL_DID_NOT_CONVERGE_WITHIN_BUDGET']
