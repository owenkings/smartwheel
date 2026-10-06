"""Synthetic-only exploration tests; none of these fixtures are device evidence."""
import copy
import json

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from wc_calibration import core
from wc_calibration.exploratory import explore, main, _input


def prepared(left, right):
    identities = {'left': 'SYN-L', 'right': 'SYN-R'}
    metadata = {side: {'raw_key': ['SYN-SESSION', identity, 'epoch', 1],
        'sensor_id': identity, 'units': 'm', 'coordinate_convention': 'FLU',
        'source_config_hash': 'a'*64, 'cloud_data_sha256': 'b'*64,
        'common_time_valid': False, 'time_source': 'arrival_only'} for side, identity in identities.items()}
    return {'schema_version': 1, 'status': 'PREPARED_NOT_VALIDATED', 'source_mode': 'synthetic',
        'sensor_ids': identities, 'units': 'm', 'coordinate_conventions': {'left': 'FLU', 'right': 'FLU'},
        'training': [{'id': 'SYN-CABINET-ONE-SCENE', 'left': left.tolist(), 'right': right.tolist(),
                      'selected_raw_frames': metadata, 'pair_dt_ns': 1234,
                      'time_quality': 'ARRIVAL_ONLY_NOT_SYNCHRONIZED'}], 'validation': [],
        'initial_T_left_right': None,
        'input_preparation': {'kind': 'rosbag2_source_frame', 'bag_uri': 'SYNTHETIC-ONLY',
                              'static_review_required': True, 'time_basis': 'host_receive'}}


@pytest.fixture(scope='module')
def scene():
    rng = np.random.default_rng(47013)
    yz = rng.uniform([-.9, -.55], [1.2, 1.05], (450, 2))
    xz = rng.uniform([.2, -.55], [2.9, 1.05], (500, 2))
    xy = rng.uniform([.2, -.9], [2.9, 1.2], (600, 2))
    left = np.vstack((np.column_stack((np.full(len(yz), 3.15), yz)),
        np.column_stack((xz[:, 0], np.full(len(xz), 1.4), xz[:, 1])),
        np.column_stack((xy, np.full(len(xy), -.73)))))
    transform = np.eye(4)
    transform[:3, :3] = Rotation.from_euler('xyz', [.073, -.051, .11]).as_matrix()
    transform[:3, 3] = [.27, -.19, .087]
    right = (left-transform[:3, 3]) @ transform[:3, :3]
    return prepared(left, right), transform


@pytest.fixture(scope='module')
def solved(scene):
    return explore(scene[0])


def no_promotion(result):
    assert result['kind'] == 'single_scene_exploration'
    assert result['status'] == 'EXPLORATION_ONLY'
    for field in ('live_eligible', 'independent_validation_performed', 'time_validated',
                  'ground_validated', 'navigation_validated', 'global_uniqueness_validated', 'scale_fitted'):
        assert result[field] is False
    assert 'T_left_right' not in result and 'calibration_id' not in result


def test_nonplanar_recovery_preserves_hypotheses_and_dual_diagnostics(scene, solved):
    no_promotion(solved)
    assert solved['computation_status'] == 'CANDIDATES_AVAILABLE'
    assert len(solved['hypotheses']) == 25
    assert sum(h['selected_for_refinement'] for h in solved['hypotheses']) == 6
    assert 1 <= len(solved['candidates']) <= 6
    best = solved['candidates'][0]
    transform = np.asarray(best['candidate_T_left_right'])
    delta = transform @ np.linalg.inv(scene[1])
    assert np.linalg.norm(delta[:3, 3]) < .02, best
    assert Rotation.from_matrix(delta[:3, :3]).magnitude() < .01
    assert np.allclose(transform @ best['candidate_T_right_left'], np.eye(4))
    assert best['diagnostics']['information']['observable']
    assert best['diagnostics']['reverse_information']['observable']
    assert len(best['diagnostics']['stages']) == 3
    assert best['diagnostics']['forward'][0]['overlap_ratio'] > .9
    assert best['diagnostics']['backward'][0]['overlap_ratio'] > .9
    assert solved['input_provenance']['pair_dt_ns'] == 1234
    assert solved['input_provenance']['time_quality'] == 'ARRIVAL_ONLY_NOT_SYNCHRONIZED'
    json.dumps(solved, allow_nan=False)


def test_formal_calibrate_still_rejects_single_scene(scene):
    with pytest.raises(core.CalibrationError, match='two training'):
        core.calibrate(scene[0])


def test_real_source_and_user_labels_cannot_promote(scene):
    data = copy.deepcopy(scene[0])
    data['source_mode'] = 'real'
    data['provenance'] = {'status': 'VALIDATED', 'live_eligible': True, 'installation_id': 'claimed'}
    no_promotion(explore(data))


def test_single_plane_reports_weak_directions_even_with_small_residual():
    xy = np.array(list(__import__('itertools').product(np.linspace(-1, 1, 22), np.linspace(-.7, .7, 17))))
    points = np.column_stack((xy, np.full(len(xy), 2.0)))
    result = explore(prepared(points, points.copy()))
    no_promotion(result)
    assert result['candidates']
    assert all(not c['geometry_usable'] for c in result['candidates'])
    best = result['candidates'][0]
    assert best['diagnostics']['forward'][0]['abs_residual_quantiles_m']['p95'] < 1e-8
    assert not best['diagnostics']['information']['observable']
    assert best['diagnostics']['information']['weak_directions']
    assert 'forward:DEGENERATE_GEOMETRY' in best['rejection_reasons']


def test_symmetric_box_retains_distinct_near_optimal_poses():
    grid = np.linspace(-1, 1, 15)
    faces = []
    for axis, extent in enumerate((1.2, .8, .5)):
        for sign in (-1, 1):
            for a in grid:
                for b in grid:
                    xyz = [0., 0., 0.]
                    xyz[axis] = sign*extent
                    other = [i for i in range(3) if i != axis]
                    xyz[other[0]] = a*(1.2, .8, .5)[other[0]]
                    xyz[other[1]] = b*(1.2, .8, .5)[other[1]]
                    faces.append(xyz)
    points = np.unique(faces, axis=0)
    result = explore(prepared(points, points.copy()))
    no_promotion(result)
    assert result['ambiguity']['near_optimal_distinct']
    assert any(c['near_optimal'] and c['rotation_difference_rad'] > 1 for c in result['ambiguity']['comparisons'])


@pytest.mark.parametrize('fault', ['status', 'source_mode', 'ids', 'units', 'axes', 'nan', 'infinity',
    'training', 'validation', 'initial', 'metadata', 'hash', 'session', 'budget', 'solver_budget',
    'ids_type', 'scene_type', 'metadata_type', 'preparation_type', 'schema_bool'])
def test_invalid_or_unbounded_inputs_are_rejected(scene, fault):
    data = copy.deepcopy(scene[0])
    if fault == 'status': data['status'] = 'VALIDATED'
    elif fault == 'source_mode': data.pop('source_mode')
    elif fault == 'ids': data['sensor_ids']['right'] = data['sensor_ids']['left']
    elif fault == 'units': data['units'] = 'mm'
    elif fault == 'axes': data['coordinate_conventions']['right'] = 'RDF'
    elif fault in ('nan', 'infinity'): data['training'][0]['left'][0][0] = float('nan' if fault == 'nan' else 'inf')
    elif fault == 'training': data['training'] *= 2
    elif fault == 'validation': data['validation'] = [data['training'][0]]
    elif fault == 'initial': data['initial_T_left_right'] = np.eye(4).tolist()
    elif fault == 'metadata': data['training'][0].pop('selected_raw_frames')
    elif fault == 'hash': data['training'][0]['selected_raw_frames']['left']['cloud_data_sha256'] = ''
    elif fault == 'session': data['training'][0]['selected_raw_frames']['right']['raw_key'][0] = 'OTHER'
    elif fault == 'budget': data['exploration_policy'] = {'max_input_points_per_side': 80, 'max_solver_points_per_side': 80}
    elif fault == 'ids_type': data['sensor_ids'] = ['left', 'right']
    elif fault == 'scene_type': data['training'] = [None]
    elif fault == 'metadata_type': data['training'][0]['selected_raw_frames']['left'] = None
    elif fault == 'preparation_type': data['input_preparation'] = None
    elif fault == 'schema_bool': data['schema_version'] = True
    else: data['exploration_policy'] = {'max_solver_points_per_side': 20001}
    with pytest.raises(core.CalibrationError):
        explore(data)


def test_direct_prepare_metadata_and_deterministic_sampling(scene):
    data = copy.deepcopy(scene[0])
    data.pop('units'); data.pop('coordinate_conventions')
    data['exploration_policy'] = {'max_solver_points_per_side': 160}
    first, _, counts, _ = _input(data, core.QualityProfile())
    data['training'][0]['left'].reverse()
    second, _, _, _ = _input(data, core.QualityProfile())
    assert counts['left']['solver_count'] == 160
    assert np.array_equal(first['left'], second['left'])


def test_numerical_unavailability_is_saved_without_silent_identity(tmp_path):
    points = np.tile([1., 2., 3.], (100, 1))
    source = tmp_path/'input.json'
    source.write_text(json.dumps(prepared(points, points)))
    assert main(['--input', str(source), '--output-root', str(tmp_path/'output'), '--version', 'v1']) == 2
    result = json.loads((tmp_path/'output/v1/result.json').read_text())
    no_promotion(result)
    assert result['computation_status'] == 'UNAVAILABLE'
    assert result['candidates'] == [] and result['best_candidate_id'] is None
    assert len(result['hypotheses']) == 25 and result['numerical_error']
    assert main(['--input', str(source), '--output-root', str(tmp_path/'output'), '--version', 'v1']) == 2


def test_bounded_candidate_count_and_no_source_mutation(scene):
    data = copy.deepcopy(scene[0])
    data['exploration_policy'] = {'refine_candidates': 4}
    before = json.dumps(data, sort_keys=True)
    result = explore(data)
    assert sum(h['selected_for_refinement'] for h in result['hypotheses']) == 4
    assert len(result['candidates']) <= 4
    assert json.dumps(data, sort_keys=True) == before
