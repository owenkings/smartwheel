import copy
import json
import math

import pytest


from wc_navigation import localization_trust_cli as MODULE


def fixtures(tmp_path):
    stamp = 1_000_000_000
    row = {'stamp_ns': stamp, 'base': {'pose_x_y_yaw': [.323, .22875, 0.],
           'observed_endpoints': 100, 'known_fraction': .95, 'supported_fraction': .92},
           'along_heading_ambiguity_evidence': False}
    mount = [[1., 0., 0., .323], [0., 1., 0., .22875], [0., 0., 1., 0.], [0., 0., 0., 1.]]
    trajectory = [{'stamp_ns': stamp, 'x_m': 0., 'y_m': 0., 'yaw_rad': 0.,
                   'covariance': [0.] * 36}]
    original = {'sources_unchanged': True, 'source_epoch_ns': 0, 'base_from_lidar': mount,
                'source_time': {'time_valid': False}, 'source_sha256': {},
                'localization_results': {n: {'base_from_lidar': mount, 'trajectory': copy.deepcopy(trajectory)}
                                         for n in ['likelihood_field', 'likelihood_field_prob']}}
    original_path = tmp_path / 'witnesses.json'
    original_path.write_text(json.dumps(original))
    contract = {'query_frames': 1, 'query_window_s': [1, 2], 'training_window_s': [3, 4],
                'segments_predeclared_s': [[1, 2]]}
    (tmp_path / 'coverage_evaluation.json').write_text(json.dumps(contract))
    for n in ['frozen_prior', 'likelihood_field', 'likelihood_field_prob']:
        (tmp_path / ('coverage_' + n + '.json')).write_text(json.dumps({'frames': [row]}))
    return original_path, original


def test_explicit_common_point_lever_arm_rotation():
    row = {'base': {'pose_x_y_yaw': [2. - .22875, 3. + .323, math.pi / 2]}}
    assert MODULE.common_axle_pose(row, [.323, .22875]) == pytest.approx([2., 3., math.pi / 2])


def test_saved_adapter_preserves_query_if_estimate_missing(tmp_path):
    original_path, original = fixtures(tmp_path)
    original['localization_results']['likelihood_field']['trajectory'] = []
    original_path.write_text(json.dumps(original))
    result = MODULE.evaluate(tmp_path, original_path)
    case = result['cases']['likelihood_field']
    assert len(case['frames']) == 1
    assert case['summary']['not_trusted_frames'] == 1
    assert 'ESTIMATE_MISSING' in case['frames'][0]['diagnostic_flags']
    assert result['all_query_frames_retained']
    assert result['inputs_unchanged']


def test_saved_adapter_refuses_pose_point_mismatch(tmp_path):
    original_path, original = fixtures(tmp_path)
    original['localization_results']['likelihood_field']['trajectory'][0]['x_m'] = .1
    original_path.write_text(json.dumps(original))
    with pytest.raises(ValueError, match='axle pose'):
        MODULE.evaluate(tmp_path, original_path)


def test_saved_adapter_duplicate_stamp_never_overwrites(tmp_path):
    original_path, original = fixtures(tmp_path)
    path = tmp_path / 'coverage_frozen_prior.json'
    content = json.loads(path.read_text())
    content['frames'] *= 2
    path.write_text(json.dumps(content))
    with pytest.raises(ValueError, match='duplicate'):
        MODULE.evaluate(tmp_path, original_path)


def test_saved_adapter_unexpected_stamp_never_nearest_matches(tmp_path):
    original_path, original = fixtures(tmp_path)
    original['localization_results']['likelihood_field']['trajectory'][0]['stamp_ns'] += 1
    original_path.write_text(json.dumps(original))
    with pytest.raises(ValueError, match='unexpected'):
        MODULE.evaluate(tmp_path, original_path)


def test_saved_adapter_unknown_near_witness_is_not_zero_evidence(tmp_path):
    original_path, _ = fixtures(tmp_path)
    result = MODULE.evaluate(tmp_path, original_path)
    row = result['cases']['likelihood_field']['frames'][0]
    assert row['near_return_witness']['status'] == 'NOT_EVALUATED_MISSING_WITNESSES'
    assert row['near_return_witness']['raw_inside_circle_count'] is None
    assert result['cases']['likelihood_field']['summary']['near_witness_missing_frames'] == 1


def test_saved_adapter_near_witness_exact_join_excludes_other_times(tmp_path):
    original_path, original = fixtures(tmp_path)
    original['frames'] = [{'stamp_ns': 999_999_999, 'raw_inside_circle_count': 10, 'overlap_voxel_count': 10},
                          {'stamp_ns': 1_000_000_000, 'raw_inside_circle_count': 0, 'overlap_voxel_count': 1}]
    original_path.write_text(json.dumps(original))
    result = MODULE.evaluate(tmp_path, original_path)
    summary = result['cases']['likelihood_field']['summary']
    assert summary['near_witness_exact_frames'] == 1
    assert summary['near_raw_overlap_frames'] == 0
    assert summary['near_voxel_overlap_frames'] == 1
