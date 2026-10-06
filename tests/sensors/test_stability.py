"""Synthetic numerical fixtures only; never access a lidar, ROS or SDK."""

import json

import numpy as np
import pytest

from wc_sensors import stability


def sample(ranges=None):
    ranges = np.asarray([2., 2., 2., 2., 2.] if ranges is None else ranges)
    if ranges.ndim == 1:
        ranges = np.repeat(ranges[:, None], 9600, axis=1)
    xyz = np.zeros((*ranges.shape, 3), dtype=np.float32)
    xyz[:, :, 0] = ranges
    return {'raw_xyz': xyz, 'frame_sequence': np.arange(len(xyz), dtype=np.uint64),
            'host_ns': 10_000_000_000 + np.arange(len(xyz), dtype=np.int64)*100_000_000}


def test_constant_range_is_repeatable_but_never_accuracy_or_static_truth():
    result = stability.analyze(sample())
    raw = result['representations']['raw']
    assert raw['summary']['range_mad_m_unscaled']['median'] == 0
    assert raw['pixel_range_median_m'] == [2.] * 9600
    assert raw['common_valid_pixel_count'] == 9600
    assert raw['rowwise']['state'] == 'DISABLED_PIXEL_LAYOUT_NOT_PROVIDED'
    assert result['status'] == 'COMPUTED_NO_QUALITY_VERDICT'
    assert result['scene_assumption'] == 'UNVERIFIED'
    assert not result['accuracy_validated'] and not result['ground_truth_available']


def test_known_temporal_statistics_and_uniform_shift_have_no_spatial_correlation():
    result = stability.analyze(sample([1., 2., 3., 4., 5.]), height=60, width=160,
        scene_static_source='user_declared_no_ground_truth', run_label='dual_A_left')
    raw = result['representations']['raw']
    assert raw['pixel_range_median_m'][0] == 3
    assert raw['pixel_range_mad_m_unscaled'][0] == 1
    assert raw['pixel_range_p95_minus_p05_m'][0] == pytest.approx(3.6)
    assert raw['pixel_adjacent_abs_delta_p95_m'][0] == 1
    assert raw['frame_adjacent_median_delta_m'] == [1.] * 4
    assert raw['rowwise']['horizontal_neighbor_delta_correlation'] == [None] * 4
    assert raw['rowwise']['row_explained_delta_variance_fraction'] == [None] * 4
    assert result['scene_assumption'] == 'DECLARED_STATIC' and not result['optical_interference_diagnosed']


def test_range_is_invariant_to_xyz_rotation():
    arrays = sample()
    arrays['raw_xyz'][:] = [0, 1.2, 1.6]
    result = stability.analyze(arrays)['representations']['raw']
    assert result['pixel_range_median_m'][0] == pytest.approx(2, abs=1e-6)


def test_invalid_and_zero_pixels_are_not_replaced_with_fake_zero_noise():
    arrays = sample()
    arrays['raw_xyz'][:, 0] = np.nan
    arrays['raw_xyz'][:, 1] = 0
    arrays['raw_xyz'][:3, 2] = np.inf
    result = stability.analyze(arrays)['representations']['raw']
    assert result['pixel_valid_observations'][:3] == [0, 0, 2]
    assert result['pixel_range_mad_m_unscaled'][:3] == [None, None, None]
    assert result['pixel_range_median_m'][:3] == [None, None, None]
    assert result['common_valid_pixel_count'] == 9597


def test_all_invalid_data_returns_insufficient_and_json_never_nan():
    arrays = sample()
    arrays['raw_xyz'][:] = np.nan
    result = stability.analyze(arrays, height=60, width=160)
    assert result['status'] == 'INSUFFICIENT_VALID_DATA'
    assert result['representations']['raw']['frame_median_range_m'] == [None] * 5
    json.dumps(result, allow_nan=False)


def test_sequence_and_arrival_faults_exclude_cross_gap_adjacent_differences():
    arrays = sample([1., 2., 100., 4., 5.])
    arrays['frame_sequence'] = np.array([5, 6, 9, 9, 2], dtype=np.uint64)
    arrays['host_ns'] = np.array([100, 200, 300, 300, 400], dtype=np.uint64)
    result = stability.analyze(arrays)
    assert result['sequence']['missing_frames_between_samples'] == 2
    assert result['sequence']['duplicate_indices'] == [3]
    assert result['sequence']['backward_indices'] == [4]
    assert result['arrival']['nonincreasing_indices'] == [3]
    assert result['adjacent_pair_eligible'] == [True, False, False, False]
    assert result['representations']['raw']['pixel_adjacent_valid_pairs'][0] == 1
    assert result['representations']['raw']['pixel_adjacent_abs_delta_median_m'][0] is None


def test_uint64_device_counter_wrap_is_reported_without_subtraction_overflow():
    arrays = sample()
    arrays['device_raw'] = np.array([2**64-3, 2**64-2, 2**64-1, 0, 1], dtype=np.uint64)
    result = stability.analyze(arrays)
    assert result['device_time']['nonincreasing_indices'] == [3]
    assert result['device_time']['difference_raw_units'][2] == -(2**64-1)
    assert result['device_time']['common_time_validated'] is False


def test_row_band_variation_is_distinct_from_spatially_independent_noise():
    rng = np.random.default_rng(8317)
    band = np.tile(np.repeat(np.arange(60) % 2, 160), (7, 1)) * np.arange(7)[:, None] * .01 + 2
    coherent = stability.analyze(sample(band), height=60, width=160)['representations']['raw']['rowwise']
    independent = stability.analyze(sample(2+rng.normal(0, .01, (7, 9600))), height=60, width=160)['representations']['raw']['rowwise']
    assert coherent['row_explained_variance_summary']['median'] > .99
    assert coherent['horizontal_correlation_summary']['median'] > .99
    assert coherent['vertical_correlation_summary']['median'] < -.99
    assert independent['row_explained_variance_summary']['median'] < .03
    assert abs(independent['horizontal_correlation_summary']['median']) < .05


def test_mask_changes_are_visible_without_calling_frame_median_a_distance_error():
    ranges = np.repeat(np.r_[np.ones(4800), np.full(4800, 3.)][None, :], 5, axis=0)
    arrays = sample(ranges)
    arrays['raw_xyz'][2:, :4800] = np.nan
    raw = stability.analyze(arrays)['representations']['raw']
    assert raw['frame_median_range_m'] == [2., 2., 3., 3., 3.]
    assert raw['frame_common_pixel_median_range_m'] == [3.] * 5
    assert raw['summary']['range_mad_m_unscaled']['median'] == 0


def test_filtered_and_raw_are_separate_and_filtered_only_is_supported():
    arrays = sample([1., 2., 3., 4., 5.])
    arrays['filtered_xyz'] = sample()['raw_xyz']
    result = stability.analyze(arrays)
    assert result['representations']['raw']['summary']['range_mad_m_unscaled']['median'] == 1
    assert result['representations']['filtered']['summary']['range_mad_m_unscaled']['median'] == 0
    del arrays['raw_xyz']
    assert set(stability.analyze(arrays)['representations']) == {'filtered'}


@pytest.mark.parametrize('fault', ['compressed_pixels', 'too_few_frames', 'integer_xyz', 'float_sequence', 'missing_host', 'unequal_streams', 'mismatched_pair'])
def test_invalid_input_cannot_be_reported_as_valid_computation(fault):
    arrays = sample()
    if fault == 'compressed_pixels': arrays['raw_xyz'] = arrays['raw_xyz'][:, :8000]
    elif fault == 'too_few_frames': arrays['raw_xyz'] = arrays['raw_xyz'][:2]
    elif fault == 'integer_xyz': arrays['raw_xyz'] = arrays['raw_xyz'].astype(np.int16)
    elif fault == 'float_sequence': arrays['frame_sequence'] = arrays['frame_sequence'].astype(float)
    elif fault == 'missing_host': del arrays['host_ns']
    elif fault == 'unequal_streams': arrays['filtered_xyz'] = arrays['raw_xyz'][:3]
    elif fault == 'mismatched_pair': arrays['filtered_frame_sequence'] = arrays['frame_sequence'] + 1
    with pytest.raises((ValueError, KeyError)):
        stability.analyze(arrays)


@pytest.mark.parametrize('layout', [(1, 9600), (None, 160), (60, 80), (0, 0)])
def test_flattened_ros_cloud_is_not_accepted_as_a_real_row_layout(layout):
    with pytest.raises(ValueError, match='layout'):
        stability.analyze(sample(), height=layout[0], width=layout[1])


def test_cli_preserves_npz_writes_new_honest_report_and_refuses_overwrite(tmp_path):
    source, output = tmp_path/'synthetic.npz', tmp_path/'result.json'
    np.savez(source, **sample())
    before = source.read_bytes()
    assert stability.main(['--input', str(source), '--output', str(output)]) == 0
    result = json.loads(output.read_text())
    assert result['status'] == 'COMPUTED_NO_QUALITY_VERDICT' and len(result['input_sha256']) == 64
    assert source.read_bytes() == before
    with pytest.raises(FileExistsError):
        stability.main(['--input', str(source), '--output', str(output)])


def test_cli_missing_data_writes_error_and_exits_nonzero(tmp_path):
    output = tmp_path/'error.json'
    assert stability.main(['--input', str(tmp_path/'missing.npz'), '--output', str(output)]) == 1
    assert json.loads(output.read_text())['status'] == 'ERROR'
