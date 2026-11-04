import copy
import importlib.util
import math
from pathlib import Path

import numpy as np
import pytest


MODULE = Path(__file__).parents[2]/'src'/'wc_navigation'/'near_returns.py'
spec = importlib.util.spec_from_file_location('near_returns_under_test', MODULE)
near = importlib.util.module_from_spec(spec)
spec.loader.exec_module(near)


def observation(points, stamp=1):
    points = np.asarray(points, dtype=float).reshape(-1, 3)
    cells, counts = np.unique(np.floor(points[:, :2]/.1).astype(int), axis=0, return_counts=True)
    return {'stamp_ns': stamp, 'points_body_xy_lidar_z': points,
            'voxel_centers_body_xy': (cells+.5)*.1, 'voxel_counts': counts,
            'voxel_radius_m': .1/math.sqrt(2),
            'summary': {'voxel_m': .1, 'retained_rows': len(points), 'input_rows': len(points)+5,
                        'finite_rows': len(points)+3, 'ground_z_lidar_m': -.7635,
                        'near_range_height_candidate_rows': 2}}


def test_raw_and_voxel_only_are_separate_and_all_members_are_witnessed():
    # The .6-.7 x .2-.3 cell has a disc touching .65, even though its
    # original point is outside. The raw .4 point is inside.
    source = observation([[.4, .01, -.4], [.699, .299, .2]])
    before = copy.deepcopy(source)
    report = near.analyze_near_frame(source, context_radius_m=.65)
    assert report['raw_inside']['count'] == 1
    assert report['overlap_voxel_count'] == 2
    assert report['voxel_only_overlap_count'] == 1
    only = next(x for x in report['voxel_overlaps'] if x['voxel_only_overlap'])
    assert only['members'][0]['retained_point_index'] == 1
    assert only['members'][0]['inside_assumed_circle'] is False
    assert only['members'][0]['body_radius_m'] > .65
    assert len(report['context_points']) == 2  # outside context but overlaps voxel
    assert report['source_counts']['rows_not_retained_by_upstream_projection'] == 5
    assert report['source_counts']['near_range_height_candidate_rows'] == 2
    np.testing.assert_array_equal(source['points_body_xy_lidar_z'], before['points_body_xy_lidar_z'])
    assert report['points_removed'] == 0


def test_initial_gap_is_capture_geometry_not_braking_minimum():
    report = near.analyze_near_frame(observation([[.65, 0., -.5]]))
    cell = report['voxel_overlaps'][0]
    expected = math.hypot(.65, .05)-.65-.1/math.sqrt(2)
    assert cell['initial_capture_gap_m'] == pytest.approx(expected)
    assert report['raw_inside']['count'] == 1
    assert report['raw_inside']['candidate_floor_height_min_max_m'] == pytest.approx([.2635, .2635])
    assert report['raw_inside_low_height_rows'] == 0


def test_negative_voxel_uses_floor_and_preserves_lidar_z():
    report = near.analyze_near_frame(observation([[-.001, -.101, -.7]]))
    member = report['voxel_overlaps'][0]['members'][0]
    assert member['cell_xy'] == [-1, -2]
    assert member['body_xy_lidar_z_m'][2] == -.7
    assert member['height_above_candidate_floor_m'] == pytest.approx(.0635)
    assert report['raw_inside_low_height_rows'] == 1


def test_empty_is_diagnostic_without_clearance():
    report = near.analyze_near_frame(observation([]))
    assert report['raw_inside']['count'] == 0
    assert report['raw_inside']['body_radius_min_max_m'] is None
    assert report['overlap_voxel_count'] == 0
    assert report['clearance_proven'] is False


@pytest.mark.parametrize('change', ['centers', 'counts', 'ground', 'radius', 'nan'])
def test_inconsistent_projection_or_geometry_rejected(change):
    source = observation([[.4, .01, -.4]])
    kwargs = {}
    if change == 'centers': source['voxel_centers_body_xy'][0, 0] += .1
    if change == 'counts': source['voxel_counts'][0] += 1
    if change == 'ground': kwargs['ground_z_lidar'] = -.7
    if change == 'radius': source['voxel_radius_m'] = .1
    if change == 'nan': source['points_body_xy_lidar_z'][0, 0] = np.nan
    with pytest.raises(ValueError): near.analyze_near_frame(source, **kwargs)


def frames(poses, speeds=None, stamps=None):
    result = []
    for i, pose in enumerate(poses):
        report = near.analyze_near_frame(observation([[.4, .01, -.4]],
                                                   stamp=(stamps or [1, 1_000_000_001, 2_000_000_001])[i]))
        if pose is not None: report['pose_xy_yaw'] = pose
        if speeds is not None: report['velocity'] = {'forward_mps': speeds[i]}
        result.append(report)
    return result


def test_stationary_repetition_never_identifies_self_return():
    reports = frames([[0, 0, 0]]*3, [0, 0, 0])
    result = near.summarize_body_fixed_candidates(reports)
    assert result['candidate_cell_count'] == 0
    assert result['repeated_cells'][0]['label'] == 'REPEATED_BODY_CELL_INSUFFICIENT_DIVERSITY'
    assert result['self_return_identified'] is False


def test_motion_and_heading_and_time_diversity_only_yield_candidate():
    reports = frames([[0, 0, 0], [.3, 0, .3], [.6, 0, .6]], [.1, .1, .1])
    result = near.summarize_body_fixed_candidates(reports)
    assert result['candidate_cell_count'] == 1
    assert result['repeated_cells'][0]['self_return_proven'] is False
    assert result['causal_detector'] is False
    assert result['mask_generated'] is False


def test_wrap_crossing_is_small_heading_diversity():
    reports = frames([[0, 0, 3.1], [.3, 0, -3.12], [.6, 0, -3.1]], [.1, .1, .1])
    result = near.summarize_body_fixed_candidates(reports)
    assert result['repeated_cells'][0]['circular_yaw_span_rad'] < .1
    assert result['candidate_cell_count'] == 0


def test_same_xy_at_distinct_heights_does_not_merge_into_persistence():
    reports = []
    for i, z in enumerate([-.4, -.2, .2]):
        report = near.analyze_near_frame(observation([[.4, .01, z]], stamp=1+i*1_000_000_000))
        report['pose_xy_yaw'] = [.3*i, 0, .3*i]
        report['velocity'] = {'forward_mps': .1}
        reports.append(report)
    result = near.summarize_body_fixed_candidates(reports)
    assert result['candidate_cell_count'] == 0
    assert result['repeated_cells'] == []


def test_distant_pose_missing_frame_cannot_supply_pose_motion_time_diversity():
    reports = frames([[0, 0, 0], [.3, 0, .3], [.6, 0, .6]], [.1]*3,
                     [1, 100_000_001, 200_000_001])
    extra = near.analyze_near_frame(observation([[.4, .01, -.4]], stamp=2_000_000_001))
    reports.append(extra)
    result = near.summarize_body_fixed_candidates(reports)
    row = result['repeated_cells'][0]
    assert row['time_span_s'] == 2
    assert row['pose_available_time_span_s'] == pytest.approx(.2)
    assert result['candidate_cell_count'] == 0
    assert result['unknown_speed_frame_denominator'] == 1


@pytest.mark.parametrize('missing', ['poses', 'speeds', 'time', 'heading', 'translation'])
def test_missing_independent_diversity_requirement_blocks_candidate(missing):
    poses = [[0, 0, 0], [.3, 0, .3], [.6, 0, .6]]
    speeds = [.1]*3
    stamps = None
    if missing == 'poses': poses = [None]*3
    if missing == 'speeds': speeds = None
    if missing == 'time': stamps = [1, 100_000_001, 200_000_001]
    if missing == 'heading': poses = [[0, 0, 0], [.3, 0, 0], [.6, 0, 0]]
    if missing == 'translation': poses = [[0, 0, 0], [0, 0, .3], [0, 0, .6]]
    result = near.summarize_body_fixed_candidates(frames(poses, speeds, stamps))
    assert result['candidate_cell_count'] == 0


def test_out_of_order_reports_rejected():
    reports = frames([[0, 0, 0]]*3)
    reports[1]['stamp_ns'] = reports[0]['stamp_ns']
    with pytest.raises(ValueError): near.summarize_body_fixed_candidates(reports)
