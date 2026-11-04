"""Hardware-free map provenance, observability and causal-replay checks."""
import json
import math

import numpy as np
import pytest

from wc_runtime.offline_planar_registration import se2
from wc_navigation.replay_localization import build_wall_map, localize_replay


def room_walls():
    # XYZ walls with sufficient spatial height and perpendicular normals.
    horizontal = np.arange(-3.0, 3.001, 0.055)
    vertical = np.arange(-0.45, 0.451, 0.055)
    along, height = np.meshgrid(horizontal, vertical)
    walls = []
    for y in [-3.0, 3.0]:
        walls.append(np.column_stack([along.ravel(), np.full(along.size, y), height.ravel()]))
    for x in [-3.0, 3.0]:
        walls.append(np.column_stack([np.full(along.size, x), along.ravel(), height.ravel()]))
    return np.concatenate(walls)


def frame(stamp, points, pose=None):
    return {'stamp_ns': stamp, 'points': points,
            'odometry_T': np.eye(4) if pose is None else pose}


def scan_at(world, pose):
    return (world - pose[:3, 3]) @ pose[:3, :3]


def training_map():
    return build_wall_map([frame(1_000_000_000, room_walls())])


def test_training_map_preserves_height_geometry_and_filters_only_declared_band():
    walls = room_walls()
    points = np.vstack([walls, [[0, 0, -0.7635], [1, 0, 2.0]]])
    original = points.copy()
    result = build_wall_map([frame(1, points, se2(2, 1, 0.2))])
    assert result['training_stamps_ns'] == [1]
    assert result['summary']['retained_height_points'] == len(walls)
    assert np.ptp(result['points'][:, 2]) > 0.8
    assert result['summary']['query_frames_added'] is False
    assert result['summary']['navigation_validated'] is False
    np.testing.assert_array_equal(points, original)


def test_corner_geometry_recovers_explicit_perturbed_initial_pose():
    wall = training_map()
    true_pose = se2(0.0, 0.0, 0.0)
    queries = [frame(2_000_000_000, scan_at(room_walls(), true_pose), true_pose)]
    result = localize_replay(wall['points'], wall['training_stamps_ns'], queries,
                             se2(0.15, 0.10, math.radians(3)))
    assert result['summary']['matched_frames'] == 1
    estimated = np.asarray(result['frames'][0]['estimated_T'])
    assert np.linalg.norm(estimated[:2, 3]) < 0.02
    assert abs(math.atan2(estimated[1, 0], estimated[0, 0])) < math.radians(0.5)
    assert result['frames'][0]['pose_covariance'] is None
    assert result['reference_pose_feedback'] is False
    json.dumps(result, allow_nan=False)


def test_parallel_corridor_walls_reject_unobservable_axis():
    along, height = np.meshgrid(np.arange(-6, 6.001, 0.055), np.arange(-0.45, 0.451, 0.055))
    wall = np.concatenate([np.column_stack([along.ravel(), np.full(along.size, y), height.ravel()])
                           for y in [-2., 2.]])
    model = build_wall_map([frame(1, wall)])
    result = localize_replay(model['points'], [1], [frame(2, wall)], se2(0.15, 0.10, math.radians(2)))
    row = result['frames'][0]
    assert row['status'] == 'ODOM_ONLY'
    assert any('DEGENERACY' in reason for reason in row['rejection_reasons'])
    assert row['localization_available'] is False


def test_forward_single_lidar_views_follow_causal_odometry_without_reseeding():
    model = training_map()
    world = room_walls()
    queries = []
    for index in range(3):
        pose = se2(-.3 + .15 * index, .1, .01 * index)
        scan = scan_at(world, pose)
        angle = np.abs(np.arctan2(scan[:, 1], scan[:, 0]))
        scan = scan[angle <= math.radians(57.5)]
        queries.append(frame(2_000_000_000 + index * 200_000_000, scan, pose))
    perturbation = se2(.15, .1, math.radians(3))
    result = localize_replay(model['points'], model['training_stamps_ns'], queries,
                             queries[0]['odometry_T'] @ perturbation)
    assert result['summary']['matched_frames'] == 3
    for row, truth in zip(result['frames'], queries):
        estimated = np.asarray(row['estimated_T'])
        assert np.linalg.norm(estimated[:2, 3] - truth['odometry_T'][:2, 3]) < .03
    assert result['global_relocalization_performed'] is False


def test_training_and_query_overlap_is_fatal_even_before_matching():
    wall = training_map()
    with pytest.raises(ValueError, match='overlap'):
        localize_replay(wall['points'], [1_000_000_000], [frame(1_000_000_000, room_walls())], np.eye(4))


def test_interleaved_temporal_split_is_rejected():
    wall = training_map()
    with pytest.raises(ValueError, match='contiguous'):
        localize_replay(wall['points'], [1, 3], [frame(2, room_walls())], np.eye(4))


def test_reference_poison_cannot_change_estimates_or_match_states():
    wall = training_map()
    queries = [frame(2_000_000_000, room_walls()), frame(2_200_000_000, room_walls())]
    initial = se2(.15, .10, math.radians(3))
    regular = localize_replay(wall['points'], wall['training_stamps_ns'], queries, initial,
                              reference_poses={row['stamp_ns']: np.eye(4) for row in queries})
    poison = localize_replay(wall['points'], wall['training_stamps_ns'], queries, initial,
                             reference_poses={row['stamp_ns']: se2(900, -800, 2.0) for row in queries})
    assert regular['frames'] == poison['frames']
    assert regular['summary'] == poison['summary']
    assert regular['reference_evaluation'] != poison['reference_evaluation']


def test_input_gap_requires_explicit_reinitialization_and_never_uses_reference():
    wall = training_map()
    queries = [frame(2_000_000_000, room_walls()), frame(4_000_000_000, room_walls()),
               frame(4_200_000_000, room_walls())]
    result = localize_replay(wall['points'], wall['training_stamps_ns'], queries, np.eye(4),
                             reference_poses={row['stamp_ns']: np.eye(4) for row in queries})
    assert [row['status'] for row in result['frames']] == ['MATCHED', 'LOST', 'LOST']
    assert all(row['estimated_T'] is None for row in result['frames'][1:])
    assert result['outcome'] == 'LOST_AFTER_GAP'


def test_rejection_still_propagates_recorded_odometry_increment():
    wall = training_map()
    empty = np.empty((0, 3))
    queries = [frame(2, empty, se2(10, 20, .1)), frame(3, empty, se2(10.2, 20, .1))]
    initial = se2(1, 2, .3)
    result = localize_replay(wall['points'], wall['training_stamps_ns'], queries, initial)
    assert [row['status'] for row in result['frames']] == ['ODOM_ONLY', 'ODOM_ONLY']
    expected = initial @ np.linalg.inv(queries[0]['odometry_T']) @ queries[1]['odometry_T']
    np.testing.assert_allclose(result['frames'][1]['estimated_T'], expected, atol=1e-10)


@pytest.mark.parametrize('kind', ['nan', 'nonplanar', 'duplicate', 'bad_height', 'bad_voxel'])
def test_invalid_training_inputs_are_rejected(kind):
    points = room_walls()
    frames = [frame(1, points)]
    kwargs = {}
    if kind == 'nan':
        points[0, 0] = np.nan
    elif kind == 'nonplanar':
        frames[0]['odometry_T'][2, 3] = .1
    elif kind == 'duplicate':
        frames.append(frame(1, points))
    elif kind == 'bad_height':
        kwargs['min_height_m'], kwargs['max_height_m'] = 1., .5
    else:
        kwargs['voxel'] = 0
    with pytest.raises(ValueError):
        build_wall_map(frames, **kwargs)


def test_invalid_reference_is_only_an_evaluation_error():
    wall = training_map()
    queries = [frame(2_000_000_000, room_walls())]
    result = localize_replay(wall['points'], wall['training_stamps_ns'], queries, np.eye(4),
                             reference_poses={'not_a_stamp': 'not_a_pose'})
    assert result['frames'][0]['status'] == 'MATCHED'
    assert result['reference_evaluation']['status'] == 'INVALID_REFERENCE'
