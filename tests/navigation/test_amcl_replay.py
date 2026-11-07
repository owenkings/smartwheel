import copy
import math
import os

import numpy as np
import pytest

from wc_navigation.amcl_replay import (AMCLReplayError, SCAN_SETTINGS, build_scan,
                                       prepare_frames, run_amcl_replay, validate_isolation,
                                       base_pose_from_lidar, planar_base_from_lidar)


def points():
    angles = np.array([-math.pi / 3, -.3, 0, .3, math.pi / 3])
    return np.column_stack([3 * np.cos(angles), 3 * np.sin(angles), np.zeros(len(angles))])


def frame(stamp):
    return {'stamp_ns': stamp, 'points': points(), 'odometry_T': np.eye(4)}


def test_planar_lever_arm_recovers_fixed_axle_from_turning_lidar_arc():
    fixed = np.eye(4)
    fixed[:2, 3] = [.323, .22875]
    lidar_positions = []
    for yaw in [0, .4, 1.1, 2.4, -2.7]:
        axle = np.eye(4)
        axle[:2, :2] = [[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]]
        axle[:2, 3] = [4, -2]
        lidar = axle @ fixed
        original = lidar.copy()
        recovered = base_pose_from_lidar(lidar, fixed)
        np.testing.assert_allclose(recovered, axle, atol=1e-12)
        np.testing.assert_array_equal(lidar, original)
        np.testing.assert_array_equal(base_pose_from_lidar(lidar), lidar)
        lidar_positions.append(lidar[:2, 3])
    assert np.linalg.norm(lidar_positions[0] - lidar_positions[3]) > .5


def test_lever_arm_never_silently_discards_z_or_roll_pitch():
    fixed = np.eye(4)
    fixed[2, 3] = .5915
    with pytest.raises(ValueError, match='planar SE'):
        planar_base_from_lidar(fixed)
    tilted = np.eye(4)
    roll = .02
    tilted[1:3, 1:3] = [[math.cos(roll), -math.sin(roll)], [math.sin(roll), math.cos(roll)]]
    with pytest.raises(ValueError, match='planar SE'):
        planar_base_from_lidar(tilted)
    yaw_only = np.eye(4)
    yaw_only[:2, :2] = [[0, -1], [1, 0]]
    np.testing.assert_array_equal(planar_base_from_lidar(yaw_only), yaw_only)


def test_nearest_actual_hit_and_missing_infinite_bins():
    source = np.vstack([points(), [1, 0, 0], [7, 0, 0]])
    original = source.copy()
    ranges, stats = build_scan(source)
    assert len(ranges) == 161 and ranges[80] == pytest.approx(1)
    assert stats['observed_bins'] == 5 and stats['missing_bins'] == 156
    assert np.isinf(ranges[79]) and not np.isnan(ranges).any()
    np.testing.assert_array_equal(source, original)


def test_height_range_and_fov_rejections_do_not_become_far_free_rays():
    source = np.array([[2, 0, -.7635], [2, 0, .8], [.1, 0, 0], [9, 0, 0],
                       [-2, 0, 0], [np.nan, 0, 0], [2, 0, 0]])
    ranges, stats = build_scan(source)
    assert stats['accepted_rows'] == 1 and stats['observed_bins'] == 1
    assert ranges[80] == 2 and np.isinf(ranges).sum() == 160
    assert SCAN_SETTINGS['per_point_acquisition_timing_known'] is False


def test_explicit_narrow_height_omits_low_self_return_but_keeps_further_wall():
    source = np.array([[.65, 0, -.46], [3, 0, 0], [2, 1, .1], [2, -1, -.1]])
    default, default_stats = build_scan(source)
    narrow, stats = build_scan(source, height_min_m=-.2, height_max_m=.2)
    assert default[80] == pytest.approx(.65) and narrow[80] == pytest.approx(3)
    assert stats['accepted_rows'] == default_stats['accepted_rows'] - 1
    assert stats['observed_bins'] == default_stats['observed_bins']
    assert np.isinf(narrow).sum() == 161 - stats['observed_bins']
    assert stats['scan_settings']['height_min_m'] == -.2
    assert stats['scan_settings']['height_max_m'] == .2
    assert SCAN_SETTINGS['height_min_m'] == -.5 and SCAN_SETTINGS['height_max_m'] == .5
    prepared, _ = prepare_frames([frame(10), frame(20)], np.eye(4), scan_height_m=(-.2, .2))
    assert all(item['scan_stats']['scan_settings']['height_min_m'] == -.2 for item in prepared)


@pytest.mark.parametrize('bounds', [(-2, .2), (-.2, 2), (.2, -.2), (0, 0),
                                    (float('nan'), .2), (-.2, float('inf')), (False, .2)])
def test_height_band_is_explicit_finite_and_ordered(bounds):
    with pytest.raises(ValueError):
        build_scan(points(), *bounds)


@pytest.mark.parametrize('value', [None, [], [[1, 2]], np.zeros((100001, 3)), [['bad', 0, 0]]])
def test_malformed_or_unbounded_scan_rejected(value):
    with pytest.raises(ValueError):
        build_scan(value)


def test_original_integer_clock_and_rigid_odometry_are_required():
    source = [frame(1_000_000_000), frame(1_600_000_000)]
    prepared, initial = prepare_frames(source, np.eye(4))
    assert [x['stamp_ns'] for x in prepared] == [1_000_000_000, 1_600_000_000]
    np.testing.assert_array_equal(initial, np.eye(4))
    for bad in [[frame(10), frame(10)], [frame(20), frame(10)],
                [frame(True), frame(30)], [frame(1.0), frame(30)]]:
        with pytest.raises(ValueError):
            prepare_frames(bad, np.eye(4))
    malformed = frame(30)
    malformed['odometry_T'][0, 0] = 2
    with pytest.raises(ValueError):
        prepare_frames([frame(10), malformed], np.eye(4))


def test_too_sparse_projection_fails_before_ros_or_output(tmp_path):
    bad = frame(20)
    bad['points'] = np.array([[2, 0, 0]])
    with pytest.raises(ValueError, match='three observed'):
        run_amcl_replay(tmp_path / 'missing.yaml', [frame(10), bad], np.eye(4), tmp_path / 'output')
    assert not (tmp_path / 'output').exists()


@pytest.mark.skipif(os.name != 'posix', reason='target isolation gate')
def test_only_private_local_domain_95_is_allowed(monkeypatch):
    monkeypatch.setenv('ROS_DOMAIN_ID', '83')
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    monkeypatch.delenv('ROS_DISCOVERY_SERVER', raising=False)
    with pytest.raises(AMCLReplayError):
        validate_isolation()
    monkeypatch.setenv('ROS_DOMAIN_ID', '95')
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '0')
    with pytest.raises(AMCLReplayError):
        validate_isolation()
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    monkeypatch.setenv('ROS_DISCOVERY_SERVER', '192.0.2.1:11811')
    with pytest.raises(AMCLReplayError):
        validate_isolation()
    monkeypatch.delenv('ROS_DISCOVERY_SERVER')
    validate_isolation()


def test_unknown_laser_model_rejected_before_ros_or_output(tmp_path):
    with pytest.raises(ValueError, match='unsupported AMCL laser model'):
        run_amcl_replay(tmp_path / 'missing.yaml', [], np.eye(4), tmp_path / 'output', laser_model='unknown')
    assert not (tmp_path / 'output').exists()


def test_amcl_overrides_are_explicit_typed_model_parameters():
    from wc_navigation.amcl_replay import amcl_parameter_overrides
    assert amcl_parameter_overrides(None) == {}
    values = amcl_parameter_overrides({'max_beams': 60.0, 'alpha1': 0, 'do_beamskip': False, 'z_hit': .5})
    assert values == {'max_beams': 60, 'alpha1': 0.0, 'do_beamskip': False, 'z_hit': .5}
    assert type(values['max_beams']) is int and type(values['alpha1']) is float
    for bad in [{'global_frame_id': 'map'}, {'scan_topic': 'scan'}, {'laser_model_type': 'beam'},
                {'use_sim_time': False}, {'max_beams': 1.5}, {'alpha1': -1}, {'alpha1': float('nan')},
                {'do_beamskip': 1}, {'max_beams': True}, {'min_particles': 900, 'max_particles': 500}, []]:
        with pytest.raises(ValueError):
            amcl_parameter_overrides(bad)


def test_initial_standard_deviations_default_and_bounds():
    from wc_navigation.amcl_replay import initial_standard_deviations
    assert initial_standard_deviations(None) == pytest.approx((.20, .20, math.radians(5.0)))
    assert initial_standard_deviations([.25, .25, .15]) == (.25, .25, .15)
    for bad in [[.25, .25], [0, .2, .1], [3, .2, .1], [.2, .2, 4], [True, .2, .1], 'abc', [float('inf'), .2, .1]]:
        with pytest.raises(ValueError):
            initial_standard_deviations(bad)


@pytest.mark.parametrize('keyword', [{'update_policy': 'SOMETIMES'}, {'amcl_overrides': {'odom_frame_id': 'x'}},
                                     {'initial_std': [1, 1]}, {'evaluation_scope': 'CROSS'}])
def test_new_options_fail_before_ros_or_output(tmp_path, keyword):
    with pytest.raises(ValueError):
        run_amcl_replay(tmp_path / 'missing.yaml', [frame(10), frame(20)], np.eye(4), tmp_path / 'output', **keyword)
    assert not (tmp_path / 'output').exists()
