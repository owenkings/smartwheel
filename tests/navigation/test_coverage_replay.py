import numpy as np

from wc_navigation.amcl_cli import replay_map_coverage
from wc_navigation.amcl_replay import SCAN_SETTINGS
from wc_navigation.geometry import Grid
from wc_navigation.map_guard import write_guarded_map


def test_replay_coverage_uses_lidar_lever_arm_and_same_height_band(tmp_path):
    data = np.zeros((100, 240), dtype=np.int8)
    data[:, 160] = 100
    exported = write_guarded_map(Grid(data, .05), tmp_path / 'map', {})
    fixed = np.eye(4)
    fixed[:2, 3] = [.323, .22875]
    base = np.eye(4)
    base[:2, 3] = [5.025, 2.525]
    angles = np.arange(-20, 21) * (np.pi / 240)
    distances = (8.025 - (base @ fixed)[0, 3]) / np.cos(angles)
    points = np.column_stack([distances * np.cos(angles), distances * np.sin(angles),
                              np.zeros(len(angles))])
    points = np.vstack([points, [.5, 0., .8]])
    frames = [{'stamp_ns': 10, 'points': points, 'odometry_T': base @ fixed}]
    result = {'scan_settings': {**SCAN_SETTINGS, 'height_min_m': -.2, 'height_max_m': .2},
              'trajectory': [{'stamp_ns': 10, 'x_m': 5.025, 'y_m': 2.525, 'yaw_rad': 0.}]}
    report = replay_map_coverage(exported['map_yaml'], frames, result, fixed)
    assert report['summary']['status'] == 'UNVALIDATED'
    assert report['summary']['analyzed_frames'] == 1
    assert report['frames'][0]['base']['observed_endpoints'] == 41
    assert report['frames'][0]['base']['supported_fraction'] == 1
    np.testing.assert_allclose(report['frames'][0]['base']['pose_x_y_yaw'], [5.348, 2.75375, 0.])
    assert report['summary']['physical_navigation_validated'] is False


def test_no_returned_poses_requires_review_instead_of_empty_success(tmp_path):
    exported = write_guarded_map(Grid(np.zeros((10, 10), dtype=np.int8), .1), tmp_path / 'map', {})
    report = replay_map_coverage(exported['map_yaml'], [], {'scan_settings': SCAN_SETTINGS, 'trajectory': []})
    assert report['summary']['analyzed_frames'] == 0
    assert report['summary']['status'] == 'REVIEW_REQUIRED'
