import pytest
from wc_navigation.amcl_cli import arguments


BASE = ['--project-root', '/tmp/project', '--source-cell', '/tmp/cell', '--map', '/tmp/map.yaml']


@pytest.mark.parametrize('extra', [
    ['--window', '70', '60'], ['--window', '-1', '60'],
    ['--interval', '-1'], ['--timeout', 'nan'], ['--initial-offset', '1', '0', '0'],
    ['--scan-height', '.2', '-.2'], ['--scan-height', '-3', '.2'],
])
def test_invalid_amcl_replay_requests_fail_before_ros(extra):
    with pytest.raises(SystemExit):
        arguments(BASE + extra)


def test_default_preserves_every_irregular_source_frame():
    from wc_navigation.replay_cli import select_rows
    args = arguments(BASE)
    assert args.interval == 0
    rows = [{'stamp_ns': 1000000000 + t} for t in [0, 199000000, 399000000, 700000000]]
    assert select_rows(rows, rows[0]['stamp_ns'], [0, 1], args.interval) == rows
    assert len(select_rows(rows, rows[0]['stamp_ns'], [0, 1], .55)) == 2


def test_optional_laser_model_is_explicit_and_bounded():
    assert arguments(BASE).laser_model == 'likelihood_field'
    assert arguments(BASE + ['--laser-model', 'likelihood_field_prob']).laser_model == 'likelihood_field_prob'
    with pytest.raises(SystemExit):
        arguments(BASE + ['--laser-model', 'unknown'])


def test_small_covariance_does_not_hide_cumulative_prior_disagreement():
    import numpy as np
    from wc_navigation.amcl_cli import prior_disagreement
    frames = [{'stamp_ns': 10, 'odometry_T': np.eye(4)}]
    result = {'trajectory': [{'stamp_ns': 10, 'x_m': 6., 'y_m': 0., 'yaw_rad': 0., 'covariance': [0.] * 36}]}
    quality = prior_disagreement(frames, result)
    assert quality['status'] == 'REVIEW_REQUIRED_ODOMETRY_DISAGREEMENT'
    assert quality['flagged_frames'] == 1
    assert quality['physical_navigation_validated'] is False


def test_disagreement_compares_same_axle_frame_and_wraps_yaw():
    import numpy as np
    import math
    from wc_navigation.amcl_cli import prior_disagreement
    fixed = np.eye(4); fixed[:2, 3] = [.323, .22875]
    base = np.eye(4); base[:2, :2] = [[-1, 0], [0, -1]]
    frames = [{'stamp_ns': 10, 'odometry_T': base @ fixed}]
    result = {'trajectory': [{'stamp_ns': 10, 'x_m': 0., 'y_m': 0., 'yaw_rad': -math.pi}]}
    quality = prior_disagreement(frames, result, fixed)
    assert quality['flagged_frames'] == 0 and quality['status'] == 'UNVALIDATED'
    assert quality['yaw_deg']['max'] < 1e-10
    result['trajectory'][0]['stamp_ns'] = 11
    with pytest.raises(ValueError, match='absent'):
        prior_disagreement(frames, result, fixed)
