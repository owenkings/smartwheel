import numpy as np

from wheelchair_3d_mapping.ground_plane_candidate import evaluate_floor_frames, fit_floor_plane


def floor(height=.735, tilt=0.0, seed=1):
    rng = np.random.default_rng(seed)
    x, z = rng.uniform(-1., 1., 600), rng.uniform(.5, 3., 600)
    y = (-height - np.sin(tilt) * z) / np.cos(tilt) + rng.normal(0, .003, 600)
    return np.column_stack((x, y, z))


def evaluate(frames, **kwargs):
    return evaluate_floor_frames(frames, stamps=[1.+i*.1 for i in range(len(frames))],
                                 frame_ids=['xtm60_right_link']*len(frames),
                                 iterations=60, **kwargs)


def test_height_is_perpendicular_distance_not_point_range():
    plane = fit_floor_plane(floor(tilt=.12), iterations=80)
    assert abs(plane['d']-.735) < .005
    assert plane['plane_y_at_sensor_xz_m'] < -.735


def test_stable_floor_candidate_requires_context_and_many_frames():
    frames = [floor(seed=i) for i in range(20)]
    unconfirmed = evaluate(frames)
    assert unconfirmed['status'] == 'REJECTED_OR_CONTEXT_UNCONFIRMED'
    result = evaluate(frames, floor_roi_confirmed=True, stationary_level_confirmed=True)
    assert result['status'] == 'CANDIDATE_REVIEW_REQUIRED'
    assert not result['automatic_apply_allowed'] and not result['formal_runtime_eligible']
    assert abs(result['height_median_m']-.735) < .005
    assert evaluate(frames[:3], floor_roi_confirmed=True, stationary_level_confirmed=True)['checks']['enough_frames'] is False


def test_wall_ceiling_and_degenerate_line_do_not_qualify():
    pts = floor()
    wall = pts[:, [1, 0, 2]]
    ceiling = pts * [1, -1, 1]
    line = pts.copy()
    line[:, 0] = 0
    assert fit_floor_plane(wall) is None
    assert fit_floor_plane(ceiling) is None
    assert evaluate([line]*20)['accepted_frames'] == 0


def test_changing_height_or_normal_is_rejected():
    heights = [floor(height=.735 + .01*i, seed=i) for i in range(20)]
    assert evaluate(heights)['checks']['height_stable'] is False
    angles = [floor(tilt=.015*i, seed=i) for i in range(20)]
    assert evaluate(angles)['checks']['normal_stable'] is False


def test_mixed_frames_or_nonmonotonic_timestamps_are_rejected():
    frames = [floor()]*20
    result = evaluate_floor_frames(frames, stamps=[1.]*20,
                                   frame_ids=['left', 'right']*10, iterations=50)
    assert result['checks']['monotonic_source_timestamps'] is False
    assert result['checks']['single_sensor_frame'] is False


def test_empty_and_nan_input_fail_closed():
    assert fit_floor_plane(np.full((100,3), np.nan)) is None
    assert evaluate_floor_frames([])['status'] == 'REJECTED_OR_CONTEXT_UNCONFIRMED'
