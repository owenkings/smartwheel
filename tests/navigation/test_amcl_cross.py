import math

import numpy as np
import pytest

from wc_navigation.live.amcl_cross import (CAP_M, EndpointScorer, arguments, convergence, door_segments,
                                           heading_components, heading_std, perturbed_start, profile_settings,
                                           se2, window_medians, xyyaw)


def corridor_grid():
    data = np.full((40, 60), -1, dtype=np.int8)
    data[10:30, :] = 0
    data[10, :] = 100
    data[29, :] = 100
    return data


def test_endpoint_fit_caps_distance_and_counts_outside_as_cap():
    scorer = EndpointScorer(corridor_grid(), 0.1, (0.0, 0.0, 0.0))
    on_wall = np.array([[1.05, 1.05], [2.05, 2.95]])
    fit, known = scorer.frame_fit(np.eye(3), on_wall)
    assert fit == pytest.approx(0.0) and known == 1.0
    near_and_outside = np.array([[1.05, 1.35], [100.0, 100.0]])   # 3 cells from the wall; off the map
    fit, known = scorer.frame_fit(np.eye(3), near_and_outside)
    assert fit == pytest.approx((0.3 + CAP_M) / 2)
    assert known == pytest.approx(0.5)
    fit, _ = scorer.frame_fit(np.eye(3), np.array([[1.05, 2.05]]))   # mid-corridor, 0.9 m from both walls
    assert fit == pytest.approx(CAP_M)


def test_endpoint_fit_uses_the_lidar_pose():
    scorer = EndpointScorer(corridor_grid(), 0.1, (0.0, 0.0, 0.0))
    points = np.array([[0.0, -0.95]])           # 0.95 m to the right of the sensor
    fit, _ = scorer.frame_fit(se2(2.05, 2.0, 0.0), points)
    assert fit == pytest.approx(0.0)
    fit, _ = scorer.frame_fit(se2(2.05, 2.0, math.pi / 2), points)
    assert fit == pytest.approx(CAP_M)


def test_scorer_requires_axis_aligned_trinary_map():
    with pytest.raises(ValueError):
        EndpointScorer(corridor_grid(), 0.1, (0.0, 0.0, 0.1))
    with pytest.raises(ValueError):
        EndpointScorer(np.zeros((5, 5), dtype=np.int8), 0.1, (0.0, 0.0, 0.0))


def test_door_segments_split_unmapped_start_and_end():
    values = np.array([-1, -2, 0, 0, -1, 0, 100, -1, -1])
    labels = door_segments(values)
    assert list(labels) == ["START_DOOR", "START_DOOR", "MIDDLE", "MIDDLE", "MIDDLE", "MIDDLE",
                            "END_DOOR", "END_DOOR", "END_DOOR"]
    assert set(door_segments(np.array([-1, -1]))) == {"START_DOOR"}


def test_window_medians_match_the_alignment_check_windows():
    times = np.array([0.0, 40.0, 40.1, 80.0, 80.1, 120.0])
    values = np.array([1.0, 3.0, 5.0, 7.0, 9.0, 11.0])
    result = window_medians(times, values)
    assert result == {"first_40s": 2.0, "40_80s": 6.0, "after_80s": 10.0}
    masked = window_medians(times, values, np.array([1, 1, 0, 1, 1, 0], dtype=bool))
    assert masked == {"first_40s": 2.0, "40_80s": 7.0, "after_80s": 9.0}


def test_heading_decomposition_and_covariance_rotation():
    along, across = heading_components(1.0, 0.0, math.pi / 2)
    assert along == pytest.approx(0.0, abs=1e-12) and across == pytest.approx(-1.0)
    covariance = [0.0] * 36
    covariance[0], covariance[7] = 0.04, 0.0025           # long in x, narrow in y
    assert heading_std(covariance, 0.0) == pytest.approx((0.2, 0.05))
    assert heading_std(covariance, math.pi / 2) == pytest.approx((0.05, 0.2))


def test_perturbation_is_in_the_start_axle_frame():
    start = se2(1.0, 2.0, math.pi / 2)
    x, y, yaw = xyyaw(perturbed_start(start, (0.5, 0.25, 10.0)))
    assert (x, y) == pytest.approx((0.75, 2.5))
    assert yaw == pytest.approx(math.pi / 2 + math.radians(10.0))


def rows(offsets, segments):
    return [{"t": float(k), "segment": s, "x": float(o), "y": 0.0, "yaw": 0.0}
            for k, (o, s) in enumerate(zip(offsets, segments))]


def test_convergence_joins_once_most_later_mapped_frames_stay_in_the_band():
    segments = ["START_DOOR"] + ["MIDDLE"] * 12 + ["END_DOOR"]
    reference = rows([0.0] * 14, segments)
    # one 0.3 m excursion after joining is tolerated (11 of 12 later frames inside 0.25 m)
    offsets = [0.9, 0.8, 0.6, 0.04, 0.05, 0.3, 0.02, 0.03, 0.05, 0.04, 0.06, 0.05, 0.03, 2.0]
    joined = convergence(rows(offsets, segments), reference)
    assert joined["status"] == "CONVERGED" and joined["joined_at_s"] == 3.0
    assert joined["after_join"]["median_xy_m"] == pytest.approx(0.045)
    assert joined["final_middle"]["xy_m"] == pytest.approx(0.03)
    loose = convergence(rows([0.9] + [0.2] * 12 + [0.0], segments), reference)
    assert loose["status"] == "NOT_CONVERGED" and loose["joined_at_s"] == 1.0
    lost = convergence(rows([0.0] * 12 + [1.5, 0.0], segments), reference)
    assert lost["status"] == "DIVERGED"
    never = convergence(rows([0.9] + [0.4] * 12 + [0.0], segments), reference)
    assert never["status"] == "NOT_CONVERGED" and never["joined_at_s"] is None


def test_a_join_cannot_precede_a_later_gross_error():
    segments = ["MIDDLE"] * 30
    reference = rows([0.0] * 30, segments)
    # 90 % of the frames after t=1 are close, but t=3..5 are 2 m off: the join is after the last gross error
    offsets = [0.05, 0.05, 0.05, 2.0, 2.0, 2.0] + [0.05] * 24
    result = convergence(rows(offsets, segments), reference)
    assert result["status"] == "CONVERGED" and result["joined_at_s"] == 6.0


def test_framework_profile_mirrors_the_navigation_amcl_parameters():
    from wc_navigation.autonomy import nav2_config
    params = nav2_config.localization_parameters(nav2_config.NavProfile(), "m.yaml")["amcl"]["ros__parameters"]
    settings = profile_settings("framework")
    assert settings["update_policy"] == "AMCL_MOTION_GATED_UPDATES"
    assert settings["laser_model"] == params["laser_model_type"]
    for key, value in settings["amcl_overrides"].items():
        assert params[key] == value
    assert settings["amcl_overrides"]["update_min_d"] == params["update_min_d"]
    assert "global_frame_id" not in settings["amcl_overrides"]
    assert profile_settings("replay_tool")["amcl_overrides"] == {}
    with pytest.raises(ValueError):
        profile_settings("other")


def test_arguments_bound_perturbations_and_keep_the_reference_case(tmp_path):
    base = ["--project-root", str(tmp_path), "--source-cell", "c", "--map", "m.yaml", "--alignment", "0", "1", "-85"]
    args = arguments(base + ["--perturbation", "0.5", "0", "0"])
    assert args.perturbation[0] == (0.0, 0.0, 0.0) and (0.5, 0.0, 0.0) in args.perturbation
    assert args.profile == ["framework", "replay_tool"] and args.perturb_profile == ["framework"]
    with pytest.raises(SystemExit):
        arguments(base + ["--perturbation", "3", "0", "0"])
    with pytest.raises(SystemExit):
        arguments(base[:-4] + ["--alignment", "0", "1", "-85", "--alignment-json", "a.json"])
