import math

import numpy as np
import pytest

from wc_navigation.live.amcl_cross import (CAP_M, EndpointScorer, arguments, convergence, door_segments,
                                           heading_components, heading_std, motion_breakdown, motion_classes,
                                           perturbed_start, planar_from_rigid, profile_settings,
                                           rigid_from_planar, se2, shifted_odometry, window_medians, wrap,
                                           xyyaw)


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


def test_gate_variants_are_explicit_and_the_default_one_matches_the_gate():
    from wc_navigation.autonomy.safety_gate import CorrectionMonitor, GateConfig
    from wc_navigation.live.amcl_cross import GATE_VARIANTS
    current = GATE_VARIANTS["gate_default"]
    monitor, config = CorrectionMonitor(), GateConfig()
    assert current["reference"] == monitor.reference and current["translation_m"] == monitor.max_translation_m
    assert math.radians(current["yaw_deg"]) == pytest.approx(monitor.max_yaw_rad)
    assert current["cov_xy_m"] == config.covariance_xy_std_max_m
    assert GATE_VARIANTS["gate_until_20261107"] == {"reference": "odom_origin", "translation_m": 0.25,
                                                    "yaw_deg": 5.0, "cov_xy_m": 0.30}


def test_evaluate_run_flags_each_gate_variant_on_a_heading_correction_far_from_the_origin():
    from wc_navigation.live.amcl_cross import evaluate_run
    scorer = EndpointScorer(corridor_grid(), 0.1, (0.0, 0.0, 0.0))
    frames, trajectory = [], []
    for k in range(12):
        axle = se2(13.0, 2.0, 0.0)                       # standing 13 m from the odom origin
        frames.append({"stamp_ns": 1_000_000_000 + k * 300_000_000, "t": k * 0.3, "segment": "MIDDLE",
                       "axle": axle, "lidar": axle, "endpoints": np.array([[0.0, -0.95]])})
    covariance = [0.0] * 36
    covariance[0] = covariance[7] = 0.16                 # 0.4 m: fails 0.30, passes 0.50
    covariance[35] = 0.01
    rows = []
    theta = math.radians(1.5)
    c, s = math.cos(theta), math.sin(theta)
    # From frame 6 on AMCL turns the heading by 1.5 degrees about the robot itself: map->odom rotates and
    # shifts by p - R p (0.34 m), the robot's map position does not move.
    shifted = (13.0 - (c * 13.0 - s * 2.0), 2.0 - (s * 13.0 + c * 2.0))
    for k, frame in enumerate(frames):
        x, y, yaw = (0.0, 0.0, 0.0) if k < 6 else (*shifted, theta)
        rows.append({"stamp_ns": frame["stamp_ns"], "x_m": x, "y_m": y, "yaw_rad": yaw, "filter_updated": True})
        trajectory.append({"stamp_ns": frame["stamp_ns"], "covariance": covariance})
    result = {"map_odom_trajectory": rows, "trajectory": trajectory}
    table, extra = evaluate_run(result, frames, np.eye(3), scorer)
    after = table[6]
    assert after["change_2s_xy"] == pytest.approx(math.hypot(*shifted)) and after["change_2s_robot_xy"] < 1e-9
    assert after["gate_until_20261107_correction_ok"] is False and after["gate_default_correction_ok"] is True
    assert after["gate_until_20261107_covariance_ok"] is False and after["gate_default_covariance_ok"] is True
    assert extra["correction_jump_events"] == {"gate_until_20261107": 1, "gate_default": 0, "gate_lateral_trial": 0}


def track(lidar_poses, dt=0.3):
    return [{"stamp_ns": 1_000_000_000 + int(round(k * dt * 1e9)), "points": np.zeros((2, 3)),
             "odometry_T": rigid_from_planar(se2(*pose))} for k, pose in enumerate(lidar_poses)]


def test_lidar_time_offset_pairs_each_scan_with_the_earlier_odometry():
    # 0.5 m/s along x while turning at 1 rad/s through +-pi; frames every 0.3 s
    frames = track([(0.15 * k, 0.0, wrap(3.0 + 0.3 * k)) for k in range(6)])
    unchanged = shifted_odometry(frames, 0.0)
    assert all(a is b for a, b in zip(unchanged, frames)) and len(unchanged) == len(frames)
    shifted = shifted_odometry(frames, 75.0)
    for k, (new, old) in enumerate(zip(shifted, frames)):
        assert new["stamp_ns"] == old["stamp_ns"] and new["points"] is old["points"]
        x, y, yaw = xyyaw(planar_from_rigid(new["odometry_T"]))
        lag = 0.075 if k else 0.0                       # the first frame holds the first pose
        assert (x, y) == pytest.approx((0.15 * k - 0.5 * lag, 0.0), abs=1e-9)
        assert wrap(yaw - (3.0 + 0.3 * k - 1.0 * lag)) == pytest.approx(0.0, abs=1e-9)
    assert xyyaw(planar_from_rigid(frames[3]["odometry_T"]))[0] == pytest.approx(0.45)   # input untouched
    with pytest.raises(ValueError):
        shifted_odometry(frames, 300.0)
    with pytest.raises(ValueError):
        shifted_odometry(frames, math.nan)


def test_motion_classes_come_from_the_recorded_axle_motion():
    lever = (0.3, 0.2)
    axle = ([(0.0, 0.0, 0.0)] * 4 + [(0.15 * k, 0.0, 0.0) for k in range(1, 5)]
            + [(0.6, 0.0, 0.18 * k) for k in range(1, 5)])      # stand, 0.5 m/s straight, turn in place 0.6 rad/s
    lidar = [xyyaw(se2(*pose) @ se2(lever[0], lever[1], 0.0)) for pose in axle]
    labels, rate, speed = motion_classes(track(lidar), lever)
    assert list(labels[:3]) == ["STATIONARY"] * 3
    assert list(labels[5:7]) == ["STRAIGHT"] * 2 and speed[5:7] == pytest.approx([0.5, 0.5])
    assert list(labels[9:11]) == ["TURNING"] * 2 and rate[9:11] == pytest.approx([0.6, 0.6])
    assert speed[9:11] == pytest.approx([0.0, 0.0], abs=1e-9)   # the lidar moves, the axle does not


def test_motion_breakdown_counts_mapped_frames_per_class():
    frames = [{"segment": s, "motion": m} for s, m in (("MIDDLE", "TURNING"), ("MIDDLE", "TURNING"),
                                                       ("MIDDLE", "STRAIGHT"), ("END_DOOR", "TURNING"))]
    rows = [{"fit": 0.1, "change_2s_yaw": math.radians(2.0), "gate_default_correction_ok": False},
            {"fit": 0.3, "change_2s_yaw": math.radians(-4.0), "gate_default_covariance_ok": True},
            {"fit": 0.05},
            {"fit": 0.5, "gate_default_correction_ok": False}]
    result = motion_breakdown(rows, frames)
    assert [result[name]["frames"] for name in ("TURNING", "STRAIGHT", "STATIONARY", "OTHER")] == [2, 1, 0, 0]
    assert result["TURNING"]["fit_m"]["median"] == pytest.approx(0.2)
    assert result["TURNING"]["change_2s_yaw_deg"]["max"] == pytest.approx(4.0)
    assert result["TURNING"]["gate_default_untrusted_frames"] == 1
    assert result["TURNING"]["gate_until_20261107_untrusted_frames"] == 0
    assert result["STATIONARY"]["fit_m"] is None


def test_lidar_time_offset_argument_is_explicit_and_bounded(tmp_path):
    base = ["--project-root", str(tmp_path), "--source-cell", "c", "--map", "m.yaml", "--alignment", "0", "1", "-85"]
    assert arguments(base).lidar_time_offset_ms == 0.0
    assert arguments(base + ["--lidar-time-offset-ms", "75"]).lidar_time_offset_ms == 75.0
    with pytest.raises(SystemExit):
        arguments(base + ["--lidar-time-offset-ms", "300"])


def test_lateral_trial_variant_passes_and_slows_where_the_default_stops():
    from wc_navigation.live.amcl_cross import GATE_VARIANTS, CSV_FIELDS, evaluate_run, summarize_run
    assert GATE_VARIANTS["gate_lateral_trial"]["covariance_mode"] == "lateral"
    assert "gate_lateral_trial_slow" in CSV_FIELDS and "gate_default_slow" not in CSV_FIELDS
    scorer = EndpointScorer(corridor_grid(), 0.1, (0.0, 0.0, 0.0))
    frames, trajectory, rows = [], [], []
    covariance = [0.0] * 36
    covariance[0], covariance[7], covariance[35] = 0.64 ** 2, 0.2 ** 2, 0.05 ** 2   # long along x
    for k in range(8):
        axle = se2(1.0 + 0.1 * k, 2.0, 0.0)                    # driving along x
        stamp = 1_000_000_000 + k * 300_000_000
        frames.append({"stamp_ns": stamp, "t": k * 0.3, "segment": "MIDDLE", "motion": "STRAIGHT",
                       "axle": axle, "lidar": axle, "endpoints": np.array([[0.0, -0.95]])})
        rows.append({"stamp_ns": stamp, "x_m": 0.0, "y_m": 0.0, "yaw_rad": 0.0, "filter_updated": True})
        trajectory.append({"stamp_ns": stamp, "covariance": covariance, "yaw_rad": 0.0})
    table, _ = evaluate_run({"map_odom_trajectory": rows, "trajectory": trajectory}, frames, np.eye(3), scorer)
    assert all(r["gate_default_covariance_ok"] is False for r in table)
    assert all(r["gate_lateral_trial_covariance_ok"] is True and r["gate_lateral_trial_slow"] is True for r in table)
    gate = summarize_run(table, frames)["gate"]
    assert gate["gate_lateral_trial"]["slow_frames"]["MIDDLE"] == 8
    assert gate["gate_lateral_trial"]["full_speed_frames"]["MIDDLE"] == 0
    assert gate["gate_default"]["slow_frames"]["MIDDLE"] == 0 and gate["gate_default"]["either_untrusted_frames"]["MIDDLE"] == 8
