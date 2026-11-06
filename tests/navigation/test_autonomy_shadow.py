"""Nav2 shadow run on a recording: pose/twist helpers, zone audit, command attribution and summary."""
import json
import math
from pathlib import Path

import numpy as np
import pytest

from wc_navigation.autonomy import nav2_config, shadow
from wc_navigation.autonomy.ros_util import compose

LEVER = (0.323, 0.229)
ANGLES = -math.pi / 3 + (math.pi / 240) * np.arange(161)


def arc(n=61, v=0.4, w=0.2, dt=0.3, start=(1.0, 2.0, 3.0)):
    """Exact poses of constant forward speed and yaw rate at times k*dt."""
    times = np.arange(n) * dt
    x, y, yaw = start
    poses = []
    for t in times:
        if abs(w) < 1e-12:
            poses.append((x + v * t * math.cos(yaw), y + v * t * math.sin(yaw), yaw))
        else:
            r, heading = v / w, yaw + w * t
            poses.append((x + r * (math.sin(heading) - math.sin(yaw)), y - r * (math.cos(heading) - math.cos(yaw)),
                          math.atan2(math.sin(heading), math.cos(heading))))
    return times, np.asarray(poses)


def test_axle_pose_places_the_laser_at_the_recorded_lidar_pose():
    lidar = np.array([[1.0, 2.0, 0.3], [-4.0, 0.5, -2.9], [0.0, 0.0, math.pi]])
    base = shadow.base_poses_from_lidar(lidar, LEVER)
    for b, l in zip(base, lidar):
        assert np.allclose(compose(tuple(b), (*LEVER, 0.0)), l, atol=1e-12)


def test_interpolation_takes_the_short_way_across_pi_and_clamps():
    times = np.array([0.0, 1.0])
    poses = np.array([[0.0, 0.0, math.pi - 0.1], [1.0, 0.0, -math.pi + 0.1]])
    mid = shadow.interpolate_poses(times, poses, 0.5)[0]
    assert mid[0] == pytest.approx(0.5) and abs(abs(mid[2]) - math.pi) < 1e-9
    assert np.allclose(shadow.interpolate_poses(times, poses, [-1.0, 2.0]), poses)
    with pytest.raises(ValueError):
        shadow.interpolate_poses([0.0, 0.0], poses, 0.0)


def test_twist_of_a_constant_arc():
    times, poses = arc()
    v, w = shadow.segment_twist(times, poses)
    assert np.allclose(w, 0.2, atol=1e-9) and np.allclose(v, 0.4, rtol=2e-3)
    v2, w2 = shadow.twist_at(times, poses, times[5:-5])
    assert np.allclose(v2, 0.4, rtol=5e-3) and np.allclose(w2, 0.2, atol=1e-6)


def test_motion_window_starts_shortly_before_the_first_motion():
    times = np.arange(0.0, 30.0, 0.5)
    poses = np.zeros((len(times), 3))
    moving = times >= 12.0
    poses[moving, 0] = 0.3 * (times[moving] - 12.0)
    start, end = shadow.motion_window(times, poses, lead_s=5.0)
    assert start == pytest.approx(7.0) and end == pytest.approx(times[-1])
    with pytest.raises(ValueError, match="never moves"):
        shadow.motion_window(times, np.zeros((len(times), 3)))


def test_carrot_goal_is_ahead_and_clamped_to_the_window_end():
    times, poses = arc(n=41, w=0.0)
    goal = shadow.carrot_goal(times, poses, 3.0, 6.0, times[-1])
    assert goal == pytest.approx(tuple(shadow.interpolate_poses(times, poses, 9.0)[0]))
    assert shadow.carrot_goal(times, poses, times[-1] - 1.0, 6.0, times[-1]) == pytest.approx(tuple(poses[-1]))
    assert not shadow.goal_moved(goal, goal, 0.3, 0.3) and shadow.goal_moved(goal, None, 0.3, 0.3)
    assert shadow.goal_moved(goal, (goal[0], goal[1], goal[2] + 0.5), 0.3, 0.3)


def test_zone_audit_counts_returns_like_the_collision_monitor():
    profile = nav2_config.NavProfile()
    stop = profile.radius_m + nav2_config.STOP_RING_M
    far = np.full(161, np.inf)
    far[80] = 3.0                      # one wall return straight ahead
    near, few = far.copy(), far.copy()
    near[:5] = 0.35                    # five returns just right of the axle, inside the stop circle
    few[:3] = 0.35                     # three returns: not more than max_points
    audit = shadow.zone_audit(ANGLES, [far, near, few], LEVER, stop, nav2_config.slowdown_polygon(profile),
                              nav2_config.MONITOR_MAX_POINTS)
    assert audit["stop_points"].tolist() == [0, 5, 3] and audit["slow_points"].tolist() == [0, 5, 3]
    assert audit["stop_expected"].tolist() == [False, True, False]
    assert audit["nearest_m"][0] == pytest.approx(math.hypot(LEVER[0] + 3.0, LEVER[1]))
    rows = shadow.near_points(ANGLES, [far, near], LEVER, profile.lethal_radius_m)
    assert {r[0] for r in rows} == {1} and len(rows) == 5
    assert shadow.zone_audit(ANGLES, [np.full(161, np.inf)], LEVER, stop, [0, 0, 1, 0, 1, 1], 3)["nearest_m"][0] == math.inf


def test_point_in_polygon_even_odd():
    square = [0.0, 0.0, 2.0, 0.0, 2.0, 2.0, 0.0, 2.0]
    inside = shadow.inside_polygon([[1.0, 1.0], [3.0, 1.0], [1.0, -0.5], [0.1, 1.9]], square)
    assert inside.tolist() == [True, False, False, True]


def test_attribution_names_the_first_stage_that_zeroes_the_command():
    go, zero, nan = [0.3, 0.1], [0.0, 0.0], [np.nan, np.nan]
    cases = [("DRIVES_FORWARD", (go, go, go, go)),
             ("STOPPED_BY_GATE", (go, go, go, zero)),
             ("STOPPED_BY_COLLISION_MONITOR", (go, go, zero, zero)),
             ("STOPPED_BY_COLLISION_MONITOR", (go, go, nan, zero)),   # monitor stopped publishing after a stop
             ("SMOOTHER_ZERO", (go, zero, zero, zero)),
             ("CONTROLLER_ZERO", (zero, zero, zero, zero)),
             ("NAV2_NOT_COMMANDING", (nan, nan, nan, zero))]
    stages = {s: np.array([c[1][i] for c in cases], dtype=float) for i, s in enumerate(shadow.STAGES)}
    outcome, slowed = shadow.attribute(stages)
    assert outcome.tolist() == [c[0] for c in cases] and not slowed.any()
    stages["monitored"][0] = stages["gated"][0] = [0.1, 0.1]     # monitor cut forward speed to a third
    outcome, slowed = shadow.attribute(stages)
    assert outcome[0] == "DRIVES_FORWARD" and slowed[0]


def test_a_slow_turn_in_place_is_a_rotation_not_a_stop():
    # Rotate-to-heading is acceleration-limited around the current yaw rate, so it can be tiny.
    turn, small, zero = [0.0, 0.004], [0.0, 0.003], [0.0, 0.0]
    stages = {"nav": np.array([turn, turn, turn]), "smoothed": np.array([small, small, zero]),
              "monitored": np.array([small, zero, zero]), "gated": np.array([small, zero, zero])}
    outcome, slowed = shadow.attribute(stages)
    assert outcome.tolist() == ["ROTATES_IN_PLACE", "STOPPED_BY_COLLISION_MONITOR", "SMOOTHER_ZERO"]
    assert not slowed.any()


def test_latest_sample_resampling_respects_the_age_limit():
    t = np.array([0.0, 1.0, 2.0])
    values = np.array([[1.0, 0.0], [2.0, 0.0], [3.0, 0.0]])
    out = shadow.resample_latest(t, values, [-0.5, 0.2, 1.0, 1.7, 2.4, 3.0], 0.5)
    assert np.allclose(out[:, 0], [np.nan, 1.0, 2.0, np.nan, 3.0, np.nan], equal_nan=True)
    assert np.isnan(shadow.resample_latest([], np.zeros((0, 2)), [0.0], 0.5)).all()


def test_evaluation_compares_each_stage_with_the_recorded_motion():
    times, poses = arc(n=101, v=0.4, w=0.25, dt=0.3)           # 30 s of left-turning motion
    t = np.arange(0.0, 30.0, 0.05)
    nav = np.column_stack([np.full(len(t), 0.3), np.full(len(t), 0.2)])
    monitored = nav.copy()
    monitored[(t >= 10.0) & (t < 14.0)] = 0.0
    samples = {"nav": (t, nav), "smoothed": (t, nav), "monitored": (t, monitored), "gated": (t, monitored)}
    table, summary = shadow.evaluate(times, poses, (0.0, 29.0), samples)
    assert summary["turn_direction_agreement"]["fraction"] == pytest.approx(1.0)
    fractions = summary["outcome_fractions_while_recording_moves"]
    assert fractions["STOPPED_BY_COLLISION_MONITOR"] == pytest.approx(4.0 / 29.0, abs=0.02)
    assert fractions["DRIVES_FORWARD"] + fractions["STOPPED_BY_COLLISION_MONITOR"] == pytest.approx(1.0)
    assert summary["drives_forward_fraction_while_recording_drives_forward"] == pytest.approx(25.0 / 29.0, abs=0.02)
    (a, b), = summary["stop_intervals_collision_monitor"]
    assert a == pytest.approx(10.0, abs=0.11) and b == pytest.approx(14.0, abs=0.11)
    assert summary["stationary_samples"] == 0
    assert summary["gated_linear_mps_while_driving_forward"]["median"] == pytest.approx(0.3)
    assert len(table["t"]) == summary["samples"]


def test_plan_agreement_flags_a_different_route():
    times, poses = arc(n=61, v=0.5, w=0.0, start=(0.0, 0.0, 0.0))     # straight along +x
    along = {"t": 3.0, "xy": [[1.5, 0.05], [9.0, 0.05]]}
    detour = {"t": 6.0, "xy": [[3.0, 0.0], [3.0, 2.5], [9.0, 2.5]]}
    summary, per_plan = shadow.plan_agreement(times, poses, [along, detour], horizon_s=4.0)
    assert summary["plans_compared"] == 2
    assert per_plan[0]["max_deviation_m"] == pytest.approx(0.05, abs=1e-6)
    assert per_plan[1]["max_deviation_m"] == pytest.approx(2.0, abs=1e-6)
    assert [c["t"] for c in summary["route_changes"]] == [6.0]


def test_replay_window_needs_two_frames_and_never_reads_commands():
    from wc_navigation.autonomy import replay_node
    recording = {"times_s": np.array([0.0, 0.3, 0.6, 0.9])}
    assert replay_node.window_frames(recording, (0.25, 0.65)).tolist() == [1, 2]
    with pytest.raises(ValueError):
        replay_node.window_frames(recording, (0.31, 0.5))
    source = Path(replay_node.__file__).read_text(encoding="utf-8")
    assert "Twist" not in source and "CMD_GATED" not in source   # the recording never reacts to commands


def test_shadow_cli_rejects_out_of_scope_options(tmp_path):
    from wc_navigation.autonomy.shadow_cli import arguments
    base = ["--project-root", str(tmp_path), "--source-cell", str(tmp_path), "--map", str(tmp_path / "m.yaml")]
    args = arguments(base)
    assert args.lead == 15.0 and args.max_linear == 0.30 and args.window is None and args.mode == "navigate"
    assert arguments(base + ["--mode", "follow"]).mode == "follow"
    for extra in (["--real-time-factor", "4"], ["--max-linear", "0.9"], ["--window", "50", "10"],
                  ["--lead", "0.5"], ["--scan-height", "0.5", "-0.5"], ["--mode", "drive"]):
        with pytest.raises(SystemExit):
            arguments(base + extra)


@pytest.mark.parametrize("key,line", [
    ("planner_failed", '[planner_server]: GridBased plugin failed to plan calculation to (1.00, 2.00): '
                       '"Failed to create plan with tolerance of: 0.250000"'),
    ("planner_failed", "[planner_server]: Planning algorithm GridBased failed to generate a valid path to (1.00, 2.00)"),
    ("controller_collision_ahead", "[controller_server]: RegulatedPurePursuitController detected collision ahead!"),
    ("controller_no_progress", "[controller_server]: Failed to make progress"),
    ("collision_monitor_stop", "[collision_monitor]: Robot to stop due to CircleStop polygon"),
    ("collision_monitor_slowdown", "[collision_monitor]: Robot to slow down for 40.000000 percents due to "
                                   "PolygonSlow polygon"),
    ("bt_goal_failed", "[bt_navigator]: Goal failed"),
])
def test_log_patterns_match_humble_messages(tmp_path, key, line):
    from wc_navigation.autonomy.shadow_cli import LOG_PATTERNS, log_counts
    process = LOG_PATTERNS[key][0]
    log = tmp_path / "x.log"
    log.write_text("[INFO] unrelated line\n" + line + "\n", encoding="utf-8")
    assert log_counts({process if process != "*" else "any": str(log)})[key] == 1


def test_map_fit_reports_where_the_recorded_path_lies(tmp_path):
    from wc_navigation.autonomy.maps import save_trinary_map
    from wc_navigation.autonomy.shadow_cli import map_fit
    data = np.full((40, 60), -1, dtype=np.int8)
    data[10:30, 10:50] = 0
    data[20, 30] = 100
    spec, _ = save_trinary_map(tmp_path, "map", data, 0.1, (0.0, 0.0, 0.0))
    poses = np.array([[2.05, 1.55, 0.0], [3.05, 2.05, 0.0], [0.5, 0.5, 0.0], [-1.0, 0.0, 0.0]])
    assert map_fit(spec, poses) == {"free": 0.25, "occupied": 0.25, "unknown": 0.25, "outside": 0.25}


def test_shadow_supervisor_uses_the_map_without_amcl_and_no_covariance_gate(tmp_path):
    import yaml
    from wc_navigation.autonomy.shadow_cli import ShadowSupervisor
    config = {"profile": nav2_config.NavProfile().as_dict(), "map_yaml": str(tmp_path / "map.yaml"),
              "recording": str(tmp_path / "recording.npz"), "window_s": [0.0, 10.0], "real_time_factor": 1.0,
              "tail_hold_s": 5.0, "lead_s": 15.0, "update_period_s": 4.0}
    (tmp_path / "out").mkdir()
    _, files = ShadowSupervisor(tmp_path, tmp_path / "out", config).prepare()
    mapping = yaml.safe_load(Path(files["mapping"]).read_text(encoding="utf-8"))
    assert set(mapping) == {"map_server", "lifecycle_manager_localization"}
    assert mapping["lifecycle_manager_localization"]["ros__parameters"]["node_names"] == ["map_server"]
    assert json.loads(Path(files["gate"]).read_text(encoding="utf-8"))["gate"]["require_covariance"] is False
    nav = yaml.safe_load(Path(files["nav2"]).read_text(encoding="utf-8"))
    assert nav["planner_server"]["ros__parameters"]["GridBased"]["allow_unknown"] is False
    assert json.loads(Path(files["task"]).read_text(encoding="utf-8"))["lead_s"] == 15.0


def test_summary_reports_open_loop_outcomes_without_ros(tmp_path):
    from wc_navigation.autonomy.maps import save_trinary_map
    from wc_navigation.autonomy.shadow_cli import summarize
    times, base = arc(n=81, v=0.4, w=0.0, start=(1.0, 2.0, 0.0))        # 24 s straight along +x
    ranges = np.full((len(times), 161), np.inf, dtype=np.float32)
    ranges[:, 80] = 2.5
    ranges[40:44, :5] = 0.35                                            # four frames inside the stop circle
    recording = {"times_s": times, "base_poses": base, "ranges": ranges, "lever_xy": np.array(LEVER)}
    data = np.zeros((60, 260), dtype=np.int8)
    data[0, :] = data[-1, :] = 100
    spec, _ = save_trinary_map(tmp_path, "map", data, 0.05, (0.0, 0.5, 0.0))
    config = {"profile": nav2_config.NavProfile().as_dict(), "window_s": [0.0, 24.0], "lead_s": 6.0,
              "map_yaml": str(spec)}
    s = np.arange(0.0, 24.0, 0.05)
    go = [(round(100.0 + v, 4), 0.3, 0.0) for v in s]
    stopped = [(round(100.0 + v, 4), 0.0 if 12.0 <= v < 13.2 else 0.3, 0.0) for v in s]
    plans = [{"time_s": 100.0 + k, "stamp_s": 100.0 + k, "frame": "map",
              "xy": [[1.0 + 0.4 * k, 2.0], [3.4 + 0.4 * k, 2.0]]} for k in range(0, 20, 2)]
    report = {"logs": {}, "recorded": {
        "events": [{"kind": "replay_state", "time_s": 100.0, "value": {"state": "PLAYING"}}],
        "commands": {"nav": go, "smoothed": go, "monitored": stopped, "gated": stopped}, "plans": plans,
        "final_replay_status": {"state": "FINISHED", "playback_start_s": 100.0, "window_s": [0.0, 24.0]}}}
    output = tmp_path / "out"
    output.mkdir()
    (output / "shadow_mission_result.json").write_text(json.dumps({"status": "COMPLETE", "goals_sent": 5}),
                                                      encoding="utf-8")
    result = summarize(config, report, output, recording)
    assert result["status"] == "COMPLETE" and result["open_loop"] is True and result["commands_sent"] is False
    assert result["scan_zone_audit"]["frames_stop_expected"] == 4
    fractions = result["commands"]["outcome_fractions_while_recording_moves"]
    assert fractions["STOPPED_BY_COLLISION_MONITOR"] == pytest.approx(1.2 / 24.0, abs=0.01)
    assert result["plans"]["plans_compared"] == 10 and result["plans"]["max_deviation_m"]["max"] < 1e-6
    for name in ("result.json", "samples.csv", "plans.json", "preview.png", "events.jsonl"):
        assert (output / name).is_file(), name
    saved = json.loads((output / "result.json").read_text(encoding="utf-8"))
    assert saved["hardware_control"] is False and saved["physical_navigation_validated"] is False


def corridor_map(pinhole=None):
    """10 m x 2.4 m corridor, walls at y=0 and y=2.4 (0.05 m cells), optional unknown pinhole cell."""
    data = np.zeros((48, 200), dtype=np.int8)
    data[0, :] = data[-1, :] = 100
    data[:, 0] = data[:, -1] = 100
    if pinhole is not None:
        data[pinhole] = -1
    return data


def test_static_planning_map_reproduces_the_pinhole_blockage():
    clean = shadow.StaticPlanningMap(corridor_map(), 0.05, (0.0, 0.0), 0.65)
    assert clean.check((1.0, 1.2), (9.0, 1.2)) == "REACHABLE"
    # Walls leave a 1.0 m plannable band; one unknown cell inflated by 0.65 m covers all of it.
    holed = corridor_map(pinhole=(24, 100))
    conservative = shadow.StaticPlanningMap(holed, 0.05, (0.0, 0.0), 0.65)
    occupied_only = shadow.StaticPlanningMap(holed, 0.05, (0.0, 0.0), 0.65, inflate_unknown=False)
    assert conservative.check((1.0, 1.2), (9.0, 1.2)) == "GOAL_NOT_CONNECTED"
    assert occupied_only.check((1.0, 1.2), (9.0, 1.2)) == "REACHABLE"
    reason = shadow.attribute_planner_failure(conservative, occupied_only, (1.0, 1.2), (9.0, 1.2))
    assert reason == "INFLATED_UNKNOWN_CUTS_THE_ROUTE"
    assert shadow.attribute_planner_failure(conservative, occupied_only, (1.0, 0.4), (3.0, 1.2)) == \
        "START_WITHIN_LETHAL_RADIUS_OF_OCCUPIED"
    assert shadow.attribute_planner_failure(conservative, occupied_only, (1.0, 1.2), (3.0, 1.2)) == \
        "NOT_EXPLAINED_BY_STATIC_MAP"
    assert shadow.attribute_planner_failure(conservative, occupied_only, (1.0, 1.2), (30.0, 1.2)) == "OUTSIDE_MAP"
    walled = corridor_map()
    walled[:, 100] = 100
    blocked = shadow.StaticPlanningMap(walled, 0.05, (0.0, 0.0), 0.65)
    assert shadow.attribute_planner_failure(blocked, blocked, (1.0, 1.2), (9.0, 1.2)) == \
        "OCCUPIED_CELLS_CUT_THE_ROUTE_OR_GOAL"


def test_goal_tolerance_window_moves_a_goal_off_the_wall_like_navfn():
    planning = shadow.StaticPlanningMap(corridor_map(), 0.05, (0.0, 0.0), 0.65)
    assert planning.check((1.0, 1.2), (9.0, 0.35)) == "GOAL_NOT_CONNECTED"        # 0.35 m from the wall
    assert planning.check((1.0, 1.2), (9.0, 0.5), tolerance_m=0.0) == "GOAL_NOT_CONNECTED"
    assert planning.check((1.0, 1.2), (9.0, 0.5), tolerance_m=0.25) == "REACHABLE"


def test_path_audit_counts_pinholes_beside_the_path():
    holed = corridor_map(pinhole=(24, 100))
    planning = shadow.StaticPlanningMap(holed, 0.05, (0.0, 0.0), 0.65)
    path = np.column_stack([np.linspace(1.0, 9.0, 81), np.full(81, 1.0)])
    audit = shadow.path_map_audit(planning, path)
    assert audit["unknown_pinholes_within_lethal_of_path"] == 1
    assert audit["clearance_to_unknown_m"]["min"] == pytest.approx(0.2, abs=0.06)   # cell centres 4-5 cells apart
    assert 0 < audit["fraction_within_lethal_of_unknown"] < 0.3
    assert audit["fraction_within_lethal_of_occupied"] == 0.0


def test_planner_report_checks_the_recorded_costmap_when_the_static_map_is_clear(tmp_path):
    from wc_navigation.autonomy.shadow_cli import RecordedCostmaps, planner_report
    data = corridor_map()
    conservative = shadow.StaticPlanningMap(data, 0.05, (0.0, 0.0), 0.65)
    occupied_only = shadow.StaticPlanningMap(data, 0.05, (0.0, 0.0), 0.65, inflate_unknown=False)
    live = np.zeros(data.shape, dtype=bool)
    live[:, 120:124] = True                                   # someone marked across the corridor at x = 6 m
    costmaps = RecordedCostmaps({"times_s": [100.0], "packed": [np.packbits(live)],
                                 "layout": {"shape": list(data.shape), "resolution": 0.05, "origin": [0.0, 0.0]}})
    times, poses = np.array([0.0, 10.0]), np.array([[1.0, 1.2, 0.0], [9.0, 1.2, 0.0]])
    records = [{"sent_time_s": 100.0, "goal": [9.0, 1.2, 0.0]}]
    results = [{"action": "compute_path_to_pose", "status": "ABORTED", "time_s": 100.5},
               {"action": "compute_path_to_pose", "status": "SUCCEEDED", "time_s": 101.0},
               {"action": "follow_path", "status": "ABORTED", "time_s": 101.0}]
    report = planner_report(results, records, times, poses, 0.0, 100.0, conservative, occupied_only, costmaps)
    assert report["plan_requests"] == 2 and report["plan_success_fraction"] == 0.5
    assert report["failure_reasons"] == {"LIVE_COSTMAP_CUTS_THE_ROUTE": 1}
    assert report["failures"][0]["costmap_age_s"] == pytest.approx(0.5)
    assert report["failures"][0]["start_xy"] == [1.4, 1.2]
    saved = costmaps.save(tmp_path / "costmaps.npz")
    assert saved and np.load(saved)["packed"].shape[0] == 1
    without = planner_report(results, records, times, poses, 0.0, 100.0, conservative, occupied_only)
    assert without["failure_reasons"] == {"NO_RECORDED_COSTMAP": 1}


def test_followed_path_is_the_recorded_path_ahead():
    times, poses = arc(n=41, v=0.5, w=0.0, start=(0.0, 0.0, 0.0))     # 12 s straight along +x
    path = shadow.path_ahead(times, poses, 2.0, 4.0, times[-1], spacing_m=0.1)
    assert path[0] == pytest.approx([1.0, 0.0, 0.0]) and path[-1] == pytest.approx([3.0, 0.0, 0.0])
    steps = np.hypot(*np.diff(path[:, :2], axis=0).T)
    assert steps[:-1].min() >= 0.1 - 1e-9 and 16 <= len(path) <= 21
    tail = shadow.path_ahead(times, poses, times[-1] - 0.5, 4.0, times[-1])
    assert tail[-1] == pytest.approx(poses[-1])                            # clamped to the window end


def test_offset_to_the_latest_plan_respects_its_age():
    plans = [{"t": 0.0, "xy": [[0.0, 0.5], [10.0, 0.5]]}, {"t": 5.0, "xy": [[0.0, 0.0], [10.0, 0.0]]}]
    offset = shadow.offset_to_latest_plan([1.0, 4.0, 5.5, 9.0], [[1.0, 0.0], [4.0, 0.0], [5.5, 0.2], [9.0, 0.0]],
                                          plans, max_age_s=2.0)
    assert offset[0] == pytest.approx(0.5) and offset[2] == pytest.approx(0.2)
    assert np.isnan(offset[1]) and np.isnan(offset[3])                      # newest plan older than 2 s
