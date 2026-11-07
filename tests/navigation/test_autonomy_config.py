"""Autonomy stack configuration, map files, CLI configuration and supervisor contracts."""
import os
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from wc_navigation.autonomy import nav2_config
from wc_navigation.autonomy.bringup import RunError, Supervisor, _same, domain_lock
from wc_navigation.autonomy.maps import read_back, save_trinary_map

PROJECT = Path(__file__).resolve().parents[2]


def params(mode="navigate", **profile):
    return nav2_config.navigation_parameters(nav2_config.NavProfile(**profile), mode=mode,
                                             bt_to_pose_path="/tmp/to.xml", bt_through_poses_path="/tmp/through.xml")


@pytest.mark.parametrize("through", [False, True])
def test_behavior_tree_never_reverses_or_spins_unprompted(through):
    xml = nav2_config.behavior_tree_xml(through_poses=through)
    tags = {element.tag for element in ET.fromstring(xml).iter()}
    assert "BackUp" not in tags and "Spin" not in tags and "DriveOnHeading" not in tags
    assert {"FollowPath", "Wait", "ClearEntireCostmap"} <= tags
    assert ("ComputePathThroughPoses" if through else "ComputePathToPose") in tags


def test_velocity_chain_never_publishes_plain_cmd_vel():
    p = params()
    monitor = p["collision_monitor"]["ros__parameters"]
    assert monitor["cmd_vel_in_topic"] == nav2_config.CMD_SMOOTHED
    assert monitor["cmd_vel_out_topic"] == nav2_config.CMD_MONITORED
    from wc_navigation.autonomy.bringup import REMAPS
    assert REMAPS["controller_server"] == ["-r", "cmd_vel:=" + nav2_config.CMD_NAV]
    assert REMAPS["behavior_server"] == ["-r", "cmd_vel:=" + nav2_config.CMD_NAV]
    assert "cmd_vel_smoothed:=" + nav2_config.CMD_SMOOTHED in REMAPS["velocity_smoother"]
    assert "/cmd_vel" not in {nav2_config.CMD_NAV, nav2_config.CMD_SMOOTHED, nav2_config.CMD_MONITORED,
                              nav2_config.CMD_GATED}


def test_no_reverse_motion_anywhere_in_the_chain():
    p = params()
    assert p["controller_server"]["ros__parameters"]["FollowPath"]["allow_reversing"] is False
    assert p["velocity_smoother"]["ros__parameters"]["min_velocity"][0] == 0.0
    assert p["behavior_server"]["ros__parameters"]["behavior_plugins"] == ["spin", "wait"]


def test_unknown_space_policy_depends_on_mode():
    explore, navigate = params("explore"), params("navigate")
    assert explore["planner_server"]["ros__parameters"]["GridBased"]["allow_unknown"] is True
    assert navigate["planner_server"]["ros__parameters"]["GridBased"]["allow_unknown"] is False
    g = lambda p: p["global_costmap"]["global_costmap"]["ros__parameters"]["inflation_layer"]
    assert g(navigate)["inflate_around_unknown"] is True and g(explore)["inflate_around_unknown"] is False


def test_margin_is_part_of_the_planning_lethal_radius():
    p = params(radius_m=0.55, margin_m=0.10)
    assert p["global_costmap"]["global_costmap"]["ros__parameters"]["robot_radius"] == pytest.approx(0.65)
    assert p["collision_monitor"]["ros__parameters"]["FootprintApproach"]["radius"] == pytest.approx(0.55)


def test_speed_limits_flow_from_one_profile():
    p = params(max_linear=0.25, max_angular=0.4)
    assert p["controller_server"]["ros__parameters"]["FollowPath"]["desired_linear_vel"] == 0.25
    assert p["velocity_smoother"]["ros__parameters"]["max_velocity"] == [0.25, 0.0, 0.4]
    with pytest.raises(ValueError):
        nav2_config.NavProfile(max_linear=2.0)


def test_readback_expectations_match_requested_parameters():
    profile = nav2_config.NavProfile()
    for mode in ("explore", "navigate"):
        p = params(mode)
        for node, expected in nav2_config.readback_expectations(profile, mode).items():
            section = p
            for part in node.split("/"):
                section = section[part]
            values = section["ros__parameters"]
            for dotted, value in expected.items():
                current = values
                for part in dotted.split("."):
                    current = current[part]
                assert _same(current, value), (mode, node, dotted)


def test_collision_monitor_stop_zone_allows_turning_in_place():
    p = params()["collision_monitor"]["ros__parameters"]
    stop = p["CircleStop"]
    assert stop["type"] == "circle" and stop["action_type"] == "stop"
    assert stop["radius"] == pytest.approx(0.55 + nav2_config.STOP_RING_M)
    assert "PolygonStop" not in p["polygons"]
    assert p["FootprintApproach"]["action_type"] == "approach"


def test_default_mapper_is_odometry_anchored():
    from wc_navigation.autonomy.cli import arguments, build_config
    config = build_config(arguments(["--project-root", str(PROJECT), "explore", "--world", "two_rooms"]))
    assert config["mapper"] == "odom_grid"
    icp_run = build_config(arguments(["--project-root", str(PROJECT), "explore", "--world", "two_rooms",
                                      "--mapper", "odom_icp"]))
    assert icp_run["mapper"] == "odom_icp"
    plain = nav2_config.odom_grid_mapper_config(nav2_config.NavProfile())
    assert plain["pose_source"] == "ODOMETRY_ONLY_NO_SCAN_MATCHING" and plain["grid"]["resolution"] == 0.05
    assert plain["scan_matching"] is False and "icp" not in plain
    icp = nav2_config.odom_grid_mapper_config(nav2_config.NavProfile(), scan_matching=True)
    assert icp["scan_matching"] is True and icp["icp"]["max_correction_xy_m"] > 0
    assert icp["pose_source"] == "ODOMETRY_PREDICTION_BOUNDED_SCAN_TO_MAP_CORRECTION"
    with pytest.raises(SystemExit):
        arguments(["--project-root", str(PROJECT), "explore", "--world", "two_rooms", "--slam-loop-closure"])
    slam = build_config(arguments(["--project-root", str(PROJECT), "explore", "--world", "two_rooms",
                                   "--mapper", "slam_toolbox", "--slam-loop-closure"]))
    assert slam["mapper"] == "slam_toolbox" and slam["slam_loop_closure"] is True


def test_slam_processes_in_place_rotation_and_amcl_uses_explicit_initial_pose():
    profile = nav2_config.NavProfile()
    slam = nav2_config.slam_parameters(profile)["slam_toolbox"]["ros__parameters"]
    assert slam["minimum_travel_distance"] == 0.0 and slam["minimum_time_interval"] > 0
    assert slam["do_loop_closing"] is False
    assert nav2_config.slam_parameters(profile, loop_closure=True)["slam_toolbox"]["ros__parameters"]["do_loop_closing"]
    loc = nav2_config.localization_parameters(profile, "/tmp/map.yaml")
    amcl = loc["amcl"]["ros__parameters"]
    assert amcl["set_initial_pose"] is False and "initial_pose" not in amcl  # waits for /initialpose
    covariance = nav2_config.initial_pose_covariance(0.25, 0.15)
    assert covariance[0] == covariance[7] == pytest.approx(0.0625) and covariance[35] == pytest.approx(0.0225)
    assert sum(1 for v in covariance if v) == 3
    for bad in ((0.0, 0.1), (float("nan"), 0.1), (3.0, 0.1)):
        with pytest.raises(ValueError):
            nav2_config.initial_pose_covariance(*bad)


def test_controller_checks_the_body_while_planning_keeps_the_margin():
    for mode in ("explore", "navigate"):
        p = params(mode, radius_m=0.55, margin_m=0.10)
        assert p["global_costmap"]["global_costmap"]["ros__parameters"]["robot_radius"] == pytest.approx(0.65)
        assert p["local_costmap"]["local_costmap"]["ros__parameters"]["robot_radius"] == pytest.approx(0.55)
        assert p["planner_server"]["ros__parameters"]["GridBased"]["use_astar"] is False
    explore = params("explore")["global_costmap"]["global_costmap"]["ros__parameters"]["plugins"]
    navigate = params("navigate")["global_costmap"]["global_costmap"]["ros__parameters"]["plugins"]
    assert "obstacle_layer" not in explore and "obstacle_layer" in navigate


def test_saved_map_round_trips_and_is_never_overwritten(tmp_path):
    data = np.full((30, 40), -1, dtype=np.int8)
    data[5:25, 5:35] = 0
    data[5, 5:35] = 100
    spec, pgm = save_trinary_map(tmp_path, "map", data, 0.05, (-1.0, 2.0, 0.0))
    _, loaded, meta = read_back(spec)
    assert np.array_equal(loaded, data) and meta["origin"] == [-1.0, 2.0, 0.0]
    with pytest.raises(FileExistsError):
        save_trinary_map(tmp_path, "map", data, 0.05, (0.0, 0.0, 0.0))
    with pytest.raises(ValueError):
        save_trinary_map(tmp_path, "bad", np.full((3, 3), 50, dtype=np.int8), 0.05, (0, 0, 0))


def test_value_comparison_is_strict_for_booleans_and_lists():
    assert _same(0.3, 0.3000000000001) and not _same(True, 1) and not _same(1, True)
    assert _same([0.3, 0.0, 0.5], [0.3, 0, 0.5]) and not _same([0.3], [0.3, 0.0])
    assert _same("a", "a") and not _same(None, False)


def test_second_simulation_domain_is_explicit(monkeypatch):
    from wc_navigation.autonomy.ros_util import require_isolated_domain, simulation_domain
    monkeypatch.delenv("WC_AUTONOMY_ROS_DOMAIN", raising=False)
    assert simulation_domain() == "96"
    monkeypatch.setenv("WC_AUTONOMY_ROS_DOMAIN", "97")
    monkeypatch.setenv("ROS_DOMAIN_ID", "97")
    monkeypatch.setenv("ROS_LOCALHOST_ONLY", "1")
    monkeypatch.delenv("ROS_DISCOVERY_SERVER", raising=False)
    assert simulation_domain() == "97"
    require_isolated_domain()
    monkeypatch.setenv("ROS_DOMAIN_ID", "96")
    with pytest.raises(RuntimeError, match="ROS_DOMAIN_ID=97"):
        require_isolated_domain()
    monkeypatch.setenv("WC_AUTONOMY_ROS_DOMAIN", "0")
    with pytest.raises(RuntimeError, match="must be one of"):
        simulation_domain()


@pytest.mark.parametrize("domain", ["98", "99", "100", "101"])
def test_parallel_checkout_domains_are_accepted(monkeypatch, domain):
    from wc_navigation.autonomy.ros_util import require_isolated_domain, simulation_domain
    monkeypatch.setenv("WC_AUTONOMY_ROS_DOMAIN", domain)
    monkeypatch.setenv("ROS_DOMAIN_ID", domain)
    monkeypatch.setenv("ROS_LOCALHOST_ONLY", "1")
    monkeypatch.delenv("ROS_DISCOVERY_SERVER", raising=False)
    assert simulation_domain() == domain
    require_isolated_domain()


@pytest.mark.parametrize("domain", ["83", "84", "89", "95", "102"])
def test_production_and_tool_domains_stay_rejected(monkeypatch, domain):
    from wc_navigation.autonomy.ros_util import simulation_domain
    monkeypatch.setenv("WC_AUTONOMY_ROS_DOMAIN", domain)
    with pytest.raises(RuntimeError, match="must be one of"):
        simulation_domain()


def test_synthetic_world_layout_is_versioned():
    from wc_navigation.autonomy.world import load_world, synthetic_world
    world, _ = synthetic_world("office_floor")
    assert world.source["revision"] == 2 and len(world.source["layout_sha256"]) == 64
    assert load_world(dict(world.source, obstacles=[])).source == world.source
    with pytest.raises(ValueError, match="layout changed"):
        load_world(dict(world.source, layout_sha256="0" * 64, obstacles=[]))


@pytest.mark.parametrize("domain,localhost", [(None, "1"), ("0", "1"), ("96", None), ("93", "1")])
def test_supervisor_requires_the_isolated_domain(tmp_path, monkeypatch, domain, localhost):
    monkeypatch.delenv("WC_AUTONOMY_ROS_DOMAIN", raising=False)
    for key, value in (("ROS_DOMAIN_ID", domain), ("ROS_LOCALHOST_ONLY", localhost)):
        if value is None:
            monkeypatch.delenv(key, raising=False)
        else:
            monkeypatch.setenv(key, value)
    with pytest.raises(RuntimeError, match="ROS_DOMAIN_ID=96"):
        Supervisor(tmp_path, tmp_path / "out", {"mode": "explore"}).run()
    assert not (tmp_path / "out").exists()


@pytest.mark.skipif(os.name != "posix", reason="POSIX advisory locks")
def test_domain_lock_is_exclusive(tmp_path, monkeypatch):
    monkeypatch.delenv("WC_AUTONOMY_ROS_DOMAIN", raising=False)
    with domain_lock(tmp_path):
        with pytest.raises(RunError, match="owns ROS domain 96"):
            with domain_lock(tmp_path):
                pass
    with domain_lock(tmp_path):
        pass


def test_cli_builds_explicit_simulation_config():
    from wc_navigation.autonomy.cli import arguments, build_config
    args = arguments(["--project-root", str(PROJECT), "explore", "--world", "two_rooms",
                      "--obstacle", "5", "1.5", "0.3", "--odometry-error", "ideal"])
    config = build_config(args)
    assert config["simulation_only"] is True and config["hardware_control"] is False
    assert config["world"]["kind"] == "synthetic" and config["world"]["obstacles"] == [[5.0, 1.5, 0.3]]
    assert config["world_T_map"] == config["start_pose_world"]
    assert config["odometry"]["linear_scale"] == 1.0
    assert config["profile"]["radius_m"] == 0.55
    assert config["slam_loop_closure"] is False
    default = build_config(arguments(["--project-root", str(PROJECT), "explore", "--world", "two_rooms"]))
    assert default["odometry"]["yaw_rate_bias"] == pytest.approx(0.0004)  # wheel + gyro heading


@pytest.mark.parametrize("extra", [["--real-time-factor", "8"], ["--max-linear", "1.2"],
                                   ["--margin", "-0.1"], ["--max-duration", "nan"]])
def test_cli_rejects_out_of_scope_options(extra):
    from wc_navigation.autonomy.cli import arguments
    with pytest.raises(SystemExit):
        arguments(["--project-root", str(PROJECT), "explore", "--world", "two_rooms", *extra])


EXAMPLE_OUTLINE = [[0.70, 0.35], [-0.35, 0.35], [-0.35, -0.35], [0.70, -0.35]]   # rectangle, not measured
PARTIAL_OUTLINE_ARGS = ["--footprint", "0.79", "0.35", "-0.35", "0.35", "-0.35", "-0.35", "0.79", "-0.35",
                        "--footprint-source", "FRONT_FROM_BLIND_RECORDING_REST_EXAMPLE"]


def _prepared(tmp_path, extra):
    from wc_navigation.autonomy.cli import arguments, build_config
    import json
    config = build_config(arguments(["--project-root", str(PROJECT), "explore", "--world", "two_rooms", *extra]))
    out = tmp_path / "run"
    out.mkdir()
    Supervisor(PROJECT, out, config).prepare()
    return config, {name: json.loads((out / name).read_text()) for name in ("sim_config.json", "explorer_config.json")}


def test_simulated_collisions_use_the_profile_outline(tmp_path):
    import yaml
    config, files = _prepared(tmp_path, PARTIAL_OUTLINE_ARGS)
    outline = [[0.79, 0.35], [-0.35, 0.35], [-0.35, -0.35], [0.79, -0.35]]
    assert config["profile"]["footprint"] == outline
    assert config["body_model"] == "POLYGON_FRONT_FROM_BLIND_RECORDING_REST_EXAMPLE"
    body = files["sim_config.json"]["collision_body"]
    assert body["type"] == "polygon" and body["source"] == config["body_model"]
    from wc_navigation.planning.body import Body
    assert Body.from_config(body) == Body(outline=tuple(map(tuple, outline)), source=config["body_model"])
    assert files["explorer_config.json"]["radius_m"] == pytest.approx(0.35)          # inscribed: what the planner passes
    nav2 = yaml.safe_load((tmp_path / "run" / "nav2_params.yaml").read_text())
    for costmap in ("local_costmap", "global_costmap"):
        assert nav2[costmap][costmap]["ros__parameters"]["footprint"] == nav2_config.footprint_string(
            nav2_config.NavProfile(footprint=outline).footprint)


def test_simulated_collisions_default_to_the_assumed_circle(tmp_path):
    config, files = _prepared(tmp_path, [])
    assert config["body_model"] == "CIRCLE_ASSUMED_NOT_MEASURED" and config["profile"]["footprint"] == []
    assert files["sim_config.json"]["collision_body"] == {"type": "circle", "radius_m": 0.55,
                                                          "source": "CIRCLE_ASSUMED_NOT_MEASURED"}
    assert files["sim_config.json"]["collision_radius_m"] == 0.55
    assert files["explorer_config.json"]["radius_m"] == 0.55


@pytest.mark.parametrize("extra", [["--footprint-source", "MEASURED_20261120"],
                                   ["--footprint", "0.7", "0.35", "-0.35", "0.35"],
                                   ["--footprint", "0.5", "0.3", "0.1", "0.3", "0.1", "-0.3", "0.5", "-0.3"],
                                   [*PARTIAL_OUTLINE_ARGS[:9], "--footprint-source", "measured today"]])
def test_simulation_rejects_bad_outline_options(extra):
    from wc_navigation.autonomy.cli import arguments
    with pytest.raises(SystemExit):
        arguments(["--project-root", str(PROJECT), "explore", "--world", "two_rooms", *extra])


def test_body_outline_radii_and_validation():
    profile = nav2_config.NavProfile(footprint=EXAMPLE_OUTLINE)
    assert profile.body_inscribed_m == pytest.approx(0.35) and profile.lethal_radius_m == pytest.approx(0.45)
    assert profile.body_circumscribed_m == pytest.approx((0.70 ** 2 + 0.35 ** 2) ** 0.5)
    assert nav2_config.NavProfile().lethal_radius_m == pytest.approx(0.65)          # the circle is unchanged
    assert nav2_config.NavProfile(**profile.as_dict()) == profile                   # JSON round trip
    for bad in ([[0.5, 0.3], [0.5, -0.3]],                                            # two points
                [[0.5, 0.3], [-0.2, 0.3], [0.5, -0.3], [-0.2, -0.3]],                # self-crossing
                [[0.5, 0.3], [0.1, 0.3], [0.1, -0.3], [0.5, -0.3]],                  # base_link outside
                [[3.0, 0.3], [-0.2, 0.3], [-0.2, -0.3], [3.0, -0.3]]):               # beyond 2 m
        with pytest.raises(ValueError):
            nav2_config.NavProfile(footprint=bad)


def test_grown_outline_keeps_a_constant_ring():
    grown = nav2_config.offset_polygon(nav2_config.NavProfile(footprint=EXAMPLE_OUTLINE).footprint, 0.02)
    assert grown == pytest.approx([0.72, 0.37, -0.37, 0.37, -0.37, -0.37, 0.72, -0.37])


def test_body_outline_flows_into_costmaps_monitor_and_readback():
    profile = nav2_config.NavProfile(footprint=EXAMPLE_OUTLINE)
    p = nav2_config.navigation_parameters(profile, mode="navigate", bt_to_pose_path="/tmp/a.xml",
                                          bt_through_poses_path="/tmp/b.xml")
    local = p["local_costmap"]["local_costmap"]["ros__parameters"]
    global_ = p["global_costmap"]["global_costmap"]["ros__parameters"]
    assert local["footprint"] == global_["footprint"] == nav2_config.footprint_string(profile.footprint)
    assert local["footprint_padding"] == 0.0 and global_["footprint_padding"] == pytest.approx(0.10)
    assert global_["inflation_layer"]["inflation_radius"] >= profile.body_circumscribed_m + profile.margin_m
    monitor = p["collision_monitor"]["ros__parameters"]
    assert monitor["polygons"] == ["FootprintStop", "PolygonSlow", "FootprintApproach"]
    assert monitor["FootprintStop"]["type"] == "polygon" and monitor["FootprintApproach"]["type"] == "polygon"
    for node, expected in nav2_config.readback_expectations(profile, "navigate").items():
        section = p
        for part in node.split("/"):
            section = section[part]
        for dotted, value in expected.items():
            current = section["ros__parameters"]
            for part in dotted.split("."):
                current = current[part]
            assert _same(current, value), (node, dotted)
