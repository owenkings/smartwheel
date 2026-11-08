"""Selectable planner/controller plugins: defaults unchanged, no reversing, outline-aware planners, readback."""
import json
from pathlib import Path

import pytest

from wc_navigation.autonomy import nav2_config
from wc_navigation.autonomy.bringup import _same
from wc_navigation.planning import stack

PROJECT = Path(__file__).resolve().parents[2]
OUTLINE = [[0.79, 0.35], [-0.35, 0.35], [-0.35, -0.35], [0.79, -0.35]]   # FRONT_FROM_BLIND_RECORDING_REST_EXAMPLE


def params(choice=None, mode="navigate", **profile):
    return nav2_config.navigation_parameters(nav2_config.NavProfile(**profile), mode=mode,
                                             bt_to_pose_path="/tmp/a.xml", bt_through_poses_path="/tmp/b.xml",
                                             planning=choice)


def _lookup(p, node, dotted):
    section = p
    for part in node.split("/"):
        section = section[part]
    current = section["ros__parameters"]
    for part in dotted.split("."):
        current = current[part]
    return current


@pytest.mark.parametrize("mode", ["explore", "navigate"])
def test_default_choice_leaves_every_parameter_unchanged(mode):
    profile = nav2_config.NavProfile()
    assert params(stack.PlanningChoice(), mode) == params(None, mode)
    assert nav2_config.readback_expectations(profile, mode, planning=stack.PlanningChoice()) == \
        nav2_config.readback_expectations(profile, mode)


@pytest.mark.parametrize("name", stack.PLANNERS)
@pytest.mark.parametrize("mode", ["explore", "navigate"])
def test_each_planner_replaces_only_the_grid_based_plugin(name, mode):
    base, chosen = params(None, mode), params(stack.PlanningChoice(planner=name), mode)
    plugin = chosen["planner_server"]["ros__parameters"]["GridBased"]
    assert plugin["plugin"] == stack.PLANNER_PLUGINS[name]
    assert plugin["allow_unknown"] is (mode == "explore")
    for key in base:
        if key != "planner_server":
            assert chosen[key] == base[key], key
    assert chosen["planner_server"]["ros__parameters"]["planner_plugins"] == ["GridBased"]


def test_outline_aware_planners_never_reverse():
    hybrid = stack.planner_plugin("smac_hybrid", exploring=False)
    lattice = stack.planner_plugin("smac_lattice", exploring=False)
    assert hybrid["motion_model_for_search"] == "DUBIN" and hybrid["minimum_turning_radius"] > 0
    assert lattice["allow_reverse_expansion"] is False
    primitives = json.loads(Path(lattice["lattice_filepath"]).read_text())["lattice_metadata"]
    assert primitives["motion_model"] == "diff" and primitives["grid_resolution"] == 0.05
    costmap = params(stack.PlanningChoice(planner="smac_lattice"))["global_costmap"]["global_costmap"]["ros__parameters"]
    assert costmap["resolution"] == primitives["grid_resolution"]
    assert set(stack.FOOTPRINT_PLANNERS) == {"smac_hybrid", "smac_lattice"}


def test_inflation_covers_the_turning_outline_for_se2_checks():
    profile = nav2_config.NavProfile(footprint=OUTLINE)
    costmap = params(stack.PlanningChoice(planner="smac_hybrid"), footprint=OUTLINE)
    inflation = costmap["global_costmap"]["global_costmap"]["ros__parameters"]["inflation_layer"]["inflation_radius"]
    assert inflation >= profile.body_circumscribed_m + profile.margin_m


@pytest.mark.parametrize("name", stack.PLANNERS)
def test_readback_matches_the_requested_parameters(name):
    choice = stack.PlanningChoice(planner=name, planner_params=stack.parse_overrides(["tolerance=0.3"])
                                  if name != "theta_star" else ())
    profile = nav2_config.NavProfile(footprint=OUTLINE)
    for mode in ("explore", "navigate"):
        p = params(choice, mode, footprint=OUTLINE)
        expected = nav2_config.readback_expectations(profile, mode, planning=choice)
        assert expected["planner_server"]["GridBased.plugin"] == stack.PLANNER_PLUGINS[name]
        assert ("GridBased.use_astar" in expected["planner_server"]) is (name == "navfn")
        assert expected["controller_server"]["FollowPath.allow_reversing"] is False
        for node, values in expected.items():
            for dotted, value in values.items():
                assert _same(_lookup(p, node, dotted), value), (name, mode, node, dotted)


def test_overrides_are_parsed_applied_and_validated():
    assert stack.parse_overrides(["minimum_turning_radius=0.6", "smooth_path=false", "lattice_filepath=/x/y.json"]) == (
        ("minimum_turning_radius", 0.6), ("smooth_path", False), ("lattice_filepath", "/x/y.json"))
    choice = stack.PlanningChoice(planner="smac_hybrid", planner_params=(("minimum_turning_radius", 0.6),),
                                  controller_params=(("lookahead_dist", 0.8),))
    p = params(choice)
    assert p["planner_server"]["ros__parameters"]["GridBased"]["minimum_turning_radius"] == 0.6
    assert p["controller_server"]["ros__parameters"]["FollowPath"]["lookahead_dist"] == 0.8
    assert stack.PlanningChoice.from_dict(json.loads(json.dumps(choice.as_dict()))) == choice
    for bad in (["minimum_turning_radius"], ["=1"], ["a b=1"]):
        with pytest.raises(ValueError):
            stack.parse_overrides(bad)
    with pytest.raises(ValueError):
        stack.PlanningChoice(planner="dwb")


@pytest.mark.parametrize("extra", [["--planner-param", "motion_model_for_search=REEDS_SHEPP"],
                                   ["--controller-param", "allow_reversing=true"],
                                   ["--planner-param", "allow_unknown=true"],
                                   ["--planner", "unknown_planner"]])
def test_cli_keeps_reversing_and_unknown_space_fixed(extra):
    from wc_navigation.autonomy.cli import arguments
    with pytest.raises(SystemExit):
        arguments(["--project-root", str(PROJECT), "explore", "--world", "two_rooms", *extra])


@pytest.mark.parametrize("name", stack.CONTROLLERS)
def test_each_controller_keeps_forward_only_motion_and_reads_back(name):
    profile = nav2_config.NavProfile(footprint=OUTLINE)
    choice = stack.PlanningChoice(controller=name)
    p = params(choice, footprint=OUTLINE)
    follow = p["controller_server"]["ros__parameters"]["FollowPath"]
    assert follow["plugin"] == stack.CONTROLLER_PLUGINS[name]
    if name == "mppi":
        assert follow["vx_min"] == 0.0 and follow["vx_max"] == profile.max_linear
        assert follow["motion_model"] == "DiffDrive" and follow["CostCritic"]["consider_footprint"] is True
        assert follow["model_dt"] == pytest.approx(1.0 / profile.controller_frequency_hz)
        assert set(follow["critics"]) <= set(follow)
    else:
        assert follow["allow_reversing"] is False and follow["use_collision_detection"] is True
    if name == "shim_rpp":
        assert follow["primary_controller"] == stack.RPP_PLUGIN and follow["use_rotate_to_heading"] is False
        assert follow["angular_dist_threshold"] == pytest.approx(0.785)
    base = params(None, footprint=OUTLINE)
    for key in base:
        if key not in ("controller_server",):
            assert p[key] == base[key], key
    assert p["velocity_smoother"]["ros__parameters"]["min_velocity"][0] == 0.0
    expected = nav2_config.readback_expectations(profile, "navigate", planning=choice)
    for node, values in expected.items():
        for dotted, value in values.items():
            assert _same(_lookup(p, node, dotted), value), (name, node, dotted)


def test_navigation_can_use_the_true_world_as_its_map(tmp_path):
    from wc_navigation.autonomy import cli
    from wc_navigation.autonomy.maps import read_back, save_trinary_map
    from wc_navigation.autonomy.world import synthetic_world
    config = cli.build_config(cli.arguments(["--project-root", str(PROJECT), "navigate", "--world", "two_rooms",
                                             "--goal", "8.0", "1.4", "0.0", "--planner", "smac_hybrid"]))
    assert config["map_from_world"] is True and config["map_yaml"] is None and config["world_T_map"] == [0.0, 0.0, 0.0]
    spec = cli.write_world_map(config, tmp_path)
    _, data, meta = read_back(spec)
    world, start = synthetic_world("two_rooms")
    assert (data == 100).sum() == world.solid.sum() and config["map_yaml"] == str(spec)
    assert config["true_start_map"] == list(start) and config["goals_map"] == [[8.0, 1.4, 0.0]]
    grid = data.copy()
    grid[:4, :] = -1
    saved, _ = save_trinary_map(tmp_path, "saved", grid, 0.05, (0.0, 0.0, 0.0))
    recorded = cli.build_config(cli.arguments(["--project-root", str(PROJECT), "navigate", "--world-map", str(saved),
                                               "--start", "1.2", "1.4", "0.0", "--goal", "8.0", "1.4", "0.0"]))
    assert recorded["map_yaml"] == str(Path(saved).resolve()) and recorded["world"]["kind"] == "map"
    with pytest.raises(SystemExit):
        cli.arguments(["--project-root", str(PROJECT), "navigate", "--world", "two_rooms",
                       "--from-exploration", str(tmp_path)])


def test_recovery_tree_and_checkers_are_explicit_options():
    import xml.etree.ElementTree as ET
    assert nav2_config.behavior_tree_xml(**stack.PlanningChoice().behavior_tree_options()) == \
        nav2_config.behavior_tree_xml()
    root = ET.fromstring(nav2_config.behavior_tree_xml(recovery_retries=8, wait_s=5.0))
    top = [e for e in root.iter("RecoveryNode") if e.get("name") == "NavigateRecovery"][0]
    assert top.get("number_of_retries") == "8" and [e.get("wait_duration") for e in root.iter("Wait")] == ["5"]
    assert {e.tag for e in root.iter()} & {"Spin", "BackUp", "DriveOnHeading"} == set()
    with pytest.raises(ValueError):
        nav2_config.behavior_tree_xml(recovery_retries=-1)
    choice = stack.PlanningChoice(server_params=(("general_goal_checker.xy_goal_tolerance", 0.35),
                                                 ("progress_checker.movement_time_allowance", 30.0)),
                                  recovery_retries=6, recovery_wait_s=5.0)
    assert not choice.is_default and stack.PlanningChoice.from_dict(choice.as_dict()) == choice
    p = params(choice)
    server = p["controller_server"]["ros__parameters"]
    assert server["general_goal_checker"]["xy_goal_tolerance"] == 0.35
    assert server["progress_checker"]["movement_time_allowance"] == 30.0
    expected = nav2_config.readback_expectations(nav2_config.NavProfile(), "navigate", planning=choice)
    assert expected["controller_server"]["general_goal_checker.xy_goal_tolerance"] == 0.35
    for bad in ((("FollowPath.allow_reversing", True),), (("failure_tolerance", 3.0),)):
        with pytest.raises(ValueError):
            stack.PlanningChoice(server_params=bad)
    with pytest.raises(ValueError):
        stack.PlanningChoice(recovery_wait_s=0.0)


@pytest.mark.parametrize("through", [False, True])
def test_fallback_planner_is_asked_only_after_the_primary(through):
    import xml.etree.ElementTree as ET
    choice = stack.PlanningChoice(planner="smac_hybrid", fallback_planner="smac_lattice")
    root = ET.fromstring(nav2_config.behavior_tree_xml(through_poses=through, **choice.behavior_tree_options()))
    fallback = list(root.iter("Fallback"))
    assert len(fallback) == 1
    tag = "ComputePathThroughPoses" if through else "ComputePathToPose"
    assert [e.get("planner_id") for e in fallback[0] if e.tag == tag] == ["GridBased", "Fallback"]
    assert {e.tag for e in root.iter()} & {"Spin", "BackUp", "DriveOnHeading"} == set()
    p = params(choice)["planner_server"]["ros__parameters"]
    assert p["planner_plugins"] == ["GridBased", "Fallback"]
    assert p["Fallback"]["plugin"] == stack.PLANNER_PLUGINS["smac_lattice"]
    assert p["Fallback"]["allow_reverse_expansion"] is False and p["GridBased"]["motion_model_for_search"] == "DUBIN"
    expected = nav2_config.readback_expectations(nav2_config.NavProfile(), "navigate", planning=choice)
    for dotted, value in expected["planner_server"].items():
        current = p
        for part in dotted.split("."):
            current = current[part]
        assert _same(current, value), dotted
    assert stack.PlanningChoice.from_dict(choice.as_dict()) == choice
    with pytest.raises(ValueError):
        stack.PlanningChoice(planner="navfn", fallback_planner="navfn")
    default = nav2_config.behavior_tree_xml(through_poses=through)
    assert 'planner_id="Fallback"' not in default and not list(ET.fromstring(default).iter("Fallback"))


def test_planning_on_the_saved_map_only_is_explicit():
    choice = stack.PlanningChoice(planner="smac_hybrid", global_live_scans=False)
    p = params(choice)
    plugins = p["global_costmap"]["global_costmap"]["ros__parameters"]["plugins"]
    assert plugins == ["static_layer", "inflation_layer"]
    assert "obstacle_layer" in p["local_costmap"]["local_costmap"]["ros__parameters"]["plugins"]
    expected = nav2_config.readback_expectations(nav2_config.NavProfile(), "navigate", planning=choice)
    assert expected["global_costmap/global_costmap"]["plugins"] == plugins
    assert "obstacle_layer" in params(stack.PlanningChoice(planner="smac_hybrid"))[
        "global_costmap"]["global_costmap"]["ros__parameters"]["plugins"]


@pytest.mark.parametrize("mode", ["amcl", "truth"])
def test_truth_localization_replaces_amcl_only_when_asked(tmp_path, mode):
    import yaml
    from wc_navigation.autonomy import cli
    from wc_navigation.autonomy.bringup import Supervisor, truth_localization
    extra = [] if mode == "amcl" else ["--localization", "truth"]
    config = cli.build_config(cli.arguments(["--project-root", str(PROJECT), "navigate", "--world", "two_rooms",
                                             "--goal", "8.0", "1.4", "0.0", *extra]))
    assert config["localization"] == mode and truth_localization(config) is (mode == "truth")
    out = tmp_path / "run"
    out.mkdir()
    cli.write_world_map(config, out)
    _, files = Supervisor(PROJECT, out, config).prepare()
    sim = json.loads((out / "sim_config.json").read_text())
    gate = json.loads((out / "gate_config.json").read_text())["gate"]
    mapping = yaml.safe_load(Path(files["mapping"]).read_text())
    if mode == "truth":
        assert sim["map_to_odom_from_truth"] == {"world_T_map": [0.0, 0.0, 0.0]}
        assert gate["require_covariance"] is False and "amcl" not in mapping
        assert mapping["lifecycle_manager_localization"]["ros__parameters"]["node_names"] == ["map_server"]
    else:
        assert "map_to_odom_from_truth" not in sim and gate["require_covariance"] is True and "amcl" in mapping


def test_simulation_and_shadow_configs_record_the_choice(tmp_path):
    from wc_navigation.autonomy import cli, shadow_cli
    config = cli.build_config(cli.arguments(["--project-root", str(PROJECT), "explore", "--world", "two_rooms",
                                             "--planner", "smac_hybrid", "--planner-param",
                                             "minimum_turning_radius=0.5"]))
    assert config["planning"] == {"planner": "smac_hybrid", "controller": "rpp",
                                  "planner_params": [["minimum_turning_radius", 0.5]], "controller_params": [],
                                  "server_params": [], "recovery_retries": 4, "recovery_wait_s": 3.0,
                                  "global_live_scans": True, "fallback_planner": ""}
    default = cli.build_config(cli.arguments(["--project-root", str(PROJECT), "explore", "--world", "two_rooms"]))
    assert stack.PlanningChoice.from_dict(default["planning"]).is_default
    args = shadow_cli.arguments(["--project-root", str(PROJECT), "--source-cell", str(tmp_path),
                                 "--map", str(tmp_path / "m.yaml"), "--planner", "theta_star"])
    assert args.planning.planner == "theta_star" and args.planning.controller == "rpp"


MEASURED_ARGS = ["0.81", "0.2425", "0.2", "0.35", "-0.255", "0.35", "-0.255", "-0.35", "0.2", "-0.35", "0.81", "-0.2425"]
RECOMMENDED = ["--planner", "smac_hybrid", "--planner-fallback", "smac_lattice", "--controller", "shim_rpp",
               "--footprint", *MEASURED_ARGS, "--footprint-source", "MEASURED_20261107"]


def test_recommended_scheme_builds_a_forward_only_stack_that_reads_back(tmp_path):
    """The command line recommended in docs/architecture/planning_control.md (an option, not the default)."""
    import yaml
    from wc_navigation.autonomy import cli, shadow_cli
    from wc_navigation.autonomy.bringup import Supervisor
    from wc_navigation.planning.body import Body
    config = cli.build_config(cli.arguments(["--project-root", str(PROJECT), "navigate", "--world", "two_rooms",
                                             "--goal", "8.0", "1.4", "0.0", *RECOMMENDED]))
    choice = stack.PlanningChoice.from_dict(config["planning"])
    assert (choice.planner, choice.fallback_planner, choice.controller) == ("smac_hybrid", "smac_lattice", "shim_rpp")
    assert not choice.is_default and config["body_model"] == "POLYGON_MEASURED_20261107"
    body = Body.from_profile(config["profile"])
    assert body.inscribed_m == pytest.approx(0.255) and body.circumscribed_m == pytest.approx(0.846, abs=5e-4)

    out = tmp_path / "run"
    out.mkdir()
    cli.write_world_map(config, out)
    _, files = Supervisor(PROJECT, out, config).prepare()
    p = yaml.safe_load(Path(files["nav2"]).read_text())
    planner = p["planner_server"]["ros__parameters"]
    assert planner["planner_plugins"] == ["GridBased", "Fallback"]
    assert planner["GridBased"]["plugin"] == stack.PLANNER_PLUGINS["smac_hybrid"]
    assert planner["GridBased"]["motion_model_for_search"] == "DUBIN"
    assert planner["Fallback"]["plugin"] == stack.PLANNER_PLUGINS["smac_lattice"]
    assert planner["Fallback"]["allow_reverse_expansion"] is False
    follow = p["controller_server"]["ros__parameters"]["FollowPath"]
    assert follow["plugin"] == stack.CONTROLLER_PLUGINS["shim_rpp"] and follow["primary_controller"] == stack.RPP_PLUGIN
    assert follow["use_rotate_to_heading"] is False and follow["allow_reversing"] is False
    assert follow["use_collision_detection"] is True

    # the measured outline is checked in both costmaps: with the margin for planning, as it is for control
    profile = nav2_config.NavProfile(**config["profile"])
    outline = nav2_config.footprint_string(profile.footprint)
    for name, padding in (("global_costmap", profile.margin_m), ("local_costmap", 0.0)):
        costmap = p[name][name]["ros__parameters"]
        assert costmap["footprint"] == outline and costmap["footprint_padding"] == pytest.approx(padding)
    inflation = p["global_costmap"]["global_costmap"]["ros__parameters"]["inflation_layer"]["inflation_radius"]
    assert inflation >= body.circumscribed_m + profile.margin_m

    tree = Path(files["bt_to_pose"]).read_text()
    assert 'planner_id="Fallback"' in tree
    assert not any(tag in tree for tag in ("<Spin", "<BackUp", "<DriveOnHeading"))
    for node, values in nav2_config.readback_expectations(profile, "navigate", planning=choice).items():
        for dotted, value in values.items():
            assert _same(_lookup(p, node, dotted), value), (node, dotted)

    args = shadow_cli.arguments(["--project-root", str(PROJECT), "--source-cell", str(tmp_path),
                                 "--map", str(tmp_path / "m.yaml"), *RECOMMENDED])
    assert args.planning == choice
