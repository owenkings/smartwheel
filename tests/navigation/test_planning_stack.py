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


def test_simulation_and_shadow_configs_record_the_choice(tmp_path):
    from wc_navigation.autonomy import cli, shadow_cli
    config = cli.build_config(cli.arguments(["--project-root", str(PROJECT), "explore", "--world", "two_rooms",
                                             "--planner", "smac_hybrid", "--planner-param",
                                             "minimum_turning_radius=0.5"]))
    assert config["planning"] == {"planner": "smac_hybrid", "controller": "rpp",
                                  "planner_params": [["minimum_turning_radius", 0.5]], "controller_params": []}
    default = cli.build_config(cli.arguments(["--project-root", str(PROJECT), "explore", "--world", "two_rooms"]))
    assert stack.PlanningChoice.from_dict(default["planning"]).is_default
    args = shadow_cli.arguments(["--project-root", str(PROJECT), "--source-cell", str(tmp_path),
                                 "--map", str(tmp_path / "m.yaml"), "--planner", "theta_star"])
    assert args.planning.planner == "theta_star" and args.planning.controller == "rpp"
