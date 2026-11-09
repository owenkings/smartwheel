"""People reaction wiring (--people-reaction, --tracks truth): Nav2 footprint radii, masks, filters, truth tracks."""
import json
import math
from pathlib import Path
from types import SimpleNamespace

import pytest

from wc_navigation.autonomy import nav2_config
from wc_navigation.autonomy.bringup import _same
from wc_navigation.planning import people_node as pn
from wc_navigation.planning import stack
from wc_navigation.planning.body import Body

PROJECT = Path(__file__).resolve().parents[2]
MEASURED = [[0.81, 0.2425], [0.2, 0.35], [-0.255, 0.35], [-0.255, -0.35], [0.2, -0.35], [0.81, -0.2425]]
MEASURED_ARGS = [str(v) for p in MEASURED for v in p]


def params(choice, footprint=MEASURED):
    return nav2_config.navigation_parameters(nav2_config.NavProfile(footprint=footprint), mode="navigate",
                                             bt_to_pose_path="/tmp/a.xml", bt_through_poses_path="/tmp/b.xml",
                                             planning=choice)


def test_footprint_radii_follow_nav2_padding():
    rectangle = [(0.79, 0.35), (-0.35, 0.35), (-0.35, -0.35), (0.79, -0.35)]
    inscribed, circumscribed = pn.footprint_radii(pn.nav2_footprint(rectangle, 0.0, 0.10))
    assert inscribed == pytest.approx(0.45) and circumscribed == pytest.approx(math.hypot(0.89, 0.45))
    inscribed, circumscribed = pn.footprint_radii(pn.nav2_footprint((), 0.55, 0.0))
    assert circumscribed == pytest.approx(0.55) and inscribed == pytest.approx(0.55 * math.cos(math.pi / 16))
    body = Body(outline=tuple(map(tuple, MEASURED)))
    inscribed, circumscribed = pn.footprint_radii(pn.nav2_footprint(body.outline, 0.0, 0.0))
    assert inscribed == pytest.approx(body.inscribed_m, abs=1e-6)
    assert circumscribed == pytest.approx(body.circumscribed_m, abs=1e-6)


def test_local_mask_window_covers_the_costmap_window():
    origin = pn.local_origin((1.23, -0.47), 140, 0.05)
    assert origin == pytest.approx((-2.3, -4.0))
    for axis, chair in enumerate((1.23, -0.47)):
        assert origin[axis] <= chair - 3.0 and origin[axis] + 140 * 0.05 >= chair + 3.0
    assert pn.speed_limit_value(None) == 0.0 and pn.speed_limit_value(0.12) == 0.12


def test_only_people_standing_for_a_while_get_keepout_zones():
    from wc_navigation.planning.people import Person
    walking = Person(1, 2.0, 0.0, -0.8, 0.0, 0.25, True, 0.1)
    standing = Person(2, 3.0, 1.0, 0.0, 0.0, 0.25, False)
    since = {}
    assert pn.masked_people([walking, standing], since, 10.0) == []           # just stopped
    assert pn.masked_people([walking, standing], since, 12.9) == []
    assert pn.masked_people([walking, standing], since, 13.0) == [standing]   # 3 s still
    assert pn.masked_people([walking, Person(2, 3.0, 1.0, 0.5, 0.0, 0.25, True, 0.1)], since, 14.0) == []
    assert since == {}                                                        # walking again: starts over
    assert pn.masked_people([standing], since, 15.0) == [] and since == {2: 15.0}
    assert pn.masked_people([], since, 16.0) == [] and since == {}


def test_node_config_takes_padding_and_inflation_from_the_costmaps(tmp_path):
    profile = nav2_config.NavProfile(footprint=MEASURED)
    body = Body.from_profile(profile)
    for planner, center in (("smac_hybrid", False), ("navfn", True)):
        choice = stack.PlanningChoice(planner=planner, controller="shim_rpp" if planner != "navfn" else "rpp",
                                      people_reaction=True)
        config = pn.node_config(params(choice), profile, body, choice, tmp_path)
        assert config["global"]["center_point"] is center and config["local"]["center_point"] is False
        assert config["local"]["inscribed_m"] == pytest.approx(body.inscribed_m, abs=1e-4)
        padded = pn.footprint_radii(pn.nav2_footprint(body.outline, 0.0, profile.margin_m))
        assert config["global"]["inscribed_m"] == pytest.approx(padded[0], abs=1e-4)
        assert config["global"]["circumscribed_m"] == pytest.approx(padded[1], abs=1e-4)
        assert config["local"]["cells"] >= 120 and config["local"]["cost_scaling_factor"] == 3.0
        json.dumps(config, allow_nan=False)
    with pytest.raises(ValueError):
        pn.node_config(params(choice), profile, body, choice, tmp_path, local_cells=100)


def test_people_keepout_adds_only_the_keepout_filters():
    base = params(None)
    assert params(stack.PlanningChoice(people_reaction=True)) == base         # the speed limit needs no parameter
    with pytest.raises(ValueError):
        stack.PlanningChoice(people_keepout=True)
    choice = stack.PlanningChoice(people_reaction=True, people_keepout=True)
    p = params(choice)
    for name in ("global_costmap", "local_costmap"):
        costmap = p[name][name]["ros__parameters"]
        assert costmap["filters"] == [stack.PEOPLE_FILTER]
        assert costmap[stack.PEOPLE_FILTER]["plugin"] == "nav2_costmap_2d::KeepoutFilter"
        assert costmap["plugins"] == base[name][name]["ros__parameters"]["plugins"]
        rest = {k: v for k, v in costmap.items() if k not in ("filters", stack.PEOPLE_FILTER)}
        assert rest == base[name][name]["ros__parameters"]
    for key in base:
        if key not in ("global_costmap", "local_costmap"):
            assert p[key] == base[key], key
    expected = nav2_config.readback_expectations(nav2_config.NavProfile(footprint=MEASURED), "navigate",
                                                 planning=choice)
    for name in ("global_costmap", "local_costmap"):
        values = expected[name + "/" + name]
        assert values["filters"] == [stack.PEOPLE_FILTER]
        assert values[stack.PEOPLE_FILTER + ".filter_info_topic"] == stack.PEOPLE_FILTER_INFO[name]
        for dotted, value in values.items():
            current = p[name][name]["ros__parameters"]
            for part in dotted.split("."):
                current = current[part]
            assert _same(current, value), dotted
    assert stack.PlanningChoice.from_dict(json.loads(json.dumps(choice.as_dict()))) == choice
    assert "people_reaction" not in stack.PlanningChoice().as_dict()


def _actor(x, y, state, target_xy=None, speed=0.8, walked=0.0):
    waypoints = [(x, y)] + ([target_xy] if target_xy else [])
    return SimpleNamespace(x=x, y=y, state=state, target=1 if target_xy else None, walked_m=walked,
                           spec=SimpleNamespace(waypoints=waypoints, speed_mps=speed, radius_m=0.25))


def test_truth_tracks_are_window_c_messages_in_the_odom_frame():
    from wc_navigation.autonomy.sim_node import truth_tracks
    from wc_navigation.perception.tracks import check_message
    first = {}
    actors = [_actor(3.0, 1.0, "walking", (3.0, 5.0)), _actor(4.0, 1.0, "paused", walked=2.0),
              _actor(5.0, 1.0, "waiting")]
    # chair at (2, 1, 0) in the world, at (1, 0, 90 deg) in odom
    message = truth_tracks(actors, (2.0, 1.0, 0.0), (1.0, 0.0, math.pi / 2), 120.0, first)
    assert check_message(message) is message and message["source"] == "truth" and message["frame"] == "odom"
    walking, paused, waiting = message["tracks"]
    assert (walking["x"], walking["y"]) == pytest.approx((1.0, 1.0), abs=1e-3)       # 1 m ahead of the chair
    assert (walking["vx"], walking["vy"]) == pytest.approx((-0.8, 0.0), abs=1e-3)     # world +y is odom -x
    assert walking["class"] == "moving" and walking["has_moved"] and walking["seen_by"] == ["truth"]
    assert paused["class"] == "person_candidate" and paused["has_moved"] and paused["speed"] == 0.0
    assert waiting["class"] == "person_candidate" and not waiting["has_moved"]
    later = truth_tracks(actors, (2.0, 1.0, 0.0), (1.0, 0.0, math.pi / 2), 121.0, first)
    assert later["tracks"][0]["age_s"] == pytest.approx(1.0)
    from wc_navigation.planning.people import people_from_message
    people, _ = people_from_message(message, 120.0)
    assert len(people) == 3 and people[0].moving and not people[1].moving


def _arguments(extra):
    from wc_navigation.autonomy import cli
    return cli.arguments(["--project-root", str(PROJECT), "navigate", "--world", "two_rooms",
                          "--goal", "8.0", "1.4", "0.0", "--footprint", *MEASURED_ARGS,
                          "--footprint-source", "MEASURED_20261107", *extra])


def test_people_reaction_is_explicit_and_needs_truth_tracks(tmp_path):
    from wc_navigation.autonomy import cli, shadow_cli
    from wc_navigation.autonomy.bringup import Supervisor
    with pytest.raises(SystemExit):
        _arguments(["--people-reaction"])
    with pytest.raises(SystemExit):
        _arguments(["--tracks", "truth", "--people-keepout"])
    assert _arguments(["--tracks", "truth", "--people-reaction", "--people-keepout"]).planning.people_keepout
    with pytest.raises(SystemExit):
        shadow_cli.arguments(["--project-root", str(PROJECT), "--source-cell", str(tmp_path),
                              "--map", str(tmp_path / "m.yaml"), "--people-reaction"])
    runs = {}
    for name, extra in (("default", []), ("people", ["--tracks", "truth", "--people-reaction"])):
        config = cli.build_config(_arguments(extra))
        out = tmp_path / name
        out.mkdir()
        cli.write_world_map(config, out)
        _, files = Supervisor(PROJECT, out, config).prepare()
        runs[name] = (config, files)
    config, files = runs["default"]
    assert "people" not in files and "tracks" not in config
    assert "tracks" not in json.loads(Path(files["sim"]).read_text())
    config, files = runs["people"]
    assert config["planning"]["people_reaction"] is True and config["tracks"] == {"source": "truth", "hz": 5.0}
    assert json.loads(Path(files["sim"]).read_text())["tracks"] == config["tracks"]
    people = json.loads(Path(files["people"]).read_text())
    assert people["log_jsonl"] == str(Path(files["people"]).parent / "people.jsonl")
    assert Body.from_config(people["body"]).is_polygon
