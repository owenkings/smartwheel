"""Goal heading at arrival (--goal-heading turn_at_goal): heading choice, turn options and the swept-area check."""
import json
import math
from pathlib import Path

import numpy as np
import pytest

from wc_navigation.planning import goal_heading as gh
from wc_navigation.planning.body import Body, wrap

PROJECT = Path(__file__).resolve().parents[2]
OUTLINE = ((0.81, 0.2425), (0.2, 0.35), (-0.255, 0.35), (-0.255, -0.35), (0.2, -0.35), (0.81, -0.2425))  # MEASURED_20261107
MEASURED_ARGS = ["0.81", "0.2425", "0.2", "0.35", "-0.255", "0.35", "-0.255", "-0.35", "0.2", "-0.35", "0.81", "-0.2425"]
RES = 0.05


def body():
    return Body(outline=OUTLINE, source="POLYGON_MEASURED_20261107")


def room(cells=(), size_m=6.0):
    """Free square room centred on the origin (outside counts as solid), with extra solid cells at world points."""
    n = int(round(size_m / RES))
    data = np.zeros((n, n), dtype=np.int16)
    origin = (-size_m / 2, -size_m / 2, 0.0)
    for x, y in cells:
        data[int((y - origin[1]) // RES), int((x - origin[0]) // RES)] = 100
    return data, origin


def test_candidates_start_at_the_requested_heading():
    c = gh.candidate_headings(2.5, 12)
    assert len(c) == 12 and c[0] == pytest.approx(2.5)
    steps = [abs(wrap(b - a)) for a, b in zip(c, c[1:])]
    assert steps == pytest.approx([math.pi / 6] * 11)
    assert gh.candidate_headings(-1.0, 1) == [pytest.approx(-1.0)]
    with pytest.raises(ValueError):
        gh.candidate_headings(0.0, 0)


def test_arrival_direction_is_the_last_half_metre():
    east = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)]
    assert gh.path_length(east) == pytest.approx(2.0)
    assert gh.arrival_direction(east) == pytest.approx(0.0)
    corner = [(0.0, 0.0), (2.0, 0.0), (2.0, 0.1), (2.0, 0.2), (2.0, 0.3), (2.0, 0.4), (2.0, 0.5), (2.0, 0.6)]
    assert gh.arrival_direction(corner) == pytest.approx(math.pi / 2)
    assert gh.arrival_direction([(0.0, 0.0), (0.1, 0.1)]) == pytest.approx(math.pi / 4)   # shorter than 0.5 m
    assert gh.arrival_direction([(1.0, 1.0), (1.0, 1.0)]) is None
    assert gh.arrival_direction([(1.0, 1.0)]) is None


def test_the_requested_heading_is_kept_unless_it_costs_a_detour():
    heading, reason, shortest = gh.choose_arrival(2.5, [(2.5, 8.2), (0.0, 8.0), (0.5, 8.1)])
    assert (heading, reason, shortest) == (2.5, "REQUESTED_HEADING_NO_DETOUR", 8.0)
    heading, reason, _ = gh.choose_arrival(2.5, [(2.5, 12.0), (0.0, 8.0), (0.5, 8.2), (1.0, 9.5), (2.0, None)])
    assert heading == pytest.approx(0.5) and reason == "SHORTER_PATH_AT_ANOTHER_HEADING"
    assert gh.choose_arrival(2.5, [(2.5, None), (0.0, None)]) == (2.5, "NO_PATH_FOR_ANY_CANDIDATE", None)


def test_turn_options_try_the_shorter_direction_first():
    a, b = gh.turn_options(0.0, math.pi / 2)
    assert a == pytest.approx(math.pi / 2) and b == pytest.approx(-1.5 * math.pi)
    a, b = gh.turn_options(3.0, -3.0)                       # across +-pi
    assert a == pytest.approx(2 * math.pi - 6.0) and b == pytest.approx(a - 2 * math.pi)
    assert gh.turn_options(1.0, 1.0) == [0.0]


def test_rotation_samples_move_no_body_point_more_than_half_a_cell():
    b = body()
    poses, grow = gh.rotation_samples((1.0, 2.0, 0.3), -1.5 * math.pi, b, RES)
    assert poses[0][2] == pytest.approx(0.3) and poses[-1][2] == pytest.approx(0.3 - 1.5 * math.pi)
    outlines = [b.outline_at(p) for p in poses]
    step = max(float(np.max(np.hypot(*(q - p).T))) for p, q in zip(outlines, outlines[1:]))
    assert step <= RES / 2 + 1e-9 and grow == pytest.approx(step / 2, abs=1e-3)


def test_turn_needs_a_clear_sweep_and_tries_the_other_direction():
    b = body()
    pose = (0.0, 0.0, 0.0)
    data, origin = room()
    clear_world = gh.grid_world(data, RES, origin, blocked=gh.saved_map_blocked)
    out = gh.decide_turn([("saved_map", clear_world, pose)], 0.0, math.pi, b, margin=0.10, tolerance=0.35)
    assert out["reason"] == "SWEEP_CLEAR_SHORTER" and abs(out["angle"]) == pytest.approx(math.pi)
    # A post in the front-left quadrant: the left turn sweeps the footrest into it, the right turn passes it.
    data, origin = room(cells=[(0.53, 0.53)])
    post = gh.grid_world(data, RES, origin, blocked=gh.saved_map_blocked)
    out = gh.decide_turn([("saved_map", post, pose)], 0.0, math.pi / 2, b, margin=0.10, tolerance=0.35)
    assert out["reason"] == "SWEEP_CLEAR_OTHER_DIRECTION" and out["angle"] == pytest.approx(-1.5 * math.pi)
    assert [o["clear"] for o in out["options"]] == [False, True]
    assert out["options"][0]["gaps_m"]["saved_map"] == 0.0 and out["options"][1]["gaps_m"]["saved_map"] > 0.1
    # A post where the footrest ends up: no direction reaches the heading, the chair stays.
    data, origin = room(cells=[(0.0, 0.75)])
    blocked = gh.grid_world(data, RES, origin, blocked=gh.saved_map_blocked)
    out = gh.decide_turn([("saved_map", blocked, pose)], 0.0, math.pi / 2, b, margin=0.10, tolerance=0.35)
    assert out["angle"] is None and out["reason"] == "SWEEP_BLOCKED" and len(out["options"]) == 2
    # Every world must be clear: the local costmap alone can block the turn.
    out = gh.decide_turn([("saved_map", clear_world, pose), ("local_costmap", blocked, pose)], 0.0, math.pi / 2, b,
                         margin=0.10, tolerance=0.35)
    assert out["reason"] == "SWEEP_BLOCKED"
    assert gh.decide_turn([], 0.0, math.pi / 2, b, margin=0.1, tolerance=0.35)["reason"] == "NO_OBSTACLE_DATA"
    out = gh.decide_turn([("saved_map", blocked, pose)], 0.0, 0.3, b, margin=0.1, tolerance=0.35)
    assert out["reason"] == "WITHIN_TOLERANCE" and out["angle"] is None and out["options"] == []


def test_margin_is_kept_to_walls_while_turning():
    b = body()
    data, origin = room(size_m=6.0)
    world = gh.grid_world(data, RES, origin, blocked=gh.saved_map_blocked)
    # The room edge (outside = solid) is 3 m away; 0.846 m turning radius + 0.10 m fits up to x = 2.05.
    assert gh.sweep_check(world, (2.0, 0.0, 0.0), math.pi, b, 0.10)[0]
    clear, gap = gh.sweep_check(world, (2.12, 0.0, 0.0), math.pi, b, 0.10)
    assert not clear and 0.0 < gap < 0.10


def test_blocked_cells_of_the_saved_map_and_the_costmap():
    values = np.array([[0, 100, -1, 50, 99, 65]])
    assert gh.saved_map_blocked(values).tolist() == [[False, True, True, False, True, True]]
    assert gh.costmap_blocked(values).tolist() == [[False, True, False, False, False, False]]


def test_mission_config_asks_heading_aware_planners_for_candidates():
    b = body()
    aware = gh.mission_config("smac_hybrid", b, margin_m=0.10)
    assert aware["heading_aware"] and aware["candidates"] == gh.CANDIDATES and aware["mode"] == "turn_at_goal"
    assert Body.from_config(aware["body"]) == b and aware["sweep_margin_m"] == 0.10
    free = gh.mission_config("navfn", b, margin_m=0.10)
    assert not free["heading_aware"] and free["candidates"] == 1
    json.dumps(aware, allow_nan=False)


def _prepared(tmp_path, extra):
    from wc_navigation.autonomy import cli
    from wc_navigation.autonomy.bringup import Supervisor
    config = cli.build_config(cli.arguments(["--project-root", str(PROJECT), "navigate", "--world", "two_rooms",
                                             "--goal", "8.0", "1.4", "0.0", *extra]))
    out = tmp_path / "run"
    out.mkdir(parents=True)
    cli.write_world_map(config, out)
    _, files = Supervisor(PROJECT, out, config).prepare()
    return config, json.loads(Path(files["task"]).read_text())


def test_goal_heading_is_an_explicit_option(tmp_path):
    from wc_navigation.autonomy import cli
    config, mission = _prepared(tmp_path, [])
    assert "mission" not in config and "goal_heading" not in mission
    config, mission = _prepared(tmp_path / "b", ["--goal-heading", "turn_at_goal", "--planner", "smac_hybrid",
                                                 "--footprint", *MEASURED_ARGS, "--footprint-source",
                                                 "MEASURED_20261107"])
    h = mission["goal_heading"]
    assert h == dict(config["mission"]["goal_heading"], yaw_tolerance_rad=0.35)
    assert h["heading_aware"] and Body.from_config(h["body"]).circumscribed_m == pytest.approx(0.846, abs=5e-4)
    _, mission = _prepared(tmp_path / "c", ["--goal-heading", "turn_at_goal", "--checker-param",
                                            "general_goal_checker.yaw_goal_tolerance=0.5"])
    assert mission["goal_heading"]["yaw_tolerance_rad"] == 0.5 and not mission["goal_heading"]["heading_aware"]
    with pytest.raises(SystemExit):
        cli.arguments(["--project-root", str(PROJECT), "explore", "--world", "two_rooms", "--goal-heading",
                       "turn_at_goal"])
