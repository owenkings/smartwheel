"""Moving pedestrians in the simulator: scenario checks, walking, yielding, world hook, log summary, CLI wiring."""
import json
import math
from pathlib import Path

import numpy as np
import pytest

from wc_navigation.autonomy import actors
from wc_navigation.autonomy.kinematics import BaseLimits, DiffDriveSim
from wc_navigation.autonomy.world import CircleObstacle, SimWorld
from wc_navigation.geometry import Grid
from wc_navigation.planning.body import Body

PROJECT = Path(__file__).resolve().parents[2]
MEASURED = ((0.81, 0.2425), (0.2, 0.35), (-0.255, 0.35), (-0.255, -0.35), (0.2, -0.35), (0.81, -0.2425))


def corridor(length=12.0, width=2.4, resolution=0.05):
    data = np.zeros((int(round(width / resolution)) + 4, int(round(length / resolution))), dtype=np.int8)
    data[:2, :] = data[-2:, :] = 100
    return SimWorld(Grid(data, resolution, (0.0, -width / 2 - 2 * resolution, 0.0)), name="corridor")


def walker(**kwargs):
    value = {"id": "w", "waypoints": [[8.0, 0.0], [2.0, 0.0]], "speed_mps": 1.0}
    value.update(kwargs)
    return value


def write_scenario(tmp_path, items, name="test"):
    path = tmp_path / "scenario.json"
    path.write_text(json.dumps({"schema_version": 1, "name": name, "actors": items}))
    return path


@pytest.mark.parametrize("bad", [{"id": "a b"}, {"waypoints": []}, {"waypoints": [[0, float("nan")]]},
                                 {"speed_mps": 3.0}, {"radius_m": 0.05}, {"repeat": "forever"},
                                 {"speed_mps": 0.0}, {"yield_gap_m": -1}, {"colour": "red"}])
def test_scenarios_are_validated(bad):
    with pytest.raises(ValueError):
        actors.parse_actors([walker(**bad)])
    with pytest.raises(ValueError):
        actors.parse_actors([walker(), walker()])            # duplicate ids


def test_actor_waits_for_the_first_command_then_walks_pauses_and_returns():
    spec = actors.parse_actors([walker(start_after_s=1.0, pause_s=0.5, repeat="pingpong")])[0]
    a = actors.Actor(spec)
    a.step(10.0, 0.1, None)
    assert a.state == "waiting" and (a.x, a.y) == (8.0, 0.0)
    t = 20.0
    for _ in range(9):                                       # armed at 20 s, starts 1 s later
        a.step(t, 0.1, 20.0)
        t += 0.1
    assert a.state == "waiting"
    for _ in range(61):
        a.step(t, 0.1, 20.0)
        t += 0.1
    assert a.x == pytest.approx(2.0) and a.state in ("paused", "walking")
    for _ in range(10):
        a.step(t, 0.1, 20.0)
        t += 0.1
    assert a.x > 2.0                                         # walking back after the pause
    once = actors.Actor(actors.parse_actors([walker(speed_mps=2.0)])[0])
    for k in range(40):
        once.step(k * 0.1, 0.1, 0.0)
    assert once.state == "done" and once.x == pytest.approx(2.0)


def test_yielding_actor_keeps_its_gap_and_a_rude_one_walks_into_the_chair():
    body = Body(outline=MEASURED, source="MEASURED_20261107")
    chair = (4.0, 0.0, 0.0)                                  # nose at x 4.81, facing the walker
    polite = actors.Actor(actors.parse_actors([walker(yield_gap_m=0.5)])[0])
    rude = actors.Actor(actors.parse_actors([walker(id="r")])[0])
    for k in range(80):
        polite.step(k * 0.1, 0.1, 0.0, chair, body)
        rude.step(k * 0.1, 0.1, 0.0, chair, body)
    gap = actors.body_gap(chair, body, polite.x, polite.y, 0.25)
    # it stops before the step that would bring it inside the gap (steps of 0.1 m here)
    assert 0.5 - 1e-6 <= gap < 0.6 + 1e-6 and polite.state == "yielding" and polite.yield_s > 0
    assert rude.x == pytest.approx(2.0)                       # walked through the chair's place


def test_simulator_hook_adds_moving_discs_vetoes_motion_and_logs(tmp_path):
    world = corridor()
    world.obstacles = (CircleObstacle(10.0, 0.9, 0.2),)
    body = Body(outline=MEASURED, source="MEASURED_20261107")
    config = {"world": {"actors": [walker(waypoints=[[6.0, 0.0], [3.0, 0.0]], speed_mps=0.5)]},
              "start_pose": [2.0, 0.0, 0.0]}
    sim = actors.SimActors.from_config(config, body, world, log_path=tmp_path / actors.LOG_NAME)
    base = DiffDriveSim((2.0, 0.0, 0.0), BaseLimits())
    t, dt = 100.0, 0.01
    for _ in range(100):                                     # no command yet: the walker stands
        sim.update(t, dt, base, world)
        base.step(dt, t, lambda a, b: world.swept_body_collides(a, b, body))
        t += dt
    assert len(world.obstacles) == 2 and world.obstacles[0].x == 10.0 and world.obstacles[1].x == 6.0
    ranges = world.raycast((2.4, 0.0), np.array([0.0]), 8.0)
    assert ranges[0] == pytest.approx(6.0 - 0.25 - 2.4, abs=0.02)
    for _ in range(1500):                                    # drive forward at 0.3 m/s into the walker
        base.set_command(0.3, 0.0, t)
        sim.on_command(0.3, 0.0, t)
        sim.update(t, dt, base, world)
        base.step(dt, t, lambda a, b: world.swept_body_collides(a, b, body))
        t += dt
    assert base.collisions >= 1                              # the disc vetoed the chair's motion
    lines = [json.loads(line) for line in (tmp_path / actors.LOG_NAME).read_text().splitlines()]
    events = [r["event"] for r in lines if "event" in r]
    assert events[0] == "first_command" and "contact_start" in events and "chair_blocked_by_actor" in events
    rows = [r for r in lines if "event" not in r]
    assert rows[1]["t"] - rows[0]["t"] == pytest.approx(0.1, abs=0.011)
    section = actors.result_entries({"world": config["world"], "actor_scenario": {"name": "t"}}, tmp_path)["actors"]
    entry = section["per_actor"]["w"]
    assert entry["contacts"] >= 1 and entry["chair_blocked_by_actor"] >= 1 and entry["min_gap_m"] <= 0.005
    assert section["first_command_sim_s"] == pytest.approx(101.0, abs=0.02)
    assert entry["seconds_within_0.5_m"] > 0 and section["chair_stopped_s"]["actor_within_1_5_m"] > 0


def test_actors_on_the_start_pose_are_refused():
    body = Body(outline=MEASURED, source="MEASURED_20261107")
    with pytest.raises(ValueError):
        actors.check_start([walker(waypoints=[[2.5, 0.0], [5.0, 0.0]])], (2.0, 0.0, 0.0), body)
    actors.check_start([walker()], (2.0, 0.0, 0.0), body)


def test_summary_stops_at_the_end_of_the_task(tmp_path):
    items = [walker()]
    log = [{"event": "first_command", "t": 100.0}]
    for k in range(40):                                      # 4 s: 2 s driving, then standing at the goal
        t = 100.0 + 0.1 * k
        log.append({"t": round(t, 1), "armed": True, "chair": [0.0, 0.0, 0.0], "chair_v": 0.3 if k < 20 else 0.0,
                    "chair_w": 0.0, "actors": [{"id": "w", "x": 8.0, "y": 0.0, "gap_m": 7.0, "state": "waiting"}]})
    (tmp_path / actors.LOG_NAME).write_text("\n".join(json.dumps(r) for r in log) + "\n")
    (tmp_path / "mission_result.json").write_text(json.dumps({"records": [{"finished_time_s": 101.95}]}))
    section = actors.result_entries({"world": {"actors": items}}, tmp_path)["actors"]
    assert section["active_until_sim_s"] == 101.95
    assert section["chair_stopped_s"] == {"actor_within_1_5_m": 0.0, "no_actor_within_1_5_m": 0.0}
    (tmp_path / "mission_result.json").unlink()
    section = actors.result_entries({"world": {"actors": items}}, tmp_path)["actors"]
    assert section["chair_stopped_s"]["no_actor_within_1_5_m"] == pytest.approx(2.0)


def test_runs_without_actors_are_unchanged_and_with_actors_carry_the_scenario(tmp_path):
    from wc_navigation.autonomy.bringup import Supervisor
    from wc_navigation.autonomy.cli import arguments, build_config
    base_args = ["--project-root", str(PROJECT), "navigate", "--world", "two_rooms", "--start", "1.2", "1.4", "0.0",
                 "--goal", "8.0", "1.4", "0.0"]
    plain = build_config(arguments(base_args))
    assert "actors" not in plain["world"] and "actor_scenario" not in plain
    assert actors.result_entries(plain, tmp_path) == {}
    path = write_scenario(tmp_path, [walker(waypoints=[[8.0, 1.4], [3.0, 1.4]])])
    config = build_config(arguments(base_args + ["--actors", str(path)]))
    assert config["world"]["actors"][0]["id"] == "w" and config["actor_scenario"]["sha256"]
    assert {k: v for k, v in config.items() if k not in ("world", "actor_scenario")} == \
        {k: v for k, v in plain.items() if k != "world"}
    out = tmp_path / "run"
    out.mkdir()
    Supervisor(PROJECT, out, config).prepare()
    sim_config = json.loads((out / "sim_config.json").read_text())
    assert sim_config["world"]["actors"] == config["world"]["actors"]
    with pytest.raises(SystemExit):
        arguments(base_args + ["--actors", str(write_scenario(tmp_path, [walker(speed_mps=9.0)]))])
