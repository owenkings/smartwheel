"""Velocity-smoother feedback (--smoother-feedback gated) and the --enable-pause validation option."""
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from wc_navigation.planning import stack
from wc_navigation.planning import velocity_feedback as vf

PROJECT = Path(__file__).resolve().parents[2]
ACCEL, DECEL, RATE = 0.3, 0.6, 20.0          # nav2_config velocity_smoother: profile limits, 20 Hz


def smoother_step(current, command):
    """One tick of Humble's velocity smoother on one axis (applyConstraints with eta = 1)."""
    accelerating = abs(command) >= abs(current) and current * command >= 0.0
    limit = (ACCEL if accelerating else DECEL) / RATE
    return current + max(-limit, min(limit, command - current))


def chain(ticks, *, mode, gate_stop=(), slowdown=(), ratio=0.4, command=0.3):
    """Smoother -> monitor (slowdown ratio) -> gate (stop) for ``ticks``; returns the gated speeds.

    mode: "open_loop" (current = own last output), "gated" (``vf.feedback``) or
    "raw" (current = the gate's output itself, which the module does not do).
    Each tick the smoother uses the feedback computed after its previous output
    with the gate's output of the tick before (one tick of chain delay).
    """
    own = gated = 0.0
    previous_gated = 0.0
    out = []
    for k in range(ticks):
        if mode == "open_loop":
            current = own
        elif mode == "raw":
            current = previous_gated
        else:
            current = vf.feedback((own, 0.0), (previous_gated, 0.0), 0.0)[0]
        own = smoother_step(current, command)
        previous_gated = gated
        monitored = own * (ratio if k in slowdown else 1.0)
        gated = 0.0 if k in gate_stop else monitored
        out.append(gated)
    return out


def rises(values):
    return [b - a for a, b in zip(values, values[1:])]


def test_feedback_is_zero_only_while_the_gate_holds_the_chair():
    own = (0.3, 0.2)
    assert vf.feedback(own, (0.3, 0.2), 0.05) == own
    assert vf.feedback(own, (0.12, 0.08), 0.05) == own            # slowdown zone: scaled, not held
    assert vf.feedback(own, (0.0, 0.2), 0.05) == own              # turning in place
    assert vf.feedback(own, (0.0, 0.0), 0.05) == (0.0, 0.0)       # gate or monitor stop
    assert vf.feedback(own, (0.3, 0.2), 0.3) == (0.0, 0.0)        # no recent gate output
    assert vf.feedback(own, None, None) == (0.0, 0.0)             # gate not running yet
    assert vf.feedback(own, (0.3, 0.2), float("nan")) == (0.0, 0.0)


def test_release_after_a_gate_stop_ramps_instead_of_jumping():
    stop = range(40, 80)
    open_loop = chain(120, mode="open_loop", gate_stop=stop)
    gated = chain(120, mode="gated", gate_stop=stop)
    assert max(rises(open_loop[78:])) == pytest.approx(0.3)       # today: zero to full speed in one tick
    assert max(rises(gated[78:])) <= ACCEL / RATE + 1e-9          # ramps at the smoother's limit
    assert gated[-1] == pytest.approx(0.3)
    assert all(v == 0.0 for v in gated[40:80])                    # the stop itself is untouched


def test_slowdown_zone_keeps_its_speed_with_gated_feedback():
    zone = range(40, 400)
    gated = chain(400, mode="gated", slowdown=zone)
    assert gated[-1] == pytest.approx(0.4 * 0.3)                  # 0.12 m/s through the zone, as today
    raw = chain(400, mode="raw", slowdown=zone)
    # feeding back the scaled speed itself settles at accel/rate/(1 - ratio) * ratio: the chair would crawl
    assert raw[-1] == pytest.approx(ACCEL / RATE / (1 - 0.4) * 0.4, abs=1e-6)
    start_in_zone = chain(200, mode="gated", slowdown=range(0, 200))
    assert start_in_zone[-1] == pytest.approx(0.12)               # starting inside a zone still gets going


def test_choice_switches_only_the_smoother_to_closed_loop():
    from wc_navigation.autonomy import nav2_config
    from wc_navigation.autonomy.bringup import _same

    def params(choice):
        return nav2_config.navigation_parameters(nav2_config.NavProfile(), mode="navigate",
                                                 bt_to_pose_path="/tmp/a.xml", bt_through_poses_path="/tmp/b.xml",
                                                 planning=choice)
    base = params(None)
    for mode, topic, duration in (("gated", stack.FEEDBACK_TOPIC, stack.FEEDBACK_DURATION_S), ("odom", "odom", 0.1)):
        choice = stack.PlanningChoice(smoother_feedback=mode)
        p = params(choice)
        smoother = p["velocity_smoother"]["ros__parameters"]
        assert (smoother["feedback"], smoother["odom_topic"], smoother["odom_duration"]) == ("CLOSED_LOOP", topic, duration)
        assert {k: v for k, v in smoother.items() if k not in ("feedback", "odom_topic", "odom_duration")} == \
            {k: v for k, v in base["velocity_smoother"]["ros__parameters"].items()
             if k not in ("feedback", "odom_topic", "odom_duration")}
        for key in base:
            if key != "velocity_smoother":
                assert p[key] == base[key], (mode, key)
        expected = nav2_config.readback_expectations(nav2_config.NavProfile(), "navigate", planning=choice)
        assert expected["velocity_smoother"]["feedback"] == "CLOSED_LOOP"
        for dotted, value in expected["velocity_smoother"].items():
            assert _same(smoother[dotted], value), dotted
        assert stack.PlanningChoice.from_dict(json.loads(json.dumps(choice.as_dict()))) == choice
        assert not choice.is_default
    assert base["velocity_smoother"]["ros__parameters"]["feedback"] == "OPEN_LOOP"
    assert "smoother_feedback" not in stack.PlanningChoice().as_dict()
    with pytest.raises(ValueError):
        stack.PlanningChoice(smoother_feedback="closed")


def _arguments(extra, mode="navigate"):
    from wc_navigation.autonomy import cli
    world = ["--world", "two_rooms"] + (["--goal", "8.0", "1.4", "0.0"] if mode == "navigate" else [])
    return cli.arguments(["--project-root", str(PROJECT), mode, *world, *extra])


def test_gated_feedback_starts_the_relay_and_pause_is_explicit(tmp_path):
    from wc_navigation.autonomy import cli
    from wc_navigation.autonomy.bringup import Supervisor
    runs = {}
    for name, extra in (("open_loop", ["--smoother-feedback", "open_loop"]), ("default", []),
                        ("gated", ["--smoother-feedback", "gated", "--enable-pause", "30", "3"])):
        config = cli.build_config(_arguments(extra))
        out = tmp_path / name
        out.mkdir()
        cli.write_world_map(config, out)
        _, files = Supervisor(PROJECT, out, config).prepare()
        runs[name] = (config, files)
    config, files = runs["open_loop"]
    assert "feedback" not in files and "enable_pause" not in config
    assert "smoother_feedback" not in config["planning"]
    # simulation and recorded shadow runs default to the gated feedback (user decision 2026-11-10)
    config, files = runs["default"]
    assert config["planning"]["smoother_feedback"] == "gated" and "feedback" in files and "enable_pause" not in config
    config, files = runs["gated"]
    assert config["planning"]["smoother_feedback"] == "gated"
    assert config["enable_pause"] == {"after_s": 30.0, "duration_s": 3.0}
    relay = json.loads(Path(files["feedback"]).read_text())
    assert relay == vf.node_config(Path(files["feedback"]).parent)
    with pytest.raises(SystemExit):
        _arguments(["--enable-pause", "30", "31"])
    with pytest.raises(SystemExit):
        _arguments(["--enable-pause", "30", "3"], mode="explore")
    assert _arguments(["--smoother-feedback", "odom"], mode="explore").planning.smoother_feedback == "odom"


def test_rescue_planner_is_asked_last_and_reads_back():
    import xml.etree.ElementTree as ET
    from wc_navigation.autonomy import nav2_config
    from wc_navigation.autonomy.bringup import _same
    profile = nav2_config.NavProfile()

    def params(choice):
        return nav2_config.navigation_parameters(profile, mode="navigate", bt_to_pose_path="/tmp/a.xml",
                                                 bt_through_poses_path="/tmp/b.xml", planning=choice)
    choice = stack.PlanningChoice(planner="smac_hybrid", fallback_planner="smac_lattice", controller="shim_rpp",
                                  rescue_planner="navfn")
    p = params(choice)["planner_server"]["ros__parameters"]
    assert p["planner_plugins"] == ["GridBased", "Fallback", "Rescue"]
    assert p["Rescue"] == stack.planner_plugin("navfn", exploring=False)
    expected = nav2_config.readback_expectations(profile, "navigate", planning=choice)["planner_server"]
    assert expected["planner_plugins"] == ["GridBased", "Fallback", "Rescue"]
    for dotted, value in expected.items():
        current = p
        for part in dotted.split("."):
            current = current[part]
        assert _same(current, value), dotted
    for through in (False, True):
        tree = nav2_config.behavior_tree_xml(through_poses=through, **choice.behavior_tree_options())
        ids = [e.get("planner_id") for e in ET.fromstring(tree).iter() if e.get("planner_id")]
        assert ids == ["GridBased", "Fallback", "Rescue"]
    # without a rescue planner the tree is exactly the one before (fallback only)
    plain = stack.PlanningChoice(planner="smac_hybrid", fallback_planner="smac_lattice", controller="shim_rpp")
    assert nav2_config.behavior_tree_xml(**plain.behavior_tree_options()) == \
        nav2_config.behavior_tree_xml(fallback_planner=True)
    only = stack.PlanningChoice(planner="smac_hybrid", controller="shim_rpp", rescue_planner="navfn")
    assert params(only)["planner_server"]["ros__parameters"]["planner_plugins"] == ["GridBased", "Rescue"]
    assert stack.PlanningChoice.from_dict(json.loads(json.dumps(choice.as_dict()))) == choice
    assert "rescue_planner" not in plain.as_dict()
    for bad in ({"rescue_planner": "smac_lattice"}, {"planner": "navfn", "rescue_planner": "navfn"}):
        with pytest.raises(ValueError):
            stack.PlanningChoice(**bad)
    assert _arguments(["--planner", "smac_hybrid", "--planner-rescue", "navfn"]).planning.rescue_planner == "navfn"


def test_smoother_timeout_is_an_explicit_choice():
    from wc_navigation.autonomy import nav2_config
    from wc_navigation.autonomy.bringup import _same

    def params(choice):
        return nav2_config.navigation_parameters(nav2_config.NavProfile(), mode="navigate",
                                                 bt_to_pose_path="/tmp/a.xml", bt_through_poses_path="/tmp/b.xml",
                                                 planning=choice)
    base = params(None)
    assert base["velocity_smoother"]["ros__parameters"]["velocity_timeout"] == stack.DEFAULT_SMOOTHER_TIMEOUT_S == 1.0
    choice = stack.PlanningChoice(smoother_timeout_s=0.3)
    p = params(choice)
    assert p["velocity_smoother"]["ros__parameters"]["velocity_timeout"] == 0.3
    rest = {k: v for k, v in p["velocity_smoother"]["ros__parameters"].items() if k != "velocity_timeout"}
    assert rest == {k: v for k, v in base["velocity_smoother"]["ros__parameters"].items() if k != "velocity_timeout"}
    expected = nav2_config.readback_expectations(nav2_config.NavProfile(), "navigate", planning=choice)
    assert _same(expected["velocity_smoother"]["velocity_timeout"], 0.3)
    assert stack.PlanningChoice.from_dict(json.loads(json.dumps(choice.as_dict()))) == choice
    assert "smoother_timeout_s" not in stack.PlanningChoice().as_dict() and not choice.is_default
    for bad in (0.1, 6.0, float("nan")):
        with pytest.raises(ValueError):
            stack.PlanningChoice(smoother_timeout_s=bad)
    assert _arguments(["--smoother-timeout", "0.3"]).planning.smoother_timeout_s == 0.3
    assert _arguments([]).planning.smoother_timeout_s == 1.0
    with pytest.raises(SystemExit):
        _arguments(["--smoother-timeout", "0.1"])


def test_enable_pause_sends_false_only_inside_its_window():
    from wc_navigation.autonomy.bringup import Recorder
    sent = []
    fake = SimpleNamespace(enabled=True, enable_pause=(130.0, 133.0), Bool=lambda: SimpleNamespace(data=None),
                           enable_pub=SimpleNamespace(publish=lambda m: sent.append(m.data)), clock=0.0)
    fake.now = lambda: fake.clock
    for fake.clock in (129.9, 130.0, 132.9, 133.0):
        Recorder._heartbeat(fake)
    assert sent == [True, False, False, True]
    fake.enable_pause, fake.clock = None, 131.0
    Recorder._heartbeat(fake)
    fake.enabled = False
    Recorder._heartbeat(fake)
    assert sent[-2:] == [True, False]
