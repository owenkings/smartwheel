"""Safety gate: every failed condition yields an explicit zero twist."""
import json
import math

import pytest

from wc_navigation.autonomy.safety_gate import CorrectionMonitor, GateConfig, SafetyGate


def active_gate(**config):
    gate = SafetyGate(GateConfig(**config))
    gate.on_enable(True, 0.0)
    gate.on_localization(True)
    gate.on_command(0.2, 0.1, 0.0)
    return gate


def test_passes_fresh_command_when_all_conditions_hold():
    decision = active_gate().evaluate(0.1)
    assert (decision.state, decision.linear, decision.angular) == ("ACTIVE", 0.2, 0.1)


def test_default_is_disabled_without_operator_enable():
    gate = SafetyGate()
    gate.on_localization(True)
    gate.on_command(0.2, 0.0, 0.0)
    assert gate.evaluate(0.1).state == "DISABLED"
    assert gate.evaluate(0.1).linear == 0.0


@pytest.mark.parametrize("now,state", [(0.4, "ACTIVE"), (0.6, "COMMAND_STALE")])
def test_stale_command_stops(now, state):
    gate = active_gate()
    gate.on_enable(True, now)
    assert gate.evaluate(now).state == state


def test_enable_heartbeat_timeout_stops():
    gate = active_gate()
    gate.on_command(0.2, 0.0, 1.5)
    decision = gate.evaluate(1.5)
    assert decision.state == "ENABLE_STALE" and decision.linear == 0.0


def test_untrusted_localization_stops_with_reason():
    gate = active_gate()
    gate.on_localization(False, "MAP_TO_BASE_TF_STALE")
    decision = gate.evaluate(0.1)
    assert decision.state == "LOCALIZATION_UNTRUSTED" and decision.reasons == ("MAP_TO_BASE_TF_STALE",)


def test_estop_latches_until_release_and_fresh_enable():
    gate = active_gate()
    gate.on_estop(True, 0.05)
    assert gate.evaluate(0.1).state == "ESTOP_LATCHED"
    gate.on_estop(False, 0.2)
    gate.on_enable(True, 0.3)            # continuing heartbeat is not a fresh enable
    gate.on_command(0.2, 0.0, 0.3)
    assert gate.evaluate(0.3).state == "ESTOP_LATCHED"
    gate.on_enable(False, 0.4)
    gate.on_enable(True, 0.5)
    gate.on_command(0.2, 0.0, 0.5)
    assert gate.evaluate(0.5).state == "ACTIVE"


def test_enable_while_estop_still_engaged_does_not_clear_latch():
    gate = active_gate()
    gate.on_estop(True, 0.1)
    gate.on_enable(False, 0.2)
    gate.on_enable(True, 0.3)
    assert gate.evaluate(0.3).state == "ESTOP_LATCHED"


def test_reverse_is_blocked_and_speeds_are_clamped():
    gate = active_gate(max_linear=0.3, max_angular=0.5)
    gate.on_command(-0.2, 0.9, 0.05)
    decision = gate.evaluate(0.1)
    assert decision.linear == 0.0 and decision.angular == 0.5
    assert set(decision.reasons) == {"REVERSE_BLOCKED", "ANGULAR_CLAMPED"}
    gate.on_command(0.8, -0.1, 0.1)
    decision = gate.evaluate(0.1)
    assert decision.linear == 0.3 and "LINEAR_CLAMPED" in decision.reasons


@pytest.mark.parametrize("value", [math.nan, math.inf, None, "0.1"])
def test_nonfinite_or_invalid_command_is_zeroed(value):
    gate = active_gate()
    gate.on_command(value, 0.1, 0.05)
    decision = gate.evaluate(0.1)
    assert decision.linear == 0.0 and decision.angular == 0.0


@pytest.mark.parametrize("mode,position_reason", [("xy", "COVARIANCE_XY_TOO_LARGE"),
                                                  ("lateral", "COVARIANCE_ACROSS_HEADING_TOO_LARGE")])
def test_covariance_is_necessary_only(mode, position_reason):
    gate = SafetyGate(GateConfig(require_covariance=True, covariance_mode=mode))
    small = [0.0] * 36
    small[0] = small[7] = 0.01
    small[35] = 0.01
    assert gate.check_covariance(small, 0.0) == (True, "")
    large = list(small)
    large[7] = 0.36                       # 0.6 m standard deviation across a heading along map x
    assert gate.check_covariance(large, 0.0) == (False, position_reason)
    yaw = list(small)
    yaw[35] = 0.2
    assert gate.check_covariance(yaw, 0.0) == (False, "COVARIANCE_YAW_TOO_LARGE")
    assert gate.check_covariance([math.nan] * 36, 0.0) == (False, "COVARIANCE_INVALID")


def test_slow_map_correction_is_trusted():
    monitor = CorrectionMonitor(window_s=2.0, max_translation_m=0.25, max_yaw_rad=math.radians(5), hold_s=3.0,
                                reference="odom_origin")
    for k in range(100):  # 1 cm and 0.2 degree per second of drift correction
        ok, reason = monitor.update(k * 0.1, (0.001 * k, 0.0, math.radians(0.02 * k)))
        assert ok and reason == ""
    assert monitor.events == 0


def test_fast_map_correction_stops_until_stable_for_hold_time():
    monitor = CorrectionMonitor(window_s=2.0, max_translation_m=0.25, max_yaw_rad=math.radians(5), hold_s=3.0,
                                reference="odom_origin")
    assert monitor.update(0.0, (0.0, 0.0, 0.0))[0]
    assert monitor.update(0.5, (0.0, 0.0, math.radians(8)))[1] == "MAP_CORRECTION_CHANGING_FAST"
    assert not monitor.update(2.4, (0.0, 0.0, math.radians(8)))[0]   # stable, but within hold time
    assert monitor.update(3.6, (0.0, 0.0, math.radians(8)))[0]       # stable for 3 s after the jump
    assert monitor.events == 1
    assert not monitor.update(3.8, (0.4, 0.0, math.radians(8)))[0]   # translation jump
    assert monitor.events == 2


def test_heading_wraparound_is_not_a_jump():
    monitor = CorrectionMonitor()
    assert monitor.update(0.0, (0.0, 0.0, math.pi - 0.01), (0.0, 0.0))[0]
    assert monitor.update(0.1, (0.0, 0.0, -math.pi + 0.01), (0.0, 0.0))[0]


@pytest.mark.parametrize("field,value", [("max_linear", 0), ("max_angular", -1), ("allow_reverse", 1),
                                         ("command_timeout_s", math.nan)])
def test_invalid_configuration_rejected(field, value):
    with pytest.raises(ValueError):
        GateConfig(**{field: value})


def test_robot_reference_ignores_heading_correction_lever_far_from_odom_origin():
    # 13 m from the odom origin a 1 degree heading correction moves map->odom by about 0.23 m,
    # while the robot itself, rotated in place about its own position, does not move.
    robot = (13.0, 0.0)
    rotate_about_robot = math.radians(1.0)
    c, s = math.cos(rotate_about_robot), math.sin(rotate_about_robot)
    shifted = (robot[0] - (c * robot[0] - s * robot[1]), robot[1] - (s * robot[0] + c * robot[1]), rotate_about_robot)
    origin = CorrectionMonitor(max_translation_m=0.2, max_yaw_rad=math.radians(5), reference="odom_origin")
    at_robot = CorrectionMonitor(max_translation_m=0.2, max_yaw_rad=math.radians(5), reference="robot")
    assert origin.update(0.0, (0.0, 0.0, 0.0))[0] and at_robot.update(0.0, (0.0, 0.0, 0.0), robot)[0]
    assert math.hypot(*shifted[:2]) > 0.2
    assert origin.update(0.5, shifted)[1] == "MAP_CORRECTION_CHANGING_FAST"
    assert at_robot.update(0.5, shifted, robot)[0]


def test_robot_reference_still_stops_on_a_translation_jump_and_needs_the_robot_pose():
    monitor = CorrectionMonitor(reference="robot")
    assert monitor.update(0.0, (0.0, 0.0, 0.0), (5.0, 1.0))[0]
    assert monitor.update(0.4, (0.3, 0.0, 0.0), (5.1, 1.0))[1] == "MAP_CORRECTION_CHANGING_FAST"
    assert monitor.events == 1
    with pytest.raises(ValueError):
        monitor.update(0.5, (0.3, 0.0, 0.0))
    with pytest.raises(ValueError):
        CorrectionMonitor(reference="map")


def test_defaults_measure_at_the_robot_with_ten_degrees_and_half_metre_covariance():
    default = CorrectionMonitor()
    assert default.reference == "robot" and default.max_yaw_rad == pytest.approx(math.radians(10))
    assert default.max_translation_m == 0.25 and default.window_s == 2.0 and default.hold_s == 3.0
    robot = (12.0, -3.0)
    assert default.update(0.0, (0.0, 0.0, 0.0), robot)[0]
    assert default.update(0.5, (0.0, 0.0, math.radians(9)), (0.0, 0.0))[0]       # 9 degrees at the origin: ok
    assert not default.update(1.0, (0.26, 0.0, math.radians(9)), (0.0, 0.0))[0]  # 0.26 m at the robot
    with pytest.raises(ValueError):
        default.update(1.1, (0.0, 0.0, 0.0))
    config = GateConfig()
    assert config.covariance_xy_std_max_m == 0.50 and config.covariance_yaw_std_max_rad == 0.25
    # lateral covariance mode since 2026-11-09 (user decision): 0.50 m across the heading, slowed above 0.50 m
    assert (config.covariance_mode, config.slow_std_m, config.slow_hysteresis_m, config.slow_linear) == (
        "lateral", 0.50, 0.05, 0.15)
    gate = SafetyGate(GateConfig(require_covariance=True))
    covariance = [0.0] * 36
    covariance[0], covariance[7], covariance[35] = 0.6 ** 2, 0.49 ** 2, 0.01     # heading along map x
    assert gate.on_covariance(covariance, 0.0) == (True, "") and gate.slow       # long along it: slowed
    covariance[7] = 0.51 ** 2
    assert gate.check_covariance(covariance, 0.0) == (False, "COVARIANCE_ACROSS_HEADING_TOO_LARGE")
    assert gate.check_covariance(covariance) == (False, "COVARIANCE_HEADING_UNKNOWN")   # needs the heading


def ticks(relocalization, start, end, pose, holding=True, available=True, dt=0.25):
    """Gate ticks at exact binary times; returns the times at which a no-motion update was requested."""
    requested = []
    for k in range(int(round((end - start) / dt)) + 1):
        t = start + k * dt
        if relocalization.update(t, holding, pose(t) if callable(pose) else pose, available):
            requested.append(t)
    return requested


def test_no_motion_updates_are_requested_only_for_a_covariance_hold_on_a_standing_robot():
    from wc_navigation.autonomy.safety_gate import StationaryRelocalization
    relocalization = StationaryRelocalization()
    # settles for 1 s first, then one request per second, five per stop
    assert ticks(relocalization, 0.0, 10.0, (2.0, 1.0, 0.3)) == [1.0, 2.0, 3.0, 4.0, 5.0]
    status = relocalization.status()
    assert status["requests_total"] == 5 and status["budget_exhausted"] and status["exhausted_stops"] == 1
    # moving by AMCL's own update distance renews the budget for the next stop
    assert ticks(relocalization, 10.25, 11.0, (2.12, 1.0, 0.3)) == []         # just moved: not settled yet
    assert ticks(relocalization, 11.25, 13.5, (2.12, 1.0, 0.3)) == [11.25, 12.25, 13.25]
    assert relocalization.status()["stops"] == 2


def test_no_request_while_moving_or_without_a_covariance_hold():
    from wc_navigation.autonomy.safety_gate import StationaryRelocalization
    assert ticks(StationaryRelocalization(), 0.0, 5.0, lambda t: (0.1 * t, 0.0, 0.0)) == []   # 0.1 m/s
    assert ticks(StationaryRelocalization(), 0.0, 5.0, (0.0, 0.0, 0.0), holding=False) == []
    assert StationaryRelocalization().update(5.0, True, None) is False
    assert ticks(StationaryRelocalization(enabled=False), 0.0, 5.0, (0.0, 0.0, 0.0)) == []
    turning = StationaryRelocalization()
    assert ticks(turning, 0.0, 0.5, (0.0, 0.0, 0.0)) == []
    assert ticks(turning, 0.75, 1.5, (0.0, 0.0, 0.02)) == []                  # 1.1 deg of turning restarts the settle
    assert ticks(turning, 1.75, 1.75, (0.0, 0.0, 0.02)) == [1.75]


def test_an_unavailable_service_does_not_use_the_budget():
    from wc_navigation.autonomy.safety_gate import StationaryRelocalization
    relocalization = StationaryRelocalization(max_requests=2)
    assert ticks(relocalization, 0.0, 3.0, (0.0, 0.0, 0.0), available=False) == []
    assert relocalization.status()["requests_total"] == 0 and relocalization.status()["service_unavailable_ticks"] == 3
    assert ticks(relocalization, 3.25, 6.0, (0.0, 0.0, 0.0)) == [4.0, 5.0]
    with pytest.raises(ValueError):
        StationaryRelocalization(max_requests=50)


# -- covariance modes (lateral: trial of 2026-11-09, the default since the user's decision the same day) -------
def ellipse(major, minor, angle, yaw_std=0.05):
    """Row-major 6x6 pose covariance with a position error ellipse at ``angle`` (rad) in the map."""
    c, s = math.cos(angle), math.sin(angle)
    xx = c * c * major ** 2 + s * s * minor ** 2
    yy = s * s * major ** 2 + c * c * minor ** 2
    xy = c * s * (major ** 2 - minor ** 2)
    covariance = [0.0] * 36
    covariance[0], covariance[1], covariance[6], covariance[7], covariance[35] = xx, xy, xy, yy, yaw_std ** 2
    return covariance


def test_covariance_spread_gives_the_ellipse_axes_and_the_heading_components():
    from wc_navigation.autonomy.safety_gate import covariance_spread
    spread = covariance_spread(ellipse(0.64, 0.2, math.radians(30)), math.radians(30))
    assert spread["major"] == pytest.approx(0.64) and spread["minor"] == pytest.approx(0.2)
    assert spread["along"] == pytest.approx(0.64) and spread["across"] == pytest.approx(0.2)
    turned = covariance_spread(ellipse(0.64, 0.2, math.radians(30)), math.radians(120))
    assert turned["along"] == pytest.approx(0.2) and turned["across"] == pytest.approx(0.64)
    assert covariance_spread([0.0] * 36)["major"] == 0.0 and "along" not in covariance_spread([0.0] * 36)
    assert covariance_spread([float("nan")] * 36) is None and covariance_spread([0.0] * 3) is None


def lateral_gate(**config):
    return active_gate(covariance_mode="lateral", require_covariance=True, **config)


def test_lateral_mode_stops_only_for_the_uncertainty_across_the_heading_or_in_the_heading():
    gate = lateral_gate()
    corridor = ellipse(0.64, 0.2, 0.0)             # long along map x, robot heading along x: B's stuck case
    assert gate.check_covariance(corridor, 0.0) == (True, "")
    assert gate.check_covariance(corridor, math.pi / 2) == (False, "COVARIANCE_ACROSS_HEADING_TOO_LARGE")
    assert gate.check_covariance(ellipse(0.3, 0.2, 0.0, yaw_std=0.3), 0.0) == (False, "COVARIANCE_YAW_TOO_LARGE")
    assert gate.check_covariance(corridor) == (False, "COVARIANCE_HEADING_UNKNOWN")
    assert gate.check_covariance([float("nan")] * 36, 0.0) == (False, "COVARIANCE_INVALID")
    # the xy mode (explicit since 2026-11-09): the same corridor ellipse stops, the heading is ignored
    xy = active_gate(require_covariance=True, covariance_mode="xy")
    assert xy.check_covariance(corridor, 0.0) == (False, "COVARIANCE_XY_TOO_LARGE")
    assert xy.check_covariance(corridor) == (False, "COVARIANCE_XY_TOO_LARGE")
    assert SafetyGate().config.covariance_mode == "lateral"


def drive(gate, start, end, linear=0.3, angular=0.1, dt=0.05):
    """Fresh commands and decisions every ``dt``; returns the decisions."""
    out = []
    for k in range(int(round((end - start) / dt)) + 1):
        t = start + k * dt
        gate.on_enable(True, t)
        gate.on_command(linear, angular, t)
        out.append(gate.evaluate(t))
    return out


def test_lateral_mode_slows_while_the_major_axis_is_large_and_keeps_the_curvature():
    gate = lateral_gate()
    assert drive(gate, 0.0, 0.2)[-1].linear == pytest.approx(0.3)     # nothing assessed yet: full speed
    assert gate.on_covariance(ellipse(0.64, 0.2, 0.0), 0.0) == (True, "") and gate.slow
    decisions = drive(gate, 0.25, 1.0)
    # ramps down at 0.6 m/s^2 (0.25 s), then holds 0.15 m/s with the same turn radius (0.3 m/s : 0.1 rad/s)
    assert decisions[0].linear == pytest.approx(0.3 - 0.6 * 0.05)
    assert decisions[0].angular == pytest.approx(0.1 * decisions[0].linear / 0.3)
    assert decisions[-1].linear == pytest.approx(0.15) and decisions[-1].angular == pytest.approx(0.05)
    assert all("LOCALIZATION_SLOW" in d.reasons and d.state == "ACTIVE" for d in decisions)
    # a slower command passes unchanged, a turn in place is not limited
    assert drive(gate, 1.05, 1.05, linear=0.1, angular=0.2)[-1].linear == pytest.approx(0.1)
    turn = drive(gate, 1.1, 1.1, linear=0.0, angular=0.4)[-1]
    assert (turn.linear, turn.angular) == (0.0, 0.4) and "LOCALIZATION_SLOW" in turn.reasons
    # hysteresis: still slow at 0.47 m, released at 0.44 m, then ramps up at 0.3 m/s^2 (0.5 s)
    gate.on_covariance(ellipse(0.47, 0.2, 0.0), 0.0)
    assert gate.slow and drive(gate, 1.15, 1.5)[-1].linear == pytest.approx(0.15)
    gate.on_covariance(ellipse(0.44, 0.2, 0.0), 0.0)
    assert not gate.slow
    rising = drive(gate, 1.55, 2.2)
    assert rising[0].linear == pytest.approx(0.15 + 0.3 * 0.05)
    assert rising[-1].linear == pytest.approx(0.3) and rising[-1].reasons == ()
    status = gate.covariance_status()
    assert status["mode"] == "lateral" and status["slow"] is False and status["std_major_m"] == pytest.approx(0.44)


def test_lateral_slow_down_starts_without_a_ramp_after_a_stop():
    gate = lateral_gate()
    gate.on_covariance(ellipse(0.64, 0.2, 0.0), 0.0)
    gate.on_localization(False, "MAP_CORRECTION_CHANGING_FAST")
    assert drive(gate, 0.0, 1.0)[-1].state == "LOCALIZATION_UNTRUSTED"
    gate.on_localization(True)
    assert drive(gate, 1.05, 1.05)[-1].linear == pytest.approx(0.15)     # the output was zero: no ramp needed


def test_the_xy_mode_never_slows():
    gate = active_gate(require_covariance=True, covariance_mode="xy")
    gate.on_covariance(ellipse(0.45, 0.1, math.radians(45)), 0.0)        # passes in xy (std_x = std_y = 0.33)
    decisions = drive(gate, 0.0, 1.0)
    assert not gate.slow and all((d.linear, d.angular, d.reasons) == (0.3, 0.1, ()) for d in decisions)
    assert gate.covariance_status() == {"mode": "xy"}


LATERAL_DEFAULT = {"covariance_mode": "lateral", "slow_std_m": 0.5, "slow_linear": 0.15}


def test_covariance_mode_settings_are_explicit_and_lateral_by_default():
    from wc_navigation.autonomy.safety_gate import covariance_mode_settings
    assert covariance_mode_settings() == LATERAL_DEFAULT and covariance_mode_settings("lateral") == LATERAL_DEFAULT
    assert covariance_mode_settings("xy") == {"covariance_mode": "xy"}
    assert covariance_mode_settings(None, (0.4, 0.1)) == {"covariance_mode": "lateral", "slow_std_m": 0.4,
                                                          "slow_linear": 0.1}
    for bad in (("xy", (0.5, 0.15)), ("lateral", (0.5, 0.4)), ("lateral", (0.02, 0.1)), ("lateral", (0.5, -1)),
                ("other", None)):
        with pytest.raises(ValueError):
            covariance_mode_settings(*bad)
    with pytest.raises(ValueError):
        GateConfig(covariance_mode="map")
    with pytest.raises(ValueError):
        GateConfig(covariance_mode="lateral", slow_linear=0.5)
    assert GateConfig().covariance_mode == "lateral"
    assert GateConfig(**LATERAL_DEFAULT) == GateConfig()          # the explicit record is the default gate


def test_simulation_navigate_records_the_covariance_mode_of_amcl_runs(tmp_path):
    from pathlib import Path
    from wc_navigation.autonomy import cli
    from wc_navigation.autonomy.bringup import Supervisor
    project = Path(__file__).resolve().parents[2]
    base = ["--project-root", str(project), "navigate", "--world", "two_rooms", "--goal", "8.0", "1.4", "0.0"]
    cases = (([], LATERAL_DEFAULT), (["--gate-covariance-mode", "xy"], {"covariance_mode": "xy"}),
             (["--gate-slow-limits", "0.4", "0.12"], {"covariance_mode": "lateral", "slow_std_m": 0.4,
                                                      "slow_linear": 0.12}),
             (["--localization", "truth"], {}))                  # truth runs do not check covariance
    for k, (extra, expected) in enumerate(cases):
        config = cli.build_config(cli.arguments(base + extra))
        assert config.get("gate_options", {}) == expected
        output = tmp_path / ("run%d" % k)
        output.mkdir()
        cli.write_world_map(config, output)
        _, files = Supervisor(project, output, config).prepare()
        gate = json.loads(Path(files["gate"]).read_text(encoding="utf-8"))["gate"]
        assert gate == {"max_linear": 0.3, "max_angular": 0.5, "require_covariance": bool(expected), **expected}
    for bad in (["--gate-covariance-mode", "lateral", "--localization", "truth"],
                ["--gate-slow-limits", "0.5", "0.15", "--localization", "truth"],
                ["--gate-covariance-mode", "xy", "--gate-slow-limits", "0.5", "0.15"]):
        with pytest.raises(SystemExit):
            cli.arguments(base + bad)
