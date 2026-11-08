"""Safety gate: every failed condition yields an explicit zero twist."""
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


def test_covariance_is_necessary_only():
    gate = SafetyGate(GateConfig(require_covariance=True))
    small = [0.0] * 36
    small[0] = small[7] = 0.01
    small[35] = 0.01
    assert gate.check_covariance(small) == (True, "")
    large = list(small)
    large[0] = 0.36                       # 0.6 m standard deviation
    assert gate.check_covariance(large) == (False, "COVARIANCE_XY_TOO_LARGE")
    yaw = list(small)
    yaw[35] = 0.2
    assert gate.check_covariance(yaw) == (False, "COVARIANCE_YAW_TOO_LARGE")
    assert gate.check_covariance([math.nan] * 36) == (False, "COVARIANCE_INVALID")


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
    gate = SafetyGate(GateConfig(require_covariance=True))
    covariance = [0.0] * 36
    covariance[0], covariance[7], covariance[35] = 0.49 ** 2, 0.1, 0.01
    assert gate.check_covariance(covariance) == (True, "")
    covariance[7] = 0.51 ** 2
    assert gate.check_covariance(covariance) == (False, "COVARIANCE_XY_TOO_LARGE")


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
