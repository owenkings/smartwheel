import pytest

from smartwheel_teleop.controller import SafetyState, TeleopController, motor_command_allowed


def make_controller():
    return TeleopController(0.3, 0.8, acceleration=1.0, deceleration=2.0, timeout_sec=0.3)


def test_wsad_combines_forward_and_left_with_ramp():
    controller = make_controller()
    controller.key_event("w", True, 0.0)
    controller.key_event("a", True, 0.0)
    controller.tick(0.0)
    command = controller.tick(0.2)
    assert command.linear == pytest.approx(0.2)
    assert command.angular == pytest.approx(0.2)


def test_timeout_ramps_to_stop():
    controller = make_controller()
    controller.key_event("W", True, 0.0)
    controller.tick(0.0)
    moving = controller.tick(0.2)
    assert moving.linear > 0.0
    stopping = controller.tick(0.4)
    assert not controller.command_fresh(0.4)
    assert stopping.linear == pytest.approx(0.0)


def test_space_stops_immediately():
    controller = make_controller()
    controller.key_event("W", True, 0.0)
    controller.tick(0.0)
    controller.tick(0.2)
    controller.key_event("SPACE", True, 0.21)
    assert controller.tick(0.21).linear == 0.0


def test_hardware_false_never_allows_motor_command():
    state = SafetyState(False, True, False, True)
    assert not motor_command_allowed(state)


def test_every_motor_gate_must_be_satisfied():
    assert motor_command_allowed(SafetyState(True, True, False, True))
    assert not motor_command_allowed(SafetyState(True, False, False, True))
    assert not motor_command_allowed(SafetyState(True, True, True, True))
    assert not motor_command_allowed(SafetyState(True, True, False, False))
