import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from wheelchair_safety.safety_supervisor_node import (  # noqa: E402
    SafetyParams,
    compute_dynamic_stop_distance,
    evaluate_safety,
)


def test_compute_dynamic_stop_distance_formula():
    distance = compute_dynamic_stop_distance(0.4, t_delay=0.35, a_brake=0.8, d_margin=0.25)
    assert distance == pytest_approx(0.4 * 0.35 + 0.4 * 0.4 / (2.0 * 0.8) + 0.25)


def test_ultrasonic_emergency_outputs_zero_velocity():
    decision = evaluate_safety(
        requested_linear_x=0.35,
        requested_angular_z=0.2,
        scan_distance=math.inf,
        ultrasonic_distances=[2.0, 0.2, 2.0],
        emergency_hw=False,
        emergency_sw=False,
        params=SafetyParams(),
    )
    assert decision.state == "EMERGENCY_STOP"
    assert decision.linear_x == 0.0
    assert decision.angular_z == 0.0


def test_physical_emergency_stop_has_priority_over_reverse_request():
    decision = evaluate_safety(
        requested_linear_x=-0.2,
        requested_angular_z=0.1,
        scan_distance=math.inf,
        ultrasonic_distances=[2.0, 2.0],
        emergency_hw=True,
        emergency_sw=False,
        params=SafetyParams(),
    )

    assert decision.state == "EMERGENCY_STOP"
    assert decision.linear_x == 0.0
    assert decision.angular_z == 0.0


def test_reverse_motion_is_stopped_by_default():
    decision = evaluate_safety(
        requested_linear_x=-0.1,
        requested_angular_z=0.0,
        scan_distance=math.inf,
        ultrasonic_distances=[2.0, 2.0],
        emergency_hw=False,
        emergency_sw=False,
        params=SafetyParams(),
    )

    assert decision.state == "STOP"
    assert "reverse motion is disabled" in decision.reason
    assert decision.linear_x == 0.0


def test_rotation_stops_when_ultrasonic_obstacle_is_too_close():
    decision = evaluate_safety(
        requested_linear_x=0.0,
        requested_angular_z=0.5,
        scan_distance=math.inf,
        ultrasonic_distances=[0.65, 2.0],
        emergency_hw=False,
        emergency_sw=False,
        params=SafetyParams(),
    )

    assert decision.state == "STOP"
    assert decision.angular_z == 0.0


def test_unmanned_profile_can_rotate_away_from_scan_block():
    decision = evaluate_safety(
        requested_linear_x=0.0,
        requested_angular_z=0.22,
        scan_distance=0.25,
        ultrasonic_distances=[math.inf],
        emergency_hw=False,
        emergency_sw=False,
        params=SafetyParams(
            max_angular_speed=0.22,
            stop_distance=0.32,
            emergency_distance=0.10,
            rotate_in_place_angular_threshold=0.2,
            allow_rotation_when_blocked=True,
        ),
    )

    assert decision.state == "WARNING"
    assert decision.linear_x == 0.0
    assert decision.angular_z == 0.22


def test_unmanned_rotation_is_not_scaled_to_zero_in_slowdown_zone():
    decision = evaluate_safety(
        requested_linear_x=0.0,
        requested_angular_z=-0.22,
        scan_distance=0.55,
        ultrasonic_distances=[0.34],
        emergency_hw=False,
        emergency_sw=False,
        params=SafetyParams(
            max_angular_speed=0.22,
            slowdown_distance=0.55,
            stop_distance=0.32,
            emergency_distance=0.10,
            rotation_stop_distance=0.12,
            rotate_in_place_angular_threshold=0.2,
            allow_rotation_when_blocked=True,
            ultrasonic_min_valid_m=0.10,
        ),
    )

    assert decision.state == "WARNING"
    assert decision.linear_x == 0.0
    assert decision.angular_z == -0.22


def test_unmanned_low_speed_rotation_is_not_scaled_to_zero():
    decision = evaluate_safety(
        requested_linear_x=0.0,
        requested_angular_z=-0.18,
        scan_distance=0.55,
        ultrasonic_distances=[0.34],
        emergency_hw=False,
        emergency_sw=False,
        params=SafetyParams(
            max_angular_speed=0.22,
            slowdown_distance=0.55,
            stop_distance=0.32,
            emergency_distance=0.10,
            rotation_stop_distance=0.12,
            rotate_in_place_angular_threshold=0.05,
            allow_rotation_when_blocked=True,
            ultrasonic_min_valid_m=0.10,
        ),
    )

    assert decision.state == "WARNING"
    assert decision.linear_x == 0.0
    assert decision.angular_z == -0.18


def test_directional_front_ultrasonic_stops_forward_at_30cm():
    decision = evaluate_safety(
        requested_linear_x=0.05,
        requested_angular_z=0.0,
        scan_distance=math.inf,
        ultrasonic_distances=[0.29, 2.0, 2.0, 2.0],
        front_ultrasonic_distances=[0.29, 2.0],
        side_ultrasonic_distances=[2.0, 2.0],
        emergency_hw=False,
        emergency_sw=False,
        params=SafetyParams(
            max_auto_speed=0.05,
            emergency_distance=0.10,
            front_ultrasonic_emergency_distance=0.10,
            front_ultrasonic_stop_distance=0.30,
            side_ultrasonic_stop_distance=0.10,
            ultrasonic_min_valid_m=0.03,
        ),
    )

    assert decision.state == "STOP"
    assert decision.linear_x == 0.0
    assert decision.angular_z == 0.0


def test_directional_side_ultrasonic_uses_10cm_threshold():
    params = SafetyParams(
        max_auto_speed=0.05,
        emergency_distance=0.10,
        side_ultrasonic_emergency_distance=0.05,
        front_ultrasonic_emergency_distance=0.10,
        front_ultrasonic_stop_distance=0.30,
        side_ultrasonic_stop_distance=0.10,
        ultrasonic_min_valid_m=0.03,
    )

    clear = evaluate_safety(
        requested_linear_x=0.05,
        requested_angular_z=0.0,
        scan_distance=math.inf,
        ultrasonic_distances=[2.0, 0.11, 2.0, 2.0],
        front_ultrasonic_distances=[2.0, 2.0],
        side_ultrasonic_distances=[0.11, 2.0],
        emergency_hw=False,
        emergency_sw=False,
        params=params,
    )
    blocked = evaluate_safety(
        requested_linear_x=0.05,
        requested_angular_z=0.0,
        scan_distance=math.inf,
        ultrasonic_distances=[2.0, 0.09, 2.0, 2.0],
        front_ultrasonic_distances=[2.0, 2.0],
        side_ultrasonic_distances=[0.09, 2.0],
        emergency_hw=False,
        emergency_sw=False,
        params=params,
    )

    assert clear.state in ("CLEAR", "WARNING", "SLOWDOWN")
    assert clear.linear_x > 0.0
    assert blocked.state == "STOP"
    assert blocked.linear_x == 0.0


def pytest_approx(value):
    import pytest

    return pytest.approx(value, rel=1e-6)
