import math
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from wheelchair_navigation.reactive_explorer_node import (  # noqa: E402
    ReactiveExplorerConfig,
    choose_turn_direction,
    combine_clearances,
    corridor_min_x_distance,
    min_valid,
    reactive_command,
    sector_min,
)


def test_sector_min_filters_to_angle_window():
    ranges = [2.0, 0.8, 0.4, 1.5, math.inf]
    # angles: -40, -20, 0, 20, 40 deg
    assert sector_min(ranges, math.radians(-40), math.radians(20), -25, 25) == 0.4


def test_reactive_command_drives_forward_when_front_clear():
    cfg = ReactiveExplorerConfig(forward_speed=0.05, turn_speed=0.22)
    linear, angular, state, direction = reactive_command(1.2, 0.8, 0.7, cfg)
    assert linear == 0.05
    assert abs(angular) < 1e-6
    assert state.startswith("FORWARD")
    assert direction == 1


def test_reactive_command_turns_to_clearer_side_when_front_blocked():
    cfg = ReactiveExplorerConfig(turn_trigger_distance=0.42, turn_speed=0.25)
    linear, angular, state, direction = reactive_command(0.25, 1.0, 0.2, cfg)
    assert linear == 0.0
    assert angular == 0.25
    assert state.startswith("TURN")
    assert direction == 1


def test_reactive_command_keeps_10cm_hard_floor():
    cfg = ReactiveExplorerConfig(hard_stop_distance=0.10, turn_speed=0.25)
    linear, angular, state, _ = reactive_command(0.09, 0.2, 1.0, cfg)
    assert linear == 0.0
    assert angular == -0.25
    assert state.startswith("TURN_HARD")


def test_reactive_command_turns_away_from_close_side_ultrasonic():
    cfg = ReactiveExplorerConfig(side_trigger_distance=0.10, turn_speed=0.25)
    linear, angular, state, direction = reactive_command(1.2, 0.08, 1.0, cfg)

    assert linear == 0.0
    assert angular == -0.25
    assert state.startswith("TURN_SIDE")
    assert direction == -1


def test_choose_turn_direction_defaults_left_on_tie():
    assert choose_turn_direction(math.inf, math.inf) == 1


def test_corridor_min_catches_wide_body_obstacle():
    # A point at 45 degrees may not be the closest in a narrow front cone, but
    # it is inside the wheelchair swept corridor: x ~= 0.35, y ~= 0.35.
    ranges = [math.inf, 0.5, math.inf]
    assert corridor_min_x_distance(
        ranges,
        math.radians(0),
        math.radians(45),
        half_width_m=0.45,
        lookahead_m=1.2,
    ) == math.sqrt(0.5 * 0.5 / 2.0)


def test_ultrasonic_clearances_override_scan_clearance():
    front, left, right = combine_clearances(
        front=1.2,
        left=1.0,
        right=1.0,
        left_front_ultra=0.48,
        left_side_ultra=0.8,
        right_front_ultra=0.7,
        right_side_ultra=0.09,
    )

    assert front == 0.48
    assert left == 0.48
    assert right == 0.09
    assert min_valid([math.inf, 0.0, 0.02, 0.11], 0.03) == 0.11
