import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from wheelchair_perception.passability_analyzer_node import (  # noqa: E402
    PassabilityConfig,
    analyze_passability,
)


def test_passability_clear_when_corridor_wide_enough():
    config = PassabilityConfig(wheelchair_width_m=0.7, clearance_margin_m=0.15)
    points = [(1.0, 0.8), (1.1, -0.8), (1.5, 0.9), (1.5, -0.9)]

    result = analyze_passability(points, config)

    assert result.state == "CLEAR"
    assert result.estimated_width_m > result.required_width_m


def test_passability_blocked_when_obstacle_inside_required_corridor():
    config = PassabilityConfig(wheelchair_width_m=0.7, clearance_margin_m=0.15)
    points = [(1.0, 0.1), (1.0, 0.8), (1.0, -0.8)]

    result = analyze_passability(points, config)

    assert result.state == "BLOCKED"
