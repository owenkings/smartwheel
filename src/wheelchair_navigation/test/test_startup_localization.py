import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from wheelchair_navigation.startup_localization import (  # noqa: E402
    fixed_target,
    named_goal_target,
    normalize_mode,
    pose_matches,
    validated_covariance,
)


def test_modes_and_fixed_target_require_finite_values():
    assert normalize_mode(" Fixed ") == "fixed"
    with pytest.raises(ValueError, match="unsupported"):
        normalize_mode("guess")
    with pytest.raises(ValueError, match="finite"):
        fixed_target(float("nan"), 0.0, 0.0)


def test_named_goal_target_requires_map_frame_and_coordinates():
    target = named_goal_target(
        "charging",
        {"frame_id": "map", "position": [1.25, -0.5, 0.0], "yaw": 3.5},
    )
    assert target.x == 1.25
    assert target.y == -0.5
    assert target.source == "named_goal:charging"
    assert -math.pi <= target.yaw <= math.pi

    with pytest.raises(ValueError, match="frame_id"):
        named_goal_target("bad", {"frame_id": "odom", "position": [0, 0]})
    with pytest.raises(ValueError, match="x and y"):
        named_goal_target("bad", {"frame_id": "map", "position": [0]})


def test_pose_matching_wraps_yaw_and_checks_distance():
    target = fixed_target(1.0, 2.0, math.pi - 0.02)
    matches, distance, yaw_error = pose_matches(
        target,
        1.1,
        2.0,
        -math.pi + 0.02,
        0.2,
        0.1,
    )
    assert matches is True
    assert distance == pytest.approx(0.1)
    assert yaw_error == pytest.approx(0.04)


def test_zero_covariance_uses_defaults_and_invalid_values_are_rejected():
    assert validated_covariance([0.0] * 36) is None
    covariance = [0.0] * 36
    covariance[0] = 0.1
    assert validated_covariance(covariance)[0] == 0.1
    with pytest.raises(ValueError, match="36"):
        validated_covariance([0.0])
    covariance[7] = float("inf")
    with pytest.raises(ValueError, match="finite"):
        validated_covariance(covariance)
