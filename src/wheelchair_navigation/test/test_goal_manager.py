import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from wheelchair_navigation.goal_manager_node import (  # noqa: E402
    is_voice_intent_confident,
    nearest_goal_label,
)


def test_voice_intent_confidence_threshold_accepts_high_confidence():
    assert is_voice_intent_confident({"confidence": 0.93}, 0.75) is True


def test_voice_intent_confidence_threshold_rejects_low_or_missing_confidence():
    assert is_voice_intent_confident({"confidence": 0.40}, 0.75) is False
    assert is_voice_intent_confident({}, 0.75) is False


def test_nearest_goal_label_respects_distance_limit():
    goals = {
        "charging": {"label": "充电点", "position": [0.0, 0.0, 0.0]},
        "door": {"label": "门口", "position": [5.0, 0.0, 0.0]},
    }

    assert nearest_goal_label(goals, 0.4, 0.2, 2.0) == "充电点"
    assert nearest_goal_label(goals, 10.0, 10.0, 2.0) is None
