import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from wheelchair_navigation.goal_manager_node import is_voice_intent_confident  # noqa: E402


def test_voice_intent_confidence_threshold_accepts_high_confidence():
    assert is_voice_intent_confident({"confidence": 0.93}, 0.75) is True


def test_voice_intent_confidence_threshold_rejects_low_or_missing_confidence():
    assert is_voice_intent_confident({"confidence": 0.40}, 0.75) is False
    assert is_voice_intent_confident({}, 0.75) is False
