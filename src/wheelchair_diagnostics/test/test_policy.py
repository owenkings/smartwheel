import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from wheelchair_diagnostics.policy import (  # noqa: E402
    LocalizationSample,
    TopicRule,
    evaluate_localization_health,
    evaluate_watchdog,
)


def test_watchdog_blocks_navigation_when_critical_topic_is_stale():
    rules = [
        TopicRule("/scan", 1.0, True, "scan"),
        TopicRule("/camera/front/image_raw", 1.0, False, "front camera"),
    ]

    decision = evaluate_watchdog(rules, {"/camera/front/image_raw": 10.0}, now_sec=12.0)

    assert decision.state == "NAV_BLOCKED"
    assert decision.stop_required is True
    assert "scan" in decision.reason


def test_watchdog_degrades_for_noncritical_topic_only():
    rules = [
        TopicRule("/scan", 1.0, True, "scan"),
        TopicRule("/camera/front/image_raw", 1.0, False, "front camera"),
    ]

    decision = evaluate_watchdog(rules, {"/scan": 12.0}, now_sec=12.0)

    assert decision.state == "DEGRADED"
    assert decision.stop_required is False


def test_watchdog_blocks_when_critical_ultrasonic_topic_is_stale():
    rules = [
        TopicRule("/scan", 1.0, True, "scan"),
        TopicRule("/ultrasonic/0/range", 1.0, True, "ultrasonic 0"),
    ]

    decision = evaluate_watchdog(rules, {"/scan": 12.0}, now_sec=12.0)

    assert decision.state == "NAV_BLOCKED"
    assert decision.stop_required is True
    assert "ultrasonic 0" in decision.reason


def test_localization_health_marks_high_covariance_degraded():
    sample = LocalizationSample(
        amcl_age_sec=0.1,
        odom_age_sec=0.1,
        scan_age_sec=0.1,
        covariance_x=0.8,
        covariance_y=0.1,
        covariance_yaw=0.1,
    )

    decision = evaluate_localization_health(sample, 2.0, 1.0, 1.0, 0.5, 0.35)

    assert decision.state == "DEGRADED"
    assert decision.healthy is False
