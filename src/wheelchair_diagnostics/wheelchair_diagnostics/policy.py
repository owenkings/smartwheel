import json
import time
from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional


@dataclass
class TopicRule:
    name: str
    timeout_sec: float
    critical: bool
    label: str


@dataclass
class TopicState:
    name: str
    label: str
    age_sec: Optional[float]
    online: bool
    critical: bool


@dataclass
class WatchdogDecision:
    state: str
    stop_required: bool
    reason: str
    topics: List[TopicState]

    def to_json(self) -> str:
        return json.dumps(
            {
                "state": self.state,
                "stop_required": self.stop_required,
                "reason": self.reason,
                "topics": [
                    {
                        "name": topic.name,
                        "label": topic.label,
                        "age_sec": topic.age_sec,
                        "online": topic.online,
                        "critical": topic.critical,
                    }
                    for topic in self.topics
                ],
            },
            ensure_ascii=False,
        )


def evaluate_watchdog(
    rules: Iterable[TopicRule],
    last_seen: Dict[str, float],
    now_sec: Optional[float] = None,
    startup_time_sec: Optional[float] = None,
    startup_grace_sec: float = 5.0,
) -> WatchdogDecision:
    now = time.monotonic() if now_sec is None else now_sec
    in_grace = startup_time_sec is not None and now - startup_time_sec < startup_grace_sec
    topics: List[TopicState] = []
    missing_critical: List[str] = []
    missing_warning: List[str] = []

    for rule in rules:
        seen = last_seen.get(rule.name)
        age = None if seen is None else max(0.0, now - seen)
        online = bool(seen is not None and age is not None and age <= rule.timeout_sec)
        if in_grace and seen is None:
            online = True
        state = TopicState(rule.name, rule.label, age, online, rule.critical)
        topics.append(state)
        if not online:
            if rule.critical:
                missing_critical.append(rule.label)
            else:
                missing_warning.append(rule.label)

    if missing_critical:
        return WatchdogDecision(
            "NAV_BLOCKED",
            True,
            "critical topic timeout: " + ", ".join(missing_critical),
            topics,
        )
    if missing_warning:
        return WatchdogDecision(
            "DEGRADED",
            False,
            "non-critical topic timeout: " + ", ".join(missing_warning),
            topics,
        )
    return WatchdogDecision("OK", False, "all monitored topics are fresh", topics)


@dataclass
class LocalizationSample:
    amcl_age_sec: Optional[float]
    odom_age_sec: Optional[float]
    scan_age_sec: Optional[float]
    covariance_x: Optional[float]
    covariance_y: Optional[float]
    covariance_yaw: Optional[float]


@dataclass
class LocalizationDecision:
    state: str
    healthy: bool
    reason: str

    def to_json(self) -> str:
        return json.dumps(
            {
                "state": self.state,
                "healthy": self.healthy,
                "reason": self.reason,
            },
            ensure_ascii=False,
        )


def evaluate_localization_health(
    sample: LocalizationSample,
    max_amcl_age_sec: float,
    max_odom_age_sec: float,
    max_scan_age_sec: float,
    max_covariance_xy: float,
    max_covariance_yaw: float,
) -> LocalizationDecision:
    missing = []
    degraded = []

    if sample.amcl_age_sec is None or sample.amcl_age_sec > max_amcl_age_sec:
        missing.append("AMCL pose stale")
    if sample.odom_age_sec is None or sample.odom_age_sec > max_odom_age_sec:
        missing.append("wheel odom stale")
    if sample.scan_age_sec is None or sample.scan_age_sec > max_scan_age_sec:
        missing.append("scan stale")

    cov_x = sample.covariance_x
    cov_y = sample.covariance_y
    cov_yaw = sample.covariance_yaw
    if cov_x is not None and cov_x > max_covariance_xy:
        degraded.append(f"cov_x {cov_x:.3f} > {max_covariance_xy:.3f}")
    if cov_y is not None and cov_y > max_covariance_xy:
        degraded.append(f"cov_y {cov_y:.3f} > {max_covariance_xy:.3f}")
    if cov_yaw is not None and cov_yaw > max_covariance_yaw:
        degraded.append(f"cov_yaw {cov_yaw:.3f} > {max_covariance_yaw:.3f}")

    if missing:
        return LocalizationDecision("LOST", False, "; ".join(missing))
    if degraded:
        return LocalizationDecision("DEGRADED", False, "; ".join(degraded))
    return LocalizationDecision("GOOD", True, "localization inputs are fresh and covariance is within limits")
