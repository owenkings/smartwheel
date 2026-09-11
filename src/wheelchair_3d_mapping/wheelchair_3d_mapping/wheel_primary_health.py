"""Pure health policy for the wheel-primary pose/TF gate."""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class HealthLimits:
    feedback_receive_age: float = 0.25
    wheel_receive_age: float = 0.20
    imu_receive_age: float = 0.10
    wheel_header_age: float = 0.20
    imu_header_age: float = 0.10
    ekf_header_age: float = 0.20
    future_tolerance: float = 0.05


@dataclass(frozen=True)
class HealthSample:
    feedback_ok: bool
    feedback_receive_age: float
    wheel_receive_age: float
    imu_receive_age: float
    wheel_header_age: float
    imu_header_age: float
    ekf_header_age: float
    wheel_valid: bool
    imu_valid: bool
    ekf_valid: bool


def evaluate_health(sample: HealthSample, limits: HealthLimits) -> str:
    """Return ``OK`` or the first fail-closed reason for one candidate pose."""

    values = tuple(sample.__dict__[name] for name in (
        "feedback_receive_age", "wheel_receive_age", "imu_receive_age",
        "wheel_header_age", "imu_header_age", "ekf_header_age",
    ))
    limit_values = tuple(limits.__dict__.values())
    if not all(math.isfinite(value) for value in values + limit_values):
        return "BLOCKED_MISSING_OR_NONFINITE_TIME"
    if min(limit_values) < 0.0:
        return "BLOCKED_INVALID_LIMIT"
    if not sample.feedback_ok:
        return "BLOCKED_FEEDBACK_UNHEALTHY"
    if sample.feedback_receive_age < 0.0 or sample.feedback_receive_age > limits.feedback_receive_age:
        return "BLOCKED_FEEDBACK_STALE"
    if sample.wheel_receive_age < 0.0 or sample.wheel_receive_age > limits.wheel_receive_age:
        return "BLOCKED_WHEEL_RECEIVE_STALE"
    if sample.imu_receive_age < 0.0 or sample.imu_receive_age > limits.imu_receive_age:
        return "BLOCKED_IMU_RECEIVE_STALE"
    if not sample.wheel_valid:
        return "BLOCKED_WHEEL_INVALID"
    if not sample.imu_valid:
        return "BLOCKED_IMU_INVALID"
    if not sample.ekf_valid:
        return "BLOCKED_EKF_INVALID"
    for label, age, maximum in (
        ("WHEEL", sample.wheel_header_age, limits.wheel_header_age),
        ("IMU", sample.imu_header_age, limits.imu_header_age),
        ("EKF", sample.ekf_header_age, limits.ekf_header_age),
    ):
        if age < -limits.future_tolerance:
            return f"BLOCKED_{label}_FROM_FUTURE"
        if age > maximum:
            return f"BLOCKED_{label}_HEADER_STALE"
    return "OK"

