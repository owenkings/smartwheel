"""Pure-math LIO versus wheel/IMU relative-motion consistency monitor.

This module deliberately has no ROS imports. It compares two separately computed
*increments* (sharing the H30 input) over a short common time window. Absolute world-frame
origins are never compared.

Pose convention is ``T_parent_child``: a pose maps child-frame coordinates
into its parent.  FAST-LIO's body pose is treated as ``T_WI`` (IMU in its
arbitrary world).  The configured static layout is ``T_BI`` (IMU in base), so
the comparable base pose is ``T_WB = T_WI * inverse(T_BI)``.
"""

from __future__ import annotations

from bisect import bisect_left
from collections import deque
from dataclasses import dataclass, field
import math
from typing import Deque, Dict, Iterable, Optional, Sequence, Tuple


Vector3 = Tuple[float, float, float]
Quaternion = Tuple[float, float, float, float]  # ROS x, y, z, w


def _finite(values: Iterable[float]) -> bool:
    return all(math.isfinite(float(value)) for value in values)


def _v_add(a: Vector3, b: Vector3) -> Vector3:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _v_sub(a: Vector3, b: Vector3) -> Vector3:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _v_scale(a: Vector3, scale: float) -> Vector3:
    return (a[0] * scale, a[1] * scale, a[2] * scale)


def _v_norm(a: Vector3) -> float:
    return math.sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2])


def normalize_quaternion(q: Sequence[float]) -> Quaternion:
    if len(q) != 4 or not _finite(q):
        raise ValueError("quaternion must contain four finite values")
    norm = math.sqrt(sum(float(value) ** 2 for value in q))
    if norm < 1.0e-12:
        raise ValueError("quaternion norm is zero")
    return tuple(float(value) / norm for value in q)  # type: ignore[return-value]


def quaternion_multiply(a: Quaternion, b: Quaternion) -> Quaternion:
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return normalize_quaternion((
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    ))


def quaternion_inverse(q: Quaternion) -> Quaternion:
    x, y, z, w = normalize_quaternion(q)
    return (-x, -y, -z, w)


def rotate_vector(q: Quaternion, vector: Vector3) -> Vector3:
    """Rotate ``vector`` by quaternion ``q`` without constructing a matrix."""
    x, y, z, w = normalize_quaternion(q)
    vx, vy, vz = vector
    # q * [v, 0] * q^-1, expanded.
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (
        vx + w * tx + (y * tz - z * ty),
        vy + w * ty + (z * tx - x * tz),
        vz + w * tz + (x * ty - y * tx),
    )


def quaternion_slerp(a: Quaternion, b: Quaternion, fraction: float) -> Quaternion:
    a = normalize_quaternion(a)
    b = normalize_quaternion(b)
    dot = sum(x * y for x, y in zip(a, b))
    if dot < 0.0:
        b = tuple(-value for value in b)  # type: ignore[assignment]
        dot = -dot
    dot = min(1.0, max(-1.0, dot))
    if dot > 0.9995:
        return normalize_quaternion(tuple(
            x + fraction * (y - x) for x, y in zip(a, b)
        ))
    theta = math.acos(dot)
    sin_theta = math.sin(theta)
    wa = math.sin((1.0 - fraction) * theta) / sin_theta
    wb = math.sin(fraction * theta) / sin_theta
    return normalize_quaternion(tuple(
        wa * x + wb * y for x, y in zip(a, b)
    ))


def quaternion_angle(q: Quaternion) -> float:
    x, y, z, w = normalize_quaternion(q)
    # q and -q are the same rotation.  The result is in [0, pi].
    return 2.0 * math.atan2(math.sqrt(x * x + y * y + z * z), abs(w))


@dataclass(frozen=True)
class Pose:
    translation: Vector3
    rotation: Quaternion

    def __post_init__(self) -> None:
        if len(self.translation) != 3 or not _finite(self.translation):
            raise ValueError("translation must contain three finite values")
        object.__setattr__(self, "translation", tuple(
            float(value) for value in self.translation
        ))
        object.__setattr__(self, "rotation", normalize_quaternion(self.rotation))


IDENTITY_POSE = Pose((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))


def compose_pose(a: Pose, b: Pose) -> Pose:
    """Return ``a * b`` using the T_parent_child pose convention."""
    return Pose(
        _v_add(a.translation, rotate_vector(a.rotation, b.translation)),
        quaternion_multiply(a.rotation, b.rotation),
    )


def inverse_pose(pose: Pose) -> Pose:
    rotation = quaternion_inverse(pose.rotation)
    return Pose(rotate_vector(rotation, _v_scale(pose.translation, -1.0)), rotation)


def relative_pose(start: Pose, end: Pose) -> Pose:
    return compose_pose(inverse_pose(start), end)


def lio_imu_pose_to_base_pose(world_to_imu: Pose, base_to_imu: Pose) -> Pose:
    """Convert ``T_WI`` to ``T_WB`` as ``T_WI * inverse(T_BI)``."""
    return compose_pose(world_to_imu, inverse_pose(base_to_imu))


@dataclass(frozen=True)
class TimedPose:
    stamp: float
    pose: Pose

    def __post_init__(self) -> None:
        if not math.isfinite(self.stamp) or self.stamp <= 0.0:
            raise ValueError("pose timestamp must be finite and positive")


@dataclass(frozen=True)
class MonitorConfig:
    window_sec: float = 1.0
    source_timeout_sec: float = 1.0
    max_alignment_age_sec: float = 0.20
    max_interpolation_gap_sec: float = 0.20
    max_translation_error_m: float = 0.10
    max_rotation_error_rad: float = 0.15
    retention_sec: float = 4.0

    def __post_init__(self) -> None:
        positive = {
            "window_sec": self.window_sec,
            "source_timeout_sec": self.source_timeout_sec,
            "max_alignment_age_sec": self.max_alignment_age_sec,
            "max_interpolation_gap_sec": self.max_interpolation_gap_sec,
            "max_translation_error_m": self.max_translation_error_m,
            "max_rotation_error_rad": self.max_rotation_error_rad,
            "retention_sec": self.retention_sec,
        }
        for name, value in positive.items():
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and positive")
        if self.retention_sec < self.window_sec + self.max_interpolation_gap_sec:
            raise ValueError("retention_sec is too short for the requested window")


@dataclass(frozen=True)
class ConsistencyResult:
    state: str
    consistent: bool
    reasons: Tuple[str, ...]
    values: Dict[str, object] = field(default_factory=dict)


class MotionConsistencyMonitor:
    """Maintain bounded pose histories and compare synchronized increments."""

    def __init__(
        self,
        base_to_imu: Pose,
        config: MonitorConfig = MonitorConfig(),
    ) -> None:
        self.base_to_imu = base_to_imu
        self.config = config
        self._lio: Deque[TimedPose] = deque()
        self._reference: Deque[TimedPose] = deque()
        self._last_receipt = {"lio": None, "reference": None}
        self._started = {"lio": False, "reference": False}
        self._input_error = {"lio": None, "reference": None}

    def _add(
        self,
        source: str,
        sample: TimedPose,
        received_at: float,
    ) -> bool:
        if not math.isfinite(received_at) or received_at < 0.0:
            self._input_error[source] = f"{source}_invalid_receive_time"
            return False
        history = self._lio if source == "lio" else self._reference
        self._started[source] = True
        self._last_receipt[source] = received_at
        if history and sample.stamp < history[-1].stamp - 1.0e-9:
            history.clear()
            self._input_error[source] = f"{source}_stamp_regression"
        elif history and abs(sample.stamp - history[-1].stamp) <= 1.0e-9:
            history[-1] = sample
            self._input_error[source] = None
            return True
        else:
            self._input_error[source] = None
        history.append(sample)
        cutoff = sample.stamp - self.config.retention_sec
        while history and history[0].stamp < cutoff:
            history.popleft()
        return True

    def add_lio_imu_pose(self, sample: TimedPose, received_at: float) -> bool:
        try:
            converted = TimedPose(
                sample.stamp,
                lio_imu_pose_to_base_pose(sample.pose, self.base_to_imu),
            )
        except (TypeError, ValueError):
            self._started["lio"] = True
            self._last_receipt["lio"] = received_at
            self._input_error["lio"] = "lio_nonfinite_or_invalid_pose"
            return False
        return self._add("lio", converted, received_at)

    def add_reference_base_pose(self, sample: TimedPose, received_at: float) -> bool:
        try:
            checked = TimedPose(sample.stamp, sample.pose)
        except (TypeError, ValueError):
            self._started["reference"] = True
            self._last_receipt["reference"] = received_at
            self._input_error["reference"] = "reference_nonfinite_or_invalid_pose"
            return False
        return self._add("reference", checked, received_at)

    def _interpolate(
        self,
        history: Deque[TimedPose],
        stamp: float,
    ) -> Tuple[Optional[Pose], Optional[str]]:
        if not history or stamp < history[0].stamp or stamp > history[-1].stamp:
            return None, "history_does_not_bracket_target"
        stamps = [sample.stamp for sample in history]
        index = bisect_left(stamps, stamp)
        if index < len(history) and abs(history[index].stamp - stamp) <= 1.0e-9:
            return history[index].pose, None
        if index == 0 or index >= len(history):
            return None, "history_does_not_bracket_target"
        before = history[index - 1]
        after = history[index]
        gap = after.stamp - before.stamp
        if gap <= 0.0 or gap > self.config.max_interpolation_gap_sec:
            return None, "interpolation_gap_exceeded"
        fraction = (stamp - before.stamp) / gap
        return Pose(
            _v_add(
                before.pose.translation,
                _v_scale(_v_sub(after.pose.translation, before.pose.translation), fraction),
            ),
            quaternion_slerp(before.pose.rotation, after.pose.rotation, fraction),
        ), None

    def _result(self, state: str, reasons: Sequence[str], **values: object) -> ConsistencyResult:
        return ConsistencyResult(state, state == "OK", tuple(reasons), values)

    def evaluate(self, now_sec: float) -> ConsistencyResult:
        if not math.isfinite(now_sec) or now_sec < 0.0:
            return self._result("INVALID", ["invalid_monitor_time"])
        not_started = [
            f"{source}_not_started"
            for source in ("lio", "reference")
            if not self._started[source]
        ]
        if not_started:
            return self._result("UNAVAILABLE", not_started)

        input_errors = [
            error for error in self._input_error.values() if error is not None
        ]
        if input_errors:
            return self._result("INVALID", input_errors)

        stale = []
        ages: Dict[str, float] = {}
        for source in ("lio", "reference"):
            receipt = self._last_receipt[source]
            assert receipt is not None
            age = now_sec - receipt
            ages[f"{source}_receipt_age_sec"] = age
            if age < -1.0e-6:
                stale.append(f"{source}_monitor_clock_regression")
            elif age > self.config.source_timeout_sec:
                stale.append(f"{source}_timeout")
            history = self._lio if source == "lio" else self._reference
            if history:
                stamp_age = now_sec - history[-1].stamp
                ages[f"{source}_stamp_age_sec"] = stamp_age
                # Fresh delivery of a repeated old message is not fresh motion.
                if stamp_age > self.config.source_timeout_sec:
                    stale.append(f"{source}_timestamp_stale")
                elif stamp_age < -0.05:
                    stale.append(f"{source}_timestamp_in_future")
        if stale:
            return self._result("STALE", stale, **ages)

        if len(self._lio) < 2 or len(self._reference) < 2:
            return self._result("WARMUP", ["insufficient_pose_history"], **ages)

        # Compare at a real LIO stamp that the reference history brackets.  If
        # the newest LIO callback arrived just ahead of reference odometry, use
        # the newest bracketed LIO stamp, but only within a bounded age.
        reference_latest_stamp = self._reference[-1].stamp
        end_sample = next(
            (sample for sample in reversed(self._lio)
             if sample.stamp <= reference_latest_stamp + 1.0e-9),
            None,
        )
        if end_sample is None:
            return self._result("WARMUP", ["reference_has_not_bracketed_lio"], **ages)
        end_stamp = end_sample.stamp
        lio_alignment_age = self._lio[-1].stamp - end_stamp
        reference_alignment_age = self._reference[-1].stamp - end_stamp
        ages.update({
            "lio_alignment_age_sec": lio_alignment_age,
            "reference_alignment_age_sec": reference_alignment_age,
            "comparison_end_stamp": end_stamp,
        })
        alignment_reasons = []
        if lio_alignment_age > self.config.max_alignment_age_sec:
            alignment_reasons.append("lio_alignment_age_exceeded")
        if reference_alignment_age > self.config.max_alignment_age_sec:
            alignment_reasons.append("reference_alignment_age_exceeded")
        if alignment_reasons:
            return self._result("STALE", alignment_reasons, **ages)

        start_stamp = end_stamp - self.config.window_sec
        poses: Dict[str, Pose] = {"lio_end": end_sample.pose}
        interpolation_errors = []
        for label, history, stamp in (
            ("lio_start", self._lio, start_stamp),
            ("reference_start", self._reference, start_stamp),
            ("reference_end", self._reference, end_stamp),
        ):
            pose, error = self._interpolate(history, stamp)
            if error is not None:
                interpolation_errors.append(f"{label}_{error}")
            else:
                assert pose is not None
                poses[label] = pose
        if interpolation_errors:
            warmup_only = all("history_does_not_bracket_target" in reason
                              for reason in interpolation_errors)
            state = "WARMUP" if warmup_only else "INVALID"
            return self._result(state, interpolation_errors, **ages)

        lio_delta = relative_pose(poses["lio_start"], poses["lio_end"])
        reference_delta = relative_pose(
            poses["reference_start"], poses["reference_end"]
        )
        translation_error_vector = _v_sub(
            lio_delta.translation, reference_delta.translation
        )
        translation_error = _v_norm(translation_error_vector)
        rotation_error = quaternion_angle(quaternion_multiply(
            quaternion_inverse(reference_delta.rotation), lio_delta.rotation
        ))
        reasons = []
        if translation_error > self.config.max_translation_error_m:
            reasons.append("translation_increment_mismatch")
        if rotation_error > self.config.max_rotation_error_rad:
            reasons.append("rotation_increment_mismatch")

        values: Dict[str, object] = dict(ages)
        values.update({
            "window_sec": self.config.window_sec,
            "translation_error_m": translation_error,
            "rotation_error_rad": rotation_error,
            "rotation_error_deg": math.degrees(rotation_error),
            "translation_error_vector_m": translation_error_vector,
            "lio_translation_increment_m": lio_delta.translation,
            "reference_translation_increment_m": reference_delta.translation,
            "lio_rotation_increment_rad": quaternion_angle(lio_delta.rotation),
            "reference_rotation_increment_rad": quaternion_angle(reference_delta.rotation),
            "max_translation_error_m": self.config.max_translation_error_m,
            "max_rotation_error_rad": self.config.max_rotation_error_rad,
            "thresholds_provisional": True,
        })
        if reasons:
            return self._result("DIVERGENCE", reasons, **values)
        return self._result("OK", ["relative_motion_within_provisional_limits"], **values)
