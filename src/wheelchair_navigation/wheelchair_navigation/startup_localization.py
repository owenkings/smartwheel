import math
from dataclasses import dataclass
from typing import Mapping, Optional, Sequence, Tuple


VALID_STARTUP_LOCALIZATION_MODES = {
    "disabled",
    "named_goal",
    "fixed",
    "external_anchor",
}


@dataclass(frozen=True)
class StartupPoseTarget:
    x: float
    y: float
    yaw: float
    source: str
    frame_id: str = "map"
    covariance: Optional[Tuple[float, ...]] = None


def normalize_mode(value: str) -> str:
    mode = str(value).strip().lower()
    if mode not in VALID_STARTUP_LOCALIZATION_MODES:
        allowed = ", ".join(sorted(VALID_STARTUP_LOCALIZATION_MODES))
        raise ValueError(f"unsupported startup localization mode {value!r}; expected {allowed}")
    return mode


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(float(angle)), math.cos(float(angle)))


def _finite_float(value, field_name: str) -> float:
    number = float(value)
    if not math.isfinite(number):
        raise ValueError(f"{field_name} must be finite")
    return number


def fixed_target(x, y, yaw, source: str = "fixed") -> StartupPoseTarget:
    return StartupPoseTarget(
        x=_finite_float(x, "x"),
        y=_finite_float(y, "y"),
        yaw=normalize_angle(_finite_float(yaw, "yaw")),
        source=str(source),
    )


def named_goal_target(name: str, goal: Mapping) -> StartupPoseTarget:
    if not isinstance(goal, Mapping):
        raise ValueError(f"named goal {name!r} is not a mapping")
    frame_id = str(goal.get("frame_id", "map")).strip()
    if frame_id != "map":
        raise ValueError(f"named goal {name!r} must use frame_id 'map'")
    position = goal.get("position")
    if not isinstance(position, Sequence) or isinstance(position, (str, bytes)):
        raise ValueError(f"named goal {name!r} position must be a sequence")
    if len(position) < 2:
        raise ValueError(f"named goal {name!r} position must contain x and y")
    target = fixed_target(
        position[0],
        position[1],
        goal.get("yaw", 0.0),
        source=f"named_goal:{name}",
    )
    return StartupPoseTarget(
        x=target.x,
        y=target.y,
        yaw=target.yaw,
        source=target.source,
        frame_id=frame_id,
    )


def validated_covariance(values: Sequence[float]) -> Optional[Tuple[float, ...]]:
    if len(values) != 36:
        raise ValueError("pose covariance must contain 36 values")
    covariance = tuple(_finite_float(value, "covariance") for value in values)
    return covariance if any(value != 0.0 for value in covariance) else None


def pose_error(target: StartupPoseTarget, x, y, yaw) -> tuple[float, float]:
    actual_x = _finite_float(x, "pose x")
    actual_y = _finite_float(y, "pose y")
    actual_yaw = _finite_float(yaw, "pose yaw")
    distance = math.hypot(actual_x - target.x, actual_y - target.y)
    yaw_error = abs(normalize_angle(actual_yaw - target.yaw))
    return distance, yaw_error


def pose_matches(
    target: StartupPoseTarget,
    x,
    y,
    yaw,
    distance_tolerance_m: float,
    yaw_tolerance_rad: float,
) -> tuple[bool, float, float]:
    distance, yaw_error = pose_error(target, x, y, yaw)
    matches = (
        distance <= max(0.0, float(distance_tolerance_m))
        and yaw_error <= max(0.0, float(yaw_tolerance_rad))
    )
    return matches, distance, yaw_error
