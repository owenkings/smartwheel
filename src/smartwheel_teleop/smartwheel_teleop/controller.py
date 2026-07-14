from dataclasses import dataclass


@dataclass(frozen=True)
class MotionCommand:
    linear: float = 0.0
    angular: float = 0.0


@dataclass(frozen=True)
class SafetyState:
    hardware_enabled: bool
    deadman_active: bool
    emergency_stop: bool
    command_not_timed_out: bool


def motor_command_allowed(state: SafetyState) -> bool:
    return (
        state.hardware_enabled
        and state.deadman_active
        and not state.emergency_stop
        and state.command_not_timed_out
    )


class TeleopController:
    VALID_KEYS = frozenset(("W", "A", "S", "D"))

    def __init__(
        self,
        max_linear: float,
        max_angular: float,
        acceleration: float,
        deceleration: float,
        timeout_sec: float,
    ) -> None:
        if min(max_linear, max_angular, acceleration, deceleration, timeout_sec) <= 0.0:
            raise ValueError("teleop limits and timeout must be positive")
        self.max_linear = max_linear
        self.max_angular = max_angular
        self.acceleration = acceleration
        self.deceleration = deceleration
        self.timeout = timeout_sec
        self._held: set[str] = set()
        self._last_event: float | None = None
        self._last_tick: float | None = None
        self._current = MotionCommand()

    def key_event(self, key: str, pressed: bool, now: float) -> None:
        normalized = key.upper()
        if normalized == "SPACE":
            self._held.clear()
            self._current = MotionCommand()
            self._last_event = now
            return
        if normalized not in self.VALID_KEYS:
            raise ValueError(f"unsupported teleop key: {key}")
        if pressed:
            self._held.add(normalized)
        else:
            self._held.discard(normalized)
        self._last_event = now

    def command_fresh(self, now: float) -> bool:
        return self._last_event is not None and now - self._last_event <= self.timeout

    def tick(self, now: float) -> MotionCommand:
        dt = 0.0 if self._last_tick is None else max(0.0, now - self._last_tick)
        self._last_tick = now
        fresh = self.command_fresh(now)
        linear_axis = (1 if "W" in self._held else 0) - (1 if "S" in self._held else 0)
        angular_axis = (1 if "A" in self._held else 0) - (1 if "D" in self._held else 0)
        target_linear = linear_axis * self.max_linear if fresh else 0.0
        target_angular = angular_axis * self.max_angular if fresh else 0.0
        self._current = MotionCommand(
            self._ramp(self._current.linear, target_linear, dt),
            self._ramp(self._current.angular, target_angular, dt),
        )
        if not fresh and abs(self._current.linear) < 1e-12 and abs(self._current.angular) < 1e-12:
            self._held.clear()
        return self._current

    def _ramp(self, current: float, target: float, dt: float) -> float:
        limit = self.acceleration if abs(target) > abs(current) else self.deceleration
        step = limit * dt
        if current < target:
            return min(current + step, target)
        if current > target:
            return max(current - step, target)
        return target

