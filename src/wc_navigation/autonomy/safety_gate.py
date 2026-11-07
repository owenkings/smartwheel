"""Last software gate between navigation velocity commands and the base.

The gate passes a command only while every condition holds: no latched
emergency stop, a fresh operator enable, trusted localization and a fresh
command. Otherwise it outputs an explicit zero twist. It also clamps speeds and
blocks reverse motion because the wheelchair has no rear-facing range sensor.
The gate is a software layer; it does not replace a validated physical stop.
"""
from __future__ import annotations

from dataclasses import dataclass
import math

from ..geometry import _number

STATES = ("ESTOP_LATCHED", "DISABLED", "ENABLE_STALE", "LOCALIZATION_UNTRUSTED",
          "COMMAND_STALE", "ACTIVE")


@dataclass(frozen=True)
class GateConfig:
    max_linear: float = 0.30
    max_angular: float = 0.50
    allow_reverse: bool = False
    command_timeout_s: float = 0.5
    enable_timeout_s: float = 1.0
    localization_max_age_s: float = 0.5
    require_covariance: bool = False
    covariance_xy_std_max_m: float = 0.30
    covariance_yaw_std_max_rad: float = 0.25

    def __post_init__(self):
        for name in ("max_linear", "max_angular", "command_timeout_s", "enable_timeout_s",
                     "localization_max_age_s", "covariance_xy_std_max_m", "covariance_yaw_std_max_rad"):
            object.__setattr__(self, name, _number(getattr(self, name), name, positive=True))
        for name in ("allow_reverse", "require_covariance"):
            if type(getattr(self, name)) is not bool:
                raise ValueError(name + " must be a boolean")

    def as_dict(self):
        return {name: getattr(self, name) for name in self.__dataclass_fields__}


class CorrectionMonitor:
    """Flag implausibly fast changes of the global correction ``map -> odom``.

    Odometry drifts slowly, so a localization backend that moves ``map -> odom``
    by more than the stated translation or heading within the window is not
    trusted until the correction has stayed within bounds for ``hold_s``.
    Loop closures therefore cause a short stop rather than a blind continuation.
    """

    REFERENCES = ("odom_origin", "robot")

    def __init__(self, window_s=2.0, max_translation_m=0.25, max_yaw_rad=math.radians(5.0), hold_s=3.0,
                 reference="odom_origin"):
        self.window_s = _number(window_s, "window_s", positive=True)
        self.max_translation_m = _number(max_translation_m, "max_translation_m", positive=True)
        self.max_yaw_rad = _number(max_yaw_rad, "max_yaw_rad", positive=True)
        self.hold_s = _number(hold_s, "hold_s", nonnegative=True)
        if reference not in self.REFERENCES:
            raise ValueError("reference must be odom_origin or robot")
        # odom_origin (default): translation of map->odom itself. robot: how far the correction change moves
        # the robot's current map position; a heading correction far from the odom origin then no longer
        # counts as a translation jump of (distance x angle).
        self.reference = reference
        self._history = []
        self._untrusted_until = -math.inf
        self.events = 0

    def update(self, now, correction, robot_in_odom=None):
        """Add ``(x, y, yaw)`` of map->odom at ``now``; return (ok, reason).

        With ``reference='robot'`` the robot's current odom pose ``(x, y[, yaw])`` is required.
        """
        x, y, yaw = (float(v) for v in correction)
        px = py = 0.0
        if self.reference == "robot":
            if robot_in_odom is None:
                raise ValueError("the robot reference needs the robot's odom position")
            px, py = float(robot_in_odom[0]), float(robot_in_odom[1])
        self._history.append((float(now), x, y, yaw))
        while self._history and now - self._history[0][0] > self.window_s:
            self._history.pop(0)
        c, s = math.cos(yaw), math.sin(yaw)
        mx, my = x + c * px - s * py, y + s * px + c * py
        jumped = False
        for t, hx, hy, hyaw in self._history[:-1]:
            dyaw = abs(math.atan2(math.sin(yaw - hyaw), math.cos(yaw - hyaw)))
            hc, hs = math.cos(hyaw), math.sin(hyaw)
            moved = math.hypot(mx - (hx + hc * px - hs * py), my - (hy + hs * px + hc * py))
            if moved > self.max_translation_m or dyaw > self.max_yaw_rad:
                jumped = True
                break
        if jumped:
            if now >= self._untrusted_until:
                self.events += 1
            self._untrusted_until = now + self.hold_s
        if now < self._untrusted_until:
            return False, "MAP_CORRECTION_CHANGING_FAST"
        return True, ""


@dataclass(frozen=True)
class GateDecision:
    linear: float
    angular: float
    state: str
    reasons: tuple


class SafetyGate:
    """Pure decision logic; the ROS node only feeds timestamps and inputs."""

    def __init__(self, config=None):
        self.config = config or GateConfig()
        self._command = None
        self._command_time = -math.inf
        self._enabled = False
        self._enable_time = -math.inf
        self._estop_engaged = False
        self._latched = False
        self._localization_ok = False
        self._localization_reason = "NO_LOCALIZATION_INPUT"

    def on_command(self, linear, angular, now):
        self._command = (linear, angular)
        self._command_time = float(now)

    def on_enable(self, enabled, now):
        enabled = bool(enabled)
        rising = enabled and (not self._enabled or now - self._enable_time > self.config.enable_timeout_s)
        if rising and self._latched and not self._estop_engaged:
            # A latched stop clears only after release followed by a fresh enable.
            self._latched = False
        self._enabled = enabled
        self._enable_time = float(now)

    def on_estop(self, engaged, now):
        self._estop_engaged = bool(engaged)
        if self._estop_engaged:
            self._latched = True

    def on_localization(self, ok, reason=""):
        self._localization_ok = bool(ok)
        self._localization_reason = str(reason) if not ok else ""

    def check_covariance(self, covariance):
        """Necessary-only covariance condition; small values never prove trust."""
        cfg = self.config
        try:
            xx, yy, tt = float(covariance[0]), float(covariance[7]), float(covariance[35])
        except (TypeError, IndexError, ValueError):
            return False, "COVARIANCE_INVALID"
        if not all(math.isfinite(v) and v >= 0 for v in (xx, yy, tt)):
            return False, "COVARIANCE_INVALID"
        if math.sqrt(max(xx, yy)) > cfg.covariance_xy_std_max_m:
            return False, "COVARIANCE_XY_TOO_LARGE"
        if math.sqrt(tt) > cfg.covariance_yaw_std_max_rad:
            return False, "COVARIANCE_YAW_TOO_LARGE"
        return True, ""

    def evaluate(self, now):
        cfg = self.config
        if self._latched:
            return GateDecision(0.0, 0.0, "ESTOP_LATCHED", ("ESTOP_ENGAGED" if self._estop_engaged else "ESTOP_RESET_REQUIRED",))
        if not self._enabled:
            return GateDecision(0.0, 0.0, "DISABLED", ("OPERATOR_ENABLE_OFF",))
        if now - self._enable_time > cfg.enable_timeout_s:
            return GateDecision(0.0, 0.0, "ENABLE_STALE", ("ENABLE_HEARTBEAT_TIMEOUT",))
        if not self._localization_ok:
            return GateDecision(0.0, 0.0, "LOCALIZATION_UNTRUSTED", (self._localization_reason or "LOCALIZATION_FAILED",))
        if self._command is None or now - self._command_time > cfg.command_timeout_s:
            return GateDecision(0.0, 0.0, "COMMAND_STALE", ("NO_FRESH_COMMAND",))
        linear, angular = self._command
        if any(isinstance(v, bool) or not isinstance(v, (int, float)) for v in (linear, angular)):
            return GateDecision(0.0, 0.0, "ACTIVE", ("INVALID_COMMAND_ZEROED",))
        linear, angular = float(linear), float(angular)
        if not (math.isfinite(linear) and math.isfinite(angular)):
            return GateDecision(0.0, 0.0, "ACTIVE", ("NONFINITE_COMMAND_ZEROED",))
        reasons = []
        if linear < 0 and not cfg.allow_reverse:
            linear = 0.0
            reasons.append("REVERSE_BLOCKED")
        if abs(linear) > cfg.max_linear:
            linear = math.copysign(cfg.max_linear, linear)
            reasons.append("LINEAR_CLAMPED")
        if abs(angular) > cfg.max_angular:
            angular = math.copysign(cfg.max_angular, angular)
            reasons.append("ANGULAR_CLAMPED")
        return GateDecision(linear, angular, "ACTIVE", tuple(reasons))
