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
    # 0.50 m since 2026-11-07 (user decision; was 0.30 m): with the navigation AMCL parameters on the ZL1107
    # corridor, 0.30 m stopped 35% of the mapped stretch while the blank-map stretch stays above 0.50 m
    # (docs/architecture/live_shadow.md).
    covariance_xy_std_max_m: float = 0.50
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

    Odometry drifts slowly, so a localization backend that moves the robot's
    map position by more than the stated translation, or its heading by more
    than the stated angle, within the window is not trusted until the
    correction has stayed within bounds for ``hold_s``. Loop closures therefore
    cause a short stop rather than a blind continuation.

    Defaults since 2026-11-07 (user decision): the translation is measured at
    the robot's current position (``reference='robot'``) and the heading limit
    is 10 degrees. Measured on ``map -> odom`` itself (``'odom_origin'``, the
    earlier default with 5 degrees), a 1 degree heading correction 13 m from the
    odom origin already looked like a 0.23 m jump.
    """

    REFERENCES = ("odom_origin", "robot")

    def __init__(self, window_s=2.0, max_translation_m=0.25, max_yaw_rad=math.radians(10.0), hold_s=3.0,
                 reference="robot"):
        self.window_s = _number(window_s, "window_s", positive=True)
        self.max_translation_m = _number(max_translation_m, "max_translation_m", positive=True)
        self.max_yaw_rad = _number(max_yaw_rad, "max_yaw_rad", positive=True)
        self.hold_s = _number(hold_s, "hold_s", nonnegative=True)
        if reference not in self.REFERENCES:
            raise ValueError("reference must be odom_origin or robot")
        # robot (default): how far the correction change moves the robot's current map position, so a heading
        # correction far from the odom origin no longer counts as a translation jump of (distance x angle).
        # odom_origin: translation of map->odom itself.
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


COVARIANCE_REASONS = ("COVARIANCE_XY_TOO_LARGE", "COVARIANCE_YAW_TOO_LARGE")


class StationaryRelocalization:
    """When to ask AMCL for an in-place filter update while the gate holds the robot for its covariance.

    AMCL updates its filter only after the robot has moved ``update_min_d`` or ``update_min_a``. Once the gate
    stops the robot because the covariance is too large, nothing moves, the filter is never updated, the
    covariance never shrinks and the gate never releases (window B, ZL1107 simulation, 2026-11-07); a start
    pose given with too large an uncertainty never becomes trusted for the same reason. While the gate holds
    for a covariance reason and the robot has stood still for ``settle_s``, this asks for one no-motion update
    (AMCL's ``request_nomotion_update``: the next scan updates the filter without motion) every
    ``interval_s``, at most ``max_requests`` times per stop. The budget renews once the robot has moved
    ``renew_translation_m`` or ``renew_yaw_rad`` (AMCL's own update thresholds) from where the stop began.

    The gate's conditions and limits are unchanged. If the current view cannot bring the covariance within the
    limit, the robot stays stopped: without a budget the same view would be counted again and again and the
    covariance would shrink without new evidence.
    """

    def __init__(self, enabled=True, settle_s=1.0, interval_s=1.0, max_requests=5, still_translation_m=0.01,
                 still_yaw_rad=math.radians(0.5), renew_translation_m=0.10, renew_yaw_rad=0.10):
        if type(enabled) is not bool:
            raise ValueError("enabled must be a boolean")
        self.enabled = enabled
        self.settle_s = _number(settle_s, "settle_s", positive=True)
        self.interval_s = _number(interval_s, "interval_s", positive=True)
        if isinstance(max_requests, bool) or not isinstance(max_requests, int) or not 0 <= max_requests <= 20:
            raise ValueError("max_requests must be an integer within 0..20")
        self.max_requests = max_requests
        self.still_translation_m = _number(still_translation_m, "still_translation_m", positive=True)
        self.still_yaw_rad = _number(still_yaw_rad, "still_yaw_rad", positive=True)
        self.renew_translation_m = _number(renew_translation_m, "renew_translation_m", positive=True)
        self.renew_yaw_rad = _number(renew_yaw_rad, "renew_yaw_rad", positive=True)
        self._poses = []
        self._anchor = None
        self._last_request = -math.inf
        self.requests_this_stop = 0
        self.requests_total = 0
        self.stops = 0
        self.exhausted_stops = 0
        self.service_unavailable = 0

    def _moved(self, a, b, translation, yaw):
        return (math.hypot(a[0] - b[0], a[1] - b[1]) >= translation
                or abs(math.atan2(math.sin(a[2] - b[2]), math.cos(a[2] - b[2]))) >= yaw)

    def update(self, now, holding_for_covariance, robot_in_odom, service_available=True):
        """Feed one gate tick; True when one no-motion update should be requested now.

        ``robot_in_odom`` is the robot's (x, y, yaw) in odom, or None when it is unknown (no request then).
        """
        if not self.enabled or robot_in_odom is None:
            return False
        now = float(now)
        pose = tuple(float(v) for v in robot_in_odom[:3])
        if self._anchor is not None and self._moved(pose, self._anchor, self.renew_translation_m, self.renew_yaw_rad):
            self._anchor, self.requests_this_stop = None, 0
        self._poses.append((now, pose))
        while len(self._poses) > 1 and now - self._poses[1][0] >= self.settle_s:
            self._poses.pop(0)
        settled = (now - self._poses[0][0] >= self.settle_s and not any(
            self._moved(pose, old, self.still_translation_m, self.still_yaw_rad) for _, old in self._poses))
        if not (holding_for_covariance and settled):
            return False
        if self._anchor is None:
            self._anchor = pose
            self.stops += 1
        if self.requests_this_stop >= self.max_requests or now - self._last_request < self.interval_s:
            return False
        if not service_available:
            self.service_unavailable += 1
            self._last_request = now
            return False
        self.requests_this_stop += 1
        self.requests_total += 1
        self._last_request = now
        if self.requests_this_stop == self.max_requests:
            self.exhausted_stops += 1
        return True

    def status(self):
        return {"enabled": self.enabled, "requests_total": self.requests_total,
                "requests_this_stop": self.requests_this_stop, "max_requests_per_stop": self.max_requests,
                "budget_exhausted": self.enabled and self.requests_this_stop >= self.max_requests,
                "stops": self.stops, "exhausted_stops": self.exhausted_stops,
                "service_unavailable_ticks": self.service_unavailable}


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
