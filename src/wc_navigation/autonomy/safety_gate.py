"""Last software gate between navigation velocity commands and the base.

The gate passes a command only while every condition holds: no latched
emergency stop, a fresh operator enable, trusted localization and a fresh
command; in the vehicle closed loop also a base that runs automatic control and
a single enable source. Otherwise it outputs an explicit zero twist. It also
clamps speeds and blocks reverse motion because the wheelchair has no
rear-facing range sensor. The gate is a software layer; it does not replace a
validated physical stop.
"""
from __future__ import annotations

from dataclasses import dataclass
import math

from ..geometry import _number

STATES = ("ESTOP_LATCHED", "DISABLED", "ENABLE_STALE", "BASE_NOT_READY", "LOCALIZATION_UNTRUSTED",
          "COMMAND_STALE", "ACTIVE")
# BASE_NOT_READY (only with GateConfig.require_base): the base bridge's /wc_nav/base_status does not say that the
# wheel serial owner runs automatic control, or it is stale or unreadable (velocity_channel.md 9.2 item 4).
BASE_REASONS = ("BASE_NOT_AUTO", "BASE_STATUS_STALE", "BASE_STATUS_INVALID")
# The gate's decision of every cycle for the base bridge, next to /wc_nav/cmd_vel_gated (velocity_channel.md 9.2
# item 1; the same field list as window A's wc_motion.auto_protocol.GATE_COMMAND_FIELDS).
GATE_COMMAND_TOPIC = "/wc_nav/gate_command"
GATE_COMMAND_SCHEMA = "wc_gate_command_v1"
GATE_COMMAND_FIELDS = ("schema", "gate_id", "seq", "stamp_ns", "mono_ns", "state", "reasons", "linear", "angular",
                       "limits", "config_sha256")
# "lateral" (default since 2026-11-09, user decision): only the deviation across the heading and the heading
# deviation stop the robot; a large deviation in another direction (in a corridor: along it) slows it instead,
# see GateConfig. "xy" (the default until 2026-11-09, still selectable): the larger of the map-axis position
# standard deviations stops it.
COVARIANCE_MODES = ("xy", "lateral")


def covariance_spread(covariance, yaw=None):
    """Position and heading standard deviations of a row-major 6x6 pose covariance, or None if it is invalid.

    ``x``/``y`` along the map axes, ``major``/``minor`` the axes of the position error ellipse, ``yaw`` the
    heading deviation (rad); with ``yaw`` also ``along``/``across`` the heading.
    """
    try:
        xx, xy, yy, tt = (float(covariance[k]) for k in (0, 1, 7, 35))
    except (TypeError, IndexError, ValueError):
        return None
    if not all(math.isfinite(v) for v in (xx, xy, yy, tt)) or min(xx, yy, tt) < 0:
        return None
    half_trace, root = (xx + yy) / 2.0, math.hypot((xx - yy) / 2.0, xy)
    spread = {"x": math.sqrt(xx), "y": math.sqrt(yy), "major": math.sqrt(max(half_trace + root, 0.0)),
              "minor": math.sqrt(max(half_trace - root, 0.0)), "yaw": math.sqrt(tt)}
    if yaw is not None:
        c, s = math.cos(float(yaw)), math.sin(float(yaw))
        spread["along"] = math.sqrt(max(c * c * xx + 2 * c * s * xy + s * s * yy, 0.0))
        spread["across"] = math.sqrt(max(s * s * xx - 2 * c * s * xy + c * c * yy, 0.0))
    return spread


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
    # (docs/architecture/live_shadow.md). In the lateral mode the same limit applies across the heading.
    covariance_xy_std_max_m: float = 0.50
    covariance_yaw_std_max_rad: float = 0.25
    # Covariance mode (see COVARIANCE_MODES): lateral since 2026-11-09 (user decision; was xy). In the ZL1107
    # corridor the deviation along it stays above the limit while the robot stands, so xy stopped it for good at
    # the corridor entrance (docs/architecture/live_shadow.md). Lateral only: above slow_std_m on the error
    # ellipse's major axis the robot is slowed to slow_linear until the axis is back at
    # slow_std_m - slow_hysteresis_m; the speed ceiling moves at slow_decel / slow_accel (the velocity smoother's
    # limits), linear and angular speed scale together.
    covariance_mode: str = "lateral"
    slow_std_m: float = 0.50
    slow_hysteresis_m: float = 0.05
    slow_linear: float = 0.15
    slow_decel: float = 0.6
    slow_accel: float = 0.3
    # Vehicle closed loop (velocity_channel.md 9.2 items 2 and 4; D.md 2026-11-10 16:45), all off by default:
    # require_base holds the robot (BASE_NOT_READY) unless the base bridge's /wc_nav/base_status, at most
    # base_status_timeout_s old, says the wheel serial owner runs automatic control; enable_source_node is the one
    # node allowed to publish /wc_nav/enable and /wc_nav/estop ("" = not checked).
    require_base: bool = False
    base_status_timeout_s: float = 0.3
    enable_source_node: str = ""

    def __post_init__(self):
        for name in ("max_linear", "max_angular", "command_timeout_s", "enable_timeout_s",
                     "localization_max_age_s", "covariance_xy_std_max_m", "covariance_yaw_std_max_rad",
                     "slow_std_m", "slow_hysteresis_m", "slow_linear", "slow_decel", "slow_accel",
                     "base_status_timeout_s"):
            object.__setattr__(self, name, _number(getattr(self, name), name, positive=True))
        for name in ("allow_reverse", "require_covariance", "require_base"):
            if type(getattr(self, name)) is not bool:
                raise ValueError(name + " must be a boolean")
        node = self.enable_source_node
        if not isinstance(node, str) or (node and (not node.startswith("/") or len(node) < 2)):
            raise ValueError("enable_source_node must be empty or a fully qualified node name like /wc_auto_bridge")
        if self.covariance_mode not in COVARIANCE_MODES:
            raise ValueError("covariance_mode must be one of " + ", ".join(COVARIANCE_MODES))
        if self.slow_hysteresis_m >= self.slow_std_m:
            raise ValueError("slow_hysteresis_m must be smaller than slow_std_m")
        if self.slow_linear > self.max_linear:
            raise ValueError("slow_linear must not exceed max_linear")

    def as_dict(self):
        return {name: getattr(self, name) for name in self.__dataclass_fields__}


SLOW_LIMITS_DEFAULT = (GateConfig.slow_std_m, GateConfig.slow_linear)


def add_covariance_mode_arguments(parser):
    """Options of the gate's covariance condition (simulation, recorded and live shadow runs)."""
    parser.add_argument("--gate-covariance-mode", choices=COVARIANCE_MODES,
                        help="covariance condition of the gate: lateral (default since 2026-11-09: the deviation "
                             "across the heading and the heading deviation stop, a large deviation along the "
                             "heading slows instead) or xy (the earlier condition: the larger map-axis deviation "
                             "stops)")
    parser.add_argument("--gate-slow-limits", nargs=2, type=float, metavar=("STD_M", "SPEED_M_S"),
                        help="lateral: slow to SPEED_M_S while the error ellipse's major axis is above STD_M "
                             "(default %g m, %g m/s)" % SLOW_LIMITS_DEFAULT)


def covariance_mode_settings(mode=None, slow_limits=None, max_linear=GateConfig.max_linear):
    """GateConfig keyword arguments of the covariance mode options, always explicit: the mode and, for lateral,
    the slow-down limits. ``mode=None`` is the gate's default mode (raises ValueError)."""
    mode = GateConfig.covariance_mode if mode is None else mode
    if mode not in COVARIANCE_MODES:
        raise ValueError("unknown gate covariance mode: " + str(mode))
    if mode == "xy":
        if slow_limits is not None:
            raise ValueError("--gate-slow-limits applies to the lateral covariance mode only")
        return {"covariance_mode": "xy"}
    std, speed = (float(v) for v in (slow_limits or SLOW_LIMITS_DEFAULT))
    if not (math.isfinite(std) and GateConfig.slow_hysteresis_m < std <= 2.0):
        raise ValueError("the slow-down deviation must be within %g..2 m" % GateConfig.slow_hysteresis_m)
    if not (math.isfinite(speed) and 0 < speed <= max_linear):
        raise ValueError("the slow speed must be within 0..%g m/s" % max_linear)
    return {"covariance_mode": "lateral", "slow_std_m": std, "slow_linear": speed}


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


COVARIANCE_REASONS = ("COVARIANCE_XY_TOO_LARGE", "COVARIANCE_YAW_TOO_LARGE", "COVARIANCE_ACROSS_HEADING_TOO_LARGE")


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
    # The linear speed limit of this decision (velocity_channel.md 9.2 item 1): the lateral slow-down's ceiling
    # (ramped) or max_linear; |linear| never exceeds it. None only for a decision built outside SafetyGate.
    linear_ceiling: float = None


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
        # closed loop only (require_base, enable_source_node); the node feeds both on every cycle
        self._base_ok, self._base_reason = False, "BASE_STATUS_STALE"
        self._sources_ok = not self.config.enable_source_node
        # lateral covariance mode: slow-down state, last assessed spread and the ramped speed ceiling
        self._slow = False
        self._spread = None
        self._ceiling = None
        self._last_active = None

    @property
    def slow(self):
        """Whether the lateral covariance mode currently slows the robot (False in the xy mode)."""
        return self.config.covariance_mode == "lateral" and self._slow

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

    def on_base(self, ok, reason=""):
        """Whether the wheel serial owner runs automatic control (require_base); ``reason`` one of BASE_REASONS."""
        self._base_ok = bool(ok)
        self._base_reason = "" if ok else (reason if reason in BASE_REASONS else "BASE_STATUS_INVALID")

    def on_sources(self, ok):
        """Whether /wc_nav/enable and /wc_nav/estop each have exactly the one allowed publisher (enable_source_node)."""
        self._sources_ok = bool(ok)

    def command_status(self, now):
        """The last monitored command and its age, for the status topic (None before the first command)."""
        if self._command is None:
            return None
        values = [float(v) if isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(float(v))
                  else None for v in self._command]
        return {"linear": values[0], "angular": values[1], "age_s": round(now - self._command_time, 3)}

    def conditions(self):
        """The input conditions as last fed, also while an earlier one holds the robot (status topic: an operator
        can see before "start automatic" whether localization is trusted)."""
        value = {"localization_ok": self._localization_ok, "localization_reason": self._localization_reason,
                 "estop_engaged": self._estop_engaged, "latched": self._latched, "enabled": self._enabled}
        if self.config.require_base:
            value.update(base_ok=self._base_ok, base_reason=self._base_reason)
        if self.config.enable_source_node:
            value["sources_ok"] = self._sources_ok
        return value

    def check_covariance(self, covariance, yaw=None):
        """Necessary-only covariance condition; small values never prove trust.

        The lateral mode needs the estimated heading ``yaw`` (rad, same frame as the covariance).
        """
        cfg = self.config
        if cfg.covariance_mode == "lateral":
            spread = covariance_spread(covariance, yaw)
            if spread is None:
                return False, "COVARIANCE_INVALID"
            if "across" not in spread:
                return False, "COVARIANCE_HEADING_UNKNOWN"
            if spread["across"] > cfg.covariance_xy_std_max_m:
                return False, "COVARIANCE_ACROSS_HEADING_TOO_LARGE"
            if spread["yaw"] > cfg.covariance_yaw_std_max_rad:
                return False, "COVARIANCE_YAW_TOO_LARGE"
            return True, ""
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

    def on_covariance(self, covariance, yaw=None):
        """Assess one localization estimate in time order: (ok, reason) as check_covariance, and in the lateral
        mode the slow-down state (major axis above slow_std_m; released at slow_std_m - slow_hysteresis_m)."""
        cfg = self.config
        ok, reason = self.check_covariance(covariance, yaw)
        if cfg.covariance_mode == "lateral":
            spread = covariance_spread(covariance, yaw)
            self._spread = spread
            if spread is None or spread["major"] > cfg.slow_std_m:
                self._slow = True
            elif spread["major"] <= cfg.slow_std_m - cfg.slow_hysteresis_m:
                self._slow = False
        return ok, reason

    def covariance_status(self):
        """Mode, last assessed deviations (lateral mode) and the slow-down state, for the status topic."""
        status = {"mode": self.config.covariance_mode}
        if self.config.covariance_mode == "lateral":
            spread = self._spread or {}
            status.update({("std_%s_m" % k if k != "yaw" else "std_yaw_rad"): round(v, 4)
                           for k, v in spread.items()})
            status.update(slow=self.slow, slow_std_m=self.config.slow_std_m, slow_linear=self.config.slow_linear,
                          speed_ceiling=None if self._ceiling is None else round(self._ceiling, 4))
        return status

    def _speed_ceiling(self, now):
        """Linear speed ceiling of the lateral mode's slow-down, ramped while commands pass."""
        cfg = self.config
        target = cfg.slow_linear if self.slow else cfg.max_linear
        if self._ceiling is None or self._last_active is None or not 0 <= now - self._last_active <= 0.25:
            self._ceiling = target              # the output was zero (or the clock jumped): no step to smooth
        elif self._ceiling > target:
            self._ceiling = max(target, self._ceiling - cfg.slow_decel * (now - self._last_active))
        else:
            self._ceiling = min(target, self._ceiling + cfg.slow_accel * (now - self._last_active))
        self._last_active = float(now)
        return self._ceiling

    def _held(self):
        """The linear limit in force while the output is zero: the lateral slow-down's target, else max_linear."""
        return self.config.slow_linear if self.slow else self.config.max_linear

    def _command_stop(self, now, held):
        """The decision when the command itself is missing, stale or invalid, else None."""
        cfg = self.config
        if self._command is None or now - self._command_time > cfg.command_timeout_s:
            return GateDecision(0.0, 0.0, "COMMAND_STALE", ("NO_FRESH_COMMAND",), held)
        linear, angular = self._command
        if any(isinstance(v, bool) or not isinstance(v, (int, float)) for v in (linear, angular)):
            return GateDecision(0.0, 0.0, "ACTIVE", ("INVALID_COMMAND_ZEROED",), held)
        if not (math.isfinite(float(linear)) and math.isfinite(float(angular))):
            return GateDecision(0.0, 0.0, "ACTIVE", ("NONFINITE_COMMAND_ZEROED",), held)
        return None

    def _limit(self, ceiling):
        """The valid command within the limits: no reverse, the linear ceiling (both speeds scaled together, so the
        curvature stays), max_linear and max_angular."""
        cfg = self.config
        linear, angular = (float(v) for v in self._command)
        reasons = []
        if linear < 0 and not cfg.allow_reverse:
            linear = 0.0
            reasons.append("REVERSE_BLOCKED")
        if ceiling < cfg.max_linear:
            reasons.append("LOCALIZATION_SLOW")
            if abs(linear) > ceiling:
                scale = ceiling / abs(linear)
                linear, angular = linear * scale, angular * scale
        if abs(linear) > cfg.max_linear:
            linear = math.copysign(cfg.max_linear, linear)
            reasons.append("LINEAR_CLAMPED")
        if abs(linear) > ceiling:        # rounding of the scaling above; the base checks |linear| <= linear_ceiling
            linear = math.copysign(ceiling, linear)
        if abs(angular) > cfg.max_angular:
            angular = math.copysign(cfg.max_angular, angular)
            reasons.append("ANGULAR_CLAMPED")
        return GateDecision(linear, angular, "ACTIVE", tuple(reasons), ceiling)

    def evaluate(self, now):
        cfg = self.config
        held = self._held()

        def stop(state, reason):
            return GateDecision(0.0, 0.0, state, (reason,), held)
        if self._latched:
            return stop("ESTOP_LATCHED", "ESTOP_ENGAGED" if self._estop_engaged else "ESTOP_RESET_REQUIRED")
        if not self._sources_ok:
            return stop("DISABLED", "ENABLE_SOURCE_AMBIGUOUS")
        if not self._enabled:
            return stop("DISABLED", "OPERATOR_ENABLE_OFF")
        if now - self._enable_time > cfg.enable_timeout_s:
            return stop("ENABLE_STALE", "ENABLE_HEARTBEAT_TIMEOUT")
        if cfg.require_base and not self._base_ok:
            return stop("BASE_NOT_READY", self._base_reason)
        if not self._localization_ok:
            return stop("LOCALIZATION_UNTRUSTED", self._localization_reason or "LOCALIZATION_FAILED")
        stopped = self._command_stop(now, held)
        if stopped is not None:
            return stopped
        ceiling = cfg.max_linear
        if cfg.covariance_mode == "lateral":
            ceiling = min(self._speed_ceiling(now), cfg.max_linear)
        return self._limit(ceiling)

    def preview(self, now):
        """What the gate would pass if the operator's enable, the base and the enable sources were all in order:
        the shadow's view while the chair stands before "start automatic" in the closed loop. The emergency stop
        latch, localization and the command count as in ``evaluate``; the lateral slow-down's target is used
        without its ramp. Changes no state, so ``evaluate`` is not affected."""
        held = self._held()
        if self._latched:
            return GateDecision(0.0, 0.0, "ESTOP_LATCHED", ("ESTOP_ENGAGED" if self._estop_engaged
                                                            else "ESTOP_RESET_REQUIRED",), held)
        if not self._localization_ok:
            return GateDecision(0.0, 0.0, "LOCALIZATION_UNTRUSTED",
                                (self._localization_reason or "LOCALIZATION_FAILED",), held)
        stopped = self._command_stop(now, held)
        return stopped if stopped is not None else self._limit(held)


def gate_command(decision, config, gate_id, seq, stamp_ns, mono_ns, config_sha256=None):
    """The ``/wc_nav/gate_command`` document of one decision (std_msgs/String JSON, schema wc_gate_command_v1).

    Exactly GATE_COMMAND_FIELDS: ``gate_id`` is drawn when the gate process starts, ``seq`` increases by one per
    cycle, ``stamp_ns`` is the node clock and ``mono_ns`` time.monotonic_ns() (the base bridge and the wheel owner
    judge freshness by it). ``limits`` gives the configured max_linear / max_angular, the decision's
    linear_ceiling and allow_reverse; ``config_sha256`` identifies the gate configuration file (or None).
    """
    ceiling = config.max_linear if decision.linear_ceiling is None else decision.linear_ceiling
    return {"schema": GATE_COMMAND_SCHEMA, "gate_id": str(gate_id), "seq": int(seq), "stamp_ns": int(stamp_ns),
            "mono_ns": int(mono_ns), "state": decision.state, "reasons": list(decision.reasons),
            "linear": float(decision.linear), "angular": float(decision.angular),
            "limits": {"max_linear": float(config.max_linear), "max_angular": float(config.max_angular),
                       "linear_ceiling": float(ceiling), "allow_reverse": bool(config.allow_reverse)},
            "config_sha256": config_sha256}
