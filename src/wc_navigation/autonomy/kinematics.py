"""Simulated differential-drive base with bounded acceleration and imperfect odometry.

The true pose follows the commanded twist through acceleration limits and a
command watchdog. Odometry integrates a separately perturbed velocity, so the
mapping and localization stack has to cope with drift. Optionally
(``--drive-response``, WheelResponse) the two wheels follow their targets the way
the chair's hub motors were measured to: lag, a slow start at low speed, the
right wheel's shortfall when turning in place and the overshoot afterwards.
Simulation only.
"""
from __future__ import annotations

from dataclasses import dataclass, field
import math

import numpy as np

from ..geometry import _number


def wrap(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


@dataclass(frozen=True)
class BaseLimits:
    """Physical capability of the simulated base, above the navigation limits."""
    max_linear: float = 0.6
    max_angular: float = 1.0
    linear_accel: float = 0.8
    linear_decel: float = 1.0
    angular_accel: float = 2.0
    command_timeout_s: float = 0.5

    def __post_init__(self):
        for name in self.__dataclass_fields__:
            object.__setattr__(self, name, _number(getattr(self, name), name, positive=True))


@dataclass(frozen=True)
class OdometryErrorModel:
    """Wheel-odometry error: scale factors, a constant yaw-rate bias and noise.

    Noise standard deviations are per square-root second of motion. ``default``
    gives mild drift; ``ideal`` returns exact odometry for isolating planning.
    """
    linear_scale: float = 1.0
    angular_scale: float = 1.0
    yaw_rate_bias: float = 0.0
    linear_noise: float = 0.0
    angular_noise: float = 0.0
    seed: int = 0

    def __post_init__(self):
        for name in ("linear_scale", "angular_scale"):
            value = _number(getattr(self, name), name, positive=True)
            if not 0.5 <= value <= 1.5:
                raise ValueError(name + " must be within 0.5..1.5")
            object.__setattr__(self, name, value)
        object.__setattr__(self, "yaw_rate_bias", _number(self.yaw_rate_bias, "yaw_rate_bias"))
        for name in ("linear_noise", "angular_noise"):
            object.__setattr__(self, name, _number(getattr(self, name), name, nonnegative=True))
        if type(self.seed) is not int:
            raise ValueError("seed must be an integer")

    @classmethod
    def preset(cls, name, seed=0):
        """Named error models.

        ``wheel_imu``: wheel distance (2 % scale residual) with gyro-fused
        heading, the planned odometry path (docs/adr/0001). The yaw-rate bias
        equals the measured static gyro z mean (about 0.000375 rad/s) and only
        accumulates while moving, as with a zero-velocity update.
        ``wheel_only``: heading from wheel speeds, a stress case with 5x the
        bias and 1.5 % turn-scale error. ``ideal``: exact odometry.
        """
        if name == "ideal":
            return cls(seed=seed)
        if name == "wheel_imu":
            return cls(linear_scale=1.02, angular_scale=0.995, yaw_rate_bias=0.0004,
                       linear_noise=0.005, angular_noise=0.002, seed=seed)
        if name == "wheel_only":
            return cls(linear_scale=1.02, angular_scale=0.985, yaw_rate_bias=0.002,
                       linear_noise=0.005, angular_noise=0.004, seed=seed)
        raise ValueError("odometry error preset must be wheel_imu, wheel_only or ideal")

    def as_dict(self):
        return {name: getattr(self, name) for name in self.__dataclass_fields__}


@dataclass(frozen=True)
class WheelResponse:
    """How the two hub motors follow their speed targets (wheel surface speed, m/s, forward positive).

    Fitted to the 2026-11-11 recordings of the ZLAC8030D chair on the ground (window B, status/B.md 11-11):
    ``ZLauto_loop_2026_1111_202618`` (closed loop) and ``ZLauto_stand_wasd_2026_1111_153154`` (keyboard).
    One feedback word is 0.112 rpm, 0.00193 m/s at the 0.1646 m wheel radius.

    - Lag: each wheel approaches its target with time constant ``tau_s``; from ``slow_full_m_s`` down to
      ``slow_below_m_s`` the constant grows to ``tau_slow_s`` (12 words from rest took 0.8 s to half speed).
    - Pivot (the two targets have opposite signs, i.e. turning in place): the left wheel reaches
      ``pivot_gain_left`` and the right only ``pivot_gain_right`` of its target (12-31 words: 0.29-0.43);
      while both wheels go the same way the right one follows (21 words inside a curve: 0.96-1.04).
    - Right-wheel windup: while pivoting, the right wheel's shortfall is integrated (clamped at
      ``windup_limit_m``). After the pivot the right wheel stays at the pivot share for ``stuck_s`` from the
      first non-zero command (observed 0.54-0.83 s, still integrating), then breaks free: if its target has
      the sign of the integral, ``windup_gain_per_s`` x integral is added on top and decays with
      ``windup_decay_s`` (observed peaks 162-442 words for 0.4-0.6 s after 2.3-8.4 s pivots); otherwise the
      integral is dropped (observed: no overshoot, 0.21-0.58 of the target while stuck).
    - Overspeed: a wheel above ``overspeed_m_s`` (14.4 rpm) for ``overspeed_hold_s`` is counted as one event;
      on the chair the wheel owner stops there (FEEDBACK_OVERSPEED). The simulator only counts it.
    """
    name: str = "zlac8030d_20261111"
    track_m: float = 0.599
    tau_s: float = 0.12
    tau_slow_s: float = 0.8
    slow_below_m_s: float = 0.025
    slow_full_m_s: float = 0.048
    pivot_gain_left: float = 0.93
    pivot_gain_right: float = 0.37
    windup_gain_per_s: float = 4.5
    windup_limit_m: float = 0.193
    windup_decay_s: float = 0.4
    stuck_s: float = 0.7
    overspeed_m_s: float = 0.248
    overspeed_hold_s: float = 0.2

    def __post_init__(self):
        if not isinstance(self.name, str) or not self.name:
            raise ValueError("wheel response needs a name")
        for name in self.__dataclass_fields__:
            if name != "name":
                object.__setattr__(self, name, _number(getattr(self, name), name, positive=True))
        if not self.slow_below_m_s < self.slow_full_m_s:
            raise ValueError("slow_below_m_s must be below slow_full_m_s")
        for name in ("pivot_gain_left", "pivot_gain_right"):
            if getattr(self, name) > 1.5:
                raise ValueError(name + " must be within 0..1.5")

    def as_dict(self):
        return {name: getattr(self, name) for name in self.__dataclass_fields__}


# Named drive responses: the wheel response and the wheel owner's ramps on the axle command (BaseLimits fields).
# zlac8030d_20261111: auto_controls of the 11-11 recordings (0.3 / 0.6 m/s^2, 0.8 rad/s^2).
DRIVE_RESPONSES = {
    "zlac8030d_20261111": (WheelResponse(), {"linear_accel": 0.3, "linear_decel": 0.6, "angular_accel": 0.8}),
}


def drive_response_config(name):
    """Simulator config entries of a named drive response; ``ideal`` (None) keeps the unicycle as it is."""
    if name in (None, "", "ideal"):
        return None
    if name not in DRIVE_RESPONSES:
        raise ValueError("drive response must be ideal or one of " + ", ".join(sorted(DRIVE_RESPONSES)))
    response, ramps = DRIVE_RESPONSES[name]
    return {"wheel_response": response.as_dict(), "base_limits": dict(ramps)}


class WheelDrive:
    """State of the two wheels under a WheelResponse; one per simulated base."""

    EPS_M_S = 1e-3

    def __init__(self, response):
        self.response = response
        self.left = self.right = 0.0
        self.windup = 0.0
        self.pivot_s = 0.0
        self.releases = 0
        self.overspeed = []
        self._armed = False          # a pivot ended and the right wheel has not broken free yet
        self._stuck_s = 0.0
        self._over_since = None
        self.excursions = 0          # times a wheel went above overspeed_m_s (any duration)
        self.fastest = 0.0

    def _tau(self, target, current):
        """Lag toward ``target``: slow only while speeding up to a low target (slowing down is quick)."""
        r = self.response
        speed = abs(target)
        if speed <= abs(current) or speed >= r.slow_full_m_s:
            return r.tau_s
        if speed <= r.slow_below_m_s:
            return r.tau_slow_s
        share = (speed - r.slow_below_m_s) / (r.slow_full_m_s - r.slow_below_m_s)
        return r.tau_slow_s + share * (r.tau_s - r.tau_slow_s)

    def reset(self):
        """Wheels held (collision veto): both stop; the integral is kept."""
        self.left = self.right = 0.0

    def step(self, target_left, target_right, dt, now):
        """Advance by ``dt`` toward the wheel targets (m/s); returns the wheel speeds (left, right)."""
        r = self.response
        eps = self.EPS_M_S
        pivot = target_left * target_right < 0 and abs(target_left) > eps and abs(target_right) > eps
        moving = abs(target_left) > eps or abs(target_right) > eps
        aim_left = target_left * (r.pivot_gain_left if pivot else 1.0)
        aim_right = target_right * (r.pivot_gain_right if pivot else 1.0)

        def integrate():
            self.windup = max(-r.windup_limit_m, min(r.windup_limit_m,
                                                     self.windup + (target_right - self.right) * dt))
        if pivot:
            self.pivot_s += dt
            self._armed, self._stuck_s = True, 0.0
            integrate()
        elif self._armed:
            if moving:
                self._stuck_s += dt
            if self._stuck_s < r.stuck_s:
                aim_right = target_right * r.pivot_gain_right       # still held back, still winding up
                if moving:
                    integrate()
            else:                                                   # breaks free
                self._armed = False
                self.releases += 1
                if self.windup * target_right <= 0:
                    self.windup = 0.0
        if not pivot and not self._armed and self.windup:
            aim_right = target_right + r.windup_gain_per_s * self.windup
            # the integral unwinds by the wheel's excess speed (like a speed loop's integrator) and leaks with
            # windup_decay_s, so the overshoot collapses after about half a second as on 11-11
            sign = math.copysign(1.0, self.windup)
            self.windup += (target_right - self.right) * dt - self.windup * dt / r.windup_decay_s
            if self.windup * sign <= 1e-5:
                self.windup = 0.0
        self.left += (aim_left - self.left) * min(1.0, dt / self._tau(aim_left, self.left))
        self.right += (aim_right - self.right) * min(1.0, dt / self._tau(aim_right, self.right))
        fastest = max(abs(self.left), abs(self.right))
        self.fastest = max(self.fastest, fastest)
        if fastest > r.overspeed_m_s:
            if self._over_since is None:
                self._over_since, self._over_counted = now, False
                self.excursions += 1
            if not self._over_counted and now - self._over_since >= r.overspeed_hold_s - 1e-9:
                self.overspeed.append({"sim_time_s": round(float(now), 3), "peak_m_s": round(fastest, 3)})
                self._over_counted = True
            elif self._over_counted:
                self.overspeed[-1]["peak_m_s"] = round(max(self.overspeed[-1]["peak_m_s"], fastest), 3)
        else:
            self._over_since = None
        return self.left, self.right

    def summary(self):
        return {"model": self.response.name, "pivot_s": round(self.pivot_s, 2), "windup_releases": self.releases,
                "overspeed_events": len(self.overspeed), "overspeed": self.overspeed[:50],
                "excursions_over_limit": self.excursions, "fastest_wheel_m_s": round(self.fastest, 3),
                "wheel_speeds_m_s": [round(self.left, 4), round(self.right, 4)]}


def _approach(current, target, up, down, dt):
    """Move toward target with separate acceleration and braking limits."""
    if target == current:
        return current
    braking = abs(target) < abs(current) or target * current < 0
    limit = (down if braking else up) * dt
    return current + max(-limit, min(limit, target - current))


def integrate(pose, v, w, dt):
    """Exact unicycle arc integration."""
    x, y, yaw = pose
    if abs(w) < 1e-9:
        return x + v * dt * math.cos(yaw), y + v * dt * math.sin(yaw), yaw
    new_yaw = yaw + w * dt
    return (x + v / w * (math.sin(new_yaw) - math.sin(yaw)),
            y - v / w * (math.cos(new_yaw) - math.cos(yaw)), wrap(new_yaw))


@dataclass
class DiffDriveSim:
    pose: tuple
    limits: BaseLimits = field(default_factory=BaseLimits)
    odometry: OdometryErrorModel = field(default_factory=OdometryErrorModel)
    v: float = 0.0
    w: float = 0.0
    odom_pose: tuple = (0.0, 0.0, 0.0)
    odom_v: float = 0.0
    odom_w: float = 0.0
    command: tuple = (0.0, 0.0)
    command_time: float = -math.inf
    distance_m: float = 0.0
    collisions: int = 0
    # Optional WheelDrive: the limits then act as the wheel owner's ramps on the axle command, the wheels follow
    # the ramped targets through the measured response, and v, w are what the wheels actually do.
    wheels: object = None
    ramp_v: float = 0.0
    ramp_w: float = 0.0

    def __post_init__(self):
        self.pose = tuple(float(v) for v in self.pose)
        if len(self.pose) != 3 or not all(math.isfinite(v) for v in self.pose):
            raise ValueError("pose must be finite x, y, yaw")
        self._rng = np.random.default_rng(self.odometry.seed)

    def set_command(self, linear, angular, now):
        if not (math.isfinite(linear) and math.isfinite(angular)):
            linear = angular = 0.0
        lim = self.limits
        self.command = (max(-lim.max_linear, min(lim.max_linear, float(linear))),
                        max(-lim.max_angular, min(lim.max_angular, float(angular))))
        self.command_time = float(now)

    def target(self, now):
        if now - self.command_time > self.limits.command_timeout_s:
            return 0.0, 0.0
        return self.command

    def step(self, dt, now, collides=None):
        """Advance by ``dt``. ``collides(old_pose, new_pose)`` (x, y, yaw) vetoes the motion.

        Full poses are passed so a non-circular body can be checked while turning.

        A vetoed step leaves the pose unchanged, zeroes the velocity and counts
        one collision event per contact onset. Returns True when moved freely.
        """
        tv, tw = self.target(now)
        lim = self.limits
        if self.wheels is None:
            v_next = _approach(self.v, tv, lim.linear_accel, lim.linear_decel, dt)
            w_next = _approach(self.w, tw, lim.angular_accel, lim.angular_accel, dt)
        else:
            self.ramp_v = _approach(self.ramp_v, tv, lim.linear_accel, lim.linear_decel, dt)
            self.ramp_w = _approach(self.ramp_w, tw, lim.angular_accel, lim.angular_accel, dt)
            half = 0.5 * self.wheels.response.track_m
            left, right = self.wheels.step(self.ramp_v - self.ramp_w * half, self.ramp_v + self.ramp_w * half,
                                           dt, now)
            v_next, w_next = 0.5 * (left + right), (right - left) / (2.0 * half)
        v_mid, w_mid = 0.5 * (self.v + v_next), 0.5 * (self.w + w_next)
        proposed = integrate(self.pose, v_mid, w_mid, dt)
        if collides is not None and (v_mid or w_mid) and collides(self.pose, proposed):
            if self.v or self.w:
                self.collisions += 1
            self.v = self.w = 0.0
            self.odom_v = self.odom_w = 0.0
            if self.wheels is not None:
                self.wheels.reset()
                self.ramp_v = self.ramp_w = 0.0
            return False
        self.distance_m += math.hypot(proposed[0] - self.pose[0], proposed[1] - self.pose[1])
        self.pose = proposed
        self.v, self.w = v_next, w_next
        model = self.odometry
        moving = abs(v_mid) > 1e-6 or abs(w_mid) > 1e-6
        noise_v = self._rng.normal(0.0, model.linear_noise / math.sqrt(dt)) if moving and model.linear_noise else 0.0
        noise_w = self._rng.normal(0.0, model.angular_noise / math.sqrt(dt)) if moving and model.angular_noise else 0.0
        self.odom_v = v_mid * model.linear_scale + noise_v
        self.odom_w = w_mid * model.angular_scale + (model.yaw_rate_bias if moving else 0.0) + noise_w
        self.odom_pose = integrate(self.odom_pose, self.odom_v, self.odom_w, dt)
        return True
