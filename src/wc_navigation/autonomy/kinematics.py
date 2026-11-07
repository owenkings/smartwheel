"""Simulated differential-drive base with bounded acceleration and imperfect odometry.

The true pose follows the commanded twist through acceleration limits and a
command watchdog. Odometry integrates a separately perturbed velocity, so the
mapping and localization stack has to cope with drift. Simulation only.
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
        v_next = _approach(self.v, tv, lim.linear_accel, lim.linear_decel, dt)
        w_next = _approach(self.w, tw, lim.angular_accel, lim.angular_accel, dt)
        v_mid, w_mid = 0.5 * (self.v + v_next), 0.5 * (self.w + w_next)
        proposed = integrate(self.pose, v_mid, w_mid, dt)
        if collides is not None and (v_mid or w_mid) and collides(self.pose, proposed):
            if self.v or self.w:
                self.collisions += 1
            self.v = self.w = 0.0
            self.odom_v = self.odom_w = 0.0
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
