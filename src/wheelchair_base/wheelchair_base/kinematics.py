import math
from dataclasses import dataclass
from typing import Tuple


@dataclass
class DifferentialDriveModel:
    wheel_radius_m: float = 0.165
    wheel_separation_m: float = 0.58
    max_linear_mps: float = 0.4
    max_angular_rps: float = 1.0

    def clamp_twist(self, linear_x: float, angular_z: float) -> Tuple[float, float]:
        linear_x = max(-self.max_linear_mps, min(self.max_linear_mps, linear_x))
        angular_z = max(-self.max_angular_rps, min(self.max_angular_rps, angular_z))
        return linear_x, angular_z

    def twist_to_wheel_rpm(self, linear_x: float, angular_z: float) -> Tuple[float, float]:
        linear_x, angular_z = self.clamp_twist(linear_x, angular_z)
        left_mps = linear_x - angular_z * self.wheel_separation_m * 0.5
        right_mps = linear_x + angular_z * self.wheel_separation_m * 0.5
        factor = 60.0 / (2.0 * math.pi * self.wheel_radius_m)
        return left_mps * factor, right_mps * factor

    def wheel_rpm_to_twist(self, left_rpm: float, right_rpm: float) -> Tuple[float, float]:
        factor = (2.0 * math.pi * self.wheel_radius_m) / 60.0
        left_mps = left_rpm * factor
        right_mps = right_rpm * factor
        linear_x = (left_mps + right_mps) * 0.5
        angular_z = (right_mps - left_mps) / self.wheel_separation_m
        return linear_x, angular_z


@dataclass
class OdometryState:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0

    def integrate(self, linear_x: float, angular_z: float, dt: float):
        if dt <= 0.0:
            return
        mid_yaw = self.yaw + angular_z * dt * 0.5
        self.x += linear_x * math.cos(mid_yaw) * dt
        self.y += linear_x * math.sin(mid_yaw) * dt
        self.yaw = math.atan2(math.sin(self.yaw + angular_z * dt), math.cos(self.yaw + angular_z * dt))


def yaw_to_quaternion(yaw: float):
    half = yaw * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)
