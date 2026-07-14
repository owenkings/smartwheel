import math
from dataclasses import dataclass
from typing import Optional


def unwrap_delta(current: int, previous: int, bits: int) -> int:
    if bits < 2 or bits > 63:
        raise ValueError("encoder bits must be in [2, 63]")
    modulus = 1 << bits
    half = modulus >> 1
    return ((int(current) - int(previous) + half) % modulus) - half


@dataclass(frozen=True)
class EncoderConfig:
    wheel_radius_m: float
    track_width_m: float
    encoder_cpr: int
    gear_ratio: float
    left_sign: int = 1
    right_sign: int = 1
    encoder_bits: int = 32
    count_stddev: float = 1.0

    def __post_init__(self) -> None:
        if self.wheel_radius_m <= 0.0 or self.track_width_m <= 0.0:
            raise ValueError("wheel radius and track width must be positive")
        if self.encoder_cpr <= 0 or self.gear_ratio <= 0.0:
            raise ValueError("encoder CPR and gear ratio must be positive")
        if self.left_sign not in (-1, 1) or self.right_sign not in (-1, 1):
            raise ValueError("wheel signs must be -1 or 1")
        if self.encoder_bits < 2 or self.encoder_bits > 63:
            raise ValueError("encoder bits must be in [2, 63]")

    @property
    def metres_per_count(self) -> float:
        return 2.0 * math.pi * self.wheel_radius_m / (self.encoder_cpr * self.gear_ratio)


@dataclass(frozen=True)
class OdomUpdate:
    x: float
    y: float
    yaw: float
    linear_mps: float
    angular_rps: float
    dt: float
    distance_left_m: float
    distance_right_m: float
    position_variance: float
    yaw_variance: float


class DifferentialOdometry:
    def __init__(self, config: EncoderConfig) -> None:
        self.config = config
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self._left: Optional[int] = None
        self._right: Optional[int] = None
        self._stamp: Optional[float] = None

    def reset(self) -> None:
        self.__init__(self.config)

    def update(self, left_count: int, right_count: int, stamp_sec: float) -> Optional[OdomUpdate]:
        if not math.isfinite(stamp_sec) or stamp_sec < 0.0:
            raise ValueError("encoder timestamp must be finite and non-negative")
        if self._stamp is None:
            self._left, self._right, self._stamp = int(left_count), int(right_count), stamp_sec
            return None
        dt = stamp_sec - self._stamp
        if dt <= 0.0:
            raise ValueError("encoder timestamps must be strictly increasing")
        left_delta = unwrap_delta(left_count, self._left, self.config.encoder_bits)
        right_delta = unwrap_delta(right_count, self._right, self.config.encoder_bits)
        dl = left_delta * self.config.left_sign * self.config.metres_per_count
        dr = right_delta * self.config.right_sign * self.config.metres_per_count
        distance = 0.5 * (dl + dr)
        dyaw = (dr - dl) / self.config.track_width_m
        heading_mid = self.yaw + 0.5 * dyaw
        self.x += distance * math.cos(heading_mid)
        self.y += distance * math.sin(heading_mid)
        self.yaw = math.atan2(math.sin(self.yaw + dyaw), math.cos(self.yaw + dyaw))
        self._left, self._right, self._stamp = int(left_count), int(right_count), stamp_sec
        count_sigma_m = self.config.count_stddev * self.config.metres_per_count
        position_variance = 0.5 * count_sigma_m * count_sigma_m
        yaw_variance = 2.0 * count_sigma_m * count_sigma_m / (self.config.track_width_m**2)
        return OdomUpdate(
            x=self.x,
            y=self.y,
            yaw=self.yaw,
            linear_mps=distance / dt,
            angular_rps=dyaw / dt,
            dt=dt,
            distance_left_m=dl,
            distance_right_m=dr,
            position_variance=position_variance,
            yaw_variance=yaw_variance,
        )

