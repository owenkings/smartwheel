from dataclasses import dataclass


@dataclass(frozen=True)
class OdomVelocity:
    linear: float
    angular: float


class ResidualMonitor:
    def __init__(self, max_linear: float, max_angular: float) -> None:
        if max_linear <= 0.0 or max_angular <= 0.0:
            raise ValueError("residual limits must be positive")
        self.max_linear = max_linear
        self.max_angular = max_angular

    def compare(self, lio: OdomVelocity, wheel: OdomVelocity) -> dict:
        linear = abs(lio.linear - wheel.linear)
        angular = abs(lio.angular - wheel.angular)
        score = max(0.0, 1.0 - max(linear / self.max_linear, angular / self.max_angular))
        return {
            "linear_residual": linear,
            "angular_residual": angular,
            "score": score,
            "degraded": linear > self.max_linear or angular > self.max_angular,
        }

