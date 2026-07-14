import math
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float


class ClosedLoopTrajectory:
    def __init__(self, waypoints=None) -> None:
        if waypoints is None:
            waypoints = (
                (1.0, 1.0),
                (7.0, 1.0),
                (9.0, 3.0),
                (13.0, 3.0),
                (13.0, 8.0),
                (13.0, 3.0),
                (7.0, 3.0),
                (7.0, 5.0),
                (1.0, 5.0),
                (1.0, 1.0),
            )
        self.waypoints = np.asarray(waypoints, dtype=np.float64)
        if self.waypoints.ndim != 2 or self.waypoints.shape[0] < 3 or self.waypoints.shape[1] != 2:
            raise ValueError("trajectory needs at least three 2D waypoints")
        if not np.allclose(self.waypoints[0], self.waypoints[-1]):
            raise ValueError("trajectory must be closed")
        self._segments = np.diff(self.waypoints, axis=0)
        self._lengths = np.linalg.norm(self._segments, axis=1)
        if np.any(self._lengths <= 0.0):
            raise ValueError("trajectory contains zero-length segments")
        self._cumulative = np.concatenate(([0.0], np.cumsum(self._lengths)))
        self.length_m = float(self._cumulative[-1])

    def sample(self, time_sec: float, duration_sec: float) -> Pose2D:
        if duration_sec <= 0.0:
            raise ValueError("duration must be positive")
        phase = min(max(time_sec / duration_sec, 0.0), 1.0)
        distance = phase * self.length_m
        if phase >= 1.0:
            index = len(self._segments) - 1
            fraction = 1.0
        else:
            index = int(np.searchsorted(self._cumulative, distance, side="right") - 1)
            fraction = (distance - self._cumulative[index]) / self._lengths[index]
        point = self.waypoints[index] + fraction * self._segments[index]
        direction = self._segments[index]
        yaw = math.atan2(direction[1], direction[0])
        return Pose2D(float(point[0]), float(point[1]), yaw)

