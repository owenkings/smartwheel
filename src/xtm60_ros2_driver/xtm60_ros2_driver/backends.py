from abc import ABC

import numpy as np

from smartwheel_sensor_api.contracts import SensorBackend, guard_real_backend


class VendorSdkAdapter(SensorBackend[np.ndarray], ABC):
    """Phase B boundary. No SDK symbols or guessed packet fields exist in Stage A."""

    def sample(self, timestamp_sec: float) -> np.ndarray:
        raise NotImplementedError("XT-M60 vendor SDK adapter is NOT_IMPLEMENTED in Stage A")


class MockXtm60Backend(SensorBackend[np.ndarray]):
    def __init__(self, seed: int = 20260714, point_count: int = 240) -> None:
        self._rng = np.random.default_rng(seed)
        self._point_count = point_count

    def sample(self, timestamp_sec: float) -> np.ndarray:
        angles = np.linspace(-np.pi / 3.0, np.pi / 3.0, self._point_count)
        distance = 4.0 + 0.15 * np.sin(angles * 5.0 + timestamp_sec)
        z = 0.2 + 1.2 * (np.arange(self._point_count) % 12) / 11.0
        points = np.column_stack((distance * np.cos(angles), distance * np.sin(angles), z))
        return points + self._rng.normal(0.0, 0.002, points.shape)


def create_backend(mode: str, hardware_enabled: bool, seed: int = 20260714):
    resolved = guard_real_backend(mode, hardware_enabled, adapter_ready=False)
    if resolved.value == "mock":
        return MockXtm60Backend(seed=seed)
    raise AssertionError("unreachable Stage A real backend")

