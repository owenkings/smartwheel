from abc import ABC, abstractmethod
from enum import Enum
from typing import Generic, Optional, TypeVar


T = TypeVar("T")


class BackendMode(str, Enum):
    MOCK = "mock"
    REAL = "real"


class SensorBackend(ABC, Generic[T]):
    @abstractmethod
    def sample(self, timestamp_sec: float) -> T:
        raise NotImplementedError

    def close(self) -> None:
        return None


def guard_real_backend(mode: str, hardware_enabled: bool, adapter_ready: bool) -> BackendMode:
    try:
        resolved = BackendMode(mode)
    except ValueError as exc:
        raise ValueError(f"unsupported backend mode: {mode}") from exc
    if resolved is BackendMode.REAL and not hardware_enabled:
        raise RuntimeError("real backend refused because hardware_enabled=false")
    if resolved is BackendMode.REAL and not adapter_ready:
        raise NotImplementedError("real vendor adapter is NOT_IMPLEMENTED in Stage A")
    return resolved


class TimestampMonitor:
    def __init__(self) -> None:
        self._last: Optional[float] = None

    @property
    def last(self) -> Optional[float]:
        return self._last

    def observe(self, timestamp_sec: float) -> None:
        if timestamp_sec < 0.0:
            raise ValueError("timestamp must be non-negative")
        if self._last is not None and timestamp_sec < self._last:
            raise ValueError(
                f"timestamp regression: current={timestamp_sec:.9f}, last={self._last:.9f}"
            )
        self._last = timestamp_sec

