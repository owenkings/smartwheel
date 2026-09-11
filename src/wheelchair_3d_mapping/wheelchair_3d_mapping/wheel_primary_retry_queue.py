"""ROS-independent bounded retry queue for exact-time cloud transforms."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from enum import Enum, auto
import math
from typing import Callable, Deque, Generic, TypeVar


Item = TypeVar("Item")
Payload = TypeVar("Payload")


class RetryDecision(Enum):
    """Result of one non-blocking transform attempt."""

    READY = auto()
    RETRY = auto()
    DROP = auto()


@dataclass(frozen=True)
class RetryStats:
    published: int = 0
    waiting: int = 0
    expired: int = 0
    permanent: int = 0
    health_cleared: int = 0


@dataclass(frozen=True)
class _Pending(Generic[Item]):
    item: Item
    received_at: float


class BoundedRetryQueue(Generic[Item, Payload]):
    """Keep a small, age-limited set of items awaiting an external resource.

    ``attempt`` must be non-blocking. It returns ``RETRY`` while the exact-time
    transform is not in the TF buffer, ``DROP`` for a permanent error, or
    ``READY`` with a payload. Health is checked again after a successful attempt
    and immediately before ``publish`` so stale vehicle state cannot leak through
    merely because transforming the cloud took some time.
    """

    def __init__(self, *, max_items: int, max_age_sec: float) -> None:
        if (
            isinstance(max_items, bool)
            or int(max_items) != max_items
            or int(max_items) < 1
            or not math.isfinite(float(max_age_sec))
            or float(max_age_sec) <= 0.0
        ):
            raise ValueError("retry queue bounds must be finite and positive")
        self._max_items = int(max_items)
        self._max_age = float(max_age_sec)
        self._items: Deque[_Pending[Item]] = deque()

    def __len__(self) -> int:
        return len(self._items)

    def clear(self) -> int:
        count = len(self._items)
        self._items.clear()
        return count

    def push(self, item: Item, received_at: float) -> int:
        received = float(received_at)
        if not math.isfinite(received):
            raise ValueError("retry receive time must be finite")
        dropped = 0
        if len(self._items) >= self._max_items:
            self._items.popleft()
            dropped = 1
        self._items.append(_Pending(item=item, received_at=received))
        return dropped

    def process(
        self,
        *,
        now: float,
        healthy: Callable[[], bool],
        attempt: Callable[[Item], tuple[RetryDecision, Payload | None]],
        publish: Callable[[Payload], None],
    ) -> RetryStats:
        current = float(now)
        if not math.isfinite(current):
            raise ValueError("retry process time must be finite")
        if not healthy():
            return RetryStats(health_cleared=self.clear())

        retained: Deque[_Pending[Item]] = deque()
        published = expired = permanent = health_cleared = 0
        while self._items:
            pending = self._items.popleft()
            if current - pending.received_at > self._max_age:
                expired += 1
                continue
            if not healthy():
                health_cleared += 1 + len(self._items) + len(retained)
                self._items.clear()
                retained.clear()
                break
            decision, payload = attempt(pending.item)
            if decision is RetryDecision.RETRY:
                retained.append(pending)
                continue
            if decision is RetryDecision.DROP:
                permanent += 1
                continue
            if decision is not RetryDecision.READY or payload is None:
                raise ValueError("READY retry result requires a payload")
            # This is deliberately after the potentially expensive XYZ copy.
            if not healthy():
                health_cleared += 1 + len(self._items) + len(retained)
                self._items.clear()
                retained.clear()
                break
            publish(payload)
            published += 1

        self._items = retained
        return RetryStats(
            published=published,
            waiting=len(retained),
            expired=expired,
            permanent=permanent,
            health_cleared=health_cleared,
        )
