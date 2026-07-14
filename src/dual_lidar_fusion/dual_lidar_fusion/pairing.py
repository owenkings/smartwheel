from dataclasses import dataclass
from typing import Generic, Optional, TypeVar


T = TypeVar("T")


@dataclass(frozen=True)
class TimedItem(Generic[T]):
    stamp: float
    value: T


class TimestampPairer(Generic[T]):
    def __init__(self, tolerance_sec: float, queue_size: int = 20) -> None:
        if tolerance_sec < 0.0 or queue_size < 1:
            raise ValueError("invalid pairer configuration")
        self.tolerance = tolerance_sec
        self.queue_size = queue_size
        self.left: list[TimedItem[T]] = []
        self.right: list[TimedItem[T]] = []
        self.dropped = 0

    def add_left(self, stamp: float, value: T) -> Optional[tuple[TimedItem[T], TimedItem[T]]]:
        return self._add(self.left, self.right, TimedItem(stamp, value), left_incoming=True)

    def add_right(self, stamp: float, value: T) -> Optional[tuple[TimedItem[T], TimedItem[T]]]:
        return self._add(self.right, self.left, TimedItem(stamp, value), left_incoming=False)

    def _add(self, own, other, item, left_incoming):
        own.append(item)
        own.sort(key=lambda candidate: candidate.stamp)
        if other:
            best_index = min(range(len(other)), key=lambda index: abs(other[index].stamp - item.stamp))
            match = other[best_index]
            if abs(match.stamp - item.stamp) <= self.tolerance:
                own.remove(item)
                other.pop(best_index)
                return (item, match) if left_incoming else (match, item)
        while len(own) > self.queue_size:
            own.pop(0)
            self.dropped += 1
        self._drop_impossible()
        return None

    def _drop_impossible(self) -> None:
        if not self.left or not self.right:
            return
        newest_left = self.left[-1].stamp
        newest_right = self.right[-1].stamp
        while self.left and self.left[0].stamp < newest_right - self.tolerance:
            self.left.pop(0)
            self.dropped += 1
        while self.right and self.right[0].stamp < newest_left - self.tolerance:
            self.right.pop(0)
            self.dropped += 1

