"""Bounded, explicit point-cloud session capture state.

The capture object deliberately keeps only the voxelized product in memory.
It records enough provenance to prove that a completed product covered one
continuous input session without pretending that voxelization preserves every
raw sample.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from smartwheel_map_products.accumulator import VoxelAccumulator


class SessionCaptureError(RuntimeError):
    """The session cannot accept the requested operation."""


class SessionStateError(SessionCaptureError):
    """The operation is invalid for the current lifecycle state."""


@dataclass(frozen=True)
class SessionSummary:
    state: str
    complete: bool
    failure_reason: str
    expected_frame_id: str
    frame_ids: tuple[str, ...]
    frame_count: int
    rejected_frame_count: int
    empty_frame_count: int
    raw_point_count: int
    voxel_point_count: int
    first_cloud_stamp: float | None
    last_cloud_stamp: float | None
    cloud_time_span_sec: float | None

    def as_dict(self) -> dict:
        return {
            "state": self.state,
            "complete": self.complete,
            "failure_reason": self.failure_reason,
            "expected_frame_id": self.expected_frame_id,
            "frame_ids": list(self.frame_ids),
            "frame_count": self.frame_count,
            "rejected_frame_count": self.rejected_frame_count,
            "empty_frame_count": self.empty_frame_count,
            "raw_point_count": self.raw_point_count,
            "voxel_point_count": self.voxel_point_count,
            "first_cloud_stamp": self.first_cloud_stamp,
            "last_cloud_stamp": self.last_cloud_stamp,
            "cloud_time_span_sec": self.cloud_time_span_sec,
        }


class SessionCapture:
    """Capture one bounded point-cloud session.

    ``start`` is explicit so callers cannot accidentally treat a partially
    initialized object as a valid session.  ``add_frame`` accepts already
    world-coordinate points; frame transforms remain the ROS-node concern.
    """

    IDLE = "IDLE"
    CAPTURING = "CAPTURING"
    STOPPED = "STOPPED"
    FAILED = "FAILED"

    def __init__(
        self,
        voxel_size_m: float,
        maximum_points: int,
        expected_frame_id: str = "",
    ) -> None:
        self._accumulator = VoxelAccumulator(voxel_size_m, maximum_points)
        self._expected_frame_id = expected_frame_id.strip()
        self._frame_ids: set[str] = set()
        self._state = self.IDLE
        self._failure_reason = ""
        self._frame_count = 0
        self._rejected_frame_count = 0
        self._empty_frame_count = 0
        self._raw_point_count = 0
        self._first_cloud_stamp: float | None = None
        self._last_cloud_stamp: float | None = None

    @property
    def accumulator(self) -> VoxelAccumulator:
        return self._accumulator

    @property
    def state(self) -> str:
        return self._state

    @property
    def failure_reason(self) -> str:
        return self._failure_reason

    def start(self) -> None:
        if self._state is not self.IDLE:
            raise SessionStateError(f"cannot start session in state {self._state}")
        self._state = self.CAPTURING

    def stop(self) -> None:
        if self._state is self.STOPPED:
            return
        if self._state is self.FAILED:
            raise SessionStateError(self._failure_reason or "session has failed")
        if self._state is not self.CAPTURING:
            raise SessionStateError(f"cannot stop session in state {self._state}")
        self._state = self.STOPPED

    def fail(self, reason: str) -> None:
        reason = reason.strip()
        if not reason:
            raise ValueError("session failure reason must be specific")
        self._failure_reason = reason
        self._state = self.FAILED

    def reject_frame(self) -> None:
        self._rejected_frame_count += 1

    def add_frame(
        self,
        stamp: float,
        frame_id: str,
        points: np.ndarray,
        origins: np.ndarray,
        intensity: np.ndarray | None = None,
    ) -> int:
        if self._state is not self.CAPTURING:
            raise SessionStateError(f"cannot add frame in state {self._state}")
        try:
            stamp = float(stamp)
        except (TypeError, ValueError) as exc:
            self.fail("cloud timestamp is not numeric")
            raise SessionCaptureError(self._failure_reason) from exc
        if not np.isfinite(stamp):
            self.fail("cloud timestamp must be finite")
            raise SessionCaptureError(self._failure_reason)
        frame_id = frame_id.strip()
        if not frame_id:
            self.fail("cloud frame_id must be non-empty")
            raise SessionCaptureError(self._failure_reason)
        if not self._expected_frame_id and self._frame_ids:
            first_frame_id = next(iter(self._frame_ids))
            if frame_id != first_frame_id:
                self.fail(
                    f"cloud frame_id changed: expected {first_frame_id}, got {frame_id}"
                )
                raise SessionCaptureError(self._failure_reason)
        if self._expected_frame_id and frame_id != self._expected_frame_id:
            self.fail(
                f"cloud frame_id changed: expected {self._expected_frame_id}, got {frame_id}"
            )
            raise SessionCaptureError(self._failure_reason)
        if self._last_cloud_stamp is not None and stamp < self._last_cloud_stamp:
            self.fail(
                f"cloud timestamps are not monotonic: {stamp} < {self._last_cloud_stamp}"
            )
            raise SessionCaptureError(self._failure_reason)

        xyz = np.asarray(points)
        if not self._expected_frame_id:
            self._expected_frame_id = frame_id
        if xyz.ndim == 2 and xyz.shape[0] == 0:
            self._empty_frame_count += 1
            self._last_cloud_stamp = stamp
            self._frame_ids.add(frame_id)
            if self._first_cloud_stamp is None:
                self._first_cloud_stamp = stamp
            return 0

        try:
            added = self._accumulator.add(points, origins, intensity)
        except (OverflowError, ValueError) as exc:
            self.fail(str(exc))
            raise SessionCaptureError(self._failure_reason) from exc

        self._frame_count += 1
        self._raw_point_count += int(xyz.shape[0])
        self._frame_ids.add(frame_id)
        if self._first_cloud_stamp is None:
            self._first_cloud_stamp = stamp
        self._last_cloud_stamp = stamp
        return added

    def summary(self) -> SessionSummary:
        span = None
        if self._first_cloud_stamp is not None and self._last_cloud_stamp is not None:
            span = self._last_cloud_stamp - self._first_cloud_stamp
        return SessionSummary(
            state=self._state,
            complete=self._state is self.STOPPED,
            failure_reason=self._failure_reason,
            expected_frame_id=self._expected_frame_id,
            frame_ids=tuple(sorted(self._frame_ids)),
            frame_count=self._frame_count,
            rejected_frame_count=self._rejected_frame_count,
            empty_frame_count=self._empty_frame_count,
            raw_point_count=self._raw_point_count,
            voxel_point_count=len(self._accumulator),
            first_cloud_stamp=self._first_cloud_stamp,
            last_cloud_stamp=self._last_cloud_stamp,
            cloud_time_span_sec=span,
        )
