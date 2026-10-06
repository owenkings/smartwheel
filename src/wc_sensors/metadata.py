"""Explicit identities, integer time and owning source-frame snapshots."""

from dataclasses import dataclass
from uuid import uuid4
import numpy as np

from .pointcloud import copy_sdk_points

NS_PER_SECOND = 1_000_000_000
UINT64_MAX = (1 << 64) - 1


def _integer(value, name, minimum=0, maximum=None):
    if isinstance(value, bool) or not isinstance(value, (int, np.integer)):
        raise TypeError(f"{name} must be an integer")
    result = int(value)
    if minimum is not None and result < minimum:
        raise ValueError(f"{name} below minimum")
    if maximum is not None and result > maximum:
        raise ValueError(f"{name} above maximum")
    return result


def normalize_time_parts(seconds: int, nanoseconds: int) -> tuple[int, int]:
    """Canonicalize explicit integer arithmetic; negative total time is rejected.

    Use timestamp_ns() at SDK boundaries to reject malformed, noncanonical
    uint64 seconds / nanoseconds fields instead of silently accepting them.
    """
    seconds = _integer(seconds, "seconds", minimum=None)
    nanoseconds = _integer(nanoseconds, "nanoseconds", minimum=None)
    total = seconds * NS_PER_SECOND + nanoseconds
    if total < 0:
        raise ValueError("negative timestamp")
    result = divmod(total, NS_PER_SECOND)
    if result[0] > UINT64_MAX:
        raise ValueError("seconds exceeds uint64")
    return result


def timestamp_ns(seconds: int, nanoseconds: int) -> int:
    seconds = _integer(seconds, "seconds", maximum=UINT64_MAX)
    nanoseconds = _integer(nanoseconds, "nanoseconds", maximum=NS_PER_SECOND - 1)
    return seconds * NS_PER_SECOND + nanoseconds


def _token(value, name):
    if not isinstance(value, str) or not value or any(c in value for c in ("/", "\\", "\0")):
        raise ValueError(f"{name} must be a nonempty path-safe identifier")
    return value


@dataclass(frozen=True)
class ClockModel:
    """Caller-supplied model; validation and error bounds require external evidence.

    common_ns = floor(raw * scale_numerator / scale_denominator) + offset_ns.
    Integer rational scaling avoids float nanosecond precision loss.
    """
    model_id: str
    device_timestamp_unit: str
    scale_numerator: int
    scale_denominator: int
    offset_ns: int
    uncertainty_ns: int | None = None
    validated: bool = False
    valid_raw_min: int = 0
    valid_raw_max: int | None = None

    def __post_init__(self):
        _token(self.model_id, "model_id")
        if not self.device_timestamp_unit or self.device_timestamp_unit.startswith("UNKNOWN"):
            raise ValueError("clock model requires a known source unit")
        object.__setattr__(self, "scale_numerator", _integer(self.scale_numerator, "scale_numerator", minimum=1))
        object.__setattr__(self, "scale_denominator", _integer(self.scale_denominator, "scale_denominator", minimum=1))
        object.__setattr__(self, "offset_ns", _integer(self.offset_ns, "offset_ns", minimum=None))
        object.__setattr__(self, "valid_raw_min", _integer(self.valid_raw_min, "valid_raw_min"))
        if self.valid_raw_max is not None:
            object.__setattr__(self, "valid_raw_max", _integer(self.valid_raw_max, "valid_raw_max", minimum=self.valid_raw_min))
        if self.uncertainty_ns is not None:
            object.__setattr__(self, "uncertainty_ns", _integer(self.uncertainty_ns, "uncertainty_ns"))
        if not isinstance(self.validated, bool):
            raise TypeError("validated must be bool")

    def convert(self, raw: int, unit: str) -> int:
        raw = _integer(raw, "raw")
        if unit != self.device_timestamp_unit:
            raise ValueError("clock model source unit mismatch")
        if raw < self.valid_raw_min or (self.valid_raw_max is not None and raw > self.valid_raw_max):
            raise ValueError("clock model outside validity interval")
        result = raw * self.scale_numerator // self.scale_denominator + self.offset_ns
        if result < 0:
            raise ValueError("clock model produced negative time")
        return result


class SourceFrameBuilder:
    """One builder per independently identified sensor stream.

    A backward device counter, host wall time, or monotonic time starts a new
    stream epoch. No counter wrap duration is invented. Reconnection must call
    restart(); the map acceptance layer independently requires explicit resume.
    """

    def __init__(self, session_id: str, side: str, sensor_id: str, frame_id: str,
                 source_config_hash: str, clock_model: ClockModel | None = None):
        self.session_id = _token(session_id, "session_id")
        if side not in ("left", "right"):
            raise ValueError("side must be left or right")
        self.side = side
        self.sensor_id = _token(sensor_id, "sensor_id")
        if not isinstance(frame_id, str) or not frame_id or frame_id.startswith("/") or any(c.isspace() or c == "\0" for c in frame_id):
            raise ValueError("frame_id must be a nonempty relative TF frame name")
        self.frame_id = frame_id
        if not isinstance(source_config_hash, str) or not source_config_hash:
            raise ValueError("source_config_hash must be explicit")
        self.source_config_hash = source_config_hash
        self.clock_model = clock_model
        self.stream_epoch = uuid4().hex
        self.frame_sequence = 0
        self._last_raw = self._last_host = self._last_monotonic = None
        self._last_unit = None
        self._epoch_reason = "stream_start"

    def restart(self, reason="reconnect"):
        if not isinstance(reason, str) or not reason:
            raise ValueError("restart reason required")
        self.stream_epoch = uuid4().hex
        self.frame_sequence = 0
        self._last_raw = self._last_host = self._last_monotonic = None
        self._last_unit = None
        self._epoch_reason = reason
        # A clock model calibrated before a device reset is no longer valid.
        self.clock_model = None

    def build(self, points, *, device_timestamp_raw: int | None,
              device_timestamp_unit: str, host_receive_ns: int,
              host_monotonic_ns: int) -> dict:
        host = _integer(host_receive_ns, "host_receive_ns")
        monotonic = _integer(host_monotonic_ns, "host_monotonic_ns")
        raw = None if device_timestamp_raw is None else _integer(device_timestamp_raw, "device_timestamp_raw")
        if not isinstance(device_timestamp_unit, str) or not device_timestamp_unit:
            raise ValueError("device_timestamp_unit required, UNKNOWN is allowed")
        owned = copy_sdk_points(points)
        flags = []
        if self._last_unit is not None and device_timestamp_unit != self._last_unit:
            flags.append("device_timestamp_unit_changed")
        if raw is not None and self._last_raw is not None and raw < self._last_raw:
            flags.append("device_time_backward_or_wrap")
        if self._last_host is not None and host < self._last_host:
            flags.append("host_clock_backward")
        if self._last_monotonic is not None and monotonic < self._last_monotonic:
            flags.append("monotonic_clock_backward")
        if flags:
            self.restart(";".join(flags))
        elif raw is not None and raw == self._last_raw:
            flags.append("duplicate_device_timestamp")

        common = host
        valid = False
        uncertainty = None
        model_id = None
        time_source = "arrival_only"
        if self.clock_model is not None and raw is not None:
            model_id = self.clock_model.model_id
            try:
                converted = self.clock_model.convert(raw, device_timestamp_unit)
                if self.clock_model.validated and self.clock_model.uncertainty_ns is not None:
                    common = converted
                    uncertainty = self.clock_model.uncertainty_ns
                    time_source = "validated_model"
                    valid = True
                else:
                    flags.append("clock_model_unvalidated_or_uncertainty_unknown")
            except ValueError:
                flags.append("clock_model_not_applicable")
        key = f"{self.sensor_id}/{self.stream_epoch}/{self.frame_sequence}"
        result = {
            "session_id": self.session_id, "side": self.side, "sensor_id": self.sensor_id,
            "stream_epoch": self.stream_epoch, "frame_sequence": self.frame_sequence,
            "points": owned, "frame_id": self.frame_id,
            "device_timestamp_raw": raw, "device_timestamp_unit": device_timestamp_unit,
            "host_receive_ns": host, "host_monotonic_ns": monotonic,
            "common_time_ns": common, "clock_model_id": model_id,
            "time_valid": valid, "uncertainty_ns": uncertainty, "time_source": time_source,
            "source_config_hash": self.source_config_hash, "raw_key": key,
            "raw_point_count": int(owned.shape[0]),
            "valid_point_count": int(np.isfinite(owned).all(axis=1).sum()),
            "diagnostic_flags": flags, "epoch_reason": self._epoch_reason,
        }
        self.frame_sequence += 1
        self._last_raw, self._last_host, self._last_monotonic = raw, host, monotonic
        self._last_unit = device_timestamp_unit
        return result
