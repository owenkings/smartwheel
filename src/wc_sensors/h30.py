"""Pure H30 byte-stream decoder, based on the supplied serial ROS 2 SDK.

Protocol: 59 53, little-endian uint16 TID, uint8 payload length, TLVs,
CK1/CK2. The SDK calls this CRC; it is a two-byte additive checksum.
No device configuration, serial opening, or command transmission is present.
"""

from dataclasses import dataclass
from math import isfinite, pi, sqrt
import struct


HEADER = b"\x59\x53"
MAX_PACKET = 262
_LENGTHS = {
    0x01: 2, 0x10: 12, 0x20: 12, 0x30: 12, 0x31: 12, 0x40: 12,
    0x41: 16, 0x50: 11, 0x51: 4, 0x52: 4, 0x68: 20, 0x70: 12, 0x80: 1,
}


def h30_checksum(data: bytes) -> bytes:
    """Return CK1 then CK2, as encoded on the wire."""
    ck1 = ck2 = 0
    for value in data:
        ck1 = (ck1 + value) & 0xFF
        ck2 = (ck2 + ck1) & 0xFF
    return bytes((ck1, ck2))


@dataclass(frozen=True)
class H30Frame:
    tid: int
    raw_packet: bytes
    raw_tlvs: tuple[tuple[int, bytes], ...]
    fields: dict
    present_ids: frozenset[int]

    def imu_dict(self, quaternion_norm_tolerance: float = 0.05) -> dict:
        """A ROS-adaptable dictionary with absent estimates explicitly unavailable.

        No identity orientation or zero-valued acceleration/gyro is synthesized.
        Zero covariance arrays mean unknown covariance, never precise estimates.
        Norm tolerance is only an input validity gate, not calibration accuracy.
        """
        if not isfinite(quaternion_norm_tolerance) or not 0 <= quaternion_norm_tolerance < 1:
            raise ValueError("quaternion_norm_tolerance must be finite in [0,1)")
        quat = self.fields.get("orientation_xyzw")
        norm = sqrt(sum(x * x for x in quat)) if quat is not None else None
        orientation_valid = quat is not None and abs(norm - 1.0) <= quaternion_norm_tolerance
        acceleration = self.fields.get("acceleration_m_s2")
        velocity = self.fields.get("angular_velocity_rad_s")

        def covariance(available):
            return [0.0] * 9 if available else [-1.0] + [0.0] * 8

        return {
            "orientation": quat if orientation_valid else None,
            "orientation_covariance": covariance(orientation_valid),
            "orientation_valid": orientation_valid,
            "orientation_norm": norm,
            "angular_velocity": velocity,
            "angular_velocity_covariance": covariance(velocity is not None),
            "linear_acceleration": acceleration,
            "linear_acceleration_covariance": covariance(acceleration is not None),
            "field_validity": {
                "orientation": orientation_valid,
                "angular_velocity": velocity is not None,
                "linear_acceleration": acceleration is not None,
            },
            "sample_timestamp_raw": self.fields.get("sample_timestamp_raw"),
            "dataready_timestamp_raw": self.fields.get("dataready_timestamp_raw"),
            "device_timestamp_unit": "UNKNOWN_us_or_100us",
            "time_source": "arrival_only",
            "time_valid": False,
            "uncertainty_ns": None,
            "tid": self.tid,
        }


def _decode_packet(packet: bytes) -> H30Frame:
    payload = packet[5:-2]
    offset = 0
    tlvs = []
    seen = set()
    fields = {}
    while offset < len(payload):
        if len(payload) - offset < 2:
            raise ValueError("truncated TLV header")
        field_id, length = payload[offset:offset + 2]
        offset += 2
        if length > len(payload) - offset:
            raise ValueError("TLV extends beyond payload")
        if field_id in seen:
            raise ValueError("duplicate TLV id")
        if field_id in _LENGTHS and length != _LENGTHS[field_id]:
            raise ValueError("known TLV has incorrect length")
        value = bytes(payload[offset:offset + length])
        offset += length
        tlvs.append((field_id, value))
        seen.add(field_id)
        if field_id == 0x01:
            raw = struct.unpack("<h", value)[0]
            fields.update(temperature_raw=raw, temperature_c=raw * 0.01)
        elif field_id in (0x10, 0x20, 0x30, 0x31, 0x40, 0x70):
            raw = struct.unpack("<iii", value)
            field_name, scale = {
                0x10: ("acceleration_m_s2", 1e-6),
                0x20: ("angular_velocity_rad_s", 1e-6 * pi / 180.0),
                0x30: ("magnetic_normalized", 1e-6),
                0x31: ("magnetic_mgauss", 1e-3),
                0x40: ("euler_pitch_roll_yaw_rad", 1e-6 * pi / 180.0),
                0x70: ("velocity_enu_m_s", 1e-3),
            }[field_id]
            fields[field_name + "_raw"] = raw
            fields[field_name] = tuple(v * scale for v in raw)
        elif field_id == 0x41:
            raw = struct.unpack("<iiii", value)
            fields["orientation_wxyz_raw"] = raw
            fields["orientation_xyzw"] = tuple(raw[i] * 1e-6 for i in (1, 2, 3, 0))
        elif field_id in (0x51, 0x52):
            key = "sample_timestamp_raw" if field_id == 0x51 else "dataready_timestamp_raw"
            fields[key] = struct.unpack("<I", value)[0]
        # UTC/nav/unknown TLVs are preserved exactly, without invented semantics.
    return H30Frame(struct.unpack_from("<H", packet, 2)[0], packet, tuple(tlvs), fields, frozenset(seen))


class H30Parser:
    """Bounded, incremental parsing with per-frame ownership and diagnostics.

    CRC failures resynchronize one byte at a time. A checksum-valid malformed
    TLV packet is discarded as a unit. An incomplete candidate waits for more
    bytes (at most 262 retained); reset() explicitly drops it at disconnect.
    Embedded sync bytes never cause speculative truncation of a valid frame.
    """

    def __init__(self):
        self._buffer = bytearray()
        self.stats = {"frames": 0, "checksum_errors": 0, "malformed_tlv": 0,
                      "discarded_bytes": 0, "resets": 0}

    @property
    def buffered_bytes(self) -> int:
        return len(self._buffer)

    def reset(self):
        self.stats["discarded_bytes"] += len(self._buffer)
        self.stats["resets"] += 1
        self._buffer.clear()

    def feed(self, data: bytes | bytearray | memoryview) -> list[H30Frame]:
        if not isinstance(data, (bytes, bytearray, memoryview)):
            raise TypeError("feed requires bytes-like input")
        incoming = memoryview(data).cast("B")
        frames = []
        # Bound the retained buffer even for an arbitrarily large input chunk.
        for start in range(0, len(incoming), MAX_PACKET):
            self._buffer.extend(incoming[start:start + MAX_PACKET])
            self._drain(frames)
        return frames

    def _discard(self, count):
        self.stats["discarded_bytes"] += count
        del self._buffer[:count]

    def _drain(self, frames):
        while self._buffer:
            start = self._buffer.find(HEADER)
            if start < 0:
                keep = 1 if self._buffer[-1] == HEADER[0] else 0
                self._discard(len(self._buffer) - keep)
                return
            if start:
                self._discard(start)
            if len(self._buffer) < 5:
                return
            packet_length = self._buffer[4] + 7
            if len(self._buffer) < packet_length:
                return
            packet = bytes(self._buffer[:packet_length])
            if h30_checksum(packet[2:-2]) != packet[-2:]:
                self.stats["checksum_errors"] += 1
                self._discard(1)
                continue
            try:
                frame = _decode_packet(packet)
            except ValueError:
                self.stats["malformed_tlv"] += 1
                self._discard(packet_length)
                continue
            del self._buffer[:packet_length]
            self.stats["frames"] += 1
            frames.append(frame)
