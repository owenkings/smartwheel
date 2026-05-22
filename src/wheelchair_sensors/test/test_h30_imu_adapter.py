import math
import struct
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from wheelchair_sensors.imu_adapter_node import YesenseParser, yesense_checksum  # noqa: E402


def make_frame(payload: bytes) -> bytes:
    head = b"\x59\x53" + struct.pack("<HB", 1, len(payload))
    check_a, check_b = yesense_checksum(head[2:] + payload)
    return head + payload + bytes([check_a, check_b])


def test_yesense_parser_decodes_accel_gyro_and_quaternion():
    payload = (
        bytes([0x10, 12])
        + struct.pack("<iii", 1_000_000, 0, 9_810_000)
        + bytes([0x20, 12])
        + struct.pack("<iii", 0, 0, 90_000_000)
        + bytes([0x41, 16])
        + struct.pack("<iiii", 1_000_000, 0, 0, 0)
    )
    samples = YesenseParser().feed(make_frame(payload))

    assert len(samples) == 1
    sample = samples[0]
    assert sample.accel_mps2 == pytest.approx((1.0, 0.0, 9.81))
    assert sample.gyro_rps == pytest.approx((0.0, 0.0, math.pi / 2.0))
    assert sample.quat_xyzw == pytest.approx((0.0, 0.0, 0.0, 1.0))


def test_yesense_parser_recovers_after_bad_checksum_frame():
    payload = bytes([0x10, 12]) + struct.pack("<iii", 0, 0, 9_810_000)
    bad = bytearray(make_frame(payload))
    bad[-1] ^= 0xFF
    parser = YesenseParser()

    samples = parser.feed(bytes(bad) + make_frame(payload))

    assert len(samples) == 1
    assert samples[0].accel_mps2 == pytest.approx((0.0, 0.0, 9.81))
