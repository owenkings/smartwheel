import math
import struct
import sys
from types import SimpleNamespace
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from wheelchair_sensors.imu_adapter_node import (  # noqa: E402
    H30ImuAdapter,
    ImuAdapterNode,
    YesenseParser,
    yesense_checksum,
)


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


def test_serial_adapter_records_host_receive_time_before_publish(monkeypatch):
    payload = bytes([0x10, 12]) + struct.pack("<iii", 0, 0, 9_810_000)

    class FakeSerial:
        in_waiting = 64

        def read(self, _count):
            return make_frame(payload)

    adapter = H30ImuAdapter(_serial=FakeSerial())
    monkeypatch.setattr(
        "wheelchair_sensors.imu_adapter_node.time.time_ns",
        lambda: 1_234_567_890_123,
    )

    samples = adapter.read_samples()

    assert len(samples) == 1
    assert samples[0].host_receive_time_ns == 1_234_567_890_123
    assert samples[0].host_interpolated_time_ns == 1_234_567_890_123


def test_serial_adapter_interpolates_frames_within_one_read(monkeypatch):
    payload = bytes([0x10, 12]) + struct.pack("<iii", 0, 0, 9_810_000)

    class FakeSerial:
        in_waiting = 128

        def read(self, _count):
            return make_frame(payload) + make_frame(payload)

    adapter = H30ImuAdapter(_serial=FakeSerial(), nominal_rate_hz=200.0)
    monkeypatch.setattr(
        "wheelchair_sensors.imu_adapter_node.time.time_ns",
        lambda: 2_000_000_000_000,
    )
    monkeypatch.setattr(
        "wheelchair_sensors.imu_adapter_node.time.monotonic_ns",
        lambda: 8_000_000_000_000,
    )

    samples = adapter.read_samples()

    assert len(samples) == 2
    assert [sample.host_receive_time_ns for sample in samples] == [
        2_000_000_000_000,
        2_000_000_000_000,
    ]
    assert [sample.host_interpolated_time_ns for sample in samples] == [
        1_999_995_000_000,
        2_000_000_000_000,
    ]
    assert [sample.host_interpolated_monotonic_ns for sample in samples] == [
        7_999_995_000_000,
        8_000_000_000_000,
    ]


def test_required_device_timestamp_never_falls_back_to_host_time():
    node = object.__new__(ImuAdapterNode)
    node.use_device_timestamp = True
    node.require_device_timestamp = True
    node._clock_offset_ns = None

    assert node._stamp(SimpleNamespace(sample_timestamp_us=None)) is None
