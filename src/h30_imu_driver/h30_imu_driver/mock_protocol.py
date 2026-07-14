import struct
from dataclasses import dataclass


MAGIC = b"H30MOCK\0"
FORMAT = "<d6f"


@dataclass(frozen=True)
class MockImuSample:
    timestamp_sec: float
    ax: float
    ay: float
    az: float
    gx: float
    gy: float
    gz: float


def encode_mock_packet(sample: MockImuSample) -> bytes:
    return MAGIC + struct.pack(
        FORMAT,
        sample.timestamp_sec,
        sample.ax,
        sample.ay,
        sample.az,
        sample.gx,
        sample.gy,
        sample.gz,
    )


def parse_mock_packet(packet: bytes) -> MockImuSample:
    expected = len(MAGIC) + struct.calcsize(FORMAT)
    if len(packet) != expected or not packet.startswith(MAGIC):
        raise ValueError("invalid synthetic H30 mock packet")
    return MockImuSample(*struct.unpack(FORMAT, packet[len(MAGIC) :]))

