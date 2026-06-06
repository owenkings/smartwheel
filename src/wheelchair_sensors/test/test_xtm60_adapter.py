import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from wheelchair_sensors.xtm60_adapter_node import (  # noqa: E402
    XTM60AdapterNode,
    XTM60SdkAdapter,
    XTM60SdkConfig,
    extract_xyzi_points,
)


class FakePoint:
    def __init__(self, x, y, z, i=0.0):
        self.x = x
        self.y = y
        self.z = z
        self.i = i


class FakeFrame:
    hasPointcloud = True

    def __init__(self):
        self.points = [
            FakePoint(1.0, 0.0, 0.0),
            FakePoint(float("nan"), 0.0, 0.0),
            FakePoint(30.0, 0.0, 0.0),
            FakePoint(0.1, 0.2, 0.3, 8.0),
        ]
        self.amplData = [100, 200, 300, 400]


def test_extract_xyzi_points_filters_invalid_and_keeps_amplitude():
    points = extract_xyzi_points(FakeFrame(), unit_scale=1.0, range_min=0.05, range_max=20.0)

    assert len(points) == 2
    assert points[0] == (1.0, 0.0, 0.0, 100.0)
    assert points[1] == (0.1, 0.2, 0.3, 400.0)
    assert math.isfinite(points[1][2])


def test_xtm60_sdk_runtime_seconds_are_not_epoch_timestamps():
    assert not XTM60AdapterNode._is_plausible_epoch_stamp((1006, 792000000))
    assert XTM60AdapterNode._is_plausible_epoch_stamp((1779897171, 676858200))


class FakeLogger:
    def info(self, _message):
        pass

    def warning(self, _message):
        pass

    def error(self, _message):
        pass


class FakeXintanSdk:
    @staticmethod
    def ImageType(value):
        return value


class FakeSdk:
    def __init__(self, udp_ok=True, start_ok=True):
        self.udp_ok = udp_ok
        self.start_ok = start_ok
        self.calls = []

    def isconnect(self):
        return True

    def setUdpDestIp(self, ip, port):
        self.calls.append(("udp", ip, port))
        return self.udp_ok

    def start(self, image_type, is_once=False):
        self.calls.append(("start", image_type, is_once))
        return self.start_ok


class FakeEvent:
    def __init__(self, eventstr, cmdid):
        self.eventstr = eventstr
        self.cmdid = cmdid


def make_sdk_adapter(fake_sdk):
    adapter = XTM60SdkAdapter(
        XTM60SdkConfig(
            udp_dest_ip="192.168.1.100",
            udp_dest_port=7688,
            reconnect_interval_sec=0.0,
        ),
        FakeLogger(),
    )
    adapter._sdk = fake_sdk
    adapter._xintan_sdk = FakeXintanSdk
    return adapter


def test_udp_destination_is_applied_after_connect_before_measurement():
    fake_sdk = FakeSdk()
    adapter = make_sdk_adapter(fake_sdk)

    adapter.poll()

    assert fake_sdk.calls == [
        ("udp", "192.168.1.100", 7688),
        ("start", 4, False),
    ]
    assert adapter.measurement_started is True


def test_udp_or_start_failure_does_not_report_measuring():
    udp_failure = make_sdk_adapter(FakeSdk(udp_ok=False))
    udp_failure.poll()
    assert udp_failure.measurement_started is False
    assert all(call[0] != "start" for call in udp_failure._sdk.calls)

    start_failure = make_sdk_adapter(FakeSdk(start_ok=False))
    start_failure.poll()
    assert start_failure.measurement_started is False


def test_device_state_ff_does_not_trigger_tcp_reconnect():
    adapter = make_sdk_adapter(FakeSdk())
    adapter._connected = True
    adapter._measurement_started = True
    adapter._udp_dest_applied = True

    adapter._on_event(FakeEvent("devState", 0xFF))

    assert adapter.connected is True
    assert adapter.measurement_started is True
    assert adapter._udp_dest_applied is True


def test_sdk_state_ff_marks_adapter_disconnected():
    adapter = make_sdk_adapter(FakeSdk())
    adapter._connected = True
    adapter._measurement_started = True
    adapter._udp_dest_applied = True

    adapter._on_event(FakeEvent("sdkState", 0xFF))

    assert adapter.connected is False
    assert adapter.measurement_started is False
    assert adapter._udp_dest_applied is False
