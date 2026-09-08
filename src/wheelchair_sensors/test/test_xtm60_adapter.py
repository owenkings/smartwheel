import json
import math
import sys
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from wheelchair_sensors.xtm60_adapter_node import (  # noqa: E402
    CloudFrameQualityGate,
    XTM60AdapterNode,
    XTM60SdkAdapter,
    XTM60SdkConfig,
    build_timing_diagnostic,
    extract_xyzi_grid,
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


class InvalidAmplitudeFrame:
    hasPointcloud = True
    width = 3
    height = 1

    def __init__(self):
        self.points = [
            FakePoint(1.0, 0.0, 0.0),
            FakePoint(1.5, 0.0, 0.0),
            FakePoint(2.0, 0.0, 0.0),
        ]
        self.amplData = [64000, 64001, 2039]


def test_extractors_reject_vendor_invalid_amplitude_sentinel():
    frame = InvalidAmplitudeFrame()

    points = extract_xyzi_points(frame, range_min=0.0, range_max=50.0)
    grid, width, height = extract_xyzi_grid(frame, range_min=0.0, range_max=50.0)

    assert points == [(2.0, 0.0, 0.0, 2039.0)]
    assert (width, height) == (3, 1)
    assert math.isnan(grid[0][0])
    assert grid[0][3] == 0.0
    assert math.isnan(grid[1][0])
    assert grid[1][3] == 0.0
    assert grid[2] == (2.0, 0.0, 0.0, 2039.0)


def test_organized_extractor_can_return_quality_ranges_without_second_pass():
    frame = InvalidAmplitudeFrame()

    grid, width, height, ranges = extract_xyzi_grid(
        frame,
        range_min=0.0,
        range_max=50.0,
        return_ranges=True,
    )

    assert (width, height) == (3, 1)
    assert len(grid) == 3
    assert math.isnan(ranges[0])
    assert math.isnan(ranges[1])
    assert math.isclose(ranges[2], 2.0)


def test_xtm60_sdk_runtime_seconds_are_not_epoch_timestamps():
    assert not XTM60AdapterNode._is_plausible_epoch_stamp((1006, 792000000))
    assert XTM60AdapterNode._is_plausible_epoch_stamp((1779897171, 676858200))


def test_xtm60_sdk_epoch_selection_copies_only_verified_epoch_values():
    assert XTM60AdapterNode._select_cloud_stamp(
        "sdk_epoch", sdk_stamp=(1_779_897_171, 676_858_200), receive_wall_time_ns=1
    ) == (1_779_897_171, 676_858_200)
    assert XTM60AdapterNode._select_cloud_stamp(
        "sdk_epoch", sdk_stamp=(1006, 792_000_000), receive_wall_time_ns=1_700_000_000_000
    ) is None


def test_xtm60_timestamp_selection_never_falls_back_from_requested_source():
    assert XTM60AdapterNode._select_cloud_stamp(
        "host_receive", sdk_stamp=(1_779_897_171, 0), receive_wall_time_ns=None
    ) is None
    assert XTM60AdapterNode._select_cloud_stamp(
        "unknown", sdk_stamp=(1_779_897_171, 0), receive_wall_time_ns=1_700_000_000_000
    ) is None
    assert XTM60AdapterNode._select_cloud_stamp(
        "host_receive", sdk_stamp=None, receive_wall_time_ns=1_700_000_000_123_456_789
    ) == (1_700_000_000, 123_456_789)


def test_timing_diagnostic_preserves_sdk_host_and_ros_domains():
    payload = json.loads(
        build_timing_diagnostic(
            sdk_stamp=(1234, 5678),
            host_receive_wall_time_ns=1_700_000_000_123_456_789,
            cloud_header_stamp=(1_700_000_000, 123_456_789),
            publish_wall_time_ns=1_700_000_000_123_999_999,
            timestamp_source="host_receive",
        )
    )

    assert payload["sdk_timestamp"] == {"sec": 1234, "nanosec": 5678}
    assert payload["host_receive_wall_time_ns"] == 1_700_000_000_123_456_789
    assert payload["cloud_header_stamp"] == {
        "sec": 1_700_000_000,
        "nanosec": 123_456_789,
    }
    assert payload["publish_wall_time_ns"] == 1_700_000_000_123_999_999
    assert payload["timestamp_source"] == "host_receive"


def test_timing_diagnostic_allows_missing_sdk_stamp():
    payload = json.loads(
        build_timing_diagnostic(
            sdk_stamp=None,
            host_receive_wall_time_ns=None,
            cloud_header_stamp=None,
            publish_wall_time_ns=None,
            timestamp_source="host_receive",
        )
    )

    assert payload["sdk_timestamp"] is None
    assert payload["host_receive_wall_time_ns"] is None
    assert payload["cloud_header_stamp"] is None
    assert payload["publish_wall_time_ns"] is None


def _quality_grid(distance, valid=4, total=4):
    points = [(float(distance), 0.0, 0.0, 100.0)] * int(valid)
    points.extend([(math.nan, math.nan, math.nan, 0.0)] * (int(total) - int(valid)))
    return points


def test_cloud_quality_gate_accepts_stable_organized_frames():
    gate = CloudFrameQualityGate(
        min_valid_fraction=0.50,
        max_median_range_delta=0.10,
        max_p95_range_delta=0.75,
    )

    first = gate.evaluate(_quality_grid(2.0), 10.0, organized=True)
    second = gate.evaluate(_quality_grid(2.02), 10.1, organized=True)

    assert first.accepted is True
    assert second.accepted is True
    assert math.isclose(second.valid_fraction, 1.0)
    assert math.isclose(second.median_range_delta, 0.02, abs_tol=1e-9)
    assert second.dropped_frames == 0


def test_cloud_quality_gate_rejects_temporal_jump_but_raw_can_continue():
    gate = CloudFrameQualityGate(
        min_valid_fraction=0.50,
        max_median_range_delta=0.10,
        max_p95_range_delta=0.75,
    )

    gate.evaluate(_quality_grid(2.0), 10.0, organized=True)
    bad = gate.evaluate(_quality_grid(3.0), 10.1, organized=True)
    recovered = gate.evaluate(_quality_grid(3.01), 10.2, organized=True)

    assert bad.accepted is False
    assert "temporal_range_jump" in bad.reason
    assert bad.dropped_frames == 1
    assert recovered.accepted is True
    assert recovered.accepted_frames == 2


def test_cloud_quality_gate_can_report_motion_jump_without_dropping_frame():
    gate = CloudFrameQualityGate(
        min_valid_fraction=0.50,
        max_median_range_delta=0.10,
        max_p95_range_delta=0.75,
        temporal_hard_reject=False,
    )

    gate.evaluate(_quality_grid(2.0), 10.0, organized=True)
    moving = gate.evaluate(_quality_grid(3.0), 10.1, organized=True)

    assert moving.accepted is True
    assert moving.reason == "ok"
    assert moving.temporal_jump_detected is True


def test_cloud_quality_report_only_mode_keeps_relative_drop_as_diagnostic():
    gate = CloudFrameQualityGate(
        min_valid_fraction=0.50,
        min_relative_valid_fraction=0.75,
        max_median_range_delta=0.10,
        max_p95_range_delta=0.75,
        temporal_hard_reject=False,
    )

    gate.evaluate(_quality_grid(2.0, valid=10, total=10), 10.0, organized=True)
    moving = gate.evaluate(
        _quality_grid(3.0, valid=6, total=10), 10.1, organized=True
    )

    assert moving.accepted is True
    assert moving.temporal_jump_detected is True
    assert moving.relative_validity_drop_detected is True


def test_disabled_cloud_quality_gate_has_defined_diagnostic_flags():
    gate = CloudFrameQualityGate(enabled=False)

    result = gate.evaluate(_quality_grid(2.0), 10.0, organized=True)

    assert result.accepted is True
    assert result.temporal_jump_detected is False
    assert result.relative_validity_drop_detected is False


def test_cloud_quality_gate_rejects_low_validity_and_nonmonotonic_time():
    gate = CloudFrameQualityGate(min_valid_fraction=0.50)

    gate.evaluate(_quality_grid(2.0), 10.0, organized=True)
    low = gate.evaluate(_quality_grid(2.0, valid=1), 10.1, organized=True)
    reversed_time = gate.evaluate(_quality_grid(2.0), 10.05, organized=True)

    assert low.accepted is False
    assert "absolute_valid_fraction" in low.reason
    assert reversed_time.accepted is False
    assert "nonmonotonic_receive_time" in reversed_time.reason


def test_cloud_quality_gate_resets_temporal_check_after_long_gap():
    gate = CloudFrameQualityGate(
        reset_after_gap_sec=0.50,
        max_median_range_delta=0.10,
    )

    gate.evaluate(_quality_grid(2.0), 10.0, organized=True)
    after_gap = gate.evaluate(_quality_grid(4.0), 10.6, organized=True)

    assert after_gap.accepted is True
    assert after_gap.median_range_delta is None


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

    def stop(self):
        self.calls.append(("stop",))
        return True


class FakeSdkWithConfig(FakeSdk):
    def __init__(self, serial="LEFT", int_time_3=30):
        super().__init__()
        self.serial = serial
        self.int_time_3 = int_time_3

    def getDevInfo(self):
        self.calls.append(("get_info",))
        return True, SimpleNamespace(sn=self.serial)

    def getDevConfig(self):
        self.calls.append(("get_config",))
        return True, SimpleNamespace(
            integrationTimeGs=2000,
            integrationTimes=[1600, 200, self.int_time_3, 1600],
            hdrMode=1,
            miniAmp=70,
            maxfps=10,
        )


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


def test_read_only_device_config_guard_allows_verified_unit():
    fake_sdk = FakeSdkWithConfig(serial="LEFT", int_time_3=30)
    adapter = XTM60SdkAdapter(
        XTM60SdkConfig(
            verify_device_config=True,
            require_device_config_match=True,
            expected_serial="LEFT",
            reconnect_interval_sec=0.0,
        ),
        FakeLogger(),
    )
    adapter._sdk = fake_sdk
    adapter._xintan_sdk = FakeXintanSdk

    adapter.poll()

    assert fake_sdk.calls == [
        ("get_info",),
        ("get_config",),
        ("start", 4, False),
    ]
    assert adapter.measurement_started is True


def test_read_only_device_config_guard_fails_closed_on_exposure_drift():
    fake_sdk = FakeSdkWithConfig(serial="LEFT", int_time_3=20)
    adapter = XTM60SdkAdapter(
        XTM60SdkConfig(
            verify_device_config=True,
            require_device_config_match=True,
            expected_serial="LEFT",
            int_time_3=30,
            reconnect_interval_sec=0.0,
        ),
        FakeLogger(),
    )
    adapter._sdk = fake_sdk
    adapter._xintan_sdk = FakeXintanSdk

    adapter.poll()

    assert ("start", 4, False) not in fake_sdk.calls
    assert adapter.measurement_started is False
    assert "integrationTimes" in adapter.last_error


def test_read_only_device_config_guard_fails_closed_on_serial_mismatch():
    fake_sdk = FakeSdkWithConfig(serial="WRONG", int_time_3=30)
    adapter = XTM60SdkAdapter(
        XTM60SdkConfig(
            verify_device_config=True,
            require_device_config_match=True,
            expected_serial="LEFT",
            reconnect_interval_sec=0.0,
        ),
        FakeLogger(),
    )
    adapter._sdk = fake_sdk
    adapter._xintan_sdk = FakeXintanSdk

    adapter.poll()

    assert ("start", 4, False) not in fake_sdk.calls
    assert adapter.measurement_started is False
    assert "serial=" in adapter.last_error


def test_phase_target_advances_to_the_next_leader_period():
    target = XTM60SdkAdapter._phase_target_after_reference(
        reference=10.0,
        now=10.16,
        period=0.1,
        offset=0.05,
    )

    assert math.isclose(target, 10.25, abs_tol=1e-9)


def test_phase_grid_target_uses_configured_offset():
    left_target = XTM60SdkAdapter._phase_target_on_host_grid(
        now=10.021,
        period=0.1,
        offset=0.0,
    )
    right_target = XTM60SdkAdapter._phase_target_on_host_grid(
        now=10.021,
        period=0.1,
        offset=0.05,
    )

    assert math.isclose(left_target, 10.1, abs_tol=1e-9)
    assert math.isclose(right_target, 10.05, abs_tol=1e-9)


def test_phase_follower_waits_for_reference_then_starts():
    fake_sdk = FakeSdk()
    adapter = XTM60SdkAdapter(
        XTM60SdkConfig(
            udp_dest_ip="192.168.0.100",
            udp_dest_port=7687,
            reconnect_interval_sec=0.0,
            wait_for_phase_reference=True,
            phase_period_sec=0.1,
            phase_offset_sec=0.05,
        ),
        FakeLogger(),
    )
    adapter._sdk = fake_sdk
    adapter._xintan_sdk = FakeXintanSdk
    adapter._now = lambda: 10.02

    adapter.poll()
    assert fake_sdk.calls == [("udp", "192.168.0.100", 7687)]
    assert adapter.measurement_started is False

    assert adapter.schedule_measurement_start_from_reference(10.0)
    assert math.isclose(
        adapter._scheduled_measurement_start_time, 10.05, abs_tol=1e-9
    )
    adapter._now = lambda: 10.05
    adapter.poll()

    assert fake_sdk.calls[-1] == ("start", 4, False)
    assert adapter.measurement_started is True


def test_phase_follower_realign_stops_and_waits_for_fresh_reference():
    fake_sdk = FakeSdk()
    adapter = XTM60SdkAdapter(
        XTM60SdkConfig(
            wait_for_phase_reference=True,
            phase_realign_interval_sec=20.0,
        ),
        FakeLogger(),
    )
    adapter._sdk = fake_sdk
    adapter._xintan_sdk = FakeXintanSdk
    adapter._connected = True
    adapter._measurement_started = True
    adapter._last_measurement_start_time = 1.0
    adapter._now = lambda: 21.1

    adapter.poll()

    assert fake_sdk.calls == [("stop",)]
    assert adapter.measurement_started is False
    assert adapter._scheduled_measurement_start_time is None


def test_phase_grid_realign_stops_before_next_grid_start():
    fake_sdk = FakeSdk()
    adapter = XTM60SdkAdapter(
        XTM60SdkConfig(
            start_on_phase_grid=True,
            phase_realign_interval_sec=30.0,
        ),
        FakeLogger(),
    )
    adapter._sdk = fake_sdk
    adapter._xintan_sdk = FakeXintanSdk
    adapter._connected = True
    adapter._measurement_started = True
    adapter._last_measurement_start_time = 1.0
    adapter._now = lambda: 31.1

    adapter.poll()

    assert fake_sdk.calls == [("stop",)]
    assert adapter.measurement_started is False


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


def test_late_connected_event_does_not_restart_an_active_measurement():
    adapter = make_sdk_adapter(FakeSdk())
    adapter._connected = True
    adapter._measurement_started = True
    adapter._udp_dest_applied = True

    adapter._on_event(FakeEvent("sdkState", 0xFE))

    assert adapter.connected is True
    assert adapter.measurement_started is True
    assert adapter._udp_dest_applied is True
