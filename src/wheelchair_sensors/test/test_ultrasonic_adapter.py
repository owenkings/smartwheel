import struct
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from wheelchair_sensors.ultrasonic_adapter_node import (  # noqa: E402
    UltrasonicArrayAdapter,
    UltrasonicSensor,
    append_modbus_crc,
    build_read_holding_registers,
    make_sensor_list,
    parse_read_holding_registers_response,
)


class FakeClock:
    def __init__(self):
        self.now = 0.0
        self.sleeps = []

    def monotonic(self):
        return self.now

    def sleep(self, duration):
        self.sleeps.append(duration)
        self.now += duration

    def advance(self, duration):
        self.now += duration


class FakeSerial:
    def __init__(self, clock=None, response_time_sec=0.0):
        self.last_request = b""
        self.clock = clock
        self.response_time_sec = response_time_sec
        self.request_start_times = []

    def reset_input_buffer(self):
        pass

    def write(self, request):
        self.last_request = request
        if self.clock is not None:
            self.request_start_times.append(self.clock.monotonic())

    def read(self, _size):
        if self.clock is not None:
            self.clock.advance(self.response_time_sec)
        address = self.last_request[0]
        return append_modbus_crc(bytes([address, 3, 2]) + struct.pack(">H", address * 100))


def test_modbus_read_command_vector():
    assert {
        address: build_read_holding_registers(address, register=0x0001, count=1)
        for address in (1, 2, 3, 4)
    } == {
        1: bytes.fromhex("01 03 00 01 00 01 d5 ca"),
        2: bytes.fromhex("02 03 00 01 00 01 d5 f9"),
        3: bytes.fromhex("03 03 00 01 00 01 d4 28"),
        4: bytes.fromhex("04 03 00 01 00 01 d5 9f"),
    }


def test_parse_ultrasonic_distance_register_response():
    response = append_modbus_crc(bytes([1, 3, 2]) + struct.pack(">H", 1000))
    address, values = parse_read_holding_registers_response(response, expected_address=1)

    assert address == 1
    assert values == [1000]


def test_make_sensor_list_disables_unconfigured_sensors():
    sensors = make_sensor_list([1, 2], [0, 1], enabled_count=1)

    assert [sensor.index for sensor in sensors if sensor.enabled] == [0]
    assert [sensor.address for sensor in sensors if sensor.enabled] == [1]


def test_four_sensor_poll_enforces_documented_actual_value_interval():
    clock = FakeClock()
    fake_serial = FakeSerial(clock=clock, response_time_sec=0.094)
    adapter = UltrasonicArrayAdapter(
        register=0x0001,
        sensors=[
            UltrasonicSensor(index=offset, address=offset + 1, frame_id=f"sensor_{offset}")
            for offset in range(4)
        ],
        inter_sensor_delay_sec=0.11,
        sleep_fn=clock.sleep,
        monotonic_fn=clock.monotonic,
    )
    adapter._serial = fake_serial

    assert adapter.read_ranges() == {0: 0.1, 1: 0.2, 2: 0.3, 3: 0.4}
    # A second cycle proves that the last-sensor -> next-cycle-first-sensor
    # boundary is paced too; there must be no timer-boundary loophole.
    assert adapter.read_ranges() == {0: 0.1, 1: 0.2, 2: 0.3, 3: 0.4}
    intervals = [
        right - left
        for left, right in zip(
            fake_serial.request_start_times,
            fake_serial.request_start_times[1:],
        )
    ]

    assert len(fake_serial.request_start_times) == 8
    assert intervals == pytest.approx([0.11] * 7)
    # Response time counts toward the vendor interval, so only the remainder
    # is slept instead of adding 110 ms after every ~94 ms response.
    assert sum(clock.sleeps) == pytest.approx((0.11 - 0.094) * 7)


@pytest.mark.parametrize(
    ("register", "delay"),
    [(0x0001, 0.1), (0x0002, 0.3)],
)
def test_documented_register_intervals_fail_closed(register, delay):
    with pytest.raises(ValueError, match="requires an inter-sensor delay greater"):
        UltrasonicArrayAdapter(register=register, inter_sensor_delay_sec=delay)
