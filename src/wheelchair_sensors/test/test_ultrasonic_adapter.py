import struct
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from wheelchair_sensors.ultrasonic_adapter_node import (  # noqa: E402
    append_modbus_crc,
    build_read_holding_registers,
    make_sensor_list,
    parse_read_holding_registers_response,
)


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
