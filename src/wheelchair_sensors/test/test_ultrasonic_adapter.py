import struct
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from wheelchair_sensors.ultrasonic_adapter_node import (  # noqa: E402
    append_modbus_crc,
    build_read_holding_registers,
    parse_read_holding_registers_response,
)


def test_modbus_read_command_vector():
    assert build_read_holding_registers(1, register=0x0001, count=1) == bytes.fromhex(
        "01 03 00 01 00 01 d5 ca"
    )


def test_parse_ultrasonic_distance_register_response():
    response = append_modbus_crc(bytes([1, 3, 2]) + struct.pack(">H", 1000))
    address, values = parse_read_holding_registers_response(response, expected_address=1)

    assert address == 1
    assert values == [1000]
