import struct
import threading
from dataclasses import dataclass
from typing import Iterable, List, Optional

try:
    import serial
except ImportError:
    serial = None


def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


def with_crc(frame: bytes) -> bytes:
    return frame + struct.pack("<H", crc16(frame))


def to_u16(value: int) -> int:
    return value & 0xFFFF


def from_i16(value: int) -> int:
    return value - 0x10000 if value & 0x8000 else value


@dataclass
class ModbusSerialConfig:
    port: str = "/dev/ttyUSB2"
    baud_rate: int = 115200
    timeout_sec: float = 0.05


class ModbusRtuClient:
    def __init__(self, config: ModbusSerialConfig):
        self.config = config
        self._serial = None
        self._lock = threading.Lock()

    def open(self):
        if self._serial is not None:
            return
        if serial is None:
            raise RuntimeError("pyserial is required for Modbus RTU")
        self._serial = serial.Serial(
            self.config.port,
            self.config.baud_rate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=self.config.timeout_sec,
        )

    def close(self):
        if self._serial is not None:
            self._serial.close()
            self._serial = None

    def write_single_register(self, slave_id: int, register: int, value: int):
        request = with_crc(struct.pack(">BBHH", slave_id, 0x06, register & 0xFFFF, to_u16(value)))
        response = self._exchange(request, 8)
        self._validate_response(response, slave_id, 0x06)
        return response

    def write_multiple_registers(self, slave_id: int, start_register: int, values: Iterable[int]):
        registers = [to_u16(v) for v in values]
        payload = b"".join(struct.pack(">H", value) for value in registers)
        request = with_crc(
            struct.pack(">BBHHB", slave_id, 0x10, start_register & 0xFFFF, len(registers), len(payload))
            + payload
        )
        response = self._exchange(request, 8)
        self._validate_response(response, slave_id, 0x10)
        return response

    def read_holding_registers(self, slave_id: int, start_register: int, count: int) -> List[int]:
        request = with_crc(struct.pack(">BBHH", slave_id, 0x03, start_register & 0xFFFF, count & 0xFFFF))
        response = self._exchange(request, 5 + 2 * count)
        self._validate_response(response, slave_id, 0x03)
        byte_count = response[2]
        if byte_count != 2 * count:
            raise RuntimeError("unexpected Modbus byte count")
        return [
            struct.unpack(">H", response[3 + i : 5 + i])[0]
            for i in range(0, byte_count, 2)
        ]

    def _exchange(self, request: bytes, response_len: int) -> bytes:
        self.open()
        with self._lock:
            self._serial.reset_input_buffer()
            self._serial.write(request)
            response = self._serial.read(response_len)
        if len(response) != response_len:
            raise TimeoutError(f"Modbus timeout: expected {response_len} bytes, got {len(response)}")
        return response

    @staticmethod
    def _validate_response(response: bytes, slave_id: int, function_code: int):
        if len(response) < 5:
            raise RuntimeError("short Modbus response")
        body = response[:-2]
        expected_crc = struct.unpack("<H", response[-2:])[0]
        actual_crc = crc16(body)
        if expected_crc != actual_crc:
            raise RuntimeError(f"bad Modbus CRC: got 0x{expected_crc:04x}, expected 0x{actual_crc:04x}")
        if response[0] != slave_id:
            raise RuntimeError(f"unexpected slave id {response[0]}, expected {slave_id}")
        if response[1] & 0x80:
            raise RuntimeError(f"Modbus exception response 0x{response[2]:02x}")
        if response[1] != function_code:
            raise RuntimeError(f"unexpected function 0x{response[1]:02x}")
