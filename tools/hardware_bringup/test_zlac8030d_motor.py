#!/usr/bin/env python3
"""ZLAC8030D read-only and guarded motion test helper.

Default mode is read-only. Any command that can move a wheel requires
``--motion-sequence`` and ``--i-confirm-wheels-off-ground``.
"""

from __future__ import annotations

import argparse
import struct
import time
from dataclasses import dataclass

import serial


SLAVE_ID = 1

REG_CONTROL_MODE = 0x200D
REG_CONTROL_WORD = 0x200E
REG_ASYNC_MODE = 0x200F
REG_TARGET_SPEED_LEFT = 0x2088
REG_TARGET_SPEED_RIGHT = 0x2089
REG_ACTUAL_SPEED_LEFT = 0x20AB
REG_ACTUAL_SPEED_RIGHT = 0x20AC
REG_ACTUAL_TORQUE_LEFT = 0x20AD
REG_ACTUAL_TORQUE_RIGHT = 0x20AE

CONTROL_EMERGENCY_STOP = 0x05
CONTROL_CLEAR_FAULT = 0x06
CONTROL_STOP = 0x07
CONTROL_ENABLE = 0x08

MODE_VELOCITY = 0x03


def parse_register_list(text: str) -> list[int]:
    return [int(item.strip(), 0) for item in text.split(",") if item.strip()]


def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 1:
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
class Snapshot:
    control_mode: int
    control_word: int
    async_mode: int
    left_rpm: int
    right_rpm: int
    left_torque: int
    right_torque: int


class Modbus:
    def __init__(self, port: str, baud: int, timeout: float):
        self.handle = serial.Serial(port, baud, timeout=timeout, write_timeout=timeout)

    def close(self) -> None:
        self.handle.close()

    def read_holding(self, slave: int, register: int, count: int) -> list[int]:
        request = with_crc(struct.pack(">BBHH", slave, 0x03, register, count))
        self.handle.reset_input_buffer()
        self.handle.write(request)
        self.handle.flush()
        response = self.handle.read(5 + 2 * count)
        self._validate(response, slave, 0x03)
        if response[2] != 2 * count:
            raise RuntimeError(f"unexpected byte count {response[2]}")
        return [
            struct.unpack(">H", response[3 + offset : 5 + offset])[0]
            for offset in range(0, response[2], 2)
        ]

    def write_single(self, slave: int, register: int, value: int) -> None:
        request = with_crc(struct.pack(">BBHH", slave, 0x06, register, to_u16(value)))
        self.handle.reset_input_buffer()
        self.handle.write(request)
        self.handle.flush()
        response = self.handle.read(8)
        self._validate(response, slave, 0x06)
        if response != request:
            raise RuntimeError(f"unexpected write response {response.hex()}")

    def write_multiple(self, slave: int, register: int, values: list[int]) -> None:
        payload = b"".join(struct.pack(">H", to_u16(value)) for value in values)
        request = with_crc(
            struct.pack(">BBHHB", slave, 0x10, register, len(values), len(payload)) + payload
        )
        self.handle.reset_input_buffer()
        self.handle.write(request)
        self.handle.flush()
        response = self.handle.read(8)
        self._validate(response, slave, 0x10)

    @staticmethod
    def _validate(response: bytes, slave: int, function_code: int) -> None:
        if len(response) < 5:
            raise RuntimeError(f"short/no response: {response.hex()}")
        expected = struct.unpack("<H", response[-2:])[0]
        actual = crc16(response[:-2])
        if expected != actual:
            raise RuntimeError(f"bad crc response={response.hex()}")
        if response[0] != slave:
            raise RuntimeError(f"unexpected slave id {response[0]}")
        if response[1] & 0x80:
            raise RuntimeError(f"modbus exception 0x{response[2]:02x}")
        if response[1] != function_code:
            raise RuntimeError(f"unexpected function 0x{response[1]:02x}")


def read_snapshot(bus: Modbus) -> Snapshot:
    mode, control = bus.read_holding(SLAVE_ID, REG_CONTROL_MODE, 2)
    async_mode = bus.read_holding(SLAVE_ID, REG_ASYNC_MODE, 1)[0]
    left_rpm, right_rpm = bus.read_holding(SLAVE_ID, REG_ACTUAL_SPEED_LEFT, 2)
    left_torque, right_torque = bus.read_holding(SLAVE_ID, REG_ACTUAL_TORQUE_LEFT, 2)
    return Snapshot(
        control_mode=mode,
        control_word=control,
        async_mode=async_mode,
        left_rpm=from_i16(left_rpm),
        right_rpm=from_i16(right_rpm),
        left_torque=from_i16(left_torque),
        right_torque=from_i16(right_torque),
    )


def print_snapshot(label: str, snapshot: Snapshot) -> None:
    print(
        f"{label}: mode={snapshot.control_mode} control={snapshot.control_word} "
        f"async={snapshot.async_mode} left_rpm={snapshot.left_rpm} "
        f"right_rpm={snapshot.right_rpm} left_torque={snapshot.left_torque} "
        f"right_torque={snapshot.right_torque}"
    )


def apply_direction(left_rpm: int, right_rpm: int, invert_left: bool, invert_right: bool) -> tuple[int, int]:
    return (
        -left_rpm if invert_left else left_rpm,
        -right_rpm if invert_right else right_rpm,
    )


def dump_registers(bus: Modbus, registers: list[int]) -> None:
    for register in registers:
        values = bus.read_holding(SLAVE_ID, register, 1)
        value = values[0]
        print(f"reg=0x{register:04X} u16={value} i16={from_i16(value)}")


def stop(bus: Modbus) -> None:
    bus.write_multiple(SLAVE_ID, REG_TARGET_SPEED_LEFT, [0, 0])
    bus.write_single(SLAVE_ID, REG_CONTROL_WORD, CONTROL_STOP)


def emergency_stop(bus: Modbus) -> None:
    bus.write_multiple(SLAVE_ID, REG_TARGET_SPEED_LEFT, [0, 0])
    bus.write_single(SLAVE_ID, REG_CONTROL_WORD, CONTROL_EMERGENCY_STOP)


def motion_sequence(
    bus: Modbus,
    rpm: int,
    duration: float,
    invert_left: bool,
    invert_right: bool,
) -> None:
    try:
        stop(bus)
        bus.write_single(SLAVE_ID, REG_CONTROL_MODE, MODE_VELOCITY)
        bus.write_single(SLAVE_ID, REG_ASYNC_MODE, 0)
        bus.write_single(SLAVE_ID, REG_CONTROL_WORD, CONTROL_CLEAR_FAULT)
        time.sleep(0.2)
        bus.write_single(SLAVE_ID, REG_CONTROL_WORD, CONTROL_ENABLE)
        time.sleep(0.2)

        print_snapshot("after_enable", read_snapshot(bus))

        left_target, right_target = apply_direction(rpm, rpm, invert_left, invert_right)
        print(f"forward: target_left={left_target} target_right={right_target} rpm")
        bus.write_multiple(SLAVE_ID, REG_TARGET_SPEED_LEFT, [left_target, right_target])
        time.sleep(duration)
        print_snapshot("forward_sample", read_snapshot(bus))

        stop(bus)
        time.sleep(0.5)
        print_snapshot("after_forward_stop", read_snapshot(bus))

        left_target, right_target = apply_direction(-rpm, -rpm, invert_left, invert_right)
        print(f"reverse: target_left={left_target} target_right={right_target} rpm")
        bus.write_multiple(SLAVE_ID, REG_TARGET_SPEED_LEFT, [left_target, right_target])
        time.sleep(duration)
        print_snapshot("reverse_sample", read_snapshot(bus))

        emergency_stop(bus)
        time.sleep(0.5)
        print_snapshot("after_emergency_stop", read_snapshot(bus))
    finally:
        emergency_stop(bus)


def forward_only(
    bus: Modbus,
    rpm: int,
    duration: float,
    invert_left: bool,
    invert_right: bool,
) -> None:
    try:
        stop(bus)
        bus.write_single(SLAVE_ID, REG_CONTROL_MODE, MODE_VELOCITY)
        bus.write_single(SLAVE_ID, REG_ASYNC_MODE, 0)
        bus.write_single(SLAVE_ID, REG_CONTROL_WORD, CONTROL_CLEAR_FAULT)
        time.sleep(0.2)
        bus.write_single(SLAVE_ID, REG_CONTROL_WORD, CONTROL_ENABLE)
        time.sleep(0.2)

        print_snapshot("after_enable", read_snapshot(bus))

        left_target, right_target = apply_direction(rpm, rpm, invert_left, invert_right)
        print(f"forward: target_left={left_target} target_right={right_target} rpm")
        bus.write_multiple(SLAVE_ID, REG_TARGET_SPEED_LEFT, [left_target, right_target])
        time.sleep(duration)
        print_snapshot("forward_sample", read_snapshot(bus))
    finally:
        emergency_stop(bus)
        time.sleep(0.2)
        print_snapshot("after_emergency_stop", read_snapshot(bus))


def single_wheel(
    bus: Modbus,
    side: str,
    rpm: int,
    duration: float,
    invert_left: bool,
    invert_right: bool,
) -> None:
    left_target = rpm if side == "left" else 0
    right_target = rpm if side == "right" else 0
    left_target, right_target = apply_direction(left_target, right_target, invert_left, invert_right)
    try:
        stop(bus)
        bus.write_single(SLAVE_ID, REG_CONTROL_MODE, MODE_VELOCITY)
        bus.write_single(SLAVE_ID, REG_ASYNC_MODE, 0)
        bus.write_single(SLAVE_ID, REG_CONTROL_WORD, CONTROL_CLEAR_FAULT)
        time.sleep(0.2)
        bus.write_single(SLAVE_ID, REG_CONTROL_WORD, CONTROL_ENABLE)
        time.sleep(0.2)

        print_snapshot("after_enable", read_snapshot(bus))

        print(f"{side}_only: target_left={left_target} target_right={right_target} rpm")
        bus.write_multiple(SLAVE_ID, REG_TARGET_SPEED_LEFT, [left_target, right_target])
        time.sleep(duration)
        print_snapshot(f"{side}_sample", read_snapshot(bus))
    finally:
        emergency_stop(bus)
        time.sleep(0.2)
        print_snapshot("after_emergency_stop", read_snapshot(bus))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--timeout", type=float, default=0.25)
    parser.add_argument("--samples", type=int, default=5)
    parser.add_argument("--interval", type=float, default=0.3)
    parser.add_argument("--software-stop", action="store_true")
    parser.add_argument("--emergency-stop-only", action="store_true")
    parser.add_argument("--dump-registers")
    parser.add_argument("--forward-only", action="store_true")
    parser.add_argument("--left-only", action="store_true")
    parser.add_argument("--right-only", action="store_true")
    parser.add_argument("--motion-sequence", action="store_true")
    parser.add_argument("--rpm", type=int, default=10)
    parser.add_argument("--duration", type=float, default=1.0)
    parser.add_argument("--invert-left", action="store_true")
    parser.add_argument("--invert-right", action="store_true")
    parser.add_argument("--i-confirm-wheels-off-ground", action="store_true")
    args = parser.parse_args()

    motion_requested = args.motion_sequence or args.forward_only or args.left_only or args.right_only
    if motion_requested and not args.i_confirm_wheels_off_ground:
        raise SystemExit(
            "Refusing motion test without --i-confirm-wheels-off-ground. "
            "Lift drive wheels, clear the area, and verify hardware E-stop first."
        )
    if args.left_only and args.right_only:
        raise SystemExit("Choose only one of --left-only or --right-only.")

    bus = Modbus(args.port, args.baud, args.timeout)
    try:
        if args.dump_registers:
            dump_registers(bus, parse_register_list(args.dump_registers))
        elif args.software_stop:
            stop(bus)
            time.sleep(0.2)
            print_snapshot("after_software_stop", read_snapshot(bus))
        elif args.emergency_stop_only:
            emergency_stop(bus)
            time.sleep(0.2)
            print_snapshot("after_emergency_stop", read_snapshot(bus))
        elif args.left_only:
            single_wheel(bus, "left", args.rpm, args.duration, args.invert_left, args.invert_right)
        elif args.right_only:
            single_wheel(bus, "right", args.rpm, args.duration, args.invert_left, args.invert_right)
        elif args.forward_only:
            forward_only(bus, args.rpm, args.duration, args.invert_left, args.invert_right)
        elif args.motion_sequence:
            motion_sequence(bus, args.rpm, args.duration, args.invert_left, args.invert_right)
        else:
            for index in range(args.samples):
                print_snapshot(f"sample_{index + 1}", read_snapshot(bus))
                time.sleep(args.interval)
    finally:
        bus.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
