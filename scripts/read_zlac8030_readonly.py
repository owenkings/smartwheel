#!/usr/bin/env python3
"""Read-only Modbus RTU probe for ZLAC8030/KeepLINK motor drivers.

The script only sends function 0x03 read-holding-register requests. It does
not write enable, control, speed, or reset registers.
"""

import argparse
import json
import struct
import time
from datetime import datetime
from typing import Iterable

import serial


COMMON_REGISTERS = [
    0x0000,
    0x0001,
    0x0002,
    0x0003,
    0x2000,
    0x2001,
    0x2002,
    0x2003,
    0x200D,
    0x200E,
    0x2010,
    0x2011,
    0x2031,
    0x2032,
    0x2033,
    0x2034,
    0x2080,
    0x2081,
    0x2082,
    0x2083,
    0x2088,
    0x2089,
    0x20A0,
    0x20A1,
    0x20AB,
    0x20AC,
    0x20AD,
    0x20AE,
]


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


def as_i16(value: int) -> int:
    return value - 0x10000 if value & 0x8000 else value


def parse_int_list(text: str) -> list[int]:
    return [int(item.strip(), 0) for item in text.split(",") if item.strip()]


def parse_baud_list(text: str) -> list[int]:
    return [int(item.strip()) for item in text.split(",") if item.strip()]


def validate_crc(frame: bytes) -> bool:
    if len(frame) < 5:
        return False
    expected = struct.unpack("<H", frame[-2:])[0]
    return expected == crc16(frame[:-2])


def read_one(handle: serial.Serial, slave_id: int, register: int, count: int, pause: float) -> dict:
    request = with_crc(struct.pack(">BBHH", slave_id, 0x03, register & 0xFFFF, count & 0xFFFF))
    handle.reset_input_buffer()
    handle.write(request)
    handle.flush()
    time.sleep(pause)
    response = handle.read(5 + 2 * count)
    record = {
        "time": datetime.now().isoformat(timespec="milliseconds"),
        "slave_id": slave_id,
        "register": register,
        "register_hex": f"0x{register:04x}",
        "count": count,
        "request_hex": request.hex(),
        "response_hex": response.hex(),
        "ok": False,
    }
    if not response:
        record["error"] = "timeout"
        return record
    if len(response) < 5:
        record["error"] = f"short response: {len(response)} bytes"
        return record
    if not validate_crc(response):
        record["error"] = "bad crc"
        return record
    if response[0] != slave_id:
        record["error"] = f"unexpected slave id {response[0]}"
        return record
    if response[1] == 0x83:
        record["error"] = f"modbus exception 0x{response[2]:02x}"
        record["exception"] = response[2]
        return record
    if response[1] != 0x03 or response[2] != 2 * count:
        record["error"] = "unexpected response header"
        return record
    values = [
        struct.unpack(">H", response[3 + offset : 5 + offset])[0]
        for offset in range(0, response[2], 2)
    ]
    record.update(
        {
            "ok": True,
            "values_u16": values,
            "values_i16": [as_i16(value) for value in values],
        }
    )
    return record


def probe(
    port: str,
    baud_rates: Iterable[int],
    slave_ids: Iterable[int],
    registers: Iterable[int],
    count: int,
    timeout: float,
    pause: float,
) -> list[dict]:
    records = []
    for baud in baud_rates:
        try:
            with serial.Serial(port, baud, timeout=timeout, write_timeout=timeout) as handle:
                for slave_id in slave_ids:
                    for register in registers:
                        record = read_one(handle, slave_id, register, count, pause)
                        record["port"] = port
                        record["baud"] = baud
                        records.append(record)
        except Exception as exc:
            records.append(
                {
                    "time": datetime.now().isoformat(timespec="milliseconds"),
                    "port": port,
                    "baud": baud,
                    "ok": False,
                    "error": f"open/probe failed: {exc}",
                }
            )
    return records


def main():
    parser = argparse.ArgumentParser(description="Read-only ZLAC8030 Modbus RTU probe.")
    parser.add_argument("--port", default="/dev/ttyUSB0")
    parser.add_argument("--baud", default="115200", help="Comma-separated baud rates.")
    parser.add_argument("--slave-ids", default="1,2", help="Comma-separated Modbus slave IDs.")
    parser.add_argument(
        "--registers",
        default=",".join(f"0x{item:04x}" for item in COMMON_REGISTERS),
        help="Comma-separated holding registers to read.",
    )
    parser.add_argument("--count", type=int, default=1)
    parser.add_argument("--timeout", type=float, default=0.12)
    parser.add_argument("--pause", type=float, default=0.015)
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args()

    records = probe(
        args.port,
        parse_baud_list(args.baud),
        parse_int_list(args.slave_ids),
        parse_int_list(args.registers),
        args.count,
        args.timeout,
        args.pause,
    )
    successes = [record for record in records if record.get("ok")]
    if args.json:
        print(json.dumps({"records": records, "successes": successes}, ensure_ascii=False, indent=2))
    else:
        for record in records:
            if record.get("ok"):
                print(
                    f"OK port={record['port']} baud={record['baud']} "
                    f"slave={record['slave_id']} reg={record['register_hex']} "
                    f"u16={record['values_u16']} i16={record['values_i16']} raw={record['response_hex']}"
                )
        if not successes:
            unique_errors = sorted({record.get("error", "unknown") for record in records})
            print("NO_VALID_ZLAC_RESPONSE")
            for error in unique_errors[:12]:
                print(f"error: {error}")
    raise SystemExit(0 if successes else 2)


if __name__ == "__main__":
    main()
