#!/usr/bin/env python3
import argparse
import json
import statistics
import struct
import time
from datetime import datetime

import serial


def modbus_crc16(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


def append_crc(frame: bytes) -> bytes:
    return frame + struct.pack("<H", modbus_crc16(frame))


def read_holding_register(address: int, register: int) -> bytes:
    return append_crc(struct.pack(">BBHH", address, 0x03, register & 0xFFFF, 1))


def parse_response(frame: bytes, address: int) -> int:
    if len(frame) != 7:
        raise ValueError(f"expected 7 bytes, got {len(frame)}: {frame.hex()}")
    expected_crc = struct.unpack("<H", frame[-2:])[0]
    actual_crc = modbus_crc16(frame[:-2])
    if expected_crc != actual_crc:
        raise ValueError(f"bad crc got=0x{expected_crc:04x} expected=0x{actual_crc:04x}")
    if frame[0] != address:
        raise ValueError(f"unexpected address {frame[0]}, expected {address}")
    if frame[1] & 0x80:
        raise ValueError(f"modbus exception 0x{frame[2]:02x}")
    if frame[1] != 0x03 or frame[2] != 2:
        raise ValueError(f"unexpected response header: {frame[:3].hex()}")
    return struct.unpack(">H", frame[3:5])[0]


def main():
    parser = argparse.ArgumentParser(description="Read FD07-34/Modbus ultrasonic distance in millimeters.")
    parser.add_argument("--port", default="/dev/smartwheel_ultrasonic")
    parser.add_argument("--baud", type=int, default=9600)
    parser.add_argument("--address", type=int, default=1)
    parser.add_argument("--register", type=lambda value: int(value, 0), default=1)
    parser.add_argument("--count", type=int, default=10)
    parser.add_argument("--interval", type=float, default=0.2)
    parser.add_argument("--timeout", type=float, default=0.2)
    parser.add_argument("--json", action="store_true", help="print JSON lines")
    args = parser.parse_args()

    values = []
    request = read_holding_register(args.address, args.register)
    with serial.Serial(args.port, args.baud, timeout=args.timeout, write_timeout=args.timeout) as handle:
        for index in range(args.count):
            started = time.monotonic()
            handle.reset_input_buffer()
            handle.write(request)
            handle.flush()
            response = handle.read(7)
            record = {
                "sample": index + 1,
                "time": datetime.now().isoformat(timespec="milliseconds"),
                "address": args.address,
                "register": args.register,
                "request_hex": request.hex(),
                "response_hex": response.hex(),
                "ok": False,
            }
            try:
                distance_mm = parse_response(response, args.address)
                values.append(distance_mm)
                record.update(
                    {
                        "ok": True,
                        "distance_mm": distance_mm,
                        "distance_m": distance_mm / 1000.0,
                    }
                )
            except Exception as exc:
                record["error"] = str(exc)

            if args.json:
                print(json.dumps(record, ensure_ascii=False))
            elif record["ok"]:
                print(
                    f"{record['sample']:02d} {record['time']} "
                    f"addr={args.address} reg=0x{args.register:04x} "
                    f"distance={record['distance_mm']} mm ({record['distance_m']:.3f} m) "
                    f"raw={record['response_hex']}"
                )
            else:
                print(
                    f"{record['sample']:02d} {record['time']} "
                    f"addr={args.address} reg=0x{args.register:04x} ERROR {record['error']} "
                    f"raw={record['response_hex']}"
                )

            remaining = args.interval - (time.monotonic() - started)
            if index + 1 < args.count and remaining > 0:
                time.sleep(remaining)

    if values:
        summary = {
            "count": len(values),
            "min_mm": min(values),
            "max_mm": max(values),
            "mean_mm": round(statistics.mean(values), 3),
            "median_mm": statistics.median(values),
        }
        if args.json:
            print(json.dumps({"summary": summary}, ensure_ascii=False))
        else:
            print(
                "summary "
                f"count={summary['count']} "
                f"min={summary['min_mm']}mm "
                f"max={summary['max_mm']}mm "
                f"mean={summary['mean_mm']}mm "
                f"median={summary['median_mm']}mm"
            )
    else:
        raise SystemExit("no valid ultrasonic samples")


if __name__ == "__main__":
    main()
