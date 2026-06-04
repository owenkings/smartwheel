#!/usr/bin/env python3
"""轮询 4 路 FD07-34 RS485 超声波传感器 (地址 1~4)，读保持寄存器 0x0001 取毫米距离。

用法:
  scripts/poll_ultrasonic_4sensors.py                 # 9600 8N1, 地址1-4, 持续轮询
  scripts/poll_ultrasonic_4sensors.py --cycles 5      # 只轮询 5 轮后退出
  scripts/poll_ultrasonic_4sensors.py --baud 115200 --addresses 1,2,3,4
"""
import argparse
import struct
import time

import serial


def modbus_crc16(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc & 0xFFFF


def read_command(address: int, register: int) -> bytes:
    body = struct.pack(">BBHH", address, 0x03, register & 0xFFFF, 1)
    return body + struct.pack("<H", modbus_crc16(body))


def parse_distance_mm(resp: bytes, address: int):
    """返回 (距离mm, None) 或 (None, 错误说明)。"""
    if len(resp) != 7:
        return None, f"收到 {len(resp)} 字节"
    if modbus_crc16(resp[:-2]) != struct.unpack("<H", resp[-2:])[0]:
        return None, "CRC 错误"
    if resp[0] != address or resp[1] != 0x03 or resp[2] != 2:
        return None, f"帧头异常 {resp[:3].hex()}"
    return struct.unpack(">H", resp[3:5])[0], None


def main() -> None:
    ap = argparse.ArgumentParser(description="轮询 4 路 RS485 超声波传感器")
    ap.add_argument("--port", default="/dev/smartwheel_ultrasonic")
    ap.add_argument("--baud", type=int, default=9600)
    ap.add_argument("--addresses", default="1,2,3,4", help="逗号分隔的从机地址")
    ap.add_argument("--register", type=lambda v: int(v, 0), default=0x0001)
    ap.add_argument("--interval", type=float, default=0.1, help="每轮间隔秒")
    ap.add_argument("--timeout", type=float, default=0.2, help="单次读超时秒")
    ap.add_argument("--cycles", type=int, default=0, help="轮询轮数, 0=无限")
    args = ap.parse_args()

    addresses = [int(a) for a in args.addresses.split(",") if a.strip()]
    commands = {a: read_command(a, args.register) for a in addresses}

    with serial.Serial(
        args.port, args.baud, bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,
        timeout=args.timeout, write_timeout=args.timeout,
    ) as handle:
        time.sleep(0.1)
        cycle = 0
        while args.cycles == 0 or cycle < args.cycles:
            cells = []
            for address in addresses:
                handle.reset_input_buffer()
                handle.write(commands[address])
                handle.flush()
                distance_mm, error = parse_distance_mm(handle.read(7), address)
                if error is None:
                    cells.append(f"#{address}={distance_mm:>4d}mm")
                else:
                    cells.append(f"#{address}=----({error})")
                time.sleep(0.02)
            print(f"[{time.strftime('%H:%M:%S')}] " + "  ".join(cells), flush=True)
            cycle += 1
            time.sleep(args.interval)


if __name__ == "__main__":
    main()
