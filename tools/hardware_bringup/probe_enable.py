#!/usr/bin/env python3
"""Probe ZLAC8030D enable: clear-fault -> enable, then read status 0x20A2.

Reports whether the drive actually enters the operation-enabled state (0x40)
after we command ENABLE (0x08). Reads status at each step. Does NOT command any
speed; safe to run. Wheels off ground recommended anyway.
"""
import time
from test_zlac8030d_motor import (
    Modbus, SLAVE_ID,
    REG_CONTROL_MODE, REG_CONTROL_WORD, REG_ASYNC_MODE,
    MODE_VELOCITY, CONTROL_CLEAR_FAULT, CONTROL_ENABLE, CONTROL_STOP,
)

STATUS_REG = 0x20A2


def axis_bits(status):
    hi = (status >> 8) & 0xFF
    lo = status & 0xFF
    return hi, lo


def show(bus, label):
    st = bus.read_holding(SLAVE_ID, STATUS_REG, 1)[0]
    hi, lo = axis_bits(st)
    en = bool(hi & 0x40) or bool(lo & 0x40)
    dis = bool(hi & 0x80) or bool(lo & 0x80)
    print(f"{label}: 0x20A2=0x{st:04X} hi=0x{hi:02X} lo=0x{lo:02X} enabled={en} disabled={dis}")
    return st


def main():
    bus = Modbus("/dev/ttyACM0", 115200, 0.25)
    try:
        show(bus, "initial")
        bus.write_single(SLAVE_ID, REG_CONTROL_WORD, CONTROL_STOP)
        time.sleep(0.2); show(bus, "after_stop")
        bus.write_single(SLAVE_ID, REG_CONTROL_MODE, MODE_VELOCITY)
        bus.write_single(SLAVE_ID, REG_ASYNC_MODE, 0)
        bus.write_single(SLAVE_ID, REG_CONTROL_WORD, CONTROL_CLEAR_FAULT)
        time.sleep(0.3); show(bus, "after_clear_fault")
        bus.write_single(SLAVE_ID, REG_CONTROL_WORD, CONTROL_ENABLE)
        time.sleep(0.3); show(bus, "after_enable")
        time.sleep(0.5); show(bus, "after_enable+0.5s")
        # leave it stopped/safe
        bus.write_single(SLAVE_ID, REG_CONTROL_WORD, CONTROL_STOP)
    finally:
        bus.close()


if __name__ == "__main__":
    main()
