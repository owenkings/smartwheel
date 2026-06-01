#!/usr/bin/env python3
"""Disable (free) the ZLAC8030D servo so the wheelchair can be pushed by hand,
and VERIFY it actually de-energized via the per-axis status word 0x20A2.

ZLAC8030D semantics confirmed on this hardware:
  control word 0x200E: 0x08=ENABLE(servo on, wheel HELD), 0x07=STOP, 0x05=EMERGENCY-STOP.
  status word  0x20A2: byte 0x40 per axis => ENABLED/held; byte 0x80 => DISABLED/free.
STOP and EMERGENCY-STOP both de-energize the servo (status 0x80 => back-drivable).
This is the canonical "make the chair pushable" command, used by the mapping
start path and scripts/hardware_shutdown.sh. Does not require ROS to be running.
"""

import argparse
import sys
from pathlib import Path

workspace = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(workspace / "src" / "wheelchair_base"))

from wheelchair_base.modbus_rtu import ModbusRtuClient, ModbusSerialConfig  # noqa: E402


def _enabled(status: int) -> bool:
    """True if either axis status byte still has the 0x40 operation-enabled bit."""
    return bool((status >> 8) & 0x40) or bool(status & 0x40)


def main() -> int:
    parser = argparse.ArgumentParser(description="Free/disable the ZLAC8030 servo and verify.")
    parser.add_argument("--port", default="/dev/smartwheel_zlac8030")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--timeout", type=float, default=0.15)
    parser.add_argument("--slave-id", type=int, default=1)
    parser.add_argument("--single-slave-dual-axis", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--command-left-register", type=lambda v: int(v, 0), default=0x2088)
    parser.add_argument("--control-word-register", type=lambda v: int(v, 0), default=0x200E)
    parser.add_argument("--status-register", type=lambda v: int(v, 0), default=0x20A2)
    parser.add_argument("--stop-value", type=lambda v: int(v, 0), default=0x07)
    parser.add_argument("--emergency-stop-value", type=lambda v: int(v, 0), default=0x05)
    parser.add_argument("--no-verify", action="store_true")
    args = parser.parse_args()

    client = ModbusRtuClient(ModbusSerialConfig(args.port, args.baud, args.timeout))
    try:
        # 1) zero the target speed, 2) STOP, and if still energized, EMERGENCY-STOP.
        client.write_multiple_registers(args.slave_id, args.command_left_register, [0, 0])
        client.write_single_register(args.slave_id, args.control_word_register, args.stop_value)
        if args.no_verify:
            print(f"ZLAC8030 stop written on {args.port} (verification skipped)")
            return 0
        status = client.read_holding_registers(args.slave_id, args.status_register, 1)[0]
        if _enabled(status):
            client.write_single_register(args.slave_id, args.control_word_register, args.emergency_stop_value)
            status = client.read_holding_registers(args.slave_id, args.status_register, 1)[0]
    except Exception as exc:
        print(f"ERROR: could not talk to ZLAC8030 on {args.port}: {exc}", file=sys.stderr)
        print("If the base driver / smartwheel.service is running it owns the serial port; "
              "stop it first. If the wheel is still locked, power-cycle the chassis.", file=sys.stderr)
        return 1
    finally:
        client.close()

    if _enabled(status):
        print(f"WARN: servo still ENABLED (status=0x{status:04X}); wheel may stay locked. "
              "Power-cycle the chassis to release.", file=sys.stderr)
        return 2
    print(f"ZLAC8030 servo DISABLED/free, pushable (status=0x{status:04X}) on {args.port}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
