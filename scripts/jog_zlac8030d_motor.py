#!/usr/bin/env python3
"""Interactive deadman jog control for ZLAC8030D.

This is intentionally a bring-up tool, not the production navigation path.
It requires a local terminal, explicit off-ground confirmation, and sends
zero speed automatically when key input stops.
"""

from __future__ import annotations

import argparse
import select
import sys
import termios
import time
import tty

from test_zlac8030d_motor import (
    CONTROL_CLEAR_FAULT,
    CONTROL_ENABLE,
    MODE_VELOCITY,
    REG_ASYNC_MODE,
    REG_CONTROL_MODE,
    REG_CONTROL_WORD,
    REG_TARGET_SPEED_LEFT,
    SLAVE_ID,
    Modbus,
    emergency_stop,
    print_snapshot,
    read_snapshot,
    stop,
)


def read_key(timeout: float) -> str | None:
    ready, _, _ = select.select([sys.stdin], [], [], timeout)
    if not ready:
        return None
    return sys.stdin.read(1)


def prepare_drive(bus: Modbus) -> None:
    stop(bus)
    bus.write_single(SLAVE_ID, REG_CONTROL_MODE, MODE_VELOCITY)
    bus.write_single(SLAVE_ID, REG_ASYNC_MODE, 0)
    bus.write_single(SLAVE_ID, REG_CONTROL_WORD, CONTROL_CLEAR_FAULT)
    time.sleep(0.2)
    bus.write_single(SLAVE_ID, REG_CONTROL_WORD, CONTROL_ENABLE)
    time.sleep(0.2)
    print_snapshot("enabled", read_snapshot(bus))


def apply_direction(left_rpm: int, right_rpm: int, invert_left: bool, invert_right: bool) -> tuple[int, int]:
    return (
        -left_rpm if invert_left else left_rpm,
        -right_rpm if invert_right else right_rpm,
    )


def write_targets(bus: Modbus, left_rpm: int, right_rpm: int) -> None:
    bus.write_multiple(SLAVE_ID, REG_TARGET_SPEED_LEFT, [left_rpm, right_rpm])


def write_zero(bus: Modbus) -> None:
    write_targets(bus, 0, 0)


def motor_targets(
    left_rpm: int,
    right_rpm: int,
    forward_sign: int,
    invert_left: bool,
    invert_right: bool,
) -> tuple[int, int]:
    return apply_direction(
        forward_sign * left_rpm,
        forward_sign * right_rpm,
        invert_left,
        invert_right,
    )


def print_help(
    rpm: int,
    forward_sign: int,
    turn_sign: int,
    invert_left: bool,
    invert_right: bool,
) -> None:
    forward_left, forward_right = motor_targets(
        rpm, rpm, forward_sign, invert_left, invert_right
    )
    reverse_left, reverse_right = motor_targets(
        -rpm, -rpm, forward_sign, invert_left, invert_right
    )
    turn_left, turn_right = motor_targets(
        -turn_sign * rpm, turn_sign * rpm, forward_sign, invert_left, invert_right
    )
    right_left, right_right = motor_targets(
        turn_sign * rpm, -turn_sign * rpm, forward_sign, invert_left, invert_right
    )
    print("")
    print("ZLAC8030D WASD jog mode")
    print(
        f"  w      forward {rpm} rpm while repeatedly pressed "
        f"(register left={forward_left}, right={forward_right})"
    )
    print(
        f"  s      reverse {rpm} rpm while repeatedly pressed "
        f"(register left={reverse_left}, right={reverse_right})"
    )
    print(
        f"  a      turn left {rpm} rpm while repeatedly pressed "
        f"(register left={turn_left}, right={turn_right})"
    )
    print(
        f"  d      turn right {rpm} rpm while repeatedly pressed "
        f"(register left={right_left}, right={right_right})"
    )
    print("  x      zero speed")
    print("  space  software emergency stop, latched")
    print("  e      software emergency stop, latched")
    print("  r      re-enable after software emergency stop")
    print("  q      quit, emergency stop")
    print("")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--timeout", type=float, default=0.25)
    parser.add_argument("--rpm", type=int, default=20)
    parser.add_argument("--deadman-sec", type=float, default=0.35)
    parser.add_argument("--status-sec", type=float, default=1.0)
    parser.add_argument(
        "--forward-sign",
        type=int,
        choices=(-1, 1),
        default=-1,
        help="Motor sign used for physical forward. Default -1 because this wheelchair moved backward with +1.",
    )
    parser.add_argument(
        "--turn-sign",
        type=int,
        choices=(-1, 1),
        default=1,
        help="Swap A/D turn direction if the physical chair turns opposite.",
    )
    parser.add_argument("--invert-left", action="store_true")
    parser.add_argument("--invert-right", dest="invert_right", action="store_true", default=True)
    parser.add_argument("--no-invert-right", dest="invert_right", action="store_false")
    parser.add_argument("--i-confirm-wheels-off-ground", action="store_true")
    args = parser.parse_args()

    if not args.i_confirm_wheels_off_ground:
        raise SystemExit(
            "Refusing jog mode without --i-confirm-wheels-off-ground. "
            "Lift drive wheels, clear the area, and verify hardware E-stop first."
        )
    if args.rpm <= 0 or args.rpm > 60:
        raise SystemExit("--rpm must be between 1 and 60 for bring-up jog mode.")
    if not sys.stdin.isatty():
        raise SystemExit("jog mode requires a local interactive terminal.")

    old_terminal = termios.tcgetattr(sys.stdin)
    bus = Modbus(args.port, args.baud, args.timeout)
    active = False
    emergency_latched = False
    last_motion_command = 0.0
    next_status = time.monotonic()

    try:
        tty.setcbreak(sys.stdin.fileno())
        print_help(
            args.rpm,
            args.forward_sign,
            args.turn_sign,
            args.invert_left,
            args.invert_right,
        )
        prepare_drive(bus)

        while True:
            now = time.monotonic()
            key = read_key(0.05)
            if key:
                if key in ("q", "\x03"):
                    print("quit requested")
                    break
                if key in (" ", "e"):
                    emergency_stop(bus)
                    emergency_latched = True
                    active = False
                    print_snapshot("emergency_stop", read_snapshot(bus))
                    continue
                if key == "r":
                    prepare_drive(bus)
                    emergency_latched = False
                    active = False
                    continue
                if emergency_latched:
                    print("emergency stop is latched; press r to re-enable")
                    continue
                if key == "w":
                    left, right = motor_targets(
                        args.rpm,
                        args.rpm,
                        args.forward_sign,
                        args.invert_left,
                        args.invert_right,
                    )
                    write_targets(bus, left, right)
                    active = True
                    last_motion_command = now
                    next_status = now
                elif key == "s":
                    left, right = motor_targets(
                        -args.rpm,
                        -args.rpm,
                        args.forward_sign,
                        args.invert_left,
                        args.invert_right,
                    )
                    write_targets(bus, left, right)
                    active = True
                    last_motion_command = now
                    next_status = now
                elif key == "a":
                    left, right = motor_targets(
                        -args.turn_sign * args.rpm,
                        args.turn_sign * args.rpm,
                        args.forward_sign,
                        args.invert_left,
                        args.invert_right,
                    )
                    write_targets(bus, left, right)
                    active = True
                    last_motion_command = now
                    next_status = now
                elif key == "d":
                    left, right = motor_targets(
                        args.turn_sign * args.rpm,
                        -args.turn_sign * args.rpm,
                        args.forward_sign,
                        args.invert_left,
                        args.invert_right,
                    )
                    write_targets(bus, left, right)
                    active = True
                    last_motion_command = now
                    next_status = now
                elif key == "x":
                    write_zero(bus)
                    active = False
                    print_snapshot("zero", read_snapshot(bus))

            if active and now - last_motion_command >= args.deadman_sec:
                write_zero(bus)
                active = False
                print_snapshot("deadman_zero", read_snapshot(bus))

            if now >= next_status:
                print_snapshot("status", read_snapshot(bus))
                next_status = now + args.status_sec
    finally:
        try:
            emergency_stop(bus)
            time.sleep(0.2)
            print_snapshot("final_emergency_stop", read_snapshot(bus))
        finally:
            bus.close()
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_terminal)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
