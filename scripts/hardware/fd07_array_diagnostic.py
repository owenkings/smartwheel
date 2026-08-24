#!/usr/bin/env python3
"""Bounded read-only diagnostic for four FD07-34R Modbus sensors."""

import argparse
import json
import statistics
import sys
import time
from pathlib import Path

import serial


REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "src" / "wheelchair_sensors"))

from wheelchair_sensors.ultrasonic_adapter_node import (  # noqa: E402
    build_read_holding_registers,
    parse_read_holding_registers_response,
)


def summarize(values):
    if not values:
        return {"count": 0}
    return {
        "count": len(values),
        "min": min(values),
        "median": statistics.median(values),
        "mean": statistics.fmean(values),
        "max": max(values),
        "stddev": statistics.pstdev(values),
    }


def read_response(port, request):
    """Read a seven-byte response, tolerating an exact eight-byte local echo."""
    first = port.read(7)
    echoed = False
    received = first
    if first == request[:7]:
        eighth = port.read(1)
        received += eighth
        if received == request:
            echoed = True
            received += port.read(7)
    response = received[8:] if echoed else received
    return response, echoed, received


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="/dev/smartwheel_ultrasonic")
    parser.add_argument("--baud-rate", type=int, default=9600)
    parser.add_argument("--timeout-sec", type=float, default=0.2)
    parser.add_argument("--addresses", nargs="+", type=int, default=[1, 2, 3, 4])
    parser.add_argument("--register", type=lambda value: int(value, 0), default=0x0001)
    parser.add_argument("--inter-request-sec", type=float, default=0.11)
    parser.add_argument("--cycle-rate-hz", type=float, default=2.0)
    parser.add_argument("--duration-sec", type=float, default=30.0)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()

    if args.register == 0x0001 and args.inter_request_sec <= 0.100:
        raise ValueError("register 0x0001 requires inter-request-sec > 0.100")
    if args.register == 0x0002 and args.inter_request_sec <= 0.300:
        raise ValueError("register 0x0002 requires inter-request-sec > 0.300")

    stats = {
        address: {
            "attempts": 0,
            "successes": 0,
            "distance_mm": [],
            "latency_sec": [],
            "errors": [],
            "raw_examples": [],
            "echoed_requests": 0,
        }
        for address in args.addresses
    }
    request_times = []
    started = time.monotonic()
    next_cycle = started
    last_request = None
    cycles = 0

    port = serial.Serial(
        args.port,
        args.baud_rate,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=args.timeout_sec,
    )
    try:
        while time.monotonic() - started < args.duration_sec:
            cycles += 1
            for address in args.addresses:
                if last_request is not None:
                    delay = args.inter_request_sec - (time.monotonic() - last_request)
                    if delay > 0:
                        time.sleep(delay)

                request = build_read_holding_registers(address, args.register, 1)
                port.reset_input_buffer()
                request_started = time.monotonic()
                request_times.append(request_started)
                last_request = request_started
                entry = stats[address]
                entry["attempts"] += 1
                port.write(request)
                port.flush()
                response, echoed, received = read_response(port, request)
                latency = time.monotonic() - request_started
                if echoed:
                    entry["echoed_requests"] += 1
                if len(entry["raw_examples"]) < 5:
                    entry["raw_examples"].append(
                        {
                            "request_hex": request.hex(),
                            "received_hex": received.hex(),
                            "response_hex": response.hex(),
                        }
                    )
                try:
                    _, registers = parse_read_holding_registers_response(
                        response, address
                    )
                    entry["successes"] += 1
                    entry["latency_sec"].append(latency)
                    entry["distance_mm"].append(int(registers[0]))
                except Exception as exc:
                    entry["errors"].append(str(exc))

            next_cycle += 1.0 / max(0.1, args.cycle_rate_hz)
            delay = next_cycle - time.monotonic()
            if delay > 0:
                time.sleep(delay)
    finally:
        port.close()

    elapsed = time.monotonic() - started
    request_deltas = [b - a for a, b in zip(request_times, request_times[1:])]
    sensors = {}
    for address, entry in stats.items():
        distances = entry["distance_mm"]
        sensors[str(address)] = {
            "attempts": entry["attempts"],
            "successes": entry["successes"],
            "failures": entry["attempts"] - entry["successes"],
            "success_fraction": (
                entry["successes"] / entry["attempts"] if entry["attempts"] else 0.0
            ),
            "distance_mm": summarize(distances),
            "within_documented_30_to_3000_mm_fraction": (
                sum(30 <= value <= 3000 for value in distances) / len(distances)
                if distances
                else None
            ),
            "latency_sec": summarize(entry["latency_sec"]),
            "echoed_requests": entry["echoed_requests"],
            "unique_errors": sorted(set(entry["errors"])),
            "raw_examples": entry["raw_examples"],
        }

    all_responsive = all(item["successes"] > 0 for item in sensors.values())
    all_successful = all(
        item["successes"] == item["attempts"] and item["attempts"] > 0
        for item in sensors.values()
    )
    result = {
        "schema": "smartwheel.fd07_array_diagnostic.v1",
        "read_only": True,
        "modbus_function_codes_sent": [3],
        "writes_performed": False,
        "port": args.port,
        "port_exists": Path(args.port).exists(),
        "baud_rate": args.baud_rate,
        "addresses": args.addresses,
        "register": args.register,
        "inter_request_sec": args.inter_request_sec,
        "cycle_rate_hz": args.cycle_rate_hz,
        "requested_duration_sec": args.duration_sec,
        "elapsed_sec": elapsed,
        "cycles": cycles,
        "request_delta_sec": summarize(request_deltas),
        "sensors": sensors,
        "checks": {
            "port_present": Path(args.port).exists(),
            "all_four_addresses_responsive": len(args.addresses) == 4 and all_responsive,
            "all_requests_successful": all_successful,
            "inter_request_strictly_over_100_ms": (
                bool(request_deltas) and min(request_deltas) > 0.100
            ),
            "no_modbus_writes": True,
        },
    }

    rendered = json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True)
    print(rendered)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered + "\n", encoding="utf-8")
    return 0 if all_responsive else 2


if __name__ == "__main__":
    raise SystemExit(main())
