#!/usr/bin/env python3
"""Read ZLAC8030 wheel-speed feedback without issuing any Modbus write."""

import argparse
import json
import struct
import statistics
import sys
import time
from pathlib import Path

import serial


REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "src" / "wheelchair_base"))

from wheelchair_base.modbus_rtu import crc16, from_i16, with_crc  # noqa: E402


class EchoAwareReadClient:
    """Function-0x03-only RTU client that strips an exact local request echo."""

    def __init__(self, port, baud_rate, timeout_sec):
        self.serial = serial.Serial(
            port,
            baud_rate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=timeout_sec,
        )
        self.echoed_requests = 0
        self.raw_examples = []

    def close(self):
        self.serial.close()

    def read_holding_register(self, slave_id, register):
        request = with_crc(struct.pack(">BBHH", slave_id, 0x03, register, 1))
        self.serial.reset_input_buffer()
        self.serial.write(request)
        self.serial.flush()
        # At most 8 echo bytes plus a normal 7-byte one-register response.
        raw = self.serial.read(len(request) + 7)
        if len(self.raw_examples) < 8:
            self.raw_examples.append(
                {"request_hex": request.hex(), "received_hex": raw.hex()}
            )
        if raw.startswith(request):
            self.echoed_requests += 1
            raw = raw[len(request) :]
        if not raw:
            raise TimeoutError("request echoed but no device response followed")
        if len(raw) < 5:
            raise TimeoutError(f"short response after echo removal: {raw.hex()}")
        function_code = raw[1]
        expected_len = 5 if function_code & 0x80 else 7
        if len(raw) != expected_len:
            raise TimeoutError(
                f"expected {expected_len} response bytes after echo removal, "
                f"got {len(raw)}: {raw.hex()}"
            )
        body = raw[:-2]
        expected_crc = struct.unpack("<H", raw[-2:])[0]
        actual_crc = crc16(body)
        if expected_crc != actual_crc:
            raise RuntimeError(
                f"bad response CRC: got 0x{expected_crc:04x}, "
                f"expected 0x{actual_crc:04x}; raw={raw.hex()}"
            )
        if raw[0] != slave_id:
            raise RuntimeError(f"unexpected slave id {raw[0]}, expected {slave_id}")
        if function_code & 0x80:
            raise RuntimeError(f"Modbus exception response 0x{raw[2]:02x}")
        if function_code != 0x03 or raw[2] != 2:
            raise RuntimeError(f"unexpected response: {raw.hex()}")
        return struct.unpack(">H", raw[3:5])[0]


def summarize(values):
    if not values:
        return {"count": 0}
    return {
        "count": len(values),
        "min": min(values),
        "max": max(values),
        "mean": statistics.fmean(values),
        "median": statistics.median(values),
        "stddev": statistics.pstdev(values),
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="/dev/smartwheel_zlac8030")
    parser.add_argument("--baud-rate", type=int, default=115200)
    parser.add_argument("--timeout-sec", type=float, default=0.2)
    parser.add_argument("--slave-id", type=int, default=1)
    parser.add_argument("--left-register", type=lambda v: int(v, 0), default=0x20AB)
    parser.add_argument("--right-register", type=lambda v: int(v, 0), default=0x20AC)
    parser.add_argument("--register-to-rpm-scale", type=float, default=0.112)
    parser.add_argument("--duration-sec", type=float, default=10.0)
    parser.add_argument("--poll-rate-hz", type=float, default=5.0)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()

    client = EchoAwareReadClient(
        args.port,
        args.baud_rate,
        args.timeout_sec,
    )
    started = time.monotonic()
    next_poll = started
    attempts = 0
    successes = 0
    errors = []
    sample_times = []
    left_raw = []
    right_raw = []
    left_rpm = []
    right_rpm = []

    try:
        while time.monotonic() - started < args.duration_sec:
            attempts += 1
            try:
                left_u16 = client.read_holding_register(
                    args.slave_id, args.left_register
                )
                right_u16 = client.read_holding_register(
                    args.slave_id, args.right_register
                )
                now = time.monotonic()
                left_i16 = from_i16(left_u16)
                right_i16 = from_i16(right_u16)
                successes += 1
                sample_times.append(now)
                left_raw.append(left_i16)
                right_raw.append(right_i16)
                left_rpm.append(left_i16 * args.register_to_rpm_scale)
                right_rpm.append(right_i16 * args.register_to_rpm_scale)
            except Exception as exc:
                errors.append(str(exc))

            next_poll += 1.0 / max(0.1, args.poll_rate_hz)
            delay = next_poll - time.monotonic()
            if delay > 0:
                time.sleep(delay)
    finally:
        client.close()

    elapsed = time.monotonic() - started
    deltas = [b - a for a, b in zip(sample_times, sample_times[1:])]
    result = {
        "schema": "smartwheel.zlac_feedback_readonly.v1",
        "read_only": True,
        "modbus_function_codes_sent": [3],
        "writes_performed": False,
        "port": args.port,
        "port_exists": Path(args.port).exists(),
        "baud_rate": args.baud_rate,
        "slave_id": args.slave_id,
        "feedback_registers": {
            "left": args.left_register,
            "right": args.right_register,
        },
        "register_to_rpm_scale": args.register_to_rpm_scale,
        "requested_duration_sec": args.duration_sec,
        "elapsed_sec": elapsed,
        "attempts": attempts,
        "successful_pairs": successes,
        "failed_attempts": attempts - successes,
        "response_rate_hz": (
            1.0 / statistics.fmean(deltas) if deltas else None
        ),
        "sample_delta_sec": summarize(deltas),
        "left_raw_i16": summarize(left_raw),
        "right_raw_i16": summarize(right_raw),
        "left_rpm": summarize(left_rpm),
        "right_rpm": summarize(right_rpm),
        "unique_errors": sorted(set(errors)),
        "echoed_request_count": client.echoed_requests,
        "raw_transaction_examples": client.raw_examples,
    }
    result["checks"] = {
        "port_present": result["port_exists"],
        "responses_present": successes > 0,
        "all_pairs_successful": successes == attempts and attempts > 0,
        "no_modbus_writes": result["writes_performed"] is False,
    }

    rendered = json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True)
    print(rendered)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered + "\n", encoding="utf-8")
    return 0 if successes > 0 else 2


if __name__ == "__main__":
    raise SystemExit(main())
