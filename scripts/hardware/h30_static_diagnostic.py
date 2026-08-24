#!/usr/bin/env python3
"""Read-only H30/Yesense stationary diagnostic.

This tool opens the configured serial port and parses the existing binary stream. It
does not transmit configuration or calibration commands to the device.
"""

import argparse
import json
import math
import statistics
import sys
import time
from pathlib import Path
from typing import Iterable, Sequence


WORKSPACE = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(WORKSPACE / "src" / "wheelchair_sensors"))

from wheelchair_sensors.imu_adapter_node import H30ImuAdapter  # noqa: E402


def _axis_stats(vectors: Sequence[Sequence[float]], labels: Iterable[str]):
    result = {"count": len(vectors)}
    if not vectors:
        return result
    for index, label in enumerate(labels):
        values = [float(vector[index]) for vector in vectors]
        result[label] = {
            "mean": statistics.fmean(values),
            "stddev": statistics.pstdev(values),
            "min": min(values),
            "max": max(values),
        }
    return result


def _norm_stats(vectors: Sequence[Sequence[float]]):
    if not vectors:
        return {"count": 0}
    norms = [math.sqrt(sum(float(value) ** 2 for value in vector)) for vector in vectors]
    return {
        "count": len(norms),
        "mean": statistics.fmean(norms),
        "stddev": statistics.pstdev(norms),
        "min": min(norms),
        "max": max(norms),
    }


def _quaternion_to_rpy(quaternion):
    x, y, z, w = (float(value) for value in quaternion)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def _timestamp_stats(timestamps):
    if not timestamps:
        return {"count": 0, "available": False}
    deltas = [current - previous for previous, current in zip(timestamps, timestamps[1:])]
    positive = [delta for delta in deltas if delta > 0]
    return {
        "count": len(timestamps),
        "available": True,
        "first_us": timestamps[0],
        "last_us": timestamps[-1],
        "nonpositive_deltas": sum(delta <= 0 for delta in deltas),
        "mean_positive_delta_us": statistics.fmean(positive) if positive else None,
        "min_delta_us": min(deltas) if deltas else None,
        "max_delta_us": max(deltas) if deltas else None,
    }


def collect(port: str, baud_rate: int, duration_sec: float):
    adapter = H30ImuAdapter(port=port, baud_rate=baud_rate, timeout_sec=0.02)
    accelerations = []
    angular_velocities = []
    quaternions = []
    device_euler = []
    device_timestamps = []
    sample_count = 0
    started = time.monotonic()
    try:
        while time.monotonic() - started < duration_sec:
            samples = adapter.read_samples()
            if not samples:
                continue
            sample_count += len(samples)
            for sample in samples:
                if sample.accel_mps2 is not None:
                    accelerations.append(sample.accel_mps2)
                if sample.gyro_rps is not None:
                    angular_velocities.append(sample.gyro_rps)
                if sample.quat_xyzw is not None:
                    quaternions.append(sample.quat_xyzw)
                if sample.euler_rad is not None:
                    # Parser order is pitch, roll, yaw; normalize report to roll, pitch, yaw.
                    pitch, roll, yaw = sample.euler_rad
                    device_euler.append((roll, pitch, yaw))
                if sample.sample_timestamp_us is not None:
                    device_timestamps.append(sample.sample_timestamp_us)
    finally:
        adapter.close()

    elapsed = time.monotonic() - started
    quaternion_rpy = [_quaternion_to_rpy(quaternion) for quaternion in quaternions]
    orientation_vectors = device_euler or quaternion_rpy
    orientation_source = "euler_tlv" if device_euler else "quaternion_tlv"

    result = {
        "schema": "smartwheel.h30_static_diagnostic.v1",
        "read_only": True,
        "port": port,
        "baud_rate": baud_rate,
        "requested_duration_sec": duration_sec,
        "elapsed_sec": elapsed,
        "sample_count": sample_count,
        "sample_rate_hz": sample_count / elapsed if elapsed > 0 else 0.0,
        "field_counts": {
            "acceleration": len(accelerations),
            "angular_velocity": len(angular_velocities),
            "quaternion": len(quaternions),
            "device_euler": len(device_euler),
            "device_timestamp": len(device_timestamps),
        },
        "acceleration_mps2": _axis_stats(accelerations, ("x", "y", "z")),
        "acceleration_norm_mps2": _norm_stats(accelerations),
        "angular_velocity_rps": _axis_stats(angular_velocities, ("x", "y", "z")),
        "angular_velocity_norm_rps": _norm_stats(angular_velocities),
        "quaternion_xyzw": _axis_stats(quaternions, ("x", "y", "z", "w")),
        "quaternion_norm": _norm_stats(quaternions),
        "orientation_source": orientation_source if orientation_vectors else "unavailable",
        "orientation_rpy_deg": _axis_stats(
            [tuple(math.degrees(value) for value in vector) for vector in orientation_vectors],
            ("roll", "pitch", "yaw"),
        ),
        "device_timestamp": _timestamp_stats(device_timestamps),
    }

    accel_norm = result["acceleration_norm_mps2"].get("mean")
    gyro_norm = result["angular_velocity_norm_rps"].get("mean")
    quaternion_norm = result["quaternion_norm"].get("mean")
    result["checks"] = {
        "samples_present": sample_count > 0,
        # H30 supports both 100 Hz and 200 Hz output profiles. The diagnostic is
        # intentionally read-only, so accept either supported stream rate here.
        "supported_rate_near_100_or_200_hz": (
            80.0 <= result["sample_rate_hz"] <= 120.0
            or 180.0 <= result["sample_rate_hz"] <= 220.0
        ),
        "gravity_norm_plausible": accel_norm is not None and 8.0 <= accel_norm <= 11.5,
        "stationary_gyro_plausible": gyro_norm is not None and gyro_norm < 0.05,
        "quaternion_norm_plausible": quaternion_norm is not None
        and 0.95 <= quaternion_norm <= 1.05,
        "device_timestamp_monotonic": (
            not device_timestamps
            or result["device_timestamp"]["nonpositive_deltas"] == 0
        ),
    }
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="/dev/smartwheel_h30_imu")
    parser.add_argument("--baud-rate", type=int, default=460800)
    parser.add_argument("--duration-sec", type=float, default=30.0)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()

    result = collect(args.port, args.baud_rate, args.duration_sec)
    rendered = json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True)
    print(rendered)
    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered + "\n", encoding="utf-8")
    return 0 if result["checks"]["samples_present"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
