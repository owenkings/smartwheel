#!/usr/bin/env python3
"""Test host-scheduled dual XT-M60 start offsets without device config writes."""

import argparse
import json
import os
import signal
import statistics
import subprocess
import sys
import time
from pathlib import Path


WORKSPACE = Path(__file__).resolve().parents[2]
DIAGNOSTIC = WORKSPACE / "scripts" / "hardware" / "xtm60_signal_diagnostic.py"
BIND_SHIM = (
    WORKSPACE
    / "install"
    / "wheelchair_bringup"
    / "lib"
    / "wheelchair_bringup"
    / "libxt_bindshim.so"
)


def _summary(data):
    return {
        "point_valid_fraction": data["point_valid_fraction"]["mean"],
        "mapping_valid_fraction": data["mapping_valid_fraction"]["mean"],
        "amplitude_median": data["amplitude_for_point_valid"]["median"],
        "range_median_m": data["point_range_m"]["median"],
        "delta_median_mm": data["consecutive_distance_delta_mm"]["median"],
        "delta_p95_mm": data["consecutive_distance_delta_mm"]["p95"],
        "changed_fraction_ge_50mm": data[
            "consecutive_changed_fraction_ge_50mm"
        ]["mean"],
        "start_call_monotonic_sec": data["start_call_monotonic_sec"],
        "start_lateness_sec": data["start_lateness_sec"],
    }


def _nearest_receive_delta(left_data, right_data):
    left_times = sorted(
        item["receive_monotonic_sec"] for item in left_data["frame_metadata"]
    )
    right_times = sorted(
        item["receive_monotonic_sec"] for item in right_data["frame_metadata"]
    )
    if not left_times or not right_times:
        return {"count": 0}
    deltas = []
    right_index = 0
    for left_time in left_times:
        while (
            right_index + 1 < len(right_times)
            and abs(right_times[right_index + 1] - left_time)
            <= abs(right_times[right_index] - left_time)
        ):
            right_index += 1
        deltas.append(abs(right_times[right_index] - left_time))
    return {
        "count": len(deltas),
        "min": min(deltas),
        "mean": statistics.fmean(deltas),
        "median": statistics.median(deltas),
        "max": max(deltas),
    }


def _indexed_receive_offset(left_data, right_data):
    left_times = [
        item["receive_monotonic_sec"] for item in left_data["frame_metadata"]
    ]
    right_times = [
        item["receive_monotonic_sec"] for item in right_data["frame_metadata"]
    ]
    count = min(len(left_times), len(right_times))
    if count <= 0:
        return {"count": 0}
    offsets = [
        right_times[index] - left_times[index] for index in range(count)
    ]
    edge_count = min(20, count)
    return {
        "count": count,
        "min": min(offsets),
        "mean": statistics.fmean(offsets),
        "median": statistics.median(offsets),
        "max": max(offsets),
        "first_edge_median": statistics.median(offsets[:edge_count]),
        "last_edge_median": statistics.median(offsets[-edge_count:]),
        "edge_drift": (
            statistics.median(offsets[-edge_count:])
            - statistics.median(offsets[:edge_count])
        ),
    }


def _spawn(sensor, start_at, frame_count, output_path):
    if sensor == "left":
        device_ip = "192.168.0.101"
        bind_ip = "192.168.0.100"
    else:
        device_ip = "192.168.1.101"
        bind_ip = "192.168.1.100"
    env = os.environ.copy()
    env.update(
        {
            "LD_PRELOAD": str(BIND_SHIM),
            "XT_BIND_IP": bind_ip,
            "XT_BIND_PORT": "7687",
        }
    )
    command = [
        sys.executable,
        str(DIAGNOSTIC),
        "--ip-address",
        device_ip,
        "--udp-dest-ip",
        bind_ip,
        "--frame-count",
        str(frame_count),
        "--frame-timeout-sec",
        f"{max(25.0, frame_count / 8.0 + 10.0):.1f}",
        "--start-at-monotonic",
        f"{start_at:.9f}",
        "--output",
        str(output_path),
    ]
    return subprocess.Popen(
        command,
        cwd=WORKSPACE,
        env=env,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.PIPE,
        text=True,
    )


def _wait_children(children, timeout_sec):
    deadline = time.monotonic() + timeout_sec
    results = []
    for name, process in children:
        remaining = max(0.0, deadline - time.monotonic())
        try:
            _, stderr = process.communicate(timeout=remaining)
        except subprocess.TimeoutExpired:
            process.send_signal(signal.SIGINT)
            try:
                _, stderr = process.communicate(timeout=10.0)
            except subprocess.TimeoutExpired:
                process.kill()
                _, stderr = process.communicate()
        results.append(
            {
                "sensor": name,
                "returncode": process.returncode,
                "stderr": stderr[-4000:],
            }
        )
    return results


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--offset-ms",
        type=float,
        nargs="+",
        default=[0.0, 25.0, 50.0, 75.0],
    )
    parser.add_argument("--frame-count", type=int, default=60)
    parser.add_argument("--ready-delay-sec", type=float, default=7.0)
    parser.add_argument("--between-trials-sec", type=float, default=2.0)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    args.output_dir.mkdir(parents=True, exist_ok=True)
    result = {
        "schema": "smartwheel.xtm60_dual_phase_ab.v1",
        "read_only_imaging_configuration": True,
        "device_configuration_written": False,
        "frame_count_per_sensor_per_trial": args.frame_count,
        "trials": [],
    }
    for trial_index, offset_ms in enumerate(args.offset_ms):
        target = time.monotonic() + args.ready_delay_sec
        left_path = args.output_dir / (
            f"XT_M60_PHASE_{trial_index:02d}_{offset_ms:g}MS_LEFT.json"
        )
        right_path = args.output_dir / (
            f"XT_M60_PHASE_{trial_index:02d}_{offset_ms:g}MS_RIGHT.json"
        )
        children = [
            ("left", _spawn("left", target, args.frame_count, left_path)),
            (
                "right",
                _spawn(
                    "right",
                    target + offset_ms / 1000.0,
                    args.frame_count,
                    right_path,
                ),
            ),
        ]
        child_results = _wait_children(
            children, timeout_sec=max(40.0, args.frame_count / 8.0 + 20.0)
        )
        trial = {
            "requested_offset_ms": offset_ms,
            "children": child_results,
            "left_evidence": str(left_path),
            "right_evidence": str(right_path),
        }
        if all(item["returncode"] == 0 for item in child_results):
            left_data = json.loads(left_path.read_text(encoding="utf-8"))
            right_data = json.loads(right_path.read_text(encoding="utf-8"))
            trial["left"] = _summary(left_data)
            trial["right"] = _summary(right_data)
            trial["actual_start_offset_ms"] = (
                right_data["start_call_monotonic_sec"]
                - left_data["start_call_monotonic_sec"]
            ) * 1000.0
            trial["nearest_receive_delta_sec"] = _nearest_receive_delta(
                left_data, right_data
            )
            trial["indexed_receive_offset_sec"] = _indexed_receive_offset(
                left_data, right_data
            )
        result["trials"].append(trial)
        time.sleep(args.between_trials_sec)

    result["success"] = all(
        all(item["returncode"] == 0 for item in trial["children"])
        for trial in result["trials"]
    )
    rendered = json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True)
    print(rendered)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(rendered + "\n", encoding="utf-8")
    return 0 if result["success"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
