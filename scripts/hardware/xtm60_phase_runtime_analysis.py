#!/usr/bin/env python3
"""Compare dual XT-M60 ROS receive phase with per-frame quality evidence."""

import argparse
import bisect
import json
import statistics
from pathlib import Path


def _stats(values):
    if not values:
        return {"count": 0}
    return {
        "count": len(values),
        "min": min(values),
        "mean": statistics.fmean(values),
        "median": statistics.median(values),
        "max": max(values),
    }


def _nearest_signed(reference_times, value):
    index = bisect.bisect_left(reference_times, value)
    candidates = []
    if index < len(reference_times):
        candidates.append(reference_times[index])
    if index > 0:
        candidates.append(reference_times[index - 1])
    nearest = min(candidates, key=lambda item: abs(item - value))
    return nearest - value


def _phase_bins(phase_abs, valid_fraction, delta_median):
    bins = {}
    for index, phase in enumerate(phase_abs):
        lower_ms = int(min(49.999, phase * 1000.0) // 5 * 5)
        key = f"{lower_ms:02d}-{lower_ms + 5:02d}ms"
        item = bins.setdefault(key, {"valid": [], "delta": []})
        if index < len(valid_fraction):
            item["valid"].append(valid_fraction[index])
        if index > 0 and index - 1 < len(delta_median):
            item["delta"].append(delta_median[index - 1])
    return {
        key: {
            "valid_fraction": _stats(value["valid"]),
            "delta_median_mm": _stats(value["delta"]),
        }
        for key, value in sorted(bins.items())
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--left", type=Path, required=True)
    parser.add_argument("--right", type=Path, required=True)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()

    left = json.loads(args.left.read_text(encoding="utf-8"))
    right = json.loads(args.right.read_text(encoding="utf-8"))
    left_times = left["receive_monotonic_sec_series"]
    right_times = right["receive_monotonic_sec_series"]
    signed = [_nearest_signed(right_times, value) for value in left_times]
    absolute = [abs(value) for value in signed]
    start = min(left_times[0], right_times[0])
    segments = {}
    for index, timestamp in enumerate(left_times):
        segment = int((timestamp - start) // 10)
        item = segments.setdefault(
            f"{segment * 10:02d}-{(segment + 1) * 10:02d}s",
            {"phase": [], "valid": [], "delta": []},
        )
        item["phase"].append(absolute[index])
        item["valid"].append(left["frame_valid_fraction_series"][index])
        if index > 0:
            item["delta"].append(
                left["consecutive_range_delta_median_mm_series"][index - 1]
            )
    result = {
        "schema": "smartwheel.xtm60_phase_runtime_analysis.v1",
        "left": str(args.left),
        "right": str(args.right),
        "nearest_right_minus_left_sec": _stats(signed),
        "nearest_absolute_phase_sec": _stats(absolute),
        "left_phase_quality_bins": _phase_bins(
            absolute,
            left["frame_valid_fraction_series"],
            left["consecutive_range_delta_median_mm_series"],
        ),
        "ten_second_segments": {
            key: {
                "phase_abs_sec": _stats(value["phase"]),
                "left_valid_fraction": _stats(value["valid"]),
                "left_delta_median_mm": _stats(value["delta"]),
            }
            for key, value in sorted(segments.items())
        },
        "left_receive_gaps_over_150ms": [
            {
                "index": index,
                "gap_sec": current - previous,
                "time_from_start_sec": current - start,
            }
            for index, (previous, current) in enumerate(
                zip(left_times, left_times[1:]), start=1
            )
            if current - previous > 0.15
        ],
    }
    rendered = json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True)
    print(rendered)
    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
