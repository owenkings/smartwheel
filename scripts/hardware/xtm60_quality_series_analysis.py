#!/usr/bin/env python3
"""Bin XT-M60 quality-topic evidence and correlate validity with temperature."""

import argparse
import json
import math
import statistics
from pathlib import Path


def _finite_pairs(left, right):
    return [
        (float(a), float(b))
        for a, b in zip(left, right)
        if a is not None
        and b is not None
        and math.isfinite(float(a))
        and math.isfinite(float(b))
    ]


def _correlation(left, right):
    pairs = _finite_pairs(left, right)
    if len(pairs) < 2:
        return None
    xs = [a for a, _b in pairs]
    ys = [b for _a, b in pairs]
    mx = statistics.fmean(xs)
    my = statistics.fmean(ys)
    numerator = sum((x - mx) * (y - my) for x, y in pairs)
    denominator = math.sqrt(
        sum((x - mx) ** 2 for x in xs)
        * sum((y - my) ** 2 for y in ys)
    )
    return numerator / denominator if denominator > 0.0 else None


def _summary(values):
    finite = [
        float(value)
        for value in values
        if value is not None and math.isfinite(float(value))
    ]
    if not finite:
        return {"count": 0}
    return {
        "count": len(finite),
        "min": min(finite),
        "mean": statistics.fmean(finite),
        "median": statistics.median(finite),
        "max": max(finite),
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("input", type=Path)
    parser.add_argument("--bin-sec", type=float, default=60.0)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()

    payload = json.loads(args.input.read_text(encoding="utf-8"))
    elapsed = payload.get("elapsed_sec_series", [])
    accepted = payload.get("accepted_series", [])
    valid = payload.get("valid_fraction_series", [])
    median_delta = payload.get("median_range_delta_mm_series", [])
    p95_delta = payload.get("p95_range_delta_mm_series", [])
    temperature = payload.get("temperature_c_series", [])
    vcsel_temperature = payload.get("vcsel_temperature_c_series", [])

    count = min(
        len(elapsed),
        len(accepted),
        len(valid),
        len(median_delta),
        len(p95_delta),
        len(temperature),
        len(vcsel_temperature),
    )
    bins = []
    if count:
        max_bin = int(max(float(value) for value in elapsed[:count]) // args.bin_sec)
        for index in range(max_bin + 1):
            indexes = [
                item
                for item, value in enumerate(elapsed[:count])
                if index * args.bin_sec
                <= float(value)
                < (index + 1) * args.bin_sec
            ]
            if not indexes:
                continue
            bins.append(
                {
                    "start_sec": index * args.bin_sec,
                    "end_sec": (index + 1) * args.bin_sec,
                    "frames": len(indexes),
                    "acceptance_fraction": statistics.fmean(
                        1.0 if accepted[item] else 0.0 for item in indexes
                    ),
                    "valid_fraction": _summary(valid[item] for item in indexes),
                    "median_range_delta_mm": _summary(
                        median_delta[item] for item in indexes
                    ),
                    "p95_range_delta_mm": _summary(
                        p95_delta[item] for item in indexes
                    ),
                    "temperature_c": _summary(
                        temperature[item] for item in indexes
                    ),
                    "vcsel_temperature_c": _summary(
                        vcsel_temperature[item] for item in indexes
                    ),
                }
            )

    result = {
        "schema": "smartwheel.xtm60_quality_series_analysis.v1",
        "source": str(args.input),
        "bin_sec": args.bin_sec,
        "frame_count": count,
        "temperature_valid_fraction_correlation": _correlation(
            temperature[:count], valid[:count]
        ),
        "vcsel_temperature_valid_fraction_correlation": _correlation(
            vcsel_temperature[:count], valid[:count]
        ),
        "bins": bins,
    }
    rendered = json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True)
    print(rendered)
    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
