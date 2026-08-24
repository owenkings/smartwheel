#!/usr/bin/env python3
"""Collect bounded statistics from one XT-M60 compact quality topic."""

import argparse
import json
import statistics
import time
from collections import Counter
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def _summary(values):
    finite = sorted(float(value) for value in values if value is not None)
    if not finite:
        return {"count": 0}
    p95_index = max(0, min(len(finite) - 1, int(0.95 * (len(finite) - 1))))
    return {
        "count": len(finite),
        "min": finite[0],
        "mean": statistics.fmean(finite),
        "median": statistics.median(finite),
        "p95": finite[p95_index],
        "max": finite[-1],
    }


class QualityDiagnostic(Node):
    def __init__(self, topic):
        super().__init__("xtm60_quality_topic_diagnostic")
        self.topic = topic
        self.receive_times = []
        self.parse_errors = 0
        self.accepted = 0
        self.dropped = 0
        self.reasons = Counter()
        self.valid_fractions = []
        self.baseline_valid_fractions = []
        self.median_delta_mm = []
        self.p95_delta_mm = []
        self.temperature_c = []
        self.vcsel_temperature_c = []
        self.elapsed_sec_series = []
        self.accepted_series = []
        self.valid_fraction_series = []
        self.median_range_delta_mm_series = []
        self.p95_range_delta_mm_series = []
        self.temperature_c_series = []
        self.vcsel_temperature_c_series = []
        self.last_cumulative_accepted = None
        self.last_cumulative_dropped = None
        self.create_subscription(String, topic, self._on_quality, 50)

    def _on_quality(self, message):
        receive_time = time.monotonic()
        self.receive_times.append(receive_time)
        try:
            payload = json.loads(message.data)
        except (json.JSONDecodeError, TypeError):
            self.parse_errors += 1
            return
        accepted = bool(payload.get("accepted"))
        self.accepted += int(accepted)
        self.dropped += int(not accepted)
        self.reasons[str(payload.get("reason", "missing"))] += 1
        self.valid_fractions.append(payload.get("valid_fraction"))
        self.baseline_valid_fractions.append(payload.get("baseline_valid_fraction"))
        self.median_delta_mm.append(payload.get("median_range_delta_mm"))
        self.p95_delta_mm.append(payload.get("p95_range_delta_mm"))
        self.temperature_c.append(payload.get("temperature_c"))
        self.vcsel_temperature_c.append(payload.get("vcsel_temperature_c"))
        self.elapsed_sec_series.append(
            receive_time - self.receive_times[0]
        )
        self.accepted_series.append(accepted)
        self.valid_fraction_series.append(payload.get("valid_fraction"))
        self.median_range_delta_mm_series.append(
            payload.get("median_range_delta_mm")
        )
        self.p95_range_delta_mm_series.append(
            payload.get("p95_range_delta_mm")
        )
        self.temperature_c_series.append(payload.get("temperature_c"))
        self.vcsel_temperature_c_series.append(
            payload.get("vcsel_temperature_c")
        )
        self.last_cumulative_accepted = payload.get("accepted_frames")
        self.last_cumulative_dropped = payload.get("dropped_frames")

    def result(self, requested_duration_sec, elapsed_sec):
        deltas = [
            current - previous
            for previous, current in zip(
                self.receive_times, self.receive_times[1:]
            )
            if current > previous
        ]
        message_count = len(self.receive_times)
        return {
            "schema": "smartwheel.xtm60_quality_topic_diagnostic.v1",
            "topic": self.topic,
            "requested_duration_sec": requested_duration_sec,
            "elapsed_sec": elapsed_sec,
            "message_count": message_count,
            "rate_hz": (
                1.0 / statistics.fmean(deltas) if deltas else None
            ),
            "max_receive_gap_sec": max(deltas) if deltas else None,
            "parse_errors": self.parse_errors,
            "accepted": self.accepted,
            "dropped": self.dropped,
            "acceptance_fraction": (
                self.accepted / (self.accepted + self.dropped)
                if self.accepted + self.dropped
                else None
            ),
            "reason_counts": dict(sorted(self.reasons.items())),
            "valid_fraction": _summary(self.valid_fractions),
            "baseline_valid_fraction": _summary(
                self.baseline_valid_fractions
            ),
            "median_range_delta_mm": _summary(self.median_delta_mm),
            "p95_range_delta_mm": _summary(self.p95_delta_mm),
            "temperature_c": _summary(self.temperature_c),
            "vcsel_temperature_c": _summary(self.vcsel_temperature_c),
            "elapsed_sec_series": self.elapsed_sec_series,
            "accepted_series": self.accepted_series,
            "valid_fraction_series": self.valid_fraction_series,
            "median_range_delta_mm_series": (
                self.median_range_delta_mm_series
            ),
            "p95_range_delta_mm_series": self.p95_range_delta_mm_series,
            "temperature_c_series": self.temperature_c_series,
            "vcsel_temperature_c_series": self.vcsel_temperature_c_series,
            "last_cumulative_accepted": self.last_cumulative_accepted,
            "last_cumulative_dropped": self.last_cumulative_dropped,
            "checks": {
                "messages_present": message_count > 0,
                "parse_errors_zero": self.parse_errors == 0,
                "rate_near_10_hz": (
                    deltas
                    and 8.0 <= 1.0 / statistics.fmean(deltas) <= 12.0
                ),
            },
        }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", required=True)
    parser.add_argument("--duration-sec", type=float, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    rclpy.init()
    node = QualityDiagnostic(args.topic)
    started = time.monotonic()
    try:
        while time.monotonic() - started < args.duration_sec:
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        elapsed = time.monotonic() - started
        result = node.result(args.duration_sec, elapsed)
        node.destroy_node()
        rclpy.shutdown()

    rendered = json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True)
    print(rendered)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(rendered + "\n", encoding="utf-8")
    return 0 if result["checks"]["messages_present"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
