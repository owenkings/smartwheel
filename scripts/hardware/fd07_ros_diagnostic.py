#!/usr/bin/env python3
"""Collect bounded statistics from four FD07-34R ROS Range topics."""

import argparse
import json
import statistics
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range


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


class RangeDiagnostic(Node):
    def __init__(self, indices):
        super().__init__("fd07_ros_diagnostic")
        self.data = {
            index: {
                "receive_times": [],
                "header_times": [],
                "ranges": [],
                "frame_ids": set(),
                "min_ranges": set(),
                "max_ranges": set(),
                "fields_of_view": set(),
                "radiation_types": set(),
            }
            for index in indices
        }
        self._subscriptions = [
            self.create_subscription(
                Range,
                f"/ultrasonic/range_{index}",
                lambda message, sensor_index=index: self.on_range(sensor_index, message),
                10,
            )
            for index in indices
        ]

    def on_range(self, index, message):
        entry = self.data[index]
        entry["receive_times"].append(time.monotonic())
        entry["header_times"].append(
            int(message.header.stamp.sec) * 1_000_000_000
            + int(message.header.stamp.nanosec)
        )
        entry["ranges"].append(float(message.range))
        entry["frame_ids"].add(message.header.frame_id)
        entry["min_ranges"].add(float(message.min_range))
        entry["max_ranges"].add(float(message.max_range))
        entry["fields_of_view"].add(float(message.field_of_view))
        entry["radiation_types"].add(int(message.radiation_type))


def timing(times):
    deltas = [b - a for a, b in zip(times, times[1:])]
    return {
        "count": len(times),
        "nonpositive_deltas": sum(delta <= 0 for delta in deltas),
        "delta_sec": summarize(deltas),
        "rate_hz": 1.0 / statistics.fmean(deltas) if deltas else None,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--indices", nargs="+", type=int, default=[0, 1, 2, 3])
    parser.add_argument("--duration-sec", type=float, default=15.0)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()

    rclpy.init()
    node = RangeDiagnostic(args.indices)
    started = time.monotonic()
    try:
        while time.monotonic() - started < args.duration_sec:
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        elapsed = time.monotonic() - started

    topics = {}
    for index, entry in node.data.items():
        receive = timing(entry["receive_times"])
        header_deltas = [
            (b - a) / 1_000_000_000.0
            for a, b in zip(entry["header_times"], entry["header_times"][1:])
        ]
        topics[str(index)] = {
            "topic": f"/ultrasonic/range_{index}",
            "frame_ids": sorted(entry["frame_ids"]),
            "ranges_m": summarize(entry["ranges"]),
            "receive_timing": receive,
            "header_nonpositive_deltas": sum(delta <= 0 for delta in header_deltas),
            "min_ranges": sorted(entry["min_ranges"]),
            "max_ranges": sorted(entry["max_ranges"]),
            "fields_of_view": sorted(entry["fields_of_view"]),
            "radiation_types": sorted(entry["radiation_types"]),
        }

    node.destroy_node()
    rclpy.shutdown()
    all_present = all(item["receive_timing"]["count"] > 0 for item in topics.values())
    all_monotonic = all(
        item["header_nonpositive_deltas"] == 0
        for item in topics.values()
    )
    result = {
        "schema": "smartwheel.fd07_ros_diagnostic.v1",
        "read_only": True,
        "requested_duration_sec": args.duration_sec,
        "elapsed_sec": elapsed,
        "topics": topics,
        "checks": {
            "all_four_topics_present": len(topics) == 4 and all_present,
            "all_header_timestamps_strictly_increasing": all_monotonic,
            "all_ranges_within_declared_bounds": all(
                item["ranges_m"].get("min") is not None
                and item["min_ranges"]
                and item["max_ranges"]
                and item["min_ranges"][0] <= item["ranges_m"]["min"]
                and item["ranges_m"]["max"] <= item["max_ranges"][0]
                for item in topics.values()
            ),
        },
    }

    rendered = json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True)
    print(rendered)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered + "\n", encoding="utf-8")
    return 0 if all(result["checks"].values()) else 2


if __name__ == "__main__":
    raise SystemExit(main())
