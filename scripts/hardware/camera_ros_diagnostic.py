#!/usr/bin/env python3
"""Measure raw or compressed ROS camera topics without displaying them."""

import argparse
import json
import math
import statistics
import time
from pathlib import Path

import rclpy
import cv2
import numpy as np
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image


class CameraRosDiagnostic(Node):
    def __init__(self, topics, transport):
        super().__init__("camera_ros_diagnostic")
        self.started = time.monotonic()
        self.data = {
            topic: {
                "receive_times": [],
                "header_times_ns": [],
                "shapes": set(),
                "encodings": set(),
                "frame_ids": set(),
                "nonempty_messages": 0,
            }
            for topic in topics
        }
        message_type = Image if transport == "raw" else CompressedImage
        self.transport = transport
        self._subscriptions = [
            self.create_subscription(
                message_type, topic, self._callback(topic), qos_profile_sensor_data
            )
            for topic in topics
        ]

    def _callback(self, topic):
        def receive(message):
            entry = self.data[topic]
            entry["receive_times"].append(time.monotonic())
            entry["header_times_ns"].append(
                int(message.header.stamp.sec) * 1_000_000_000
                + int(message.header.stamp.nanosec)
            )
            if self.transport == "raw":
                entry["shapes"].add((int(message.height), int(message.width)))
                entry["encodings"].add(str(message.encoding))
            else:
                entry["encodings"].add(str(message.format).lower())
                if len(entry["shapes"]) == 0 and len(message.data) > 0:
                    encoded = np.frombuffer(message.data, dtype=np.uint8)
                    image = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
                    if image is not None:
                        entry["shapes"].add(
                            (int(image.shape[0]), int(image.shape[1]))
                        )
            entry["frame_ids"].add(str(message.header.frame_id))
            if len(message.data) > 0:
                entry["nonempty_messages"] += 1

        return receive


def _timing(values):
    deltas = [right - left for left, right in zip(values, values[1:])]
    return {
        "count": len(values),
        "span_sec": values[-1] - values[0] if len(values) > 1 else 0.0,
        "rate_hz": 1.0 / statistics.fmean(deltas) if deltas else None,
        "min_delta_sec": min(deltas) if deltas else None,
        "mean_delta_sec": statistics.fmean(deltas) if deltas else None,
        "max_delta_sec": max(deltas) if deltas else None,
        "nonpositive_deltas": sum(delta <= 0 for delta in deltas),
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--topics", nargs="+", required=True)
    parser.add_argument("--duration-sec", type=float, default=20.0)
    parser.add_argument("--min-rate-hz", type=float, default=1.0)
    parser.add_argument("--transport", choices=("raw", "compressed"), default="raw")
    parser.add_argument("--expected-width", type=int, default=640)
    parser.add_argument("--expected-height", type=int, default=480)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    rclpy.init()
    node = CameraRosDiagnostic(args.topics, args.transport)
    started = time.monotonic()
    try:
        while time.monotonic() - started < args.duration_sec:
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        elapsed = time.monotonic() - started
        minimum_window_coverage_fraction = 0.8
        minimum_message_count = max(
            2,
            int(
                math.ceil(
                    args.min_rate_hz
                    * args.duration_sec
                    * minimum_window_coverage_fraction
                )
            ),
        )
        topics = {}
        for topic, raw in node.data.items():
            receive = _timing(raw["receive_times"])
            header_seconds = [value / 1_000_000_000.0 for value in raw["header_times_ns"]]
            header = _timing(header_seconds)
            topics[topic] = {
                "message_count": len(raw["receive_times"]),
                "nonempty_messages": raw["nonempty_messages"],
                "shapes": [list(shape) for shape in sorted(raw["shapes"])],
                "encodings": sorted(raw["encodings"]),
                "frame_ids": sorted(raw["frame_ids"]),
                "receive_timing": receive,
                "header_timing": header,
                "checks": {
                    "messages_present": len(raw["receive_times"]) > 0,
                    "all_messages_nonempty": (
                        len(raw["receive_times"]) > 0
                        and raw["nonempty_messages"] == len(raw["receive_times"])
                    ),
                    "rate_above_minimum": (
                        receive["rate_hz"] is not None
                        and receive["rate_hz"] >= args.min_rate_hz
                    ),
                    "message_count_above_sustained_minimum": (
                        len(raw["receive_times"]) >= minimum_message_count
                    ),
                    "covers_requested_test_window": (
                        receive["span_sec"]
                        >= args.duration_sec
                        * minimum_window_coverage_fraction
                    ),
                    "header_timestamps_strictly_increasing": (
                        header["count"] > 1 and header["nonpositive_deltas"] == 0
                    ),
                    "expected_shape_and_encoding": (
                        raw["shapes"] == {(args.expected_height, args.expected_width)}
                        and (
                            raw["encodings"] == {"bgr8"}
                            if args.transport == "raw"
                            else any("jpeg" in value for value in raw["encodings"])
                        )
                    ),
                },
            }
        node.destroy_node()
        rclpy.shutdown()

    result = {
        "schema": "smartwheel.camera_ros_diagnostic.v2",
        "read_only": True,
        "requested_duration_sec": args.duration_sec,
        "elapsed_sec": elapsed,
        "minimum_rate_hz": args.min_rate_hz,
        "minimum_message_count": minimum_message_count,
        "minimum_window_coverage_fraction": minimum_window_coverage_fraction,
        "transport": args.transport,
        "expected_shape": [args.expected_height, args.expected_width],
        "topics": topics,
        "checks": {
            "all_topics_pass": all(
                all(entry["checks"].values()) for entry in topics.values()
            )
        },
    }
    rendered = json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True)
    print(rendered)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(rendered + "\n", encoding="utf-8")
    return 0 if result["checks"]["all_topics_pass"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
