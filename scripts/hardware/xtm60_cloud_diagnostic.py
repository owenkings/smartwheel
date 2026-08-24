#!/usr/bin/env python3
"""Collect bounded, read-only statistics from one XT-M60 PointCloud2 topic."""

import argparse
import json
import math
import statistics
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField


_NUMPY_TYPES = {
    PointField.INT8: "i1",
    PointField.UINT8: "u1",
    PointField.INT16: "i2",
    PointField.UINT16: "u2",
    PointField.INT32: "i4",
    PointField.UINT32: "u4",
    PointField.FLOAT32: "f4",
    PointField.FLOAT64: "f8",
}


def _summary(values):
    if not values:
        return {"count": 0}
    array = np.concatenate(values).astype(np.float64, copy=False)
    return {
        "count": int(array.size),
        "min": float(np.min(array)),
        "p05": float(np.percentile(array, 5)),
        "median": float(np.median(array)),
        "mean": float(np.mean(array)),
        "p95": float(np.percentile(array, 95)),
        "max": float(np.max(array)),
        "stddev": float(np.std(array)),
    }


def _rate_stats(times_ns):
    deltas = [
        (current - previous) / 1_000_000_000.0
        for previous, current in zip(times_ns, times_ns[1:])
    ]
    positive = [delta for delta in deltas if delta > 0]
    return {
        "count": len(times_ns),
        "nonpositive_deltas": sum(delta <= 0 for delta in deltas),
        "mean_delta_sec": statistics.fmean(positive) if positive else None,
        "min_delta_sec": min(deltas) if deltas else None,
        "max_delta_sec": max(deltas) if deltas else None,
        "rate_hz": 1.0 / statistics.fmean(positive) if positive else None,
    }


def _dtype_for(message):
    byte_order = ">" if message.is_bigendian else "<"
    names = []
    formats = []
    offsets = []
    for field in message.fields:
        scalar = _NUMPY_TYPES.get(field.datatype)
        if scalar is None:
            continue
        names.append(field.name)
        if field.count == 1:
            formats.append(byte_order + scalar)
        else:
            formats.append((byte_order + scalar, (field.count,)))
        offsets.append(field.offset)
    return np.dtype(
        {
            "names": names,
            "formats": formats,
            "offsets": offsets,
            "itemsize": message.point_step,
        }
    )


def _points_array(message):
    dtype = _dtype_for(message)
    rows = []
    packed_row_size = message.width * message.point_step
    raw = memoryview(message.data)
    for row in range(message.height):
        start = row * message.row_step
        rows.append(np.frombuffer(raw[start : start + packed_row_size], dtype=dtype))
    return np.concatenate(rows) if rows else np.empty(0, dtype=dtype)


class CloudDiagnostic(Node):
    def __init__(self, topic):
        super().__init__("xtm60_cloud_diagnostic")
        self.topic = topic
        self.messages = 0
        self.receive_times_ns = []
        self.header_times_ns = []
        self.frames = set()
        self.widths = set()
        self.heights = set()
        self.point_steps = set()
        self.row_steps = set()
        self.is_dense_values = set()
        self.fields = None
        self.total_points = 0
        self.valid_points = 0
        self.axes = {"x": [], "y": [], "z": []}
        self.ranges = []
        self.intensities = []
        self.frame_valid_fractions = []
        self.consecutive_range_delta_mm = []
        self.consecutive_range_delta_median_mm = []
        self.consecutive_changed_fraction_ge_50mm = []
        self.previous_range = None
        self.previous_valid = None
        self.create_subscription(PointCloud2, topic, self.on_cloud, qos_profile_sensor_data)

    def on_cloud(self, message):
        self.messages += 1
        self.receive_times_ns.append(time.monotonic_ns())
        self.header_times_ns.append(
            int(message.header.stamp.sec) * 1_000_000_000
            + int(message.header.stamp.nanosec)
        )
        self.frames.add(message.header.frame_id)
        self.widths.add(int(message.width))
        self.heights.add(int(message.height))
        self.point_steps.add(int(message.point_step))
        self.row_steps.add(int(message.row_step))
        self.is_dense_values.add(bool(message.is_dense))
        if self.fields is None:
            self.fields = [
                {
                    "name": field.name,
                    "offset": int(field.offset),
                    "datatype": int(field.datatype),
                    "count": int(field.count),
                }
                for field in message.fields
            ]

        points = _points_array(message)
        self.total_points += int(points.size)
        if not all(axis in points.dtype.names for axis in ("x", "y", "z")):
            return
        x = points["x"].astype(np.float64, copy=False)
        y = points["y"].astype(np.float64, copy=False)
        z = points["z"].astype(np.float64, copy=False)
        valid = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
        full_range = np.sqrt(x * x + y * y + z * z)
        self.frame_valid_fractions.append(
            float(np.count_nonzero(valid) / valid.size) if valid.size else 0.0
        )
        if (
            self.previous_range is not None
            and self.previous_range.size == full_range.size
            and self.previous_valid is not None
        ):
            pair_valid = valid & self.previous_valid
            if np.any(pair_valid):
                delta_mm = (
                    np.abs(full_range[pair_valid] - self.previous_range[pair_valid])
                    * 1000.0
                )
                self.consecutive_range_delta_mm.append(delta_mm)
                self.consecutive_range_delta_median_mm.append(
                    float(np.median(delta_mm))
                )
                self.consecutive_changed_fraction_ge_50mm.append(
                    np.asarray(
                        [np.count_nonzero(delta_mm >= 50.0) / delta_mm.size],
                        dtype=np.float64,
                    )
                )
        self.previous_range = full_range.copy()
        self.previous_valid = valid.copy()
        x = x[valid]
        y = y[valid]
        z = z[valid]
        self.valid_points += int(x.size)
        if x.size == 0:
            return
        self.axes["x"].append(x)
        self.axes["y"].append(y)
        self.axes["z"].append(z)
        self.ranges.append(np.sqrt(x * x + y * y + z * z))
        if "intensity" in points.dtype.names:
            intensity = points["intensity"].astype(np.float64, copy=False)[valid]
            self.intensities.append(intensity[np.isfinite(intensity)])

    def result(self, requested_duration_sec, elapsed_sec):
        field_names = [field["name"] for field in self.fields or []]
        valid_fraction = self.valid_points / self.total_points if self.total_points else 0.0
        ranges = _summary(self.ranges)
        intensities = _summary(self.intensities)
        receive_timing = _rate_stats(self.receive_times_ns)
        header_timing = _rate_stats(self.header_times_ns)
        result = {
            "schema": "smartwheel.xtm60_cloud_diagnostic.v1",
            "read_only": True,
            "topic": self.topic,
            "requested_duration_sec": requested_duration_sec,
            "elapsed_sec": elapsed_sec,
            "message_count": self.messages,
            "frame_ids": sorted(self.frames),
            "widths": sorted(self.widths),
            "heights": sorted(self.heights),
            "point_steps": sorted(self.point_steps),
            "row_steps": sorted(self.row_steps),
            "is_dense_values": sorted(self.is_dense_values),
            "fields": self.fields or [],
            "total_points": self.total_points,
            "valid_points": self.valid_points,
            "invalid_points": self.total_points - self.valid_points,
            "valid_fraction": valid_fraction,
            "receive_timing": receive_timing,
            "header_timing": header_timing,
            "axis_m": {axis: _summary(values) for axis, values in self.axes.items()},
            "range_m": ranges,
            "intensity": intensities,
            "frame_valid_fraction": _summary(
                [
                    np.asarray(self.frame_valid_fractions, dtype=np.float64)
                ]
                if self.frame_valid_fractions
                else []
            ),
            "consecutive_range_delta_mm": _summary(
                self.consecutive_range_delta_mm
            ),
            "consecutive_changed_fraction_ge_50mm": _summary(
                self.consecutive_changed_fraction_ge_50mm
            ),
            "receive_monotonic_sec_series": [
                value / 1_000_000_000.0 for value in self.receive_times_ns
            ],
            "frame_valid_fraction_series": self.frame_valid_fractions,
            "consecutive_range_delta_median_mm_series": (
                self.consecutive_range_delta_median_mm
            ),
            "consecutive_changed_fraction_ge_50mm_series": [
                float(value[0])
                for value in self.consecutive_changed_fraction_ge_50mm
            ],
        }
        result["checks"] = {
            "messages_present": self.messages > 0,
            "rate_near_10_hz": (
                receive_timing["rate_hz"] is not None
                and 8.0 <= receive_timing["rate_hz"] <= 12.0
            ),
            "header_timestamps_strictly_increasing": (
                header_timing["count"] > 1 and header_timing["nonpositive_deltas"] == 0
            ),
            "organized_160x60": self.widths == {160} and self.heights == {60},
            "xyzi_fields_present": all(name in field_names for name in ("x", "y", "z", "intensity")),
            "valid_points_present": self.valid_points > 0,
            "no_vendor_invalid_amplitude_sentinels": (
                intensities.get("max") is not None
                and intensities["max"] < 64000.0
            ),
            "intensity_within_documented_range": (
                intensities.get("min") is not None
                and intensities.get("max") is not None
                and 0.0 <= intensities["min"]
                and intensities["max"] <= 2039.0
            ),
            "meter_scale_plausible": (
                ranges.get("median") is not None
                and math.isfinite(ranges["median"])
                and 0.05 <= ranges["median"] <= 50.0
            ),
        }
        return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", default="/xtm60/left/points")
    parser.add_argument("--duration-sec", type=float, default=20.0)
    parser.add_argument(
        "--source",
        default="live",
        help="Evidence label such as 'live' or a rosbag path.",
    )
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()

    rclpy.init()
    node = CloudDiagnostic(args.topic)
    started = time.monotonic()
    try:
        while time.monotonic() - started < args.duration_sec:
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        elapsed = time.monotonic() - started
        result = node.result(args.duration_sec, elapsed)
        result["source"] = args.source
        node.destroy_node()
        rclpy.shutdown()

    rendered = json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True)
    print(rendered)
    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered + "\n", encoding="utf-8")
    return 0 if result["checks"]["messages_present"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
