#!/usr/bin/env python3
"""Save a bounded, fixed-window FAST-LIO cloud for diagnostics only.

This script is intentionally not the complete mapping-session exporter.  A
manual or production session must start ``map_products_node`` before mapping
and finish through ``/map_session/stop`` followed by ``/map_export/export``.
The fixed window here is retained only for short diagnostic captures.

Usage:
  python3 scripts/lio_save_cloud.py <out.ply> [--seconds N] [--voxel 0.05]
      [--max-points N] [--expected-frame-id FRAME] [--topic /cloud_registered]
"""

from __future__ import annotations

import argparse
import os
import sys
import tempfile
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

from smartwheel_map_products.session import SessionCapture, SessionCaptureError


def write_ply(path: str | Path, xyzi: np.ndarray) -> None:
    """Atomically write a binary diagnostic PLY."""

    output = Path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    n = xyzi.shape[0]
    header = (
        "ply\nformat binary_little_endian 1.0\n"
        "comment FAST-LIO diagnostic fixed-window cloud (xyz+intensity)\n"
        f"element vertex {n}\n"
        "property float x\nproperty float y\nproperty float z\nproperty float intensity\n"
        "end_header\n"
    ).encode("ascii")
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{output.name}.", suffix=".ply", dir=str(output.parent)
    )
    temporary = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "wb") as stream:
            descriptor = -1
            stream.write(header)
            stream.write(np.ascontiguousarray(xyzi, dtype="<f4").tobytes())
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, output)
    finally:
        if descriptor >= 0:
            os.close(descriptor)
        temporary.unlink(missing_ok=True)


def _stamp_seconds(message: PointCloud2) -> float:
    return float(message.header.stamp.sec) + float(message.header.stamp.nanosec) * 1e-9


class Accum(Node):
    def __init__(self, topic: str, voxel: float, max_points: int, expected_frame_id: str):
        super().__init__("lio_save_cloud_diagnostic")
        self.session = SessionCapture(voxel, max_points, expected_frame_id=expected_frame_id)
        self.session.start()
        self.error = ""
        self.create_subscription(PointCloud2, topic, self._cb, qos_profile_sensor_data)

    def _cb(self, msg: PointCloud2) -> None:
        if self.error or self.session.state != SessionCapture.CAPTURING:
            return
        try:
            names = [field.name for field in msg.fields]
            has_intensity = "intensity" in names
            fields = ("x", "y", "z", "intensity") if has_intensity else ("x", "y", "z")
            values = point_cloud2.read_points(msg, field_names=fields, skip_nans=True)
            if values is None or len(values) == 0:
                points = np.empty((0, 3), dtype=np.float64)
                intensity = np.empty((0,), dtype=np.float32) if has_intensity else None
            else:
                points = np.column_stack((values["x"], values["y"], values["z"])).astype(np.float64)
                intensity = (
                    values["intensity"].astype(np.float32)
                    if has_intensity
                    else None
                )
            origins = np.zeros_like(points)
            self.session.add_frame(
                _stamp_seconds(msg),
                msg.header.frame_id,
                points,
                origins,
                intensity,
            )
        except (SessionCaptureError, TypeError, ValueError, KeyError) as exc:
            self.error = str(exc)
            if self.session.state != SessionCapture.FAILED:
                self.session.fail(self.error)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("out")
    parser.add_argument("--seconds", type=float, default=6.0)
    parser.add_argument("--voxel", type=float, default=0.05)
    parser.add_argument("--max-points", type=int, default=500_000)
    parser.add_argument("--expected-frame-id", default="")
    parser.add_argument("--topic", default="/cloud_registered")
    args = parser.parse_args()
    if args.seconds <= 0.0:
        parser.error("--seconds must be positive")
    if args.voxel <= 0.0:
        parser.error("--voxel must be positive")
    if args.max_points <= 0:
        parser.error("--max-points must be positive")

    print(
        "WARNING: fixed-window diagnostic only; use map_products_node session "
        "stop/export for complete mapping saves.",
        file=sys.stderr,
    )
    rclpy.init()
    node = Accum(args.topic, args.voxel, args.max_points, args.expected_frame_id)
    runtime_error = ""
    try:
        deadline = time.monotonic() + args.seconds
        while rclpy.ok() and time.monotonic() < deadline and not node.error:
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, Exception) as exc:
        runtime_error = str(exc)
        if node.session.state != SessionCapture.FAILED:
            node.session.fail(runtime_error or "diagnostic capture interrupted")
    finally:
        if node.session.state == SessionCapture.CAPTURING:
            node.session.stop()
        summary = node.session.summary()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    if runtime_error or node.error or summary.state == SessionCapture.FAILED:
        print(
            f"ERROR: diagnostic capture failed: {runtime_error or node.error or summary.failure_reason}",
            file=sys.stderr,
        )
        return 2
    points, _origins, intensity = node.session.accumulator.arrays_with_intensity()
    if summary.frame_count == 0 or points.shape[0] == 0:
        print(
            f"ERROR: no non-empty frames on {args.topic} in {args.seconds}s",
            file=sys.stderr,
        )
        return 1
    amplitudes = np.zeros(points.shape[0], dtype=np.float32) if intensity is None else intensity
    xyzi = np.column_stack((points.astype(np.float32), amplitudes))
    try:
        write_ply(args.out, xyzi)
    except OSError as exc:
        print(f"ERROR: diagnostic PLY write failed: {exc}", file=sys.stderr)
        return 3
    print(
        f"wrote {args.out}: {points.shape[0]} points from {summary.frame_count} frames "
        f"(raw {summary.raw_point_count}, voxel {args.voxel}, "
        f"time span {summary.cloud_time_span_sec}s)"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
