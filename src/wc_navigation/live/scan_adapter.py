"""Left lidar SourceFrame -> 161-bin planar scan with the offline projection.

The steps reproduce the frozen single-left frontend: the driver's host-filtered
cloud (``source_frame_filtered``) passes the production host filter
(``wc_runtime.mapping_filter.filter_cloud``: range band, organized neighbour
support, ground height), is rotated into the axle-aligned frame at the lidar
origin (``mapping_reference``, as ``mapping_prior.prepare_cloud`` does), and is
projected by ``amcl_replay.build_scan``: 0.75-degree bins over +-60 degrees,
nearest return per bin, 0.30-8.0 m, height band -0.5..0.5 m in that frame.
Bins without a return stay +inf; nothing invents an obstacle or free space.
The ``laser`` frame is ``mapping_reference``: base_link -> laser is the planar
lever arm without rotation, as in the recorded shadow runs.
"""
from __future__ import annotations

import math

import numpy as np

ANGLE_MIN, ANGLE_INCREMENT, BINS = -math.pi / 3, math.pi / 240, 161
RANGE_MIN, RANGE_MAX = 0.30, 8.0
SCAN_HEIGHT_M = (-0.5, 0.5)
REPRESENTATION = "representation=host_filtered"
MAX_LIDAR_TIME_OFFSET_MS = 250.0


def scan_stamp_ns(source_stamp_ns, lidar_time_offset_ms=0.0):
    """Stamp of the published scan: the lidar frame's stamp minus an explicit lidar time offset.

    A positive offset means the lidar stamps are late against the wheel/IMU odometry (window A, 2026-11-07:
    left lidar about 75 ms). Stamping the scan earlier makes AMCL, the costmaps and the collision monitor read
    the odometry at the scan's capture time. The default 0 keeps the recorded stamps.
    """
    offset = float(lidar_time_offset_ms)
    if not math.isfinite(offset) or abs(offset) > MAX_LIDAR_TIME_OFFSET_MS:
        raise ValueError("lidar time offset must be finite and within +-%g ms" % MAX_LIDAR_TIME_OFFSET_MS)
    return int(source_stamp_ns) - int(round(offset * 1e6))


class ScanRejected(ValueError):
    """The frame is not a left host-filtered lidar frame of the configured sensor."""


def check_source(message, sensor_id):
    if message.side != "left" or message.cloud.header.frame_id != "lidar_left":
        raise ScanRejected("left lidar frame required")
    if sensor_id and message.sensor_id != sensor_id:
        raise ScanRejected("unexpected lidar sensor " + message.sensor_id)
    flags = [flag for flag in message.diagnostic_flags if flag.startswith("representation=")]
    if flags != [REPRESENTATION]:
        raise ScanRejected("host-filtered cloud required, got " + repr(flags))


def scan_from_source(message, filter_config, transforms, rotation, height_band=SCAN_HEIGHT_M):
    """Return (ranges float32[161], stats) for one SourceFrame; the message is not modified."""
    from wc_runtime.mapping_filter import filter_cloud
    from ..amcl_replay import build_scan
    from ..rebuild import pointcloud_xyz
    filtered, report = filter_cloud(message.cloud, [(message, {})], transforms, filter_config)
    if not report.get("publishable", True):
        raise ScanRejected("host filter left no points")
    xyz = pointcloud_xyz(filtered)
    finite = np.isfinite(xyz).all(axis=1)
    if not finite.any():
        raise ScanRejected("cloud has no finite points after filtering")
    # prepare_cloud writes the rotated coordinates back into the cloud's FLOAT32 fields; keep that rounding.
    reference = (xyz[finite] @ np.asarray(rotation, dtype=float).T).astype(np.float32).astype(np.float64)
    ranges, stats = build_scan(reference, *height_band)
    stats["filter_retained_points"] = report.get("retained_points")
    return ranges, stats


def laser_scan_message(ranges, stamp, frame_id="laser"):
    from sensor_msgs.msg import LaserScan
    message = LaserScan()
    message.header.stamp = stamp
    message.header.frame_id = frame_id
    message.angle_min = ANGLE_MIN
    message.angle_increment = ANGLE_INCREMENT
    message.angle_max = ANGLE_MIN + ANGLE_INCREMENT * (BINS - 1)
    message.time_increment, message.scan_time = 0.0, 0.0
    message.range_min, message.range_max = RANGE_MIN, RANGE_MAX
    message.ranges = [float(v) for v in ranges]
    return message
