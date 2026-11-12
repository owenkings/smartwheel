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
``reference_points`` is the cloud step alone; the 3D obstacle input (window C,
``perception/live_obstacles.py``) uses it for both lidars, each with its own mount.
"""
from __future__ import annotations

import math

import numpy as np

ANGLE_MIN, ANGLE_INCREMENT, BINS = -math.pi / 3, math.pi / 240, 161
RANGE_MIN, RANGE_MAX = 0.30, 8.0
SCAN_HEIGHT_M = (-0.5, 0.5)
REPRESENTATION = "representation=host_filtered"
MAX_LIDAR_TIME_OFFSET_MS = 250.0
# The live chain's offset is the approved value in window A's configuration (user decision 2026-11-08: 75 ms,
# on IMU arrival time); offline replay and evaluation tools keep 0 unless it is given explicitly.
LIDAR_TIME_OFFSET_CONFIG = "config/calibration/lidar_time_offset.json"
LIDAR_TIME_OFFSET_SNAPSHOT = "lidar_time_offset.json"     # copy kept in a frozen live configuration


def load_lidar_time_offset(path):
    """Approved lidar time offset (ms) and its provenance from a LIDAR_TIME_OFFSET file."""
    import hashlib
    import json
    from pathlib import Path
    path = Path(path).expanduser().resolve(strict=True)
    raw = path.read_bytes()
    value = json.loads(raw)
    if not isinstance(value, dict) or value.get("kind") != "LIDAR_TIME_OFFSET" or value.get("schema_version") != 1:
        raise ValueError("not a lidar time offset file: " + str(path))
    offset = value.get("lidar_time_offset_ms")
    if isinstance(offset, bool) or not isinstance(offset, (int, float)):
        raise ValueError("lidar_time_offset_ms must be a number: " + str(path))
    scan_stamp_ns(0, offset)
    return float(offset), {"path": str(path), "sha256": hashlib.sha256(raw).hexdigest(), "status": value.get("status")}


def scan_stamp_ns(source_stamp_ns, lidar_time_offset_ms=0.0):
    """Stamp of the published scan: the lidar frame's stamp minus an explicit lidar time offset.

    A positive offset means the lidar stamps are late against the wheel/IMU odometry (window A, 2026-11-07:
    left lidar about 75 ms). Stamping the scan earlier makes AMCL, the costmaps and the collision monitor read
    the odometry at the scan's capture time. Offset 0 keeps the recorded stamps.
    """
    offset = float(lidar_time_offset_ms)
    if not math.isfinite(offset) or abs(offset) > MAX_LIDAR_TIME_OFFSET_MS:
        raise ValueError("lidar time offset must be finite and within +-%g ms" % MAX_LIDAR_TIME_OFFSET_MS)
    return int(source_stamp_ns) - int(round(offset * 1e6))


class ScanRejected(ValueError):
    """The frame is not a host-filtered lidar frame of the expected lidar and sensor."""


def check_source(message, sensor_id, side="left"):
    """``side``: the lidar the frame must come from (frame id lidar_<side>); the scan is always the left one."""
    if message.side != side or message.cloud.header.frame_id != "lidar_" + side:
        raise ScanRejected(side + " lidar frame required")
    if sensor_id and message.sensor_id != sensor_id:
        raise ScanRejected("unexpected lidar sensor " + message.sensor_id)
    flags = [flag for flag in message.diagnostic_flags if flag.startswith("representation=")]
    if flags != [REPRESENTATION]:
        raise ScanRejected("host-filtered cloud required, got " + repr(flags))


def reference_points(message, filter_config, transforms, rotation):
    """(float32 N x 3 points in mapping_reference, host filter report) of one SourceFrame; not modified.

    The production host filter with this lidar's ``T_base_axle``, the finite points, rotated into the axle-aligned
    frame at the lidar origin and rounded to float32 as prepare_cloud does: the frozen frontend's cloud. Also used
    for the right lidar by the 3D obstacle input, with the right lidar's own transform and rotation.
    """
    from wc_runtime.mapping_filter import filter_cloud
    from ..rebuild import pointcloud_xyz
    filtered, report = filter_cloud(message.cloud, [(message, {})], transforms, filter_config)
    if not report.get("publishable", True):
        raise ScanRejected("host filter left no points")
    xyz = pointcloud_xyz(filtered)
    finite = np.isfinite(xyz).all(axis=1)
    if not finite.any():
        raise ScanRejected("cloud has no finite points after filtering")
    # prepare_cloud writes the rotated coordinates back into the cloud's FLOAT32 fields; keep that rounding.
    return (xyz[finite] @ np.asarray(rotation, dtype=float).T).astype(np.float32), report


SCAN_BODY_NEAR_M = 0.40          # --scan-inside-body drop: only points this close to the lidar (window C 11-12 15:12)


def scan_body(outline, lever, axle_height_m, max_height_m, near_m=SCAN_BODY_NEAR_M):
    """Settings of ``drop_inside_body`` (``--scan-inside-body drop``; 总管 2026-11-12 item 5): the body outline in
    base_link, the lidar origin in base_link (x, y, z), the floor ``axle_height_m`` below base_link, the height above
    the floor up to which the body reaches (window C's ObstacleFilter.body_max_height_m), and how close to the lidar
    (horizontally) a point inside the outline must be to be dropped."""
    outline = [[float(x), float(y)] for x, y in outline]
    lever = [float(v) for v in lever]
    if len(outline) < 3 or len(lever) != 3 or not all(math.isfinite(v) for p in outline for v in p):
        raise ValueError("the scan's body needs an outline of at least three points and the lidar's x, y, z")
    if not 0.0 < float(near_m) <= 1.0:
        raise ValueError("near_m must be within (0, 1] m")
    return {"outline": outline, "lever": lever, "axle_height_m": float(axle_height_m),
            "max_height_m": float(max_height_m), "near_m": float(near_m)}


def drop_inside_body(reference, body):
    """(points, number dropped): reference points (base_link axes at the lidar) without the lidar's near ring.

    Dropped are points inside the convex outline, up to the body's height above the floor (the 3D obstacle input's
    stage ``body``), and closer than ``near_m`` to the lidar horizontally. On 2026-11-11 the left lidar's near ring
    (0.31-0.35 m from it, 0.86 m above the floor, 0.15-0.20 m behind the outline's front edge) put 4 bins into the
    collision monitor's stop zone in one of 220 standing scans facing map -x (C.md 11-12 01:45). Dropping everything
    inside the outline also cancelled 7 real stops on 11-07: people and things against the front edge, 0.45-0.5 m
    from the lidar (C.md 11-12 15:12); with ``near_m`` only the ring's stop goes.
    """
    from ..perception.obstacle_cloud import inside_convex
    points = np.asarray(reference)
    p = points.astype(np.float64)
    base = p + np.asarray(body["lever"], dtype=float)
    inside = (inside_convex(base[:, :2], body["outline"]) & (base[:, 2] + body["axle_height_m"] <= body["max_height_m"])
              & (np.hypot(p[:, 0], p[:, 1]) < body["near_m"]))
    return points[~inside], int(inside.sum())


def scan_from_source(message, filter_config, transforms, rotation, height_band=SCAN_HEIGHT_M, body=None):
    """Return (ranges float32[161], stats) for one SourceFrame; the message is not modified.

    ``body`` (``scan_body``, None by default): the lidar's near ring inside the body outline (``drop_inside_body``)
    is dropped before the projection, so its bins keep their next return (stats ``inside_body_dropped``).
    """
    from ..amcl_replay import build_scan
    reference, report = reference_points(message, filter_config, transforms, rotation)
    dropped = None
    if body is not None:
        reference, dropped = drop_inside_body(reference, body)
    ranges, stats = build_scan(reference.astype(np.float64), *height_band)
    stats["filter_retained_points"] = report.get("retained_points")
    if dropped is not None:
        stats["inside_body_dropped"] = dropped
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
