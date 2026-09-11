#!/usr/bin/env python3
"""Bounded full-bag integration replay for the wheel/IMU-primary architecture.

Run only on the Orin from a sourced ROS 2 + workspace shell.  The harness starts
no hardware and replays only the allowlisted recorded sensor/health topics plus
``/clock``.  It launches the wheel/IMU primary backend, RTAB-Map, and a TF-free
FAST-LIO shadow in one owned child process group, observes their products, then
archives and restores FAST-LIO's fixed debug-log directory by hash.

This is an integration/continuity diagnostic.  It cannot validate wheel scale,
physical extrinsics, the shared H30 as independent truth, RTAB map quality, or
passenger/navigation safety.
"""

from __future__ import annotations

import argparse
from bisect import bisect_right
from collections import Counter, defaultdict
import hashlib
import json
import math
import os
from pathlib import Path
import re
import shutil
import signal
import sqlite3
import subprocess
import sys
import time
import traceback
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import yaml


ROOT = Path("/home/nvidia/smartwheel")
DEFAULT_BAG = ROOT / "bags/diag/right_vertical_jump_20260910_210202"
ALLOWED_REPLAY_TOPICS = (
    "/xtm60/right/points",
    "/imu/data",
    "/wheel/odom",
    "/base/wheel_feedback_healthy",
)
OUTPUT_TOPICS = (
    "/odometry/filtered",
    "/wheel_mapping/odometry_gated",
    "/wheel_mapping/path",
    "/wheel_mapping/cloud_registered",
    "/rtabmap/cloud_map",
    "/rtabmap/grid_map",
    "/rtabmap/optimized_cloud",
    "/rtabmap/optimized_path",
    "/lio/shadow/odometry",
    "/lio/shadow/path",
    "/lio/shadow/cloud_registered",
    "/lio/consistency",
    "/lio/consistent",
    "/tf",
    "/tf_static",
)


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _hash_tree(root: Path) -> Dict[str, str]:
    if not root.exists():
        return {}
    return {
        str(path.relative_to(root)): _sha256(path)
        for path in sorted(root.rglob("*"))
        if path.is_file()
    }


def _write_json(path: Path, payload) -> None:
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")


def _inspect_bag(bag: Path) -> dict:
    bag = bag.resolve()
    metadata_path = bag / "metadata.yaml"
    if not metadata_path.is_file():
        raise ValueError(f"bag metadata not found: {metadata_path}")
    metadata = yaml.safe_load(metadata_path.read_text(encoding="utf-8"))[
        "rosbag2_bagfile_information"
    ]
    counts = {
        entry["topic_metadata"]["name"]: int(entry["message_count"])
        for entry in metadata["topics_with_message_count"]
    }
    missing = [topic for topic in ALLOWED_REPLAY_TOPICS if counts.get(topic, 0) <= 0]
    if missing:
        raise ValueError(f"required replay topics missing or empty: {missing}")
    first_record = {}
    last_record = {}
    for relative in metadata["relative_file_paths"]:
        database = (bag / relative).resolve()
        if database.parent != bag:
            raise ValueError(f"bag database escapes bag directory: {database}")
        with sqlite3.connect(f"file:{database}?mode=ro", uri=True) as connection:
            rows = connection.execute(
                "SELECT topics.name,MIN(messages.timestamp),MAX(messages.timestamp) "
                "FROM messages JOIN topics ON topics.id=messages.topic_id "
                "WHERE topics.name IN (?,?,?,?) GROUP BY topics.name",
                ALLOWED_REPLAY_TOPICS,
            )
            for topic, first, last in rows:
                # Retain integer nanoseconds until every split database has
                # been visited.  Converting inside this loop would compare
                # seconds from an earlier file with nanoseconds from a later
                # file and corrupt the extrema.
                first_record[topic] = min(first_record.get(topic, first), first)
                last_record[topic] = max(last_record.get(topic, last), last)
    start = metadata["starting_time"]["nanoseconds_since_epoch"] * 1.0e-9
    duration = metadata["duration"]["nanoseconds"] * 1.0e-9
    return {
        "bag": str(bag),
        "duration_sec": duration,
        "bag_start_sec": start,
        "topic_counts": {topic: counts[topic] for topic in ALLOWED_REPLAY_TOPICS},
        "first_record_offset_sec": {
            topic: first_record[topic] * 1.0e-9 - start
            for topic in ALLOWED_REPLAY_TOPICS
        },
        "last_record_offset_sec": {
            topic: last_record[topic] * 1.0e-9 - start
            for topic in ALLOWED_REPLAY_TOPICS
        },
        "database_files": list(metadata["relative_file_paths"]),
        "storage_identifier": metadata.get("storage_identifier"),
    }


def _process_conflicts() -> List[str]:
    process_table = subprocess.check_output(
        ["ps", "-eo", "pid,args"], text=True, errors="replace"
    )
    tokens = (
        "/xtm60_adapter_node ",
        "/imu_adapter_node ",
        "/zlac8030_driver_node ",
        "/fastlio_mapping ",
        "ros2 bag play ",
    )
    return [line.strip() for line in process_table.splitlines()
            if any(token in line for token in tokens)]


def _quaternion_rpy(q: Sequence[float]) -> Tuple[float, float, float]:
    x, y, z, w = (float(value) for value in q)
    norm = math.sqrt(x*x + y*y + z*z + w*w)
    if not math.isfinite(norm) or norm < 1.0e-12:
        return (math.nan, math.nan, math.nan)
    x, y, z, w = x/norm, y/norm, z/norm, w/norm
    roll = math.atan2(2.0*(w*x + y*z), 1.0 - 2.0*(x*x + y*y))
    sin_pitch = max(-1.0, min(1.0, 2.0*(w*y - z*x)))
    pitch = math.asin(sin_pitch)
    yaw = math.atan2(2.0*(w*z + x*y), 1.0 - 2.0*(y*y + z*z))
    return roll, pitch, yaw


def _unwrap(values: Sequence[float]) -> List[float]:
    if not values:
        return []
    result = [float(values[0])]
    for value in values[1:]:
        delta = math.atan2(math.sin(value - result[-1]), math.cos(value - result[-1]))
        result.append(result[-1] + delta)
    return result


def _integrate_gyro_z(
    samples: Sequence[Sequence[float]],
    start: float,
    end: float,
    rotation_base_from_imu: Sequence[float],
) -> Optional[float]:
    if end <= start or len(rotation_base_from_imu) != 9:
        return None
    # Transform angular velocity into base and linearly interpolate at the exact
    # odometry endpoints before trapezoidal integration.
    series = []
    for stamp, wx, wy, wz in samples:
        base_z = (rotation_base_from_imu[6] * wx +
                  rotation_base_from_imu[7] * wy +
                  rotation_base_from_imu[8] * wz)
        if all(math.isfinite(value) for value in (stamp, base_z)):
            series.append((stamp, base_z))
    if len(series) < 2 or start < series[0][0] or end > series[-1][0]:
        return None

    def at(stamp):
        for before, after in zip(series, series[1:]):
            if abs(stamp - before[0]) <= 1.0e-9:
                return before[1]
            if before[0] <= stamp <= after[0] and after[0] > before[0]:
                fraction = (stamp - before[0]) / (after[0] - before[0])
                return before[1] + fraction * (after[1] - before[1])
        if abs(stamp - series[-1][0]) <= 1.0e-9:
            return series[-1][1]
        return None

    clipped = [(start, at(start))]
    clipped.extend((stamp, value) for stamp, value in series if start < stamp < end)
    clipped.append((end, at(end)))
    if any(value is None for _, value in clipped):
        return None
    return sum(
        0.5 * (before[1] + after[1]) * (after[0] - before[0])
        for before, after in zip(clipped, clipped[1:])
    )


def _interpolate_series(
    series: Sequence[Sequence[float]], stamp: float
) -> Optional[float]:
    """Linearly interpolate a sorted ``(stamp, value)`` scalar series."""
    if len(series) < 2 or stamp < series[0][0] or stamp > series[-1][0]:
        return None
    index = bisect_right(series, stamp, key=lambda row: row[0])
    if index == len(series):
        return float(series[-1][1])
    before, after = series[max(0, index - 1)], series[index]
    if after[0] <= before[0]:
        return float(before[1])
    fraction = (stamp - before[0]) / (after[0] - before[0])
    return float(before[1]) + fraction * (float(after[1]) - float(before[1]))


def _integrate_series(
    series: Sequence[Sequence[float]], start: float, end: float
) -> Optional[float]:
    clean = [
        (float(row[0]), float(row[1]))
        for row in series
        if len(row) >= 2 and all(math.isfinite(float(value)) for value in row[:2])
    ]
    if len(clean) < 2 or end <= start:
        return None
    start_value = _interpolate_series(clean, start)
    end_value = _interpolate_series(clean, end)
    if start_value is None or end_value is None:
        return None
    clipped = [(start, start_value)]
    clipped.extend((stamp, value) for stamp, value in clean if start < stamp < end)
    clipped.append((end, end_value))
    return sum(
        0.5 * (before[1] + after[1]) * (after[0] - before[0])
        for before, after in zip(clipped, clipped[1:])
    )


def _pearson(pairs: Sequence[Sequence[float]]) -> Optional[float]:
    if len(pairs) < 3:
        return None
    xs = [float(pair[0]) for pair in pairs]
    ys = [float(pair[1]) for pair in pairs]
    mean_x = sum(xs) / len(xs)
    mean_y = sum(ys) / len(ys)
    xx = sum((value - mean_x) ** 2 for value in xs)
    yy = sum((value - mean_y) ** 2 for value in ys)
    if xx <= 1.0e-18 or yy <= 1.0e-18:
        return None
    return sum((x - mean_x) * (y - mean_y) for x, y in zip(xs, ys)) / math.sqrt(xx * yy)


def _wheel_imu_yaw_agreement(
    wheel_samples: Sequence[Sequence[float]],
    imu_samples: Sequence[Sequence[float]],
    rotation_base_from_imu: Sequence[float],
    start: float,
    end: float,
) -> dict:
    """Compare wheel-derived yaw rate with the independently recorded gyro."""
    if len(rotation_base_from_imu) != 9:
        return {"error": "base_to_imu_R_not_3x3"}
    wheel_series = sorted(
        (float(row[0]), float(row[2])) for row in wheel_samples
        if len(row) >= 3 and all(math.isfinite(float(value)) for value in (row[0], row[2]))
    )
    imu_series = sorted(
        (
            float(row[0]),
            rotation_base_from_imu[6] * float(row[1])
            + rotation_base_from_imu[7] * float(row[2])
            + rotation_base_from_imu[8] * float(row[3]),
        )
        for row in imu_samples
        if len(row) >= 4 and all(math.isfinite(float(value)) for value in row[:4])
    )
    paired = []
    for stamp, wheel_wz in wheel_series:
        if not start <= stamp <= end:
            continue
        imu_wz = _interpolate_series(imu_series, stamp)
        if imu_wz is not None:
            paired.append((wheel_wz, imu_wz))
    wheel_delta = _integrate_series(wheel_series, start, end)
    imu_delta = _integrate_series(imu_series, start, end)
    return {
        "window_start_sec": start,
        "window_end_sec": end,
        "paired_rate_samples": len(paired),
        "yaw_rate_pearson": _pearson(paired),
        "wheel_yaw_integral_deg": (
            math.degrees(wheel_delta) if wheel_delta is not None else None
        ),
        "transformed_imu_yaw_integral_deg": (
            math.degrees(imu_delta) if imu_delta is not None else None
        ),
        "wheel_minus_imu_yaw_integral_deg": (
            math.degrees(wheel_delta - imu_delta)
            if wheel_delta is not None and imu_delta is not None else None
        ),
    }


def _gid_hex(value) -> str:
    try:
        return bytes(value).hex()
    except (TypeError, ValueError):
        return str(value)


def _stamp_key(message) -> Optional[Tuple[int, int]]:
    if not hasattr(message, "header"):
        return None
    sec = int(message.header.stamp.sec)
    nanosec = int(message.header.stamp.nanosec)
    if sec < 0 or not 0 <= nanosec < 1_000_000_000 or (sec == 0 and nanosec == 0):
        return None
    return sec, nanosec


def _cloud_digests(message, *, hash_intensity: bool) -> dict:
    """Return structural, payload-change, and exact intensity-byte evidence."""
    width = int(message.width)
    height = int(message.height)
    point_step = int(message.point_step)
    row_step = int(message.row_step)
    payload = bytes(message.data)
    required_bytes = row_step * height
    structurally_nonempty = (
        width > 0 and height > 0 and point_step > 0 and row_step >= width * point_step
        and required_bytes > 0 and len(payload) >= required_bytes
    )
    result = {
        "nonempty": structurally_nonempty,
        "data_hash": hashlib.blake2b(payload, digest_size=16).hexdigest(),
        "intensity_hash": None,
        "intensity_error": None,
    }
    if not hash_intensity:
        return result
    intensity_fields = [field for field in message.fields if field.name == "intensity"]
    if len(intensity_fields) != 1:
        result["intensity_error"] = f"intensity_field_count:{len(intensity_fields)}"
        return result
    field = intensity_fields[0]
    datatype_bytes = {1: 1, 2: 1, 3: 2, 4: 2, 5: 4, 6: 4, 7: 4, 8: 8}
    element_size = datatype_bytes.get(int(field.datatype))
    field_bytes = (element_size or 0) * int(field.count)
    if (
        not structurally_nonempty or element_size is None or int(field.count) <= 0
        or int(field.offset) < 0 or int(field.offset) + field_bytes > point_step
    ):
        result["intensity_error"] = "invalid_intensity_layout_or_cloud_shape"
        return result
    digest = hashlib.blake2b(digest_size=16)
    for row in range(height):
        row_base = row * row_step
        for column in range(width):
            begin = row_base + column * point_step + int(field.offset)
            digest.update(payload[begin:begin + field_bytes])
    result["intensity_hash"] = digest.hexdigest()
    return result


def _launch_child(output: Path, database_path: Path) -> int:
    """Run the entire launch graph inside one parent-owned process group."""
    from ament_index_python.packages import get_package_share_directory
    from launch import LaunchDescription, LaunchService
    from launch.substitutions import Command
    from launch_ros.actions import Node
    from launch_ros.parameter_descriptions import ParameterValue
    from wheelchair_3d_mapping.diagnostic_layout import load_diagnostic_layout
    from wheelchair_3d_mapping.wheel_primary_pipeline import build_wheel_primary_nodes

    bringup = get_package_share_directory("wheelchair_bringup")
    description = get_package_share_directory("wheelchair_description")
    mapping = get_package_share_directory("wheelchair_3d_mapping")
    layout_path = os.path.join(
        description, "config", "diagnostic_measured_layout.yaml"
    )
    layout = load_diagnostic_layout(layout_path, radar="right")
    robot_description = ParameterValue(
        Command([
            "xacro ", os.path.join(description, "urdf", "wheelchair.urdf.xacro"),
            " diagnostic_measured_layout:=true",
            " diagnostic_layout_profile:=", layout_path,
        ]),
        value_type=str,
    )
    fast_remappings = [
        ("/Odometry", "/lio/shadow/odometry"),
        ("/path", "/lio/shadow/path"),
        ("/cloud_registered", "/lio/shadow/cloud_registered"),
        ("/cloud_registered_body", "/lio/shadow/cloud_registered_body"),
        ("/cloud_effected", "/lio/shadow/cloud_effected"),
        ("/Laser_map", "/lio/shadow/laser_map"),
        ("map_save", "/lio/shadow/map_save"),
    ]
    actions = [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description, "use_sim_time": True}],
        ),
        # The health gate, not robot_localization, owns odom->base_link.  This
        # prevents prediction-only TF from continuing after wheel/IMU health loss.
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="wheel_imu_ekf",
            output="screen",
            parameters=[
                os.path.join(bringup, "config", "right_diag_wheel_imu_ekf.yaml"),
                {"use_sim_time": True, "publish_tf": False},
            ],
        ),
        Node(
            package="wheelchair_3d_mapping",
            executable="lio_cloud_adapter",
            name="lio_shadow_cloud_adapter",
            output="screen",
            parameters=[{
                "use_sim_time": True,
                "input_topic": "/xtm60/right/points",
                "output_topic": "/lio/cloud_in",
                "restamp_to_now": False,
                "add_zero_time_field": False,
                "min_range": 0.3,
                "max_range": 12.0,
                "output_qos": "best_effort",
            }],
        ),
        Node(
            package="fast_lio",
            executable="fastlio_mapping",
            name="fastlio_shadow",
            output="screen",
            parameters=[
                os.path.join(mapping, "config", "xtm60_right_lio.yaml"),
                {
                    "use_sim_time": True,
                    "publish.tf_en": False,
                    "mapping.extrinsic_R": layout["extrinsic_R"],
                    "mapping.extrinsic_T": layout["extrinsic_T"],
                    "wheel_update.enabled": False,
                    "zupt.enabled": False,
                },
            ],
            remappings=fast_remappings,
        ),
    ]
    # The shared helper owns the single report-only consistency-monitor action,
    # already configured against the health-gated wheel/IMU pose.  Adding a
    # second copy here would create duplicate diagnostic publishers and make
    # state/count evidence ambiguous.
    actions.extend(build_wheel_primary_nodes(
        bringup, mapping, layout, str(database_path), use_sim_time=True
    ))
    _write_json(output / "launch_contract.json", {
        "wheel_primary_helper": (
            "wheelchair_3d_mapping.wheel_primary_pipeline.build_wheel_primary_nodes"
        ),
        "use_sim_time": True,
        "ekf_publish_tf": False,
        "expected_odom_base_owner": "wheel_pose_health_gate",
        "fast_publish_tf": False,
        "fast_wheel_update": False,
        "fast_zupt": False,
        "fast_remappings": fast_remappings,
        "consistency_reference": "/wheel_mapping/odometry_gated",
        "consistency_instances_expected": 1,
        "database_path": str(database_path),
        "layout_path": layout_path,
    })
    service = LaunchService(argv=[])
    service.include_launch_description(LaunchDescription(actions))
    return int(service.run())


class Observer:
    def __init__(self, node, qos_sensor, qos_tf, qos_tf_static) -> None:
        from diagnostic_msgs.msg import DiagnosticArray
        from nav_msgs.msg import OccupancyGrid, Odometry, Path
        from rosgraph_msgs.msg import Clock
        from sensor_msgs.msg import Imu, PointCloud2
        from std_msgs.msg import Bool
        from tf2_msgs.msg import TFMessage
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        reliable_output = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE)

        self.node = node
        self.topic_stats = defaultdict(lambda: {
            "count": 0, "first_header_sec": None, "last_header_sec": None,
            "max_header_gap_sec": 0.0, "header_regressions": 0, "nonfinite": 0,
        })
        self.odom = defaultdict(list)
        self.odom_pose_by_stamp = defaultdict(dict)
        self.imu = []
        self.wheel = []
        self.consistency_states = Counter()
        self.consistency_events = []
        self.consistent_bool = Counter()
        self.tf_edges = defaultdict(lambda: {
            "counts": Counter(),
            "publisher_gids": {"/tf": set(), "/tf_static": set()},
        })
        self.tf_publishers = {"/tf": {}, "/tf_static": {}}
        self.tf_samples = defaultdict(list)
        self.last_clock = None
        self.map_details = {}
        self.cloud_evidence = defaultdict(lambda: {
            "count": 0, "nonempty_count": 0, "invalid_count": 0,
            "data_hashes": set(), "intensity_hashes": set(),
            "intensity_errors": Counter(), "by_stamp": {},
        })
        self.path_evidence = defaultdict(lambda: {
            "count": 0, "max_poses": 0, "last_poses": 0,
            "pose_counts": set(), "frame_ids": set(),
        })
        self.grid_evidence = {
            "count": 0, "nonempty_count": 0, "invalid_count": 0,
            "data_hashes": set(),
        }

        # Pre-seed every input/output so a startup exception still produces a
        # complete fail-closed result rather than raising a secondary KeyError
        # while summarizing the original failure.
        for topic in set(ALLOWED_REPLAY_TOPICS) | set(OUTPUT_TOPICS) | {"/clock"}:
            self.topic_stats[topic]

        def subscribe(kind, topic, callback, qos=qos_sensor):
            node.create_subscription(kind, topic, callback, qos)

        def topic_callback(callback, topic):
            # Keep one explicit argument for the installed Humble executor;
            # per-message GIDs are collected by the separate C++ subscriber.
            def wrapped(message):
                callback(topic, message)
            return wrapped

        for topic in (
            "/odometry/filtered", "/wheel_mapping/odometry_gated",
            "/lio/shadow/odometry",
        ):
            subscribe(Odometry, topic, topic_callback(self.on_odom, topic),
                      reliable_output if topic != "/lio/shadow/odometry" else qos_sensor)
        subscribe(Odometry, "/wheel/odom", self.on_wheel)
        subscribe(Imu, "/imu/data", self.on_imu)
        for topic in (
            "/wheel_mapping/path", "/rtabmap/optimized_path", "/lio/shadow/path",
        ):
            subscribe(Path, topic, topic_callback(self.on_path, topic))
        for topic in (
            "/xtm60/right/points", "/wheel_mapping/cloud_registered",
            "/rtabmap/cloud_map", "/rtabmap/optimized_cloud",
            "/lio/shadow/cloud_registered",
        ):
            subscribe(PointCloud2, topic, topic_callback(self.on_cloud, topic),
                      reliable_output if topic.startswith("/rtabmap/") else qos_sensor)
        subscribe(OccupancyGrid, "/rtabmap/grid_map", self.on_grid)
        subscribe(DiagnosticArray, "/lio/consistency", self.on_consistency)
        subscribe(Bool, "/lio/consistent", self.on_consistent)
        subscribe(Bool, "/base/wheel_feedback_healthy",
                  lambda msg: self.on_headerless("/base/wheel_feedback_healthy", msg))
        subscribe(Clock, "/clock", self.on_clock)
        # This host's Humble executor invokes callbacks with only the message;
        # take_message metadata has timestamps but no publisher GID (verified).
        # Audit actual TF values and the complete publisher endpoint graph;
        # never invent a per-message publisher identity.
        subscribe(TFMessage, "/tf", lambda msg: self.on_tf("/tf", msg), qos_tf)
        subscribe(TFMessage, "/tf_static", lambda msg: self.on_tf(
            "/tf_static", msg), qos_tf_static)

    @staticmethod
    def stamp(message) -> Optional[float]:
        if not hasattr(message, "header"):
            return None
        stamp = message.header.stamp
        value = float(stamp.sec) + float(stamp.nanosec) * 1.0e-9
        return value if math.isfinite(value) and value > 0.0 else None

    def count(self, topic: str, message=None) -> Optional[float]:
        stats = self.topic_stats[topic]
        stats["count"] += 1
        stamp = self.stamp(message) if message is not None else None
        if stamp is not None:
            if stats["first_header_sec"] is None:
                stats["first_header_sec"] = stamp
            if stats["last_header_sec"] is not None:
                delta = stamp - stats["last_header_sec"]
                if delta < -1.0e-9:
                    stats["header_regressions"] += 1
                else:
                    stats["max_header_gap_sec"] = max(
                        stats["max_header_gap_sec"], delta
                    )
            stats["last_header_sec"] = stamp
        return stamp

    def on_headerless(self, topic, _message) -> None:
        self.count(topic)

    def on_clock(self, message) -> None:
        self.topic_stats["/clock"]["count"] += 1
        self.last_clock = float(message.clock.sec) + float(message.clock.nanosec) * 1.0e-9

    def on_odom(self, topic, message) -> None:
        stamp = self.count(topic, message)
        pose = message.pose.pose
        quaternion = (
            pose.orientation.x, pose.orientation.y,
            pose.orientation.z, pose.orientation.w,
        )
        roll, pitch, yaw = _quaternion_rpy(quaternion)
        values = (
            pose.position.x, pose.position.y, pose.position.z,
            *quaternion, roll, pitch, yaw,
            message.twist.twist.linear.x, message.twist.twist.linear.y,
            message.twist.twist.linear.z, message.twist.twist.angular.x,
            message.twist.twist.angular.y, message.twist.twist.angular.z,
            *message.pose.covariance, *message.twist.covariance,
        )
        if stamp is None or not all(math.isfinite(value) for value in values):
            self.topic_stats[topic]["nonfinite"] += 1
            return
        self.odom[topic].append([
            stamp, pose.position.x, pose.position.y, pose.position.z,
            roll, pitch, yaw, message.header.frame_id, message.child_frame_id,
        ])
        key = _stamp_key(message)
        if key is not None:
            self.odom_pose_by_stamp[topic][key] = (
                float(pose.position.x), float(pose.position.y), float(pose.position.z),
                float(pose.orientation.x), float(pose.orientation.y),
                float(pose.orientation.z), float(pose.orientation.w),
            )

    def on_wheel(self, message) -> None:
        stamp = self.count("/wheel/odom", message)
        if stamp is not None:
            self.wheel.append([
                stamp, message.twist.twist.linear.x,
                message.twist.twist.angular.z,
            ])

    def on_imu(self, message) -> None:
        stamp = self.count("/imu/data", message)
        angular = message.angular_velocity
        if stamp is not None and all(math.isfinite(value) for value in
                                     (angular.x, angular.y, angular.z)):
            self.imu.append([stamp, angular.x, angular.y, angular.z])

    def on_path(self, topic, message) -> None:
        self.count(topic, message)
        self.map_details[topic] = {"poses": len(message.poses)}
        evidence = self.path_evidence[topic]
        evidence["count"] += 1
        evidence["last_poses"] = len(message.poses)
        evidence["max_poses"] = max(evidence["max_poses"], len(message.poses))
        evidence["pose_counts"].add(len(message.poses))
        evidence["frame_ids"].add(message.header.frame_id)

    def on_cloud(self, topic, message) -> None:
        self.count(topic, message)
        field_names = [field.name for field in message.fields]
        digests = _cloud_digests(
            message,
            hash_intensity=topic in (
                "/xtm60/right/points", "/wheel_mapping/cloud_registered"
            ),
        )
        evidence = self.cloud_evidence[topic]
        evidence["count"] += 1
        evidence["nonempty_count"] += int(digests["nonempty"])
        evidence["invalid_count"] += int(not digests["nonempty"])
        evidence["data_hashes"].add(digests["data_hash"])
        if digests["intensity_hash"] is not None:
            evidence["intensity_hashes"].add(digests["intensity_hash"])
        if digests["intensity_error"] is not None:
            evidence["intensity_errors"][digests["intensity_error"]] += 1
        key = _stamp_key(message)
        if key is not None:
            evidence["by_stamp"][key] = {
                "data_hash": digests["data_hash"],
                "intensity_hash": digests["intensity_hash"],
            }
        self.map_details[topic] = {
            "width": int(message.width), "height": int(message.height),
            "points": int(message.width) * int(message.height),
            "fields": field_names, "has_intensity": "intensity" in field_names,
            "data_bytes": len(message.data),
        }

    def on_grid(self, message) -> None:
        self.count("/rtabmap/grid_map", message)
        width = int(message.info.width)
        height = int(message.info.height)
        valid = width > 0 and height > 0 and len(message.data) == width * height
        payload = bytes((int(value) & 0xff) for value in message.data)
        self.grid_evidence["count"] += 1
        self.grid_evidence["nonempty_count"] += int(valid)
        self.grid_evidence["invalid_count"] += int(not valid)
        self.grid_evidence["data_hashes"].add(
            hashlib.blake2b(payload, digest_size=16).hexdigest()
        )
        self.map_details["/rtabmap/grid_map"] = {
            "width": width, "height": height,
            "resolution": float(message.info.resolution), "cells": len(message.data),
        }

    def on_consistency(self, message) -> None:
        stamp = self.count("/lio/consistency", message)
        for status in message.status:
            values = {item.key: item.value for item in status.values}
            state = values.get("state", status.message or "UNKNOWN")
            self.consistency_states[state] += 1
            if not self.consistency_events or self.consistency_events[-1][1] != state:
                self.consistency_events.append([
                    stamp, state, values.get("reasons", ""),
                    (status.level[0] if isinstance(status.level, bytes) else int(status.level))
                ])

    def on_consistent(self, message) -> None:
        self.count("/lio/consistent")
        self.consistent_bool[str(bool(message.data)).lower()] += 1

    def on_tf(self, topic, message, info=None) -> None:
        self.topic_stats[topic]["count"] += 1
        gid = _gid_hex(info.publisher_gid) if info is not None else "unavailable"
        for transform in message.transforms:
            edge = f"{transform.header.frame_id}->{transform.child_frame_id}"
            entry = self.tf_edges[edge]
            entry["counts"][topic] += 1
            entry["publisher_gids"][topic].add(gid)
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            values = (
                float(translation.x), float(translation.y), float(translation.z),
                float(rotation.x), float(rotation.y), float(rotation.z),
                float(rotation.w),
            )
            self.tf_samples[edge].append({
                "stamp": _stamp_key(transform),
                "topic": topic,
                "gid": gid,
                "values": values,
                "finite": all(math.isfinite(value) for value in values),
            })

    def refresh_tf_publishers(self) -> None:
        for topic in ("/tf", "/tf_static"):
            for endpoint in self.node.get_publishers_info_by_topic(topic):
                gid = _gid_hex(endpoint.endpoint_gid)
                namespace = endpoint.node_namespace.rstrip("/")
                name = f"{namespace}/{endpoint.node_name}" if namespace else f"/{endpoint.node_name}"
                self.tf_publishers[topic][gid] = name

    def serializable_tf(self) -> dict:
        result = {}
        for edge, entry in self.tf_edges.items():
            owners_by_topic = {}
            gids_by_topic = {}
            for topic in ("/tf", "/tf_static"):
                gids = sorted(entry["publisher_gids"][topic])
                gids_by_topic[topic] = gids
                owners_by_topic[topic] = sorted({
                    self.tf_publishers[topic].get(gid, f"unknown_gid:{gid}")
                    for gid in gids
                })
            result[edge] = {
                "count": sum(entry["counts"].values()),
                "dynamic": entry["counts"]["/tf"] > 0,
                "static": entry["counts"]["/tf_static"] > 0,
                "counts_by_topic": dict(entry["counts"]),
                "publisher_gids_by_topic": gids_by_topic,
                "owners_by_topic": owners_by_topic,
                "dynamic_owners": owners_by_topic["/tf"],
                "static_owners": owners_by_topic["/tf_static"],
            }
        return result

    def serializable_cloud_evidence(self) -> dict:
        result = {}
        for topic, entry in self.cloud_evidence.items():
            stamped = list(entry["by_stamp"].items())
            result[topic] = {
                "count": entry["count"],
                "nonempty_count": entry["nonempty_count"],
                "invalid_count": entry["invalid_count"],
                "unique_data_hashes": len(entry["data_hashes"]),
                "unique_intensity_hashes": len(entry["intensity_hashes"]),
                "intensity_errors": dict(entry["intensity_errors"]),
                "unique_header_stamps": len(entry["by_stamp"]),
                "first_data_hash": stamped[0][1]["data_hash"] if stamped else None,
                "last_data_hash": stamped[-1][1]["data_hash"] if stamped else None,
                "first_intensity_hash": (
                    stamped[0][1]["intensity_hash"] if stamped else None
                ),
                "last_intensity_hash": (
                    stamped[-1][1]["intensity_hash"] if stamped else None
                ),
            }
        return result

    def serializable_path_evidence(self) -> dict:
        result = {}
        for topic, entry in self.path_evidence.items():
            result[topic] = {
                "count": entry["count"],
                "max_poses": entry["max_poses"],
                "last_poses": entry["last_poses"],
                "unique_pose_counts": len(entry["pose_counts"]),
                "frame_ids": sorted(entry["frame_ids"]),
            }
        return result


def _odom_summary(samples: Sequence[Sequence]) -> dict:
    if not samples:
        return {"count": 0}
    numeric = [row[:7] for row in samples]
    yaws = _unwrap([row[6] for row in numeric])
    return {
        "count": len(samples),
        "first_stamp": samples[0][0], "last_stamp": samples[-1][0],
        "first_xyz": list(samples[0][1:4]), "last_xyz": list(samples[-1][1:4]),
        "min_xyz": [min(row[index] for row in samples) for index in (1, 2, 3)],
        "max_xyz": [max(row[index] for row in samples) for index in (1, 2, 3)],
        "max_abs_z_m": max(abs(row[3]) for row in samples),
        "max_abs_roll_deg": math.degrees(max(abs(row[4]) for row in samples)),
        "max_abs_pitch_deg": math.degrees(max(abs(row[5]) for row in samples)),
        "unwrapped_yaw_change_deg": math.degrees(yaws[-1] - yaws[0]),
        "parent_frames": sorted({row[7] for row in samples}),
        "child_frames": sorted({row[8] for row in samples}),
    }


def _terminate_owned(processes) -> dict:
    before = {label: process.poll() for label, process in processes}
    for _label, process in reversed(processes):
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
    deadline = time.monotonic() + 8.0
    for _label, process in reversed(processes):
        if process.poll() is not None:
            continue
        try:
            process.wait(timeout=max(0.1, deadline - time.monotonic()))
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=3.0)
    return {
        "exit_before_cleanup": before,
        "exit_after_cleanup": {label: process.returncode for label, process in processes},
    }


def _archive_and_restore_fast_logs(log_root: Path, output: Path, original_hashes: dict) -> dict:
    generated = output / "generated_fast_logs"
    if generated.exists():
        raise RuntimeError("generated FAST log archive unexpectedly already exists")
    shutil.copytree(log_root, generated)
    generated_hashes = _hash_tree(log_root)
    _write_json(output / "generated_fast_log_hashes.json", generated_hashes)
    conflicts = _process_conflicts()
    fast_conflicts = [line for line in conflicts if "/fastlio_mapping " in line]
    if fast_conflicts:
        raise RuntimeError(
            "FAST process appeared before log restoration; backup retained: " +
            repr(fast_conflicts)
        )
    original_backup = output / "original_fast_logs"
    concurrent = []
    for relative, generated_hash in generated_hashes.items():
        current = log_root / relative
        if current.is_file() and _sha256(current) != generated_hash:
            concurrent.append(relative)
    if concurrent:
        raise RuntimeError(f"concurrent FAST log changes detected: {concurrent}")
    for relative in original_hashes:
        target = log_root / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(original_backup / relative, target)
    restored = _hash_tree(log_root)
    return {
        "original_hashes_restored": all(
            restored.get(relative) == digest
            for relative, digest in original_hashes.items()
        ),
        "original_hashes": original_hashes,
        "generated_hashes": generated_hashes,
        "extra_files_left_in_fixed_log_directory": sorted(
            set(restored) - set(original_hashes)
        ),
        "generated_archive": str(generated),
    }


def _main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", type=Path, default=DEFAULT_BAG)
    parser.add_argument("--domain", type=int, choices=[91], default=91)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--timeout-sec", type=float, default=200.0)
    parser.add_argument("--inspect-only", action="store_true")
    parser.add_argument("--tf-audit-executable", type=Path,
                        default=ROOT / "auto_test/tf_audit_build/cmake-build/smartwheel_tf_audit")
    parser.add_argument("--launch-child", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument("--database-path", type=Path, help=argparse.SUPPRESS)
    args = parser.parse_args()

    if args.launch_child:
        if args.output is None or args.database_path is None:
            raise ValueError("launch child requires output and database paths")
        return _launch_child(args.output.resolve(), args.database_path.resolve())
    if not math.isfinite(args.timeout_sec) or not 185.0 <= args.timeout_sec <= 240.0:
        raise ValueError("timeout-sec must be within 185..240 seconds for the full bag")

    conflicts = _process_conflicts()
    if conflicts:
        raise RuntimeError("existing hardware/FAST/bag-player process: " + repr(conflicts))
    bag = args.bag.resolve()
    preflight = _inspect_bag(bag)
    print(json.dumps(preflight, ensure_ascii=False, indent=2), flush=True)
    if args.inspect_only:
        return 0
    if not args.tf_audit_executable.is_file():
        raise ValueError("Compile scripts/mapping/tf_audit with CMake first; TF audit executable is missing")
    if not 165.0 <= preflight["duration_sec"] <= 180.0:
        raise ValueError(
            "this harness requires the complete 172 s diagnostic recording; "
            f"found duration {preflight['duration_sec']:.3f} s"
        )

    output = (args.output or ROOT / "auto_test" /
              time.strftime("wheel_primary_full_replay_%Y%m%d_%H%M%S")).resolve()
    if output.exists():
        raise ValueError(f"evidence output exists; refusing overwrite: {output}")
    output.mkdir(parents=True)
    database_path = output / "wheel_primary_rtabmap.db"
    os.environ.update({
        "ROS_DOMAIN_ID": str(args.domain),
        "ROS_LOCALHOST_ONLY": "1",
        "ROS_LOG_DIR": str(output / "ros_logs"),
        "RCUTILS_COLORIZED_OUTPUT": "0",
    })
    _write_json(output / "preflight.json", preflight)

    import rclpy
    from rclpy.qos import (
        DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy,
        qos_profile_sensor_data,
    )
    from ament_index_python.packages import get_package_share_directory
    from wheelchair_3d_mapping.diagnostic_layout import load_diagnostic_layout

    rclpy.init(args=[])
    node = rclpy.create_node("wheel_primary_full_replay_observer")
    node.set_parameters([rclpy.parameter.Parameter(
        "use_sim_time", rclpy.Parameter.Type.BOOL, True
    )])
    for _ in range(10):
        rclpy.spin_once(node, timeout_sec=0.1)
    occupied = [
        f"{namespace.rstrip('/')}/{name}" if namespace != "/" else f"/{name}"
        for name, namespace in node.get_node_names_and_namespaces()
        if name != node.get_name()
    ]
    if occupied:
        node.destroy_node()
        rclpy.shutdown()
        raise RuntimeError(f"ROS domain {args.domain} is occupied: {occupied}")

    tf_qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST, depth=100,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )
    tf_static_qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST, depth=100,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
    observer_sensor_qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST, depth=200,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )
    observer = Observer(node, observer_sensor_qos, tf_qos, tf_static_qos)
    layout_path = (
        Path(get_package_share_directory("wheelchair_description"))
        / "config"
        / "diagnostic_measured_layout.yaml"
    )
    layout = load_diagnostic_layout(str(layout_path), radar="right")
    _write_json(output / "exact_layout.json", layout)

    log_root = ROOT / "src/third_party/FAST_LIO_ROS2/Log"
    if not log_root.is_dir():
        node.destroy_node()
        rclpy.shutdown()
        raise RuntimeError(f"FAST fixed Log directory missing: {log_root}")
    original_hashes = _hash_tree(log_root)
    shutil.copytree(log_root, output / "original_fast_logs")
    _write_json(output / "original_fast_log_hashes.json", original_hashes)

    processes = []
    handles = []
    runtime_error = None
    cleanup = {}
    restoration = {}
    started = time.monotonic()

    def launch(label: str, command: Sequence[str]):
        handle = (output / f"{label}.log").open("w", encoding="utf-8")
        handles.append(handle)
        process = subprocess.Popen(
            list(command), stdout=handle, stderr=subprocess.STDOUT,
            start_new_session=True, env=os.environ.copy(),
        )
        processes.append((label, process))
        print(f"started owned {label} process-group leader PID={process.pid}", flush=True)
        return process

    try:
        launch("tf_audit", [str(args.tf_audit_executable.resolve())])
        child = launch("launch_graph", [
            sys.executable, str(Path(__file__).resolve()),
            "--launch-child", "--domain", str(args.domain),
            "--output", str(output), "--database-path", str(database_path),
        ])
        launch_deadline = time.monotonic() + 8.0
        while time.monotonic() < launch_deadline:
            if child.poll() is not None:
                raise RuntimeError(f"launch graph exited during startup: {child.returncode}")
            rclpy.spin_once(node, timeout_sec=0.1)
            observer.refresh_tf_publishers()
        graph_before_play = sorted(
            [f"{namespace.rstrip('/')}/{name}" for name, namespace
             in node.get_node_names_and_namespaces()]
        )
        _write_json(output / "graph_before_play.json", graph_before_play)

        player_command = [
            "ros2", "bag", "play", str(bag),
            "--clock", "100", "--rate", "1.0",
            "--disable-keyboard-controls", "--read-ahead-queue-size", "3000",
            "--topics", *ALLOWED_REPLAY_TOPICS,
        ]
        _write_json(output / "player_command.json", player_command)
        player = launch("player", player_command)
        deadline = started + args.timeout_sec
        next_report = time.monotonic() + 10.0
        while player.poll() is None:
            if time.monotonic() >= deadline:
                raise TimeoutError("full replay exceeded owned-process deadline")
            if child.poll() is not None:
                raise RuntimeError(f"launch graph exited during replay: {child.returncode}")
            rclpy.spin_once(node, timeout_sec=0.1)
            if time.monotonic() >= next_report:
                observer.refresh_tf_publishers()
                print(
                    f"elapsed={time.monotonic()-started:.1f}s "
                    f"counts={ {topic: values['count'] for topic, values in observer.topic_stats.items()} }", flush=True,
                )
                next_report += 10.0
        if player.returncode != 0:
            raise RuntimeError(f"bag player exited {player.returncode}")
        drain_deadline = min(deadline, time.monotonic() + 8.0)
        while time.monotonic() < drain_deadline:
            if child.poll() is not None:
                raise RuntimeError(f"launch graph exited during drain: {child.returncode}")
            rclpy.spin_once(node, timeout_sec=0.1)
        observer.refresh_tf_publishers()
        _write_json(output / "graph_after_play.json", sorted(
            [f"{namespace.rstrip('/')}/{name}" for name, namespace
             in node.get_node_names_and_namespaces()]
        ))
    except (Exception, KeyboardInterrupt) as exc:
        runtime_error = f"{type(exc).__name__}: {exc}"
        runtime_traceback = traceback.format_exc()
        # Preserve the primary failure before cleanup/restoration can surface a
        # second exception.  This file is intentionally written while every
        # owned process group is still registered for the finally block.
        _write_json(output / "runtime_error.json", {
            "runtime_error": runtime_error,
            "traceback": runtime_traceback,
            "elapsed_sec": time.monotonic() - started,
        })
        print(f"RUNTIME_ERROR={runtime_error}", flush=True)
        print(runtime_traceback, flush=True)
    finally:
        cleanup = _terminate_owned(processes)
        for handle in handles:
            handle.close()
        try:
            restoration = _archive_and_restore_fast_logs(
                log_root, output, original_hashes
            )
        except Exception as exc:
            restoration = {"error": f"{type(exc).__name__}: {exc}"}
            if runtime_error is None:
                runtime_error = "FAST log restoration failed: " + restoration["error"]

    topic_stats = {topic: dict(values) for topic, values in observer.topic_stats.items()}
    tf_edges = observer.serializable_tf()
    cpp_endpoints = {}
    cpp_tf_rows = []
    for line in (output / "tf_audit.log").read_text(errors="replace").splitlines():
        fields = line.split("\t")
        if len(fields) == 3 and fields[0] == "ENDPOINT":
            cpp_endpoints[fields[1]] = fields[2]
        elif len(fields) == 6 and fields[0] == "TF":
            cpp_tf_rows.append(fields)
    cpp_edge_gids = defaultdict(set)
    cpp_edge_counts = Counter()
    for fields in cpp_tf_rows:
        edge = fields[1] + "->" + fields[2]
        cpp_edge_gids[edge].add(fields[5])
        cpp_edge_counts[edge] += 1
    cpp_tf_evidence = {
        edge: {"count": cpp_edge_counts[edge], "publisher_gids": sorted(gids),
               "owners": sorted({cpp_endpoints.get(gid, "UNKNOWN") for gid in gids})}
        for edge, gids in cpp_edge_gids.items()
    }
    cpp_tf_pass = (
        set(cpp_tf_evidence) == {"odom->base_link", "map->odom"}
        and cpp_tf_evidence["odom->base_link"]["owners"] == ["/wheel_pose_health_gate"]
        and cpp_tf_evidence["map->odom"]["owners"] == ["/rtabmap"]
        and all(len(value["publisher_gids"]) == 1 for value in cpp_tf_evidence.values())
    )
    observed_cloud_topics = (
        "/xtm60/right/points", "/wheel_mapping/cloud_registered",
        "/rtabmap/cloud_map", "/rtabmap/optimized_cloud",
        "/lio/shadow/cloud_registered",
    )
    observed_path_topics = (
        "/wheel_mapping/path", "/rtabmap/optimized_path", "/lio/shadow/path",
    )
    for topic in observed_cloud_topics:
        observer.cloud_evidence[topic]
    for topic in observed_path_topics:
        observer.path_evidence[topic]
    cloud_evidence = observer.serializable_cloud_evidence()
    path_evidence = observer.serializable_path_evidence()
    grid_evidence = {
        "count": observer.grid_evidence["count"],
        "nonempty_count": observer.grid_evidence["nonempty_count"],
        "invalid_count": observer.grid_evidence["invalid_count"],
        "unique_data_hashes": len(observer.grid_evidence["data_hashes"]),
    }
    primary = _odom_summary(observer.odom["/wheel_mapping/odometry_gated"])
    unchecked = _odom_summary(observer.odom["/odometry/filtered"])
    shadow = _odom_summary(observer.odom["/lio/shadow/odometry"])
    gyro_integral = None
    gyro_yaw_error = None
    wheel_imu_yaw = {"error": "primary_window_unavailable"}
    gyro_comparison_window = None
    if primary.get("count", 0) >= 2 and observer.imu and observer.wheel:
        comparison_start = max(primary["first_stamp"], observer.imu[0][0], observer.wheel[0][0])
        comparison_end = min(primary["last_stamp"], observer.imu[-1][0], observer.wheel[-1][0])
        gyro_comparison_window = [comparison_start, comparison_end]
        gyro_integral = _integrate_gyro_z(
            observer.imu, comparison_start, comparison_end,
            layout["base_to_imu_R"],
        )
        wheel_imu_yaw = _wheel_imu_yaw_agreement(
            observer.wheel,
            observer.imu,
            layout["base_to_imu_R"],
            comparison_start,
            comparison_end,
        )
        if gyro_integral is not None:
            pose_rows = observer.odom["/wheel_mapping/odometry_gated"]
            yaw_series = list(zip([row[0] for row in pose_rows], _unwrap([row[6] for row in pose_rows])))
            initial_yaw = _interpolate_series(yaw_series, comparison_start)
            final_yaw = _interpolate_series(yaw_series, comparison_end)
            if initial_yaw is not None and final_yaw is not None:
                gyro_yaw_error = final_yaw - initial_yaw - gyro_integral

    last_sensor_stamp = min(
        topic_stats.get("/imu/data", {}).get("last_header_sec") or math.inf,
        topic_stats.get("/wheel/odom", {}).get("last_header_sec") or math.inf,
        topic_stats.get("/xtm60/right/points", {}).get("last_header_sec") or math.inf,
    )
    primary_tail_lag = None
    if primary.get("last_stamp") is not None and math.isfinite(last_sensor_stamp):
        primary_tail_lag = last_sensor_stamp - primary["last_stamp"]

    fast_log = (output / "launch_graph.log").read_text(
        encoding="utf-8", errors="replace"
    ) if (output / "launch_graph.log").exists() else ""
    lidar_counters = [tuple(map(int, match)) for match in re.findall(
        r"accepted=(\d+) rejected=(\d+).*?projected=(\d+) adapted=(\d+)", fast_log
    )]

    def dynamic_owners(edge):
        return tf_edges.get(edge, {}).get("dynamic_owners", [])

    camera_body_present = "camera_init->body" in tf_edges
    base_parents = sorted({edge.split("->", 1)[0] for edge in tf_edges
                           if edge.endswith("->base_link") and tf_edges[edge]["dynamic"]})
    odom_parents = sorted({edge.split("->", 1)[0] for edge in tf_edges
                           if edge.endswith("->odom") and tf_edges[edge]["dynamic"]})
    odom_base_owners = dynamic_owners("odom->base_link")
    map_odom_owners = dynamic_owners("map->odom")

    # The health gate copies each accepted pose verbatim into odom->base_link.
    # Verify values and exact header stamps, not only graph edge ownership.
    gated_poses = observer.odom_pose_by_stamp["/wheel_mapping/odometry_gated"]
    dynamic_odom_base = [
        sample for sample in observer.tf_samples["odom->base_link"]
        if sample["topic"] == "/tf"
    ]
    tf_by_stamp = defaultdict(list)
    for sample in dynamic_odom_base:
        tf_by_stamp[sample["stamp"]].append(sample)
    exact_tf_matches = 0
    missing_tf_stamps = 0
    mismatched_tf_stamps = 0
    for stamp, pose_values in gated_poses.items():
        candidates = tf_by_stamp.get(stamp, [])
        if not candidates:
            missing_tf_stamps += 1
        elif any(sample["finite"] and sample["values"] == pose_values
                 for sample in candidates):
            exact_tf_matches += 1
        else:
            mismatched_tf_stamps += 1
    odom_base_tf_gids = sorted({sample["gid"] for sample in dynamic_odom_base})
    odom_base_tf_duplicate_stamps = sum(
        max(0, len(samples) - 1) for samples in tf_by_stamp.values()
    )
    odom_base_tf_stamp_regressions = 0
    previous_tf_stamp = None
    for sample in dynamic_odom_base:
        stamp = sample["stamp"]
        if stamp is not None and previous_tf_stamp is not None and stamp < previous_tf_stamp:
            odom_base_tf_stamp_regressions += 1
        if stamp is not None:
            previous_tf_stamp = stamp
    odom_tf_exact = {
        "gated_odom_unique_stamps": len(gated_poses),
        "dynamic_tf_samples": len(dynamic_odom_base),
        "exact_value_and_stamp_matches": exact_tf_matches,
        "missing_tf_stamps": missing_tf_stamps,
        "mismatched_tf_stamps": mismatched_tf_stamps,
        "nonfinite_tf_samples": sum(not sample["finite"] for sample in dynamic_odom_base),
        "duplicate_tf_stamps": odom_base_tf_duplicate_stamps,
        "tf_stamp_regressions": odom_base_tf_stamp_regressions,
        "publisher_gids": odom_base_tf_gids,
    }
    odom_tf_exact["pass"] = (
        len(gated_poses) > 0
        and len(dynamic_odom_base) == len(gated_poses)
        and exact_tf_matches == len(gated_poses)
        and missing_tf_stamps == 0
        and mismatched_tf_stamps == 0
        and odom_tf_exact["nonfinite_tf_samples"] == 0
        and odom_base_tf_duplicate_stamps == 0
        and odom_base_tf_stamp_regressions == 0
        and len(odom_base_tf_gids) == 1
    )
    # Advertised endpoints may exist even with publish_tf=false. They are not
    # evidence of emitted transforms; actual writer authority comes from C++.
    dynamic_endpoint_names = list(observer.tf_publishers["/tf"].values())
    tf_contract_pass = (
        not camera_body_present
        and base_parents == ["odom"]
        and odom_parents == ["map"]
        and cpp_tf_pass
        and odom_tf_exact["pass"]
    )
    main_continuity_pass = (
        primary.get("count", 0) > 0
        and topic_stats["/wheel_mapping/odometry_gated"]["nonfinite"] == 0
        and topic_stats["/wheel_mapping/odometry_gated"]["header_regressions"] == 0
        and primary_tail_lag is not None
        and -1.0 <= primary_tail_lag <= 2.0
        and topic_stats["/wheel_mapping/odometry_gated"]["max_header_gap_sec"] <= 2.0
    )
    planar_model_pass = (
        primary.get("count", 0) > 0
        and primary["max_abs_z_m"] <= 1.0e-3
        and primary["max_abs_roll_deg"] <= 0.1
        and primary["max_abs_pitch_deg"] <= 0.1
    )
    required_mapping = (
        "/wheel_mapping/cloud_registered", "/rtabmap/cloud_map",
        "/rtabmap/grid_map", "/rtabmap/optimized_cloud",
    )
    mapping_products_present = all(topic_stats[topic]["count"] > 0
                                   for topic in required_mapping)
    intensity_preserved = all(
        observer.map_details.get(topic, {}).get("has_intensity") is True
        for topic in (
            "/wheel_mapping/cloud_registered", "/rtabmap/optimized_cloud",
        )
    )
    cloud_nonempty_and_changing = all(
        cloud_evidence[topic]["count"] >= 2
        and cloud_evidence[topic]["invalid_count"] == 0
        and cloud_evidence[topic]["nonempty_count"] == cloud_evidence[topic]["count"]
        and cloud_evidence[topic]["unique_header_stamps"] == cloud_evidence[topic]["count"]
        and cloud_evidence[topic]["unique_data_hashes"] >= 2
        for topic in observed_cloud_topics
    )

    raw_intensity_by_stamp = observer.cloud_evidence["/xtm60/right/points"]["by_stamp"]
    registered_intensity_by_stamp = observer.cloud_evidence[
        "/wheel_mapping/cloud_registered"
    ]["by_stamp"]
    intensity_matches = 0
    intensity_mismatches = 0
    intensity_missing_raw = 0
    for stamp, registered_sample in registered_intensity_by_stamp.items():
        raw_sample = raw_intensity_by_stamp.get(stamp)
        if raw_sample is None:
            intensity_missing_raw += 1
        elif (
            registered_sample["intensity_hash"] is not None
            and registered_sample["intensity_hash"] == raw_sample["intensity_hash"]
        ):
            intensity_matches += 1
        else:
            intensity_mismatches += 1
    intensity_coverage = (
        intensity_matches / len(registered_intensity_by_stamp)
        if registered_intensity_by_stamp else 0.0
    )
    raw_registered_intensity = {
        "registered_unique_stamps": len(registered_intensity_by_stamp),
        "bit_exact_matches": intensity_matches,
        "mismatches": intensity_mismatches,
        "registered_stamps_missing_raw_observation": intensity_missing_raw,
        "comparison_coverage": intensity_coverage,
    }
    raw_registered_intensity["pass"] = (
        len(registered_intensity_by_stamp) > 0
        and intensity_mismatches == 0
        and intensity_coverage >= 0.95
        and not cloud_evidence["/xtm60/right/points"]["intensity_errors"]
        and not cloud_evidence["/wheel_mapping/cloud_registered"]["intensity_errors"]
    )

    raw_tail_stamp = topic_stats["/xtm60/right/points"]["last_header_sec"]
    tail_limits = {
        "/wheel_mapping/cloud_registered": 2.0,
        "/rtabmap/cloud_map": 5.0,
        "/rtabmap/grid_map": 5.0,
        "/rtabmap/optimized_cloud": 5.0,
        "/wheel_mapping/path": 2.0,
        "/rtabmap/optimized_path": 5.0,
        "/lio/shadow/cloud_registered": 2.0,
        "/lio/shadow/path": 2.0,
    }
    product_tail_lag = {}
    for topic, limit in tail_limits.items():
        output_tail = topic_stats[topic]["last_header_sec"]
        lag = (
            raw_tail_stamp - output_tail
            if raw_tail_stamp is not None and output_tail is not None else None
        )
        product_tail_lag[topic] = {
            "source_minus_output_sec": lag,
            "maximum_allowed_sec": limit,
            "pass": lag is not None and -1.0 <= lag <= limit,
        }
    product_full_tail_pass = all(item["pass"] for item in product_tail_lag.values())
    last_motion_stamp = max((row[0] for row in observer.wheel
                             if abs(row[1]) > 0.02 or abs(row[2]) > 0.02), default=None)
    grid_tail = topic_stats["/rtabmap/grid_map"]["last_header_sec"]
    # Grid publication is keyframe-driven: an unchanged map after the recorded
    # vehicle stops is expected, not a sensor freeze. Keep the literal tail lag
    # above, and separately require the grid to cover the last measured motion.
    grid_covers_motion = (last_motion_stamp is not None and grid_tail is not None
                          and grid_tail >= last_motion_stamp - 2.0)
    paths_nonempty = all(
        path_evidence[topic]["count"] > 0
        and path_evidence[topic]["max_poses"] >= 2
        for topic in observed_path_topics
    )
    grid_nonempty_and_changing = (
        grid_evidence["count"] >= 2
        and grid_evidence["invalid_count"] == 0
        and grid_evidence["nonempty_count"] == grid_evidence["count"]
        and grid_evidence["unique_data_hashes"] >= 2
    )
    consistency_observed = topic_stats["/lio/consistency"]["count"] > 0
    unavailable_observed = observer.consistency_states["UNAVAILABLE"] > 0
    relative_comparison_observed = (
        observer.consistency_states["OK"]
        + observer.consistency_states["DIVERGENCE"]
    ) > 0
    shadow_output_pass = (
        shadow.get("count", 0) > 0
        and topic_stats["/lio/shadow/odometry"]["nonfinite"] == 0
        and topic_stats["/lio/shadow/odometry"]["header_regressions"] == 0
        and bool(lidar_counters)
        and max((row[0] for row in lidar_counters), default=0) > 0
    )

    result = {
        "preflight": preflight,
        "runtime_error": runtime_error,
        "ros_domain_id": args.domain,
        "replayed_topics": list(ALLOWED_REPLAY_TOPICS),
        "only_allowlisted_bag_topics_replayed": True,
        "published_clock": True,
        "topic_stats": topic_stats,
        "map_details": observer.map_details,
        "cloud_evidence": cloud_evidence,
        "path_evidence": path_evidence,
        "grid_evidence": grid_evidence,
        "wheel_primary_odometry": primary,
        "unchecked_ekf_odometry": unchecked,
        "fastlio_shadow_odometry": shadow,
        "primary_last_sensor_minus_odom_sec": primary_tail_lag,
        "primary_gyro_yaw_integral_deg": (
            math.degrees(gyro_integral) if gyro_integral is not None else None
        ),
        "gyro_comparison_common_window_sec": gyro_comparison_window,
        "last_measured_wheel_motion_stamp": last_motion_stamp,
        "grid_covers_last_measured_motion": grid_covers_motion,
        "primary_yaw_minus_transformed_imu_integral_deg": (
            math.degrees(gyro_yaw_error) if gyro_yaw_error is not None else None
        ),
        "wheel_imu_yaw_agreement": wheel_imu_yaw,
        "fastlio_guard_counters": {
            "accepted_max": max((row[0] for row in lidar_counters), default=0),
            "rejected_max": max((row[1] for row in lidar_counters), default=0),
            "projected_max": max((row[2] for row in lidar_counters), default=0),
            "adapted_max": max((row[3] for row in lidar_counters), default=0),
        },
        "consistency_state_counts": dict(observer.consistency_states),
        "consistency_state_transitions": observer.consistency_events,
        "consistent_bool_counts": dict(observer.consistent_bool),
        "tf_edges": tf_edges,
        "tf_publishers": observer.tf_publishers,
        "tf_contract": {
            "per_message_publisher_identity_available": True,
            "identity_evidence_source": "C++ rclcpp MessageInfo; Python Humble has no GID metadata",
            "cpp_observed_tf_writers": cpp_tf_evidence,
            "cpp_single_writer_contract_pass": cpp_tf_pass,
            "dynamic_publisher_endpoints": dynamic_endpoint_names,
            "registered_endpoints_are_not_proof_of_emitted_messages": True,
            "camera_init_to_body_absent": not camera_body_present,
            "dynamic_base_link_parents": base_parents,
            "dynamic_odom_parents": odom_parents,
            "odom_to_base_link_owners": cpp_tf_evidence.get("odom->base_link", {}).get("owners", []),
            "map_to_odom_owners": cpp_tf_evidence.get("map->odom", {}).get("owners", []),
            "gated_odometry_exact_tf_equality": odom_tf_exact,
            "pass": tf_contract_pass,
        },
        "main_continuity_pass": main_continuity_pass,
        "diagnostic_level_floor_model_pass": planar_model_pass,
        "mapping_products_present": mapping_products_present,
        "intensity_fields_preserved": intensity_preserved,
        "raw_to_registered_intensity_bit_exact": raw_registered_intensity,
        "clouds_nonempty_unique_and_changing": cloud_nonempty_and_changing,
        "grid_nonempty_and_changing": grid_nonempty_and_changing,
        "paths_nonempty": paths_nonempty,
        "product_tail_lag": product_tail_lag,
        "products_reach_full_bag_tail": product_full_tail_pass,
        "shadow_consistency_observed": consistency_observed,
        "unavailable_before_gated_reference_observed": unavailable_observed,
        "valid_relative_comparison_observed": relative_comparison_observed,
        "fastlio_shadow_output_pass": shadow_output_pass,
        "cleanup": cleanup,
        "fast_log_restoration": restoration,
        "hardware_started": False,
        "motor_or_command_topics_replayed": False,
        "fastlio_has_tf_authority": False,
        "fastlio_wheel_or_zupt_aiding_enabled": False,
        "shared_h30_comparison_is_independent_ground_truth": False,
        "consistency_thresholds_calibrated": False,
        "physical_accuracy_or_safety_validated": False,
        "wall_duration_sec": time.monotonic() - started,
    }
    result["offline_integration_smoke_pass"] = (
        runtime_error is None
        and restoration.get("original_hashes_restored") is True
        and main_continuity_pass
        and planar_model_pass
        and mapping_products_present
        and intensity_preserved
        and raw_registered_intensity["pass"]
        and cloud_nonempty_and_changing
        and grid_nonempty_and_changing
        and paths_nonempty
        and product_full_tail_pass
        and consistency_observed
        and unavailable_observed
        and relative_comparison_observed
        and shadow_output_pass
        and tf_contract_pass
    )
    # Separate active-route delivery from the deliberately untrusted shadow's
    # continuity. A shadow failure must stay visible, but must not be confused
    # with the primary trajectory/map freezing. RTAB's grid-derived cloud_map
    # is not the promised XYZI product; optimized_cloud preserves raw scan XYZI.
    result["primary_mapping_smoke_pass"] = (
        runtime_error is None
        and restoration.get("original_hashes_restored") is True
        and main_continuity_pass and planar_model_pass and tf_contract_pass
        and intensity_preserved and raw_registered_intensity["pass"]
        and grid_nonempty_and_changing
        and all(product_tail_lag[topic]["pass"] for topic in (
            "/wheel_mapping/cloud_registered", "/wheel_mapping/path",
            "/rtabmap/optimized_cloud", "/rtabmap/optimized_path",
        ))
        and grid_covers_motion
        and all(cloud_evidence[topic]["count"] >= 2
                and cloud_evidence[topic]["invalid_count"] == 0
                and cloud_evidence[topic]["unique_data_hashes"] >= 2
                for topic in ("/wheel_mapping/cloud_registered", "/rtabmap/optimized_cloud"))
        and all(path_evidence[topic]["max_poses"] >= 2 for topic in (
            "/wheel_mapping/path", "/rtabmap/optimized_path",
        ))
        and consistency_observed
    )
    _write_json(output / "primary_odom_samples.json",
                observer.odom["/wheel_mapping/odometry_gated"])
    _write_json(output / "shadow_odom_samples.json",
                observer.odom["/lio/shadow/odometry"])
    _write_json(output / "consistency_events.json", observer.consistency_events)
    _write_json(output / "tf_evidence.json", {
        "edges": tf_edges, "publishers": observer.tf_publishers,
        "gated_odometry_exact_tf_equality": odom_tf_exact,
    })
    _write_json(output / "cloud_evidence.json", {
        "clouds": cloud_evidence,
        "raw_to_registered_intensity_bit_exact": raw_registered_intensity,
        "grid": grid_evidence,
        "product_tail_lag": product_tail_lag,
    })
    _write_json(output / "result.json", result)
    print(json.dumps(result, ensure_ascii=False, indent=2), flush=True)
    print(f"EVIDENCE_OUTPUT={output}", flush=True)
    node.destroy_node()
    rclpy.shutdown()
    return 0 if result["offline_integration_smoke_pass"] else 1


if __name__ == "__main__":
    raise SystemExit(_main())
