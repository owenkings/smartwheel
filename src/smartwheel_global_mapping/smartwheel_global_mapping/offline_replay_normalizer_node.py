"""Normalize recorded odometry and scan timing for offline mapping only.

The formal H1 bag records FAST-LIO odometry as camera_init -> body and
contains sparse, discrete TF samples. Offline SLAM consumers need one
odom -> body owner at each scan timestamp, with the recorded static
body -> base_link edge completing the chain. This node is the explicit
boundary between those contracts: it preserves recorded pose values,
normalizes frame IDs, interpolates nearby odometry samples, publishes the
transform before each scan, and never touches hardware.
"""

from bisect import bisect_right
from collections import deque
from copy import deepcopy
from dataclasses import dataclass
import math
import time
from typing import Deque, List, Optional, Tuple

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformBroadcaster


@dataclass(frozen=True)
class PoseSample:
    stamp: float
    position: Tuple[float, float, float]
    orientation: Tuple[float, float, float, float]


def _stamp_seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def _normalize_quaternion(quaternion: Tuple[float, float, float, float]):
    norm = math.sqrt(sum(value * value for value in quaternion))
    if norm < 1e-12:
        return (0.0, 0.0, 0.0, 1.0)
    return tuple(value / norm for value in quaternion)


def _interpolate_quaternion(first, second, ratio: float):
    q0 = _normalize_quaternion(first)
    q1 = _normalize_quaternion(second)
    dot = sum(a * b for a, b in zip(q0, q1))
    if dot < 0.0:
        q1 = tuple(-value for value in q1)
        dot = -dot
    if dot > 0.9995:
        return _normalize_quaternion(
            tuple(a + ratio * (b - a) for a, b in zip(q0, q1))
        )
    theta = math.acos(max(-1.0, min(1.0, dot)))
    sine = math.sin(theta)
    if abs(sine) < 1e-12:
        return q0
    weight0 = math.sin((1.0 - ratio) * theta) / sine
    weight1 = math.sin(ratio * theta) / sine
    return _normalize_quaternion(
        tuple(weight0 * a + weight1 * b for a, b in zip(q0, q1))
    )


def interpolate_pose(
    samples: List[PoseSample], stamp: float, max_gap_sec: float
) -> Optional[PoseSample]:
    """Return an interpolated pose, or a nearby endpoint within max_gap_sec."""

    if not samples:
        return None
    stamps = [sample.stamp for sample in samples]
    index = bisect_right(stamps, stamp)
    if index == 0:
        # A future transform cannot satisfy a scan stamped before the first
        # odometry sample. Wait for a prior sample instead of poisoning the TF
        # cache with a transform later than the scan.
        return None
    if index == len(samples):
        return samples[-1] if stamp - samples[-1].stamp <= max_gap_sec else None
    before = samples[index - 1]
    after = samples[index]
    gap = after.stamp - before.stamp
    if gap <= 0.0 or gap > max_gap_sec:
        return before if stamp - before.stamp <= max_gap_sec else None
    ratio = (stamp - before.stamp) / gap
    position = tuple(
        a + ratio * (b - a) for a, b in zip(before.position, after.position)
    )
    orientation = _interpolate_quaternion(
        before.orientation, after.orientation, ratio
    )
    return PoseSample(stamp, position, orientation)


class OfflineReplayNormalizerNode(Node):
    """Single TF owner for the offline replay path."""

    def __init__(self) -> None:
        super().__init__("offline_replay_normalizer")
        self.declare_parameter("odom_input_topic", "/offline/selected_odom")
        self.declare_parameter("odom_output_topic", "/odom/fused")
        self.declare_parameter("scan_input_topic", "/offline/raw_scan")
        self.declare_parameter("scan_output_topic", "/scan")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("child_frame", "body")
        self.declare_parameter("max_interpolation_gap_sec", 0.25)
        self.declare_parameter("sample_history_size", 512)
        self.declare_parameter("tf_broadcast_delay_sec", 0.20)

        self._odom_frame = str(self.get_parameter("odom_frame").value)
        self._child_frame = str(self.get_parameter("child_frame").value)
        self._max_gap = float(
            self.get_parameter("max_interpolation_gap_sec").value
        )
        if not self._odom_frame or not self._child_frame:
            raise ValueError("odom_frame and child_frame must be non-empty")
        if self._max_gap <= 0.0:
            raise ValueError("max_interpolation_gap_sec must be positive")
        history_size = int(self.get_parameter("sample_history_size").value)
        if history_size < 2:
            raise ValueError("sample_history_size must be at least 2")
        self._tf_delay = float(self.get_parameter("tf_broadcast_delay_sec").value)
        if self._tf_delay < 0.0:
            raise ValueError("tf_broadcast_delay_sec must be non-negative")

        self._samples: Deque[PoseSample] = deque(maxlen=history_size)
        self._pending_scans: Deque[LaserScan] = deque(maxlen=128)
        self._scan_queue: Deque[Tuple[float, LaserScan]] = deque(maxlen=128)
        self._last_warning_time = -math.inf

        self._odom_publisher = self.create_publisher(
            Odometry, str(self.get_parameter("odom_output_topic").value), 20
        )
        self._scan_publisher = self.create_publisher(
            LaserScan,
            str(self.get_parameter("scan_output_topic").value),
            20,
        )
        self._tf = TransformBroadcaster(self)
        self.create_subscription(
            Odometry,
            str(self.get_parameter("odom_input_topic").value),
            self._on_odom,
            20,
        )
        self.create_subscription(
            LaserScan,
            str(self.get_parameter("scan_input_topic").value),
            self._on_scan,
            qos_profile_sensor_data,
        )
        self.create_timer(0.01, self._drain_scan_queue)

    def _on_odom(self, message: Odometry) -> None:
        stamp = _stamp_seconds(message.header.stamp)
        sample = PoseSample(
            stamp,
            (
                float(message.pose.pose.position.x),
                float(message.pose.pose.position.y),
                float(message.pose.pose.position.z),
            ),
            (
                float(message.pose.pose.orientation.x),
                float(message.pose.pose.orientation.y),
                float(message.pose.pose.orientation.z),
                float(message.pose.pose.orientation.w),
            ),
        )
        if self._samples and stamp < self._samples[-1].stamp:
            ordered = sorted([*self._samples, sample], key=lambda item: item.stamp)
            self._samples = deque(
                ordered[-self._samples.maxlen :], maxlen=self._samples.maxlen
            )
        else:
            self._samples.append(sample)

        normalized = deepcopy(message)
        normalized.header.frame_id = self._odom_frame
        normalized.child_frame_id = self._child_frame
        self._odom_publisher.publish(normalized)
        self._broadcast(sample, message.header.stamp)
        self._flush_pending_scans()

    def _on_scan(self, message: LaserScan) -> None:
        if self._pose_at(message.header.stamp) is None:
            self._pending_scans.append(deepcopy(message))
            self._warn_throttled(
                "waiting for a nearby recorded odometry sample before publishing scan"
            )
            return
        self._enqueue_scan(message)

    def _flush_pending_scans(self) -> None:
        if not self._pending_scans:
            return
        pending = list(self._pending_scans)
        self._pending_scans.clear()
        for scan in pending:
            if self._pose_at(scan.header.stamp) is None:
                self._warn_throttled(
                    "dropping an offline scan outside the odometry interpolation window"
                )
                continue
            self._enqueue_scan(scan)

    def _pose_at(self, stamp) -> Optional[PoseSample]:
        return interpolate_pose(
            list(self._samples), _stamp_seconds(stamp), self._max_gap
        )

    def _enqueue_scan(self, message: LaserScan) -> None:
        pose = self._pose_at(message.header.stamp)
        if pose is None:
            return
        # Publish TF first and hold the scan briefly so DDS subscribers receive
        # the transform before their message filter evaluates this scan.
        self._broadcast(pose, message.header.stamp)
        self._scan_queue.append((time.monotonic() + self._tf_delay, deepcopy(message)))

    def _drain_scan_queue(self) -> None:
        now = time.monotonic()
        while self._scan_queue and self._scan_queue[0][0] <= now:
            _, message = self._scan_queue.popleft()
            self._scan_publisher.publish(message)

    def _broadcast(self, pose: PoseSample, stamp=None) -> None:
        transform = TransformStamped()
        transform.header.frame_id = self._odom_frame
        transform.child_frame_id = self._child_frame
        transform.transform.translation.x = pose.position[0]
        transform.transform.translation.y = pose.position[1]
        transform.transform.translation.z = pose.position[2]
        (
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w,
        ) = pose.orientation
        if stamp is None:
            transform.header.stamp.sec = int(pose.stamp)
            transform.header.stamp.nanosec = int(
                round((pose.stamp - int(pose.stamp)) * 1e9)
            )
            if transform.header.stamp.nanosec >= 1_000_000_000:
                transform.header.stamp.sec += 1
                transform.header.stamp.nanosec -= 1_000_000_000
        else:
            # Preserve the exact recorded ROS timestamp; do not round-trip it
            # through float seconds before the scan's TF lookup.
            transform.header.stamp.sec = int(stamp.sec)
            transform.header.stamp.nanosec = int(stamp.nanosec)
        self._tf.sendTransform(transform)

    def _warn_throttled(self, message: str) -> None:
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self._last_warning_time >= 5.0:
            self.get_logger().warning(message)
            self._last_warning_time = now


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OfflineReplayNormalizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
