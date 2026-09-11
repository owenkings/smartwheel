"""ROS 2 report-only wrapper for :mod:`lio_consistency`.

The node publishes diagnostics and a boolean health shadow signal.  It never
publishes TF, never changes either odometry stream, and has no estimator or
motor-control output.
"""

from __future__ import annotations

import math
from typing import Sequence

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool

try:  # Package install path.
    from .lio_consistency import (
        MonitorConfig,
        MotionConsistencyMonitor,
        Pose,
        TimedPose,
    )
except ImportError:  # Standalone prototype path in this .work directory.
    from lio_consistency import (
        MonitorConfig,
        MotionConsistencyMonitor,
        Pose,
        TimedPose,
    )


def _stamp_seconds(message: Odometry) -> float:
    return float(message.header.stamp.sec) + float(message.header.stamp.nanosec) * 1.0e-9


def _pose(message: Odometry) -> Pose:
    position = message.pose.pose.position
    orientation = message.pose.pose.orientation
    return Pose(
        (position.x, position.y, position.z),
        (orientation.x, orientation.y, orientation.z, orientation.w),
    )


def _rotation_from_row_major(values: Sequence[float]):
    if len(values) != 9 or not all(math.isfinite(float(value)) for value in values):
        raise ValueError("base_to_imu_R must have nine finite row-major entries")
    rows = tuple(tuple(float(values[3 * row + column]) for column in range(3))
                 for row in range(3))
    for row in range(3):
        for column in range(3):
            dot = sum(rows[index][row] * rows[index][column] for index in range(3))
            expected = 1.0 if row == column else 0.0
            if abs(dot - expected) > 1.0e-5:
                raise ValueError("base_to_imu_R must be orthonormal")
    determinant = (
        rows[0][0] * (rows[1][1] * rows[2][2] - rows[1][2] * rows[2][1])
        - rows[0][1] * (rows[1][0] * rows[2][2] - rows[1][2] * rows[2][0])
        + rows[0][2] * (rows[1][0] * rows[2][1] - rows[1][1] * rows[2][0])
    )
    if abs(determinant - 1.0) > 1.0e-5:
        raise ValueError("base_to_imu_R determinant must be +1")

    # Stable matrix-to-quaternion conversion, output ROS xyzw.
    trace = rows[0][0] + rows[1][1] + rows[2][2]
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        quaternion = (
            (rows[2][1] - rows[1][2]) / scale,
            (rows[0][2] - rows[2][0]) / scale,
            (rows[1][0] - rows[0][1]) / scale,
            0.25 * scale,
        )
    elif rows[0][0] > rows[1][1] and rows[0][0] > rows[2][2]:
        scale = math.sqrt(1.0 + rows[0][0] - rows[1][1] - rows[2][2]) * 2.0
        quaternion = (
            0.25 * scale,
            (rows[0][1] + rows[1][0]) / scale,
            (rows[0][2] + rows[2][0]) / scale,
            (rows[2][1] - rows[1][2]) / scale,
        )
    elif rows[1][1] > rows[2][2]:
        scale = math.sqrt(1.0 + rows[1][1] - rows[0][0] - rows[2][2]) * 2.0
        quaternion = (
            (rows[0][1] + rows[1][0]) / scale,
            0.25 * scale,
            (rows[1][2] + rows[2][1]) / scale,
            (rows[0][2] - rows[2][0]) / scale,
        )
    else:
        scale = math.sqrt(1.0 + rows[2][2] - rows[0][0] - rows[1][1]) * 2.0
        quaternion = (
            (rows[0][2] + rows[2][0]) / scale,
            (rows[1][2] + rows[2][1]) / scale,
            0.25 * scale,
            (rows[1][0] - rows[0][1]) / scale,
        )
    return quaternion


class LioConsistencyNode(Node):
    def __init__(self) -> None:
        super().__init__("lio_consistency_monitor")
        self.declare_parameter("lio_topic", "/lio/shadow/odometry")
        self.declare_parameter("reference_topic", "/wheel_mapping/odometry_gated")
        self.declare_parameter("status_topic", "/lio/consistency")
        self.declare_parameter("consistent_topic", "/lio/consistent")
        self.declare_parameter("lio_child_frame", "body")
        self.declare_parameter("reference_child_frame", "base_link")
        self.declare_parameter("base_to_imu_R", [1.0, 0.0, 0.0,
                                                   0.0, 1.0, 0.0,
                                                   0.0, 0.0, 1.0])
        self.declare_parameter("base_to_imu_T", [0.0, 0.0, 0.0])
        self.declare_parameter("window_sec", 1.0)
        self.declare_parameter("source_timeout_sec", 1.0)
        self.declare_parameter("max_alignment_age_sec", 0.20)
        self.declare_parameter("max_interpolation_gap_sec", 0.20)
        # These are intentionally conservative starting points, not calibration.
        self.declare_parameter("max_translation_error_m", 0.10)
        self.declare_parameter("max_rotation_error_rad", 0.15)
        self.declare_parameter("publish_rate_hz", 5.0)

        rotation = _rotation_from_row_major(
            self.get_parameter("base_to_imu_R").value
        )
        translation = tuple(float(value) for value in
                            self.get_parameter("base_to_imu_T").value)
        if len(translation) != 3 or not all(math.isfinite(value) for value in translation):
            raise ValueError("base_to_imu_T must have three finite entries")
        config = MonitorConfig(
            window_sec=float(self.get_parameter("window_sec").value),
            source_timeout_sec=float(self.get_parameter("source_timeout_sec").value),
            max_alignment_age_sec=float(self.get_parameter("max_alignment_age_sec").value),
            max_interpolation_gap_sec=float(
                self.get_parameter("max_interpolation_gap_sec").value
            ),
            max_translation_error_m=float(
                self.get_parameter("max_translation_error_m").value
            ),
            max_rotation_error_rad=float(
                self.get_parameter("max_rotation_error_rad").value
            ),
            retention_sec=max(4.0, 3.0 * float(self.get_parameter("window_sec").value)),
        )
        self._monitor = MotionConsistencyMonitor(Pose(translation, rotation), config)
        self._lio_child = str(self.get_parameter("lio_child_frame").value)
        self._reference_child = str(
            self.get_parameter("reference_child_frame").value
        )
        self._frame_error = {"lio": None, "reference": None}

        qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(
            Odometry, str(self.get_parameter("lio_topic").value),
            self._on_lio, qos,
        )
        self.create_subscription(
            Odometry, str(self.get_parameter("reference_topic").value),
            self._on_reference, qos,
        )
        self._status_pub = self.create_publisher(
            DiagnosticArray, str(self.get_parameter("status_topic").value), 10
        )
        self._consistent_pub = self.create_publisher(
            Bool, str(self.get_parameter("consistent_topic").value), 10
        )
        rate = float(self.get_parameter("publish_rate_hz").value)
        if not math.isfinite(rate) or rate <= 0.0:
            raise ValueError("publish_rate_hz must be finite and positive")
        self.create_timer(1.0 / rate, self._publish)
        self.get_logger().info(
            "LIO consistency monitor started in report-only shadow mode; "
            "it does not publish TF or feed either estimator"
        )

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1.0e-9

    def _on_lio(self, message: Odometry) -> None:
        if message.child_frame_id != self._lio_child:
            self._frame_error["lio"] = (
                f"lio_child_frame_mismatch:{message.child_frame_id or '<empty>'}"
            )
            return
        self._frame_error["lio"] = None
        try:
            self._monitor.add_lio_imu_pose(
                TimedPose(_stamp_seconds(message), _pose(message)), self._now()
            )
        except ValueError:
            # The pure monitor records invalid poses where it can. A malformed
            # timestamp fails before construction and is represented here.
            self._frame_error["lio"] = "lio_invalid_timestamp_or_pose"

    def _on_reference(self, message: Odometry) -> None:
        if message.child_frame_id != self._reference_child:
            self._frame_error["reference"] = (
                "reference_child_frame_mismatch:"
                f"{message.child_frame_id or '<empty>'}"
            )
            return
        self._frame_error["reference"] = None
        try:
            self._monitor.add_reference_base_pose(
                TimedPose(_stamp_seconds(message), _pose(message)), self._now()
            )
        except ValueError:
            self._frame_error["reference"] = "reference_invalid_timestamp_or_pose"

    def _publish(self) -> None:
        frame_reasons = tuple(
            reason for reason in self._frame_error.values() if reason is not None
        )
        result = self._monitor.evaluate(self._now())
        if frame_reasons:
            state = "INVALID"
            consistent = False
            reasons = frame_reasons
        else:
            state = result.state
            consistent = result.consistent
            reasons = result.reasons

        level = {
            "OK": DiagnosticStatus.OK,
            "DIVERGENCE": DiagnosticStatus.WARN,
            "UNAVAILABLE": DiagnosticStatus.STALE,
            "WARMUP": DiagnosticStatus.STALE,
            "STALE": DiagnosticStatus.STALE,
            "INVALID": DiagnosticStatus.ERROR,
        }.get(state, DiagnosticStatus.ERROR)
        status = DiagnosticStatus()
        status.name = "SmartWheel/LIO relative-motion consistency"
        status.hardware_id = "lio_vs_wheel_imu"
        status.level = level
        status.message = state
        status.values = [
            KeyValue(key="state", value=state),
            KeyValue(key="consistent", value=str(bool(consistent)).lower()),
            KeyValue(key="reasons", value=",".join(reasons)),
            KeyValue(key="mode", value="report_only_shadow"),
            KeyValue(key="thresholds", value="provisional_not_calibrated"),
        ]
        for key, value in sorted(result.values.items()):
            status.values.append(KeyValue(key=key, value=str(value)))
        message = DiagnosticArray()
        message.header.stamp = self.get_clock().now().to_msg()
        message.status = [status]
        self._status_pub.publish(message)
        self._consistent_pub.publish(Bool(data=bool(consistent)))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LioConsistencyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
