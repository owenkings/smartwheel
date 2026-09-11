"""Fail-closed owner of accepted wheel/IMU odometry TF plus a bounded path."""

from __future__ import annotations

import json
import math
import time
from collections import deque

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, String
from tf2_ros import TransformBroadcaster

from .wheel_primary_health import HealthLimits, HealthSample, evaluate_health


def _stamp_seconds(message) -> float:
    stamp = message.header.stamp
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


def _finite_odometry(message: Odometry) -> bool:
    pose = message.pose.pose
    twist = message.twist.twist
    values = (
        pose.position.x, pose.position.y, pose.position.z,
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w,
        twist.linear.x, twist.linear.y, twist.linear.z,
        twist.angular.x, twist.angular.y, twist.angular.z,
    )
    q_norm = math.sqrt(sum(value * value for value in values[3:7]))
    return all(math.isfinite(value) for value in values) and q_norm > 1.0e-9


class WheelPoseHealthGate(Node):
    """The only ``odom -> base_link`` broadcaster in wheel-primary mode.

    robot_localization publishes an internal comparison pose but has TF disabled.
    This node accepts and republishes that pose only while real controller feedback,
    wheel messages, and IMU messages are fresh and structurally valid. A stale
    source therefore stops both gated Odometry and TF instead of extrapolating a
    prediction-only trajectory into RTAB-Map.
    """

    def __init__(self) -> None:
        super().__init__("wheel_pose_health_gate")
        defaults = {
            "input_odom_topic": "/odometry/filtered",
            "output_odom_topic": "/wheel_mapping/odometry_gated",
            "path_topic": "/wheel_mapping/path",
            "wheel_odom_topic": "/wheel/odom",
            "imu_topic": "/imu/data",
            "feedback_health_topic": "/base/wheel_feedback_healthy",
            "status_topic": "/wheel_mapping/pose_gate/status",
            "expected_parent_frame": "odom",
            "expected_child_frame": "base_link",
            "expected_wheel_parent_frame": "odom",
            "expected_wheel_child_frame": "base_link",
            "expected_imu_frame": "imu_link",
            "publish_tf": True,
            "max_feedback_receive_age_sec": 0.25,
            "max_wheel_receive_age_sec": 0.20,
            "max_imu_receive_age_sec": 0.10,
            "max_wheel_header_age_sec": 0.20,
            "max_imu_header_age_sec": 0.10,
            "max_ekf_header_age_sec": 0.20,
            "future_stamp_tolerance_sec": 0.05,
            "status_period_sec": 0.20,
            "path_min_period_sec": 0.10,
            "path_max_poses": 3000,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self._limits = HealthLimits(
            feedback_receive_age=float(self.get_parameter("max_feedback_receive_age_sec").value),
            wheel_receive_age=float(self.get_parameter("max_wheel_receive_age_sec").value),
            imu_receive_age=float(self.get_parameter("max_imu_receive_age_sec").value),
            wheel_header_age=float(self.get_parameter("max_wheel_header_age_sec").value),
            imu_header_age=float(self.get_parameter("max_imu_header_age_sec").value),
            ekf_header_age=float(self.get_parameter("max_ekf_header_age_sec").value),
            future_tolerance=float(self.get_parameter("future_stamp_tolerance_sec").value),
        )
        self._path_period = float(self.get_parameter("path_min_period_sec").value)
        self._status_period = float(self.get_parameter("status_period_sec").value)
        self._path = deque(maxlen=int(self.get_parameter("path_max_poses").value))
        if any(not math.isfinite(value) or value < 0.0 for value in self._limits.__dict__.values()):
            raise ValueError("health limits must be finite and non-negative")
        if (
            self._path.maxlen is None or self._path.maxlen < 1
            or not math.isfinite(self._path_period) or self._path_period <= 0.0
            or not math.isfinite(self._status_period) or self._status_period <= 0.0
        ):
            raise ValueError("invalid path/status bounds")

        self._feedback_ok = False
        self._feedback_received = self._wheel_received = self._imu_received = -math.inf
        self._wheel_stamp = self._imu_stamp = -math.inf
        self._wheel_valid = self._imu_valid = False
        self._last_odom_stamp = self._last_path_stamp = -math.inf
        self._latest_ekf_stamp = -math.inf
        self._latest_ekf_valid = False
        self._last_state = "WAITING_FOR_SOURCES"
        self._publish_tf = bool(self.get_parameter("publish_tf").value)

        self._odom_pub = self.create_publisher(
            Odometry, str(self.get_parameter("output_odom_topic").value), 20
        )
        self._path_pub = self.create_publisher(
            Path, str(self.get_parameter("path_topic").value), 5
        )
        self._status_pub = self.create_publisher(
            String, str(self.get_parameter("status_topic").value), 10
        )
        self._tf = TransformBroadcaster(self) if self._publish_tf else None
        self.create_subscription(
            Bool, str(self.get_parameter("feedback_health_topic").value),
            self._on_feedback, qos_profile_sensor_data
        )
        self.create_subscription(
            Odometry, str(self.get_parameter("wheel_odom_topic").value),
            self._on_wheel, qos_profile_sensor_data
        )
        self.create_subscription(
            Imu, str(self.get_parameter("imu_topic").value),
            self._on_imu, qos_profile_sensor_data
        )
        self.create_subscription(
            Odometry, str(self.get_parameter("input_odom_topic").value), self._on_odom, 20
        )
        self.create_timer(self._status_period, self._status_tick)

    def _ros_now_seconds(self) -> float:
        return self.get_clock().now().nanoseconds * 1.0e-9

    def _on_feedback(self, message: Bool) -> None:
        self._feedback_ok = bool(message.data)
        self._feedback_received = time.monotonic()

    def _on_wheel(self, message: Odometry) -> None:
        self._wheel_received = time.monotonic()
        self._wheel_stamp = _stamp_seconds(message)
        covariance = float(message.twist.covariance[0])
        self._wheel_valid = (
            message.header.frame_id
            == str(self.get_parameter("expected_wheel_parent_frame").value)
            and message.child_frame_id
            == str(self.get_parameter("expected_wheel_child_frame").value)
            and math.isfinite(self._wheel_stamp) and self._wheel_stamp > 0.0
            and math.isfinite(message.twist.twist.linear.x)
            and math.isfinite(message.twist.twist.angular.z)
            and math.isfinite(covariance) and covariance > 0.0
        )

    def _on_imu(self, message: Imu) -> None:
        self._imu_received = time.monotonic()
        self._imu_stamp = _stamp_seconds(message)
        angular_covariance = float(message.angular_velocity_covariance[8])
        self._imu_valid = (
            message.header.frame_id == str(self.get_parameter("expected_imu_frame").value)
            and math.isfinite(self._imu_stamp) and self._imu_stamp > 0.0
            and math.isfinite(message.angular_velocity.z)
            # ROS uses covariance[0] == -1 for an unavailable measurement.
            and message.angular_velocity_covariance[0] >= 0.0
            and math.isfinite(angular_covariance) and angular_covariance >= 0.0
        )

    def _sample(self, now_ros: float) -> HealthSample:
        now_steady = time.monotonic()
        return HealthSample(
            feedback_ok=self._feedback_ok,
            feedback_receive_age=now_steady - self._feedback_received,
            wheel_receive_age=now_steady - self._wheel_received,
            imu_receive_age=now_steady - self._imu_received,
            wheel_header_age=now_ros - self._wheel_stamp,
            imu_header_age=now_ros - self._imu_stamp,
            ekf_header_age=now_ros - self._latest_ekf_stamp,
            wheel_valid=self._wheel_valid,
            imu_valid=self._imu_valid,
            ekf_valid=self._latest_ekf_valid,
        )

    def _publish_status(self, state: str, sample: HealthSample) -> None:
        if state != self._last_state:
            self._last_state = state
            self.get_logger().info(f"wheel pose gate state={state}")
        payload = {
            "state": state,
            "mapping_pose_valid": state == "OK",
            "feedback_ok": sample.feedback_ok,
            "feedback_receive_age_sec": sample.feedback_receive_age,
            "wheel_receive_age_sec": sample.wheel_receive_age,
            "imu_receive_age_sec": sample.imu_receive_age,
            "wheel_header_age_sec": sample.wheel_header_age,
            "imu_header_age_sec": sample.imu_header_age,
            "ekf_header_age_sec": sample.ekf_header_age,
        }
        self._status_pub.publish(String(data=json.dumps(payload, sort_keys=True)))

    def _status_tick(self) -> None:
        sample = self._sample(self._ros_now_seconds())
        self._publish_status(evaluate_health(sample, self._limits), sample)

    def _on_odom(self, message: Odometry) -> None:
        self._latest_ekf_stamp = _stamp_seconds(message)
        self._latest_ekf_valid = (
            message.header.frame_id == str(self.get_parameter("expected_parent_frame").value)
            and message.child_frame_id == str(self.get_parameter("expected_child_frame").value)
            and math.isfinite(self._latest_ekf_stamp) and self._latest_ekf_stamp > 0.0
            and _finite_odometry(message)
        )
        sample = self._sample(self._ros_now_seconds())
        state = evaluate_health(sample, self._limits)
        if state != "OK" or self._latest_ekf_stamp <= self._last_odom_stamp:
            self._publish_status(
                "BLOCKED_EKF_NONMONOTONIC" if state == "OK" else state, sample
            )
            return

        self._last_odom_stamp = self._latest_ekf_stamp
        self._odom_pub.publish(message)
        if self._tf is not None:
            transform = TransformStamped()
            transform.header = message.header
            transform.child_frame_id = message.child_frame_id
            transform.transform.translation.x = message.pose.pose.position.x
            transform.transform.translation.y = message.pose.pose.position.y
            transform.transform.translation.z = message.pose.pose.position.z
            transform.transform.rotation = message.pose.pose.orientation
            self._tf.sendTransform(transform)
        if self._latest_ekf_stamp - self._last_path_stamp >= self._path_period:
            pose = PoseStamped()
            pose.header = message.header
            pose.pose = message.pose.pose
            self._path.append(pose)
            self._last_path_stamp = self._latest_ekf_stamp
            path = Path()
            path.header = message.header
            path.poses = list(self._path)
            self._path_pub.publish(path)
        self._publish_status("OK", sample)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WheelPoseHealthGate()
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
