"""Publish the live right XT-M60 XYZI frame in wheel-odometry coordinates."""

from __future__ import annotations

import math
import time

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Bool, Header
from tf2_ros import Buffer, TransformException, TransformListener

from .wheel_primary_cloud_math import transform_xyz_buffer, validate_xyz_buffer_layout
from .wheel_primary_retry_queue import BoundedRetryQueue, RetryDecision


def _rotation_matrix(quaternion) -> np.ndarray:
    values = np.asarray(
        [quaternion.x, quaternion.y, quaternion.z, quaternion.w], dtype=np.float64
    )
    norm = float(np.linalg.norm(values))
    if not np.isfinite(values).all() or not math.isfinite(norm) or norm < 1.0e-9:
        raise ValueError("invalid transform quaternion")
    x, y, z, w = values / norm
    return np.asarray(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


class WheelRegisteredCloud(Node):
    def __init__(self) -> None:
        super().__init__("wheel_registered_cloud")
        defaults = {
            "input_cloud_topic": "/xtm60/right/points",
            "output_cloud_topic": "/wheel_mapping/cloud_registered",
            "gated_odom_topic": "/wheel_mapping/odometry_gated",
            "feedback_health_topic": "/base/wheel_feedback_healthy",
            "expected_source_frame": "xtm60_right_link",
            "fixed_frame": "odom",
            "max_odom_receive_age_sec": 0.20,
            "max_feedback_receive_age_sec": 0.25,
            # Zero timeout is intentional: a subscription callback must never
            # block the same executor thread that fills the TF buffer.
            "transform_timeout_sec": 0.0,
            "retry_queue_max_clouds": 10,
            "retry_queue_max_age_sec": 0.25,
            "retry_period_sec": 0.02,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self._expected_source = str(self.get_parameter("expected_source_frame").value)
        self._fixed_frame = str(self.get_parameter("fixed_frame").value)
        self._odom_timeout = float(self.get_parameter("max_odom_receive_age_sec").value)
        self._feedback_timeout = float(
            self.get_parameter("max_feedback_receive_age_sec").value
        )
        self._transform_timeout = float(self.get_parameter("transform_timeout_sec").value)
        self._retry_max_clouds = int(self.get_parameter("retry_queue_max_clouds").value)
        self._retry_max_age = float(self.get_parameter("retry_queue_max_age_sec").value)
        self._retry_period = float(self.get_parameter("retry_period_sec").value)
        if (
            not all(math.isfinite(value) for value in (
                self._odom_timeout, self._feedback_timeout, self._transform_timeout,
                self._retry_max_age, self._retry_period,
            ))
            or self._odom_timeout <= 0.0
            or self._feedback_timeout <= 0.0
            or self._transform_timeout != 0.0
            or self._retry_max_clouds < 1
            or self._retry_max_age <= 0.0
            or self._retry_period <= 0.0
        ):
            raise ValueError("invalid cloud freshness/non-blocking retry bounds")
        self._last_odom_receive = self._last_feedback_receive = -math.inf
        self._feedback_ok = False
        self._expired_since_warning = 0
        self._last_expiry_warning = -math.inf
        self._pending = BoundedRetryQueue(
            max_items=self._retry_max_clouds,
            max_age_sec=self._retry_max_age,
        )
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._publisher = self.create_publisher(
            PointCloud2,
            str(self.get_parameter("output_cloud_topic").value),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter("gated_odom_topic").value),
            self._on_odom,
            qos_profile_sensor_data,
        )
        self.create_timer(self._retry_period, self._retry_pending)
        self.create_subscription(
            Bool,
            str(self.get_parameter("feedback_health_topic").value),
            self._on_feedback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PointCloud2,
            str(self.get_parameter("input_cloud_topic").value),
            self._on_cloud,
            qos_profile_sensor_data,
        )

    def _on_odom(self, _message: Odometry) -> None:
        self._last_odom_receive = time.monotonic()

    def _on_feedback(self, message: Bool) -> None:
        self._feedback_ok = bool(message.data)
        self._last_feedback_receive = time.monotonic()
        if not self._feedback_ok:
            self._pending.clear()

    def _health_fresh(self) -> bool:
        now = time.monotonic()
        return (
            self._feedback_ok
            and now - self._last_feedback_receive <= self._feedback_timeout
            and now - self._last_odom_receive <= self._odom_timeout
        )

    @staticmethod
    def _xyz_offsets(message: PointCloud2) -> tuple[int, int, int]:
        selected = {}
        for field in message.fields:
            if field.name in {"x", "y", "z"}:
                if field.name in selected:
                    raise ValueError(f"cloud has duplicate {field.name} fields")
                if field.datatype != PointField.FLOAT32 or field.count != 1:
                    raise ValueError(f"{field.name} must be a scalar FLOAT32 field")
                selected[field.name] = int(field.offset)
        if set(selected) != {"x", "y", "z"}:
            raise ValueError("cloud is missing x/y/z fields")
        return selected["x"], selected["y"], selected["z"]

    def _on_cloud(self, message: PointCloud2) -> None:
        if not self._health_fresh():
            self._pending.clear()
            return
        if message.header.frame_id != self._expected_source:
            self.get_logger().error(
                f"rejecting cloud frame {message.header.frame_id!r}; expected {self._expected_source!r}"
            )
            return
        stamp = message.header.stamp
        if stamp.sec < 0 or not 0 <= stamp.nanosec < 1_000_000_000 or (
            stamp.sec == 0 and stamp.nanosec == 0
        ):
            # tf2 treats stamp zero as "latest", which would violate the
            # same-stamp wheel pose contract.
            self.get_logger().error("rejecting cloud without a valid nonzero timestamp")
            return
        try:
            x_offset, y_offset, z_offset = self._xyz_offsets(message)
            validate_xyz_buffer_layout(
                message.data,
                width=message.width,
                height=message.height,
                point_step=message.point_step,
                row_step=message.row_step,
                x_offset=x_offset,
                y_offset=y_offset,
                z_offset=z_offset,
            )
        except ValueError as exc:
            self.get_logger().error(f"rejecting malformed wheel cloud: {exc}")
            return
        overflow = self._pending.push(message, time.monotonic())
        if overflow:
            self.get_logger().warning("wheel cloud retry queue full; dropped oldest frame")

    def _transform(self, message: PointCloud2):
        try:
            x_offset, y_offset, z_offset = self._xyz_offsets(message)
            transform = self._tf_buffer.lookup_transform(
                self._fixed_frame,
                message.header.frame_id,
                Time.from_msg(message.header.stamp),
                timeout=Duration(seconds=self._transform_timeout),
            )
            rotation = _rotation_matrix(transform.transform.rotation)
            translation = transform.transform.translation
            data = transform_xyz_buffer(
                message.data,
                width=message.width,
                height=message.height,
                point_step=message.point_step,
                row_step=message.row_step,
                x_offset=x_offset,
                y_offset=y_offset,
                z_offset=z_offset,
                bigendian=message.is_bigendian,
                rotation=rotation,
                translation=(translation.x, translation.y, translation.z),
            )
        except TransformException:
            return RetryDecision.RETRY, None
        except ValueError as exc:
            self.get_logger().error(f"wheel cloud transform permanently rejected: {exc}")
            return RetryDecision.DROP, None
        output = PointCloud2()
        output.header = Header(stamp=message.header.stamp, frame_id=self._fixed_frame)
        output.height = message.height
        output.width = message.width
        output.fields = message.fields
        output.is_bigendian = message.is_bigendian
        output.point_step = message.point_step
        output.row_step = message.row_step
        output.data = data
        output.is_dense = message.is_dense
        return RetryDecision.READY, output

    def _retry_pending(self) -> None:
        stats = self._pending.process(
            now=time.monotonic(),
            healthy=self._health_fresh,
            attempt=self._transform,
            publish=self._publisher.publish,
        )
        if stats.expired:
            self._expired_since_warning += stats.expired
            now = time.monotonic()
            if now - self._last_expiry_warning >= 1.0:
                self.get_logger().warning(
                    f"expired {self._expired_since_warning} wheel cloud frame(s) "
                    "waiting for exact-time TF"
                )
                self._expired_since_warning = 0
                self._last_expiry_warning = now


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WheelRegisteredCloud()
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
