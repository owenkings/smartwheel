import math

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from nav_msgs.msg import Odometry
from rclpy.node import Node
from smartwheel_interfaces.msg import WheelEncoder

from wheel_odom_driver.kinematics import DifferentialOdometry, EncoderConfig


def _stamp_seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


class WheelOdomNode(Node):
    def __init__(self) -> None:
        super().__init__("wheel_odom_driver")
        self.declare_parameter("wheel_radius_m", 0.0)
        self.declare_parameter("track_width_m", 0.0)
        self.declare_parameter("encoder_cpr", 0)
        self.declare_parameter("gear_ratio", 0.0)
        self.declare_parameter("left_sign", 0)
        self.declare_parameter("right_sign", 0)
        self.declare_parameter("encoder_bits", 32)
        self.declare_parameter("input_topic", "/wheel/encoder_counts")
        self.declare_parameter("output_topic", "/wheel/odom")
        self.declare_parameter("odom_frame_id", "odom")
        self.declare_parameter("base_frame_id", "base_link")
        self._model = DifferentialOdometry(
            EncoderConfig(
                wheel_radius_m=float(self.get_parameter("wheel_radius_m").value),
                track_width_m=float(self.get_parameter("track_width_m").value),
                encoder_cpr=int(self.get_parameter("encoder_cpr").value),
                gear_ratio=float(self.get_parameter("gear_ratio").value),
                left_sign=int(self.get_parameter("left_sign").value),
                right_sign=int(self.get_parameter("right_sign").value),
                encoder_bits=int(self.get_parameter("encoder_bits").value),
            )
        )
        self._odom_frame = str(self.get_parameter("odom_frame_id").value)
        self._base_frame = str(self.get_parameter("base_frame_id").value)
        self._publisher = self.create_publisher(
            Odometry, str(self.get_parameter("output_topic").value), 20
        )
        self._diagnostics = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self.create_subscription(
            WheelEncoder, str(self.get_parameter("input_topic").value), self._on_encoder, 20
        )

    def _on_encoder(self, message: WheelEncoder) -> None:
        if not message.valid:
            self._publish_diagnostic(DiagnosticStatus.WARN, "encoder feedback unavailable")
            return
        try:
            update = self._model.update(
                message.left_count, message.right_count, _stamp_seconds(message.stamp)
            )
        except ValueError as exc:
            self._publish_diagnostic(DiagnosticStatus.ERROR, str(exc))
            return
        if update is None:
            return
        odom = Odometry()
        odom.header.stamp = message.stamp
        odom.header.frame_id = self._odom_frame
        odom.child_frame_id = self._base_frame
        odom.pose.pose.position.x = update.x
        odom.pose.pose.position.y = update.y
        odom.pose.pose.orientation.z = math.sin(update.yaw * 0.5)
        odom.pose.pose.orientation.w = math.cos(update.yaw * 0.5)
        odom.twist.twist.linear.x = update.linear_mps
        odom.twist.twist.angular.z = update.angular_rps
        odom.pose.covariance[0] = update.position_variance
        odom.pose.covariance[7] = update.position_variance
        odom.pose.covariance[35] = update.yaw_variance
        odom.twist.covariance[0] = update.position_variance / max(update.dt * update.dt, 1e-9)
        odom.twist.covariance[35] = update.yaw_variance / max(update.dt * update.dt, 1e-9)
        self._publisher.publish(odom)

    def _publish_diagnostic(self, level: int, message: str) -> None:
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus(
            level=level,
            name="wheel_odom_driver/encoder",
            hardware_id="wheel_encoder",
            message=message,
            values=[KeyValue(key="publishes_tf", value="false")],
        )
        array.status = [status]
        self._diagnostics.publish(array)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WheelOdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

