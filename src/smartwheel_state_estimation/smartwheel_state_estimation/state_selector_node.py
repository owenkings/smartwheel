import json

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

from smartwheel_state_estimation.residuals import OdomVelocity, ResidualMonitor


class StateSelectorNode(Node):
    def __init__(self) -> None:
        super().__init__("smartwheel_state_selector")
        self.declare_parameter("state_mode", "lio_primary")
        self.declare_parameter("lio_topic", "/lio/odom")
        self.declare_parameter("wheel_topic", "/wheel/odom")
        self.declare_parameter("output_topic", "/odom/fused")
        self.declare_parameter("source_timeout_sec", 0.5)
        self.declare_parameter("max_linear_residual", 0.25)
        self.declare_parameter("max_angular_residual", 0.5)
        self._mode = str(self.get_parameter("state_mode").value)
        if self._mode not in ("lio_primary", "wheel_imu_fallback"):
            raise ValueError("state_mode must be lio_primary or wheel_imu_fallback")
        self._timeout = float(self.get_parameter("source_timeout_sec").value)
        self._selected_source = "lio" if self._mode == "lio_primary" else "wheel"
        self._monitor = ResidualMonitor(
            float(self.get_parameter("max_linear_residual").value),
            float(self.get_parameter("max_angular_residual").value),
        )
        self._latest = {"lio": None, "wheel": None}
        self._received_at = {"lio": 0.0, "wheel": 0.0}
        self._selected_latest = None
        self._publisher = self.create_publisher(
            Odometry, str(self.get_parameter("output_topic").value), 20
        )
        self._diagnostics = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self._tf = TransformBroadcaster(self)
        self.create_subscription(
            Odometry, str(self.get_parameter("lio_topic").value), lambda msg: self._on_odom("lio", msg), 20
        )
        self.create_subscription(
            Odometry, str(self.get_parameter("wheel_topic").value), lambda msg: self._on_odom("wheel", msg), 20
        )
        self.create_timer(0.1, self._refresh_tf)

    def _on_odom(self, source: str, message: Odometry) -> None:
        self._latest[source] = message
        self._received_at[source] = self.get_clock().now().nanoseconds * 1e-9
        if source == self._selected_source:
            self._selected_latest = message
            self._publisher.publish(message)
            self._broadcast(message)
        if self._latest["lio"] is not None and self._latest["wheel"] is not None:
            lio = self._latest["lio"].twist.twist
            wheel = self._latest["wheel"].twist.twist
            result = self._monitor.compare(
                OdomVelocity(lio.linear.x, lio.angular.z),
                OdomVelocity(wheel.linear.x, wheel.angular.z),
            )
            self._publish_residual(result)

    def _broadcast(self, odom: Odometry) -> None:
        transform = TransformStamped()
        transform.header.stamp = odom.header.stamp
        transform.header.frame_id = "odom"
        transform.child_frame_id = "base_link"
        transform.transform.translation.x = odom.pose.pose.position.x
        transform.transform.translation.y = odom.pose.pose.position.y
        transform.transform.translation.z = odom.pose.pose.position.z
        transform.transform.rotation = odom.pose.pose.orientation
        self._tf.sendTransform(transform)

    def _refresh_tf(self) -> None:
        age = self.get_clock().now().nanoseconds * 1e-9 - self._received_at[self._selected_source]
        if self._selected_latest is not None and age <= self._timeout:
            self._broadcast(self._selected_latest)

    def _publish_residual(self, result: dict) -> None:
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [
            DiagnosticStatus(
                level=DiagnosticStatus.WARN if result["degraded"] else DiagnosticStatus.OK,
                name="smartwheel_state_estimation/lio_wheel_residual",
                hardware_id="state_estimation",
                message="DEGRADED" if result["degraded"] else "OK",
                values=[KeyValue(key="metrics_json", value=json.dumps(result, sort_keys=True))],
            )
        ]
        self._diagnostics.publish(array)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = StateSelectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
