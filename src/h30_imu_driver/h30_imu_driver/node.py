import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

from smartwheel_sensor_api.contracts import guard_real_backend


class H30ImuNode(Node):
    def __init__(self) -> None:
        super().__init__("h30_imu_driver")
        self.declare_parameter("mode", "mock")
        self.declare_parameter("hardware_enabled", False)
        self.declare_parameter("frame_id", "imu_link")
        self.declare_parameter("topic", "/imu/data_raw")
        self.declare_parameter("rate_hz", 50.0)
        guard_real_backend(
            str(self.get_parameter("mode").value),
            bool(self.get_parameter("hardware_enabled").value),
            adapter_ready=False,
        )
        self._frame = str(self.get_parameter("frame_id").value)
        self._publisher = self.create_publisher(
            Imu, str(self.get_parameter("topic").value), qos_profile_sensor_data
        )
        rate = max(1.0, float(self.get_parameter("rate_hz").value))
        self.create_timer(1.0 / rate, self._tick)

    def _tick(self) -> None:
        now = self.get_clock().now()
        t = now.nanoseconds * 1e-9
        msg = Imu()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = self._frame
        msg.orientation.w = 1.0
        msg.angular_velocity.z = 0.05 * math.sin(t)
        msg.linear_acceleration.z = 9.80665
        msg.orientation_covariance[0] = 0.01
        msg.angular_velocity_covariance[0] = 0.001
        msg.linear_acceleration_covariance[0] = 0.02
        self._publisher.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = H30ImuNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

