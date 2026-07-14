import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Header, String

from smartwheel_sensor_api import pointcloud2_from_xyz
from xtm60_ros2_driver.backends import create_backend


class Xtm60DriverNode(Node):
    def __init__(self) -> None:
        super().__init__("xtm60_driver")
        self.declare_parameter("mode", "mock")
        self.declare_parameter("hardware_enabled", False)
        self.declare_parameter("side", "left")
        self.declare_parameter("frame_id", "xtm60_left_link")
        self.declare_parameter("topic", "/lidar/left/points_raw")
        self.declare_parameter("rate_hz", 5.0)
        self.declare_parameter("seed", 20260714)
        mode = str(self.get_parameter("mode").value)
        enabled = bool(self.get_parameter("hardware_enabled").value)
        self._backend = create_backend(mode, enabled, int(self.get_parameter("seed").value))
        self._frame_id = str(self.get_parameter("frame_id").value)
        self._publisher = self.create_publisher(
            __import__("sensor_msgs.msg", fromlist=["PointCloud2"]).PointCloud2,
            str(self.get_parameter("topic").value),
            qos_profile_sensor_data,
        )
        self._status = self.create_publisher(String, "/hardware/status_text", 10)
        rate = max(0.1, float(self.get_parameter("rate_hz").value))
        self.create_timer(1.0 / rate, self._tick)

    def _tick(self) -> None:
        now = self.get_clock().now()
        points = self._backend.sample(now.nanoseconds * 1e-9)
        header = Header(stamp=now.to_msg(), frame_id=self._frame_id)
        self._publisher.publish(pointcloud2_from_xyz(header, points))
        self._status.publish(String(data="XT-M60 mock backend active; no hardware transport"))

    def destroy_node(self):
        self._backend.close()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Xtm60DriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

