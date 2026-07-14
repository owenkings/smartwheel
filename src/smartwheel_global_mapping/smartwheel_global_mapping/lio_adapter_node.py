import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2

from smartwheel_sensor_api import validate_pointcloud_fields


class LioInputAdapterNode(Node):
    def __init__(self) -> None:
        super().__init__("smartwheel_lio_input_adapter")
        self.declare_parameter("input_topic", "/lidar/left/points_raw")
        self.declare_parameter("output_topic", "/lidar/primary/points_lio")
        self.declare_parameter("expected_frame_id", "xtm60_left_link")
        self.declare_parameter("require_intensity", False)
        self._expected_frame = str(self.get_parameter("expected_frame_id").value)
        self._require_intensity = bool(self.get_parameter("require_intensity").value)
        self._publisher = self.create_publisher(
            PointCloud2, str(self.get_parameter("output_topic").value), qos_profile_sensor_data
        )
        self._diagnostics = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self.create_subscription(
            PointCloud2, str(self.get_parameter("input_topic").value), self._on_cloud, qos_profile_sensor_data
        )

    def _on_cloud(self, message: PointCloud2) -> None:
        required = ("x", "y", "z", "intensity") if self._require_intensity else ("x", "y", "z")
        try:
            validate_pointcloud_fields(message, required)
            if message.header.frame_id != self._expected_frame:
                raise ValueError(
                    f"primary LiDAR frame mismatch: {message.header.frame_id} != {self._expected_frame}"
                )
        except ValueError as exc:
            array = DiagnosticArray()
            array.header.stamp = self.get_clock().now().to_msg()
            array.status = [
                DiagnosticStatus(
                    level=DiagnosticStatus.ERROR,
                    name="smartwheel_global_mapping/lio_input",
                    hardware_id="primary_lidar",
                    message=str(exc),
                )
            ]
            self._diagnostics.publish(array)
            return
        self._publisher.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LioInputAdapterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

