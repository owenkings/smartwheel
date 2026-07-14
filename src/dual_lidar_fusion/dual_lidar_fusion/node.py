import math

import numpy as np
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header

from dual_lidar_fusion.pairing import TimestampPairer
from smartwheel_sensor_api import (
    apply_transform,
    pointcloud2_from_xyz,
    pointcloud2_to_xyz,
    voxel_downsample,
)
from smartwheel_sensor_api.pointcloud import transform_matrix


def _stamp_seconds(message: PointCloud2) -> float:
    return float(message.header.stamp.sec) + float(message.header.stamp.nanosec) * 1e-9


class DualLidarFusionNode(Node):
    def __init__(self) -> None:
        super().__init__("dual_lidar_fusion")
        self.declare_parameter("left_topic", "/lidar/left/points_raw")
        self.declare_parameter("right_topic", "/lidar/right/points_raw")
        self.declare_parameter("left_registered_topic", "/lidar/left/points_registered")
        self.declare_parameter("right_registered_topic", "/lidar/right/points_registered")
        self.declare_parameter("merged_topic", "/lidar/merged/points")
        self.declare_parameter("target_frame", "base_link")
        self.declare_parameter("pair_tolerance_sec", 0.03)
        self.declare_parameter("queue_size", 20)
        self.declare_parameter("voxel_size_m", 0.05)
        self.declare_parameter("integration_mode", "map_only")
        self.declare_parameter("left_xyz", [0.0, 0.0, 0.0])
        self.declare_parameter("left_rpy", [0.0, 0.0, 0.0])
        self.declare_parameter("right_xyz", [0.0, 0.0, 0.0])
        self.declare_parameter("right_rpy", [0.0, 0.0, 0.0])
        self._mode = str(self.get_parameter("integration_mode").value)
        if self._mode not in ("map_only", "dual_lio"):
            raise ValueError("integration_mode must be map_only or dual_lio")
        self._pairer = TimestampPairer(
            float(self.get_parameter("pair_tolerance_sec").value),
            int(self.get_parameter("queue_size").value),
        )
        self._target_frame = str(self.get_parameter("target_frame").value)
        self._voxel = float(self.get_parameter("voxel_size_m").value)
        self._left_transform = transform_matrix(
            self.get_parameter("left_xyz").value, self.get_parameter("left_rpy").value
        )
        self._right_transform = transform_matrix(
            self.get_parameter("right_xyz").value, self.get_parameter("right_rpy").value
        )
        self._left_pub = self.create_publisher(
            PointCloud2, str(self.get_parameter("left_registered_topic").value), qos_profile_sensor_data
        )
        self._right_pub = self.create_publisher(
            PointCloud2, str(self.get_parameter("right_registered_topic").value), qos_profile_sensor_data
        )
        self._merged_pub = self.create_publisher(
            PointCloud2, str(self.get_parameter("merged_topic").value), qos_profile_sensor_data
        )
        self._diagnostics = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self.create_subscription(
            PointCloud2, str(self.get_parameter("left_topic").value), self._on_left, qos_profile_sensor_data
        )
        self.create_subscription(
            PointCloud2, str(self.get_parameter("right_topic").value), self._on_right, qos_profile_sensor_data
        )
        if self._mode == "dual_lio":
            self.create_timer(1.0, self._publish_not_implemented)

    def _on_left(self, message: PointCloud2) -> None:
        if self._mode == "dual_lio":
            return
        self._publish_pair(self._pairer.add_left(_stamp_seconds(message), message))

    def _on_right(self, message: PointCloud2) -> None:
        if self._mode == "dual_lio":
            return
        self._publish_pair(self._pairer.add_right(_stamp_seconds(message), message))

    def _publish_pair(self, pair) -> None:
        if pair is None:
            return
        left_message, right_message = pair[0].value, pair[1].value
        try:
            left_xyz, left_intensity = pointcloud2_to_xyz(left_message)
            right_xyz, right_intensity = pointcloud2_to_xyz(right_message)
        except ValueError as exc:
            self._publish_diagnostic(DiagnosticStatus.ERROR, str(exc))
            return
        left_base = apply_transform(left_xyz, self._left_transform)
        right_base = apply_transform(right_xyz, self._right_transform)
        merged_xyz = np.vstack((left_base, right_base))
        if left_intensity is not None and right_intensity is not None:
            merged_intensity = np.concatenate((left_intensity, right_intensity))
        else:
            merged_intensity = None
        merged_xyz, merged_intensity = voxel_downsample(
            merged_xyz, self._voxel, merged_intensity
        )
        stamp = left_message.header.stamp if pair[0].stamp >= pair[1].stamp else right_message.header.stamp
        header = Header(stamp=stamp, frame_id=self._target_frame)
        self._left_pub.publish(pointcloud2_from_xyz(header, left_base, left_intensity))
        self._right_pub.publish(pointcloud2_from_xyz(header, right_base, right_intensity))
        self._merged_pub.publish(pointcloud2_from_xyz(header, merged_xyz, merged_intensity))

    def _publish_not_implemented(self) -> None:
        self._publish_diagnostic(
            DiagnosticStatus.ERROR, "dual_lio is NOT_IMPLEMENTED in Stage A; no cloud emitted"
        )

    def _publish_diagnostic(self, level: int, message: str) -> None:
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [
            DiagnosticStatus(
                level=level,
                name="dual_lidar_fusion/pairing",
                hardware_id="synthetic_or_adapter",
                message=message,
                values=[KeyValue(key="dropped_unpaired", value=str(self._pairer.dropped))],
            )
        ]
        self._diagnostics.publish(array)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DualLidarFusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

