import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2

from smartwheel_sensor_api import pointcloud2_to_xyz


def points_to_ranges(
    points: np.ndarray,
    angle_min: float,
    angle_max: float,
    angle_increment: float,
    range_min: float,
    range_max: float,
    min_height: float,
    max_height: float,
) -> np.ndarray:
    if angle_increment <= 0.0 or angle_max <= angle_min:
        raise ValueError("invalid scan angular limits")
    count = int(math.ceil((angle_max - angle_min) / angle_increment))
    ranges = np.full(count, np.inf, dtype=np.float32)
    xyz = np.asarray(points, dtype=np.float64)
    if xyz.size == 0:
        return ranges
    finite = np.isfinite(xyz).all(axis=1)
    distances = np.hypot(xyz[:, 0], xyz[:, 1])
    angles = np.arctan2(xyz[:, 1], xyz[:, 0])
    valid = (
        finite
        & (xyz[:, 2] >= min_height)
        & (xyz[:, 2] <= max_height)
        & (distances >= range_min)
        & (distances <= range_max)
        & (angles >= angle_min)
        & (angles < angle_max)
    )
    bins = np.floor((angles[valid] - angle_min) / angle_increment).astype(np.int64)
    np.minimum.at(ranges, bins, distances[valid].astype(np.float32))
    return ranges


class CloudToScanNode(Node):
    def __init__(self) -> None:
        super().__init__("smartwheel_cloud_to_scan")
        self.declare_parameter("input_topic", "/lidar/merged/points")
        self.declare_parameter("output_topic", "/scan")
        self.declare_parameter("target_frame", "base_link")
        self.declare_parameter("angle_min", -math.pi)
        self.declare_parameter("angle_max", math.pi)
        self.declare_parameter("angle_increment", math.radians(0.5))
        self.declare_parameter("range_min", 0.15)
        self.declare_parameter("range_max", 20.0)
        self.declare_parameter("min_height", 0.1)
        self.declare_parameter("max_height", 1.8)
        self.declare_parameter("scan_time", 0.1)
        self._publisher = self.create_publisher(
            LaserScan, str(self.get_parameter("output_topic").value), qos_profile_sensor_data
        )
        self.create_subscription(
            PointCloud2,
            str(self.get_parameter("input_topic").value),
            self._on_cloud,
            qos_profile_sensor_data,
        )

    def _on_cloud(self, message: PointCloud2) -> None:
        points, _ = pointcloud2_to_xyz(message)
        angle_min = float(self.get_parameter("angle_min").value)
        angle_max = float(self.get_parameter("angle_max").value)
        increment = float(self.get_parameter("angle_increment").value)
        range_min = float(self.get_parameter("range_min").value)
        range_max = float(self.get_parameter("range_max").value)
        ranges = points_to_ranges(
            points,
            angle_min,
            angle_max,
            increment,
            range_min,
            range_max,
            float(self.get_parameter("min_height").value),
            float(self.get_parameter("max_height").value),
        )
        scan = LaserScan()
        scan.header = message.header
        scan.header.frame_id = str(self.get_parameter("target_frame").value)
        scan.angle_min = angle_min
        scan.angle_max = angle_min + increment * len(ranges)
        scan.angle_increment = increment
        scan.scan_time = float(self.get_parameter("scan_time").value)
        scan.time_increment = 0.0
        scan.range_min = range_min
        scan.range_max = range_max
        scan.ranges = ranges.tolist()
        self._publisher.publish(scan)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CloudToScanNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
