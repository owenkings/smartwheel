import math
from dataclasses import dataclass
from typing import Iterable, List, Optional, Sequence, Tuple

try:
    import rclpy
    from rclpy.duration import Duration
    from rclpy.node import Node
    from rclpy.time import Time
    from geometry_msgs.msg import Point
    from sensor_msgs.msg import LaserScan, PointCloud2
    from sensor_msgs_py import point_cloud2
    from visualization_msgs.msg import Marker, MarkerArray
    import tf2_ros
except ImportError:  # Allows pure helper tests without a sourced ROS environment.
    rclpy = None
    Duration = None
    Node = object
    Time = None
    Point = None
    LaserScan = None
    PointCloud2 = None
    point_cloud2 = None
    Marker = None
    MarkerArray = None
    tf2_ros = None


Point3 = Tuple[float, float, float]


@dataclass(frozen=True)
class ScanProjectionConfig:
    angle_min: float = -1.0472
    angle_max: float = 1.0472
    angle_increment: float = 0.0087
    range_min: float = 0.08
    range_max: float = 8.0
    z_min: float = -0.10
    z_max: float = 1.20
    use_inf: bool = True

    @property
    def beam_count(self) -> int:
        return int(math.ceil((self.angle_max - self.angle_min) / self.angle_increment)) + 1


def project_points_to_scan(
    points: Iterable[Point3], config: ScanProjectionConfig
) -> List[float]:
    """Project base_link-frame points into LaserScan bins using nearest range."""
    if config.angle_increment <= 0.0:
        raise ValueError("angle_increment must be positive")
    if config.angle_max <= config.angle_min:
        raise ValueError("angle_max must be greater than angle_min")

    empty_value = math.inf if config.use_inf else config.range_max + 1.0
    ranges = [empty_value for _ in range(config.beam_count)]

    for x, y, z in points:
        if not all(math.isfinite(v) for v in (x, y, z)):
            continue
        if z < config.z_min or z > config.z_max:
            continue
        planar_range = math.hypot(x, y)
        if planar_range < config.range_min or planar_range > config.range_max:
            continue
        angle = math.atan2(y, x)
        if angle < config.angle_min or angle > config.angle_max:
            continue
        index = int(round((angle - config.angle_min) / config.angle_increment))
        if 0 <= index < len(ranges) and planar_range < ranges[index]:
            ranges[index] = planar_range
    return ranges


def quaternion_to_matrix(x: float, y: float, z: float, w: float) -> List[List[float]]:
    xx = x * x
    yy = y * y
    zz = z * z
    xy = x * y
    xz = x * z
    yz = y * z
    wx = w * x
    wy = w * y
    wz = w * z
    return [
        [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
        [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
        [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
    ]


def transform_point(point: Point3, transform) -> Point3:
    """Apply a geometry_msgs TransformStamped transform to a point."""
    t = transform.transform.translation
    q = transform.transform.rotation
    matrix = quaternion_to_matrix(q.x, q.y, q.z, q.w)
    x, y, z = point
    return (
        matrix[0][0] * x + matrix[0][1] * y + matrix[0][2] * z + t.x,
        matrix[1][0] * x + matrix[1][1] * y + matrix[1][2] * z + t.y,
        matrix[2][0] * x + matrix[2][1] * y + matrix[2][2] * z + t.z,
    )


class PointCloudToLaserScanNode(Node):
    def __init__(self):
        super().__init__("pointcloud_to_laserscan_node")
        self.declare_parameter("input_topic", "/xtm60/points")
        self.declare_parameter("output_topic", "/scan")
        self.declare_parameter("marker_topic", "/scan_projection_markers")
        self.declare_parameter("target_frame", "base_link")
        self.declare_parameter("source_frame", "laser_link")
        self.declare_parameter("angle_min", -1.0472)
        self.declare_parameter("angle_max", 1.0472)
        self.declare_parameter("angle_increment", 0.0087)
        self.declare_parameter("range_min", 0.08)
        self.declare_parameter("range_max", 8.0)
        self.declare_parameter("z_min", -0.10)
        self.declare_parameter("z_max", 1.20)
        self.declare_parameter("scan_time", 0.1)
        self.declare_parameter("use_inf", True)
        self.declare_parameter("queue_size", 10)
        self.declare_parameter("publish_debug_markers", True)
        self.declare_parameter("restamp_output", True)

        self.input_topic = self.get_parameter("input_topic").value
        self.output_topic = self.get_parameter("output_topic").value
        self.marker_topic = self.get_parameter("marker_topic").value
        self.target_frame = self.get_parameter("target_frame").value
        self.source_frame = self.get_parameter("source_frame").value
        self.scan_time = float(self.get_parameter("scan_time").value)
        self.publish_debug_markers = bool(self.get_parameter("publish_debug_markers").value)
        self.restamp_output = bool(self.get_parameter("restamp_output").value)
        self.config = ScanProjectionConfig(
            angle_min=float(self.get_parameter("angle_min").value),
            angle_max=float(self.get_parameter("angle_max").value),
            angle_increment=float(self.get_parameter("angle_increment").value),
            range_min=float(self.get_parameter("range_min").value),
            range_max=float(self.get_parameter("range_max").value),
            z_min=float(self.get_parameter("z_min").value),
            z_max=float(self.get_parameter("z_max").value),
            use_inf=bool(self.get_parameter("use_inf").value),
        )

        queue_size = int(self.get_parameter("queue_size").value)
        self.scan_pub = self.create_publisher(LaserScan, self.output_topic, queue_size)
        self.marker_pub = self.create_publisher(MarkerArray, self.marker_topic, queue_size)
        self.cloud_sub = self.create_subscription(
            PointCloud2, self.input_topic, self.on_cloud, queue_size
        )

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.last_scan: Optional[LaserScan] = None

    def on_cloud(self, msg):
        frame_id = msg.header.frame_id or self.source_frame
        try:
            points = list(self._read_points_in_target_frame(msg, frame_id))
        except Exception as exc:  # Keep the navigation chain alive on TF or data failures.
            self.get_logger().warning(f"point cloud projection skipped: {exc}")
            if self.last_scan is not None:
                self.last_scan.header.stamp = self.get_clock().now().to_msg()
                self.scan_pub.publish(self.last_scan)
            else:
                self.scan_pub.publish(self._make_scan(msg.header, []))
            return

        if not points:
            self.get_logger().warning("received empty point cloud after filtering input fields")

        ranges = project_points_to_scan(points, self.config)
        scan = self._make_scan(msg.header, ranges)
        self.last_scan = scan
        self.scan_pub.publish(scan)
        if self.publish_debug_markers:
            self.marker_pub.publish(self._make_markers(scan))

    def _read_points_in_target_frame(self, msg, frame_id: str) -> Sequence[Point3]:
        raw_points = point_cloud2.read_points(
            msg, field_names=("x", "y", "z"), skip_nans=True
        )
        if frame_id == self.target_frame:
            return [(float(x), float(y), float(z)) for x, y, z in raw_points]

        transform = self.tf_buffer.lookup_transform(
            self.target_frame, frame_id, Time(), timeout=Duration(seconds=0.1)
        )
        return [transform_point((float(x), float(y), float(z)), transform) for x, y, z in raw_points]

    def _make_scan(self, header, ranges: Sequence[float]):
        scan = LaserScan()
        if self.restamp_output or not (header.stamp.sec or header.stamp.nanosec):
            scan.header.stamp = self.get_clock().now().to_msg()
        else:
            scan.header.stamp = header.stamp
        scan.header.frame_id = self.target_frame
        scan.angle_min = self.config.angle_min
        scan.angle_max = self.config.angle_max
        scan.angle_increment = self.config.angle_increment
        scan.time_increment = 0.0
        scan.scan_time = self.scan_time
        scan.range_min = self.config.range_min
        scan.range_max = self.config.range_max
        scan.ranges = list(ranges) if ranges else project_points_to_scan([], self.config)
        scan.intensities = []
        return scan

    def _make_markers(self, scan):
        marker = Marker()
        marker.header = scan.header
        marker.ns = "projected_scan"
        marker.id = 0
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.scale.x = 0.04
        marker.scale.y = 0.04
        marker.color.r = 0.1
        marker.color.g = 0.8
        marker.color.b = 0.2
        marker.color.a = 0.8
        for index, value in enumerate(scan.ranges):
            if not math.isfinite(value) or value < scan.range_min or value > scan.range_max:
                continue
            angle = scan.angle_min + index * scan.angle_increment
            point = Point()
            point.x = value * math.cos(angle)
            point.y = value * math.sin(angle)
            point.z = 0.0
            marker.points.append(point)
        markers = MarkerArray()
        markers.markers.append(marker)
        return markers


def main(args=None):
    if rclpy is None:
        raise RuntimeError("ROS2 Python packages are required to run this node")
    rclpy.init(args=args)
    node = PointCloudToLaserScanNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception:
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
