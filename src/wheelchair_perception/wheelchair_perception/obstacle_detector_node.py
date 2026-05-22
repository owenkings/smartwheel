import math
from typing import List, Optional, Tuple

try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Point
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import String
    from visualization_msgs.msg import Marker, MarkerArray
except ImportError:
    rclpy = None
    Node = object
    Point = None
    LaserScan = None
    String = None
    Marker = None
    MarkerArray = None


def extract_obstacle_points(
    ranges: List[float],
    angle_min: float,
    angle_increment: float,
    range_min: float,
    range_max: float,
    cluster_distance: float = 0.25,
) -> List[Tuple[float, float]]:
    """Return simple cluster centroids from valid LaserScan returns."""
    clusters: List[List[Tuple[float, float]]] = []
    current: List[Tuple[float, float]] = []
    last_xy: Optional[Tuple[float, float]] = None

    for index, value in enumerate(ranges):
        if not math.isfinite(value) or value < range_min or value > range_max:
            if current:
                clusters.append(current)
                current = []
                last_xy = None
            continue
        angle = angle_min + index * angle_increment
        xy = (value * math.cos(angle), value * math.sin(angle))
        if last_xy is not None and math.hypot(xy[0] - last_xy[0], xy[1] - last_xy[1]) > cluster_distance:
            if current:
                clusters.append(current)
            current = []
        current.append(xy)
        last_xy = xy

    if current:
        clusters.append(current)

    centroids: List[Tuple[float, float]] = []
    for cluster in clusters:
        if len(cluster) < 2:
            continue
        x = sum(p[0] for p in cluster) / len(cluster)
        y = sum(p[1] for p in cluster) / len(cluster)
        centroids.append((x, y))
    return centroids


class ObstacleDetectorNode(Node):
    def __init__(self):
        super().__init__("obstacle_detector_node")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("obstacles_topic", "/obstacles")
        self.declare_parameter("summary_topic", "/obstacle_summary")
        self.declare_parameter("cluster_distance", 0.25)

        self.cluster_distance = float(self.get_parameter("cluster_distance").value)
        self.marker_pub = self.create_publisher(
            MarkerArray, self.get_parameter("obstacles_topic").value, 10
        )
        self.summary_pub = self.create_publisher(
            String, self.get_parameter("summary_topic").value, 10
        )
        self.create_subscription(
            LaserScan, self.get_parameter("scan_topic").value, self.on_scan, 10
        )

    def on_scan(self, msg):
        centroids = extract_obstacle_points(
            list(msg.ranges),
            msg.angle_min,
            msg.angle_increment,
            msg.range_min,
            msg.range_max,
            self.cluster_distance,
        )
        markers = MarkerArray()
        marker = Marker()
        marker.header = msg.header
        marker.ns = "scan_obstacles"
        marker.id = 0
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.18
        marker.scale.y = 0.18
        marker.scale.z = 0.18
        marker.color.r = 1.0
        marker.color.g = 0.35
        marker.color.b = 0.05
        marker.color.a = 0.85
        for x, y in centroids:
            point = Point()
            point.x = x
            point.y = y
            point.z = 0.12
            marker.points.append(point)
        markers.markers.append(marker)
        self.marker_pub.publish(markers)

        summary = String()
        nearest = min((math.hypot(x, y) for x, y in centroids), default=math.inf)
        summary.data = (
            f"obstacle_count={len(centroids)} nearest={nearest:.2f}"
            if math.isfinite(nearest)
            else "obstacle_count=0 nearest=inf"
        )
        self.summary_pub.publish(summary)


def main(args=None):
    if rclpy is None:
        raise RuntimeError("ROS2 Python packages are required to run this node")
    rclpy.init(args=args)
    node = ObstacleDetectorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
