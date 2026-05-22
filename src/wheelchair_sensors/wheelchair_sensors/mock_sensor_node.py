import math
from typing import List

try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import TransformStamped, Twist
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Image, Imu, PointCloud2, Range
    from sensor_msgs_py import point_cloud2
    from std_msgs.msg import Bool, Header
    from tf2_ros import TransformBroadcaster
except ImportError:
    rclpy = None
    Node = object
    TransformStamped = None
    Twist = None
    Odometry = None
    Image = None
    Imu = None
    PointCloud2 = None
    Range = None
    point_cloud2 = None
    Bool = None
    Header = None
    TransformBroadcaster = None


class MockSensorNode(Node):
    """Synthetic sensors for validating the stack without hardware."""

    def __init__(self):
        super().__init__("mock_sensor_node")
        self.declare_parameter("front_obstacle_distance", 2.0)
        self.declare_parameter("cycle_obstacle", True)
        self.declare_parameter("publish_scan_directly", False)
        self.declare_parameter("publish_cmd_vel_nav", True)
        self.declare_parameter("publish_odom", True)
        self.declare_parameter("publish_map_to_odom_tf", True)
        self.declare_parameter("publish_odom_to_base_tf", True)
        self.declare_parameter("pointcloud_rate_hz", 10.0)
        self.declare_parameter("state_rate_hz", 20.0)

        self.front_obstacle_distance = float(
            self.get_parameter("front_obstacle_distance").value
        )
        self.cycle_obstacle = bool(self.get_parameter("cycle_obstacle").value)
        self.publish_cmd_vel_nav = bool(self.get_parameter("publish_cmd_vel_nav").value)
        self.publish_odom_enabled = bool(self.get_parameter("publish_odom").value)
        self.publish_map_to_odom_tf = bool(
            self.get_parameter("publish_map_to_odom_tf").value
        )
        self.publish_odom_to_base_tf = bool(
            self.get_parameter("publish_odom_to_base_tf").value
        )

        self.cloud_pub = self.create_publisher(PointCloud2, "/xtm60/points", 10)
        self.imu_pub = self.create_publisher(Imu, "/imu/data", 10)
        self.image_pub = self.create_publisher(Image, "/camera/front/image_raw", 10)
        self.odom_pub = self.create_publisher(Odometry, "/wheel/odom", 10)
        self.estop_pub = self.create_publisher(Bool, "/emergency_stop_hw", 10)
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel_nav", 10)
        self.range_pubs = [
            self.create_publisher(Range, f"/ultrasonic/{i}/range", 10) for i in range(6)
        ]
        self.tf_broadcaster = TransformBroadcaster(self)

        self.t = 0.0
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.create_timer(1.0 / float(self.get_parameter("pointcloud_rate_hz").value), self.publish_pointcloud)
        self.create_timer(1.0 / float(self.get_parameter("state_rate_hz").value), self.publish_state)

    def current_obstacle_distance(self) -> float:
        if not self.cycle_obstacle:
            return self.front_obstacle_distance
        # Cycles through CLEAR -> WARNING -> SLOWDOWN -> STOP -> EMERGENCY_STOP.
        return 1.15 + 1.05 * math.sin(self.t * 0.35)

    def publish_pointcloud(self):
        now = self.get_clock().now().to_msg()
        header = Header()
        header.stamp = now
        header.frame_id = "laser_link"
        points = self.generate_xtm60_points(max(0.18, self.current_obstacle_distance() - 0.45))
        cloud = point_cloud2.create_cloud_xyz32(header, points)
        self.cloud_pub.publish(cloud)

    def generate_xtm60_points(self, obstacle_x_from_laser: float) -> List[List[float]]:
        points: List[List[float]] = []
        # Side walls and front clutter inside the XT-M60 120 degree horizontal FOV.
        for deg in range(-60, 61, 2):
            angle = math.radians(deg)
            wall_range = 3.5
            points.append([wall_range * math.cos(angle), wall_range * math.sin(angle), 0.0])
        for y_index in range(-5, 6):
            y = 0.04 * y_index
            points.append([obstacle_x_from_laser, y, 0.0])
            points.append([obstacle_x_from_laser, y, 0.25])
        return points

    def publish_state(self):
        self.t += 0.05
        now = self.get_clock().now().to_msg()
        obstacle_distance = max(0.12, self.current_obstacle_distance())

        imu = Imu()
        imu.header.stamp = now
        imu.header.frame_id = "imu_link"
        imu.orientation.w = 1.0
        imu.angular_velocity.z = 0.0
        imu.linear_acceleration.z = 9.81
        self.imu_pub.publish(imu)

        for i, pub in enumerate(self.range_pubs):
            msg = Range()
            msg.header.stamp = now
            msg.header.frame_id = f"ultrasonic_{i}_link"
            msg.radiation_type = Range.ULTRASOUND
            msg.field_of_view = 0.45
            msg.min_range = 0.03
            msg.max_range = 3.0
            msg.range = obstacle_distance if i in (0, 1, 2) else 2.5
            pub.publish(msg)

        image = Image()
        image.header.stamp = now
        image.header.frame_id = "camera_front_link"
        image.height = 120
        image.width = 160
        image.encoding = "rgb8"
        image.step = image.width * 3
        image.data = bytes([40, 80, 130]) * (image.width * image.height)
        self.image_pub.publish(image)

        if self.publish_odom_enabled:
            odom = Odometry()
            odom.header.stamp = now
            odom.header.frame_id = "odom"
            odom.child_frame_id = "base_link"
            odom.pose.pose.orientation.w = 1.0
            odom.twist.twist.linear.x = 0.15
            self.odom_pub.publish(odom)

        estop = Bool()
        estop.data = False
        self.estop_pub.publish(estop)

        if self.publish_cmd_vel_nav:
            cmd = Twist()
            cmd.linear.x = 0.35
            cmd.angular.z = 0.0
            self.cmd_pub.publish(cmd)

        self.publish_tf(now)

    def publish_tf(self, stamp):
        transforms = []
        if self.publish_map_to_odom_tf:
            map_to_odom = TransformStamped()
            map_to_odom.header.stamp = stamp
            map_to_odom.header.frame_id = "map"
            map_to_odom.child_frame_id = "odom"
            map_to_odom.transform.rotation.w = 1.0
            transforms.append(map_to_odom)

        if self.publish_odom_to_base_tf:
            odom_to_base = TransformStamped()
            odom_to_base.header.stamp = stamp
            odom_to_base.header.frame_id = "odom"
            odom_to_base.child_frame_id = "base_link"
            odom_to_base.transform.rotation.w = 1.0
            transforms.append(odom_to_base)

        if transforms:
            self.tf_broadcaster.sendTransform(transforms)


def main(args=None):
    if rclpy is None:
        raise RuntimeError("ROS2 Python packages are required to run this node")
    rclpy.init(args=args)
    node = MockSensorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
