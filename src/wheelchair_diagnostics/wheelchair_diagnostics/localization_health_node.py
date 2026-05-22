import time

try:
    import rclpy
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import Bool, String
except ImportError:
    rclpy = None
    Node = object
    DiagnosticArray = None
    DiagnosticStatus = None
    KeyValue = None
    PoseWithCovarianceStamped = None
    Odometry = None
    LaserScan = None
    Bool = None
    String = None

from wheelchair_diagnostics.policy import LocalizationSample, evaluate_localization_health


class LocalizationHealthNode(Node):
    def __init__(self):
        super().__init__("localization_health_node")
        self.declare_parameter("publish_rate_hz", 2.0)
        self.declare_parameter("max_amcl_age_sec", 2.0)
        self.declare_parameter("max_odom_age_sec", 1.0)
        self.declare_parameter("max_scan_age_sec", 1.0)
        self.declare_parameter("max_covariance_xy", 0.50)
        self.declare_parameter("max_covariance_yaw", 0.35)

        self.last_amcl_time = None
        self.last_odom_time = None
        self.last_scan_time = None
        self.covariance_x = None
        self.covariance_y = None
        self.covariance_yaw = None

        self.health_pub = self.create_publisher(String, "/localization/health", 10)
        self.healthy_pub = self.create_publisher(Bool, "/localization/is_healthy", 10)
        self.diag_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)

        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self.on_amcl_pose, 10)
        self.create_subscription(Odometry, "/wheel/odom", self.on_odom, 10)
        self.create_subscription(LaserScan, "/scan", self.on_scan, 10)
        rate = max(0.5, float(self.get_parameter("publish_rate_hz").value))
        self.timer = self.create_timer(1.0 / rate, self.tick)

    def on_amcl_pose(self, msg):
        self.last_amcl_time = time.monotonic()
        cov = msg.pose.covariance
        self.covariance_x = float(cov[0])
        self.covariance_y = float(cov[7])
        self.covariance_yaw = float(cov[35])

    def on_odom(self, _msg):
        self.last_odom_time = time.monotonic()

    def on_scan(self, _msg):
        self.last_scan_time = time.monotonic()

    def tick(self):
        now = time.monotonic()
        sample = LocalizationSample(
            amcl_age_sec=None if self.last_amcl_time is None else now - self.last_amcl_time,
            odom_age_sec=None if self.last_odom_time is None else now - self.last_odom_time,
            scan_age_sec=None if self.last_scan_time is None else now - self.last_scan_time,
            covariance_x=self.covariance_x,
            covariance_y=self.covariance_y,
            covariance_yaw=self.covariance_yaw,
        )
        decision = evaluate_localization_health(
            sample,
            float(self.get_parameter("max_amcl_age_sec").value),
            float(self.get_parameter("max_odom_age_sec").value),
            float(self.get_parameter("max_scan_age_sec").value),
            float(self.get_parameter("max_covariance_xy").value),
            float(self.get_parameter("max_covariance_yaw").value),
        )

        msg = String()
        msg.data = decision.to_json()
        self.health_pub.publish(msg)

        healthy = Bool()
        healthy.data = decision.healthy
        self.healthy_pub.publish(healthy)

        diag = DiagnosticArray()
        diag.header.stamp = self.get_clock().now().to_msg()
        item = DiagnosticStatus()
        item.name = "localization/health"
        item.hardware_id = "amcl_odom_scan"
        item.level = DiagnosticStatus.OK if decision.state == "GOOD" else (DiagnosticStatus.ERROR if decision.state == "LOST" else DiagnosticStatus.WARN)
        item.message = decision.reason
        item.values = [
            KeyValue(key="state", value=decision.state),
            KeyValue(key="healthy", value=str(decision.healthy)),
        ]
        diag.status = [item]
        self.diag_pub.publish(diag)


def main(args=None):
    if rclpy is None:
        raise RuntimeError("ROS2 Python packages are required to run this node")
    rclpy.init(args=args)
    node = LocalizationHealthNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
