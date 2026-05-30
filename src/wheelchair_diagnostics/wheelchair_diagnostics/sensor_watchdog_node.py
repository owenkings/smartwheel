import json
import time
from typing import Dict

try:
    import rclpy
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from sensor_msgs.msg import Image, Imu, LaserScan, PointCloud2, Range
    from std_msgs.msg import Bool, String
except ImportError:
    rclpy = None
    Node = object
    DiagnosticArray = None
    DiagnosticStatus = None
    KeyValue = None
    Odometry = None
    Image = None
    Imu = None
    LaserScan = None
    PointCloud2 = None
    Range = None
    Bool = None
    String = None

from wheelchair_diagnostics.policy import TopicRule, evaluate_watchdog


class SensorWatchdogNode(Node):
    def __init__(self):
        super().__init__("sensor_watchdog_node")
        self.declare_parameter("startup_grace_sec", 8.0)
        self.declare_parameter("publish_rate_hz", 2.0)
        self.declare_parameter("scan_timeout_sec", 0.8)
        self.declare_parameter("odom_timeout_sec", 1.0)
        self.declare_parameter("base_timeout_sec", 1.0)
        self.declare_parameter("imu_timeout_sec", 1.5)
        self.declare_parameter("ultrasonic_timeout_sec", 1.5)
        self.declare_parameter("ultrasonic_topics", ["/ultrasonic/range_0"])
        self.declare_parameter("ultrasonic_0_critical", True)
        self.declare_parameter("ultrasonic_1_critical", False)
        self.declare_parameter("camera_timeout_sec", 3.0)
        self.declare_parameter("points_timeout_sec", 1.5)
        self.declare_parameter("points_topics", ["/xtm60/points"])

        self.startup_time = time.monotonic()
        self.last_seen: Dict[str, float] = {}
        self.rules = self._make_rules()
        self.status_pub = self.create_publisher(String, "/hardware/status", 10)
        self.stop_pub = self.create_publisher(Bool, "/system_stop_required", 10)
        self.reason_pub = self.create_publisher(String, "/system_stop_reason", 10)
        self.diag_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self._create_subscriptions()
        rate = max(0.5, float(self.get_parameter("publish_rate_hz").value))
        self.timer = self.create_timer(1.0 / rate, self.tick)

    def _make_rules(self):
        rules = [
            TopicRule("/scan", float(self.get_parameter("scan_timeout_sec").value), True, "2D scan"),
            TopicRule("/wheel/odom", float(self.get_parameter("odom_timeout_sec").value), True, "wheel odom"),
            TopicRule("/base/status", float(self.get_parameter("base_timeout_sec").value), True, "base driver"),
            TopicRule("/imu/data", float(self.get_parameter("imu_timeout_sec").value), False, "H30 IMU"),
            TopicRule("/camera/front/image_raw", float(self.get_parameter("camera_timeout_sec").value), False, "front camera"),
        ]
        for index, topic in enumerate(self.get_parameter("points_topics").value):
            rules.append(
                TopicRule(
                    str(topic),
                    float(self.get_parameter("points_timeout_sec").value),
                    False,
                    f"XT-M60 points {index}",
                )
            )
        for index, topic in enumerate(self.get_parameter("ultrasonic_topics").value):
            critical_name = f"ultrasonic_{index}_critical"
            critical = (
                bool(self.get_parameter(critical_name).value)
                if self.has_parameter(critical_name)
                else index == 0
            )
            rules.append(
                TopicRule(
                    str(topic),
                    float(self.get_parameter("ultrasonic_timeout_sec").value),
                    critical,
                    f"ultrasonic {index}",
                )
            )
        return rules

    def _create_subscriptions(self):
        topic_types = {
            "/scan": LaserScan,
            "/wheel/odom": Odometry,
            "/base/status": String,
            "/imu/data": Imu,
            "/camera/front/image_raw": Image,
        }
        for topic in self.get_parameter("points_topics").value:
            topic_types[str(topic)] = PointCloud2
        for topic in self.get_parameter("ultrasonic_topics").value:
            topic_types[str(topic)] = Range
        for topic, msg_type in topic_types.items():
            self.create_subscription(msg_type, topic, lambda _msg, t=topic: self.mark_seen(t), 10)

    def mark_seen(self, topic: str):
        self.last_seen[topic] = time.monotonic()

    def tick(self):
        decision = evaluate_watchdog(
            self.rules,
            self.last_seen,
            now_sec=time.monotonic(),
            startup_time_sec=self.startup_time,
            startup_grace_sec=float(self.get_parameter("startup_grace_sec").value),
        )
        status = String()
        status.data = decision.to_json()
        self.status_pub.publish(status)

        stop = Bool()
        stop.data = decision.stop_required
        self.stop_pub.publish(stop)

        reason = String()
        reason.data = decision.reason
        self.reason_pub.publish(reason)

        diag = DiagnosticArray()
        diag.header.stamp = self.get_clock().now().to_msg()
        diag.status = []
        for topic in decision.topics:
            item = DiagnosticStatus()
            item.name = f"runtime_watchdog/{topic.label}"
            item.hardware_id = topic.name
            item.level = DiagnosticStatus.OK if topic.online else (DiagnosticStatus.ERROR if topic.critical else DiagnosticStatus.WARN)
            item.message = "fresh" if topic.online else "timeout"
            item.values = [
                KeyValue(key="topic", value=topic.name),
                KeyValue(key="age_sec", value="unknown" if topic.age_sec is None else f"{topic.age_sec:.3f}"),
                KeyValue(key="critical", value=str(topic.critical)),
            ]
            diag.status.append(item)
        self.diag_pub.publish(diag)


def main(args=None):
    if rclpy is None:
        raise RuntimeError("ROS2 Python packages are required to run this node")
    rclpy.init(args=args)
    node = SensorWatchdogNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
