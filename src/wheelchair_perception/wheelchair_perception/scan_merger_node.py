import math
from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional, Sequence

try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import LaserScan
except ImportError:
    rclpy = None
    Node = object
    LaserScan = None


@dataclass(frozen=True)
class ScanSlice:
    angle_min: float
    angle_increment: float
    range_min: float
    range_max: float
    ranges: Sequence[float]


@dataclass(frozen=True)
class MergeConfig:
    angle_min: float = -1.5708
    angle_max: float = 1.5708
    angle_increment: float = 0.0087
    range_min: float = 0.08
    range_max: float = 8.0
    use_inf: bool = True

    @property
    def beam_count(self) -> int:
        return int(math.ceil((self.angle_max - self.angle_min) / self.angle_increment)) + 1


def merge_scan_slices(scans: Iterable[ScanSlice], config: MergeConfig) -> List[float]:
    if config.angle_increment <= 0.0:
        raise ValueError("angle_increment must be positive")
    if config.angle_max <= config.angle_min:
        raise ValueError("angle_max must be greater than angle_min")

    empty = math.inf if config.use_inf else config.range_max + 1.0
    merged = [empty for _ in range(config.beam_count)]
    for scan in scans:
        for index, value in enumerate(scan.ranges):
            if not math.isfinite(value):
                continue
            if value < max(config.range_min, scan.range_min):
                continue
            if value > min(config.range_max, scan.range_max):
                continue
            angle = scan.angle_min + index * scan.angle_increment
            if angle < config.angle_min or angle > config.angle_max:
                continue
            output_index = int(round((angle - config.angle_min) / config.angle_increment))
            if 0 <= output_index < len(merged) and value < merged[output_index]:
                merged[output_index] = float(value)
    return merged


class ScanMergerNode(Node):
    def __init__(self):
        super().__init__("scan_merger_node")
        self.declare_parameter("input_topics", ["/scan_left", "/scan_right"])
        self.declare_parameter("output_topic", "/scan")
        self.declare_parameter("frame_id", "base_link")
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("stale_timeout_sec", 0.5)
        self.declare_parameter("require_all_sources", False)
        self.declare_parameter("angle_min", -1.5708)
        self.declare_parameter("angle_max", 1.5708)
        self.declare_parameter("angle_increment", 0.0087)
        self.declare_parameter("range_min", 0.08)
        self.declare_parameter("range_max", 8.0)
        self.declare_parameter("scan_time", 0.1)
        self.declare_parameter("use_inf", True)

        self.config = MergeConfig(
            angle_min=float(self.get_parameter("angle_min").value),
            angle_max=float(self.get_parameter("angle_max").value),
            angle_increment=float(self.get_parameter("angle_increment").value),
            range_min=float(self.get_parameter("range_min").value),
            range_max=float(self.get_parameter("range_max").value),
            use_inf=bool(self.get_parameter("use_inf").value),
        )
        self.input_topics = [str(topic) for topic in self.get_parameter("input_topics").value]
        self.latest: Dict[str, LaserScan] = {}
        self.seen_at: Dict[str, object] = {}
        self.output_pub = self.create_publisher(
            LaserScan, str(self.get_parameter("output_topic").value), 10
        )
        for topic in self.input_topics:
            self.create_subscription(LaserScan, topic, lambda msg, t=topic: self.on_scan(t, msg), 10)

        rate = max(1.0, float(self.get_parameter("publish_rate_hz").value))
        self.timer = self.create_timer(1.0 / rate, self.tick)

    def on_scan(self, topic: str, msg):
        self.latest[topic] = msg
        self.seen_at[topic] = self.get_clock().now()

    def tick(self):
        now = self.get_clock().now()
        stale_timeout = float(self.get_parameter("stale_timeout_sec").value)
        fresh_scans = []
        stale_topics = []
        for topic in self.input_topics:
            msg = self.latest.get(topic)
            seen_at = self.seen_at.get(topic)
            if msg is None or seen_at is None:
                stale_topics.append(topic)
                continue
            age = (now - seen_at).nanoseconds / 1e9
            if age > stale_timeout:
                stale_topics.append(f"{topic}({age:.2f}s)")
                continue
            fresh_scans.append(
                ScanSlice(
                    angle_min=float(msg.angle_min),
                    angle_increment=float(msg.angle_increment),
                    range_min=float(msg.range_min),
                    range_max=float(msg.range_max),
                    ranges=msg.ranges,
                )
            )

        if not fresh_scans:
            return
        if bool(self.get_parameter("require_all_sources").value) and stale_topics:
            self.get_logger().warning(
                "not publishing merged scan because sources are stale: " + ", ".join(stale_topics)
            )
            return

        output = LaserScan()
        output.header.stamp = now.to_msg()
        output.header.frame_id = str(self.get_parameter("frame_id").value)
        output.angle_min = self.config.angle_min
        output.angle_max = self.config.angle_max
        output.angle_increment = self.config.angle_increment
        output.time_increment = 0.0
        output.scan_time = float(self.get_parameter("scan_time").value)
        output.range_min = self.config.range_min
        output.range_max = self.config.range_max
        output.ranges = merge_scan_slices(fresh_scans, self.config)
        output.intensities = []
        self.output_pub.publish(output)


def main(args=None):
    if rclpy is None:
        raise RuntimeError("ROS2 Python packages are required to run this node")
    rclpy.init(args=args)
    node = ScanMergerNode()
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
