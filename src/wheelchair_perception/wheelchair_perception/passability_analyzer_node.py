import json
import math
from dataclasses import dataclass
from typing import Iterable, List, Optional, Tuple

try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import String
except ImportError:
    rclpy = None
    Node = object
    LaserScan = None
    String = None


@dataclass
class PassabilityConfig:
    wheelchair_width_m: float = 0.70
    clearance_margin_m: float = 0.15
    warning_margin_m: float = 0.20
    lookahead_distance_m: float = 1.8
    min_forward_distance_m: float = 0.25
    max_side_distance_m: float = 2.0
    # Only HARD-block (zero speed) when an obstacle is imminently in front within
    # this range. Farther obstacles are left to the Nav2 costmap + the safety
    # supervisor's scan-distance stop, so the chair can still rotate/approach a
    # clear path instead of freezing for anything within lookahead.
    block_forward_distance_m: float = 0.6


@dataclass
class PassabilityResult:
    state: str
    estimated_width_m: Optional[float]
    required_width_m: float
    reason: str

    def to_json(self) -> str:
        return json.dumps(
            {
                "state": self.state,
                "estimated_width_m": self.estimated_width_m,
                "required_width_m": self.required_width_m,
                "reason": self.reason,
            },
            ensure_ascii=False,
        )


def scan_to_points(
    ranges: Iterable[float], angle_min: float, angle_increment: float
) -> List[Tuple[float, float]]:
    points = []
    for index, value in enumerate(ranges):
        if not math.isfinite(value) or value <= 0.0:
            continue
        angle = angle_min + index * angle_increment
        points.append((value * math.cos(angle), value * math.sin(angle)))
    return points


def analyze_passability(points: Iterable[Tuple[float, float]], config: PassabilityConfig) -> PassabilityResult:
    required = config.wheelchair_width_m + 2.0 * config.clearance_margin_m
    left_boundary = math.inf
    right_boundary = -math.inf
    any_left = False
    any_right = False
    front_blocked = False

    for x, y in points:
        if x < config.min_forward_distance_m or x > config.lookahead_distance_m:
            continue
        if abs(y) > config.max_side_distance_m:
            continue
        if abs(y) < required * 0.5 and x <= config.block_forward_distance_m:
            front_blocked = True
        if y > 0.0:
            left_boundary = min(left_boundary, y)
            any_left = True
        elif y < 0.0:
            right_boundary = max(right_boundary, y)
            any_right = True

    if front_blocked:
        return PassabilityResult("BLOCKED", 0.0, required, "obstacle lies inside required wheelchair corridor")
    if not any_left or not any_right:
        return PassabilityResult("UNKNOWN", None, required, "both corridor boundaries were not observed")

    width = left_boundary - right_boundary
    if width < required:
        return PassabilityResult("BLOCKED", width, required, f"corridor width {width:.2f} m below required {required:.2f} m")
    if width < required + config.warning_margin_m:
        return PassabilityResult("NARROW", width, required, f"corridor width {width:.2f} m is near minimum")
    return PassabilityResult("CLEAR", width, required, f"corridor width {width:.2f} m is passable")


class PassabilityAnalyzerNode(Node):
    def __init__(self):
        super().__init__("passability_analyzer_node")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("status_topic", "/passability/status")
        self.declare_parameter("wheelchair_width_m", 0.70)
        self.declare_parameter("clearance_margin_m", 0.15)
        self.declare_parameter("warning_margin_m", 0.20)
        self.declare_parameter("lookahead_distance_m", 1.8)
        self.declare_parameter("min_forward_distance_m", 0.25)
        self.declare_parameter("max_side_distance_m", 2.0)
        self.declare_parameter("block_forward_distance_m", 0.6)

        self.config = PassabilityConfig(
            wheelchair_width_m=float(self.get_parameter("wheelchair_width_m").value),
            clearance_margin_m=float(self.get_parameter("clearance_margin_m").value),
            warning_margin_m=float(self.get_parameter("warning_margin_m").value),
            lookahead_distance_m=float(self.get_parameter("lookahead_distance_m").value),
            min_forward_distance_m=float(self.get_parameter("min_forward_distance_m").value),
            max_side_distance_m=float(self.get_parameter("max_side_distance_m").value),
            block_forward_distance_m=float(self.get_parameter("block_forward_distance_m").value),
        )
        self.pub = self.create_publisher(String, self.get_parameter("status_topic").value, 10)
        self.create_subscription(LaserScan, self.get_parameter("scan_topic").value, self.on_scan, 10)

    def on_scan(self, msg):
        points = scan_to_points(msg.ranges, msg.angle_min, msg.angle_increment)
        result = analyze_passability(points, self.config)
        status = String()
        status.data = result.to_json()
        self.pub.publish(status)


def main(args=None):
    if rclpy is None:
        raise RuntimeError("ROS2 Python packages are required to run this node")
    rclpy.init(args=args)
    node = PassabilityAnalyzerNode()
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
