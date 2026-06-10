#!/usr/bin/env python3
"""Low-speed reactive exploration for first-pass autonomous mapping.

This node is intentionally simpler than frontier exploration: it does not need
a mature map before motion starts. It reads /scan, drives forward slowly while
the front sector is open, and rotates toward the clearer side when the front
sector is blocked or the chair appears stuck. It publishes /cmd_vel_nav, so the
base command still flows through the configured safety_supervisor and then to
/cmd_vel_safe.
"""
import math
from dataclasses import dataclass
from typing import Dict, Iterable, Optional, Sequence, Tuple

try:
    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from sensor_msgs.msg import LaserScan, Range
    from std_msgs.msg import Bool, String
    from visualization_msgs.msg import Marker
except ImportError:  # allow unit tests / py_compile without ROS installed
    rclpy = None
    Node = object
    Twist = LaserScan = Range = Odometry = Bool = String = Marker = None


@dataclass(frozen=True)
class ReactiveExplorerConfig:
    forward_speed: float = 0.05
    turn_speed: float = 0.22
    turn_trigger_distance: float = 0.75
    hard_stop_distance: float = 0.10
    front_angle_min_deg: float = -60.0
    front_angle_max_deg: float = 60.0
    left_angle_min_deg: float = 20.0
    left_angle_max_deg: float = 95.0
    right_angle_min_deg: float = -95.0
    right_angle_max_deg: float = -20.0
    corridor_half_width_m: float = 0.45
    corridor_lookahead_m: float = 1.20
    side_trigger_distance: float = 0.10
    ultrasonic_stale_timeout_sec: float = 1.0
    ultrasonic_min_valid_m: float = 0.03
    min_turn_sec: float = 1.2
    stuck_timeout_sec: float = 4.0
    stuck_min_motion_m: float = 0.03
    stale_scan_timeout_sec: float = 1.0


def sector_min(
    ranges: Sequence[float],
    angle_min: float,
    angle_increment: float,
    min_deg: float,
    max_deg: float,
) -> float:
    lo = math.radians(min_deg)
    hi = math.radians(max_deg)
    nearest = math.inf
    for index, value in enumerate(ranges):
        if not math.isfinite(value) or value <= 0.0:
            continue
        angle = angle_min + index * angle_increment
        if lo <= angle <= hi:
            nearest = min(nearest, float(value))
    return nearest


def corridor_min_x_distance(
    ranges: Sequence[float],
    angle_min: float,
    angle_increment: float,
    half_width_m: float,
    lookahead_m: float,
) -> float:
    """Nearest forward x distance inside the wheelchair's swept corridor.

    A thin chair leg can be outside a narrow angular cone but still inside the
    physical body width. Corridor checking catches that case better than a
    simple "front sector min range".
    """
    nearest = math.inf
    for index, value in enumerate(ranges):
        if not math.isfinite(value) or value <= 0.0:
            continue
        angle = angle_min + index * angle_increment
        x = float(value) * math.cos(angle)
        y = float(value) * math.sin(angle)
        if x <= 0.0 or x > lookahead_m:
            continue
        if abs(y) <= half_width_m:
            nearest = min(nearest, x)
    return nearest


def choose_turn_direction(left_clearance: float, right_clearance: float) -> int:
    """Return +1 for left turn, -1 for right turn."""
    if left_clearance == right_clearance:
        return 1
    return 1 if left_clearance > right_clearance else -1


def min_valid(values: Iterable[float], min_valid_m: float = 0.03) -> float:
    valid = [
        float(value)
        for value in values
        if math.isfinite(float(value)) and float(value) > 0.0 and float(value) >= min_valid_m
    ]
    return min(valid, default=math.inf)


def combine_clearances(
    front: float,
    left: float,
    right: float,
    left_front_ultra: float = math.inf,
    left_side_ultra: float = math.inf,
    right_front_ultra: float = math.inf,
    right_side_ultra: float = math.inf,
) -> Tuple[float, float, float]:
    combined_front = min(front, left_front_ultra, right_front_ultra)
    combined_left = min(left, left_front_ultra, left_side_ultra)
    combined_right = min(right, right_front_ultra, right_side_ultra)
    return combined_front, combined_left, combined_right


def reactive_command(
    front: float,
    left: float,
    right: float,
    config: ReactiveExplorerConfig,
    forced_turn_direction: Optional[int] = None,
) -> Tuple[float, float, str, int]:
    direction = forced_turn_direction or choose_turn_direction(left, right)
    if left <= config.side_trigger_distance or right <= config.side_trigger_distance:
        direction = -1 if left < right else 1
        return 0.0, direction * config.turn_speed, "TURN_SIDE: ultrasonic side clearance below 10cm", direction
    if front <= config.hard_stop_distance:
        return 0.0, direction * config.turn_speed, "TURN_HARD: obstacle inside 10cm floor", direction
    if front <= config.turn_trigger_distance:
        return 0.0, direction * config.turn_speed, "TURN: front blocked", direction

    # While driving forward, gently bias toward the more open side. This gives a
    # vacuum-like wandering path instead of a perfectly straight line into the
    # next wall.
    side_delta = left - right
    if math.isfinite(side_delta) and abs(side_delta) > 0.35:
        angular = 0.25 * config.turn_speed * (1 if side_delta > 0 else -1)
    else:
        angular = 0.0
    return config.forward_speed, angular, "FORWARD: front clear", direction


class ReactiveExplorerNode(Node):
    def __init__(self):
        super().__init__("reactive_explorer_node")
        p = self.declare_parameter
        p("auto_start", True)
        p("require_enable_signal", False)
        p("enable_topic", "/autonomy/enable")
        p("scan_topic", "/scan")
        p("odom_topic", "/wheel/odom")
        p("cmd_vel_topic", "/cmd_vel_nav")
        p("status_topic", "/exploration/status")
        p("status_marker_topic", "/exploration/status_marker")
        p("status_marker_frame", "base_link")
        p("forward_speed", 0.05)
        p("turn_speed", 0.22)
        p("turn_trigger_distance", 0.75)
        p("hard_stop_distance", 0.10)
        p("front_angle_min_deg", -60.0)
        p("front_angle_max_deg", 60.0)
        p("left_angle_min_deg", 20.0)
        p("left_angle_max_deg", 95.0)
        p("right_angle_min_deg", -95.0)
        p("right_angle_max_deg", -20.0)
        p("corridor_half_width_m", 0.45)
        p("corridor_lookahead_m", 1.20)
        p("side_trigger_distance", 0.10)
        p("ultrasonic_stale_timeout_sec", 1.0)
        p("ultrasonic_min_valid_m", 0.03)
        p("left_front_ultrasonic_topic", "/ultrasonic/range_0")
        p("left_side_ultrasonic_topic", "/ultrasonic/range_1")
        p("right_front_ultrasonic_topic", "/ultrasonic/range_2")
        p("right_side_ultrasonic_topic", "/ultrasonic/range_3")
        p("min_turn_sec", 1.2)
        p("stuck_timeout_sec", 4.0)
        p("stuck_min_motion_m", 0.03)
        p("stale_scan_timeout_sec", 1.0)
        p("publish_rate_hz", 10.0)

        self.auto_start = bool(self.get_parameter("auto_start").value)
        self.require_enable_signal = bool(self.get_parameter("require_enable_signal").value)
        self.enable_received = not self.require_enable_signal
        self.config = ReactiveExplorerConfig(
            forward_speed=float(self.get_parameter("forward_speed").value),
            turn_speed=float(self.get_parameter("turn_speed").value),
            turn_trigger_distance=float(self.get_parameter("turn_trigger_distance").value),
            hard_stop_distance=float(self.get_parameter("hard_stop_distance").value),
            front_angle_min_deg=float(self.get_parameter("front_angle_min_deg").value),
            front_angle_max_deg=float(self.get_parameter("front_angle_max_deg").value),
            left_angle_min_deg=float(self.get_parameter("left_angle_min_deg").value),
            left_angle_max_deg=float(self.get_parameter("left_angle_max_deg").value),
            right_angle_min_deg=float(self.get_parameter("right_angle_min_deg").value),
            right_angle_max_deg=float(self.get_parameter("right_angle_max_deg").value),
            corridor_half_width_m=float(self.get_parameter("corridor_half_width_m").value),
            corridor_lookahead_m=float(self.get_parameter("corridor_lookahead_m").value),
            side_trigger_distance=float(self.get_parameter("side_trigger_distance").value),
            ultrasonic_stale_timeout_sec=float(self.get_parameter("ultrasonic_stale_timeout_sec").value),
            ultrasonic_min_valid_m=float(self.get_parameter("ultrasonic_min_valid_m").value),
            min_turn_sec=float(self.get_parameter("min_turn_sec").value),
            stuck_timeout_sec=float(self.get_parameter("stuck_timeout_sec").value),
            stuck_min_motion_m=float(self.get_parameter("stuck_min_motion_m").value),
            stale_scan_timeout_sec=float(self.get_parameter("stale_scan_timeout_sec").value),
        )

        self.latest_scan = None
        self.last_scan_time = None
        self.ultrasonic_ranges: Dict[str, float] = {}
        self.ultrasonic_seen_at: Dict[str, float] = {}
        self.ultrasonic_roles = {
            "left_front": str(self.get_parameter("left_front_ultrasonic_topic").value),
            "left_side": str(self.get_parameter("left_side_ultrasonic_topic").value),
            "right_front": str(self.get_parameter("right_front_ultrasonic_topic").value),
            "right_side": str(self.get_parameter("right_side_ultrasonic_topic").value),
        }
        self.last_odom_xy = None
        self.forward_started_at = None
        self.forward_start_xy = None
        self.turn_until = 0.0
        self.turn_direction = 1

        self.cmd_pub = self.create_publisher(Twist, self.get_parameter("cmd_vel_topic").value, 10)
        self.status_pub = self.create_publisher(String, self.get_parameter("status_topic").value, 10)
        self.status_marker_pub = self.create_publisher(
            Marker, self.get_parameter("status_marker_topic").value, 10
        )
        self.create_subscription(LaserScan, self.get_parameter("scan_topic").value, self.on_scan, 10)
        self.create_subscription(Odometry, self.get_parameter("odom_topic").value, self.on_odom, 10)
        self.create_subscription(
            Bool,
            str(self.get_parameter("enable_topic").value),
            self.on_enable,
            10,
        )
        for topic in dict.fromkeys(self.ultrasonic_roles.values()):
            if topic:
                self.create_subscription(Range, topic, lambda msg, t=topic: self.on_ultrasonic(t, msg), 10)
        rate = max(1.0, float(self.get_parameter("publish_rate_hz").value))
        self.create_timer(1.0 / rate, self.tick)

    def now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def on_scan(self, msg):
        self.latest_scan = msg
        self.last_scan_time = self.now()

    def on_odom(self, msg):
        p = msg.pose.pose.position
        self.last_odom_xy = (float(p.x), float(p.y))

    def on_ultrasonic(self, topic: str, msg):
        self.ultrasonic_ranges[topic] = float(msg.range)
        self.ultrasonic_seen_at[topic] = self.now()

    def on_enable(self, msg):
        self.enable_received = bool(msg.data)

    def _ultrasonic(self, role: str, now: float) -> float:
        topic = self.ultrasonic_roles.get(role, "")
        if not topic:
            return math.inf
        seen_at = self.ultrasonic_seen_at.get(topic)
        if seen_at is None or now - seen_at > self.config.ultrasonic_stale_timeout_sec:
            return math.inf
        return min_valid([self.ultrasonic_ranges.get(topic, math.inf)], self.config.ultrasonic_min_valid_m)

    def _publish(self, linear: float, angular: float, status: str):
        cmd = Twist()
        cmd.linear.x = float(linear)
        cmd.angular.z = float(angular)
        self.cmd_pub.publish(cmd)
        self._publish_status(status)

    def _publish_status(self, status: str):
        self.status_pub.publish(String(data=status))

        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = str(self.get_parameter("status_marker_frame").value)
        marker.ns = "reactive_explorer_status"
        marker.id = 0
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.z = 1.35
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.16
        marker.color.a = 1.0
        if status.startswith("FORWARD"):
            marker.color.g = 1.0
        elif status.startswith(("TURN", "TURNING")):
            marker.color.r = 1.0
            marker.color.g = 0.75
        elif status.startswith("IDLE"):
            marker.color.r = 0.6
            marker.color.g = 0.8
            marker.color.b = 1.0
        else:
            marker.color.r = 1.0
            marker.color.g = 0.2
            marker.color.b = 0.2
        marker.text = status
        self.status_marker_pub.publish(marker)

    def _distance_from_forward_start(self) -> float:
        if self.last_odom_xy is None or self.forward_start_xy is None:
            return math.inf
        return math.hypot(
            self.last_odom_xy[0] - self.forward_start_xy[0],
            self.last_odom_xy[1] - self.forward_start_xy[1],
        )

    def _stuck(self, now: float) -> bool:
        if self.forward_started_at is None:
            return False
        if now - self.forward_started_at < self.config.stuck_timeout_sec:
            return False
        return self._distance_from_forward_start() < self.config.stuck_min_motion_m

    def _start_forward_window(self, now: float):
        if self.forward_started_at is None:
            self.forward_started_at = now
            self.forward_start_xy = self.last_odom_xy

    def _clear_forward_window(self):
        self.forward_started_at = None
        self.forward_start_xy = None

    def tick(self):
        now = self.now()
        if not self.auto_start:
            self._publish_status("IDLE: reactive exploration disabled")
            return
        if not self.enable_received:
            self._clear_forward_window()
            self._publish(0.0, 0.0, "IDLE: waiting for explicit autonomy enable")
            return
        if self.latest_scan is None or self.last_scan_time is None:
            self._publish(0.0, 0.0, "BLOCKED: waiting for /scan")
            return
        scan_age = now - self.last_scan_time
        if scan_age > self.config.stale_scan_timeout_sec:
            self._publish(0.0, 0.0, f"BLOCKED: scan stale for {scan_age:.2f}s")
            return

        scan = self.latest_scan
        front = sector_min(
            scan.ranges,
            scan.angle_min,
            scan.angle_increment,
            self.config.front_angle_min_deg,
            self.config.front_angle_max_deg,
        )
        left = sector_min(
            scan.ranges,
            scan.angle_min,
            scan.angle_increment,
            self.config.left_angle_min_deg,
            self.config.left_angle_max_deg,
        )
        right = sector_min(
            scan.ranges,
            scan.angle_min,
            scan.angle_increment,
            self.config.right_angle_min_deg,
            self.config.right_angle_max_deg,
        )
        corridor = corridor_min_x_distance(
            scan.ranges,
            scan.angle_min,
            scan.angle_increment,
            self.config.corridor_half_width_m,
            self.config.corridor_lookahead_m,
        )
        left_front_ultra = self._ultrasonic("left_front", now)
        left_side_ultra = self._ultrasonic("left_side", now)
        right_front_ultra = self._ultrasonic("right_front", now)
        right_side_ultra = self._ultrasonic("right_side", now)
        front_clearance, left_clearance, right_clearance = combine_clearances(
            min(front, corridor),
            left,
            right,
            left_front_ultra,
            left_side_ultra,
            right_front_ultra,
            right_side_ultra,
        )

        if now < self.turn_until:
            self._clear_forward_window()
            self._publish(
                0.0,
                self.turn_direction * self.config.turn_speed,
                (
                    f"TURNING: front={front_clearance:.2f} scan_front={front:.2f} "
                    f"corridor={corridor:.2f} left={left_clearance:.2f} right={right_clearance:.2f} "
                    f"ultra_lf={left_front_ultra:.2f} ultra_ls={left_side_ultra:.2f} "
                    f"ultra_rf={right_front_ultra:.2f} ultra_rs={right_side_ultra:.2f}"
                ),
            )
            return

        if self._stuck(now):
            self.turn_direction = choose_turn_direction(left_clearance, right_clearance)
            self.turn_until = now + self.config.min_turn_sec * 1.5
            self._clear_forward_window()
            self._publish(0.0, self.turn_direction * self.config.turn_speed, "TURN_STUCK: odom did not advance")
            return

        linear, angular, reason, direction = reactive_command(
            front_clearance, left_clearance, right_clearance, self.config
        )
        self.turn_direction = direction
        if abs(linear) < 1e-4 and abs(angular) > 1e-4:
            self.turn_until = now + self.config.min_turn_sec
            self._clear_forward_window()
        elif linear > 1e-4:
            self._start_forward_window(now)
        else:
            self._clear_forward_window()

        self._publish(
            linear,
            angular,
            (
                f"{reason}; front={front_clearance:.2f} scan_front={front:.2f} "
                f"corridor={corridor:.2f} left={left_clearance:.2f} right={right_clearance:.2f} "
                f"ultra_lf={left_front_ultra:.2f} ultra_ls={left_side_ultra:.2f} "
                f"ultra_rf={right_front_ultra:.2f} ultra_rs={right_side_ultra:.2f}"
            ),
        )


def main(args=None):
    if rclpy is None:
        raise RuntimeError("ROS2 Python packages are required to run this node")
    rclpy.init(args=args)
    node = ReactiveExplorerNode()
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
