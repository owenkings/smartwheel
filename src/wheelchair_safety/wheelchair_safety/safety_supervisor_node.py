import json
import math
from dataclasses import dataclass
from typing import Dict, Iterable, Optional

try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Twist
    from sensor_msgs.msg import LaserScan, Range
    from std_msgs.msg import Bool, String
except ImportError:
    rclpy = None
    Node = object
    Twist = None
    LaserScan = None
    Range = None
    Bool = None
    String = None


@dataclass
class SafetyParams:
    max_auto_speed: float = 0.4
    hard_max_speed: float = 0.8
    max_angular_speed: float = 0.8
    allow_reverse_motion: bool = False
    max_reverse_speed: float = 0.15
    rotate_in_place_angular_threshold: float = 0.2
    rotation_stop_distance: float = 0.7
    warning_distance: float = 1.5
    slowdown_distance: float = 1.0
    stop_distance: float = 0.6
    emergency_distance: float = 0.35
    t_delay: float = 0.35
    a_brake: float = 0.8
    d_margin: float = 0.25
    front_angle_min_deg: float = -35.0
    front_angle_max_deg: float = 35.0


@dataclass
class SafetyDecision:
    state: str
    reason: str
    linear_x: float
    angular_z: float
    min_scan_distance: float
    min_ultrasonic_distance: float
    dynamic_stop_distance: float


def compute_dynamic_stop_distance(
    velocity: float, t_delay: float = 0.35, a_brake: float = 0.8, d_margin: float = 0.25
) -> float:
    """d_safe = v * t_delay + v^2 / (2 * a_brake) + d_margin."""
    v = max(0.0, abs(float(velocity)))
    brake = max(0.05, float(a_brake))
    return v * float(t_delay) + (v * v) / (2.0 * brake) + float(d_margin)


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def min_front_scan_distance(
    ranges: Iterable[float],
    angle_min: float,
    angle_increment: float,
    front_angle_min_deg: float,
    front_angle_max_deg: float,
) -> float:
    min_angle = math.radians(front_angle_min_deg)
    max_angle = math.radians(front_angle_max_deg)
    nearest = math.inf
    for index, value in enumerate(ranges):
        if not math.isfinite(value) or value <= 0.0:
            continue
        angle = angle_min + index * angle_increment
        if min_angle <= angle <= max_angle:
            nearest = min(nearest, value)
    return nearest


def evaluate_safety(
    requested_linear_x: float,
    requested_angular_z: float,
    scan_distance: float,
    ultrasonic_distances: Iterable[float],
    emergency_hw: bool,
    emergency_sw: bool,
    params: SafetyParams,
) -> SafetyDecision:
    min_linear = -min(params.max_auto_speed, params.max_reverse_speed)
    if not params.allow_reverse_motion:
        min_linear = 0.0
    capped_linear = clamp(
        requested_linear_x,
        min_linear,
        params.max_auto_speed,
    )
    capped_angular = clamp(
        requested_angular_z,
        -params.max_angular_speed,
        params.max_angular_speed,
    )
    if abs(capped_linear) > params.hard_max_speed:
        capped_linear = math.copysign(params.hard_max_speed, capped_linear)

    valid_ultra = [d for d in ultrasonic_distances if math.isfinite(d) and d > 0.0]
    min_ultra = min(valid_ultra, default=math.inf)
    dynamic_stop = compute_dynamic_stop_distance(
        capped_linear, params.t_delay, params.a_brake, params.d_margin
    )
    stop_threshold = max(params.stop_distance, dynamic_stop)

    if emergency_hw:
        return SafetyDecision(
            "EMERGENCY_STOP",
            "physical emergency stop input is active",
            0.0,
            0.0,
            scan_distance,
            min_ultra,
            dynamic_stop,
        )
    if emergency_sw:
        return SafetyDecision(
            "EMERGENCY_STOP",
            "software stop command is active",
            0.0,
            0.0,
            scan_distance,
            min_ultra,
            dynamic_stop,
        )
    if requested_linear_x < -1e-3 and not params.allow_reverse_motion:
        return SafetyDecision(
            "STOP",
            "reverse motion is disabled for automatic navigation",
            0.0,
            0.0,
            scan_distance,
            min_ultra,
            dynamic_stop,
        )
    if min_ultra < params.emergency_distance:
        return SafetyDecision(
            "EMERGENCY_STOP",
            f"ultrasonic distance {min_ultra:.2f} m below emergency threshold",
            0.0,
            0.0,
            scan_distance,
            min_ultra,
            dynamic_stop,
        )
    if scan_distance < params.emergency_distance:
        return SafetyDecision(
            "EMERGENCY_STOP",
            f"front scan distance {scan_distance:.2f} m below emergency threshold",
            0.0,
            0.0,
            scan_distance,
            min_ultra,
            dynamic_stop,
        )
    if (
        abs(capped_angular) >= params.rotate_in_place_angular_threshold
        and abs(capped_linear) < 0.05
        and min_ultra < params.rotation_stop_distance
    ):
        return SafetyDecision(
            "STOP",
            f"rotation blocked by nearby ultrasonic obstacle at {min_ultra:.2f} m",
            0.0,
            0.0,
            scan_distance,
            min_ultra,
            dynamic_stop,
        )
    if scan_distance < stop_threshold or min_ultra < params.stop_distance:
        return SafetyDecision(
            "STOP",
            f"obstacle within stop threshold {stop_threshold:.2f} m",
            0.0,
            0.0,
            scan_distance,
            min_ultra,
            dynamic_stop,
        )
    if scan_distance < params.slowdown_distance or min_ultra < params.slowdown_distance:
        nearest = min(scan_distance, min_ultra)
        span = max(0.01, params.slowdown_distance - stop_threshold)
        scale = clamp((nearest - stop_threshold) / span, 0.15, 0.6)
        return SafetyDecision(
            "SLOWDOWN",
            f"obstacle within slowdown zone at {nearest:.2f} m",
            capped_linear * scale,
            capped_angular * scale,
            scan_distance,
            min_ultra,
            dynamic_stop,
        )
    if scan_distance < params.warning_distance or min_ultra < params.warning_distance:
        nearest = min(scan_distance, min_ultra)
        return SafetyDecision(
            "WARNING",
            f"obstacle within warning zone at {nearest:.2f} m",
            capped_linear,
            capped_angular,
            scan_distance,
            min_ultra,
            dynamic_stop,
        )

    return SafetyDecision(
        "CLEAR",
        "no obstacle inside configured safety zones",
        capped_linear,
        capped_angular,
        scan_distance,
        min_ultra,
        dynamic_stop,
    )


class SafetySupervisorNode(Node):
    def __init__(self):
        super().__init__("safety_supervisor_node")
        self.declare_parameter("cmd_vel_nav_topic", "/cmd_vel_nav")
        self.declare_parameter("cmd_vel_safe_topic", "/cmd_vel_safe")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("safety_state_topic", "/safety_state")
        self.declare_parameter("emergency_stop_hw_topic", "/emergency_stop_hw")
        self.declare_parameter("emergency_stop_sw_topic", "/emergency_stop_sw")
        self.declare_parameter("system_stop_required_topic", "/system_stop_required")
        self.declare_parameter("system_stop_reason_topic", "/system_stop_reason")
        self.declare_parameter("passability_status_topic", "/passability/status")
        self.declare_parameter("localization_healthy_topic", "/localization/is_healthy")
        self.declare_parameter("localization_health_topic", "/localization/health")
        self.declare_parameter("require_localization_healthy", False)
        self.declare_parameter("localization_timeout_sec", 2.5)
        self.declare_parameter(
            "ultrasonic_topics",
            [
                "/ultrasonic/range_0",
                "/ultrasonic/range_1",
                "/ultrasonic/range_2",
                "/ultrasonic/range_3",
                "/ultrasonic/range_4",
                "/ultrasonic/range_5",
            ],
        )
        for field, default in SafetyParams().__dict__.items():
            self.declare_parameter(field, default)
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("stale_timeout_sec", 1.0)
        self.declare_parameter("ultrasonic_stale_timeout_sec", 1.5)
        self.declare_parameter("min_ultrasonic_online", 2)

        self.params = SafetyParams(
            **self._load_safety_params()
        )
        self.stale_timeout_sec = float(self.get_parameter("stale_timeout_sec").value)
        self.ultrasonic_stale_timeout_sec = float(
            self.get_parameter("ultrasonic_stale_timeout_sec").value
        )
        self.min_ultrasonic_online = int(self.get_parameter("min_ultrasonic_online").value)
        self.require_localization_healthy = bool(
            self.get_parameter("require_localization_healthy").value
        )
        self.localization_timeout_sec = float(
            self.get_parameter("localization_timeout_sec").value
        )

        self.latest_cmd = (0.0, 0.0)
        self.latest_scan_distance = math.inf
        self.ultrasonic_ranges: Dict[str, float] = {}
        self.emergency_hw = False
        self.emergency_sw = False
        self.system_stop_required = False
        self.system_stop_reason = ""
        self.passability_state = "UNKNOWN"
        self.passability_reason = ""
        self.last_scan_time = self.get_clock().now()
        self.ultrasonic_seen_at: Dict[str, Optional[object]] = {}
        self.localization_healthy: Optional[bool] = None
        self.localization_reason = "no localization health message received"
        self.last_localization_time: Optional[object] = None

        self.cmd_pub = self.create_publisher(
            Twist, self.get_parameter("cmd_vel_safe_topic").value, 10
        )
        self.state_pub = self.create_publisher(
            String, self.get_parameter("safety_state_topic").value, 10
        )
        self.create_subscription(
            Twist,
            self.get_parameter("cmd_vel_nav_topic").value,
            self.on_cmd_vel,
            10,
        )
        self.create_subscription(
            LaserScan, self.get_parameter("scan_topic").value, self.on_scan, 10
        )
        self.create_subscription(
            Bool,
            self.get_parameter("emergency_stop_hw_topic").value,
            self.on_emergency_hw,
            10,
        )
        self.create_subscription(
            Bool,
            self.get_parameter("emergency_stop_sw_topic").value,
            self.on_emergency_sw,
            10,
        )
        self.create_subscription(
            Bool,
            self.get_parameter("system_stop_required_topic").value,
            self.on_system_stop_required,
            10,
        )
        self.create_subscription(
            String,
            self.get_parameter("system_stop_reason_topic").value,
            self.on_system_stop_reason,
            10,
        )
        self.create_subscription(
            String,
            self.get_parameter("passability_status_topic").value,
            self.on_passability_status,
            10,
        )
        self.create_subscription(
            Bool,
            self.get_parameter("localization_healthy_topic").value,
            self.on_localization_healthy,
            10,
        )
        self.create_subscription(
            String,
            self.get_parameter("localization_health_topic").value,
            self.on_localization_health,
            10,
        )
        for topic in self.get_parameter("ultrasonic_topics").value:
            self.ultrasonic_ranges[topic] = math.inf
            self.ultrasonic_seen_at[topic] = None
            self.create_subscription(
                Range, topic, lambda msg, t=topic: self.on_ultrasonic(t, msg), 10
            )

        rate = max(1.0, float(self.get_parameter("publish_rate_hz").value))
        self.timer = self.create_timer(1.0 / rate, self.publish_safe_command)

    def _load_safety_params(self):
        values = {}
        for field, default in SafetyParams().__dict__.items():
            value = self.get_parameter(field).value
            values[field] = bool(value) if isinstance(default, bool) else float(value)
        return values

    def on_cmd_vel(self, msg):
        self.latest_cmd = (float(msg.linear.x), float(msg.angular.z))

    def on_scan(self, msg):
        self.latest_scan_distance = min_front_scan_distance(
            msg.ranges,
            msg.angle_min,
            msg.angle_increment,
            self.params.front_angle_min_deg,
            self.params.front_angle_max_deg,
        )
        self.last_scan_time = self.get_clock().now()

    def on_ultrasonic(self, topic: str, msg):
        self.ultrasonic_ranges[topic] = float(msg.range)
        self.ultrasonic_seen_at[topic] = self.get_clock().now()

    def on_emergency_hw(self, msg):
        self.emergency_hw = bool(msg.data)

    def on_emergency_sw(self, msg):
        self.emergency_sw = bool(msg.data)

    def on_system_stop_required(self, msg):
        self.system_stop_required = bool(msg.data)

    def on_system_stop_reason(self, msg):
        self.system_stop_reason = msg.data

    def on_passability_status(self, msg):
        try:
            payload = json.loads(msg.data)
            self.passability_state = payload.get("state", "UNKNOWN")
            self.passability_reason = payload.get("reason", "")
        except Exception:
            self.passability_state = "UNKNOWN"
            self.passability_reason = "invalid passability status payload"

    def on_localization_healthy(self, msg):
        self.localization_healthy = bool(msg.data)
        self.last_localization_time = self.get_clock().now()

    def on_localization_health(self, msg):
        try:
            payload = json.loads(msg.data)
            self.localization_reason = payload.get("reason", msg.data)
            if "healthy" in payload:
                self.localization_healthy = bool(payload["healthy"])
            self.last_localization_time = self.get_clock().now()
        except Exception:
            self.localization_reason = msg.data
            self.last_localization_time = self.get_clock().now()

    def publish_safe_command(self):
        now = self.get_clock().now()
        scan_distance = self.latest_scan_distance
        age = (now - self.last_scan_time).nanoseconds / 1e9
        stale_reason = ""
        if age > self.stale_timeout_sec:
            self._publish_zero_state(f"SENSOR_FAULT: scan stale for {age:.2f}s")
            return

        ultrasonic_fault = self._ultrasonic_fault_reason(now)
        if ultrasonic_fault:
            self._publish_zero_state(f"SENSOR_FAULT: {ultrasonic_fault}")
            return

        localization_fault = self._localization_fault_reason(now)
        if localization_fault:
            self._publish_zero_state(f"SENSOR_FAULT: {localization_fault}")
            return

        if self.system_stop_required:
            self._publish_zero_state(
                f"SENSOR_FAULT: diagnostic stop required; {self.system_stop_reason}"
            )
            return

        if self.passability_state == "BLOCKED":
            self._publish_zero_state(f"STOP: passability blocked; {self.passability_reason}")
            return

        decision = evaluate_safety(
            self.latest_cmd[0],
            self.latest_cmd[1],
            scan_distance,
            self.ultrasonic_ranges.values(),
            self.emergency_hw,
            self.emergency_sw,
            self.params,
        )

        cmd = Twist()
        cmd.linear.x = decision.linear_x
        cmd.angular.z = decision.angular_z
        self.cmd_pub.publish(cmd)

        state = String()
        state.data = (
            f"{decision.state}: {stale_reason}{decision.reason}; "
            f"scan={decision.min_scan_distance:.2f}m ultrasonic={decision.min_ultrasonic_distance:.2f}m "
            f"dynamic_stop={decision.dynamic_stop_distance:.2f}m"
        )
        self.state_pub.publish(state)

    def _ultrasonic_fault_reason(self, now) -> str:
        if self.min_ultrasonic_online <= 0:
            return ""
        online = 0
        stale_topics = []
        for topic, seen_at in self.ultrasonic_seen_at.items():
            if seen_at is None:
                stale_topics.append(topic)
                continue
            age = (now - seen_at).nanoseconds / 1e9
            if age <= self.ultrasonic_stale_timeout_sec:
                online += 1
            else:
                stale_topics.append(f"{topic}({age:.2f}s)")
        if online < self.min_ultrasonic_online:
            return (
                f"only {online}/{self.min_ultrasonic_online} required ultrasonic "
                f"topics are fresh; stale={', '.join(stale_topics)}"
            )
        return ""

    def _localization_fault_reason(self, now) -> str:
        if not self.require_localization_healthy:
            return ""
        if self.last_localization_time is None:
            return "localization health is missing"
        age = (now - self.last_localization_time).nanoseconds / 1e9
        if age > self.localization_timeout_sec:
            return f"localization health stale for {age:.2f}s"
        if self.localization_healthy is False:
            return f"localization unhealthy: {self.localization_reason}"
        return ""

    def _publish_zero_state(self, reason: str):
        cmd = Twist()
        self.cmd_pub.publish(cmd)
        state = String()
        state.data = reason
        self.state_pub.publish(state)


def main(args=None):
    if rclpy is None:
        raise RuntimeError("ROS2 Python packages are required to run this node")
    rclpy.init(args=args)
    node = SafetySupervisorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
