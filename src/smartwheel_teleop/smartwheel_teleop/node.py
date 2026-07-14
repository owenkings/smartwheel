import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool, String

from smartwheel_teleop.controller import SafetyState, TeleopController, motor_command_allowed


class SafeTeleopNode(Node):
    def __init__(self) -> None:
        super().__init__("smartwheel_safe_teleop")
        self.declare_parameter("hardware_enabled", False)
        self.declare_parameter("max_linear_mps", 0.25)
        self.declare_parameter("max_angular_rps", 0.6)
        self.declare_parameter("acceleration_mps2", 0.4)
        self.declare_parameter("deceleration_mps2", 0.8)
        self.declare_parameter("command_timeout_sec", 0.35)
        self.declare_parameter("publish_rate_hz", 20.0)
        self._hardware_enabled = bool(self.get_parameter("hardware_enabled").value)
        self._controller = TeleopController(
            float(self.get_parameter("max_linear_mps").value),
            float(self.get_parameter("max_angular_rps").value),
            float(self.get_parameter("acceleration_mps2").value),
            float(self.get_parameter("deceleration_mps2").value),
            float(self.get_parameter("command_timeout_sec").value),
        )
        self._deadman = False
        self._emergency_stop = False
        self._safe_pub = self.create_publisher(Twist, "/cmd_vel/safe", 10)
        self._motor_pub = self.create_publisher(Twist, "/motor/command", 10)
        self.create_subscription(String, "/teleop/key", self._on_key, 10)
        self.create_subscription(Bool, "/teleop/deadman", self._on_deadman, 10)
        self.create_subscription(Bool, "/emergency_stop", self._on_estop, 10)
        rate = max(1.0, float(self.get_parameter("publish_rate_hz").value))
        self.create_timer(1.0 / rate, self._tick)

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_key(self, message: String) -> None:
        parts = message.data.strip().split(":", 1)
        key = parts[0]
        pressed = len(parts) == 1 or parts[1].lower() in ("down", "press", "1", "true")
        try:
            self._controller.key_event(key, pressed, self._now())
        except ValueError as exc:
            self.get_logger().warning(str(exc))

    def _on_deadman(self, message: Bool) -> None:
        self._deadman = bool(message.data)

    def _on_estop(self, message: Bool) -> None:
        self._emergency_stop = bool(message.data)

    def _tick(self) -> None:
        now = self._now()
        command = self._controller.tick(now)
        safe = Twist()
        if self._deadman and not self._emergency_stop and self._controller.command_fresh(now):
            safe.linear.x = command.linear
            safe.angular.z = command.angular
        self._safe_pub.publish(safe)
        gate = SafetyState(
            hardware_enabled=self._hardware_enabled,
            deadman_active=self._deadman,
            emergency_stop=self._emergency_stop,
            command_not_timed_out=self._controller.command_fresh(now),
        )
        if self._hardware_enabled:
            self._motor_pub.publish(safe if motor_command_allowed(gate) else Twist())

    def stop(self) -> None:
        zero = Twist()
        self._safe_pub.publish(zero)
        if self._hardware_enabled:
            self._motor_pub.publish(zero)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SafeTeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        rclpy.spin_once(node, timeout_sec=0.05)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
