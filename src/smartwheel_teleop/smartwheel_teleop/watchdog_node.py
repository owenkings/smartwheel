import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class CommandWatchdogNode(Node):
    def __init__(self) -> None:
        super().__init__("smartwheel_command_watchdog")
        self.declare_parameter("hardware_enabled", False)
        self.declare_parameter("timeout_sec", 0.5)
        self._hardware_enabled = bool(self.get_parameter("hardware_enabled").value)
        self._timeout = float(self.get_parameter("timeout_sec").value)
        self._last = self.get_clock().now()
        self._publisher = self.create_publisher(Twist, "/motor/watchdog_stop", 10)
        self.create_subscription(Twist, "/motor/command", self._on_command, 10)
        self.create_timer(max(0.02, self._timeout / 4.0), self._tick)

    def _on_command(self, _message: Twist) -> None:
        self._last = self.get_clock().now()

    def _tick(self) -> None:
        age = (self.get_clock().now() - self._last).nanoseconds * 1e-9
        if self._hardware_enabled and age > self._timeout:
            self._publisher.publish(Twist())


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CommandWatchdogNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

