try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Bool, String
except ImportError:
    rclpy = None
    Node = object
    Bool = None
    String = None


class EmergencyStopNode(Node):
    """Small command bridge for software stop requests.

    It accepts text commands on /emergency_stop_command: stop, release, true,
    false. The safety supervisor is still the only node that writes /cmd_vel_safe.
    """

    def __init__(self):
        super().__init__("emergency_stop_node")
        self.declare_parameter("output_topic", "/emergency_stop_sw")
        self.pub = self.create_publisher(Bool, self.get_parameter("output_topic").value, 10)
        self.create_subscription(String, "/emergency_stop_command", self.on_command, 10)
        self.active = False
        self.timer = self.create_timer(0.1, self.publish_state)

    def on_command(self, msg):
        text = msg.data.strip().lower()
        if text in ("stop", "true", "1", "pause", "halt"):
            self.active = True
        elif text in ("release", "resume", "false", "0", "continue"):
            self.active = False
        else:
            self.get_logger().warning(f"unknown emergency_stop_command: {msg.data}")

    def publish_state(self):
        msg = Bool()
        msg.data = self.active
        self.pub.publish(msg)


def main(args=None):
    if rclpy is None:
        raise RuntimeError("ROS2 Python packages are required to run this node")
    rclpy.init(args=args)
    node = EmergencyStopNode()
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
