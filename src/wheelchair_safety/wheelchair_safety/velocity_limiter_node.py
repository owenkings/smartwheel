try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Twist
except ImportError:
    rclpy = None
    Node = object
    Twist = None


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


class VelocityLimiterNode(Node):
    def __init__(self):
        super().__init__("velocity_limiter_node")
        self.declare_parameter("input_topic", "/cmd_vel_nav")
        self.declare_parameter("output_topic", "/cmd_vel_limited")
        self.declare_parameter("max_linear_x", 0.4)
        self.declare_parameter("max_angular_z", 0.8)
        self.max_linear_x = float(self.get_parameter("max_linear_x").value)
        self.max_angular_z = float(self.get_parameter("max_angular_z").value)
        self.pub = self.create_publisher(Twist, self.get_parameter("output_topic").value, 10)
        self.create_subscription(Twist, self.get_parameter("input_topic").value, self.on_cmd, 10)

    def on_cmd(self, msg):
        out = Twist()
        out.linear.x = clamp(msg.linear.x, -self.max_linear_x, self.max_linear_x)
        out.angular.z = clamp(msg.angular.z, -self.max_angular_z, self.max_angular_z)
        self.pub.publish(out)


def main(args=None):
    if rclpy is None:
        raise RuntimeError("ROS2 Python packages are required to run this node")
    rclpy.init(args=args)
    node = VelocityLimiterNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
