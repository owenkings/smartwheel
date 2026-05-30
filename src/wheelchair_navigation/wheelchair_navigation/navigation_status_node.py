try:
    import rclpy
    from action_msgs.msg import GoalStatusArray
    from rclpy.node import Node
    from std_msgs.msg import String
except ImportError:
    rclpy = None
    GoalStatusArray = None
    Node = object
    String = None


class NavigationStatusNode(Node):
    STATUS_NAMES = {
        0: "UNKNOWN",
        1: "ACCEPTED",
        2: "EXECUTING",
        3: "CANCELING",
        4: "SUCCEEDED",
        5: "CANCELED",
        6: "ABORTED",
    }

    def __init__(self):
        super().__init__("navigation_status_node")
        self.declare_parameter("nav2_status_topic", "/navigate_to_pose/_action/status")
        self.pub = self.create_publisher(String, "/navigation/status", 10)
        self.create_subscription(String, "/navigation/goal_status", self.forward_goal_status, 10)
        self.create_subscription(
            GoalStatusArray,
            self.get_parameter("nav2_status_topic").value,
            self.on_nav2_status,
            10,
        )
        self.latest = "IDLE"
        self.timer = self.create_timer(1.0, self.publish_status)

    def forward_goal_status(self, msg):
        self.latest = msg.data

    def on_nav2_status(self, msg):
        if not msg.status_list:
            return
        status = msg.status_list[-1]
        status_name = self.STATUS_NAMES.get(int(status.status), f"STATUS_{status.status}")
        if status_name == "ABORTED":
            self.latest = "FAILED: Nav2 aborted current goal"
        elif status_name == "CANCELED":
            self.latest = "CANCELED: Nav2 canceled current goal"
        else:
            self.latest = f"NAV2_{status_name}"

    def publish_status(self):
        msg = String()
        msg.data = self.latest
        self.pub.publish(msg)


def main(args=None):
    if rclpy is None:
        raise RuntimeError("ROS2 Python packages are required to run this node")
    rclpy.init(args=args)
    node = NavigationStatusNode()
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
