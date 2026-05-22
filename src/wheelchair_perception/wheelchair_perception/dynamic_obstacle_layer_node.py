try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
except ImportError:
    rclpy = None
    Node = object
    String = None


class DynamicObstacleLayerNode(Node):
    """Placeholder for map maintenance mode.

    Temporary obstacles belong in the local costmap. This node deliberately does
    not write them into the static map. A later map maintenance workflow can
    accumulate stable long-term changes and ask the user to confirm before
    modifying the saved map.
    """

    def __init__(self):
        super().__init__("dynamic_obstacle_layer_node")
        self.pub = self.create_publisher(String, "/map_maintenance/status", 10)
        self.timer = self.create_timer(2.0, self.publish_status)

    def publish_status(self):
        msg = String()
        msg.data = "NORMAL_NAVIGATION: dynamic obstacles are handled by local costmap only"
        self.pub.publish(msg)


def main(args=None):
    if rclpy is None:
        raise RuntimeError("ROS2 Python packages are required to run this node")
    rclpy.init(args=args)
    node = DynamicObstacleLayerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
