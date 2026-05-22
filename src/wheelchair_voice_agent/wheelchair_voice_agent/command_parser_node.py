import json

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
except ImportError:
    rclpy = None
    Node = object
    String = None

from wheelchair_voice_agent.model_api_stub import TextCommandModelStub


class CommandParserNode(Node):
    def __init__(self):
        super().__init__("command_parser_node")
        self.declare_parameter("known_goal_names", ["卫生间", "餐厅", "卧室", "门口", "充电点"])
        self.model = TextCommandModelStub()
        self.intent_pub = self.create_publisher(String, "/voice/intent", 10)
        self.status_pub = self.create_publisher(String, "/voice/status", 10)
        self.create_subscription(String, "/voice/text_command", self.on_text_command, 10)

    def on_text_command(self, msg):
        known = list(self.get_parameter("known_goal_names").value)
        intent = self.model.parse(msg.data, known)
        out = String()
        out.data = json.dumps(intent.to_dict(), ensure_ascii=False)
        self.intent_pub.publish(out)

        status = String()
        status.data = f"PARSED: {out.data}"
        self.status_pub.publish(status)


def main(args=None):
    if rclpy is None:
        raise RuntimeError("ROS2 Python packages are required to run this node")
    rclpy.init(args=args)
    node = CommandParserNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
