import json
from pathlib import Path

try:
    import rclpy
    from ament_index_python.packages import get_package_share_directory
    from rclpy.node import Node
    from geometry_msgs.msg import PoseStamped
    from std_msgs.msg import Bool, String
except ImportError:
    rclpy = None
    get_package_share_directory = None
    Node = object
    PoseStamped = None
    Bool = None
    String = None

from wheelchair_navigation.named_goal_store import NamedGoalStore, yaw_to_quaternion


def is_voice_intent_confident(intent, threshold: float) -> bool:
    return float(intent.get("confidence", 0.0)) >= float(threshold)


class GoalManagerNode(Node):
    def __init__(self):
        super().__init__("goal_manager_node")
        default_path = "named_goals.yaml"
        if get_package_share_directory is not None:
            default_path = str(
                Path(get_package_share_directory("wheelchair_navigation"))
                / "config"
                / "named_goals.yaml"
        )
        self.declare_parameter("named_goals_path", default_path)
        self.declare_parameter("voice_confidence_threshold", 0.75)
        self.store = NamedGoalStore(self.get_parameter("named_goals_path").value)
        self.voice_confidence_threshold = float(
            self.get_parameter("voice_confidence_threshold").value
        )
        self.goal_pub = self.create_publisher(PoseStamped, "/goal_pose", 10)
        self.status_pub = self.create_publisher(String, "/navigation/goal_status", 10)
        self.stop_cmd_pub = self.create_publisher(String, "/emergency_stop_command", 10)
        self.stop_bool_pub = self.create_publisher(Bool, "/emergency_stop_sw", 10)
        self.create_subscription(String, "/named_goal_command", self.on_named_goal_command, 10)
        self.create_subscription(String, "/voice/intent", self.on_voice_intent, 10)

    def on_named_goal_command(self, msg):
        try:
            command = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.publish_status(f"ERROR: invalid goal command JSON: {exc}")
            return
        self.handle_command(command)

    def on_voice_intent(self, msg):
        try:
            intent = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.publish_status(f"ERROR: invalid voice intent JSON: {exc}")
            return
        confidence = float(intent.get("confidence", 0.0))
        if not is_voice_intent_confident(intent, self.voice_confidence_threshold):
            self.publish_status(
                f"ERROR: voice intent confidence {confidence:.2f} below threshold "
                f"{self.voice_confidence_threshold:.2f}"
            )
            return
        self.handle_command(intent)

    def handle_command(self, command):
        action = command.get("action") or command.get("intent")
        if action in ("navigate_to", "go_to"):
            self.navigate_to_name(command.get("goal_name") or command.get("name", ""))
        elif action in ("stop", "pause"):
            self.publish_stop_command("stop")
            self.publish_status("STOP_REQUESTED: software stop active")
        elif action in ("continue", "resume"):
            self.publish_stop_command("release")
            self.publish_status("RESUME_REQUESTED: software stop released")
        elif action == "add_goal":
            key = self.store.upsert_goal(
                command["name"],
                float(command["x"]),
                float(command["y"]),
                float(command.get("yaw", 0.0)),
                command.get("frame_id", "map"),
                command.get("label"),
            )
            self.publish_status(f"GOAL_SAVED: {key}")
        elif action == "delete_goal":
            ok = self.store.delete_goal(command.get("name", ""))
            self.publish_status("GOAL_DELETED" if ok else "ERROR: goal not found")
        else:
            self.publish_status(f"ERROR: unsupported goal action {action}")

    def navigate_to_name(self, name: str):
        goal = self.store.get_goal(name)
        if goal is None:
            self.publish_status(f"ERROR: named goal not found: {name}")
            return
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = goal.get("frame_id", "map")
        pose.pose.position.x = float(goal["position"][0])
        pose.pose.position.y = float(goal["position"][1])
        pose.pose.position.z = float(goal["position"][2]) if len(goal["position"]) > 2 else 0.0
        q = yaw_to_quaternion(float(goal.get("yaw", 0.0)))
        pose.pose.orientation.x = q["x"]
        pose.pose.orientation.y = q["y"]
        pose.pose.orientation.z = q["z"]
        pose.pose.orientation.w = q["w"]
        self.goal_pub.publish(pose)
        self.publish_status(f"GOAL_SENT: {goal.get('label', name)}")

    def publish_stop_command(self, command: str):
        msg = String()
        msg.data = command
        self.stop_cmd_pub.publish(msg)
        bool_msg = Bool()
        bool_msg.data = command in ("stop", "pause")
        self.stop_bool_pub.publish(bool_msg)

    def publish_status(self, text: str):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)


def main(args=None):
    if rclpy is None:
        raise RuntimeError("ROS2 Python packages are required to run this node")
    rclpy.init(args=args)
    node = GoalManagerNode()
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
