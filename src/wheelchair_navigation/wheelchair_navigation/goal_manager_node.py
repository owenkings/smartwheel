import json
import math
from pathlib import Path

try:
    import rclpy
    from ament_index_python.packages import get_package_share_directory
    from action_msgs.msg import GoalStatus
    from rclpy.node import Node
    from rclpy.action import ActionClient
    from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
    from nav_msgs.msg import Path as NavPath
    from nav2_msgs.action import ComputePathToPose, NavigateToPose
    from std_msgs.msg import Bool, String
except ImportError:
    rclpy = None
    get_package_share_directory = None
    GoalStatus = None
    ActionClient = None
    Node = object
    PoseStamped = None
    PoseWithCovarianceStamped = None
    NavPath = None
    ComputePathToPose = None
    NavigateToPose = None
    Bool = None
    String = None

from wheelchair_navigation.named_goal_store import NamedGoalStore, yaw_to_quaternion


def is_voice_intent_confident(intent, threshold: float) -> bool:
    return float(intent.get("confidence", 0.0)) >= float(threshold)


def nearest_goal_label(goals, x: float, y: float, max_distance: float = 2.0):
    nearest = None
    nearest_distance = math.inf
    for key, goal in goals.items():
        position = goal.get("position", [])
        if len(position) < 2:
            continue
        distance = math.hypot(float(position[0]) - x, float(position[1]) - y)
        if distance < nearest_distance:
            nearest = goal.get("label", key)
            nearest_distance = distance
    if nearest is None or nearest_distance > max_distance:
        return None
    return nearest


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
        self.declare_parameter("pose_query_nearby_distance", 2.0)
        self.store = NamedGoalStore(self.get_parameter("named_goals_path").value)
        self.voice_confidence_threshold = float(
            self.get_parameter("voice_confidence_threshold").value
        )
        self.pose_query_nearby_distance = float(
            self.get_parameter("pose_query_nearby_distance").value
        )
        self._latest_global_pose = None
        self.goal_pub = self.create_publisher(PoseStamped, "/goal_pose", 10)
        self.path_pub = self.create_publisher(NavPath, "/navigation/preview_path", 10)
        self.status_pub = self.create_publisher(String, "/navigation/goal_status", 10)
        self.stop_cmd_pub = self.create_publisher(String, "/emergency_stop_command", 10)
        self.stop_bool_pub = self.create_publisher(Bool, "/emergency_stop_sw", 10)
        self.tts_pub = self.create_publisher(String, "/voice/tts_text", 10)
        self.path_client = ActionClient(self, ComputePathToPose, "/compute_path_to_pose")
        self.nav_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self._last_preview_goal_stamp = 0.0
        self.create_subscription(PoseStamped, "/goal_pose", self.on_goal_pose, 10)
        self.create_subscription(PoseStamped, "/navigation/preview_goal", self.on_goal_pose, 10)
        self.create_subscription(String, "/named_goal_command", self.on_named_goal_command, 10)
        self.create_subscription(String, "/voice/intent", self.on_voice_intent, 10)
        self.create_subscription(
            PoseWithCovarianceStamped,
            "/localization_pose",
            self.on_global_pose,
            10,
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            "/global_pose",
            self.on_global_pose,
            10,
        )

    def on_global_pose(self, msg):
        self._latest_global_pose = msg

    def on_goal_pose(self, msg):
        """Ask Nav2 for a preview path so the GUI can draw the route.

        Nav2's /goal_pose topic is enough to start navigation, but it does not
        itself provide a user-visible route. The ComputePathToPose action gives
        us an explicit Path when planner_server is active; failures are reported
        on /navigation/goal_status instead of failing silently in the UI.
        """
        if ComputePathToPose is None:
            return
        now = self.get_clock().now().nanoseconds / 1e9
        if now - self._last_preview_goal_stamp < 0.2:
            return
        self._last_preview_goal_stamp = now
        if not self.path_client.server_is_ready():
            self.publish_status("PLANNING_WAIT: planner action server not ready")
            return
        goal = ComputePathToPose.Goal()
        goal.goal = msg
        goal.planner_id = ""
        goal.use_start = False
        self.publish_status("PLANNING: computing route preview")
        future = self.path_client.send_goal_async(goal)
        future.add_done_callback(self._on_path_goal_response)

    def _on_path_goal_response(self, future):
        try:
            handle = future.result()
        except Exception as exc:
            self.publish_status(f"ERROR: route preview request failed: {exc}")
            return
        if not handle.accepted:
            self.publish_status("ERROR: route preview rejected by planner")
            return
        result_future = handle.get_result_async()
        result_future.add_done_callback(self._on_path_result)

    def _on_path_result(self, future):
        try:
            wrapped = future.result()
        except Exception as exc:
            self.publish_status(f"ERROR: route preview failed: {exc}")
            return
        if wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            self.publish_status(f"ERROR: route preview status {wrapped.status}")
            return
        path = wrapped.result.path
        self.path_pub.publish(path)
        self.publish_status(f"PATH_READY: {len(path.poses)} poses")

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
            self.publish_tts("已停止")
        elif action in ("continue", "resume"):
            self.publish_stop_command("release")
            self.publish_status("RESUME_REQUESTED: software stop released")
            self.publish_tts("已恢复")
        elif action == "query_pose":
            self.report_current_location()
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

    def report_current_location(self):
        if self._latest_global_pose is None:
            self.publish_status("POSE_UNAVAILABLE: no global localization pose")
            self.publish_tts("当前定位不可用")
            return
        position = self._latest_global_pose.pose.pose.position
        label = nearest_goal_label(
            self.store.list_goals(),
            float(position.x),
            float(position.y),
            self.pose_query_nearby_distance,
        )
        if label is None:
            self.publish_status("POSE_AVAILABLE: no nearby named goal")
            self.publish_tts("已定位，但附近没有已标注位置")
            return
        self.publish_status(f"POSE_NEAR: {label}")
        self.publish_tts(f"您现在在{label}附近")

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
        self.send_navigate_goal(pose, goal.get("label", name))

    def send_navigate_goal(self, pose, label: str):
        if NavigateToPose is None:
            self.publish_status("ERROR: NavigateToPose action unavailable")
            return
        if not self.nav_client.server_is_ready():
            self.publish_status("ERROR: /navigate_to_pose action server not ready")
            return
        goal = NavigateToPose.Goal()
        goal.pose = pose
        self.publish_status(f"GOAL_SENT: {label}")
        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(lambda fut, text=label: self._on_nav_goal_response(fut, text))

    def _on_nav_goal_response(self, future, label: str):
        try:
            handle = future.result()
        except Exception as exc:
            self.publish_status(f"ERROR: NavigateToPose request failed: {exc}")
            return
        if not handle.accepted:
            self.publish_status(f"ERROR: NavigateToPose rejected: {label}")
            self.publish_tts(f"无法前往{label}")
            return
        self.publish_status(f"NAV2_ACCEPTED: {label}")
        self.publish_tts(f"正在前往{label}")
        result_future = handle.get_result_async()
        result_future.add_done_callback(lambda fut, text=label: self._on_nav_result(fut, text))

    def _on_nav_result(self, future, label: str):
        try:
            wrapped = future.result()
        except Exception as exc:
            self.publish_status(f"ERROR: NavigateToPose result failed: {exc}")
            return
        status_name = {
            GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
            GoalStatus.STATUS_ABORTED: "ABORTED",
            GoalStatus.STATUS_CANCELED: "CANCELED",
        }.get(int(wrapped.status), f"STATUS_{wrapped.status}")
        self.publish_status(f"NAV2_{status_name}: {label}")
        if wrapped.status == GoalStatus.STATUS_SUCCEEDED:
            self.publish_tts(f"已到达{label}")
        elif wrapped.status == GoalStatus.STATUS_CANCELED:
            self.publish_tts(f"前往{label}的任务已取消")
        else:
            self.publish_tts(f"无法到达{label}")

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

    def publish_tts(self, text: str):
        msg = String()
        msg.data = text
        self.tts_pub.publish(msg)


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
