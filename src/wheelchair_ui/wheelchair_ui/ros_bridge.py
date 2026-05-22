import json
import threading
from pathlib import Path
from typing import Dict, Optional

try:
    import rclpy
    from ament_index_python.packages import get_package_share_directory
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import OccupancyGrid, Odometry
    from rclpy.node import Node
    from sensor_msgs.msg import Image, Imu, LaserScan, Range
    from std_msgs.msg import Bool, String
except ImportError:
    rclpy = None
    get_package_share_directory = None
    PoseStamped = None
    OccupancyGrid = None
    Odometry = None
    Node = object
    Image = None
    Imu = None
    LaserScan = None
    Range = None
    Bool = None
    String = None

from wheelchair_navigation.named_goal_store import NamedGoalStore, yaw_to_quaternion
from wheelchair_navigation.semantic_map_store import SemanticMapStore, default_semantic_map_path


def default_named_goals_path() -> str:
    source_candidate = (
        Path(__file__).resolve().parents[2]
        / "wheelchair_navigation"
        / "config"
        / "named_goals.yaml"
    )
    if source_candidate.exists():
        return str(source_candidate)
    if get_package_share_directory is not None:
        return str(
            Path(get_package_share_directory("wheelchair_navigation"))
            / "config"
            / "named_goals.yaml"
        )
    return "named_goals.yaml"


class WheelchairUiRosNode(Node):
    def __init__(self, named_goals_path: str, semantic_map_path: Optional[str] = None):
        super().__init__("wheelchair_ui_ros_bridge")
        self.store = NamedGoalStore(named_goals_path)
        self.semantic_store = SemanticMapStore(semantic_map_path or default_semantic_map_path())
        self.safety_state = "UNKNOWN"
        self.navigation_status = "IDLE"
        self.hardware_status = {"state": "UNKNOWN", "reason": "waiting for watchdog"}
        self.localization_health = {"state": "UNKNOWN", "reason": "waiting for localization health"}
        self.passability_status = {"state": "UNKNOWN", "reason": "waiting for passability analyzer"}
        self.pose = {"x": 0.0, "y": 0.0, "yaw": 0.0, "frame_id": "odom"}
        self.current_goal: Optional[Dict] = None
        self.map_info: Optional[Dict] = None
        self.sensor_status = {
            "laser": False,
            "imu": False,
            "ultrasonic": [False] * 6,
            "camera_front": False,
            "camera_left": False,
            "odom": False,
            "base": False,
        }

        self.goal_pub = self.create_publisher(PoseStamped, "/goal_pose", 10)
        self.named_goal_command_pub = self.create_publisher(String, "/named_goal_command", 10)
        self.emergency_stop_pub = self.create_publisher(Bool, "/emergency_stop_sw", 10)
        self.emergency_command_pub = self.create_publisher(String, "/emergency_stop_command", 10)

        self.create_subscription(String, "/safety_state", self.on_safety_state, 10)
        self.create_subscription(String, "/navigation/status", self.on_navigation_status, 10)
        self.create_subscription(String, "/navigation/goal_status", self.on_navigation_status, 10)
        self.create_subscription(String, "/hardware/status", self.on_hardware_status, 10)
        self.create_subscription(String, "/localization/health", self.on_localization_health, 10)
        self.create_subscription(String, "/passability/status", self.on_passability_status, 10)
        self.create_subscription(PoseStamped, "/goal_pose", self.on_goal_pose, 10)
        self.create_subscription(OccupancyGrid, "/map", self.on_map, 10)
        self.create_subscription(Odometry, "/wheel/odom", self.on_odom, 10)
        self.create_subscription(LaserScan, "/scan", self.on_scan, 10)
        self.create_subscription(Imu, "/imu/data", self.on_imu, 10)
        self.create_subscription(Image, "/camera/front/image_raw", self.on_camera, 10)
        self.create_subscription(Image, "/camera/left/image_raw", self.on_camera_left, 10)
        self.create_subscription(String, "/base/status", self.on_base_status, 10)
        for index in range(6):
            self.create_subscription(
                Range,
                f"/ultrasonic/{index}/range",
                lambda msg, i=index: self.on_ultrasonic(i, msg),
                10,
            )

    def on_safety_state(self, msg):
        self.safety_state = msg.data

    def on_navigation_status(self, msg):
        self.navigation_status = msg.data

    def on_hardware_status(self, msg):
        self.hardware_status = self._parse_json_status(msg.data, self.hardware_status)

    def on_localization_health(self, msg):
        self.localization_health = self._parse_json_status(msg.data, self.localization_health)

    def on_passability_status(self, msg):
        self.passability_status = self._parse_json_status(msg.data, self.passability_status)

    @staticmethod
    def _parse_json_status(data: str, fallback: Dict) -> Dict:
        try:
            return json.loads(data)
        except Exception:
            updated = dict(fallback)
            updated["reason"] = data
            return updated

    def on_goal_pose(self, msg):
        self.current_goal = {
            "frame_id": msg.header.frame_id,
            "x": msg.pose.position.x,
            "y": msg.pose.position.y,
        }

    def on_map(self, msg):
        self.map_info = {
            "frame_id": msg.header.frame_id,
            "width": msg.info.width,
            "height": msg.info.height,
            "resolution": msg.info.resolution,
            "origin": {
                "x": msg.info.origin.position.x,
                "y": msg.info.origin.position.y,
            },
            "data": list(msg.data),
        }

    def on_odom(self, msg):
        self.sensor_status["odom"] = True
        q = msg.pose.pose.orientation
        yaw = 0.0
        if q.w or q.z:
            import math

            yaw = math.atan2(2.0 * q.w * q.z, 1.0 - 2.0 * q.z * q.z)
        self.pose = {
            "x": msg.pose.pose.position.x,
            "y": msg.pose.pose.position.y,
            "yaw": yaw,
            "frame_id": msg.header.frame_id,
        }

    def on_scan(self, _msg):
        self.sensor_status["laser"] = True

    def on_imu(self, _msg):
        self.sensor_status["imu"] = True

    def on_camera(self, _msg):
        self.sensor_status["camera_front"] = True

    def on_camera_left(self, _msg):
        self.sensor_status["camera_left"] = True

    def on_base_status(self, _msg):
        self.sensor_status["base"] = True

    def on_ultrasonic(self, index: int, _msg):
        self.sensor_status["ultrasonic"][index] = True

    def status(self) -> Dict:
        return {
            "safety_state": self.safety_state,
            "navigation_status": self.navigation_status,
            "pose": self.pose,
            "current_goal": self.current_goal,
            "sensor_status": self.sensor_status,
            "hardware_status": self.hardware_status,
            "localization_health": self.localization_health,
            "passability_status": self.passability_status,
            "map_available": self.map_info is not None,
        }

    def list_goals(self) -> Dict:
        return self.store.list_goals()

    def add_goal(self, payload: Dict) -> Dict:
        key = self.store.upsert_goal(
            payload["name"],
            float(payload["x"]),
            float(payload["y"]),
            float(payload.get("yaw", 0.0)),
            payload.get("frame_id", "map"),
            payload.get("label"),
        )
        command = String()
        command.data = json.dumps(
            {
                "action": "add_goal",
                "name": payload["name"],
                "x": float(payload["x"]),
                "y": float(payload["y"]),
                "yaw": float(payload.get("yaw", 0.0)),
                "frame_id": payload.get("frame_id", "map"),
                "label": payload.get("label", payload["name"]),
            },
            ensure_ascii=False,
        )
        self.named_goal_command_pub.publish(command)
        return {"key": key, "goals": self.list_goals()}

    def delete_goal(self, name: str) -> bool:
        ok = self.store.delete_goal(name)
        command = String()
        command.data = json.dumps({"action": "delete_goal", "name": name}, ensure_ascii=False)
        self.named_goal_command_pub.publish(command)
        return ok

    def send_named_goal(self, name: str) -> bool:
        goal = self.store.get_goal(name)
        if goal is None:
            return False
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

        command = String()
        command.data = json.dumps(
            {"action": "navigate_to", "goal_name": name}, ensure_ascii=False
        )
        self.named_goal_command_pub.publish(command)
        return True

    def set_software_stop(self, active: bool):
        msg = Bool()
        msg.data = active
        self.emergency_stop_pub.publish(msg)
        command = String()
        command.data = "stop" if active else "release"
        self.emergency_command_pub.publish(command)

    def map_snapshot(self) -> Optional[Dict]:
        return self.map_info

    def semantic_map(self) -> Dict:
        return self.semantic_store.load()

    def save_semantic_map(self, payload: Dict) -> Dict:
        return self.semantic_store.save(payload)

    def upsert_room(self, payload: Dict) -> Dict:
        return self.semantic_store.upsert_room(
            payload["name"],
            payload["polygon"],
            payload.get("color", "#6aa6ff"),
        )

    def delete_room(self, name: str) -> bool:
        return self.semantic_store.delete_room(name)

    def upsert_no_go_zone(self, payload: Dict) -> Dict:
        return self.semantic_store.upsert_no_go_zone(payload["name"], payload["polygon"])

    def delete_no_go_zone(self, name: str) -> bool:
        return self.semantic_store.delete_no_go_zone(name)


class RosBridge:
    def __init__(self, named_goals_path: Optional[str] = None, semantic_map_path: Optional[str] = None):
        if rclpy is None:
            raise RuntimeError("ROS2 Python packages are required to run the UI bridge")
        if not rclpy.ok():
            rclpy.init(args=[])
        self.node = WheelchairUiRosNode(named_goals_path or default_named_goals_path(), semantic_map_path)
        self.thread = threading.Thread(target=rclpy.spin, args=(self.node,), daemon=True)
        self.thread.start()

    def shutdown(self):
        self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
