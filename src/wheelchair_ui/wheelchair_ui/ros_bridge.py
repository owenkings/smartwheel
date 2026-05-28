import json
import math
import threading
import time
from pathlib import Path
from typing import Any, Dict, List, Optional

try:
    import rclpy
    from ament_index_python.packages import get_package_share_directory
    from geometry_msgs.msg import PoseStamped, Twist
    from nav_msgs.msg import OccupancyGrid, Odometry
    from rclpy.node import Node
    from sensor_msgs.msg import Image, Imu, LaserScan, PointCloud2, Range
    from std_msgs.msg import Bool, String
except ImportError:
    rclpy = None
    get_package_share_directory = None
    PoseStamped = None
    Twist = None
    OccupancyGrid = None
    Odometry = None
    Node = object
    Image = None
    Imu = None
    LaserScan = None
    PointCloud2 = None
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


def parse_key_value_status(data: str) -> Dict[str, Any]:
    result: Dict[str, Any] = {}
    for part in data.split(";"):
        if "=" not in part:
            continue
        key, value = part.split("=", 1)
        key = key.strip()
        raw = value.strip()
        if not key:
            continue
        lowered = raw.lower()
        if lowered in ("true", "false"):
            result[key] = lowered == "true"
            continue
        try:
            number = float(raw)
            result[key] = int(number) if number.is_integer() else number
        except ValueError:
            result[key] = raw
    return result


class WheelchairUiRosNode(Node):
    def __init__(
        self,
        named_goals_path: str,
        semantic_map_path: Optional[str] = None,
        node_name: str = "wheelchair_ui_ros_bridge",
    ):
        super().__init__(node_name)
        self.store = NamedGoalStore(named_goals_path)
        self.semantic_store = SemanticMapStore(semantic_map_path or default_semantic_map_path())
        self.sensor_timeout_sec = 3.0
        self.safety_state = "UNKNOWN"
        self.navigation_status = "IDLE"
        self.hardware_status = {"state": "UNKNOWN", "reason": "waiting for watchdog"}
        self.localization_health = {"state": "UNKNOWN", "reason": "waiting for localization health"}
        self.passability_status = {"state": "UNKNOWN", "reason": "waiting for passability analyzer"}
        self.pose = {"x": 0.0, "y": 0.0, "yaw": 0.0, "frame_id": "odom"}
        self.current_goal: Optional[Dict] = None
        self.map_info: Optional[Dict] = None
        self.declare_parameter("ultrasonic_indices", [0])
        self.declare_parameter("enabled_cameras", ["front"])
        self.ultrasonic_indices = [
            int(index) for index in self.get_parameter("ultrasonic_indices").value
        ]
        enabled_cameras = set(self.get_parameter("enabled_cameras").value)
        self.camera_topics = self._camera_topics(enabled_cameras)
        self.sensor_status = {
            "laser": False,
            "imu": False,
            "ultrasonic": [False] * len(self.ultrasonic_indices),
            "odom": False,
            "base": False,
        }
        for key in self.camera_topics:
            self.sensor_status[key] = False
        self.sensor_last_seen: Dict[str, float] = {}
        self.sensor_counts: Dict[str, int] = {}
        self.sensor_details: Dict[str, Dict[str, Any]] = {}

        self.goal_pub = self.create_publisher(PoseStamped, "/goal_pose", 10)
        self.named_goal_command_pub = self.create_publisher(String, "/named_goal_command", 10)
        self.emergency_stop_pub = self.create_publisher(Bool, "/emergency_stop_sw", 10)
        self.emergency_command_pub = self.create_publisher(String, "/emergency_stop_command", 10)
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.cmd_vel_safe_pub = self.create_publisher(Twist, "/cmd_vel_safe", 10)

        self.create_subscription(String, "/safety_state", self.on_safety_state, 10)
        self.create_subscription(String, "/navigation/status", self.on_navigation_status, 10)
        self.create_subscription(String, "/navigation/goal_status", self.on_navigation_status, 10)
        self.create_subscription(String, "/hardware/status", self.on_hardware_status, 10)
        self.create_subscription(String, "/localization/health", self.on_localization_health, 10)
        self.create_subscription(String, "/passability/status", self.on_passability_status, 10)
        self.create_subscription(PoseStamped, "/goal_pose", self.on_goal_pose, 10)
        self.create_subscription(OccupancyGrid, "/map", self.on_map, 10)
        self.create_subscription(Odometry, "/wheel/odom", self.on_odom, 10)
        self.create_subscription(PointCloud2, "/xtm60/points", self.on_xtm60_points, 10)
        self.create_subscription(LaserScan, "/scan", self.on_scan, 10)
        self.create_subscription(Imu, "/imu/data", self.on_imu, 10)
        for key, topic in self.camera_topics.items():
            self.create_subscription(
                Image, topic, lambda msg, k=key, t=topic: self.on_camera(k, t, msg), 10
            )
        self.create_subscription(String, "/base/status", self.on_base_status, 10)
        for index in self.ultrasonic_indices:
            self.create_subscription(
                Range,
                f"/ultrasonic/range_{index}",
                lambda msg, i=index: self.on_ultrasonic(i, msg),
                10,
            )

    @staticmethod
    def _camera_topics(enabled_cameras: set) -> Dict[str, str]:
        topics = {
            "front": ("camera_front", "/camera/front/image_raw"),
            "left": ("camera_left", "/camera/left/image_raw"),
            "right": ("camera_right", "/camera/right/image_raw"),
            "rear": ("camera_rear", "/camera/rear/image_raw"),
        }
        return {
            key: topic
            for name, (key, topic) in topics.items()
            if name in enabled_cameras
        }

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

    def _mark_sensor(self, key: str, details: Optional[Dict[str, Any]] = None):
        self.sensor_last_seen[key] = time.monotonic()
        self.sensor_counts[key] = self.sensor_counts.get(key, 0) + 1
        if details:
            self.sensor_details[key] = details

    def _sensor_entry(self, key: str, now: float) -> Dict[str, Any]:
        last_seen = self.sensor_last_seen.get(key)
        age = None if last_seen is None else max(0.0, now - last_seen)
        details = dict(self.sensor_details.get(key, {}))
        details.update(
            {
                "online": age is not None and age <= self.sensor_timeout_sec,
                "age_sec": age,
                "messages": self.sensor_counts.get(key, 0),
            }
        )
        return details

    def _sensor_snapshot(self) -> Dict[str, Any]:
        now = time.monotonic()
        sensors = {
            "laser": self._sensor_entry("laser", now),
            "scan": self._sensor_entry("scan", now),
            "imu": self._sensor_entry("imu", now),
            "odom": self._sensor_entry("odom", now),
            "base": self._sensor_entry("base", now),
            "ultrasonic": [
                {"index": index, **self._sensor_entry(f"ultrasonic_{index}", now)}
                for index in self.ultrasonic_indices
            ],
        }
        for key in self.camera_topics:
            sensors[key] = self._sensor_entry(key, now)
        self.sensor_status = {
            "laser": sensors["laser"]["online"] or sensors["scan"]["online"],
            "imu": sensors["imu"]["online"],
            "ultrasonic": [entry["online"] for entry in sensors["ultrasonic"]],
            "odom": sensors["odom"]["online"],
            "base": sensors["base"]["online"],
        }
        for key in self.camera_topics:
            self.sensor_status[key] = sensors[key]["online"]
        return sensors

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
        self._mark_sensor(
            "odom",
            {
                "topic": "/wheel/odom",
                "frame_id": msg.header.frame_id,
                "linear_x": msg.twist.twist.linear.x,
                "angular_z": msg.twist.twist.angular.z,
            },
        )

    def on_xtm60_points(self, msg):
        self._mark_sensor(
            "laser",
            {
                "topic": "/xtm60/points",
                "frame_id": msg.header.frame_id,
                "points": int(msg.width) * int(msg.height),
            },
        )

    def on_scan(self, msg):
        import math

        finite_ranges = [value for value in msg.ranges if math.isfinite(value)]
        details = {
            "topic": "/scan",
            "frame_id": msg.header.frame_id,
            "samples": len(msg.ranges),
        }
        if finite_ranges:
            details["nearest_m"] = min(finite_ranges)
        self._mark_sensor("scan", details)

    def on_imu(self, msg):
        import math

        q = msg.orientation
        yaw = 0.0
        if q.w or q.z:
            yaw = math.atan2(2.0 * q.w * q.z, 1.0 - 2.0 * q.z * q.z)
        accel_norm = math.sqrt(
            msg.linear_acceleration.x ** 2
            + msg.linear_acceleration.y ** 2
            + msg.linear_acceleration.z ** 2
        )
        gyro_norm = math.sqrt(
            msg.angular_velocity.x ** 2
            + msg.angular_velocity.y ** 2
            + msg.angular_velocity.z ** 2
        )
        self._mark_sensor(
            "imu",
            {
                "topic": "/imu/data",
                "frame_id": msg.header.frame_id,
                "yaw": yaw,
                "accel_norm": accel_norm,
                "gyro_norm": gyro_norm,
            },
        )

    def on_camera(self, key: str, topic: str, msg):
        self._mark_sensor(
            key,
            {
                "topic": topic,
                "frame_id": msg.header.frame_id,
                "width": int(msg.width),
                "height": int(msg.height),
                "encoding": msg.encoding,
            },
        )

    def on_base_status(self, msg):
        details = {"topic": "/base/status", "raw": msg.data}
        details.update(parse_key_value_status(msg.data))
        self._mark_sensor("base", details)

    def on_ultrasonic(self, index: int, msg):
        range_mm = int(round(float(msg.range) * 1000.0)) if math.isfinite(float(msg.range)) else None
        self._mark_sensor(
            f"ultrasonic_{index}",
            {
                "topic": f"/ultrasonic/range_{index}",
                "frame_id": msg.header.frame_id,
                "range_m": msg.range,
                "range_mm": range_mm,
                "min_range": msg.min_range,
                "max_range": msg.max_range,
            },
        )

    def status(self) -> Dict:
        sensors = self._sensor_snapshot()
        return {
            "safety_state": self.safety_state,
            "navigation_status": self.navigation_status,
            "pose": self.pose,
            "current_goal": self.current_goal,
            "sensor_status": self.sensor_status,
            "sensors": sensors,
            "hardware_config": {
                "ultrasonic_indices": self.ultrasonic_indices,
                "cameras": list(self.camera_topics.keys()),
            },
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
        if active:
            self.publish_zero_velocity()

    def publish_zero_velocity(self):
        msg = Twist()
        self.cmd_vel_pub.publish(msg)
        self.cmd_vel_safe_pub.publish(msg)

    def request_hardware_shutdown(self) -> Dict:
        self.set_software_stop(True)
        self.publish_zero_velocity()
        return {"ok": True, "emergency_stop": True, "zero_velocity": True}

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
    def __init__(
        self,
        named_goals_path: Optional[str] = None,
        semantic_map_path: Optional[str] = None,
        node_name: str = "wheelchair_ui_ros_bridge",
    ):
        if rclpy is None:
            raise RuntimeError("ROS2 Python packages are required to run the UI bridge")
        if not rclpy.ok():
            rclpy.init(args=[])
        self.node = WheelchairUiRosNode(
            named_goals_path or default_named_goals_path(),
            semantic_map_path,
            node_name=node_name,
        )
        self.thread = threading.Thread(target=self._spin, daemon=True)
        self.thread.start()

    def _spin(self):
        try:
            rclpy.spin(self.node)
        except Exception:
            if rclpy.ok():
                raise

    def shutdown(self):
        try:
            self.node.request_hardware_shutdown()
        except Exception as exc:
            self.node.get_logger().warning(f"hardware shutdown publish failed: {exc}")
        finally:
            self.node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
            if self.thread.is_alive():
                self.thread.join(timeout=2.0)
