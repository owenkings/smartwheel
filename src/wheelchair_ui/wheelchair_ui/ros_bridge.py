import json
import math
import threading
import time
from pathlib import Path
from typing import Any, Dict, List, Optional

try:
    import rclpy
    from action_msgs.msg import GoalStatus
    from ament_index_python.packages import get_package_share_directory
    from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
    from nav2_msgs.action import NavigateToPose
    from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
    from nav_msgs.srv import GetMap
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from sensor_msgs.msg import Image, Imu, LaserScan, PointCloud2, Range
    from std_msgs.msg import Bool, String
except ImportError:
    rclpy = None
    GoalStatus = None
    get_package_share_directory = None
    PoseStamped = None
    PoseWithCovarianceStamped = None
    Twist = None
    NavigateToPose = None
    OccupancyGrid = None
    Odometry = None
    NavPath = None
    GetMap = None
    ActionClient = None
    Node = object
    DurabilityPolicy = None
    QoSProfile = None
    ReliabilityPolicy = None
    qos_profile_sensor_data = 10
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


def map_point_to_cell(map_info: Optional[Dict[str, Any]], x: float, y: float) -> Optional[tuple[int, int]]:
    if not map_info:
        return None
    resolution = float(map_info.get("resolution") or 0.0)
    if resolution <= 0.0:
        return None
    origin = map_info.get("origin") or {}
    origin_x = float(origin.get("x", 0.0))
    origin_y = float(origin.get("y", 0.0))
    return (
        int(math.floor((float(x) - origin_x) / resolution)),
        int(math.floor((float(y) - origin_y) / resolution)),
    )


def is_map_point_navigable(
    map_info: Optional[Dict[str, Any]],
    x: float,
    y: float,
    clearance_m: float = 0.40,
    occupied_threshold: int = 50,
) -> tuple[bool, str]:
    """Return whether a clicked map point is usable as a navigation goal."""
    if not map_info:
        return False, "地图尚未加载，不能发送导航目标"
    width = int(map_info.get("width") or 0)
    height = int(map_info.get("height") or 0)
    data = map_info.get("data") or []
    cell = map_point_to_cell(map_info, x, y)
    if width <= 0 or height <= 0 or len(data) < width * height or cell is None:
        return False, "地图数据无效，不能发送导航目标"
    mx, my = cell
    if not (0 <= mx < width and 0 <= my < height):
        return False, "目标点在地图范围外"

    resolution = float(map_info.get("resolution") or 0.0)
    radius_cells = max(0, int(math.ceil(float(clearance_m) / resolution)))
    blocked_seen = False
    unknown_seen = False
    radius_sq = radius_cells * radius_cells
    for dy in range(-radius_cells, radius_cells + 1):
        for dx in range(-radius_cells, radius_cells + 1):
            if dx * dx + dy * dy > radius_sq:
                continue
            cx = mx + dx
            cy = my + dy
            if not (0 <= cx < width and 0 <= cy < height):
                unknown_seen = True
                continue
            value = int(data[cy * width + cx])
            if value < 0:
                unknown_seen = True
            elif value >= occupied_threshold:
                blocked_seen = True
    if blocked_seen:
        return False, f"目标点离障碍物小于 {clearance_m:.2f}m"
    if unknown_seen:
        return False, f"目标点周围 {clearance_m:.2f}m 内有未知区域"
    return True, "目标点可导航"


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(float(angle)), math.cos(float(angle)))


def pose_delta(
    pose: Optional[Dict[str, Any]],
    x: float,
    y: float,
    yaw: Optional[float] = None,
) -> tuple[float, Optional[float]]:
    if not pose:
        return float("inf"), None
    distance = math.hypot(float(pose.get("x", 0.0)) - float(x), float(pose.get("y", 0.0)) - float(y))
    if yaw is None:
        return distance, None
    yaw_error = abs(normalize_angle(float(pose.get("yaw", 0.0)) - float(yaw)))
    return distance, yaw_error


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
        self.sensor_timeout_sec = 5.0
        self.safety_state = "UNKNOWN"
        self.navigation_status = "IDLE"
        self.hardware_status = {"state": "UNKNOWN", "reason": "waiting for watchdog"}
        self.localization_health = {"state": "UNKNOWN", "reason": "waiting for localization health"}
        self.passability_status = {"state": "UNKNOWN", "reason": "waiting for passability analyzer"}
        self.pose = {"x": 0.0, "y": 0.0, "yaw": 0.0, "frame_id": "odom"}
        self.pose_last_seen = 0.0
        self.pose_source = "odom"
        self.initial_pose_target: Optional[Dict[str, float]] = None
        self.initial_pose_requested_at = 0.0
        self.current_goal: Optional[Dict] = None
        self.goal_clearance_m = 0.40
        self.last_goal_error: Optional[str] = None
        self.route_path: Dict[str, Any] = {"frame_id": "map", "points": [], "age_sec": None}
        self.route_last_seen = 0.0
        self.map_info: Optional[Dict] = None
        self._map_request_in_flight = False
        self.declare_parameter("ultrasonic_indices", [0, 1, 2, 3])
        self.declare_parameter("enabled_cameras", ["left", "right"])
        self.declare_parameter("initial_pose_ack_tolerance_m", 0.75)
        self.declare_parameter("initial_pose_ack_tolerance_yaw", 1.0)
        self.declare_parameter("initial_pose_ack_timeout_sec", 4.0)
        self.ultrasonic_indices = [
            int(index) for index in self.get_parameter("ultrasonic_indices").value
        ]
        self.initial_pose_ack_tolerance_m = float(
            self.get_parameter("initial_pose_ack_tolerance_m").value
        )
        self.initial_pose_ack_tolerance_yaw = float(
            self.get_parameter("initial_pose_ack_tolerance_yaw").value
        )
        self.initial_pose_ack_timeout_sec = float(
            self.get_parameter("initial_pose_ack_timeout_sec").value
        )
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
        self.preview_goal_pub = self.create_publisher(PoseStamped, "/navigation/preview_goal", 10)
        self.initialpose_pub = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
        self.named_goal_command_pub = self.create_publisher(String, "/named_goal_command", 10)
        self.goal_status_pub = self.create_publisher(String, "/navigation/goal_status", 10)
        self.emergency_stop_pub = self.create_publisher(Bool, "/emergency_stop_sw", 10)
        self.emergency_command_pub = self.create_publisher(String, "/emergency_stop_command", 10)
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.cmd_vel_safe_pub = self.create_publisher(Twist, "/cmd_vel_safe", 10)
        self.nav_client = (
            ActionClient(self, NavigateToPose, "/navigate_to_pose")
            if ActionClient is not None and NavigateToPose is not None
            else None
        )
        self.map_client = (
            self.create_client(GetMap, "/map_server/map")
            if GetMap is not None
            else None
        )

        map_qos = self._latched_map_qos()
        self.create_subscription(String, "/safety_state", self.on_safety_state, 10)
        self.create_subscription(String, "/navigation/status", self.on_navigation_status, 10)
        self.create_subscription(String, "/navigation/goal_status", self.on_navigation_status, 10)
        self.create_subscription(String, "/exploration/status", self.on_navigation_status, 10)
        self.create_subscription(String, "/hardware/status", self.on_hardware_status, 10)
        self.create_subscription(String, "/localization/health", self.on_localization_health, 10)
        self.create_subscription(String, "/passability/status", self.on_passability_status, 10)
        self.create_subscription(PoseStamped, "/goal_pose", self.on_goal_pose, 10)
        self.create_subscription(OccupancyGrid, "/map", self.on_map, map_qos)
        self.create_subscription(OccupancyGrid, "/rtabmap/grid_map", self.on_rtabmap_grid_map, map_qos)
        self.create_subscription(Odometry, "/wheel/odom", self.on_odom, 10)
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self.on_amcl_pose, 10)
        for topic in ("/navigation/preview_path", "/plan", "/global_plan", "/received_global_plan"):
            self.create_subscription(
                NavPath,
                topic,
                lambda msg, t=topic: self.on_route_path(t, msg),
                10,
            )
        self.create_subscription(
            Odometry,
            "/rtabmap/odom",
            lambda msg: self.on_mapping_odom("/rtabmap/odom", msg),
            10,
        )
        self.create_subscription(
            PointCloud2,
            "/points_merged",
            lambda msg: self.on_mapping_cloud("/points_merged", "points_merged", msg),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PointCloud2,
            "/rtabmap/cloud_map",
            lambda msg: self.on_mapping_cloud("/rtabmap/cloud_map", "rtabmap_cloud_map", msg),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PointCloud2,
            "/rgb_cloud_map",
            lambda msg: self.on_mapping_cloud("/rgb_cloud_map", "rgb_cloud_map", msg),
            qos_profile_sensor_data,
        )
        self.create_subscription(PointCloud2, "/xtm60/points", self.on_xtm60_points, qos_profile_sensor_data)
        self.create_subscription(
            PointCloud2,
            "/xtm60/left/points",
            lambda msg: self.on_xtm60_points_topic("/xtm60/left/points", msg),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PointCloud2,
            "/xtm60/right/points",
            lambda msg: self.on_xtm60_points_topic("/xtm60/right/points", msg),
            qos_profile_sensor_data,
        )
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
        self.create_timer(1.0, self._fetch_map_if_missing)

    @staticmethod
    def _latched_map_qos():
        if QoSProfile is None:
            return 10
        return QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
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

    def _mapping_3d_snapshot(self) -> Dict[str, Any]:
        now = time.monotonic()
        return {
            "points_merged": self._sensor_entry("points_merged", now),
            "rtabmap_odom": self._sensor_entry("rtabmap_odom", now),
            "rtabmap_cloud_map": self._sensor_entry("rtabmap_cloud_map", now),
            "rtabmap_grid_map": self._sensor_entry("rtabmap_grid_map", now),
            "rgb_cloud_map": self._sensor_entry("rgb_cloud_map", now),
        }

    def on_goal_pose(self, msg):
        self.current_goal = {
            "frame_id": msg.header.frame_id,
            "x": msg.pose.position.x,
            "y": msg.pose.position.y,
        }

    def _fetch_map_if_missing(self):
        if self.map_info is not None or self._map_request_in_flight or self.map_client is None:
            return
        if not self.map_client.service_is_ready():
            return
        self._map_request_in_flight = True
        future = self.map_client.call_async(GetMap.Request())
        future.add_done_callback(self._on_map_service_response)

    def _on_map_service_response(self, future):
        self._map_request_in_flight = False
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().warning(f"map service request failed: {exc}")
            return
        self.on_map(response.map)

    def _store_map(self, msg):
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

    def on_map(self, msg):
        self._store_map(msg)

    def on_rtabmap_grid_map(self, msg):
        self._mark_sensor(
            "rtabmap_grid_map",
            {
                "topic": "/rtabmap/grid_map",
                "frame_id": msg.header.frame_id,
                "width": int(msg.info.width),
                "height": int(msg.info.height),
                "resolution": msg.info.resolution,
            },
        )
        self._store_map(msg)

    def on_odom(self, msg):
        if self.pose_source == "amcl" and time.monotonic() - self.pose_last_seen <= self.sensor_timeout_sec:
            pass
        else:
            self.pose = {
                "x": msg.pose.pose.position.x,
                "y": msg.pose.pose.position.y,
                "yaw": self._yaw_from_orientation(msg.pose.pose.orientation),
                "frame_id": msg.header.frame_id,
                "source": "wheel_odom",
            }
            self.pose_last_seen = time.monotonic()
            self.pose_source = "odom"
        self._mark_sensor(
            "odom",
            {
                "topic": "/wheel/odom",
                "frame_id": msg.header.frame_id,
                "linear_x": msg.twist.twist.linear.x,
                "angular_z": msg.twist.twist.angular.z,
            },
        )

    def on_amcl_pose(self, msg):
        self.pose = {
            "x": msg.pose.pose.position.x,
            "y": msg.pose.pose.position.y,
            "yaw": self._yaw_from_orientation(msg.pose.pose.orientation),
            "frame_id": msg.header.frame_id,
            "source": "amcl",
        }
        self.pose_last_seen = time.monotonic()
        self.pose_source = "amcl"

    @staticmethod
    def _yaw_from_orientation(q) -> float:
        if not (q.w or q.x or q.y or q.z):
            return 0.0
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

    def on_route_path(self, topic: str, msg):
        points = []
        poses = list(msg.poses)
        if len(poses) > 600:
            stride = max(1, len(poses) // 600)
            poses = poses[::stride]
        for pose_stamped in poses:
            p = pose_stamped.pose.position
            points.append([float(p.x), float(p.y)])
        self.route_path = {
            "topic": topic,
            "frame_id": msg.header.frame_id or "map",
            "points": points,
            "poses": len(msg.poses),
            "age_sec": 0.0,
        }
        self.route_last_seen = time.monotonic()

    def on_mapping_odom(self, topic: str, msg):
        self._mark_sensor(
            "rtabmap_odom",
            {
                "topic": topic,
                "frame_id": msg.header.frame_id,
                "child_frame_id": msg.child_frame_id,
                "linear_x": msg.twist.twist.linear.x,
                "angular_z": msg.twist.twist.angular.z,
            },
        )

    def on_mapping_cloud(self, topic: str, key: str, msg):
        self._mark_sensor(
            key,
            {
                "topic": topic,
                "frame_id": msg.header.frame_id,
                "points": int(msg.width) * int(msg.height),
            },
        )

    def on_xtm60_points(self, msg):
        self.on_xtm60_points_topic("/xtm60/points", msg)

    def on_xtm60_points_topic(self, topic: str, msg):
        self._mark_sensor(
            "laser",
            {
                "topic": topic,
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
        route = dict(self.route_path)
        if self.route_last_seen:
            route["age_sec"] = max(0.0, time.monotonic() - self.route_last_seen)
        return {
            "safety_state": self.safety_state,
            "navigation_status": self.navigation_status,
            "pose": self.pose,
            "current_goal": self.current_goal,
            "route_path": route,
            "sensor_status": self.sensor_status,
            "sensors": sensors,
            "hardware_config": {
                "ultrasonic_indices": self.ultrasonic_indices,
                "cameras": list(self.camera_topics.keys()),
            },
            "hardware_status": self.hardware_status,
            "localization_health": self.localization_health,
            "passability_status": self.passability_status,
            "mapping_3d": self._mapping_3d_snapshot(),
            "map_available": self.map_info is not None,
            "initial_pose": self.initial_pose_status(),
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
            self.last_goal_error = f"目标点不存在：{name}"
            self._publish_goal_status(f"GOAL_REJECTED: {self.last_goal_error}")
            return False
        if not self._ensure_localized_for_goal():
            return False
        ok, reason = is_map_point_navigable(
            self.map_info,
            float(goal["position"][0]),
            float(goal["position"][1]),
            self.goal_clearance_m,
        )
        if not ok:
            self.last_goal_error = f"{goal.get('label', name)}：{reason}"
            self._publish_goal_status(f"GOAL_REJECTED: {self.last_goal_error}")
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
        self.on_goal_pose(pose)
        self.preview_goal_pub.publish(pose)
        if not self._send_navigate_goal(pose, goal.get("label", name)):
            return False
        self.last_goal_error = None
        return True

    def send_goal_pose(self, x: float, y: float, yaw: float = 0.0) -> bool:
        """Send an ad-hoc clicked map point to Nav2 after validating clearance."""
        if not self._ensure_localized_for_goal():
            return False
        ok, reason = is_map_point_navigable(self.map_info, x, y, self.goal_clearance_m)
        if not ok:
            self.last_goal_error = reason
            self._publish_goal_status(f"GOAL_REJECTED: {reason}")
            return False
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "map"
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        q = yaw_to_quaternion(float(yaw))
        pose.pose.orientation.x = q["x"]
        pose.pose.orientation.y = q["y"]
        pose.pose.orientation.z = q["z"]
        pose.pose.orientation.w = q["w"]
        self.on_goal_pose(pose)
        self.preview_goal_pub.publish(pose)
        if not self._send_navigate_goal(pose, f"x={float(x):.2f}, y={float(y):.2f}"):
            return False
        self.last_goal_error = None
        return True

    def _ensure_localized_for_goal(self) -> bool:
        if self.initial_pose_target is not None and not self.wait_for_initial_pose(
            self.initial_pose_ack_timeout_sec
        ):
            self.last_goal_error = "初始位姿尚未被 AMCL 确认，已拒绝导航以避免从错误位置行驶"
            self._publish_goal_status(f"GOAL_REJECTED: {self.last_goal_error}")
            return False
        if self.pose_source == "amcl" and time.monotonic() - self.pose_last_seen <= self.sensor_timeout_sec:
            return True
        healthy = bool(self.localization_health.get("healthy")) or self.localization_health.get("state") == "GOOD"
        if healthy:
            return True
        self.last_goal_error = "AMCL 定位未就绪，请先设置初始位姿或等待定位恢复"
        self._publish_goal_status(f"GOAL_REJECTED: {self.last_goal_error}")
        return False

    def _publish_goal_status(self, text: str):
        msg = String()
        msg.data = text
        self.goal_status_pub.publish(msg)

    def _send_navigate_goal(self, pose: PoseStamped, label: str) -> bool:
        if self.nav_client is None:
            self.last_goal_error = "Nav2 NavigateToPose action client 不可用"
            self._publish_goal_status(f"GOAL_REJECTED: {self.last_goal_error}")
            return False
        try:
            ready = self.nav_client.server_is_ready() or self.nav_client.wait_for_server(timeout_sec=0.5)
        except Exception as exc:
            self.last_goal_error = f"Nav2 action server 检查失败：{exc}"
            self._publish_goal_status(f"GOAL_REJECTED: {self.last_goal_error}")
            return False
        if not ready:
            self.last_goal_error = "Nav2 /navigate_to_pose action server 未就绪"
            self._publish_goal_status(f"GOAL_REJECTED: {self.last_goal_error}")
            return False
        goal = NavigateToPose.Goal()
        goal.pose = pose
        self._publish_goal_status(f"GOAL_SENT: {label}")
        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(lambda fut, text=label: self._on_nav_goal_response(fut, text))
        return True

    def _on_nav_goal_response(self, future, label: str):
        try:
            handle = future.result()
        except Exception as exc:
            self._publish_goal_status(f"ERROR: NavigateToPose request failed: {exc}")
            return
        if not handle.accepted:
            self._publish_goal_status(f"ERROR: NavigateToPose rejected: {label}")
            return
        self._publish_goal_status(f"NAV2_ACCEPTED: {label}")
        result_future = handle.get_result_async()
        result_future.add_done_callback(lambda fut, text=label: self._on_nav_result(fut, text))

    def _on_nav_result(self, future, label: str):
        try:
            wrapped = future.result()
        except Exception as exc:
            self._publish_goal_status(f"ERROR: NavigateToPose result failed: {exc}")
            return
        status = int(wrapped.status)
        status_name = {
            GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
            GoalStatus.STATUS_ABORTED: "ABORTED",
            GoalStatus.STATUS_CANCELED: "CANCELED",
        }.get(status, f"STATUS_{status}")
        self._publish_goal_status(f"NAV2_{status_name}: {label}")

    def _initial_pose_message(self, x: float, y: float, yaw: float) -> PoseWithCovarianceStamped:
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        q = yaw_to_quaternion(float(yaw))
        msg.pose.pose.orientation.x = q["x"]
        msg.pose.pose.orientation.y = q["y"]
        msg.pose.pose.orientation.z = q["z"]
        msg.pose.pose.orientation.w = q["w"]
        cov = [0.0] * 36
        cov[0] = 0.25      # x variance
        cov[7] = 0.25      # y variance
        cov[35] = 0.0685   # yaw variance (~15 deg)
        msg.pose.covariance = cov
        return msg

    def set_initial_pose(self, x: float, y: float, yaw: float = 0.0) -> bool:
        """Publish /initialpose so AMCL localizes before Nav2 drives."""
        self.initial_pose_target = {"x": float(x), "y": float(y), "yaw": float(yaw)}
        self.initial_pose_requested_at = time.monotonic()
        msg = self._initial_pose_message(x, y, yaw)
        self.initialpose_pub.publish(msg)
        return True

    def initial_pose_acknowledged(self) -> bool:
        if self.initial_pose_target is None:
            return self.pose_source == "amcl" and time.monotonic() - self.pose_last_seen <= self.sensor_timeout_sec
        if self.pose_source != "amcl" or self.pose_last_seen < self.initial_pose_requested_at:
            return False
        distance, yaw_error = pose_delta(
            self.pose,
            self.initial_pose_target["x"],
            self.initial_pose_target["y"],
            self.initial_pose_target["yaw"],
        )
        return (
            distance <= self.initial_pose_ack_tolerance_m
            and yaw_error is not None
            and yaw_error <= self.initial_pose_ack_tolerance_yaw
        )

    def wait_for_initial_pose(self, timeout_sec: Optional[float] = None) -> bool:
        timeout = self.initial_pose_ack_timeout_sec if timeout_sec is None else float(timeout_sec)
        deadline = time.monotonic() + max(0.0, timeout)
        next_republish = 0.0
        while time.monotonic() <= deadline:
            if self.initial_pose_acknowledged():
                return True
            if self.initial_pose_target is not None and time.monotonic() >= next_republish:
                target = self.initial_pose_target
                self.initialpose_pub.publish(
                    self._initial_pose_message(target["x"], target["y"], target["yaw"])
                )
                next_republish = time.monotonic() + 0.5
            time.sleep(0.05)
        return self.initial_pose_acknowledged()

    def initial_pose_status(self) -> Dict[str, Any]:
        target = dict(self.initial_pose_target) if self.initial_pose_target else None
        distance = None
        yaw_error = None
        if target:
            distance, yaw_error = pose_delta(self.pose, target["x"], target["y"], target["yaw"])
        return {
            "target": target,
            "acknowledged": self.initial_pose_acknowledged(),
            "pose_source": self.pose_source,
            "pose_age_sec": None if not self.pose_last_seen else max(0.0, time.monotonic() - self.pose_last_seen),
            "distance_m": distance,
            "yaw_error_rad": yaw_error,
        }

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
