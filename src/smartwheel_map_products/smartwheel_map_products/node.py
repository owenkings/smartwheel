import json
import math
import time
from collections import deque
from copy import deepcopy
from datetime import datetime
from pathlib import Path

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool, Header, String
from std_srvs.srv import Trigger

from smartwheel_map_products.colorizer import CameraFrame, colorize_points
from smartwheel_map_products.formal_acceptance import (
    template as formal_acceptance_template,
    validate_hardware_evidence_binding,
)
from smartwheel_map_products.metrics import build_trajectory_evaluation, evaluate_trajectory
from smartwheel_map_products.occupancy import raycast_occupancy
from smartwheel_map_products.session import SessionCapture, SessionCaptureError
from smartwheel_map_products.writers import export_map_bundle, publish_latest_path
from smartwheel_sensor_api import apply_transform, pointcloud2_from_xyz, pointcloud2_to_xyz
from smartwheel_sensor_api.pointcloud import transform_matrix


CAMERAS = ("front", "left", "right", "rear")
OPTICAL_FROM_LINK = np.array(
    [[0.0, -1.0, 0.0, 0.0], [0.0, 0.0, -1.0, 0.0], [1.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]],
    dtype=np.float64,
)


def _stamp_seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def _yaw_from_quaternion(quaternion) -> float:
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    )


def _normalised_pose_sample(stamp: float, pose) -> tuple[float, float, float, float, float, float, float, float]:
    values = np.asarray(
        [
            stamp,
            pose.position.x,
            pose.position.y,
            pose.position.z,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ],
        dtype=np.float64,
    )
    if not np.isfinite(values).all() or float(stamp) <= 0.0:
        raise ValueError("pose timestamp and components must be finite with a positive timestamp")
    norm = float(np.linalg.norm(values[4:]))
    if not math.isfinite(norm) or norm < 1.0e-9:
        raise ValueError("pose quaternion must be finite and non-zero")
    values[4:] /= norm
    return tuple(float(value) for value in values)


def _validated_backend_path(message: NavPath, expected_frame: str = "map"):
    if message.header.frame_id != expected_frame:
        raise ValueError(
            f"backend path frame must be {expected_frame}, got {message.header.frame_id or '<empty>'}"
        )
    samples = []
    for item in message.poses:
        if item.header.frame_id != expected_frame:
            raise ValueError(
                f"backend path pose frame must be {expected_frame}, got {item.header.frame_id or '<empty>'}"
            )
        sample = _normalised_pose_sample(_stamp_seconds(item.header.stamp), item.pose)
        if samples and sample[0] <= samples[-1][0]:
            raise ValueError("backend path timestamps must be strictly increasing")
        samples.append(sample)
    if len(samples) < 2:
        raise ValueError("backend path must contain at least two optimized poses")
    return samples


def _make_xyz_cloud(header: Header, points: np.ndarray) -> PointCloud2:
    """Publish XYZ without manufacturing a zero-valued intensity channel."""

    xyz = np.asarray(points, dtype=np.float32).reshape(-1, 3)
    fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    return point_cloud2.create_cloud(header, fields, xyz.tolist())


def _backend_snapshot_status(
    *,
    cloud_snapshot_stamp: float | None,
    path_snapshot_stamp: float | None,
    trajectory,
    session_last_stamp: float | None,
    stopped_at_monotonic: float | None,
    cloud_received_at: float | None,
    path_received_at: float | None,
    require_fresh_after_stop: bool,
    maximum_trajectory_lag_sec: float,
    maximum_future_sec: float,
) -> tuple[bool, str]:
    if cloud_snapshot_stamp is None or path_snapshot_stamp is None:
        return False, "backend cloud/path snapshot is incomplete"
    if (
        not math.isfinite(float(cloud_snapshot_stamp))
        or not math.isfinite(float(path_snapshot_stamp))
        or float(cloud_snapshot_stamp) <= 0.0
        or float(path_snapshot_stamp) <= 0.0
    ):
        return False, "backend cloud/path snapshot timestamps must be finite and positive"
    if abs(cloud_snapshot_stamp - path_snapshot_stamp) > 1.0e-6:
        return False, "backend cloud and optimized path came from different snapshots"
    if not trajectory:
        return False, "backend optimized trajectory is empty"
    if require_fresh_after_stop:
        if stopped_at_monotonic is None:
            return False, "map session has not stopped"
        if (
            cloud_received_at is None
            or path_received_at is None
            or cloud_received_at <= stopped_at_monotonic
            or path_received_at <= stopped_at_monotonic
        ):
            return False, "waiting for a backend cloud/path snapshot captured after session stop"
    if session_last_stamp is None:
        return False, "map session has no final cloud timestamp"
    backend_last = float(trajectory[-1][0])
    if backend_last > session_last_stamp + maximum_future_sec:
        return False, "backend trajectory extends beyond the stopped input session"
    if session_last_stamp - backend_last > maximum_trajectory_lag_sec:
        return False, "backend optimized trajectory is too stale for the stopped input session"
    return True, "backend cloud/path snapshot matches the stopped input session"


def _validated_tf_edges(values: dict[str, str]) -> dict[str, str]:
    """Validate the formal global->local->base TF ownership chain."""

    result = {}
    parsed = {}
    for role in ("global_correction", "local_odometry", "body_bridge"):
        value = str(values.get(role, "")).strip()
        if role == "body_bridge" and not value:
            continue
        parts = value.split("->")
        if (
            len(parts) != 2
            or any(not part or part.startswith("/") or part.strip() != part for part in parts)
            or parts[0] == parts[1]
        ):
            raise ValueError(f"invalid {role} TF edge: {value or '<empty>'}")
        result[role] = value
        parsed[role] = tuple(parts)
    if "global_correction" not in parsed or "local_odometry" not in parsed:
        raise ValueError("global_correction and local_odometry TF edges are required")
    if parsed["global_correction"][1] != parsed["local_odometry"][0]:
        raise ValueError("global correction child must match local odometry parent")
    if "body_bridge" in parsed:
        if (
            parsed["local_odometry"][1] != parsed["body_bridge"][0]
            or parsed["body_bridge"][1] != "base_link"
        ):
            raise ValueError("local odometry/body bridge chain must end at base_link")
    elif parsed["local_odometry"][1] != "base_link":
        raise ValueError("local odometry TF edge must end at base_link without a bridge")
    return result


def _tf_evidence_matches(tf_gate: object, expected_edges: dict[str, str]) -> bool:
    if not isinstance(tf_gate, dict):
        return False
    if str(tf_gate.get("status", "")).strip().upper() not in {
        "PASS",
        "PASSED",
        "APPROVED",
        "VALIDATED",
        "TRUE",
    }:
        return False
    evidence_edges = tf_gate.get("edges")
    if not isinstance(evidence_edges, dict):
        return False
    for role, expected in expected_edges.items():
        item = evidence_edges.get(role)
        try:
            publishers = float(item.get("publishers")) if isinstance(item, dict) else 0.0
        except (TypeError, ValueError):
            return False
        if not isinstance(item, dict) or item.get("edge") != expected or publishers != 1.0:
            return False
    return True


def _load_formal_acceptance_evidence(path: str) -> dict | None:
    """Read one complete evidence snapshot, rejecting partial/old schemas."""

    normalized = str(path).strip()
    if not normalized:
        return None
    try:
        evidence = json.loads(
            Path(normalized).expanduser().read_text(encoding="utf-8")
        )
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise ValueError(f"formal_acceptance_evidence_path is invalid: {exc}") from exc
    if not isinstance(evidence, dict):
        raise ValueError("formal_acceptance_evidence_path must contain a JSON object")
    if evidence.get("schema_version") != 1:
        raise ValueError(
            "formal_acceptance_evidence_path has unsupported schema_version"
        )
    return evidence


class MapProductsNode(Node):
    def __init__(self) -> None:
        super().__init__("smartwheel_map_products")
        self.declare_parameter("map_name", "stage_a_sim")
        self.declare_parameter("output_root", "maps/versions")
        self.declare_parameter("version_directory", "")
        self.declare_parameter("hardware_profile_path", "")
        self.declare_parameter("calibration_contract_path", "")
        self.declare_parameter("mapping_backend", "rtabmap")
        self.declare_parameter("state_mode", "lio_primary")
        self.declare_parameter("lidar_mode", "dual_map_only")
        self.declare_parameter("bag_path", "")
        self.declare_parameter("resolution_m", 0.05)
        self.declare_parameter("voxel_size_m", 0.05)
        self.declare_parameter("maximum_accumulated_points", 500000)
        self.declare_parameter("min_obstacle_z", 0.1)
        self.declare_parameter("max_obstacle_z", 2.2)
        self.declare_parameter("export_delay_sec", 1.0)
        self.declare_parameter("enable_offline_colorization", True)
        self.declare_parameter("ground_truth_topic", "")
        self.declare_parameter("backend_cloud_topic", "")
        self.declare_parameter("require_backend_cloud", False)
        self.declare_parameter("backend_path_topic", "")
        self.declare_parameter("require_backend_trajectory", False)
        self.declare_parameter("require_fresh_backend_after_stop", False)
        self.declare_parameter("backend_max_trajectory_lag_sec", 2.5)
        self.declare_parameter("cloud_topic", "/lidar/merged/points")
        self.declare_parameter("odom_topic", "/odom/fused")
        self.declare_parameter("cloud_frame_mode", "base_frame")
        self.declare_parameter("expected_cloud_frame", "")
        self.declare_parameter("require_session_stop", True)
        self.declare_parameter("validation_stage", "A_SYNTHETIC")
        self.declare_parameter("hardware_validated", False)
        self.declare_parameter("mock_lio", True)
        self.declare_parameter("require_intensity", False)
        self.declare_parameter("formal_acceptance_evidence_path", "")
        self.declare_parameter("tf_global_edge", "map->camera_init")
        self.declare_parameter("tf_local_edge", "camera_init->body")
        self.declare_parameter("tf_body_bridge_edge", "body->base_link")
        self.declare_parameter("maximum_evaluation_time_delta_sec", 0.1)
        self.declare_parameter("maximum_pose_time_delta_sec", 0.15)
        self.declare_parameter("pending_cloud_queue_size", 8192)
        camera_defaults = {
            "front": ([0.40, 0.0, 0.82], [0.0, 0.0, 0.0]),
            "left": ([0.0, 0.28, 0.80], [0.0, 0.0, math.pi / 2.0]),
            "right": ([0.0, -0.28, 0.80], [0.0, 0.0, -math.pi / 2.0]),
            "rear": ([-0.28, 0.0, 0.78], [0.0, 0.0, math.pi]),
        }
        for name, (xyz, rpy) in camera_defaults.items():
            self.declare_parameter(f"{name}_camera_xyz", xyz)
            self.declare_parameter(f"{name}_camera_rpy", rpy)

        # Resolve and validate formal provenance before creating an output
        # directory.  The same fixed paths are revalidated immediately before
        # export, and the writer validates the copied bundle snapshots again.
        self._hardware_profile_path = str(
            self.get_parameter("hardware_profile_path").value
        ).strip()
        self._calibration_contract_path = str(
            self.get_parameter("calibration_contract_path").value
        ).strip()
        self._formal_acceptance_evidence_path = str(
            self.get_parameter("formal_acceptance_evidence_path").value
        ).strip()
        self._hardware_validated = bool(
            self.get_parameter("hardware_validated").value
        )
        self._formal_acceptance = _load_formal_acceptance_evidence(
            self._formal_acceptance_evidence_path
        )
        if self._hardware_validated:
            provenance_ok, provenance_reason = validate_hardware_evidence_binding(
                self._formal_acceptance,
                hardware_profile_path=self._hardware_profile_path,
                calibration_contract_path=self._calibration_contract_path,
            )
            if not provenance_ok:
                raise ValueError(
                    "hardware_validated=true requires bound hardware provenance: "
                    + provenance_reason
                )

        root = Path(str(self.get_parameter("output_root").value)).expanduser().resolve()
        self._output_root = root
        explicit = str(self.get_parameter("version_directory").value).strip()
        if explicit:
            self._directory = Path(explicit).expanduser().resolve()
        else:
            stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            self._directory = root / f"{self.get_parameter('map_name').value}_{stamp}"
        self._directory.mkdir(parents=True, exist_ok=True)
        root.mkdir(parents=True, exist_ok=True)

        self._resolution = float(self.get_parameter("resolution_m").value)
        self._voxel = float(self.get_parameter("voxel_size_m").value)
        self._min_z = float(self.get_parameter("min_obstacle_z").value)
        self._max_z = float(self.get_parameter("max_obstacle_z").value)
        self._export_delay = float(self.get_parameter("export_delay_sec").value)
        self._color_enabled = bool(self.get_parameter("enable_offline_colorization").value)
        self._backend_cloud_topic = str(self.get_parameter("backend_cloud_topic").value).strip()
        self._require_backend_cloud = bool(self.get_parameter("require_backend_cloud").value)
        self._backend_path_topic = str(self.get_parameter("backend_path_topic").value).strip()
        self._require_backend_trajectory = bool(
            self.get_parameter("require_backend_trajectory").value
        )
        self._require_fresh_backend_after_stop = bool(
            self.get_parameter("require_fresh_backend_after_stop").value
        )
        self._backend_max_trajectory_lag = float(
            self.get_parameter("backend_max_trajectory_lag_sec").value
        )
        if self._require_backend_trajectory and not self._backend_path_topic:
            raise ValueError("backend_path_topic is required when backend trajectory is required")
        if not math.isfinite(self._backend_max_trajectory_lag) or self._backend_max_trajectory_lag < 0.0:
            raise ValueError("backend_max_trajectory_lag_sec must be finite and non-negative")
        self._cloud_topic = str(self.get_parameter("cloud_topic").value).strip()
        self._odom_topic = str(self.get_parameter("odom_topic").value).strip()
        if not self._cloud_topic or not self._odom_topic:
            raise ValueError("cloud_topic and odom_topic must be non-empty")
        self._cloud_frame_mode = str(self.get_parameter("cloud_frame_mode").value).strip()
        if self._cloud_frame_mode not in ("base_frame", "world_registered"):
            raise ValueError("cloud_frame_mode must be base_frame or world_registered")
        self._expected_cloud_frame = str(self.get_parameter("expected_cloud_frame").value).strip()
        if self._cloud_frame_mode == "world_registered" and not self._expected_cloud_frame:
            raise ValueError("expected_cloud_frame is required for world_registered clouds")
        self._require_session_stop = bool(self.get_parameter("require_session_stop").value)
        self._validation_stage = str(self.get_parameter("validation_stage").value).strip()
        self._mock_lio = bool(self.get_parameter("mock_lio").value)
        self._require_intensity = bool(self.get_parameter("require_intensity").value)
        self._tf_edges = _validated_tf_edges(
            {
                "global_correction": self.get_parameter("tf_global_edge").value,
                "local_odometry": self.get_parameter("tf_local_edge").value,
                "body_bridge": self.get_parameter("tf_body_bridge_edge").value,
            }
        )
        self._tf_runtime_check = "NOT_PERFORMED_BY_EXPORTER"
        evidence = self._formal_acceptance or {}
        gates = evidence.get("gates") if isinstance(evidence, dict) else None
        tf_gate = gates.get("tf_ownership") if isinstance(gates, dict) else None
        if _tf_evidence_matches(tf_gate, self._tf_edges):
            self._tf_runtime_check = "PASS_EVIDENCE"
        self._maximum_evaluation_delta = float(
            self.get_parameter("maximum_evaluation_time_delta_sec").value
        )
        self._maximum_pose_delta = float(self.get_parameter("maximum_pose_time_delta_sec").value)
        self._camera_extrinsics = {
            name: transform_matrix(
                self.get_parameter(f"{name}_camera_xyz").value,
                self.get_parameter(f"{name}_camera_rpy").value,
            )
            for name in CAMERAS
        }
        self._session = SessionCapture(
            self._voxel,
            int(self.get_parameter("maximum_accumulated_points").value),
            expected_frame_id=self._expected_cloud_frame,
        )
        self._session.start()
        self._accumulator = self._session.accumulator
        self._accumulation_failure = ""
        self._poses = []
        self._odom_trajectory = []
        self._ground_truth = []
        self._backend_points = None
        self._backend_intensity = None
        self._backend_frame = ""
        self._backend_cloud_snapshot_stamp = None
        self._backend_cloud_received_at = None
        self._backend_cloud_error = ""
        self._backend_trajectory = None
        self._backend_path_frame = ""
        self._backend_path_snapshot_stamp = None
        self._backend_path_received_at = None
        self._backend_path_error = ""
        self._session_stopped_at = None
        self._camera_info = {}
        self._camera_frames: list[CameraFrame] = []
        self._completed_at = None
        self._exported = False
        self._export_failure = ""
        self._rejected_pose_associations = 0
        pending_queue_size = int(
            self.get_parameter("pending_cloud_queue_size").value
        )
        if pending_queue_size < 1:
            raise ValueError("pending_cloud_queue_size must be positive")
        self._pending_clouds = deque(maxlen=pending_queue_size)
        self._pending_cloud_drops = 0
        self._bag_path = str(self.get_parameter("bag_path").value).strip()

        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._cloud_pub = self.create_publisher(PointCloud2, "/map_products/cloud", latched)
        self._cloud_amp_pub = self.create_publisher(
            PointCloud2,
            "/map_products/cloud_amp",
            latched,
        )
        self._map_pub = self.create_publisher(OccupancyGrid, "/map_products/occupancy", latched)
        self._complete_pub = self.create_publisher(String, "/map_export/completed", latched)
        self._failure_pub = self.create_publisher(String, "/map_export/failed", latched)
        self.create_subscription(String, "/workbench/bag_path", self._on_bag_path, latched)
        self.create_service(Trigger, "/map_export/export", self._on_export_request)
        self.create_service(Trigger, "/map_session/stop", self._on_stop_request)
        self.create_subscription(
            PointCloud2,
            self._cloud_topic,
            self._on_cloud,
            qos_profile_sensor_data,
        )
        odom_qos = QoSProfile(depth=500, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Odometry, self._odom_topic, self._on_odom, odom_qos)
        ground_truth_topic = str(self.get_parameter("ground_truth_topic").value).strip()
        if ground_truth_topic:
            self.create_subscription(Odometry, ground_truth_topic, self._on_ground_truth, odom_qos)
        if self._backend_cloud_topic:
            self.create_subscription(
                PointCloud2,
                self._backend_cloud_topic,
                self._on_backend_cloud,
                qos_profile_sensor_data,
            )
        if self._backend_path_topic:
            self.create_subscription(
                NavPath,
                self._backend_path_topic,
                self._on_backend_path,
                latched,
            )
        self.create_subscription(Bool, "/sim/completed", self._on_completed, latched)
        if self._color_enabled:
            for name in CAMERAS:
                self.create_subscription(
                    CameraInfo,
                    f"/camera/{name}/camera_info",
                    lambda msg, camera=name: self._on_camera_info(camera, msg),
                    latched,
                )
                self.create_subscription(
                    Image,
                    f"/camera/{name}/image_raw",
                    lambda msg, camera=name: self._on_image(camera, msg),
                    qos_profile_sensor_data,
                )
        self.create_timer(0.5, self._tick)

    def _on_bag_path(self, message: String) -> None:
        self._bag_path = message.data.strip()

    def _on_odom(self, message: Odometry) -> None:
        if self._exported or self._session.state != SessionCapture.CAPTURING:
            return
        stamp = _stamp_seconds(message.header.stamp)
        pose = message.pose.pose
        try:
            full_pose = _normalised_pose_sample(stamp, pose)
            if self._odom_trajectory and stamp <= self._odom_trajectory[-1][0]:
                raise ValueError("odometry timestamps must be strictly increasing")
        except ValueError as exc:
            self._accumulation_failure = str(exc)
            self._session.fail(self._accumulation_failure)
            self._failure_pub.publish(String(data=self._accumulation_failure))
            self.get_logger().error(self._accumulation_failure)
            return
        self._odom_trajectory.append(full_pose)
        self._poses.append((stamp, pose.position.x, pose.position.y, _yaw_from_quaternion(pose.orientation)))
        self._flush_pending_clouds()

    def _on_ground_truth(self, message: Odometry) -> None:
        if self._exported or self._session.state != SessionCapture.CAPTURING:
            return
        pose = message.pose.pose
        self._ground_truth.append(
            (_stamp_seconds(message.header.stamp), pose.position.x, pose.position.y, _yaw_from_quaternion(pose.orientation))
        )

    def _on_backend_cloud(self, message: PointCloud2) -> None:
        if self._exported:
            return
        try:
            points, intensity = pointcloud2_to_xyz(message)
        except ValueError as exc:
            self._backend_points = None
            self._backend_intensity = None
            self._backend_cloud_error = str(exc)
            self.get_logger().error(f"backend cloud rejected: {exc}")
            return
        if (
            points.size == 0
            or not np.isfinite(points).all()
            or np.any(np.linalg.norm(points, axis=1) <= 0.0)
        ):
            self._backend_points = None
            self._backend_intensity = None
            self._backend_cloud_error = "backend cloud is empty or contains invalid XYZ"
            return
        if intensity is not None and (
            intensity.shape[0] != points.shape[0]
            or not np.isfinite(intensity).all()
            or np.any(intensity < 0.0)
        ):
            self._backend_points = None
            self._backend_intensity = None
            self._backend_cloud_error = "backend intensity is non-finite, negative, or misaligned"
            self.get_logger().error("backend cloud rejected: intensity is non-finite or misaligned")
            return
        if self._require_intensity and (intensity is None or not np.any(intensity > 0.0)):
            self._backend_points = None
            self._backend_intensity = None
            self._backend_cloud_error = "required backend intensity is missing or all zero"
            self.get_logger().error(self._backend_cloud_error)
            return
        self._backend_points = points
        self._backend_intensity = intensity
        self._backend_frame = message.header.frame_id
        self._backend_cloud_snapshot_stamp = _stamp_seconds(message.header.stamp)
        self._backend_cloud_received_at = time.monotonic()
        self._backend_cloud_error = ""

    def _on_backend_path(self, message: NavPath) -> None:
        if self._exported:
            return
        try:
            trajectory = _validated_backend_path(message)
        except ValueError as exc:
            self._backend_trajectory = None
            self._backend_path_error = str(exc)
            self.get_logger().error(f"backend path rejected: {exc}")
            return
        self._backend_trajectory = trajectory
        self._backend_path_frame = message.header.frame_id
        self._backend_path_snapshot_stamp = _stamp_seconds(message.header.stamp)
        self._backend_path_received_at = time.monotonic()
        self._backend_path_error = ""

    def _nearest_pose(self, stamp: float):
        if not self._poses:
            return None
        pose = min(self._poses, key=lambda candidate: abs(candidate[0] - stamp))
        return pose if abs(pose[0] - stamp) <= self._maximum_pose_delta else None

    def _queue_pending_cloud(self, message: PointCloud2) -> None:
        if len(self._pending_clouds) == self._pending_clouds.maxlen:
            self._pending_clouds.popleft()
            self._pending_cloud_drops += 1
            self._rejected_pose_associations += 1
            self._session.reject_frame()
        self._pending_clouds.append(deepcopy(message))

    def _flush_pending_clouds(self) -> None:
        while self._pending_clouds and not self._accumulation_failure:
            message = self._pending_clouds[0]
            stamp = _stamp_seconds(message.header.stamp)
            pose = self._nearest_pose(stamp)
            if pose is None:
                if not self._poses:
                    break
                latest_stamp = self._poses[-1][0]
                if latest_stamp - stamp <= self._maximum_pose_delta:
                    # The cloud is ahead of the latest odometry sample; retain
                    # it until the corresponding future pose arrives.
                    break
                self._pending_clouds.popleft()
                self._pending_cloud_drops += 1
                self._rejected_pose_associations += 1
                self._session.reject_frame()
                continue
            self._pending_clouds.popleft()
            self._process_cloud(message, pose)

    def _reject_pending_clouds(self) -> None:
        while self._pending_clouds:
            self._pending_clouds.popleft()
            self._pending_cloud_drops += 1
            self._rejected_pose_associations += 1
            self._session.reject_frame()

    def _on_cloud(self, message: PointCloud2) -> None:
        if (
            self._exported
            or self._accumulation_failure
            or self._session.state != SessionCapture.CAPTURING
        ):
            return
        pose = self._nearest_pose(_stamp_seconds(message.header.stamp))
        if pose is None:
            self._queue_pending_cloud(message)
            return
        self._process_cloud(message, pose)

    def _process_cloud(self, message: PointCloud2, pose) -> None:
        stamp = _stamp_seconds(message.header.stamp)
        try:
            points, intensity = pointcloud2_to_xyz(message)
        except ValueError as exc:
            self._accumulation_failure = str(exc)
            self._session.fail(self._accumulation_failure)
            self._failure_pub.publish(String(data=self._accumulation_failure))
            self.get_logger().error(self._accumulation_failure)
            return
        if self._cloud_frame_mode == "base_frame":
            map_from_base = transform_matrix([pose[1], pose[2], 0.0], [0.0, 0.0, pose[3]])
            world = apply_transform(points, map_from_base)
        else:
            world = points
        origins = np.tile(np.array([pose[1], pose[2], 0.0]), (world.shape[0], 1))
        try:
            self._session.add_frame(
                stamp,
                message.header.frame_id,
                world,
                origins,
                intensity,
            )
        except SessionCaptureError as exc:
            self._accumulation_failure = str(exc)
            self._failure_pub.publish(String(data=self._accumulation_failure))
            self.get_logger().error(self._accumulation_failure)

    def _on_camera_info(self, name: str, message: CameraInfo) -> None:
        if message.k[0] > 0.0 and message.k[4] > 0.0:
            self._camera_info[name] = np.asarray(message.k, dtype=np.float64).reshape(3, 3)

    def _on_image(self, name: str, message: Image) -> None:
        if self._exported:
            return
        if name not in self._camera_info or message.encoding not in ("rgb8", "bgr8"):
            return
        pose = self._nearest_pose(_stamp_seconds(message.header.stamp))
        if pose is None:
            return
        expected = int(message.height) * int(message.width) * 3
        data = np.frombuffer(bytes(message.data), dtype=np.uint8)
        if data.size < expected:
            return
        image = data[:expected].reshape(int(message.height), int(message.width), 3).copy()
        if message.encoding == "bgr8":
            image = image[..., ::-1]
        map_from_base = transform_matrix([pose[1], pose[2], 0.0], [0.0, 0.0, pose[3]])
        map_from_camera_link = map_from_base @ self._camera_extrinsics[name]
        camera_from_map = OPTICAL_FROM_LINK @ np.linalg.inv(map_from_camera_link)
        self._camera_frames.append(
            CameraFrame(image, self._camera_info[name], camera_from_map, _stamp_seconds(message.header.stamp))
        )
        if len(self._camera_frames) > 240:
            self._camera_frames = self._camera_frames[-240:]

    def _on_completed(self, message: Bool) -> None:
        if message.data and self._completed_at is None:
            if self._session.state is SessionCapture.CAPTURING:
                self._flush_pending_clouds()
                # No future odometry can arrive after completion.  Account for
                # any still-ahead clouds explicitly so formal acceptance cannot
                # mistake an unassociated tail for a complete capture.
                self._reject_pending_clouds()
                self._session.stop()
                self._session_stopped_at = time.monotonic()
            self._completed_at = time.monotonic()

    def _on_stop_request(self, _request, response):
        if self._session.state is SessionCapture.STOPPED:
            response.success = True
            response.message = "map session already stopped"
            return response
        if self._session.state is SessionCapture.FAILED:
            response.success = False
            response.message = self._session.failure_reason or "map session failed"
            return response
        try:
            self._flush_pending_clouds()
            # Once STOP is acknowledged no future odometry may be used to
            # associate the remaining clouds.  Count the tail as rejected so a
            # formal zero-drop gate cannot silently ignore it.
            self._reject_pending_clouds()
            self._session.stop()
            self._session_stopped_at = time.monotonic()
        except SessionCaptureError as exc:
            response.success = False
            response.message = str(exc)
            return response
        response.success = True
        response.message = "map session stopped; call /map_export/export to finalize"
        return response

    def _on_export_request(self, _request, response):
        if self._exported:
            self._complete_pub.publish(String(data=str(self._directory)))
            response.success = True
            response.message = f"already exported: {self._directory}"
            return response
        if self._require_session_stop and self._session.state is not SessionCapture.STOPPED:
            response.success = False
            response.message = (
                self._session.failure_reason
                or "map session is still capturing; call /map_session/stop first"
            )
            self._failure_pub.publish(String(data=response.message))
            return response
        products = self._current_products()
        if products is None:
            response.success = False
            response.message = self._missing_products_reason()
            self._failure_pub.publish(String(data=response.message))
            return response
        if not self._try_export(products):
            response.success = False
            response.message = self._export_failure
            return response
        response.success = True
        response.message = str(self._directory)
        return response

    def _current_products(self):
        if self._accumulation_failure or len(self._accumulator) == 0:
            return None
        local_points, origins, local_intensity = (
            self._accumulator.arrays_with_intensity()
        )
        grid = raycast_occupancy(
            local_points,
            origins,
            resolution=self._resolution,
            min_obstacle_z=self._min_z,
            max_obstacle_z=self._max_z,
        )
        if self._require_backend_cloud:
            if self._backend_points is None or self._backend_frame != "map":
                return None
            geometry = self._backend_points
            geometry_source = f"backend:{self._backend_cloud_topic}"
            if self._backend_intensity is not None and self._backend_intensity.shape[0] == geometry.shape[0]:
                amp_points, amp_intensity = geometry, self._backend_intensity
            else:
                if self._require_intensity:
                    return None
                amp_points, amp_intensity = local_points, local_intensity
        else:
            geometry = local_points
            geometry_source = "local_odometry_accumulator"
            amp_points, amp_intensity = local_points, local_intensity
        if self._require_backend_trajectory:
            summary = self._session.summary()
            snapshot_ok, _ = _backend_snapshot_status(
                cloud_snapshot_stamp=self._backend_cloud_snapshot_stamp,
                path_snapshot_stamp=self._backend_path_snapshot_stamp,
                trajectory=self._backend_trajectory,
                session_last_stamp=summary.last_cloud_stamp,
                stopped_at_monotonic=self._session_stopped_at,
                cloud_received_at=self._backend_cloud_received_at,
                path_received_at=self._backend_path_received_at,
                require_fresh_after_stop=self._require_fresh_backend_after_stop,
                maximum_trajectory_lag_sec=self._backend_max_trajectory_lag,
                maximum_future_sec=self._maximum_pose_delta,
            )
            if not snapshot_ok:
                return None
            trajectory = self._backend_trajectory
            trajectory_source = f"backend:{self._backend_path_topic}"
        else:
            trajectory = self._odom_trajectory
            trajectory_source = f"odometry:{self._odom_topic}"
        if not trajectory:
            return None
        return (
            geometry,
            origins,
            grid,
            geometry_source,
            amp_points,
            amp_intensity,
            trajectory,
            trajectory_source,
        )

    def _missing_products_reason(self) -> str:
        if self._accumulation_failure:
            return self._accumulation_failure
        if self._session.state is SessionCapture.FAILED:
            return self._session.failure_reason or "map session failed"
        if self._require_session_stop and self._session.state is not SessionCapture.STOPPED:
            return "map session is still capturing; call /map_session/stop first"
        if len(self._accumulator) == 0:
            return "no merged point cloud has been received"
        if self._require_backend_cloud and self._backend_points is None:
            return self._backend_cloud_error or f"required backend cloud has not been received: {self._backend_cloud_topic}"
        if self._require_backend_cloud and self._backend_frame != "map":
            return f"required backend cloud frame must be map, got: {self._backend_frame or '<empty>'}"
        if self._require_intensity:
            if self._require_backend_cloud and self._backend_points is not None and (self._backend_intensity is None or self._backend_intensity.shape[0] != self._backend_points.shape[0]):
                return "required backend cloud does not preserve a matching intensity field"
            if not self._require_backend_cloud:
                _, _, local_intensity = self._accumulator.arrays_with_intensity()
                if local_intensity is None:
                    return "required map input does not contain intensity"
        if self._require_backend_trajectory:
            if self._backend_trajectory is None:
                return self._backend_path_error or f"required backend path has not been received: {self._backend_path_topic}"
            summary = self._session.summary()
            ok, reason = _backend_snapshot_status(
                cloud_snapshot_stamp=self._backend_cloud_snapshot_stamp,
                path_snapshot_stamp=self._backend_path_snapshot_stamp,
                trajectory=self._backend_trajectory,
                session_last_stamp=summary.last_cloud_stamp,
                stopped_at_monotonic=self._session_stopped_at,
                cloud_received_at=self._backend_cloud_received_at,
                path_received_at=self._backend_path_received_at,
                require_fresh_after_stop=self._require_fresh_backend_after_stop,
                maximum_trajectory_lag_sec=self._backend_max_trajectory_lag,
                maximum_future_sec=self._maximum_pose_delta,
            )
            if not ok:
                return reason
        if not (self._backend_trajectory if self._require_backend_trajectory else self._odom_trajectory):
            return "no trajectory poses have been received"
        return "map products are unavailable"

    def _tick(self) -> None:
        if (
            self._completed_at is not None
            and not self._exported
            and not self._export_failure
            and time.monotonic() - self._completed_at >= self._export_delay
        ):
            products = self._current_products()
            if products is not None:
                self._try_export(products)
            else:
                self._failure_pub.publish(String(data=self._missing_products_reason()))

    def _try_export(self, products) -> bool:
        try:
            self._publish(products[0], products[2], products[4], products[5])
            self._export(*products)
        except Exception as exc:
            self._export_failure = f"map export failed: {exc}"
            self._failure_pub.publish(String(data=self._export_failure))
            self.get_logger().error(self._export_failure)
            return False
        return True

    def _publish(self, points, grid, intensity_points, intensity) -> None:
        header = Header(stamp=self.get_clock().now().to_msg(), frame_id="map")
        self._cloud_pub.publish(_make_xyz_cloud(header, points))
        if intensity is not None:
            self._cloud_amp_pub.publish(
                pointcloud2_from_xyz(header, intensity_points, intensity)
            )
        message = OccupancyGrid()
        message.header = header
        message.info.map_load_time = header.stamp
        message.info.resolution = grid.resolution
        message.info.width = grid.width
        message.info.height = grid.height
        message.info.origin.position.x = grid.origin_x
        message.info.origin.position.y = grid.origin_y
        message.info.origin.orientation.w = 1.0
        message.data = grid.cells.ravel().astype(np.int8).tolist()
        self._map_pub.publish(message)

    def _export(
        self,
        points,
        _origins,
        grid,
        geometry_source: str,
        intensity_points,
        intensity,
        trajectory,
        trajectory_source: str,
    ) -> None:
        # Evidence may be completed atomically after the session is stopped.
        # Re-read it for this export so the bundle never embeds the startup
        # snapshot by accident.
        self._formal_acceptance = _load_formal_acceptance_evidence(
            self._formal_acceptance_evidence_path
        )
        if self._hardware_validated:
            provenance_ok, provenance_reason = validate_hardware_evidence_binding(
                self._formal_acceptance,
                hardware_profile_path=self._hardware_profile_path,
                calibration_contract_path=self._calibration_contract_path,
            )
            if not provenance_ok:
                raise ValueError(
                    "hardware_validated=true requires current bound hardware "
                    "provenance: "
                    + provenance_reason
                )
        gates = (
            self._formal_acceptance.get("gates")
            if isinstance(self._formal_acceptance, dict)
            else None
        )
        tf_gate = gates.get("tf_ownership") if isinstance(gates, dict) else None
        self._tf_runtime_check = (
            "PASS_EVIDENCE"
            if _tf_evidence_matches(tf_gate, self._tf_edges)
            else "NOT_PERFORMED_BY_EXPORTER"
        )
        colors = None
        colored_count = 0
        if self._color_enabled and self._camera_frames:
            stride = max(1, len(self._camera_frames) // 64)
            colors, colored = colorize_points(points, self._camera_frames[::stride])
            colored_count = int(colored.sum())
        occupied = int(np.count_nonzero(grid.cells == 100))
        free = int(np.count_nonzero(grid.cells == 0))
        unknown = int(np.count_nonzero(grid.cells == -1))
        trajectory_metrics = evaluate_trajectory(
            self._poses,
            self._ground_truth,
            max_time_delta_sec=self._maximum_evaluation_delta,
        )
        trajectory_evaluation = build_trajectory_evaluation(self._poses)
        minimum = points.min(axis=0)
        maximum = points.max(axis=0)
        amp_source = "backend_registered_cloud" if self._require_backend_cloud and intensity_points is self._backend_points and intensity is self._backend_intensity else "input_accumulator"
        amplitude_metrics = {
            "pointcloud_amp_preserved": intensity is not None,
            "pointcloud_amp_point_count": 0 if intensity is None else int(intensity.shape[0]),
            "pointcloud_amp_min": None if intensity is None else float(np.min(intensity)),
            "pointcloud_amp_median": None if intensity is None else float(np.median(intensity)),
            "pointcloud_amp_p95": None if intensity is None else float(np.percentile(intensity, 95.0)),
            "pointcloud_amp_max": None if intensity is None else float(np.max(intensity)),
            "pointcloud_amp_source": amp_source,
        }
        quality = {
            "stage": self._validation_stage,
            "point_count": int(points.shape[0]),
            "colored_point_count": colored_count,
            "occupied_cells": occupied,
            "free_cells": free,
            "unknown_cells": unknown,
            "trajectory_pose_count": len(trajectory),
            "trajectory_source": trajectory_source,
            "trajectory_representation": "full_6dof_quaternion",
            "trajectory_first_stamp": float(trajectory[0][0]),
            "trajectory_last_stamp": float(trajectory[-1][0]),
            "ground_truth_pose_count": len(self._ground_truth),
            "rejected_pose_associations": self._rejected_pose_associations,
            "trajectory_evaluation": trajectory_evaluation,
            "geometry_source": geometry_source,
            "backend_cloud_snapshot_stamp": self._backend_cloud_snapshot_stamp,
            "backend_path_snapshot_stamp": self._backend_path_snapshot_stamp,
            "backend_snapshot_after_session_stop": bool(
                self._session_stopped_at is not None
                and self._backend_cloud_received_at is not None
                and self._backend_path_received_at is not None
                and self._backend_cloud_received_at > self._session_stopped_at
                and self._backend_path_received_at > self._session_stopped_at
            ),
            "raycast_occupancy_source": "local_odometry_accumulator",
            **trajectory_metrics,
            "map_bounds_min_m": minimum.tolist(),
            "map_bounds_max_m": maximum.tolist(),
            "tf_conflict_runtime_check": self._tf_runtime_check,
            "hardware_validated": self._hardware_validated,
            "formal_acceptance": self._formal_acceptance or formal_acceptance_template(),
            **amplitude_metrics,
        }
        quality["mock_lio"] = self._mock_lio
        profile = {
            "mapping_backend": str(self.get_parameter("mapping_backend").value),
            "state_mode": str(self.get_parameter("state_mode").value),
            "lidar_mode": str(self.get_parameter("lidar_mode").value),
            "resolution_m": self._resolution,
            "voxel_size_m": self._voxel,
            "cloud_topic": self._cloud_topic,
            "odom_topic": self._odom_topic,
            "cloud_frame_mode": self._cloud_frame_mode,
            "expected_cloud_frame": self._expected_cloud_frame,
            "mock_lio": self._mock_lio,
            "require_backend_cloud": self._require_backend_cloud,
            "require_intensity": self._require_intensity,
            "backend_cloud_topic": self._backend_cloud_topic,
            "require_backend_trajectory": self._require_backend_trajectory,
            "backend_path_topic": self._backend_path_topic,
            "require_fresh_backend_after_stop": self._require_fresh_backend_after_stop,
            "backend_max_trajectory_lag_sec": self._backend_max_trajectory_lag,
            "trajectory_source": trajectory_source,
            "trajectory_representation": "full_6dof_quaternion",
            "calibration_contract_bundled": bool(self._calibration_contract_path),
            "tf_edges": self._tf_edges,
            "formal_acceptance_evidence_path": self._formal_acceptance_evidence_path,
            "hardware_validated": self._hardware_validated,
            "validation_stage": self._validation_stage,
        }
        quality["session"] = self._session.summary().as_dict()
        export_map_bundle(
            self._directory,
            points,
            grid,
            self._poses,
            colors,
            self._hardware_profile_path,
            profile,
            self._bag_path,
            quality,
            intensity_points=intensity_points,
            intensity=intensity,
            trajectory_poses=trajectory,
            calibration_contract_path=self._calibration_contract_path,
        )
        publish_latest_path(self._output_root, self._directory)
        self._exported = True
        self._complete_pub.publish(String(data=str(self._directory)))
        self.get_logger().info(f"map products exported to {self._directory}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MapProductsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
