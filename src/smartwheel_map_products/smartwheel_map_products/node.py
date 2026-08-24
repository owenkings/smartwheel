import json
import math
import time
from datetime import datetime
from pathlib import Path

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from std_msgs.msg import Bool, Header, String
from std_srvs.srv import Trigger

from smartwheel_map_products.accumulator import VoxelAccumulator
from smartwheel_map_products.colorizer import CameraFrame, colorize_points
from smartwheel_map_products.metrics import build_trajectory_evaluation, evaluate_trajectory
from smartwheel_map_products.occupancy import raycast_occupancy
from smartwheel_map_products.writers import export_map_bundle
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


class MapProductsNode(Node):
    def __init__(self) -> None:
        super().__init__("smartwheel_map_products")
        self.declare_parameter("map_name", "stage_a_sim")
        self.declare_parameter("output_root", "maps/versions")
        self.declare_parameter("version_directory", "")
        self.declare_parameter("hardware_profile_path", "")
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
        self.declare_parameter("cloud_topic", "/lidar/merged/points")
        self.declare_parameter("odom_topic", "/odom/fused")
        self.declare_parameter("validation_stage", "A_SYNTHETIC")
        self.declare_parameter("hardware_validated", False)
        self.declare_parameter("mock_lio", True)
        self.declare_parameter("maximum_evaluation_time_delta_sec", 0.1)
        self.declare_parameter("maximum_pose_time_delta_sec", 0.15)
        camera_defaults = {
            "front": ([0.40, 0.0, 0.82], [0.0, 0.0, 0.0]),
            "left": ([0.0, 0.28, 0.80], [0.0, 0.0, math.pi / 2.0]),
            "right": ([0.0, -0.28, 0.80], [0.0, 0.0, -math.pi / 2.0]),
            "rear": ([-0.28, 0.0, 0.78], [0.0, 0.0, math.pi]),
        }
        for name, (xyz, rpy) in camera_defaults.items():
            self.declare_parameter(f"{name}_camera_xyz", xyz)
            self.declare_parameter(f"{name}_camera_rpy", rpy)

        root = Path(str(self.get_parameter("output_root").value)).expanduser().resolve()
        explicit = str(self.get_parameter("version_directory").value).strip()
        if explicit:
            self._directory = Path(explicit).expanduser().resolve()
        else:
            stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            self._directory = root / f"{self.get_parameter('map_name').value}_{stamp}"
        self._directory.mkdir(parents=True, exist_ok=True)
        root.mkdir(parents=True, exist_ok=True)
        (root / "latest_path.txt").write_text(str(self._directory) + "\n", encoding="utf-8")

        self._resolution = float(self.get_parameter("resolution_m").value)
        self._voxel = float(self.get_parameter("voxel_size_m").value)
        self._min_z = float(self.get_parameter("min_obstacle_z").value)
        self._max_z = float(self.get_parameter("max_obstacle_z").value)
        self._export_delay = float(self.get_parameter("export_delay_sec").value)
        self._color_enabled = bool(self.get_parameter("enable_offline_colorization").value)
        self._backend_cloud_topic = str(self.get_parameter("backend_cloud_topic").value).strip()
        self._require_backend_cloud = bool(self.get_parameter("require_backend_cloud").value)
        self._cloud_topic = str(self.get_parameter("cloud_topic").value).strip()
        self._odom_topic = str(self.get_parameter("odom_topic").value).strip()
        if not self._cloud_topic or not self._odom_topic:
            raise ValueError("cloud_topic and odom_topic must be non-empty")
        self._validation_stage = str(self.get_parameter("validation_stage").value).strip()
        self._hardware_validated = bool(self.get_parameter("hardware_validated").value)
        self._mock_lio = bool(self.get_parameter("mock_lio").value)
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
        self._accumulator = VoxelAccumulator(
            self._voxel,
            int(self.get_parameter("maximum_accumulated_points").value),
        )
        self._accumulation_failure = ""
        self._poses = []
        self._ground_truth = []
        self._backend_points = None
        self._backend_frame = ""
        self._camera_info = {}
        self._camera_frames: list[CameraFrame] = []
        self._completed_at = None
        self._exported = False
        self._rejected_pose_associations = 0
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
        if self._exported:
            return
        stamp = _stamp_seconds(message.header.stamp)
        pose = message.pose.pose
        self._poses.append((stamp, pose.position.x, pose.position.y, _yaw_from_quaternion(pose.orientation)))

    def _on_ground_truth(self, message: Odometry) -> None:
        if self._exported:
            return
        pose = message.pose.pose
        self._ground_truth.append(
            (_stamp_seconds(message.header.stamp), pose.position.x, pose.position.y, _yaw_from_quaternion(pose.orientation))
        )

    def _on_backend_cloud(self, message: PointCloud2) -> None:
        if self._exported:
            return
        try:
            points, _ = pointcloud2_to_xyz(message)
        except ValueError as exc:
            self.get_logger().error(f"backend cloud rejected: {exc}")
            return
        if points.size == 0:
            return
        self._backend_points = points
        self._backend_frame = message.header.frame_id

    def _nearest_pose(self, stamp: float):
        if not self._poses:
            return None
        pose = min(self._poses, key=lambda candidate: abs(candidate[0] - stamp))
        return pose if abs(pose[0] - stamp) <= self._maximum_pose_delta else None

    def _on_cloud(self, message: PointCloud2) -> None:
        if self._exported or self._accumulation_failure:
            return
        pose = self._nearest_pose(_stamp_seconds(message.header.stamp))
        if pose is None:
            self._rejected_pose_associations += 1
            return
        try:
            points, intensity = pointcloud2_to_xyz(message)
        except ValueError as exc:
            self.get_logger().error(str(exc))
            return
        map_from_base = transform_matrix([pose[1], pose[2], 0.0], [0.0, 0.0, pose[3]])
        world = apply_transform(points, map_from_base)
        origins = np.tile(np.array([pose[1], pose[2], 0.0]), (world.shape[0], 1))
        try:
            self._accumulator.add(world, origins, intensity)
        except (OverflowError, ValueError) as exc:
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
            self._completed_at = time.monotonic()

    def _on_export_request(self, _request, response):
        if self._exported:
            self._complete_pub.publish(String(data=str(self._directory)))
            response.success = True
            response.message = f"already exported: {self._directory}"
            return response
        products = self._current_products()
        if products is None:
            response.success = False
            response.message = self._missing_products_reason()
            self._failure_pub.publish(String(data=response.message))
            return response
        self._publish(products[0], products[2], products[4], products[5])
        self._export(*products)
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
        else:
            geometry = local_points
            geometry_source = "local_odometry_accumulator"
        return (
            geometry,
            origins,
            grid,
            geometry_source,
            local_points,
            local_intensity,
        )

    def _missing_products_reason(self) -> str:
        if self._accumulation_failure:
            return self._accumulation_failure
        if len(self._accumulator) == 0:
            return "no merged point cloud has been received"
        if self._require_backend_cloud and self._backend_points is None:
            return f"required backend cloud has not been received: {self._backend_cloud_topic}"
        if self._require_backend_cloud and self._backend_frame != "map":
            return f"required backend cloud frame must be map, got: {self._backend_frame or '<empty>'}"
        return "map products are unavailable"

    def _tick(self) -> None:
        if (
            self._completed_at is not None
            and not self._exported
            and time.monotonic() - self._completed_at >= self._export_delay
        ):
            products = self._current_products()
            if products is not None:
                self._publish(products[0], products[2], products[4], products[5])
                self._export(*products)
            else:
                self._failure_pub.publish(String(data=self._missing_products_reason()))

    def _publish(self, points, grid, intensity_points, intensity) -> None:
        header = Header(stamp=self.get_clock().now().to_msg(), frame_id="map")
        self._cloud_pub.publish(pointcloud2_from_xyz(header, points))
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
    ) -> None:
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
        amplitude_metrics = {
            "pointcloud_amp_preserved": intensity is not None,
            "pointcloud_amp_point_count": 0 if intensity is None else int(intensity.shape[0]),
            "pointcloud_amp_min": None if intensity is None else float(np.min(intensity)),
            "pointcloud_amp_median": None if intensity is None else float(np.median(intensity)),
            "pointcloud_amp_p95": None if intensity is None else float(np.percentile(intensity, 95.0)),
            "pointcloud_amp_max": None if intensity is None else float(np.max(intensity)),
        }
        quality = {
            "stage": self._validation_stage,
            "point_count": int(points.shape[0]),
            "colored_point_count": colored_count,
            "occupied_cells": occupied,
            "free_cells": free,
            "unknown_cells": unknown,
            "trajectory_pose_count": len(self._poses),
            "ground_truth_pose_count": len(self._ground_truth),
            "rejected_pose_associations": self._rejected_pose_associations,
            "trajectory_evaluation": trajectory_evaluation,
            "geometry_source": geometry_source,
            "raycast_occupancy_source": "local_odometry_accumulator",
            **trajectory_metrics,
            "map_bounds_min_m": minimum.tolist(),
            "map_bounds_max_m": maximum.tolist(),
            "tf_conflict_runtime_check": "NOT_PERFORMED_BY_EXPORTER",
            "hardware_validated": self._hardware_validated,
            **amplitude_metrics,
        }
        profile = {
            "mapping_backend": str(self.get_parameter("mapping_backend").value),
            "state_mode": str(self.get_parameter("state_mode").value),
            "lidar_mode": str(self.get_parameter("lidar_mode").value),
            "resolution_m": self._resolution,
            "voxel_size_m": self._voxel,
            "cloud_topic": self._cloud_topic,
            "odom_topic": self._odom_topic,
            "mock_lio": self._mock_lio,
        }
        export_map_bundle(
            self._directory,
            points,
            grid,
            self._poses,
            colors,
            str(self.get_parameter("hardware_profile_path").value),
            profile,
            self._bag_path,
            quality,
            intensity_points=intensity_points,
            intensity=intensity,
        )
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
