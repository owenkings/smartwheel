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

from smartwheel_map_products.colorizer import CameraFrame, colorize_points
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
        self.declare_parameter("min_obstacle_z", 0.1)
        self.declare_parameter("max_obstacle_z", 2.2)
        self.declare_parameter("export_delay_sec", 1.0)
        self.declare_parameter("enable_offline_colorization", True)
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
        self._camera_extrinsics = {
            name: transform_matrix(
                self.get_parameter(f"{name}_camera_xyz").value,
                self.get_parameter(f"{name}_camera_rpy").value,
            )
            for name in CAMERAS
        }
        self._points = []
        self._origins = []
        self._poses = []
        self._ground_truth = []
        self._camera_info = {}
        self._camera_frames: list[CameraFrame] = []
        self._completed_at = None
        self._exported = False

        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._cloud_pub = self.create_publisher(PointCloud2, "/map_cloud", latched)
        self._map_pub = self.create_publisher(OccupancyGrid, "/map", latched)
        self._complete_pub = self.create_publisher(String, "/map_export/completed", latched)
        self.create_subscription(PointCloud2, "/lidar/merged/points", self._on_cloud, qos_profile_sensor_data)
        self.create_subscription(Odometry, "/odom/fused", self._on_odom, 20)
        self.create_subscription(Odometry, "/sim/ground_truth/odom", self._on_ground_truth, 20)
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

    def _on_odom(self, message: Odometry) -> None:
        stamp = _stamp_seconds(message.header.stamp)
        pose = message.pose.pose
        self._poses.append((stamp, pose.position.x, pose.position.y, _yaw_from_quaternion(pose.orientation)))

    def _on_ground_truth(self, message: Odometry) -> None:
        pose = message.pose.pose
        self._ground_truth.append(
            (_stamp_seconds(message.header.stamp), pose.position.x, pose.position.y, _yaw_from_quaternion(pose.orientation))
        )

    def _nearest_pose(self, stamp: float):
        if not self._poses:
            return None
        return min(self._poses, key=lambda pose: abs(pose[0] - stamp))

    def _on_cloud(self, message: PointCloud2) -> None:
        pose = self._nearest_pose(_stamp_seconds(message.header.stamp))
        if pose is None:
            return
        try:
            points, _ = pointcloud2_to_xyz(message)
        except ValueError as exc:
            self.get_logger().error(str(exc))
            return
        map_from_base = transform_matrix([pose[1], pose[2], 0.0], [0.0, 0.0, pose[3]])
        world = apply_transform(points, map_from_base)
        self._points.append(world)
        self._origins.append(np.tile(np.array([pose[1], pose[2], 0.0]), (world.shape[0], 1)))

    def _on_camera_info(self, name: str, message: CameraInfo) -> None:
        if message.k[0] > 0.0 and message.k[4] > 0.0:
            self._camera_info[name] = np.asarray(message.k, dtype=np.float64).reshape(3, 3)

    def _on_image(self, name: str, message: Image) -> None:
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

    def _current_products(self):
        if not self._points:
            return None
        points = np.vstack(self._points)
        origins = np.vstack(self._origins)
        if self._voxel > 0.0:
            keys = np.floor(points / self._voxel).astype(np.int64)
            _, first = np.unique(keys, axis=0, return_index=True)
            first.sort()
            points = points[first]
            origins = origins[first]
        grid = raycast_occupancy(
            points,
            origins,
            resolution=self._resolution,
            min_obstacle_z=self._min_z,
            max_obstacle_z=self._max_z,
        )
        return points, origins, grid

    def _tick(self) -> None:
        products = self._current_products()
        if products is not None:
            self._publish(products[0], products[2])
        if (
            self._completed_at is not None
            and not self._exported
            and time.monotonic() - self._completed_at >= self._export_delay
            and products is not None
        ):
            self._export(*products)

    def _publish(self, points, grid) -> None:
        header = Header(stamp=self.get_clock().now().to_msg(), frame_id="map")
        self._cloud_pub.publish(pointcloud2_from_xyz(header, points))
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

    def _export(self, points, _origins, grid) -> None:
        colors = None
        colored_count = 0
        if self._color_enabled and self._camera_frames:
            colors, colored = colorize_points(points, self._camera_frames)
            colored_count = int(colored.sum())
        occupied = int(np.count_nonzero(grid.cells == 100))
        free = int(np.count_nonzero(grid.cells == 0))
        unknown = int(np.count_nonzero(grid.cells == -1))
        loop_error = None
        if len(self._poses) >= 2:
            loop_error = math.hypot(self._poses[-1][1] - self._poses[0][1], self._poses[-1][2] - self._poses[0][2])
        quality = {
            "stage": "A_SYNTHETIC",
            "point_count": int(points.shape[0]),
            "colored_point_count": colored_count,
            "occupied_cells": occupied,
            "free_cells": free,
            "unknown_cells": unknown,
            "trajectory_pose_count": len(self._poses),
            "ground_truth_pose_count": len(self._ground_truth),
            "loop_closure_position_error_m": loop_error,
            "tf_conflicts_detected": 0,
            "hardware_validated": False,
        }
        profile = {
            "mapping_backend": str(self.get_parameter("mapping_backend").value),
            "state_mode": str(self.get_parameter("state_mode").value),
            "lidar_mode": str(self.get_parameter("lidar_mode").value),
            "resolution_m": self._resolution,
            "voxel_size_m": self._voxel,
            "mock_lio": True,
        }
        export_map_bundle(
            self._directory,
            points,
            grid,
            self._poses,
            colors,
            str(self.get_parameter("hardware_profile_path").value),
            profile,
            str(self.get_parameter("bag_path").value),
            quality,
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

