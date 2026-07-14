import math
import time

import numpy as np
import rclpy
from builtin_interfaces.msg import Time
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, Imu, PointCloud2
from smartwheel_interfaces.msg import HardwareStatus, SimMotion, WheelEncoder
from std_msgs.msg import Bool, Header

from smartwheel_sensor_api import pointcloud2_from_xyz
from smartwheel_sim.environment import IndoorScene, LidarModel
from smartwheel_sim.trajectory import ClosedLoopTrajectory


CAMERAS = ("front", "left", "right", "rear")


def _time_message(seconds: float) -> Time:
    total_ns = max(0, int(round(seconds * 1_000_000_000)))
    return Time(sec=total_ns // 1_000_000_000, nanosec=total_ns % 1_000_000_000)


def _angle_delta(current: float, previous: float) -> float:
    return math.atan2(math.sin(current - previous), math.cos(current - previous))


class IndoorSimNode(Node):
    def __init__(self) -> None:
        super().__init__("smartwheel_indoor_sim")
        self.declare_parameter("seed", 20260714)
        self.declare_parameter("duration_sec", 12.0)
        self.declare_parameter("sim_step_sec", 0.02)
        self.declare_parameter("playback_rate", 4.0)
        self.declare_parameter("publisher_discovery_warmup_sec", 1.5)
        self.declare_parameter("lidar_rate_hz", 5.0)
        self.declare_parameter("wheel_rate_hz", 20.0)
        self.declare_parameter("camera_rate_hz", 5.0)
        self.declare_parameter("left_time_offset_sec", 0.0)
        self.declare_parameter("right_time_offset_sec", 0.008)
        self.declare_parameter("left_drop_every_n", 37)
        self.declare_parameter("right_drop_every_n", 43)
        self.declare_parameter("camera_drop_every_n", 31)
        self.declare_parameter("imu_drop_every_n", 0)
        self.declare_parameter("wheel_drop_every_n", 0)
        self.declare_parameter("occlusion_every_n", 19)
        self.declare_parameter("blur_every_n", 17)
        self.declare_parameter("imu_time_offset_sec", 0.0)
        self.declare_parameter("imu_gyro_noise_stddev_rps", 0.002)
        self.declare_parameter("imu_gyro_bias_rps", 0.0)
        self.declare_parameter("imu_accel_noise_stddev_mps2", 0.02)
        self.declare_parameter("imu_accel_bias_xyz", [0.0, 0.0, 0.0])
        self.declare_parameter("wheel_common_scale", 1.0)
        self.declare_parameter("wheel_left_scale", 1.0)
        self.declare_parameter("wheel_right_scale", 1.0)
        self.declare_parameter("wheel_slip_ratio", 0.0)
        self.declare_parameter("wheel_slip_start_fraction", 0.35)
        self.declare_parameter("wheel_slip_end_fraction", 0.45)
        self.declare_parameter("lidar_noise_stddev_m", 0.008)
        self.declare_parameter("lidar_horizontal_fov_deg", 120.0)
        self.declare_parameter("lidar_vertical_fov_deg", 45.0)
        self.declare_parameter("wheel_radius_m", 0.16)
        self.declare_parameter("track_width_m", 0.58)
        self.declare_parameter("encoder_cpr", 2048)
        self.declare_parameter("gear_ratio", 1.0)
        self.declare_parameter("left_lidar_xyz", [0.32, 0.18, 0.72])
        self.declare_parameter("left_lidar_rpy", [0.0, math.radians(-4.0), math.radians(28.0)])
        self.declare_parameter("right_lidar_xyz", [0.32, -0.18, 0.72])
        self.declare_parameter("right_lidar_rpy", [0.0, math.radians(-4.0), math.radians(-28.0)])
        self.declare_parameter("enable_cameras", True)
        self.declare_parameter("camera_width", 160)
        self.declare_parameter("camera_height", 120)

        self._seed = int(self.get_parameter("seed").value)
        self._duration = float(self.get_parameter("duration_sec").value)
        self._step = float(self.get_parameter("sim_step_sec").value)
        playback_rate = float(self.get_parameter("playback_rate").value)
        warmup_sec = float(self.get_parameter("publisher_discovery_warmup_sec").value)
        if self._duration <= 0.0 or self._step <= 0.0 or playback_rate <= 0.0:
            raise ValueError("simulation duration, step, and playback rate must be positive")
        if warmup_sec < 0.0:
            raise ValueError("publisher discovery warmup must be non-negative")
        self._scene = IndoorScene()
        self._trajectory = ClosedLoopTrajectory()
        self._left_model = LidarModel(
            tuple(self.get_parameter("left_lidar_xyz").value),
            tuple(self.get_parameter("left_lidar_rpy").value),
            horizontal_fov_deg=float(self.get_parameter("lidar_horizontal_fov_deg").value),
            vertical_fov_deg=float(self.get_parameter("lidar_vertical_fov_deg").value),
            noise_stddev_m=float(self.get_parameter("lidar_noise_stddev_m").value),
        )
        self._right_model = LidarModel(
            tuple(self.get_parameter("right_lidar_xyz").value),
            tuple(self.get_parameter("right_lidar_rpy").value),
            horizontal_fov_deg=float(self.get_parameter("lidar_horizontal_fov_deg").value),
            vertical_fov_deg=float(self.get_parameter("lidar_vertical_fov_deg").value),
            noise_stddev_m=float(self.get_parameter("lidar_noise_stddev_m").value),
        )
        self._lidar_period = max(1, int(round(1.0 / (float(self.get_parameter("lidar_rate_hz").value) * self._step))))
        self._wheel_period = max(1, int(round(1.0 / (float(self.get_parameter("wheel_rate_hz").value) * self._step))))
        self._camera_period = max(1, int(round(1.0 / (float(self.get_parameter("camera_rate_hz").value) * self._step))))
        self._left_offset = float(self.get_parameter("left_time_offset_sec").value)
        self._right_offset = float(self.get_parameter("right_time_offset_sec").value)
        self._left_drop = int(self.get_parameter("left_drop_every_n").value)
        self._right_drop = int(self.get_parameter("right_drop_every_n").value)
        self._camera_drop = int(self.get_parameter("camera_drop_every_n").value)
        self._imu_drop = int(self.get_parameter("imu_drop_every_n").value)
        self._wheel_drop = int(self.get_parameter("wheel_drop_every_n").value)
        self._occlusion_every = int(self.get_parameter("occlusion_every_n").value)
        self._blur_every = int(self.get_parameter("blur_every_n").value)
        self._imu_offset = float(self.get_parameter("imu_time_offset_sec").value)
        self._imu_gyro_noise = float(self.get_parameter("imu_gyro_noise_stddev_rps").value)
        self._imu_gyro_bias = float(self.get_parameter("imu_gyro_bias_rps").value)
        self._imu_accel_noise = float(self.get_parameter("imu_accel_noise_stddev_mps2").value)
        self._imu_accel_bias = np.asarray(self.get_parameter("imu_accel_bias_xyz").value, dtype=np.float64)
        self._wheel_common_scale = float(self.get_parameter("wheel_common_scale").value)
        self._wheel_left_scale = float(self.get_parameter("wheel_left_scale").value)
        self._wheel_right_scale = float(self.get_parameter("wheel_right_scale").value)
        self._wheel_slip_ratio = float(self.get_parameter("wheel_slip_ratio").value)
        self._wheel_slip_start = float(self.get_parameter("wheel_slip_start_fraction").value)
        self._wheel_slip_end = float(self.get_parameter("wheel_slip_end_fraction").value)
        self._wheel_radius = float(self.get_parameter("wheel_radius_m").value)
        self._track = float(self.get_parameter("track_width_m").value)
        self._cpr = int(self.get_parameter("encoder_cpr").value)
        self._gear = float(self.get_parameter("gear_ratio").value)
        self._enable_cameras = bool(self.get_parameter("enable_cameras").value)
        self._camera_width = int(self.get_parameter("camera_width").value)
        self._camera_height = int(self.get_parameter("camera_height").value)

        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._left_pub = self.create_publisher(PointCloud2, "/lidar/left/points_raw", qos_profile_sensor_data)
        self._right_pub = self.create_publisher(PointCloud2, "/lidar/right/points_raw", qos_profile_sensor_data)
        self._imu_pub = self.create_publisher(Imu, "/imu/data_raw", qos_profile_sensor_data)
        self._encoder_pub = self.create_publisher(WheelEncoder, "/wheel/encoder_counts", 20)
        self._ground_truth_pub = self.create_publisher(Odometry, "/sim/ground_truth/odom", 20)
        self._mock_lio_motion_pub = self.create_publisher(SimMotion, "/sim/mock_lio/motion", 20)
        self._completed_pub = self.create_publisher(Bool, "/sim/completed", latched)
        self._hardware_pub = self.create_publisher(HardwareStatus, "/hardware/status", latched)
        self._image_pubs = {}
        self._info_pubs = {}
        for name in CAMERAS:
            self._image_pubs[name] = self.create_publisher(
                Image, f"/camera/{name}/image_raw", qos_profile_sensor_data
            )
            self._info_pubs[name] = self.create_publisher(
                CameraInfo, f"/camera/{name}/camera_info", latched
            )

        self._index = 0
        self._sim_time = 0.0
        self._previous_pose = self._trajectory.sample(0.0, self._duration)
        self._left_distance = 0.0
        self._right_distance = 0.0
        self._pending_distance = 0.0
        self._pending_yaw = 0.0
        self._left_count = 0
        self._right_count = 0
        self._completed = False
        self._mock_lio_distance = 0.0
        self._mock_lio_yaw = 0.0
        self._mock_lio_sequence = 0
        self._start_after = time.monotonic() + warmup_sec
        self._publish_hardware_status()
        self._timer = self.create_timer(self._step / playback_rate, self._tick)

    def _tick(self) -> None:
        if self._completed or time.monotonic() < self._start_after:
            return
        pose = self._trajectory.sample(self._sim_time, self._duration)
        stamp_sec = self.get_clock().now().nanoseconds * 1e-9
        stamp = _time_message(stamp_sec)
        distance = math.hypot(pose.x - self._previous_pose.x, pose.y - self._previous_pose.y)
        dyaw = _angle_delta(pose.yaw, self._previous_pose.yaw)
        yaw_rate = dyaw / self._step if self._index > 0 else 0.0
        self._pending_distance += distance
        self._pending_yaw += dyaw
        self._publish_ground_truth(pose, stamp, distance / self._step, yaw_rate)
        self._publish_mock_lio_motion(distance, dyaw, stamp)
        if not (self._imu_drop > 0 and self._index % self._imu_drop == 0 and self._index > 0):
            self._publish_imu(pose, _time_message(stamp_sec + self._imu_offset), yaw_rate)
        if self._index % self._wheel_period == 0:
            wheel_frame = self._index // self._wheel_period
            if not (self._wheel_drop > 0 and wheel_frame % self._wheel_drop == 0 and wheel_frame > 0):
                self._publish_encoder(self._pending_distance, self._pending_yaw, stamp)
            self._pending_distance = 0.0
            self._pending_yaw = 0.0
        if self._index % self._lidar_period == 0:
            self._publish_lidars(pose, stamp_sec)
        if self._enable_cameras and self._index % self._camera_period == 0:
            self._publish_cameras(stamp)
        self._previous_pose = pose
        self._index += 1
        self._sim_time = min(self._duration, self._sim_time + self._step)
        if self._sim_time >= self._duration:
            final_pose = self._trajectory.sample(self._duration, self._duration)
            final_stamp = self.get_clock().now().to_msg()
            final_distance = math.hypot(final_pose.x - pose.x, final_pose.y - pose.y)
            final_dyaw = _angle_delta(final_pose.yaw, pose.yaw)
            self._publish_ground_truth(final_pose, final_stamp, 0.0, 0.0)
            self._publish_mock_lio_motion(final_distance, final_dyaw, final_stamp)
            self._completed = True
            self._completed_pub.publish(Bool(data=True))
            self.get_logger().info("deterministic closed-loop simulation completed")

    def _publish_ground_truth(self, pose, stamp, linear: float, angular: float) -> None:
        message = Odometry()
        message.header.stamp = stamp
        message.header.frame_id = "map"
        message.child_frame_id = "base_link_gt"
        message.pose.pose.position.x = pose.x
        message.pose.pose.position.y = pose.y
        message.pose.pose.orientation.z = math.sin(pose.yaw * 0.5)
        message.pose.pose.orientation.w = math.cos(pose.yaw * 0.5)
        message.twist.twist.linear.x = linear
        message.twist.twist.angular.z = angular
        self._ground_truth_pub.publish(message)

    def _publish_imu(self, pose, stamp, yaw_rate: float) -> None:
        rng = np.random.default_rng(self._seed + self._index * 17)
        message = Imu()
        message.header.stamp = stamp
        message.header.frame_id = "imu_link"
        message.orientation.z = math.sin(pose.yaw * 0.5)
        message.orientation.w = math.cos(pose.yaw * 0.5)
        message.angular_velocity.z = (
            yaw_rate + self._imu_gyro_bias + float(rng.normal(0.0, self._imu_gyro_noise))
        )
        acceleration = self._imu_accel_bias + rng.normal(0.0, self._imu_accel_noise, 3)
        acceleration[2] += 9.80665
        message.linear_acceleration.x = float(acceleration[0])
        message.linear_acceleration.y = float(acceleration[1])
        message.linear_acceleration.z = float(acceleration[2])
        message.orientation_covariance[0] = 0.0025
        message.angular_velocity_covariance[0] = 0.0004
        message.linear_acceleration_covariance[0] = 0.0025
        self._imu_pub.publish(message)

    def _publish_encoder(self, centre_distance: float, dyaw: float, stamp) -> None:
        left = centre_distance - dyaw * self._track * 0.5
        right = centre_distance + dyaw * self._track * 0.5
        left *= self._wheel_common_scale * self._wheel_left_scale
        right *= self._wheel_common_scale * self._wheel_right_scale
        phase = self._sim_time / self._duration
        if self._wheel_slip_start <= phase <= self._wheel_slip_end:
            left *= 1.0 + self._wheel_slip_ratio
        self._left_distance += left
        self._right_distance += right
        counts_per_metre = self._cpr * self._gear / (2.0 * math.pi * self._wheel_radius)
        self._left_count = int(round(self._left_distance * counts_per_metre))
        self._right_count = int(round(self._right_distance * counts_per_metre))
        message = WheelEncoder()
        message.stamp = stamp
        message.left_count = self._left_count
        message.right_count = self._right_count
        message.valid = True
        message.source = "deterministic_stage_a_simulator"
        self._encoder_pub.publish(message)

    def _publish_lidars(self, pose, stamp_sec: float) -> None:
        lidar_frame = self._index // self._lidar_period
        occluded = self._occlusion_every > 0 and lidar_frame % self._occlusion_every == 0 and lidar_frame > 0
        if not (self._left_drop > 0 and lidar_frame % self._left_drop == 0 and lidar_frame > 0):
            points, intensity = self._left_model.observe(
                self._scene.points, pose, self._seed + 11, lidar_frame, occluded=occluded
            )
            header = Header(stamp=_time_message(stamp_sec + self._left_offset), frame_id="xtm60_left_link")
            self._left_pub.publish(pointcloud2_from_xyz(header, points, intensity))
        if not (self._right_drop > 0 and lidar_frame % self._right_drop == 0 and lidar_frame > 0):
            points, intensity = self._right_model.observe(
                self._scene.points, pose, self._seed + 29, lidar_frame, occluded=False
            )
            header = Header(stamp=_time_message(stamp_sec + self._right_offset), frame_id="xtm60_right_link")
            self._right_pub.publish(pointcloud2_from_xyz(header, points, intensity))

    def _publish_mock_lio_motion(self, distance: float, dyaw: float, stamp) -> None:
        self._mock_lio_distance += distance
        self._mock_lio_yaw += dyaw
        message = SimMotion()
        message.stamp = stamp
        message.sequence = self._mock_lio_sequence
        message.cumulative_distance_m = self._mock_lio_distance
        message.cumulative_yaw_rad = self._mock_lio_yaw
        self._mock_lio_motion_pub.publish(message)
        self._mock_lio_sequence += 1

    def _publish_cameras(self, stamp) -> None:
        camera_frame = self._index // self._camera_period
        if self._camera_drop > 0 and camera_frame % self._camera_drop == 0 and camera_frame > 0:
            return
        x = np.arange(self._camera_width, dtype=np.uint16)[None, :]
        y = np.arange(self._camera_height, dtype=np.uint16)[:, None]
        blurred = self._blur_every > 0 and camera_frame % self._blur_every == 0 and camera_frame > 0
        for index, name in enumerate(CAMERAS):
            image = np.empty((self._camera_height, self._camera_width, 3), dtype=np.uint8)
            image[..., 0] = (x + index * 47 + camera_frame * 2) % 255
            image[..., 1] = (y + index * 61 + camera_frame) % 255
            image[..., 2] = (x // 2 + y // 2 + index * 31) % 255
            if blurred:
                image[:] = image.mean(axis=(0, 1)).astype(np.uint8)
            message = Image()
            message.header.stamp = stamp
            message.header.frame_id = f"camera_{name}_link"
            message.height = self._camera_height
            message.width = self._camera_width
            message.encoding = "rgb8"
            message.step = self._camera_width * 3
            message.data = image.tobytes()
            self._image_pubs[name].publish(message)
            info = CameraInfo()
            info.header = message.header
            info.height = self._camera_height
            info.width = self._camera_width
            focal = float(self._camera_width) * 0.8
            info.k = [focal, 0.0, self._camera_width / 2.0, 0.0, focal, self._camera_height / 2.0, 0.0, 0.0, 1.0]
            info.p = [focal, 0.0, self._camera_width / 2.0, 0.0, 0.0, focal, self._camera_height / 2.0, 0.0, 0.0, 0.0, 1.0, 0.0]
            info.distortion_model = "plumb_bob"
            info.d = [0.0] * 5
            self._info_pubs[name].publish(info)

    def _publish_hardware_status(self) -> None:
        message = HardwareStatus()
        message.stamp = self.get_clock().now().to_msg()
        message.hardware_enabled = False
        message.lidar_left_available = True
        message.lidar_right_available = True
        message.imu_available = True
        message.wheel_available = True
        message.cameras_available = self._enable_cameras
        message.mode = "mock"
        message.detail = "deterministic synthetic data; no physical interface opened"
        self._hardware_pub.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = IndoorSimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
