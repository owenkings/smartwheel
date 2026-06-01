"""Color a point cloud using one or two cameras (fallback colorizer).

Use this when the LIVO backend does NOT already publish an RGB cloud. Each
camera has its own camera_info (intrinsics) and a calibrated
T_camera_optical_from_lidar extrinsic (from YAML, never hardcoded). The cloud
is colored by the main camera first; points the main camera does not see are
filled from the optional aux camera. Output is a single /rgb_cloud_map.

Design note: only ONE camera feeds the SLAM/LIVO estimator, but BOTH forward
cameras can color the map here for wider color coverage. Missing image /
camera_info / extrinsic only warns - the node never crashes.
"""
import time

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException

from wheelchair_3d_mapping import cloud_utils


class _Cam:
    """Per-camera state: extrinsic (R,t) + latest image/intrinsics."""

    def __init__(self, name, translation, quaternion):
        self.name = name
        self.R = cloud_utils.quat_to_rotation(quaternion[0], quaternion[1], quaternion[2], quaternion[3])
        self.t = np.array(translation, dtype=np.float64)
        self.identity = np.allclose(self.t, 0.0) and np.allclose(quaternion, [0.0, 0.0, 0.0, 1.0])
        self.image = None
        self.image_stamp = None
        self.K = None

    def ready(self):
        return self.image is not None and self.K is not None


class RgbCloudColorizerNode(Node):
    def __init__(self):
        super().__init__("rgb_cloud_colorizer_node")
        self.declare_parameter("input_cloud_topic", "/points_merged")
        self.declare_parameter("output_topic", "/rgb_cloud_map")
        self.declare_parameter("lidar_frame", "base_link")
        self.declare_parameter("default_gray", 128)
        self.declare_parameter("tf_timeout_sec", 0.1)
        self.declare_parameter("max_image_age_sec", 0.15)
        # Main camera (also the one that feeds LIVO).
        self.declare_parameter("image_topic", "/main_camera/image_raw")
        self.declare_parameter("camera_info_topic", "/main_camera/camera_info")
        self.declare_parameter("cam_lidar_translation", [0.0, 0.0, 0.0])
        self.declare_parameter("cam_lidar_quaternion", [0.0, 0.0, 0.0, 1.0])
        # Optional aux camera (colorization only). Empty image topic disables it.
        self.declare_parameter("aux_image_topic", "")
        self.declare_parameter("aux_camera_info_topic", "")
        self.declare_parameter("aux_cam_lidar_translation", [0.0, 0.0, 0.0])
        self.declare_parameter("aux_cam_lidar_quaternion", [0.0, 0.0, 0.0, 1.0])

        self.lidar_frame = self.get_parameter("lidar_frame").value
        self.gray = int(self.get_parameter("default_gray").value)
        self.tf_timeout = float(self.get_parameter("tf_timeout_sec").value)
        self.max_image_age = float(self.get_parameter("max_image_age_sec").value)
        self._warned = {}
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.cameras = []
        self._add_camera("main", "image_topic", "camera_info_topic",
                         "cam_lidar_translation", "cam_lidar_quaternion")
        if str(self.get_parameter("aux_image_topic").value).strip():
            self._add_camera("aux", "aux_image_topic", "aux_camera_info_topic",
                             "aux_cam_lidar_translation", "aux_cam_lidar_quaternion")

        self.create_subscription(PointCloud2, self.get_parameter("input_cloud_topic").value,
                                 self._on_cloud, qos_profile_sensor_data)
        self.pub = self.create_publisher(PointCloud2, self.get_parameter("output_topic").value, qos_profile_sensor_data)
        self.get_logger().info(f"rgb_cloud_colorizer started with {len(self.cameras)} camera(s)")

    def _add_camera(self, name, img_param, info_param, t_param, q_param):
        cam = _Cam(name, list(self.get_parameter(t_param).value), list(self.get_parameter(q_param).value))
        if cam.identity:
            self.get_logger().warning(
                f"{name} cam_lidar extrinsic is identity - {name} colors WILL be misaligned. "
                f"Set {t_param}/{q_param} from camera-lidar calibration.")
        self.create_subscription(Image, self.get_parameter(img_param).value,
                                 lambda m, c=cam: self._on_image(m, c), qos_profile_sensor_data)
        self.create_subscription(CameraInfo, self.get_parameter(info_param).value,
                                 lambda m, c=cam: self._on_info(m, c), 10)
        self.cameras.append(cam)

    def _warn(self, key, msg, period=5.0):
        now = time.monotonic()
        if now - self._warned.get(key, 0.0) >= period:
            self.get_logger().warning(msg)
            self._warned[key] = now

    def _on_image(self, msg, cam):
        rgb = cloud_utils.image_to_rgb(msg)
        if rgb is None:
            self._warn(f"img_{cam.name}", f"{cam.name}: cannot decode image encoding '{msg.encoding}'")
            return
        cam.image = rgb
        cam.image_stamp = msg.header.stamp

    def _on_info(self, msg, cam):
        k = msg.k
        if len(k) >= 9 and k[0] > 0 and k[4] > 0:
            cam.K = (k[0], k[4], k[2], k[5])  # fx, fy, cx, cy

    def _on_cloud(self, msg):
        if not any(c.ready() for c in self.cameras):
            self._warn("no_cam", "no camera image+camera_info yet; cannot colorize")
            return
        xyz, _ = cloud_utils.read_xyz_intensity(msg)
        if xyz.shape[0] == 0:
            return

        xyz_l = xyz
        if msg.header.frame_id and msg.header.frame_id != self.lidar_frame:
            mat = self._lookup(msg.header.frame_id)
            if mat is None:
                self._warn("tf", f"no TF {self.lidar_frame}<-{msg.header.frame_id}; cannot colorize")
                return
            xyz_l = cloud_utils.apply_transform(xyz, mat)

        cloud_t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        rgb = np.full((xyz.shape[0], 3), self.gray, dtype=np.uint8)
        todo = np.ones(xyz.shape[0], dtype=bool)  # points still needing color
        for cam in self.cameras:  # main first, then aux fills the rest
            self._color_into(cam, xyz_l, cloud_t, rgb, todo)

        self.pub.publish(cloud_utils.make_xyzrgb_cloud(msg.header, xyz, rgb))

    def _color_into(self, cam, xyz_l, cloud_t, rgb, todo):
        if not cam.ready() or not np.any(todo):
            return
        if cam.image_stamp is not None:
            it = cam.image_stamp.sec + cam.image_stamp.nanosec * 1e-9
            delta = abs(cloud_t - it)
            if self.max_image_age > 0.0 and delta > self.max_image_age:
                self._warn(
                    f"age_{cam.name}",
                    f"{cam.name}: image/cloud time delta {delta:.3f}s > "
                    f"{self.max_image_age:.3f}s; skipping",
                )
                return
        fx, fy, cx, cy = cam.K
        h, w = cam.image.shape[:2]
        pc = xyz_l @ cam.R.T + cam.t
        z = pc[:, 2]
        cand = todo & (z > 1e-3)
        if not np.any(cand):
            return
        ci = np.nonzero(cand)[0]
        u = np.round(fx * pc[ci, 0] / z[ci] + cx).astype(np.int64)
        v = np.round(fy * pc[ci, 1] / z[ci] + cy).astype(np.int64)
        inb = (u >= 0) & (u < w) & (v >= 0) & (v < h)
        idx = ci[inb]
        rgb[idx] = cam.image[v[inb], u[inb]]
        todo[idx] = False  # these points are now colored; aux only fills the rest

    def _lookup(self, source):
        try:
            tf = self.tf_buffer.lookup_transform(self.lidar_frame, source, rclpy.time.Time(),
                                                 timeout=rclpy.duration.Duration(seconds=self.tf_timeout))
            return cloud_utils.transform_to_matrix(tf)
        except (LookupException, ConnectivityException, ExtrapolationException):
            return None


def main(args=None):
    rclpy.init(args=args)
    node = RgbCloudColorizerNode()
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
