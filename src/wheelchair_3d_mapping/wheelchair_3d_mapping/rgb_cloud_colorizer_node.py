"""Color a point cloud with the main camera image (fallback colorizer).

Use this only when the LIVO backend does NOT already publish an RGB cloud.
Points are transformed into the camera optical frame using a calibrated
extrinsic (from YAML, never hardcoded), projected with the camera_info
intrinsics, and colored by sampling the image. Missing camera_info, image,
or extrinsic only produces a warning - the node never crashes.
"""
import time

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException

from wheelchair_3d_mapping import cloud_utils


class RgbCloudColorizerNode(Node):
    def __init__(self):
        super().__init__("rgb_cloud_colorizer_node")
        self.declare_parameter("input_cloud_topic", "/points_merged")
        self.declare_parameter("image_topic", "/main_camera/image_raw")
        self.declare_parameter("camera_info_topic", "/main_camera/camera_info")
        self.declare_parameter("output_topic", "/rgb_cloud_map")
        self.declare_parameter("lidar_frame", "base_link")
        # T_camera_optical_from_lidar. MUST be calibrated (camera<->lidar).
        self.declare_parameter("cam_lidar_translation", [0.0, 0.0, 0.0])
        self.declare_parameter("cam_lidar_quaternion", [0.0, 0.0, 0.0, 1.0])
        self.declare_parameter("default_gray", 128)
        self.declare_parameter("tf_timeout_sec", 0.1)

        self.lidar_frame = self.get_parameter("lidar_frame").value
        t = list(self.get_parameter("cam_lidar_translation").value)
        q = list(self.get_parameter("cam_lidar_quaternion").value)
        self.R = cloud_utils.quat_to_rotation(q[0], q[1], q[2], q[3])
        self.t = np.array(t, dtype=np.float64)
        self.gray = int(self.get_parameter("default_gray").value)
        self.tf_timeout = float(self.get_parameter("tf_timeout_sec").value)
        if np.allclose(self.t, 0.0) and np.allclose(q, [0.0, 0.0, 0.0, 1.0]):
            self.get_logger().warning(
                "cam_lidar extrinsic is identity - colors WILL be misaligned. "
                "Set cam_lidar_translation/quaternion from camera-lidar calibration.")

        self.image = None
        self.K = None
        self._warned = {}
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(Image, self.get_parameter("image_topic").value, self._on_image, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, self.get_parameter("camera_info_topic").value, self._on_info, 10)
        self.create_subscription(PointCloud2, self.get_parameter("input_cloud_topic").value, self._on_cloud, qos_profile_sensor_data)
        self.pub = self.create_publisher(PointCloud2, self.get_parameter("output_topic").value, qos_profile_sensor_data)
        self.get_logger().info("rgb_cloud_colorizer started")

    def _warn(self, key, msg, period=5.0):
        now = time.monotonic()
        if now - self._warned.get(key, 0.0) >= period:
            self.get_logger().warning(msg)
            self._warned[key] = now

    def _on_image(self, msg):
        rgb = cloud_utils.image_to_rgb(msg)
        if rgb is None:
            self._warn("img_decode", f"cannot decode image encoding '{msg.encoding}'")
            return
        self.image = rgb

    def _on_info(self, msg):
        k = msg.k
        if len(k) >= 9 and k[0] > 0 and k[4] > 0:
            self.K = (k[0], k[4], k[2], k[5])  # fx, fy, cx, cy

    def _on_cloud(self, msg):
        if self.K is None:
            self._warn("no_info", "no camera_info yet; cannot colorize (publish /main_camera/camera_info)")
            return
        if self.image is None:
            self._warn("no_image", "no image yet; cannot colorize")
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

        fx, fy, cx, cy = self.K
        h, w = self.image.shape[:2]
        pc = xyz_l @ self.R.T + self.t
        z = pc[:, 2]
        rgb = np.full((xyz.shape[0], 3), self.gray, dtype=np.uint8)
        front = z > 1e-3
        if np.any(front):
            u = np.round(fx * pc[front, 0] / z[front] + cx).astype(np.int64)
            v = np.round(fy * pc[front, 1] / z[front] + cy).astype(np.int64)
            inb = (u >= 0) & (u < w) & (v >= 0) & (v < h)
            idx = np.nonzero(front)[0][inb]
            rgb[idx] = self.image[v[inb], u[inb]]

        out = cloud_utils.make_xyzrgb_cloud(msg.header, xyz, rgb)
        self.pub.publish(out)

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
