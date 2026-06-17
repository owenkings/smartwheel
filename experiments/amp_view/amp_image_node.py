#!/usr/bin/env python3
"""Render the XT-M60 organized cloud as upper-computer-style PointCloud+Amp images.

The XT-M60 is a 160x60 flash-ToF *camera*. The upper-computer's pretty
"PointCloud+Amp" view is really the amplitude image colour-mapped (plus depth).
RViz can't mesh a point cloud into a surface, but it CAN show a dense 2D image.
This node reshapes the organized cloud back into 2D images and publishes them so
you see the same clean, dense "scan of what's in front" picture in RViz's Image
panel (or any image viewer) -- no SLAM, just the live frame.

Subscribes:
  /xtm60/left/points        (organized XYZI cloud, height=rows, width=cols)
Publishes:
  /xtm60/left/amp_image     (amplitude, JET colormap)   -- the "PointCloud+Amp" look
  /xtm60/left/depth_image   (range/depth, JET colormap)
  /xtm60/left/amp_mono      (amplitude, grayscale)      -- infrared-photo look
"""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, Image
from sensor_msgs_py import point_cloud2

try:
    import cv2
    HAVE_CV2 = True
except Exception:
    HAVE_CV2 = False


def jet_colormap(norm):
    """norm in [0,1] -> (N,3) uint8 RGB, jet-like. Pure numpy fallback."""
    x = np.clip(norm, 0.0, 1.0)
    r = np.clip(1.5 - np.abs(4 * x - 3), 0, 1)
    g = np.clip(1.5 - np.abs(4 * x - 2), 0, 1)
    b = np.clip(1.5 - np.abs(4 * x - 1), 0, 1)
    return (np.stack([r, g, b], axis=-1) * 255).astype(np.uint8)


class AmpImageNode(Node):
    def __init__(self):
        super().__init__("amp_image_node")
        self.declare_parameter("input_topic", "/xtm60/left/points")
        self.declare_parameter("amp_min", 0.0)
        self.declare_parameter("amp_max", 2039.0)   # SDK valid amplitude range
        self.declare_parameter("depth_min", 0.0)
        self.declare_parameter("depth_max", 8.0)    # m
        self.declare_parameter("upscale", 4)        # enlarge tiny 160x60 for viewing
        self.amp_min = float(self.get_parameter("amp_min").value)
        self.amp_max = float(self.get_parameter("amp_max").value)
        self.depth_min = float(self.get_parameter("depth_min").value)
        self.depth_max = float(self.get_parameter("depth_max").value)
        self.upscale = int(self.get_parameter("upscale").value)

        self.amp_pub = self.create_publisher(Image, "/amp_view/amp_image", 5)
        self.depth_pub = self.create_publisher(Image, "/amp_view/depth_image", 5)
        self.mono_pub = self.create_publisher(Image, "/amp_view/amp_mono", 5)
        # Republish the input cloud under a neutral topic so one RViz config
        # works for either radar (left or right).
        self.cloud_pub = self.create_publisher(PointCloud2, "/amp_view/cloud", qos_profile_sensor_data)
        self.sub = self.create_subscription(
            PointCloud2, self.get_parameter("input_topic").value, self._on_cloud, qos_profile_sensor_data)
        self._warned = False
        self.get_logger().info(
            f"amp_image_node: reshaping {self.get_parameter('input_topic').value} -> "
            f"amp/depth images (cv2={'yes' if HAVE_CV2 else 'no, numpy jet'})")

    def _on_cloud(self, msg: PointCloud2):
        # Republish under neutral topic for RViz (works for left or right radar).
        self.cloud_pub.publish(msg)
        h, w = msg.height, msg.width
        if h <= 1 or w <= 1:
            if not self._warned:
                self.get_logger().warn(
                    f"cloud is not organized (h={h}, w={w}); need organized_cloud:=true on the adapter")
                self._warned = True
            return
        # Read ALL points in order (keep NaNs as gaps).
        arr = point_cloud2.read_points(msg, field_names=("x", "y", "z", "intensity"), skip_nans=False)
        x = np.asarray(arr["x"]).reshape(h, w)
        y = np.asarray(arr["y"]).reshape(h, w)
        z = np.asarray(arr["z"]).reshape(h, w)
        amp = np.asarray(arr["intensity"]).reshape(h, w).astype(np.float32)
        depth = np.sqrt(x * x + y * y + z * z)
        valid = np.isfinite(depth)

        # --- amplitude image (the "PointCloud+Amp" look) ---
        an = (amp - self.amp_min) / max(1e-6, (self.amp_max - self.amp_min))
        an = np.clip(an, 0, 1)
        amp_rgb = self._colorize(an, valid)
        mono = (np.clip(an, 0, 1) * 255).astype(np.uint8)
        mono_rgb = np.stack([mono, mono, mono], axis=-1)
        mono_rgb[~valid] = 0

        # --- depth image ---
        dn = (depth - self.depth_min) / max(1e-6, (self.depth_max - self.depth_min))
        dn = np.clip(np.nan_to_num(dn, nan=0.0), 0, 1)
        depth_rgb = self._colorize(dn, valid)

        if self.upscale > 1:
            amp_rgb = self._upscale(amp_rgb)
            depth_rgb = self._upscale(depth_rgb)
            mono_rgb = self._upscale(mono_rgb)

        stamp = msg.header.stamp
        self.amp_pub.publish(self._to_img(amp_rgb, stamp, msg.header.frame_id))
        self.depth_pub.publish(self._to_img(depth_rgb, stamp, msg.header.frame_id))
        self.mono_pub.publish(self._to_img(mono_rgb, stamp, msg.header.frame_id))

    def _colorize(self, norm, valid):
        if HAVE_CV2:
            g = (np.clip(norm, 0, 1) * 255).astype(np.uint8)
            rgb = cv2.applyColorMap(g, cv2.COLORMAP_JET)[:, :, ::-1].copy()  # BGR->RGB
        else:
            rgb = jet_colormap(norm)
        rgb[~valid] = 0  # invalid pixels black
        return rgb

    def _upscale(self, rgb):
        if HAVE_CV2:
            return cv2.resize(rgb, (rgb.shape[1] * self.upscale, rgb.shape[0] * self.upscale),
                              interpolation=cv2.INTER_NEAREST)
        return np.repeat(np.repeat(rgb, self.upscale, axis=0), self.upscale, axis=1)

    @staticmethod
    def _to_img(rgb, stamp, frame_id):
        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.height, msg.width = rgb.shape[0], rgb.shape[1]
        msg.encoding = "rgb8"
        msg.is_bigendian = 0
        msg.step = rgb.shape[1] * 3
        msg.data = np.ascontiguousarray(rgb).tobytes()
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = AmpImageNode()
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
