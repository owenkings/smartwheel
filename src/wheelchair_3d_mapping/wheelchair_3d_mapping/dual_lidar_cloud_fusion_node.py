"""Fuse the two XT-M60 point clouds into a single /points_merged cloud.

Each incoming cloud is transformed into target_frame via TF, range/height
filtered and optionally voxel-downsampled. A timer merges the most recent
left/right results and publishes them. Missing TF or a missing lidar never
crashes the node: it warns and, if allow_single_lidar_fallback is true, keeps
publishing from whichever lidar is available.
"""
import json
import time
from typing import Optional

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header, String
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException

from wheelchair_3d_mapping import cloud_utils


class _LidarState:
    def __init__(self):
        self.xyz: Optional[np.ndarray] = None
        self.inten: Optional[np.ndarray] = None
        self.stamp = None
        self.recv_time = 0.0
        self.rate_hz = 0.0
        self.tf_ok = False
        self.raw_points = 0


class DualLidarCloudFusionNode(Node):
    def __init__(self):
        super().__init__("dual_lidar_cloud_fusion_node")
        self.declare_parameter("left_points_topic", "/xtm60/left/points")
        self.declare_parameter("right_points_topic", "/xtm60/right/points")
        self.declare_parameter("target_frame", "base_link")
        self.declare_parameter("output_topic", "/points_merged")
        self.declare_parameter("status_topic", "/points_merged/status")
        self.declare_parameter("min_range", 0.05)
        self.declare_parameter("max_range", 20.0)
        self.declare_parameter("z_min", -2.0)
        self.declare_parameter("z_max", 3.0)
        self.declare_parameter("voxel_leaf_size", 0.05)
        self.declare_parameter("allow_single_lidar_fallback", True)
        self.declare_parameter("publish_diagnostics", True)
        self.declare_parameter("output_rate_hz", 10.0)
        self.declare_parameter("input_timeout_sec", 0.5)
        self.declare_parameter("tf_timeout_sec", 0.1)

        self.target_frame = self.get_parameter("target_frame").value
        self.min_range = float(self.get_parameter("min_range").value)
        self.max_range = float(self.get_parameter("max_range").value)
        self.z_min = float(self.get_parameter("z_min").value)
        self.z_max = float(self.get_parameter("z_max").value)
        self.voxel = float(self.get_parameter("voxel_leaf_size").value)
        self.fallback = bool(self.get_parameter("allow_single_lidar_fallback").value)
        self.input_timeout = float(self.get_parameter("input_timeout_sec").value)
        self.tf_timeout = float(self.get_parameter("tf_timeout_sec").value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.left = _LidarState()
        self.right = _LidarState()
        self._warn_log = {}

        self.pub = self.create_publisher(PointCloud2, self.get_parameter("output_topic").value, qos_profile_sensor_data)
        self.status_pub = None
        if bool(self.get_parameter("publish_diagnostics").value):
            self.status_pub = self.create_publisher(String, self.get_parameter("status_topic").value, 10)

        self.create_subscription(
            PointCloud2, self.get_parameter("left_points_topic").value,
            lambda m: self._on_cloud(m, self.left), qos_profile_sensor_data)
        self.create_subscription(
            PointCloud2, self.get_parameter("right_points_topic").value,
            lambda m: self._on_cloud(m, self.right), qos_profile_sensor_data)

        period = 1.0 / max(1.0, float(self.get_parameter("output_rate_hz").value))
        self.timer = self.create_timer(period, self._publish_merged)
        self.get_logger().info(
            f"dual_lidar_cloud_fusion: target_frame={self.target_frame} "
            f"voxel={self.voxel} fallback={self.fallback}")

    def _warn(self, key: str, msg: str, period: float = 5.0):
        now = time.monotonic()
        if now - self._warn_log.get(key, 0.0) >= period:
            self.get_logger().warning(msg)
            self._warn_log[key] = now

    def _on_cloud(self, msg: PointCloud2, state: _LidarState):
        now = time.monotonic()
        if state.recv_time:
            dt = now - state.recv_time
            if dt > 0:
                state.rate_hz = 0.7 * state.rate_hz + 0.3 * (1.0 / dt)
        state.recv_time = now

        xyz, inten = cloud_utils.read_xyz_intensity(msg)
        state.raw_points = int(xyz.shape[0])
        if xyz.shape[0] == 0:
            state.tf_ok = True
            state.xyz, state.inten, state.stamp = xyz, inten, msg.header.stamp
            return

        mat = self._lookup(msg.header.frame_id)
        if mat is None:
            state.tf_ok = False
            self._warn(f"tf_{msg.header.frame_id}",
                       f"no TF {self.target_frame}<-{msg.header.frame_id}; skipping frame")
            return
        state.tf_ok = True
        try:
            xyz, inten = cloud_utils.filter_by_range(xyz, inten, self.min_range, self.max_range)
            n_range = xyz.shape[0]
            xyz = cloud_utils.apply_transform(xyz, mat)
            zlo = float(xyz[:, 2].min()) if xyz.shape[0] else 0.0
            zhi = float(xyz[:, 2].max()) if xyz.shape[0] else 0.0
            xyz, inten = cloud_utils.filter_by_height(xyz, inten, self.z_min, self.z_max)
            n_height = xyz.shape[0]
            xyz, inten = cloud_utils.voxel_downsample(xyz, inten, self.voxel)
            state.xyz, state.inten, state.stamp = xyz, inten, msg.header.stamp
            self._warn(f"stage_{msg.header.frame_id}",
                       f"{msg.header.frame_id}: raw={state.raw_points} after_range={n_range} "
                       f"base_z=[{zlo:.2f},{zhi:.2f}] after_height={n_height} after_voxel={xyz.shape[0]}", 3.0)
        except Exception as exc:
            self._warn(f"flt_{msg.header.frame_id}", f"filter pipeline error: {exc}")

    def _lookup(self, source_frame: str):
        if not source_frame:
            return None
        try:
            tf = self.tf_buffer.lookup_transform(
                self.target_frame, source_frame, rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=self.tf_timeout))
            return cloud_utils.transform_to_matrix(tf)
        except (LookupException, ConnectivityException, ExtrapolationException):
            return None

    def _fresh(self, state: _LidarState) -> bool:
        return state.xyz is not None and (time.monotonic() - state.recv_time) <= self.input_timeout

    def _publish_merged(self):
        left_ok, right_ok = self._fresh(self.left), self._fresh(self.right)
        parts_xyz, parts_i = [], []
        have_intensity = True
        for ok, st in ((left_ok, self.left), (right_ok, self.right)):
            if ok and st.xyz is not None and st.xyz.shape[0] > 0:
                parts_xyz.append(st.xyz)
                if st.inten is None:
                    have_intensity = False
                parts_i.append(st.inten)

        fallback_active = (left_ok != right_ok)
        if not parts_xyz:
            self._warn("no_input", "no fresh lidar input on either topic")
            self._publish_status(left_ok, right_ok, 0, False)
            return
        if fallback_active and not self.fallback:
            self._warn("fallback_off", "only one lidar fresh and fallback disabled; skipping")
            self._publish_status(left_ok, right_ok, 0, fallback_active)
            return

        xyz = np.vstack(parts_xyz)
        inten = np.concatenate([p for p in parts_i]) if have_intensity else None
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.target_frame
        self.pub.publish(cloud_utils.make_xyzi_cloud(header, xyz, inten))
        self._publish_status(left_ok, right_ok, int(xyz.shape[0]), fallback_active)

    def _publish_status(self, left_ok, right_ok, out_points, fallback_active):
        if self.status_pub is None:
            return
        status = {
            "target_frame": self.target_frame,
            "left_fresh": bool(left_ok), "right_fresh": bool(right_ok),
            "left_hz": round(self.left.rate_hz, 2), "right_hz": round(self.right.rate_hz, 2),
            "left_raw_points": self.left.raw_points, "right_raw_points": self.right.raw_points,
            "left_tf_ok": self.left.tf_ok, "right_tf_ok": self.right.tf_ok,
            "output_points": out_points, "single_lidar_fallback": bool(fallback_active),
        }
        self.status_pub.publish(String(data=json.dumps(status)))


def main(args=None):
    rclpy.init(args=args)
    node = DualLidarCloudFusionNode()
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
