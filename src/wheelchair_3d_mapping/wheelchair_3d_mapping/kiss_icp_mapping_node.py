"""Minimal KISS-ICP fallback 3D mapping node.

Subscribes /points_merged (XYZI, base_link, best-effort QoS), runs KISS-ICP
point-cloud odometry, and publishes /kiss/odom, /kiss/path, and /kiss/map_cloud
(KISS-ICP's rolling LOCAL map - bounded, NOT a global accumulation) plus TF
odom->base_link. Fallback for when RTAB-Map is blocked: a quick 3D geometry
check from LiDAR alone (no camera, no per-point time).

Requires the kiss-icp core: `pip install --user kiss-icp`.
"""
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, TransformStamped
from std_msgs.msg import Header
from tf2_ros import TransformBroadcaster

from wheelchair_3d_mapping import cloud_utils


def _mat_to_quat(R):
    """3x3 rotation -> (x, y, z, w)."""
    m00, m01, m02 = R[0]
    m10, m11, m12 = R[1]
    m20, m21, m22 = R[2]
    tr = m00 + m11 + m22
    if tr > 0.0:
        s = np.sqrt(tr + 1.0) * 2.0
        return (m21 - m12) / s, (m02 - m20) / s, (m10 - m01) / s, 0.25 * s
    if m00 > m11 and m00 > m22:
        s = np.sqrt(1.0 + m00 - m11 - m22) * 2.0
        return 0.25 * s, (m01 + m10) / s, (m02 + m20) / s, (m21 - m12) / s
    if m11 > m22:
        s = np.sqrt(1.0 + m11 - m00 - m22) * 2.0
        return (m01 + m10) / s, 0.25 * s, (m12 + m21) / s, (m02 - m20) / s
    s = np.sqrt(1.0 + m22 - m00 - m11) * 2.0
    return (m02 + m20) / s, (m12 + m21) / s, 0.25 * s, (m10 - m01) / s


class KissIcpMappingNode(Node):
    def __init__(self):
        super().__init__("kiss_icp_mapping_node")
        self.declare_parameter("cloud_topic", "/points_merged")
        self.declare_parameter("odom_topic", "/kiss/odom")
        self.declare_parameter("path_topic", "/kiss/path")
        self.declare_parameter("map_cloud_topic", "/kiss/map_cloud")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("deskew", False)  # XT-M60 flash ToF has no per-point time
        self.declare_parameter("max_range", 20.0)
        self.declare_parameter("min_range", 0.3)
        self.declare_parameter("voxel_size", 0.5)
        self.declare_parameter("map_publish_every", 5)  # publish map_cloud every N frames

        try:
            from kiss_icp.kiss_icp import KissICP
            from kiss_icp.config import load_config
        except ImportError as exc:
            self.get_logger().fatal(
                "kiss-icp not installed; install with: pip install --user kiss-icp "
                f"(import error: {exc})")
            raise SystemExit(1)

        cfg = load_config(None)
        # deskew needs per-point timestamps; XT-M60 flash ToF has none, so default
        # False. (deskew=True with constant timestamps also aborts the kiss-icp core.)
        cfg.data.deskew = bool(self.get_parameter("deskew").value)
        cfg.data.max_range = float(self.get_parameter("max_range").value)
        cfg.data.min_range = float(self.get_parameter("min_range").value)
        cfg.mapping.voxel_size = float(self.get_parameter("voxel_size").value)
        self._odom = KissICP(cfg)
        self._deskew = cfg.data.deskew

        self.odom_frame = self.get_parameter("odom_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.publish_tf = bool(self.get_parameter("publish_tf").value)
        self.map_every = max(1, int(self.get_parameter("map_publish_every").value))
        self._n = 0

        self.odom_pub = self.create_publisher(Odometry, self.get_parameter("odom_topic").value, 10)
        self.path_pub = self.create_publisher(Path, self.get_parameter("path_topic").value, 10)
        self.map_pub = self.create_publisher(PointCloud2, self.get_parameter("map_cloud_topic").value, 1)
        self.tf_bc = TransformBroadcaster(self)
        self.path = Path()
        self.create_subscription(
            PointCloud2, self.get_parameter("cloud_topic").value,
            self._on_cloud, qos_profile_sensor_data)
        self.get_logger().info(
            f"kiss_icp_mapping: voxel={cfg.mapping.voxel_size} "
            f"range=[{cfg.data.min_range},{cfg.data.max_range}] deskew={cfg.data.deskew}")

    def _on_cloud(self, msg: PointCloud2):
        xyz, _ = cloud_utils.read_xyz_intensity(msg)
        if xyz.shape[0] == 0:
            return
        xyz = xyz.astype(np.float64)
        xyz = xyz[np.isfinite(xyz).all(axis=1)]  # drop NaN/Inf points
        n = xyz.shape[0]
        if n == 0:
            return
        # No per-point time on a flash ToF; a tiny ramp avoids the kiss-icp core
        # aborting if deskew was force-enabled.
        ts = np.linspace(0.0, 1.0, n) if self._deskew else np.zeros(n)
        try:
            self._odom.register_frame(xyz, ts)
        except Exception as exc:  # noqa: BLE001 - keep node alive on a bad frame
            self.get_logger().warning(f"kiss-icp register_frame failed: {exc}")
            return
        self._publish(np.asarray(self._odom.last_pose), msg.header.stamp)

    def _publish(self, T, stamp):
        self._n += 1
        px, py, pz = float(T[0, 3]), float(T[1, 3]), float(T[2, 3])
        qx, qy, qz, qw = _mat_to_quat(T[:3, :3])

        od = Odometry()
        od.header.stamp = stamp
        od.header.frame_id = self.odom_frame
        od.child_frame_id = self.base_frame
        od.pose.pose.position.x, od.pose.pose.position.y, od.pose.pose.position.z = px, py, pz
        od.pose.pose.orientation.x = qx
        od.pose.pose.orientation.y = qy
        od.pose.pose.orientation.z = qz
        od.pose.pose.orientation.w = qw
        self.odom_pub.publish(od)

        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = px, py, pz
            t.transform.rotation = od.pose.pose.orientation
            self.tf_bc.sendTransform(t)

        ps = PoseStamped()
        ps.header = od.header
        ps.pose = od.pose.pose
        self.path.header = od.header
        self.path.poses.append(ps)
        if len(self.path.poses) > 5000:
            self.path.poses.pop(0)
        self.path_pub.publish(self.path)

        if self._n % self.map_every == 0:
            mp = np.asarray(self._odom.local_map.point_cloud())
            if mp.size:
                hdr = Header()
                hdr.stamp = stamp
                hdr.frame_id = self.odom_frame
                self.map_pub.publish(cloud_utils.make_xyzi_cloud(hdr, mp[:, :3], None))


def main(args=None):
    rclpy.init(args=args)
    node = KissIcpMappingNode()
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
