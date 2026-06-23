"""Project a 3D point cloud to a 2D OccupancyGrid for Nav2.

This does NOT replace the 3D map. It flattens the LIVO registered/map cloud
into a 2.5D occupancy grid so the existing Nav2 stack keeps working with
click-to-goal navigation. Obstacle points (within an obstacle z-band) become
occupied cells; ground points become free; everything else is unknown.
Occupied cells are inflated by inflation_radius.
"""
import time

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import Pose
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException

from wheelchair_3d_mapping import cloud_utils


class CloudToOccupancyGridNode(Node):
    def __init__(self):
        super().__init__("cloud_to_occupancy_grid_node")
        self.declare_parameter("input_cloud_topic", "/livo/cloud_registered")
        self.declare_parameter("output_map_topic", "/map_2d_from_3d")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("resolution", 0.05)
        self.declare_parameter("obstacle_z_min", 0.15)
        self.declare_parameter("obstacle_z_max", 1.8)
        self.declare_parameter("ground_z_min", -0.30)
        self.declare_parameter("ground_z_max", 0.15)
        self.declare_parameter("inflation_radius", 0.30)
        self.declare_parameter("unknown_value", -1)
        self.declare_parameter("free_value", 0)
        self.declare_parameter("occupied_value", 100)
        self.declare_parameter("rolling_or_static", "rolling")
        self.declare_parameter("margin_m", 1.0)
        self.declare_parameter("map_width_m", 40.0)
        self.declare_parameter("map_height_m", 40.0)
        self.declare_parameter("origin_x", -20.0)
        self.declare_parameter("origin_y", -20.0)
        self.declare_parameter("max_cells", 4000)
        self.declare_parameter("publish_rate_hz", 2.0)
        self.declare_parameter("tf_timeout_sec", 0.1)
        self.declare_parameter("input_timeout_sec", 1.0)
        # D038 fix: accumulate occupancy across frames instead of rebuilding from
        # the current frame only. With /cloud_registered (current-frame cloud) and
        # no accumulation the 2D map was just the instantaneous view -- driving
        # past / turning erased already-seen obstacles. accumulate=False keeps the
        # old instantaneous behaviour for callers that feed a full /cloud_map.
        self.declare_parameter("accumulate", True)

        self.map_frame = self.get_parameter("map_frame").value
        self.res = float(self.get_parameter("resolution").value)
        self.oz = (float(self.get_parameter("obstacle_z_min").value), float(self.get_parameter("obstacle_z_max").value))
        self.gz = (float(self.get_parameter("ground_z_min").value), float(self.get_parameter("ground_z_max").value))
        self.inflation = float(self.get_parameter("inflation_radius").value)
        self.v_unknown = int(self.get_parameter("unknown_value").value)
        self.v_free = int(self.get_parameter("free_value").value)
        self.v_occ = int(self.get_parameter("occupied_value").value)
        self.rolling = str(self.get_parameter("rolling_or_static").value).lower() == "rolling"
        self.margin = float(self.get_parameter("margin_m").value)
        self.max_cells = int(self.get_parameter("max_cells").value)
        self.tf_timeout = float(self.get_parameter("tf_timeout_sec").value)
        self.input_timeout = float(self.get_parameter("input_timeout_sec").value)
        self.accumulate = bool(self.get_parameter("accumulate").value)
        # Persistent accumulation state (D038): anchored grid that grows-by-OR.
        self._acc = None          # int16 grid (h,w) of v_unknown/v_free/v_occ
        self._acc_origin = None   # (ox, oy) anchored once, fixed thereafter (D039)
        self._acc_wh = None       # (w, h)

        self._latest = None
        self._latest_recv = 0.0
        self._warned = {}
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(PointCloud2, self.get_parameter("input_cloud_topic").value,
                                 self._on_cloud, qos_profile_sensor_data)
        map_qos = QoSProfile(depth=1)
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.pub = self.create_publisher(OccupancyGrid, self.get_parameter("output_map_topic").value, map_qos)
        self.timer = self.create_timer(1.0 / max(0.2, float(self.get_parameter("publish_rate_hz").value)), self._tick)
        self._offsets = self._circle_offsets(self.inflation, self.res)
        self.get_logger().info(f"cloud_to_occupancy_grid: frame={self.map_frame} res={self.res} mode={'rolling' if self.rolling else 'static'}")

    def _warn(self, key, msg, period=5.0):
        now = time.monotonic()
        if now - self._warned.get(key, 0.0) >= period:
            self.get_logger().warning(msg)
            self._warned[key] = now

    @staticmethod
    def _circle_offsets(radius_m, res):
        if radius_m <= 0.0 or res <= 0.0:
            return [(0, 0)]
        r = int(round(radius_m / res))
        out = [(dx, dy) for dx in range(-r, r + 1) for dy in range(-r, r + 1) if dx * dx + dy * dy <= r * r]
        return out or [(0, 0)]

    def _on_cloud(self, msg):
        xyz, _ = cloud_utils.read_xyz_intensity(msg)
        if xyz.shape[0] == 0:
            return
        if msg.header.frame_id and msg.header.frame_id != self.map_frame:
            mat = self._lookup(msg.header.frame_id)
            if mat is None:
                self._warn("tf", f"no TF {self.map_frame}<-{msg.header.frame_id}; skipping cloud")
                return
            xyz = cloud_utils.apply_transform(xyz, mat)
        self._latest = xyz
        self._latest_recv = time.monotonic()

    def _lookup(self, source):
        try:
            tf = self.tf_buffer.lookup_transform(self.map_frame, source, rclpy.time.Time(),
                                                 timeout=rclpy.duration.Duration(seconds=self.tf_timeout))
            return cloud_utils.transform_to_matrix(tf)
        except (LookupException, ConnectivityException, ExtrapolationException):
            return None

    def _bounds(self, xyz):
        if self.rolling:
            mn = xyz[:, :2].min(axis=0) - self.margin
            mx = xyz[:, :2].max(axis=0) + self.margin
            ox, oy = float(mn[0]), float(mn[1])
            w = int(np.ceil((mx[0] - mn[0]) / self.res))
            h = int(np.ceil((mx[1] - mn[1]) / self.res))
        else:
            ox = float(self.get_parameter("origin_x").value)
            oy = float(self.get_parameter("origin_y").value)
            w = int(np.ceil(float(self.get_parameter("map_width_m").value) / self.res))
            h = int(np.ceil(float(self.get_parameter("map_height_m").value) / self.res))
        w = max(1, min(w, self.max_cells))
        h = max(1, min(h, self.max_cells))
        return ox, oy, w, h

    def _fixed_bounds(self):
        """Fixed canvas (anchored once) for accumulate mode. Uses the static
        origin/size config so the persistent map never shifts (D039)."""
        ox = float(self.get_parameter("origin_x").value)
        oy = float(self.get_parameter("origin_y").value)
        w = int(np.ceil(float(self.get_parameter("map_width_m").value) / self.res))
        h = int(np.ceil(float(self.get_parameter("map_height_m").value) / self.res))
        w = max(1, min(w, self.max_cells))
        h = max(1, min(h, self.max_cells))
        return ox, oy, w, h

    def _tick(self):
        xyz = self._latest
        if xyz is None:
            return
        if (time.monotonic() - self._latest_recv) > self.input_timeout:
            self._warn("stale_cloud", "input cloud stale; suppressing 2D map (LIVO backend down?)")
            return

        if self.accumulate:
            self._tick_accumulate(xyz)
        else:
            self._tick_instantaneous(xyz)

    def _classify(self, xyz, ox, oy, w, h):
        """Return (free_mask_grid, occ_bool_grid) for the points in this frame."""
        col = np.floor((xyz[:, 0] - ox) / self.res).astype(np.int64)
        row = np.floor((xyz[:, 1] - oy) / self.res).astype(np.int64)
        inb = (col >= 0) & (col < w) & (row >= 0) & (row < h)
        z = xyz[:, 2]
        free = np.zeros((h, w), dtype=bool)
        occ = np.zeros((h, w), dtype=bool)
        gmask = inb & (z >= self.gz[0]) & (z <= self.gz[1])
        if np.any(gmask):
            free[row[gmask], col[gmask]] = True
        omask = inb & (z >= self.oz[0]) & (z <= self.oz[1])
        if np.any(omask):
            occ[row[omask], col[omask]] = True
        dropped = int((~inb).sum())
        if dropped:
            self._warn("oob", f"{dropped} pts outside 2D canvas; enlarge map_width/height_m")
        return free, occ

    def _tick_accumulate(self, xyz):
        ox, oy, w, h = self._fixed_bounds()
        if self._acc is None or self._acc_wh != (w, h) or self._acc_origin != (ox, oy):
            self._acc = np.full((h, w), self.v_unknown, dtype=np.int16)
            self._acc_origin = (ox, oy)
            self._acc_wh = (w, h)
        free, occ = self._classify(xyz, ox, oy, w, h)
        occ = self._inflate(occ)
        # Merge into persistent grid: free fills unknown; occupied is sticky and
        # overrides free (obstacles persist once seen).
        fill_free = free & (self._acc == self.v_unknown)
        self._acc[fill_free] = self.v_free
        self._acc[occ] = self.v_occ
        self._publish(self._acc, ox, oy, w, h)

    def _tick_instantaneous(self, xyz):
        ox, oy, w, h = self._bounds(xyz)
        grid = np.full((h, w), self.v_unknown, dtype=np.int16)
        free, occ = self._classify(xyz, ox, oy, w, h)
        grid[free] = self.v_free
        occ = self._inflate(occ)
        grid[occ] = self.v_occ
        self._publish(grid, ox, oy, w, h)

    def _inflate(self, occ):
        if len(self._offsets) <= 1:
            return occ
        h, w = occ.shape
        ys, xs = np.nonzero(occ)
        out = np.zeros_like(occ)
        for dx, dy in self._offsets:
            ny, nx = ys + dy, xs + dx
            valid = (ny >= 0) & (ny < h) & (nx >= 0) & (nx < w)
            out[ny[valid], nx[valid]] = True
        return out

    def _publish(self, grid, ox, oy, w, h):
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        msg.info.resolution = self.res
        msg.info.width = w
        msg.info.height = h
        origin = Pose()
        origin.position.x = ox
        origin.position.y = oy
        origin.orientation.w = 1.0
        msg.info.origin = origin
        msg.data = np.clip(grid, -128, 127).astype(np.int8).reshape(-1).tolist()
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CloudToOccupancyGridNode()
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
