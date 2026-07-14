import math
import struct
import time
import zlib

import numpy as np
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rtabmap_msgs.srv import GetMap
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header

from smartwheel_sensor_api import pointcloud2_from_xyz


_CV_DEPTHS = {
    0: np.uint8,
    1: np.int8,
    2: np.uint16,
    3: np.int16,
    4: np.int32,
    5: np.float32,
    6: np.float64,
}

# RTAB-Map LaserScan::Format channel counts. Only 3D formats are accepted here.
_FORMAT_CHANNELS = {
    5: 3,   # XYZ
    6: 4,   # XYZI
    7: 4,   # XYZRGB
    8: 6,   # XYZNormal
    9: 7,   # XYZINormal
    10: 7,  # XYZRGBNormal
    11: 5,  # XYZIT
    12: 6,  # XYZIRT
}


def decode_compressed_cv_mat(data: bytes | bytearray | list[int]) -> np.ndarray:
    """Decode the generic cv::Mat representation written by RTAB-Map."""
    encoded = bytes(data)
    if len(encoded) <= 12:
        raise ValueError("compressed cv::Mat payload is too short")
    rows, columns, cv_type = struct.unpack_from("iii", encoded, len(encoded) - 12)
    depth = cv_type & 7
    channels = (cv_type >> 3) + 1
    if rows <= 0 or columns <= 0 or depth not in _CV_DEPTHS or channels <= 0:
        raise ValueError(
            f"invalid compressed cv::Mat footer: rows={rows}, columns={columns}, type={cv_type}"
        )
    try:
        raw = zlib.decompress(encoded[:-12])
    except zlib.error as exc:
        raise ValueError(f"RTAB-Map scan decompression failed: {exc}") from exc
    values = np.frombuffer(raw, dtype=_CV_DEPTHS[depth])
    expected = rows * columns * channels
    if values.size != expected:
        raise ValueError(f"decompressed cv::Mat has {values.size} values, expected {expected}")
    if channels == 1:
        return values.reshape(rows, columns)
    return values.reshape(rows, columns, channels)


def _transform_matrix(translation, rotation) -> np.ndarray:
    x, y, z, w = (
        float(rotation.x),
        float(rotation.y),
        float(rotation.z),
        float(rotation.w),
    )
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if not math.isfinite(norm) or norm < 1e-9:
        raise ValueError("pose contains an invalid quaternion")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    matrix = np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w), 0.0],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w), 0.0],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y), 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )
    matrix[:3, 3] = [float(translation.x), float(translation.y), float(translation.z)]
    return matrix


def assemble_optimized_cloud(map_data, voxel_size_m: float = 0.05) -> tuple[np.ndarray, int]:
    """Place every stored 3D scan at its optimized graph pose."""
    if len(map_data.graph.poses_id) != len(map_data.graph.poses):
        raise ValueError("RTAB-Map graph pose IDs and poses have different lengths")
    optimized_poses = dict(zip(map_data.graph.poses_id, map_data.graph.poses))
    clouds = []
    used_nodes = 0
    for node in map_data.nodes:
        pose = optimized_poses.get(node.id)
        if pose is None:
            continue
        sensor = node.data
        if not sensor.laser_scan_compressed:
            continue
        expected_channels = _FORMAT_CHANNELS.get(int(sensor.laser_scan_format))
        if expected_channels is None:
            raise ValueError(f"node {node.id} has unsupported 2D/unknown scan format {sensor.laser_scan_format}")
        decoded = decode_compressed_cv_mat(sensor.laser_scan_compressed)
        if decoded.ndim != 3 or decoded.shape[2] != expected_channels:
            raise ValueError(
                f"node {node.id} scan shape {decoded.shape} disagrees with format {sensor.laser_scan_format}"
            )
        xyz = np.asarray(decoded[..., :3], dtype=np.float64).reshape(-1, 3)
        xyz = xyz[np.isfinite(xyz).all(axis=1)]
        if xyz.size == 0:
            continue
        local = sensor.laser_scan_local_transform
        map_from_base = _transform_matrix(pose.position, pose.orientation)
        base_from_sensor = _transform_matrix(local.translation, local.rotation)
        homogeneous = np.column_stack((xyz, np.ones(xyz.shape[0], dtype=np.float64)))
        clouds.append((homogeneous @ (map_from_base @ base_from_sensor).T)[:, :3])
        used_nodes += 1
    if not clouds:
        raise ValueError("RTAB-Map response contains no usable optimized 3D scans")
    points = np.vstack(clouds)
    if voxel_size_m > 0.0:
        keys = np.floor(points / voxel_size_m).astype(np.int64)
        _, first = np.unique(keys, axis=0, return_index=True)
        points = points[np.sort(first)]
    return points.astype(np.float32), used_nodes


class RtabmapOptimizedCloudNode(Node):
    def __init__(self) -> None:
        super().__init__("smartwheel_rtabmap_optimized_cloud")
        self.declare_parameter("service_name", "/rtabmap/get_map_data")
        self.declare_parameter("output_topic", "/rtabmap/optimized_cloud")
        self.declare_parameter("request_interval_sec", 2.0)
        self.declare_parameter("voxel_size_m", 0.05)
        self.declare_parameter("minimum_nodes", 2)
        self._interval = float(self.get_parameter("request_interval_sec").value)
        self._voxel = float(self.get_parameter("voxel_size_m").value)
        self._minimum_nodes = int(self.get_parameter("minimum_nodes").value)
        if self._interval <= 0.0 or self._voxel < 0.0 or self._minimum_nodes < 1:
            raise ValueError("invalid optimized cloud node parameters")
        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._publisher = self.create_publisher(
            PointCloud2, str(self.get_parameter("output_topic").value), latched
        )
        self._diagnostics = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self._client = self.create_client(GetMap, str(self.get_parameter("service_name").value))
        self._future = None
        self._last_request = 0.0
        self.create_timer(0.25, self._tick)

    def _tick(self) -> None:
        if self._future is not None:
            if not self._future.done():
                return
            future = self._future
            self._future = None
            try:
                response = future.result()
                points, node_count = assemble_optimized_cloud(response.data, self._voxel)
                if node_count < self._minimum_nodes:
                    raise ValueError(f"only {node_count} optimized scan nodes, require {self._minimum_nodes}")
                header = Header(stamp=self.get_clock().now().to_msg(), frame_id="map")
                self._publisher.publish(pointcloud2_from_xyz(header, points))
                self._publish_diagnostic(
                    DiagnosticStatus.OK,
                    "optimized RTAB-Map cloud published",
                    node_count,
                    int(points.shape[0]),
                )
            except Exception as exc:  # ROS futures may raise transport exceptions.
                self._publish_diagnostic(DiagnosticStatus.ERROR, str(exc), 0, 0)
        now = time.monotonic()
        if now - self._last_request < self._interval or not self._client.service_is_ready():
            return
        request = GetMap.Request()
        request.global_map = True
        request.optimized = True
        request.graph_only = False
        self._future = self._client.call_async(request)
        self._last_request = now

    def _publish_diagnostic(self, level: int, message: str, nodes: int, points: int) -> None:
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus(
            level=level,
            name="smartwheel_global_mapping/rtabmap_optimized_cloud",
            hardware_id="simulation_backend",
            message=message,
        )
        status.values = [KeyValue(key="optimized_nodes", value=str(nodes)), KeyValue(key="points", value=str(points))]
        array.status = [status]
        self._diagnostics.publish(array)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RtabmapOptimizedCloudNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
