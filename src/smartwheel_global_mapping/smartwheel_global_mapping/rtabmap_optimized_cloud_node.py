import math
import struct
import time
import zlib

import numpy as np
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rtabmap_msgs.srv import GetMap
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

from smartwheel_sensor_api import pointcloud2_from_xyz, pointcloud2_to_xyz


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

# Channels in RTAB-Map's packed 3-D scan formats.  The first three channels
# are always XYZ; RGB/normal channels must never be mistaken for reflectance.
# ``LaserScan::kFormatXYZI`` and its time/normal variants keep intensity in
# channel three (zero based).  Formats without an intensity channel return
# ``None`` and are reported as such by the node instead of manufacturing an
# amplitude value.
_INTENSITY_CHANNEL = {
    6: 3,   # XYZI
    9: 3,   # XYZINormal
    11: 3,  # XYZIT
    12: 3,  # XYZIRT
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
    translation_values = np.asarray(
        [float(translation.x), float(translation.y), float(translation.z)],
        dtype=np.float64,
    )
    if not np.isfinite(translation_values).all():
        raise ValueError("pose contains a non-finite translation")
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
    matrix[:3, 3] = translation_values
    return matrix


def _normalised_quaternion(rotation) -> tuple[float, float, float, float]:
    values = np.asarray(
        [rotation.x, rotation.y, rotation.z, rotation.w], dtype=np.float64
    )
    norm = float(np.linalg.norm(values))
    if not np.isfinite(values).all() or not math.isfinite(norm) or norm < 1.0e-9:
        raise ValueError("pose contains an invalid quaternion")
    values /= norm
    return tuple(float(value) for value in values)


def _node_scan(sensor):
    """Return ``(xyz, intensity, has_intensity)`` for one RTAB-Map scan.

    RTAB-Map can return either its compressed cv::Mat representation or the
    original ``sensor_msgs/PointCloud2`` in ``GetMap`` responses, depending on
    database/configuration.  The old implementation silently ignored the
    latter and always discarded channel four from the former.  Keeping both
    paths here makes the export deterministic and lets callers distinguish a
    real amplitude channel from an XYZ-only scan.
    """

    compressed = getattr(sensor, "laser_scan_compressed", None)
    if compressed is not None and len(compressed) > 0:
        scan_format = int(getattr(sensor, "laser_scan_format", 0))
        expected_channels = _FORMAT_CHANNELS.get(scan_format)
        if expected_channels is None:
            raise ValueError(
                f"unsupported 3D scan format {scan_format} in compressed RTAB-Map node"
            )
        decoded = decode_compressed_cv_mat(compressed)
        if decoded.ndim != 3 or decoded.shape[2] != expected_channels:
            raise ValueError(
                f"scan shape {decoded.shape} disagrees with format {scan_format}"
            )
        values = np.asarray(decoded, dtype=np.float64).reshape(-1, expected_channels)
        xyz = values[:, :3]
        intensity_index = _INTENSITY_CHANNEL.get(scan_format)
        intensity = None if intensity_index is None else values[:, intensity_index]
        return _finite_scan_values(xyz, intensity)

    raw = getattr(sensor, "laser_scan", None)
    if raw is not None:
        # A default-constructed PointCloud2 has no fields/data.  Treat it as
        # absent so an incomplete node does not masquerade as an empty scan.
        fields = getattr(raw, "fields", None)
        data = getattr(raw, "data", None)
        if fields is not None and len(fields) > 0 and data is not None and len(data) > 0:
            xyz, intensity = pointcloud2_to_xyz(raw)
            return _finite_scan_values(xyz, intensity)
    return None, None, False


def _finite_scan_values(xyz, intensity):
    xyz = np.asarray(xyz, dtype=np.float64).reshape(-1, 3)
    finite_xyz = np.isfinite(xyz).all(axis=1)
    nonzero = np.linalg.norm(xyz, axis=1) > 0.0
    keep = finite_xyz & nonzero
    xyz = xyz[keep]
    if intensity is None:
        return xyz, None, False
    values = np.asarray(intensity, dtype=np.float64).reshape(-1)
    if values.shape[0] != keep.shape[0]:
        raise ValueError("scan intensity channel length does not match XYZ")
    if not np.isfinite(values).all():
        raise ValueError("scan intensity channel contains non-finite values")
    return xyz, values[keep].astype(np.float32), True


def assemble_optimized_cloud_with_intensity(
    map_data, voxel_size_m: float = 0.05
) -> tuple[np.ndarray, np.ndarray | None, int]:
    """Place stored 3-D scans at optimized graph poses and retain intensity.

    The returned intensity array is ``None`` when no scan contains a genuine
    intensity channel.  If a database mixes XYZI and XYZ-only keyframes, the
    function fails closed rather than padding the missing values with zero;
    an exported product must not claim complete PointCloud+Amp provenance in
    that case.  Voxel reduction keeps the first deterministic sample and its
    corresponding amplitude.
    """

    if len(map_data.graph.poses_id) != len(map_data.graph.poses):
        raise ValueError("RTAB-Map graph pose IDs and poses have different lengths")
    pose_ids = list(map_data.graph.poses_id)
    if len(set(pose_ids)) != len(pose_ids):
        raise ValueError("RTAB-Map graph contains duplicate optimized pose IDs")
    optimized_poses = dict(zip(pose_ids, map_data.graph.poses))
    clouds = []
    intensities = []
    intensity_flags = []
    used_nodes = 0
    for node in map_data.nodes:
        pose = optimized_poses.get(node.id)
        if pose is None:
            continue
        sensor = node.data
        xyz, intensity, has_intensity = _node_scan(sensor)
        if xyz is None:
            continue
        if xyz.size == 0:
            continue
        local = sensor.laser_scan_local_transform
        map_from_base = _transform_matrix(pose.position, pose.orientation)
        base_from_sensor = _transform_matrix(local.translation, local.rotation)
        homogeneous = np.column_stack((xyz, np.ones(xyz.shape[0], dtype=np.float64)))
        transformed = (homogeneous @ (map_from_base @ base_from_sensor).T)[:, :3]
        if not np.isfinite(transformed).all():
            raise ValueError(f"optimized pose for node {node.id} produced non-finite points")
        clouds.append(transformed)
        intensities.append(intensity)
        intensity_flags.append(has_intensity)
        used_nodes += 1
    if not clouds:
        raise ValueError("RTAB-Map response contains no usable optimized 3D scans")
    points = np.vstack(clouds)
    if any(intensity_flags) and not all(intensity_flags):
        raise ValueError(
            "RTAB-Map database mixes intensity and non-intensity scans; "
            "refusing to fabricate missing amplitude values"
        )
    intensity_out = None
    if intensity_flags and all(intensity_flags):
        intensity_out = np.concatenate(intensities).astype(np.float32, copy=False)
    if voxel_size_m > 0.0:
        keys = np.floor(points / voxel_size_m).astype(np.int64)
        _, first = np.unique(keys, axis=0, return_index=True)
        points = points[np.sort(first)]
        if intensity_out is not None:
            intensity_out = intensity_out[np.sort(first)]
    return points.astype(np.float32), intensity_out, used_nodes


def assemble_optimized_cloud(map_data, voxel_size_m: float = 0.05) -> tuple[np.ndarray, int]:
    """Backward-compatible XYZ-only view of the optimized cloud.

    Existing callers/tests used the two-value return signature.  New export
    code should call :func:`assemble_optimized_cloud_with_intensity`.
    """

    points, _intensity, nodes = assemble_optimized_cloud_with_intensity(
        map_data, voxel_size_m
    )
    return points, nodes


def assemble_optimized_trajectory(map_data) -> list[tuple[int, float, object]]:
    """Return timestamped optimized sensor poses in chronological order.

    ``MapGraph`` contains optimized poses but not their acquisition times;
    ``Node.stamp`` carries the matching sensor timestamp.  A formal trajectory
    must use both rather than exporting the unoptimized odometry callback
    history alongside optimized map geometry.
    """

    if len(map_data.graph.poses_id) != len(map_data.graph.poses):
        raise ValueError("RTAB-Map graph pose IDs and poses have different lengths")
    pose_ids = list(map_data.graph.poses_id)
    if len(set(pose_ids)) != len(pose_ids):
        raise ValueError("RTAB-Map graph contains duplicate optimized pose IDs")
    optimized_poses = dict(zip(pose_ids, map_data.graph.poses))

    samples: list[tuple[int, float, object]] = []
    seen_nodes: set[int] = set()
    for node in map_data.nodes:
        node_id = int(node.id)
        if node_id in seen_nodes:
            raise ValueError(f"RTAB-Map response contains duplicate node ID {node_id}")
        seen_nodes.add(node_id)
        pose = optimized_poses.get(node_id)
        if pose is None:
            continue
        try:
            stamp = float(node.stamp)
        except (AttributeError, TypeError, ValueError) as exc:
            raise ValueError(f"RTAB-Map node {node_id} has no numeric timestamp") from exc
        if not math.isfinite(stamp) or stamp <= 0.0:
            raise ValueError(f"RTAB-Map node {node_id} has an invalid timestamp")
        # Reuse the pose validation used by cloud assembly.  Also validate and
        # normalize the quaternion that will be copied into nav_msgs/Path.
        _transform_matrix(pose.position, pose.orientation)
        _normalised_quaternion(pose.orientation)
        samples.append((node_id, stamp, pose))

    if not samples:
        raise ValueError("RTAB-Map response contains no timestamped optimized poses")
    samples.sort(key=lambda sample: (sample[1], sample[0]))
    if any(current[1] <= previous[1] for previous, current in zip(samples, samples[1:])):
        raise ValueError("RTAB-Map optimized trajectory timestamps are not strictly increasing")
    return samples


class RtabmapOptimizedCloudNode(Node):
    def __init__(self) -> None:
        super().__init__("smartwheel_rtabmap_optimized_cloud")
        self.declare_parameter("service_name", "/rtabmap/get_map_data")
        self.declare_parameter("output_topic", "/rtabmap/optimized_cloud")
        self.declare_parameter("path_output_topic", "/rtabmap/optimized_path")
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
        self._path_publisher = self.create_publisher(
            Path, str(self.get_parameter("path_output_topic").value), latched
        )
        self._diagnostics = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self._client = self.create_client(GetMap, str(self.get_parameter("service_name").value))
        self._future = None
        self._last_request = 0.0
        self._last_intensity_preserved = False
        self.create_timer(0.25, self._tick)

    @staticmethod
    def _cloud_message(header, points, intensity):
        """Build a cloud without inventing an intensity channel.

        ``pointcloud2_from_xyz`` intentionally emits an intensity field for
        historical compatibility, even when its argument is ``None``.  That
        behaviour is unsuitable for an optimized-map exporter: zero padding
        would look like a valid PointCloud+Amp product.  Emit a genuine XYZI
        cloud only when RTAB-Map supplied intensity; otherwise emit XYZ.
        """

        if intensity is not None:
            return pointcloud2_from_xyz(header, points, intensity)
        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        values = np.asarray(points, dtype=np.float32).reshape(-1, 3)
        return point_cloud2.create_cloud(header, fields, values.tolist())

    @staticmethod
    def _path_message(header, samples) -> Path:
        message = Path()
        message.header = header
        for _node_id, stamp, pose in samples:
            item = PoseStamped()
            item.header.frame_id = header.frame_id
            seconds = int(math.floor(stamp))
            nanoseconds = int(round((stamp - seconds) * 1.0e9))
            if nanoseconds >= 1_000_000_000:
                seconds += 1
                nanoseconds -= 1_000_000_000
            item.header.stamp.sec = seconds
            item.header.stamp.nanosec = nanoseconds
            item.pose.position.x = float(pose.position.x)
            item.pose.position.y = float(pose.position.y)
            item.pose.position.z = float(pose.position.z)
            qx, qy, qz, qw = _normalised_quaternion(pose.orientation)
            item.pose.orientation.x = qx
            item.pose.orientation.y = qy
            item.pose.orientation.z = qz
            item.pose.orientation.w = qw
            message.poses.append(item)
        return message

    def _tick(self) -> None:
        if self._future is not None:
            if not self._future.done():
                return
            future = self._future
            self._future = None
            try:
                response = future.result()
                points, intensity, node_count = assemble_optimized_cloud_with_intensity(
                    response.data, self._voxel
                )
                trajectory = assemble_optimized_trajectory(response.data)
                if node_count < self._minimum_nodes:
                    raise ValueError(f"only {node_count} optimized scan nodes, require {self._minimum_nodes}")
                if len(trajectory) < self._minimum_nodes:
                    raise ValueError(
                        f"only {len(trajectory)} optimized trajectory poses, require {self._minimum_nodes}"
                    )
                header = Header(stamp=self.get_clock().now().to_msg(), frame_id="map")
                self._publisher.publish(self._cloud_message(header, points, intensity))
                self._path_publisher.publish(self._path_message(header, trajectory))
                self._last_intensity_preserved = intensity is not None
                self._publish_diagnostic(
                    DiagnosticStatus.OK,
                    "optimized RTAB-Map cloud published",
                    node_count,
                    int(points.shape[0]),
                    intensity_preserved=self._last_intensity_preserved,
                    trajectory_poses=len(trajectory),
                )
            except Exception as exc:  # ROS futures may raise transport exceptions.
                self._last_intensity_preserved = False
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

    def _publish_diagnostic(
        self,
        level: int,
        message: str,
        nodes: int,
        points: int,
        intensity_preserved: bool | None = None,
        trajectory_poses: int = 0,
    ) -> None:
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus(
            level=level,
            name="smartwheel_global_mapping/rtabmap_optimized_cloud",
            hardware_id="simulation_backend",
            message=message,
        )
        if intensity_preserved is None:
            intensity_preserved = self._last_intensity_preserved
        status.values = [
            KeyValue(key="optimized_nodes", value=str(nodes)),
            KeyValue(key="points", value=str(points)),
            KeyValue(key="optimized_trajectory_poses", value=str(trajectory_poses)),
            KeyValue(
                key="intensity_preserved",
                value=str(bool(intensity_preserved)).lower(),
            ),
        ]
        array.status = [status]
        self._diagnostics.publish(array)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RtabmapOptimizedCloudNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    except SystemError:
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
