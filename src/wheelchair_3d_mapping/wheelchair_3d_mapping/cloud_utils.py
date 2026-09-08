"""Shared point-cloud helpers for the 3D mapping nodes.

Everything here is plain numpy so the package has no hard dependency on
cv_bridge or tf2_sensor_msgs. PointCloud2 (de)serialization uses sensor_msgs_py
which ships with ROS 2 Humble.
"""
from typing import Optional, Tuple

import numpy as np

try:
    from sensor_msgs.msg import PointCloud2, PointField
    from sensor_msgs_py import point_cloud2
except ImportError:  # allows pure-python unit tests without ROS sourced
    PointCloud2 = None
    PointField = None
    point_cloud2 = None


def field_names(msg) -> list:
    return [f.name for f in msg.fields]


def read_xyz_intensity(msg) -> Tuple[np.ndarray, Optional[np.ndarray]]:
    """Return (N,3) float64 xyz and optional (N,) intensity, NaNs removed.

    Only x/y/z are required. intensity is used when present and ignored
    otherwise, so clouds without intensity still work.
    """
    names = field_names(msg)
    if not {"x", "y", "z"}.issubset(names):
        return np.empty((0, 3), dtype=np.float64), None
    has_i = "intensity" in names
    want = ("x", "y", "z", "intensity") if has_i else ("x", "y", "z")
    arr = point_cloud2.read_points(msg, field_names=want, skip_nans=True)
    if arr is None or len(arr) == 0:
        return np.empty((0, 3), dtype=np.float64), None
    xyz = np.column_stack((arr["x"], arr["y"], arr["z"])).astype(np.float64)
    inten = arr["intensity"].astype(np.float32) if has_i else None
    return xyz, inten


def quat_to_rotation(x: float, y: float, z: float, w: float) -> np.ndarray:
    n = x * x + y * y + z * z + w * w
    if n < 1e-12:
        return np.eye(3)
    s = 2.0 / n
    xx, yy, zz = x * x * s, y * y * s, z * z * s
    xy, xz, yz = x * y * s, x * z * s, y * z * s
    wx, wy, wz = w * x * s, w * y * s, w * z * s
    return np.array(
        [
            [1.0 - (yy + zz), xy - wz, xz + wy],
            [xy + wz, 1.0 - (xx + zz), yz - wx],
            [xz - wy, yz + wx, 1.0 - (xx + yy)],
        ]
    )


def transform_to_matrix(transform) -> np.ndarray:
    """geometry_msgs/TransformStamped -> 4x4 homogeneous matrix."""
    t = transform.transform.translation
    q = transform.transform.rotation
    mat = np.eye(4)
    mat[:3, :3] = quat_to_rotation(q.x, q.y, q.z, q.w)
    mat[:3, 3] = (t.x, t.y, t.z)
    return mat


def apply_transform(xyz: np.ndarray, mat: np.ndarray) -> np.ndarray:
    if xyz.shape[0] == 0:
        return xyz
    return xyz @ mat[:3, :3].T + mat[:3, 3]


def filter_by_range(
    xyz: np.ndarray, inten, min_range: float, max_range: float
) -> Tuple[np.ndarray, Optional[np.ndarray]]:
    """Radial range filter in the sensor frame (call before transforming)."""
    if xyz.shape[0] == 0:
        return xyz, inten
    # audit D179 (低): explicitly reject (0,0,0) placeholder points and any
    # point with non-finite components.  Do NOT rely on min_range side-effect
    # (would silently pass through when min_range == 0).  min_range semantics
    # (user-configured near-clip) are preserved unchanged by the r >= min_range
    # term below.
    finite_mask = (
        np.isfinite(xyz[:, 0])
        & np.isfinite(xyz[:, 1])
        & np.isfinite(xyz[:, 2])
    )
    r = np.linalg.norm(xyz, axis=1)
    mask = finite_mask & (r > 0) & (r >= min_range) & (r <= max_range)
    return xyz[mask], (inten[mask] if inten is not None else None)


def filter_by_height(
    xyz: np.ndarray, inten, z_min: float, z_max: float
) -> Tuple[np.ndarray, Optional[np.ndarray]]:
    """Z filter in the target frame (call after transforming)."""
    if xyz.shape[0] == 0:
        return xyz, inten
    mask = (xyz[:, 2] >= z_min) & (xyz[:, 2] <= z_max)
    return xyz[mask], (inten[mask] if inten is not None else None)


def voxel_downsample(
    xyz: np.ndarray, inten, leaf: float
) -> Tuple[np.ndarray, Optional[np.ndarray]]:
    """Keep one representative point per voxel. leaf<=0 disables it."""
    if leaf <= 0.0 or xyz.shape[0] == 0:
        return xyz, inten
    keys = np.floor(xyz / leaf).astype(np.int64)
    _, idx = np.unique(keys, axis=0, return_index=True)
    return xyz[idx], (inten[idx] if inten is not None else None)


_XYZ_FIELDS = None
_XYZI_FIELDS = None


def _xyz_fields():
    global _XYZ_FIELDS
    if _XYZ_FIELDS is None:
        f32 = PointField.FLOAT32
        _XYZ_FIELDS = [
            PointField(name="x", offset=0, datatype=f32, count=1),
            PointField(name="y", offset=4, datatype=f32, count=1),
            PointField(name="z", offset=8, datatype=f32, count=1),
        ]
    return _XYZ_FIELDS
_XYZRGB_FIELDS = None


def _xyzi_fields():
    global _XYZI_FIELDS
    if _XYZI_FIELDS is None:
        f32 = PointField.FLOAT32
        _XYZI_FIELDS = [
            PointField(name="x", offset=0, datatype=f32, count=1),
            PointField(name="y", offset=4, datatype=f32, count=1),
            PointField(name="z", offset=8, datatype=f32, count=1),
            PointField(name="intensity", offset=12, datatype=f32, count=1),
        ]
    return _XYZI_FIELDS


def make_xyz_cloud(header, xyz: np.ndarray) -> "PointCloud2":
    """Build a genuine XYZ-only cloud.

    A missing amplitude channel is represented by missing metadata, never by a
    column of zeroes.  This distinction is important for formal PointCloud+Amp
    acceptance: downstream consumers can reject an XYZ-only stream instead of
    mistaking fabricated values for sensor reflectance.
    """

    xyz = np.asarray(xyz, dtype=np.float32)
    if xyz.ndim != 2 or xyz.shape[1] != 3:
        raise ValueError("points must have shape (N, 3)")
    if not np.isfinite(xyz).all():
        raise ValueError("points must be finite")
    n = int(xyz.shape[0])
    msg = PointCloud2()
    msg.header = header
    msg.height = 1
    msg.width = n
    msg.fields = _xyz_fields()
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = 12 * n
    msg.is_dense = True
    msg.data = xyz.tobytes()
    return msg


def make_xyzi_cloud(header, xyz: np.ndarray, inten=None) -> "PointCloud2":
    n = int(xyz.shape[0])
    xyz = np.asarray(xyz, dtype=np.float32)
    if xyz.ndim != 2 or xyz.shape[1] != 3:
        raise ValueError("points must have shape (N, 3)")
    if inten is not None:
        inten = np.asarray(inten, dtype=np.float32).reshape(-1)
        if inten.shape[0] != n:
            raise ValueError("intensity length must match points")
    data = np.zeros((n, 4), dtype=np.float32)
    if n:
        data[:, :3] = xyz.astype(np.float32)
        if inten is not None:
            data[:, 3] = inten
    msg = PointCloud2()
    msg.header = header
    msg.height = 1
    msg.width = n
    msg.fields = _xyzi_fields()
    msg.is_bigendian = False
    msg.point_step = 16
    msg.row_step = 16 * n
    msg.is_dense = True
    msg.data = data.tobytes()
    return msg


_XYZIT_FIELDS = None


def _xyzit_fields():
    global _XYZIT_FIELDS
    if _XYZIT_FIELDS is None:
        f32 = PointField.FLOAT32
        _XYZIT_FIELDS = [
            PointField(name="x", offset=0, datatype=f32, count=1),
            PointField(name="y", offset=4, datatype=f32, count=1),
            PointField(name="z", offset=8, datatype=f32, count=1),
            PointField(name="intensity", offset=12, datatype=f32, count=1),
            PointField(name="time", offset=16, datatype=f32, count=1),
        ]
    return _XYZIT_FIELDS


def make_xyzi_time_cloud(header, xyz: np.ndarray, inten=None, time_value: float = 0.0) -> "PointCloud2":
    """Like make_xyzi_cloud but appends a per-point float32 'time' field.

    Every point gets the same ``time_value`` (default 0.0). This satisfies
    FAST-LIO parsers that strictly require a per-point time field while keeping
    the integer-frame single-timestamp semantics (XT-M60 is a whole-frame
    snapshot with no per-point timing).
    """
    n = int(xyz.shape[0])
    data = np.zeros((n, 5), dtype=np.float32)
    if n:
        data[:, :3] = xyz.astype(np.float32)
        if inten is not None:
            data[:, 3] = inten.astype(np.float32)
        data[:, 4] = np.float32(time_value)
    msg = PointCloud2()
    msg.header = header
    msg.height = 1
    msg.width = n
    msg.fields = _xyzit_fields()
    msg.is_bigendian = False
    msg.point_step = 20
    msg.row_step = 20 * n
    msg.is_dense = True
    msg.data = data.tobytes()
    return msg


_VELODYNE_FIELDS = None


def _velodyne_fields():
    """FAST-LIO velodyne_ros::Point layout: x,y,z,intensity (float32) + time
    (float32) + ring (uint16). Field offsets match the PCL-registered struct so
    pcl::fromROSMsg<velodyne_ros::Point> reads them by name."""
    global _VELODYNE_FIELDS
    if _VELODYNE_FIELDS is None:
        f32 = PointField.FLOAT32
        u16 = PointField.UINT16
        _VELODYNE_FIELDS = [
            PointField(name="x", offset=0, datatype=f32, count=1),
            PointField(name="y", offset=4, datatype=f32, count=1),
            PointField(name="z", offset=8, datatype=f32, count=1),
            PointField(name="intensity", offset=12, datatype=f32, count=1),
            PointField(name="time", offset=16, datatype=f32, count=1),
            PointField(name="ring", offset=20, datatype=u16, count=1),
        ]
    return _VELODYNE_FIELDS


def make_velodyne_cloud(header, xyz: np.ndarray, inten=None, ring=None,
                        sweep_time: float = 0.1) -> "PointCloud2":
    """Build a FAST-LIO velodyne-format cloud (x,y,z,intensity,time,ring).

    Rationale (spec fastlio-narrow-fov-mapping, Task 7):
    FAST-LIO's lidar_type=2 path deserializes into velodyne_ros::Point which
    REQUIRES both 'ring' (uint16) and 'time' (float32). XT-M60 is a flash
    snapshot with neither.

    Two coupled requirements discovered by investigation:
      1. RING: emit a valid ring index (0 for all points) so FAST-LIO's
         ring-indexed arrays are never read out of bounds.
      2. PER-POINT TIME SWEEP: FAST-LIO's sync_packages() derives each scan's
         end-time from points.back().time, and only batches IMU samples up to
         that end-time. If every point shares one timestamp, lidar_end_time
         collapses to ~lidar_beg_time, so almost NO IMU is integrated between
         frames and the orientation never tracks the gyro. Fix: ramp 'time'
         LINEARLY from 0 .. sweep_time across the frame (default 0.1 s = one
         10 Hz frame period) so lidar_end_time ~= beg + 0.1 s and the full IMU
         stream between frames is consumed. Use timestamp_unit=0 (seconds) in
         the FAST-LIO config so time is interpreted as seconds.

    Note: this 'sweep' is a synthetic ordering for IMU batching only; XT-M60 is
    a whole-frame snapshot so the absolute per-point timing is not physical, but
    a monotone 0..0.1 s ramp gives FAST-LIO the per-frame window it needs without
    introducing real de-skew error (motion within 0.1 s at <=0.3 rad/s is tiny).
    """
    n = int(xyz.shape[0])
    point_step = 24
    buf = np.zeros((n, point_step), dtype=np.uint8)
    if n:
        f = np.zeros((n, 5), dtype=np.float32)
        f[:, :3] = xyz.astype(np.float32)
        if inten is not None:
            f[:, 3] = inten.astype(np.float32)
        # per-point time ramp 0..sweep_time (seconds) across the frame
        if n > 1:
            f[:, 4] = np.linspace(0.0, sweep_time, n, dtype=np.float32)
        else:
            f[:, 4] = np.float32(sweep_time)
        buf[:, 0:20] = f.view(np.uint8).reshape(n, 20)
        if ring is None:
            r = np.zeros(n, dtype=np.uint16)
        else:
            r = np.asarray(ring, dtype=np.uint16)
        buf[:, 20:22] = r.view(np.uint8).reshape(n, 2)
    msg = PointCloud2()
    msg.header = header
    msg.height = 1
    msg.width = n
    msg.fields = _velodyne_fields()
    msg.is_bigendian = False
    msg.point_step = point_step
    msg.row_step = point_step * n
    msg.is_dense = True
    msg.data = buf.tobytes()
    return msg


def _xyzrgb_fields():
    global _XYZRGB_FIELDS
    if _XYZRGB_FIELDS is None:
        f32 = PointField.FLOAT32
        _XYZRGB_FIELDS = [
            PointField(name="x", offset=0, datatype=f32, count=1),
            PointField(name="y", offset=4, datatype=f32, count=1),
            PointField(name="z", offset=8, datatype=f32, count=1),
            PointField(name="rgb", offset=12, datatype=f32, count=1),
        ]
    return _XYZRGB_FIELDS


def make_xyzrgb_cloud(header, xyz: np.ndarray, rgb_u8: np.ndarray) -> "PointCloud2":
    """rgb_u8 is (N,3) uint8 in R,G,B order."""
    n = int(xyz.shape[0])
    packed = np.zeros(n, dtype=np.uint32)
    if n:
        r = rgb_u8[:, 0].astype(np.uint32)
        g = rgb_u8[:, 1].astype(np.uint32)
        b = rgb_u8[:, 2].astype(np.uint32)
        packed = (r << 16) | (g << 8) | b
    data = np.zeros((n, 4), dtype=np.float32)
    if n:
        data[:, :3] = xyz.astype(np.float32)
        data[:, 3] = packed.view(np.float32)
    msg = PointCloud2()
    msg.header = header
    msg.height = 1
    msg.width = n
    msg.fields = _xyzrgb_fields()
    msg.is_bigendian = False
    msg.point_step = 16
    msg.row_step = 16 * n
    msg.is_dense = True
    msg.data = data.tobytes()
    return msg


def image_to_rgb(msg) -> Optional[np.ndarray]:
    """Decode sensor_msgs/Image (bgr8/rgb8/mono8) to an (H,W,3) uint8 RGB array."""
    enc = msg.encoding
    channels = 1 if enc == "mono8" else 3
    buf = np.frombuffer(msg.data, dtype=np.uint8)
    expected = msg.height * msg.step
    if buf.size < expected or msg.step <= 0:
        return None
    img = buf[:expected].reshape(msg.height, msg.step)
    img = img[:, : msg.width * channels].reshape(msg.height, msg.width, channels)
    if enc == "bgr8":
        img = img[:, :, ::-1]
    elif enc == "mono8":
        img = np.repeat(img, 3, axis=2)
    return np.ascontiguousarray(img[:, :, :3])
