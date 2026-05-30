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
    r = np.linalg.norm(xyz, axis=1)
    mask = np.isfinite(r) & (r >= min_range) & (r <= max_range)
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


_XYZI_FIELDS = None
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


def make_xyzi_cloud(header, xyz: np.ndarray, inten=None) -> "PointCloud2":
    n = int(xyz.shape[0])
    data = np.zeros((n, 4), dtype=np.float32)
    if n:
        data[:, :3] = xyz.astype(np.float32)
        if inten is not None:
            data[:, 3] = inten.astype(np.float32)
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
