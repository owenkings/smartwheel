import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2


def pack_rgb(red: np.ndarray, green: np.ndarray, blue: np.ndarray) -> np.ndarray:
    return (
        (red.astype(np.uint32) << 16)
        | (green.astype(np.uint32) << 8)
        | blue.astype(np.uint32)
    )


def colors_for_cloud(
    z: np.ndarray,
    packed_rgb: np.ndarray | None = None,
    intensity: np.ndarray | None = None,
) -> np.ndarray:
    if packed_rgb is not None:
        return np.asarray(packed_rgb, dtype=np.uint32)
    if intensity is not None:
        values = np.asarray(intensity, dtype=np.float64)
        finite = np.isfinite(values)
        gray = np.zeros(values.shape, dtype=np.uint8)
        if finite.any():
            low, high = np.percentile(values[finite], [1.0, 99.0])
            if high <= low:
                high = low + 1.0
            gray[finite] = np.clip((values[finite] - low) * 255.0 / (high - low), 0, 255).astype(np.uint8)
        return pack_rgb(gray, gray, gray)

    values = np.asarray(z, dtype=np.float64)
    finite = np.isfinite(values)
    normalized = np.zeros(values.shape, dtype=np.float64)
    if finite.any():
        low, high = np.percentile(values[finite], [1.0, 99.0])
        if high <= low:
            high = low + 1.0
        normalized[finite] = np.clip((values[finite] - low) / (high - low), 0.0, 1.0)
    red = np.clip(1.5 - np.abs(4.0 * normalized - 3.0), 0.0, 1.0)
    green = np.clip(1.5 - np.abs(4.0 * normalized - 2.0), 0.0, 1.0)
    blue = np.clip(1.5 - np.abs(4.0 * normalized - 1.0), 0.0, 1.0)
    return pack_rgb((red * 255).astype(np.uint8), (green * 255).astype(np.uint8), (blue * 255).astype(np.uint8))


def _packed_field(points: np.ndarray, name: str) -> np.ndarray:
    values = np.ascontiguousarray(points[name])
    if values.dtype.kind == "f" and values.dtype.itemsize == 4:
        return values.view(np.uint32)
    return values.astype(np.uint32)


def normalize_cloud(message: PointCloud2) -> tuple[np.ndarray, np.ndarray]:
    points = point_cloud2.read_points(message)
    names = set(points.dtype.names or ())
    if not {"x", "y", "z"}.issubset(names):
        raise ValueError("PointCloud2 must contain x, y and z fields")
    xyz = np.column_stack((points["x"], points["y"], points["z"])).astype(np.float32, copy=False)
    packed = None
    intensity = None
    if "rgb" in names:
        packed = _packed_field(points, "rgb")
    elif "rgba" in names:
        packed = _packed_field(points, "rgba") & np.uint32(0x00FFFFFF)
    elif "intensity" in names:
        intensity = np.asarray(points["intensity"])
    valid = np.isfinite(xyz).all(axis=1)
    xyz = xyz[valid]
    if packed is not None:
        packed = packed[valid]
    if intensity is not None:
        intensity = intensity[valid]
    return xyz, colors_for_cloud(xyz[:, 2], packed, intensity)


def xyzrgb_message(header, xyz: np.ndarray, packed_rgb: np.ndarray) -> PointCloud2:
    points = np.asarray(xyz, dtype=np.float32)
    colors = np.asarray(packed_rgb, dtype=np.uint32)
    if points.ndim != 2 or points.shape[1] != 3 or colors.shape != (points.shape[0],):
        raise ValueError("xyz must be Nx3 and packed_rgb must have N entries")
    records = np.empty(
        points.shape[0],
        dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("rgb", "<f4")],
    )
    records["x"] = points[:, 0]
    records["y"] = points[:, 1]
    records["z"] = points[:, 2]
    records["rgb"].view(np.uint32)[:] = colors
    message = PointCloud2()
    message.header = header
    message.height = 1
    message.width = records.shape[0]
    message.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="rgb", offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    message.is_bigendian = False
    message.point_step = 16
    message.row_step = 16 * records.shape[0]
    message.is_dense = bool(np.isfinite(points).all())
    message.data = records.tobytes()
    return message
