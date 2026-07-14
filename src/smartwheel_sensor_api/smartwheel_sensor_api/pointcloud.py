import math
from typing import Iterable, Optional, Sequence

import numpy as np


SUPPORTED_UNITS = {"m": 1.0, "mm": 0.001}


def convert_points_to_metres(points: np.ndarray, unit: str) -> np.ndarray:
    if unit not in SUPPORTED_UNITS:
        raise ValueError(f"unsupported point unit: {unit}")
    array = np.asarray(points, dtype=np.float64)
    if array.ndim != 2 or array.shape[1] != 3:
        raise ValueError("points must have shape (N, 3)")
    converted = array * SUPPORTED_UNITS[unit]
    if not np.all(np.isfinite(converted)):
        raise ValueError("point cloud contains non-finite coordinates")
    return converted


def validate_pointcloud_fields(message, required: Sequence[str] = ("x", "y", "z")) -> None:
    names = {field.name for field in message.fields}
    missing = [name for name in required if name not in names]
    if missing:
        raise ValueError(f"PointCloud2 missing required fields: {', '.join(missing)}")
    if not message.header.frame_id:
        raise ValueError("PointCloud2 frame_id is empty")


def pointcloud2_from_xyz(header, points: np.ndarray, intensity: Optional[np.ndarray] = None):
    from sensor_msgs.msg import PointField
    from sensor_msgs_py import point_cloud2

    xyz = np.asarray(points, dtype=np.float32)
    if xyz.ndim != 2 or xyz.shape[1] != 3:
        raise ValueError("points must have shape (N, 3)")
    if intensity is None:
        intensity = np.zeros(xyz.shape[0], dtype=np.float32)
    intensity = np.asarray(intensity, dtype=np.float32).reshape(-1)
    if intensity.shape[0] != xyz.shape[0]:
        raise ValueError("intensity length must match points")
    fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    values = np.column_stack((xyz, intensity)).astype(np.float32)
    return point_cloud2.create_cloud(header, fields, values.tolist())


def pointcloud2_to_xyz(message) -> tuple[np.ndarray, Optional[np.ndarray]]:
    from sensor_msgs_py import point_cloud2

    validate_pointcloud_fields(message)
    names = {field.name for field in message.fields}
    read_names = ("x", "y", "z", "intensity") if "intensity" in names else ("x", "y", "z")
    values = point_cloud2.read_points(message, field_names=read_names, skip_nans=False)
    if hasattr(values, "dtype") and values.dtype.names:
        xyz = np.column_stack((values["x"], values["y"], values["z"])).astype(np.float64)
        intensity = np.asarray(values["intensity"], dtype=np.float32) if "intensity" in read_names else None
    else:
        array = np.asarray(list(values), dtype=np.float64)
        if array.size == 0:
            return np.empty((0, 3), dtype=np.float64), None
        xyz = array[:, :3]
        intensity = array[:, 3].astype(np.float32) if array.shape[1] > 3 else None
    finite = np.all(np.isfinite(xyz), axis=1)
    nonzero = np.linalg.norm(xyz, axis=1) > 0.0
    keep = finite & nonzero
    return xyz[keep], intensity[keep] if intensity is not None else None


def apply_transform(points: np.ndarray, matrix: np.ndarray) -> np.ndarray:
    xyz = np.asarray(points, dtype=np.float64)
    transform = np.asarray(matrix, dtype=np.float64)
    if xyz.ndim != 2 or xyz.shape[1] != 3:
        raise ValueError("points must have shape (N, 3)")
    if transform.shape != (4, 4) or not np.all(np.isfinite(transform)):
        raise ValueError("transform must be a finite 4x4 matrix")
    return xyz @ transform[:3, :3].T + transform[:3, 3]


def voxel_downsample(
    points: np.ndarray, voxel_size_m: float, intensity: Optional[np.ndarray] = None
) -> tuple[np.ndarray, Optional[np.ndarray]]:
    xyz = np.asarray(points, dtype=np.float64)
    if voxel_size_m <= 0.0 or xyz.shape[0] == 0:
        return xyz.copy(), None if intensity is None else np.asarray(intensity).copy()
    keys = np.floor(xyz / voxel_size_m).astype(np.int64)
    _, first = np.unique(keys, axis=0, return_index=True)
    first.sort()
    out_intensity = None if intensity is None else np.asarray(intensity)[first]
    return xyz[first], out_intensity


def rpy_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ],
        dtype=np.float64,
    )


def transform_matrix(xyz: Iterable[float], rpy: Iterable[float]) -> np.ndarray:
    matrix = np.eye(4, dtype=np.float64)
    matrix[:3, :3] = rpy_matrix(*list(rpy))
    matrix[:3, 3] = np.asarray(list(xyz), dtype=np.float64)
    return matrix

