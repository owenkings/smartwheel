"""ROS-independent PointCloud2 byte transformation helpers."""

from __future__ import annotations

import numpy as np


def validate_xyz_buffer_layout(
    data,
    *,
    width: int,
    height: int,
    point_step: int,
    row_step: int,
    x_offset: int,
    y_offset: int,
    z_offset: int,
) -> None:
    """Reject permanent PointCloud2 layout errors before waiting for TF."""

    if width < 0 or height < 0 or point_step <= 0 or row_step < width * point_step:
        raise ValueError("invalid PointCloud2 dimensions")
    offsets = (int(x_offset), int(y_offset), int(z_offset))
    if any(offset < 0 or offset + 4 > point_step for offset in offsets):
        raise ValueError("XYZ field offset lies outside point_step")
    intervals = [(offset, offset + 4) for offset in offsets]
    if any(
        max(a_start, b_start) < min(a_end, b_end)
        for index, (a_start, a_end) in enumerate(intervals)
        for b_start, b_end in intervals[index + 1 :]
    ):
        raise ValueError("XYZ FLOAT32 fields overlap")
    if len(data) < row_step * height:
        raise ValueError("PointCloud2 data is shorter than row_step*height")


def transform_xyz_buffer(
    data,
    *,
    width: int,
    height: int,
    point_step: int,
    row_step: int,
    x_offset: int,
    y_offset: int,
    z_offset: int,
    bigendian: bool,
    rotation,
    translation,
) -> bytes:
    """Transform XYZ in a PointCloud2 buffer while preserving every other byte."""

    offsets = (int(x_offset), int(y_offset), int(z_offset))
    validate_xyz_buffer_layout(
        data,
        width=width,
        height=height,
        point_step=point_step,
        row_step=row_step,
        x_offset=offsets[0],
        y_offset=offsets[1],
        z_offset=offsets[2],
    )
    matrix = np.asarray(rotation, dtype=np.float64)
    shift = np.asarray(translation, dtype=np.float64).reshape(-1)
    if matrix.shape != (3, 3) or shift.shape != (3,):
        raise ValueError("rotation must be 3x3 and translation must have length 3")
    if not np.isfinite(matrix).all() or not np.isfinite(shift).all():
        raise ValueError("transform contains non-finite values")

    output = bytearray(data)
    endian = ">" if bigendian else "<"
    dtype = np.dtype(
        {
            "names": ["x", "y", "z"],
            "formats": [endian + "f4", endian + "f4", endian + "f4"],
            "offsets": list(offsets),
            "itemsize": point_step,
        }
    )
    points = np.ndarray(
        shape=(height, width),
        dtype=dtype,
        buffer=output,
        strides=(row_step, point_step),
    )
    xyz = np.column_stack(
        (
            points["x"].reshape(-1),
            points["y"].reshape(-1),
            points["z"].reshape(-1),
        )
    ).astype(np.float64, copy=False)
    finite = np.isfinite(xyz).all(axis=1)
    transformed = xyz.copy()
    transformed[finite] = xyz[finite] @ matrix.T + shift
    shaped = transformed.reshape(height, width, 3)
    points["x"] = shaped[:, :, 0]
    points["y"] = shaped[:, :, 1]
    points["z"] = shaped[:, :, 2]
    return bytes(output)
