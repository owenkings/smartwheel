"""ROS-independent PointCloud2 layout checks and owning point snapshots."""

from collections.abc import Mapping
import numpy as np

_DTYPES = {1: ("i1", 1), 2: ("u1", 1), 3: ("i2", 2), 4: ("u2", 2),
           5: ("i4", 4), 6: ("u4", 4), 7: ("f4", 4), 8: ("f8", 8)}


def copy_sdk_points(points) -> np.ndarray:
    """Copy Nx3 coordinates before the SDK can reuse its callback buffer.

    NaN/Inf are preserved for raw recording; downstream algorithms must reject
    them explicitly. Unit/axis conversion is deliberately not guessed here.
    """
    result = np.array(points, dtype=np.float64, copy=True, order="C")
    if result.ndim != 2 or result.shape[1] != 3:
        raise ValueError("points must have shape (N,3)")
    return result


def _get(obj, field):
    return obj[field] if isinstance(obj, Mapping) else getattr(obj, field)


def _nonnegative_int(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, np.integer)):
        raise TypeError(f"{name} must be integer")
    if value < 0:
        raise ValueError(f"{name} must be nonnegative")
    return int(value)


def validate_pointcloud2(message) -> dict:
    """Validate dict/ROS-object field layout, row padding and endianness.

    Reject overlapping fields, duplicate names and data-length mismatches. XYZ
    must each be scalar float32/64. Other valid fields are retained in layout.
    """
    width = _nonnegative_int(_get(message, "width"), "width")
    height = _nonnegative_int(_get(message, "height"), "height")
    point_step = _nonnegative_int(_get(message, "point_step"), "point_step")
    row_step = _nonnegative_int(_get(message, "row_step"), "row_step")
    if height < 1 or point_step < 1 or row_step < width * point_step:
        raise ValueError("invalid dimensions or point/row stride")
    endian = _get(message, "is_bigendian")
    if not isinstance(endian, bool):
        raise TypeError("is_bigendian must be bool")
    data = memoryview(_get(message, "data"))
    if data.nbytes != row_step * height:
        raise ValueError("data length does not equal row_step*height")
    fields = {}
    ranges = []
    for field in _get(message, "fields"):
        name = _get(field, "name")
        if not isinstance(name, str) or not name or name in fields:
            raise ValueError("empty or duplicate field name")
        offset = _nonnegative_int(_get(field, "offset"), "offset")
        datatype = _nonnegative_int(_get(field, "datatype"), "datatype")
        count = _nonnegative_int(_get(field, "count"), "count")
        if datatype not in _DTYPES or count == 0:
            raise ValueError("unsupported datatype or zero count")
        dtype, size = _DTYPES[datatype]
        end = offset + count * size
        if end > point_step or any(offset < old_end and old_start < end for old_start, old_end in ranges):
            raise ValueError("field exceeds point stride or overlaps another field")
        ranges.append((offset, end))
        fields[name] = {"offset": offset, "datatype": datatype, "count": count, "dtype": dtype}
    for name in ("x", "y", "z"):
        if name not in fields or fields[name]["count"] != 1 or fields[name]["datatype"] not in (7, 8):
            raise ValueError("XYZ fields must be scalar FLOAT32 or FLOAT64")
    return {"width": width, "height": height, "point_step": point_step, "row_step": row_step,
            "is_bigendian": endian, "fields": fields, "point_count": width * height}


def decode_pointcloud2(message) -> np.ndarray:
    layout = validate_pointcloud2(message)
    # First copy the callback-owned byte storage. numpy views below never alias
    # the SDK/ROS input, including when data was a mutable bytearray.
    storage = bytes(_get(message, "data"))
    result = np.empty((layout["point_count"], 3), dtype=np.float64)
    if layout["point_count"] == 0:
        return result
    for column, name in enumerate(("x", "y", "z")):
        field = layout["fields"][name]
        dtype = np.dtype((">" if layout["is_bigendian"] else "<") + field["dtype"])
        values = np.ndarray((layout["height"], layout["width"]), dtype=dtype, buffer=storage,
                            offset=field["offset"], strides=(layout["row_step"], layout["point_step"]))
        result[:, column] = values.reshape(-1)
    return result
