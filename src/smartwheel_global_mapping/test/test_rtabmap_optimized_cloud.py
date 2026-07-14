import struct
import zlib
from types import SimpleNamespace

import numpy as np
import pytest

from smartwheel_global_mapping.rtabmap_optimized_cloud_node import (
    assemble_optimized_cloud,
    decode_compressed_cv_mat,
)


def _compressed_cv_mat(values: np.ndarray) -> bytes:
    depths = {
        np.dtype(np.uint8): 0,
        np.dtype(np.int8): 1,
        np.dtype(np.uint16): 2,
        np.dtype(np.int16): 3,
        np.dtype(np.int32): 4,
        np.dtype(np.float32): 5,
        np.dtype(np.float64): 6,
    }
    rows, columns, channels = values.shape
    cv_type = depths[values.dtype] + ((channels - 1) << 3)
    return zlib.compress(values.tobytes()) + struct.pack("iii", rows, columns, cv_type)


def _vector(x=0.0, y=0.0, z=0.0):
    return SimpleNamespace(x=x, y=y, z=z)


def _quaternion(z=0.0, w=1.0):
    return SimpleNamespace(x=0.0, y=0.0, z=z, w=w)


def _map_data(scan: np.ndarray, scan_format: int = 6):
    pose = SimpleNamespace(position=_vector(1.0, 2.0, 0.0), orientation=_quaternion())
    local = SimpleNamespace(translation=_vector(0.5, 0.0, 0.0), rotation=_quaternion())
    sensor = SimpleNamespace(
        laser_scan_compressed=_compressed_cv_mat(scan),
        laser_scan_format=scan_format,
        laser_scan_local_transform=local,
    )
    node = SimpleNamespace(id=7, data=sensor)
    graph = SimpleNamespace(poses_id=[7], poses=[pose])
    return SimpleNamespace(graph=graph, nodes=[node])


def test_decode_compressed_multichannel_cv_mat():
    source = np.arange(24, dtype=np.float32).reshape(1, 6, 4)
    decoded = decode_compressed_cv_mat(_compressed_cv_mat(source))
    np.testing.assert_array_equal(decoded, source)


def test_assemble_uses_optimized_pose_and_scan_local_transform():
    scan = np.array([[[1.0, 0.0, 0.0, 9.0], [0.0, 1.0, 0.0, 8.0]]], dtype=np.float32)
    points, nodes = assemble_optimized_cloud(_map_data(scan), voxel_size_m=0.0)
    assert nodes == 1
    np.testing.assert_allclose(points, [[2.5, 2.0, 0.0], [1.5, 3.0, 0.0]])


def test_assemble_rejects_unsupported_scan_format():
    scan = np.array([[[1.0, 0.0]]], dtype=np.float32)
    with pytest.raises(ValueError, match="unsupported"):
        assemble_optimized_cloud(_map_data(scan, scan_format=1))


def test_decode_rejects_corrupt_payload():
    with pytest.raises(ValueError):
        decode_compressed_cv_mat(b"not-a-valid-payload")
