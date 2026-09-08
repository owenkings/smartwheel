import struct
import zlib
from types import SimpleNamespace

import numpy as np
import pytest

from smartwheel_global_mapping.rtabmap_optimized_cloud_node import (
    RtabmapOptimizedCloudNode,
    assemble_optimized_cloud,
    assemble_optimized_cloud_with_intensity,
    assemble_optimized_trajectory,
    decode_compressed_cv_mat,
)
from smartwheel_sensor_api import pointcloud2_from_xyz
from std_msgs.msg import Header


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
    node = SimpleNamespace(id=7, stamp=12.25, data=sensor)
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


def test_assemble_preserves_compressed_xyzi_intensity_with_voxel_indices():
    scan = np.array(
        [[[1.01, 0.0, 0.0, 11.0], [1.02, 0.0, 0.0, 22.0], [0.0, 1.0, 0.0, 33.0]]],
        dtype=np.float32,
    )
    points, intensity, nodes = assemble_optimized_cloud_with_intensity(
        _map_data(scan), voxel_size_m=0.05
    )
    assert nodes == 1
    assert intensity is not None
    # The first sample in each deterministic voxel carries its matching amp.
    np.testing.assert_allclose(points, [[2.51, 2.0, 0.0], [1.5, 3.0, 0.0]])
    np.testing.assert_allclose(intensity, [11.0, 33.0])


def test_assemble_falls_back_to_raw_pointcloud2_and_keeps_intensity():
    header = Header()
    header.frame_id = "xtm60_right_link"
    raw = pointcloud2_from_xyz(
        header,
        np.asarray([[1.0, 0.0, 0.0], [0.0, 2.0, 0.0]], dtype=np.float64),
        np.asarray([101.0, 202.0], dtype=np.float32),
    )
    pose = SimpleNamespace(position=_vector(0.0, 0.0, 0.0), orientation=_quaternion())
    local = SimpleNamespace(translation=_vector(), rotation=_quaternion())
    sensor = SimpleNamespace(
        laser_scan_compressed=b"",
        laser_scan=raw,
        laser_scan_local_transform=local,
        laser_scan_format=0,
    )
    data = SimpleNamespace(
        graph=SimpleNamespace(poses_id=[7], poses=[pose]),
        nodes=[SimpleNamespace(id=7, data=sensor)],
    )
    points, intensity, nodes = assemble_optimized_cloud_with_intensity(data, 0.0)
    assert nodes == 1
    np.testing.assert_allclose(points, [[1.0, 0.0, 0.0], [0.0, 2.0, 0.0]])
    np.testing.assert_allclose(intensity, [101.0, 202.0])


def test_assemble_rejects_mixed_intensity_provenance():
    xyzi = np.array([[[1.0, 0.0, 0.0, 5.0]]], dtype=np.float32)
    first = _map_data(xyzi)
    xyz = np.array([[[0.0, 1.0, 0.0]]], dtype=np.float32)
    second = _map_data(xyz, scan_format=5)
    second.nodes[0].id = 8
    second.graph.poses_id = [7, 8]
    second.graph.poses = [first.graph.poses[0], first.graph.poses[0]]
    data = SimpleNamespace(
        graph=second.graph,
        nodes=[first.nodes[0], second.nodes[0]],
    )
    with pytest.raises(ValueError, match="mixes intensity"):
        assemble_optimized_cloud_with_intensity(data, voxel_size_m=0.0)


def test_xyz_only_cloud_does_not_fabricate_zero_intensity_field():
    header = Header()
    header.frame_id = "map"
    message = RtabmapOptimizedCloudNode._cloud_message(
        header,
        np.asarray([[1.0, 2.0, 3.0]], dtype=np.float32),
        None,
    )
    assert [field.name for field in message.fields] == ["x", "y", "z"]


def test_optimized_trajectory_uses_node_timestamp_and_graph_pose():
    scan = np.array([[[1.0, 0.0, 0.0, 5.0]]], dtype=np.float32)
    data = _map_data(scan)
    samples = assemble_optimized_trajectory(data)
    assert len(samples) == 1
    assert samples[0][0] == 7
    assert samples[0][1] == pytest.approx(12.25)
    assert samples[0][2] is data.graph.poses[0]

    header = Header()
    header.frame_id = "map"
    message = RtabmapOptimizedCloudNode._path_message(header, samples)
    assert len(message.poses) == 1
    assert message.poses[0].header.stamp.sec == 12
    assert message.poses[0].header.stamp.nanosec == 250_000_000
    assert message.poses[0].pose.position.x == pytest.approx(1.0)


def test_optimized_trajectory_rejects_missing_or_duplicate_timestamps():
    scan = np.array([[[1.0, 0.0, 0.0, 5.0]]], dtype=np.float32)
    first = _map_data(scan)
    first.nodes[0].stamp = 0.0
    with pytest.raises(ValueError, match="invalid timestamp"):
        assemble_optimized_trajectory(first)

    first = _map_data(scan)
    second = _map_data(scan)
    second.nodes[0].id = 8
    second.nodes[0].stamp = first.nodes[0].stamp
    second.graph.poses_id = [7, 8]
    second.graph.poses = [first.graph.poses[0], first.graph.poses[0]]
    data = SimpleNamespace(graph=second.graph, nodes=[first.nodes[0], second.nodes[0]])
    with pytest.raises(ValueError, match="strictly increasing"):
        assemble_optimized_trajectory(data)


def test_assemble_rejects_duplicate_pose_ids():
    scan = np.array([[[1.0, 0.0, 0.0, 5.0]]], dtype=np.float32)
    data = _map_data(scan)
    data.graph.poses_id = [7, 7]
    data.graph.poses = [data.graph.poses[0], data.graph.poses[0]]
    with pytest.raises(ValueError, match="duplicate"):
        assemble_optimized_cloud_with_intensity(data, voxel_size_m=0.0)


def test_assemble_rejects_nonfinite_pose_translation():
    scan = np.array([[[1.0, 0.0, 0.0, 5.0]]], dtype=np.float32)
    data = _map_data(scan)
    data.graph.poses[0].position.x = float("nan")
    with pytest.raises(ValueError, match="non-finite translation"):
        assemble_optimized_cloud_with_intensity(data, voxel_size_m=0.0)


def test_assemble_rejects_unsupported_scan_format():
    scan = np.array([[[1.0, 0.0]]], dtype=np.float32)
    with pytest.raises(ValueError, match="unsupported"):
        assemble_optimized_cloud(_map_data(scan, scan_format=1))


def test_decode_rejects_corrupt_payload():
    with pytest.raises(ValueError):
        decode_compressed_cv_mat(b"not-a-valid-payload")
