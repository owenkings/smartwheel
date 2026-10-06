"""Synthetic mask geometry, layout evidence, and immutable source contracts."""
import copy
import struct
from types import SimpleNamespace as NS

import numpy as np
import pytest

from wc_runtime import mapping_filter as f
from wc_runtime.mapping_input import build_cloud, mount_transforms
from wc_sensors.pointcloud import decode_pointcloud2
from test_mapping_input import config, source


def settings(mode='left'):
    value = config(mode)
    value.update(mapping_enabled=True, continuous_mapping=True,
                 wheel_candidate={'wheel_radius_m': .165, 'provenance': 'TEST_ONLY'},
                 cloud_filter={'enabled': True})
    return value


def frame(xyz, *, side='left', shape=None, flags=None, sequence=0, host=None):
    value = source(side, sequence, **({'host': host} if host is not None else {}))
    xyz = np.asarray(xyz, dtype=np.float32).reshape(-1, 3)
    data = np.zeros((len(xyz), 4), dtype='<f4')
    data[:, :3] = xyz
    data[:, 3] = np.arange(len(xyz)) + 101
    height, width = (1, len(xyz)) if shape is None else shape
    value.cloud.width, value.cloud.height = width, height
    value.cloud.row_step = width*16
    value.cloud.data = data.tobytes()
    value.raw_count = len(xyz)
    value.valid_count = int(np.isfinite(xyz).all(axis=1).sum())
    value.cloud.is_dense = value.valid_count == value.raw_count
    value.diagnostic_flags = flags or []
    return value


def apply(value, frames):
    transforms = mount_transforms(value, None)
    members = [(message, {'raw_key': ['fixture', message.side, message.frame_sequence]}) for message in frames]
    before = build_cloud(members, transforms)
    after, report = f.filter_cloud(before, members, transforms, value)
    return before, after, report


@pytest.mark.parametrize('mapping_enabled,block,reason', [
    (True, None, 'SWITCH_OFF'), (True, {'enabled': False}, 'SWITCH_OFF'),
    (False, {'enabled': True}, 'PREVIEW_PRESERVES_SOURCE')])
def test_explicit_off_and_preview_are_byte_exact(mapping_enabled, block, reason):
    value = settings()
    value['mapping_enabled'] = mapping_enabled
    if block is None:
        value.pop('cloud_filter')
    else:
        value['cloud_filter'] = block
    message = frame([[0, 0, 0], [25, 0, 8], [np.nan, 0, 0]])
    before, after, report = apply(value, [message])
    assert after is before
    assert after.data == message.cloud.data
    assert not report['enabled'] and report['disabled_reason'] == reason


@pytest.mark.parametrize('override', [
    {'bogus': 1}, {'enabled': 1}, {'range': None}, {'range': {'min_m': -1}},
    {'range': {'min_m': 15, 'max_m': 15}}, {'ground_height': {'max_m': float('nan')}},
    {'organized_support': {'min_neighbors': 9}}, {'organized_support': {'min_neighbors': True}},
    {'organized_support': {'distance_scale': -.1}}, {'ground_height': {'extra': True}},
])
def test_invalid_parameters_fail_before_use(override):
    value = settings()
    value['cloud_filter'].update(override)
    with pytest.raises(ValueError, match='CLOUD_FILTER_'):
        f.resolve_cloud_filter(value)


def test_ground_requires_an_explicit_wheel_radius_and_reports_assumption():
    value = settings()
    del value['wheel_candidate']
    with pytest.raises(ValueError, match='EXPLICIT_WHEEL_RADIUS_REQUIRED'):
        f.resolve_cloud_filter(value)
    value['cloud_filter']['ground_height'] = {'enabled': False}
    assert f.resolve_cloud_filter(value)['enabled']


def test_source_range_rejects_zero_nonfinite_near_far_but_retains_rows_and_intensity():
    value = settings()
    value['cloud_filter'].update(organized_support={'enabled': False}, ground_height={'enabled': False})
    message = frame([[0, 0, 0], [np.nan, 0, 0], [.2, 0, 0], [1, 0, 0], [15, 0, 0], [15.01, 0, 0]])
    original = message.cloud.data
    before, after, report = apply(value, [message])
    keep = np.isfinite(decode_pointcloud2(after)).all(axis=1)
    assert keep.tolist() == [False, False, False, True, True, False]
    assert after.width == before.width == 6 and after.point_step == before.point_step
    assert bytes(message.cloud.data) == original
    for n in range(6):
        assert after.data[n*16+12:n*16+16] == original[n*16+12:n*16+16]
    assert report['sources'][0]['finite_points'] == 5
    assert report['sources'][0]['nonzero_points'] == 4
    assert report['sources'][0]['after_range'] == report['retained_points'] == 2
    assert not after.is_dense


def test_organized_neighbors_remove_island_without_wrapping_or_iterative_erosion():
    xyz = np.full((4, 5, 3), np.nan)
    xyz[:2, :2] = [[[1, 0, 0], [1, .01, 0]], [[1, 0, .01], [1, .01, .01]]]
    # Two adjacent isolated pixels only support one another, below min=2.
    xyz[3, 3:] = [[1, .01, .01], [1, .02, .01]]
    value = settings()
    value['cloud_filter']['ground_height'] = {'enabled': False}
    _, after, report = apply(value, [frame(xyz, shape=(4, 5))])
    mask = np.isfinite(decode_pointcloud2(after)).all(axis=1).reshape(4, 5)
    assert mask[:2, :2].all() and mask.sum() == 4
    assert report['sources'][0]['support_status'] == 'APPLIED'
    assert report['sources'][0]['layout_evidence'] == 'POINTCLOUD2_ORGANIZED'


def test_pixel_boundary_is_not_a_neighbor_and_range_rejected_pixels_cannot_support():
    xyz = np.full((2, 4, 3), np.nan)
    xyz[0, 3] = [1, 0, 0]
    xyz[1, 0] = [1, .01, 0]
    value = settings()
    value['cloud_filter'].update(ground_height={'enabled': False},
                                 organized_support={'min_neighbors': 1})
    _, after, report = apply(value, [frame(xyz, shape=(2, 4))])
    assert not np.isfinite(decode_pointcloud2(after)).any()
    assert not report['publishable']
    xyz = np.array([[[1., 0, 0], [1.01, 0, 0]], [[1.01, 0, 0], [1.01, 0, 0]]])
    value['cloud_filter']['range'] = {'max_m': 1.005}
    _, _, report = apply(value, [frame(xyz, shape=(2, 2))])
    assert report['sources'][0]['after_range'] == 1
    assert report['sources'][0]['after_support'] == 0


@pytest.mark.parametrize('flags,status,applied', [
    ([], 'LAYOUT_UNAVAILABLE', False),
    (['sdk_width=3', 'sdk_height=2', 'sdk_point_order=row_major'], 'APPLIED', True),
    (['sdk_width=160', 'sdk_height=60', 'sdk_point_order=row_major'], 'LAYOUT_METADATA_INVALID', False),
    (['sdk_width=3', 'sdk_height=2'], 'LAYOUT_METADATA_INVALID', False),
    (['sdk_width=3', 'sdk_width=3', 'sdk_height=2', 'sdk_point_order=row_major'], 'LAYOUT_METADATA_INVALID', False),
])
def test_flat_point_count_never_invents_layout(flags, status, applied):
    value = settings()
    value['cloud_filter']['ground_height'] = {'enabled': False}
    _, _, report = apply(value, [frame([[1, 0, 0]]*5+[[5, 0, 0]], flags=flags)])
    row = report['sources'][0]
    assert row['support_status'] == status
    assert row['retained_points'] == (5 if applied else 6)


def test_adaptive_neighbor_distance_keeps_far_wall_but_rejects_depth_step():
    value = settings()
    value['cloud_filter']['ground_height'] = {'enabled': False}
    wall = np.array([[[10., -.1, 0], [10., 0, 0], [10., .1, 0]],
                     [[10., -.1, .1], [10., 0, .1], [12., .1, .1]]])
    _, after, report = apply(value, [frame(wall, shape=(2, 3))])
    assert report['retained_points'] == 5
    assert np.isnan(decode_pointcloud2(after)[-1]).all()


@pytest.mark.parametrize('mode', ['left', 'right', 'all'])
def test_ground_height_uses_axle_rotation_translation_and_radius_for_each_side(mode):
    value = settings(mode)
    value['cloud_filter']['organized_support'] = {'enabled': False}
    # Choose points in the common axle frame, then generate each real sensor frame.
    heights = np.array([-.2, 0, 2., 2.6])
    axle = np.column_stack([np.full(4, 2.), np.zeros(4), heights-.165])
    frames = []
    for side in (('left', 'right') if mode == 'all' else (mode,)):
        mount = value['mounts'][side]
        xyz = (axle-np.asarray(mount['t_axle_lidar_m'])) @ np.asarray(mount['R_axle_lidar'])
        frames.append(frame(xyz, side=side))
    _, after, report = apply(value, frames)
    assert np.isfinite(decode_pointcloud2(after)).all(axis=1).tolist() == [False, True, True, False]*len(frames)
    assert report['ground_reference']['basis'] == f.GROUND_BASIS
    assert report['ground_reference']['ground_truth_validated'] is False
    assert report['ground_reference']['wheel_parameter_provenance'] == 'TEST_ONLY'


def test_dual_range_is_per_sensor_not_shifted_base_and_side_time_bytes_survive():
    value = settings('all')
    value['mounts']['right']['t_axle_lidar_m'] = [20, -.3125, .52]
    value['cloud_filter'].update(organized_support={'enabled': False}, ground_height={'enabled': False})
    left = frame([[1, 0, 0], [20, 0, 0]])
    right = frame([[1, 0, 0], [20, 0, 0]], side='right', host=left.host_monotonic_ns+20_000_000)
    before, after, report = apply(value, [left, right])
    assert report['publishable'] and report['retained_points'] == 2
    assert np.isfinite(decode_pointcloud2(after)).all(axis=1).tolist() == [True, False, True, False]
    for i in range(4):
        assert after.data[i*32+12:(i+1)*32] == before.data[i*32+12:(i+1)*32]
    assert struct.unpack_from('<d', after.data, 16)[0] == -.02
    assert after.data[24] == 1 and after.data[2*32+24] == 2


def test_one_empty_side_rejects_entire_dual_pair_and_keeps_row_identity():
    value = settings('all')
    value['cloud_filter'].update(organized_support={'enabled': False}, ground_height={'enabled': False})
    _, after, report = apply(value, [frame([[1, 0, 0]]), frame([[30, 0, 0]], side='right')])
    assert after.width == 2 and report['retained_points'] == 1
    assert not report['publishable'] and report['empty_sides'] == ['right']
    assert report['skip_reason'] == 'FILTERED_SOURCE_EMPTY'
    assert report['sources'][1]['output_row_start'] == 1


def test_big_endian_float64_xyz_with_row_padding_preserves_every_non_xyz_byte():
    value = settings()
    value['cloud_filter'].update(organized_support={'enabled': False}, ground_height={'enabled': False})
    message = frame([[1, 0, 0], [30, 0, 0]])
    cloud = message.cloud
    cloud.height, cloud.width, cloud.point_step, cloud.row_step = 2, 1, 32, 36
    cloud.fields = [NS(name=name, offset=i*8, datatype=8, count=1) for i, name in enumerate(('x','y','z','intensity'))]
    cloud.is_bigendian = True
    cloud.data = struct.pack('>4d', 1, 0, 0, 101)+b'abcd'+struct.pack('>4d', 30, 0, 0, 102)+b'efgh'
    original = cloud.data
    _, after, _ = apply(value, [message])
    assert after.data[:36] == original[:36]
    assert after.data[60:] == original[60:]
    assert np.isnan(decode_pointcloud2(after)[1]).all()
    assert message.cloud.data == original
