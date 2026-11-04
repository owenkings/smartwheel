import copy
import hashlib
from pathlib import Path
import sys
from types import SimpleNamespace as Obj

import numpy as np
import pytest

from wc_navigation.rebuild import (NonReferenceOriginError, pointcloud_xyz,
                                  profile_overrides, select_left_cloud, validate_frontend_row)


def cloud(codes=(2, 1, 2, 1), offsets=None, big=False):
    endian = '>' if big else '<'
    dtype = np.dtype({'names': ['x', 'y', 'z', 'offset', 'source'],
                      'formats': [endian + 'f4'] * 3 + [endian + 'f8', 'u1'],
                      'offsets': [0, 4, 8, 16, 24], 'itemsize': 32})
    records = np.zeros(len(codes), dtype=dtype)
    records['x'] = np.arange(len(codes)) + .25
    records['y'] = 2.0
    records['z'] = -.7
    records['source'] = codes
    records['offset'] = offsets if offsets is not None else [0 if n == 1 else -.002 for n in codes]
    # Non-field bytes are intentional provenance bytes and must survive selection.
    raw = records.view(np.uint8).reshape(len(codes), 32)
    raw[:, 28:32] = np.arange(len(codes))[:, None] + 31
    fields = [Obj(name=n, offset=o, datatype=t, count=1) for n, o, t in
              [('x', 0, 7), ('y', 4, 7), ('z', 8, 7), ('time_offset_s', 16, 8), ('source_code', 24, 2)]]
    return Obj(width=len(codes), height=1, point_step=32, row_step=len(codes) * 32,
               fields=fields, data=raw.tobytes(), is_bigendian=big, is_dense=False,
               header=Obj(frame_id='mapping_reference', stamp=Obj(sec=5, nanosec=30)))


@pytest.mark.parametrize('big', [False, True])
def test_selects_by_code_not_row_position_preserving_every_byte(big):
    original = cloud(big=big)
    before = bytes(original.data)
    selected, report = select_left_cloud(original)
    expected = np.frombuffer(before, dtype=np.uint8).reshape(4, 32)[[1, 3]].tobytes()
    assert bytes(selected.data) == expected
    assert bytes(original.data) == before and original.width == 4
    assert (selected.width, selected.height, selected.row_step) == (2, 1, 64)
    assert selected.header.frame_id == original.header.frame_id
    assert selected.header.stamp.nanosec == 30
    assert report['ray_origin_zero_valid'] and report['excluded_right_rows'] == 2
    assert report['derived_data_sha256'] == hashlib.sha256(expected).hexdigest()
    np.testing.assert_allclose(pointcloud_xyz(selected)[:, 0], [1.25, 3.25])


def test_nonreference_origin_is_not_cleared_or_retimed():
    original = cloud(offsets=[0, -.001, 0, -.001])
    with pytest.raises(NonReferenceOriginError) as error:
        select_left_cloud(original)
    assert error.value.metadata['left_time_offset_s'] == -.001
    selected, info = select_left_cloud(original, require_zero_offset=False)
    assert not info['ray_origin_zero_valid'] and info['left_time_offset_s'] == -.001
    assert selected.header.stamp.sec == original.header.stamp.sec


def test_single_left_without_source_fields_requires_explicit_frozen_provenance():
    original = cloud(codes=[1, 1, 1, 1])
    original.fields = original.fields[:3]
    config = {'mode': 'left', 'offline_source_selection': {'selected_sides': ['left']},
              'offline_lidar_selection': {'selected_sides': ['left']}}
    for bad in [None, {'mode': 'left'}, {'mode': 'all', 'offline_source_selection': {'selected_sides': ['left']}},
                {'mode': 'left', 'offline_source_selection': {'selected_sides': ['right']}}]:
        with pytest.raises(ValueError):
            select_left_cloud(original, source_config=bad)
    selected, report = select_left_cloud(original, source_config=config)
    assert bytes(selected.data) == bytes(original.data)
    assert selected.width == original.width and selected.height == original.height
    assert report['selection'] == 'VALIDATED_SINGLE_LEFT_FRONTEND'
    assert report['ray_origin_zero_valid'] and report['excluded_right_rows'] == 0


def test_partial_source_fields_never_infer_single_left():
    value = cloud(codes=[1, 1])
    value.fields = [f for f in value.fields if f.name != 'time_offset_s']
    with pytest.raises(ValueError):
        select_left_cloud(value, source_config={'mode': 'left', 'offline_source_selection': {'selected_sides': ['left']}})


@pytest.mark.parametrize('mutate', [
    lambda c: setattr(c, 'row_step', c.row_step + 4),
    lambda c: setattr(c, 'data', bytes(c.data)[:-1]),
    lambda c: setattr(c.fields[-1], 'datatype', 7),
    lambda c: setattr(c.fields[-2], 'offset', 4),
    lambda c: setattr(c.fields[0], 'count', 2),
    lambda c: setattr(c, 'is_bigendian', 1),
])
def test_malformed_layout_is_rejected(mutate):
    value = cloud()
    mutate(value)
    with pytest.raises(ValueError):
        select_left_cloud(value)


def test_unknown_codes_missing_left_and_mixed_offsets_rejected():
    for value in [cloud(codes=[2, 2]), cloud(codes=[1, 3]),
                  cloud(codes=[1, 1], offsets=[0, -.001]),
                  cloud(codes=[1, 2], offsets=[0, float('nan')])]:
        with pytest.raises(ValueError):
            select_left_cloud(value)


def test_invalid_xyz_rows_remain_invalid_and_are_counted():
    value = cloud(codes=[1, 2, 1])
    data = bytearray(value.data)
    np.ndarray((3,), dtype='<f4', buffer=data, strides=(32,))[2] = np.nan
    value.data = bytes(data)
    selected, info = select_left_cloud(value)
    assert np.isnan(pointcloud_xyz(selected)[1, 0])
    assert info['selected_rows'] == 2 and info['selected_finite_xyz_rows'] == 1


def row():
    return {'stamp_ns': 100, 'frame_id': 'mapping_reference', 'odom_parent': 'mapping_odom',
            'odom_child': 'mapping_reference', 'T_prior_reference': np.eye(4).tolist(),
            'cloud_cdr_file': '100.scan.cdr', 'odom_cdr_file': '100.odom.cdr',
            'cloud_cdr_sha256': 'a' * 64, 'cloud_data_sha256': 'b' * 64, 'odom_cdr_sha256': 'c' * 64}


def test_frontend_provenance_rejects_paths_clock_and_nonrigid_pose():
    assert validate_frontend_row(row()) == 100
    for key, value in [('cloud_cdr_file', '../100.cdr'), ('odom_cdr_file', '/tmp/1.cdr'),
                       ('cloud_data_sha256', 'not-a-sha'), ('stamp_ns', True),
                       ('frame_id', 'base_link'), ('T_prior_reference', np.zeros((4, 4)).tolist())]:
        bad = row()
        bad[key] = value
        with pytest.raises(ValueError):
            validate_frontend_row(bad)


def test_profile_variants_are_explicit_and_leave_input_unchanged():
    # The production profile helper is pure and kept as the parameter authority.
    reference = Path(__file__).resolve().parents[3] / 'reference/src'
    sys.path.insert(0, str(reference))
    from wc_runtime.mapping_profile import DEFAULT_PROFILE
    config = {'motion_model': 'planar_ekf', 'mode': 'all',
              'map_profile': copy.deepcopy(DEFAULT_PROFILE),
              'mounts': {'left': {'t_axle_lidar_m': [.323, .22875, .5915]}},
              'wheel_candidate': {'wheel_radius_m': .172}}
    before = copy.deepcopy(config)
    for variant, normals, height in [('left_normals_ray', 'true', .15),
                                      ('left_height_ray_05', 'false', .05),
                                      ('left_height_ray_15', 'false', .15)]:
        result = profile_overrides(config, variant)
        params = result['native_parameter_overrides']
        assert params['Grid/RayTracing'] == 'true' and params['Grid/3D'] == 'false'
        assert params['Grid/NormalsSegmentation'] == normals
        assert set(params) == {'Grid/3D', 'Grid/RayTracing', 'Grid/NormalsSegmentation'}
        resolved = result['resolved_native_profile_parameters']
        assert float(resolved['Grid/MaxGroundHeight']) == pytest.approx(-.7635 + height)
        assert resolved['Grid/MapFrameProjection'] == 'false'
        assert result['ground_model']['ground_height_validated'] is False
        assert result['navigation_validated'] is False
    assert config == before
    with pytest.raises(ValueError):
        profile_overrides(config, 'silent_cleanup')
