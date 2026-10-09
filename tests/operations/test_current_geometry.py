"""Current measured geometry configuration and strict original-CAD update evidence.

These tests open project snapshots only; they never start ROS or hardware.
"""
import copy
import hashlib
import json
from pathlib import Path
import shutil
import uuid

import numpy as np
import pytest

from wc_runtime.calibration_geometry import verify_geometry_sources
from wc_runtime.hardware_setup import resolve_hardware_setup


ROOT = Path(__file__).resolve().parents[2]
OVERRIDE_ID = next(row['id'] for row in json.loads((ROOT/'config/hardware_setup.json').read_text(encoding='utf-8'))['source_documents']
                   if row['evidence_type'] == 'GEOMETRY_MEASUREMENT' and row['snapshot_path'].endswith('.json'))


def current():
    return json.loads((ROOT/'config/hardware_setup.json').read_text(encoding='utf-8'))


def historical():
    return json.loads((ROOT/'tests/fixtures/hardware_unknown.json').read_text(encoding='utf-8'))


def test_current_geometry_is_source_verified_and_all_same_box_dimensions_are_preserved():
    setup = current(); before = copy.deepcopy(setup)
    report = verify_geometry_sources(setup, ROOT)
    assert report['status'] == 'PASS'
    assert setup == before
    proof = report['sources'][0]['layout_override']
    assert proof['reference_points_checked'] == 11
    assert proof['auxiliary_points_checked'] == 1
    assert proof['left_shift_mm'] == [0, 10.9, 0]
    assert proof['right_shift_mm'] == [0, -10.9, 0]
    old = historical()['mechanical_reference']
    new = setup['mechanical_reference']
    for prior, actual in zip(old['reference_points']+old['auxiliary_points'], new['reference_points']+new['auxiliary_points']):
        expected = np.array(prior['position_mm'], dtype=float)
        expected[1] += 10.9 if prior['sensor_frame'] == 'imu_native' or expected[1] > 0 else -10.9
        assert np.allclose(actual['position_mm'], expected)
        assert actual['point_kind'] == prior['point_kind']
        assert actual['status'] == 'CAD_NOMINAL'
        assert actual.get('envelope_dimensions_mm') == prior.get('envelope_dimensions_mm')
        assert actual.get('nominal_view_direction_M') == prior.get('nominal_view_direction_M')
        assert any(e['source_id'] == OVERRIDE_ID for e in actual['evidence'])
    measures = {row['id']: row['value_mm'] for row in new['measurements']}
    assert measures['outer_front_top_corner_spacing'] == 623
    assert measures['inner_box_wall_gap'] == 415
    assert measures['nominal_lidar_face_spacing'] == 457.5
    assert measures['clamp_center_spacing'] == 514
    assert not {'clamp_seat_faces', 'clamp_plate_faces', 'lidar_window_spacing'} & measures.keys()


def test_current_known_axle_and_imu_rotation_enable_no_invented_lidar_origin_or_chip_translation():
    setup = current(); result = resolve_hardware_setup(setup)
    rows = {row['id']: row for row in setup['data_transforms']}
    axle = rows['axle_from_mounting_M']
    assert axle['translation']['value_m'] == [.273, 0, .4582]
    assert axle['translation']['status'] == 'USER_MEASURED_EXPERIMENT'
    assert axle['rotation']['value_matrix'] == np.eye(3).tolist()
    assert np.allclose(result['imu_mount']['R_axle_imu'], np.eye(3))
    assert result['imu_mount']['t_axle_imu_m'] is None
    assert result['imu_mount']['translation_used'] is False
    assert result['mounts'] == {}
    assert setup['assembly']['confirmed'] is True
    for mode in ('left', 'right', 'all'):
        gate = result['geometry_capabilities']['wheel_imu_motion_'+mode]
        assert gate['status'] == 'BLOCKED'
        assert len(gate['reasons']) == (2 if mode == 'all' else 1)
        assert all('T_axle_lidar_' in reason for reason in gate['reasons'])
    for capability in ('native_preview', 'native_capture', 'lidar_relative_calibration', 'imu_native_bias_calibration'):
        assert result['geometry_capabilities'][capability]['status'] == 'AVAILABLE'
    assert result['geometry_capabilities']['dual_lidar_static_fusion']['status'] == 'BLOCKED'
    assert setup['wheel_odometry']['wheel_radius_m'] == .185
    assert setup['wheel_odometry']['track_width_m'] == .610
    assert setup['wheel_odometry']['register_to_wheel_rpm'] == .112
    assert setup['manual_controls'] == historical()['manual_controls']
    assert setup['legacy_profile']['active'] is False


@pytest.mark.parametrize('mutation', [
    lambda s: s['mechanical_reference']['reference_points'][0]['position_mm'].__setitem__(1, 217.85),
    lambda s: s['mechanical_reference']['reference_points'][0]['envelope_center_mm'].__setitem__(1, 217.85),
    lambda s: s['mechanical_reference']['reference_points'][0]['envelope_dimensions_mm'].__setitem__(0, 42),
    lambda s: s['mechanical_reference']['reference_points'][0]['nominal_view_direction_M'].__setitem__(0, -1),
    lambda s: s['mechanical_reference']['reference_points'][0]['evidence'].pop(),
    lambda s: s['mechanical_reference']['cad_to_M']['right']['T_M_CAD_mm'][1].__setitem__(3, -379.5),
    lambda s: s['mechanical_reference']['auxiliary_points'][0]['position_mm'].__setitem__(1, 268.93),
    lambda s: s['mechanical_reference']['measurements'][2].__setitem__('value_mm', 394),
    lambda s: s['data_transforms'][0]['translation']['value_m'].__setitem__(2, .458),
])
def test_current_config_cannot_silently_diverge_from_the_measured_update(mutation):
    setup = current(); mutation(setup)
    report = verify_geometry_sources(setup, ROOT)
    assert report['status'] == 'FAIL'
    assert report['sources'][0]['reason']


@pytest.fixture
def snapshot_root():
    parent = Path(__file__).resolve().parent
    root = parent/('.current_geometry_'+uuid.uuid4().hex)
    root.mkdir()
    try:
        setup = current()
        for source in setup['source_documents']:
            destination = root/source['snapshot_path']
            destination.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(ROOT/source['snapshot_path'], destination)
        yield root, setup
    finally:
        resolved = root.resolve()
        assert resolved.parent == parent and resolved.name.startswith('.current_geometry_')
        shutil.rmtree(resolved)


@pytest.mark.parametrize('mutation', [
    lambda p: p['layout_override'].__setitem__('base_sha256', '0'*64),
    lambda p: p.__setitem__('assembly_revision', 'OTHER_INSTALLATION'),
    lambda p: p['layout_override']['delta_M_mm']['left'].__setitem__(0, 1),
    lambda p: p['layout_override']['delta_M_mm']['right'].__setitem__(1, -9.9),
    lambda p: p['layout_override'].__setitem__('old_corner_spacing_mm', 610),
    lambda p: p['current_mechanical_geometry']['reference_points'][0]['position_mm'].__setitem__(0, 51),
    lambda p: p['current_mechanical_geometry']['reference_points'][0]['envelope_center_mm'].__setitem__(1, 220),
    lambda p: p['current_mechanical_geometry']['cad_to_M']['left'][1].__setitem__(3, 129.6),
    lambda p: p['current_mechanical_geometry']['auxiliary_points'][0]['position_mm'].__setitem__(1, 280.83),
    lambda p: p['raw_document'].__setitem__('sha256', '0'*64),
    lambda p: p['latest_confirmations'].__setitem__('C4_H_reference', 'TIRE_MIDPLANE'),
    lambda p: p['latest_confirmations'].__setitem__('closure_mm', 622),
])
def test_rehashing_a_forged_update_does_not_bypass_the_original_cad_translation_chain(snapshot_root, mutation):
    root, setup = snapshot_root
    source = next(row for row in setup['source_documents'] if row['id'] == OVERRIDE_ID)
    path = root/source['snapshot_path']
    payload = json.loads(path.read_text(encoding='utf-8')); mutation(payload)
    path.write_text(json.dumps(payload, ensure_ascii=False), encoding='utf-8')
    source['sha256'] = hashlib.sha256(path.read_bytes()).hexdigest()
    assert verify_geometry_sources(setup, root)['status'] == 'FAIL'


def test_multiple_current_overrides_of_one_cad_are_ambiguous_and_fail(snapshot_root):
    root, setup = snapshot_root
    duplicate = copy.deepcopy(next(row for row in setup['source_documents'] if row['id'] == OVERRIDE_ID))
    duplicate['id'] += '_duplicate'
    setup['source_documents'].append(duplicate)
    report = verify_geometry_sources(setup, root)
    assert report['status'] == 'FAIL'
    assert 'multiple current layout overrides' in report['sources'][0]['reason']


def test_changed_raw_measurement_bytes_and_missing_override_cannot_pass(snapshot_root):
    root, setup = snapshot_root
    source = next(row for row in setup['source_documents']
                  if row['evidence_type'] == 'GEOMETRY_MEASUREMENT' and row['snapshot_path'].endswith('.md'))
    path = root/source['snapshot_path']; path.write_bytes(path.read_bytes()+b'changed')
    assert verify_geometry_sources(setup, root)['status'] == 'FAIL'
    setup = current()
    setup['source_documents'] = [row for row in setup['source_documents'] if row['id'] != OVERRIDE_ID]
    assert verify_geometry_sources(setup, ROOT)['status'] == 'FAIL'
