"""Coordinate direction, absent measurements and principal-unit conversion."""
import copy
import json
from pathlib import Path
import shutil

import numpy as np
import pytest

from wc_panel.installation_parameters import (derived_transforms, installation_summary,
    matrix_rpy, principal_values, rpy_matrix, update_component)


def transform(parent, child, position, angles, status='USER_MEASURED_EXPERIMENT'):
    return dict(id=parent + '_from_' + child, parent_frame=parent, child_frame=child,
        direction='child_to_parent', unit='m', note='fixture only',
        translation=dict(value_m=position, status='UNKNOWN' if position is None else status,
            reference_kind='FRAME_ORIGIN', evidence=[dict(source_id='measured')], note='fixture origin'),
        rotation=dict(value_matrix=None if angles is None else rpy_matrix(angles),
            status='UNKNOWN' if angles is None else status, reference_kind='DATA_AXES',
            evidence=[dict(source_id='measured')], note='fixture axes'))


@pytest.mark.parametrize('angles', ([12, -27, 61], [-120, 89.999, 175], [20, 90, 30],
                                    [20, -90, 30], [179, 0, -179], [0, 0, 0]))
def test_rotation_display_roundtrip_including_gimbal_lock(angles):
    matrix = rpy_matrix(angles)
    assert np.allclose(rpy_matrix(matrix_rpy(matrix)), matrix, atol=1e-10, rtol=0)


def test_positive_yaw_rotates_child_front_into_parent_left():
    assert np.allclose(np.asarray(rpy_matrix([0, 0, 90])) @ [1, 0, 0], [0, 1, 0])


def test_relative_coordinates_use_inverse_left_then_right_not_raw_subtraction():
    rows = [transform('axle', 'lidar_left', [1, 2, 0], [0, 0, 90]),
            transform('axle', 'lidar_right', [2, 2, 0], [0, 0, 0])]
    relative = derived_transforms(rows)[2]
    assert np.allclose(relative['position_mm'], [0, -1000, 0], atol=1e-10)
    # The right sensor's front point is (3,2,0) in axle and (0,-2,0) in left.
    assert np.allclose(np.asarray(relative['matrix']) @ [1, 0, 0, 1], [0, -2, 0, 1])


def test_known_imu_axes_survive_unknown_imu_position():
    rows = [transform('axle', 'mounting_M', [.1, .2, .3], [0, 0, 90]),
            transform('mounting_M', 'imu_native', None, [0, 0, 0])]
    imu = derived_transforms(rows)[3]
    assert imu['position_mm'] is None and imu['matrix'] is None
    assert np.allclose(imu['orientation_deg'], [0, 0, 90])
    assert imu['status'] == 'DECLARED'


def test_unknown_origin_and_orientation_do_not_become_zero_or_identity():
    row = transform('axle', 'lidar_left', None, None)
    assert principal_values(row) == dict(position_mm=None, orientation_deg=None)
    assert derived_transforms([row])[0]['status'] == 'UNKNOWN'
    original = copy.deepcopy(row)
    derived_transforms([row])
    assert row == original


def test_unverified_candidate_is_available_only_as_explicit_preview():
    row = transform('axle', 'lidar_left', [.1, .2, .3], [0, 0, 0], 'LEGACY_CANDIDATE')
    assert derived_transforms([row])[0]['status'] == 'CANDIDATE'
    assert derived_transforms([row], usable_only=True)[0]['matrix'] is None


def test_inconsistent_coordinate_cycles_are_rejected():
    rows = [transform('axle', 'mounting_M', [.1, 0, 0], [0, 0, 0]),
            transform('mounting_M', 'lidar_left', [.2, 0, 0], [0, 0, 0]),
            transform('axle', 'lidar_left', [.9, 0, 0], [0, 0, 0])]
    with pytest.raises(ValueError, match='矛盾'):
        derived_transforms(rows)


def test_measured_edit_needs_complete_finite_measurements_and_source():
    original = transform('axle', 'lidar_left', None, None)['translation']
    with pytest.raises(ValueError, match='空白'):
        update_component(original, 'translation', [1, None, 2], 'USER_MEASURED_EXPERIMENT', 'source', 'measured')
    with pytest.raises(ValueError):
        update_component(original, 'translation', [1, float('nan'), 2], 'USER_MEASURED_EXPERIMENT', 'source', 'measured')
    with pytest.raises(ValueError, match='绑定'):
        update_component(original, 'translation', [1, 2, 3], 'USER_MEASURED_EXPERIMENT')
    value = update_component(original, 'translation', [100, 200, 300], 'USER_MEASURED_EXPERIMENT', 'new-source', 'measured')
    assert value['value_m'] == [.1, .2, .3]
    assert value['evidence'] == [dict(source_id='new-source')]
    assert original['value_m'] is None


def test_unknown_clear_preserves_unknown_and_candidate_never_inherits_measured_status():
    original = transform('axle', 'lidar_left', [.1, .2, .3], [0, 0, 0])['translation']
    with pytest.raises(ValueError, match='留空'):
        update_component(original, 'translation', [0, 0, 0], 'UNKNOWN')
    cleared = update_component(original, 'translation', None, 'UNKNOWN')
    assert cleared['value_m'] is None and cleared['status'] == 'UNKNOWN'
    candidate = update_component(original, 'translation', [400, 500, 600], 'LEGACY_CANDIDATE')
    assert candidate['status'] == 'LEGACY_CANDIDATE' and candidate['evidence'] == original['evidence']


def test_summary_uses_current_offline_helper_without_promoting_cad_origin():
    root = Path(__file__).resolve().parents[2]
    setup = json.loads((root/'config/hardware_setup.json').read_text(encoding='utf-8'))
    snapshot = dict(installation=setup, groups=[dict(id='extrinsics', value=setup['data_transforms'])])
    before = copy.deepcopy(snapshot)
    summary = installation_summary(snapshot)
    assert summary['offline_initial']['status'] == 'ASSUMED_OFFLINE_INITIAL'
    assert all(row['position_mm'] is None for row in summary['derived'])
    assert summary['derived'][3]['orientation_deg'] is not None
    assert snapshot == before
    left = summary['offline_initial']['initial_transforms']['T_axle_lidar_left']
    assert np.allclose([left[i][3]*1000 for i in range(3)], [323, 228.75, 591.5])


def test_incomplete_snapshot_cannot_invent_offline_initial():
    result = installation_summary(dict(groups=[]))
    assert result['offline_initial'] is None and '完整设备配置' in result['offline_error']


def test_matrix_display_rejects_reflection_and_scaling():
    for matrix in ([[1, 0, 0], [0, -1, 0], [0, 0, 1]], [[2, 0, 0], [0, 1, 0], [0, 0, 1]]):
        with pytest.raises(ValueError):
            matrix_rpy(matrix)


@pytest.mark.parametrize('update_base_and_imu', (False, True))
def test_principal_edits_save_via_real_revision_evidence_and_derive_both_sensors(tmp_path, update_base_and_imu):
    """Synthetic source file exercises the real save boundary, not UI mocks."""
    from wc_panel.parameters import ParameterStore
    root = Path(__file__).resolve().parents[2]
    project = tmp_path/'project'
    (project/'config').mkdir(parents=True)
    for name in ('hardware_setup.json', 'mapping_live.json', 'cameras.json'):
        shutil.copyfile(root/'config'/name, project/'config'/name)
    setup = json.loads((project/'config/hardware_setup.json').read_text(encoding='utf-8'))
    for source in setup['source_documents']:
        target = project/source['snapshot_path']
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(root/source['snapshot_path'], target)
    store = ParameterStore(project, tmp_path/'backups')
    document = store.read()
    group = next(row for row in document['groups'] if row['id'] == 'extrinsics')
    permission = store.request_edit('extrinsics')
    record = tmp_path/'synthetic_measurement.txt'
    record.write_text('SYNTHETIC TEST ONLY: manufactured values, not actual calibration.\n', encoding='utf-8')
    proof = store.prepare_extrinsic_evidence(record, 'GEOMETRY_MEASUREMENT', group['assembly_revision'],
        'Synthetic fixture data origins and parallel axes; no physical accuracy claim.', document['revision'])
    value = copy.deepcopy(group['value'])
    original_bytes = (project/'config/hardware_setup.json').read_bytes()
    for row in value:
        side = row['child_frame']
        if update_base_and_imu and side == 'mounting_M':
            row['translation'] = update_component(row['translation'], 'translation', [280, 0, 460],
                'USER_MEASURED_EXPERIMENT', proof['source']['id'], 'New synthetic fixture base measurement')
            row['rotation'] = update_component(row['rotation'], 'rotation', [0, 0, 10],
                'USER_MEASURED_EXPERIMENT', proof['source']['id'], 'New synthetic fixture base axes')
        if update_base_and_imu and side == 'imu_native':
            row['rotation'] = update_component(row['rotation'], 'rotation', [1, 2, 3],
                'USER_MEASURED_EXPERIMENT', proof['source']['id'], 'New synthetic fixture IMU axes')
        if side not in ('lidar_left', 'lidar_right'):
            continue
        row['translation'] = update_component(row['translation'], 'translation',
            [50, 225 if side == 'lidar_left' else -225, 133], 'USER_MEASURED_EXPERIMENT',
            proof['source']['id'], 'Synthetic fixture position')
        row['rotation'] = update_component(row['rotation'], 'rotation', [0, 0, 0],
            'USER_MEASURED_EXPERIMENT', proof['source']['id'], 'Synthetic fixture axes')
    result = store.save('extrinsics', value, permission['token'], document['revision'], [proof['token']])
    actual = store.read()
    rows = next(row for row in actual['groups'] if row['id'] == 'extrinsics')['value']
    computed = derived_transforms(rows, usable_only=True)
    assert np.allclose(computed[2]['position_mm'], [0, -450, 0])
    expected = ([280, 0, 460] + np.asarray(rpy_matrix([0, 0, 10])) @ [50, 225, 133]
                if update_base_and_imu else [323, 225, 591.2])
    assert np.allclose(computed[0]['position_mm'], expected)
    if update_base_and_imu:
        assert np.allclose(computed[3]['orientation_deg'], [1, 2, 13])
        base = next(row for row in rows if row['child_frame'] == 'mounting_M')
        assert base['rotation']['evidence'] == [dict(source_id=proof['source']['id'])]
    assert computed[3]['position_mm'] is None
    assert (Path(result['backup'])/'hardware_setup.json').read_bytes() == original_bytes
    assert actual['revision'] != document['revision']
    with pytest.raises(ValueError, match='配置已改变'):
        store.save('extrinsics', value, permission['token'], document['revision'], [proof['token']])
