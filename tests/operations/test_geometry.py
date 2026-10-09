"""Synthetic geometry/capability checks, source integrity and legacy replay math.

No ROS, devices, serial ports or SDK acquisition are used by these tests.
"""
import copy
import hashlib
import json
from pathlib import Path
import shutil
import sys
import types
import uuid

import numpy as np
import pytest

from wc_runtime.calibration_geometry import (fit_rigid_transform, inspect_installation_document, motion_capability,
    proper_rotation, require_geometry_capability, verify_geometry_sources)
from wc_runtime.hardware_setup import resolve_hardware_setup, rpy_rotation
if sys.platform == 'win32' and 'fcntl' not in sys.modules:
    # Controller imports its Linux supervisor. A Windows configuration test
    # must fail if it ever tries to acquire a device/process lock.
    unavailable = types.ModuleType('fcntl')
    unavailable.LOCK_EX, unavailable.LOCK_NB, unavailable.LOCK_UN = 2, 4, 8
    def forbidden_flock(*args, **kwargs):
        raise AssertionError('Synthetic Windows configuration checks cannot acquire Linux hardware locks')
    unavailable.flock = forbidden_flock
    sys.modules['fcntl'] = unavailable
from wc_runtime import mapping_controller as controller
from wc_runtime.mapping_input import validate_config as validate_mapping_input
from wc_runtime.single_mapping_input import InputFailure


ROOT = Path(__file__).resolve().parents[2]
SOURCE_HASH = 'b2b63b7b2c59c8dc0c48755b1cf4f17baee684b14f384961c09d827e136ec46e'


def unknown_hardware():
    # Historical UNKNOWN fixture is independent of the now measured runtime defaults.
    return json.loads((ROOT/'tests/fixtures/hardware_unknown.json').read_text(encoding='utf-8'))


def measured_source(setup):
    setup['source_documents'].append({'id': 'synthetic_calibration',
        'snapshot_path': 'tests/synthetic.json', 'sha256': '0'*64,
        'evidence_type': 'EXTRINSIC_CALIBRATION', 'assembly_revision': setup['assembly']['revision'],
        'note': 'SYNTHETIC fixture only, never physical calibration.'})


def edge(setup, parent, child, translation=None, rotation=None):
    row = next((x for x in setup['data_transforms'] if x['parent_frame'] == parent and x['child_frame'] == child), None)
    if row is None:
        row = copy.deepcopy(setup['data_transforms'][0])
        row.update(id=parent+'_from_'+child, parent_frame=parent, child_frame=child)
        setup['data_transforms'].append(row)
    for field, value, key in (('translation', translation, 'value_m'), ('rotation', rotation, 'value_matrix')):
        if value is not None:
            row[field].update({key: value, 'status': 'CALIBRATED',
                               'evidence': [{'source_id': 'synthetic_calibration'}], 'note': 'Synthetic measured data-frame fixture.'})
    return row


def complete_pair():
    setup = unknown_hardware(); measured_source(setup); setup['assembly']['confirmed'] = True
    edge(setup, 'axle', 'mounting_M', [.22, -.03, .41], rpy_rotation([7, -4, 21]).tolist())
    edge(setup, 'mounting_M', 'lidar_left', [.051, .218, .132], rpy_rotation([-3, 5, 11]).tolist())
    edge(setup, 'mounting_M', 'lidar_right', [.049, -.216, .134], rpy_rotation([4, 3, -8]).tolist())
    # Gyro-only prior needs rotation, not an invented IMU translation.
    edge(setup, 'mounting_M', 'imu_native', rotation=rpy_rotation([180, 0, 90]).tolist())
    return setup


def test_default_geometry_keeps_unknowns_and_does_not_inherit_old_baseline():
    setup = unknown_hardware(); before = copy.deepcopy(setup)
    result = resolve_hardware_setup(setup)
    assert setup == before
    assert result['mount_model'] == 'fixed_hardware_setup_v2'
    assert result['mounts'] == {}
    assert result['imu_mount']['R_axle_imu'] is None
    assert result['imu_mount']['t_axle_imu_m'] is None
    assert result['imu_mount']['translation_used'] is False
    assert len(result['geometry_report']['unresolved_components']) == 24
    for name in ('native_preview', 'native_capture', 'mechanical_layout', 'camera_intrinsic_calibration',
                 'imu_native_bias_calibration', 'lidar_relative_calibration'):
        assert require_geometry_capability(result, name)['status'] == 'AVAILABLE'
    for mode in ('left', 'right', 'all'):
        with pytest.raises(ValueError, match='外参能力阻塞'):
            require_geometry_capability(result, motion_capability(mode))
    assert setup['legacy_profile']['active'] is False


def test_all_11_mechanical_points_and_source_snapshot_are_traceable_and_not_data_origins():
    setup = unknown_hardware()
    points = setup['mechanical_reference']['reference_points']
    expected = {
        'lidar_left': [50,217.85,133.3], 'lidar_right': [50,-217.85,133.3],
        'camera_left_front_optical': [50,270.6,133.3], 'camera_right_front_optical': [50,-270.6,133.3],
        'camera_left_side_optical': [-22.5,298.6,133.3], 'camera_right_side_optical': [-22.5,-298.6,133.3],
        'double_hole_left_front_unbound': [50,248.6,76.15], 'double_hole_right_front_unbound': [50,-248.6,76.15],
        'double_hole_left_side_unbound': [0,298.6,76.15], 'double_hole_right_side_unbound': [0,-298.6,76.15],
        'imu_native': [-2.5,268.6,167.8]}
    assert {p['sensor_frame']:p['position_mm'] for p in points} == expected
    assert all(p['status'] == 'CAD_NOMINAL' and p['evidence'][0]['line_start'] > 0 for p in points)
    assert np.linalg.norm(np.array(expected['lidar_left'])-expected['lidar_right']) == pytest.approx(435.7)
    assert setup['source_documents'][0]['sha256'] == SOURCE_HASH
    assert verify_geometry_sources(setup, ROOT)['status'] == 'PASS'
    assert hashlib.sha256((ROOT/setup['source_documents'][0]['snapshot_path']).read_bytes()).hexdigest() == SOURCE_HASH


def test_document_import_rechecks_all_points_units_cad_axes_and_distance_rows():
    setup=unknown_hardware(); source=setup['source_documents'][0]
    report=inspect_installation_document(ROOT/source['snapshot_path'],expected_sha256=SOURCE_HASH)
    assert report['unit']=='mm' and report['frame_id']=='mounting_M'
    assert report['data_extrinsics_created'] is False
    assert len(report['reference_points'])==11
    assert len(report['distance_checks'])==21
    assert all(row['rounding_consistent'] for row in report['distance_checks'])
    for side in ('left','right'):
        actual=np.array(report['cad_to_M'][side]['T_M_CAD_mm'])
        assert np.linalg.det(actual[:3,:3])==pytest.approx(1.)
        assert np.allclose(actual[:3,:3],[[0,-1,0],[1,0,0],[0,0,1]])


def test_document_hash_does_not_hide_manually_changed_mechanical_catalogue():
    setup=unknown_hardware()
    setup['mechanical_reference']['reference_points'][0]['position_mm'][1]+=10
    report=verify_geometry_sources(setup,ROOT)
    assert report['status']=='FAIL'
    assert 'catalogue differs' in report['sources'][0]['reason']


def test_nonidentity_M_to_axle_composes_true_data_frames_and_unknown_imu_translation_is_not_required():
    setup = complete_pair(); result = resolve_hardware_setup(setup)
    def matrix(parent, child):
        row = next(x for x in setup['data_transforms'] if x['parent_frame'] == parent and x['child_frame'] == child)
        t = np.eye(4); t[:3,:3] = row['rotation']['value_matrix']; t[:3,3] = row['translation']['value_m']
        return t
    axle_M = matrix('axle', 'mounting_M')
    for side in ('left', 'right'):
        expected = axle_M @ matrix('mounting_M', 'lidar_'+side)
        assert np.allclose(result['mounts'][side]['R_axle_lidar'], expected[:3,:3])
        assert np.allclose(result['mounts'][side]['t_axle_lidar_m'], expected[:3,3])
    expected_imu = axle_M[:3,:3] @ rpy_rotation([180,0,90])
    assert np.allclose(result['imu_mount']['R_axle_imu'], expected_imu)
    assert result['imu_mount']['t_axle_imu_m'] is None
    assert require_geometry_capability(result, 'wheel_imu_motion_all')['status'] == 'AVAILABLE'
    assert result['geometry_capabilities']['ultrasonic_spatial_projection']['status'] == 'BLOCKED'


def test_relative_lidar_calibration_can_unlock_static_fusion_without_axle_mount():
    setup = unknown_hardware(); measured_source(setup)
    row = edge(setup, 'lidar_left', 'lidar_right', [.013,-.434,.006], rpy_rotation([2,-1,7]).tolist())
    result = resolve_hardware_setup(setup)
    assert require_geometry_capability(result, 'dual_lidar_static_fusion')['status'] == 'AVAILABLE'
    assert result['mounts'] == {}
    actual = np.array(result['geometry_report']['resolved_data_transforms']['lidar_left']['lidar_right'])
    assert np.allclose(actual[:3,3], row['translation']['value_m'])
    with pytest.raises(ValueError, match='T_axle_lidar'):
        require_geometry_capability(result, 'wheel_imu_motion_all')


def test_left_debug_motion_does_not_require_unknown_right_mount():
    setup = complete_pair()
    row = next(x for x in setup['data_transforms'] if x['child_frame']=='lidar_right')
    for field, key in (('translation','value_m'), ('rotation','value_matrix')):
        row[field].update(status='UNKNOWN', **{key: None})
    result = resolve_hardware_setup(setup)
    assert require_geometry_capability(result, 'wheel_imu_motion_left')['status']=='AVAILABLE'
    assert result['geometry_capabilities']['wheel_imu_motion_all']['status']=='BLOCKED'


@pytest.mark.parametrize('mutation', [
    lambda s: s['data_transforms'][0].update(direction='parent_to_child'),
    lambda s: s['data_transforms'][0].update(unit='mm'),
    lambda s: s['data_transforms'][0]['translation'].update(value_m=[0,0,0]),
    lambda s: s['data_transforms'][0]['rotation'].update(value_matrix=np.eye(3).tolist()),
    lambda s: s['data_transforms'][0]['translation'].update(reference_kind='CAD_OUTWARD_FACE_CENTER'),
    lambda s: s['mechanical_reference']['reference_points'].pop(),
    lambda s: s['assembly'].update(confirmed=1),
    lambda s: s['legacy_profile'].update(active=True),
    lambda s: s['source_documents'][0].update(snapshot_path='../outside.md'),
    lambda s: s['source_documents'][0].update(snapshot_path='C:/private/file.md')])
def test_ambiguous_units_origins_directions_sources_or_filled_unknowns_are_rejected(mutation):
    setup = unknown_hardware(); mutation(setup)
    with pytest.raises(ValueError):
        resolve_hardware_setup(setup)


def test_mechanical_evidence_cannot_be_relabelled_into_a_measured_data_origin():
    setup = unknown_hardware()
    row = setup['data_transforms'][1]
    row['translation'].update(status='USER_MEASURED_EXPERIMENT', value_m=[.05,.21785,.1333])
    with pytest.raises(ValueError, match='assembly-matching geometry measurement'):
        resolve_hardware_setup(setup)


@pytest.mark.parametrize('bad', [np.diag([1,-1,1]).tolist(), (np.eye(3)*1.01).tolist(),
    [[1,0,.1],[0,1,0],[0,0,1]], [['1',0,0],[0,1,0],[0,0,1]],
    [[True,0,0],[0,1,0],[0,0,1]], [[float('nan'),0,0],[0,1,0],[0,0,1]]])
def test_matrix_input_rejects_reflection_scaling_skew_and_fake_numbers(bad):
    with pytest.raises(ValueError):
        proper_rotation(bad)


def test_inconsistent_redundant_transform_paths_are_rejected():
    setup = complete_pair()
    edge(setup, 'lidar_left', 'lidar_right', [0,-.4357,0], np.eye(3).tolist())
    with pytest.raises(ValueError, match='inconsistent transform'):
        resolve_hardware_setup(setup)


def test_measurement_from_another_assembly_does_not_enable_current_motion():
    setup = complete_pair()
    setup['source_documents'][-1]['assembly_revision'] = 'V6_OTHER_INSTALLATION'
    with pytest.raises(ValueError, match='assembly-matching'):
        resolve_hardware_setup(setup)


def test_full_data_chain_still_requires_actual_assembly_confirmation():
    setup = complete_pair(); setup['assembly']['confirmed'] = False
    result = resolve_hardware_setup(setup)
    with pytest.raises(ValueError, match='装配版本'):
        require_geometry_capability(result, 'wheel_imu_motion_all')


def test_source_snapshot_hash_failure_is_explicit_and_original_path_is_not_a_fallback(monkeypatch):
    setup = unknown_hardware()
    assert verify_geometry_sources(setup,ROOT)['status']=='PASS'
    first = (ROOT/setup['source_documents'][0]['snapshot_path']).resolve()
    original_read = Path.read_bytes
    def modified_snapshot(path):
        content = original_read(path)
        return content+b'\nchanged\n' if path == first else content
    monkeypatch.setattr(Path, 'read_bytes', modified_snapshot)
    result = verify_geometry_sources(setup,ROOT)
    assert result['status']=='FAIL'
    assert 'hash mismatch' in result['sources'][0]['reason']


def test_legacy_profile_preserves_matrix_math_but_is_not_automatically_current_geometry():
    setup = unknown_hardware()
    legacy = json.loads((ROOT/setup['legacy_profile']['path']).read_text(encoding='utf-8'))
    result = resolve_hardware_setup(legacy)
    assert result['mount_model']=='fixed_hardware_setup_v1'
    assert result['mounts']['left']['t_axle_lidar_m']==[.34,.3125,.52]
    assert np.allclose(result['mounts']['left']['R_axle_lidar'], rpy_rotation(legacy['lidars']['left']['rpy_deg']))
    assert require_geometry_capability(result,'legacy_replay')['status']=='AVAILABLE'
    with pytest.raises(ValueError, match='schema1'):
        require_geometry_capability(result,'wheel_imu_motion_all')


def test_offline_point_fit_recovers_rotation_translation_without_scale_or_auto_install():
    child = np.array([[0,0,0],[.2,0,0],[0,.3,0],[0,0,.4],[.13,.21,.17]])
    rotation = rpy_rotation([23,-31,67]); translation=np.array([.31,-.23,.12])
    parent = child @ rotation.T+translation
    result = fit_rigid_transform(parent.tolist(),child.tolist())
    actual=np.array(result['T_parent_child'])
    assert np.allclose(actual[:3,:3],rotation)
    assert np.allclose(actual[:3,3],translation)
    assert result['rmse_m']<1e-12
    assert result['status']=='FIT_ONLY_NOT_VALIDATED'
    assert result['independent_validation'] is False
    assert result['configuration_written'] is False


def test_collinear_correspondences_do_not_fake_a_6DoF_calibration():
    with pytest.raises(ValueError, match='unobservable'):
        fit_rigid_transform([[0,0,0],[1,0,0],[2,0,0]],[[0,0,0],[1,0,0],[2,0,0]])


@pytest.fixture
def configuration_root(monkeypatch):
    parent = Path(__file__).resolve().parent
    root = parent/('.geometry_test_'+uuid.uuid4().hex)
    root.mkdir()
    try:
        (root/'config').mkdir()
        for name in ('mapping_live.json','live_unvalidated.json','wheel_feedback_current.json','cameras.json'):
            shutil.copyfile(ROOT/'config'/name,root/'config'/name)
        setup = unknown_hardware()
        for source in setup['source_documents']:
            destination=root/source['snapshot_path'];destination.parent.mkdir(parents=True,exist_ok=True)
            shutil.copyfile(ROOT/source['snapshot_path'],destination)
        (root/'config/hardware_setup.json').write_text(json.dumps(setup),encoding='utf-8')
        monkeypatch.setattr(controller,'ROOT',root)
        def forbidden(*args,**kwargs):
            raise AssertionError('Configuration and capability checks must not start subprocesses or hardware')
        monkeypatch.setattr(controller.subprocess,'run',forbidden)
        monkeypatch.setattr(controller.subprocess,'Popen',forbidden)
        yield root, {'mode':'all','session_id':'geometry_synthetic_check','config_path':str(root/'config/mapping_live.json'),
                     'mapping_enabled':False}
    finally:
        resolved=root.resolve()
        assert resolved.parent==parent and resolved.name.startswith('.geometry_test_')
        shutil.rmtree(resolved)


def alter_mapping_config(root, **values):
    path=root/'config/mapping_live.json'
    config=json.loads(path.read_text(encoding='utf-8'));config.update(values)
    path.write_text(json.dumps(config),encoding='utf-8')


def test_configuration_checks_native_preview_with_unknowns_and_explicit_hybrid_control(configuration_root):
    root, request=configuration_root
    config=controller.configuration(request)
    assert config['native_preview_only'] is True
    assert config['manual_controls']['interaction_policy'] == 'hybrid_manual'
    assert config['manual_authorization_scope']=='USER_SELECTED_HYBRID_MANUAL_INDEPENDENT_OF_LIDAR_GEOMETRY'
    assert config['native_mapping_profile']['status']=='UNAVAILABLE_GEOMETRY'
    assert config['prior_template']['estimator']=='five_state'
    assert validate_mapping_input(config)==('left','right')
    report=controller.check_configuration(request)
    assert report['hardware_started'] is False
    assert report['geometry_report']['source_verification']['status']=='PASS'
    assert report['resolved_mounts']=={}
    assert report['resolved_imu']['R_axle_imu'] is None
    assert report['lidars'] is None
    assert report['required_geometry_capability']=='native_preview'


def test_configuration_blocks_motion_before_any_hardware_when_geometry_is_unknown(configuration_root):
    root, request=configuration_root;request['mapping_enabled']=True
    with pytest.raises(ValueError,match='外参能力阻塞'):
        controller.configuration(request)


def test_controller_and_input_allow_10hz_only_with_explicit_offline_flag(configuration_root):
    root, request=configuration_root
    alter_mapping_config(root,input_rate_hz=10.)
    with pytest.raises(ValueError,match='10 Hz 仅限'):
        controller.configuration(request)
    alter_mapping_config(root,offline_experiment=True)
    config=controller.configuration(request)
    assert config['input_rate_hz']==10
    assert validate_mapping_input(config)==('left','right')
    config['offline_experiment']=False
    with pytest.raises(InputFailure,match='AT_MOST_5HZ'):
        validate_mapping_input(config)


def test_estimator_selection_reaches_prior_and_rejects_invalid_or_nonplanar_rl(configuration_root):
    root, request=configuration_root
    alter_mapping_config(root,wheel_imu_estimator='robot_localization')
    config=controller.configuration(request)
    assert config['prior_template']['estimator']=='robot_localization'
    assert validate_mapping_input(config)==('left','right')
    alter_mapping_config(root,motion_model='se3_gyro',map_profile=None)
    with pytest.raises(ValueError,match='robot_localization 对照'):
        controller.configuration(request)
    alter_mapping_config(root,wheel_imu_estimator='auto')
    with pytest.raises(ValueError,match='wheel_imu_estimator'):
        controller.configuration(request)
