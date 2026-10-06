"""Offline derived geometry never changes installation evidence or live gates."""
import copy
import json
from pathlib import Path

import numpy as np
import pytest

from wc_runtime.compare_geometry import mechanical_initial_geometry, MOUNT_MODEL
from wc_runtime.hardware_setup import resolve_hardware_setup


ROOT = Path(__file__).resolve().parents[2]


def setup():
    return json.loads((ROOT / 'config/hardware_setup.json').read_text(encoding='utf-8'))


def apply(value=None):
    value = setup() if value is None else value
    return mechanical_initial_geometry(value, resolve_hardware_setup(value))


def test_current_initial_composes_mm_geometry_with_axle_transform_and_records_assumptions():
    value = setup(); resolved = resolve_hardware_setup(value)
    before, old_resolved = copy.deepcopy(value), copy.deepcopy(resolved)
    result = mechanical_initial_geometry(value, resolved)
    assert value == before and resolved == old_resolved
    assert result['mount_model'] == MOUNT_MODEL and result['offline_experiment'] is True
    for side, y in [('left', .22875), ('right', -.22875)]:
        assert np.allclose(result['mounts'][side]['t_axle_lidar_m'], [.323, y, .5915])
        assert result['mounts'][side]['R_axle_lidar'] == np.eye(3).tolist()
    evidence = result['offline_geometry_initialization']
    assert evidence['status'] == 'ASSUMED_OFFLINE_INITIAL'
    assert evidence['scope'] == 'ONLY_OFFLINE_COMPARE'
    assert evidence['source_documents'] == resolved['geometry_report']['source_documents']
    assert evidence['source_hash_verification_required_by_caller'] is True
    assert len(evidence['setup_content_sha256']) == 64
    assert evidence['setup_content_hash_kind'] == 'canonical_JSON_not_original_file_bytes'
    assert np.allclose(np.array(evidence['initial_transforms']['T_lidar_left_lidar_right'])[:3, 3], [0, -.4575, 0])
    assert any('data-origin INITIAL' in text for text in evidence['assumptions'])
    assert evidence['remaining_datum_uncertainty']['face_to_actual_data_origin_m'] == {'left': None, 'right': None}
    assert evidence['remaining_datum_uncertainty']['offset_bound_m'] is None
    assert evidence['coordinate_conversion_applied_here'] is False
    assert not evidence['live_eligible'] and not evidence['calibration_validated']
    assert not evidence['formal_accuracy_acceptance'] and not evidence['time_alignment_validated']
    assert not evidence['hardware_started'] and not evidence['configuration_written']


def test_original_unknown_gates_are_preserved_and_only_offline_motion_is_enabled():
    result = apply(); original = result['geometry_report']['original_report']
    assert all(row['reason'].endswith(':UNKNOWN') for row in original['unresolved_components'])
    for mode in ('left', 'right', 'all'):
        assert original['capabilities']['wheel_imu_motion_' + mode]['status'] == 'BLOCKED'
        current = result['geometry_capabilities']['wheel_imu_motion_' + mode]
        assert current['status'] == 'AVAILABLE' and current['offline_only'] is True
        assert current['validation_level'] == 'OFFLINE_INITIAL'
    for key, row in original['capabilities'].items():
        if not key.startswith('wheel_imu_motion_'):
            assert result['geometry_capabilities'][key] == row
    assert result['geometry_report']['capabilities'] == result['geometry_capabilities']
    assert result['geometry_report']['resolved_data_transforms'] == original['resolved_data_transforms']


def test_imu_translation_and_uncalibrated_wheel_parameters_remain_unchanged_and_independent():
    value = setup(); resolved = resolve_hardware_setup(value); result = apply(value)
    assert result['imu_mount'] == resolved['imu_mount']
    assert result['imu_mount']['t_axle_imu_m'] is None
    assert result['imu_mount']['translation_used'] is False
    assert result['wheel_candidate'] == resolved['wheel_candidate']
    assert result['wheel_candidate']['state'] == 'UNVALIDATED'
    result['wheel_candidate']['wheel_radius_m'] = 9
    result['geometry_report']['original_report']['source_documents'][0]['sha256'] = 'changed'
    assert resolved['wheel_candidate']['wheel_radius_m'] == .185
    assert resolved['geometry_report']['source_documents'][0]['sha256'] != 'changed'


@pytest.mark.parametrize('key', ['mounts', 'imu_mount', 'wheel_candidate', 'mount_model', 'geometry_capabilities'])
def test_stale_or_modified_resolution_cannot_be_used(key):
    value = setup(); resolved = resolve_hardware_setup(value); resolved[key] = {'forged': True}
    with pytest.raises(ValueError, match='does not match setup'):
        mechanical_initial_geometry(value, resolved)


def test_modified_parsed_transform_is_rejected_instead_of_composed():
    value = setup(); resolved = resolve_hardware_setup(value)
    resolved['geometry_report']['resolved_data_transforms']['axle']['mounting_M'][0][3] += 1
    with pytest.raises(ValueError, match='report does not match'):
        mechanical_initial_geometry(value, resolved)


@pytest.mark.parametrize('change, message', [
    ('unconfirmed', 'confirmed current assembly'),
    ('unknown_axle', 'known T_axle_mounting_M'),
    ('unknown_imu', 'known IMU axes'),
    ('tilted_M', 'level/parallel M orientation'),
    ('tilted_imu', 'confirmed X-front/Y-left/Z-up'),
    ('known_lidar', 'must not replace known data extrinsics'),
    ('known_lidar_rotation', 'contradicts the offline identity assumption'),
])
def test_initial_does_not_invent_missing_prerequisites_or_replace_known_geometry(change, message):
    value = setup(); rows = {row['id']: row for row in value['data_transforms']}
    quarter_turn = [[0, -1, 0], [1, 0, 0], [0, 0, 1]]
    if change == 'unconfirmed':
        value['assembly']['confirmed'] = False
    elif change == 'unknown_axle':
        rows['axle_from_mounting_M']['translation'].update(value_m=None, status='UNKNOWN')
    elif change == 'unknown_imu':
        rows['mounting_M_from_imu_native']['rotation'].update(value_matrix=None, status='UNKNOWN')
    elif change == 'tilted_M':
        rows['axle_from_mounting_M']['rotation']['value_matrix'] = quarter_turn
    elif change == 'tilted_imu':
        rows['mounting_M_from_imu_native']['rotation']['value_matrix'] = quarter_turn
    else:
        lidar = rows['mounting_M_from_lidar_left']
        lidar['rotation'].update(value_matrix=quarter_turn if change == 'known_lidar_rotation' else np.eye(3).tolist(),
                                 status='USER_MEASURED_EXPERIMENT', evidence=[{'source_id': 'v7_current_geometry_20261005'}])
        if change == 'known_lidar':
            lidar['translation'].update(value_m=[.05, .22875, .1333], status='USER_MEASURED_EXPERIMENT',
                                       evidence=[{'source_id': 'v7_current_geometry_20261005'}])
    with pytest.raises(ValueError, match=message):
        apply(value)


def test_legacy_profile_cannot_be_promoted_to_current_mechanical_initial():
    value = json.loads((ROOT/'config/calibration/legacy_hardware_setup_before_v7.json').read_text(encoding='utf-8'))
    with pytest.raises(ValueError, match='schema2 V7'):
        mechanical_initial_geometry(value, resolve_hardware_setup(value))


def test_side_facing_reference_cannot_be_used_as_forward_identity():
    value = setup()
    value['mechanical_reference']['reference_points'][0]['nominal_view_direction_M'] = [0, 1, 0]
    with pytest.raises(ValueError, match='forward nominal view direction'):
        apply(value)


def test_source_configuration_is_never_written():
    path = ROOT/'config/hardware_setup.json'; before = path.read_bytes()
    apply()
    assert path.read_bytes() == before


def runtime_config(geometry, mode='all'):
    """Minimal real-source consumer contract; validation creates no workers/devices."""
    result = {'schema_version': 1, 'status': 'EXPERIMENT', 'source_mode': 'real',
              'mode': mode, 'session_id': 'offline_mechanical_guard_test',
              'sensor_ids': {'left': 'recorded-left', 'right': 'recorded-right'},
              'imu_sensor_id': 'recorded-H30', 'wheel_device_id': 'recorded-wheel',
              'input_rate_hz': 5., 'max_pair_delta_ns': 50_000_000,
              'prior_template': {}, 'formal_acceptance': False,
              'navigation_validated': False, 'common_measurement_time_validated': False}
    result.update(copy.deepcopy(geometry))
    return result


@pytest.mark.parametrize('mode, expected', [('left', ('left',)), ('right', ('right',)),
                                            ('all', ('left', 'right'))])
def test_mapping_input_accepts_explicit_offline_initial_only_for_the_requested_sources(mode, expected):
    from wc_runtime.mapping_input import validate_config
    candidate = runtime_config(apply(), mode)
    before = copy.deepcopy(candidate)
    assert validate_config(candidate) == expected
    assert candidate == before


@pytest.mark.parametrize('field, value', [
    ('offline_experiment', False), ('offline_experiment', None), ('offline_experiment', 1),
    ('offline_experiment', 'true'), ('scope', None), ('scope', 'LIVE_MAPPING'),
    ('status', None), ('status', 'CALIBRATED'), ('live_eligible', None),
    ('live_eligible', True), ('live_eligible', 0),
])
def test_mapping_input_rejects_partial_or_live_offline_initial_authorization(field, value):
    from wc_runtime.mapping_input import validate_config, InputFailure
    candidate = runtime_config(apply())
    target = candidate if field == 'offline_experiment' else candidate['offline_geometry_initialization']
    if value is None:
        target.pop(field)
    else:
        target[field] = value
    with pytest.raises(InputFailure, match='FIXED_HARDWARE_SETUP_REQUIRED'):
        validate_config(candidate)


def test_mapping_input_rejects_unmarked_initial_model_and_default_unknown_geometry():
    from wc_runtime.mapping_input import validate_config, InputFailure
    candidate = runtime_config(apply())
    candidate.pop('offline_geometry_initialization')
    with pytest.raises(InputFailure, match='FIXED_HARDWARE_SETUP_REQUIRED'):
        validate_config(candidate)
    unknown = runtime_config(resolve_hardware_setup(setup()))
    for offline in (False, True):
        unknown['offline_experiment'] = offline
        with pytest.raises(ValueError, match='外参能力阻塞'):
            validate_config(unknown)
