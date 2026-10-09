"""Explicit, hardware-free mechanical initial geometry for offline comparison.

This module does not install calibration or change schema2's UNKNOWN semantics.
Its caller must verify the frozen source hashes and obtain an explicit offline
comparison opt-in. A face-centre model is an assumption, not a measured optical
origin. Live controllers must continue to reject this mount model.
"""
import copy
import hashlib
import json
from functools import lru_cache
from pathlib import Path

import numpy as np

from .calibration_geometry import finite_array, proper_rotation, rigid_transform
from .hardware_setup import resolve_hardware_setup


MOUNT_MODEL = 'offline_mechanical_initial'
INITIAL_STATUS = 'ASSUMED_OFFLINE_INITIAL'


@lru_cache(maxsize=1)
def _legacy_mount_models():
    value = json.loads(Path(__file__).with_name('legacy_contracts.json').read_text(encoding='utf-8'))
    if (value.get('schema_version') != 1 or
            not isinstance(value.get('offline_mount_models'), dict) or
            any(not isinstance(key, str) or target != MOUNT_MODEL
                for key, target in value['offline_mount_models'].items())):
        raise ValueError('invalid historical offline geometry contract')
    return frozenset(value['offline_mount_models'])


def is_offline_mechanical_initial(config):
    """Recognize current and exact archived names with identical offline gates.

    This does not rewrite the input or certify its geometry. The caller retains
    source-hash, transform and capability validation for both identifier forms.
    """
    initial = config.get('offline_geometry_initialization') or {}
    model = config.get('mount_model')
    return (isinstance(initial, dict) and isinstance(model, str) and
            config.get('offline_experiment') is True and
            initial.get('scope') == 'ONLY_OFFLINE_COMPARE' and
            initial.get('status') == INITIAL_STATUS and
            initial.get('offline_only') is True and
            initial.get('live_eligible') is False and
            (model == MOUNT_MODEL or model in _legacy_mount_models()))


def mechanical_initial_geometry(setup, resolved):
    """Return a separate offline-only resolved copy, preserving the original report.

    Preconditions: sourced schema2 geometry, confirmed assembly, measured M-to-axle
    transform and IMU orientation, both forward-facing mechanical radar centres.
    Source byte/hash verification belongs to the caller (which knows the archive
    path mapping). This pure function additionally binds ``resolved`` to ``setup``
    by resolving it again; it never fills or writes the source configuration.
    """
    if not isinstance(setup, dict) or setup.get('schema_version') != 2:
        raise ValueError('offline mechanical initial requires schema2 geometry')
    if not isinstance(resolved, dict):
        raise ValueError('resolved geometry must be an object')
    canonical = resolve_hardware_setup(setup)
    for key in ('mounts', 'imu_mount', 'wheel_candidate', 'mount_model', 'geometry_capabilities'):
        if resolved.get(key) != canonical[key]:
            raise ValueError('resolved geometry does not match setup: ' + key)
    report = resolved.get('geometry_report', {})
    for key in ('assembly_revision', 'assembly_confirmed', 'source_documents', 'resolved_data_transforms'):
        if report.get(key) != canonical['geometry_report'][key]:
            raise ValueError('resolved geometry report does not match setup: ' + key)
    if report['assembly_confirmed'] is not True:
        raise ValueError('offline mechanical initial requires the confirmed current assembly')
    if canonical['mounts']:
        raise ValueError('offline mechanical initial must not replace known data extrinsics')
    matrix_value = report['resolved_data_transforms'].get('axle', {}).get('mounting_M')
    if matrix_value is None:
        raise ValueError('known T_axle_mounting_M required; UNKNOWN cannot become zero')
    axle_from_m = rigid_transform(matrix_value, 'T_axle_mounting_M')
    if not np.allclose(axle_from_m[:3, :3], np.eye(3), atol=1e-6, rtol=0):
        raise ValueError('offline mechanical initial requires the documented level/parallel M orientation')
    imu_rotation = canonical['imu_mount']['R_axle_imu']
    if imu_rotation is None:
        raise ValueError('known IMU axes required; UNKNOWN cannot become identity')
    imu_rotation = proper_rotation(imu_rotation, 'R_axle_imu')
    if not np.allclose(imu_rotation, np.eye(3), atol=1e-6, rtol=0):
        raise ValueError('offline mechanical initial requires the confirmed X-front/Y-left/Z-up IMU axes')

    catalogue = setup['mechanical_reference']
    if catalogue['frame_id'] != 'mounting_M' or catalogue['unit'] != 'mm':
        raise ValueError('mechanical centres must explicitly use mounting_M and millimetres')
    selected, mounts, transforms = {}, {}, {}
    for side in ('left', 'right'):
        frame = 'lidar_' + side
        points = [row for row in catalogue['reference_points'] if row['sensor_frame'] == frame]
        if len(points) != 1 or points[0]['point_kind'] != 'CAD_OUTWARD_FACE_CENTER':
            raise ValueError('one CAD_OUTWARD_FACE_CENTER required for ' + frame)
        point = points[0]
        direction = finite_array(point['nominal_view_direction_M'], (3,), frame + '.view_direction')
        if not np.allclose(direction, [1, 0, 0], atol=1e-6, rtol=0):
            raise ValueError(frame + ' must have the documented forward nominal view direction')
        # A known, contrary installation rotation must not be silently overwritten.
        for row in setup['data_transforms']:
            if row['parent_frame'] == 'mounting_M' and row['child_frame'] == frame:
                rotation = row['rotation']
                if rotation['status'] in ('USER_MEASURED_EXPERIMENT', 'CALIBRATED') and \
                        not np.allclose(proper_rotation(rotation['value_matrix']), np.eye(3), atol=1e-6, rtol=0):
                    raise ValueError('known ' + frame + ' rotation contradicts the offline identity assumption')
        face_m = finite_array(point['position_mm'], (3,), frame + '.face_mm') / 1000.
        m_from_data_initial = np.eye(4)
        m_from_data_initial[:3, 3] = face_m
        axle_from_data_initial = axle_from_m @ m_from_data_initial
        mounts[side] = {'t_axle_lidar_m': axle_from_data_initial[:3, 3].tolist(),
                        'R_axle_lidar': axle_from_data_initial[:3, :3].tolist()}
        transforms['T_axle_' + frame] = axle_from_data_initial.tolist()
        selected[side] = copy.deepcopy(point)

    left_from_right = np.linalg.inv(np.asarray(transforms['T_axle_lidar_left'])) @ \
        np.asarray(transforms['T_axle_lidar_right'])
    initialization = {
        'schema_version': 1, 'status': INITIAL_STATUS, 'scope': 'ONLY_OFFLINE_COMPARE',
        'offline_only': True, 'live_eligible': False, 'calibration_validated': False,
        'formal_accuracy_acceptance': False, 'source_hash_verification_required_by_caller': True,
        'assembly_revision': report['assembly_revision'],
        'setup_content_sha256': hashlib.sha256(json.dumps(setup, ensure_ascii=False, sort_keys=True,
            separators=(',', ':'), allow_nan=False).encode('utf-8')).hexdigest(),
        'setup_content_hash_kind': 'canonical_JSON_not_original_file_bytes',
        'source_documents': copy.deepcopy(report['source_documents']),
        'mechanical_reference_points': selected,
        'known_T_axle_mounting_M': axle_from_m.tolist(),
        'known_R_axle_imu': imu_rotation.tolist(),
        'assembly_confirmation': copy.deepcopy(setup['assembly']),
        'mechanical_installation_assumptions': copy.deepcopy(catalogue['assumptions']),
        'coordinate_contract': 'xt_sdk_car_fru_to_flu_yneg_v1',
        'coordinate_conversion_applied_here': False,
        'coordinate_note': 'Recorded driver cloud is already FLU; do not apply another y reflection.',
        'assumptions': [
            'Use each documented radar outward-face centre as the data-origin INITIAL approximation.',
            'Assume R_mounting_M_data=I for the level, parallel, forward-facing installation.',
            'Forward view direction alone does not prove sensor roll; the identity is an explicit installation assumption.',
            'Matched radar package layout does not prove equal optical datum offsets.'
        ],
        'equations': [
            't_mounting_M_data_initial = mechanical_face_position_mm / 1000',
            'R_mounting_M_data_initial = I (explicit offline installation assumption)',
            'T_axle_data_initial = T_axle_mounting_M @ T_mounting_M_data_initial',
            'T_lidar_left_lidar_right_initial = inverse(T_axle_lidar_left_initial) @ T_axle_lidar_right_initial'
        ],
        'initial_transforms': {**transforms, 'T_lidar_left_lidar_right': left_from_right.tolist()},
        'remaining_datum_uncertainty': {
            'face_to_actual_data_origin_m': {'left': None, 'right': None},
            'offset_components_frame': 'mounting_M',
            'installation_rotation_calibrated': False,
            'origin_relation': 't_axle_data_true = t_axle_face + R_axle_mounting_M @ delta_mounting_M_face_to_data',
            'offset_bound_m': None, 'bound_note': 'Housing size is not a verified bound on the SDK virtual origin.',
            'relative_note': 'A common identical offset cancels only under the additional same-offset/same-axis assumption.',
            'static_geometry_note': 'Relative cloud alignment cannot determine the common absolute radar-to-axle datum.',
            'planar_motion_note': 'Pure planar motion cannot independently identify vertical lever-arm translation.'
        },
        'imu_translation_used': False, 'wheel_parameters_calibrated': False,
        'time_alignment_validated': False, 'hardware_started': False, 'configuration_written': False
    }
    result = copy.deepcopy(resolved)
    result.update(mounts=mounts, mount_model=MOUNT_MODEL, offline_experiment=True,
                  offline_geometry_initialization=initialization)
    for mode in ('left', 'right', 'all'):
        row = result['geometry_capabilities']['wheel_imu_motion_' + mode]
        row.update(status='AVAILABLE', reasons=[], validation_level='OFFLINE_INITIAL',
                   offline_only=True, initialization_status=INITIAL_STATUS)
    result['geometry_report']['original_report'] = copy.deepcopy(report)
    result['geometry_report']['capabilities'] = copy.deepcopy(result['geometry_capabilities'])
    result['geometry_report']['validation_level'] = 'OFFLINE_INITIAL'
    result['geometry_report']['offline_only'] = True
    result['geometry_report']['offline_geometry_initialization'] = copy.deepcopy(initialization)
    result['geometry_report']['limitations'].append(
        '显式离线初值：用雷达外壳前面中心近似数据原点；原UNKNOWN和阻塞报告保留，不是外参标定或现场验收。')
    return result
