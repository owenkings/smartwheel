"""Resolve explicit, fixed hardware mounting candidates without device access.

T_A_B maps coordinates in B into A. Survey gravity attitudes describe the
unchanged wheelchair pose used for installation measurements; they are not
live measurements of an axle mounting angle. Startup gravity leveling is a
separate operation and must never rewrite these rigid mounting transforms.
"""
import copy
import math

import numpy as np

from wc_motion.history_preview import HistoryPreview
from .mapping_visuals import validate_visualization


ROTATION_CONVENTION = 'Rz(yaw) Ry(pitch) Rx(roll)'
REFERENCE_STATUSES = {'ASSUMED_LEVEL_NOT_MEASURED', 'USER_MEASURED_EXPERIMENT'}
WHEEL_FIELDS = {'schema', 'state', 'formal_odometry_eligible', 'provenance',
                'wheel_radius_m', 'track_width_m', 'register_to_wheel_rpm',
                'left_sign', 'right_sign', 'max_gap_s', 'max_wheel_speed_m_s'}


def _object(value, required, label, *, optional=()):
    if not isinstance(value, dict):
        raise ValueError(label + ' must be an object')
    unknown = set(value) - set(required) - set(optional) - {'note', '_help'}
    missing = set(required) - set(value)
    if unknown or missing:
        raise ValueError(label + ' has unknown or missing fields: unknown=' +
                         repr(sorted(unknown, key=str)) + ', missing=' + repr(sorted(missing)))
    if 'note' in value and not isinstance(value['note'], str):
        raise ValueError(label + '.note must be text')
    if '_help' in value and not isinstance(value['_help'], dict):
        raise ValueError(label + '._help must be an object')
    return value


def _vector(value, label):
    if not isinstance(value, (list, tuple)) or len(value) != 3 or any(
            type(item) not in (int, float) or not math.isfinite(item) for item in value):
        raise ValueError(label + ' must contain three finite numbers')
    return np.asarray(value, dtype=float)


def _position(value, label):
    result = _vector(value, label)
    if np.linalg.norm(result) > 3:
        raise ValueError(label + ' must be in metres with norm at most 3 m')
    return result


def rpy_rotation(degrees):
    """Proper active rotation Rz(yaw) @ Ry(pitch) @ Rx(roll), input degrees."""
    value = _vector(degrees, 'rpy_deg')
    if np.any(np.abs(value) > 180):
        raise ValueError('rpy_deg must lie within [-180, 180] degrees')
    roll, pitch, yaw = np.radians(value)
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1., 0., 0.], [0., cr, -sr], [0., sr, cr]])
    ry = np.array([[cp, 0., sp], [0., 1., 0.], [-sp, 0., cp]])
    rz = np.array([[cy, -sy, 0.], [sy, cy, 0.], [0., 0., 1.]])
    return rz @ ry @ rx


def _resolve_v1(value):
    """Validate one editable setup and return JSON-ready fixed transforms.

The IMU position may explicitly be unknown (null). It is recorded, but is not
used by the present gyro/static-gravity prior: this is not translational IMU
acceleration integration or acceleration lever-arm compensation.
"""
    setup = _object(value, {'schema_version', 'status', 'coordinate_convention',
                           'rotation_convention', 'calibration_reference',
                           'lidars', 'imu', 'wheel_odometry'}, 'hardware_setup', optional={'visualization','manual_controls'})
    if type(setup['schema_version']) is not int or setup['schema_version'] != 1 or \
            setup['status'] != 'EXPERIMENT':
        raise ValueError('hardware_setup requires schema_version 1 and status EXPERIMENT')
    if setup['coordinate_convention'] != 'FLU' or setup['rotation_convention'] != ROTATION_CONVENTION:
        raise ValueError('hardware_setup requires FLU and explicit ' + ROTATION_CONVENTION)

    reference = _object(setup['calibration_reference'], {'axle_rpy_deg', 'status'},
                        'calibration_reference')
    if not isinstance(reference['status'], str) or reference['status'] not in REFERENCE_STATUSES:
        raise ValueError('calibration_reference.status must state assumed level or user-measured experiment')
    r_survey_axle = rpy_rotation(reference['axle_rpy_deg'])
    if reference['status'] == 'ASSUMED_LEVEL_NOT_MEASURED' and \
            any(angle != 0 for angle in reference['axle_rpy_deg']):
        raise ValueError('an assumed-level reference requires explicit zero axle_rpy_deg')

    lidars = _object(setup['lidars'], {'left', 'right'}, 'lidars')
    mounts = {}
    for side in ('left', 'right'):
        mount = _object(lidars[side], {'position_m', 'rpy_deg', 'rotation_reference'}, 'lidars.' + side)
        position = _position(mount['position_m'], 'lidars.' + side + '.position_m')
        measured_rotation = rpy_rotation(mount['rpy_deg'])
        basis = mount['rotation_reference']
        if basis == 'calibration_level':
            r_axle_lidar = r_survey_axle.T @ measured_rotation
        elif basis == 'axle':
            r_axle_lidar = measured_rotation
        else:
            raise ValueError('lidars.' + side + '.rotation_reference must be calibration_level or axle')
        mounts[side] = {'t_axle_lidar_m': position.tolist(), 'R_axle_lidar': r_axle_lidar.tolist()}

    imu = _object(setup['imu'], {'parent_frame', 'position_m', 'rpy_deg'}, 'imu')
    parent = imu['parent_frame']
    if not isinstance(parent, str) or parent not in ('lidar_left', 'lidar_right', 'axle'):
        raise ValueError('imu.parent_frame must be lidar_left, lidar_right or axle')
    r_parent_imu = rpy_rotation(imu['rpy_deg'])
    position = None if imu['position_m'] is None else _position(imu['position_m'], 'imu.position_m')
    if parent == 'axle':
        r_axle_parent, t_axle_parent = np.eye(3), np.zeros(3)
    else:
        mount = mounts[parent[len('lidar_'):]]
        r_axle_parent = np.asarray(mount['R_axle_lidar'])
        t_axle_parent = np.asarray(mount['t_axle_lidar_m'])
    t_axle_imu = None if position is None else t_axle_parent + r_axle_parent @ position
    if t_axle_imu is not None and np.linalg.norm(t_axle_imu) > 3:
        raise ValueError('resolved IMU position must be within 3 m of axle origin')

    wheel = _object(setup['wheel_odometry'], WHEEL_FIELDS, 'wheel_odometry')
    if not isinstance(wheel['provenance'], str) or not wheel['provenance'].strip():
        raise ValueError('wheel_odometry.provenance must be non-empty text')
    wheel_candidate = copy.deepcopy(wheel)
    HistoryPreview(wheel_candidate)
    resolved = {'mounts': mounts,
            'imu_mount': {'R_axle_imu': (r_axle_parent @ r_parent_imu).tolist(),
                          'parent_frame': parent,
                          't_parent_imu_m': None if position is None else position.tolist(),
                          't_axle_imu_m': None if t_axle_imu is None else t_axle_imu.tolist(),
                          'translation_used': False},
            'wheel_candidate': wheel_candidate,
            'mount_model': 'fixed_hardware_setup_v1'}
    # Older explicit mount setups remain valid; missing display dimensions
    # never become invented physical measurements or implicit model defaults.
    if 'visualization' in setup:
        resolved['visualization'] = validate_visualization(setup['visualization'])
    if 'manual_controls' in setup:
        from .mapping_wheel import validate_manual_controls
        resolved['manual_controls'] = validate_manual_controls(setup['manual_controls'])
    return resolved


def _resolve_v2(value):
    from .calibration_geometry import (TRANSFORM_CONVENTION, _text, validate_sources,
        validate_mechanical_catalog, resolve_data_graph, capability_assessment)
    setup = _object(value, {'schema_version', 'status', 'coordinate_convention',
        'rotation_convention', 'transform_convention', 'profile', 'assembly',
        'source_documents', 'mechanical_reference', 'data_transforms', 'wheel_odometry'},
        'hardware_setup_v2', optional={'visualization', 'manual_controls', 'legacy_profile'})
    if type(setup['schema_version']) is not int or setup['schema_version'] != 2 or setup['status'] != 'EXPERIMENT':
        raise ValueError('hardware_setup_v2 requires schema_version 2 and status EXPERIMENT')
    if setup['coordinate_convention'] != 'FLU' or setup['rotation_convention'] != ROTATION_CONVENTION:
        raise ValueError('hardware_setup_v2 requires FLU and explicit ' + ROTATION_CONVENTION)
    if setup['transform_convention'] != TRANSFORM_CONVENTION:
        raise ValueError('hardware_setup_v2 requires explicit T_parent_child direction')
    _text(setup['profile'], 'hardware_setup_v2.profile')
    assembly = _object(setup['assembly'], {'revision', 'confirmed', 'components', 'note'}, 'assembly')
    revision = _text(assembly['revision'], 'assembly.revision')
    if type(assembly['confirmed']) is not bool:
        raise ValueError('assembly.confirmed must be an explicit boolean')
    if not isinstance(assembly['components'], dict) or not assembly['components']:
        raise ValueError('assembly.components requires documented mechanical versions')
    for key, entry in assembly['components'].items():
        _text(key, 'assembly component name'); _text(entry, 'assembly component version')
    _text(assembly['note'], 'assembly.note')
    sources = validate_sources(setup['source_documents'])
    mechanical = validate_mechanical_catalog(setup['mechanical_reference'], sources)
    full, rotations, unresolved = resolve_data_graph(setup['data_transforms'], sources, revision)
    mounts = {}
    for side in ('left', 'right'):
        matrix = full['axle'].get('lidar_' + side)
        if matrix is not None:
            mounts[side] = {'t_axle_lidar_m': matrix[:3, 3].tolist(), 'R_axle_lidar': matrix[:3, :3].tolist()}
    imu_rotation = rotations['axle'].get('imu_native')
    imu_transform = full['axle'].get('imu_native')
    imu_position = None if imu_transform is None else imu_transform[:3, 3].tolist()
    wheel = _object(setup['wheel_odometry'], WHEEL_FIELDS, 'wheel_odometry')
    _text(wheel['provenance'], 'wheel_odometry.provenance')
    HistoryPreview(copy.deepcopy(wheel))
    capabilities = capability_assessment(full, rotations, assembly_confirmed=assembly['confirmed'])
    report = {'schema_version': 1, 'profile': setup['profile'], 'assembly_revision': revision,
        'assembly_confirmed': assembly['confirmed'], 'capabilities': copy.deepcopy(capabilities),
        'unresolved_components': unresolved, 'source_documents': list(sources.values()),
        'mechanical_reference': mechanical,
        'resolved_data_transforms': {parent: {child: matrix.tolist() for child, matrix in children.items()
                                            if child != parent} for parent, children in full.items()},
        'limitations': ['能力 AVAILABLE 只表示外参依赖完整，不认证物理精度、设备状态、时间同步或运动授权。',
                        '机械面中心、外形中心、IMU 芯片封装中心不能自动成为数据原点。',
                        '当前仅使用 IMU 角速度/静止重力，不使用未知 IMU 平移或加速度杆臂。',
                        '轮参数仍为 UNVALIDATED 候选；高度筛选还假设地面平行于轮轴 XY 平面。'],
        'hardware_started': False, 'configuration_written': False}
    resolved = {'mounts': mounts,
        'imu_mount': {'R_axle_imu': None if imu_rotation is None else imu_rotation.tolist(),
                      'parent_frame': 'axle', 't_parent_imu_m': imu_position,
                      't_axle_imu_m': imu_position, 'translation_used': False},
        'wheel_candidate': copy.deepcopy(wheel), 'mount_model': 'fixed_hardware_setup_v2',
        'geometry_capabilities': capabilities, 'geometry_report': report}
    if 'legacy_profile' in setup:
        legacy = _object(setup['legacy_profile'], {'path', 'sha256', 'active', 'note'}, 'legacy_profile')
        from .calibration_geometry import _safe_snapshot_path
        _safe_snapshot_path(legacy['path'])
        if legacy['active'] is not False:
            raise ValueError('V7 legacy_profile must remain inactive; old candidates are not current mounting evidence')
        if not isinstance(legacy['sha256'], str) or len(legacy['sha256']) != 64 or \
                any(c not in '0123456789abcdef' for c in legacy['sha256']):
            raise ValueError('legacy_profile.sha256 requires a lowercase SHA256')
        _text(legacy['note'], 'legacy_profile.note')
        report['legacy_profile'] = copy.deepcopy(legacy)
    if 'visualization' in setup:
        resolved['visualization'] = validate_visualization(setup['visualization'])
    if 'manual_controls' in setup:
        from .mapping_wheel import validate_manual_controls
        resolved['manual_controls'] = validate_manual_controls(setup['manual_controls'])
    return resolved


def resolve_hardware_setup(value, *, required_capabilities=()):
    """Resolve schema1 legacy or schema2 sourced geometry, without hardware.

    Schema2 UNKNOWN components remain absent/None and block only capabilities
    needing them. Native capture/preview never inherits old mount candidates.
    Schema1 matrix math remains unchanged for historical replay; explicit new
    motion gates refuse automatic use of that profile for the current V7.
    """
    from .calibration_geometry import FRAMES, capability_assessment, require_geometry_capability
    if not isinstance(value, dict) or type(value.get('schema_version')) is not int:
        raise ValueError('hardware_setup requires an explicit integer schema_version')
    if value['schema_version'] == 2:
        resolved = _resolve_v2(value)
    elif value['schema_version'] == 1:
        resolved = _resolve_v1(value)
        disconnected = {frame: {} for frame in FRAMES}
        capabilities = capability_assessment(disconnected, disconnected, assembly_confirmed=False, legacy=True)
        capabilities['mechanical_layout'] = {'status': 'BLOCKED', 'reasons': ['schema1 不包含 V7 的 M 系机械资料'],
                                            'requires': ['V7 mechanical_reference'], 'validation_level': 'CONFIG_ONLY'}
        resolved['geometry_capabilities'] = capabilities
        resolved['geometry_report'] = {'schema_version': 1, 'profile': 'legacy_configuration_v1',
            'assembly_revision': 'UNKNOWN_LEGACY', 'assembly_confirmed': False,
            'capabilities': copy.deepcopy(capabilities), 'source_documents': [], 'unresolved_components': [],
            'limitations': ['历史 schema1 数学兼容，不代表当前 V7 已标定；旧数值不会自动成为 V7 外参。'],
            'hardware_started': False, 'configuration_written': False}
    else:
        raise ValueError('unsupported hardware_setup schema_version')
    if not isinstance(required_capabilities, (list, tuple)):
        raise ValueError('required_capabilities must be an explicit list/tuple')
    for name in required_capabilities:
        require_geometry_capability(resolved, name)
    return resolved
