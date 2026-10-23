"""Explicit real-time reuse of uncalibrated mechanical and motion candidates.

The offline candidate remains an unchanged, same-recording artifact. This
separate opt-in permits only its fixed numerical coefficients in a new causal
session; neither physical calibration nor independent bias is asserted.
"""
import copy
import hashlib
import json
from pathlib import Path
import re

import numpy as np

from .calibration_geometry import verify_geometry_sources, proper_rotation
from .compare_geometry import mechanical_initial_geometry
from .offline_motion_calibration import validate_for_session
from .prepare_picker_input import _json_loads
from .storage_policy import StoragePolicy


STATUS = 'EXPERIMENTAL_REUSE_NOT_CALIBRATED'
SCOPE = 'EXPLICIT_REALTIME_EXPERIMENT'
MOUNT_MODEL = 'live_experimental_mechanical_initial'
DEFAULT_PROFILE = 'config/calibration/live_experiment_profile.json'


def canonical_sha256(value):
    return hashlib.sha256(json.dumps(value, ensure_ascii=False, sort_keys=True,
        separators=(',', ':'), allow_nan=False).encode('utf-8')).hexdigest()


def _sha(value):
    return isinstance(value, str) and re.fullmatch('[0-9a-f]{64}', value) is not None


def _read_json(path, limit, label):
    if not path.is_file() or path.is_symlink() or path.stat().st_size > limit:
        raise ValueError(label + ' missing, nonordinary or too large')
    raw = path.read_bytes()
    if len(raw) > limit:
        raise ValueError(label + ' too large')
    text = raw.decode('utf-8')
    return _json_loads(text), text, hashlib.sha256(raw).hexdigest()


def validate_motion_parameters(value, config):
    """Validate a frozen coefficient-only snapshot, with no old time model."""
    fields = {'schema_version', 'status', 'scope', 'imu_sensor_id', 'wheel_device_id',
        'R_reference_imu', 'bias_native_rad_s', 'wheel_yaw_scale',
        'wheel_yaw_speed_coefficient', 'candidate_id', 'candidate_sha256',
        'profile_sha256', 'source_session_id', 'assembly_revision',
        'hardware_setup_content_sha256', 'physical_accuracy_validated',
        'gyro_bias_independently_confirmed', 'source_time_policy', 'parameters_sha256'}
    if not isinstance(value, dict) or set(value) != fields:
        raise ValueError('实时实验需要已核对来源的独立实验参数快照')
    if (value['schema_version'] != 1 or value['status'] != STATUS or value['scope'] != SCOPE or
            value['physical_accuracy_validated'] is not False or
            value['gyro_bias_independently_confirmed'] is not False or
            value['source_time_policy'] != 'HOST_ARRIVAL_ONLY_NO_OFFLINE_ANCHOR'):
        raise ValueError('实验参数不能声明已标定、独立零偏或复用离线时间锚点')
    expected_wheel = config.get('wheel_device_id', config.get('wheel_candidate', {}).get('device_id'))
    if (value['imu_sensor_id'] != config.get('imu_sensor_id') or not expected_wheel or
            value['wheel_device_id'] != expected_wheel):
        raise ValueError('实时实验 IMU 或轮设备身份与当前会话不匹配')
    proper_rotation(value['R_reference_imu'], 'experimental R_reference_imu')
    bias = np.asarray(value['bias_native_rad_s'])
    if bias.shape != (3,) or bias.dtype.kind not in 'iuf' or not np.isfinite(bias).all() or np.linalg.norm(bias) > .0015:
        raise ValueError('实验候选零偏必须为有限原生轴向量并在原候选界限内')
    numbers = np.asarray([value['wheel_yaw_scale'], value['wheel_yaw_speed_coefficient']])
    if (numbers.dtype.kind not in 'iuf' or not np.isfinite(numbers).all() or
            not .5 <= numbers[0] <= 1.5 or abs(numbers[1]) > .1):
        raise ValueError('实验轮修正参数超出离线候选原有界限')
    for key in ('candidate_id', 'candidate_sha256', 'profile_sha256', 'hardware_setup_content_sha256', 'parameters_sha256'):
        if not _sha(value[key]):
            raise ValueError('实时实验缺少有效来源 SHA-256: ' + key)
    for key in ('source_session_id', 'assembly_revision'):
        if not isinstance(value[key], str) or not value[key].strip():
            raise ValueError('实时实验缺少来源/装配身份: ' + key)
    if canonical_sha256({k: v for k, v in value.items() if k != 'parameters_sha256'}) != value['parameters_sha256']:
        raise ValueError('实时实验参数快照内容哈希不一致')
    return copy.deepcopy(value)


def prepare_experiment(root, setup, resolved, *, imu_sensor_id, wheel_device_id,
                       profile_path=DEFAULT_PROFILE):
    """Read and validate available evidence; never open devices or write files.

    Original recording source digests may describe a selected-message hash,
    rather than a file hash. They are retained with their original definitions;
    this check verifies the candidate artifact, not a fresh multi-GB bag replay.
    """
    policy = StoragePolicy(Path(root))
    path = policy.resolve(profile_path)
    profile, profile_text, profile_sha = _read_json(path, 32768, 'Real-time experiment profile')
    required = {'schema_version', 'status', 'scope', 'assembly_revision',
        'hardware_setup_content_sha256', 'imu_sensor_id', 'wheel_device_id',
        'motion_candidate', 'limitations'}
    if (not isinstance(profile, dict) or set(profile) != required or profile['schema_version'] != 1 or
            profile['status'] != STATUS or profile['scope'] != SCOPE):
        raise ValueError('实时实验方案须明确保留未标定状态和显式复用范围')
    if profile['imu_sensor_id'] != imu_sensor_id or profile['wheel_device_id'] != wheel_device_id:
        raise ValueError('实时实验方案与当前 IMU/轮设备身份不匹配')
    assembly = setup.get('assembly', {})
    revision = resolved.get('geometry_report', {}).get('assembly_revision')
    if (not revision or profile['assembly_revision'] != revision or
            resolved.get('geometry_report', {}).get('assembly_confirmed') is not True):
        raise ValueError('实时实验方案与当前已确认装配不匹配')
    if profile['hardware_setup_content_sha256'] != canonical_sha256(setup):
        raise ValueError('实时实验方案的当前硬件安装内容 SHA-256 不一致')
    if not isinstance(profile['limitations'], list) or not profile['limitations'] or any(
            not isinstance(item, str) or not item.strip() for item in profile['limitations']):
        raise ValueError('实时实验方案必须保留未标定限制说明')
    sources = verify_geometry_sources(setup, Path(root))
    if sources['status'] != 'PASS':
        raise ValueError('实时实验机械资料原件缺失或 SHA-256 不一致')
    initial = mechanical_initial_geometry(setup, resolved)
    reference = copy.deepcopy(initial['offline_geometry_initialization'])
    candidate_ref = profile['motion_candidate']
    if (not isinstance(candidate_ref, dict) or set(candidate_ref) != {'source_path', 'sha256'} or
            not isinstance(candidate_ref['source_path'], str) or
            not candidate_ref['source_path'].startswith('data/analysis/') or not _sha(candidate_ref['sha256'])):
        raise ValueError('实时实验须引用 data/analysis 中原始离线候选及其 SHA-256')
    candidate_path = policy.resolve(candidate_ref['source_path'])
    candidate, candidate_text, candidate_sha = _read_json(candidate_path, 262144, 'Original offline motion candidate')
    if candidate_sha != candidate_ref['sha256']:
        raise ValueError('原始离线运动候选 SHA-256 不一致')
    provenance = candidate.get('provenance', {})
    # Validate the source in its ORIGINAL session. This does not relabel the
    # candidate as valid_for_session(new_live_session) or change its scope.
    validate_for_session(candidate, session_id=provenance.get('session_id'),
        imu_device_id=imu_sensor_id, wheel_device_id=wheel_device_id)
    if candidate.get('formal_calibration', False) is not False or candidate.get('automatic_apply', False) is not False:
        raise ValueError('原离线候选必须保留非正式标定、非自动应用状态')
    if not np.allclose(provenance['R_reference_imu'], initial['imu_mount']['R_axle_imu'], atol=1e-9, rtol=0):
        raise ValueError('原离线候选 IMU 旋转与当前安装不一致')
    conversion = provenance.get('wheel_conversion')
    if not isinstance(conversion, dict) or any(conversion.get(key) != initial['wheel_candidate'].get(key)
            for key in ('wheel_radius_m', 'track_width_m', 'register_to_wheel_rpm', 'left_sign', 'right_sign')):
        raise ValueError('原离线候选轮转换参数与当前实验几何不一致')
    parameters = {'schema_version': 1, 'status': STATUS, 'scope': SCOPE,
        'imu_sensor_id': imu_sensor_id, 'wheel_device_id': wheel_device_id,
        'R_reference_imu': copy.deepcopy(provenance['R_reference_imu']),
        'bias_native_rad_s': copy.deepcopy(candidate['bias_native_rad_s']),
        'wheel_yaw_scale': candidate['wheel_yaw_scale'],
        'wheel_yaw_speed_coefficient': candidate['wheel_yaw_speed_coefficient'],
        'candidate_id': candidate['candidate_id'], 'candidate_sha256': candidate_sha,
        'profile_sha256': profile_sha, 'source_session_id': provenance['session_id'],
        'assembly_revision': revision, 'hardware_setup_content_sha256': canonical_sha256(setup),
        'physical_accuracy_validated': False, 'gyro_bias_independently_confirmed': False,
        'source_time_policy': 'HOST_ARRIVAL_ONLY_NO_OFFLINE_ANCHOR'}
    parameters['parameters_sha256'] = canonical_sha256(parameters)
    parameters = validate_motion_parameters(parameters,
        {'imu_sensor_id': imu_sensor_id, 'wheel_device_id': wheel_device_id})
    evidence = {'status': STATUS, 'scope': SCOPE,
        'profile_source_path': policy.logical(path), 'profile_sha256': profile_sha,
        'profile_snapshot_file': 'live_experiment_profile.json',
        'candidate_source_path': policy.logical(candidate_path), 'candidate_sha256': candidate_sha,
        'candidate_snapshot_file': 'live_experiment_motion_candidate.json',
        'candidate_id': candidate['candidate_id'], 'source_session_id': provenance['session_id'],
        'assembly_revision': revision, 'hardware_setup_content_sha256': canonical_sha256(setup),
        'original_candidate_status': candidate['status'], 'original_candidate_scope': candidate.get('scope'),
        'original_source_hashes': copy.deepcopy(provenance['sources']),
        'original_source_hash_validation': 'PRESERVED_HISTORICAL_DEFINITIONS_NOT_REREAD_DURING_LIVE_START',
        'candidate_file_sha256_verified': True, 'candidate_content_digest_verified': True,
        'physical_accuracy_validated': False, 'gyro_bias_independently_confirmed': False,
        'offline_time_model_applied': False, 'limitations': copy.deepcopy(profile['limitations'])}
    geometry = copy.deepcopy(initial)
    geometry.pop('offline_geometry_initialization')
    geometry.update(mount_model=MOUNT_MODEL, offline_experiment=False, live_scheme='experiment')
    initialization = copy.deepcopy(reference)
    initialization.update(status=STATUS, scope=SCOPE, offline_only=False,
        live_experimental_eligible=True, live_eligible=False,
        source_hash_verification_required_by_caller=False, source_verification=sources,
        profile_sha256=profile_sha, candidate_sha256=candidate_sha,
        offline_formula_reference=reference)
    geometry['live_experiment_initialization'] = initialization
    for mode in ('left', 'right', 'all'):
        geometry['geometry_capabilities']['wheel_imu_motion_' + mode].update(
            validation_level='REALTIME_EXPERIMENT_UNCALIBRATED', offline_only=False,
            experimental_only=True, initialization_status=STATUS)
    geometry['geometry_report'].pop('offline_geometry_initialization')
    geometry['geometry_report'].update(validation_level='REALTIME_EXPERIMENT_UNCALIBRATED',
        offline_only=False, live_experiment_initialization=copy.deepcopy(initialization),
        source_verification=sources, capabilities=copy.deepcopy(geometry['geometry_capabilities']))
    geometry['geometry_report']['limitations'][-1] = (
        '显式实时实验：复用雷达机械面中心初值和离线运动候选；原始 UNKNOWN、来源及未标定限制保留。')
    return {'resolved_geometry': geometry, 'motion_parameters': parameters,
        'provenance': evidence, 'profile': profile,
        'profile_raw_text': profile_text, 'candidate_raw_text': candidate_text}


def experiment_capability(root, setup, resolved, *, imu_sensor_id, wheel_device_id,
                          profile_path=DEFAULT_PROFILE):
    try:
        prepared = prepare_experiment(root, setup, resolved, imu_sensor_id=imu_sensor_id,
            wheel_device_id=wheel_device_id, profile_path=profile_path)
        return {'available': True, 'reason': '', 'provenance': prepared['provenance']}
    except (ValueError, OSError, KeyError, TypeError) as error:
        return {'available': False, 'reason': str(error), 'provenance': None}


def is_experiment_geometry(config):
    """Recognize the separate live opt-in, never the old offline-only model."""
    initial = config.get('live_experiment_initialization')
    if (config.get('live_scheme', 'validated') != 'experiment' or config.get('offline_experiment') is not False or
            config.get('mount_model') != MOUNT_MODEL or not isinstance(initial, dict) or
            initial.get('status') != STATUS or initial.get('scope') != SCOPE or
            initial.get('calibration_validated') is not False or initial.get('live_eligible') is not False or
            initial.get('live_experimental_eligible') is not True):
        return False
    try:
        parameters = validate_motion_parameters(config.get('live_experiment_motion'), config)
        if (initial['setup_content_sha256'] != parameters['hardware_setup_content_sha256'] or
                initial['profile_sha256'] != parameters['profile_sha256'] or
                initial['candidate_sha256'] != parameters['candidate_sha256']):
            return False
        for side in ('left', 'right'):
            transform = np.asarray(initial['initial_transforms']['T_axle_lidar_' + side])
            mount = config['mounts'][side]
            if (not np.array_equal(transform[:3, 3], mount['t_axle_lidar_m']) or
                    not np.array_equal(transform[:3, :3], mount['R_axle_lidar'])):
                return False
        return True
    except (ValueError, KeyError, TypeError, IndexError):
        return False
