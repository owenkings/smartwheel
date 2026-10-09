"""Explicit live algorithm options, device-bound calibration and causal geometry.

Offline candidates retain their session-only guard. Nothing here loads one,
infers stationarity, or silently turns an unavailable option off.
"""
from collections import deque
from dataclasses import asdict
import copy
import hashlib
import json
import math
import re

import numpy as np
from scipy.spatial.transform import Rotation

from .mapping_planar import validate_confirmed_bias
from .offline_planar_process import validate_process_noise


DEFAULT_CONTINUOUS_PROCESS_NOISE = {
    'model': 'continuous_white_acceleration', 'status': 'EXPERIMENTAL_NOT_CALIBRATED',
    'linear_acceleration_psd_m2_s3': .25, 'angular_acceleration_psd_rad2_s3': .25,
}


def validate_live_options(config):
    """Return a normalized copy; reject missing conditions before device startup.

    Mount-derived IMU rotation is compared after initialization, when it exists.
    The default PSD is an explicit independent experimental preset, shared with
    the offline refinement default; legacy acceleration variance is never read.
    """
    result = copy.deepcopy(config)
    result.setdefault('motion_correction', False)
    result.setdefault('process_noise', 'legacy')
    result.setdefault('geometry', False)
    for key in ('motion_correction', 'geometry'):
        if type(result[key]) is not bool:
            raise ValueError(key + ' must be boolean')
    if result['process_noise'] not in ('legacy', 'white_acceleration'):
        raise ValueError('process_noise must be legacy or white_acceleration')
    estimator = result.get('estimator', result.get('wheel_imu_estimator', 'five_state'))
    if estimator not in ('five_state', 'robot_localization'):
        raise ValueError('Unknown live EKF provider')
    enabled = result['motion_correction'] or result['geometry'] or result['process_noise'] != 'legacy'
    if enabled and (result.get('motion_model') != 'planar_ekf' or not result.get('continuous_mapping')):
        raise ValueError('Live correction options require continuous planar EKF mapping')
    if estimator != 'five_state' and result['geometry']:
        raise ValueError('官方 EKF 几何反馈尚未实现；此选项不可选择')
    if estimator != 'five_state' and result['process_noise'] != 'legacy':
        raise ValueError('五状态连续噪声 PSD 不能套用于官方 EKF')
    if result['process_noise'] == 'white_acceleration':
        result['continuous_process_noise'] = validate_process_noise(
            result.get('continuous_process_noise', DEFAULT_CONTINUOUS_PROCESS_NOISE))
        if result['continuous_process_noise'] is None:
            raise ValueError('Continuous process noise requires explicit non-null PSD values')
    if result['motion_correction']:
        if result.get('offline_experiment') or result.get('offline_motion_correction') is not None:
            raise ValueError('Live device calibration and session-only offline candidates cannot be combined')
        calibration = validate_live_calibration(result.get('live_motion_calibration'), result)
        result['live_motion_calibration'] = calibration
        supplied = validate_confirmed_bias(result.get('confirmed_gyro_bias'),
                                            expected_sensor_id=result.get('imu_sensor_id'))
        if supplied is not None and supplied != calibration['confirmed_gyro_bias']:
            raise ValueError('Live calibration conflicts with separately configured gyro bias; refusing double correction')
        result['confirmed_gyro_bias'] = copy.deepcopy(calibration['confirmed_gyro_bias'])
    return result


def validate_live_calibration(value, config):
    required = {'schema_version', 'status', 'scope', 'imu_sensor_id', 'wheel_device_id',
                'R_reference_imu', 'wheel_yaw_scale', 'wheel_yaw_speed_coefficient',
                'confirmed_gyro_bias', 'evidence_id', 'evidence_sha256'}
    if not isinstance(value, dict) or set(value) != required:
        raise ValueError('运动修正需要已保存的设备校正声明：IMU/轮身份、旋转、轮偏航修正及独立陀螺零偏证据')
    if value['schema_version'] != 1 or value['status'] != 'EXPLICIT_DEVICE_CALIBRATION_DECLARATION' or value['scope'] != 'device':
        raise ValueError('在线仅接受显式设备校正声明，不能直接复用某次录包的离线候选')
    expected_wheel = config.get('wheel_device_id', config.get('wheel_candidate', {}).get('device_id'))
    if value['imu_sensor_id'] != config.get('imu_sensor_id') or not expected_wheel or value['wheel_device_id'] != expected_wheel:
        raise ValueError('在线运动修正的 IMU 或轮设备身份与当前会话不匹配')
    rotation = np.asarray(value['R_reference_imu'])
    if rotation.shape != (3, 3) or rotation.dtype.kind not in 'iuf' or not np.isfinite(rotation).all() or not np.allclose(
            rotation.T @ rotation, np.eye(3), atol=1e-7, rtol=0) or not np.isclose(np.linalg.det(rotation), 1., atol=1e-7):
        raise ValueError('Live calibration requires a proper explicit R_reference_imu rotation')
    for key in ('wheel_yaw_scale', 'wheel_yaw_speed_coefficient'):
        number = value[key]
        if isinstance(number, bool) or not isinstance(number, (int, float)) or not math.isfinite(number):
            raise ValueError('Finite wheel correction parameters required')
    if not .5 <= value['wheel_yaw_scale'] <= 1.5 or abs(value['wheel_yaw_speed_coefficient']) > .5:
        raise ValueError('Wheel correction exceeds the explicit experimental bounds: scale [0.5,1.5], speed coefficient +/-0.5 rad/m')
    if not isinstance(value['evidence_id'], str) or not value['evidence_id'].strip() or len(value['evidence_id']) > 256:
        raise ValueError('Live calibration requires a bounded evidence ID')
    if not isinstance(value['evidence_sha256'], str) or not re.fullmatch('[0-9a-fA-F]{64}', value['evidence_sha256']):
        raise ValueError('Live calibration requires an evidence SHA-256')
    result = copy.deepcopy(value)
    result['confirmed_gyro_bias'] = validate_confirmed_bias(value['confirmed_gyro_bias'],
                                                           expected_sensor_id=value['imu_sensor_id'])
    if result['confirmed_gyro_bias'] is None:
        raise ValueError('运动修正缺少独立证据声明的陀螺零偏，不能自动假定为零或静止')
    return result


def calibration_report(config):
    if not config.get('motion_correction'):
        return {'enabled': False, 'status': 'DISABLED'}
    calibration = config['live_motion_calibration']
    digest = hashlib.sha256(json.dumps(calibration, sort_keys=True, separators=(',', ':'),
                                      allow_nan=False).encode()).hexdigest()
    return {'enabled': True, 'status': 'EXPLICIT_DEVICE_CALIBRATION_APPLIED',
            'calibration_sha256': digest, 'evidence_id': calibration['evidence_id'],
            'evidence_sha256': calibration['evidence_sha256'],
            'wheel_yaw_scale': calibration['wheel_yaw_scale'],
            'wheel_yaw_speed_coefficient': calibration['wheel_yaw_speed_coefficient'],
            'bias_native_rad_s': calibration['confirmed_gyro_bias']['bias_native_rad_s'],
            'physical_accuracy_validated': False, 'stationarity_automatically_inferred': False,
            'wheel_covariance_policy': 'same J R J^T as offline corrected-wheel observation',
            'gyro_bias_policy': 'CausalBiasEstimator versioned native-axis subtraction exactly once'}


class LiveGeometry:
    """Causal bounded geometry corrects published poses, not EKF measurements.

    Uses the same registration engine and uncertainty budget as the offline
    geometry cell. Only corrections known at this output are interpolated for
    its two lidar arrival times; no future scan or prior output is rewritten.
    """
    def __init__(self):
        from .offline_planar_registration import TrajectoryRefiner
        self.engine = TrajectoryRefiner()
        self.corrections = deque(maxlen=128)
        self.last_pose = None
        self.last_stamp = None

    def correct(self, stamp, points, offsets, prior_pose, report):
        from .offline_geometry_replay import redeskew_points
        registration = self.engine.update(stamp, points, prior_pose)
        pose = np.asarray(registration['T_world_scan'], dtype=float)
        correction = pose @ np.linalg.inv(prior_pose)
        self.corrections.append((stamp, correction))
        # Keep a bounded source-time history covering more than the 50 ms pair
        # offset. The deque capacity is an additional hard memory limit.
        while len(self.corrections) > 2 and self.corrections[1][0] < stamp - 2_000_000_000:
            self.corrections.popleft()
        stamps = np.array([item[0] for item in self.corrections], dtype=np.int64)
        corrected, maximum = redeskew_points(points, offsets, stamp, prior_pose, pose, stamps,
                                              [item[1] for item in self.corrections])
        result = copy.deepcopy(report)
        result['T_uncorrected_prior_reference'] = prior_pose.tolist()
        result['uncorrected_motion_report'] = copy.deepcopy(report)
        result['T_prior_reference'] = pose.tolist()
        velocity, angular = np.zeros(3), np.zeros(3)
        if self.last_pose is not None:
            dt = (stamp-self.last_stamp)*1e-9
            velocity = pose[:3, :3].T @ (pose[:3, 3]-self.last_pose[:3, 3])/dt
            angular[2] = Rotation.from_matrix(self.last_pose[:3, :3].T @ pose[:3, :3]).as_rotvec()[2]/dt
        result['linear_velocity_reference_m_s'] = velocity.tolist()
        result['angular_velocity_reference_rad_s'] = angular.tolist()
        jacobian = np.eye(6)
        jacobian[:3, :3] = correction[:3, :3]
        jacobian[3:, 3:] = correction[:3, :3]
        result['pose_covariance'] = (jacobian @ np.array(report['pose_covariance']).reshape(6, 6) @ jacobian.T +
            np.diag([.25, .25, 0., 0., 0., math.radians(15.)**2])).reshape(-1).tolist()
        result['twist_covariance'] = (np.array(report['twist_covariance']).reshape(6, 6) + np.eye(6)).reshape(-1).tolist()
        result['covariance_status'] = 'UNKNOWN_GEOMETRY_UNCERTAINTY_WITH_EXPLICIT_UNVALIDATED_BUDGET'
        result['live_geometry'] = {'registration': registration, 'enabled': True,
            'published_pose_corrected': True, 'ekf_measurement_feedback': False,
            'method': 'same bounded SE2 point-to-plane registration as offline geometry',
            'redeskew_max_point_displacement_m': maximum,
            'within_pair_correction': 'past-to-current interpolation using only scans received by current publication',
            'offline_difference': 'offline may interpolate against later scans; live never reads a future scan',
            'source_offsets_ns': sorted(map(int, np.unique(offsets))),
            'source_time_basis': 'host arrival, not per-point timing',
            'policy': asdict(self.engine.policy), 'summary': self.engine.summary()}
        self.last_pose, self.last_stamp = pose.copy(), stamp
        return corrected, pose, result
