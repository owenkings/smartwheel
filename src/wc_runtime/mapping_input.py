"""Validated mapping input and experimental gravity bootstrap; never opens devices.

Preserves authoritative source CDR using an explicit publication contract.
The default waits for writes; queued_realtime publishes fresh clouds after
bounded immutable in-memory acceptance. Only the final checkpoint certifies
that every accepted source envelope has reached durable storage.
Dual-source time offsets remain explicit arrival-time observations, not validated
measurement synchronization. Mounts remain user-measured experiment candidates.
"""
import argparse
from collections import deque
import copy
import hashlib
import json
import math
import os
from pathlib import Path
import queue
import shutil
import signal
import sys
import threading
import time
import traceback
from types import SimpleNamespace

import numpy as np

from wc_motion.protocol import parse_exchange
from wc_sensors.pointcloud import decode_pointcloud2, validate_pointcloud2
from .mapping_cloud import selected_cloud_source, source_frame_topic, cloud_description
from .mapping_filter import resolve_cloud_filter, effective_enabled, filter_cloud
from .single_mapping_input import (SourceGate, FrameArchive, InputFailure, project_path,
    _require, _uint, _stamp, _json_bytes, _sync_dir, STALE_NS, STARTUP_NS,
    MAX_FRAMES, MAX_CDR_BYTES, MIN_FREE_BYTES)


BOOT_WINDOW_NS = 2_000_000_000
BOOT_GAP_NS = 500_000_000
MAX_GYRO_RAD_S = .05
PAIR_DELTA_NS = 50_000_000
MAX_WINDOW_SAMPLES = 2000
QUERY_HEX = '010320ab0002be2b'
OUTPUT_TOPIC = '/wc_mapping/app/input_cloud'
IMU_TOPIC = '/wc_mapping/imu/source_frame'
WHEEL_TOPIC = '/wc_mapping/wheel/feedback_raw'
MAX_PENDING_OUTPUTS = 10
MAX_REALTIME_PENDING_OUTPUTS = 100
MAX_PENDING_BYTES = 128 * 1024 * 1024
QUEUED_OBJECT_ALLOWANCE_BYTES = 4096
# LIVE_STATIC 06 measured a 21.6446s checkpoint. A captured prefix advances
# only after sync completes: two such commits plus the 1s interval at 5Hz
# account for ~222 outputs. 256 is an experimental backlog stop budget, not
# a promised time/crash-loss bound. RAM remains 100 jobs / 128 MiB separately.
MAX_UNCHECKPOINTED_OUTPUTS = 256
CHECKPOINT_INTERVAL_S = 1.0
CHECKPOINT_DATA_WORKERS = 4
CHECKPOINT_POLICIES = ('periodic', 'on_close')
CLOSE_TIMEOUT_S = 20.0


def qos_profiles(profile, reliability, durability):
    """Health/bootstrap consumers need latest samples after durable I/O stalls.

    The bag remains the full-rate raw record. Keeping 200 H30 samples here can
    replay almost a second of obsolete callbacks after fsync and falsely fail
    the unchanged 0.5-second freshness gate despite a live, healthy producer.
    """
    common = {'durability': durability.VOLATILE}
    return {'imu': profile(depth=1, reliability=reliability.BEST_EFFORT, **common),
            'wheel': profile(depth=1, reliability=reliability.RELIABLE, **common),
            'lidar': profile(depth=2, reliability=reliability.BEST_EFFORT, **common),
            'output': profile(depth=2, reliability=reliability.RELIABLE, **common)}


def vector(value, size, reason):
    _require(isinstance(value, (list, tuple)) and len(value) == size and
             all(type(x) in (int, float) and math.isfinite(x) for x in value), reason)
    return np.asarray(value, dtype=float)


def rotation(value, reason):
    result = np.asarray(value, dtype=float)
    _require(result.shape == (3, 3) and np.isfinite(result).all() and
             np.allclose(result.T @ result, np.eye(3), atol=1e-6, rtol=0) and
             abs(np.linalg.det(result)-1) <= 1e-6, reason)
    return result


def validate_config(config):
    _require(isinstance(config, dict) and config.get('schema_version') == 1 and
             config.get('status') == 'EXPERIMENT' and config.get('source_mode') == 'real', 'EXPERIMENT_CONFIG_REQUIRED')
    _require(type(config.get('continuous_mapping', False)) is bool, 'INVALID_CONTINUOUS_MAPPING_PROFILE')
    selected_cloud_source(config)
    resolve_cloud_filter(config)
    mode = config.get('mode')
    _require(mode in ('left', 'right', 'all'), 'EXPLICIT_MAPPING_MODE_REQUIRED')
    sides = ('left', 'right') if mode == 'all' else (mode,)
    from .compare_geometry import is_offline_mechanical_initial
    explicit_initial = is_offline_mechanical_initial(config)
    _require(config.get('mount_model') in ('fixed_hardware_setup_v1','fixed_hardware_setup_v2') or explicit_initial,
             'FIXED_HARDWARE_SETUP_REQUIRED')
    mapping_enabled = config.get('mapping_enabled', True)
    _require(type(mapping_enabled) is bool, 'INVALID_MAPPING_ENABLED')
    offline = config.get('offline_experiment', False)
    _require(type(offline) is bool, 'INVALID_OFFLINE_EXPERIMENT')
    if mapping_enabled and ('geometry_capabilities' in config or config['mount_model']=='fixed_hardware_setup_v2'):
        from .calibration_geometry import require_geometry_capability, motion_capability
        capability = ('legacy_replay' if offline and config['mount_model']=='fixed_hardware_setup_v1'
                      else motion_capability(mode))
        require_geometry_capability(config, capability)
    for side in sides:
        SourceGate(config['session_id'], side, config['sensor_ids'][side])
        if mapping_enabled:
            mount = config['mounts'][side]
            vector(mount['t_axle_lidar_m'], 3, 'MEASURED_MOUNT_TRANSLATION_REQUIRED')
            rotation(mount['R_axle_lidar'], 'FIXED_LIDAR_ROTATION_REQUIRED')
    if mapping_enabled:
        rotation(config['imu_mount']['R_axle_imu'], 'FIXED_IMU_ROTATION_REQUIRED')
    for field in ('imu_sensor_id', 'wheel_device_id'):
        _require(isinstance(config.get(field), str) and bool(config[field].strip()), field.upper() + '_REQUIRED')
    rate_limit = 10 if offline else 5
    _require(type(config.get('input_rate_hz')) in (int, float) and math.isfinite(config['input_rate_hz'])
             and (0 < config['input_rate_hz'] <= rate_limit or (offline and config['input_rate_hz'] == 0)),
             'INPUT_RATE_MUST_BE_0_OR_AT_MOST_10HZ_OFFLINE' if offline else 'INPUT_RATE_MUST_BE_AT_MOST_5HZ')
    _require(type(config.get('max_pair_delta_ns')) is int and 0 <= config['max_pair_delta_ns'] <= PAIR_DELTA_NS,
             'PAIR_DELTA_MUST_NOT_EXCEED_50MS')
    _require(isinstance(config.get('prior_template'), dict), 'PRIOR_TEMPLATE_REQUIRED')
    estimator=config['prior_template'].get('estimator',config.get('wheel_imu_estimator','five_state'))
    _require(estimator in ('five_state','robot_localization'), 'INVALID_WHEEL_IMU_ESTIMATOR')
    if estimator=='robot_localization':
        _require(config.get('motion_model',config['prior_template'].get('motion_model'))=='planar_ekf',
                 'ROBOT_LOCALIZATION_REQUIRES_PLANAR_EKF')
    _require(config.get('archive_publication_mode', 'written_before_publish') in
             ('written_before_publish', 'queued_realtime'), 'INVALID_ARCHIVE_PUBLICATION_MODE')
    _require(config.get('archive_checkpoint_policy', 'periodic') in CHECKPOINT_POLICIES,
             'INVALID_ARCHIVE_CHECKPOINT_POLICY')
    _require(config.get('formal_acceptance') is False and config.get('navigation_validated') is False and
             config.get('common_measurement_time_validated') is False, 'EXPERIMENT_LIMITATIONS_REQUIRED')
    return sides


def subscribe_source_frames(node, config, message_type, receive, guard, qos):
    """Bind actual acquisition callbacks to the same selection as the snapshot."""
    selected = selected_cloud_source(config)
    sides = ('left', 'right') if config['mode'] == 'all' else (config['mode'],)
    return [node.create_subscription(message_type, source_frame_topic(side, selected),
                guard(side, lambda message, expected=side: receive(message, expected)), qos) for side in sides]


def mount_transforms(config, mean_acceleration):
    """Fixed hardware extrinsics; startup gravity is evidence, never a mount.

    T_A_B maps B into A. The motion prior separately estimates initial leveling
    from live gravity using the fixed R_base_imu derived here. Parking on a
    slope must not change left/right geometry or the axle-to-lidar lever arm.
    """
    if mean_acceleration is None:
        _require(config.get('continuous_mapping') is True, 'INVALID_GRAVITY_VECTOR')
        acceleration, orientation = None, None
    else:
        acceleration = vector(list(mean_acceleration), 3, 'INVALID_GRAVITY_VECTOR')
        ax, ay, az = acceleration
        _require(8 <= np.linalg.norm(acceleration) <= 12, 'BOOTSTRAP_ACCELERATION_NORM')
        orientation = [math.atan2(ay, az), math.atan2(-ax, math.hypot(ay, az)), 0.]
    r_axle_imu = rotation(config['imu_mount']['R_axle_imu'], 'INVALID_FIXED_IMU_ROTATION')
    base_side = 'left' if config['mode'] == 'all' else config['mode']
    selected = ('left', 'right') if config['mode'] == 'all' else (base_side,)
    axle_side = {}
    for side in selected:
        transform = np.eye(4)
        transform[:3, :3] = rotation(config['mounts'][side]['R_axle_lidar'], 'INVALID_FIXED_LIDAR_ROTATION')
        transform[:3, 3] = vector(config['mounts'][side]['t_axle_lidar_m'], 3, 'INVALID_MOUNT_TRANSLATION')
        axle_side[side] = transform
    t_base_axle = np.linalg.inv(axle_side[base_side])
    t_base_side = {side: (t_base_axle @ value) for side, value in axle_side.items()}
    # Base coordinates are the selected lidar's actual coordinates. Avoid tiny
    # numerical deviations from identity changing a single-source raw cloud.
    t_base_side[base_side] = np.eye(4)
    return {'base_frame': 'lidar_' + base_side, 'T_base_axle': t_base_axle.tolist(),
            'R_base_imu': (t_base_axle[:3, :3] @ r_axle_imu).tolist(),
            'R_axle_imu': r_axle_imu.tolist(), 'T_base_side': {k: v.tolist() for k, v in t_base_side.items()},
            'imu_roll_pitch_yaw_rad': orientation,
            'mean_acceleration_imu_m_s2': acceleration.tolist() if acceleration is not None else None,
            'mounting_basis': config.get('mount_model', 'fixed_hardware_setup_v1'), 'gravity_used_to_define_mounts': False,
            'imu_yaw_from_gravity_observed': False,
            'hardware_setup_provenance': copy.deepcopy(config.get('hardware_setup_provenance'))}


class GravityBootstrap:
    """Require a fresh continuous two-second low-gyro / zero-feedback window."""
    def __init__(self, config, *, started_ns):
        self.config, self.started_ns = config, started_ns
        self.continuous_mapping = config.get('continuous_mapping', False)
        self.last_imu = self.last_wheel = None
        self.imu_epoch = self.wheel_epoch = None
        self.imu_seq = self.wheel_seq = None
        self.imu_stamp = self.wheel_stamp = None
        self.window = []
        self.wheels = {}
        self.ready = None
        self.resets = 0
        self.imu_sequence_gaps = self.wheel_sequence_gaps = 0
        self.last_imu_observation = self.last_wheel_observation = None
        if self.continuous_mapping:
            self.ready = {
                'schema_version': 1, 'status': 'EXPERIMENT_BOOTSTRAP', 'session_id': config['session_id'],
                'reference_mode': 'FIXED_MOUNT_REFERENCE_NO_STATIC_CALIBRATION', 'continuous_mapping': True,
                'stationary_window_required': False, 'gravity_observation_available': False,
                'time_source': 'arrival_only', 'common_measurement_time_validated': False,
                'formal_acceptance': False, 'navigation_validated': False,
                'stationary_ground_truth_validated': False,
                'yaw_reference': 'fixed_hardware_setup_mount_yaw; no startup heading observation',
                'window_host_span_ns': None, 'window_resets': 0,
                'imu_sequence_gaps_observed': 0, 'wheel_sequence_gaps_observed': 0,
                'sampling_policy': 'No startup samples or static calibration required. '
                                   'Fixed mounting geometry supplies the initial coordinate reference.',
                'imu_samples_used': [], 'wheel_zero_records_used': [],
                'wheel_stream_epoch': None, 'imu_stream_epoch': None,
                'wheel_identity_binding': 'no wheel samples used in fixed mount initialization',
                'thresholds': None, 'mount_candidates': copy.deepcopy(config['mounts']),
                **mount_transforms(config, None)}

    def reset(self):
        if self.window:
            self.resets += 1
        self.window = []
        self.wheels = {}

    def wheel(self, value, now_ns):
        if isinstance(value, str):
            _require(len(value.encode()) <= 16384, 'WHEEL_RECORD_OVERSIZED')
            value = json.loads(value)
        _require(isinstance(value, dict) and value.get('schema') == 'wc_wheel_feedback_v1' and
                 value.get('device_id') == self.config['wheel_device_id'], 'WHEEL_IDENTITY_MISMATCH')
        count=value.get('control_transmissions')
        manual=(self.config.get('manual_controls',{}).get('arm_allowed') is True and
                value.get('control_context')=='user_manual_mapping' and
                value.get('session_id')==self.config['session_id'])
        _require(value.get('status') == 'RESPONSE_VALID' and value.get('time_source') == 'arrival_only' and
                 value.get('time_valid') is False and value.get('formal_odometry_eligible') is False and
                 type(count) is int and count>=0 and (count==0 or manual),
                 'WHEEL_NOT_READ_ONLY_ARRIVAL_FEEDBACK')
        _require(value.get('request_hex') == QUERY_HEX, 'WHEEL_REQUEST_MISMATCH')
        response = bytes.fromhex(value['response_hex'])
        _, _, words = parse_exchange(bytes.fromhex(QUERY_HEX), response)
        _require(value.get('response_sha256') == hashlib.sha256(response).hexdigest() and
                 value.get('register_words_u16') == list(words), 'WHEEL_RESPONSE_METADATA_MISMATCH')
        host = _uint(value['receive_monotonic_ns'], 'INVALID_WHEEL_HOST_TIME')
        stamp = _uint(value['stamp_ns'], 'INVALID_WHEEL_STAMP')
        seq = _uint(value['sequence'], 'INVALID_WHEEL_SEQUENCE')
        epoch = value.get('stream_epoch')
        _require(isinstance(epoch, str) and 0 < len(epoch.strip()) <= 256, 'INVALID_WHEEL_EPOCH')
        self.last_wheel_observation = {'sequence': seq, 'source_stamp_ns': stamp, 'host_monotonic_ns': host,
                                      'callback_monotonic_ns': now_ns, 'callback_source_age_ns': now_ns-host}
        _require(0 <= now_ns-host and (self.continuous_mapping or now_ns-host <= BOOT_GAP_NS),
                 'WHEEL_STALE_OR_FUTURE')
        if self.last_wheel is not None:
            _require(epoch == self.wheel_epoch and seq > self.wheel_seq and host > self.last_wheel['receive_monotonic_ns']
                     and stamp > self.wheel_stamp, 'WHEEL_EPOCH_OR_TIME_CHANGED')
            self.wheel_sequence_gaps += seq-self.wheel_seq-1
        self.last_wheel, self.wheel_epoch, self.wheel_seq, self.wheel_stamp = copy.deepcopy(value), epoch, seq, stamp
        if words != (0, 0) and self.ready is None:
            self.reset()

    def imu(self, message, now_ns):
        _require(message.session_id == self.config['session_id'] and message.sensor_id == self.config['imu_sensor_id'], 'IMU_IDENTITY_MISMATCH')
        _require(message.header.frame_id == 'imu_h30_native' and message.imu.header.frame_id == 'imu_h30_native' and
                 message.coordinate_convention == 'H30_NATIVE_UNVALIDATED', 'IMU_AXES_OR_FRAME_MISMATCH')
        _require(message.time_source == 'arrival_only' and message.common_time_valid is False and
                 message.uncertainty_valid is False and message.common_time_ns == 0, 'IMU_NOT_EXPLICIT_ARRIVAL_ONLY')
        stamp = _stamp(message.header.stamp)
        _require(stamp > 0 and _stamp(message.imu.header.stamp) == stamp and _stamp(message.host_receive_time) == stamp,
                 'IMU_HEADER_TIME_MISMATCH')
        host, seq = _uint(message.host_monotonic_ns, 'INVALID_IMU_HOST_TIME'), _uint(message.frame_sequence, 'INVALID_IMU_SEQUENCE')
        epoch = message.stream_epoch
        _require(isinstance(epoch, str) and 0 < len(epoch.strip()) <= 256, 'INVALID_IMU_EPOCH')
        self.last_imu_observation = {'sequence': seq, 'source_stamp_ns': stamp, 'host_monotonic_ns': host,
                                    'callback_monotonic_ns': now_ns, 'callback_source_age_ns': now_ns-host}
        _require(0 <= now_ns-host and (self.continuous_mapping or now_ns-host <= BOOT_GAP_NS),
                 'IMU_STALE_OR_FUTURE')
        _require(message.angular_velocity_valid is True and message.linear_acceleration_valid is True, 'IMU_REQUIRED_FIELDS_INVALID')
        gyro = vector([message.imu.angular_velocity.x, message.imu.angular_velocity.y, message.imu.angular_velocity.z], 3, 'INVALID_IMU_GYRO')
        acceleration = vector([message.imu.linear_acceleration.x, message.imu.linear_acceleration.y, message.imu.linear_acceleration.z], 3, 'INVALID_IMU_ACCELERATION')
        if self.last_imu is not None:
            # H30Node.acquire timestamps one serial read before decoding it.
            # Distinct packets in that read share both host clocks; sequence,
            # not an invented per-packet time offset, orders those observations.
            _require(epoch == self.imu_epoch and seq > self.imu_seq and host >= self.last_imu['host_monotonic_ns']
                     and stamp >= self.imu_stamp, 'IMU_EPOCH_OR_TIME_CHANGED')
            self.imu_sequence_gaps += seq-self.imu_seq-1
            if host-self.last_imu['host_monotonic_ns'] > BOOT_GAP_NS and self.ready is None:
                self.reset()
        raw = bytes(message.raw_packet)
        _require(0 < len(raw) <= 4096, 'IMU_RAW_PACKET_REQUIRED')
        sample = {'sensor_id': message.sensor_id, 'session_id': message.session_id, 'stream_epoch': epoch,
                  'sequence': seq, 'source_stamp_ns': stamp, 'host_monotonic_ns': host,
                  'angular_velocity_rad_s': gyro.tolist(), 'linear_acceleration_m_s2': acceleration.tolist(),
                  'raw_packet_hex': raw.hex(), 'raw_packet_sha256': hashlib.sha256(raw).hexdigest()}
        self.last_imu, self.imu_epoch, self.imu_seq, self.imu_stamp = sample, epoch, seq, stamp
        if self.ready is not None:
            return None
        wheel = self.last_wheel
        stationary = (wheel is not None and wheel['register_words_u16'] == [0, 0] and
                      0 <= now_ns-wheel['receive_monotonic_ns'] <= BOOT_GAP_NS and
                      np.linalg.norm(gyro) <= MAX_GYRO_RAD_S and 8 <= np.linalg.norm(acceleration) <= 12)
        if not stationary:
            self.reset()
            return None
        self.window.append(sample)
        self.wheels[wheel['sequence']] = copy.deepcopy(wheel)
        _require(len(self.window) <= MAX_WINDOW_SAMPLES, 'BOOTSTRAP_SAMPLE_BOUND_EXCEEDED')
        if host-self.window[0]['host_monotonic_ns'] < BOOT_WINDOW_NS:
            return None
        _require(now_ns-self.started_ns <= STARTUP_NS, 'BOOTSTRAP_STARTUP_TIMEOUT')
        mean = np.mean([sample['linear_acceleration_m_s2'] for sample in self.window], axis=0)
        transforms = mount_transforms(self.config, mean.tolist())
        self.ready = {'schema_version': 1, 'status': 'EXPERIMENT_BOOTSTRAP', 'session_id': self.config['session_id'],
                      'time_source': 'arrival_only', 'common_measurement_time_validated': False,
                      'formal_acceptance': False, 'navigation_validated': False,
                      'stationary_ground_truth_validated': False,
                      'yaw_reference': 'fixed_hardware_setup_mount_yaw; gravity_does_not_observe_heading',
                      'window_host_span_ns': host-self.window[0]['host_monotonic_ns'], 'window_resets': self.resets,
                      'imu_sequence_gaps_observed': self.imu_sequence_gaps,
                      'wheel_sequence_gaps_observed': self.wheel_sequence_gaps,
                      'sampling_policy': 'Latest available IMU/wheel callbacks; source timestamps and observed continuity retained. '
                                         'Raw full-rate observations remain in the separate bag recording.',
                      'imu_samples_used': copy.deepcopy(self.window), 'wheel_zero_records_used': list(self.wheels.values()),
                      'wheel_stream_epoch': self.wheel_epoch, 'imu_stream_epoch': self.imu_epoch,
                      'wheel_identity_binding': 'device_id_and_fresh_epoch_under_backend_exclusive_owner; no wheel session_id field',
                      'thresholds': {'gyro_norm_max_rad_s': MAX_GYRO_RAD_S, 'acceleration_norm_m_s2': [8, 12],
                                     'continuous_window_ns': BOOT_WINDOW_NS, 'freshness_and_gap_ns': BOOT_GAP_NS},
                      'mount_candidates': copy.deepcopy(self.config['mounts']), **transforms}
        return self.ready

    def check_stale(self, now_ns):
        if self.continuous_mapping:
            return
        if self.ready is None:
            _require(now_ns-self.started_ns <= STARTUP_NS, 'BOOTSTRAP_STARTUP_TIMEOUT')
        for name, sample, key in (('IMU', self.last_imu, 'host_monotonic_ns'),
                                  ('WHEEL', self.last_wheel, 'receive_monotonic_ns')):
            if sample is None:
                _require(now_ns-self.started_ns <= STARTUP_NS, name + '_STARTUP_TIMEOUT')
            else:
                _require(now_ns-sample[key] <= STALE_NS, name + '_SOURCE_STALE')


def intensity_values(cloud):
    layout = validate_pointcloud2(cloud)
    field = layout['fields']['intensity']
    dtype = np.dtype(('>' if layout['is_bigendian'] else '<') + field['dtype'])
    return np.ndarray((layout['height'], layout['width']), dtype=dtype, buffer=bytes(cloud.data),
                      offset=field['offset'], strides=(layout['row_step'], layout['point_step'])).reshape(-1).copy()


def build_cloud(members, transforms, *, cloud_factory=SimpleNamespace, field_factory=SimpleNamespace):
    """Concatenate in left/right source row order; original inputs stay untouched."""
    if len(members) == 1:
        message = members[0][0]
        _require(message.cloud.header.frame_id == transforms['base_frame'], 'SINGLE_SOURCE_BASE_MISMATCH')
        return copy.deepcopy(message.cloud)
    _require([item[0].side for item in members] == ['left', 'right'], 'DUAL_SOURCE_ORDER_REQUIRED')
    stamps = [_stamp(message.header.stamp) for message, _metadata in members]
    reference = max(stamps)
    total = sum(message.raw_count for message, _metadata in members)
    dtype = np.dtype({'names': ['x', 'y', 'z', 'intensity', 'time_offset_s', 'source_code'],
                      'formats': ['<f4', '<f4', '<f4', '<f4', '<f8', 'u1'],
                      'offsets': [0, 4, 8, 12, 16, 24], 'itemsize': 32})
    data = np.zeros(total, dtype=dtype)
    offset = 0
    for message, _metadata in members:
        size = message.raw_count
        transform = np.asarray(transforms['T_base_side'][message.side])
        xyz = decode_pointcloud2(message.cloud)
        originally_finite = np.isfinite(xyz).all(axis=1)
        # Preserve every raw row, including invalid rows. Raw bytes live in CDR;
        # the derived PCL-compatible cloud is explicitly FLOAT32 geometry.
        xyz = xyz @ transform[:3, :3].T + transform[:3, 3]
        for column, axis in enumerate(('x', 'y', 'z')):
            data[axis][offset:offset+size] = xyz[:, column]
        converted_finite = np.column_stack([data[axis][offset:offset+size] for axis in ('x', 'y', 'z')])
        _require(np.isfinite(converted_finite[originally_finite]).all(), 'DERIVED_CLOUD_FLOAT32_OVERFLOW')
        data['intensity'][offset:offset+size] = intensity_values(message.cloud)
        data['time_offset_s'][offset:offset+size] = (_stamp(message.header.stamp)-reference)/1e9
        data['source_code'][offset:offset+size] = 1 if message.side == 'left' else 2
        offset += size
    cloud = cloud_factory()
    cloud.header = copy.deepcopy(members[stamps.index(reference)][0].cloud.header)
    cloud.header.frame_id = transforms['base_frame']
    cloud.width, cloud.height, cloud.point_step, cloud.row_step = total, 1, 32, total*32
    cloud.is_bigendian = False
    cloud.is_dense = all(message.cloud.is_dense for message, _ in members)
    cloud.fields = [field_factory(name=name, offset=position, datatype=kind, count=1)
                    for name, position, kind in [('x', 0, 7), ('y', 4, 7), ('z', 8, 7),
                                                ('intensity', 12, 7), ('time_offset_s', 16, 8), ('source_code', 24, 2)]]
    cloud.data = data.tobytes()
    return cloud


def write_exclusive_json(project_root, path, value, *, sync=True):
    """Publish a complete new file atomically; never replace an existing config."""
    destination = project_path(project_root, path)
    temporary = project_path(project_root, destination.with_name(destination.name + '.partial'))
    with temporary.open('xb') as stream:
        stream.write(_json_bytes(value)); stream.flush()
        if sync:
            os.fsync(stream.fileno())
    # A hard link is atomic and fails if destination already exists. Both paths
    # are in the same owned directory/filesystem; readers never see partial JSON.
    try:
        os.link(temporary, destination)
    except OSError as error:
        import errno
        if error.errno not in (errno.EPERM, errno.EOPNOTSUPP, errno.ENOSYS):
            raise
        # exFAT has no hard links. Linux renameat2 retains both atomic
        # publication and no-overwrite semantics; unsupported commits fail.
        from wc_maps.store import _rename_no_replace
        _rename_no_replace(temporary, destination)
    else:
        temporary.unlink()
    if sync:
        _sync_dir(destination.parent)


class FlushedFrameArchive(FrameArchive):
    """One writer flushes original CDR and indexes without per-frame fsync.

    The legacy single-lidar archive keeps its stricter per-frame contract. This
    implementation has exactly one writer; CheckpointJournal owns disk sync.
    """
    def append(self, cdr, metadata):
        _require(isinstance(cdr, bytes) and 0 < len(cdr) <= MAX_CDR_BYTES, 'INVALID_CDR_ENVELOPE')
        _require(self.continuous_mapping or self.count < MAX_FRAMES, 'FRAME_ARCHIVE_LIMIT_REACHED')
        _require(shutil.disk_usage(self.root).free >= MIN_FREE_BYTES+len(cdr)+65536, 'DISK_FREE_MARGIN_EXHAUSTED')
        destination = project_path(self.project_root, self.frames/('%08d.cdr' % (self.count+1)), storage=self.storage)
        with destination.open('xb') as stream:
            stream.write(cdr); stream.flush()
        row = dict(metadata, archive_index=self.count+1, file=destination.relative_to(self.root).as_posix(),
                   serialization_format='cdr', ros_type='wc_interfaces/msg/SourceFrame',
                   cdr_bytes=len(cdr), cdr_sha256=hashlib.sha256(cdr).hexdigest(), source_mode='real',
                   formal_acceptance=False, coordinate_transform_applied=False,
                   persistence='WRITTEN_AND_FLUSHED; see ../checkpoint.json for fsynced prefix')
        self.index.write(_json_bytes(row)); self.index.flush()
        self.count += 1
        return row


def atomic_status(project_root, output, payload, *, sync=False):
    destination = project_path(project_root, output/'status.json')
    temporary = project_path(project_root, output/'status.json.partial')
    with temporary.open('wb') as stream:
        stream.write(_json_bytes(payload)); stream.flush()
        if sync:
            os.fsync(stream.fileno())
    os.replace(temporary, destination)
    if sync:
        _sync_dir(output)


class CheckpointJournal:
    """One coordinator, at most four daemon data-fsync workers, captured prefixes.

    Locks protect small in-memory snapshots only, never filesystem calls. The
    optional one-second interval is scheduling intent, not a power-loss window promise:
    an fsync may itself take seconds. Journal suffixes beyond the manifest may
    have reached disk but are deliberately not certified by this checkpoint.
    on_close requests no runtime fsync; it waits until acquisition and the
    archive writer stop before applying the same full durability barriers.
    """
    def __init__(self, project_root, output, sides, *, policy='periodic'):
        _require(policy in CHECKPOINT_POLICIES, 'INVALID_ARCHIVE_CHECKPOINT_POLICY')
        self.policy = policy
        self.project_root, self.output, self.sides = project_root, output, sides
        from .storage_policy import StoragePolicy
        self.storage = StoragePolicy(project_root)
        self.condition = threading.Condition()
        self.latest = {'archived_outputs': 0, 'filtered_empty_outputs': 0, 'pairs_index_bytes': 0,
                       'sources': {side: {'frames': 0, 'index_bytes': 0} for side in sides}, 'artifacts': []}
        self.committed = None
        self.error = None
        self.closing = False
        self.generation = 0
        self.last_duration_ns = self.max_duration_ns = 0
        self.skipped_unchanged = 0
        self.sync_operation_timings = {}
        self.cancelled = threading.Event()
        self.data_batch_lock = threading.Lock()
        self.data_threads = []
        self.data_active = self.data_max_active = self.data_started = self.data_completed = 0
        self.thread = threading.Thread(target=self._run, name='mapping-checkpoint', daemon=True)
        self.thread.start()

    def update(self, snapshot):
        with self.condition:
            self.latest = copy.deepcopy(snapshot)
            self.condition.notify_all()

    def snapshot(self):
        with self.condition:
            return {'written_prefix': copy.deepcopy(self.latest), 'checkpoint': copy.deepcopy(self.committed),
                    'checkpoint_policy': self.policy,
                    'error': self.error, 'last_duration_ns': self.last_duration_ns,
                    'max_duration_ns': self.max_duration_ns,
                    'skipped_unchanged': self.skipped_unchanged,
                    'sync_operation_timings': copy.deepcopy(self.sync_operation_timings),
                    'data_sync_workers': {'limit': CHECKPOINT_DATA_WORKERS,
                        'active': self.data_active, 'max_active': self.data_max_active,
                        'jobs_started': self.data_started, 'jobs_completed': self.data_completed,
                        'threads_alive': sum(thread.is_alive() for thread in self.data_threads),
                        'daemon_only': True, 'dispatch': 'locked lazy iterator; no queued futures',
                        'cancelled': self.cancelled.is_set()}}

    def _fsync_path(self, path):
        # Opening separately avoids descriptor lifetime and buffered-writer
        # races. Appends beyond the captured byte prefix are not certified.
        # Windows FlushFileBuffers requires a writable handle. These are owned
        # session artifacts; r+b never truncates or writes their contents.
        with project_path(self.project_root, path, storage=self.storage).open('r+b' if os.name == 'nt' else 'rb') as stream:
            os.fsync(stream.fileno())

    def _sync_operation(self, category, operation):
        """Bounded phase totals; no filesystem call holds the state lock."""
        _require(not self.cancelled.is_set(), 'STORAGE_CHECKPOINT_CLOSE_TIMEOUT')
        started, completed = time.monotonic_ns(), False
        try:
            result = operation()
            completed = True
            return result
        finally:
            duration = max(0, time.monotonic_ns()-started)
            with self.condition:
                row = self.sync_operation_timings.setdefault(category, {
                    'count': 0, 'failures': 0, 'last_duration_ns': 0,
                    'max_duration_ns': 0, 'total_duration_ns': 0})
                row['count'] += 1
                row['failures'] += int(not completed)
                row['last_duration_ns'] = duration
                row['max_duration_ns'] = max(row['max_duration_ns'], duration)
                row['total_duration_ns'] += duration

    def _sync_data_files(self, jobs):
        """Pull at most four file jobs; join every started worker before return.

        A failed job stops further dispatch. Already-running fsync calls cannot
        be safely cancelled, so only this daemon coordinator waits for them.
        The owner can time out close without a non-daemon pool hanging exit.
        """
        with self.data_batch_lock:
            iterator, dispatch, failures = iter(jobs), threading.Lock(), []

            def fail(error):
                if not failures:
                    failures.append(error)

            def work():
                while True:
                    with dispatch:
                        if failures or self.cancelled.is_set():
                            return
                        try:
                            category, path = next(iterator)
                        except StopIteration:
                            return
                        except BaseException as error:
                            fail(error); return
                        with self.condition:
                            self.data_active += 1; self.data_started += 1
                            self.data_max_active = max(self.data_max_active, self.data_active)
                    completed = False
                    try:
                        self._sync_operation(category, lambda: self._fsync_path(path))
                        completed = True
                    except BaseException as error:
                        with dispatch:
                            fail(error)
                    finally:
                        with self.condition:
                            self.data_active -= 1
                            self.data_completed += int(completed)

            threads = []
            try:
                for number in range(CHECKPOINT_DATA_WORKERS):
                    thread = threading.Thread(target=work, name='mapping-checkpoint-data-'+str(number), daemon=True)
                    thread.start(); threads.append(thread)
            except BaseException as error:
                with dispatch:
                    fail(error)
            finally:
                with self.condition:
                    self.data_threads = threads
                for thread in threads:
                    thread.join()
            if failures:
                error = failures[0]
                raise error if isinstance(error, Exception) else RuntimeError(type(error).__name__+': '+str(error))
            _require(not self.cancelled.is_set(), 'STORAGE_CHECKPOINT_CLOSE_TIMEOUT')

    def _commit(self, prefix, final):
        previous = self.committed
        first = previous is None
        if not first:
            _require(prefix['archived_outputs'] >= previous['archived_outputs'] and
                     prefix['pairs_index_bytes'] >= previous['pairs_index_bytes'] and
                     set(previous['artifacts']) <= set(prefix['artifacts']) and
                     all(prefix['sources'][side][key] >= previous['sources'][side][key]
                         for side in self.sides for key in ('frames', 'index_bytes')),
                     'CHECKPOINT_PREFIX_REGRESSION')
            if not final and all(prefix[key] == previous[key] for key in
                                 ('archived_outputs', 'pairs_index_bytes', 'sources', 'artifacts')):
                with self.condition:
                    self.skipped_unchanged += 1
                return False
        previous = previous or {'sources': {side: {'frames': 0, 'index_bytes': 0} for side in self.sides},
                                 'artifacts': [], 'pairs_index_bytes': 0}
        new_artifacts = [Path(path) for path in prefix['artifacts'] if path not in previous['artifacts']]
        def data_jobs():
            # Start the independent indexes together, then immutable data. No
            # complete CDR path/future list is built, even for a large prefix.
            for side in self.sides:
                if first or prefix['sources'][side]['index_bytes'] != previous['sources'][side]['index_bytes']:
                    yield 'source_index', self.output/side/'frames.jsonl'
            if first or prefix['pairs_index_bytes'] != previous['pairs_index_bytes']:
                yield 'pairs_index', self.output/'pairs.jsonl'
            for artifact in new_artifacts:
                yield 'immutable_artifact', artifact
            for side in self.sides:
                for number in range(previous['sources'][side]['frames']+1, prefix['sources'][side]['frames']+1):
                    yield 'cdr_file', self.output/side/'frames'/('%08d.cdr' % number)
        self._sync_data_files(data_jobs())
        # All captured data must succeed before any namespace/manifest barrier.
        for side in self.sides:
            old, current = previous['sources'][side], prefix['sources'][side]
            if first or current['frames'] != old['frames']:
                self._sync_operation('frames_directory', lambda: _sync_dir(self.output/side/'frames'))
            if first:
                # Later appends change file contents, not these directory entries.
                self._sync_operation('source_directory', lambda: _sync_dir(self.output/side))
        # New names must become durable BEFORE a manifest can name them. A
        # post-rename directory sync alone could leave a crash-visible manifest
        # referencing a lost new artifact. Keep this barrier only for new names.
        namespaces = {project_path(self.project_root, path, storage=self.storage).parent for path in new_artifacts}
        if first:
            namespaces.update((self.output, self.output.parent))
        for directory in sorted(namespaces, key=lambda path: (-len(path.parts), str(path))):
            self._sync_operation('new_entry_directory', lambda directory=directory: _sync_dir(directory))
        manifest = dict(prefix, schema_version=1, kind='FSYNCED_MAPPING_ARCHIVE_PREFIX',
                        generation=self.generation+1, final=final,
                        checkpoint_record_monotonic_ns=time.monotonic_ns(),
                        checkpoint_policy=self.policy,
                        checkpoint_interval_s=CHECKPOINT_INTERVAL_S if self.policy == 'periodic' else None,
                        crash_loss_bound_s=None,
                        limitation='Only these file counts and journal byte prefixes are confirmed synced. '
                                   'Later flushed data is not guaranteed after a crash; fsync stalls can exceed one second.')
        path = project_path(self.project_root, self.output/'checkpoint.json', storage=self.storage)
        temporary = project_path(self.project_root, self.output/'checkpoint.json.partial', storage=self.storage)
        def write_manifest():
            with temporary.open('wb') as stream:
                stream.write(_json_bytes(manifest)); stream.flush(); os.fsync(stream.fileno())
        self._sync_operation('manifest_file', write_manifest)
        self._sync_operation('manifest_replace', lambda: os.replace(temporary, path))
        self._sync_operation('manifest_directory', lambda: _sync_dir(self.output))
        with self.condition:
            _require(not self.cancelled.is_set(), 'STORAGE_CHECKPOINT_CLOSE_TIMEOUT')
            self.committed = manifest
            self.generation += 1
        return True

    def _run(self):
        next_at = time.monotonic()+CHECKPOINT_INTERVAL_S
        try:
            while True:
                with self.condition:
                    while not self.closing and (self.policy == 'on_close' or time.monotonic() < next_at):
                        self.condition.wait(None if self.policy == 'on_close' else next_at-time.monotonic())
                    prefix, final = copy.deepcopy(self.latest), self.closing
                started = time.monotonic_ns()
                committed = self._commit(prefix, final)
                duration = time.monotonic_ns()-started
                if committed:
                    with self.condition:
                        self.last_duration_ns = duration
                        self.max_duration_ns = max(self.max_duration_ns, duration)
                if final:
                    return
                next_at = time.monotonic()+CHECKPOINT_INTERVAL_S
        except Exception as error:
            with self.condition:
                self.error = self.error or 'STORAGE_CHECKPOINT_FAILED: '+str(error)

    def close(self, timeout):
        with self.condition:
            self.closing = True
            self.condition.notify_all()
        self.thread.join(max(0, timeout))
        if self.thread.is_alive():
            self.cancelled.set()
            with self.condition:
                self.error = self.error or 'STORAGE_CHECKPOINT_CLOSE_TIMEOUT'
            raise InputFailure(self.error)
        _require(self.error is None, self.error)


class StatusWriter:
    """One replaceable preview, isolated from authoritative cloud archiving.

    A slow status.json replace must not stall authoritative cloud archiving.
    The worker owns all preview writes; close joins it before final status I/O.
    """
    def __init__(self, owner):
        self.owner=owner
        self.condition=threading.Condition()
        self.latest_status=None
        self.closing=False
        self.error=None
        self.thread=threading.Thread(target=self._run,name='mapping-input-status',daemon=True)
        self.thread.start()

    def status(self,payload):
        with self.condition:
            _require(not self.closing and self.error is None,self.error or 'STATUS_WRITER_CLOSED')
            self.latest_status=payload
            self.condition.notify_all()

    def _run(self):
        try:
            while True:
                with self.condition:
                    while self.latest_status is None and not self.closing:
                        self.condition.wait()
                    if self.latest_status is None:return
                    payload,self.latest_status=self.latest_status,None
                self.owner.timed('status_write',lambda:atomic_status(self.owner.project_root,self.owner.output,payload))
        except Exception as error:
            self.error='STATUS_WRITE_FAILED: '+str(error)

    def close(self,timeout):
        with self.condition:
            self.closing=True
            self.condition.notify_all()
        self.thread.join(max(0,timeout))
        _require(not self.thread.is_alive(),'STATUS_CLOSE_TIMEOUT')
        _require(self.error is None,self.error)


class ArchiveWorker:
    """Bounded authoritative jobs; preview status uses a separate worker."""
    def __init__(self, owner):
        self.owner = owner
        self.jobs = queue.Queue(maxsize=owner.pending_output_limit+1)
        self.results = queue.Queue(maxsize=owner.pending_output_limit+2)
        self.condition = threading.Condition()
        self.closing = False
        self.error = None
        self.artifacts = []
        self.outputs = 0
        self.empty_outputs = 0
        self.thread = threading.Thread(target=self._run, name='mapping-archive', daemon=True)
        self.thread.start()

    def submit(self, job):
        _require(not self.closing and self.error is None, self.error or 'ARCHIVE_WORKER_CLOSED')
        try:
            self.jobs.put_nowait(job)
        except queue.Full as error:
            raise InputFailure('ARCHIVE_QUEUE_FULL') from error
        with self.condition:
            self.condition.notify_all()

    def status(self, payload):
        self.owner.status_writer.status(payload)

    def capture(self):
        self.owner.checkpoints.update({'archived_outputs': self.outputs, 'filtered_empty_outputs': self.empty_outputs,
            'pairs_index_bytes': self.owner.index.tell(),
            'sources': {side: {'frames': archive.count, 'index_bytes': archive.index.tell()}
                        for side, archive in self.owner.archives.items()}, 'artifacts': list(self.artifacts)})

    def _bootstrap(self, job):
        completed = job['completed']
        path = self.owner.output/'bootstrap.json'
        self.owner.timed('bootstrap_json_write', lambda: write_exclusive_json(self.owner.project_root, path, completed, sync=False))
        self.artifacts.append(str(path)); self.capture()
        prior = copy.deepcopy(self.owner.config['prior_template'])
        prior.setdefault('estimator',self.owner.config.get('wheel_imu_estimator','five_state'))
        prior.setdefault('offline_experiment',self.owner.config.get('offline_experiment',False))
        prior.update({key: completed[key] for key in ('base_frame', 'R_base_imu', 'T_base_axle')})
        prior.update(bootstrap_file=str(path), bootstrap_sha256=hashlib.sha256(_json_bytes(completed)).hexdigest())
        if self.owner.continuous_mapping:
            prior.update(continuous_mapping=True, initial_reference_mode=completed['reference_mode'])
        path = self.owner.output.parent/'prior_config.json'
        self.owner.timed('prior_config_write', lambda: write_exclusive_json(self.owner.project_root, path, prior, sync=False))
        self.artifacts.append(str(path)); self.capture()
        return {'kind': 'bootstrap', 'completed': completed, 'now_ns': job['now_ns']}

    def _archive(self, job):
        members, transforms = job['members'], job['transforms']
        reference = job['reference']
        realtime = job.get('archive_publication_mode') == 'queued_realtime'
        cloud = None if realtime else build_cloud(members, transforms, cloud_factory=job['cloud_factory'], field_factory=job['field_factory'])
        if realtime:
            filter_report = job['filter_report']
        else:
            cloud, filter_report = filter_cloud(cloud, members, transforms, self.owner.config)
        output_data = job['output_data'] if realtime else bytes(cloud.data)
        saved, row_start = [], 0
        for index, (source, metadata) in enumerate(members):
            cdr = job['source_cdrs'][index] if realtime else None
            archived = self.owner.timed(source.side+'_cdr_archive',
                lambda: self.owner.archives[source.side].append(cdr if realtime else job['serialize'](source), metadata))
            self.capture()
            transform = transforms['T_base_side'][source.side]
            saved.append({'side': source.side, 'raw_key': metadata['raw_key'],
                          'source_stamp_ns': metadata['header_stamp_ns'], 'host_monotonic_ns': metadata['host_monotonic_ns'],
                          'time_offset_ns': metadata['header_stamp_ns']-reference,
                          'time_offset_s': (metadata['header_stamp_ns']-reference)/1e9,
                          'T_base_side': transform, 'source_origin_in_base_m': [transform[i][3] for i in range(3)],
                          'source_code': 1 if source.side == 'left' else 2,
                          'output_row_start': row_start, 'output_row_count': source.raw_count,
                          'cdr_file': source.side+'/'+archived['file'], 'cdr_sha256': archived['cdr_sha256'],
                          'cloud_data_sha256': metadata['cloud_data_sha256']})
            row_start += source.raw_count
        row = {'schema_version': 1, 'session_id': self.owner.config['session_id'], 'mode': self.owner.config['mode'],
               'archive_output_index': self.outputs+1, 'reference_stamp_ns': reference,
               'base_frame': transforms['base_frame'], 'source_mode': 'real', 'time_source': 'arrival_only',
               'common_measurement_time_validated': False, 'formal_acceptance': False, 'sources': saved,
               'output_point_count': row_start, 'output_data_sha256': hashlib.sha256(output_data).hexdigest(),
               'cloud_filter': filter_report,
               'publication_eligible': filter_report['publishable'],
               'coordinate_transform_applied': len(members) == 2,
               'archive_publication_mode': 'queued_realtime' if realtime else 'written_before_publish',
               'publication_state': ('QUEUED_BEFORE_PUBLICATION_NOT_YET_DURABLE; '
                                     'this row confirms later writes; confirm publication count in status.json') if realtime else
                                    'WRITTEN_AND_FLUSHED_BEFORE_PUBLICATION; confirm published count in status.json',
               'durability': 'Only checkpoint.json certifies an fsynced prefix; this row alone does not.'}
        if not filter_report['publishable']:
            row['publication_state'] = 'FILTERED_SOURCE_EMPTY_ARCHIVED_NOT_PUBLISHED'
        def append_pair():
            self.owner.index.write(_json_bytes(row)); self.owner.index.flush()
        self.owner.timed('pair_index_write', append_pair)
        self.outputs += 1
        self.empty_outputs += not filter_report['publishable']
        self.capture()
        if realtime:
            # Completion acknowledges only archive progress. Never retain or
            # republish a cloud after disk I/O; its measurement may now be old.
            return {'kind': 'output', 'archive_publication_mode': 'queued_realtime',
                    'archive_output_index': self.outputs, 'members': members}
        return dict(job, kind='output', cloud=cloud, filter_report=filter_report, archive_output_index=self.outputs)

    def _run(self):
        try:
            while True:
                with self.condition:
                    while self.jobs.empty() and not self.closing:
                        self.condition.wait()
                try:
                    job = self.jobs.get_nowait()
                except queue.Empty:
                    if self.closing:
                        return
                    continue
                try:
                    result = self._bootstrap(job) if job['kind'] == 'bootstrap' else self._archive(job)
                    self.results.put_nowait(result)
                    del result
                finally:
                    self.jobs.task_done()
                del job  # Do not retain the last frozen payload while idle.
        except Exception as error:
            self.error = 'ARCHIVE_WRITE_FAILED: '+str(error)

    def close(self, timeout):
        with self.condition:
            self.closing = True
            self.condition.notify_all()
        self.thread.join(max(0, timeout))
        _require(not self.thread.is_alive(), 'ARCHIVE_CLOSE_TIMEOUT')


class MappingInput:
    def __init__(self, config, project_root, output_root, *, started_ns=None, close_timeout_s=CLOSE_TIMEOUT_S):
        from .mapping_shutdown import persistence_wait
        self.close_timeout_s = persistence_wait(close_timeout_s)
        self.config = copy.deepcopy(config)
        if self.config.get('mapping_enabled',True) is False:
            mode=self.config.get('mode')
            sides=('left','right') if mode=='all' else (mode,)
            complete=(not self.config.get('native_preview_only',False)
                      and all(side in self.config.get('mounts',{}) for side in sides)
                      and self.config.get('imu_mount',{}).get('R_axle_imu') is not None)
            _require(complete,'NATIVE_PREVIEW_REQUIRES_NATIVE_SOURCE_PATH; unknown geometry cannot initialize MappingInput')
        self.sides = validate_config(self.config)
        self.cloud_source = selected_cloud_source(self.config)
        self.cloud_filter = resolve_cloud_filter(self.config)
        self.continuous_mapping = self.config.get('continuous_mapping', False)
        self.archive_publication_mode = self.config.get('archive_publication_mode', 'written_before_publish')
        self.archive_checkpoint_policy = self.config.get('archive_checkpoint_policy', 'periodic')
        self.pending_output_limit = (MAX_REALTIME_PENDING_OUTPUTS if self.archive_publication_mode == 'queued_realtime'
                                     else MAX_PENDING_OUTPUTS)
        self.project_root = Path(project_root).absolute()
        self.output = project_path(self.project_root, output_root)
        self.started_ns = time.monotonic_ns() if started_ns is None else started_ns
        self.bootstrap = GravityBootstrap(self.config, started_ns=self.started_ns)
        self.gates = {side: SourceGate(config['session_id'], side, config['sensor_ids'][side], config['input_rate_hz'],
                                      started_ns=self.started_ns, continuous_mapping=self.continuous_mapping,
                                      offline_experiment=config.get('offline_experiment', False))
                      for side in self.sides}
        self.buffers = {side: deque() for side in self.sides}
        self.archives = {}
        self.index = None
        self.worker = self.checkpoints = self.status_writer = None
        self.closed = self.closing = False
        self.state, self.failure_reason = 'STARTING', None
        self.transforms = None
        self.bootstrap_completed_ns = None
        self.published_frames = self.archived_outputs = self.unpaired_discarded = self.bootstrap_discarded = 0
        self.filtered_empty_outputs = 0
        self.filter_last_report = None
        self.filter_totals = {side: {'frames': 0, 'input_rows': 0, 'retained_points': 0,
            'empty_frames': 0, 'support_applied_frames': 0, 'support_unavailable_frames': 0}
            for side in self.sides}
        self.last_published_host_ns = self.last_published_stamp_ns = None
        self.last_pair_delta_ns = None
        self.timings = {}
        self.timing_lock = threading.Lock()
        self.enqueued_outputs = self.pending_bytes = self.outputs_discarded_on_close = 0
        self.pending = deque()
        self.last_enqueued_host_ns = self.last_enqueued_stamp_ns = None
        self.output.mkdir(parents=True, exist_ok=False)
        try:
            for side in self.sides:
                self.archives[side] = FlushedFrameArchive(self.project_root, self.output/side,
                                                        continuous_mapping=self.continuous_mapping)
            self.index = (self.output/'pairs.jsonl').open('xb')
            self.checkpoints = CheckpointJournal(self.project_root, self.output, self.sides,
                                                 policy=self.archive_checkpoint_policy)
            self.status_writer = StatusWriter(self)
            self.worker = ArchiveWorker(self)
            if self.continuous_mapping:
                completed = copy.deepcopy(self.bootstrap.ready)
                completed['runtime_config_sha256'] = hashlib.sha256(_json_bytes(self.config)).hexdigest()
                self.worker.submit({'kind': 'bootstrap', 'completed': completed, 'now_ns': self.started_ns})
            self.write_status(self.started_ns)
        except BaseException as error:
            self.fail('INITIALIZATION_FAILED: '+str(error))
            self.close()
            raise

    def ensure_active(self):
        _require(self.failure_reason is None, self.failure_reason or 'MAPPING_INPUT_FAILED')
        _require(self.state != 'STOPPED' and not self.closing, 'MAPPING_INPUT_STOPPED')
        _require(self.worker is None or self.worker.error is None, self.worker.error if self.worker else None)
        checkpoint_error = self.checkpoints.snapshot()['error'] if self.checkpoints else None
        _require(checkpoint_error is None, checkpoint_error)

    def timed(self, label, operation, *, clock=time.monotonic_ns):
        """Record real callback/I/O cost, including failures; never redate inputs."""
        started = clock()
        try:
            return operation()
        finally:
            duration = max(0, clock()-started)
            with self.timing_lock:
                value = self.timings.setdefault(label, {'count': 0, 'last_duration_ns': 0, 'max_duration_ns': 0})
                value['count'] += 1
                value['last_duration_ns'] = duration
                value['max_duration_ns'] = max(value['max_duration_ns'], duration)

    def fail(self, reason):
        self.failure_reason = self.failure_reason or str(reason)
        self.state = 'FAILED'
        for gate in self.gates.values():
            gate.fail(self.failure_reason)

    def receive_wheel(self, value, now_ns):
        try:
            self.ensure_active()
            self.bootstrap.wheel(value, now_ns)
        except Exception as error:
            self.fail(str(error)); raise InputFailure(self.failure_reason) from error

    def receive_imu(self, message, now_ns):
        try:
            self.ensure_active()
            completed = self.bootstrap.imu(message, now_ns)
            if completed is None:
                return False
            completed['runtime_config_sha256'] = hashlib.sha256(_json_bytes(self.config)).hexdigest()
            self.worker.submit({'kind': 'bootstrap', 'completed': copy.deepcopy(completed), 'now_ns': now_ns})
            return True
        except Exception as error:
            self.fail(str(error)); raise InputFailure(self.failure_reason) from error

    def _members(self, message, metadata):
        if len(self.sides) == 1:
            return [(message, metadata)]
        side = message.side
        other = 'right' if side == 'left' else 'left'
        own, opposite = self.buffers[side], self.buffers[other]
        stamp = metadata['header_stamp_ns']
        delta = self.config['max_pair_delta_ns']
        # The other stream is monotonic. Samples older than this bound cannot
        # match this or any future sample on the receiving side.
        while opposite and opposite[0][1]['header_stamp_ns'] < stamp-delta:
            opposite.popleft(); self.unpaired_discarded += 1
        candidates = [(abs(row[1]['header_stamp_ns']-stamp), index) for index, row in enumerate(opposite)
                      if abs(row[1]['header_stamp_ns']-stamp) <= delta]
        if not candidates:
            own.append((message, metadata))
            while len(own) > 32:
                own.popleft(); self.unpaired_discarded += 1
            return None
        _, nearest = min(candidates)
        partner = opposite[nearest]
        del opposite[nearest]
        members = [(message, metadata), partner]
        members.sort(key=lambda row: row[0].side)
        return members

    def _observe_filter(self, report):
        """Bounded counters owned by the callback thread, never historical arrays."""
        if not report['enabled']:
            return
        self.filter_last_report = copy.deepcopy(report)
        if not report['publishable']:
            self.filtered_empty_outputs += 1
        for row in report['sources']:
            total = self.filter_totals[row['side']]
            total['frames'] += 1
            total['input_rows'] += row['input_rows']
            total['retained_points'] += row['retained_points']
            total['empty_frames'] += row['retained_points'] == 0
            total['support_applied_frames'] += row['support_status'] == 'APPLIED'
            total['support_unavailable_frames'] += row['support_status'] in ('LAYOUT_UNAVAILABLE', 'LAYOUT_METADATA_INVALID')

    def receive_source(self, message, now_ns, serialize, publish, *, clock=time.monotonic_ns,
                       cloud_factory=SimpleNamespace, field_factory=SimpleNamespace,
                       should_stop=lambda: False):
        """Accept one selected output under the configured publication contract."""
        try:
            if should_stop():
                return False
            self.ensure_active()
            _require(message.side in self.gates, 'UNREQUESTED_LIDAR_SIDE')
            # New explicit selections must match driver provenance. Missing-field
            # legacy configs retain their previous filtered gate contract.
            if 'cloud_source' in self.config:
                expected='before_host_filter' if self.cloud_source=='raw' else 'host_filtered'
                representations=[flag.split('=',1)[1] for flag in getattr(message,'diagnostic_flags',[])
                                 if flag.startswith('representation=')]
                _require(representations == [expected], 'CLOUD_SOURCE_REPRESENTATION_MISMATCH')
            gate = self.gates[message.side]
            metadata = gate.inspect(message, now_ns)
            if metadata is None:
                return False
            if self.transforms is None:
                self.bootstrap_discarded += 1
                return False
            members = self._members(message, metadata)
            if members is None:
                return False
            stamps = [row[1]['header_stamp_ns'] for row in members]
            hosts = [row[1]['host_monotonic_ns'] for row in members]
            reference, host = max(stamps), max(hosts)
            _require(max(stamps)-min(stamps) <= self.config['max_pair_delta_ns'], 'PAIR_TIME_DELTA_EXCEEDED')
            if gate.period_ns and self.last_enqueued_host_ns is not None and host-self.last_enqueued_host_ns < gate.period_ns:
                self.unpaired_discarded += len(members)
                return False
            _require(self.last_enqueued_stamp_ns is None or reference > self.last_enqueued_stamp_ns,
                     'NONMONOTONIC_OUTPUT_TIMESTAMP')
            _require(len(self.pending) < self.pending_output_limit, 'ARCHIVE_QUEUE_FULL')
            checkpoint = self.checkpoints.snapshot()['checkpoint']
            synced = checkpoint['archived_outputs'] if checkpoint else 0
            if self.archive_checkpoint_policy == 'periodic':
                _require(self.enqueued_outputs-synced < MAX_UNCHECKPOINTED_OUTPUTS, 'STORAGE_CHECKPOINT_BACKLOG')
            _require(self.continuous_mapping or self.enqueued_outputs < MAX_FRAMES, 'FRAME_ARCHIVE_LIMIT_REACHED')
            job = {'kind': 'output', 'members': members, 'transforms': self.transforms,
                   'reference': reference, 'host': host, 'pair_delta_ns': max(stamps)-min(stamps),
                   'serialize': serialize, 'publish': publish, 'cloud_factory': cloud_factory,
                   'field_factory': field_factory,
                   'enqueued_monotonic_ns': now_ns}
            cloud = None
            if self.archive_publication_mode == 'queued_realtime':
                cloud = self.timed('realtime_cloud_build', lambda: build_cloud(
                    members, self.transforms, cloud_factory=cloud_factory, field_factory=field_factory))
                cloud, filter_report = self.timed('realtime_cloud_filter',
                    lambda: filter_cloud(cloud, members, self.transforms, self.config))
                cdrs = []
                for source, _metadata in members:
                    cdr = self.timed(source.side+'_cdr_freeze', lambda: serialize(source))
                    _require(isinstance(cdr, bytes) and 0 < len(cdr) <= MAX_CDR_BYTES, 'INVALID_CDR_ENVELOPE')
                    cdrs.append(cdr)
                # The worker keeps no mutable original SourceFrame, ROS cloud,
                # publisher or serializer callback. Freeze all bytes and metadata
                # before queue acceptance, including the derived output's bytes.
                frozen = [(SimpleNamespace(side=source.side, raw_count=source.raw_count), copy.deepcopy(metadata))
                          for source, metadata in members]
                frozen_transforms = copy.deepcopy(self.transforms)
                output_data = bytes(cloud.data)
                metadata_bytes = len(_json_bytes({'members': [metadata for _, metadata in frozen],
                                                 'transforms': frozen_transforms, 'cloud_filter': filter_report}))
                reservation = (sum(map(len, cdrs))+len(output_data)+metadata_bytes+
                               QUEUED_OBJECT_ALLOWANCE_BYTES*(1+len(members)))
                job = {'kind': 'output', 'archive_publication_mode': 'queued_realtime',
                       'members': frozen, 'transforms': frozen_transforms, 'source_cdrs': tuple(cdrs),
                       'filter_report': filter_report,
                       'output_data': output_data, 'reference': reference, 'host': host,
                       'pair_delta_ns': max(stamps)-min(stamps), 'enqueued_monotonic_ns': now_ns}
            else:
                # Preserve the stricter mode's conservative reservation and
                # writer-side serialization/build order for existing callers.
                reservation = len(members)*MAX_CDR_BYTES+sum(row[0].raw_count*32 for row in members)
            _require(self.pending_bytes+reservation <= MAX_PENDING_BYTES, 'ARCHIVE_QUEUE_FULL')
            job['reservation'] = reservation
            if should_stop():
                return False  # Not accepted yet; no archive/publication claimed.
            self.worker.submit(job)
            self.pending.append((host, reservation))
            self.pending_bytes += reservation
            self.enqueued_outputs += 1
            self.last_enqueued_host_ns, self.last_enqueued_stamp_ns = host, reference
            if self.archive_publication_mode == 'queued_realtime':
                self._observe_filter(filter_report)
                # Accepted work remains archived even if freshness or publish
                # subsequently fails. This difference counts accepted outputs
                # that never successfully reached the publisher, also on close.
                self.outputs_discarded_on_close = self.enqueued_outputs-self.published_frames-self.filtered_empty_outputs
                if not filter_report['publishable']:
                    return True  # Original records still drain; an empty side is not a usable dual observation.
                if should_stop():
                    return True  # Accepted bytes must still drain during close.
                self.ensure_active()
                _require(self.status_writer.error is None, self.status_writer.error)
                publication_ns = clock()
                if should_stop():
                    return True
                self.bootstrap.check_stale(publication_ns)
                if not self.continuous_mapping:
                    for name, sample, key in (('IMU', self.bootstrap.last_imu, 'host_monotonic_ns'),
                                              ('WHEEL', self.bootstrap.last_wheel, 'receive_monotonic_ns')):
                        _require(sample is not None and 0 <= publication_ns-sample[key] <= BOOT_GAP_NS,
                                 name+'_STALE_OR_FUTURE')
                for selected_gate in self.gates.values():
                    selected_gate.check_stale(publication_ns)
                _require(all(0 <= publication_ns-source_host and
                             (self.continuous_mapping or publication_ns-source_host <= STALE_NS)
                             for source_host in hosts),
                         'REALTIME_OUTPUT_STALE_OR_FUTURE')
                self.timed('queue_accept_to_publish', lambda: None,
                           clock=iter((now_ns, publication_ns)).__next__)
                if should_stop():
                    return True
                publish(cloud)
                self.published_frames += 1
                self.outputs_discarded_on_close = self.enqueued_outputs-self.published_frames-self.filtered_empty_outputs
                self.last_published_host_ns, self.last_published_stamp_ns = host, reference
                self.last_pair_delta_ns = max(stamps)-min(stamps)
                for source, source_metadata in members:
                    selected_gate = self.gates[source.side]
                    selected_gate.published_frames += 1
                    selected_gate.last_forwarded_host_ns = source_metadata['host_monotonic_ns']
                    selected_gate.last_published_stamp_ns = source_metadata['header_stamp_ns']
                self.state = 'RUNNING'
            return True
        except Exception as error:
            self.fail(str(error)); raise InputFailure(self.failure_reason) from error

    def poll_io(self, now_ns, *, publish=True, clock=time.monotonic_ns, should_stop=lambda: False):
        """Apply worker results on the ROS thread; only this thread owns gates."""
        if self.worker is None:
            return
        try:
            while True:
                try:
                    result = self.worker.results.get_nowait()
                except queue.Empty:
                    break
                if result['kind'] == 'bootstrap':
                    completed = result['completed']
                    self.transforms = {key: completed[key] for key in ('base_frame', 'R_base_imu', 'T_base_axle', 'T_base_side')}
                    # This is readiness for lidar publication, after complete
                    # config visibility. Actual bootstrap sample times remain
                    # unchanged in bootstrap.json and bootstrap_progress.
                    self.bootstrap_completed_ns = now_ns
                    continue
                self.archived_outputs = result['archive_output_index']
                if self.pending:
                    _host, reservation = self.pending.popleft()
                    self.pending_bytes -= reservation
                for source, _metadata in result['members']:
                    self.gates[source.side].archived_frames += 1
                if result.get('archive_publication_mode') == 'queued_realtime':
                    continue  # An archive acknowledgment is never a live cloud.
                self._observe_filter(result['filter_report'])
                if not result['filter_report']['publishable']:
                    continue  # Successfully archived, deliberately not sent to the motion/map chain.
                if not publish or self.failure_reason is not None or should_stop():
                    self.outputs_discarded_on_close += 1
                    continue
                self.ensure_active()
                # A previous publish may have taken time while completions
                # accumulated. Recheck each result against the current host
                # clock without changing any original measurement timestamp.
                publication_ns = clock()
                if should_stop():
                    self.outputs_discarded_on_close += 1
                    continue
                self.bootstrap.check_stale(publication_ns)
                for gate in self.gates.values():
                    gate.check_stale(publication_ns)
                _require(0 <= publication_ns-result['host'] and
                         (self.continuous_mapping or publication_ns-result['host'] <= STALE_NS),
                         'ARCHIVED_OUTPUT_STALE_OR_FUTURE')
                self.timed('archive_to_publish_wait', lambda: None,
                           clock=iter((result['enqueued_monotonic_ns'], publication_ns)).__next__)
                if should_stop():
                    self.outputs_discarded_on_close += 1
                    continue
                result['publish'](result['cloud'])
                self.published_frames += 1
                self.last_published_host_ns, self.last_published_stamp_ns = result['host'], result['reference']
                self.last_pair_delta_ns = result['pair_delta_ns']
                for source, metadata in result['members']:
                    gate = self.gates[source.side]
                    gate.published_frames += 1
                    gate.last_forwarded_host_ns = metadata['host_monotonic_ns']
                    gate.last_published_stamp_ns = metadata['header_stamp_ns']
                self.state = 'RUNNING'
            # Include successfully written orphan CDRs if a later side/index
            # write failed. Worker snapshots are immutable, conservative prefixes.
            snapshot = self.checkpoints.snapshot()
            for side, row in snapshot['written_prefix']['sources'].items():
                self.gates[side].archived_frames = row['frames']
            self.archived_outputs = snapshot['written_prefix']['archived_outputs']
            _require(self.worker.error is None, self.worker.error)
            _require(self.status_writer.error is None, self.status_writer.error)
            _require(snapshot['error'] is None, snapshot['error'])
        except Exception as error:
            self.fail(str(error)); raise InputFailure(self.failure_reason) from error

    def check_stale(self, now_ns):
        try:
            self.ensure_active()
            self.bootstrap.check_stale(now_ns)
            if self.transforms is None and not self.continuous_mapping:
                _require(now_ns-self.started_ns <= STARTUP_NS, 'BOOTSTRAP_CONFIG_WRITE_TIMEOUT')
            for side, gate in self.gates.items():
                try:
                    gate.check_stale(now_ns)
                except InputFailure as error:
                    raise InputFailure(side.upper() + ': ' + str(error)) from error
            origin = self.last_published_host_ns if self.last_published_host_ns is not None else self.bootstrap_completed_ns
            if origin is not None and not self.continuous_mapping:
                _require(now_ns-origin <= STALE_NS, 'INPUT_PAIR_OR_PUBLICATION_STALE')
        except Exception as error:
            self.fail(str(error)); raise InputFailure(self.failure_reason) from error

    def status(self, now_ns):
        with self.timing_lock:
            timings = copy.deepcopy(self.timings)
        storage = self.checkpoints.snapshot() if self.checkpoints else {'written_prefix': None, 'checkpoint': None, 'error': None}
        checkpoint = storage['checkpoint']
        synced = checkpoint['archived_outputs'] if checkpoint else 0
        synced_eligible = synced-(checkpoint.get('filtered_empty_outputs', 0) if checkpoint else 0)
        written = storage['written_prefix'] or {}
        archived_eligible = written.get('archived_outputs', 0)-written.get('filtered_empty_outputs', 0)
        realtime = self.archive_publication_mode == 'queued_realtime'
        periodic = self.archive_checkpoint_policy == 'periodic'
        storage.update(checkpoint_interval_s=CHECKPOINT_INTERVAL_S if periodic else None, crash_loss_bound_s=None,
                       accepted_outputs_not_checkpointed=max(0, self.enqueued_outputs-synced),
                       checkpoint_age_ns=max(0, now_ns-checkpoint['checkpoint_record_monotonic_ns']) if checkpoint else None,
                       publication_contract=('QUEUED_BEFORE_PUBLICATION_NOT_YET_DURABLE: immutable source CDRs, '
                                             'output bytes and metadata accepted in a bounded RAM queue before live publish. '
                                             'Only checkpoint.json certifies a disk-synced prefix.') if realtime else
                                            'All member CDRs and source/pair indexes written and flushed before publish; '
                                            'only checkpoint.json certifies a disk-synced prefix.',
                       limitation=('A one-second checkpoint schedule does not bound crash loss to one second when storage stalls.'
                                   if periodic else 'No runtime fsync checkpoint is requested. Source files are written and flushed '
                                   'while recording; only successful final close certifies durable storage. '
                                   'Power loss or abnormal exit before that can lose unsynced records.'))
        return {'schema_version': 1, 'kind': 'MAPPING_INPUT_EXPERIMENT', 'session_id': self.config['session_id'],
                'mode': self.config['mode'], 'source_mode': 'real', 'state': self.state, 'failure_reason': self.failure_reason,
                'cloud_source': self.cloud_source, 'cloud_description': cloud_description(self.cloud_source),
                'cloud_filter': {'settings': copy.deepcopy(self.cloud_filter),
                    'effective_enabled': effective_enabled(self.config, self.cloud_filter),
                    'empty_outputs_skipped': self.filtered_empty_outputs,
                    'totals_by_side': copy.deepcopy(self.filter_totals),
                    'last_output': copy.deepcopy(self.filter_last_report)},
                'continuous_mapping': self.continuous_mapping,
                'max_archived_outputs': None if self.continuous_mapping else MAX_FRAMES,
                'source_wait_policy': 'WAIT_WITHOUT_DEADLINE' if self.continuous_mapping else 'STRICT_TIMEOUT',
                'initial_reference_mode': 'FIXED_MOUNT_REFERENCE_NO_STATIC_CALIBRATION'
                    if self.continuous_mapping else 'MEASURED_STATIC_GRAVITY_WINDOW',
                'input_wait_state': ('WRITING_REFERENCE_CONFIG' if self.transforms is None else
                    'WAITING_MATCHING_PAIR' if any(self.buffers.values()) else
                    'WAITING_FILTERED_POINTS' if self.filter_last_report and not self.filter_last_report['publishable'] else
                    'WAITING_FIRST_INPUT' if self.published_frames == 0 else 'READY_FOR_NEXT_INPUT'),
                'not_yet_received_sources': [side for side, gate in self.gates.items()
                                             if gate.last_source_host_monotonic_ns is None],
                'last_publication_source_age_ns': now_ns-self.last_published_host_ns
                    if self.last_published_host_ns is not None else None,
                'published_frames': self.published_frames, 'archived_outputs': self.archived_outputs,
                'archive_publication_mode': self.archive_publication_mode,
                'archive_checkpoint_policy': self.archive_checkpoint_policy,
                'published_outputs_not_confirmed_archived': max(0, self.published_frames-archived_eligible),
                'published_outputs_not_checkpointed': max(0, self.published_frames-synced_eligible),
                'enqueued_outputs': self.enqueued_outputs, 'pending_outputs': len(self.pending),
                'pending_reserved_bytes': self.pending_bytes,
                'pending_oldest_age_ns': max(0, now_ns-self.pending[0][0]) if self.pending else None,
                'outputs_discarded_on_close': self.outputs_discarded_on_close,
                'archive_queue_limits': {'outputs': self.pending_output_limit, 'reserved_bytes': MAX_PENDING_BYTES,
                                         'accepted_not_checkpointed': MAX_UNCHECKPOINTED_OUTPUTS if periodic else None,
                                         'accounting': 'actual CDR and output bytes, JSON metadata size, and fixed object allowance'
                                         if realtime else 'maximum permitted CDR envelopes plus derived point bytes'},
                'storage': storage, 'close_timeout_s': self.close_timeout_s,
                'status_write_isolated_from_archive': True,
                'status_writer_error': self.status_writer.error if self.status_writer else None,
                'archived_frames': sum(gate.archived_frames for gate in self.gates.values()),
                'received_frames': sum(gate.received_frames for gate in self.gates.values()),
                'bootstrap_complete': self.transforms is not None, 'bootstrap_completed_monotonic_ns': self.bootstrap_completed_ns,
                'bootstrap_progress': {'window_samples': len(self.bootstrap.window), 'window_resets': self.bootstrap.resets,
                    'window_host_span_ns': self.bootstrap.window[-1]['host_monotonic_ns']-self.bootstrap.window[0]['host_monotonic_ns']
                        if self.bootstrap.window else 0,
                    'last_wheel_raw_words': self.bootstrap.last_wheel['register_words_u16'] if self.bootstrap.last_wheel else None,
                    'last_gyro_norm_rad_s': float(np.linalg.norm(self.bootstrap.last_imu['angular_velocity_rad_s']))
                        if self.bootstrap.last_imu else None},
                'bootstrap_discarded': self.bootstrap_discarded, 'unpaired_discarded': self.unpaired_discarded,
                'last_published_stamp_ns': self.last_published_stamp_ns, 'last_pair_delta_ns': self.last_pair_delta_ns,
                'sources': {side: {**gate.status(now_ns), 'cloud_source': self.cloud_source,
                                  'input_topic': source_frame_topic(side, self.cloud_source), 'output_topic': OUTPUT_TOPIC,
                                  'archive_format': 'wc_interfaces/msg/SourceFrame CDR; selected '+self.cloud_source+' SDK cloud bytes preserved'}
                            for side, gate in self.gates.items()},
                'health_subscription_depth': {'imu': 1, 'wheel': 1},
                'health_sequence_gaps_observed': {'imu': self.bootstrap.imu_sequence_gaps,
                                                  'wheel': self.bootstrap.wheel_sequence_gaps},
                'health_sequence_gap_note': 'Latest-sample health/bootstrap subscriptions can intentionally skip sequences; '
                                            'these counters are not hardware packet loss counts. Full-rate recording is in bag.',
                'last_health_observations': {'imu': self.bootstrap.last_imu_observation,
                                             'wheel': self.bootstrap.last_wheel_observation},
                'callback_and_io_timings': timings,
                'output_topic': OUTPUT_TOPIC, 'rate_limit_hz': self.config['input_rate_hz'],
                'rate_policy': ('ALL_VALID_PAIRS_NOT_THROTTLED' if self.config['input_rate_hz'] == 0
                                else 'MINIMUM_SOURCE_AND_PAIR_INTERVAL'),
                'time_source': 'arrival_only', 'formal_acceptance': False, 'navigation_validated': False,
                'common_measurement_time_validated': False, 'updated_monotonic_ns': now_ns,
                'mount_status': 'USER_MEASURED_EXPERIMENT_CANDIDATES', 'publishes_tf': False}

    def write_status(self, now_ns):
        # Coalescing is bounded and does not wait for a slow write/replace/fsync.
        self.worker.status(self.status(now_ns))

    def close(self):
        if self.closed:
            return
        self.closing = True
        deadline = time.monotonic()+self.close_timeout_s
        try:
            if self.worker:
                self.worker.close(deadline-time.monotonic())
                self.poll_io(time.monotonic_ns(), publish=False)
        except Exception as error:
            self.fail(str(error))
        writer_stopped = self.worker is None or not self.worker.thread.is_alive()
        try:
            if self.checkpoints and writer_stopped:
                self.checkpoints.close(deadline-time.monotonic())
        except Exception as error:
            self.fail(str(error))
        sync_stopped = self.checkpoints is None or not self.checkpoints.thread.is_alive()
        try:
            if self.status_writer:
                self.status_writer.close(deadline-time.monotonic())
        except Exception as error:
            self.fail(str(error))
        status_stopped = self.status_writer is None or not self.status_writer.thread.is_alive()
        if writer_stopped and sync_stopped and status_stopped:
            if self.failure_reason is None:
                self.state = 'STOPPED'
                for gate in self.gates.values():
                    gate.state = 'STOPPED'
            try:
                # This final write occurs after ROS callbacks stop and after the
                # final archive checkpoint, so STOPPED never promises pending I/O.
                atomic_status(self.project_root, self.output, self.status(time.monotonic_ns()), sync=True)
            except Exception as error:
                self.fail('FINAL_STATUS_SYNC_FAILED: '+str(error))
                try:
                    atomic_status(self.project_root, self.output, self.status(time.monotonic_ns()), sync=False)
                except Exception:
                    pass  # The caller still returns failure and logs it to stderr.
        if writer_stopped and sync_stopped:
            if self.index is not None:
                self.index.close()
            for archive in self.archives.values():
                archive.close()
        # A daemon stuck in kernel I/O cannot be safely killed or have its fd
        # closed by another thread. Failure is explicit; the supervisor bounds
        # process shutdown. No successful close/checkpoint is claimed.
        self.closed = True


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', required=True, type=Path)
    parser.add_argument('--output-root', required=True, type=Path)
    from .mapping_shutdown import persistence_wait
    parser.add_argument('--close-timeout-s', type=persistence_wait, default=CLOSE_TIMEOUT_S)
    args = parser.parse_args(argv)
    from .cli import ROOT, target
    target()  # Identity check only. The backend is the sole hardware scheduler.
    config_path = project_path(ROOT, args.config)
    output = project_path(ROOT, args.output_root)
    _require(config_path.parent == output.parent and config_path.name == 'runtime_config.json', 'SESSION_CONFIG_OUTPUT_MISMATCH')
    _require(config_path.is_file() and config_path.stat().st_size <= 1_000_000, 'RUNTIME_CONFIG_MISSING_OR_OVERSIZED')
    config = json.loads(config_path.read_text(encoding='utf-8'))
    owner = MappingInput(config, ROOT, output, close_timeout_s=args.close_timeout_s)
    node = None
    ros_started = False
    stopped = False

    def stop(*_):
        nonlocal stopped
        stopped = True  # Never shut down a ROS context from a signal callback.

    def should_stop():
        return stopped

    previous_signals = {number: signal.signal(number, stop)
                        for number in (signal.SIGINT, signal.SIGTERM)}
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        from rclpy.serialization import serialize_message
        from rclpy.signals import SignalHandlerOptions
        from sensor_msgs.msg import PointCloud2, PointField
        from std_msgs.msg import String
        from wc_interfaces.msg import SourceFrame, H30Frame
        rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
        ros_started = True
        node = Node('wc_mapping_input')
        qos = qos_profiles(QoSProfile, ReliabilityPolicy, DurabilityPolicy)
        publisher = node.create_publisher(PointCloud2, OUTPUT_TOPIC, qos['output'])
        def guarded(label, callback):
            def receive(message):
                if stopped or owner.failure_reason is not None:
                    return
                try:
                    owner.timed(label+'_callback', lambda: callback(message))
                except Exception as error:
                    owner.fail(str(error))
                    traceback.print_exc(file=sys.stderr)
                    node.get_logger().error('Mapping input failed: ' + str(error))
            return receive
        def source(message, expected_side):
            _require(message.side == expected_side, 'SOURCE_TOPIC_SIDE_MISMATCH')
            owner.receive_source(message, time.monotonic_ns(), serialize_message, publisher.publish,
                                 cloud_factory=PointCloud2, field_factory=PointField, should_stop=should_stop)
        subscribe_source_frames(node, config, SourceFrame, source, guarded, qos['lidar'])
        node.create_subscription(H30Frame, IMU_TOPIC,
                                 guarded('imu', lambda msg: owner.receive_imu(msg, time.monotonic_ns())), qos['imu'])
        node.create_subscription(String, WHEEL_TOPIC,
                                 guarded('wheel', lambda msg: owner.receive_wheel(msg.data, time.monotonic_ns())), qos['wheel'])
        last_status = 0
        while not stopped and owner.failure_reason is None:
            if not rclpy.ok():
                raise RuntimeError('ROS_CONTEXT_STOPPED_WITHOUT_INPUT_STOP_REQUEST')
            rclpy.spin_once(node, timeout_sec=.1)
            if stopped:
                break
            if not rclpy.ok():
                raise RuntimeError('ROS_CONTEXT_STOPPED_WITHOUT_INPUT_STOP_REQUEST')
            now = time.monotonic_ns()
            if owner.failure_reason is None:
                owner.poll_io(now, should_stop=should_stop)
                if stopped:
                    break
                now = time.monotonic_ns()
                if stopped:
                    break
                owner.check_stale(now)
            if not stopped and (now-last_status >= 1_000_000_000 or owner.failure_reason):
                owner.write_status(now); last_status = now
    except Exception as error:
        owner.fail(str(error))
        traceback.print_exc(file=sys.stderr)
        print('mapping_input: ' + str(error), file=sys.stderr, flush=True)
    finally:
        stopped = True
        try:
            # Strict-mode archive jobs may still serialize ROS messages. Keep
            # their context and objects alive through the bounded archive close.
            try:
                owner.close()
            except Exception as error:
                owner.fail('INPUT_CLOSE_FAILED: '+str(error))
                traceback.print_exc(file=sys.stderr)
            try:
                if node is not None:
                    node.destroy_node()
            except Exception as error:
                owner.fail('INPUT_NODE_DESTROY_FAILED: '+str(error))
                traceback.print_exc(file=sys.stderr)
            try:
                if ros_started and rclpy.ok():
                    rclpy.shutdown()
            except Exception as error:
                owner.fail('INPUT_CONTEXT_SHUTDOWN_FAILED: '+str(error))
                traceback.print_exc(file=sys.stderr)
        finally:
            for number, handler in previous_signals.items():
                signal.signal(number, handler)
        if owner.failure_reason:
            print(owner.failure_reason, file=sys.stderr, flush=True)
    return 1 if owner.failure_reason else 0


if __name__ == '__main__':
    sys.exit(main())
