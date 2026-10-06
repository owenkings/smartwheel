"""Private experimental wheel/gyro motion guess and scan reference transform.

This is arrival-time dead reckoning, not a calibrated common-time estimator.
No serial, device, control, global TF, or formal configuration is accessed.
T_A_B maps B coordinates into A; gyro is SI native-IMU angular velocity.
"""
import argparse
from array import array
from bisect import bisect_left, bisect_right
from collections import deque
import copy
import json
import math
from pathlib import Path
import re
import signal
import threading
import time

import numpy as np
from scipy.spatial.transform import Rotation

from wc_calibration.core import CalibrationError, validate_transform
from wc_calibration.gravity_level import GravityLevelPolicy, estimate_gravity_level
from wc_motion.history_preview import HistoryPreview
from wc_sensors.pointcloud import decode_pointcloud2, validate_pointcloud2
from .mapping_shutdown import persistence_wait
from .mapping_bias import CausalBiasEstimator
from .measurement import prepare_gyro, gyro_z
from .mapping_planar import (PlanarEKF, validate_planar_config, validate_confirmed_bias,
                             native_covariances, NATIVE_CONSTRAINT_VARIANCE)
from .mapping_odometry import (AUTHORITY_TOPIC, adapt_prior_odometry,
                               authority_identity_transform, validate_covariance)


class PriorError(ValueError):
    """Latched estimator/input failure; never silently reset."""


class NotReady(PriorError):
    """Await a stationary initialization or a causal source watermark."""


class DroppedCloud(PriorError):
    """This scan has no retained or observed motion coverage; the session continues."""


DEFAULT_POLICY = {
    'static_duration_s': 2.0, 'static_wheel_speed_m_s': 0.02,
    'max_imu_age_s': 0.25, 'max_wheel_age_s': 0.5,
    'startup_timeout_s': 30.0, 'max_cloud_wait_s': 0.75,
    'max_gyro_rad_s': 3.0, 'max_accel_m_s2': 30.0,
    'max_arrival_clock_delta_s': 0.05, 'history_s': 2.0,
    'max_pair_offset_s': 0.05, 'max_imu_samples': 16000,
    'max_wheel_samples': 4096, 'max_cloud_queue': 8,
    'max_cloud_points': 100000, 'max_cloud_bytes': 16000000,
    # The mapping component has a 30 s normal-stop budget. Leave room for ROS
    # cleanup after observed multi-second eMMC writes finish on their owner.
    'max_pending_writes': 128, 'writer_close_timeout_s': 20.0,
}


def _record_timing(records, name, duration):
    row = records.setdefault(name, {'calls': 0, 'last_s': 0., 'max_s': 0., 'total_s': 0.})
    row['calls'] += 1
    row['last_s'] = max(0., duration)
    row['max_s'] = max(row['max_s'], row['last_s'])
    row['total_s'] += row['last_s']


def drain_ready_callbacks(spin_once, callback_count, *, clock=time.monotonic,
                          max_ready_callbacks=256, max_ready_seconds=.05):
    """One bounded wait, then consume ready work before judging source silence.

    A single spin can choose a different subscription while an IMU callback is
    already ready. Zero-wait draining gives that callback a chance to run. Both
    count and elapsed-time bounds prevent a busy source starving the main loop.
    A callback itself is not interruptible; its measured duration stays visible.
    """
    started = clock()
    before = callback_count()
    spin_once(timeout_sec=.02)
    after_initial = clock()
    initial = callback_count()-before
    ready = attempts = 0
    exhausted = False
    if initial:
        while True:
            if ready >= max_ready_callbacks or clock()-after_initial >= max_ready_seconds:
                exhausted = True
                break
            before_ready = callback_count()
            spin_once(timeout_sec=0.)
            attempts += 1
            count = callback_count()-before_ready
            ready += count
            if not count:
                break
    return {'initial_callbacks': initial, 'ready_callbacks': ready, 'ready_attempts': attempts,
            'ready_budget_exhausted': exhausted, 'initial_spin_s': after_initial-started,
            'ready_drain_s': clock()-after_initial, 'total_s': clock()-started,
            'max_ready_callbacks': max_ready_callbacks, 'max_ready_seconds': max_ready_seconds}


def source_ages(prior, last_receipt, now):
    """Keep consumer receipt age separate from the producer's original clock."""
    rows = {'imu': prior.imu, 'wheel': prior.wheel}
    latest = {key: getattr(prior, 'last_seen', {}).get(key) or (values[-1] if values else None)
              for key, values in rows.items()}
    now_ns = round(now*1e9)
    return {'latest_source_age_s': {key: None if value is None else now-value for key, value in last_receipt.items()},
            'latest_producer_age_s': {key: None if row is None else (now_ns-row['mono'])*1e-9
                                      for key, row in latest.items()},
            'retained_source_samples': {key: len(values) for key, values in rows.items()},
            'last_source_sequence': {key: row['sequence'] if row else None for key, row in latest.items()},
            'last_source_monotonic_ns': {key: row['mono'] if row else None for key, row in latest.items()},
            'last_source_stamp_ns': {key: row['stamp'] if row else None for key, row in latest.items()}}


def check_source_freshness(prior, last_receipt, now):
    ages = source_ages(prior, last_receipt, now)
    recover = getattr(prior, 'recover_startup_candidate', lambda reason, now: False)
    if getattr(prior, 'startup_deadline', None) is not None:
        prior._startup_open(now)  # Missing every source still has the original deadline.
    for name, limit in (('imu', prior.policy['max_imu_age_s']), ('wheel', prior.policy['max_wheel_age_s'])):
        receipt_age = ages['latest_source_age_s'][name]
        producer_age = ages['latest_producer_age_s'][name]
        if producer_age is not None and producer_age < 0:
            raise PriorError(name+' original source time stale/future at consumer')
        reason = None
        if receipt_age is not None and receipt_age > limit:
            reason = name+' source disconnected/stale'
        elif producer_age is not None and producer_age > limit:
            reason = name+' original source time stale/future at consumer'
        if reason and not getattr(prior, 'continuous', False) and not recover(reason, now):
            raise PriorError(reason)
    return ages


def _vector(value, size, name):
    a = np.asarray(value)
    if a.shape != (size,) or a.dtype.kind not in 'iuf' or not np.isfinite(a).all():
        raise PriorError(name + ' must be a finite numeric vector')
    return a.astype(float)


def _integer(value, name, minimum=1):
    if type(value) is not int or value < minimum:
        raise PriorError(name + ' must be an integer >= ' + str(minimum))
    return value


def _rotation(value, name):
    a = np.asarray(value)
    if a.shape != (3, 3) or a.dtype.kind not in 'iuf' or not np.isfinite(a).all() or \
            not np.allclose(a.T @ a, np.eye(3), atol=1e-6, rtol=0) or \
            not math.isclose(float(np.linalg.det(a)), 1., abs_tol=1e-6, rel_tol=0):
        raise PriorError(name + ' must be an explicit proper rotation')
    return a.astype(float)


def validate_config(config):
    config = copy.deepcopy(config)
    config.setdefault('continuous_mapping', False)
    if type(config['continuous_mapping']) is not bool:
        raise PriorError('continuous_mapping must be boolean')
    config.setdefault('motion_model', 'se3_gyro')
    config.setdefault('estimator', 'five_state')
    if config['estimator'] not in ('five_state', 'robot_localization'):
        raise PriorError('estimator must be five_state or robot_localization')
    if config['estimator'] != 'five_state' and config['motion_model'] != 'planar_ekf':
        raise PriorError('robot_localization comparison requires planar_ekf motion_model')
    if config['motion_model'] not in ('se3_gyro', 'planar_ekf'):
        raise PriorError('motion_model must be se3_gyro or planar_ekf')
    try:
        config['confirmed_gyro_bias'] = validate_confirmed_bias(config.get('confirmed_gyro_bias'))
        if config['motion_model'] == 'planar_ekf' or 'planar_ekf' in config:
            config['planar_ekf'] = validate_planar_config(config.get('planar_ekf'))
    except ValueError as error:
        raise PriorError(str(error)) from error
    if config.get('schema_version') != 1 or config.get('status') != 'EXPERIMENT' or \
            config.get('source_mode') != 'real':
        raise PriorError('explicit schema 1 / EXPERIMENT / real configuration required')
    if config.get('formal_odometry_eligible', False) is not False or config.get('time_valid', False) is not False:
        raise PriorError('experimental configuration cannot claim formal eligibility or common time')
    config.setdefault('odometry_source', 'icp')  # Existing diagnostic-only sessions remain compatible.
    if config['odometry_source'] not in ('icp', 'wheel_imu'):
        raise PriorError('odometry_source must be icp or wheel_imu')
    if config['odometry_source'] == 'wheel_imu' or 'wheel_imu_covariance' in config:
        try:
            config['wheel_imu_covariance'] = validate_covariance(config.get('wheel_imu_covariance'))
        except ValueError as error:
            raise PriorError(str(error)) from error
    if config.get('base_frame') not in ('lidar_left', 'lidar_right') or \
            config.get('reference_frame') != 'mapping_reference' or config.get('guess_frame_id') != 'prior_odom':
        raise PriorError('explicit reviewed base/reference/private guess frames required')
    if config.get('imu_sensor_id') != 'H30-0000000015':
        raise PriorError('explicit current H30 identity required')
    _rotation(config.get('R_base_imu'), 'R_base_imu')
    if config.get('mount_model') == 'forward_aligned_axle':
        if 'T_base_axle' in config or config.get('same_forward') is not True:
            raise PriorError('forward-aligned model requires same_forward and excludes T_base_axle')
        position = _vector(config.get('t_axle_base_m'), 3, 't_axle_base_m')
        if np.linalg.norm(position) > 3:
            raise PriorError('axle lever arm exceeds experimental plausibility bound')
    else:
        if any(k in config for k in ('mount_model', 'same_forward', 't_axle_base_m')):
            raise PriorError('unknown/incomplete mounting model')
        try:
            transform = validate_transform(config.get('T_base_axle'))
        except (CalibrationError, ValueError, TypeError) as error:
            raise PriorError('explicit T_base_axle required: ' + str(error)) from error
        if np.linalg.norm(transform[:3, 3]) > 3:
            raise PriorError('axle lever arm exceeds experimental plausibility bound')
    HistoryPreview(config.get('wheel_candidate', {}), continuous=config['continuous_mapping'])
    config.setdefault('imu_topic', '/wc_mapping/imu/source_frame')
    config.setdefault('wheel_topic', '/wc_mapping/wheel/feedback_raw')
    config.setdefault('prior_odom_topic', '/wc_mapping/app/prior_odom')
    config.setdefault('require_dual_time_offsets', False)
    if type(config['require_dual_time_offsets']) is not bool:
        raise PriorError('require_dual_time_offsets must be boolean')
    for key in ('imu_topic', 'wheel_topic', 'input_cloud_topic', 'output_cloud_topic',
                'private_tf_topic', 'prior_odom_topic'):
        value = config.get(key)
        if not isinstance(value, str) or not re.fullmatch(r'/wc_mapping/[A-Za-z0-9_/]+', value):
            raise PriorError(key + ' must name a private /wc_mapping/ topic')
    if len({config[key] for key in ('imu_topic', 'wheel_topic', 'input_cloud_topic',
                                   'output_cloud_topic', 'private_tf_topic', 'prior_odom_topic')}) != 6:
        raise PriorError('input and output topics must all differ')
    if config['odometry_source'] == 'wheel_imu' and AUTHORITY_TOPIC in {
            config[key] for key in ('imu_topic', 'wheel_topic', 'input_cloud_topic',
                                    'output_cloud_topic', 'private_tf_topic', 'prior_odom_topic')}:
        raise PriorError('wheel_imu authority odometry topic must be distinct from all prior topics')
    policy = dict(DEFAULT_POLICY)
    declared = config.get('policy', {})
    if not isinstance(declared, dict) or set(declared) - set(policy):
        raise PriorError('unknown motion prior policy fields')
    policy.update(declared)
    for key, value in policy.items():
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
            raise PriorError('policy values must be finite and positive')
        if key in ('max_imu_samples', 'max_wheel_samples', 'max_cloud_queue', 'max_cloud_points', 'max_cloud_bytes', 'max_pending_writes'):
            _integer(value, key)
    if policy['static_duration_s'] < 2 or policy['history_s'] < policy['max_pair_offset_s'] or \
            policy['max_pair_offset_s'] > .05 or policy['max_imu_samples'] > 100000 or \
            policy['max_wheel_samples'] > 100000 or policy['max_cloud_queue'] > 64 or policy['max_pending_writes'] > 4096:
        raise PriorError('initialization/history/buffer policy outside supported bounds')
    config['policy'] = policy
    gravity = config.get('gravity_policy', {})
    GravityLevelPolicy(**gravity)
    return config


def _skew(v):
    x, y, z = v
    return np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])


def integrate_body_twist(pose, linear, angular, dt):
    """Exact constant-body-twist SE(3) step, including the axle lever arm."""
    phi = np.asarray(angular) * dt
    theta = float(np.linalg.norm(phi))
    skew = _skew(phi)
    if theta < 1e-5:
        jacobian = np.eye(3) + (.5 - theta * theta / 24) * skew + (1/6 - theta * theta / 120) * (skew @ skew)
    else:
        jacobian = np.eye(3) + (1-math.cos(theta))/theta**2 * skew + (theta-math.sin(theta))/theta**3 * (skew @ skew)
    delta = np.eye(4)
    delta[:3, :3] = Rotation.from_rotvec(phi).as_matrix()
    delta[:3, 3] = jacobian @ (np.asarray(linear) * dt)
    return pose @ delta


class MotionPrior:
    """Causal, bounded pure estimator; values after requested time are never used.

    Later timestamps serve only as watermarks, so requests wait for both source
    streams to cover the cloud arrival timestamp. Source packets may share one
    host timestamp; their original sequence and timestamp are preserved.
    """
    def __init__(self, config, session_id, *, clock=time.monotonic):
        self.config = validate_config(config)
        if not isinstance(session_id, str) or not re.fullmatch(r'[A-Za-z0-9_][A-Za-z0-9_.-]{0,95}', session_id):
            raise PriorError('explicit safe session_id required')
        self.session_id = session_id
        self.policy = self.config['policy']
        self.continuous = self.config['continuous_mapping']
        self.planar = self.config['motion_model'] == 'planar_ekf'
        self.planar_anchor = None
        self.planar_cache = {}
        self.wheel_decoder = HistoryPreview(self.config['wheel_candidate'], continuous=self.continuous)
        self.imu, self.wheel, self.checkpoints = [], [], []
        self.failure = None
        self.initialization = None
        self.static_wait_reason = 'waiting_for_source_samples'
        self.imu_sequence_gaps = 0
        self.forwarded = 0
        self.initialization_timings = {}
        self.clock = clock
        self.last_seen = {'imu': None, 'wheel': None}
        self.last_wheel_record = None
        self.startup_deadline = None
        self.startup_discards = 0
        self.startup_discarded_samples = {'imu': 0, 'wheel': 0}
        self.startup_reasons = {}
        self.startup_events = deque(maxlen=16)
        self.startup_enabled = False
        self.coverage_events = deque(maxlen=64)
        self.coverage_gap_count = 0
        self.coverage_incomplete = False
        self.imu_accel_bound_events = deque(maxlen=64)
        self.imu_accel_bound_count = 0
        self.history_discarded_samples = {'imu': 0, 'wheel': 0}
        self.last_forwarded_stamp = None
        # Background candidates never delay startup or change motion by themselves.
        self.bias_estimator = (CausalBiasEstimator(confirmed_bias=self.config['confirmed_gyro_bias'])
                               if self.continuous or self.planar or self.config['confirmed_gyro_bias'] else None)
        self.last_bias_candidate = None

    def _observe_bias(self, row):
        estimator = self.bias_estimator
        if estimator is None:
            return
        if estimator.last_stamp_ns is not None and row['stamp'] < estimator.last_stamp_ns:
            estimator.reset_observations('NONINCREASING_IMU_TIME_NO_BIAS_ESTIMATE')
            return
        if row.get('gap_before'):
            estimator.reset_observations('IMU_SOURCE_COVERAGE_GAP')
        # Use only wheel data actually received by this callback, never future data.
        wh = next((value for value in reversed(self.wheel) if value['stamp'] <= row['stamp']), None)
        if wh is None:
            estimator.reset_observations('NO_CAUSAL_WHEEL_OBSERVATION')
            return
        candidate = estimator.observe(row['stamp'], row['gyro'], row['accel'], wh['stamp'], wh['speeds'])
        if candidate is not None:
            self.last_bias_candidate = candidate

    def bias_report(self):
        if self.bias_estimator is None:
            return {'mode': 'legacy_startup_estimate', 'background_monitor_enabled': False}
        return {**self.bias_estimator.status(), 'background_monitor_enabled': True,
                'automatic_confirmation_enabled': False,
                'latest_candidate': copy.deepcopy(self.last_bias_candidate)}

    def confirm_bias_stationarity(self, start, end, observed, *, evidence_id, source, independent):
        """Explicit evidence API; no live topic or startup path confirms candidates.

        Do not rewrite an already observed/published pose, including when lidar
        confirmation arrives after its source window. Static truth is the caller's
        responsibility; zero wheel feedback alone must never call this method.
        """
        if self.bias_estimator is None:
            raise PriorError('background bias is available only in continuous mapping')
        _integer(observed, 'bias confirmation observation stamp')
        frontier = max([observed, self.last_forwarded_stamp or 0] +
                       [row['stamp'] for row in self.last_seen.values() if row is not None] +
                       [self.checkpoints[-1][0] if self.checkpoints else 0])
        return self.bias_estimator.confirm_stationary(start, end, frontier, evidence_id=evidence_id,
                                                     source=source, independent=independent)

    def enable_startup_recovery(self, started):
        """Adapter-only connecting phase; never extend its original deadline."""
        if self.startup_enabled or self.initialization is not None or self.forwarded or any(self.last_seen.values()):
            raise PriorError('startup recovery must be enabled once before receiving sources')
        self.startup_enabled = True
        self.startup_deadline = None if self.continuous or self.planar else started+self.policy['startup_timeout_s']

    def _startup_open(self, now):
        if self.startup_deadline is None or self.initialization is not None or self.forwarded:
            return False
        if now > self.startup_deadline:
            raise PriorError('startup timeout: '+self.static_wait_reason+'; no initialized forwarded scan')
        return True

    def recover_startup_candidate(self, reason, now):
        if not self._startup_open(now):
            return False
        counts = {'imu': len(self.imu), 'wheel': len(self.wheel)}
        if any(counts.values()):
            self.startup_discards += 1
            self.startup_reasons[reason] = self.startup_reasons.get(reason, 0)+1
            for name, count in counts.items():
                self.startup_discarded_samples[name] += count
            self.startup_events.append({'reason': reason, 'at_monotonic_s': now, 'discarded_samples': counts})
        self.imu.clear()
        self.wheel.clear()
        # last_seen and the strict decoder are never reset by candidate recovery.
        self.static_wait_reason = 'waiting_for_stable_sources: '+reason
        return True

    def startup_report(self):
        return {'enabled': self.startup_enabled, 'deadline_monotonic_s': self.startup_deadline,
                'continuous_mapping': self.continuous,
                'recovery_closed': self.initialization is not None or bool(self.forwarded),
                'discarded_candidates': self.startup_discards,
                'discarded_samples': dict(self.startup_discarded_samples),
                'reasons': dict(self.startup_reasons), 'recent_discards': list(self.startup_events)}

    def _startup_sample_fresh(self, name, row, now):
        if not self._startup_open(now):
            return True
        age = (round(now*1e9)-row['mono'])*1e-9
        if age < 0:
            raise PriorError(name+' original source time stale/future at consumer')
        if age > self.policy['max_'+name+'_age_s']:
            self.recover_startup_candidate(name+' original sample stale during startup', now)
            self.startup_discarded_samples[name] += 1
            return False
        return True

    def _coverage_gap(self, source, start, end, reason):
        if end <= start:
            return
        self.coverage_incomplete = True
        self.coverage_gap_count += 1
        self.coverage_events.append({'source': source, 'start_ns': start, 'end_ns': end, 'reason': reason,
                                     'motion_integrated_across_gap': False})

    def coverage_report(self):
        return {'continuous_mapping': self.continuous, 'motion_coverage_complete': not self.coverage_incomplete,
                'coverage_gap_count': self.coverage_gap_count, 'recent_coverage_gaps': list(self.coverage_events),
                'imu_acceleration_diagnostics': {
                    'total_bound_exceeded_samples': self.imu_accel_bound_count,
                    'recent_examples': list(self.imu_accel_bound_events),
                    'example_capacity': self.imu_accel_bound_events.maxlen,
                    'policy': 'initialized planar only: retain gyro and original acceleration; reset/skip bias observation',
                    'acceleration_fused_into_planar_motion': False,
                    'max_accel_m_s2': self.policy['max_accel_m_s2'],
                    'max_gyro_rad_s': self.policy['max_gyro_rad_s']},
                'history_discarded_samples': dict(self.history_discarded_samples),
                'retained_pose_start_ns': self.checkpoints[0][0] if self.checkpoints else None,
                'unobserved_motion_policy': 'hold last pose; no extrapolation through missing intervals'}

    def _startup_candidate_fresh(self):
        now = self.clock()
        if self._startup_open(now):
            for name in ('imu', 'wheel'):
                row = self.last_seen[name]
                if row is not None:
                    age = (round(now*1e9)-row['mono'])*1e-9
                    if age < 0:
                        raise PriorError(name+' original source time stale/future at consumer')
                    if age > self.policy['max_'+name+'_age_s']:
                        self.recover_startup_candidate(name+' candidate original time stale before initialization', now)
                        return False
        return True

    def _check(self):
        if self.failure:
            raise PriorError('motion prior FAILED: ' + self.failure)

    def _fail(self, error):
        self.failure = str(error)
        raise PriorError(self.failure) from error

    def add_imu(self, *, stamp_ns, monotonic_ns, acceleration, angular_velocity,
                sensor_id, session_id, stream_epoch, sequence,
                coordinate_convention='H30_NATIVE_UNVALIDATED', time_source='arrival_only', common_time_valid=False):
        self._check()
        try:
            _integer(stamp_ns, 'IMU stamp'); _integer(monotonic_ns, 'IMU monotonic'); _integer(sequence, 'IMU sequence', 0)
            if sensor_id != self.config['imu_sensor_id'] or session_id != self.session_id or \
                    coordinate_convention != 'H30_NATIVE_UNVALIDATED' or time_source != 'arrival_only' or \
                    common_time_valid is not False or not isinstance(stream_epoch, str) or not stream_epoch:
                raise PriorError('IMU identity, axes or arrival-only classification mismatch')
            accel = _vector(acceleration, 3, 'SI acceleration')
            gyro = _vector(angular_velocity, 3, 'SI gyro')
            accel_norm = float(np.linalg.norm(accel))
            gyro_norm = float(np.linalg.norm(gyro))
            accel_exceeds_bound = accel_norm > self.policy['max_accel_m_s2']
            # Acceleration is not a planar motion measurement. After startup,
            # a finite out-of-bound value invalidates stationary bias evidence,
            # not the independent gyro packet. Startup/se3 and gyro stay strict.
            skip_bias_observation = (accel_exceeds_bound and math.isfinite(accel_norm)
                                     and self.planar and self.initialization is not None)
            if gyro_norm > self.policy['max_gyro_rad_s'] or (accel_exceeds_bound and not skip_bias_observation):
                raise PriorError('IMU SI magnitude exceeds declared experimental bound')
            now = self.clock()
            old = self.last_seen['imu']
            gap = False
            if old is not None:
                if old['epoch'] != stream_epoch or sequence <= old['sequence'] or stamp_ns < old['stamp'] or monotonic_ns < old['mono']:
                    raise PriorError('IMU epoch/sequence/time discontinuity')
                self.imu_sequence_gaps += sequence - old['sequence'] - 1
                clock_gap = self._clock_interval(stamp_ns-old['stamp'], monotonic_ns-old['mono'])
                gap = clock_gap or (stamp_ns-old['stamp'])*1e-9 > self.policy['max_imu_age_s'] or sequence > old['sequence']+1
                if self.continuous and gap:
                    self._coverage_gap('imu', old['stamp'], stamp_ns, 'positive source time/sequence gap')
                elif (stamp_ns-old['stamp'])*1e-9 > self.policy['max_imu_age_s']:
                    if not self.recover_startup_candidate('IMU sample gap exceeds declared bound', now):
                        raise PriorError('IMU sample gap exceeds declared bound')
            row = dict(stamp=stamp_ns, mono=monotonic_ns, accel=accel, gyro=gyro, epoch=stream_epoch, sequence=sequence,
                       gap_before=gap)
            self.last_seen['imu'] = row
            if not self._startup_sample_fresh('imu', row, now):
                return
            self.imu.append(row)
            self._invalidate_planar_after(old['stamp'] if old else 0)
            if skip_bias_observation:
                self.imu_accel_bound_count += 1
                self.imu_accel_bound_events.append({
                    'reason': 'ACCELERATION_BOUND_EXCEEDED_NOT_FUSED_PLANAR',
                    'action': 'GYRO_RETAINED_BIAS_OBSERVATION_SKIPPED_AND_WINDOW_RESET',
                    'sensor_id': sensor_id, 'session_id': session_id, 'stream_epoch': stream_epoch,
                    'sequence': sequence, 'stamp_ns': stamp_ns, 'receive_monotonic_ns': monotonic_ns,
                    'acceleration_native_m_s2': accel.tolist(), 'gyro_native_rad_s': gyro.tolist(),
                    'acceleration_norm_m_s2': accel_norm, 'gyro_norm_rad_s': gyro_norm,
                    'max_accel_m_s2': self.policy['max_accel_m_s2'],
                    'max_gyro_rad_s': self.policy['max_gyro_rad_s'],
                    'motion_sample_retained': True, 'bias_observation_eligible': False})
                if self.bias_estimator is not None:
                    self.bias_estimator.reset_observations('ACCELERATION_BOUND_EXCEEDED_NOT_FUSED_PLANAR')
            else:
                self._observe_bias(row)
            self._bound()
            self.try_initialize()
        except (ValueError, TypeError, KeyError) as error:
            self._fail(error)

    def _clock_interval(self, wall_delta_ns, mono_delta_ns):
        if abs(wall_delta_ns-mono_delta_ns)*1e-9 > self.policy['max_arrival_clock_delta_s']:
            if self.continuous and wall_delta_ns >= 0 and mono_delta_ns >= 0:
                return True  # A forward clock jump leaves an unobserved interval.
            raise PriorError('host arrival clock discontinuity relative to monotonic clock')
        return False

    def add_wheel(self, record):
        self._check()
        try:
            now = self.clock()
            connecting = self._startup_open(now)
            # Stateless validation runs the same CRC, device, classification and
            # magnitude checks. Strict continuity is kept independently below;
            # no previously blocked HistoryPreview is reset or reused.
            decoder = HistoryPreview(self.config['wheel_candidate'], continuous=self.continuous) if connecting else self.wheel_decoder
            sample = decoder.update(record)
            old = self.last_seen['wheel']
            gap = False
            if old is not None:
                if sample['stream_epoch'] != old['epoch'] or sample['sequence'] <= old['sequence'] or \
                        sample['stamp_ns'] <= old['stamp'] or sample['receive_monotonic_ns'] <= old['mono']:
                    raise PriorError('wheel identity/sequence/time discontinuity')
                clock_gap = self._clock_interval(sample['stamp_ns']-old['stamp'], sample['receive_monotonic_ns']-old['mono'])
                limit = min(self.policy['max_wheel_age_s'], self.config['wheel_candidate']['max_gap_s'])
                gap = clock_gap or sample['sequence'] != old['sequence']+1 or (sample['receive_monotonic_ns']-old['mono'])*1e-9 > limit
                if self.continuous and gap:
                    self._coverage_gap('wheel', old['stamp'], sample['stamp_ns'], 'positive source time/sequence gap')
                elif gap:
                    if not self.recover_startup_candidate('wheel sequence/sample gap exceeds declared bound', now):
                        raise PriorError('wheel sequence/sample gap exceeds declared bound')
            factor = 2*math.pi*self.config['wheel_candidate']['wheel_radius_m']/60
            speeds = np.array([sample['left_wheel_rpm_candidate'], sample['right_wheel_rpm_candidate']]) * factor
            row = dict(stamp=sample['stamp_ns'], mono=sample['receive_monotonic_ns'], speeds=speeds,
                       v=sample['linear_velocity_m_s'], wheel_yaw=sample['angular_velocity_rad_s'],
                       sequence=sample['sequence'], epoch=sample['stream_epoch'], gap_before=gap)
            self.last_seen['wheel'] = row
            self.last_wheel_record = copy.deepcopy(record)
            if not self._startup_sample_fresh('wheel', row, now):
                return
            self.wheel.append(row)
            self._invalidate_planar_after(old['stamp'] if old else 0)
            if gap and self.bias_estimator is not None:
                self.bias_estimator.reset_observations('WHEEL_SOURCE_COVERAGE_GAP')
            self._bound()
            self.try_initialize()
        except (ValueError, TypeError, KeyError) as error:
            self._fail(error)

    def _bound(self):
        if self.continuous or self.planar:
            self._retire_history()
            return
        if self.initialization is None:
            # Keep a little more than the initialization window while waiting for stillness.
            for rows in (self.imu, self.wheel):
                if rows:
                    cutoff = rows[-1]['stamp'] - int((self.policy['static_duration_s']+1)*1e9)
                    index = max(0, bisect_right([r['stamp'] for r in rows], cutoff)-1)
                    del rows[:index]
        if len(self.imu) > self.policy['max_imu_samples'] or len(self.wheel) > self.policy['max_wheel_samples']:
            raise PriorError('motion source history exceeded bounded capacity')

    def _retire_history(self):
        """Advance a bounded pose anchor even when clouds or one source stop."""
        if self.planar_anchor is not None:
            return self._retire_planar_history()
        streams = {'imu': self.imu, 'wheel': self.wheel}
        latest = max((rows[-1]['stamp'] for rows in streams.values() if rows), default=0)
        cutoff = latest-int(self.policy['history_s']*1e9)
        for name, rows in streams.items():
            capacity = self.policy['max_'+name+'_samples']
            if len(rows) > capacity:
                cutoff = max(cutoff, rows[-capacity]['stamp'])
        if self.checkpoints and cutoff > self.checkpoints[0][0]:
            start = self.checkpoints[0][0]
            watermark = min(self.imu[-1]['stamp'], self.wheel[-1]['stamp'])
            observed_end = min(cutoff, watermark)
            pose = self.pose_at(observed_end) if observed_end >= start else self.checkpoints[0][1].copy()
            if cutoff > max(start, watermark):
                self._coverage_gap('history', max(start, watermark), cutoff, 'source absent while bounded history advances')
            self.checkpoints = [(cutoff, pose)]+[p for p in self.checkpoints if p[0] > cutoff]
            if self.bias_estimator is not None and self.bias_estimator.last_stamp_ns is not None:
                anchor = min(cutoff, self.bias_estimator.last_stamp_ns)
                if anchor >= self.bias_estimator.retained_from_ns:
                    self.bias_estimator.prune_before(anchor)
        for name, rows in streams.items():
            if rows:
                index = max(0, bisect_right([r['stamp'] for r in rows], cutoff)-1)
                self.history_discarded_samples[name] += index
                del rows[:index]

    def try_initialize(self):
        started = time.monotonic()
        try:
            return self._try_initialize()
        finally:
            _record_timing(self.initialization_timings, 'try_initialize', time.monotonic()-started)

    def _try_initialize(self):
        if not self._startup_candidate_fresh():
            return False
        if self.initialization is not None or not self.imu or not self.wheel:
            return self.initialization is not None
        if self.continuous or self.planar:
            return self._initialize_from_mount()
        horizon = min(self.imu[-1]['stamp'], self.wheel[-1]['stamp'])
        cutoff = horizon-int(self.policy['static_duration_s']*1e9)
        end = bisect_right([r['stamp'] for r in self.imu], horizon)
        begin = max(0, bisect_right([r['stamp'] for r in self.imu], cutoff)-1)
        samples = self.imu[begin:end]
        if not samples or samples[-1]['stamp']-samples[0]['stamp'] < int(self.policy['static_duration_s']*1e9):
            return False
        wheel_start = bisect_right([r['stamp'] for r in self.wheel], samples[0]['stamp'])-1
        if wheel_start < 0:
            return False
        wheels = [r for r in self.wheel[wheel_start:] if r['stamp'] <= horizon]
        if not wheels or max(float(np.max(np.abs(r['speeds']))) for r in wheels) > self.policy['static_wheel_speed_m_s']:
            self.static_wait_reason = 'wheel_motion_during_gravity_window'
            return False
        try:
            gravity = estimate_gravity_level([r['accel'] for r in samples], self.config['R_base_imu'],
                                              angular_velocity=[r['gyro'] for r in samples], policy=self.config.get('gravity_policy'))
        except CalibrationError as error:
            self.static_wait_reason = str(error)
            return False
        # Fitting cost is measured separately. It cannot make old samples fresh
        # or allow a completed old window to commit after a callback delay.
        if not self._startup_candidate_fresh():
            return False
        if self.startup_deadline is not None:
            self.wheel_decoder.update(self.last_wheel_record)
        self.L = np.asarray(gravity['R_level_left'])
        self.R_reference_imu = self.L @ np.asarray(self.config['R_base_imu'])
        self.bias = np.mean([r['gyro'] for r in samples], axis=0)
        level4 = np.eye(4); level4[:3, :3] = self.L
        if self.config.get('mount_model') == 'forward_aligned_axle':
            self.T_reference_axle = np.eye(4)
            self.T_reference_axle[:3, 3] = -np.asarray(self.config['t_axle_base_m'])
            base_axle = np.linalg.inv(level4) @ self.T_reference_axle
            mount_basis = 'user_reported_forward_aligned_unvalidated'
        else:
            base_axle = np.asarray(self.config['T_base_axle'])
            self.T_reference_axle = level4 @ base_axle
            mount_basis = 'explicit_T_base_axle_unvalidated'
        self.initialization = {
            'schema_version': 1, 'status': 'EXPERIMENT', 'source_mode': 'real',
            'odometry_source': self.config['odometry_source'],
            'wheel_imu_covariance': self.config.get('wheel_imu_covariance'),
            'session_id': self.session_id, 'time_source': 'arrival_only', 'time_valid': False,
            'formal_odometry_eligible': False, 'mounting_validated': False, 'ground_height_validated': False,
            'stamp_ns': horizon, 'window_start_ns': samples[0]['stamp'], 'window_end_ns': horizon,
            'imu_sequence_start': samples[0]['sequence'], 'imu_sequence_end': samples[-1]['sequence'],
            'imu_epoch': samples[0]['epoch'], 'wheel_epoch': wheels[0]['epoch'],
            'gravity': gravity, 'R_reference_base': self.L.tolist(),
            'gravity_input_base_frame': self.config['base_frame'],
            'gravity_field_convention': 'Reusable gravity module left-named fields refer to the selected base_frame in this experiment.',
            'R_reference_imu': self.R_reference_imu.tolist(), 'T_reference_axle': self.T_reference_axle.tolist(),
            'T_base_axle_derived_or_supplied': base_axle.tolist(), 'mounting_basis': mount_basis,
            'gyro_bias_candidate_rad_s': self.bias.tolist(), 'wheel_samples': len(wheels),
            'definition': 'Reference origin is lidar origin; initial gravity +Z, projected lidar forward +X; no absolute yaw or ground height.',
            'estimator': 'wheel longitudinal velocity plus bias-subtracted 3-axis gyro, axle lever arm, causal zero-order hold',
        }
        self.checkpoints = [(horizon, np.eye(4))]
        return True

    def _initialize_from_mount(self):
        """Use configured axle axes; moving accelerometers cannot establish gravity."""
        if self.config.get('mount_model') == 'forward_aligned_axle':
            base_axle = np.eye(4)
            base_axle[:3, 3] = -np.asarray(self.config['t_axle_base_m'])
            mount_basis = 'user_reported_forward_aligned_unvalidated'
        else:
            base_axle = np.asarray(self.config['T_base_axle'])
            mount_basis = 'explicit_T_base_axle_unvalidated'
        self.L = base_axle[:3, :3].T.copy()
        level = np.eye(4); level[:3, :3] = self.L
        self.R_reference_imu = self.L @ np.asarray(self.config['R_base_imu'])
        self.T_reference_axle = level @ base_axle
        self.bias = (self.bias_estimator.bias_at(0) if self.bias_estimator is not None else np.zeros(3))
        horizon = max(self.imu[-1]['stamp'], self.wheel[-1]['stamp'])
        self.initialization = {
            'schema_version': 1, 'status': 'EXPERIMENT', 'source_mode': 'real',
            'continuous_mapping': self.continuous, 'odometry_source': self.config['odometry_source'],
            'wheel_imu_covariance': self.config.get('wheel_imu_covariance'),
            'session_id': self.session_id, 'time_source': 'arrival_only', 'time_valid': False,
            'formal_odometry_eligible': False, 'mounting_validated': False, 'ground_height_validated': False,
            'stamp_ns': horizon, 'window_start_ns': None, 'window_end_ns': None,
            'imu_sequence_start': self.imu[-1]['sequence'], 'imu_sequence_end': self.imu[-1]['sequence'],
            'imu_epoch': self.imu[-1]['epoch'], 'wheel_epoch': self.wheel[-1]['epoch'],
            'gravity': None, 'gravity_estimated': False, 'reference_basis': 'configured_axle_axes_not_measured_level',
            # Legacy consumers use this field to identify the input coordinate
            # frame. Keeping its name does not claim a gravity observation.
            'gravity_input_base_frame': self.config['base_frame'],
            'R_reference_base': self.L.tolist(), 'R_reference_imu': self.R_reference_imu.tolist(),
            'T_reference_axle': self.T_reference_axle.tolist(), 'T_base_axle_derived_or_supplied': base_axle.tolist(),
            'mounting_basis': mount_basis, 'gyro_bias_candidate_rad_s': self.bias.tolist(),
            'gyro_bias_status': 'UNKNOWN_UNESTIMATED_ZERO_CANDIDATE', 'wheel_samples': 1,
            'definition': 'Reference origin is lidar origin; axes follow configured axle; no measured gravity, absolute yaw or ground height.',
            'estimator': 'wheel longitudinal velocity plus versioned native-bias-subtracted 3-axis gyro; missing intervals hold pose',
        }
        self.static_wait_reason = 'initialized_from_configured_mount_without_static_window'
        self.checkpoints = [(horizon, np.eye(4))]
        self.initialization.update(motion_model=self.config['motion_model'],
                                   confirmed_gyro_bias=copy.deepcopy(self.config['confirmed_gyro_bias']))
        if self.config['confirmed_gyro_bias'] is not None:
            self.initialization['gyro_bias_status'] = 'EXPLICIT_EXTERNAL_CALIBRATION_DECLARATION'
        if self.planar:
            from .estimator_provider import create_estimator
            state = create_estimator(self.config['estimator'], self.config['planar_ekf'],
                                     self.T_reference_axle[:2, 3])
            self._planar_observe(state, horizon, initial=True)
            self.planar_anchor = (horizon, state)
            self.initialization.update(planar_ekf=self.config['planar_ekf'],
                estimator=self.config['estimator'],
                definition='Lidar origin reference; configured axle XY plane, constrained z/roll/pitch motion; not measured level.')
        return True

    def _invalidate_planar_after(self, stamp):
        # Future samples can delimit a missing interval starting at the old
        # last packet. No cached state strictly after that boundary is reusable.
        self.planar_cache = {t: value for t, value in self.planar_cache.items() if t <= stamp}

    def _planar_observe(self, state, stamp, *, initial=False, grouped=None):
        """Each original sequence updates once; never fabricate intermediate observations."""
        for kind, rows in (('wheel', self.wheel), ('imu', self.imu)):
            if grouped is not None:
                selected = grouped[kind].get(stamp, ())
            else:
                stamps = [r['stamp'] for r in rows]
                if initial:
                    selected = rows[max(0, bisect_right(stamps, stamp)-1):bisect_right(stamps, stamp)]
                else:
                    selected = rows[bisect_left(stamps, stamp):bisect_right(stamps, stamp)]
            for row in selected:
                if kind == 'wheel':
                    updated = state.wheel(row['sequence'], row['v'], row['wheel_yaw'])
                    if updated is not None:
                        state.last_update['wheel'].update(stamp_ns=stamp,stream_epoch=row['epoch'],
                            measurement=[row['v'],row['wheel_yaw']],measurement_frame='axle')
                else:
                    bias = self.bias if self.bias_estimator is None else self.bias_estimator.bias_at(stamp)
                    prepared = prepare_gyro(row['gyro'], bias, self.R_reference_imu)
                    if self.bias_estimator is not None:
                        prepared['bias_provenance']=self.bias_estimator.bias_metadata_at(stamp)
                        prepared['bias_status']=prepared['bias_provenance']['calibration_status']
                    updated = state.gyro(row['sequence'], prepared['target_rad_s'][2])
                    if updated is not None:
                        state.last_update['gyro'].update(stamp_ns=stamp,stream_epoch=row['epoch'],
                            measurement=[prepared['target_rad_s'][2]],measurement_frame='configured_axle_z',
                            preparation=prepared)

    def _planar_state_at(self, stamp_ns, *, allow_uncovered=False):
        """Replay from an event anchor; queries and scan checkpoints never commit a filter.

        A query at an event timestamp is before that timestamp's measurement
        corrections. Those affect subsequent time only. This also preserves
        history when multiple valid IMU sequences share one original timestamp.
        """
        start, saved = self.planar_anchor
        cached = [t for t in self.planar_cache if start < t <= stamp_ns]
        if cached:
            start = max(cached)
            saved = self.planar_cache[start]
        state = saved.copy()
        its, wts = [r['stamp'] for r in self.imu], [r['stamp'] for r in self.wheel]
        real_events = set(its+wts)
        events = sorted({start, stamp_ns} | {s for s in real_events if start <= s < stamp_ns})
        grouped = {'wheel': {}, 'imu': {}}
        for kind, rows in (('wheel', self.wheel), ('imu', self.imu)):
            for row in rows:
                if start <= row['stamp'] < stamp_ns:
                    grouped[kind].setdefault(row['stamp'], []).append(row)
        watermark = min(its[-1], wts[-1])
        for begin, end in zip(events, events[1:]):
            ii, wi = bisect_right(its, begin)-1, bisect_right(wts, begin)-1
            observed = end <= watermark and min(ii, wi) >= 0 and self._span_observed(ii, wi, begin, end)
            if not observed:
                if not self.continuous and not allow_uncovered:
                    raise PriorError('causal motion sample age exceeds declared limit')
                state.decorrelate_gap()
            self._planar_observe(state, begin, grouped=grouped)
            state.predict((end-begin)*1e-9, observed=observed)
            if end in real_events and end <= watermark:
                self.planar_cache[end] = state.copy()
        return state

    def _retire_planar_history(self):
        streams = {'imu': self.imu, 'wheel': self.wheel}
        latest = max(rows[-1]['stamp'] for rows in streams.values() if rows)
        cutoff = latest-int(self.policy['history_s']*1e9)
        for name, rows in streams.items():
            capacity = self.policy['max_'+name+'_samples']
            if len(rows) > capacity:
                cutoff = max(cutoff, rows[-capacity]['stamp'])
        # Anchor only at a real source event. Partial cloud-query predictions
        # never split process noise intervals or change future state estimates.
        events = [row['stamp'] for rows in streams.values() for row in rows
                  if self.planar_anchor[0] < row['stamp'] <= cutoff]
        if events:
            anchor = max(events)
            state = self._planar_state_at(anchor, allow_uncovered=True)
            watermark = min(self.imu[-1]['stamp'], self.wheel[-1]['stamp'])
            if anchor > max(self.planar_anchor[0], watermark):
                self._coverage_gap('history', max(self.planar_anchor[0], watermark), anchor,
                                   'source absent while bounded planar history advances')
            self.planar_anchor = (anchor, state)
            self.planar_cache = {t: value for t, value in self.planar_cache.items() if t > anchor}
            pose = state.output(self.T_reference_axle[:3, 3])[0]
            self.checkpoints = [(anchor, pose)]+[p for p in self.checkpoints if p[0] > anchor]
            for name, rows in streams.items():
                # Keep all distinct IMU packets at the anchor timestamp: they
                # have not yet updated its pre-event state.
                index = max(0, bisect_left([r['stamp'] for r in rows], anchor)-1)
                self.history_discarded_samples[name] += index
                del rows[:index]
            if self.bias_estimator is not None and self.bias_estimator.last_stamp_ns is not None:
                retired = min(anchor, self.bias_estimator.last_stamp_ns)
                if retired >= self.bias_estimator.retained_from_ns:
                    self.bias_estimator.prune_before(retired)

    def _twist(self, imu, wheel, stamp_ns=None):
        if self.bias_estimator is None:
            bias = self.bias
        else:
            query = imu.get('stamp') if stamp_ns is None else stamp_ns
            bias = (np.asarray(self.bias_estimator.versions[-1].bias_native_rad_s) if query is None else
                    self.bias_estimator.bias_at(query))
        angular = np.asarray(prepare_gyro(imu['gyro'], bias, self.R_reference_imu)['target_rad_s'])
        linear = self.T_reference_axle[:3, :3] @ np.array([wheel['v'], 0., 0.]) - np.cross(angular, self.T_reference_axle[:3, 3])
        return linear, angular

    def pose_at(self, stamp_ns):
        """Non-mutating historical pose query, no extrapolation beyond either watermark."""
        self._check()
        _integer(stamp_ns, 'pose query stamp')
        if self.initialization is None:
            raise NotReady(self.static_wait_reason)
        if stamp_ns < self.checkpoints[0][0]:
            error = DroppedCloud if self.continuous else PriorError
            raise error('source time predates retained initialized pose history')
        if min(self.imu[-1]['stamp'], self.wheel[-1]['stamp']) < stamp_ns:
            raise NotReady('waiting for IMU and wheel watermarks')
        if self.planar:
            return self._planar_state_at(stamp_ns).output(self.T_reference_axle[:3, 3])[0]
        checkpoint = bisect_right([r[0] for r in self.checkpoints], stamp_ns)-1
        start, pose = self.checkpoints[checkpoint]
        pose = pose.copy()
        its, wts = [r['stamp'] for r in self.imu], [r['stamp'] for r in self.wheel]
        ii, wi = bisect_right(its, start)-1, bisect_right(wts, start)-1
        if min(ii, wi) < 0:
            raise PriorError('source history cannot cover retained pose checkpoint')
        bias_events = self.bias_estimator.change_stamps(start, stamp_ns) if self.bias_estimator is not None else []
        events = sorted(set([start, stamp_ns] + bias_events + [s for s in its[ii+1:] if s < stamp_ns] + [s for s in wts[wi+1:] if s < stamp_ns]))
        for begin, end in zip(events, events[1:]):
            ii, wi = bisect_right(its, begin)-1, bisect_right(wts, begin)-1
            im, wh = self.imu[ii], self.wheel[wi]
            if self.continuous and not self._span_observed(ii, wi, begin, end):
                continue
            if (end-im['stamp'])*1e-9 > self.policy['max_imu_age_s'] or (end-wh['stamp'])*1e-9 > self.policy['max_wheel_age_s']:
                raise PriorError('causal motion sample age exceeds declared limit')
            linear, angular = self._twist(im, wh, begin)
            pose = integrate_body_twist(pose, linear, angular, (end-begin)*1e-9)
        return pose

    def _span_observed(self, ii, wi, begin, end):
        for name, rows, index in (('imu', self.imu, ii), ('wheel', self.wheel, wi)):
            row = rows[index]
            if (end-row['stamp'])*1e-9 > self.policy['max_'+name+'_age_s']:
                return False
            if index+1 < len(rows) and rows[index+1].get('gap_before') and begin < rows[index+1]['stamp']:
                return False
        return True

    def require_observed_cloud_time(self, stamp):
        if not self.continuous:
            return
        for name, rows in (('imu', self.imu), ('wheel', self.wheel)):
            index = bisect_right([r['stamp'] for r in rows], stamp)-1
            if index < 0:
                raise DroppedCloud('cloud predates retained '+name+' coverage')
            row = rows[index]
            if (stamp-row['stamp'])*1e-9 > self.policy['max_'+name+'_age_s'] or (
                    stamp > row['stamp'] and index+1 < len(rows) and rows[index+1].get('gap_before')):
                raise DroppedCloud('cloud lies in unobserved '+name+' interval')

    def record_forwarded(self, stamp_ns, pose):
        previous = self.last_forwarded_stamp if self.continuous else (self.checkpoints[-1][0] if self.forwarded else None)
        if previous is not None and stamp_ns <= previous:
            raise PriorError('forwarded cloud stamps must strictly increase')
        self.checkpoints.append((stamp_ns, pose.copy()))
        self.forwarded += 1
        self.last_forwarded_stamp = stamp_ns
        if self.continuous or self.planar:
            self._retire_history()
            return
        cutoff = stamp_ns-int(self.policy['history_s']*1e9)
        index = max(0, bisect_right([p[0] for p in self.checkpoints], cutoff)-1)
        del self.checkpoints[:index]
        start = self.checkpoints[0][0]
        for rows in (self.imu, self.wheel):
            index = max(0, bisect_right([r['stamp'] for r in rows], start)-1)
            del rows[:index]

    def sample_report(self, stamp_ns, pose):
        im = self.imu[bisect_right([r['stamp'] for r in self.imu], stamp_ns)-1]
        wh = self.wheel[bisect_right([r['stamp'] for r in self.wheel], stamp_ns)-1]
        linear, angular = self._twist(im, wh, stamp_ns)
        details = {'motion_model': self.config['motion_model']}
        if self.planar:
            state = self._planar_state_at(stamp_ns)
            _, linear, angular, pose_cov, twist_cov = state.output(self.T_reference_axle[:3, 3])
            pose_cov, twist_cov = native_covariances(pose_cov, twist_cov)
            details.update(planar_ekf=state.report(), pose_covariance=pose_cov.reshape(-1).tolist(),
                           twist_covariance=twist_cov.reshape(-1).tolist(),
                           native_covariance_constraint_variance=NATIVE_CONSTRAINT_VARIANCE,
                           covariance_status='PROPAGATED_MODEL_WITH_CONSTRAINT_RESIDUAL_NOT_MEASURED_ACCURACY')
        return {'stamp_ns': stamp_ns, 'status': 'EXPERIMENT', 'time_source': 'arrival_only',
                'time_valid': False, 'formal_odometry_eligible': False,
                'T_prior_reference': pose.tolist(), 'linear_velocity_reference_m_s': linear.tolist(),
                'angular_velocity_reference_rad_s': angular.tolist(), 'imu_sequence': im['sequence'],
                'wheel_sequence': wh['sequence'], 'imu_stamp_ns': im['stamp'], 'wheel_stamp_ns': wh['stamp'],
                'wheel_yaw_rate_diagnostic_rad_s': wh['wheel_yaw'], 'imu_sequence_gaps': self.imu_sequence_gaps,
                'gyro_bias_applied_native_rad_s': (self.bias if self.bias_estimator is None else
                    self.bias_estimator.bias_at(stamp_ns)).tolist(),
                **self.coverage_report(), **details}


def _stamp(value):
    sec, ns = int(value.sec), int(value.nanosec)
    if sec < 0 or not 0 <= ns < 1000000000:
        raise PriorError('invalid ROS source timestamp')
    return _integer(sec*1000000000+ns, 'ROS stamp')


def adapt_motion_odometry(message, config):
    """Keep the propagated planar covariance across the authority frame adapter."""
    result = adapt_prior_odometry(message, config['wheel_imu_covariance'])
    if config['motion_model'] == 'planar_ekf':
        result.pose.covariance = list(message.pose.covariance)
        result.twist.covariance = list(message.twist.covariance)
    return result


def _field_values(message, layout, name, storage=None):
    field = layout['fields'][name]
    dtype = np.dtype(('>' if layout['is_bigendian'] else '<') + field['dtype'])
    return np.ndarray((layout['height'], layout['width']), dtype=dtype,
                      buffer=message.data if storage is None else storage, offset=field['offset'],
                      strides=(layout['row_step'], layout['point_step']))


def prepare_cloud(message, prior):
    """Preserve fields/point order; rotate and optionally compensate two arrivals."""
    if message.header.frame_id != prior.config['base_frame']:
        raise PriorError('input cloud frame differs from explicitly configured lidar base')
    if len(message.data) > prior.policy['max_cloud_bytes']:
        raise PriorError('cloud byte capacity exceeded')
    layout = validate_pointcloud2(message)
    if not 0 < layout['point_count'] <= prior.policy['max_cloud_points']:
        raise PriorError('cloud point capacity invalid')
    stamp = _stamp(message.header.stamp)
    pose = prior.pose_at(stamp)
    prior.require_observed_cloud_time(stamp)
    xyz = decode_pointcloud2(message)
    finite = np.isfinite(xyz).all(axis=1)
    if not finite.any():
        raise PriorError('cloud contains no finite XYZ points')
    offsets = np.zeros(layout['point_count'], dtype=np.int64)
    dual = 'time_offset_s' in layout['fields']
    if prior.config['require_dual_time_offsets'] and not dual:
        raise PriorError('all mode requires explicit dual arrival offsets and source codes')
    if dual:
        field = layout['fields']['time_offset_s']
        source = layout['fields'].get('source_code')
        if field['datatype'] != 8 or field['count'] != 1 or not source or source['datatype'] != 2 or source['count'] != 1:
            raise PriorError('dual input requires FLOAT64 time_offset_s and UINT8 source_code')
        values = _field_values(message, layout, 'time_offset_s').reshape(-1)
        codes = _field_values(message, layout, 'source_code').reshape(-1)
        if not np.isfinite(values).all() or np.any(values > 0) or np.any(values < -prior.policy['max_pair_offset_s']) or set(codes.tolist()) != {1, 2}:
            raise PriorError('dual arrival offsets or source codes invalid')
        offsets = np.rint(values*1e9).astype(np.int64)
        for code in (1, 2):
            if len(np.unique(offsets[codes == code])) != 1:
                raise PriorError('one arrival timestamp required for each lidar frame')
        if int(offsets.max()) != 0:
            raise PriorError('dual header must equal the latest source arrival')
    elif 'source_code' in layout['fields']:
        raise PriorError('source_code without explicit arrival offsets')
    xyz[finite] = xyz[finite] @ prior.L.T
    groups = np.unique(offsets)
    for offset in groups:
        if offset:
            old_pose = prior.pose_at(stamp+int(offset))
            prior.require_observed_cloud_time(stamp+int(offset))
            relative = np.linalg.inv(pose) @ old_pose
            selected = finite & (offsets == offset)
            xyz[selected] = xyz[selected] @ relative[:3, :3].T + relative[:3, 3]
    if not np.isfinite(xyz[finite]).all():
        raise PriorError('transformed coordinates exceed finite numerical bounds')
    result = copy.deepcopy(message)
    storage = bytearray(message.data)
    for axis, name in enumerate(('x', 'y', 'z')):
        view = _field_values(result, layout, name, storage)
        # Keep every originally invalid XYZ triplet byte-for-byte; do not invent points.
        values = view.copy().reshape(-1)
        if np.max(np.abs(xyz[finite, axis])) > np.finfo(view.dtype).max:
            raise PriorError('transformed coordinates exceed original XYZ datatype range')
        values[finite] = xyz[finite, axis]
        view[:] = values.reshape(view.shape)
    result.data = array('B', storage)
    result.header.frame_id = prior.config['reference_frame']
    report = prior.sample_report(stamp, pose)
    report.update(point_count=layout['point_count'], finite_point_count=int(finite.sum()),
                  arrival_motion_compensated=dual, source_offsets_ns=groups.tolist(),
                  compensation_basis='host-arrival approximation; not device synchronization or per-point acquisition timing')
    return result, pose, report


def enqueue_cloud(prior, queued, counters, received, message):
    """Bound pending work without shutting down continuous source collection."""
    if message.header.frame_id != prior.config['base_frame']:
        raise PriorError('input cloud base frame mismatch')
    stamp = _stamp(message.header.stamp)
    if prior.continuous:
        previous = counters.get('last_input_cloud_stamp_ns')
        if previous is not None and stamp <= previous:
            raise PriorError('non-increasing input cloud arrival timestamps')
        counters['last_input_cloud_stamp_ns'] = stamp
    if prior.initialization is None or stamp < prior.initialization['stamp_ns']+int(prior.policy['max_pair_offset_s']*1e9):
        counters['warmup_clouds_dropped'] += 1
        return
    if len(message.data) > prior.policy['max_cloud_bytes']:
        raise PriorError('pending cloud byte capacity exceeded')
    if not prior.continuous and len(queued) >= prior.policy['max_cloud_queue']:
        raise PriorError('pending cloud buffer capacity exceeded')
    if (queued and stamp <= _stamp(queued[-1][1].header.stamp)) or (
            not prior.continuous and prior.forwarded and stamp <= prior.checkpoints[-1][0]):
        raise PriorError('non-increasing input cloud arrival timestamps')
    if prior.continuous:
        while len(queued) >= prior.policy['max_cloud_queue']:
            queued.popleft()
            counters['cloud_queue_dropped'] = counters.get('cloud_queue_dropped', 0)+1
    queued.append((received, message))


def next_prepared_cloud(prior, queued, counters, now):
    """Return the next covered scan, dropping unrecoverable old work only."""
    while queued:
        received, message = queued[0]
        try:
            return prepare_cloud(message, prior)
        except DroppedCloud as error:
            queued.popleft()
            counters['uncovered_clouds_dropped'] = counters.get('uncovered_clouds_dropped', 0)+1
            counters['last_cloud_drop_reason'] = str(error)
        except NotReady:
            if not prior.continuous and now-received > prior.policy['max_cloud_wait_s']:
                raise PriorError('cloud timed out waiting for causal IMU/wheel coverage')
            return None
    return None


def _write_json(path, value):
    # Atomic reader visibility; no per-status fsync or promise of power-loss durability.
    payload = json.dumps(value, ensure_ascii=False, allow_nan=False, indent=2)
    temporary = path.with_suffix(path.suffix+'.tmp')
    temporary.write_text(payload+'\n', encoding='utf-8')
    temporary.replace(path)


class AsyncPriorJournal:
    """One disk owner; ROS callbacks only copy into a bounded memory queue.

    Initialization and guesses retain order and are never dropped. Periodic
    status is replaceable, so a stalled disk retains only its newest snapshot.
    Shutdown drains all accepted records and the final status, or reports an
    explicit error/timeout. A stuck kernel write cannot hold ROS callbacks.
    """
    def __init__(self, output, *, max_pending=128, write_json=None):
        _integer(max_pending, 'max pending journal writes')
        self.output = Path(output)
        self.max_pending = max_pending
        self.write_json = write_json or _write_json
        self.condition = threading.Condition()
        self.pending = deque()
        self.latest_status = None
        self.closing = False
        self.failure = None
        self.shutdown_error = None
        self.worker_exited = self.drain_complete = False
        self.in_flight = False
        self.completed_records = 0
        self.max_write_duration_s = 0.
        self.thread = threading.Thread(target=self._run, name='mapping-prior-journal', daemon=True)
        self.thread.start()

    def check(self):
        with self.condition:
            if self.failure or self.shutdown_error:
                raise PriorError('prior journal failed: '+(self.failure or self.shutdown_error))

    def ensure_capacity(self):
        with self.condition:
            self.check()
            if self.closing:
                raise PriorError('prior journal is closing')
            if len(self.pending) >= self.max_pending:
                raise PriorError('prior journal bounded write queue is full; no record discarded')

    def has_capacity(self):
        with self.condition:
            self.check()
            if self.closing:
                raise PriorError('prior journal is closing')
            return len(self.pending) < self.max_pending

    def _submit(self, kind, value):
        snapshot = copy.deepcopy(value)
        with self.condition:
            self.ensure_capacity()
            self.pending.append((kind, snapshot))
            self.condition.notify()

    def initialization(self, value):
        self._submit('initialization', value)

    def guess(self, value):
        self._submit('guess', value)

    def status(self, value):
        snapshot = copy.deepcopy(value)
        with self.condition:
            self.check()
            if self.closing:
                raise PriorError('prior journal is closing')
            self.latest_status = snapshot
            self.condition.notify()

    def snapshot(self):
        with self.condition:
            return {'pending_records': len(self.pending), 'pending_status': self.latest_status is not None,
                    'write_in_flight': self.in_flight, 'completed_records': self.completed_records,
                    'max_write_duration_s': self.max_write_duration_s, 'failure': self.failure or self.shutdown_error,
                    'shutdown_error': self.shutdown_error, 'worker_exited': self.worker_exited,
                    'drain_complete': self.drain_complete,
                    'storage_policy': 'bounded asynchronous writes; atomic JSON; drain/close does not certify fsync'}

    def _run(self):
        try:
            with (self.output/'guesses.jsonl').open('x', encoding='utf-8') as guesses:
                while True:
                    with self.condition:
                        while not self.pending and self.latest_status is None and not self.closing:
                            self.condition.wait()
                        if self.pending:
                            kind, value = self.pending.popleft()
                        elif self.latest_status is not None:
                            kind, value = 'status', self.latest_status
                            self.latest_status = None
                        else:
                            return
                        self.in_flight = True
                    started = time.monotonic()
                    if kind == 'guess':
                        guesses.write(json.dumps(value, allow_nan=False, separators=(',', ':'))+'\n')
                        guesses.flush()
                    else:
                        self.write_json(self.output/(kind+'.json'), value)
                    with self.condition:
                        self.in_flight = False
                        self.completed_records += 1
                        self.max_write_duration_s = max(self.max_write_duration_s, time.monotonic()-started)
        except BaseException as error:
            with self.condition:
                self.failure = str(error) or type(error).__name__
                self.in_flight = False
                self.condition.notify_all()
        finally:
            with self.condition:
                # This runs after the owner has left the guesses file context.
                # A late drain after a close timeout does not clear that error.
                self.worker_exited = True
                self.drain_complete = not self.failure and not self.pending and self.latest_status is None and not self.in_flight
                self.condition.notify_all()

    def close(self, timeout_s=20.):
        if not math.isfinite(timeout_s) or timeout_s <= 0:
            raise PriorError('journal close timeout must be finite positive')
        with self.condition:
            self.closing = True
            self.condition.notify_all()
        self.thread.join(timeout_s)
        if self.thread.is_alive():
            with self.condition:
                self.shutdown_error = self.shutdown_error or 'prior journal shutdown timed out; final persistence is incomplete'
            raise PriorError(self.shutdown_error)
        self.check()


def _owned_output(path):
    path = Path(path).absolute()
    if '..' in path.parts or any(p.is_symlink() or (hasattr(p, 'is_junction') and p.is_junction()) for p in (path, *path.parents)):
        raise PriorError('prior output must not traverse symlinks, junctions or parent components')
    path.mkdir(parents=True, exist_ok=False)
    return path


def _wait_for_config(path, timeout_seconds):
    """Stop cleanly even before ROS and the owned output directory exist."""
    stopped = [False]
    previous = {}
    try:
        for signum in (signal.SIGINT, signal.SIGTERM):
            previous[signum] = signal.signal(signum, lambda *_: stopped.__setitem__(0, True))
        deadline = None if timeout_seconds == 0 else time.monotonic()+timeout_seconds
        while not stopped[0]:
            if path.exists():
                return True
            if deadline is not None and time.monotonic() >= deadline:
                raise PriorError('timed out waiting for explicit bootstrap prior configuration')
            time.sleep(.05)
        return False
    except KeyboardInterrupt:
        return False
    finally:
        for signum, handler in previous.items():
            signal.signal(signum, handler)


def _parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', required=True)
    parser.add_argument('--session-id', required=True)
    parser.add_argument('--output-root', required=True)
    parser.add_argument('--wait-config-seconds', type=float, default=15., help='0 waits until config exists or the task is stopped')
    parser.add_argument('--close-timeout-s', type=persistence_wait, default=None,
                        help='Override only stop-time journal wait in (0,90] seconds; config policy is the default')
    args = parser.parse_args(argv)
    if not math.isfinite(args.wait_config_seconds) or not 0 <= args.wait_config_seconds <= 120:
        parser.error('--wait-config-seconds must be in [0,120]')
    return args


def _apply_persistence_wait(prior, override):
    """Keep the archived configuration intact; expose an explicit CLI override."""
    if override is not None:
        prior.policy = dict(prior.policy, writer_close_timeout_s=persistence_wait(override))
    return {'persistence_close_timeout_s': prior.policy['writer_close_timeout_s'],
            'persistence_close_timeout_source': 'config_policy' if override is None else 'command_line'}


def main(argv=None):
    args = _parse_args(argv)
    config_path = Path(args.config)
    if not _wait_for_config(config_path, args.wait_config_seconds):
        return 0
    if config_path.is_symlink():
        raise PriorError('bootstrap configuration must not be a symlink')
    prior = MotionPrior(json.loads(config_path.read_text(encoding='utf-8-sig')), args.session_id)
    output = _owned_output(args.output_root)
    _write_json(output/'config.json', prior.config)
    close_policy = _apply_persistence_wait(prior, args.close_timeout_s)
    # ROS imports are confined to the executable adapter; pure offline tests need no ROS.
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from rclpy.signals import SignalHandlerOptions
    from sensor_msgs.msg import PointCloud2
    from std_msgs.msg import String
    from wc_interfaces.msg import H30Frame
    from geometry_msgs.msg import TransformStamped
    from tf2_msgs.msg import TFMessage
    from nav_msgs.msg import Odometry

    stop = [False]
    old_signals = {}
    for signum in (signal.SIGINT, signal.SIGTERM):
        old_signals[signum] = signal.signal(signum, lambda *_: stop.__setitem__(0, True))
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    started = time.monotonic()
    prior.enable_startup_recovery(started)
    last_receipt = {'imu': None, 'wheel': None, 'cloud': None}
    queued = deque()
    counters = {'input_clouds': 0, 'warmup_clouds_dropped': 0, 'forwarded_clouds': 0}
    failure = None
    node = None
    executor = None
    callback_counts = {'imu': 0, 'wheel': 0, 'cloud': 0}
    adapter_timings = {}
    drain_report = None
    drain_budget_exhaustions = 0
    journal = AsyncPriorJournal(output, max_pending=prior.policy['max_pending_writes'])
    status = {'schema_version': 1, 'session_id': args.session_id, 'status': 'STARTING',
              'source_mode': 'real', 'experiment': True, 'time_source': 'arrival_only',
              'continuous_mapping': prior.continuous,
              'odometry_source': prior.config['odometry_source'],
              'wheel_imu_covariance': prior.config.get('wheel_imu_covariance'),
              'time_valid': False, 'formal_odometry_eligible': False, 'navigation_enabled': False,
              **close_policy}
    reliable = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)
    sensor_qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE)
    try:
        journal.status(status)
        node = Node('wc_mapping_motion_prior')
        executor = SingleThreadedExecutor()
        executor.add_node(node)  # Preserve one executor membership for the session.
        tf_pub = node.create_publisher(TFMessage, prior.config['private_tf_topic'], reliable)
        odom_pub = node.create_publisher(Odometry, prior.config['prior_odom_topic'], reliable)
        authority_pub = (node.create_publisher(Odometry, AUTHORITY_TOPIC, reliable)
                         if prior.config['odometry_source'] == 'wheel_imu' else None)
        cloud_pub = node.create_publisher(PointCloud2, prior.config['output_cloud_topic'], reliable)

        def imu_callback(msg):
            if not msg.angular_velocity_valid or not msg.linear_acceleration_valid or msg.uncertainty_valid:
                raise PriorError('H30 requires valid SI gyro/acceleration and honest unknown time uncertainty')
            if msg.imu.header.frame_id != 'imu_h30_native' or msg.header.frame_id != 'imu_h30_native':
                raise PriorError('unexpected native H30 frame id')
            a, g = msg.imu.linear_acceleration, msg.imu.angular_velocity
            prior.add_imu(stamp_ns=_stamp(msg.host_receive_time), monotonic_ns=int(msg.host_monotonic_ns),
                          acceleration=[a.x, a.y, a.z], angular_velocity=[g.x, g.y, g.z],
                          sensor_id=msg.sensor_id, session_id=msg.session_id, stream_epoch=msg.stream_epoch,
                          sequence=int(msg.frame_sequence), coordinate_convention=msg.coordinate_convention,
                          time_source=msg.time_source, common_time_valid=msg.common_time_valid)
            last_receipt['imu'] = time.monotonic()

        def wheel_callback(msg):
            prior.add_wheel(json.loads(msg.data))
            last_receipt['wheel'] = time.monotonic()

        def cloud_callback(msg):
            counters['input_clouds'] += 1
            last_receipt['cloud'] = time.monotonic()
            enqueue_cloud(prior, queued, counters, time.monotonic(), msg)

        def observed(name, callback):
            def receive(msg):
                begin = time.monotonic()
                try:
                    return callback(msg)
                finally:
                    callback_counts[name] += 1
                    _record_timing(adapter_timings, name+'_callback', time.monotonic()-begin)
            return receive

        subscriptions = [node.create_subscription(H30Frame, prior.config['imu_topic'], observed('imu', imu_callback), sensor_qos),
                         node.create_subscription(String, prior.config['wheel_topic'], observed('wheel', wheel_callback), reliable),
                         node.create_subscription(PointCloud2, prior.config['input_cloud_topic'], observed('cloud', cloud_callback), reliable)]
        last_status = 0.
        init_written = False
        while not stop[0]:
            drain_report = drain_ready_callbacks(executor.spin_once, lambda: sum(callback_counts.values()))
            drain_budget_exhaustions += int(drain_report['ready_budget_exhausted'])
            _record_timing(adapter_timings, 'executor_cycle', drain_report['total_s'])
            if stop[0]:
                break  # Producers may be stopping too; do not misclassify an intentional close.
            journal.check()
            if prior.initialization is not None and not init_written:
                journal.initialization(prior.initialization)
                init_written = True
            now = time.monotonic()
            check_source_freshness(prior, last_receipt, now)
            if not prior.continuous and now-started > prior.policy['startup_timeout_s'] and (not prior.initialization or not prior.forwarded):
                raise PriorError('startup timeout: '+prior.static_wait_reason+'; no initialized forwarded scan')
            while queued:
                if prior.continuous and not journal.has_capacity():
                    break  # Bounded backpressure; source callbacks and queue retirement continue.
                prepare_started = time.monotonic()
                try:
                    prepared = next_prepared_cloud(prior, queued, counters, now)
                finally:
                    _record_timing(adapter_timings, 'prepare_cloud', time.monotonic()-prepare_started)
                if prepared is None:
                    break
                transformed, pose, report = prepared
                _, msg = queued[0]
                journal.ensure_capacity()
                stamp = report['stamp_ns']
                transform = TransformStamped()
                transform.header.stamp = msg.header.stamp
                transform.header.frame_id = prior.config['guess_frame_id']
                transform.child_frame_id = prior.config['reference_frame']
                p, q = pose[:3, 3], Rotation.from_matrix(pose[:3, :3]).as_quat()
                transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z = map(float, p)
                transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w = map(float, q)
                odom = Odometry()
                odom.header = transform.header; odom.child_frame_id = transform.child_frame_id
                odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z = map(float, p)
                odom.pose.pose.orientation = transform.transform.rotation
                v, w = report['linear_velocity_reference_m_s'], report['angular_velocity_reference_rad_s']
                odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.linear.z = v
                odom.twist.twist.angular.x, odom.twist.twist.angular.y, odom.twist.twist.angular.z = w
                for index in (0, 7, 14, 21, 28, 35):
                    odom.pose.covariance[index] = odom.twist.covariance[index] = 1e6
                if prior.planar:
                    odom.pose.covariance = report['pose_covariance']
                    odom.twist.covariance = report['twist_covariance']
                authoritative = None
                transforms = [transform]
                if authority_pub is not None:
                    authoritative = adapt_motion_odometry(odom, prior.config)
                    transforms.insert(0, authority_identity_transform(odom, transform_type=TransformStamped))
                tf_pub.publish(TFMessage(transforms=transforms))
                odom_pub.publish(odom)
                if authoritative is not None:
                    authority_pub.publish(authoritative)
                cloud_pub.publish(transformed)
                prior.record_forwarded(stamp, pose)
                journal.guess(report)
                counters['forwarded_clouds'] += 1
                queued.popleft()
            now = time.monotonic()
            if now-last_status >= .5:
                status.update(status='RUNNING' if prior.forwarded else ('WAITING_FOR_SOURCES' if prior.continuous else 'WAITING_FOR_STATIC'),
                              counts=counters, queue_depth=len(queued), static_wait_reason=prior.static_wait_reason,
                              initialized=prior.initialization is not None,
                              motion_model=prior.config['motion_model'],
                              journal=journal.snapshot(),
                              callback_counts=callback_counts, adapter_timings=adapter_timings,
                              initialization_timings=prior.initialization_timings,
                              startup_candidate=prior.startup_report(),
                              gyro_bias=prior.bias_report(),
                              executor='persistent SingleThreadedExecutor', executor_drain=drain_report,
                              executor_drain_budget_exhaustions=drain_budget_exhaustions,
                              **prior.coverage_report(),
                              **source_ages(prior, last_receipt, now))
                journal.status(status)
                last_status = now
    except Exception as error:
        failure = str(error)
        if node is not None:
            node.get_logger().error(failure)
    finally:
        status.update(status='FAILED' if failure else 'STOPPED', failure=failure, counts=counters,
                      motion_model=prior.config['motion_model'],
                      initialized=prior.initialization is not None, journal=journal.snapshot(),
                      static_wait_reason=prior.static_wait_reason,
                      callback_counts=callback_counts, adapter_timings=adapter_timings,
                      initialization_timings=prior.initialization_timings,
                      startup_candidate=prior.startup_report(),
                      gyro_bias=prior.bias_report(),
                      executor='persistent SingleThreadedExecutor', executor_drain=drain_report,
                      executor_drain_budget_exhaustions=drain_budget_exhaustions,
                      **prior.coverage_report(),
                      **source_ages(prior, last_receipt, time.monotonic()))
        try:
            journal.status(status)
        except PriorError as error:
            failure = failure or str(error)
        try:
            journal.close(prior.policy['writer_close_timeout_s'])
        except PriorError as error:
            failure = failure or str(error)
            if node is not None:
                node.get_logger().error('Journal close failed: '+str(error))
        if executor is not None:
            if node is not None:
                executor.remove_node(node)
            executor.shutdown(timeout_sec=1.)
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for signum, handler in old_signals.items():
            signal.signal(signum, handler)
    return 1 if failure else 0


if __name__ == '__main__':
    raise SystemExit(main())
