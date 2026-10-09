"""Numerical front end; production ICP is a separate RTAB-Map adapter.

Arrays use metres. Integer nanoseconds remain integers until a time difference
has been formed. No guessed motion, physical extrinsics, or clock uncertainty.
"""
from collections import Counter, deque
from dataclasses import dataclass
import math
import uuid

import numpy as np
from scipy.spatial.transform import Rotation


class FusionError(ValueError):
    pass


def matrix(value):
    t = np.asarray(value, dtype=np.float64)
    if t.shape != (4, 4) or not np.isfinite(t).all():
        raise FusionError('INVALID_TRANSFORM')
    if not np.allclose(t[3], [0, 0, 0, 1], atol=1e-9):
        raise FusionError('INVALID_HOMOGENEOUS_ROW')
    if not np.allclose(t[:3, :3].T @ t[:3, :3], np.eye(3), atol=1e-7):
        raise FusionError('ROTATION_NOT_ORTHONORMAL')
    if not np.isclose(np.linalg.det(t[:3, :3]), 1, atol=1e-7):
        raise FusionError('ROTATION_REFLECTION')
    return t.copy()


def transform(t, points):
    t = matrix(t)
    p = np.asarray(points, dtype=np.float64)
    if p.ndim != 2 or p.shape[1] != 3:
        raise FusionError('INVALID_POINTS_SHAPE')
    return p @ t[:3, :3].T + t[:3, 3]


def integer_ns(value):
    if isinstance(value, bool) or not isinstance(value, (int, np.integer)):
        raise FusionError('TIME_MUST_BE_INTEGER_NS')
    return int(value)


@dataclass(frozen=True)
class Policy:
    max_pair_ns: int = 20_000_000
    max_queue: int = 8
    max_age_ns: int = 300_000_000
    min_points_per_source: int = 20
    min_range_m: float = 0.1
    max_range_m: float = 10.0
    velocity_bound_mps: float = 0.2
    angular_bound_radps: float = 0.2
    max_time_error_m: float = 0.02
    prediction_error_bound_m: float = 0.003
    max_prediction_ns: int = 150_000_000
    max_motion_speed_mps: float = 0.5
    max_motion_angular_radps: float = 0.5

    def __post_init__(self):
        for name, value in vars(self).items():
            if not math.isfinite(value) or value <= 0:
                raise FusionError('INVALID_POLICY:' + name)
        for name in ('max_pair_ns', 'max_queue', 'max_age_ns',
                     'min_points_per_source', 'max_prediction_ns'):
            integer_ns(getattr(self, name))
        if self.min_range_m >= self.max_range_m:
            raise FusionError('INVALID_RANGE')


class MotionHistory:
    """Bounded constant velocity prediction from completed poses, never current ICP."""
    def __init__(self, policy):
        self.policy = policy
        self.poses = deque(maxlen=3)
        self.epoch = None

    def add(self, stamp_ns, pose, epoch):
        stamp_ns = integer_ns(stamp_ns)
        pose = matrix(pose)
        if self.epoch is not None and epoch != self.epoch:
            self.poses.clear()
            self.epoch = epoch
            raise FusionError('ODOM_EPOCH_CHANGED')
        self.epoch = epoch
        if self.poses and stamp_ns <= self.poses[-1][0]:
            raise FusionError('ODOM_TIME_NOT_INCREASING')
        self.poses.append((stamp_ns, pose))

    def predict(self, stamp_ns):
        stamp_ns = integer_ns(stamp_ns)
        if len(self.poses) < 2:
            raise FusionError('PREDICTION_NEEDS_COMPLETED_HISTORY')
        (t0, p0), (t1, p1) = list(self.poses)[-2:]
        if stamp_ns < t0 or stamp_ns - t1 > self.policy.max_prediction_ns:
            raise FusionError('PREDICTION_EXPIRED')
        dt = (t1 - t0) * 1e-9
        velocity = (p1[:3, 3] - p0[:3, 3]) / dt
        rotation_delta = Rotation.from_matrix(p0[:3, :3].T @ p1[:3, :3]).as_rotvec()
        if np.linalg.norm(velocity) > min(self.policy.max_motion_speed_mps, self.policy.velocity_bound_mps):
            raise FusionError('PREDICTION_SPEED_EXCEEDED')
        if np.linalg.norm(rotation_delta) / dt > min(self.policy.max_motion_angular_radps, self.policy.angular_bound_radps):
            raise FusionError('PREDICTION_ROTATION_EXCEEDED')
        ratio = (stamp_ns - t1) / (t1 - t0)
        result = p1.copy()
        result[:3, 3] += velocity * ((stamp_ns - t1) * 1e-9)
        result[:3, :3] = p1[:3, :3] @ Rotation.from_rotvec(rotation_delta * ratio).as_matrix()
        return result


class DualFusion:
    """Two bounded sorted queues, exactly-once frames, source-preserving samples.

The left reference identity is definitional. T_rig_right must be supplied and
validated independently. max_time_error_m and motion bounds are experimental
quality settings; live requires a validated profile ID and evidence.
"""
    def __init__(self, session_id, sensor_ids, calibration, clock_model_ids,
                 policy=None, source_mode='real', quality_profile=None):
        self.policy = policy or Policy()
        if set(sensor_ids) != {'left', 'right'} or len(set(sensor_ids.values())) != 2:
            raise FusionError('DUPLICATE_OR_MISSING_SENSOR_ID')
        if set(clock_model_ids) != {'left', 'right'}:
            raise FusionError('MISSING_CLOCK_MODELS')
        if source_mode not in ('real', 'synthetic'):
            raise FusionError('INVALID_SOURCE_MODE')
        if calibration.get('sensor_ids') != sensor_ids:
            raise FusionError('CALIBRATION_IDENTITY_MISMATCH')
        if calibration.get('T_left_right') is None:
            raise FusionError('CALIBRATION_REQUIRED')
        if source_mode == 'real':
            if calibration.get('status') != 'VALIDATED' or calibration.get('source_mode') != 'real':
                raise FusionError('LIVE_CALIBRATION_UNVALIDATED')
            if not quality_profile or quality_profile.get('status') != 'VALIDATED' or not quality_profile.get('evidence'):
                raise FusionError('LIVE_QUALITY_PROFILE_UNVALIDATED')
        self.session_id = str(session_id)
        self.sensor_ids = dict(sensor_ids)
        self.clock_model_ids = dict(clock_model_ids)
        self.calibration_id = calibration['calibration_id']
        self.source_mode = source_mode
        self.extrinsics = {'left': np.eye(4), 'right': matrix(calibration['T_left_right'])}
        self.queues = {'left': [], 'right': []}
        self.epochs = {}
        self.seen = set()
        self.seen_order = deque()
        self.sequence_highwater = {}
        self.sequence_stamp_highwater = {}
        self.last_seen_monotonic_ns = {}
        self.started_monotonic_ns = None
        self.last_bundle_monotonic_ns = None
        self.counts = Counter()
        self.bundle_counter = 0
        self.latched_reason = None
        self.history = MotionHistory(self.policy)

    def latch(self, reason):
        self.latched_reason = reason
        self.queues['left'].clear()
        self.queues['right'].clear()
        self.counts[reason] += 1

    def check_freshness(self, now_monotonic_ns):
        now = integer_ns(now_monotonic_ns)
        if self.latched_reason:
            return False
        if self.started_monotonic_ns is None:
            return True
        for side in ('left', 'right'):
            age = now - self.last_seen_monotonic_ns.get(side, self.started_monotonic_ns)
            if age < 0 or age > self.policy.max_age_ns:
                self.latch('SOURCE_TIMEOUT:' + side)
                return False
        last = self.last_bundle_monotonic_ns
        if now - (last if last is not None else self.started_monotonic_ns) > self.policy.max_age_ns:
            self.latch('PAIR_STREAM_TIMEOUT')
            return False
        return True

    def resume(self, *, explicit, stable, continuity_verified, calibration_id, clock_model_ids):
        if not (explicit and stable and continuity_verified):
            raise FusionError('RESUME_PRECONDITIONS_UNMET')
        if calibration_id != self.calibration_id or clock_model_ids != self.clock_model_ids:
            raise FusionError('RESUME_CONFIG_MISMATCH')
        self.latched_reason = None
        self.queues['left'].clear()
        self.queues['right'].clear()
        self.last_seen_monotonic_ns.clear()
        self.started_monotonic_ns = None
        self.last_bundle_monotonic_ns = None

    def _check(self, frame, now_monotonic_ns):
        side = frame.get('side')
        if side not in self.sensor_ids or frame['sensor_id'] != self.sensor_ids[side]:
            raise FusionError('SOURCE_IDENTITY_MISMATCH')
        if frame['session_id'] != self.session_id:
            raise FusionError('SOURCE_SESSION_MISMATCH')
        if frame['clock_model_id'] != self.clock_model_ids[side]:
            raise FusionError('CLOCK_MODEL_MISMATCH')
        if not frame['time_valid'] or frame['uncertainty_ns'] is None:
            raise FusionError('TIME_UNVALIDATED')
        if self.source_mode == 'real' and frame['time_source'] != 'validated_model':
            raise FusionError('LIVE_TIME_SOURCE_UNVALIDATED')
        stamp = integer_ns(frame['common_time_ns'])
        uncertainty = integer_ns(frame['uncertainty_ns'])
        if uncertainty < 0:
            raise FusionError('NEGATIVE_UNCERTAINTY')
        age = integer_ns(now_monotonic_ns) - integer_ns(frame['host_monotonic_ns'])
        if age < 0 or age > self.policy.max_age_ns:
            raise FusionError('SOURCE_STALE_OR_MONOTONIC_DOMAIN_MISMATCH')
        if not frame.get('stream_epoch'):
            raise FusionError('MISSING_STREAM_EPOCH')
        epoch = frame['stream_epoch']
        if side in self.epochs and epoch != self.epochs[side]:
            self.epochs[side] = epoch
            raise FusionError('STREAM_EPOCH_CHANGED')
        self.epochs[side] = epoch
        seq = integer_ns(frame['frame_sequence'])
        if seq < 0:
            raise FusionError('NEGATIVE_SEQUENCE')
        key = f"{frame['sensor_id']}/{epoch}/{seq}"
        stream = (side, epoch)
        high = self.sequence_stamp_highwater.get(stream)
        if high is not None and seq > high[0] and stamp <= high[1]:
            raise FusionError('SOURCE_TIME_JUMP')
        # Permit bounded reordering but never old frames beyond the history window.
        if key in self.seen or seq <= self.sequence_highwater.get(stream, -1) - self.policy.max_queue * 4:
            raise FusionError('DUPLICATE_OR_TOO_OLD_FRAME')
        self.sequence_highwater[stream] = max(seq, self.sequence_highwater.get(stream, -1))
        if high is None or seq > high[0]:
            self.sequence_stamp_highwater[stream] = (seq, stamp)
        self.seen.add(key)
        self.seen_order.append(key)
        while len(self.seen_order) > self.policy.max_queue * 16:
            self.seen.discard(self.seen_order.popleft())
        points = np.array(frame['points'], dtype=np.float64, copy=True)
        if points.ndim != 2 or points.shape[1] != 3:
            raise FusionError('INVALID_POINTS_SHAPE')
        ranges = np.linalg.norm(points, axis=1)
        valid = np.isfinite(points).all(axis=1) & (ranges >= self.policy.min_range_m) & (ranges <= self.policy.max_range_m)
        kept = points[valid]
        if len(kept) < self.policy.min_points_per_source:
            raise FusionError('SOURCE_TOO_FEW_VALID_POINTS')
        # A host-filtered cloud can move/remove/add points. Preserve independent
        # measured endpoints for occupancy; never raycast smoothing/fill output.
        if (frame.get('representation') == 'host_filtered' or frame.get('filter_config_hash')) and 'original_points' not in frame:
            raise FusionError('FILTERED_INPUT_REQUIRES_ORIGINAL_RAYS')
        original = np.array(frame.get('original_points', points), dtype=np.float64, copy=True)
        if original.ndim != 2 or original.shape[1] != 3:
            raise FusionError('INVALID_ORIGINAL_POINTS_SHAPE')
        original_range = np.linalg.norm(original, axis=1)
        original_valid = (np.isfinite(original).all(axis=1) &
                          (original_range >= self.policy.min_range_m) &
                          (original_range <= self.policy.max_range_m))
        if np.count_nonzero(original_valid) < self.policy.min_points_per_source:
            raise FusionError('SOURCE_TOO_FEW_ORIGINAL_RETURNS')
        result = dict(frame)
        result.update(points=kept, raw_points=points, raw_key=key, common_time_ns=stamp,
                      raw_count=len(points), valid_count=len(kept), valid_indices=np.flatnonzero(valid))
        result.update(original_points=original, ray_points=original[original_valid],
                      ray_indices=np.flatnonzero(original_valid))
        return side, result

    def push(self, frame, now_monotonic_ns):
        if self.latched_reason:
            raise FusionError('PAUSED_INVALID:' + self.latched_reason)
        try:
            side, frame = self._check(frame, now_monotonic_ns)
        except FusionError as exc:
            if str(exc) == 'DUPLICATE_OR_TOO_OLD_FRAME':
                self.counts[str(exc)] += 1
            else:
                self.latch(str(exc))
            raise
        if self.started_monotonic_ns is None:
            self.started_monotonic_ns = integer_ns(now_monotonic_ns)
        # _check permits unseen frames within the bounded reorder window. A
        # delayed older frame must not erase evidence of a newer valid arrival.
        # Keep source acquisition time, not callback time; per-frame age checks
        # above and the existing source/pair deadlines remain unchanged.
        self.last_seen_monotonic_ns[side] = max(
            frame['host_monotonic_ns'],
            self.last_seen_monotonic_ns.get(side, frame['host_monotonic_ns']))
        if not self.check_freshness(now_monotonic_ns):
            raise FusionError(self.latched_reason)
        q = self.queues[side]
        q.append(frame)
        q.sort(key=lambda f: (f['common_time_ns'], f['frame_sequence']))
        while len(q) > self.policy.max_queue:
            q.pop(0)
            self.counts['QUEUE_OVERFLOW'] += 1
        result = self._pair(now_monotonic_ns)
        if result is not None:
            self.last_bundle_monotonic_ns = integer_ns(now_monotonic_ns)
        return result

    def _pair(self, now_monotonic_ns):
        left, right = self.queues['left'], self.queues['right']
        while left and right:
            l, r = left[0], right[0]
            if now_monotonic_ns - min(l['host_monotonic_ns'], r['host_monotonic_ns']) > self.policy.max_age_ns:
                self.latch('PAIR_STALE')
                raise FusionError('PAIR_STALE')
            dt = l['common_time_ns'] - r['common_time_ns']
            if abs(dt) > self.policy.max_pair_ns:
                (left if dt < 0 else right).pop(0)
                self.counts['PAIR_WINDOW_EXCEEDED'] += 1
                continue
            left.pop(0)
            right.pop(0)
            try:
                return self._fuse(l, r)
            except FusionError as exc:
                self.latch(str(exc))
                raise
        return None

    def _fuse(self, left, right):
        t_ref = max(left['common_time_ns'], right['common_time_ns'])
        dt_ns = abs(left['common_time_ns'] - right['common_time_ns'])
        uncertainty_ns = left['uncertainty_ns'] + right['uncertainty_ns']
        bound_per_s = self.policy.velocity_bound_mps + self.policy.max_range_m * self.policy.angular_bound_radps
        static_error = bound_per_s * ((dt_ns + uncertainty_ns) * 1e-9)
        transforms = {}
        compensation = 'bounded_static'
        error = static_error
        if static_error > self.policy.max_time_error_m or getattr(self.history, 'requires_prediction', False):
            # A past estimate supplies prediction. A missing predictor is a real
            # rejection; no fabricated zero-motion fallback.
            reference = self.history.predict(t_ref)
            for side, f in (('left', left), ('right', right)):
                transforms[side] = np.linalg.inv(reference) @ self.history.predict(f['common_time_ns'])
            error = bound_per_s * (uncertainty_ns * 1e-9) + self.policy.prediction_error_bound_m
            if error > self.policy.max_time_error_m:
                raise FusionError('TIME_ERROR_BUDGET_EXCEEDED')
            compensation = getattr(self.history, 'mode', 'history_constant_velocity')
        else:
            transforms = {'left': np.eye(4), 'right': np.eye(4)}
        chunks, observations, source_codes = [], [], []
        for code, side, f in ((1, 'left', left), (2, 'right', right)):
            total = transforms[side] @ self.extrinsics[side]
            chunks.append(transform(total, f['points']))
            source_codes.append(np.full(len(f['points']), code, dtype=np.uint8))
            observations.append(dict(raw_key=f['raw_key'], sensor_id=f['sensor_id'], side=side,
                                     stamp_ns=f['common_time_ns'], points=f['ray_points'].copy(),
                                     raw_points=f['original_points'].copy(), T_ref_sensor=total,
                                     T_ref_rig_at_source=transforms[side], valid_indices=f['ray_indices'],
                                     origin_in_ref=total[:3, 3].copy(), raw_count=len(f['original_points']),
                                     valid_count=len(f['ray_points']), solver_input_count=f['valid_count'],
                                     source_config_hash=f.get('original_config_hash', f.get('source_config_hash')),
                                     filter_config_hash=f.get('filter_config_hash'),
                                     uncertainty_ns=f['uncertainty_ns']))
        self.bundle_counter += 1
        self.counts['BUNDLE_DUAL'] += 1
        return dict(session_id=self.session_id, bundle_id=self.bundle_counter,
                    t_ref_ns=t_ref, source_mode=self.source_mode, sensor_mode='dual',
                    calibration_id=self.calibration_id, clock_model_ids=self.clock_model_ids.copy(),
                    frame_id='rig_link', compensation_mode=compensation,
                    temporal_error_bound_m=error, points=np.concatenate(chunks),
                    source_codes=np.concatenate(source_codes), observations=observations,
                    source_counts={'left': len(chunks[0]), 'right': len(chunks[1])})


class Gate:
    """Join post-ICP results by session, bundle and exact measurement time.

The accepted output is the only graph input. Raw source recording occurs before
this object and continues on failures. Metrics absent from an ICP API stay None.
"""
    def __init__(self, session_id, max_pending=8, timeout_ns=500_000_000,
                 max_speed_mps=0.5, max_angular_radps=0.5):
        self.session_id = session_id
        self.max_pending = max_pending
        self.timeout_ns = timeout_ns
        self.pending = {}
        self.paused_reason = None
        self.odom_epoch = None
        self.last_stamp_ns = None
        self.accepted = 0
        self.last_pose = None
        self.last_result_monotonic_ns = None
        self.max_speed_mps = max_speed_mps
        self.max_angular_radps = max_angular_radps
        if not all(math.isfinite(x) and x > 0 for x in (max_speed_mps, max_angular_radps)):
            raise FusionError('INVALID_TRACKING_LIMITS')

    def pause(self, reason):
        self.paused_reason = reason
        self.pending.clear()

    def submit(self, bundle, now_monotonic_ns):
        if self.paused_reason:
            raise FusionError('GATE_PAUSED:' + self.paused_reason)
        if bundle['session_id'] != self.session_id or bundle['sensor_mode'] != 'dual':
            self.pause('BUNDLE_SESSION_OR_MODE_MISMATCH')
            raise FusionError(self.paused_reason)
        if not all(bundle['source_counts'].get(side, 0) > 0 for side in ('left', 'right')):
            self.pause('DUAL_PARTICIPATION_MISSING')
            raise FusionError(self.paused_reason)
        key = bundle['bundle_id']
        if key in self.pending or len(self.pending) >= self.max_pending:
            self.pause('DUPLICATE_BUNDLE_OR_BACKLOG')
            raise FusionError(self.paused_reason)
        self.pending[key] = (bundle, integer_ns(now_monotonic_ns))

    def assess(self, result, now_monotonic_ns):
        if self.paused_reason:
            raise FusionError('GATE_PAUSED:' + self.paused_reason)
        key = result['bundle_id']
        pending = self.pending.get(key)
        if not pending:
            self.pause('UNKNOWN_BUNDLE')
            raise FusionError(self.paused_reason)
        bundle, received = pending
        reason = None
        if result['session_id'] != self.session_id or result['stamp_ns'] != bundle['t_ref_ns']:
            reason = 'ASSESSMENT_ID_TIME_MISMATCH'
        elif now_monotonic_ns - received < 0 or now_monotonic_ns - received > self.timeout_ns:
            reason = 'ICP_RESULT_TIMEOUT'
        elif not result['tracking_valid'] or result.get('T_odom_rig') is None:
            reason = 'TRACKING_FAILED'
        elif result.get('actual_source_participation') != bundle['source_counts']:
            reason = 'ICP_INPUT_SOURCE_MISMATCH'
        elif self.odom_epoch is not None and result['odom_epoch'] != self.odom_epoch:
            reason = 'ODOM_EPOCH_CHANGED'
        elif self.last_stamp_ns is not None and result['stamp_ns'] <= self.last_stamp_ns:
            reason = 'ODOM_TIME_NOT_INCREASING'
        if reason:
            self.pause(reason)
            raise FusionError(reason)
        try:
            pose = matrix(result['T_odom_rig'])
        except FusionError:
            self.pause('INVALID_ODOM_POSE')
            raise
        if self.last_pose is not None:
            dt = (result['stamp_ns'] - self.last_stamp_ns) * 1e-9
            translation = np.linalg.norm(pose[:3, 3] - self.last_pose[:3, 3])
            angle = np.linalg.norm(Rotation.from_matrix(self.last_pose[:3, :3].T @ pose[:3, :3]).as_rotvec())
            if translation / dt > self.max_speed_mps or angle / dt > self.max_angular_radps:
                self.pause('ODOM_MOTION_JUMP')
                raise FusionError('ODOM_MOTION_JUMP')
        self.pending.pop(key)
        self.odom_epoch = result['odom_epoch']
        self.last_stamp_ns = result['stamp_ns']
        self.last_pose = pose
        self.last_result_monotonic_ns = integer_ns(now_monotonic_ns)
        self.accepted += 1
        return dict(bundle=bundle, T_odom_rig=pose, assessment=dict(result))

    def expire(self, now_monotonic_ns):
        if any(now_monotonic_ns - received > self.timeout_ns for _, received in self.pending.values()):
            self.pause('ICP_RESULT_TIMEOUT')
            return False
        if self.last_result_monotonic_ns is not None and now_monotonic_ns - self.last_result_monotonic_ns > self.timeout_ns:
            self.pause('TRACKING_STREAM_TIMEOUT')
            return False
        return not self.paused_reason

    def resume(self, *, explicit, stable, continuity_verified, odom_epoch):
        if not explicit or not stable or not continuity_verified or odom_epoch != self.odom_epoch:
            raise FusionError('RESUME_PRECONDITIONS_UNMET')
        self.pending.clear()
        self.paused_reason = None
        # New in-flight bundles get their own timeout. Keeping the old receive
        # time would immediately relatch after an intentional long pause.
        self.last_result_monotonic_ns = None
