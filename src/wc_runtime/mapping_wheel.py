"""Single wheel serial owner for mapping and directly user-operated WASD.

Unauthorised startup and exit read FC03 feedback only. The persistently authorised
hybrid_manual policy enters hand-push only with fresh foreground UI intent and
continuous zero feedback; WASD takes over and normal key release returns to push.
Focus loss, timeout, disconnect and fault never request release. This adapter is not verified
firmware/watchdog evidence. No autonomous input, reconnect or nonzero retry.
"""
import argparse
from collections import deque
import copy
import hashlib
import json
import math
import os
from pathlib import Path
import re
import select
import signal
import socket
import stat
import struct
import time
import uuid

from wc_motion.protocol import FeedbackError, checked_frame, crc16, parse_exchange, read_request


KEY_TIMEOUT_S = .25
FEEDBACK_PERIOD_S = .1
CONTROL_PERIOD_S = .05
FEEDBACK_FRESH_S = .25
DISABLED_REASON = '尚未确认控制器 USB/RS485 物理通信中断时的停车行为；本次只读反馈'
CONTROL_CONTEXT = 'user_manual_mapping'
QUERY = read_request(1, 0x20AB, 2)
MEMORY_CAPTURE_ROOT = Path('/dev/shm/wc_capture')


def bounded(value, name, low, high):
    if type(value) not in (int, float) or not math.isfinite(value) or not low <= value <= high:
        raise FeedbackError(name + ' must be finite in [' + str(low) + ', ' + str(high) + ']')
    return value


class QueryFailure(FeedbackError):
    def __init__(self, reason, response=b'', transmitted_bytes=0):
        super().__init__(reason)
        self.response, self.transmitted_bytes = bytes(response), transmitted_bytes


def single_write(address, value):
    payload = struct.pack('>BBHH', 1, 6, address, value)
    return payload + crc16(payload)


def speed_write(left, right):
    payload = struct.pack('>BBHHBhh', 1, 0x10, 0x2088, 2, 4, left, right)
    return payload + crc16(payload)


ZERO_REQUEST = speed_write(0, 0)
RELEASE_REQUEST = single_write(0x200E, 7)
INITIALIZATION_WRITES = (single_write(0x200D, 3), single_write(0x200F, 0),
                         single_write(0x200E, 6), single_write(0x200E, 8))


def validate_manual_controls(value=None):
    value = {} if value is None else copy.deepcopy(value)
    if not isinstance(value, dict):
        raise FeedbackError('manual_controls must be an object')
    allowed = {'arm_allowed', 'reason', 'max_linear_m_s', 'max_angular_rad_s',
               'command_rpm_to_register_scale', 'operation_evidence', 'interaction_policy', 'note', '_help'}
    if set(value) - allowed:
        raise FeedbackError('unknown manual_controls fields')
    value.setdefault('arm_allowed', False)
    value.setdefault('reason', DISABLED_REASON)
    value.setdefault('max_linear_m_s', .10)
    value.setdefault('max_angular_rad_s', .20)
    value.setdefault('command_rpm_to_register_scale', 8.9)
    value.setdefault('interaction_policy', 'explicit_push')
    if value['interaction_policy'] not in ('explicit_push', 'hybrid_manual'):
        raise FeedbackError('interaction_policy must be explicit_push or hybrid_manual')
    if type(value['arm_allowed']) is not bool or not isinstance(value['reason'], str):
        raise FeedbackError('explicit manual arm flag and reason required')
    bounded(value['max_linear_m_s'], 'manual linear limit', .001, .15)
    bounded(value['max_angular_rad_s'], 'manual angular limit', .001, .3)
    bounded(value['command_rpm_to_register_scale'], 'legacy command register/rpm scale', .01, 100)
    if value['arm_allowed'] and (not isinstance(value.get('operation_evidence'), dict) or
            value['operation_evidence'].get('status') != 'USER_REPORTED_LEGACY_OPERATION'):
        raise FeedbackError('manual operation must retain its user-reported evidence classification')
    return value


controls_config = validate_manual_controls


def velocity_request(wheel, control, linear, angular):
    bounded(linear, 'linear target', -control['max_linear_m_s'], control['max_linear_m_s'])
    bounded(angular, 'angular target', -control['max_angular_rad_s'], control['max_angular_rad_s'])
    velocity = (linear-angular*wheel['track_width_m']/2, linear+angular*wheel['track_width_m']/2)
    rpm = [v*60/(2*math.pi*wheel['wheel_radius_m']) for v in velocity]
    if max(map(abs, rpm)) > 30:
        raise FeedbackError('manual wheel rpm exceeds 30 rpm bound')
    scale = control['command_rpm_to_register_scale']
    words = [round(rpm[i]*wheel[sign]*scale) for i, sign in enumerate(('left_sign', 'right_sign'))]
    if max(map(abs, words)) > 32767:
        raise FeedbackError('manual speed register overflow')
    return speed_write(*words)


def validate_response(request, response):
    response = checked_frame(response)
    if request == QUERY:
        parse_exchange(request, response)
    elif request in (*INITIALIZATION_WRITES, RELEASE_REQUEST):
        if response != request:
            raise FeedbackError('FC06 ACK must echo the exact reviewed register/value')
    elif request[1:7] == ZERO_REQUEST[1:7]:
        if len(response) != 8 or response[:6] != request[:6]:
            raise FeedbackError('FC10 ACK must echo exact slave/function/register/count')
    else:
        raise FeedbackError('unreviewed request')
    return bytes(response)


class Cancelled(FeedbackError):
    """An expired or cancelled user intent was rejected before writing."""


class Superseded(Cancelled):
    """A fresh changed target replaces the pending write without cancelling input."""


class WheelChannel:
    """One bounded RTU exchange at a time, using the existing exclusive lease."""
    def __init__(self, lease, journal, *, clock=time.monotonic):
        self.lease, self.journal, self.clock = lease, journal, clock
        self.session_id = None
        self.failed = False
        self.control_transmissions = self.control_attempts = self.control_bytes = 0
        self.motion_may_exist = False
        self.recovery_attempted = False
        self.last_stop = None

    def _log(self, record, *, required=True):
        try:
            self.journal({'schema': 'wc_mapping_wheel_transaction_v1', 'session_id': self.session_id,
                          'device_id': getattr(self.lease, 'config', {}).get('device_id'), **record})
        except Exception:
            if required:
                raise

    def exchange(self, request, timeout, *, guard=None):
        request = checked_frame(request)
        control = request != QUERY
        if control and request not in (*INITIALIZATION_WRITES, RELEASE_REQUEST) and not (
                len(request) == 13 and request[:7] == ZERO_REQUEST[:7]):
            raise FeedbackError('wheel write outside FC06 initialization/release / FC10 paired-speed allowlist')
        if control and guard is None:
            raise FeedbackError('every control write requires an explicit current user-state guard')
        if self.failed or self.lease.failed or self.lease.fd is None:
            raise QueryFailure('closed or failed wheel channel')
        bounded(timeout, 'wheel transaction timeout', .01, .2)
        response, sent = bytearray(), 0
        started = time.monotonic_ns()
        try:
            self._log({'event': 'wheel_io_planned', 'request_hex': request.hex(), 'control': control,
                       'control_transmissions': self.control_transmissions})
            if select.select([self.lease.fd], [], [], .002)[0]:
                response.extend(os.read(self.lease.fd, 256))
                raise QueryFailure('unsolicited RTU bytes; no flush or retry', response)
            deadline = self.clock()+timeout
            if not select.select([], [self.lease.fd], [], max(0, deadline-self.clock()))[1]:
                raise QueryFailure('wheel write readiness timeout')
            if guard is not None:
                guard()  # After potentially blocking reads/select, immediately before write.
            if control:
                self.control_attempts += 1
                if request == INITIALIZATION_WRITES[-1] or (request[1] == 0x10 and request != ZERO_REQUEST):
                    self.motion_may_exist = True
            sent = os.write(self.lease.fd, request)
            if control and sent:
                self.control_transmissions += 1
                self.control_bytes += sent
            if sent != len(request):
                raise QueryFailure('partial wheel transmission; no retry', response, sent)
            expected = 9 if request == QUERY else 8
            while len(response) < expected:
                remaining = deadline-self.clock()
                if remaining <= 0 or not select.select([self.lease.fd], [], [], remaining)[0]:
                    raise QueryFailure('wheel response timeout; no retry', response, sent)
                chunk = os.read(self.lease.fd, 256)
                if not chunk:
                    raise QueryFailure('wheel serial disconnected; no reconnect', response, sent)
                response.extend(chunk)
                if len(response) >= 2 and response[1] & 0x80:
                    expected = 5
                if len(response) > expected:
                    raise QueryFailure('wheel RTU overlapping response', response, sent)
            result = validate_response(request, response)
            self._log({'event': 'wheel_io_complete', 'request_hex': request.hex(), 'response_hex': result.hex(),
                       'transmitted_bytes': sent, 'control_transmissions': self.control_transmissions,
                       'exchange_started_monotonic_ns': started, 'exchange_completed_monotonic_ns': time.monotonic_ns()})
            return result
        except BaseException as error:
            if not isinstance(error, Cancelled) or sent:
                self.failed = self.lease.failed = True
            self._log({'event': 'wheel_io_failed', 'request_hex': request.hex(), 'response_hex': bytes(response).hex(),
                       'transmitted_bytes': sent, 'control': control, 'error': str(error),
                       'control_transmissions': self.control_transmissions, 'control_write_attempts': self.control_attempts}, required=False)
            if isinstance(error, (Cancelled, KeyboardInterrupt, SystemExit)):
                raise
            raise QueryFailure(str(error), response, sent) from error

    def best_effort_zero(self):
        """At most one zero write after failed RTU association; never accepts ACK."""
        if not self.motion_may_exist or self.recovery_attempted:
            return self.last_stop
        self.recovery_attempted = True
        sent, error = 0, None
        try:
            if self.lease.fd is None or not select.select([], [self.lease.fd], [], .05)[1]:
                raise FeedbackError('no writable wheel descriptor for bounded zero')
            self.control_attempts += 1
            sent = os.write(self.lease.fd, ZERO_REQUEST)
            if sent:
                self.control_transmissions += 1
                self.control_bytes += sent
            if sent != len(ZERO_REQUEST):
                raise FeedbackError('partial best-effort zero transmission')
        except Exception as failure:
            error = str(failure)
        self.last_stop = {'event': 'best_effort_zero', 'request_hex':ZERO_REQUEST.hex(), 'control':True,
                          'response_hex':'', 'transmitted_bytes': sent, 'error': error,
                          'acknowledged': False, 'stationary_feedback_observed': False,
                          'physical_stop_confirmed': False, 'control_transmissions': self.control_transmissions}
        self._log(self.last_stop, required=False)
        return self.last_stop


class MappingWheel:
    """Pure scheduler/state machine; command packets never directly write devices."""
    def __init__(self, config, channel, journal, *, publish=None, clock=time.monotonic,
                 wall_ns=time.time_ns, mono_ns=time.monotonic_ns, before_control=None, cancelled=None):
        from wc_motion.history_preview import HistoryPreview
        self.config, self.channel, self.journal = config, channel, journal
        self.session_id = config['session_id']
        if not isinstance(self.session_id, str) or not re.fullmatch(r'[A-Za-z0-9_][A-Za-z0-9_.-]{0,95}', self.session_id):
            raise FeedbackError('explicit safe session identity required')
        self.channel.session_id = self.session_id
        self.control = controls_config(config.get('manual_controls'))
        self.wheel = copy.deepcopy(config['wheel_candidate'])
        self.preview = HistoryPreview(self.wheel, continuous=config.get('continuous_mapping', False))
        self.publish = publish or (lambda record, preview: None)
        self.clock, self.wall_ns, self.mono_ns = clock, wall_ns, mono_ns
        self.before_control = before_control or (lambda: None)
        self.cancelled = cancelled or (lambda: False)
        self.timeout = min(config.get('feedback_timeout_s', .08), .08)
        bounded(self.timeout, 'mapping wheel transaction timeout', .01, .08)
        self.state = 'READY' if self.control['arm_allowed'] else 'DISARMED'
        self.reason = 'READY_FOR_FOREGROUND_HYBRID' if self.control['arm_allowed'] and self.control['interaction_policy']=='hybrid_manual' else (
            'READY_FOR_WASD_OR_HAND_PUSH' if self.control['arm_allowed'] else 'STARTUP_READ_ONLY')
        self.failure = None
        self.keys = set()
        self.sequence = self.last_key_sequence = self.arm_generation = 0
        self.epoch = str(uuid.uuid4())
        self.last_keys = self.last_feedback = self.last_clock = None
        self.samples = deque(maxlen=3)
        self.peer_connected = False
        self.initialized_by_user = False
        self.init_step = 0
        self.next_init = self.next_feedback = self.next_command = 0.
        self.stop_requested = False
        self.stop_deadline = None
        self.stop_zero_samples = 0
        self.push_mode = self.push_mode_requested = False
        self.hybrid_manual = self.control['interaction_policy'] == 'hybrid_manual'
        self.foreground = False
        self.last_ui_heartbeat = None
        self.hybrid_release_inhibited = False
        self.servo_release_acknowledged = False
        self.published_feedback_count = self.publication_failures = 0
        self.closed = self.closing = False

    def _log(self, record, *, required=True):
        try:
            self.journal({'session_id': self.session_id, **record})
        except Exception:
            if required:
                raise

    def connected(self):
        self.peer_connected = True
        self.last_key_sequence = 0
        self.foreground = False
        self.last_ui_heartbeat = None

    def disconnected(self):
        self.peer_connected = False
        self.disarm('OPERATOR_CONNECTION_CLOSED')

    def disarm(self, reason):
        normal_release = reason == 'ALL_KEYS_RELEASED' and self.hybrid_manual and self._ui_fresh()
        self.keys.clear()
        self.last_keys = None
        self.arm_generation += 1
        self.reason = reason
        if self.state != 'FAULT':
            self.state = 'READY' if self.control['arm_allowed'] else 'DISARMED'
        self.init_step = 0
        self.push_mode_requested = False
        if self.hybrid_manual:
            self.hybrid_release_inhibited = not normal_release
            if normal_release:
                self.push_mode_requested = True
                self.stop_deadline = None
                self.stop_requested = True
            else:
                self.foreground = False
                self.last_ui_heartbeat = None
        self.stop_requested = self.stop_requested or (
            self.channel.motion_may_exist and self.stop_deadline is None)

    def _ui_fresh(self):
        return (self.peer_connected and self.foreground and self.last_ui_heartbeat is not None
                and 0 <= self.clock()-self.last_ui_heartbeat < KEY_TIMEOUT_S)

    def _request_hybrid_push(self):
        if (not self.hybrid_manual or not self.control['arm_allowed'] or self.hybrid_release_inhibited
                or not self._ui_fresh() or self.keys or self.push_mode or self.push_mode_requested
                or self.state != 'READY' or self.closing or self.cancelled()):
            return
        self.push_mode_requested = True
        self.reason = 'HYBRID_WAITING_FRESH_STATIONARY_FEEDBACK'
        self.stop_zero_samples = 0
        if self.channel.motion_may_exist:
            self.stop_deadline = None
            self.stop_requested = True
        else:
            # Startup does not initialise or issue a speed command. Observe three
            # new zero samples before the explicitly configured release command.
            self.stop_deadline = self.clock()

    def _stationary_window_fresh(self):
        return (self.stop_zero_samples >= 3 and len(self.samples) == 3
                and all(words == [0, 0] and 0 <= self.clock()-stamp <= FEEDBACK_FRESH_S
                        for stamp, words in self.samples))

    def fault(self, reason):
        self.failure = self.failure or str(reason)
        self.state = 'FAULT'
        self.disarm(self.failure)
        self._log({'event': 'manual_fault', 'reason': self.failure}, required=False)

    def command(self, value):
        """Mutate intent only. Socket servicing during an RTU guard is safe."""
        if not isinstance(value, dict) or value.get('session_id') != self.session_id:
            raise FeedbackError('manual packet session identity mismatch')
        kind = value.get('type')
        if kind in ('disarm', 'close'):
            self.disarm('USER_DISARM' if kind == 'disarm' else 'USER_CLOSE')
            return
        if kind == 'arm':
            # Accept old UI packets without turning an idle connection into an
            # enable request. New clients only send fresh key intents.
            if not self.control['arm_allowed']:
                self.reason = self.control['reason']
            return
        if kind == 'push_mode':
            seq = value.get('sequence')
            if type(seq) is not int or seq <= self.last_key_sequence or type(value.get('enabled')) is not bool:
                raise FeedbackError('hand-push mode requires explicit boolean and increasing sequence')
            self.last_key_sequence = seq
            if (self.closed or self.closing or self.state == 'FAULT' or not self.control['arm_allowed']
                    or not self.peer_connected or type(value.get('arm_generation')) is not int
                    or value['arm_generation'] != self.arm_generation):
                return
            enabled = value['enabled']
            self.disarm('USER_HAND_PUSH_REQUEST' if enabled else 'USER_LEFT_HAND_PUSH')
            self.push_mode = False
            self.push_mode_requested = enabled
            # Exiting push mode remains passive until a fresh WASD intent.
            # Entering first commands zero, then waits for fresh stationary feedback.
            if enabled:
                self.stop_deadline = None
                self.stop_requested = True
            self._log({'event': 'user_hand_push_requested', 'enabled': enabled,
                       'arm_generation': self.arm_generation, 'physical_stop_confirmed': False})
            return
        if kind != 'keys':
            raise FeedbackError('unknown manual packet type')
        seq, keys = value.get('sequence'), value.get('keys')
        if type(seq) is not int or seq <= self.last_key_sequence or not isinstance(keys, list) or \
                len(keys) > 4 or any(type(key) is not str or key not in ('w', 'a', 's', 'd') for key in keys) or len(set(keys)) != len(keys):
            raise FeedbackError('keys require unique WASD values and an increasing integer sequence')
        self.last_key_sequence = seq
        # Cancellation generations reject buffered old intent, but an
        # initialization itself must retain continuously refreshed held keys.
        if self.closed or self.closing or self.state not in ('READY', 'INITIALIZING', 'ARMED') or not self.control['arm_allowed'] or \
                type(value.get('arm_generation')) is not int or value['arm_generation'] != self.arm_generation:
            return
        if not self.peer_connected:
            raise FeedbackError('manual client disconnected')
        now = self.clock()
        if self.hybrid_manual:
            if type(value.get('foreground')) is not bool:
                raise FeedbackError('hybrid_manual keys heartbeat requires explicit foreground boolean')
            if not value['foreground']:
                if self.foreground or self.keys or self.push_mode_requested:
                    self.disarm('UI_NOT_FOREGROUND')
                return
            self.foreground = True
            self.last_ui_heartbeat = now
            if keys:
                self.hybrid_release_inhibited = False
                self.push_mode = self.push_mode_requested = False
                self.stop_deadline = None
                self.stop_zero_samples = 0
        elif self.push_mode or self.push_mode_requested:
            return
        if self.keys and self.last_keys is not None and now-self.last_keys >= KEY_TIMEOUT_S:
            self.disarm('KEY_TIMEOUT_BEFORE_REFRESH')
            return
        had_intent = bool(self.keys) or self.state in ('INITIALIZING', 'ARMED')
        self.keys = set(keys)
        if self.target() == (0., 0.):
            # Opposing held keys mean zero but remain real current input:
            # releasing one of them may resume motion without a new keydown.
            if self.keys:
                self.last_keys = now
                self.state, self.reason = 'READY', 'OPPOSING_KEYS_ZERO'
                self.init_step = 0
                self.stop_requested = self.stop_requested or (
                    self.channel.motion_may_exist and self.stop_deadline is None)
            elif had_intent:
                self.disarm('ALL_KEYS_RELEASED')
            else:
                self.keys.clear()
                self.last_keys = None
                self._request_hybrid_push()
        else:
            self.last_keys = now

    def target(self):
        return ((int('w' in self.keys)-int('s' in self.keys))*self.control['max_linear_m_s'],
                (int('a' in self.keys)-int('d' in self.keys))*self.control['max_angular_rad_s'])

    def _guard(self, *, initialization=False, expected=None):
        self.before_control()
        now = self.clock()
        if self.cancelled() or not self.peer_connected or not self.control['arm_allowed']:
            raise Cancelled('user control cancelled before write')
        if self.hybrid_manual and not self._ui_fresh():
            raise Cancelled('fresh foreground user intent required before write')
        if self.last_feedback is None or now-self.last_feedback > FEEDBACK_FRESH_S:
            raise Cancelled('fresh wheel feedback required immediately before write')
        if not self.keys or self.last_keys is None or now-self.last_keys >= KEY_TIMEOUT_S:
            raise Cancelled('fresh keyboard intent required before write')
        if self.target() == (0., 0.):
            if self.state == 'READY':
                # A newly held opposite key is current zero intent, not a
                # cancellation. Preserve both keys while the pending stop is
                # serviced; releasing either one can then resume naturally.
                raise Superseded('fresh opposing keys replace pending write with zero')
            raise Cancelled('nonzero keyboard intent required before write')
        if self.stop_requested:
            raise Cancelled('user stop pending before write')
        if initialization:
            if self.state != 'INITIALIZING':
                raise Cancelled('initialization cancelled before write')
        elif self.state != 'ARMED':
            raise Cancelled('keyboard intent cancelled before write')
        elif self.target() != expected:
            raise Superseded('fresh changed keyboard target replaces pending write')

    def sample(self):
        started, wall = self.mono_ns(), self.wall_ns()
        self._log({'event':'request_planned', 'stream_epoch':self.epoch, 'sequence':self.sequence,
                   'request_hex':QUERY.hex(), 'request_unix_ns':wall, 'request_monotonic_ns':started})
        response = self.channel.exchange(QUERY, self.timeout)
        ended, stamp = self.mono_ns(), self.wall_ns()
        words = list(parse_exchange(QUERY, response)[2])
        now = self.clock()
        record = {'schema': 'wc_wheel_feedback_v1', 'device_id': self.config['wheel_device_id'],
                  'session_id': self.session_id, 'stream_epoch': self.epoch, 'sequence': self.sequence,
                  'request_hex': QUERY.hex(), 'response_hex': response.hex(),
                  'response_sha256': hashlib.sha256(response).hexdigest(), 'register_words_u16': words,
                  'request_unix_ns': wall, 'request_monotonic_ns': started, 'stamp_ns': stamp,
                  'receive_monotonic_ns': ended, 'exchange_started_monotonic_ns': started,
                  'exchange_completed_monotonic_ns': ended, 'transmitted_bytes': len(QUERY),
                  'status': 'RESPONSE_VALID', 'time_source': 'arrival_only', 'time_valid': False,
                  'uncertainty_ns': None, 'formal_odometry_eligible': False, 'protocol_units_state': 'UNVALIDATED',
                  'control_context': CONTROL_CONTEXT, 'control_transmissions': self.channel.control_transmissions,
                  'control_write_attempts': self.channel.control_attempts,
                  'manual_state': self.state}
        self.sequence += 1  # Count every valid source response even if archiving/publication fails.
        self._log({'event': 'transaction_complete', **record})
        candidate = self.preview.update(record)
        self._log({'event': 'history_preview', **candidate})
        try:
            self.publish(record, candidate)
            self.published_feedback_count += 1
        except Exception:
            self.publication_failures += 1
            raise
        self.last_feedback = now
        self.samples.append((now, words))
        if self.stop_deadline is not None:
            self.stop_zero_samples = self.stop_zero_samples+1 if words == [0, 0] else 0
            if self.stop_zero_samples >= 3:
                self.channel.last_stop = {'acknowledged': True, 'stationary_feedback_observed': True,
                                          'physical_stop_confirmed': False}
        return record

    def _stop(self):
        self.stop_requested = False
        if not self.channel.motion_may_exist and not self.push_mode_requested:
            return
        if self.channel.failed:
            self.channel.best_effort_zero()
            return
        if self.stop_deadline is not None:
            return
        try:
            # Motion/enable was attempted only after nonzero user intent. No stop
            # writes are reachable during the default read-only lifetime.
            self.channel.exchange(ZERO_REQUEST, self.timeout, guard=lambda: None)
            self.channel.last_stop = {'acknowledged': True, 'stationary_feedback_observed': False,
                                      'physical_stop_confirmed': False}
            self.stop_deadline, self.stop_zero_samples = self.clock(), 0
        except Exception:
            self.channel.best_effort_zero()
            raise

    def _release(self):
        """Release only for fresh explicit push or persistently authorised hybrid idle."""
        if not self.push_mode_requested or self.stop_deadline is None or not self._stationary_window_fresh():
            return
        generation = self.arm_generation
        def guard():
            self.before_control()
            if (self.closing or self.cancelled() or not self.peer_connected or not self.control['arm_allowed']
                    or not self.push_mode_requested or generation != self.arm_generation
                    or self.target() != (0., 0.) or self.state in ('INITIALIZING', 'ARMED', 'FAULT')
                    or self.stop_deadline is None or self.stop_zero_samples < 3
                    or not self._stationary_window_fresh()
                    or (self.hybrid_manual and (self.hybrid_release_inhibited or not self._ui_fresh()))
                    or self.last_feedback is None or self.clock()-self.last_feedback > FEEDBACK_FRESH_S):
                raise Cancelled('explicit hand-push intent and fresh stationary feedback required before release')
        try:
            self.channel.exchange(RELEASE_REQUEST, self.timeout, guard=guard)
        except Cancelled:
            # A new press received while waiting to write cancels release,
            # preserving that fresh intent for the normal scheduler below.
            return
        self.stop_deadline = None
        self.push_mode, self.push_mode_requested = True, False
        self.servo_release_acknowledged = True
        self.initialized_by_user = self.channel.motion_may_exist = False
        self.channel.last_stop = {**(self.channel.last_stop or {}), 'servo_released': True,
                                  'physical_stop_confirmed': False}
        if self.state != 'FAULT':
            self.reason = 'HYBRID_HAND_PUSH_ACTIVE' if self.hybrid_manual else 'USER_HAND_PUSH_ACTIVE'
        self._log({'event': 'hand_push_release_acknowledged', 'physical_stop_confirmed': False})

    def step(self):
        generation_before_step = self.arm_generation
        try:
            now = self.clock()
            if self.last_clock is not None and now < self.last_clock:
                raise FeedbackError('mapping wheel monotonic clock reversed')
            self.last_clock = now
            check = getattr(self.journal, 'check_health', None)
            if check is not None:
                check()
            if self.cancelled():
                self.disarm('SESSION_CANCELLED')
            if self.hybrid_manual and self.last_ui_heartbeat is not None and not self._ui_fresh():
                self.disarm('UI_HEARTBEAT_TIMEOUT')
            if self.state in ('READY', 'INITIALIZING', 'ARMED') and self.keys and now-self.last_keys >= KEY_TIMEOUT_S:
                self.disarm('KEY_TIMEOUT')
            if self.stop_requested:
                self._stop()
            if self.state == 'FAULT':
                return
            if now >= self.next_feedback:
                self.next_feedback = now+FEEDBACK_PERIOD_S
                self.sample()
            now = self.clock()
            if self.stop_deadline is not None and self.target() == (0., 0.):
                self._release()
            if self.cancelled():
                return
            if self.state == 'READY' and self.target() != (0., 0.):
                self.state = 'ARMED' if self.initialized_by_user else 'INITIALIZING'
                self.reason = 'USER_KEYBOARD_INTENT' if self.initialized_by_user else 'USER_REQUESTED_INITIALIZATION'
                self.init_step, self.next_init = 0, now
                self.stop_deadline = None
                self._log({'event': 'user_motion_requested', 'operation_evidence': self.control.get('operation_evidence'),
                           'hardware_watchdog_verified': False})
            if self.state == 'INITIALIZING' and now >= self.next_init:
                # The pre-enable zero prevents a retained old target becoming
                # active on enable. Every step is separately guarded and logged.
                sequence = ((INITIALIZATION_WRITES[0], 0), (INITIALIZATION_WRITES[1], 0),
                            (INITIALIZATION_WRITES[2], .2), (ZERO_REQUEST, 0),
                            (INITIALIZATION_WRITES[3], .2))
                if self.init_step < len(sequence):
                    request, delay = sequence[self.init_step]
                    self.channel.exchange(request, self.timeout, guard=lambda: self._guard(initialization=True))
                    self.init_step += 1
                    self.next_init = self.clock()+delay
                else:
                    self._guard(initialization=True)
                    self.initialized_by_user = True
                    self.servo_release_acknowledged = False
                    self.state, self.reason = 'ARMED', 'USER_KEYBOARD_INTENT'
                    self.next_command = self.clock()
            elif self.state == 'ARMED' and self.keys and now >= self.next_command:
                target = self.target()
                if target == (0., 0.):
                    self.keys.clear()
                    self.stop_requested = self.channel.motion_may_exist
                else:
                    self.channel.exchange(velocity_request(self.wheel, self.control, *target), self.timeout,
                                          guard=lambda: self._guard(expected=target))
                    self.next_command = self.clock()+CONTROL_PERIOD_S
                    self.reason = 'USER_KEYBOARD_INTENT'
        except Superseded:
            # Socket servicing in the prewrite guard may add/remove a held
            # direction. Discard only that old velocity write; keep the fresh
            # keys and generation so the next scheduler pass sends the change.
            self.next_command = self.clock()
        except Cancelled as error:
            # The prewrite socket pump may already have cancelled intent and
            # published its new generation. Revoking it again would invalidate
            # fresh keys sent in response to that acknowledgement.
            if self.arm_generation == generation_before_step:
                self.disarm(str(error))
        except Exception as error:
            self.fault(str(error))
            try:
                self._stop()
            except Exception:
                pass

    def status(self):
        linear, angular = self.target() if self.state == 'ARMED' else (0., 0.)
        return {'type': 'status', 'schema': 'wc_mapping_manual_status_v1', 'session_id': self.session_id,
                'state': self.state, 'reason': self.reason, 'arm_allowed': self.control['arm_allowed'],
                'arm_block_reason': None if self.control['arm_allowed'] else self.control['reason'],
                'arm_generation': self.arm_generation, 'keys': sorted(self.keys),
                'max_linear_m_s': self.control['max_linear_m_s'], 'max_angular_rad_s': self.control['max_angular_rad_s'],
                'target_linear_m_s': linear, 'target_angular_rad_s': angular,
                'control_transmissions': self.channel.control_transmissions, 'control_write_attempts': self.channel.control_attempts,
                'control_bytes_observed': self.channel.control_bytes,
                'last_feedback_age_s': None if self.last_feedback is None else max(0, self.clock()-self.last_feedback),
                'feedback_samples': self.sequence, 'feedback_rate_hz': 10, 'key_timeout_s': KEY_TIMEOUT_S,
                'continuous_mapping': self.preview.continuous, 'feedback_gap_count': self.preview.gap_count,
                'feedback_uncovered_duration_s': self.preview.uncovered_duration_s,
                'max_continuous_motion_s': None,
                'interaction_policy': self.control['interaction_policy'],
                'control_mode': 'hybrid_manual' if self.hybrid_manual else 'direct_wasd',
                'release_after_zero_s': None,
                'brake_release_policy': 'FRESH_FOREGROUND_NORMAL_IDLE' if self.hybrid_manual else 'EXPLICIT_USER_HAND_PUSH',
                'foreground': self.foreground, 'ui_heartbeat_age_s': None if self.last_ui_heartbeat is None else max(0,self.clock()-self.last_ui_heartbeat),
                'hybrid_release_inhibited': self.hybrid_release_inhibited,
                'servo_release_acknowledged': self.servo_release_acknowledged,
                'brake_holding_confirmed': False,
                'published_feedback_count': self.published_feedback_count, 'publication_failures': self.publication_failures,
                'push_mode': self.push_mode, 'push_mode_requested': self.push_mode_requested,
                'ready_without_arming': self.control['arm_allowed'],
                'peer_connected': self.peer_connected, 'last_stop': self.channel.last_stop,
                'hardware_watchdog_verified': False, 'hardware_watchdog_timeout_s': None,
                'firmware_id': None, 'physical_stop_confirmed': False,
                'operation_evidence': self.control.get('operation_evidence'), 'failure': self.failure}

    def close(self):
        if self.closed:
            return
        self.closing = True
        self.disarm('SESSION_EXIT' if self.failure is None else self.failure)
        try:
            self._stop()
            # Shutdown requests zero for attempted motion; it never releases holding.
        except Exception as error:
            self.fault(str(error))
            self.channel.best_effort_zero()
        finally:
            self.closed = True


class ManualSocket:
    """One same-UID local UI client; bounded nonblocking JSON-line protocol."""
    def __init__(self, path, owner):
        self.path, self.owner = Path(path), owner
        self.listener = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self.client = None
        self.ready = False
        self.received = bytearray()
        self.pending = bytearray()
        self.accepted_at = None
        self.last_status = 0.
        self.identity = None
        try:
            self.listener.bind(str(self.path))  # Existing sockets are never removed/reused.
            os.chmod(self.path, 0o600)
            info = self.path.stat(); self.identity = (info.st_dev, info.st_ino)
            self.listener.listen(1); self.listener.setblocking(False)
        except BaseException:
            self.listener.close()
            raise

    def drop(self):
        if self.client is not None:
            self.client.close(); self.client = None
            self.owner.disconnected()
        self.ready = False
        self.received.clear(); self.pending.clear()

    def _send_status(self):
        payload = (json.dumps(self.owner.status(), ensure_ascii=False, allow_nan=False)+'\n').encode()
        if len(self.pending)+len(payload) > 16384:
            raise FeedbackError('manual client is not consuming bounded status output')
        self.pending.extend(payload)
        self.last_status = self.owner.clock()

    def pump(self):
        try:
            if select.select([self.listener], [], [], 0)[0]:
                client, _ = self.listener.accept()
                if self.client is not None:
                    client.close()
                else:
                    if hasattr(socket, 'SO_PEERCRED'):
                        _, uid, _ = struct.unpack('3i', client.getsockopt(socket.SOL_SOCKET, socket.SO_PEERCRED, 12))
                        if uid != os.getuid():
                            client.close(); return
                    self.client = client; client.setblocking(False)
                    self.accepted_at = self.owner.clock()
            if self.client is None:
                return
            if not self.ready and self.owner.clock()-self.accepted_at > 1.:
                raise FeedbackError('manual session handshake timeout')
            if select.select([self.client], [], [], 0)[0]:
                data = self.client.recv(8193)
                if not data:
                    self.drop(); return
                self.received.extend(data)
                if len(self.received) > 8192:
                    raise FeedbackError('manual input buffer overflow')
                count = 0
                while b'\n' in self.received:
                    line, _, rest = self.received.partition(b'\n')
                    self.received[:] = rest
                    count += 1
                    if count > 8 or not line or len(line) > 4096:
                        raise FeedbackError('manual input line/flood bound')
                    from .prepare_picker_input import _json_loads
                    value = _json_loads(line.decode('utf-8'))
                    if not self.ready:
                        if not isinstance(value, dict) or value != {'type': 'hello', 'session_id': self.owner.session_id}:
                            raise FeedbackError('manual session hello required')
                        self.ready = True; self.owner.connected(); self._send_status()
                    else:
                        self.owner.command(value)
                        if value.get('type') == 'close':
                            self.drop(); return
                if len(self.received) > 4096:
                    raise FeedbackError('unterminated manual line exceeds bound')
            if self.ready and self.owner.clock()-self.last_status >= .1:
                self._send_status()
            if self.pending and select.select([], [self.client], [], 0)[1]:
                sent = self.client.send(self.pending)
                if sent <= 0:
                    self.drop(); return
                del self.pending[:sent]
        except (ValueError, TypeError, UnicodeError, OSError) as error:
            # Malformed/expired clients cannot cause movement; closing revokes
            # any lease. The feedback stream continues for the mapping owner.
            self.owner.disarm('MANUAL_SOCKET_REJECTED: '+str(error))
            self.drop()

    def close(self):
        self.drop(); self.listener.close()
        try:
            value = self.path.lstat()
            if stat.S_ISSOCK(value.st_mode) and (value.st_dev, value.st_ino) == self.identity:
                self.path.unlink()
        except FileNotFoundError:
            pass


class RosOutput:
    def __init__(self):
        import rclpy
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.signals import SignalHandlerOptions
        from std_msgs.msg import String
        from nav_msgs.msg import Odometry
        from wc_motion.history_preview import assign_preview_odometry
        self.assign_preview_odometry = assign_preview_odometry
        self.ros, self.context, self.String, self.Odometry = rclpy, Context(), String, Odometry
        rclpy.init(args=[], context=self.context, signal_handler_options=SignalHandlerOptions.NO)
        self.node = rclpy.create_node('wc_mapping_wheel', context=self.context)
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        self.raw = self.node.create_publisher(String, '/wc_mapping/wheel/feedback_raw',
            QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE))
        self.raw_publish_count = self.raw_publish_failures = 0
        self.odom = self.node.create_publisher(Odometry, '/wc_mapping/wheel/odom_preview', 20)
        self.preview = self.node.create_publisher(String, '/wc_mapping/wheel/preview_diagnostics', 20)
        self.manual = self.node.create_publisher(String, '/wc_mapping/wheel/manual_status', 10)
        # This publisher owns a private context. The implicit global executor
        # uses an uninitialized default context when discovery is not immediate.
        self.executor = SingleThreadedExecutor(context=self.context)

    def publish(self, record, candidate):
        try:
            self.raw.publish(self.String(data=json.dumps(record, allow_nan=False)))
            self.raw_publish_count += 1
        except Exception:
            self.raw_publish_failures += 1
            raise
        self.odom.publish(self.assign_preview_odometry(candidate, self.Odometry()))
        self.preview.publish(self.String(data=json.dumps(candidate, allow_nan=False)))

    def wait_for_recorder(self, cancelled, timeout_s=10.):
        deadline = time.monotonic()+timeout_s
        while self.raw.get_subscription_count() < 1:
            if cancelled() or time.monotonic() >= deadline:
                raise FeedbackError('recording subscriber discovery timeout')
            self.ros.spin_once(self.node, executor=self.executor, timeout_sec=.05)
        return True

    def close(self):
        try:
            self.executor.shutdown(timeout_sec=1.0)
        finally:
            try:
                self.node.destroy_node()
            finally:
                if self.context.ok():
                    self.ros.shutdown(context=self.context, uninstall_handlers=False)

    def wait_for_recording_ack(self):
        from rclpy.duration import Duration
        if not self.raw.wait_for_all_acked(Duration(seconds=5.0)):
            raise FeedbackError('wheel reliable publication acknowledgement timeout')
        return True


def wheel_session_scope(args, project_root):
    """Keep normal mapping project-bound; admit only authenticated RAM capture.

    Returned snapshots are decoded from the same bytes whose hashes were
    checked. No serial/device access or directory creation occurs here.
    """
    from .prepare_picker_input import project_path, read_json
    from wc_calibration.picker import _json_loads
    directory = Path(args.session_root)
    root = Path(project_root).absolute()
    if not directory.is_absolute() or directory.is_relative_to(root):
        return project_path(root, directory), root, {}
    from .storage_policy import StoragePolicy
    storage = StoragePolicy(root)
    if (storage.enabled and directory.is_relative_to(storage.archive_root)
            and not args.require_recorder):
        return storage.resolve(directory), root, {}
    if (not args.require_recorder or args.config is None or args.summary_path is None
            or directory.parent != MEMORY_CAPTURE_ROOT
            or re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_.-]{0,63}', directory.name) is None):
        raise FeedbackError('outside-project wheel paths require an identified /dev/shm/wc_capture manual capture')
    directory = project_path(MEMORY_CAPTURE_ROOT, directory)
    if not hasattr(os, 'geteuid'):
        raise FeedbackError('RAM manual capture requires POSIX directory ownership')
    for path in (MEMORY_CAPTURE_ROOT, directory):
        info = path.lstat()
        if (not stat.S_ISDIR(info.st_mode) or info.st_uid != os.geteuid()
                or stat.S_IMODE(info.st_mode) & 0o022):
            raise FeedbackError('RAM capture directories must belong to this user and forbid other-user writes: '
                +str(path)+' uid='+str(info.st_uid)+' expected_uid='+str(os.geteuid())
                +' mode='+format(stat.S_IMODE(info.st_mode),'04o'))
    manifest = read_json(directory, directory/'capture_manifest.json', limit=1_000_000)
    if (not isinstance(manifest, dict) or manifest.get('status') != 'RECORDING'
            or manifest.get('manual_drive') is not True or manifest.get('session_id') != directory.name
            or manifest.get('control_authorization') != 'EXPLICIT_CAPTURE_MANUAL_DRIVE_ARGUMENT'):
        raise FeedbackError('RAM capture manifest is not an explicitly authorized current manual session')
    config_path = project_path(directory, args.config)
    if config_path != directory/'configuration/manual_runtime.json':
        raise FeedbackError('RAM wheel owner requires this session configuration/manual_runtime.json')
    project_path(directory, args.raw_output if args.raw_output is not None else directory/'wheel_feedback.jsonl')
    project_path(directory, args.summary_path)
    hashes = manifest.get('input_hashes')
    if not isinstance(hashes, dict):
        raise FeedbackError('RAM manual capture manifest requires frozen configuration hashes')
    snapshots = {}
    for relative in ('configuration/manual_runtime.json', 'configuration/wheel_feedback.json'):
        path = project_path(directory, directory/relative)
        expected = hashes.get(relative)
        if (not isinstance(expected, str) or re.fullmatch('[0-9a-fA-F]{64}', expected) is None
                or not path.is_file() or path.stat().st_size > 1_000_000):
            raise FeedbackError('RAM capture configuration snapshot/hash missing: '+relative)
        raw = path.read_bytes()
        if len(raw) > 1_000_000 or hashlib.sha256(raw).hexdigest() != expected.lower():
            raise FeedbackError('RAM capture configuration snapshot hash mismatch: '+relative)
        value = _json_loads(raw.decode('utf-8'))
        if not isinstance(value, dict):
            raise FeedbackError('RAM capture configuration snapshot must be an object: '+relative)
        snapshots[path] = value
    config = snapshots[config_path]
    if (config.get('session_id') != directory.name or config.get('source_mode') != 'real'
            or config.get('status') != 'EXPERIMENT'
            or project_path(directory, config['wheel_hardware_config']) != directory/'configuration/wheel_feedback.json'):
        raise FeedbackError('RAM wheel configuration identity or hardware snapshot scope mismatch')
    return directory, directory, snapshots


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session-root', required=True, type=Path)
    parser.add_argument('--config', type=Path, help='Frozen capture manual_runtime.json; default session/runtime_config.json')
    parser.add_argument('--raw-output', type=Path, help='Unique source transaction JSONL; default session/wheel_feedback.jsonl')
    parser.add_argument('--summary-path', type=Path, help='Post-fsync wheel source summary')
    parser.add_argument('--require-recorder', action='store_true', help='Discover reliable raw feedback recorder before serial open')
    from .mapping_shutdown import persistence_wait
    parser.add_argument('--close-timeout-s', type=persistence_wait, default=10.)
    args = parser.parse_args(argv)
    from .cli import ROOT, RUN, target
    from wc_motion.feedback_transport import FeedbackSerialLease, create_journal, validate_config
    from .prepare_picker_input import project_path, read_json, write_new, json_bytes
    target()
    directory, path_root, snapshots = wheel_session_scope(args, ROOT)
    config_path = project_path(path_root,args.config) if args.config else directory/'runtime_config.json'
    config = snapshots[config_path] if config_path in snapshots else read_json(path_root, config_path, limit=1_000_000)
    if args.config and config.get('session_id') != directory.name:
        raise FeedbackError('capture manual config identity must match dataset directory')
    controls = controls_config(config.get('manual_controls'))
    if controls['arm_allowed'] and (config.get('source_mode') != 'real' or config.get('offline_experiment') is True):
        raise FeedbackError('manual control requires an identified live source mode; replay cannot control hardware')
    hardware_path = project_path(path_root, config['wheel_hardware_config'])
    hardware = validate_config(snapshots[hardware_path] if hardware_path in snapshots else read_json(path_root, hardware_path))
    if config['wheel_device_id'] != hardware['device_id']:
        raise FeedbackError('mapping wheel hardware identity mismatch')
    if type(config['duration_s']) is not int or config['duration_s'] < 0:
        raise FeedbackError('mapping duration must be a nonnegative integer; 0 means until stopped')
    config = {**config, 'feedback_timeout_s': hardware['timeout_s']}
    raw_path = project_path(path_root,args.raw_output) if args.raw_output else directory/'wheel_feedback.jsonl'
    from .mapping_control_paths import manual_socket_path
    socket_path = manual_socket_path(ROOT, directory, config['session_id'])
    if socket_path.parent != directory:
        socket_path.parent.mkdir(parents=True, exist_ok=True)
    summary_path = project_path(path_root,args.summary_path) if args.summary_path else None
    if args.require_recorder and (not args.config or summary_path is None):
        raise FeedbackError('capture --require-recorder requires frozen --config and --summary-path')
    if args.require_recorder and (not controls['arm_allowed'] or controls['interaction_policy']!='hybrid_manual'):
        raise FeedbackError('manual capture owner requires persistently authorised hybrid_manual configuration')
    stopped = [False]
    def stop(_signum, _frame):
        stopped[0] = True
    for signum in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(signum, stop)
    lease = FeedbackSerialLease(hardware, RUN)
    journal = create_journal(raw_path, asynchronous=True, close_timeout_s=args.close_timeout_s)
    owner = output = control_socket = None
    result, failure = 0, None
    recorder_discovered = False
    ready_written = False
    try:
        if args.require_recorder:
            output = RosOutput()
            recorder_discovered = output.wait_for_recorder(lambda: stopped[0])
        lease.open()
        journal({'event': 'lease_open', 'session_id': config['session_id'], 'control_transmissions': 0,
                 'host_serial': lease.host_configuration,
                  'control_access': ('AUTHORIZED_HYBRID_FOREGROUND_ZERO_FEEDBACK_RELEASE; WASD_INITIALIZES_FC06_FC10'
                    if controls['arm_allowed'] and controls['interaction_policy']=='hybrid_manual' else
                    'IDLE_FC03_ONLY; authorized_nonzero_WASD_initializes_legacy_FC06_FC10'),
                  'interaction_policy':controls['interaction_policy']})
        if output is None:
            output = RosOutput()
        channel = WheelChannel(lease, journal)
        owner = MappingWheel(config, channel, journal, publish=output.publish, cancelled=lambda: stopped[0])
        control_socket = ManualSocket(socket_path, owner)
        owner.before_control = control_socket.pump
        last_status = 0.
        # The mapping supervisor alone owns the session duration. Keep feedback
        # available until its stop signal; source and key deadlines stay active.
        while not stopped[0] and owner.failure is None:
            control_socket.pump(); owner.step()
            if (args.require_recorder and not ready_written and owner.failure is None and owner.sequence >= 1
                    and owner.publication_failures == 0 and output.raw_publish_count >= 1):
                from .source_archive import atomic_json
                atomic_json(directory/'wheel_ready.json', {'session_id':config['session_id'], 'ready':True,
                    'raw_recorder_discovered':recorder_discovered, 'manual_socket_ready':True,
                    'feedback_samples':owner.sequence, 'source_type':'HYBRID_MANUAL_CAPTURE'})
                ready_written = True
            now = time.monotonic()
            if now-last_status >= .1:
                output.manual.publish(output.String(data=json.dumps(owner.status(), ensure_ascii=False, allow_nan=False)))
                last_status = now
            time.sleep(.005)
        if owner.failure:
            raise FeedbackError(owner.failure)
    except Exception as error:
        result, failure = 1, str(error)
        # Keep the actual failing call in wheel.log, rather than only a terse
        # exception string such as "__enter__" in the final status.
        import traceback
        traceback.print_exc()
        if owner is not None:
            owner.fault(failure)
    finally:
        if control_socket is not None:
            control_socket.close()
        if owner is not None:
            owner.close()
            if owner.failure:
                result, failure = 1, owner.failure
        if summary_path is not None:
            try:
                journal({'event':'capture_complete' if result==0 else 'capture_failed',
                    'completed':owner.sequence if owner is not None else 0, 'until_stopped':True,
                    'interrupted':stopped[0], 'formal_odometry_eligible':False,
                    'control_transmissions':owner.channel.control_transmissions if owner is not None else 0,
                    'error':failure})
            except Exception as error:
                result, failure = 1, failure or str(error)
        lease.close()  # Sensor descriptor closes before potentially slow journal fsync.
        try:
            journal({'event': 'lease_closed', 'control_transmissions': owner.channel.control_transmissions if owner else 0})
        except Exception as error:
            result, failure = 1, failure or str(error)
        # A full/failed journal can reject the terminal event. Its previously
        # accepted events still require a bounded drain and final fsync attempt.
        if args.require_recorder and output is not None:
            try:
                output.wait_for_recording_ack()
                journal({'event': 'publication_ack', 'acknowledged': True, 'timeout_s': 5.0})
            except Exception as error:
                result, failure = 1, failure or str(error)
        try:
            journal.close()
        except Exception as error:
            result, failure = 1, failure or str(error)
        if output is not None:
            try:
                output.close()
            except Exception as error:
                result, failure = 1, failure or str(error)
        final = {'state': 'STOPPED' if result == 0 else 'FAILED', 'failure': failure,
                 'manual': owner.status() if owner is not None else None, 'journal': journal.status()}
        write_new(directory/'wheel_status.json', json_bytes(final))
        if summary_path is not None:
            from .source_archive import atomic_json, digest
            completed = owner.sequence if owner is not None else 0
            published = getattr(output,'raw_publish_count',0) if output is not None else 0
            publish_failures = getattr(output,'raw_publish_failures',0) if output is not None else 0
            journal_status = journal.status()
            synchronized = result == 0 and journal_status.get('final_fsync_complete') is True
            atomic_json(summary_path, {'state':'RAW_FEEDBACK_CAPTURED' if result==0 else 'FAILED',
                'session_id':config['session_id'], 'completed':completed, 'sequence_end_exclusive':completed,
                'stream_epoch':owner.epoch if owner is not None else None,
                'published_feedback_count':published, 'publish_failures':publish_failures,
                'control_transmissions':owner.channel.control_transmissions if owner is not None else 0,
                'control_write_attempts':owner.channel.control_attempts if owner is not None else 0,
                'control_bytes_observed':owner.channel.control_bytes if owner is not None else 0,
                'interaction_policy':config['manual_controls'].get('interaction_policy','explicit_push'),
                'recorder_discovered':recorder_discovered, 'ready_observed':ready_written,
                'formal_odometry_eligible':False, 'closed_normally':result==0,
                'synchronized':synchronized, 'journal':journal_status, 'events_sha256':digest(raw_path),
                'failure':failure, 'publication_delivery_proven':False})
        print(json.dumps(final, ensure_ascii=False, allow_nan=False), flush=True)
    return result


if __name__ == '__main__':
    raise SystemExit(main())
