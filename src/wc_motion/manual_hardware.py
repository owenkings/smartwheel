"""Opt-in, evidence-gated user WASD over an exclusive 8030D RS485 connection.

Default execution delegates to the existing dry-run UI. Real mode is executable
only with a reviewed stop/watchdog contract, matching live state readback,
stationary feedback and an explicit on-site operator confirmation. No enable,
clear-fault, mode, control-word, brake-release or autonomous command is emitted.
"""

import argparse
import ctypes
import getpass
import hashlib
import json
import math
import os
import platform
from pathlib import Path
import select
import signal
import socket
import struct
import sys
import threading
import time
import uuid

from .feedback_transport import FeedbackSerialLease, QUERY, SERIAL, QueryFailure, bounded, validate_config
from .protocol import FeedbackError, checked_frame, crc16, integer, parse_exchange, read_request


ONSITE_CONFIRMATION = 'EMPTY_CHAIR_OPERATOR_PRESENT_PHYSICAL_STOP_READY'
REQUIRED_CHECKS = {'firmware', 'drive_enabled', 'velocity_mode', 'controller_watchdog', 'stop_policy'}
ZERO_PAYLOAD = struct.pack('>BBHHBhh', 1, 0x10, 0x2088, 2, 4, 0, 0)
ZERO_REQUEST = ZERO_PAYLOAD+crc16(ZERO_PAYLOAD)


def canonical_hash(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()).hexdigest()


def profile_from_config(config):
    """This is a candidate command profile until bound to independent evidence."""
    if config.get('schema') != 'wc_manual_hardware_v1':
        raise FeedbackError('manual hardware configuration schema required')
    profile = config['command_profile']
    if profile.get('slave_id') != 1 or profile.get('first_register') != 0x2088 or profile.get('register_count') != 2:
        raise FeedbackError('only reviewed dual speed registers 0x2088/0x2089 are supported')
    for name, lo, hi in (('wheel_radius_m', .01, 1), ('track_width_m', .1, 2),
                         ('rpm_to_register_scale', .01, 100), ('register_to_feedback_rpm', .0001, 100),
                         ('max_linear_m_s', .001, .15), ('max_angular_rad_s', .001, .3),
                         ('max_wheel_rpm', .1, 30)):
        bounded(profile.get(name), name, lo, hi)
    for name in ('left_sign', 'right_sign'):
        if type(profile.get(name)) is not int or profile[name] not in (-1, 1):
            raise FeedbackError('explicit reviewed wheel signs required')
    return profile


class ManualContract:
    """Verified-file evidence plus live predicates, not a hardware_enabled flag.

    Evidence files are human-reviewed records, not cryptographic attestations
    by the manufacturer. Hashes prevent using a different parameter/doc revision
    accidentally; the live state is checked independently before every command.
    """
    def __init__(self, config, evidence, project_root):
        self.profile = profile_from_config(config)
        self.project_root = Path(project_root).resolve()
        if evidence.get('schema') != 'wc_manual_stop_evidence_v1' or evidence.get('state') != 'REVIEWED_FOR_EMPTY_CHAIR_MANUAL':
            raise FeedbackError('reviewed stop/firmware/watchdog evidence is missing')
        if evidence.get('device_id') != 'ZLAC8030D-'+SERIAL or not evidence.get('firmware_id'):
            raise FeedbackError('stop evidence must identify this controller and firmware')
        if evidence.get('command_profile_sha256') != canonical_hash(self.profile):
            raise FeedbackError('manual command profile differs from the reviewed evidence')
        if not evidence.get('reviewer') or not evidence.get('reviewed_at'):
            raise FeedbackError('explicit reviewer and review date required')
        required_attestations = ('command_registers_units_directions_verified', 'controller_disconnect_stop_verified',
            'zero_velocity_hold_verified', 'physical_stop_and_brake_behavior_verified')
        if any(evidence.get(name) is not True for name in required_attestations):
            raise FeedbackError('manual control/stop/physical-stop evidence is incomplete')
        sources = evidence.get('sources', [])
        if not sources or not {'protocol', 'onsite_stop_test'}.issubset({item.get('kind') for item in sources}):
            raise FeedbackError('protocol and on-site stop test source files are both required')
        for item in sources:
            path = Path(item['path'])
            path = (self.project_root/path).resolve() if not path.is_absolute() else path.resolve()
            if not path.is_relative_to(self.project_root) or not path.is_file() or path.stat().st_size > 50_000_000:
                raise FeedbackError('review source must be a bounded file inside this project')
            if hashlib.sha256(path.read_bytes()).hexdigest() != item.get('sha256'):
                raise FeedbackError('manual review source content hash mismatch')
        timing = evidence.get('timing', {})
        self.transaction_timeout = bounded(timing.get('transaction_timeout_s'), 'manual transaction timeout', .01, .1)
        self.key_timeout = bounded(timing.get('key_timeout_s'), 'manual key timeout', .05, .5)
        self.watchdog_timeout = bounded(timing.get('controller_watchdog_timeout_s'), 'controller watchdog timeout', .05, .5)
        self.period = bounded(timing.get('command_period_s'), 'manual command period', .02, .15)
        self.stop_timeout = bounded(timing.get('zero_feedback_timeout_s'), 'stop feedback timeout', .1, 1)
        if self.period >= min(self.key_timeout, self.watchdog_timeout):
            raise FeedbackError('command period must be below key/controller watchdog limits')
        if self.transaction_timeout >= self.watchdog_timeout:
            raise FeedbackError('serial transaction budget must be below controller watchdog time')
        if evidence.get('stop_strategy') != 'zero_velocity_hold':
            raise FeedbackError('only independently verified zero-velocity hold is implemented; no guessed control word')
        self.stationary_bound = integer(evidence.get('stationary_feedback_abs_raw_max'), 'stationary feedback bound', 0, 10)
        self.checks = evidence.get('state_checks', [])
        if {item.get('purpose') for item in self.checks} != REQUIRED_CHECKS or len(self.checks) != len(REQUIRED_CHECKS):
            raise FeedbackError('firmware, enabled, velocity, watchdog and stop-policy live readback checks required')
        self.reads = {QUERY}
        for item in self.checks:
            request = checked_frame(bytes.fromhex(item['request_hex']))
            if len(request) != 8 or request[0:2] != b'\x01\x03':
                raise FeedbackError('state readback permits only slave 1 FC03')
            _, _, address, count = struct.unpack('>BBHH', request[:-2])
            if not 1 <= count <= 8 or request != read_request(1, address, count):
                raise FeedbackError('state readback range is invalid')
            integer(item.get('word_index'), 'state word index', 0, count-1)
            integer(item.get('mask'), 'state bit mask', 1, 65535)
            integer(item.get('equals'), 'expected state value', 0, 65535)
            if item['equals'] & ~item['mask']:
                raise FeedbackError('expected state value lies outside mask')
            self.reads.add(request)
        # A moving tick includes unique status reads, feedback and one write.
        # Account for each exchange's quiet guard and the inter-command period.
        exchange_count = len({item['request_hex'] for item in self.checks})+2
        self.command_budget = exchange_count*(self.transaction_timeout+.002)+self.period
        if self.command_budget >= min(self.key_timeout, self.watchdog_timeout):
            raise FeedbackError('complete feedback/state/write/period budget exceeds key or controller watchdog deadline')
        self.evidence_hash = canonical_hash(evidence)
        self.firmware_id = evidence['firmware_id']

    def velocity_request(self, linear, angular):
        p = self.profile
        bounded(linear, 'linear velocity', -p['max_linear_m_s'], p['max_linear_m_s'])
        bounded(angular, 'angular velocity', -p['max_angular_rad_s'], p['max_angular_rad_s'])
        wheel_speeds = (linear-angular*p['track_width_m']/2, linear+angular*p['track_width_m']/2)
        rpm = [value*60/(2*math.pi*p['wheel_radius_m']) for value in wheel_speeds]
        if max(abs(value) for value in rpm) > p['max_wheel_rpm']:
            raise FeedbackError('requested wheel rpm exceeds reviewed bound')
        words = [round(rpm[i]*p[name]*p['rpm_to_register_scale']) for i, name in enumerate(('left_sign', 'right_sign'))]
        if any(not -32768 <= value <= 32767 for value in words):
            raise FeedbackError('wheel command cannot be encoded as signed 16-bit value')
        payload = struct.pack('>BBHHBhh', 1, 0x10, 0x2088, 2, 4, *words)
        return payload+crc16(payload)

    def validate_write(self, request):
        request = checked_frame(request)
        if len(request) != 13 or request[:7] != ZERO_REQUEST[:7]:
            raise FeedbackError('only paired FC10 velocity registers are writable')
        values = struct.unpack('>hh', request[7:11])
        limit = math.ceil(self.profile['max_wheel_rpm']*self.profile['rpm_to_register_scale'])
        if max(abs(value) for value in values) > limit:
            raise FeedbackError('encoded speed lies above reviewed register bound')
        p = self.profile
        speeds = [values[i]/p['rpm_to_register_scale']*p[sign]*2*math.pi*p['wheel_radius_m']/60
                  for i, sign in enumerate(('left_sign', 'right_sign'))]
        quantum = .5/p['rpm_to_register_scale']*2*math.pi*p['wheel_radius_m']/60
        if abs(sum(speeds)/2) > p['max_linear_m_s']+quantum+1e-12 or \
                abs((speeds[1]-speeds[0])/p['track_width_m']) > p['max_angular_rad_s']+2*quantum/p['track_width_m']+1e-12:
            raise FeedbackError('encoded chassis motion exceeds reviewed linear/angular bound')
        return request


def validate_ack(request, response):
    response = checked_frame(response)
    if response[0] != 1:
        raise FeedbackError('manual response slave mismatch')
    if response[1] == (request[1] | 0x80):
        if len(response) != 5:
            raise FeedbackError('invalid Modbus exception length')
        raise FeedbackError(f'manual Modbus exception {response[2]}')
    if request[1] == 3:
        return parse_exchange(request, response)[2]
    if request[1] != 0x10 or len(response) != 8 or response[:6] != request[:6]:
        raise FeedbackError('FC10 ACK does not echo exact slave/function/register/count')
    return None


class ManualRtuChannel:
    """Actual FD exchanges, serialized for feedback and user velocity writes.

    A failed transaction can leave an indistinguishable late FC10 ACK. The
    failure path therefore permits only one best-effort zero transmission and
    never treats any following bytes as proof that zero was acknowledged.
    """
    def __init__(self, lease, contract, journal, *, cancel_check=None):
        self.lease, self.contract, self.journal = lease, contract, journal
        self.lock = threading.RLock()
        self.armed = False
        self.failed = False
        self.motion_may_exist = False
        self.best_effort_attempted = False
        self.sequence = 0
        self.last_stop = None
        self.feedback_sequence = 0
        self.feedback_epoch = str(uuid.uuid4())
        self.cancel_check = cancel_check or (lambda: False)
        self.stopping = False
        self.motion_deadline = None
        self.last_nonzero_write_time = None

    def require_not_cancelled(self):
        if not self.stopping and self.cancel_check():
            raise MotionRejected('operator signal/parent exit cancelled manual motion')

    def require_fresh_motion(self):
        if self.stopping:
            raise MotionRejected('nonzero commands are prohibited during stopping')
        self.require_not_cancelled()
        now = time.monotonic()
        if self.motion_deadline is None or now >= self.motion_deadline:
            raise MotionRejected('keyboard intent expired before serial write')
        if self.last_nonzero_write_time is not None and now-self.last_nonzero_write_time >= self.contract.watchdog_timeout:
            raise MotionRejected('controller command gap exceeded watchdog; new arm required')

    def _emit(self, data):
        self.journal({'schema': 'wc_manual_transaction_v1', 'sequence': self.sequence,
            'monotonic_ns': time.monotonic_ns(), 'device_id': 'ZLAC8030D-'+SERIAL,
            'stop_contract_sha256': self.contract.evidence_hash, **data})

    def _exchange(self, request):
        with self.lock:
            if self.lease.fd is None or self.failed:
                raise QueryFailure('manual serial channel is closed/failed')
            self.require_not_cancelled()
            if request[1] == 3:
                if request not in self.contract.reads:
                    raise FeedbackError('read outside reviewed state/feedback allowlist')
            else:
                self.contract.validate_write(request)
                if not self.armed:
                    raise FeedbackError('user arming is required before any velocity write')
            response = bytearray()
            sent = 0
            try:
                self._emit({'event': 'request_planned', 'request_hex': request.hex()})
                readable, _, _ = select.select([self.lease.fd], [], [], .002)
                if readable:
                    response.extend(os.read(self.lease.fd, 256))
                    raise QueryFailure('unexpected bytes; transaction association unsafe', response)
                deadline = time.monotonic()+self.contract.transaction_timeout
                _, writable, _ = select.select([], [self.lease.fd], [], max(0, deadline-time.monotonic()))
                if not writable:
                    raise QueryFailure('manual request readiness timeout')
                if request[1] == 0x10 and request != ZERO_REQUEST:
                    self.require_fresh_motion()  # Last check immediately before the actuator write.
                    self.motion_may_exist = True  # A partial transmission/ACK loss is not proof of no motion.
                    self.last_nonzero_write_time = time.monotonic()
                sent = os.write(self.lease.fd, request)
                if sent != len(request):
                    raise QueryFailure('partial manual request write', response, sent)
                expected = 8 if request[1] == 0x10 else 5+2*struct.unpack('>H', request[4:6])[0]
                while len(response) < expected:
                    remaining = deadline-time.monotonic()
                    if remaining <= 0:
                        raise QueryFailure('manual response timeout', response, sent)
                    readable, _, _ = select.select([self.lease.fd], [], [], remaining)
                    if not readable:
                        raise QueryFailure('manual response timeout', response, sent)
                    chunk = os.read(self.lease.fd, 256)
                    if not chunk:
                        raise QueryFailure('manual serial disconnected', response, sent)
                    response.extend(chunk)
                    if len(response) >= 2 and response[1] & 0x80:
                        expected = 5
                    if len(response) > expected:
                        raise QueryFailure('manual response overflow / overlapping traffic', response, sent)
                validate_ack(request, response)
                self._emit({'event': 'transaction_complete', 'request_hex': request.hex(),
                    'response_hex': bytes(response).hex(), 'transmitted_bytes': sent})
                self.sequence += 1
                return bytes(response)
            except BaseException as error:
                if not isinstance(error, MotionRejected) or sent:
                    self.failed = True
                try:
                    self._emit({'event': 'transaction_failed', 'request_hex': request.hex(),
                        'response_hex': bytes(response).hex(), 'transmitted_bytes': sent, 'error': str(error)})
                except Exception:
                    pass  # Logging failure must not prevent the caller's stop attempt.
                raise

    def read(self, request=QUERY):
        response = self._exchange(request)
        words = parse_exchange(request, response)[2]
        self.require_not_cancelled()  # A signal during a blocking read cannot lead to a later velocity write.
        if request == QUERY:
            self.journal({'schema': 'wc_wheel_feedback_v1', 'event': 'manual_feedback',
                'device_id': 'ZLAC8030D-'+SERIAL, 'stream_epoch': self.feedback_epoch,
                'sequence': self.feedback_sequence, 'request_hex': request.hex(), 'response_hex': response.hex(),
                'stamp_ns': time.time_ns(), 'receive_monotonic_ns': time.monotonic_ns(),
                'time_valid': False, 'time_source': 'arrival_only', 'uncertainty_ns': None,
                'status': 'RESPONSE_VALID', 'formal_odometry_eligible': False,
                'acquisition_context': 'user_manual_session', 'register_words_u16': list(words)})
            self.feedback_sequence += 1
        return words

    def velocity(self, linear, angular):
        return self._exchange(self.contract.velocity_request(linear, angular))

    def best_effort_zero(self):
        with self.lock:
            if not self.motion_may_exist or self.best_effort_attempted:
                return self.last_stop
            self.best_effort_attempted = True
            sent = 0
            error = None
            try:
                # This exact zero frame is the sole write allowed after failed
                # transaction association. Never retry a nonzero command.
                if self.lease.fd is None:
                    raise FeedbackError('serial already closed')
                _, writable, _ = select.select([], [self.lease.fd], [], self.contract.transaction_timeout)
                if not writable:
                    raise FeedbackError('stop write readiness timeout')
                sent = os.write(self.lease.fd, ZERO_REQUEST)
                if sent != len(ZERO_REQUEST):
                    raise FeedbackError('partial best-effort zero write')
            except Exception as problem:
                error = str(problem)
            self.last_stop = {'event': 'best_effort_stop', 'request_hex': ZERO_REQUEST.hex(),
                'transmitted_bytes': sent, 'error': error, 'acknowledged': False,
                'stationary_feedback_observed': False, 'physical_stop_confirmed': False,
                'operator_action': 'Use the verified physical stop; controller watchdog must handle host/link loss.'}
            try:
                self._emit(self.last_stop)
            except Exception:
                pass
            return self.last_stop


class ManualHardwareSession:
    def __init__(self, contract, channel, *, onsite_confirmation, journal=None):
        if onsite_confirmation != ONSITE_CONFIRMATION:
            raise FeedbackError('explicit on-site empty-chair/physical-stop confirmation required')
        self.contract, self.channel = contract, channel
        self.journal = journal or channel.journal
        self.state = 'DISARMED'
        self.keys = set()
        self.key_times = {}
        self.last_key_time = self.last_command_time = None
        self.last_now = None
        self.reason = 'STARTUP_READ_ONLY'
        self.closed = False

    def status(self):
        return {'schema': 'wc_manual_hardware_status_v1', 'state': self.state, 'reason': self.reason,
            'keys': sorted(self.keys), 'controller_enable_written': False, 'mode_written': False,
            'fault_clear_written': False, 'brake_release_written': False,
            'motion_may_exist': self.channel.motion_may_exist,
            'last_stop': self.channel.last_stop, 'physical_stop_confirmed': False,
            'stop_contract_sha256': self.contract.evidence_hash}

    def _clock(self, now):
        bounded(now, 'manual monotonic timestamp', 0, 1e15)
        current = time.monotonic()
        if now > current+.005 or current-now > self.contract.key_timeout:
            raise FeedbackError('manual key/tick timestamp is stale or future-dated')
        if self.last_now is not None and now < self.last_now:
            raise FeedbackError('manual monotonic clock reversed')
        self.last_now = now

    def _feedback(self):
        words = self.channel.read(QUERY)
        signed = tuple(value-65536 if value >= 32768 else value for value in words)
        bound = self.contract.profile['max_wheel_rpm']/self.contract.profile['register_to_feedback_rpm']
        if max(abs(value) for value in signed) > bound:
            raise FeedbackError('actual wheel feedback exceeds reviewed manual speed bound')
        return signed

    def _read_checks(self):
        replies = {}
        for check in self.contract.checks:
            request = bytes.fromhex(check['request_hex'])
            if request not in replies:
                replies[request] = self.channel.read(request)
            value = replies[request][check['word_index']]
            if value & check['mask'] != check['equals']:
                raise FeedbackError('required live controller state is not ready: '+check['purpose'])

    def preflight(self):
        if self.closed or self.state == 'FAULT' or self.channel.lease.fd is None:
            raise FeedbackError('open, nonfaulted serial owner required for manual preflight')
        self._read_checks()
        for index in range(3):
            if max(abs(value) for value in self._feedback()) > self.contract.stationary_bound:
                raise FeedbackError('stationary actual feedback required before user arm')
            if index < 2:
                time.sleep(.03)
        self.reason = 'READ_ONLY_PREFLIGHT_PASSED_DRIVE_ALREADY_READY'
        self.channel.require_not_cancelled()
        self.journal(self.status())

    def arm(self, *, operator_key):
        if operator_key != 'e' or self.state != 'DISARMED' or self.closed:
            raise FeedbackError('fresh explicit E arming is required in a disarmed live session')
        self.preflight()  # No enable write: required state must already be read back.
        self.channel.armed = True
        self.channel.last_nonzero_write_time = None
        self.state = 'ARMED'
        self.keys.clear()
        self.key_times.clear()
        self.last_key_time = time.monotonic()
        self.reason = 'USER_ARMED_NO_ENABLE_COMMAND'
        self.journal(self.status())

    def stop(self, reason, *, fault=False):
        self.keys.clear()
        self.key_times.clear()
        self.reason = reason
        self.state = 'FAULT' if fault or self.state == 'FAULT' else 'DISARMED'
        self.channel.stopping = True
        try:
            if self.channel.motion_may_exist:
                if self.channel.failed:
                    self.channel.best_effort_zero()
                    self.state = 'FAULT'
                else:
                    self.channel.velocity(0.0, 0.0)
                    deadline = time.monotonic()+self.contract.stop_timeout
                    stationary = 0
                    while time.monotonic() < deadline and stationary < 3:
                        stationary = stationary+1 if max(abs(value) for value in self._feedback()) <= self.contract.stationary_bound else 0
                        if stationary < 3:
                            time.sleep(.02)
                    self._read_checks()
                    if stationary != 3:
                        raise FeedbackError('zero command ACK without confirmed stationary feedback')
                    self.channel.motion_may_exist = False
                    self.channel.last_nonzero_write_time = None
                    self.channel.last_stop = {'event': 'zero_stop_observed', 'acknowledged': True,
                        'stationary_feedback_observed': True, 'physical_stop_confirmed': False}
        except Exception:
            self.state = 'FAULT'
            self.channel.failed = True
            self.channel.best_effort_zero()
        finally:
            self.channel.armed = False
            self.channel.motion_deadline = None
            self.channel.stopping = False
            try:
                self.journal(self.status())
            except Exception:
                self.state = 'FAULT'

    def key_event(self, key, pressed, now):
        try:
            self._clock(now)
            if self.closed:
                raise FeedbackError('manual hardware session is closed')
            self.channel.require_not_cancelled()
            if key == ' ':
                self.stop('USER_SPACE_STOP')
                return
            if key not in {'w', 'a', 's', 'd'} or type(pressed) is not bool:
                raise FeedbackError('explicit WASD event required')
            if self.state != 'ARMED':
                return
            if self.last_key_time is None or now-self.last_key_time > self.contract.key_timeout:
                self.stop('KEY_TIMEOUT_BEFORE_EVENT')
                return
            if pressed:
                self.keys.add(key)
                self.key_times[key] = now
            else:
                self.keys.discard(key)
                self.key_times.pop(key, None)
            self.last_key_time = now
            self.tick(now, force=True)
        except Exception:
            self.stop('KEY_EVENT_FAILURE', fault=True)
            raise

    def terminal_key(self, key, now):
        """TTY events replace the active direction; terminals have no key-up."""
        self.keys.clear()
        self.key_times.clear()
        self.key_event(key, True, now)

    def tick(self, now=None, *, force=False):
        now = time.monotonic() if now is None else now
        try:
            self._clock(now)
            self.channel.require_not_cancelled()
            if self.state != 'ARMED':
                return
            if self.last_key_time is None or now-self.last_key_time > self.contract.key_timeout:
                self.stop('KEY_TIMEOUT')
                return
            self.key_times = {key: stamp for key, stamp in self.key_times.items() if now-stamp < self.contract.key_timeout}
            self.keys = set(self.key_times)
            if not self.keys:
                if self.channel.motion_may_exist:
                    self.stop('ALL_KEYS_RELEASED')
                return
            if not force and self.last_command_time is not None and now-self.last_command_time < self.contract.period:
                return
            self._feedback()
            self._read_checks()
            # Reads can block. Do not dispatch an expired keyboard intent after
            # they return; firmware watchdog covers a stalled/terminated host.
            if time.monotonic()-self.last_key_time > self.contract.key_timeout:
                self.stop('KEY_EXPIRED_DURING_FEEDBACK')
                return
            self.channel.require_not_cancelled()
            self.channel.motion_deadline = min(self.key_times.values())+self.contract.key_timeout
            p = self.contract.profile
            linear = (int('w' in self.keys)-int('s' in self.keys))*p['max_linear_m_s']
            angular = (int('a' in self.keys)-int('d' in self.keys))*p['max_angular_rad_s']
            self.channel.velocity(linear, angular)
            self.last_command_time = time.monotonic()
        except Exception:
            self.stop('TRANSACTION_OR_STATE_FAILURE', fault=True)
            raise

    def close(self):
        try:
            self.stop('USER_EXIT', fault=self.state == 'FAULT')
        finally:
            self.channel.lease.close()
            self.closed = True


class MotionRejected(FeedbackError):
    """Motion was cancelled before a write; this does not desynchronize RTU."""


def _ros_bindings():
    # Import only after the explicit real-mode flag and live preflight pass.
    import rclpy
    from rclpy.context import Context
    from rclpy.signals import SignalHandlerOptions
    from std_msgs.msg import String
    return rclpy, Context(), SignalHandlerOptions.NO, String


class RosFeedbackPublisher:
    """Publish completed same-owner feedback; never commands, TF or odometry."""

    def __init__(self):
        self.ros, self.context, no_handlers, self.message_type = _ros_bindings()
        self.node = None
        self.failed = False
        self.closed = False
        try:
            self.ros.init(args=[], context=self.context, signal_handler_options=no_handlers)
            self.node = self.ros.create_node('wc_manual_wheel_feedback', context=self.context,
                start_parameter_services=False, enable_rosout=False)
            # Bounded KEEP_LAST, reliable delivery matches the wheel decoder.
            self.publisher = self.node.create_publisher(self.message_type,
                '/wc_mapping/wheel/feedback_raw', 20)
        except Exception:
            self.close()
            raise

    def publish_record(self, record):
        if self.failed or self.closed or record.get('schema') != 'wc_wheel_feedback_v1':
            return False
        try:
            if record.get('event') != 'manual_feedback' or record.get('status') != 'RESPONSE_VALID' or \
                    record.get('device_id') != 'ZLAC8030D-'+SERIAL or record.get('request_hex') != QUERY.hex():
                raise FeedbackError('only completed same-owner wheel feedback may be published')
            parse_exchange(QUERY, bytes.fromhex(record['response_hex']))
            self.publisher.publish(self.message_type(data=json.dumps(record, allow_nan=False)))
            return True
        except Exception:
            # The first failure aborts the active session; subsequent stop/read
            # journaling must still work so ROS failure cannot prevent stopping.
            self.failed = True
            raise

    def close(self):
        if self.closed:
            return
        self.closed = True
        try:
            if self.node is not None:
                self.node.destroy_node()
        finally:
            if self.context.ok():
                self.ros.shutdown(context=self.context, uninstall_handlers=False)


def terminal_event(descriptor):
    payload = os.read(descriptor, 64)
    if not payload:
        return 'q'
    if len(payload) != 1 or payload.lower() not in tuple(bytes((value,)) for value in b'ewasd q\x03'):
        raise FeedbackError('terminal escape, pasted batch or unrecognized input rejected')
    return payload.decode('ascii').lower()


def discard_terminal_backlog(descriptor):
    discarded = 0
    while select.select([descriptor], [], [], 0)[0]:
        payload = os.read(descriptor, 64)
        if not payload:
            return
        discarded += len(payload)
        if discarded > 1024:
            raise FeedbackError('terminal input flood; manual session stopped')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--mode', choices=('dry-run', 'real'), default='dry-run')
    parser.add_argument('--feedback-config', type=Path)
    parser.add_argument('--evidence', type=Path)
    parser.add_argument('--run-root', type=Path)
    parser.add_argument('--onsite-confirmation')
    parser.add_argument('--publish-ros', action='store_true',
        help='publish same-owner raw feedback after real-mode live preflight; no TF or odometry')
    parser.add_argument('--duration', type=float, default=60)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args(argv)
    bounded(args.duration, 'manual session duration', .1, 300)
    if args.mode == 'dry-run':
        if args.publish_ros:
            raise FeedbackError('--publish-ros requires gated real mode')
        from .manual_teleop import main as dry_run
        return dry_run(['--config', str(args.config.with_name('wheel_manual_unvalidated.json')),
            '--duration', str(args.duration), '--output', str(args.output)])
    if args.onsite_confirmation != ONSITE_CONFIRMATION:
        raise FeedbackError('real mode requires explicit on-site operator confirmation')
    if args.evidence is None or args.run_root is None or not args.run_root.is_absolute():
        raise FeedbackError('real mode requires reviewed evidence and an absolute shared run root')
    config = json.loads(args.config.read_text(encoding='utf-8'))
    contract = ManualContract(config, json.loads(args.evidence.read_text(encoding='utf-8')), args.run_root.parent)
    feedback_path = args.feedback_config or args.config.with_name('wheel_feedback_current.json')
    feedback_config = validate_config(json.loads(feedback_path.read_text(encoding='utf-8')))
    if args.run_root.resolve() != Path('/home/nvidia/wheelchair/.phase1_runtime') or \
            getpass.getuser() != 'nvidia' or socket.gethostname() != 'ubuntu' or platform.machine() != 'aarch64':
        raise FeedbackError('real manual mode requires the confirmed Orin project/identity')
    if not sys.stdin.isatty() or sys.platform != 'linux':
        raise FeedbackError('real manual entry requires the user interactive Linux terminal')
    stopped = False

    def stop_signal(signum, frame):
        nonlocal stopped
        stopped = True

    signal.signal(signal.SIGINT, stop_signal)
    signal.signal(signal.SIGTERM, stop_signal)
    signal.signal(signal.SIGHUP, stop_signal)
    expected_parent = os.getppid()
    libc = ctypes.CDLL(None, use_errno=True)
    if libc.prctl(1, signal.SIGTERM, 0, 0, 0) != 0:
        raise OSError(ctypes.get_errno(), 'manual parent-death signal setup failed')
    if os.getppid() != expected_parent or stopped:
        raise FeedbackError('manual parent changed before device acquisition')
    lease = FeedbackSerialLease(feedback_config, args.run_root)
    session = None
    ros_output = None
    previous = None
    with args.output.open('x', encoding='utf-8') as output:
        def journal(record):
            output.write(json.dumps(record, allow_nan=False)+'\n')
            output.flush()
            if ros_output is not None:
                ros_output.publish_record(record)
        try:
            if stopped:
                raise FeedbackError('manual session stopped before device acquisition')
            lease.open()
            channel = ManualRtuChannel(lease, contract, journal,
                cancel_check=lambda: stopped or os.getppid() != expected_parent)
            session = ManualHardwareSession(contract, channel, onsite_confirmation=args.onsite_confirmation, journal=journal)
            session.preflight()
            channel.require_not_cancelled()
            if args.publish_ros:
                ros_output = RosFeedbackPublisher()
            import termios
            import tty
            previous = termios.tcgetattr(sys.stdin.fileno())
            tty.setcbreak(sys.stdin.fileno())
            discard_terminal_backlog(sys.stdin.fileno())
            print('USER MANUAL SESSION, DISARMED. E: verify/arm; WASD: bounded keyboard intent; Space: stop; Q: stop/exit.', flush=True)
            print('No mode/enable/fault/brake commands. Key input expires; controller watchdog and verified physical stop remain necessary.', flush=True)
            started = time.monotonic()
            while not stopped and time.monotonic()-started < args.duration:
                descriptor = sys.stdin.fileno()
                previous_state = session.state
                readable, _, _ = select.select([descriptor], [], [], .02)
                if readable:
                    key = terminal_event(descriptor)
                    if key in ('', 'q', '\x03'):
                        break
                    if key == 'e':
                        session.arm(operator_key='e')
                        discard_terminal_backlog(descriptor)
                    elif key in 'wasd ':
                        session.terminal_key(key, time.monotonic())
                        if key == ' ':
                            discard_terminal_backlog(descriptor)
                session.tick()
                if previous_state == 'ARMED' and session.state != 'ARMED':
                    discard_terminal_backlog(descriptor)
                if session.state == 'FAULT':
                    break
        finally:
            try:
                if session is not None:
                    session.close()
                else:
                    lease.close()
            finally:
                try:
                    if ros_output is not None:
                        ros_output.close()
                finally:
                    if previous is not None:
                        termios.tcsetattr(sys.stdin.fileno(), termios.TCSANOW, previous)
                    output.flush()
                    os.fsync(output.fileno())
    return 1 if session is not None and session.state == 'FAULT' else 0


if __name__ == '__main__':
    raise SystemExit(main())
