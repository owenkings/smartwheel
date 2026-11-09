"""Automatic control source of the single wheel serial owner (docs/architecture/velocity_channel.md, sections 3-6).

Default: absent. ``mapping_wheel.main`` only calls ``attach_auto`` when the session's frozen configuration carries
``auto_controls``; the capture and mapping configuration builders never write it. Even then automatic control is
UNAVAILABLE, and no ``auto.sock`` is created, unless all of these hold:

- ``auto_controls.allowed`` is true and today's user approval ``WC_AUTO_DRIVE_APPROVAL=APPROVED_BY_USER_<YYYYMMDD>``
  is set;
- the reviewed evidence file validates for this controller (``wc_motion.auto_evidence``);
- a reader of the hold-to-run switch and the emergency stop exists;
- the driver parameters listed in the evidence can be read back.

In this revision (level 0) the vehicle has neither the switch reader nor the driver readback, so automatic control
only runs against the fake serial channel of the tests.

While automatic control is active (or stopping) the manual state machine of ``MappingWheel`` is frozen: manual
packets are intercepted here, where any key, Space or hand-push request ends automatic control, and
``MappingWheel.step`` returns right after the feedback read. Every change of control source passes through a written
zero and a confirmed standstill. Nothing here releases the servo for hand pushing.
"""
import datetime
import math
import os
from pathlib import Path
import select
import socket
import stat
import struct

from wc_motion.auto_evidence import HARD_LIMITS, read_evidence_file
from wc_motion.auto_protocol import (DECISION_STALE_S, GATE_ACTIVE, GATE_ESTOP, MAX_LINE_BYTES, STATUS_PERIOD_S,
                                     STATUS_SCHEMA, ProtocolError, check_hello, decode_line, encode_line,
                                     parse_decision)
from .mapping_wheel import (CONTROL_PERIOD_S, FEEDBACK_FRESH_S, INITIALIZATION_WRITES, ZERO_REQUEST, Cancelled,
                            FeedbackError, speed_write)

APPROVAL_ENV = 'WC_AUTO_DRIVE_APPROVAL'
AUTO_SOCKET_NAME = 'auto.sock'
LINEAR_TOLERANCE_M_S, ANGULAR_TOLERANCE_RAD_S = 0.005, 0.01   # beyond the limit by more: the chain is broken
ENTRY_STATIONARY_SAMPLES = 5      # 0.5 s of zero wheel feedback before automatic control starts
EXIT_STATIONARY_SAMPLES = 3       # same rule as the manual stop
STOP_CONFIRM_S = 1.0              # zero written and still not standing still after this: FAULT
SWITCH_FRESH_S = 0.2
BACKWARD_M_S, BACKWARD_S = -0.02, 0.2
MOVING_AT_ZERO_S = 1.0
ACTIVE_STATES = ('INITIALIZING', 'AUTO', 'STOPPING')
# Exit reasons that write zero at once instead of decelerating at the software limit.
IMMEDIATE = frozenset({'NO_SWITCH_INPUT', 'SWITCH_INPUT_STALE', 'ESTOP', 'DEAD_MAN_RELEASED', 'UI_NOT_FRESH',
                       'UI_CLOSED', 'USER_STOP', 'GATE_ESTOP_LATCHED', 'MOVING_BACKWARD', 'MANUAL_LAYER_STOP'})


class AutoFault(FeedbackError):
    """Raised from the owner's step: its existing fault path latches FAULT and writes zero."""


def auto_socket_path(project_root, directory, session_id):
    """Next to the session's manual socket, in the same UID-owned 0700 directory."""
    from .mapping_control_paths import manual_socket_path
    return manual_socket_path(project_root, directory, session_id).with_name(AUTO_SOCKET_NAME)


def _limit(value, name, cap):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or not 0 < value <= cap:
        raise FeedbackError('auto_controls.%s must be a finite number in (0, %s]' % (name, cap))
    return float(value)


def validate_auto_controls(value):
    """``auto_controls`` of a session configuration: explicit, bounded, and off unless ``allowed`` is true."""
    if not isinstance(value, dict) or set(value) - ({'allowed', 'evidence', 'note'} | set(HARD_LIMITS)):
        raise FeedbackError('auto_controls must be an object with allowed, evidence and the limits only')
    if type(value.get('allowed', False)) is not bool:
        raise FeedbackError('auto_controls.allowed must be a boolean')
    result = {'allowed': value.get('allowed', False), 'evidence': value.get('evidence'), 'note': value.get('note')}
    for name, cap in HARD_LIMITS.items():
        if name in value or result['allowed']:
            result[name] = _limit(value.get(name), name, cap)
    if result['allowed'] and (not isinstance(result['evidence'], str) or not result['evidence']):
        raise FeedbackError('auto_controls.evidence must name the reviewed evidence file')
    return result


def check_approval(environ, today):
    """None when today's explicit user approval is present, otherwise the refusal reason."""
    if environ.get(APPROVAL_ENV) != 'APPROVED_BY_USER_' + today.strftime('%Y%m%d'):
        return 'NO_USER_APPROVAL_TODAY'
    return None


def limits_consistent(limits, wheel):
    """The approved linear and angular limits together must fit under the approved wheel rpm."""
    edge = limits['max_linear_m_s'] + limits['max_angular_rad_s'] * wheel['track_width_m'] / 2
    return edge * 60 / (2 * math.pi * wheel['wheel_radius_m']) <= limits['max_wheel_rpm'] + 1e-9


class AutoDrive:
    """Automatic control source inside the wheel owner; ``owner`` is the session's ``MappingWheel``."""

    def __init__(self, owner, limits, *, switches=None, readback=None, readback_checks=(), unavailable=None,
                 evidence=None):
        wheel = owner.wheel
        for key in ('wheel_radius_m', 'track_width_m', 'register_to_wheel_rpm', 'left_sign', 'right_sign'):
            if key not in wheel:
                raise FeedbackError('wheel candidate lacks ' + key + ' for automatic control')
        if unavailable is None and not (owner.hybrid_manual and owner.control['arm_allowed']):
            unavailable = 'NEEDS_AUTHORISED_HYBRID_MANUAL_SESSION'
        self.owner = owner
        self.limits = {name: float(limits[name]) for name in HARD_LIMITS}
        self.switches, self.readback, self.readback_checks = switches, readback, list(readback_checks)
        self.evidence = dict(evidence or {})
        self.unavailable = unavailable
        self.state = 'UNAVAILABLE' if unavailable else 'NONE'
        self.reason = unavailable or 'IDLE'
        self.decision = self.decision_time = None
        self.last_decision_sequence = 0
        self.bridge_connected, self.bridge_id = False, None
        self.entry_generation = None
        self.command, self.last_words, self.last_write = (0.0, 0.0), (0, 0), None
        self.immediate, self.zero_reached, self.zero_since = False, None, None
        self.init_step, self.next_init = 0, 0.0
        self.seen_sequence, self.stationary, self.backward_since, self.feedback = owner.sequence, 0, None, None
        self.settled = 0   # consecutive zero samples taken after the zero command of the current stop
        self.takeover_latch = False
        self.counters = {name: 0 for name in ('decisions', 'decisions_rejected', 'writes', 'zero_writes',
                                              'cancelled_writes', 'entries', 'refusals', 'exits', 'violations')}
        owner.auto = self

    # ---------------------------------------------------------------- bridge side
    def on_bridge_connected(self, bridge_id):
        self.bridge_connected, self.bridge_id = True, bridge_id
        self.decision = self.decision_time = None
        self.last_decision_sequence = 0
        self._log({'event': 'auto_bridge_connected', 'bridge_id': bridge_id}, required=False)

    def on_bridge_lost(self, reason, *, closing=False):
        self.bridge_connected = False
        self.decision = self.decision_time = None
        self._log({'event': 'auto_bridge_lost', 'reason': reason}, required=False)
        if self.state in ACTIVE_STATES:
            if closing:
                # Session exit: MappingWheel.close() writes zero right after this.
                self._set('NONE', 'SESSION_EXIT_ZERO_BY_OWNER_CLOSE')
            else:
                self._begin_stop('BRIDGE_DISCONNECTED', immediate=False)

    def on_decision(self, value):
        try:
            decision = parse_decision(value, self.owner.session_id, self.last_decision_sequence)
        except ProtocolError:
            self.counters['decisions_rejected'] += 1
            raise
        self.last_decision_sequence = decision['sequence']
        self.decision, self.decision_time = decision, self.owner.clock()
        self.counters['decisions'] += 1

    # ---------------------------------------------------------------- manual side (MappingWheel.command)
    def intercept(self, value):
        """True when the manual packet is handled here and must not reach the manual state machine."""
        owner = self.owner
        kind = value.get('type')
        if kind == 'auto':
            self._auto_packet(value)
            return True
        active = self.state in ACTIVE_STATES
        if kind == 'keys' and (active or self.takeover_latch):
            sequence, keys = value.get('sequence'), value.get('keys')
            if type(sequence) is not int or sequence <= owner.last_key_sequence or not isinstance(keys, list) or \
                    len(keys) > 4 or len(set(keys)) != len(keys) or any(key not in ('w', 'a', 's', 'd') for key in keys) \
                    or type(value.get('foreground')) is not bool:
                raise FeedbackError('keys require unique WASD values, an increasing sequence and foreground')
            owner.last_key_sequence = sequence
            if value['foreground']:
                owner.foreground, owner.last_ui_heartbeat = True, owner.clock()
            elif active:
                self._begin_stop('UI_NOT_FRESH', immediate=True)
            if keys:
                if active:
                    self._begin_stop('MANUAL_TAKEOVER', immediate=False)
                self.takeover_latch = True   # the takeover key only stops; release and press again to drive
            elif not active:
                self.takeover_latch = False
            return True
        if active:
            if kind in ('disarm', 'close'):
                self._begin_stop('USER_STOP' if kind == 'disarm' else 'UI_CLOSED', immediate=True)
            elif kind == 'push_mode':
                if type(value.get('sequence')) is int and value['sequence'] > owner.last_key_sequence:
                    owner.last_key_sequence = value['sequence']
                self._begin_stop('USER_HAND_PUSH_REQUEST', immediate=False)
            return True
        return False

    def _auto_packet(self, value):
        owner = self.owner
        sequence = value.get('sequence')
        if type(sequence) is not int or sequence <= owner.last_key_sequence or type(value.get('enabled')) is not bool \
                or type(value.get('arm_generation')) is not int or type(value.get('foreground')) is not bool:
            raise FeedbackError('auto request requires an increasing sequence, enabled, arm_generation and foreground')
        owner.last_key_sequence = sequence
        if value['foreground']:
            owner.foreground, owner.last_ui_heartbeat = True, owner.clock()
        if not value['enabled']:
            if self.state in ACTIVE_STATES:
                self._begin_stop('USER_STOPPED_AUTO', immediate=False)
            return
        if self.state == 'UNAVAILABLE':
            self._refuse(self.unavailable)
        elif value['arm_generation'] != owner.arm_generation:
            self._refuse('STALE_UI_GENERATION')
        else:
            self._try_start(owner.clock())

    # ---------------------------------------------------------------- owner hooks
    def on_fault(self, reason):
        if self.state != 'UNAVAILABLE':
            self.state, self.reason = 'FAULT', 'OWNER_FAULT: ' + str(reason)

    def step(self, now):
        """Called by MappingWheel.step after the feedback read; True while automatic control owns the tick."""
        self._observe()
        if self.state not in ACTIVE_STATES:
            return False
        try:
            return self._step_active(now)
        except AutoFault:
            self._zero_now()   # the owner's fault path may skip its own zero while a manual stop is pending
            raise

    def _zero_now(self):
        owner = self.owner
        try:
            owner.channel.exchange(ZERO_REQUEST, owner.timeout, guard=lambda: None)
            self.command = (0.0, 0.0)
        except Exception:  # noqa: BLE001 - the owner's fault path follows; never retry a nonzero command
            owner.channel.best_effort_zero()

    def _step_active(self, now):
        owner = self.owner
        if owner.arm_generation != self.entry_generation:
            # The manual layer disarmed (interface lost, session cancelled, socket rejected) and wrote zero itself.
            self.entry_generation = owner.arm_generation
            self._begin_stop('MANUAL_LAYER_STOP', immediate=True)
        if self.state == 'INITIALIZING':
            reason = self._exit_reason(now)
            if reason is None:
                self._initialize(now)
                return True
            self._begin_stop(reason, immediate=True)
        elif self.state == 'AUTO':
            reason = self._exit_reason(now)
            if reason is not None:
                self._begin_stop(reason, immediate=reason in IMMEDIATE)
        if self.state == 'AUTO':
            if self.command == (0.0, 0.0) and self.zero_since is not None and now - self.zero_since > MOVING_AT_ZERO_S \
                    and self.stationary == 0:
                raise AutoFault('MOVING_WHILE_COMMANDED_ZERO')
            self._drive(now, self._target())
        elif self.state == 'STOPPING':
            self._drive(now, (0.0, 0.0))
            self._finish_or_fault(now)
        return True

    # ---------------------------------------------------------------- entry
    def _try_start(self, now):
        blockers = self._entry_blockers(now)
        if blockers:
            self._refuse(blockers[0])
            return
        for check in self.readback_checks:
            words = self.readback(check['request'])
            if words[check['word_index']] & check['mask'] != check['equals']:
                self._refuse('DRIVER_READBACK_MISMATCH:' + check['purpose'])
                return
        owner = self.owner
        self.entry_generation = owner.arm_generation
        self.command, self.last_write, self.zero_since = (0.0, 0.0), None, None
        self.immediate, self.zero_reached = False, None
        owner.hybrid_release_inhibited = True
        owner.stop_deadline, owner.stop_zero_samples = None, 0
        self.counters['entries'] += 1
        if owner.push_mode or not owner.initialized_by_user:
            self.init_step, self.next_init = 0, now
            self._set('INITIALIZING', 'AUTO_INITIALIZING')
        else:
            self._set('AUTO', 'AUTO_STARTED')

    def _entry_blockers(self, now):
        owner, out = self.owner, []
        if self.state != 'NONE':
            out.append('AUTO_NOT_IDLE_' + self.state)
        if owner.state != 'READY':
            out.append('MANUAL_STATE_' + owner.state)
        if owner.closing or owner.closed or owner.cancelled():
            out.append('SESSION_CLOSING')
        if owner.keys or self.takeover_latch:
            out.append('MANUAL_KEYS_HELD')
        if owner.push_mode_requested:
            out.append('HAND_PUSH_PENDING')
        if not owner._ui_fresh():
            out.append('UI_NOT_FRESH')
        problem = self._switch_problem(now, entry=True)
        if problem:
            out.append(problem)
        if not self.bridge_connected:
            out.append('BRIDGE_DISCONNECTED')
        elif self._decision_age(now) > DECISION_STALE_S:
            out.append('DECISION_STALE')
        elif not self._gate_limits_ok():
            out.append('GATE_LIMITS_ABOVE_APPROVED')
        if self.stationary < ENTRY_STATIONARY_SAMPLES or not self._feedback_fresh(now):
            out.append('NOT_STATIONARY')
        if self.readback is None:
            out.append('DRIVER_READBACK_UNAVAILABLE')
        return out

    def _initialize(self, now):
        """The reviewed manual initialization (mode, flags, clear, zero, enable) with the target held at zero."""
        if now < self.next_init:
            return
        sequence = ((INITIALIZATION_WRITES[0], 0.0), (INITIALIZATION_WRITES[1], 0.0), (INITIALIZATION_WRITES[2], 0.2),
                    (ZERO_REQUEST, 0.0), (INITIALIZATION_WRITES[3], 0.2))
        owner = self.owner
        if self.init_step < len(sequence):
            request, delay = sequence[self.init_step]
            generation = owner.arm_generation
            try:
                owner.channel.exchange(request, owner.timeout, guard=lambda: self._guard('INITIALIZING', generation))
            except Cancelled:
                self.counters['cancelled_writes'] += 1
                return
            self.init_step += 1
            self.next_init = owner.clock() + delay
            return
        owner.initialized_by_user, owner.servo_release_acknowledged = True, False
        owner.push_mode = owner.push_mode_requested = False
        owner.stop_deadline = None
        self.command, self.last_write = (0.0, 0.0), None
        self._set('AUTO', 'AUTO_STARTED')

    # ---------------------------------------------------------------- running and stopping
    def _exit_reason(self, now):
        owner = self.owner
        problem = self._switch_problem(now)
        if problem:
            return problem
        if not owner._ui_fresh():
            return 'UI_NOT_FRESH'
        if not self.bridge_connected:
            return 'BRIDGE_DISCONNECTED'
        if self._decision_age(now) > DECISION_STALE_S:
            return 'DECISION_STALE'
        decision = self.decision
        if decision['state'] == GATE_ESTOP:
            return 'GATE_ESTOP_LATCHED'
        if not self._gate_limits_ok():
            return 'GATE_LIMITS_ABOVE_APPROVED'
        violation = self._violation(decision)
        if violation:
            return violation
        if not self._feedback_fresh(now):
            return 'FEEDBACK_STALE'
        if self.backward_since is not None and now - self.backward_since > BACKWARD_S:
            return 'MOVING_BACKWARD'
        return None

    def _violation(self, decision):
        linear, angular = decision['linear'], decision['angular']
        reason = None
        if decision['state'] != GATE_ACTIVE and (linear != 0.0 or angular != 0.0):
            reason = 'GATE_NONZERO_WHILE_NOT_ACTIVE'
        elif linear < 0.0:
            reason = 'NEGATIVE_LINEAR'
        elif linear > self.limits['max_linear_m_s'] + LINEAR_TOLERANCE_M_S:
            reason = 'LINEAR_ABOVE_LIMIT'
        elif abs(angular) > self.limits['max_angular_rad_s'] + ANGULAR_TOLERANCE_RAD_S:
            reason = 'ANGULAR_ABOVE_LIMIT'
        if reason:
            self.counters['violations'] += 1
        return reason

    def _target(self):
        decision = self.decision
        if decision['state'] != GATE_ACTIVE:
            return 0.0, 0.0
        limit = self.limits['max_angular_rad_s']
        return min(decision['linear'], self.limits['max_linear_m_s']), max(-limit, min(limit, decision['angular']))

    def _drive(self, now, target):
        if self.last_write is not None and now - self.last_write < CONTROL_PERIOD_S - 1e-9:
            return
        dt = CONTROL_PERIOD_S if self.last_write is None else min(now - self.last_write, 0.1)
        linear, angular = self.command
        if self.state == 'STOPPING' and self.immediate:
            linear = angular = 0.0
        else:
            up, down = self.limits['max_linear_accel_m_s2'] * dt, self.limits['max_linear_decel_m_s2'] * dt
            linear = min(target[0], linear + up) if target[0] > linear else max(target[0], linear - down)
            turn = self.limits['max_angular_accel_rad_s2'] * dt
            angular += max(-turn, min(turn, target[1] - angular))
            linear, angular = (0.0 if abs(linear) < 1e-9 else linear), (0.0 if abs(angular) < 1e-9 else angular)
        request, words = self._encode(linear, angular)
        owner = self.owner
        zero = (linear, angular) == (0.0, 0.0)
        expected, generation = self.state, owner.arm_generation
        guard = (lambda: None) if zero else (lambda: self._guard(expected, generation))
        try:
            owner.channel.exchange(request, owner.timeout, guard=guard)
        except Cancelled:
            self.counters['cancelled_writes'] += 1
            return
        self.command, self.last_words, self.last_write = (linear, angular), words, owner.clock()
        self.counters['zero_writes' if zero else 'writes'] += 1
        self.zero_since = (self.zero_since or self.last_write) if zero else None

    def _guard(self, expected, generation):
        """Right before a nonzero write: pump the sockets, then recheck everything the write depends on.

        While stopping, a deceleration step is only cancelled when the stop turned immediate (the next tick writes
        zero); the exit condition that started the stop must not block the deceleration itself."""
        owner = self.owner
        owner.before_control()
        if self.state != expected or owner.arm_generation != generation:
            raise Cancelled('automatic control changed before write')
        if expected == 'STOPPING':
            if self.immediate:
                raise Cancelled('stop became immediate before write')
        elif self._exit_reason(owner.clock()) is not None:
            raise Cancelled('automatic exit condition before write')

    def _encode(self, linear, angular):
        wheel = self.owner.wheel
        half = wheel['track_width_m'] / 2
        rpm = [(linear + side * angular * half) * 60 / (2 * math.pi * wheel['wheel_radius_m']) for side in (-1, 1)]
        if max(abs(value) for value in rpm) > self.limits['max_wheel_rpm'] + 1e-9:
            raise AutoFault('WHEEL_RPM_ABOVE_LIMIT')
        scale = self.owner.control['command_rpm_to_register_scale']
        # Truncate toward zero: register rounding never takes a wheel above the approved speed.
        words = tuple(int(rpm[i] * wheel[sign] * scale) for i, sign in enumerate(('left_sign', 'right_sign')))
        if max(abs(value) for value in words) > 32767:
            raise AutoFault('WHEEL_COMMAND_REGISTER_OVERFLOW')
        return speed_write(*words), words

    def _begin_stop(self, reason, *, immediate):
        if self.state == 'STOPPING':
            self.immediate = self.immediate or immediate
            return
        self.counters['exits'] += 1
        self.immediate, self.zero_reached, self.settled = immediate, None, 0
        self._set('STOPPING', reason)

    def _finish_or_fault(self, now):
        if self.command != (0.0, 0.0) or self.last_write is None:
            return
        if self.zero_reached is None:
            self.zero_reached, self.settled = self.last_write, 0
        if self.settled >= EXIT_STATIONARY_SAMPLES and self._feedback_fresh(now):
            owner = self.owner
            self._set('NONE', 'AUTO_EXITED: ' + self.reason)
            self.command, self.last_write, self.zero_since = (0.0, 0.0), None, None
            owner.hybrid_release_inhibited = True
            owner.disarm('AUTO_EXITED')   # manual state back to READY; no servo release follows
        elif now - self.zero_reached > STOP_CONFIRM_S:
            raise AutoFault('NOT_STATIONARY_AFTER_STOP')

    # ---------------------------------------------------------------- inputs
    def _observe(self):
        owner = self.owner
        if owner.sequence == self.seen_sequence or not owner.samples:
            return
        self.seen_sequence = owner.sequence
        stamp, words = owner.samples[-1]
        zero = list(words) == [0, 0]
        self.stationary = self.stationary + 1 if zero else 0
        if self.zero_reached is not None and stamp > self.zero_reached:
            self.settled = self.settled + 1 if zero else 0
        wheel = owner.wheel
        scale = wheel['register_to_wheel_rpm'] * 2 * math.pi * wheel['wheel_radius_m'] / 60
        signed = [value - 65536 if value >= 32768 else value for value in words]
        left, right = signed[0] * scale * wheel['left_sign'], signed[1] * scale * wheel['right_sign']
        self.feedback = {'left_m_s': left, 'right_m_s': right, 'linear_m_s': (left + right) / 2, 'stamp': stamp}
        if self.feedback['linear_m_s'] < BACKWARD_M_S:
            self.backward_since = stamp if self.backward_since is None else self.backward_since
        else:
            self.backward_since = None

    def _feedback_fresh(self, now):
        last = self.owner.last_feedback
        return last is not None and 0 <= now - last <= FEEDBACK_FRESH_S

    def _decision_age(self, now):
        return math.inf if self.decision_time is None else now - self.decision_time

    def _gate_limits_ok(self):
        limits = self.decision['limits']
        return limits['max_linear'] <= self.limits['max_linear_m_s'] + 1e-9 and \
            limits['max_angular'] <= self.limits['max_angular_rad_s'] + 1e-9

    def _switch_values(self, now):
        if self.switches is None:
            return None
        value = self.switches()
        if not isinstance(value, dict) or type(value.get('dead_man')) is not bool or \
                type(value.get('estop')) is not bool:
            return None
        stamp = value.get('stamp')
        if isinstance(stamp, bool) or not isinstance(stamp, (int, float)) or not math.isfinite(stamp) or \
                not -0.05 <= now - stamp <= SWITCH_FRESH_S:
            return None
        return value

    def _switch_problem(self, now, *, entry=False):
        if self.switches is None:
            return 'NO_SWITCH_INPUT'
        value = self._switch_values(now)
        if value is None:
            return 'SWITCH_INPUT_STALE'
        if value['estop']:
            return 'ESTOP'
        if not value['dead_man']:
            return 'DEAD_MAN_NOT_HELD' if entry else 'DEAD_MAN_RELEASED'
        return None

    # ---------------------------------------------------------------- records
    def _log(self, record, *, required=True):
        self.owner._log(record, required=required)

    def _set(self, state, reason):
        self.state, self.reason = state, reason
        self._log({'event': 'auto_state', 'state': state, 'reason': reason, 'command': list(self.command),
                   'decision_sequence': self.last_decision_sequence, 'physical_stop_confirmed': False})

    def _refuse(self, reason):
        self.counters['refusals'] += 1
        self.reason = 'AUTO_REFUSED: ' + reason
        self._log({'event': 'auto_refused', 'reason': reason}, required=False)

    def status(self):
        owner, now = self.owner, self.owner.clock()
        switches = self._switch_values(now)
        decision = self.decision
        manual = owner.state in ('INITIALIZING', 'ARMED')
        source = 'AUTO' if self.state in ACTIVE_STATES else ('MANUAL' if manual else 'NONE')
        feedback = None
        if self.feedback is not None:
            feedback = {key: value for key, value in self.feedback.items() if key != 'stamp'}
            feedback['age_s'] = max(0.0, now - self.feedback['stamp'])
        return {'type': 'status', 'schema': STATUS_SCHEMA, 'session_id': owner.session_id, 'source': source,
                'auto_state': self.state, 'reason': self.reason, 'limits': dict(self.limits),
                'command': {'linear': self.command[0], 'angular': self.command[1], 'words': list(self.last_words)},
                'feedback': feedback, 'stationary_samples': self.stationary,
                'switch_input': self.switches is not None,
                'dead_man': None if switches is None else switches['dead_man'],
                'estop': None if switches is None else switches['estop'],
                'bridge_connected': self.bridge_connected, 'bridge_id': self.bridge_id,
                'decision_age_s': None if decision is None else round(self._decision_age(now), 3),
                'gate_state': None if decision is None else decision['state'],
                'gate_seq': None if decision is None else decision['gate_seq'],
                'takeover_latched': self.takeover_latch, 'manual_state': owner.state, 'push_mode': owner.push_mode,
                'evidence_stage': self.evidence.get('stage'), 'evidence_sha256': self.evidence.get('evidence_sha256'),
                'counters': dict(self.counters), 'physical_stop_confirmed': False}


class AutoSocket:
    """The base bridge's local endpoint: one same-UID client, hello first, then decisions; status every 0.1 s."""

    def __init__(self, path, auto):
        self.path, self.auto = Path(path), auto
        self.clock = auto.owner.clock
        self.listener = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self.client, self.ready, self.accepted_at, self.last_status = None, False, None, 0.0
        self.received, self.pending = bytearray(), bytearray()
        self.identity = None
        try:
            self.listener.bind(str(self.path))   # never removes or reuses an existing socket
            os.chmod(self.path, 0o600)
            info = self.path.stat()
            self.identity = (info.st_dev, info.st_ino)
            self.listener.listen(1)
            self.listener.setblocking(False)
        except BaseException:
            self.listener.close()
            raise

    def drop(self, reason, *, closing=False):
        if self.client is not None:
            self.client.close()
            self.client = None
        if self.ready:
            self.auto.on_bridge_lost(reason, closing=closing)
        self.ready = False
        self.received.clear()
        self.pending.clear()

    def _queue_status(self):
        payload = encode_line(self.auto.status())
        if len(self.pending) + len(payload) > 16384:
            raise ProtocolError('base bridge is not consuming status')
        self.pending.extend(payload)
        self.last_status = self.clock()

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
                            client.close()
                            return
                    client.setblocking(False)
                    self.client, self.accepted_at = client, self.clock()
            if self.client is None:
                return
            if not self.ready and self.clock() - self.accepted_at > 1.0:
                raise ProtocolError('base bridge hello timeout')
            if select.select([self.client], [], [], 0)[0]:
                data = self.client.recv(8193)
                if not data:
                    self.drop('BRIDGE_DISCONNECTED')
                    return
                self.received.extend(data)
                if len(self.received) > 8192:
                    raise ProtocolError('base bridge input buffer overflow')
                count = 0
                while b'\n' in self.received:
                    line, _, rest = self.received.partition(b'\n')
                    self.received[:] = rest
                    count += 1
                    if count > 16 or not line or len(line) > MAX_LINE_BYTES:
                        raise ProtocolError('base bridge line or flood bound')
                    value = decode_line(bytes(line))
                    if not self.ready:
                        bridge_id = check_hello(value, self.auto.owner.session_id)
                        self.ready = True
                        self.auto.on_bridge_connected(bridge_id)
                        self._queue_status()
                    else:
                        self.auto.on_decision(value)
                if len(self.received) > MAX_LINE_BYTES:
                    raise ProtocolError('unterminated base bridge line exceeds bound')
            if self.ready and self.clock() - self.last_status >= STATUS_PERIOD_S:
                self._queue_status()
            if self.pending and select.select([], [self.client], [], 0)[1]:
                sent = self.client.send(self.pending)
                if sent <= 0:
                    self.drop('BRIDGE_DISCONNECTED')
                    return
                del self.pending[:sent]
        except (ProtocolError, FeedbackError, ValueError, TypeError, UnicodeError, OSError) as error:
            self.drop('BRIDGE_REJECTED: ' + str(error))

    def close(self):
        self.drop('SESSION_EXIT', closing=True)
        self.listener.close()
        try:
            value = self.path.lstat()
            if stat.S_ISSOCK(value.st_mode) and (value.st_dev, value.st_ino) == self.identity:
                self.path.unlink()
        except FileNotFoundError:
            pass


def attach_auto(config, owner, project_root, session_dir, *, environ=None, today=None, switches=None, readback=None,
                socket_path=None):
    """Attach the automatic control source to ``owner``; returns its socket, or None while it is unavailable."""
    controls = validate_auto_controls(config.get('auto_controls'))
    environ = os.environ if environ is None else environ
    today = today or datetime.date.today()
    unavailable, evidence = None, {}
    if not controls['allowed']:
        unavailable = 'AUTO_NOT_ALLOWED'
    else:
        unavailable = check_approval(environ, today)
        if unavailable is None:
            try:
                evidence = read_evidence_file(controls['evidence'], project_root, config['wheel_device_id'])
            except FeedbackError as error:
                unavailable = 'EVIDENCE_REJECTED: ' + str(error)
    approved = evidence.get('limits', HARD_LIMITS)
    limits = {name: min(HARD_LIMITS[name], approved[name], controls.get(name, 0.0)) for name in HARD_LIMITS}
    if unavailable is None and not limits_consistent(limits, owner.wheel):
        unavailable = 'LIMITS_INCONSISTENT'
    if unavailable is None and switches is None:
        unavailable = 'NO_SWITCH_INPUT'
    if unavailable is None and readback is None:
        unavailable = 'DRIVER_READBACK_NOT_IMPLEMENTED'
    auto = AutoDrive(owner, limits, switches=switches, readback=readback, readback_checks=evidence.get('readback', ()),
                     unavailable=unavailable, evidence=evidence)
    owner._log({'event': 'auto_attached', 'state': auto.state, 'reason': auto.reason, 'limits': auto.limits,
                'evidence_stage': evidence.get('stage'), 'evidence_sha256': evidence.get('evidence_sha256'),
                'physical_stop_confirmed': False})
    if auto.state == 'UNAVAILABLE':
        return None
    path = Path(socket_path) if socket_path is not None else auto_socket_path(project_root, session_dir,
                                                                             owner.session_id)
    return AutoSocket(path, auto)
