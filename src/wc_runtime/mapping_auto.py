"""Automatic control source of the single wheel serial owner (docs/architecture/velocity_channel.md, sections 3-6).

Default: absent. ``mapping_wheel.main`` only calls ``attach_auto`` when the session's frozen configuration carries
``auto_controls``, which only a hardware setup that has the key puts there. Even then automatic control is
UNAVAILABLE, and no ``auto.sock`` is created, unless all of these hold:

- ``auto_controls.allowed`` is true and today's user approval ``WC_AUTO_DRIVE_APPROVAL=APPROVED_BY_USER_<YYYYMMDD>``
  is set;
- the reviewed evidence file validates for this controller (``wc_motion.auto_evidence``) and records the same
  ``hold_to_run_switch`` decision as ``auto_controls``;
- with ``hold_to_run_switch: INSTALLED``, a reader of the hold-to-run switch and the emergency stop exists;
- the driver parameters listed in the evidence read back as recorded.

The capture copies ``auto_controls`` from the hardware setup into the session configuration when the setup has it.
On the vehicle ``attach_auto`` reads the evidence's readback requests once at session start, before the owner's
first feedback read, and only after the approval and the evidence passed (``startup_readback``: FC03 of the
read-only whitelist of ``wc_motion.driver_readback``); every start of automatic control compares those values.

``hold_to_run_switch: NOT_INSTALLED`` (the user's decision of 2026-11-09 23:50): there is no switch and no switch
reader; the status reports ``dead_man`` and ``estop`` as null and the gate's enable follows the operator's
"start automatic" (``enable_source: OPERATOR_START``, ``enable_request``). The vehicle's emergency stop acts on the
driver only; it is inferred here from the wheel feedback (a wheel commanded faster than 0.05 m/s while both wheels
report less than 20 % of that command for 0.3 s: at least 0.01 m/s, about 5 feedback words, so a wheel that still
reads a word or two counts as stopped), which writes zero at once so that releasing the emergency stop cannot
resume an old command.
The status then reports ``estop_request`` for at least 0.5 s, and the bridge turns it into ``/wc_nav/estop`` so that
the gate latches. The panel's software emergency stop (manual packet ``estop``) is reported the same way while it
is pressed: the bridge is the only publisher of ``/wc_nav/estop`` (D.md 2026-11-10 13:15).

While automatic control is active (or stopping) the manual state machine of ``MappingWheel`` is frozen: manual
packets are intercepted here, where any key, Space or hand-push request ends automatic control, and
``MappingWheel.step`` returns right after the feedback read. Every change of control source passes through a written
zero and a confirmed standstill. Nothing here releases the servo for hand pushing.

The gate is one source for the whole automatic run: a different ``gate_id`` (the gate restarted) or a gate ``seq``
that does not increase ends automatic control. Its decision must report ``allow_reverse`` false and limits not
above the approved ones; the linear velocity must stay under its current ``linear_ceiling``. A decision is fresh for
0.3 s counted from the gate's own ``mono_ns`` (end to end), not only from its arrival here.

Until the RViz panel has its "start automatic" and "stop automatic" buttons, the operator gives them with
``wc_runtime.auto_operator`` over ``operator.sock`` next to ``auto.sock`` (``OperatorSocket``, same user only). A start
from there waits up to ``OPERATOR_WAIT_S`` for the RViz window to have the focus again and then goes through the same
entry conditions as the panel's packet; the base bridge and navigation never use that socket.

``switches`` and ``readback`` are called from write guards and packet handlers: they must return cached values and
must never perform an RTU exchange themselves (level 1 reads them in ``step`` between feedback reads).
"""
import datetime
import math
import os
from pathlib import Path
import select
import socket
import stat
import struct
import time

from wc_motion.auto_evidence import HARD_LIMITS, read_evidence_file
from wc_motion.auto_protocol import (DECISION_STALE_S, GATE_ACTIVE, GATE_ESTOP, MAX_LINE_BYTES, STATUS_PERIOD_S,
                                     STATUS_SCHEMA, ProtocolError, check_hello, decode_line, encode_line,
                                     parse_decision)
from .mapping_wheel import (CONTROL_PERIOD_S, FEEDBACK_FRESH_S, INITIALIZATION_WRITES, ZERO_REQUEST, Cancelled,
                            FeedbackError, speed_write)

APPROVAL_ENV = 'WC_AUTO_DRIVE_APPROVAL'
AUTO_SOCKET_NAME = 'auto.sock'
OPERATOR_SOCKET_NAME = 'operator.sock'
HOLD_TO_RUN_MODES = ('INSTALLED', 'NOT_INSTALLED')
ENABLE_SOURCES = {'INSTALLED': 'HOLD_TO_RUN_SWITCH', 'NOT_INSTALLED': 'OPERATOR_START'}
LINEAR_TOLERANCE_M_S, ANGULAR_TOLERANCE_RAD_S = 0.005, 0.01   # beyond the limit by more: the chain is broken
ENTRY_STATIONARY_SAMPLES = 5      # 0.5 s of standing still before automatic control starts
EXIT_STATIONARY_SAMPLES = 3       # standing still for three samples after the zero write ends a stop
STOP_CONFIRM_S = 1.0              # zero written and still not standing still after this: FAULT
# Standing still: both wheels read at most this many words (about 0.34 rpm, 6 mm/s). Wheels held by the servo
# after a stop read 1-3 words on the ground (11-11: (1, -2) for good after an emergency stop); a wheel that keeps
# moving faster than this still counts as moving (coordinator 2026-11-11, item 3).
STATIONARY_WORDS = 3
SWITCH_FRESH_S = 0.2
FUTURE_TOLERANCE_S = 0.05         # a gate or bridge stamp further ahead of this clock is not the same clock
BACKWARD_M_S, BACKWARD_S = -0.02, 0.2
MOVING_AT_ZERO_S = 1.0
COMMANDED_WHEEL_M_S, WHEELS_STOPPED_S = 0.05, 0.3   # emergency stop on the driver, inferred from the feedback
WHEELS_STOPPED_SHARE = 0.2        # ... when both wheels read below this share of the faster wheel's command
OVERSPEED_FACTOR, OVERSPEED_S = 1.2, 0.2            # wheel feedback above 1.2 x the approved rpm for 0.2 s
START_PENDING_S = 1.0             # operator start held while the gate unlatches on the new enable edge
OPERATOR_WAIT_S = 30.0            # a command-line start waits this long for the RViz window to have the focus
                                  # (10 s until 2026-11-11: too short to click through NoMachine)
OPERATOR_CLIENTS, OPERATOR_LINE_S = 4, 1.0   # operator.sock: clients at a time, seconds to send the request line
INFERRED_ESTOP_HOLD_S = 0.5       # an inferred hardware emergency stop is reported this long, so the gate latches
ACTIVE_STATES = ('INITIALIZING', 'AUTO', 'STOPPING')
# Exit reasons that write zero at once instead of decelerating at the software limit.
IMMEDIATE = frozenset({'NO_SWITCH_INPUT', 'SWITCH_INPUT_STALE', 'ESTOP', 'DEAD_MAN_RELEASED', 'UI_NOT_FRESH',
                       'UI_CLOSED', 'USER_STOP', 'GATE_ESTOP_LATCHED', 'MOVING_BACKWARD', 'MANUAL_LAYER_STOP',
                       'WHEELS_STOPPED_WHILE_COMMANDED', 'FEEDBACK_OVERSPEED', 'SOFTWARE_ESTOP'})


class AutoFault(FeedbackError):
    """Raised from the owner's step: its existing fault path latches FAULT and writes zero."""


def auto_socket_path(project_root, directory, session_id):
    """Next to the session's manual socket, in the same UID-owned 0700 directory."""
    from .mapping_control_paths import manual_socket_path
    return manual_socket_path(project_root, directory, session_id).with_name(AUTO_SOCKET_NAME)


def recording_sessions(project_root):
    """(directory, session id, status) of this checkout's recordings in the capture staging area, newest first.

    A recording stays there until its copy on the data disk has been verified at its end, COMPLETE or PARTIAL;
    a copy that could not be verified stays, with the reason in its manifest."""
    import json
    from .project_paths import capture_staging_root
    root, found = Path(project_root).resolve(), []
    for manifest in capture_staging_root().glob('*/capture_manifest.json'):
        try:
            value = json.loads(manifest.read_text(encoding='utf-8'))
            if Path(value.get('project_root') or '/').resolve() == root and isinstance(value.get('session_id'), str):
                found.append((manifest.stat().st_mtime_ns, str(manifest.parent), value['session_id'],
                              value.get('status')))
        except (OSError, ValueError, TypeError, AttributeError):
            continue
    return [(Path(directory), session_id, status) for _, directory, session_id, status in sorted(found, reverse=True)]


def operator_socket_path(project_root, directory, session_id):
    """The operator's command-line socket of a session, next to manual.sock and auto.sock."""
    return auto_socket_path(project_root, directory, session_id).with_name(OPERATOR_SOCKET_NAME)


def running_auto_sessions(project_root):
    """(session directory, session id) of this checkout's recordings whose wheel owner has an auto.sock now."""
    found = []
    for directory, session_id, status in recording_sessions(project_root):
        try:
            if status == 'RECORDING' and auto_socket_path(project_root, directory, session_id).is_socket():
                found.append((directory, session_id))
        except (OSError, ValueError):
            continue
    return found


def _limit(value, name, cap):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or not 0 < value <= cap:
        raise FeedbackError('auto_controls.%s must be a finite number in (0, %s]' % (name, cap))
    return float(value)


def validate_auto_controls(value):
    """``auto_controls`` of a session configuration: explicit, bounded, and off unless ``allowed`` is true."""
    if not isinstance(value, dict) or \
            set(value) - ({'allowed', 'evidence', 'note', 'hold_to_run_switch'} | set(HARD_LIMITS)):
        raise FeedbackError('auto_controls must be an object with allowed, evidence, hold_to_run_switch and the '
                            'limits only')
    if type(value.get('allowed', False)) is not bool:
        raise FeedbackError('auto_controls.allowed must be a boolean')
    result = {'allowed': value.get('allowed', False), 'evidence': value.get('evidence'), 'note': value.get('note'),
              'hold_to_run_switch': value.get('hold_to_run_switch')}
    for name, cap in HARD_LIMITS.items():
        if name in value or result['allowed']:
            result[name] = _limit(value.get(name), name, cap)
    if result['allowed'] and (not isinstance(result['evidence'], str) or not result['evidence']):
        raise FeedbackError('auto_controls.evidence must name the reviewed evidence file')
    if (result['allowed'] or 'hold_to_run_switch' in value) and result['hold_to_run_switch'] not in HOLD_TO_RUN_MODES:
        raise FeedbackError('auto_controls.hold_to_run_switch must be INSTALLED or NOT_INSTALLED')
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

    def __init__(self, owner, limits, *, hold_to_run_switch='INSTALLED', switches=None, readback=None,
                 readback_checks=(), unavailable=None, evidence=None):
        wheel = owner.wheel
        for key in ('wheel_radius_m', 'track_width_m', 'register_to_wheel_rpm', 'left_sign', 'right_sign'):
            if key not in wheel:
                raise FeedbackError('wheel candidate lacks ' + key + ' for automatic control')
        if hold_to_run_switch not in HOLD_TO_RUN_MODES:
            raise FeedbackError('hold_to_run_switch must be INSTALLED or NOT_INSTALLED')
        if hold_to_run_switch == 'NOT_INSTALLED' and switches is not None:
            raise FeedbackError('a switch reader was given although the hold-to-run switch is not installed')
        if unavailable is None and not (owner.hybrid_manual and owner.control['arm_allowed']):
            unavailable = 'NEEDS_AUTHORISED_HYBRID_MANUAL_SESSION'
        self.owner = owner
        self.limits = {name: float(limits[name]) for name in HARD_LIMITS}
        self.hold_to_run_switch, self.enable_source = hold_to_run_switch, ENABLE_SOURCES[hold_to_run_switch]
        self.switches, self.readback, self.readback_checks = switches, readback, list(readback_checks)
        self.evidence = dict(evidence or {})
        self.unavailable = unavailable
        self.state = 'UNAVAILABLE' if unavailable else 'NONE'
        self.reason = unavailable or 'IDLE'
        self.refusal = None              # last refused start; kept apart from the stop reason
        self.immediate_reason = None     # what made the current stop immediate
        self.start_pending_until = None
        self.operator_pending_until = None   # a command-line start waiting for the RViz window's focus
        self.software_estop = False           # the panel's emergency stop button, pressed
        self.inferred_estop_until = None      # report a stop inferred from the wheel feedback until then
        self.decision = self.decision_time = None
        self.last_decision_sequence = 0
        self.bridge_connected, self.bridge_id = False, None
        self.entry_generation = None
        self.entry_gate_id = None   # gate process that the current automatic run follows
        self.command, self.last_words, self.last_write = (0.0, 0.0), (0, 0), None
        self.immediate, self.zero_reached, self.zero_since = False, None, None
        self.init_step, self.next_init = 0, 0.0
        self.seen_sequence, self.stationary, self.backward_since, self.feedback = owner.sequence, 0, None, None
        self.commanded_since = self.slow_feedback_since = self.overspeed_since = None
        self.settled = 0   # consecutive zero samples taken after the zero command of the current stop
        self.takeover_latch = False
        self.counters = {name: 0 for name in ('decisions', 'decisions_rejected', 'writes', 'zero_writes',
                                              'cancelled_writes', 'entries', 'refusals', 'exits', 'violations')}
        self.dry_run = False   # True only in wc_runtime.auto_dry_run, whose serial line is a simulated driver
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
        self.start_pending_until = None
        self._log({'event': 'auto_bridge_lost', 'reason': reason}, required=False)
        if self.state in ACTIVE_STATES:
            if closing:
                # Session exit: MappingWheel.close() writes zero right after this; a failing log must not stop it.
                self._set('NONE', 'SESSION_EXIT_ZERO_BY_OWNER_CLOSE', required=False)
            else:
                self._begin_stop('BRIDGE_DISCONNECTED', immediate=False)

    def on_decision(self, value):
        try:
            decision = parse_decision(value, self.owner.session_id, self.last_decision_sequence)
            now = self.owner.clock()
            for name in ('gate_mono_ns', 'received_mono_ns'):
                ahead = decision[name] * 1e-9 - now
                if ahead > FUTURE_TOLERANCE_S:
                    raise ProtocolError('%s is %.3f s ahead of the owner clock: not the same monotonic clock'
                                        % (name, ahead))
        except ProtocolError:
            self.counters['decisions_rejected'] += 1
            raise
        previous = self.decision
        self.last_decision_sequence = decision['sequence']
        self.decision, self.decision_time = decision, now
        self.counters['decisions'] += 1
        if self.state in ('INITIALIZING', 'AUTO'):
            if decision['gate_id'] != self.entry_gate_id:
                self._begin_stop('GATE_RESTARTED', immediate=False)
            elif previous is not None and previous['gate_id'] == decision['gate_id'] and \
                    decision['gate_seq'] <= previous['gate_seq']:
                self._begin_stop('GATE_SEQ_NOT_INCREASING', immediate=False)

    # ---------------------------------------------------------------- manual side (MappingWheel.command)
    def intercept(self, value):
        """True when the manual packet is handled here and must not reach the manual state machine."""
        owner = self.owner
        kind = value.get('type')
        if kind == 'auto':
            self._auto_packet(value)
            return True
        if kind == 'estop':
            self._estop_packet(value)
            return True
        if self._pending() and (kind in ('disarm', 'close', 'push_mode') or (kind == 'keys' and value.get('keys'))):
            self._cancel_pending('START_CANCELLED_BY_OPERATOR')
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

    def _estop_packet(self, value):
        """The panel's software emergency stop: pressed stops everything at once; released only allows a new start."""
        owner = self.owner
        sequence = value.get('sequence')
        if type(sequence) is not int or sequence <= owner.last_key_sequence or type(value.get('pressed')) is not bool:
            raise FeedbackError('software emergency stop requires an increasing sequence and pressed')
        owner.last_key_sequence = sequence
        pressed = value['pressed']
        if pressed != self.software_estop:
            self._log({'event': 'auto_software_estop', 'pressed': pressed}, required=False)
        self.software_estop = pressed
        if not pressed:
            return
        if self._pending():
            self._cancel_pending('SOFTWARE_ESTOP')
        if self.state in ACTIVE_STATES:
            self._begin_stop('SOFTWARE_ESTOP', immediate=True)
        elif owner.keys or owner.state in ('INITIALIZING', 'ARMED'):
            owner.disarm('SOFTWARE_ESTOP')   # manual driving stops too; the manual layer writes zero

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
            self._operator_stop()
            return
        if self.state == 'UNAVAILABLE':
            self._refuse(self.unavailable)
        elif value['arm_generation'] != owner.arm_generation:
            self._refuse('STALE_UI_GENERATION')
        else:
            self.operator_pending_until = None
            self._request_start(owner.clock())

    def _operator_stop(self):
        """"Stop automatic": cancel a waiting start, decelerate an active run to zero."""
        if self._pending():
            self._cancel_pending('START_CANCELLED_BY_OPERATOR')
        if self.state in ACTIVE_STATES:
            self._begin_stop('USER_STOPPED_AUTO', immediate=False)

    def operator_request(self, enabled):
        """"Start / stop automatic" from the command line (``wc_runtime.auto_operator`` over ``operator.sock``).

        The operator types the command outside RViz, so the RViz window has lost the focus: a start waits up to
        OPERATOR_WAIT_S for the operator to click into it, then is checked like the panel's request."""
        self._log({'event': 'auto_operator_request', 'enabled': enabled}, required=False)
        if not enabled:
            self._operator_stop()
        elif self.state == 'UNAVAILABLE':
            self._refuse(self.unavailable)
        elif self.state != 'NONE':
            self._refuse('AUTO_NOT_IDLE_' + self.state)
        elif self.software_estop:
            self._refuse('SOFTWARE_ESTOP_PRESSED')
        else:
            self.refusal = None
            self.operator_pending_until = self.owner.clock() + OPERATOR_WAIT_S
            self._log({'event': 'auto_start_pending', 'reason': 'WAITING_FOR_RVIZ_FOCUS'}, required=False)

    def _operator_pending(self, now):
        if self.owner._ui_fresh():
            self.operator_pending_until = None
            self._request_start(now)
        elif now > self.operator_pending_until:
            self.operator_pending_until = None
            self._refuse('UI_NOT_FRESH')   # nobody clicked into the RViz window in time

    def _pending(self):
        return self.start_pending_until is not None or self.operator_pending_until is not None

    # ---------------------------------------------------------------- owner hooks
    def on_fault(self, reason):
        self.start_pending_until = self.operator_pending_until = None
        if self.state != 'UNAVAILABLE':
            self.state, self.reason = 'FAULT', 'OWNER_FAULT: ' + str(reason)

    def step(self, now):
        """Called by MappingWheel.step after the feedback read; True while automatic control owns the tick."""
        self._observe()
        if self.operator_pending_until is not None and self.state == 'NONE':
            self._operator_pending(now)
        if self.start_pending_until is not None and self.state == 'NONE':
            self._pending_start(now)
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
        elif self.state == 'STOPPING' and not self.immediate:
            # A controlled stop already decelerating still turns immediate on any emergency-class condition.
            reason = self._immediate_reason(now)
            if reason is not None:
                self._begin_stop(reason, immediate=True)
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
    def _request_start(self, now):
        blockers = self._entry_blockers(now)
        if blockers == ['GATE_ESTOP_LATCHED'] and self.enable_source == 'OPERATOR_START':
            # The gate unlatches only on a new enable edge, and in this mode the enable follows enable_request: keep
            # the request for START_PENDING_S so that the bridge sends it, and enter once the gate is unlatched.
            self.start_pending_until = now + START_PENDING_S
            self.refusal = None
            self._log({'event': 'auto_start_pending', 'reason': 'GATE_ESTOP_LATCHED'}, required=False)
            return
        self._try_start(now, blockers)

    def _pending_start(self, now):
        if now > self.start_pending_until:
            self.start_pending_until = None
            self._refuse('GATE_ESTOP_LATCHED')
            return
        blockers = self._entry_blockers(now)
        if not blockers:
            self.start_pending_until = None
            self._try_start(now, blockers)
        elif blockers != ['GATE_ESTOP_LATCHED']:
            self.start_pending_until = None
            self._refuse(blockers[0])

    def _cancel_pending(self, reason):
        self.start_pending_until = self.operator_pending_until = None
        self._log({'event': 'auto_start_cancelled', 'reason': reason}, required=False)

    def _try_start(self, now, blockers=None):
        blockers = self._entry_blockers(now) if blockers is None else blockers
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
        self.entry_gate_id = self.decision['gate_id']
        self.command, self.last_write, self.zero_since = (0.0, 0.0), None, None
        self.immediate, self.immediate_reason, self.zero_reached = False, None, None
        self.commanded_since, self.refusal = None, None
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
        if self.software_estop:
            out.append('SOFTWARE_ESTOP_PRESSED')
        if not owner._ui_fresh():
            out.append('UI_NOT_FRESH')
        problem = self._switch_problem(now, entry=True)
        if problem:
            out.append(problem)
        if not self.bridge_connected:
            out.append('BRIDGE_DISCONNECTED')
        elif self._decision_age(now) > DECISION_STALE_S:
            out.append('DECISION_STALE')
        elif self.decision['state'] == GATE_ESTOP:
            out.append('GATE_ESTOP_LATCHED')
        elif self._gate_limits_problem():
            out.append(self._gate_limits_problem())
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
            if request == INITIALIZATION_WRITES[3]:
                # Enabled: the servo holds the wheels from now on, even if automatic control ends before AUTO.
                owner.initialized_by_user, owner.servo_release_acknowledged = True, False
                owner.push_mode = owner.push_mode_requested = False
            return
        owner.stop_deadline = None
        self.command, self.last_write = (0.0, 0.0), None
        self._set('AUTO', 'AUTO_STARTED')

    # ---------------------------------------------------------------- running and stopping
    def _immediate_reason(self, now):
        """Conditions that write zero at once, also while a controlled stop is still decelerating."""
        if self.software_estop:
            return 'SOFTWARE_ESTOP'
        problem = self._switch_problem(now)
        if problem:
            return problem
        if self.decision is not None and self.decision['state'] == GATE_ESTOP:
            return 'GATE_ESTOP_LATCHED'
        if self.backward_since is not None and now - self.backward_since > BACKWARD_S:
            return 'MOVING_BACKWARD'
        if self.overspeed_since is not None and now - self.overspeed_since > OVERSPEED_S:
            return 'FEEDBACK_OVERSPEED'
        if self.commanded_since is not None and self.slow_feedback_since is not None and self._feedback_fresh(now) \
                and now - max(self.commanded_since, self.slow_feedback_since) > WHEELS_STOPPED_S:
            return 'WHEELS_STOPPED_WHILE_COMMANDED'
        return None

    def _exit_reason(self, now):
        owner = self.owner
        reason = self._immediate_reason(now)
        if reason:
            return reason
        if not owner._ui_fresh():
            return 'UI_NOT_FRESH'
        if not self.bridge_connected:
            return 'BRIDGE_DISCONNECTED'
        if self._decision_age(now) > DECISION_STALE_S:
            return 'DECISION_STALE'
        problem = self._gate_limits_problem()
        if problem:
            return problem
        violation = self._violation(self.decision)
        if violation:
            return violation
        if not self._feedback_fresh(now):
            return 'FEEDBACK_STALE'
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
        elif linear > decision['limits']['linear_ceiling'] + LINEAR_TOLERANCE_M_S:
            reason = 'LINEAR_ABOVE_CEILING'
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
        linear = min(decision['linear'], decision['limits']['linear_ceiling'], self.limits['max_linear_m_s'])
        return linear, max(-limit, min(limit, decision['angular']))

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
        half = owner.wheel['track_width_m'] / 2
        if max(abs(linear - angular * half), abs(linear + angular * half)) > COMMANDED_WHEEL_M_S:
            if self.commanded_since is None:
                self.commanded_since = self.last_write
        else:
            self.commanded_since = None

    def _guard(self, expected, generation):
        """Right before a nonzero write: pump the sockets, then recheck everything the write depends on.

        While stopping, a deceleration step is cancelled when the stop turned immediate or an emergency-class
        condition appeared (the next tick writes zero); the exit condition that started the controlled stop must
        not block the deceleration itself."""
        owner = self.owner
        owner.before_control()
        if self.state != expected or owner.arm_generation != generation:
            raise Cancelled('automatic control changed before write')
        if expected == 'STOPPING':
            if self.immediate or self._immediate_reason(owner.clock()) is not None:
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
        self.start_pending_until = self.operator_pending_until = None
        if reason == 'WHEELS_STOPPED_WHILE_COMMANDED':
            self.inferred_estop_until = self.owner.clock() + INFERRED_ESTOP_HOLD_S
        if self.state == 'STOPPING':
            if immediate and not self.immediate:
                self.immediate, self.immediate_reason = True, reason
                self._log({'event': 'auto_stop_immediate', 'reason': reason, 'stop_reason': self.reason},
                          required=False)
            return
        self.counters['exits'] += 1
        self.immediate, self.zero_reached, self.settled = immediate, None, 0
        self.immediate_reason = reason if immediate else None
        self._set('STOPPING', reason)

    def _finish_or_fault(self, now):
        if self.command != (0.0, 0.0) or self.last_write is None:
            return
        if self.zero_reached is None:
            self.zero_reached, self.settled = self.last_write, 0
        if self.settled >= EXIT_STATIONARY_SAMPLES and self._feedback_fresh(now):
            owner = self.owner
            if 'WHEELS_STOPPED_WHILE_COMMANDED' in (self.reason, self.immediate_reason):
                # Probably the vehicle's emergency stop acted on the driver: initialize again before the next run.
                owner.initialized_by_user = False
            self._set('NONE', 'AUTO_EXITED: ' + self.reason)
            self.command, self.last_write, self.zero_since, self.commanded_since = (0.0, 0.0), None, None, None
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
        signed = [value - 65536 if value >= 32768 else value for value in words]
        still = max(abs(value) for value in signed) <= STATIONARY_WORDS
        self.stationary = self.stationary + 1 if still else 0
        if self.zero_reached is not None and stamp > self.zero_reached:
            self.settled = self.settled + 1 if still else 0
        wheel = owner.wheel
        if max(abs(value) for value in signed) * wheel['register_to_wheel_rpm'] > \
                OVERSPEED_FACTOR * self.limits['max_wheel_rpm']:
            self.overspeed_since = stamp if self.overspeed_since is None else self.overspeed_since
        else:
            self.overspeed_since = None
        scale = wheel['register_to_wheel_rpm'] * 2 * math.pi * wheel['wheel_radius_m'] / 60
        left, right = signed[0] * scale * wheel['left_sign'], signed[1] * scale * wheel['right_sign']
        self.feedback = {'left_m_s': left, 'right_m_s': right, 'linear_m_s': (left + right) / 2, 'stamp': stamp,
                         'sequence': owner.sequence - 1}
        # The vehicle's emergency stop seen from the wheels: both read less than WHEELS_STOPPED_SHARE of the faster
        # wheel's current command. Not only exact zero: a wheel still settling, wobbling or held by the servo may
        # read a word or two (second review of 2026-11-10, item 1).
        linear, angular = self.command
        half = wheel['track_width_m'] / 2
        commanded = max(abs(linear - angular * half), abs(linear + angular * half))
        if max(abs(left), abs(right)) < WHEELS_STOPPED_SHARE * commanded:
            self.slow_feedback_since = stamp if self.slow_feedback_since is None else self.slow_feedback_since
        else:
            self.slow_feedback_since = None
        if self.feedback['linear_m_s'] < BACKWARD_M_S:
            self.backward_since = stamp if self.backward_since is None else self.backward_since
        else:
            self.backward_since = None

    def _feedback_fresh(self, now):
        last = self.owner.last_feedback
        return last is not None and 0 <= now - last <= FEEDBACK_FRESH_S

    def _decision_age(self, now):
        """End to end: from the gate's own stamp (same CLOCK_MONOTONIC as the owner) and from the arrival here."""
        if self.decision_time is None:
            return math.inf
        return max(now - self.decision_time, now - self.decision['gate_mono_ns'] * 1e-9)

    def _gate_limits_problem(self):
        limits = self.decision['limits']
        if limits['allow_reverse']:
            return 'GATE_ALLOWS_REVERSE'
        if limits['max_linear'] > self.limits['max_linear_m_s'] + 1e-9 or \
                limits['max_angular'] > self.limits['max_angular_rad_s'] + 1e-9:
            return 'GATE_LIMITS_ABOVE_APPROVED'
        return None

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
        if self.hold_to_run_switch == 'NOT_INSTALLED':
            return None   # the user's recorded decision; the operator's start is the enable
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

    def estop_reasons(self, now):
        """Why the bridge must report an emergency stop now (empty: none)."""
        out = ['PANEL_SOFTWARE_ESTOP'] if self.software_estop else []
        if self.inferred_estop_until is not None and now < self.inferred_estop_until:
            out.append('WHEELS_STOPPED_WHILE_COMMANDED')
        return out

    def enable_request(self, now):
        """The operator's start in effect: running, initializing, or held while the gate unlatches."""
        if self.software_estop:
            return False
        return self.state in ('INITIALIZING', 'AUTO') or \
            (self.start_pending_until is not None and now <= self.start_pending_until)

    # ---------------------------------------------------------------- records
    def _log(self, record, *, required=True):
        self.owner._log(record, required=required)

    def _set(self, state, reason, *, required=True):
        self.state, self.reason = state, reason
        self._log({'event': 'auto_state', 'state': state, 'reason': reason, 'immediate_reason': self.immediate_reason,
                   'command': list(self.command), 'decision_sequence': self.last_decision_sequence,
                   'physical_stop_confirmed': False}, required=required)

    def _refuse(self, reason):
        self.counters['refusals'] += 1
        self.refusal = 'AUTO_REFUSED: ' + reason
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
            # Owner clock (CLOCK_MONOTONIC) when the response was read: comparable with mono_ns on this machine.
            feedback['stamp_mono_ns'] = int(round(self.feedback['stamp'] * 1e9))
        return {'type': 'status', 'schema': STATUS_SCHEMA, 'session_id': owner.session_id, 'source': source,
                'auto_state': self.state, 'reason': self.reason, 'refusal': self.refusal,
                'immediate_reason': self.immediate_reason, 'limits': dict(self.limits),
                'command': {'linear': self.command[0], 'angular': self.command[1], 'words': list(self.last_words)},
                'feedback': feedback, 'stationary_samples': self.stationary,
                'hold_to_run_switch': self.hold_to_run_switch, 'enable_source': self.enable_source,
                'enable_request': self.enable_request(now), 'start_pending': self.start_pending_until is not None,
                'operator_pending': self.operator_pending_until is not None,
                'estop_request': bool(self.estop_reasons(now)), 'estop_reasons': self.estop_reasons(now),
                'switch_input': self.switches is not None,
                'dead_man': None if switches is None else switches['dead_man'],
                'estop': None if switches is None else switches['estop'],
                'bridge_connected': self.bridge_connected, 'bridge_id': self.bridge_id,
                'decision_age_s': None if decision is None else round(self._decision_age(now), 3),
                'gate_state': None if decision is None else decision['state'],
                'gate_seq': None if decision is None else decision['gate_seq'],
                'gate_id': None if decision is None else decision['gate_id'], 'entry_gate_id': self.entry_gate_id,
                'takeover_latched': self.takeover_latch, 'manual_state': owner.state, 'push_mode': owner.push_mode,
                'evidence_stage': self.evidence.get('stage'), 'evidence_sha256': self.evidence.get('evidence_sha256'),
                'counters': dict(self.counters), 'dry_run': self.dry_run, 'physical_stop_confirmed': False}


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
        try:
            self.operator = OperatorSocket(self.path.with_name(OPERATOR_SOCKET_NAME), auto)
        except BaseException:
            self.close()
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
        self.operator.pump()
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
                        self.auto.on_decision(value)   # only decisions: navigation can never request automatic
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
        operator = getattr(self, 'operator', None)
        if operator is not None:
            operator.close()
        self.listener.close()
        _unlink_own_socket(self.path, self.identity)


def _unlink_own_socket(path, identity):
    try:
        value = path.lstat()
        if stat.S_ISSOCK(value.st_mode) and (value.st_dev, value.st_ino) == identity:
            path.unlink()
    except FileNotFoundError:
        pass


class OperatorSocket:
    """The operator's command line (``wc_runtime.auto_operator``): same-UID clients, one request line each.

    ``{"type": "auto_request", "session_id": ..., "enabled": true|false}`` is "start / stop automatic";
    ``{"type": "status", "session_id": ...}`` only asks. The answer is one line, ``{"type": "operator_status",
    "status": <the automatic control status>}``, then the connection closes. At most OPERATOR_CLIENTS at a time;
    one that has not sent its line within OPERATOR_LINE_S is closed. Nothing here can make the wheels move by
    itself: a start still needs the RViz window's focus and every entry condition."""

    def __init__(self, path, auto):
        self.path, self.auto, self.clock = Path(path), auto, auto.owner.clock
        self.listener = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self.clients, self.identity = {}, None
        try:
            self.listener.bind(str(self.path))   # never removes or reuses an existing socket
            os.chmod(self.path, 0o600)
            info = self.path.stat()
            self.identity = (info.st_dev, info.st_ino)
            self.listener.listen(OPERATOR_CLIENTS)
            self.listener.setblocking(False)
        except BaseException:
            self.listener.close()
            raise

    def _reject(self, client, reason):
        self.clients.pop(client, None)
        client.close()
        self.auto._log({'event': 'auto_operator_rejected', 'reason': reason}, required=False)

    def _answer(self, line):
        value = decode_line(bytes(line))
        if value.get('session_id') != self.auto.owner.session_id:
            raise ProtocolError('operator request for another session')
        if value.get('type') == 'auto_request' and set(value) == {'type', 'session_id', 'enabled'} and \
                type(value['enabled']) is bool:
            self.auto.operator_request(value['enabled'])
        elif value.get('type') != 'status' or set(value) != {'type', 'session_id'}:
            raise ProtocolError('operator request must be auto_request with enabled, or status')
        return encode_line({'type': 'operator_status', 'session_id': self.auto.owner.session_id,
                            'status': self.auto.status()})

    def pump(self):
        try:
            while select.select([self.listener], [], [], 0)[0]:
                client, _ = self.listener.accept()
                if hasattr(socket, 'SO_PEERCRED'):
                    _, uid, _ = struct.unpack('3i', client.getsockopt(socket.SOL_SOCKET, socket.SO_PEERCRED, 12))
                    if uid != os.getuid():
                        client.close()
                        continue
                if len(self.clients) >= OPERATOR_CLIENTS:
                    client.close()
                    continue
                client.setblocking(False)
                self.clients[client] = (self.clock(), bytearray())
        except OSError:
            pass   # a client that went away while being accepted
        for client, (accepted, received) in list(self.clients.items()):
            try:
                if select.select([client], [], [], 0)[0]:
                    data = client.recv(MAX_LINE_BYTES + 1)
                    if not data:
                        self._reject(client, 'closed before a request')
                        continue
                    received.extend(data)
                if b'\n' in received:
                    line = received.partition(b'\n')[0]
                    if not line or len(line) > MAX_LINE_BYTES:
                        raise ProtocolError('operator request line bound')
                    answer = self._answer(line)
                    self.clients.pop(client, None)
                    try:
                        client.send(answer)   # a few kB into an empty local socket buffer
                    finally:
                        client.close()
                elif len(received) > MAX_LINE_BYTES:
                    raise ProtocolError('unterminated operator request exceeds bound')
                elif self.clock() - accepted > OPERATOR_LINE_S:
                    self._reject(client, 'no request line within %.1f s' % OPERATOR_LINE_S)
            except (ProtocolError, ValueError, TypeError, UnicodeError, OSError) as error:
                self._reject(client, str(error))

    def close(self):
        for client in list(self.clients):
            client.close()
        self.clients.clear()
        self.listener.close()
        _unlink_own_socket(self.path, self.identity)


READBACK_TIMEOUT_S = 0.1   # one FC03 exchange
READBACK_GAP_S = 0.05      # bus idle time between two reads, as in wc_motion.driver_readback


def startup_readback(owner, checks, *, timeout_s=READBACK_TIMEOUT_S, gap_s=READBACK_GAP_S, sleep=time.sleep):
    """Read the evidence's readback requests once on the owner's serial line: (lookup, None) or (None, reason).

    Runs before the owner's first feedback read, so nothing else is on the bus. Only requests of the read-only
    whitelist of ``wc_motion.driver_readback`` are sent, each once; every answer goes into the owner's journal. A
    Modbus exception answer leaves automatic control unavailable. A transport failure (timeout, CRC, stray bytes,
    disconnect) fails the line like any other wheel exchange, without a retry, and is raised: the session ends.
    """
    from wc_motion.driver_readback import exchange, whitelisted
    from wc_motion.feedback_transport import QueryFailure
    channel, words = owner.channel, {}
    for check in checks:
        request = check['request']
        if request in words:
            continue
        whitelisted(request)
        if words:
            sleep(gap_s)
        if channel.failed or channel.lease.failed or channel.lease.fd is None:
            raise FeedbackError('AUTO_STARTUP_READBACK_FAILED: closed or failed wheel channel')
        started = time.monotonic_ns()
        try:
            response, values, code = exchange(channel.lease.fd, request, timeout_s)
        except (QueryFailure, OSError) as failure:
            channel.failed = channel.lease.failed = True
            owner._log({'event': 'auto_readback_failed', 'purpose': check['purpose'], 'request_hex': request.hex(),
                        'response_hex': bytes(getattr(failure, 'response', b'')).hex(), 'error': str(failure)},
                       required=False)
            raise FeedbackError('AUTO_STARTUP_READBACK_FAILED: %s: %s' % (check['purpose'], failure)) from failure
        owner._log({'event': 'auto_readback', 'purpose': check['purpose'], 'request_hex': request.hex(),
                    'response_hex': response.hex(), 'words_u16': values, 'exception_code': code,
                    'exchange_started_monotonic_ns': started, 'exchange_completed_monotonic_ns': time.monotonic_ns()})
        if code is not None:
            return None, 'DRIVER_READBACK_EXCEPTION: %s answered Modbus exception %d' % (check['purpose'], code)
        words[request] = values
    return (lambda request: list(words[request])), None


def attach_auto(config, owner, project_root, session_dir, *, environ=None, today=None, switches=None, readback=None,
                socket_path=None):
    """Attach the automatic control source to ``owner``; returns its socket, or None while it is unavailable.

    Without ``readback`` (the vehicle) the evidence's readback requests are read once here (``startup_readback``),
    after everything else passed."""
    controls = validate_auto_controls(config.get('auto_controls'))
    environ = os.environ if environ is None else environ
    today = today or datetime.date.today()
    unavailable, evidence = None, {}
    hold = controls['hold_to_run_switch'] or 'INSTALLED'
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
    if unavailable is None and evidence['hold_to_run_switch']['installed'] != (hold == 'INSTALLED'):
        unavailable = 'HOLD_TO_RUN_SWITCH_DIFFERS_FROM_EVIDENCE'
    if unavailable is None and not limits_consistent(limits, owner.wheel):
        unavailable = 'LIMITS_INCONSISTENT'
    if hold == 'NOT_INSTALLED' and switches is not None:
        unavailable, switches = unavailable or 'SWITCH_READER_GIVEN_BUT_NOT_INSTALLED', None
    if unavailable is None and hold == 'INSTALLED' and switches is None:
        unavailable = 'NO_SWITCH_INPUT'
    if unavailable is None and readback is None:
        readback, unavailable = startup_readback(owner, evidence['readback'])
    auto = AutoDrive(owner, limits, hold_to_run_switch=hold, switches=switches, readback=readback,
                     readback_checks=evidence.get('readback', ()), unavailable=unavailable, evidence=evidence)
    owner._log({'event': 'auto_attached', 'state': auto.state, 'reason': auto.reason, 'limits': auto.limits,
                'hold_to_run_switch': hold, 'enable_source': auto.enable_source,
                'evidence_stage': evidence.get('stage'), 'evidence_sha256': evidence.get('evidence_sha256'),
                'physical_stop_confirmed': False})
    if auto.state == 'UNAVAILABLE':
        return None
    path = Path(socket_path) if socket_path is not None else auto_socket_path(project_root, session_dir,
                                                                             owner.session_id)
    return AutoSocket(path, auto)
