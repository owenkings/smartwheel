"""Automatic control source of the wheel owner (velocity_channel.md). Fake serial channel only: no device, no motion."""
import itertools
import json
import math
from pathlib import Path
import socket as sk
import struct
import sys
import tempfile
import threading
from types import SimpleNamespace as NS

import pytest

from wc_motion import auto_protocol as p
from wc_runtime import mapping_auto as a
from wc_runtime import mapping_wheel as m


ROOT = Path(__file__).resolve().parents[2]
LIMITS = {'max_linear_m_s': 0.10, 'max_angular_rad_s': 0.30, 'max_wheel_rpm': 27.0, 'max_linear_accel_m_s2': 0.30,
          'max_linear_decel_m_s2': 0.60, 'max_angular_accel_rad_s2': 0.80}
TIMEOUT_REQUEST = m.read_request(1, 0x2000, 1)   # synthetic register, not a reviewed ZLAC8030D address
FORWARD = ((-40) & 0xffff, 40)                  # left sign -1: both wheels report forward motion
BACKWARD = (40, (-40) & 0xffff)
NOT_INSTALLED = {'installed': False, 'decided_by': 'user', 'decided_at': '2026-11-09',
                 'decision': '不做硬件松手即停；第一次空车低速闭环用车上现有急停、人在车旁'}


class Clock:
    def __init__(self): self.value = 10.
    def __call__(self): return self.value
    def advance(self, seconds): self.value += seconds
    def ns(self): return round(self.value*1e9)
    def wall(self): return 1_700_000_000_000_000_000+self.ns()


def response(words):
    payload = b'\x01\x03\x04'+struct.pack('>HH', *[w & 0xffff for w in words])
    return payload+m.crc16(payload)


class FakeChannel:
    """Same contract as WheelChannel.exchange: guard right before a write, Cancelled propagates without sending.

    The wheels follow the last speed write unless a test fixes ``words`` (rolling, stuck or backward wheels)."""
    def __init__(self):
        self.control_transmissions = self.control_attempts = self.control_bytes = 0
        self.motion_may_exist = self.failed = self.recovery_attempted = False
        self.last_stop = None
        self.requests, self.words, self.failure = [], None, None
        self.follow_factor, self.followed = 1.0, (0, 0)

    def feedback(self):
        if self.words is not None:
            return self.words
        return tuple(int(round(word*self.follow_factor)) for word in self.followed)

    def exchange(self, request, timeout, *, guard=None):
        if guard is not None:
            guard()
        if self.failure:
            self.failed = True
            raise m.QueryFailure(self.failure)
        self.requests.append(request)
        if request == m.QUERY:
            return response(self.feedback())
        self.control_transmissions += 1; self.control_attempts += 1; self.control_bytes += len(request)
        if request[1] == 0x10:
            self.followed = struct.unpack('>hh', request[7:11])
        if request == m.INITIALIZATION_WRITES[-1] or request[1] == 16 and request != m.ZERO_REQUEST:
            self.motion_may_exist = True
        return request if request[1] == 6 else request[:6]+m.crc16(request[:6])

    def best_effort_zero(self):
        if self.motion_may_exist and not self.recovery_attempted:
            self.recovery_attempted = True
            self.requests.append(m.ZERO_REQUEST)
        return self.last_stop


class Switches:
    def __init__(self, clock): self.clock, self.dead_man, self.estop, self.frozen = clock, True, False, None
    def __call__(self):
        return {'dead_man': self.dead_man, 'estop': self.estop,
                'stamp': self.clock() if self.frozen is None else self.frozen}


def config():
    wheel = json.loads((ROOT/'config/wheel_history_calibration.json').read_text())
    return {'session_id': 'synthetic_auto', 'wheel_device_id': 'ZLAC8030D-0000000014', 'wheel_candidate': wheel,
            'manual_controls': {'arm_allowed': True, 'interaction_policy': 'hybrid_manual',
                                'operation_evidence': {'status': 'USER_REPORTED_LEGACY_OPERATION', 'note': 'SYNTHETIC'}}}


def make(*, unavailable=None, readback_value=7, hold='INSTALLED', journal=None, cancel=None):
    clock, channel, records = Clock(), FakeChannel(), []
    owner = m.MappingWheel(config(), channel, journal or records.append, clock=clock, wall_ns=clock.wall,
                           mono_ns=clock.ns, cancelled=cancel)
    channel.follow_factor = 1/(owner.control['command_rpm_to_register_scale']*owner.wheel['register_to_wheel_rpm'])
    env = NS(owner=owner, clock=clock, channel=channel, records=records, reads=[], seq=0, ui=True, gate=True,
             linear=0.0, angular=0.0, state='ACTIVE', gate_limits=(0.10, 0.30), ceiling=None, reverse=False,
             gate_id='gate_test', gate_age_s=0.0, tick=0)
    env.switches = Switches(clock) if hold == 'INSTALLED' else None

    def readback(request):
        env.reads.append(request)
        return [readback_value]
    checks = [{'purpose': 'communication_timeout', 'request': TIMEOUT_REQUEST, 'word_index': 0, 'mask': 0xffff,
               'equals': 7}]
    env.auto = a.AutoDrive(owner, LIMITS, hold_to_run_switch=hold, switches=env.switches, readback=readback,
                           readback_checks=checks, unavailable=unavailable)
    owner.connected()
    env.auto.on_bridge_connected('bridge_test')
    return env


def keys(env, pressed, foreground=True):
    o = env.owner
    o.command({'type': 'keys', 'session_id': o.session_id, 'sequence': o.last_key_sequence+1,
               'arm_generation': o.arm_generation, 'keys': pressed, 'foreground': foreground})


def packet(env, **change):
    env.seq += 1
    value = {'type': 'decision', 'session_id': env.owner.session_id, 'sequence': env.seq,
             'received_mono_ns': env.clock.ns(), 'gate_id': env.gate_id, 'gate_seq': env.seq,
             'gate_mono_ns': env.clock.ns() - round(env.gate_age_s*1e9), 'state': env.state, 'reasons': [],
             'linear': env.linear, 'angular': env.angular, 'config_sha256': None,
             'limits': {'max_linear': env.gate_limits[0], 'max_angular': env.gate_limits[1],
                        'linear_ceiling': env.gate_limits[0] if env.ceiling is None else env.ceiling,
                        'allow_reverse': env.reverse}}
    value.update(change)
    return value


def decision(env, gate_seq=None):
    env.auto.on_decision(packet(env, **({} if gate_seq is None else {'gate_seq': gate_seq})))


def run(env, seconds, held=None, dt=0.01):
    """Interface heartbeat every tick (the RViz panel sends one every 50 ms), gate decision at 20 Hz."""
    for _ in range(round(seconds/dt)):
        if env.ui:
            keys(env, held or [])
        if env.gate and env.tick % 5 == 0:
            decision(env)
        env.tick += 1
        env.owner.step()
        env.clock.advance(dt)


def request_auto(env, enabled=True):
    o = env.owner
    o.command({'type': 'auto', 'session_id': o.session_id, 'sequence': o.last_key_sequence+1,
               'arm_generation': o.arm_generation, 'enabled': enabled, 'foreground': True})


def speeds(env, since=0):
    """(left, right) command words of every FC10 write after index ``since``."""
    return [struct.unpack('>hh', r[7:11]) for r in env.channel.requests[since:] if r[1] == 0x10]


def chassis(words, env):
    wheel = env.owner.wheel
    scale = env.owner.control['command_rpm_to_register_scale']
    v = [words[i]/scale/wheel[s]*2*math.pi*wheel['wheel_radius_m']/60 for i, s in enumerate(('left_sign', 'right_sign'))]
    return (v[0]+v[1])/2, (v[1]-v[0])/wheel['track_width_m']


def driving(env, linear=0.10, angular=0.0):
    run(env, 1.0)                                  # idle: hybrid heartbeat puts the servo in hand-push release
    assert env.owner.push_mode and env.owner.state == 'READY'
    env.linear, env.angular = linear, angular
    request_auto(env)
    run(env, 1.5)
    assert env.auto.state == 'AUTO', (env.auto.reason, env.auto.refusal)
    return env


def until(env, predicate, seconds=3.0, held=None):
    for _ in range(round(seconds/0.01)):
        if predicate():
            return True
        run(env, 0.01, held=held)
    return predicate()


def decelerates(profile):
    return min(b - c for c, b in zip(profile, profile[1:])) >= -LIMITS['max_linear_decel_m_s2']*0.05 - 2e-3


# ---------------------------------------------------------------- default path and entry (section 4)
def test_owner_without_auto_is_unchanged():
    clock, channel, records = Clock(), FakeChannel(), []
    owner = m.MappingWheel(config(), channel, records.append, clock=clock, wall_ns=clock.wall, mono_ns=clock.ns)
    assert owner.auto is None and 'auto' not in owner.status()


def test_unavailable_source_refuses_and_never_writes_speed():
    env = make(unavailable='NO_SWITCH_INPUT')
    run(env, 1.0)
    env.linear = 0.1
    before = len(env.channel.requests)
    request_auto(env)
    run(env, 1.0)
    assert env.auto.state == 'UNAVAILABLE' and env.auto.refusal == 'AUTO_REFUSED: NO_SWITCH_INPUT'
    assert env.auto.reason == 'NO_SWITCH_INPUT'
    assert not any(w != (0, 0) for w in speeds(env, before))
    request_auto(env)                              # repeated requests do not nest the reason
    assert env.auto.refusal == 'AUTO_REFUSED: NO_SWITCH_INPUT'


@pytest.mark.parametrize('cause, reason', [
    ('bridge', 'BRIDGE_DISCONNECTED'), ('stale', 'DECISION_STALE'), ('old_decision', 'DECISION_STALE'),
    ('limits', 'GATE_LIMITS_ABOVE_APPROVED'), ('reverse', 'GATE_ALLOWS_REVERSE'), ('gate_estop', 'GATE_ESTOP_LATCHED'),
    ('dead_man', 'DEAD_MAN_NOT_HELD'), ('estop', 'ESTOP'), ('switch_stale', 'SWITCH_INPUT_STALE'),
    ('keys', 'MANUAL_STATE_ARMED'), ('moving', 'NOT_STATIONARY'), ('readback', 'DRIVER_READBACK_MISMATCH'),
    ('ui', 'UI_NOT_FRESH'), ('generation', 'STALE_UI_GENERATION')])
def test_entry_is_refused_until_every_condition_holds(cause, reason):
    env = make(readback_value=8 if cause == 'readback' else 7)
    run(env, 1.0)
    env.linear = 0.1
    if cause == 'bridge':
        env.auto.on_bridge_lost('TEST')
    elif cause == 'stale':
        env.gate = False; run(env, 0.5)
    elif cause == 'old_decision':
        env.gate_age_s = 0.35; run(env, 0.1)       # arrives on time, but made by the gate 0.35 s ago
    elif cause == 'limits':
        env.gate_limits = (0.30, 0.50); run(env, 0.1)
    elif cause == 'reverse':
        env.reverse = True; run(env, 0.1)
    elif cause == 'gate_estop':
        env.state, env.linear = 'ESTOP_LATCHED', 0.0; run(env, 0.1)
    elif cause == 'dead_man':
        env.switches.dead_man = False
    elif cause == 'estop':
        env.switches.estop = True
    elif cause == 'switch_stale':
        env.switches.frozen = env.clock() - 0.5
    elif cause == 'keys':
        run(env, 1.0, held=['w'])                  # manual driving right now
    elif cause == 'moving':
        env.channel.words = FORWARD; run(env, 0.3)
    elif cause == 'ui':
        env.ui = False; run(env, 0.4)
    before = len(env.channel.requests)
    o = env.owner
    if cause in ('generation', 'ui'):            # the request itself is a fresh heartbeat unless foreground=false
        o.command({'type': 'auto', 'session_id': o.session_id, 'sequence': o.last_key_sequence+1,
                   'arm_generation': o.arm_generation-(cause == 'generation'), 'enabled': True,
                   'foreground': cause != 'ui'})
    else:
        request_auto(env)
    assert env.auto.state == 'NONE' and env.auto.refusal.startswith('AUTO_REFUSED: '+reason), env.auto.refusal
    assert not env.auto.start_pending_until            # with the switch installed nothing waits for the gate
    run(env, 0.5, held=['w'] if cause == 'keys' else None)
    assert env.auto.state == 'NONE'
    if cause != 'keys':
        assert not any(w != (0, 0) for w in speeds(env, before))


def test_auto_requests_need_an_increasing_sequence_and_every_field():
    env = make()
    run(env, 1.0)
    o = env.owner
    base = {'type': 'auto', 'session_id': o.session_id, 'arm_generation': o.arm_generation, 'enabled': True,
            'foreground': True}
    with pytest.raises(m.FeedbackError):
        o.command({**base, 'sequence': o.last_key_sequence})          # not increasing
    for name in ('enabled', 'arm_generation', 'foreground'):
        with pytest.raises(m.FeedbackError):
            o.command({**{k: v for k, v in base.items() if k != name}, 'sequence': o.last_key_sequence+1})
    assert env.auto.counters['entries'] == 0 and env.auto.state == 'NONE'


def test_start_initializes_released_servo_then_ramps_within_limits():
    env = make()
    run(env, 1.0)
    assert env.owner.push_mode
    env.linear = 0.1
    start = len(env.channel.requests)
    request_auto(env)
    assert env.reads == [TIMEOUT_REQUEST]
    run(env, 3.0)
    assert env.auto.state == 'AUTO'
    writes = [r for r in env.channel.requests[start:] if r != m.QUERY]
    assert writes[:5] == [*m.INITIALIZATION_WRITES[:3], m.ZERO_REQUEST, m.INITIALIZATION_WRITES[3]]
    profile = [chassis(w, env)[0] for w in speeds(env, start)]
    assert max(profile) <= LIMITS['max_linear_m_s'] + 1e-9               # register values truncated toward zero
    steps = [b - c for c, b in zip(profile, profile[1:])]
    assert max(steps) <= LIMITS['max_linear_accel_m_s2']*0.05 + 2e-3      # 0.30 m/s² at 20 Hz plus rounding
    assert abs(profile[-1] - 0.1) < 2e-3
    assert m.RELEASE_REQUEST not in writes
    stamps = [r['receive_monotonic_ns'] for r in env.records if r.get('event') == 'transaction_complete']
    assert stamps                                             # feedback kept running at 10 Hz throughout
    assert env.owner.status()['auto']['source'] == 'AUTO'


def test_angular_velocity_ramps_at_the_angular_acceleration_limit():
    env = make()
    run(env, 1.0)
    env.angular = 0.3
    start = len(env.channel.requests)
    request_auto(env)
    run(env, 2.0)
    assert env.auto.state == 'AUTO'
    turns = [chassis(w, env)[1] for w in speeds(env, start)]
    assert max(b - c for c, b in zip(turns, turns[1:])) <= LIMITS['max_angular_accel_rad_s2']*0.05 + 0.01
    assert max(turns) <= LIMITS['max_angular_rad_s'] + 0.01 and abs(turns[-1] - 0.3) < 0.01


def test_exit_during_initialization_after_the_enable_write_reports_the_servo_held():
    env = make()
    run(env, 1.0)
    assert env.owner.push_mode
    env.linear = 0.1
    request_auto(env)
    assert env.auto.state == 'INITIALIZING'
    assert until(env, lambda: m.INITIALIZATION_WRITES[3] in env.channel.requests, 1.0)
    assert env.auto.state == 'INITIALIZING'            # 0.2 s settle after the enable, AUTO not reached yet
    env.switches.dead_man = False
    run(env, 0.6)
    assert env.auto.state == 'NONE'
    assert env.owner.push_mode is False and env.owner.initialized_by_user is True   # the servo holds the wheels


def test_a_refused_start_does_not_hide_the_stop_reason():
    env = driving(make())
    env.auto.on_bridge_lost('TEST')
    request_auto(env)                                  # refused while stopping
    assert env.auto.refusal == 'AUTO_REFUSED: AUTO_NOT_IDLE_STOPPING' and env.auto.reason == 'BRIDGE_DISCONNECTED'
    run(env, 1.0)
    assert env.auto.reason == 'AUTO_EXITED: BRIDGE_DISCONNECTED'
    assert env.owner.status()['auto']['refusal'] == 'AUTO_REFUSED: AUTO_NOT_IDLE_STOPPING'


# ---------------------------------------------------------------- limits (section 5)
@pytest.mark.parametrize('change, reason', [
    (dict(linear=0.2), 'LINEAR_ABOVE_LIMIT'), (dict(linear=-0.05), 'NEGATIVE_LINEAR'),
    (dict(angular=0.5), 'ANGULAR_ABOVE_LIMIT'), (dict(state='LOCALIZATION_UNTRUSTED'), 'GATE_NONZERO_WHILE_NOT_ACTIVE'),
    (dict(ceiling=0.05), 'LINEAR_ABOVE_CEILING')])
def test_violations_end_auto_with_a_controlled_stop(change, reason):
    env = driving(make())
    start = len(env.channel.requests)
    for key, value in change.items():
        setattr(env, key, value)
    run(env, 0.1)
    assert env.auto.reason == reason and env.auto.state == 'STOPPING'
    run(env, 1.0)
    assert env.auto.state == 'NONE' and env.owner.state == 'READY'
    written = [chassis(w, env) for w in speeds(env, start)]
    assert all(v <= LIMITS['max_linear_m_s'] + 1e-9 and abs(w) <= LIMITS['max_angular_rad_s'] + 1e-3 for v, w in written)
    assert written[-1] == (0.0, 0.0) and env.auto.counters['violations'] >= 1


@pytest.mark.parametrize('linear, angular, reason', [
    (0.104, 0.0, None), (0.106, 0.0, 'LINEAR_ABOVE_LIMIT'), (0.05, 0.309, None), (0.05, 0.311, 'ANGULAR_ABOVE_LIMIT'),
    (0.05, -0.311, 'ANGULAR_ABOVE_LIMIT')])
def test_tolerance_is_5_mm_s_and_0_01_rad_s_and_values_inside_it_are_held_at_the_limit(linear, angular, reason):
    env = driving(make(), linear=0.05)
    env.linear, env.angular = linear, angular
    run(env, 0.8)
    if reason is None:
        assert env.auto.state == 'AUTO', env.auto.reason
        written = [chassis(w, env) for w in speeds(env)[-5:]]
        assert all(v <= LIMITS['max_linear_m_s'] + 1e-9 and abs(w) <= LIMITS['max_angular_rad_s'] + 0.01
                   for v, w in written)
    else:
        assert env.auto.reason.endswith(reason) and env.auto.counters['violations'] >= 1


def test_wheel_rpm_limit_inconsistent_limits_and_register_overflow(short_tmp):
    controls = {'allowed': True, 'evidence': 'evidence/auto.json', 'hold_to_run_switch': 'INSTALLED',
                **{**LIMITS, 'max_wheel_rpm': 5.0}}
    owner, sock, _ = attached(short_tmp, controls=controls, socket_path=short_tmp/'auto.sock')
    assert sock is None and owner.auto.reason == 'LIMITS_INCONSISTENT'
    env = make()
    tight = a.AutoDrive(env.owner, {**LIMITS, 'max_wheel_rpm': 5.0}, switches=env.switches, readback=lambda r: [7])
    with pytest.raises(a.AutoFault, match='WHEEL_RPM_ABOVE_LIMIT'):
        tight._encode(0.10, 0.0)
    env.owner.control = {**env.owner.control, 'command_rpm_to_register_scale': 20000.0}
    with pytest.raises(a.AutoFault, match='WHEEL_COMMAND_REGISTER_OVERFLOW'):
        tight._encode(0.04, 0.0)                     # 2.3 rpm under the 5 rpm limit, but 46 000 register units


@pytest.mark.parametrize('change, reason', [(dict(reverse=True), 'GATE_ALLOWS_REVERSE'),
                                            (dict(gate_limits=(0.30, 0.30)), 'GATE_LIMITS_ABOVE_APPROVED')])
def test_gate_limits_changing_during_auto_end_it_with_a_controlled_stop(change, reason):
    env = driving(make())
    start = len(env.channel.requests)
    for key, value in change.items():
        setattr(env, key, value)
    run(env, 0.1)
    assert env.auto.reason == reason and env.auto.state == 'STOPPING' and not env.auto.immediate
    run(env, 1.0)
    assert env.auto.state == 'NONE'
    profile = [chassis(w, env)[0] for w in speeds(env, start)]
    assert profile[-1] == 0.0 and max(profile) <= LIMITS['max_linear_m_s'] + 1e-9


def test_target_is_held_under_the_gate_ceiling_within_tolerance():
    env = driving(make())
    env.ceiling = 0.097                            # gate output 0.10 is 3 mm/s above its own ceiling: tolerated
    start = len(env.channel.requests)
    run(env, 1.0)
    assert env.auto.state == 'AUTO', env.auto.reason
    assert max(chassis(w, env)[0] for w in speeds(env, start)) <= 0.097 + 1e-9


def test_wheel_feedback_above_the_approved_rpm_stops_at_once():
    env = driving(make())
    fast = round(1.25*LIMITS['max_wheel_rpm']/env.owner.wheel['register_to_wheel_rpm'])
    env.channel.words = ((-fast) & 0xffff, fast)    # both wheels forward, 25 % above 27 rpm
    run(env, 0.15)
    assert env.auto.state == 'AUTO'                 # shorter than 0.2 s: not yet
    assert until(env, lambda: env.auto.reason == 'FEEDBACK_OVERSPEED', 0.3)
    start = len(env.channel.requests)
    run(env, 0.06)
    assert env.auto.immediate and speeds(env, start)[0] == (0, 0)


# ---------------------------------------------------------------- gate decisions
@pytest.mark.parametrize('change, reason', [('restart', 'GATE_RESTARTED'), ('repeat', 'GATE_SEQ_NOT_INCREASING')])
def test_a_new_gate_source_ends_auto_with_a_controlled_stop(change, reason):
    env = driving(make())
    start = len(env.channel.requests)
    if change == 'restart':
        env.gate_id = 'gate_restarted'             # seq starts again after a restart; the new id alone is enough
        decision(env, gate_seq=1)
    else:
        decision(env, gate_seq=env.seq - 3)        # the bridge sequence still increases
    assert env.auto.state == 'STOPPING' and env.auto.reason == reason and not env.auto.immediate
    run(env, 1.5)
    assert env.auto.state == 'NONE'
    profile = [chassis(w, env)[0] for w in speeds(env, start)]
    assert profile[0] > 0 and profile[-1] == 0.0 and decelerates(profile)


def test_decision_freshness_is_counted_from_the_gate_stamp_end_to_end():
    env = driving(make())
    env.gate_age_s = 0.2                           # each decision arrives 0.2 s after the gate made it
    run(env, 0.5)
    assert env.auto.state == 'AUTO'                # 0.2 s plus a 0.05 s gap stays under 0.3 s
    env.gate_age_s = 0.28
    run(env, 0.1)
    assert env.auto.reason == 'DECISION_STALE' and env.auto.state == 'STOPPING'


@pytest.mark.parametrize('field', ['gate_mono_ns', 'received_mono_ns'])
def test_stamps_ahead_of_the_owner_clock_are_rejected(field):
    env = make()
    with pytest.raises(p.ProtocolError, match='ahead of the owner clock'):
        env.auto.on_decision(packet(env, **{field: env.clock.ns() + 100_000_000}))
    assert env.auto.counters['decisions_rejected'] == 1 and env.auto.decision is None
    env.auto.on_decision(packet(env, **{field: env.clock.ns() + 40_000_000}))       # within 50 ms: same clock
    assert env.auto.decision is not None


def test_gate_hold_keeps_auto_writes_zeros_and_resumes_with_the_ramp():
    env = driving(make())
    env.state, env.linear = 'LOCALIZATION_UNTRUSTED', 0.0
    start = len(env.channel.requests)
    run(env, 1.0)
    assert env.auto.state == 'AUTO'
    held = speeds(env, start)
    assert held[-1] == (0, 0) and len(held) >= 15            # zero rewritten at about 20 Hz
    env.state, env.linear = 'ACTIVE', 0.1
    start = len(env.channel.requests)
    run(env, 0.2)
    first = chassis(speeds(env, start)[0], env)[0]
    assert 0 < first <= LIMITS['max_linear_accel_m_s2']*0.05 + 2e-3


def test_gate_emergency_stop_latched_stops_at_once():
    env = driving(make())
    env.state, env.linear = 'ESTOP_LATCHED', 0.0
    decision(env)
    start = len(env.channel.requests)
    run(env, 0.06)
    assert env.auto.reason == 'GATE_ESTOP_LATCHED' and env.auto.immediate and speeds(env, start)[0] == (0, 0)


# ---------------------------------------------------------------- operator and switches (section 4)
@pytest.mark.parametrize('how, reason, immediate', [('space', 'USER_STOP', True), ('close', 'UI_CLOSED', True),
                                                     ('stop_auto', 'USER_STOPPED_AUTO', False)])
def test_space_interface_close_and_stop_auto(how, reason, immediate):
    env = driving(make())
    start = len(env.channel.requests)
    o = env.owner
    if how == 'space':
        o.command({'type': 'disarm', 'session_id': o.session_id})
    elif how == 'close':
        o.command({'type': 'close', 'session_id': o.session_id})
    else:
        request_auto(env, enabled=False)
    assert env.auto.reason == reason and env.auto.immediate is immediate
    run(env, 1.0)
    profile = [chassis(w, env)[0] for w in speeds(env, start)]
    if immediate:
        assert profile[0] == 0.0
    else:
        assert profile[0] > 0 and decelerates(profile)
    assert profile[-1] == 0.0 and env.auto.state == 'NONE'


def test_takeover_key_only_stops_and_manual_drive_needs_a_new_press():
    env = driving(make())
    keys(env, ['w'])
    assert env.auto.state == 'STOPPING' and env.auto.reason == 'MANUAL_TAKEOVER'
    run(env, 1.0, held=['w'])                       # still held after the standstill
    assert env.auto.state == 'NONE' and env.owner.state == 'READY'
    assert speeds(env)[-1] == (0, 0)
    held_writes = len(env.channel.requests)
    run(env, 0.5, held=['w'])
    assert not any(w != (0, 0) for w in speeds(env, held_writes))
    keys(env, [])                                   # release clears the latch
    assert not env.auto.takeover_latch
    run(env, 1.0, held=['w'])                       # a new press drives manually again
    assert env.owner.state == 'ARMED'
    manual = chassis(speeds(env)[-1], env)[0]
    assert abs(manual - env.owner.control['max_linear_m_s']) < 2e-3


@pytest.mark.parametrize('cause', ['dead_man', 'estop', 'switch_stale'])
def test_switches_write_zero_at_once(cause):
    env = driving(make())
    env.channel.words = FORWARD                    # still rolling: the stop is not confirmed at once
    start = len(env.channel.requests)
    if cause == 'dead_man':
        env.switches.dead_man = False
    elif cause == 'estop':
        env.switches.estop = True
    else:
        env.switches.frozen = env.clock()
        env.clock.advance(0.3)
    run(env, 0.06)
    first = speeds(env, start)
    assert first and first[0] == (0, 0), first   # no deceleration ramp
    assert env.auto.state == 'STOPPING' and env.auto.immediate


@pytest.mark.parametrize('cause, reason', [('estop', 'ESTOP'), ('dead_man', 'DEAD_MAN_RELEASED'),
                                           ('switch_stale', 'SWITCH_INPUT_STALE'), ('gate_estop', 'GATE_ESTOP_LATCHED')])
def test_an_emergency_during_a_controlled_stop_writes_zero_at_once(cause, reason):
    env = driving(make())
    request_auto(env, enabled=False)                # USER_STOPPED_AUTO decelerates at 0.60 m/s²
    run(env, 0.05)
    assert env.auto.state == 'STOPPING' and not env.auto.immediate
    assert chassis(speeds(env)[-1], env)[0] > 0     # still decelerating
    if cause == 'estop':
        env.switches.estop = True
    elif cause == 'dead_man':
        env.switches.dead_man = False
    elif cause == 'switch_stale':
        env.switches.frozen = env.clock() - 0.5
    else:
        env.state, env.linear = 'ESTOP_LATCHED', 0.0
        decision(env)
    start = len(env.channel.requests)
    run(env, 0.06)
    assert speeds(env, start)[0] == (0, 0)
    assert env.auto.immediate and env.auto.immediate_reason == reason and env.auto.reason == 'USER_STOPPED_AUTO'


@pytest.mark.parametrize('cause', ['bridge', 'stale'])
def test_bridge_loss_or_stale_decisions_decelerate_to_zero(cause):
    env = driving(make())
    start = len(env.channel.requests)
    if cause == 'bridge':
        env.auto.on_bridge_lost('TEST')
        reason = 'BRIDGE_DISCONNECTED'
    else:
        env.gate = False
        reason = 'DECISION_STALE'
    run(env, 1.0)
    assert env.auto.reason.endswith(reason)
    profile = [chassis(w, env)[0] for w in speeds(env, start)]
    assert profile[0] > 0 and profile[-1] == 0.0 and decelerates(profile)


def test_interface_loss_stops_through_the_manual_layer_without_nonzero_after():
    env = driving(make())
    env.ui = False
    run(env, 0.3)
    assert env.auto.reason == 'MANUAL_LAYER_STOP' or env.auto.reason.startswith('AUTO_EXITED')
    index = max(i for i, r in enumerate(env.channel.requests) if r[1] == 0x10 and r != m.ZERO_REQUEST)
    run(env, 1.0)
    assert all(r == m.ZERO_REQUEST or r[1] != 0x10 for r in env.channel.requests[index+1:])
    assert env.auto.state == 'NONE'


def test_no_servo_release_after_auto_until_the_user_drives_and_releases_again():
    env = driving(make())
    request_auto(env, enabled=False)
    run(env, 2.0)
    assert env.auto.state == 'NONE' and env.owner.hybrid_release_inhibited
    count = env.channel.requests.count(m.RELEASE_REQUEST)
    run(env, 2.0)                                            # idle heartbeats alone never release
    assert env.channel.requests.count(m.RELEASE_REQUEST) == count
    run(env, 0.6, held=['w'])                                # existing hybrid rule: a normal key release
    run(env, 1.5)
    assert env.channel.requests.count(m.RELEASE_REQUEST) == count+1


def test_hand_push_request_during_auto_stops_without_release():
    env = driving(make())
    o = env.owner
    o.command({'type': 'push_mode', 'session_id': o.session_id, 'sequence': o.last_key_sequence+1,
               'arm_generation': o.arm_generation, 'enabled': True})
    assert env.auto.reason == 'USER_HAND_PUSH_REQUEST'
    count = env.channel.requests.count(m.RELEASE_REQUEST)
    run(env, 1.5)
    assert env.auto.state == 'NONE' and env.channel.requests.count(m.RELEASE_REQUEST) == count


# ---------------------------------------------------------------- hold-to-run switch not installed
def test_switch_not_installed_reports_null_and_the_operator_start_is_the_enable():
    env = make(hold='NOT_INSTALLED')
    run(env, 1.0)
    status = env.owner.status()['auto']
    assert status['dead_man'] is None and status['estop'] is None and status['switch_input'] is False
    assert status['hold_to_run_switch'] == 'NOT_INSTALLED' and status['enable_source'] == 'OPERATOR_START'
    assert status['enable_request'] is False
    env.linear = 0.1
    request_auto(env)
    assert env.owner.status()['auto']['enable_request'] is True and env.auto.state == 'INITIALIZING'
    run(env, 1.5)
    assert env.auto.state == 'AUTO'
    with pytest.raises(m.FeedbackError, match='not installed'):
        a.AutoDrive(env.owner, LIMITS, hold_to_run_switch='NOT_INSTALLED', switches=Switches(env.clock))


@pytest.mark.parametrize('how', ['stop_auto', 'space', 'key'])
def test_without_the_switch_stop_auto_space_and_any_key_end_the_enable(how):
    env = driving(make(hold='NOT_INSTALLED'))
    o = env.owner
    if how == 'stop_auto':
        request_auto(env, enabled=False)
    elif how == 'space':
        o.command({'type': 'disarm', 'session_id': o.session_id})
    else:
        keys(env, ['w'])
    assert env.auto.state == 'STOPPING' and o.status()['auto']['enable_request'] is False
    run(env, 1.0, held=['w'] if how == 'key' else None)
    assert env.auto.state == 'NONE'


def test_without_the_switch_a_latched_gate_is_released_by_the_new_start():
    env = make(hold='NOT_INSTALLED')
    env.state = 'ESTOP_LATCHED'                     # the panel's software emergency stop latched the gate
    run(env, 1.0)
    env.linear = 0.1
    request_auto(env)
    status = env.owner.status()['auto']
    assert env.auto.state == 'NONE' and status['start_pending'] and status['enable_request'] is True
    run(env, 0.3)                                   # the bridge sends a new enable; the gate unlatches
    env.state = 'ACTIVE'
    run(env, 1.5)
    assert env.auto.state == 'AUTO' and env.auto.refusal is None


def test_without_the_switch_a_gate_that_stays_latched_refuses_after_one_second():
    env = make(hold='NOT_INSTALLED')
    env.state = 'ESTOP_LATCHED'
    run(env, 1.0)
    request_auto(env)
    run(env, 0.5)
    assert env.owner.status()['auto']['enable_request'] is True
    run(env, 0.6)
    assert env.auto.state == 'NONE' and env.auto.refusal == 'AUTO_REFUSED: GATE_ESTOP_LATCHED'
    assert env.owner.status()['auto']['enable_request'] is False
    request_auto(env)                               # a stop request cancels a pending start
    request_auto(env, enabled=False)
    assert env.owner.status()['auto']['enable_request'] is False and not env.auto.start_pending_until


def test_vehicle_emergency_stop_is_inferred_from_stopped_wheels_and_needs_a_new_start():
    env = driving(make(hold='NOT_INSTALLED'))
    env.channel.words = (0, 0)                      # the vehicle's emergency stop: the driver stops the wheels
    run(env, 0.2)
    assert env.auto.state == 'AUTO'
    assert until(env, lambda: env.auto.reason == 'WHEELS_STOPPED_WHILE_COMMANDED', 0.4)
    start = len(env.channel.requests)
    run(env, 0.06)
    assert env.auto.immediate and speeds(env, start)[0] == (0, 0)
    run(env, 1.0)
    assert env.auto.state == 'NONE' and env.owner.initialized_by_user is False
    assert env.owner.status()['auto']['enable_request'] is False
    env.channel.words = None                        # emergency stop released: nothing moves until a new start
    run(env, 1.0)
    assert env.auto.state == 'NONE' and speeds(env)[-1] == (0, 0)
    start = len(env.channel.requests)
    request_auto(env)
    run(env, 1.5)
    assert env.auto.state == 'AUTO'
    writes = [r for r in env.channel.requests[start:] if r != m.QUERY]
    assert writes[:5] == [*m.INITIALIZATION_WRITES[:3], m.ZERO_REQUEST, m.INITIALIZATION_WRITES[3]]


def test_wheels_get_time_to_start_before_the_stopped_wheels_rule():
    env = make(hold='NOT_INSTALLED')
    run(env, 1.0)
    env.linear = 0.1
    env.channel.words = (0, 0)                      # the wheels lag behind the first commands
    request_auto(env)
    assert until(env, lambda: env.auto.commanded_since is not None, 2.0)
    run(env, 0.15)
    env.channel.words = None                        # and then turn
    run(env, 0.6)
    assert env.auto.state == 'AUTO', env.auto.reason


@pytest.mark.parametrize('pattern', [[(0, 1)], [(0, 0), (1, 0)], [(-1, 2)]], ids=['one_word', 'alternating', 'both'])
def test_inferred_emergency_stop_also_sees_wheels_that_read_a_word_or_two(pattern):
    """Second review of 2026-11-10, item 1: after the emergency stop a wheel may still read a word or two."""
    env = driving(make(hold='NOT_INSTALLED'))
    readings = itertools.cycle(pattern)
    env.channel.feedback = lambda: next(readings)   # raw words; -1 is 0xffff on the line
    run(env, 0.2)
    assert env.auto.state == 'AUTO'
    assert until(env, lambda: env.auto.reason == 'WHEELS_STOPPED_WHILE_COMMANDED', 0.4)
    start = len(env.channel.requests)
    run(env, 0.06)
    assert env.auto.immediate and speeds(env, start)[0] == (0, 0)
    status = env.owner.status()['auto']
    assert status['estop_request'] is True and status['enable_request'] is False
    # The stop is never confirmed while a wheel keeps reading: zero stays written and the owner faults.
    assert until(env, lambda: env.owner.state == 'FAULT', 1.5)
    assert 'NOT_STATIONARY_AFTER_STOP' in env.owner.failure and env.channel.requests[-1] == m.ZERO_REQUEST
    assert not any(w != (0, 0) for w in speeds(env, start))


def test_inferred_emergency_stop_needs_both_wheels_below_a_fifth_of_the_command():
    env = driving(make(hold='NOT_INSTALLED'))
    follow = env.channel.follow_factor
    env.channel.follow_factor = follow * 0.3          # both wheels slow, but turning: a heavy load, not a stop
    run(env, 1.0)
    assert env.auto.state == 'AUTO', env.auto.reason
    env.channel.follow_factor = follow
    run(env, 0.3)
    env.channel.words = (0, FORWARD[1])               # one wheel stopped, the other turning: not the emergency stop
    run(env, 1.0)
    assert env.auto.state == 'AUTO', env.auto.reason
    env.channel.words = None
    env.channel.follow_factor = follow * 0.15         # coasting down after the emergency stop
    assert until(env, lambda: env.auto.reason == 'WHEELS_STOPPED_WHILE_COMMANDED', 0.6)
    env.channel.follow_factor, env.channel.words = follow, (0, 0)
    assert until(env, lambda: env.auto.state == 'NONE', 1.0)   # standing still: back to NONE, no fault
    assert env.owner.state != 'FAULT' and env.owner.initialized_by_user is False


# ---------------------------------------------------------------- faults and exits (section 6)
def test_wheels_still_turning_after_the_stop_faults_the_owner():
    env = driving(make())
    env.channel.words = FORWARD
    env.auto.on_bridge_lost('TEST')
    with_error = False
    for _ in range(200):
        keys(env, [])
        env.owner.step(); env.clock.advance(0.01)
        if env.owner.state == 'FAULT':
            with_error = True
            break
    assert with_error and 'NOT_STATIONARY_AFTER_STOP' in env.owner.failure
    assert env.auto.state == 'FAULT' and env.channel.requests[-1] == m.ZERO_REQUEST


def test_wheels_turning_during_a_gate_hold_fault_the_owner():
    env = driving(make())
    env.state, env.linear = 'LOCALIZATION_UNTRUSTED', 0.0
    run(env, 0.5)
    env.channel.words = FORWARD                     # zero commanded for a while, wheels still turning
    assert until(env, lambda: env.owner.state == 'FAULT', 2.0)
    assert 'MOVING_WHILE_COMMANDED_ZERO' in env.owner.failure and env.channel.requests[-1] == m.ZERO_REQUEST


def test_stale_feedback_stops_and_without_feedback_the_stop_cannot_be_confirmed():
    env = driving(make())
    env.owner.next_feedback = env.clock() + 100.0   # no more wheel reads
    run(env, 0.3)
    assert env.auto.reason == 'FEEDBACK_STALE' and env.auto.state == 'STOPPING' and not env.auto.immediate
    assert until(env, lambda: env.owner.state == 'FAULT', 3.0)
    assert 'NOT_STATIONARY_AFTER_STOP' in env.owner.failure and env.channel.requests[-1] == m.ZERO_REQUEST


def test_backward_motion_stops_at_once():
    env = driving(make())
    env.channel.words = BACKWARD                    # left sign -1: both wheels report backward
    assert until(env, lambda: env.auto.reason == 'MOVING_BACKWARD', 0.5)
    start = len(env.channel.requests)
    run(env, 0.06)
    assert env.auto.immediate and speeds(env, start)[0] == (0, 0)


def test_serial_failure_during_auto_latches_fault_and_close_resends_zero():
    env = driving(make())
    env.channel.failure = 'SYNTHETIC SERIAL FAILURE'
    run(env, 0.2)
    assert env.owner.state == 'FAULT' and env.auto.state == 'FAULT'
    env.owner.close()
    assert env.channel.requests[-1] == m.ZERO_REQUEST and env.channel.recovery_attempted


def test_session_cancel_during_auto_stops_at_once():
    flag = [False]
    env = driving(make(cancel=lambda: flag[0]))
    flag[0] = True
    start = len(env.channel.requests)
    run(env, 0.06)
    assert env.auto.reason == 'MANUAL_LAYER_STOP' and env.auto.immediate
    assert speeds(env, start)[0] == (0, 0)


def test_owner_close_during_auto_writes_zero():
    env = driving(make())
    env.owner.close()
    assert env.channel.requests[-1] == m.ZERO_REQUEST


def test_a_failing_journal_at_session_exit_still_lets_the_owner_write_zero():
    fail = [False]

    def journal(record):
        if fail[0]:
            raise m.FeedbackError('journal queue full')
    env = driving(make(journal=journal))
    fail[0] = True
    env.auto.on_bridge_lost('SESSION_EXIT', closing=True)       # what AutoSocket.close() does on session exit
    assert env.auto.state == 'NONE'
    env.owner.close()
    assert env.channel.requests[-1] == m.ZERO_REQUEST


def test_status_is_json_and_never_claims_a_physical_stop():
    env = driving(make())
    status = env.owner.status()['auto']
    json.dumps(status, allow_nan=False)
    assert status['physical_stop_confirmed'] is False and status['dead_man'] is True and status['estop'] is False
    assert status['limits'] == LIMITS and status['gate_state'] == 'ACTIVE'
    assert status['hold_to_run_switch'] == 'INSTALLED' and status['enable_source'] == 'HOLD_TO_RUN_SWITCH'
    assert status['enable_request'] is True and status['refusal'] is None and status['immediate_reason'] is None
    assert status['gate_id'] == status['entry_gate_id'] == 'gate_test'
    feedback = status['feedback']
    assert feedback['sequence'] == env.owner.sequence - 1 and 0 <= feedback['age_s'] <= m.FEEDBACK_FRESH_S
    assert abs(feedback['stamp_mono_ns'] - env.clock.ns()) <= feedback['age_s'] * 1e9 + 1
    assert p.parse_status(status, env.owner.session_id) is status


# ---------------------------------------------------------------- attach_auto and the socket
def evidence_project(tmp, hold=None):
    root = tmp/'project'
    (root/'evidence').mkdir(parents=True, exist_ok=True)
    source = root/'evidence'/'stand_test.txt'
    source.write_text('synthetic stand test record\n')
    digest = __import__('hashlib').sha256(source.read_bytes()).hexdigest()
    test = {'verified': True, 'trials': 3, 'max_stop_time_s': 0.4,
            'sources': [{'kind': 'recording', 'path': 'evidence/stand_test.txt', 'sha256': digest}]}
    names = ('estop', 'dead_man', 'owner_killed', 'usb_unplugged', 'orin_power_cut')
    evidence = {'schema': 'wc_auto_drive_evidence_v1', 'state': 'REVIEWED', 'stage': 'STAND_TEST',
                'device_id': 'ZLAC8030D-0000000014', 'firmware_id': 'SYNTHETIC', 'reviewer': 'test',
                'reviewed_at': '2026-11-09', 'hold_to_run_switch': hold or {'installed': True},
                'communication_timeout_s': 0.3, 'approved_limits': dict(LIMITS),
                'driver_readback': [{'purpose': 'communication_timeout', 'request_hex': TIMEOUT_REQUEST.hex(),
                                     'word_index': 0, 'mask': 65535, 'equals': 7}],
                'stop_tests': {name: dict(test) for name in names if hold is None or name != 'dead_man'}}
    evidence['stop_tests']['estop']['moved_after_reset'] = False
    (root/'evidence'/'auto.json').write_text(json.dumps(evidence))
    return root


def attached(tmp, *, controls=None, environ=None, switches=True, readback=True, socket_path=None, hold=None,
             lease=None):
    import datetime
    root = evidence_project(tmp, hold)
    cfg = config()
    cfg['auto_controls'] = controls or {'allowed': True, 'evidence': 'evidence/auto.json',
                                        'hold_to_run_switch': 'INSTALLED', **LIMITS}
    clock, channel, records = Clock(), FakeChannel(), []
    channel.lease = lease   # the vehicle's serial line, for the startup readback
    owner = m.MappingWheel(cfg, channel, records.append, clock=clock, wall_ns=clock.wall, mono_ns=clock.ns)
    today = datetime.date(2026, 11, 9)
    env = {'WC_AUTO_DRIVE_APPROVAL': 'APPROVED_BY_USER_20261109'} if environ is None else environ
    sock = a.attach_auto(cfg, owner, root, tmp/'session', environ=env, today=today,
                         switches=Switches(clock) if switches else None,
                         readback=(lambda request: [7]) if readback else None, socket_path=socket_path)
    return owner, sock, records


@pytest.fixture
def short_tmp():
    with tempfile.TemporaryDirectory(prefix='wc-auto-', dir='/tmp') as folder:
        yield Path(folder)


NOT_INSTALLED_CONTROLS = {'allowed': True, 'evidence': 'evidence/auto.json', 'hold_to_run_switch': 'NOT_INSTALLED',
                          **LIMITS}


@pytest.mark.parametrize('case, reason', [
    ('not_allowed', 'AUTO_NOT_ALLOWED'), ('no_approval', 'NO_USER_APPROVAL_TODAY'),
    ('old_approval', 'NO_USER_APPROVAL_TODAY'), ('template', 'EVIDENCE_REJECTED'),
    ('no_switches', 'NO_SWITCH_INPUT'),
    ('differs', 'HOLD_TO_RUN_SWITCH_DIFFERS_FROM_EVIDENCE'), ('reader_given', 'SWITCH_READER_GIVEN_BUT_NOT_INSTALLED')])
def test_attach_auto_stays_unavailable_without_every_precondition(short_tmp, case, reason):
    controls = None
    environ = None
    hold = None
    switches = case != 'no_switches'
    if case == 'not_allowed':
        controls = {'allowed': False}
    elif case == 'no_approval':
        environ = {}
    elif case == 'old_approval':
        environ = {'WC_AUTO_DRIVE_APPROVAL': 'APPROVED_BY_USER_20261108'}
    elif case == 'template':
        controls = {'allowed': True, 'evidence': str(ROOT/'config/wheel_auto_drive_evidence.template.json'),
                    'hold_to_run_switch': 'INSTALLED', **LIMITS}
    elif case == 'differs':
        controls = NOT_INSTALLED_CONTROLS                    # the evidence says installed
        switches = False
    elif case == 'reader_given':
        controls, hold = NOT_INSTALLED_CONTROLS, NOT_INSTALLED
    # No readback reader and no serial line: every refusal here comes before the startup readback would read.
    owner, sock, records = attached(short_tmp, controls=controls, environ=environ, switches=switches,
                                    readback=False, socket_path=short_tmp/'auto.sock', hold=hold)
    assert sock is None and not (short_tmp/'auto.sock').exists()
    assert owner.auto.state == 'UNAVAILABLE' and owner.auto.reason.startswith(reason), owner.auto.reason
    assert any(r.get('event') == 'auto_attached' for r in records)
    assert not any(r.get('event', '').startswith('auto_readback') for r in records)


class DriverEnd(threading.Thread):
    """The driver end of a socket pair: answers every 8-byte request with ``reply(request)`` (None: silence)."""
    def __init__(self, sock, reply):
        super().__init__(daemon=True)
        self.sock, self.reply, self.requests = sock, reply, []

    def run(self):
        while True:
            try:
                request = self.sock.recv(8)
            except OSError:
                return
            if len(request) < 8:
                return
            self.requests.append(request)
            answer = self.reply(request)
            if answer is not None:
                self.sock.sendall(answer)


@pytest.mark.parametrize('answer', ['words', 'exception', 'silent'])
def test_vehicle_owner_reads_the_evidence_registers_once_at_session_start(short_tmp, answer):
    near, far = sk.socketpair()

    def reply(request):
        payload = {'words': b'\x01\x03\x02\x00\x07', 'exception': b'\x01\x83\x02'}.get(answer)
        return None if payload is None else payload + m.crc16(payload)
    driver = DriverEnd(far, reply)
    driver.start()
    lease = NS(fd=near.fileno(), failed=False)
    try:
        if answer == 'silent':
            with pytest.raises(m.FeedbackError, match='AUTO_STARTUP_READBACK_FAILED: communication_timeout'):
                attached(short_tmp, readback=False, lease=lease, socket_path=short_tmp/'auto.sock')
            assert lease.failed and not (short_tmp/'auto.sock').exists()
            return
        owner, sock, records = attached(short_tmp, readback=False, lease=lease, socket_path=short_tmp/'auto.sock')
        reads = [r for r in records if r.get('event') == 'auto_readback']
        assert driver.requests == [TIMEOUT_REQUEST] and len(reads) == 1
        assert reads[0]['request_hex'] == TIMEOUT_REQUEST.hex() and reads[0]['purpose'] == 'communication_timeout'
        if answer == 'words':
            assert sock is not None and owner.auto.state == 'NONE' and reads[0]['words_u16'] == [7]
            assert owner.auto.readback(TIMEOUT_REQUEST) == [7] and driver.requests == [TIMEOUT_REQUEST]  # cached
            sock.close()
        else:
            assert sock is None and owner.auto.reason.startswith('DRIVER_READBACK_EXCEPTION: communication_timeout')
            assert reads[0]['exception_code'] == 2 and not lease.failed
    finally:
        near.close()
        far.close()
        driver.join(timeout=2)


def test_attach_auto_with_the_switch_not_installed_needs_no_reader(short_tmp):
    owner, sock, records = attached(short_tmp, controls=NOT_INSTALLED_CONTROLS, switches=False, hold=NOT_INSTALLED,
                                    socket_path=short_tmp/'auto.sock')
    assert sock is not None and owner.auto.state == 'NONE' and owner.auto.enable_source == 'OPERATOR_START'
    status = owner.auto.status()
    assert status['dead_man'] is None and status['estop'] is None and status['hold_to_run_switch'] == 'NOT_INSTALLED'
    attached_record = [r for r in records if r.get('event') == 'auto_attached'][0]
    assert attached_record['hold_to_run_switch'] == 'NOT_INSTALLED'
    sock.close()


def test_attach_auto_rejects_malformed_controls(short_tmp):
    with pytest.raises(m.FeedbackError):
        attached(short_tmp, controls={'allowed': True, 'evidence': 'evidence/auto.json',
                                      'hold_to_run_switch': 'INSTALLED', 'max_linear_m_s': 0.5})
    with pytest.raises(m.FeedbackError):
        attached(short_tmp, controls={'allowed': 'yes'})
    with pytest.raises(m.FeedbackError, match='hold_to_run_switch'):
        attached(short_tmp, controls={'allowed': True, 'evidence': 'evidence/auto.json', **LIMITS})
    with pytest.raises(m.FeedbackError, match='hold_to_run_switch'):
        attached(short_tmp, controls={'allowed': True, 'evidence': 'evidence/auto.json', 'hold_to_run_switch': 'MAYBE',
                                      **LIMITS})


def test_socket_hello_decisions_and_status(short_tmp):
    owner, sock, records = attached(short_tmp, socket_path=short_tmp/'auto.sock')
    assert sock is not None and owner.auto.state == 'NONE'
    assert oct((short_tmp/'auto.sock').stat().st_mode & 0o777) == '0o600'
    client = sk.socket(sk.AF_UNIX, sk.SOCK_STREAM); client.connect(str(short_tmp/'auto.sock'))
    second = sk.socket(sk.AF_UNIX, sk.SOCK_STREAM); second.connect(str(short_tmp/'auto.sock'))
    client.sendall(p.encode_line(p.hello_packet(owner.session_id, 'bridge_test')))
    sock.pump(); sock.pump()
    assert owner.auto.bridge_connected and owner.auto.bridge_id == 'bridge_test'
    gate = {'gate_id': 'gate_test', 'seq': 1, 'stamp_ns': 1, 'mono_ns': 5, 'state': 'ACTIVE', 'reasons': [],
            'linear': 0.05, 'angular': 0.0, 'config_sha256': None,
            'limits': {'max_linear': 0.1, 'max_angular': 0.3, 'linear_ceiling': 0.1, 'allow_reverse': False}}
    client.sendall(p.encode_line(p.decision_packet(gate, owner.session_id, 1, 10)))
    sock.pump()
    assert owner.auto.decision['linear'] == 0.05 and owner.auto.counters['decisions'] == 1
    client.settimeout(1.0)
    line = client.recv(65536).split(b'\n')[0]
    status = p.parse_status(p.decode_line(line), owner.session_id)
    assert status['auto_state'] == 'NONE' and status['bridge_connected'] is True
    second.settimeout(0.2)
    assert second.recv(16) == b''                                # a second client is closed at once
    client.sendall(p.encode_line(p.decision_packet(gate, owner.session_id, 1, 11)))   # sequence did not increase
    sock.pump()
    assert not owner.auto.bridge_connected and sock.client is None
    sock.close()
    assert not (short_tmp/'auto.sock').exists()


def test_socket_requires_hello_within_one_second(short_tmp):
    owner, sock, _ = attached(short_tmp, socket_path=short_tmp/'auto.sock')
    client = sk.socket(sk.AF_UNIX, sk.SOCK_STREAM); client.connect(str(short_tmp/'auto.sock'))
    sock.pump()
    owner.clock.advance(1.1)
    sock.pump()
    assert sock.client is None and not owner.auto.bridge_connected
    sock.close()


def connected_bridge(env, path):
    """A raw client in place of the base bridge, connected to a real AutoSocket of ``env``."""
    env.auto.on_bridge_lost('RESET')
    sock = a.AutoSocket(path, env.auto)
    client = sk.socket(sk.AF_UNIX, sk.SOCK_STREAM); client.connect(str(path))
    client.sendall(p.encode_line(p.hello_packet(env.owner.session_id, 'bridge_raw')))
    sock.pump()
    client.setblocking(False)
    assert env.auto.bridge_connected
    return sock, client


def test_navigation_cannot_request_auto_over_the_bridge_socket(short_tmp):
    env = make()
    run(env, 1.0)
    sock, client = connected_bridge(env, short_tmp/'auto.sock')
    o = env.owner
    client.sendall(p.encode_line({'type': 'auto', 'session_id': o.session_id, 'sequence': o.last_key_sequence+1,
                                  'arm_generation': o.arm_generation, 'enabled': True, 'foreground': True}))
    for _ in range(3):
        sock.pump()
    assert not env.auto.bridge_connected and sock.client is None
    assert env.auto.state == 'NONE' and env.auto.counters['entries'] == 0
    sock.close()


def test_bridge_socket_closing_during_auto_decelerates_to_zero(short_tmp):
    env = make()
    sock, client = connected_bridge(env, short_tmp/'auto.sock')
    env.owner.before_control = sock.pump
    env.gate = False                                 # decisions now only come through the socket

    def tick(seconds):
        for _ in range(round(seconds/0.01)):
            keys(env, [])
            if env.tick % 5 == 0 and client.fileno() != -1:
                env.seq += 1
                gate = {'gate_id': env.gate_id, 'seq': env.seq, 'stamp_ns': 1, 'mono_ns': env.clock.ns(),
                        'state': 'ACTIVE', 'reasons': [], 'linear': env.linear, 'angular': 0.0, 'config_sha256': None,
                        'limits': {'max_linear': 0.1, 'max_angular': 0.3, 'linear_ceiling': 0.1,
                                   'allow_reverse': False}}
                client.sendall(p.encode_line(p.decision_packet(gate, env.owner.session_id, env.seq, env.clock.ns())))
            env.tick += 1
            sock.pump()
            env.owner.step()
            env.clock.advance(0.01)
            if client.fileno() != -1:
                try:
                    while client.recv(65536):
                        pass
                except BlockingIOError:
                    pass
    tick(1.0)
    env.linear = 0.1
    request_auto(env)
    tick(1.5)
    assert env.auto.state == 'AUTO', (env.auto.reason, env.auto.refusal)
    start = len(env.channel.requests)
    client.close()                                   # the bridge process went away
    tick(1.0)
    assert env.auto.reason == 'AUTO_EXITED: BRIDGE_DISCONNECTED' and not env.auto.bridge_connected
    profile = [chassis(w, env)[0] for w in speeds(env, start)]
    assert profile[0] > 0 and profile[-1] == 0.0 and decelerates(profile)
    sock.close()


@pytest.mark.parametrize('variant', ['absent', 'present', 'close_fails'])
def test_main_attaches_auto_only_when_the_session_configuration_has_auto_controls(tmp_path, monkeypatch, variant):
    """The real entrypoint with in-memory stand-ins: no serial port, no ROS."""
    from types import ModuleType
    from wc_motion import feedback_transport as ft
    cli = ModuleType('wc_runtime.cli')
    cli.ROOT, cli.RUN, cli.target = tmp_path, tmp_path/'run', lambda: None
    monkeypatch.setitem(sys.modules, 'wc_runtime.cli', cli)
    hardware = tmp_path/'hardware.json'
    hardware.write_bytes((ROOT/'config/wheel_feedback_current.json').read_bytes())
    session = tmp_path/'session'; session.mkdir()
    cfg = {**config(), 'duration_s': 0, 'wheel_hardware_config': str(hardware), 'source_mode': 'real'}
    if variant != 'absent':
        cfg['auto_controls'] = {'allowed': False}
    (session/'runtime_config.json').write_text(json.dumps(cfg))
    calls, signals = [], {}
    monkeypatch.setattr(m.signal, 'SIGHUP', 1, raising=False)
    monkeypatch.setattr(m.signal, 'signal', lambda number, handler: signals.update({number: handler}))
    monkeypatch.setattr(m.time, 'sleep', lambda _: None)

    class Lease:
        host_configuration = {'fixture': True}
        def __init__(self, *_): pass
        def open(self): pass
        def close(self): calls.append('lease_close')

    class Journal:
        def __call__(self, row): pass
        def close(self): pass
        def status(self): return {}

    class Owner:
        failure = None
        def __init__(self, *args, **kwargs): self.channel = NS(control_transmissions=0)
        def step(self):
            calls.append('step')
            signals[m.signal.SIGINT](m.signal.SIGINT, None)
        def status(self): return {'state': 'READY'}
        def close(self): calls.append('owner_close')
        def fault(self, reason): self.failure = reason

    attaches = []

    def auto_close():
        calls.append('auto_close')
        if variant == 'close_fails':
            raise m.FeedbackError('journal queue full')

    def fake_attach(config_value, owner, root, directory):
        attaches.append(config_value['auto_controls'])
        return NS(pump=lambda: calls.append('auto_pump'), close=auto_close)
    monkeypatch.setattr(ft, 'FeedbackSerialLease', Lease)
    monkeypatch.setattr(ft, 'create_journal', lambda *args, **kwargs: Journal())
    monkeypatch.setattr(m, 'RosOutput', lambda: NS(publish=lambda *_: None, manual=NS(publish=lambda _: None),
                                                   String=lambda **kwargs: NS(**kwargs), close=lambda: None))
    monkeypatch.setattr(m, 'WheelChannel', lambda *_: None)
    monkeypatch.setattr(m, 'MappingWheel', Owner)
    monkeypatch.setattr(m, 'ManualSocket', lambda *_: NS(pump=lambda: None, close=lambda: calls.append('socket_close')))
    monkeypatch.setattr(a, 'attach_auto', fake_attach)
    result = m.main(['--session-root', str(session)])
    if variant == 'absent':
        assert result == 0 and attaches == [] and 'auto_pump' not in calls and 'auto_close' not in calls
    else:
        assert attaches == [{'allowed': False}]
        assert calls.index('auto_pump') < calls.index('step')
        assert calls.index('auto_close') < calls.index('socket_close') < calls.index('owner_close') < \
            calls.index('lease_close')
        assert result == (1 if variant == 'close_fails' else 0)   # the session reports it, after writing zero


def software_estop(env, pressed):
    o = env.owner
    o.command({'type': 'estop', 'session_id': o.session_id, 'sequence': o.last_key_sequence+1, 'pressed': pressed})


def test_panel_software_estop_stops_at_once_blocks_starts_and_is_reported():
    env = driving(make(hold='NOT_INSTALLED'))
    start = len(env.channel.requests)
    software_estop(env, True)
    assert env.auto.reason == 'SOFTWARE_ESTOP' and env.auto.immediate
    run(env, 0.06)
    assert speeds(env, start)[0] == (0, 0)
    status = env.owner.status()['auto']
    assert status['estop_request'] is True and status['estop_reasons'] == ['PANEL_SOFTWARE_ESTOP']
    assert status['enable_request'] is False
    run(env, 1.0)
    assert env.auto.state == 'NONE'
    request_auto(env)
    assert env.auto.refusal == 'AUTO_REFUSED: SOFTWARE_ESTOP_PRESSED' and env.auto.state == 'NONE'
    software_estop(env, False)
    assert env.owner.status()['auto']['estop_request'] is False
    request_auto(env)                                # released: a new start drives again
    run(env, 1.5)
    assert env.auto.state == 'AUTO'


def test_panel_software_estop_also_stops_manual_driving():
    env = make(hold='NOT_INSTALLED')
    run(env, 1.0)
    run(env, 1.0, held=['w'])
    assert env.owner.state == 'ARMED'
    software_estop(env, True)
    assert env.owner.state == 'READY' and env.owner.reason == 'SOFTWARE_ESTOP'
    run(env, 0.1)
    assert speeds(env)[-1] == (0, 0)
    with pytest.raises(m.FeedbackError):
        env.owner.command({'type': 'estop', 'session_id': env.owner.session_id,
                           'sequence': env.owner.last_key_sequence, 'pressed': True})   # sequence not increasing
    with pytest.raises(m.FeedbackError):
        env.owner.command({'type': 'estop', 'session_id': env.owner.session_id,
                           'sequence': env.owner.last_key_sequence+1})                 # pressed missing


def test_an_inferred_hardware_stop_is_reported_for_half_a_second_so_the_gate_latches():
    env = driving(make(hold='NOT_INSTALLED'))
    env.channel.words = (0, 0)
    assert until(env, lambda: env.auto.reason == 'WHEELS_STOPPED_WHILE_COMMANDED', 0.7)
    began = env.clock()
    status = env.owner.status()['auto']
    assert status['estop_request'] is True and status['estop_reasons'] == ['WHEELS_STOPPED_WHILE_COMMANDED']
    assert until(env, lambda: not env.owner.status()['auto']['estop_request'], 1.0)
    assert 0.5 - 0.011 <= env.clock() - began <= 0.5 + 0.011


# ---------------------------------------------------------------- operator command line (operator.sock)
def unfocused(env, seconds):
    """The operator works in a terminal: RViz sends foreground false heartbeats, the gate keeps deciding."""
    for _ in range(round(seconds/0.01)):
        keys(env, [], foreground=False)
        if env.gate and env.tick % 5 == 0:
            decision(env)
        env.tick += 1
        env.owner.step()
        env.clock.advance(0.01)


def test_operator_start_waits_for_the_rviz_focus_and_then_checks_every_entry_condition():
    env = make()
    run(env, 1.0)                                    # RViz has the focus, idle: hand-push release
    env.linear = 0.10
    unfocused(env, 0.5)                              # the operator clicks into a terminal and types the command
    env.auto.operator_request(True)
    unfocused(env, 3.0)
    status = env.auto.status()
    assert env.auto.state == 'NONE' and status['operator_pending'] and env.auto.refusal is None
    assert not status['enable_request'] and env.auto.counters['entries'] == 0
    run(env, 1.5)                                    # a click into RViz: fresh foreground heartbeats again
    assert env.auto.state == 'AUTO', (env.auto.reason, env.auto.refusal)
    assert not env.auto.status()['operator_pending']
    env.auto.operator_request(False)                 # "stop automatic" from the command line
    assert env.auto.state == 'STOPPING' and env.auto.reason == 'USER_STOPPED_AUTO' and not env.auto.immediate
    assert until(env, lambda: env.auto.state == 'NONE')
    requests = [r['enabled'] for r in env.records if r.get('event') == 'auto_operator_request']
    assert requests == [True, False]


def test_operator_start_is_refused_without_a_click_and_cancelled_by_keys_or_the_software_estop():
    env = make()
    run(env, 1.0)
    unfocused(env, 0.3)
    env.auto.operator_request(True)
    unfocused(env, a.OPERATOR_WAIT_S + 0.1)
    assert env.auto.state == 'NONE' and env.auto.refusal == 'AUTO_REFUSED: UI_NOT_FRESH'
    assert not env.auto.status()['operator_pending'] and env.auto.counters['entries'] == 0
    env.auto.operator_request(True)
    run(env, 0.01, held=['w'])                       # the first thing back in RViz is a key: manual driving only
    assert not env.auto.status()['operator_pending'] and env.auto.state == 'NONE'
    run(env, 1.0)
    unfocused(env, 0.3)
    env.auto.operator_request(True)
    software_estop(env, True)
    assert not env.auto.status()['operator_pending']
    env.auto.operator_request(True)                  # refused while the panel's emergency stop is pressed
    assert env.auto.refusal == 'AUTO_REFUSED: SOFTWARE_ESTOP_PRESSED' and not env.auto.status()['operator_pending']
    software_estop(env, False)
    env.auto.operator_request(False)                 # "stop automatic" when idle changes nothing
    assert env.auto.state == 'NONE' and env.auto.counters['entries'] == 0
    unavailable = make(unavailable='NO_SWITCH_INPUT')
    unavailable.auto.operator_request(True)
    assert unavailable.auto.refusal == 'AUTO_REFUSED: NO_SWITCH_INPUT'
    assert not unavailable.auto.status()['operator_pending']


def operator_client(path, message):
    client = sk.socket(sk.AF_UNIX, sk.SOCK_STREAM)
    client.connect(str(path))
    client.sendall(message if isinstance(message, bytes) else p.encode_line(message))
    return client


def operator_answer(sock, client):
    sock.pump()
    client.settimeout(1.0)
    data = b''
    while not data.endswith(b'\n'):
        chunk = client.recv(65536)
        if not chunk:
            break
        data += chunk
    client.close()
    return json.loads(data) if data else None


def test_operator_socket_answers_one_line_per_request_and_rejects_anything_else(short_tmp):
    import stat
    env = make()
    run(env, 1.0)
    sock = a.AutoSocket(short_tmp/'auto.sock', env.auto)
    path = short_tmp/'operator.sock'
    assert path.is_socket() and stat.S_IMODE(path.stat().st_mode) == 0o600
    sid = env.owner.session_id
    value = operator_answer(sock, operator_client(path, {'type': 'status', 'session_id': sid}))
    assert value['type'] == 'operator_status' and value['session_id'] == sid
    assert value['status']['auto_state'] == 'NONE' and value['status']['operator_pending'] is False
    value = operator_answer(sock, operator_client(path, {'type': 'auto_request', 'session_id': sid, 'enabled': True}))
    assert value['status']['operator_pending'] is True
    value = operator_answer(sock, operator_client(path, {'type': 'auto_request', 'session_id': sid, 'enabled': False}))
    assert value['status']['operator_pending'] is False
    for bad in ({'type': 'auto_request', 'session_id': 'other_session', 'enabled': True},
                {'type': 'auto_request', 'session_id': sid, 'enabled': 'yes'},
                {'type': 'auto', 'session_id': sid, 'enabled': True},
                {'type': 'status', 'session_id': sid, 'extra': 1},
                b'{"type": "status", "session_id": NaN}\n', b'not json\n'):
        assert operator_answer(sock, operator_client(path, bad)) is None    # closed without an answer
    assert not env.auto.status()['operator_pending'] and env.auto.counters['entries'] == 0
    silent = operator_client(path, b'{"type": "sta')
    sock.pump()
    env.clock.advance(a.OPERATOR_LINE_S + 0.1)
    sock.pump()
    silent.settimeout(1.0)
    assert silent.recv(16) == b''                                       # no complete line within 1 s: closed
    silent.close()
    assert len([r for r in env.records if r.get('event') == 'auto_operator_rejected']) == 7
    sock.close()
    assert not path.exists() and not (short_tmp/'auto.sock').exists()
