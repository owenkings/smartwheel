"""Automatic control source of the wheel owner (velocity_channel.md). Fake serial channel only: no device, no motion."""
import json
import math
from pathlib import Path
import struct
import sys
import tempfile
from types import SimpleNamespace as NS

import pytest

from wc_runtime import mapping_auto as a
from wc_runtime import mapping_wheel as m


ROOT = Path(__file__).resolve().parents[2]
LIMITS = {'max_linear_m_s': 0.10, 'max_angular_rad_s': 0.30, 'max_wheel_rpm': 27.0, 'max_linear_accel_m_s2': 0.30,
          'max_linear_decel_m_s2': 0.60, 'max_angular_accel_rad_s2': 0.80}
TIMEOUT_REQUEST = m.read_request(1, 0x2000, 1)   # synthetic register, not a reviewed ZLAC8030D address
FORWARD = ((-40) & 0xffff, 40)                  # left sign -1: both wheels report forward motion


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
    """Same contract as WheelChannel.exchange: guard right before a write, Cancelled propagates without sending."""
    def __init__(self):
        self.control_transmissions = self.control_attempts = self.control_bytes = 0
        self.motion_may_exist = self.failed = self.recovery_attempted = False
        self.last_stop = None
        self.requests, self.words, self.failure = [], (0, 0), None

    def exchange(self, request, timeout, *, guard=None):
        if guard is not None:
            guard()
        if self.failure:
            self.failed = True
            raise m.QueryFailure(self.failure)
        self.requests.append(request)
        if request == m.QUERY:
            return response(self.words)
        self.control_transmissions += 1; self.control_attempts += 1; self.control_bytes += len(request)
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


def make(*, unavailable=None, readback_value=7, switches=True):
    clock, channel, records = Clock(), FakeChannel(), []
    owner = m.MappingWheel(config(), channel, records.append, clock=clock, wall_ns=clock.wall, mono_ns=clock.ns)
    env = NS(owner=owner, clock=clock, channel=channel, records=records, reads=[], seq=0, ui=True, gate=True,
             linear=0.0, angular=0.0, state='ACTIVE', gate_limits=(0.10, 0.30), tick=0)
    env.switches = Switches(clock) if switches else None

    def readback(request):
        env.reads.append(request)
        return [readback_value]
    checks = [{'purpose': 'communication_timeout', 'request': TIMEOUT_REQUEST, 'word_index': 0, 'mask': 0xffff,
               'equals': 7}]
    env.auto = a.AutoDrive(owner, LIMITS, switches=env.switches, readback=readback, readback_checks=checks,
                           unavailable=unavailable)
    owner.connected()
    env.auto.on_bridge_connected('bridge_test')
    return env


def keys(env, pressed, foreground=True):
    o = env.owner
    o.command({'type': 'keys', 'session_id': o.session_id, 'sequence': o.last_key_sequence+1,
               'arm_generation': o.arm_generation, 'keys': pressed, 'foreground': foreground})


def decision(env):
    env.seq += 1
    env.auto.on_decision({'type': 'decision', 'session_id': env.owner.session_id, 'sequence': env.seq,
                          'received_mono_ns': env.clock.ns(), 'gate_seq': env.seq, 'gate_mono_ns': env.clock.ns(),
                          'state': env.state, 'reasons': [], 'linear': env.linear, 'angular': env.angular,
                          'limits': {'max_linear': env.gate_limits[0], 'max_angular': env.gate_limits[1]},
                          'config_sha256': None})


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
    assert env.auto.state == 'AUTO', env.auto.reason
    return env


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
    assert env.auto.state == 'UNAVAILABLE' and env.auto.reason == 'AUTO_REFUSED: NO_SWITCH_INPUT'
    assert not any(w != (0, 0) for w in speeds(env, before))
    request_auto(env)                              # repeated requests do not nest the reason
    assert env.auto.reason == 'AUTO_REFUSED: NO_SWITCH_INPUT'


@pytest.mark.parametrize('cause, reason', [
    ('bridge', 'BRIDGE_DISCONNECTED'), ('stale', 'DECISION_STALE'), ('limits', 'GATE_LIMITS_ABOVE_APPROVED'),
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
    elif cause == 'limits':
        env.gate_limits = (0.30, 0.50); run(env, 0.1)
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
    assert env.auto.state == 'NONE' and env.auto.reason.startswith('AUTO_REFUSED: '+reason), env.auto.reason
    run(env, 0.5, held=['w'] if cause == 'keys' else None)
    assert env.auto.state == 'NONE'
    if cause != 'keys':
        assert not any(w != (0, 0) for w in speeds(env, before))


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
    steps = [b - a for a, b in zip(profile, profile[1:])]
    assert max(steps) <= LIMITS['max_linear_accel_m_s2']*0.05 + 2e-3      # 0.30 m/s² at 20 Hz plus rounding
    assert abs(profile[-1] - 0.1) < 2e-3
    assert m.RELEASE_REQUEST not in writes
    stamps = [r['receive_monotonic_ns'] for r in env.records if r.get('event') == 'transaction_complete']
    assert stamps                                             # feedback kept running at 10 Hz throughout
    assert env.owner.status()['auto']['source'] == 'AUTO'


@pytest.mark.parametrize('change, reason', [
    (dict(linear=0.2), 'LINEAR_ABOVE_LIMIT'), (dict(linear=-0.05), 'NEGATIVE_LINEAR'),
    (dict(angular=0.5), 'ANGULAR_ABOVE_LIMIT'), (dict(state='LOCALIZATION_UNTRUSTED'), 'GATE_NONZERO_WHILE_NOT_ACTIVE')])
def test_violations_end_auto_with_a_controlled_stop(change, reason):
    env = driving(make())
    start = len(env.channel.requests)
    for key, value in change.items():
        setattr(env, key, value)
    run(env, 0.1)
    assert env.auto.reason == reason and env.auto.state == 'STOPPING'
    env.channel.words = (0, 0)
    run(env, 1.0)
    assert env.auto.state == 'NONE' and env.owner.state == 'READY'
    written = [chassis(w, env) for w in speeds(env, start)]
    assert all(v <= LIMITS['max_linear_m_s'] + 1e-9 and abs(w) <= LIMITS['max_angular_rad_s'] + 1e-3 for v, w in written)
    assert written[-1] == (0.0, 0.0) and env.auto.counters['violations'] >= 1


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
    assert profile[0] > 0 and profile[-1] == 0.0
    assert min(b - a for a, b in zip(profile, profile[1:])) >= -LIMITS['max_linear_decel_m_s2']*0.05 - 2e-3


def test_interface_loss_stops_through_the_manual_layer_without_nonzero_after():
    env = driving(make())
    env.ui = False
    run(env, 0.3)
    assert env.auto.reason == 'MANUAL_LAYER_STOP' or env.auto.reason.startswith('AUTO_EXITED')
    index = max(i for i, r in enumerate(env.channel.requests) if r[1] == 0x10 and r != m.ZERO_REQUEST)
    run(env, 1.0)
    assert all(r == m.ZERO_REQUEST or r[1] != 0x10 for r in env.channel.requests[index+1:])
    assert env.auto.state == 'NONE'


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


def test_backward_motion_stops_at_once():
    env = driving(make())
    env.channel.words = (40, (-40) & 0xffff)                 # left sign -1: both wheels report backward
    run(env, 0.5)
    assert env.auto.reason in ('MOVING_BACKWARD',) or env.owner.state == 'FAULT', env.auto.reason


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


def test_serial_failure_during_auto_latches_fault():
    env = driving(make())
    env.channel.failure = 'SYNTHETIC SERIAL FAILURE'
    run(env, 0.2)
    assert env.owner.state == 'FAULT' and env.auto.state == 'FAULT'


def test_status_is_json_and_never_claims_a_physical_stop():
    env = driving(make())
    status = env.owner.status()['auto']
    json.dumps(status, allow_nan=False)
    assert status['physical_stop_confirmed'] is False and status['dead_man'] is True and status['estop'] is False
    assert status['limits'] == LIMITS and status['gate_state'] == 'ACTIVE'


# ---------------------------------------------------------------- attach_auto and the socket
def evidence_project(tmp):
    root = tmp/'project'
    (root/'evidence').mkdir(parents=True, exist_ok=True)
    source = root/'evidence'/'stand_test.txt'
    source.write_text('synthetic stand test record\n')
    digest = __import__('hashlib').sha256(source.read_bytes()).hexdigest()
    test = {'verified': True, 'trials': 3, 'max_stop_time_s': 0.4,
            'sources': [{'kind': 'recording', 'path': 'evidence/stand_test.txt', 'sha256': digest}]}
    evidence = {'schema': 'wc_auto_drive_evidence_v1', 'state': 'REVIEWED', 'stage': 'STAND_TEST',
                'device_id': 'ZLAC8030D-0000000014', 'firmware_id': 'SYNTHETIC', 'reviewer': 'test',
                'reviewed_at': '2026-11-09', 'approved_limits': dict(LIMITS),
                'driver_readback': [{'purpose': 'communication_timeout', 'request_hex': TIMEOUT_REQUEST.hex(),
                                     'word_index': 0, 'mask': 65535, 'equals': 7}],
                'stop_tests': {name: test for name in ('estop', 'dead_man', 'owner_killed', 'usb_unplugged',
                                                       'orin_power_cut')}}
    (root/'evidence'/'auto.json').write_text(json.dumps(evidence))
    return root


def attached(tmp, *, controls=None, environ=None, switches=True, readback=True, socket_path=None):
    import datetime
    root = evidence_project(tmp)
    cfg = config()
    cfg['auto_controls'] = controls or {'allowed': True, 'evidence': 'evidence/auto.json', **LIMITS}
    clock, channel, records = Clock(), FakeChannel(), []
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


@pytest.mark.parametrize('case, reason', [
    ('not_allowed', 'AUTO_NOT_ALLOWED'), ('no_approval', 'NO_USER_APPROVAL_TODAY'),
    ('old_approval', 'NO_USER_APPROVAL_TODAY'), ('template', 'EVIDENCE_REJECTED'),
    ('no_switches', 'NO_SWITCH_INPUT'), ('no_readback', 'DRIVER_READBACK_NOT_IMPLEMENTED')])
def test_attach_auto_stays_unavailable_without_every_precondition(short_tmp, case, reason):
    controls = None
    environ = None
    if case == 'not_allowed':
        controls = {'allowed': False}
    elif case == 'no_approval':
        environ = {}
    elif case == 'old_approval':
        environ = {'WC_AUTO_DRIVE_APPROVAL': 'APPROVED_BY_USER_20261108'}
    elif case == 'template':
        controls = {'allowed': True, 'evidence': str(ROOT/'config/wheel_auto_drive_evidence.template.json'), **LIMITS}
    owner, sock, records = attached(short_tmp, controls=controls, environ=environ,
                                    switches=case != 'no_switches', readback=case != 'no_readback',
                                    socket_path=short_tmp/'auto.sock')
    assert sock is None and not (short_tmp/'auto.sock').exists()
    assert owner.auto.state == 'UNAVAILABLE' and owner.auto.reason.startswith(reason), owner.auto.reason
    assert any(r.get('event') == 'auto_attached' for r in records)


def test_attach_auto_rejects_malformed_controls(short_tmp):
    with pytest.raises(m.FeedbackError):
        attached(short_tmp, controls={'allowed': True, 'evidence': 'evidence/auto.json', 'max_linear_m_s': 0.5})
    with pytest.raises(m.FeedbackError):
        attached(short_tmp, controls={'allowed': 'yes'})


def test_socket_hello_decisions_and_status(short_tmp):
    import socket as s
    from wc_motion import auto_protocol as p
    owner, sock, records = attached(short_tmp, socket_path=short_tmp/'auto.sock')
    assert sock is not None and owner.auto.state == 'NONE'
    assert oct((short_tmp/'auto.sock').stat().st_mode & 0o777) == '0o600'
    client = s.socket(s.AF_UNIX, s.SOCK_STREAM); client.connect(str(short_tmp/'auto.sock'))
    second = s.socket(s.AF_UNIX, s.SOCK_STREAM); second.connect(str(short_tmp/'auto.sock'))
    client.sendall(p.encode_line(p.hello_packet(owner.session_id, 'bridge_test')))
    sock.pump(); sock.pump()
    assert owner.auto.bridge_connected and owner.auto.bridge_id == 'bridge_test'
    gate = {'seq': 1, 'stamp_ns': 1, 'mono_ns': 5, 'state': 'ACTIVE', 'reasons': [], 'linear': 0.05, 'angular': 0.0,
            'limits': {'max_linear': 0.1, 'max_angular': 0.3}, 'config_sha256': None}
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
    import socket as s
    owner, sock, _ = attached(short_tmp, socket_path=short_tmp/'auto.sock')
    client = s.socket(s.AF_UNIX, s.SOCK_STREAM); client.connect(str(short_tmp/'auto.sock'))
    sock.pump()
    owner.clock.advance(1.1)
    sock.pump()
    assert sock.client is None and not owner.auto.bridge_connected
    sock.close()


def test_main_attaches_auto_only_when_the_session_configuration_has_auto_controls(tmp_path, monkeypatch):
    """The real entrypoint with in-memory stand-ins: no serial port, no ROS."""
    from types import ModuleType
    from wc_motion import feedback_transport as ft
    for with_auto in (False, True):
        cli = ModuleType('wc_runtime.cli')
        cli.ROOT, cli.RUN, cli.target = tmp_path, tmp_path/'run', lambda: None
        monkeypatch.setitem(sys.modules, 'wc_runtime.cli', cli)
        hardware = tmp_path/'hardware.json'
        hardware.write_bytes((ROOT/'config/wheel_feedback_current.json').read_bytes())
        session = tmp_path/('session_%d' % with_auto); session.mkdir()
        cfg = {**config(), 'duration_s': 0, 'wheel_hardware_config': str(hardware), 'source_mode': 'real'}
        if with_auto:
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

        def fake_attach(config_value, owner, root, directory):
            attaches.append(config_value['auto_controls'])
            return NS(pump=lambda: calls.append('auto_pump'), close=lambda: calls.append('auto_close'))
        monkeypatch.setattr(ft, 'FeedbackSerialLease', Lease)
        monkeypatch.setattr(ft, 'create_journal', lambda *args, **kwargs: Journal())
        monkeypatch.setattr(m, 'RosOutput', lambda: NS(publish=lambda *_: None, manual=NS(publish=lambda _: None),
                                                       String=lambda **kwargs: NS(**kwargs), close=lambda: None))
        monkeypatch.setattr(m, 'WheelChannel', lambda *_: None)
        monkeypatch.setattr(m, 'MappingWheel', Owner)
        monkeypatch.setattr(m, 'ManualSocket', lambda *_: NS(pump=lambda: None, close=lambda: calls.append('socket_close')))
        monkeypatch.setattr(a, 'attach_auto', fake_attach)
        assert m.main(['--session-root', str(session)]) == 0
        if with_auto:
            assert attaches == [{'allowed': False}]
            assert calls.index('auto_pump') < calls.index('step')
            assert calls.index('auto_close') < calls.index('socket_close') < calls.index('owner_close')
        else:
            assert attaches == [] and 'auto_pump' not in calls and 'auto_close' not in calls
