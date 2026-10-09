"""Synthetic wheel broker/protocol tests. No real serial device or motion."""
import copy
import hashlib
import io
import json
from pathlib import Path
import shutil
import socket
import struct
import sys
import threading
import tempfile
import time
import uuid
from types import SimpleNamespace as NS

import pytest

from wc_runtime import mapping_wheel as m


ROOT = Path(__file__).resolve().parents[2]


@pytest.fixture
def tmp_path():
    parent = Path(__file__).absolute().parent
    path = parent/('.mapping_wheel_test_'+uuid.uuid4().hex)
    path.mkdir()
    try:
        yield path
    finally:
        resolved = path.resolve()
        assert resolved.parent == parent and resolved.name.startswith('.mapping_wheel_test_')
        shutil.rmtree(resolved)


@pytest.fixture
def socket_tmp_path():
    # Linux sun_path is bounded independently of the checkout location.
    with tempfile.TemporaryDirectory(prefix="wc-ms-", dir="/tmp") as folder:
        yield Path(folder)


class Clock:
    def __init__(self): self.value = 10.
    def __call__(self): return self.value
    def advance(self, seconds): self.value += seconds
    def ns(self): return round(self.value*1e9)
    def wall(self): return 1_700_000_000_000_000_000+self.ns()


def response(words=(0, 0)):
    payload = b'\x01\x03\x04'+struct.pack('>HH', *words)
    return payload+m.crc16(payload)


def config(allowed=False):
    wheel = json.loads((ROOT/'config/wheel_history_calibration.json').read_text())
    return {'session_id': 'synthetic_mapping_wheel', 'wheel_device_id': 'ZLAC8030D-0000000014',
            'wheel_candidate': wheel, 'manual_controls': {'arm_allowed': allowed,
                'operation_evidence': {'status': 'USER_REPORTED_LEGACY_OPERATION', 'note': 'SYNTHETIC TEST ONLY'}}}


class FakeChannel:
    def __init__(self):
        self.control_transmissions = self.control_attempts = self.control_bytes = 0
        self.motion_may_exist = self.failed = False
        self.recovery_attempted = False
        self.last_stop = None
        self.requests = []
        self.words = (0, 0)
        self.before_guard = lambda: None
        self.failure = None

    def exchange(self, request, timeout, *, guard=None):
        self.before_guard()
        if guard is not None: guard()
        if self.failure:
            self.failed = True
            raise m.QueryFailure(self.failure)
        self.requests.append(request)
        if request == m.QUERY: return response(self.words)
        self.control_transmissions += 1; self.control_attempts += 1; self.control_bytes += len(request)
        if request == m.INITIALIZATION_WRITES[-1] or request[1] == 16 and request != m.ZERO_REQUEST:
            self.motion_may_exist = True
        return request if request[1] == 6 else request[:6]+m.crc16(request[:6])

    def best_effort_zero(self):
        if self.motion_may_exist and not self.recovery_attempted:
            self.recovery_attempted = True
            self.requests.append(m.ZERO_REQUEST)
            self.control_transmissions += 1; self.control_attempts += 1; self.control_bytes += len(m.ZERO_REQUEST)
            self.last_stop = {'acknowledged': False, 'stationary_feedback_observed': False, 'physical_stop_confirmed': False}
        return self.last_stop


def owner(allowed=False, journal=None):
    clock, channel, records, published = Clock(), FakeChannel(), [], []
    value = m.MappingWheel(config(allowed), channel, journal or records.append,
                          clock=clock, wall_ns=clock.wall, mono_ns=clock.ns,
                          publish=lambda source, preview: published.append((source, preview)))
    return value, clock, channel, records, published


def ticks(value, clock, count=31, delta=.01):
    for _ in range(count):
        value.step(); clock.advance(delta)


def armed():
    value, clock, channel, records, published = owner(True)
    value.connected()
    hold(value, clock, ['w'], 70)
    assert value.state == 'ARMED' and value.reason == 'USER_KEYBOARD_INTENT'
    return value, clock, channel, records, published


def keys(value, pressed, generation=None, foreground=True):
    value.command({'type': 'keys', 'session_id': value.session_id, 'sequence': value.last_key_sequence+1,
                   'arm_generation': value.arm_generation if generation is None else generation,
                   'keys': pressed, 'foreground':foreground})


def hold(value, clock, pressed, count=70):
    for _ in range(count):
        keys(value, pressed)
        ticks(value, clock, 1)


def test_default_lifetime_reads_continuously_and_never_writes_zero_on_exit():
    value, clock, channel, records, published = owner()
    ticks(value, clock, 230)
    value.connected()
    value.command({'type': 'arm', 'session_id': value.session_id, 'confirmed': True})
    assert value.reason == m.DISABLED_REASON and value.state == 'DISARMED'
    keys(value, ['w']); ticks(value, clock, 20)
    value.disconnected(); value.close()
    assert 20 <= len(published) <= 26
    assert channel.requests and set(channel.requests) == {m.QUERY}
    assert channel.control_transmissions == 0 and channel.last_stop is None
    assert all(source['control_transmissions'] == 0 for source, _ in published)


def test_feedback_exact_bytes_identity_hash_and_wheel_candidate_match():
    value, clock, channel, records, published = owner()
    channel.words = (65526, 10)
    source = value.sample()
    assert source['response_hex'] == response(channel.words).hex()
    assert source['response_sha256'] == hashlib.sha256(response(channel.words)).hexdigest()
    assert source['register_words_u16'] == list(channel.words)
    assert source['session_id'] == value.session_id and source['control_context'] == 'user_manual_mapping'
    assert source['receive_monotonic_ns'] == clock.ns() and source['stamp_ns'] == clock.wall()
    assert source['time_valid'] is False and source['formal_odometry_eligible'] is False
    assert published[0][1]['left_wheel_rpm_candidate'] == pytest.approx(1.12)
    assert published[0][1]['right_wheel_rpm_candidate'] == pytest.approx(1.12)


@pytest.mark.parametrize('continuous', [False, True])
def test_mapping_feedback_pause_has_explicit_continuous_policy_and_no_dead_reckoning(continuous):
    clock, channel, records = Clock(), FakeChannel(), []
    settings = {**config(True), 'continuous_mapping': continuous}
    value = m.MappingWheel(settings, channel, records.append, clock=clock,
                           wall_ns=clock.wall, mono_ns=clock.ns)
    channel.words = (65526, 10)
    value.step(); clock.advance(.1); value.step()
    distance = value.preview.distance
    clock.advance(6.); value.step()
    if not continuous:
        assert value.state == 'FAULT' and 'gap' in value.failure
        return
    assert value.failure is None and value.state == 'READY'
    assert value.preview.distance == distance
    assert value.status()['feedback_gap_count'] == 1
    assert value.status()['feedback_uncovered_duration_s'] == pytest.approx(6.)
    gap = [r for r in records if r.get('event') == 'history_preview'][-1]
    assert gap['integration_gap'] and gap['integration_interval_s'] is None
    clock.advance(.1); value.step()
    assert value.preview.distance > distance and value.failure is None
    assert not any(r != m.QUERY for r in channel.requests)


def test_continuous_mapping_gap_cancels_old_driving_intent_without_failing_mapping():
    clock, channel, records = Clock(), FakeChannel(), []
    value = m.MappingWheel({**config(True), 'continuous_mapping': True}, channel, records.append,
                           clock=clock, wall_ns=clock.wall, mono_ns=clock.ns)
    value.connected(); hold(value, clock, ['w'])
    before = len(channel.requests)
    clock.advance(4.); value.step()
    assert value.state == 'READY' and value.failure is None and not value.keys
    assert m.ZERO_REQUEST in channel.requests[before:]
    assert not any(r[1] == 16 and r != m.ZERO_REQUEST for r in channel.requests[before:])
    assert value.status()['feedback_gap_count'] == 1
    hold(value, clock, ['s'])
    assert value.state == 'ARMED' and value.failure is None
    channel.failure = 'synthetic CRC or ACK failure'
    clock.advance(.1); value.step()
    assert value.state == 'FAULT' and 'synthetic CRC or ACK failure' in value.failure


def test_idle_ready_never_enables_and_first_held_key_initializes_with_moving_feedback():
    value, clock, channel, records, published = owner(True)
    value.connected()
    channel.words = (65526, 10)  # The operator may already be hand pushing.
    ticks(value, clock, 150)
    keys(value, []); value.command({'type': 'arm', 'session_id': value.session_id})
    assert value.state == 'READY' and channel.control_transmissions == 0
    old_generation = value.arm_generation
    request_times = []
    exchange = channel.exchange
    def timed_exchange(request, timeout, **kwargs):
        result = exchange(request, timeout, **kwargs)
        request_times.append((request, clock()))
        return result
    channel.exchange = timed_exchange
    hold(value, clock, ['w'])
    controls = [r for r in channel.requests if r != m.QUERY]
    assert controls[:5] == [*m.INITIALIZATION_WRITES[:3], m.ZERO_REQUEST, m.INITIALIZATION_WRITES[3]]
    assert controls[5:] and all(r == m.velocity_request(value.wheel, value.control, .1, 0.) for r in controls[5:])
    assert value.state == 'ARMED' and value.arm_generation == old_generation and value.keys == {'w'}
    clear_at = next(at for request, at in request_times if request == m.INITIALIZATION_WRITES[2])
    enable_at = next(at for request, at in request_times if request == m.INITIALIZATION_WRITES[3])
    moving_at = next(at for request, at in request_times if request[1] == 16 and request != m.ZERO_REQUEST)
    assert enable_at-clear_at >= .2 and moving_at-enable_at >= .2
    assert len(published) >= 8
    assert max(b[0]['receive_monotonic_ns']-a[0]['receive_monotonic_ns'] for a,b in zip(published,published[1:])) < 150_000_000


def test_hand_push_requires_explicit_request_stationary_feedback_and_explicit_exit():
    value, clock, channel, _, _ = armed()
    hold(value, clock, ['w', 'a'], 40)
    nonzero = [r for r in channel.requests if r[1] == 16 and r != m.ZERO_REQUEST]
    assert nonzero
    assert nonzero[-1] == m.velocity_request(value.wheel, value.control, .1, .2)
    keys(value, []); value.step()
    stopped_at, deadline = clock(), value.stop_deadline
    assert m.ZERO_REQUEST in channel.requests[-2:] and value.state == 'READY'
    channel.words = (65526, 10)
    zero_count = channel.requests.count(m.ZERO_REQUEST)
    generation = value.arm_generation
    for _ in range(70):
        keys(value, []); ticks(value, clock, 1)
        assert value.stop_deadline == deadline and value.arm_generation == generation
    assert m.RELEASE_REQUEST not in channel.requests
    ticks(value, clock, 40)
    assert channel.requests.count(m.ZERO_REQUEST) == zero_count
    assert channel.requests.count(m.RELEASE_REQUEST) == 0
    value.command({'type':'push_mode', 'session_id':value.session_id, 'enabled':True,
                   'sequence':value.last_key_sequence+1, 'arm_generation':value.arm_generation})
    ticks(value, clock, 100)
    assert value.push_mode_requested and not value.push_mode and m.RELEASE_REQUEST not in channel.requests
    channel.words = (0, 0); ticks(value, clock, 40)
    assert channel.requests.count(m.RELEASE_REQUEST) == 1 and value.push_mode
    assert value.failure is None and value.state == 'READY' and not value.initialized_by_user
    assert channel.last_stop['servo_released'] is True
    assert channel.last_stop['stationary_feedback_observed'] is True
    assert channel.last_stop['physical_stop_confirmed'] is False
    initializations = channel.requests.count(m.INITIALIZATION_WRITES[0])
    keys(value, ['s']); ticks(value, clock, 40)
    assert not value.keys and channel.requests.count(m.INITIALIZATION_WRITES[0]) == initializations
    value.command({'type':'push_mode', 'session_id':value.session_id, 'enabled':False,
                   'sequence':value.last_key_sequence+1, 'arm_generation':value.arm_generation})
    assert not value.push_mode and not value.keys
    hold(value, clock, ['s'])
    assert value.state == 'ARMED' and channel.requests.count(m.INITIALIZATION_WRITES[0]) == initializations+1
    assert m.velocity_request(value.wheel, value.control, -.1, 0.) in channel.requests
    assert m.single_write(0x200E, 5) not in channel.requests


def test_readonly_and_stale_generation_push_packets_never_release():
    for allowed in (False, True):
        value, clock, channel, _, _ = owner(allowed); value.connected(); ticks(value, clock, 30)
        value.command({'type':'push_mode','session_id':value.session_id,'enabled':True,
            'sequence':1,'arm_generation':value.arm_generation+(1 if allowed else 0)})
        ticks(value, clock, 100); value.close()
        assert channel.control_transmissions==0 and set(channel.requests)=={m.QUERY}


def test_pending_push_revocation_in_prewrite_guard_never_releases():
    value, clock, channel, _, _ = owner(True); value.connected(); ticks(value, clock, 30)
    value.command({'type':'push_mode','session_id':value.session_id,'enabled':True,
        'sequence':1,'arm_generation':value.arm_generation})
    while value.stop_zero_samples < 2: ticks(value, clock, 1)
    value.before_control = value.disconnected
    ticks(value, clock, 30)
    assert not value.push_mode_requested and not value.push_mode and m.RELEASE_REQUEST not in channel.requests


def test_close_cancels_pending_hand_push_without_release():
    value, clock, channel, _, _ = armed()
    value.command({'type':'push_mode','session_id':value.session_id,'enabled':True,
        'sequence':value.last_key_sequence+1,'arm_generation':value.arm_generation})
    value.step(); value.close()
    assert value.closed and not value.push_mode_requested and m.RELEASE_REQUEST not in channel.requests


@pytest.mark.parametrize('cause', ['timeout', 'disconnect', 'signal', 'release', 'space'])
def test_revocation_stops_without_another_nonzero_command(cause):
    value, clock, channel, _, _ = armed()
    keys(value, ['w']); value.step()
    count = len([r for r in channel.requests if r[1] == 16 and r != m.ZERO_REQUEST])
    generation = value.arm_generation
    if cause == 'timeout': clock.advance(.251)
    elif cause == 'disconnect': value.disconnected()
    elif cause == 'signal': value.cancelled = lambda: True
    elif cause == 'space': value.command({'type': 'disarm', 'session_id': value.session_id})
    else: keys(value, [])
    value.step()
    assert m.ZERO_REQUEST in channel.requests[-2:]
    assert len([r for r in channel.requests if r[1] == 16 and r != m.ZERO_REQUEST]) == count
    assert value.state == 'READY' and not value.keys and value.arm_generation > generation
    # Old queued packets cannot resurrect the cancelled intent.
    keys(value, ['w'], generation); value.step()
    assert not value.keys
    if cause != 'signal':
        if cause == 'disconnect': value.connected()
        hold(value, clock, ['s'])
        assert value.state == 'ARMED' and value.target() == (-.1, 0.)


def test_opposing_held_keys_stop_and_releasing_one_resumes_without_losing_intent():
    value, clock, channel, _, _ = armed()
    generation = value.arm_generation
    keys(value, ['w', 's']); value.step()
    assert m.ZERO_REQUEST in channel.requests[-2:]
    hold(value, clock, ['w', 's'], 110)
    assert value.state == 'READY' and value.keys == {'w', 's'} and value.arm_generation == generation
    assert channel.requests.count(m.RELEASE_REQUEST) == 0
    hold(value, clock, ['w'])
    assert value.state == 'ARMED' and value.keys == {'w'} and value.arm_generation == generation


@pytest.mark.parametrize('after_enable', [False, True])
@pytest.mark.parametrize('cause', ['release', 'timeout', 'disconnect'])
def test_initialization_cancellation_never_runs_retained_keys_and_fresh_press_recovers(after_enable, cause):
    value, clock, channel, _, _ = owner(True)
    value.connected()
    hold(value, clock, ['w'], 26 if after_enable else 3)
    assert value.state == 'INITIALIZING'
    assert (m.INITIALIZATION_WRITES[-1] in channel.requests) == after_enable
    generation = value.arm_generation
    if cause == 'release': keys(value, [])
    elif cause == 'timeout': clock.advance(.251)
    else: value.disconnected()
    value.step(); ticks(value, clock, 100)
    assert value.state == 'READY' and not value.keys and value.arm_generation > generation
    assert not any(r[1] == 16 and r != m.ZERO_REQUEST for r in channel.requests)
    assert m.RELEASE_REQUEST not in channel.requests
    if cause == 'disconnect': value.connected()
    keys(value, ['w'], generation); ticks(value, clock, 1)
    assert not value.keys
    hold(value, clock, ['s'])
    assert value.state == 'ARMED' and value.target() == (-.1, 0.)


def test_repress_during_zero_delay_resumes_without_waiting_for_servo_release():
    value, clock, channel, _, _ = armed()
    keys(value, []); value.step(); ticks(value, clock, 10)
    before = len(channel.requests)
    keys(value, ['s']); value.step()
    assert m.velocity_request(value.wheel, value.control, -.1, 0.) in channel.requests[before:]
    assert m.RELEASE_REQUEST not in channel.requests and value.stop_deadline is None


def test_idle_does_not_invoke_release_guard_and_new_press_resumes():
    value, clock, channel, _, _ = armed()
    keys(value, []); value.step(); ticks(value, clock, 74); clock.advance(.02)
    before = len(channel.requests)
    keys(value, ['s'])
    value.step()
    assert m.RELEASE_REQUEST not in channel.requests
    assert value.state == 'ARMED' and value.keys == {'s'} and value.stop_deadline is None
    assert m.velocity_request(value.wheel, value.control, -.1, 0.) in channel.requests[before:]


def test_close_after_movement_sends_zero_and_never_releases_holding(monkeypatch):
    value, clock, channel, _, _ = armed()
    channel.words = (65526, 10)
    def queued_press():
        pytest.fail('closing must not service a closed socket or restore queued motion')
    value.before_control = queued_press
    monkeypatch.setattr(m.time, 'sleep', clock.advance)
    value.close()
    assert value.closed and value.failure is None
    assert channel.requests[-1] == m.ZERO_REQUEST
    assert m.RELEASE_REQUEST not in channel.requests
    assert channel.last_stop.get('servo_released', False) is False
    assert channel.last_stop['stationary_feedback_observed'] is False
    keys(value, ['w'])
    assert not value.keys


def test_initialization_can_be_cancelled_before_enable_without_default_stop_write():
    value, clock, channel, _, _ = owner(True)
    value.connected(); keys(value, ['w'])
    value.step()
    assert channel.control_transmissions == 1
    value.disconnected(); ticks(value, clock, 40); value.close()
    assert [r for r in channel.requests if r != m.QUERY] == [m.INITIALIZATION_WRITES[0]]


def test_serial_wait_cannot_dispatch_expired_keyboard_intent():
    value, clock, channel, _, _ = armed()
    keys(value, ['w'])
    before = channel.control_transmissions
    channel.before_guard = lambda: clock.advance(.26)
    value.step()
    assert channel.control_transmissions == before
    assert value.state == 'READY' and not value.keys


def test_ui_disarm_during_prewrite_guard_prevents_stale_velocity():
    value, clock, channel, _, _ = armed()
    keys(value, ['w'])
    value.before_control = lambda: value.command({'type':'disarm','session_id':value.session_id})
    before = channel.control_transmissions
    value.step()
    assert channel.control_transmissions == before and value.state == 'READY'


@pytest.mark.parametrize('phase', ['initializing', 'armed'])
@pytest.mark.parametrize('cause', ['release', 'space'])
@pytest.mark.parametrize('new_key_in_pump', [False, True])
def test_prewrite_cancel_publishes_one_generation_and_preserves_acknowledged_new_key(phase, cause, new_key_in_pump):
    if phase == 'armed':
        value, clock, channel, _, _ = armed()
        clock.advance(.06)
    else:
        value, clock, channel, _, _ = owner(True)
        value.connected(); hold(value, clock, ['w'], 2)
        assert value.state == 'INITIALIZING'
    keys(value, ['w'])
    generation, before = value.arm_generation, channel.control_transmissions
    published_status = []
    def pump_cancel_and_status():
        value.before_control = lambda: None
        if cause == 'release': keys(value, [])
        else: value.command({'type': 'disarm', 'session_id': value.session_id})
        published_status.append(value.status())  # This ACK can already be visible to Qt.
        if new_key_in_pump:
            keys(value, ['s'], published_status[-1]['arm_generation'])
    value.before_control = pump_cancel_and_status
    value.step()
    assert channel.control_transmissions == before
    assert published_status[0]['arm_generation'] == generation+1 == value.arm_generation
    assert value.state == 'READY' and value.failure is None
    assert value.stop_requested == (phase == 'armed')
    if not new_key_in_pump:
        keys(value, ['s'], published_status[0]['arm_generation'])
    assert value.keys == {'s'}  # The ACK generation accepts the first fresh press.
    requests_before_resume = len(channel.requests)
    hold(value, clock, ['s'])
    assert value.state == 'ARMED' and value.arm_generation == generation+1
    assert m.velocity_request(value.wheel, value.control, -.1, 0.) in channel.requests[requests_before_resume:]
    if phase == 'armed':
        controls = [r for r in channel.requests[requests_before_resume:] if r != m.QUERY]
        assert controls[0] == m.ZERO_REQUEST  # Pending cancellation is serviced before resume.


@pytest.mark.parametrize('replacement', [['w', 'a'], ['s'], ['a']])
def test_fresh_direction_change_during_prewrite_guard_retains_keys_and_generation(replacement):
    value, clock, channel, _, _ = armed()
    clock.advance(.06)
    keys(value, ['w'])
    generation, before = value.arm_generation, channel.control_transmissions
    def queued_change():
        value.before_control = lambda: None
        keys(value, replacement)
    value.before_control = queued_change
    value.step()
    assert channel.control_transmissions == before  # The pending W-only packet was cancelled.
    assert value.state == 'ARMED' and value.keys == set(replacement)
    assert value.arm_generation == generation and value.failure is None and not channel.failed
    value.step()  # No keyup, new keydown or activation is needed.
    assert channel.control_transmissions == before+1
    assert channel.requests[-1] == m.velocity_request(value.wheel, value.control, *value.target())
    assert value.arm_generation == generation and value.keys == set(replacement)


@pytest.mark.parametrize('phase', ['initializing', 'armed'])
@pytest.mark.parametrize('opposing', [['w', 's'], ['a', 'd']])
def test_opposing_keys_in_prewrite_guard_preserve_held_keys_until_one_is_released(phase, opposing):
    if phase == 'armed':
        value, clock, channel, _, _ = armed()
        clock.advance(.06)
    else:
        value, clock, channel, _, _ = owner(True)
        value.connected(); hold(value, clock, ['w'], 26)
        assert value.state == 'INITIALIZING' and channel.motion_may_exist
        clock.advance(.2)  # Reach the final initialization guard after enable settles.
    keys(value, ['w'])
    generation, before = value.arm_generation, channel.control_transmissions
    def queued_opposite():
        value.before_control = lambda: None
        keys(value, opposing)
    value.before_control = queued_opposite
    value.step()
    assert channel.control_transmissions == before
    assert value.state == 'READY' and value.keys == set(opposing)
    assert value.arm_generation == generation and value.stop_requested
    assert value.failure is None and not channel.failed
    value.step()
    assert channel.control_transmissions == before+1 and channel.requests[-1] == m.ZERO_REQUEST
    assert value.keys == set(opposing) and value.arm_generation == generation
    # Idle opposing keys preserve holding; releasing one resumes the held direction.
    hold(value, clock, opposing, 90)
    assert channel.requests.count(m.RELEASE_REQUEST) == 0 and value.keys == set(opposing)
    hold(value, clock, [opposing[0]])
    assert value.state == 'ARMED' and value.keys == {opposing[0]}
    assert value.arm_generation == generation


@pytest.mark.parametrize('invalidated', ['timeout', 'disconnect'])
def test_opposing_input_in_prewrite_guard_does_not_bypass_expiry_or_disconnect(invalidated):
    value, clock, channel, _, _ = armed()
    clock.advance(.06); keys(value, ['w'])
    before = channel.control_transmissions
    def invalid_opposite():
        value.before_control = lambda: None
        keys(value, ['w', 's'])
        if invalidated == 'timeout': clock.advance(.251)
        else: value.disconnected()
    value.before_control = invalid_opposite
    value.step()
    assert channel.control_transmissions == before and value.state == 'READY'
    assert not value.keys and value.stop_requested


def test_changed_target_that_expires_in_prewrite_guard_is_still_cancelled():
    value, clock, channel, _, _ = armed()
    clock.advance(.06)
    keys(value, ['w'])
    generation, before = value.arm_generation, channel.control_transmissions
    def changed_but_expired():
        value.before_control = lambda: None
        keys(value, ['w', 'a'])
        clock.advance(.251)
    value.before_control = changed_but_expired
    value.step()
    assert channel.control_transmissions == before and not channel.failed
    assert value.state == 'READY' and not value.keys
    assert value.arm_generation == generation+1 and value.stop_requested


def test_expired_key_refresh_does_not_restart_old_motion():
    value, clock, channel, _, _ = armed()
    keys(value, ['w']); value.step(); clock.advance(.251)
    generation = value.arm_generation
    keys(value, ['w'])
    assert value.state == 'READY' and not value.keys and value.arm_generation > generation
    keys(value, ['w'], generation)
    assert not value.keys
    hold(value, clock, ['s'])
    assert value.state == 'ARMED' and value.target() == (-.1, 0.)


def test_continuous_fresh_user_keys_keep_motion_enabled_beyond_two_seconds():
    value, clock, channel, _, _ = armed()
    keys(value, ['w']); value.step()
    started = clock()
    for seq in range(2, 85):
        keys(value, ['w'] if seq % 2 else ['w', 'a'])
        ticks(value, clock, 4)
    assert clock()-started > 3.0
    assert value.state == 'ARMED' and value.keys
    assert value.status()['max_continuous_motion_s'] is None
    nonzero = [r for r in channel.requests if r[1] == 16 and r != m.ZERO_REQUEST]
    assert len(nonzero) > 30
    assert nonzero[-1] == m.velocity_request(value.wheel, value.control, .1, .2)
    # Removing the duration cap does not remove a real missing-key deadline.
    clock.advance(.251); value.step()
    assert value.state == 'READY' and not value.keys and value.reason == 'KEY_TIMEOUT'
    ticks(value, clock, 40)
    assert channel.last_stop['stationary_feedback_observed'] is True


def test_released_keys_stop_and_new_press_resumes_without_rearming():
    value, clock, channel, _, _ = armed()
    keys(value, ['w']); value.step()
    keys(value, []); ticks(value, clock, 40)
    generation = value.arm_generation
    assert value.state == 'READY' and not value.keys
    assert channel.last_stop['stationary_feedback_observed'] is True
    ticks(value, clock, 300)  # Healthy idle feedback alone does not disarm.
    before = len([r for r in channel.requests if r[1] == 16 and r != m.ZERO_REQUEST])
    for seq in range(3, 85):
        keys(value, ['s']); ticks(value, clock, 4)
    assert value.state == 'ARMED' and value.arm_generation == generation
    nonzero = [r for r in channel.requests if r[1] == 16 and r != m.ZERO_REQUEST]
    assert len(nonzero) > before+30
    assert nonzero[-1] == m.velocity_request(value.wheel, value.control, -.1, 0.)
    value.disconnected(); value.step()
    assert value.state == 'READY' and not value.keys
    assert m.ZERO_REQUEST in channel.requests[-2:]


@pytest.mark.parametrize('change', [{'session_id':'other'}, {'sequence':True}, {'sequence':0},
                                  {'keys':['w','w']}, {'keys':['x']}, {'keys':'w'}])
def test_key_identity_sequence_and_input_are_explicit(change):
    value, *_ = armed()
    message = {'type':'keys','session_id':value.session_id,'sequence':value.last_key_sequence+1,'arm_generation':value.arm_generation,'keys':['w']}
    message.update(change)
    with pytest.raises(m.FeedbackError): value.command(message)


def test_read_failure_while_disarmed_never_sends_stop_command():
    value, clock, channel, _, _ = owner()
    channel.failure = 'synthetic read timeout'
    value.step(); value.close()
    assert value.state == 'FAULT' and channel.control_transmissions == 0


def test_failure_after_movement_latches_fault_and_attempts_only_one_zero():
    value, clock, channel, _, _ = armed()
    keys(value, ['w']); value.step()
    channel.failure = 'synthetic dropped ACK'; clock.advance(.11)
    value.step(); value.step(); value.close()
    assert value.state == 'FAULT' and channel.recovery_attempted
    assert channel.requests[-1] == m.ZERO_REQUEST
    assert channel.last_stop['acknowledged'] is False
    before = len(channel.requests)
    value.command({'type':'arm','session_id':value.session_id,'confirmed':True})
    keys(value, ['w']); value.step()
    assert value.state == 'FAULT' and len(channel.requests) == before


@pytest.mark.parametrize('frame', [m.single_write(0x200E, 5), m.single_write(0x200E, 9), m.single_write(0x200D, 4)])
def test_io_allowlist_rejects_unreviewed_control_words_before_any_fd_access(frame):
    channel = m.WheelChannel(NS(fd=999, failed=False), lambda row: None)
    with pytest.raises(m.FeedbackError, match='allowlist'):
        channel.exchange(frame, .08, guard=lambda: None)


@pytest.mark.parametrize('kind', ['fc06','fc10','release'])
def test_ack_must_echo_exact_requested_register_and_function(kind):
    request = {'fc06': m.INITIALIZATION_WRITES[0], 'fc10': m.speed_write(-5,5), 'release': m.RELEASE_REQUEST}[kind]
    good = request if kind != 'fc10' else request[:6]+m.crc16(request[:6])
    assert m.validate_response(request, good) == good
    wrong = bytearray(good[:-2]); wrong[3] ^= 1
    with pytest.raises(m.FeedbackError): m.validate_response(request, wrong+m.crc16(wrong))


def test_partial_write_counts_real_bytes_and_never_retries_nonzero(monkeypatch):
    records, writes = [], []
    channel = m.WheelChannel(NS(fd=999, failed=False), records.append)
    monkeypatch.setattr(m.select, 'select', lambda readers,writers,errors,timeout: ([],writers,[]))
    def write(fd, payload): writes.append(payload); return 4 if len(writes) == 1 else len(payload)
    monkeypatch.setattr(m.os, 'write', write)
    with pytest.raises(m.QueryFailure) as found:
        channel.exchange(m.speed_write(-5,5), .08, guard=lambda: None)
    assert found.value.transmitted_bytes == 4 and channel.control_transmissions == 1 and channel.control_bytes == 4
    channel.best_effort_zero(); channel.best_effort_zero()
    assert writes == [m.speed_write(-5,5), m.ZERO_REQUEST]
    assert channel.control_transmissions == 2 and channel.control_bytes == 4+len(m.ZERO_REQUEST)
    assert channel.last_stop['acknowledged'] is False
    with pytest.raises(m.QueryFailure): channel.exchange(m.speed_write(-5,5), .08, guard=lambda: None)


def test_no_control_write_without_guard(monkeypatch):
    channel = m.WheelChannel(NS(fd=999, failed=False), lambda row: None)
    monkeypatch.setattr(m.os,'write',lambda *args: pytest.fail('unguarded actuator write'))
    with pytest.raises(m.FeedbackError, match='guard'):
        channel.exchange(m.ZERO_REQUEST,.08)


def test_slow_async_journal_does_not_block_key_expiry_or_zero(monkeypatch):
    from wc_motion.feedback_transport import AsyncJournal
    entered, release = threading.Event(), threading.Event()
    class Sink(io.StringIO):
        def write(self, value):
            entered.set(); assert release.wait(3)
            return super().write(value)
        def fileno(self): return 777
    monkeypatch.setattr(m.os,'fsync',lambda fd: None)
    journal = AsyncJournal(Sink())
    value, clock, channel, _, _ = owner(True,journal)
    try:
        ticks(value,clock); assert entered.wait(.5)
        value.connected(); hold(value, clock, ['w'])
        clock.advance(.251)
        started=time.monotonic(); value.step()
        assert time.monotonic()-started < .05
        assert channel.requests[-1] == m.ZERO_REQUEST or channel.requests[-2] == m.ZERO_REQUEST
        assert value.state == 'READY'
    finally:
        release.set(); journal.close()


def test_blocked_writer_accepts_eighty_real_mapping_samples_and_all_five_events(monkeypatch):
    from wc_motion.feedback_transport import AsyncJournal
    entered, release = threading.Event(), threading.Event()
    class Sink(io.StringIO):
        def write(self, text):
            entered.set(); assert release.wait(3)
            return super().write(text)
        def fileno(self): return 777
        def close(self):
            self.saved = self.getvalue()
            super().close()
    sink=Sink()
    monkeypatch.setattr(m.os,'fsync',lambda fd: None)
    journal=AsyncJournal(sink)
    value, clock, channel, _, published=owner(False,journal)
    original=channel.exchange
    def logged_exchange(request, timeout, **kwargs):
        journal({'event':'wheel_io_planned', 'request_hex':request.hex(), 'control':False})
        result=original(request,timeout,**kwargs)
        journal({'event':'wheel_io_complete', 'request_hex':request.hex(), 'response_hex':result.hex()})
        return result
    channel.exchange=logged_exchange
    try:
        value.sample(); clock.advance(.1); assert entered.wait(1)
        for _ in range(79):
            value.sample(); clock.advance(.1)
        state=journal.status()
        assert state['pending_events']==state['enqueued_events']==400 and state['written_events']==0
        assert state['pending_bytes']<state['max_pending_bytes']==4*1024*1024
        assert state['pending_events']>256 and state['max_pending_events']==1024
        assert len(published)==80 and len(channel.requests)==80 and set(channel.requests)=={m.QUERY}
        assert channel.control_transmissions==0 and value.failure is None and state['error'] is None
    finally:
        release.set(); journal.close()
    rows=[json.loads(line) for line in sink.saved.splitlines()]
    assert [row['journal_event_index'] for row in rows[:-1]]==list(range(1,401))
    assert [row['event'] for row in rows[:-1]]==[
        'request_planned','wheel_io_planned','wheel_io_complete','transaction_complete','history_preview']*80
    raw=[row for row in rows if row['event']=='transaction_complete']
    assert [row['stamp_ns'] for row in raw]==[source['stamp_ns'] for source,_ in published]
    assert [row['receive_monotonic_ns'] for row in raw]==[source['receive_monotonic_ns'] for source,_ in published]
    assert all(row['response_hex']==response().hex() and row['request_hex']==m.QUERY.hex() for row in raw)
    state=journal.status()
    assert state['pending_events']==state['pending_bytes']==0
    assert state['written_events']==400 and state['final_fsync_complete']
    assert state['completed_event_batches']<10 and state['max_event_batch_events']==64


@pytest.mark.parametrize('first_failure',[None,'synthetic original session failure'])
def test_main_drains_full_journal_even_when_terminal_event_is_rejected(tmp_path,monkeypatch,capsys,first_failure):
    from wc_motion import feedback_transport as ft
    from types import ModuleType
    cli=ModuleType('wc_runtime.cli')
    cli.ROOT, cli.RUN, cli.target=tmp_path, tmp_path/'run', lambda: None
    monkeypatch.setitem(sys.modules,'wc_runtime.cli',cli)
    calls=[]
    entered, release=threading.Event(),threading.Event()
    class Sink(io.StringIO):
        def write(self,text):
            entered.set(); assert release.wait(3)
            return super().write(text)
        def fileno(self): return 777
    class Lease:
        host_configuration={'fixture':True}
        def __init__(self,*_): pass
        def open(self):
            calls.append('lease_open')
            if first_failure: raise m.FeedbackError(first_failure)
        def close(self): calls.append('lease_close')
    monkeypatch.setattr(m.os,'fsync',lambda fd: calls.append('final_fsync'))
    monkeypatch.setattr(ft,'FeedbackSerialLease',Lease)
    monkeypatch.setattr(m.signal,'signal',lambda *_: None)
    monkeypatch.setattr(m.signal,'SIGHUP',1,raising=False)
    hardware=tmp_path/'hardware.json'
    hardware.write_bytes((ROOT/'config/wheel_feedback_current.json').read_bytes())
    session=tmp_path/'session'; session.mkdir()
    (session/'runtime_config.json').write_text(json.dumps({**config(), 'duration_s':60,
                                                        'wheel_hardware_config':str(hardware)}))
    sink=Sink(); journal=ft.AsyncJournal(sink,max_events=1)
    journal({'event':'already_accepted'}); assert entered.wait(1)
    original_close=journal.close
    def close():
        calls.append('journal_close'); release.set(); original_close()
    journal.close=close
    monkeypatch.setattr(ft,'create_journal',lambda *args,**kwargs: journal)
    try:
        result=m.main(['--session-root',str(session)])
    finally:
        release.set(); journal.thread.join(2)
    final=json.loads((session/'wheel_status.json').read_text())
    assert result==1 and final['state']=='FAILED'
    assert final['failure']==(first_failure or 'ASYNC_JOURNAL_QUEUE_FULL')
    assert final['journal']['error']=='ASYNC_JOURNAL_QUEUE_FULL'
    assert final['journal']['enqueued_events']==final['journal']['written_events']==1
    assert final['journal']['pending_events']==0 and final['journal']['worker_exited']
    assert final['journal']['final_fsync_complete'] is True and sink.closed
    assert calls.index('lease_close') < calls.index('journal_close') < calls.index('final_fsync')


@pytest.mark.parametrize('change', [{'arm_allowed':'true'}, {'max_linear_m_s':1}, {'max_angular_rad_s':.31},
                                  {'command_rpm_to_register_scale':float('nan')}, {'unknown':1},
                                  {'arm_allowed':True}, {'reason':None}])
def test_manual_config_defaults_and_rejections(change):
    with pytest.raises(m.FeedbackError): m.validate_manual_controls(change)
    assert m.validate_manual_controls()['arm_allowed'] is False


@pytest.mark.parametrize('duration', [0, 1, 59, 2401, 86400, -1, True, False, 1.5, '60'])
def test_main_duration_is_owned_by_supervisor_and_stops_on_its_signal(tmp_path, monkeypatch, duration):
    """Exercise the actual entrypoint with only in-memory serial/ROS stand-ins."""
    from types import ModuleType
    from wc_motion import feedback_transport as ft
    cli = ModuleType('wc_runtime.cli')
    cli.ROOT, cli.RUN, cli.target = tmp_path, tmp_path/'run', lambda: None
    monkeypatch.setitem(sys.modules, 'wc_runtime.cli', cli)
    hardware = tmp_path/'hardware.json'
    hardware.write_bytes((ROOT/'config/wheel_feedback_current.json').read_bytes())
    session = tmp_path/'session'; session.mkdir()
    (session/'runtime_config.json').write_text(json.dumps({**config(), 'duration_s': duration,
                                                        'wheel_hardware_config': str(hardware)}))
    calls, signals, clock = [], {}, Clock()
    monkeypatch.setattr(m.signal, 'SIGHUP', 1, raising=False)
    monkeypatch.setattr(m.signal, 'signal', lambda number, handler: signals.update({number: handler}))
    monkeypatch.setattr(m.time, 'monotonic', clock)
    monkeypatch.setattr(m.time, 'sleep', lambda _: None)
    class Lease:
        host_configuration = {'fixture': True}
        def __init__(self, *_): calls.append('lease_created')
        def open(self): calls.append('lease_open')
        def close(self): calls.append('lease_close')
    class Journal:
        def __call__(self, row): calls.append(row['event'])
        def close(self): calls.append('journal_close')
        def status(self): return {'fixture': True}
    class Owner:
        failure = None
        def __init__(self, *args, **kwargs):
            self.channel = NS(control_transmissions=0)
        def step(self):
            calls.append('step')
            clock.advance(100000)  # Greater than every finite requested duration.
            if calls.count('step') == 2:
                signals[m.signal.SIGINT](m.signal.SIGINT, None)
        def status(self): return {'state': 'DISARMED', 'control_transmissions': 0}
        def close(self): calls.append('owner_close')
        def fault(self, reason): self.failure = reason
    monkeypatch.setattr(ft, 'FeedbackSerialLease', Lease)
    monkeypatch.setattr(ft, 'create_journal', lambda *args, **kwargs: Journal())
    monkeypatch.setattr(m, 'RosOutput', lambda: NS(publish=lambda *_: None,
        manual=NS(publish=lambda _: None), String=lambda **kwargs: NS(**kwargs), close=lambda: calls.append('ros_close')))
    monkeypatch.setattr(m, 'WheelChannel', lambda *_: None)
    monkeypatch.setattr(m, 'MappingWheel', Owner)
    monkeypatch.setattr(m, 'ManualSocket', lambda *_: NS(pump=lambda: None, close=lambda: calls.append('socket_close')))
    if type(duration) is not int or duration < 0:
        with pytest.raises(m.FeedbackError, match='nonnegative integer'):
            m.main(['--session-root', str(session)])
        assert calls == []  # Reject before even constructing a serial lease.
    else:
        assert m.main(['--session-root', str(session)]) == 0
        final = json.loads((session/'wheel_status.json').read_text())
        assert calls.count('step') == 2  # No independent expiry or finite cap.
        assert final['state'] == 'STOPPED' and final['failure'] is None
        assert final['manual']['control_transmissions'] == 0
        assert calls.index('owner_close') < calls.index('lease_close') < calls.index('journal_close')


def test_config_validator_copies_user_values_and_preserves_unknown_watchdog():
    original = config(True)['manual_controls']; before=copy.deepcopy(original)
    result=m.validate_manual_controls(original)
    result['operation_evidence']['note']='changed'
    assert original==before
    value,*_=owner()
    assert value.status()['firmware_id'] is None
    assert value.status()['hardware_watchdog_timeout_s'] is None
    assert value.status()['hardware_watchdog_verified'] is False


def test_current_authorized_hybrid_config_requires_fresh_foreground_before_release():
    manual = json.loads((ROOT/'config/hardware_setup.json').read_text(encoding='utf-8'))['manual_controls']
    assert manual['arm_allowed'] is True
    assert manual['operation_evidence']['status'] == 'USER_REPORTED_LEGACY_OPERATION'
    assert '2026-09-14' in manual['operation_evidence']['note']
    assert '明确授权解除' in manual['operation_evidence']['note']
    assert '硬件看门狗时限仍未知' in manual['operation_evidence']['note']
    cfg = config(); cfg['manual_controls'] = manual
    clock, channel, records = Clock(), FakeChannel(), []
    value = m.MappingWheel(cfg, channel, records.append,
                           clock=clock, wall_ns=clock.wall, mono_ns=clock.ns)
    value.connected(); ticks(value, clock, 230)
    assert value.state == 'READY' and set(channel.requests) == {m.QUERY}
    assert channel.control_transmissions == 0
    status = value.status()
    assert status['arm_allowed'] is True and status['arm_block_reason'] is None
    assert status['interaction_policy'] == status['control_mode'] == 'hybrid_manual'
    assert status['ready_without_arming'] is True
    assert status['release_after_zero_s'] is None and status['max_continuous_motion_s'] is None
    assert status['brake_release_policy'] == 'FRESH_FOREGROUND_NORMAL_IDLE'
    assert status['hardware_watchdog_verified'] is False and status['hardware_watchdog_timeout_s'] is None
    # Even a legacy arm packet is no longer an enable request.
    value.command({'type': 'arm', 'session_id': value.session_id, 'confirmed': True})
    value.close()
    assert set(channel.requests) == {m.QUERY} and channel.control_transmissions == 0


def hybrid_owner(allowed=True):
    clock, channel, records, published = Clock(), FakeChannel(), [], []
    cfg = config(allowed)
    cfg['manual_controls']['interaction_policy'] = 'hybrid_manual'
    value = m.MappingWheel(cfg,channel,records.append,clock=clock,wall_ns=clock.wall,mono_ns=clock.ns,
        publish=lambda source,preview:published.append((source,preview)))
    value.connected()
    return value,clock,channel,records,published


def test_hybrid_startup_release_then_wasd_and_normal_release_cycle_without_buttons():
    value,clock,channel,records,published = hybrid_owner()
    ticks(value,clock,50)
    assert set(channel.requests) == {m.QUERY}
    hold(value,clock,[],50)
    assert value.push_mode and channel.requests.count(m.RELEASE_REQUEST) == 1
    assert m.ZERO_REQUEST not in channel.requests and not any(r in channel.requests for r in m.INITIALIZATION_WRITES)
    hold(value,clock,['w'],90)
    assert not value.push_mode and value.state == 'ARMED'
    assert any(r[1] == 16 and r != m.ZERO_REQUEST for r in channel.requests)
    assert not value.servo_release_acknowledged
    keys(value,[])
    hold(value,clock,[],60)
    assert value.push_mode and channel.requests.count(m.RELEASE_REQUEST) == 2
    assert channel.requests.index(m.ZERO_REQUEST) < len(channel.requests)-1
    assert value.status()['brake_holding_confirmed'] is False
    assert value.status()['physical_stop_confirmed'] is False
    assert [record['sequence'] for record,_ in published] == list(range(len(published)))
    assert value.sequence == value.published_feedback_count == len(published)


@pytest.mark.parametrize('cause',['foreground','timeout','disconnect','stop','fault','cancel'])
def test_hybrid_abnormal_cancellation_never_becomes_idle_release(cause):
    value,clock,channel,records,published = hybrid_owner()
    hold(value,clock,['w'],90)
    assert value.state == 'ARMED'
    if cause == 'foreground': keys(value,[],foreground=False)
    elif cause == 'timeout': clock.advance(.3)
    elif cause == 'disconnect': value.disconnected()
    elif cause == 'stop': value.command({'type':'disarm','session_id':value.session_id})
    elif cause == 'fault': value.fault('SYNTHETIC FAULT')
    else: value.cancelled = lambda:True
    ticks(value,clock,50)
    assert m.RELEASE_REQUEST not in channel.requests
    assert value.hybrid_release_inhibited
    if cause not in ('fault','cancel'):
        if cause == 'disconnect': value.connected()
        hold(value,clock,[],50)
        assert m.RELEASE_REQUEST not in channel.requests
        hold(value,clock,['d'],90)
        assert value.state == 'ARMED'
        keys(value,[]); hold(value,clock,[],50)
        assert value.push_mode and channel.requests.count(m.RELEASE_REQUEST) == 1


def test_hybrid_disconnect_while_released_reports_release_without_claiming_brakes():
    value,clock,channel,_,_ = hybrid_owner()
    hold(value,clock,[],50)
    assert value.push_mode
    count = channel.control_transmissions
    value.disconnected(); ticks(value,clock,40)
    status = value.status()
    assert status['push_mode'] and status['servo_release_acknowledged']
    assert not status['brake_holding_confirmed'] and not status['physical_stop_confirmed']
    assert channel.control_transmissions == count


def test_hybrid_requires_explicit_foreground_field_and_zero_feedback_window():
    value,clock,channel,_,_ = hybrid_owner()
    with pytest.raises(m.FeedbackError,match='foreground'):
        value.command({'type':'keys','session_id':value.session_id,'sequence':1,
            'arm_generation':value.arm_generation,'keys':[]})
    channel.words = (1,0)
    hold(value,clock,[],70)
    assert not value.push_mode and m.RELEASE_REQUEST not in channel.requests
    channel.words = (0,0)
    hold(value,clock,[],50)
    assert value.push_mode


@pytest.mark.parametrize('cancel',['new_key','focus_loss','expired_heartbeat','generation_change'])
def test_hybrid_release_prewrite_rechecks_current_user_intent(cancel):
    value,clock,channel,_,_ = hybrid_owner()
    original = channel.exchange
    def intercept(request,timeout,**kwargs):
        if request == m.RELEASE_REQUEST:
            if cancel == 'new_key': keys(value,['w'])
            elif cancel == 'focus_loss': keys(value,[],foreground=False)
            elif cancel == 'expired_heartbeat': clock.advance(.3)
            else: value.disarm('SYNTHETIC GENERATION CHANGE')
        return original(request,timeout,**kwargs)
    channel.exchange = intercept
    hold(value,clock,[],30)
    assert m.RELEASE_REQUEST not in channel.requests and not value.push_mode


def test_hybrid_config_without_authorization_stays_readonly_despite_heartbeat():
    value,clock,channel,_,_ = hybrid_owner(False)
    hold(value,clock,[],100); hold(value,clock,['w'],100)
    value.close()
    assert set(channel.requests) == {m.QUERY} and channel.control_transmissions == 0


def test_source_counter_preserves_captured_response_if_publication_fails():
    value,clock,channel,records,published = hybrid_owner(False)
    value.publish = lambda *args: (_ for _ in ()).throw(RuntimeError('synthetic publication failed'))
    value.step()
    assert value.sequence == 1 and value.published_feedback_count == 0 and value.publication_failures == 1
    assert len([r for r in records if r['event']=='transaction_complete']) == 1
    assert value.state == 'FAULT'


@pytest.mark.parametrize('source_mode,ack_mode',[('real','pass'),('real','false'),('real','throw'),
                                                ('replay','pass'),('synthetic','pass')])
def test_actual_manual_capture_main_records_ready_raw_summary_and_control_counts(tmp_path,monkeypatch,source_mode,ack_mode):
    from types import ModuleType
    from wc_motion import feedback_transport as ft
    cli = ModuleType('wc_runtime.cli')
    cli.ROOT,cli.RUN,cli.target = tmp_path,tmp_path/'run',lambda:None
    monkeypatch.setitem(sys.modules,'wc_runtime.cli',cli)
    dataset = tmp_path/'synthetic_manual_capture'; (dataset/'configuration').mkdir(parents=True)
    (dataset/'sources').mkdir()
    hardware = dataset/'configuration/wheel_feedback.json'
    hardware.write_bytes((ROOT/'config/wheel_feedback_current.json').read_bytes())
    cfg = config(True); cfg.update(session_id=dataset.name,source_mode=source_mode,status='EXPERIMENT',
        duration_s=0,continuous_mapping=True,wheel_hardware_config=str(hardware),
        manual_authorization={'source':'EXPLICIT_CAPTURE_MANUAL_DRIVE_ARGUMENT'})
    cfg['manual_controls']['interaction_policy'] = 'hybrid_manual'
    manual_config = dataset/'configuration/manual_runtime.json'
    manual_config.write_text(json.dumps(cfg),encoding='utf-8')
    (dataset/'capture_manifest.json').write_text(json.dumps({'manual_drive':True,'session_id':dataset.name,
        'input_hashes':{'configuration/manual_runtime.json':hashlib.sha256(manual_config.read_bytes()).hexdigest()}}))
    calls,signals,clock,owners = [],{},Clock(),[]
    duration_module=ModuleType('rclpy.duration');duration_module.Duration=lambda **kwargs:NS(**kwargs)
    if 'rclpy' not in sys.modules:monkeypatch.setitem(sys.modules,'rclpy',ModuleType('rclpy'))
    monkeypatch.setitem(sys.modules,'rclpy.duration',duration_module)
    real_recording_ack=m.RosOutput.wait_for_recording_ack
    monkeypatch.setattr(m.signal,'SIGHUP',1,raising=False)
    monkeypatch.setattr(m.signal,'signal',lambda number,handler:signals.update({number:handler}))
    monkeypatch.setattr(m.time,'monotonic',clock)
    monkeypatch.setattr(m.time,'sleep',lambda _:clock.advance(.01))
    class Lease:
        host_configuration = {'fixture':'SYNTHETIC_NO_SERIAL'}
        def __init__(self,*args): calls.append('lease_created')
        def open(self): calls.append('lease_open')
        def close(self): calls.append('lease_close')
    class Output:
        raw_publish_count = raw_publish_failures = 0
        def __init__(self):
            self.manual = NS(publish=lambda _:None); self.String = lambda **kwargs:NS(**kwargs)
        def wait_for_recorder(self,cancelled):
            calls.append('recorder_discovered'); return True
        def publish(self,record,candidate):
            self.raw_publish_count += 1
            assert record['sequence'] == self.raw_publish_count-1
        @property
        def raw(self):return NS(wait_for_all_acked=self.publication_ack)
        def publication_ack(self,timeout):
            calls.append('recording_ack');assert timeout.seconds==5.0
            if ack_mode=='throw':raise RuntimeError('synthetic DDS ACK exception')
            return ack_mode=='pass'
        def wait_for_recording_ack(self):return real_recording_ack(self)
        def close(self): calls.append('ros_close')
    class Socket:
        def __init__(self,path,owner):
            self.owner=owner; owners.append(owner); owner.connected(); calls.append('socket_open')
        def pump(self):
            keys(self.owner,['w'] if 4 <= self.owner.sequence < 10 else [])
            if self.owner.sequence >= 16:
                assert json.loads((dataset/'wheel_ready.json').read_text())['ready'] is True
                signals[m.signal.SIGINT](m.signal.SIGINT,None)
        def close(self): self.owner.disconnected(); calls.append('socket_close')
    class JournalChannel(FakeChannel):
        def exchange(self,request,timeout,**kwargs):
            response_bytes = super().exchange(request,timeout,**kwargs)
            self.journal({'event':'wheel_io_complete','request_hex':request.hex(),
                'response_hex':response_bytes.hex(),'transmitted_bytes':len(request),
                'control_transmissions':self.control_transmissions})
            return response_bytes
    channel = JournalChannel()
    monkeypatch.setattr(ft,'FeedbackSerialLease',Lease)
    monkeypatch.setattr(m,'RosOutput',Output)
    def make_channel(lease,journal):
        channel.journal = journal
        return channel
    monkeypatch.setattr(m,'WheelChannel',make_channel)
    monkeypatch.setattr(m,'ManualSocket',Socket)
    real_owner = m.MappingWheel
    original_create_journal=ft.create_journal
    def create_journal(*args,**kwargs):
        journal=original_create_journal(*args,**kwargs);original_close=journal.close
        def close():calls.append('journal_close');return original_close()
        journal.close=close
        return journal
    monkeypatch.setattr(ft,'create_journal',create_journal)
    def make_owner(*args,**kwargs):
        return real_owner(*args,**kwargs,clock=clock,wall_ns=clock.wall,mono_ns=clock.ns)
    monkeypatch.setattr(m,'MappingWheel',make_owner)
    raw = dataset/'sources/wheel_feedback.jsonl'; summary = dataset/'sources/wheel_summary.json'
    arguments = ['--session-root',str(dataset),'--config',str(manual_config),
        '--raw-output',str(raw),'--summary-path',str(summary),'--require-recorder']
    if source_mode != 'real':
        with pytest.raises(m.FeedbackError,match='replay cannot control'):
            m.main(arguments)
        assert calls == [] and not raw.exists() and not summary.exists()
        return
    assert m.main(arguments) == (0 if ack_mode=='pass' else 1)
    assert calls.index('recorder_discovered') < calls.index('lease_open') < calls.index('socket_open')
    rows = [json.loads(line) for line in raw.read_text().splitlines()]
    transactions = [r for r in rows if r['event']=='transaction_complete']
    assert len(transactions) >= 16
    assert [r['sequence'] for r in transactions] == list(range(len(transactions)))
    assert all(r['response_sha256']==hashlib.sha256(bytes.fromhex(r['response_hex'])).hexdigest() for r in transactions)
    terminal = [r for r in rows if r['event']=='capture_complete']
    assert len(terminal)==1 and terminal[0]['completed']==len(transactions)
    semantic = [r for r in rows if r['event'] != 'journal_finalization']
    if ack_mode=='pass':
        assert [row['event'] for row in semantic[-2:]]==['lease_closed','publication_ack']
        assert semantic[-1]['acknowledged'] is True
    else:assert semantic[-1]['event']=='lease_closed'
    assert calls.index('lease_close')<calls.index('recording_ack')<calls.index('journal_close')<calls.index('ros_close')
    result = json.loads(summary.read_text())
    assert result['closed_normally'] is (ack_mode=='pass') and result['synchronized'] is (ack_mode=='pass')
    assert result['journal']['final_fsync_complete']
    assert result['events_sha256']==hashlib.sha256(raw.read_bytes()).hexdigest()
    assert result['completed']==result['published_feedback_count']==result['sequence_end_exclusive']==len(transactions)
    assert result['control_transmissions']==channel.control_transmissions>0
    assert result['publish_failures']==0 and result['recorder_discovered'] and result['ready_observed']
    assert channel.requests.count(m.RELEASE_REQUEST) >= 1 and channel.requests.count(m.ZERO_REQUEST) >= 1
    if ack_mode!='pass':
        assert result['state']=='FAILED'
        expected='acknowledgement timeout' if ack_mode=='false' else 'synthetic DDS ACK exception'
        assert expected in result['failure']
        final=json.loads((dataset/'wheel_status.json').read_text())
        assert final['state']=='FAILED' and expected in final['failure']
    from wc_runtime.capture_audit import audit_wheel_control
    assert audit_wheel_control(dataset,rows,result)['control_transmissions'] == channel.control_transmissions


@pytest.mark.skipif(sys.platform != 'linux',reason='real local Unix socket semantics on Linux only; no hardware')
def test_local_socket_handshake_mode_single_client_and_disconnect(socket_tmp_path):
    value,clock,channel,_,_=owner()
    server=m.ManualSocket(socket_tmp_path/'manual.sock',value)
    first=socket.socket(socket.AF_UNIX,socket.SOCK_STREAM)
    second=socket.socket(socket.AF_UNIX,socket.SOCK_STREAM)
    try:
        assert server.path.stat().st_mode & 0o777 == 0o600
        first.connect(str(server.path)); server.pump()
        first.sendall((json.dumps({'type':'hello','session_id':value.session_id})+'\n').encode()); server.pump()
        first.settimeout(.5)
        status=json.loads(first.recv(8192))
        assert status['arm_allowed'] is False and value.peer_connected
        second.connect(str(server.path)); server.pump(); second.settimeout(.5)
        assert second.recv(1)==b''
        first.close(); server.pump()
        assert not value.peer_connected and channel.control_transmissions==0
    finally:
        first.close();second.close();server.close()
    assert not server.path.exists()


@pytest.mark.skipif(sys.platform != 'linux',reason='local Unix socket semantics')
def test_wrong_socket_session_cannot_start_direct_control(socket_tmp_path):
    value,clock,channel,_,_=owner(True)
    server=m.ManualSocket(socket_tmp_path/'manual.sock',value)
    client=socket.socket(socket.AF_UNIX,socket.SOCK_STREAM)
    try:
        client.connect(str(server.path));server.pump()
        client.sendall(b'{"type":"hello","session_id":"wrong"}\n');server.pump()
        assert not value.peer_connected and value.state=='READY'
        value.step()
        assert not value.keys and channel.control_transmissions == 0
        assert set(channel.requests) == {m.QUERY}
    finally:
        client.close();server.close()
