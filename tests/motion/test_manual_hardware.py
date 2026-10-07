"""Mock-only executable control-path verification. No real device/actuator I/O.

All approved-looking evidence below is generated SYNTHETIC TEST DATA using
dummy status addresses. It must never be copied into an actual configuration.
"""

import copy
import hashlib
import json
from pathlib import Path
import struct

import pytest

from wc_motion import manual_hardware as mh
from wc_motion.protocol import FeedbackError, crc16, read_request


ROOT = Path(__file__).resolve().parents[2]
DEVICE_ID = json.loads((ROOT/'config/wheel_feedback_current.json').read_text())['device_id']
STATE_READ = read_request(1, 0x3000, 5)  # SYNTHETIC only, not an 8030D register claim.


def holding_response(words):
    payload = bytes((1, 3, len(words)*2))+struct.pack('>'+'H'*len(words), *words)
    return payload+crc16(payload)


def ack(request):
    return request[:6]+crc16(request[:6])


@pytest.fixture
def files(tmp_path):
    config = json.loads((ROOT/'config/wheel_manual_hardware.json').read_text())
    sources = []
    for kind in ('protocol', 'onsite_stop_test'):
        path = tmp_path/(kind+'.txt')
        path.write_text('SYNTHETIC TEST FIXTURE ONLY. NO VERIFIED DEVICE EVIDENCE. '+kind)
        sources.append({'kind': kind, 'path': path.name, 'sha256': hashlib.sha256(path.read_bytes()).hexdigest()})
    checks = [{'purpose': purpose, 'request_hex': STATE_READ.hex(), 'word_index': index,
               'mask': 65535, 'equals': value} for index, (purpose, value) in enumerate([
                   ('firmware', 7), ('drive_enabled', 1), ('velocity_mode', 3), ('controller_watchdog', 250), ('stop_policy', 0)])]
    evidence = {'schema': 'wc_manual_stop_evidence_v1', 'state': 'REVIEWED_FOR_EMPTY_CHAIR_MANUAL',
        'device_id': DEVICE_ID, 'firmware_id': 'SYNTHETIC-TEST-ONLY',
        'reviewer': 'SYNTHETIC_FIXTURE', 'reviewed_at': 'TEST_ONLY', 'sources': sources,
        'command_profile_sha256': mh.canonical_hash(config['command_profile']),
        'command_registers_units_directions_verified': True, 'controller_disconnect_stop_verified': True,
        'zero_velocity_hold_verified': True, 'physical_stop_and_brake_behavior_verified': True,
        'stop_strategy': 'zero_velocity_hold', 'stationary_feedback_abs_raw_max': 0,
        'timing': {'transaction_timeout_s': .02, 'key_timeout_s': .25, 'command_period_s': .1,
            'controller_watchdog_timeout_s': .25, 'zero_feedback_timeout_s': .5}, 'state_checks': checks}
    return config, evidence, tmp_path


@pytest.fixture
def contract(files):
    return mh.ManualContract(*files)


@pytest.mark.parametrize('field', ['state', 'firmware_id', 'reviewer', 'command_profile_sha256',
    'command_registers_units_directions_verified', 'controller_disconnect_stop_verified',
    'zero_velocity_hold_verified', 'physical_stop_and_brake_behavior_verified'])
def test_missing_or_changed_approval_cannot_enable_real_contract(files, field):
    config, evidence, root = files
    changed = copy.deepcopy(evidence)
    changed[field] = None
    config['hardware_enabled'] = True
    with pytest.raises(FeedbackError):
        mh.ManualContract(config, changed, root)


def test_source_hash_and_profile_binding_are_independent_gates(files):
    config, evidence, root = files
    config['command_profile']['wheel_radius_m'] = .2
    with pytest.raises(FeedbackError, match='profile'):
        mh.ManualContract(config, evidence, root)
    config['command_profile']['wheel_radius_m'] = .165
    (root/'protocol.txt').write_text('changed after review')
    with pytest.raises(FeedbackError, match='hash'):
        mh.ManualContract(config, evidence, root)


@pytest.mark.parametrize('fault', ['source_missing', 'state_missing', 'write_as_read', 'zero_watchdog', 'infinite_timeout', 'guessed_stop'])
def test_contract_rejects_incomplete_or_unsafe_stop_predicates(files, fault):
    config, evidence, root = files
    if fault == 'source_missing': evidence['sources'] = evidence['sources'][:1]
    elif fault == 'state_missing': evidence['state_checks'] = evidence['state_checks'][:4]
    elif fault == 'write_as_read': evidence['state_checks'][0]['request_hex'] = mh.ZERO_REQUEST.hex()
    elif fault == 'zero_watchdog': evidence['timing']['controller_watchdog_timeout_s'] = 0
    elif fault == 'infinite_timeout': evidence['timing']['transaction_timeout_s'] = float('inf')
    elif fault == 'guessed_stop': evidence['stop_strategy'] = 'write_200e_value_7'
    with pytest.raises(FeedbackError):
        mh.ManualContract(config, evidence, root)


def test_config_template_rejects_real_entry_before_serial_open(monkeypatch, tmp_path):
    monkeypatch.setattr(mh, 'FeedbackSerialLease', lambda *a: pytest.fail('unverified template reached serial constructor'))
    monkeypatch.setattr(mh, 'RosFeedbackPublisher', lambda: pytest.fail('unverified template initialized ROS'))
    output = tmp_path/'never-created.jsonl'
    with pytest.raises(FeedbackError, match='evidence'):
        mh.main(['--config', str(ROOT/'config/wheel_manual_hardware.json'), '--mode', 'real',
            '--evidence', str(ROOT/'config/wheel_manual_stop_evidence.template.json'),
            '--run-root', str(tmp_path/'.phase1_runtime'), '--onsite-confirmation', mh.ONSITE_CONFIRMATION,
            '--publish-ros', '--output', str(output)])
    assert not output.exists()


def test_default_mode_delegates_only_to_dry_run_without_reading_control_evidence(monkeypatch, tmp_path):
    from wc_motion import manual_teleop
    seen = []
    monkeypatch.setattr(manual_teleop, 'main', lambda args: seen.append(args) or 0)
    monkeypatch.setattr(mh, 'FeedbackSerialLease', lambda *a: pytest.fail('default dry-run accessed serial'))
    monkeypatch.setattr(mh, 'RosFeedbackPublisher', lambda: pytest.fail('default dry-run initialized ROS'))
    assert mh.main(['--config', str(ROOT/'config/wheel_manual_hardware.json'), '--output', str(tmp_path/'dry.jsonl')]) == 0
    assert 'wheel_manual_unvalidated.json' in seen[0][1] and '--mode' not in seen[0]


@pytest.mark.parametrize('linear, angular', [(0, 0), (.1, 0), (-.1, 0), (0, .2), (0, -.2), (.1, .2)])
def test_fc10_matches_reviewed_paired_signed_format_and_exact_ack(contract, linear, angular):
    request = contract.velocity_request(linear, angular)
    assert len(request) == 13 and request[:7] == bytes.fromhex('01102088000204')
    assert request[-2:] == crc16(request[:-2])
    assert contract.validate_write(request) == request
    assert mh.validate_ack(request, ack(request)) is None
    if linear > 0 and angular == 0:
        left, right = struct.unpack('>hh', request[7:11])
        assert left < 0 < right


@pytest.mark.parametrize('bad', ['crc', 'slave', 'function', 'register', 'count', 'extra', 'exception'])
def test_fc10_ack_corruption_and_wrong_echo_are_rejected(contract, bad):
    request = contract.velocity_request(.1, 0)
    reply = bytearray(ack(request))
    if bad == 'crc': reply[-1] ^= 1
    elif bad == 'extra': reply.extend(b'x')
    elif bad == 'exception': reply = bytearray(b'\x01\x90\x02'+crc16(b'\x01\x90\x02'))
    else:
        index = {'slave': 0, 'function': 1, 'register': 3, 'count': 5}[bad]
        reply[index] ^= 1
        reply[-2:] = crc16(reply[:-2])
    with pytest.raises(FeedbackError):
        mh.validate_ack(request, bytes(reply))


class Clock:
    def __init__(self): self.now = 10.
    def sleep(self, value): self.now += max(0, value)


class Wire:
    def __init__(self, clock):
        self.clock = clock
        self.writes = []
        self.pending = []
        self.read_count = 0
        self.states = [7, 1, 3, 250, 0]
        self.feedback = [0, 0]
        self.fail_next_nonzero = None
        self.failed = False

    def select(self, readable, writable, exceptional, timeout):
        if writable:
            return [], list(writable), []
        if self.pending:
            return list(readable), [], []
        self.clock.sleep(timeout)
        return [], [], []

    def write(self, fd, request):
        assert fd == 900
        self.writes.append(request)
        if request[1] == 3:
            self.pending.append(holding_response(self.feedback if request == mh.QUERY else self.states))
        else:
            reply = ack(request)
            if request != mh.ZERO_REQUEST and self.fail_next_nonzero:
                failure, self.fail_next_nonzero = self.fail_next_nonzero, None
                self.failed = True
                if failure == 'timeout': return len(request)
                if failure == 'partial': return 3
                if failure == 'crc': reply = reply[:-1]+bytes((reply[-1] ^ 1,))
                if failure == 'exception': reply = b'\x01\x90\x02'+crc16(b'\x01\x90\x02')
            self.pending.append(reply)
        return len(request)

    def read(self, fd, count):
        assert fd == 900
        self.read_count += 1
        return self.pending.pop(0)


@pytest.fixture
def hardware(contract, monkeypatch):
    clock = Clock()
    wire = Wire(clock)
    monkeypatch.setattr(mh.time, 'monotonic', lambda: clock.now)
    monkeypatch.setattr(mh.time, 'monotonic_ns', lambda: int(clock.now*1e9))
    monkeypatch.setattr(mh.time, 'time_ns', lambda: int(clock.now*1e9))
    monkeypatch.setattr(mh.time, 'sleep', clock.sleep)
    monkeypatch.setattr(mh.select, 'select', wire.select)
    monkeypatch.setattr(mh.os, 'write', wire.write)
    monkeypatch.setattr(mh.os, 'read', wire.read)
    class Lease:
        fd = 900
        def close(self): self.fd = None
    journal = []
    channel = mh.ManualRtuChannel(Lease(), contract, journal.append)
    session = mh.ManualHardwareSession(contract, channel, onsite_confirmation=mh.ONSITE_CONFIRMATION)
    return session, channel, wire, clock, journal


def writes_only(wire):
    return [request for request in wire.writes if request[1] != 3]


def test_startup_preflight_and_disarmed_exit_are_read_only_and_include_no_enable(hardware):
    session, channel, wire, clock, journal = hardware
    session.preflight()
    session.close()
    assert writes_only(wire) == [] and channel.lease.fd is None
    assert len([event for event in journal if event.get('schema') == 'wc_wheel_feedback_v1']) == 3
    assert session.status()['controller_enable_written'] is False


def test_nonzero_requires_e_and_valid_live_feedback(hardware):
    session, channel, wire, clock, journal = hardware
    with pytest.raises(FeedbackError, match='arming'):
        channel.velocity(.1, 0)
    session.key_event('w', True, clock.now)
    assert writes_only(wire) == []
    session.arm(operator_key='e')
    assert writes_only(wire) == []
    session.key_event('w', True, clock.now)
    assert len(writes_only(wire)) == 1 and writes_only(wire)[0] != mh.ZERO_REQUEST
    session.key_event(' ', True, clock.now)
    assert writes_only(wire)[-1] == mh.ZERO_REQUEST
    assert channel.last_stop['acknowledged'] and channel.last_stop['stationary_feedback_observed']
    assert channel.last_stop['physical_stop_confirmed'] is False
    session.close()
    assert len(writes_only(wire)) == 2, 'closed, already stopped session must not send another zero'


@pytest.mark.parametrize('fault', ['disabled', 'firmware', 'watchdog', 'moving'])
def test_failed_preflight_cannot_arm_or_write_motor_registers(hardware, fault):
    session, channel, wire, clock, journal = hardware
    if fault == 'disabled': wire.states[1] = 0
    elif fault == 'firmware': wire.states[0] = 999
    elif fault == 'watchdog': wire.states[3] = 0
    else: wire.feedback = [1, 1]
    with pytest.raises(FeedbackError):
        session.arm(operator_key='e')
    session.close()
    assert writes_only(wire) == []


@pytest.mark.parametrize('failure', ['timeout', 'partial', 'crc', 'exception'])
def test_failed_nonzero_transaction_attempts_exactly_one_zero_and_never_claims_late_ack(hardware, failure):
    session, channel, wire, clock, journal = hardware
    session.arm(operator_key='e')
    wire.fail_next_nonzero = failure
    with pytest.raises((FeedbackError, OSError)):
        session.key_event('w', True, clock.now)
    reads_before = wire.read_count
    assert session.state == 'FAULT' and channel.failed
    assert writes_only(wire) == [channel.contract.velocity_request(.1, 0), mh.ZERO_REQUEST]
    assert channel.last_stop['acknowledged'] is False and channel.last_stop['physical_stop_confirmed'] is False
    session.close()
    assert wire.read_count == reads_before and len(writes_only(wire)) == 2
    assert wire.pending, 'late/zero ACK fixture bytes were incorrectly treated as verified stop response'


def test_key_timeout_and_replayed_key_cannot_resume_without_new_arm(hardware):
    session, channel, wire, clock, journal = hardware
    session.arm(operator_key='e')
    session.key_event('w', True, clock.now)
    clock.sleep(.3)
    session.key_event('w', True, clock.now)
    assert session.state == 'DISARMED' and writes_only(wire)[-1] == mh.ZERO_REQUEST
    count = len(writes_only(wire))
    session.key_event('w', True, clock.now)
    assert len(writes_only(wire)) == count


@pytest.mark.parametrize('offset', [-1, 1])
def test_stale_or_future_key_time_stops_instead_of_extending_lease(hardware, offset):
    session, channel, wire, clock, journal = hardware
    session.arm(operator_key='e')
    session.key_event('w', True, clock.now)
    with pytest.raises(FeedbackError, match='stale|future'):
        session.key_event('w', True, clock.now+offset)
    assert session.state == 'FAULT' and writes_only(wire)[-1] == mh.ZERO_REQUEST


def test_feedback_delay_does_not_dispatch_expired_user_intent(hardware, monkeypatch):
    session, channel, wire, clock, journal = hardware
    session.arm(operator_key='e')
    monkeypatch.setattr(session, '_read_checks', lambda: clock.sleep(.3))
    session.key_event('w', True, clock.now)
    assert writes_only(wire) == [] and session.state == 'DISARMED'


def test_uncertain_zero_feedback_is_not_a_successful_stop(hardware):
    session, channel, wire, clock, journal = hardware
    session.arm(operator_key='e')
    session.key_event('w', True, clock.now)
    wire.feedback = [5, 5]
    session.stop('FIXTURE_STOP')
    assert session.state == 'FAULT' and channel.last_stop['acknowledged'] is False
    assert channel.last_stop['stationary_feedback_observed'] is False


def test_serial_allowlist_has_no_control_word_fault_mode_or_enable_path(hardware):
    session, channel, wire, clock, journal = hardware
    for function, address in ((6, 0x200e), (6, 0x200d), (0x10, 0x200e)):
        payload = struct.pack('>BBHH', 1, function, address, 8)
        with pytest.raises(FeedbackError):
            channel._exchange(payload+crc16(payload))
    assert wire.writes == []


def test_unexpected_existing_bytes_cannot_be_accepted_as_new_reply(hardware):
    session, channel, wire, clock, journal = hardware
    wire.pending.append(holding_response([0, 0]))
    with pytest.raises(FeedbackError, match='unexpected'):
        session.preflight()
    assert wire.writes == []


def test_each_gui_key_expires_independently_and_terminal_direction_replaces_previous(hardware):
    session, channel, wire, clock, journal = hardware
    session.arm(operator_key='e')
    session.key_event('w', True, clock.now)
    clock.sleep(.1)
    session.key_event('a', True, clock.now)
    clock.sleep(.2)
    session.key_event('a', True, clock.now)
    assert session.keys == {'a'}
    assert writes_only(wire)[-1] == channel.contract.velocity_request(0, .2)
    session.terminal_key('w', clock.now)
    session.terminal_key('a', clock.now)
    assert session.keys == {'a'} and writes_only(wire)[-1] == channel.contract.velocity_request(0, .2)


def test_stop_signal_during_feedback_return_allows_only_zero_afterward(hardware, monkeypatch):
    session, channel, wire, clock, journal = hardware
    cancelled = [False]
    channel.cancel_check = lambda: cancelled[0]
    session.arm(operator_key='e')
    session.key_event('w', True, clock.now)
    original_read = wire.read
    def signal_during_read(fd, count):
        reply = original_read(fd, count)
        cancelled[0] = True
        return reply
    monkeypatch.setattr(mh.os, 'read', signal_during_read)
    clock.sleep(.11)
    with pytest.raises(mh.MotionRejected, match='cancelled'):
        session.tick()
    assert writes_only(wire) == [channel.contract.velocity_request(.1, 0), mh.ZERO_REQUEST]
    assert channel.last_stop['acknowledged'] is True
    assert session.state == 'FAULT'


def test_signal_after_request_planning_but_before_write_blocks_initial_nonzero(hardware):
    session, channel, wire, clock, journal = hardware
    cancelled = [False]
    channel.cancel_check = lambda: cancelled[0]
    session.arm(operator_key='e')
    def observe(record):
        journal.append(record)
        request = bytes.fromhex(record.get('request_hex', ''))
        if record.get('event') == 'request_planned' and len(request) > 1 and request[1] == 16 and request != mh.ZERO_REQUEST:
            cancelled[0] = True
    channel.journal = observe
    with pytest.raises(mh.MotionRejected):
        session.key_event('w', True, clock.now)
    assert writes_only(wire) == [] and session.state == 'FAULT'


def test_actual_watchdog_gap_cannot_be_refreshed_by_a_new_key(hardware):
    session, channel, wire, clock, journal = hardware
    # Valid reviewed variation: key lease .5s, independent firmware watchdog .25s.
    channel.contract.key_timeout = .5
    session.arm(operator_key='e')
    session.key_event('w', True, clock.now)
    clock.sleep(.3)
    with pytest.raises(mh.MotionRejected, match='watchdog'):
        session.key_event('w', True, clock.now)
    assert writes_only(wire) == [channel.contract.velocity_request(.1, 0), mh.ZERO_REQUEST]
    assert session.state == 'FAULT'


@pytest.mark.parametrize('fault', ['transaction_total', 'unique_read_count'])
def test_full_tick_budget_must_fit_hardware_watchdog(files, fault):
    config, evidence, root = files
    if fault == 'transaction_total':
        evidence['timing']['transaction_timeout_s'] = .08
    else:
        for index, check in enumerate(evidence['state_checks']):
            check['request_hex'] = read_request(1, 0x3100+index, 1).hex()
            check['word_index'] = 0
    with pytest.raises(FeedbackError, match='complete.*budget'):
        mh.ManualContract(config, evidence, root)


@pytest.mark.parametrize('payload', [b'\x1b[A', b'wa', b'ewww', b'w\n', b'\x00'])
def test_terminal_escape_or_buffered_paste_is_not_replayed_as_current_key(monkeypatch, payload):
    monkeypatch.setattr(mh.os, 'read', lambda fd, count: payload)
    with pytest.raises(FeedbackError, match='terminal'):
        mh.terminal_event(900)


@pytest.mark.parametrize('payload, expected', [(b'W', 'w'), (b'a', 'a'), (b'e', 'e'), (b'', 'q')])
def test_terminal_accepts_single_raw_event_or_eof_only(monkeypatch, payload, expected):
    monkeypatch.setattr(mh.os, 'read', lambda fd, count: payload)
    assert mh.terminal_event(900) == expected


def test_terminal_backlog_is_discarded_after_state_transition(monkeypatch):
    queue = [b'ewww', b'\x1b[A']
    monkeypatch.setattr(mh.select, 'select', lambda *a: ([900] if queue else [], [], []))
    monkeypatch.setattr(mh.os, 'read', lambda fd, count: queue.pop(0))
    mh.discard_terminal_backlog(900)
    assert queue == []


def test_terminal_backlog_flush_has_a_bound(monkeypatch):
    monkeypatch.setattr(mh.select, 'select', lambda *a: ([900], [], []))
    monkeypatch.setattr(mh.os, 'read', lambda fd, count: b'w'*64)
    with pytest.raises(FeedbackError, match='flood'):
        mh.discard_terminal_backlog(900)


@pytest.fixture
def ros_double(monkeypatch):
    from types import SimpleNamespace
    events = []
    context = SimpleNamespace(active=False)
    context.ok = lambda: context.active
    no_handlers = object()
    messages = []
    def publish(message):
        messages.append(message)
    def create_publisher(message_type, topic, depth):
        events.append(('publisher', message_type, topic, depth))
        return SimpleNamespace(publish=publish)
    node = SimpleNamespace(create_publisher=create_publisher,
        destroy_node=lambda: events.append(('destroy',)))
    class String:
        def __init__(self, *, data): self.data = data
    def init(**kwargs):
        events.append(('init', kwargs))
        context.active = True
    def create_node(name, **kwargs):
        events.append(('node', name, kwargs))
        return node
    def shutdown(**kwargs):
        events.append(('shutdown', kwargs))
        context.active = False
    ros = SimpleNamespace(init=init, create_node=create_node, shutdown=shutdown)
    monkeypatch.setattr(mh, '_ros_bindings', lambda: (ros, context, no_handlers, String))
    return SimpleNamespace(ros=ros, context=context, no_handlers=no_handlers,
        String=String, events=events, messages=messages, node=node)


def test_raw_publisher_preserves_same_owner_feedback_without_claiming_valid_time(hardware, ros_double):
    session, channel, wire, clock, journal = hardware
    session.preflight()
    raw = next(record for record in journal if record.get('schema') == 'wc_wheel_feedback_v1')
    publisher = mh.RosFeedbackPublisher(DEVICE_ID)
    assert publisher.publish_record(raw)
    assert not publisher.publish_record({'event': 'transaction_complete', 'request_hex': mh.ZERO_REQUEST.hex()})
    assert len(ros_double.messages) == 1
    assert json.loads(ros_double.messages[0].data) == raw
    assert raw['time_valid'] is False and raw['time_source'] == 'arrival_only'
    assert raw['formal_odometry_eligible'] is False
    assert ('publisher', ros_double.String, '/wc_mapping/wheel/feedback_raw', 20) in ros_double.events
    assert ros_double.events[0] == ('init', {'args': [], 'context': ros_double.context,
        'signal_handler_options': ros_double.no_handlers})
    assert ros_double.events[1][2] == {'context': ros_double.context,
        'start_parameter_services': False, 'enable_rosout': False}
    publisher.close()
    publisher.close()
    assert ros_double.events[-2:] == [('destroy',), ('shutdown', {
        'context': ros_double.context, 'uninstall_handlers': False})]


def test_ros_creation_failure_releases_its_context_without_replacing_signal_handlers(ros_double):
    def fail(*args, **kwargs):
        raise RuntimeError('synthetic node creation failure')
    ros_double.ros.create_node = fail
    with pytest.raises(RuntimeError, match='synthetic'):
        mh.RosFeedbackPublisher(DEVICE_ID)
    assert not ros_double.context.active
    assert ros_double.events[-1] == ('shutdown', {'context': ros_double.context, 'uninstall_handlers': False})


def test_publish_failure_cancels_active_motion_and_does_not_block_stop_feedback(hardware, ros_double):
    session, channel, wire, clock, journal = hardware
    publisher = mh.RosFeedbackPublisher(DEVICE_ID)
    def observe(record):
        journal.append(record)
        publisher.publish_record(record)
    channel.journal = observe
    session.journal = observe
    session.arm(operator_key='e')
    session.key_event('w', True, clock.now)
    def fail(message):
        raise RuntimeError('synthetic ROS failure')
    publisher.publisher.publish = fail
    clock.sleep(.11)
    with pytest.raises(RuntimeError, match='synthetic ROS'):
        session.tick()
    assert session.state == 'FAULT' and publisher.failed
    assert writes_only(wire) == [channel.contract.velocity_request(.1, 0), mh.ZERO_REQUEST]
    assert channel.last_stop['acknowledged'] and channel.last_stop['stationary_feedback_observed']
    session.close()
    assert session.closed
    publisher.close()


def test_ros_rejects_uncompleted_raw_feedback(ros_double):
    publisher = mh.RosFeedbackPublisher(DEVICE_ID)
    with pytest.raises(FeedbackError, match='completed'):
        publisher.publish_record({'schema': 'wc_wheel_feedback_v1', 'status': 'TIMEOUT'})
    assert not ros_double.messages and publisher.failed
    publisher.close()


def test_dry_run_publish_flag_fails_without_creating_ros_or_serial(monkeypatch, tmp_path):
    monkeypatch.setattr(mh, '_ros_bindings', lambda: pytest.fail('dry run imported ROS'))
    monkeypatch.setattr(mh, 'FeedbackSerialLease', lambda *a: pytest.fail('dry run acquired serial'))
    with pytest.raises(FeedbackError, match='gated real mode'):
        mh.main(['--config', str(ROOT/'config/wheel_manual_hardware.json'), '--publish-ros',
            '--output', str(tmp_path/'never.jsonl')])
    assert not (tmp_path/'never.jsonl').exists()
