"""Base bridge without ROS: socket forwarding, faults, enable/estop outputs, base status and the end-to-end path."""
import datetime
import json
from pathlib import Path
import socket
import sys
import tempfile

import pytest

from wc_motion import auto_bridge as b
from wc_motion import auto_protocol as p


class ClockNs:
    def __init__(self): self.value = 5_000_000_000
    def __call__(self): return self.value
    def advance(self, seconds): self.value += round(seconds*1e9)


@pytest.fixture
def short_tmp():
    with tempfile.TemporaryDirectory(prefix='wc-bridge-', dir='/tmp') as folder:
        yield Path(folder)


def gate_text(clock, **change):
    value = {'schema': p.GATE_COMMAND_SCHEMA, 'gate_id': 'gate_1', 'seq': 1, 'stamp_ns': 1, 'mono_ns': clock(),
             'state': 'ACTIVE', 'reasons': [], 'linear': 0.05, 'angular': 0.0, 'config_sha256': None,
             'limits': {'max_linear': 0.1, 'max_angular': 0.3, 'linear_ceiling': 0.1, 'allow_reverse': False}}
    value.update(change)
    return json.dumps(value)


def served(path):
    server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    server.bind(str(path))
    server.listen(1)
    return server


def read_lines(conn, count):
    conn.settimeout(1.0)
    data = b''
    while data.count(b'\n') < count:
        chunk = conn.recv(65536)
        assert chunk
        data += chunk
    return [p.decode_line(line) for line in data.split(b'\n') if line]


def connected(short_tmp, clock):
    server = served(short_tmp/'auto.sock')
    core = b.BridgeCore('session_a', 'bridge_1', clock_ns=clock)
    core.connect(short_tmp/'auto.sock')
    conn, _ = server.accept()
    return server, core, conn


def owner_status(**change):
    value = {'type': 'status', 'schema': p.STATUS_SCHEMA, 'session_id': 'session_a', 'source': 'AUTO',
             'auto_state': 'AUTO', 'dead_man': True, 'estop': False,
             'feedback': {'sequence': 41, 'stamp_mono_ns': 4_990_000_000, 'age_s': 0.01, 'linear_m_s': 0.05}}
    value.update(change)
    return value


def deliver(core, conn, status):
    conn.sendall(p.encode_line(status))
    before = core.counters['status'] + core.counters['status_rejected']
    for _ in range(50):
        core.pump()
        if core.counters['status'] + core.counters['status_rejected'] > before:
            return


def test_bridge_sends_hello_then_one_decision_per_fresh_gate_command(short_tmp):
    clock = ClockNs()
    server, core, conn = connected(short_tmp, clock)
    assert core.on_gate_command(gate_text(clock)) is True
    assert core.on_gate_command(gate_text(clock, seq=2, linear=0.07)) is True
    hello, first, second = read_lines(conn, 3)
    assert p.check_hello(hello, 'session_a') == 'bridge_1'
    assert [first['sequence'], second['sequence']] == [1, 2] and second['linear'] == 0.07
    decision = p.parse_decision(second, 'session_a', 1)
    assert decision['gate_seq'] == 2 and decision['gate_id'] == 'gate_1' and decision['limits']['allow_reverse'] is False
    core.close(); conn.close(); server.close()


def test_stale_gate_commands_are_dropped(short_tmp):
    clock = ClockNs()
    server, core, conn = connected(short_tmp, clock)
    old = gate_text(clock)
    clock.advance(0.31)
    assert core.on_gate_command(old) is False                                       # older than 0.3 s
    assert core.counters['gate_stale'] == 1 and core.sequence == 0
    assert len(read_lines(conn, 1)) == 1                                            # only the hello
    core.close(); conn.close(); server.close()


@pytest.mark.parametrize('text', [
    '{"schema": "wc_gate_command_v1"}', 'not json', '[]',
    'NAN', 'FUTURE', 'NO_GATE_ID', 'TWO_LIMITS'])
def test_malformed_or_future_gate_commands_stop_the_bridge(short_tmp, text):
    clock = ClockNs()
    server, core, conn = connected(short_tmp, clock)
    if text == 'NAN':
        text = gate_text(clock, linear=float('nan'))                               # json.dumps writes NaN
    elif text == 'FUTURE':
        text = gate_text(clock, mono_ns=clock() + 100_000_000)
    elif text == 'NO_GATE_ID':
        text = json.dumps({k: v for k, v in json.loads(gate_text(clock)).items() if k != 'gate_id'})
    elif text == 'TWO_LIMITS':
        text = gate_text(clock, limits={'max_linear': 0.1, 'max_angular': 0.3})
    with pytest.raises(b.BridgeError):
        core.on_gate_command(text)
    assert core.sequence == 0 and len(read_lines(conn, 1)) == 1                     # nothing forwarded
    core.close(); conn.close(); server.close()


def test_enable_heartbeat_release_and_estop_outputs(short_tmp):
    clock = ClockNs()
    server, core, conn = connected(short_tmp, clock)
    assert core.switch_outputs(True) == []                     # no owner status yet: nothing at all
    deliver(core, conn, owner_status(dead_man=True, estop=False))
    assert core.switch_outputs(False) == [('enable', True), ('estop', False)]   # first known state, at once
    assert core.switch_outputs(False) == []                    # between heartbeats
    assert core.switch_outputs(True) == [('enable', True)]     # 10 Hz heartbeat
    deliver(core, conn, owner_status(dead_man=False))
    assert core.switch_outputs(False) == [('enable', False)]   # released: one false at once
    assert core.switch_outputs(True) == [] and core.switch_outputs(True) == []
    deliver(core, conn, owner_status(dead_man=True))
    assert core.switch_outputs(False) == [('enable', True)]
    deliver(core, conn, owner_status(estop=True))
    assert core.switch_outputs(False) == [('enable', False), ('estop', True)]
    assert core.switch_outputs(True) == [('estop', True)]      # repeated while pressed
    assert core.switch_outputs(False) == []
    deliver(core, conn, owner_status(estop=False, dead_man=False))
    assert core.switch_outputs(False) == [('estop', False)]    # released: one false, enable stays silent
    deliver(core, conn, owner_status(dead_man=True))
    clock.advance(0.31)                                        # owner status stale: unknown, publish nothing
    assert core.switch_outputs(True) == [] and core.enable() is False
    core.close(); conn.close(); server.close()


def test_missing_switch_input_publishes_nothing(short_tmp):
    clock = ClockNs()
    server, core, conn = connected(short_tmp, clock)
    deliver(core, conn, owner_status(dead_man=None, estop=None, auto_state='UNAVAILABLE', source='NONE'))
    assert core.switch_outputs(True) == [] and core.enable() is False
    core.close(); conn.close(); server.close()


def test_base_status_reasons_feedback_and_staleness(short_tmp):
    clock = ClockNs()
    server, core, conn = connected(short_tmp, clock)
    first = core.base_status()
    assert first['base_auto'] is False and first['reason'] == 'BASE_STATUS_STALE' and first['owner_status_stale']
    conn.sendall(p.encode_line(owner_status()) + b'{"type": "status"}\n')
    for _ in range(50):
        core.pump()
        if core.status is not None and core.counters['status_rejected']:
            break
    status = core.base_status()
    assert status['base_auto'] is True and status['reason'] is None and core.counters['status_rejected'] == 1
    assert status['feedback'] == owner_status()['feedback'] and status['stamp_mono_ns'] == clock()
    json.dumps(status, allow_nan=False)
    deliver(core, conn, owner_status(auto_state='NONE', source='MANUAL'))
    assert core.base_status()['base_auto'] is False and core.base_status()['reason'] == 'BASE_NOT_AUTO'
    clock.advance(0.31)
    stale = core.base_status()
    assert stale['reason'] == 'BASE_STATUS_STALE' and stale['feedback'] is None and stale['auto_state'] is None
    conn.close()
    with pytest.raises(b.BridgeError):
        for _ in range(50):
            core.pump()
    core.close(); server.close()


def test_environment_must_be_the_navigation_domain_with_todays_approval():
    today = datetime.date(2026, 11, 9)
    good = {'ROS_DOMAIN_ID': '100', 'ROS_LOCALHOST_ONLY': '1', 'WC_AUTO_DRIVE_APPROVAL': 'APPROVED_BY_USER_20261109'}
    b.check_environment(good, today)
    for bad in ({**good, 'ROS_DOMAIN_ID': '83'}, {**good, 'ROS_LOCALHOST_ONLY': '0'},
                {**good, 'ROS_DISCOVERY_SERVER': 'x'}, {**good, 'WC_AUTO_DRIVE_APPROVAL': 'APPROVED_BY_USER_20261108'},
                {k: v for k, v in good.items() if k != 'WC_AUTO_DRIVE_APPROVAL'}):
        with pytest.raises(SystemExit):
            b.check_environment(bad, today)


def test_bridge_into_the_owner_socket_end_to_end(short_tmp):
    """BridgeCore -> AutoSocket -> AutoDrive with the fake serial channel of the owner tests."""
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'operations'))
    import test_mapping_auto as owner_tests
    from wc_runtime import mapping_auto as a
    from wc_runtime import mapping_wheel as m
    env = owner_tests.make()
    env.auto.on_bridge_lost('RESET')
    sock = a.AutoSocket(short_tmp/'auto.sock', env.auto)
    clock = ClockNs()
    core = b.BridgeCore(env.owner.session_id, 'bridge_e2e', clock_ns=clock)
    core.connect(short_tmp/'auto.sock')
    for _ in range(5):
        sock.pump()
    assert env.auto.bridge_connected and env.auto.bridge_id == 'bridge_e2e'
    assert core.on_gate_command(gate_text(clock, linear=0.04))
    for _ in range(5):
        sock.pump()
    assert env.auto.decision['linear'] == 0.04 and env.auto.decision['gate_id'] == 'gate_1'
    env.clock.advance(0.2)
    for _ in range(5):
        sock.pump(); core.pump()
    assert core.status is not None and core.status['session_id'] == env.owner.session_id
    assert core.status['bridge_connected'] is True and core.status['gate_state'] == 'ACTIVE'
    assert core.enable() is True                              # fake switches: hold-to-run held, no estop
    relay = core.base_status()
    assert relay['base_auto'] is False and relay['reason'] == 'BASE_NOT_AUTO'   # owner is not in AUTO
    core.close()
    for _ in range(5):
        sock.pump()
    assert not env.auto.bridge_connected
    sock.close()
    assert not any(r[1] == 0x10 and r != m.ZERO_REQUEST for r in env.channel.requests)   # nothing ever drove
