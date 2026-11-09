"""Base bridge without ROS: socket forwarding, staleness, status and the end-to-end path into a fake wheel owner."""
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
    value = {'schema': p.GATE_COMMAND_SCHEMA, 'seq': 1, 'stamp_ns': 1, 'mono_ns': clock(), 'state': 'ACTIVE',
             'reasons': [], 'linear': 0.05, 'angular': 0.0, 'limits': {'max_linear': 0.1, 'max_angular': 0.3},
             'config_sha256': None}
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


def test_bridge_sends_hello_then_one_decision_per_fresh_gate_command(short_tmp):
    server = served(short_tmp/'auto.sock')
    clock = ClockNs()
    core = b.BridgeCore('session_a', 'bridge_1', clock_ns=clock)
    core.connect(short_tmp/'auto.sock')
    conn, _ = server.accept()
    assert core.on_gate_command(gate_text(clock)) is True
    assert core.on_gate_command(gate_text(clock, seq=2, linear=0.07)) is True
    hello, first, second = read_lines(conn, 3)
    assert p.check_hello(hello, 'session_a') == 'bridge_1'
    assert [first['sequence'], second['sequence']] == [1, 2] and second['linear'] == 0.07
    assert p.parse_decision(second, 'session_a', 1)['gate_seq'] == 2
    core.close(); conn.close(); server.close()


def test_stale_future_or_malformed_gate_commands_are_dropped(short_tmp):
    server = served(short_tmp/'auto.sock')
    clock = ClockNs()
    core = b.BridgeCore('session_a', 'bridge_1', clock_ns=clock)
    core.connect(short_tmp/'auto.sock')
    conn, _ = server.accept()
    old = gate_text(clock)
    clock.advance(0.31)
    assert core.on_gate_command(old) is False                                       # older than 0.3 s
    assert core.on_gate_command(gate_text(clock, mono_ns=clock()+100_000_000)) is False   # from the future
    assert core.on_gate_command('{"schema": "wc_gate_command_v1"}') is False
    assert core.on_gate_command(gate_text(clock, linear=float('nan'))) is False          # json.dumps writes NaN
    assert core.counters['gate_stale'] == 2 and core.counters['gate_rejected'] == 2 and core.sequence == 0
    assert len(read_lines(conn, 1)) == 1                                            # only the hello
    core.close(); conn.close(); server.close()


def test_status_enable_and_estop_follow_a_fresh_owner_status(short_tmp):
    server = served(short_tmp/'auto.sock')
    clock = ClockNs()
    core = b.BridgeCore('session_a', 'bridge_1', clock_ns=clock)
    core.connect(short_tmp/'auto.sock')
    conn, _ = server.accept()
    status = {'type': 'status', 'schema': p.STATUS_SCHEMA, 'session_id': 'session_a', 'source': 'AUTO',
              'dead_man': True, 'estop': False}
    conn.sendall(p.encode_line(status) + b'{"type": "status"}\n')
    for _ in range(50):
        core.pump()
        if core.status is not None:
            break
    assert core.enable() is True and core.estop() is False and core.counters['status_rejected'] == 1
    assert core.base_status()['owner']['source'] == 'AUTO' and core.base_status()['reason'] is None
    clock.advance(0.31)
    assert core.enable() is False and core.estop() is None and core.base_status()['owner_status_stale']
    assert core.base_status()['reason'] == 'BASE_STATUS_STALE'
    conn.sendall(p.encode_line({**status, 'dead_man': False}))
    for _ in range(50):
        core.pump()
        if core.status['dead_man'] is False:
            break
    assert core.enable() is False
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
    assert env.auto.decision['linear'] == 0.04 and env.auto.decision['sequence'] == 1
    env.clock.advance(0.2)
    for _ in range(5):
        sock.pump(); core.pump()
    assert core.status is not None and core.status['session_id'] == env.owner.session_id
    assert core.status['bridge_connected'] is True and core.status['gate_state'] == 'ACTIVE'
    assert core.enable() is True                              # fake switches: hold-to-run held, no estop
    core.close()
    for _ in range(5):
        sock.pump()
    assert not env.auto.bridge_connected
    sock.close()
    assert not any(r[1] == 0x10 and r != m.ZERO_REQUEST for r in env.channel.requests)   # nothing ever drove
