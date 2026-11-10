"""Base bridge without ROS: socket forwarding, faults, enable/estop outputs, base status, main() wiring, end to end."""
import datetime
import json
from pathlib import Path
import socket
import sys
import tempfile
from types import ModuleType, SimpleNamespace as NS

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
             'auto_state': 'AUTO', 'dead_man': True, 'estop': False, 'enable_source': 'HOLD_TO_RUN_SWITCH',
             'enable_request': True, 'estop_request': False, 'estop_reasons': [],
             'feedback': {'sequence': 41, 'stamp_mono_ns': 4_990_000_000, 'age_s': 0.01, 'linear_m_s': 0.05}}
    value.update(change)
    return value


def operator_status(**change):
    return owner_status(dead_man=None, estop=None, enable_source='OPERATOR_START', **change)


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


def test_without_the_switch_the_enable_follows_the_operator_start(short_tmp):
    clock = ClockNs()
    server, core, conn = connected(short_tmp, clock)
    deliver(core, conn, operator_status(enable_request=False, auto_state='NONE', source='NONE'))
    assert core.switch_outputs(False) == [('enable', False), ('estop', False)]   # known: not enabled, no estop
    assert core.switch_outputs(True) == []
    deliver(core, conn, operator_status(enable_request=True, auto_state='INITIALIZING'))
    assert core.switch_outputs(False) == [('enable', True)]    # the new start: false -> true, the gate's unlatch edge
    assert core.switch_outputs(True) == [('enable', True)]
    deliver(core, conn, operator_status(enable_request=False, auto_state='STOPPING'))
    assert core.switch_outputs(False) == [('enable', False)]   # stop auto, Space, WASD: one false at once
    assert core.switch_outputs(True) == []
    relay = core.base_status()
    assert relay['enable_source'] == 'OPERATOR_START' and relay['enable_request'] is False
    assert relay['dead_man'] is None and relay['estop'] is None and relay['estop_request'] is False
    clock.advance(0.31)
    assert core.switch_outputs(True) == [] and core.base_status()['enable_source'] is None
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


def test_dry_run_needs_no_approval_and_forwards_only_to_the_dry_run_owner(short_tmp):
    today = datetime.date(2026, 11, 10)
    b.check_environment({'ROS_DOMAIN_ID': '100', 'ROS_LOCALHOST_ONLY': '1'}, today, dry_run=True)
    with pytest.raises(SystemExit):
        b.check_environment({'ROS_DOMAIN_ID': '83', 'ROS_LOCALHOST_ONLY': '1'}, today, dry_run=True)
    clock = ClockNs()
    server = served(short_tmp/'auto.sock')
    core = b.BridgeCore('session_a', 'bridge_1', clock_ns=clock, require_dry_run=True)
    core.connect(short_tmp/'auto.sock')
    conn, _ = server.accept()
    assert read_lines(conn, 1)[0]['type'] == 'hello'
    assert core.on_gate_command(gate_text(clock)) is False and core.counters['gate_held'] == 1
    deliver(core, conn, operator_status(dry_run=True))
    assert core.dry_run_confirmed and core.on_gate_command(gate_text(clock, seq=2)) is True
    assert read_lines(conn, 1)[0]['gate_seq'] == 2
    core.close(); conn.close()
    other = b.BridgeCore('session_a', 'bridge_2', clock_ns=clock, require_dry_run=True)
    other.connect(short_tmp/'auto.sock')
    conn, _ = server.accept()
    conn.sendall(p.encode_line(operator_status()))   # a vehicle owner: dry_run absent
    with pytest.raises(b.BridgeError, match='not the dry run'):
        for _ in range(50):
            other.pump()
    other.close(); conn.close(); server.close()


def fake_ros(monkeypatch, hooks):
    """Minimal rclpy / std_msgs stand-ins: timers fire on a fake 10 ms spin, ``hooks`` run at given spin counts."""
    class Msg:
        def __init__(self, data=None): self.data = data

    class Publisher:
        def __init__(self, topic, qos): self.topic, self.qos, self.sent = topic, qos, []
        def publish(self, message): self.sent.append(message.data)

    class Node:
        def __init__(self, name):
            self.name, self.publishers, self.subscriptions, self.timers, self.destroyed = name, {}, {}, [], False
        def create_publisher(self, kind, topic, qos):
            self.publishers[topic] = Publisher(topic, qos)
            return self.publishers[topic]
        def create_subscription(self, kind, topic, callback, qos):
            self.subscriptions[topic] = (callback, qos)
        def create_timer(self, period, callback):
            self.timers.append([period, callback, 0.0])
        def destroy_node(self):
            self.destroyed = True

    state = NS(ok=True, time=0.0, spins=0, node=None, inbox=[])

    def create_node(name):
        state.node = Node(name)
        return state.node

    def spin_once(node, timeout_sec=None):
        state.spins += 1
        state.time += 0.01
        if state.spins in hooks:
            hooks[state.spins](state)
        for timer in node.timers:
            if state.time - timer[2] >= timer[0] - 1e-9:
                timer[2] = state.time
                timer[1]()
        while state.inbox:
            topic, data = state.inbox.pop(0)
            node.subscriptions[topic][0](Msg(data))
        if state.spins > 1000:
            state.ok = False

    rclpy = ModuleType('rclpy')
    rclpy.init = lambda args=None: None
    rclpy.ok = lambda: state.ok
    rclpy.shutdown = lambda: setattr(state, 'ok', False)
    rclpy.create_node = create_node
    rclpy.spin_once = spin_once
    qos = ModuleType('rclpy.qos')
    qos.QoSProfile = lambda **kwargs: NS(**kwargs)
    qos.ReliabilityPolicy = NS(RELIABLE='RELIABLE')
    qos.DurabilityPolicy = NS(VOLATILE='VOLATILE', TRANSIENT_LOCAL='TRANSIENT_LOCAL')
    msg = ModuleType('std_msgs.msg')
    msg.Bool = msg.String = Msg
    std_msgs = ModuleType('std_msgs')
    std_msgs.msg = msg
    for name, module in (('rclpy', rclpy), ('rclpy.qos', qos), ('std_msgs', std_msgs), ('std_msgs.msg', msg)):
        monkeypatch.setitem(sys.modules, name, module)
    return state


def test_bridge_main_wires_qos_timers_switches_and_stops_on_a_malformed_gate_command(short_tmp, monkeypatch, capsys):
    from wc_runtime import mapping_auto as a
    path = short_tmp/'auto.sock'
    server = served(path)
    today = datetime.date.today().strftime('%Y%m%d')
    for name, value in (('ROS_DOMAIN_ID', '100'), ('ROS_LOCALHOST_ONLY', '1'),
                        ('WC_AUTO_DRIVE_APPROVAL', 'APPROVED_BY_USER_' + today)):
        monkeypatch.setenv(name, value)
    monkeypatch.delenv('ROS_DISCOVERY_SERVER', raising=False)
    monkeypatch.setattr(a, 'auto_socket_path', lambda project, root, session: path)
    owner_side = {}

    def accept(state):
        conn, _ = server.accept()
        owner_side['conn'] = conn
        conn.sendall(p.encode_line(operator_status(session_id='session_main', enable_request=True)))

    def malformed(state):
        state.inbox.append(('/wc_nav/gate_command', '{"schema": "wc_gate_command_v1"}'))
    state = fake_ros(monkeypatch, {3: accept, 40: malformed})
    result = b.main(['--project-root', str(short_tmp), '--session-root', str(short_tmp), '--session-id', 'session_main',
                     '--bridge-id', 'bridge_main'])
    node = state.node
    assert result == 1 and node.destroyed and state.spins < 100
    assert json.loads(capsys.readouterr().out.strip().splitlines()[-1])['failure'].startswith('gate command rejected')
    assert sorted(period for period, _, _ in node.timers) == [0.02, b.SWITCH_PERIOD_S]
    callback, subscribed = node.subscriptions['/wc_nav/gate_command']
    assert (subscribed.depth, subscribed.reliability, subscribed.durability) == (1, 'RELIABLE', 'VOLATILE')
    estop = node.publishers['/wc_nav/estop'].qos
    assert (estop.depth, estop.reliability, estop.durability) == (1, 'RELIABLE', 'TRANSIENT_LOCAL')
    assert node.publishers['/wc_nav/enable'].sent[:1] == [True]
    assert node.publishers['/wc_nav/estop'].sent[:1] == [False]          # current state at once, no switch installed
    statuses = [json.loads(text) for text in node.publishers['/wc_nav/base_status'].sent]
    assert statuses and statuses[-1]['enable_source'] == 'OPERATOR_START' and statuses[-1]['bridge_id'] == 'bridge_main'
    hello = read_lines(owner_side['conn'], 1)[0]
    assert p.check_hello(hello, 'session_main') == 'bridge_main'
    owner_side['conn'].close(); server.close()


def test_bridge_into_the_owner_socket_end_to_end(short_tmp):
    """BridgeCore -> AutoSocket -> AutoDrive with the fake serial channel of the owner tests, on one clock."""
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'operations'))
    import test_mapping_auto as owner_tests
    from wc_runtime import mapping_auto as a
    from wc_runtime import mapping_wheel as m
    env = owner_tests.make()
    env.auto.on_bridge_lost('RESET')
    sock = a.AutoSocket(short_tmp/'auto.sock', env.auto)
    core = b.BridgeCore(env.owner.session_id, 'bridge_e2e', clock_ns=env.clock.ns)
    core.connect(short_tmp/'auto.sock')
    for _ in range(5):
        sock.pump()
    assert env.auto.bridge_connected and env.auto.bridge_id == 'bridge_e2e'
    clock = env.clock.ns
    assert core.on_gate_command(gate_text(clock, linear=0.04))
    for _ in range(5):
        sock.pump()
    assert env.auto.decision['linear'] == 0.04 and env.auto.decision['gate_id'] == 'gate_1'
    assert env.auto._decision_age(env.clock()) == 0.0            # same clock end to end
    env.clock.advance(0.2)
    for _ in range(5):
        sock.pump(); core.pump()
    assert core.status is not None and core.status['session_id'] == env.owner.session_id
    assert core.status['bridge_connected'] is True and core.status['gate_state'] == 'ACTIVE'
    assert core.enable() is True                              # fake switches: hold-to-run held, no estop
    relay = core.base_status()
    assert relay['base_auto'] is False and relay['reason'] == 'BASE_NOT_AUTO'   # owner is not in AUTO
    env.clock.advance(0.2)
    assert env.auto._decision_age(env.clock()) > p.DECISION_STALE_S   # 0.4 s after the gate made it
    core.close()
    for _ in range(5):
        sock.pump()
    assert not env.auto.bridge_connected
    sock.close()
    assert not any(r[1] == 0x10 and r != m.ZERO_REQUEST for r in env.channel.requests)   # nothing ever drove


def test_without_the_switch_the_panel_estop_and_the_inferred_stop_reach_the_gate(short_tmp):
    clock = ClockNs()
    server, core, conn = connected(short_tmp, clock)
    deliver(core, conn, operator_status(enable_request=True))
    assert core.switch_outputs(False) == [('enable', True), ('estop', False)]   # first known state at once
    deliver(core, conn, operator_status(enable_request=False, estop_request=True,
                                        estop_reasons=['WHEELS_STOPPED_WHILE_COMMANDED'], auto_state='STOPPING'))
    assert core.switch_outputs(False) == [('enable', False), ('estop', True)]  # the gate latches
    assert core.switch_outputs(True) == [('estop', True)]                     # 10 Hz while requested
    assert core.base_status()['estop_reasons'] == ['WHEELS_STOPPED_WHILE_COMMANDED']
    deliver(core, conn, operator_status(enable_request=False, auto_state='NONE', source='NONE'))
    assert core.switch_outputs(False) == [('estop', False)]                   # released: one false
    deliver(core, conn, operator_status(enable_request=False, estop_request=True, estop_reasons=['PANEL_SOFTWARE_ESTOP']))
    assert core.switch_outputs(False) == [('estop', True)]
    deliver(core, conn, operator_status(enable_request=True))                 # released, then a new start
    assert core.switch_outputs(False) == [('enable', True), ('estop', False)]
    core.close(); conn.close(); server.close()


def test_an_estop_request_never_enables_the_gate(short_tmp):
    clock = ClockNs()
    server, core, conn = connected(short_tmp, clock)
    deliver(core, conn, operator_status(enable_request=True, estop_request=True, estop_reasons=['PANEL_SOFTWARE_ESTOP']))
    assert core.enable() is False and core.switch_outputs(False) == [('enable', False), ('estop', True)]
    deliver(core, conn, owner_status(estop_request=True, estop_reasons=['PANEL_SOFTWARE_ESTOP']))   # switch installed
    assert core.enable() is False and core.switch_outputs(True) == [('estop', True)]
    core.close(); conn.close(); server.close()
