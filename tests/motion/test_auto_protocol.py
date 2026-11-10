"""Messages of the automatic velocity channel: gate command, bridge hello/decision, owner status. No ROS, no device."""
import json

import pytest

from wc_motion import auto_protocol as p


def limits(**change):
    value = {'max_linear': 0.1, 'max_angular': 0.3, 'linear_ceiling': 0.1, 'allow_reverse': False}
    value.update(change)
    return value


def gate(**change):
    value = {'schema': p.GATE_COMMAND_SCHEMA, 'gate_id': 'gate_3f2a', 'seq': 3, 'stamp_ns': 1_700_000_000_000_000_000,
             'mono_ns': 5_000_000_000, 'state': 'ACTIVE', 'reasons': ['LINEAR_CLAMPED'], 'linear': 0.08,
             'angular': -0.1, 'limits': limits(), 'config_sha256': 'a'*64}
    value.update(change)
    return value


def test_valid_gate_command_round_trips_into_a_decision():
    parsed = p.parse_gate_command(json.dumps(gate()))
    assert parsed['linear'] == 0.08 and parsed['limits'] == limits() and parsed['gate_id'] == 'gate_3f2a'
    packet = p.decision_packet(parsed, 'session_a', 1, 6_000_000_000)
    line = p.encode_line(packet)
    assert line.endswith(b'\n') and b'NaN' not in line
    decision = p.parse_decision(p.decode_line(line[:-1]), 'session_a', 0)
    assert decision['sequence'] == 1 and decision['gate_seq'] == 3 and decision['state'] == 'ACTIVE'
    assert decision['gate_id'] == 'gate_3f2a' and decision['limits']['linear_ceiling'] == 0.1


def test_lateral_slow_down_ceiling_is_kept():
    parsed = p.parse_gate_command(json.dumps(gate(limits=limits(linear_ceiling=0.05))))
    assert parsed['limits'] == limits(linear_ceiling=0.05)


@pytest.mark.parametrize('text', [
    json.dumps(gate(schema='other')), json.dumps({**gate(), 'extra': 1}),
    json.dumps({k: v for k, v in gate().items() if k != 'limits'}),
    json.dumps({k: v for k, v in gate().items() if k != 'gate_id'}),
    json.dumps(gate(gate_id='')), json.dumps(gate(gate_id='bad id')), json.dumps(gate(gate_id=7)),
    json.dumps(gate(seq=-1)), json.dumps(gate(seq=True)), json.dumps(gate(mono_ns=0)),
    json.dumps(gate(linear=True)), json.dumps(gate(linear='0.1')), json.dumps(gate(angular=11.0)),
    json.dumps(gate(state='active')), json.dumps(gate(state='X'*60)), json.dumps(gate(reasons='LINEAR_CLAMPED')),
    json.dumps(gate(reasons=['']*1)), json.dumps(gate(reasons=['R']*17)),
    json.dumps(gate(limits={'max_linear': 0.1, 'max_angular': 0.3})),
    json.dumps(gate(limits=limits(max_linear=-0.1))), json.dumps(gate(limits=limits(linear_ceiling=0.2))),
    json.dumps(gate(limits=limits(allow_reverse=0))), json.dumps(gate(limits=limits(allow_reverse='false'))),
    json.dumps(gate(limits={**limits(), 'slow_linear': 0.05})),
    json.dumps(gate(config_sha256='A'*64)), json.dumps(gate(config_sha256='a'*63)),
    json.dumps(gate()).replace('0.08', 'NaN'), json.dumps(gate()).replace('0.08', 'Infinity'),
    '{"schema": "wc_gate_command_v1", "schema": "wc_gate_command_v1"}', '[]', 'not json',
    json.dumps({**gate(), 'reasons': ['R'*80]*16}) + ' '*4096])
def test_malformed_gate_commands_are_rejected(text):
    with pytest.raises(p.ProtocolError):
        p.parse_gate_command(text)


def test_unknown_gate_state_is_kept_and_null_hash_allowed():
    parsed = p.parse_gate_command(json.dumps(gate(state='BASE_NOT_READY', config_sha256=None, linear=0.0, angular=0.0)))
    assert parsed['state'] == 'BASE_NOT_READY' and parsed['config_sha256'] is None


def test_decision_requires_session_and_increasing_sequence():
    packet = p.decision_packet(p.parse_gate_command(json.dumps(gate())), 'session_a', 5, 1)
    with pytest.raises(p.ProtocolError):
        p.parse_decision(packet, 'session_b', 0)
    with pytest.raises(p.ProtocolError):
        p.parse_decision(packet, 'session_a', 5)
    with pytest.raises(p.ProtocolError):
        p.parse_decision({**packet, 'extra': 1}, 'session_a', 0)
    with pytest.raises(p.ProtocolError):
        p.parse_decision({**packet, 'type': 'status'}, 'session_a', 0)
    with pytest.raises(p.ProtocolError):
        p.parse_decision({k: v for k, v in packet.items() if k != 'gate_id'}, 'session_a', 0)
    with pytest.raises(p.ProtocolError):
        p.parse_decision({**packet, 'limits': {'max_linear': 0.1, 'max_angular': 0.3}}, 'session_a', 0)
    assert p.parse_decision(packet, 'session_a', 4)['sequence'] == 5


def test_hello_and_status_checks():
    hello = p.hello_packet('session_a', 'bridge_1')
    assert p.check_hello(hello, 'session_a') == 'bridge_1'
    for bad in ({**hello, 'session_id': 'session_b'}, {**hello, 'schema': 'x'}, {**hello, 'extra': 1}):
        with pytest.raises(p.ProtocolError):
            p.check_hello(bad, 'session_a')
    with pytest.raises(p.ProtocolError):
        p.hello_packet('bad id', 'bridge_1')
    status = {'type': 'status', 'schema': p.STATUS_SCHEMA, 'session_id': 'session_a', 'source': 'AUTO',
              'auto_state': 'AUTO', 'dead_man': True, 'estop': False, 'enable_source': 'HOLD_TO_RUN_SWITCH',
              'enable_request': True, 'estop_request': False}
    assert p.parse_status(status, 'session_a') is status
    not_installed = {**status, 'dead_man': None, 'estop': None, 'enable_source': 'OPERATOR_START'}
    assert p.parse_status(not_installed, 'session_a') is not_installed
    for bad in ({**status, 'dead_man': 'yes'}, {**status, 'source': 'NAV'}, {**status, 'session_id': 'other'},
                {k: v for k, v in status.items() if k != 'auto_state'}, {**status, 'enable_source': 'PANEL'},
                {k: v for k, v in status.items() if k != 'enable_source'}, {**status, 'enable_request': 1},
                {k: v for k, v in status.items() if k != 'enable_request'}, {**status, 'estop_request': None},
                {k: v for k, v in status.items() if k != 'estop_request'}):
        with pytest.raises(p.ProtocolError):
            p.parse_status(bad, 'session_a')


def test_line_decoding_bounds():
    with pytest.raises(p.ProtocolError):
        p.decode_line(b'{"a": 1}' + b' '*p.MAX_LINE_BYTES)
    with pytest.raises(p.ProtocolError):
        p.decode_line(b'\xff\xfe')
    assert p.decode_line(b'{"a": 1}') == {'a': 1}
