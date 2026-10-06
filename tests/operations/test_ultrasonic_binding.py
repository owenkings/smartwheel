"""Topology identity and reliable publication finalization, no serial devices."""
import copy
import json
from pathlib import Path
import signal
import sys
from types import ModuleType, SimpleNamespace

import pytest

from wc_runtime import ultrasonic_capture as source


ROOT = Path(__file__).resolve().parents[2]


def configuration():
    return json.loads((ROOT/'config/ultrasonic_capture.json').read_text(encoding='utf-8'))


def test_current_topology_is_configuration_driven_but_other_identity_protocol_fields_remain_strict():
    config = configuration()
    assert config['expected_usb_path'] == 'platform-3610000.usb-usb-0:2.1:1.0'
    assert source.validate_config(config) == config
    config['expected_usb_path'] = 'platform-3610000.usb-usb-0:3.1:1.0'
    assert source.validate_config(config) == config
    for key, value in [('usb_pid', '55d3'), ('baud', 115200), ('device', '/dev/ttyUSB0'),
                       ('expected_by_id', '/dev/serial/by-id/OTHER'), ('register', 2)]:
        changed = copy.deepcopy(config); changed[key] = value
        with pytest.raises(ValueError): source.validate_config(changed)


@pytest.mark.parametrize('invalid', [None, '', '/dev/ttyUSB0', 'platform-3610000.usb-usb-0:*:1.0', True])
def test_explicit_usb_topology_must_be_well_formed(invalid):
    config = configuration(); config['expected_usb_path'] = invalid
    with pytest.raises(ValueError): source.validate_config(config)


def ros_stub(monkeypatch, *, acknowledged, events):
    handlers = {}
    monkeypatch.setattr(source.signal, 'signal', lambda sig, callback: handlers.__setitem__(sig, callback))
    class Publisher:
        def get_subscription_count(self): return 1
        def publish(self, value):
            events.append('publish'); handlers[signal.SIGINT]()
        def wait_for_all_acked(self, timeout):
            events.append(('ack', timeout.seconds)); return acknowledged
    publisher = Publisher()
    class Node:
        def create_publisher(self, *args): return publisher
        def destroy_node(self): events.append('node_destroy')
    ros = ModuleType('rclpy')
    ros.init = lambda **kwargs: None
    ros.create_node = lambda name: Node()
    ros.spin_once = lambda *args, **kwargs: None
    ros.ok = lambda: True
    ros.shutdown = lambda: events.append('ros_shutdown')
    signals = ModuleType('rclpy.signals'); signals.SignalHandlerOptions = SimpleNamespace(NO=0)
    duration = ModuleType('rclpy.duration'); duration.Duration = lambda **kwargs: SimpleNamespace(**kwargs)
    messages = ModuleType('std_msgs.msg'); messages.String = lambda **kwargs: SimpleNamespace(**kwargs)
    monkeypatch.setitem(sys.modules, 'rclpy', ros)
    monkeypatch.setitem(sys.modules, 'rclpy.signals', signals)
    monkeypatch.setitem(sys.modules, 'rclpy.duration', duration)
    monkeypatch.setitem(sys.modules, 'std_msgs', ModuleType('std_msgs'))
    monkeypatch.setitem(sys.modules, 'std_msgs.msg', messages)
    return publisher


@pytest.mark.parametrize('acknowledged, expected_code', [(True, 0), (False, 2)])
def test_source_ack_follows_lease_close_and_precedes_final_journal_sync(monkeypatch, tmp_path, acknowledged, expected_code):
    events, rows, close_errors = [], [], []
    ros_stub(monkeypatch, acknowledged=acknowledged, events=events)
    class Lease:
        def __init__(self, *args): pass
        def open(self): events.append('lease_open')
        def exchange(self, address): return {'address': address, 'status': 'VALID_RESPONSE'}
        def close(self): events.append('lease_close')
    class Journal:
        def __init__(self, *args): pass
        def append(self, row): rows.append(row)
        def close(self, source_error=None): events.append('journal_close'); close_errors.append(source_error)
    monkeypatch.setattr(source, 'DistanceLease', Lease)
    # Identity freezing is tested separately against actual udev/fstat
    # substitutes; this fixture isolates publication ACK/final close ordering.
    monkeypatch.setattr(source,'observed_identity',lambda config,lease,session:
        {'schema_version':1,'source_id':'ultrasonic','session_id':session,'synthetic_test_only':True})
    monkeypatch.setattr(source, 'SourceJournal', Journal)
    monkeypatch.setattr(source.time, 'sleep', lambda value: None)
    code = source.main(['--config', str(ROOT/'config/ultrasonic_capture.json'), '--output', str(tmp_path),
        '--run-root', str(tmp_path), '--session', 'synthetic', '--publish-ros', '--allow-read-queries'])
    assert code == expected_code
    assert events.index('lease_close') < events.index(('ack', 5.)) < events.index('journal_close') < events.index('node_destroy')
    assert rows[-1]['event'] == 'source_finalization'
    assert rows[-1]['completed_transactions'] == rows[-1]['published_transactions'] == 1
    assert rows[-1]['publication_ack']['status'] == ('PASS' if acknowledged else 'FAIL')
    assert bool(close_errors[0]) is (not acknowledged)
    if not acknowledged: assert 'ACK_TIMEOUT' in close_errors[0]
