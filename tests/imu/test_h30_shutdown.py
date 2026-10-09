"""No hardware/ROS: shutdown must finish the current batch before draining DDS."""
import importlib.util
from pathlib import Path
import signal
import sys
from types import ModuleType, SimpleNamespace

import pytest


@pytest.fixture
def shutdown_runtime(monkeypatch):
    source = Path(__file__).resolve().parents[2]/'src'/'wc_imu'/'ros_node.py'
    spec = importlib.util.spec_from_file_location('h30_shutdown_under_test', source)
    runtime = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(runtime)
    state = SimpleNamespace(events=[], handlers={}, originals={}, records=[], frames=[],
                            ok=True, ack=True, ack_error=None, stop_at='frame_journal',
                            stop_signal=signal.SIGINT, stop_in_init=False, init_error=False,
                            journal_error=None)
    for name in ('SIGINT', 'SIGTERM', 'SIGHUP'):
        signum = getattr(signal, name, None)
        if signum is not None:
            state.originals[signum] = object()
    state.handlers.update(state.originals)
    monkeypatch.setattr(runtime.signal, 'getsignal', lambda number: state.handlers[number])
    monkeypatch.setattr(runtime.signal, 'signal', lambda number, handler: state.handlers.__setitem__(number, handler))

    def stop():
        state.handlers[state.stop_signal](state.stop_signal, None)

    class Publisher:
        def __init__(self, topic):
            self.topic = topic
        def get_subscription_count(self):
            return 1
        def publish(self, message):
            if self.topic.endswith('source_frame'):
                state.frames.append(message.sequence)
                state.events.append(('publish', message.sequence))
                if len(state.frames) == 1 and state.stop_at == 'frame_publish':
                    stop()
        def wait_for_all_acked(self, duration):
            assert state.ok, 'DDS context must remain alive until acknowledgment'
            assert 'serial_close' in state.events
            assert duration.seconds == 3.0
            state.events.append('ack')
            if state.ack_error:
                raise state.ack_error
            return state.ack

    class Node:
        def __init__(self, name):
            state.node = self
        def get_parameter(self, name):
            return SimpleNamespace(value=False)
        def create_publisher(self, message, topic, qos):
            return Publisher(topic)
        def create_timer(self, period, callback):
            return None
        def get_logger(self):
            return SimpleNamespace(error=lambda text: state.events.append(('error', text)))
        def destroy_node(self):
            state.events.append('destroy')

    class Serial:
        def __init__(self, *args):
            pass
        def open(self):
            state.events.append('serial_open')
            return self
        def close(self):
            state.events.append('serial_close')
        def read_available(self):
            return b'unchanged-raw-batch'

    class Parser:
        stats = {}
        def feed(self, data):
            assert data == b'unchanged-raw-batch'
            return (1, 2)
        def reset(self):
            state.events.append('parser_reset')

    class Journal:
        def __init__(self, *args):
            pass
        def append(self, record):
            state.records.append(record)
            if record['event'] == 'byte_batch' and state.stop_at == 'byte_journal':
                stop()
            if record['event'] == 'frame':
                state.events.append(('journal', record['sequence']))
                if record['sequence'] == 1 and state.stop_at == 'frame_journal':
                    stop()
        def close(self, *, source_error):
            state.source_error = source_error
            state.events.append('journal_close')
            if state.journal_error:
                raise state.journal_error

    def install_module(name, **attributes):
        module = ModuleType(name)
        module.__dict__.update(attributes)
        monkeypatch.setitem(sys.modules, name, module)
        return module

    def init(**kwargs):
        assert kwargs['signal_handler_options'] == 'NO'
        state.events.append('init')
        if state.init_error:
            raise RuntimeError('mock init failed')
        if state.stop_in_init:
            stop()

    def shutdown():
        state.events.append('shutdown')
        state.ok = False

    install_module('rclpy', init=init, ok=lambda: state.ok,
                   spin_once=lambda node, **kwargs: node.acquire(), shutdown=shutdown)
    install_module('rclpy.node', Node=Node)
    install_module('rclpy.duration', Duration=lambda **kwargs: SimpleNamespace(**kwargs))
    install_module('rclpy.signals', SignalHandlerOptions=SimpleNamespace(NO='NO'))
    install_module('rclpy.qos', qos_profile_sensor_data=object(),
                   QoSProfile=lambda **kwargs: SimpleNamespace(**kwargs),
                   ReliabilityPolicy=SimpleNamespace(RELIABLE='RELIABLE'),
                   DurabilityPolicy=SimpleNamespace(VOLATILE='VOLATILE'))
    install_module('sensor_msgs.msg', Imu=SimpleNamespace)
    install_module('diagnostic_msgs.msg', DiagnosticArray=SimpleNamespace,
                   DiagnosticStatus=SimpleNamespace, KeyValue=SimpleNamespace)
    install_module('wc_interfaces.msg', H30Frame=SimpleNamespace)
    install_module('wc_runtime.source_archive', SourceJournal=Journal)
    monkeypatch.setattr(runtime, 'ReadOnlySerialLease', Serial)
    monkeypatch.setattr(runtime, 'H30Parser', Parser)
    monkeypatch.setattr(runtime, 'frame_record', lambda decoded, **kwargs: dict(kwargs, packet_sha256=str(decoded)))
    monkeypatch.setattr(runtime, 'assign_ros_frame', lambda record, output:
                        SimpleNamespace(sequence=record['frame_sequence'], imu=object()))
    state.runtime = runtime
    state.argv = ['--device', '/dev/SYNTHETIC', '--expected-by-id', '/dev/serial/by-id/mock',
                  '--hardware-serial', 'mock', '--sensor-id', 'H30-mock', '--session-id', 'synthetic',
                  '--run-root', '/synthetic', '--journal-dir', '/synthetic/journal',
                  '--require-recorder', '--duration', '0', '--stale-timeout', '0']
    return state


@pytest.mark.parametrize('stop_at', ['byte_journal', 'frame_journal', 'frame_publish'])
@pytest.mark.parametrize('signal_name', ['SIGINT', 'SIGTERM', 'SIGHUP'])
def test_signal_finishes_current_batch_then_closes_and_acks(shutdown_runtime, stop_at, signal_name):
    state = shutdown_runtime
    if not hasattr(signal, signal_name):
        pytest.skip('signal absent on this platform')
    state.stop_at, state.stop_signal = stop_at, getattr(signal, signal_name)
    assert state.runtime.main(state.argv) == 0
    assert state.frames == [1, 2]
    assert [record['sequence'] for record in state.records if record['event'] == 'frame'] == [1, 2]
    assert state.source_error is None
    assert state.events.index(('publish', 2)) < state.events.index('serial_close')
    assert [event for event in state.events if isinstance(event, str)][-6:] == [
        'parser_reset', 'serial_close', 'ack', 'journal_close', 'destroy', 'shutdown']
    assert state.handlers == state.originals


@pytest.mark.parametrize('ack_error', [None, RuntimeError('mock DDS wait failed')])
def test_ack_failure_marks_source_summary_and_nonzero_exit(shutdown_runtime, ack_error):
    state = shutdown_runtime
    state.ack, state.ack_error = False, ack_error
    assert state.runtime.main(state.argv) == 2
    assert state.frames == [1, 2]
    assert 'IMU_RELIABLE_ACK_FAILED' in state.source_error
    if ack_error is None:
        assert 'IMU_RELIABLE_ACK_TIMEOUT' in state.source_error
    assert state.events.index('ack') < state.events.index('journal_close') < state.events.index('destroy')
    assert state.handlers == state.originals


def test_preview_has_no_reliable_ack_requirement(shutdown_runtime):
    state = shutdown_runtime
    state.argv.remove('--require-recorder')
    assert state.runtime.main(state.argv) == 0
    assert 'ack' not in state.events
    assert state.handlers == state.originals


def test_journal_close_failure_still_destroys_context_and_restores_handlers(shutdown_runtime):
    state = shutdown_runtime
    state.journal_error = RuntimeError('mock disk failure')
    with pytest.raises(RuntimeError, match='mock disk failure'):
        state.runtime.main(state.argv)
    assert state.events[-2:] == ['destroy', 'shutdown']
    assert state.handlers == state.originals


def test_init_failure_restores_handlers(shutdown_runtime):
    state = shutdown_runtime
    state.init_error = True
    with pytest.raises(RuntimeError, match='mock init failed'):
        state.runtime.main(state.argv)
    assert state.handlers == state.originals


def test_stop_before_acquisition_closes_journal_without_opening_serial(shutdown_runtime):
    state = shutdown_runtime
    state.stop_in_init = True
    with pytest.raises(state.runtime.ImuAcquisitionError, match='stopped before serial'):
        state.runtime.main(state.argv)
    assert 'serial_open' not in state.events
    assert state.source_error == 'stopped before serial acquisition'
    assert state.events[-3:] == ['journal_close', 'destroy', 'shutdown']
    assert state.handlers == state.originals
