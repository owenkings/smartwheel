"""Exercise real H30 node control flow with inert ROS and serial substitutes."""
import signal
import sys
from types import SimpleNamespace as NS

import pytest

from wc_imu import ros_node as imu


@pytest.mark.parametrize('stale_timeout,read_error', [(0, False), (2, False), (0, True)])
def test_silent_h30_can_wait_and_resume_but_device_errors_still_exit(monkeypatch, stale_timeout, read_error):
    clock, events, publications, errors, nodes = [1_000_000_000], [], [], [], []
    state = NS(ok=True, spins=0)

    class DiagnosticStatus(NS):
        ERROR, WARN = 2, 1

    class Node:
        def __init__(self, *args): nodes.append(self)
        def get_parameter(self, name): return NS(value=False)
        def create_timer(self, *args): pass
        def create_publisher(self, kind, topic, qos):
            return NS(publish=lambda message: publications.append((topic, message)))
        def get_clock(self): return NS(now=lambda: NS(to_msg=lambda: object()))
        def get_logger(self): return NS(error=errors.append)
        def destroy_node(self): events.append('destroy')

    class Lease:
        def __init__(self, *args): pass
        def open(self): events.append('open'); return self
        def close(self): events.append('close')

    def read(serial):
        if read_error and state.spins == 2:
            raise OSError('synthetic serial disconnect')
        return (b'SYNTHETIC' if state.spins in (1, 4) else b'', clock[0], clock[0])

    def spin(node, **kwargs):
        state.spins += 1
        if state.spins == 5:
            signal.getsignal(signal.SIGINT)(signal.SIGINT, None)
            return
        clock[0] += 100_000_000 if state.spins in (1, 4) else 50_000_000_000_000
        node.acquire()
        node.diagnostics()

    modules = {
        'rclpy': NS(init=lambda **kwargs: None, ok=lambda: state.ok, spin_once=spin,
                    shutdown=lambda: setattr(state, 'ok', False)),
        'rclpy.node': NS(Node=Node),
        'rclpy.duration': NS(Duration=NS),
        'rclpy.signals': NS(SignalHandlerOptions=NS(NO='NO')),
        'rclpy.qos': NS(qos_profile_sensor_data=object()),
        'sensor_msgs.msg': NS(Imu=object),
        'diagnostic_msgs.msg': NS(DiagnosticArray=lambda: NS(header=NS()),
                                  DiagnosticStatus=DiagnosticStatus, KeyValue=NS),
        'wc_interfaces.msg': NS(H30Frame=object),
    }
    for name, module in modules.items(): monkeypatch.setitem(sys.modules, name, module)
    monkeypatch.setattr(imu, 'ReadOnlySerialLease', Lease)
    monkeypatch.setattr(imu, 'H30Parser', lambda: NS(feed=lambda data: [object()], stats={}, reset=lambda: events.append('reset')))
    monkeypatch.setattr(imu, 'time', NS(monotonic_ns=lambda: clock[0]))
    monkeypatch.setattr(imu, 'read_with_host_arrival', read)
    monkeypatch.setattr(imu, 'frame_record', lambda decoded, **kwargs: {
        **kwargs, 'tid': 1, 'packet_sha256': 'synthetic', 'imu': {'field_validity': {}}})
    monkeypatch.setattr(imu, 'assign_ros_frame', lambda record, message: NS(imu=object(), source_stamp_ns=record['host_receive_ns']))
    code = imu.main(['--device', '/dev/SYNTHETIC', '--expected-by-id', '/dev/serial/by-id/SYNTHETIC',
                     '--hardware-serial', 'SYNTHETIC', '--sensor-id', 'H30-SYNTHETIC', '--session-id', 'synthetic',
                     '--run-root', '/SYNTHETIC', '--duration', '0', '--stale-timeout', str(stale_timeout)])
    assert events == ['open', 'reset', 'close', 'destroy']
    if read_error or stale_timeout:
        assert code == 2 and state.spins == 2
        assert ('disconnect' if read_error else 'no fresh checksum-valid') in errors[-1]
    else:
        assert code == 0 and state.spins == 5 and not errors
        frames = [message for topic, message in publications if topic.endswith('/source_frame')]
        assert len(frames) == 2 and frames[-1].source_stamp_ns == clock[0]
        diagnostics = [message.status[0] for topic, message in publications if topic.endswith('/diagnostics')]
        silence = {item.key: item.value for item in diagnostics[2].values}
        assert silence['silence_exit_enabled'] == 'false'
        assert float(silence['source_age_s']) > 43200
        assert silence['error'] == 'null'


@pytest.mark.parametrize('timeout', ['-1', 'nan', 'inf'])
def test_invalid_h30_silence_timeout_rejected_before_ros(monkeypatch, timeout):
    monkeypatch.setitem(sys.modules, 'rclpy', NS(init=lambda **kwargs: pytest.fail('invalid timeout reached ROS')))
    with pytest.raises(imu.ImuAcquisitionError, match='nonnegative'):
        imu.main(['--device', '/dev/SYNTHETIC', '--expected-by-id', '/dev/serial/by-id/SYNTHETIC',
                  '--hardware-serial', 'SYNTHETIC', '--sensor-id', 'H30-SYNTHETIC', '--session-id', 'synthetic', '--run-root', '/SYNTHETIC',
                  '--stale-timeout', timeout])
