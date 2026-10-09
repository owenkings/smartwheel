"""Actual node loops with synthetic ROS/serial/capture adapters; no devices."""
import json
from pathlib import Path
import shutil
import sys
from types import SimpleNamespace as NS
import uuid

import pytest

from wc_cameras import node as camera
from wc_imu import ros_node as imu
from wc_runtime import mapping_wheel as wheel

ROOT = Path(__file__).resolve().parents[2]


@pytest.fixture
def workspace():
    parent = Path(__file__).absolute().parent
    path = parent/('.until_close_nodes_' + uuid.uuid4().hex)
    path.mkdir()
    try:
        yield path
    finally:
        resolved = path.resolve()
        assert resolved.parent == parent and resolved.name.startswith('.until_close_nodes_')
        shutil.rmtree(resolved)


def mock_ros(monkeypatch):
    state = NS(nodes=[], publications=[], ok=True, spin=None, errors=[])
    class Node:
        def __init__(self, *args, **kwargs):
            self.destroyed = False
            state.nodes.append(self)
        def get_parameter(self, name): return NS(value=False)
        def create_timer(self, period, callback): return callback
        def create_publisher(self, kind, topic, qos):
            return NS(publish=lambda message: state.publications.append((topic, message)))
        def get_logger(self): return NS(error=state.errors.append, info=lambda message: None)
        def destroy_node(self): self.destroyed = True
    def spin(node, **kwargs): state.spin(node)
    modules = {
        'rclpy': NS(init=lambda **kwargs: None, ok=lambda: state.ok, spin_once=spin,
                    shutdown=lambda: setattr(state, 'ok', False)),
        'rclpy.node': NS(Node=Node),
        'rclpy.duration': NS(Duration=lambda **kwargs: NS(**kwargs)),
        'rclpy.signals': NS(SignalHandlerOptions=NS(NO=0)),
        'rclpy.qos': NS(qos_profile_sensor_data=object(), QoSProfile=lambda **kwargs: object(),
                        ReliabilityPolicy=NS(BEST_EFFORT=0), HistoryPolicy=NS(KEEP_LAST=0)),
        'sensor_msgs.msg': NS(Imu=object, Image=object),
        'diagnostic_msgs.msg': NS(DiagnosticArray=object, DiagnosticStatus=object, KeyValue=object),
        'wc_interfaces.msg': NS(H30Frame=object),
    }
    for name, module in modules.items(): monkeypatch.setitem(sys.modules, name, module)
    return state


@pytest.mark.parametrize('duration,failure,expected_count', [(0,False,2), (0,True,2), (60,False,2)])
def test_h30_actual_loop_until_stop_retains_stale_failure_and_serial_close(monkeypatch, workspace, duration, failure, expected_count):
    ros, clock, events = mock_ros(monkeypatch), [1_000_000_000], []
    class Lease:
        def __init__(self, *args): pass
        def open(self): events.append('open'); return self
        def close(self): events.append('close')
    monkeypatch.setattr(imu, 'ReadOnlySerialLease', Lease)
    monkeypatch.setattr(imu, 'H30Parser', lambda: NS(feed=lambda data: [object()], reset=lambda: events.append('reset')))
    monkeypatch.setattr(imu, 'time', NS(monotonic_ns=lambda: clock[0]))
    handlers = {}
    monkeypatch.setattr(imu.signal, 'getsignal', lambda number: None)
    monkeypatch.setattr(imu.signal, 'signal', lambda number, handler: handlers.__setitem__(number, handler))
    monkeypatch.setattr(imu, 'read_with_host_arrival', lambda serial: (
        b'' if failure and len(events) >= 4 else b'SYNTHETIC', clock[0], clock[0]))
    monkeypatch.setattr(imu, 'frame_record', lambda decoded, **kwargs: kwargs)
    monkeypatch.setattr(imu, 'assign_ros_frame', lambda record, message: NS(imu=object()))
    spins = [0]
    def spin(node):
        spins[0] += 1
        assert spins[0] <= 3, 'unbounded synthetic loop did not obey its stop/failure'
        node.diagnostics = lambda: None
        if spins[0] == 1: clock[0] += 100_000_000
        elif spins[0] == 2: clock[0] = 50_000_000_000_000
        else:
            if not failure:
                handlers[imu.signal.SIGINT](None, None)
                return
            clock[0] += 3_000_000_000
        events.append('spin')
        node.acquire()
    ros.spin = spin
    code = imu.main(['--device','/dev/SYNTHETIC_H30',
                     '--expected-by-id','/dev/serial/by-id/usb-test_SYNTHETIC-if00',
                     '--hardware-serial','SYNTHETIC','--sensor-id','H30-SYNTHETIC','--session-id','synthetic',
                     '--run-root',str(workspace),'--duration',str(duration)])
    assert code == (2 if failure else 0)
    assert len([p for p in ros.publications if p[0].endswith('/source_frame')]) == expected_count
    assert spins[0] == (3 if duration == 0 else 2)
    assert events[-2:] == ['reset','close'] and ros.nodes[0].destroyed
    if failure: assert 'no fresh checksum-valid' in ros.errors[-1]


@pytest.mark.parametrize('duration,failure,expected_count', [(0,False,2), (0,True,2), (60,False,1)])
def test_camera_actual_loop_has_no_hidden_duration_but_keeps_read_deadline(monkeypatch, workspace, duration, failure, expected_count):
    ros, clock, spins = mock_ros(monkeypatch), [1_000_000_000], [0]
    class Capture:
        def __init__(self, *args):
            self.process = NS(is_alive=lambda: True, exitcode=None)
            self.shared = NS(snapshot=self.snapshot)
            self.sequence = 0
        def start(self): pass
        def events(self): return []
        def snapshot(self, after):
            if failure and spins[0] == 3: return None
            self.sequence += 1
            return {'capture_sequence':self.sequence, 'host_monotonic_ns':clock[0],
                    'host_arrival_ns':clock[0], 'host_read_elapsed_ns':1}
    monkeypatch.setattr(camera, 'CaptureProcess', Capture)
    monkeypatch.setattr(camera, 'sys', NS(platform='linux'))
    monkeypatch.setattr(camera, 'time', NS(monotonic_ns=lambda: clock[0]))
    monkeypatch.setattr(camera, 'fill_image', lambda message, frame, frame_id: frame)
    monkeypatch.setattr(camera, 'finish_capture', lambda node: {'status':'PASS'})
    monkeypatch.setattr(camera, 'exit_if_capture_unreaped', lambda node: None)
    monkeypatch.setattr(camera.signal, 'signal', lambda *args: None)
    def spin(node):
        spins[0] += 1
        assert spins[0] <= 3
        node.diagnostic = lambda *args: None
        if spins[0] == 1:
            clock[0] += 100_000_000
            node.tracker.ready_ns = clock[0]
        elif spins[0] == 2: clock[0] = 50_000_000_000_000
        else:
            clock[0] += 4_000_000_000
            if not failure: node.stop_requested = True
        node.tick()
    ros.spin = spin
    code = camera.main(['--config',str(ROOT/'config/cameras.json'),'--role','right_front',
                        '--session-id','synthetic','--run-root',str(workspace),'--duration',str(duration)])
    assert code == int(failure) and ros.nodes[0].tracker.published_count == expected_count
    assert spins[0] == (3 if duration == 0 else 2) and ros.nodes[0].destroyed
    if failure: assert 'camera read deadline exceeded' in ros.errors[-1]


@pytest.mark.parametrize('node,args', [
    (camera, ['--config',str(ROOT/'config/cameras.json'),'--role','left_front','--session-id','synthetic','--run-root',str(ROOT)]),
    (imu, ['--session-id','synthetic','--run-root',str(ROOT),'--device','/SYNTHETIC',
           '--expected-by-id','/SYNTHETIC','--hardware-serial','SYNTHETIC']),
])
@pytest.mark.parametrize('duration', ['-1','nan','inf','43201'])
def test_invalid_sensor_durations_still_fail_before_ros(node, args, duration, monkeypatch):
    monkeypatch.setitem(sys.modules, 'rclpy', NS(init=lambda **kwargs: pytest.fail('invalid duration reached ROS')))
    with pytest.raises((camera.CameraError, imu.ImuAcquisitionError)):
        node.main(args+['--duration',duration])


@pytest.mark.parametrize('failure', [False, True])
def test_mapping_wheel_actual_loop_until_stop_retains_failure_and_closes_serial_first(monkeypatch, workspace, failure):
    from wc_motion import feedback_transport as transport
    config = json.loads((ROOT/'config/hardware_setup.json').read_text(encoding='utf-8'))
    hardware = json.loads((ROOT/'config/wheel_feedback_current.json').read_text(encoding='utf-8'))
    session = workspace/'session'; session.mkdir()
    (workspace/'hardware.json').write_text(json.dumps(hardware), encoding='utf-8')
    (session/'runtime_config.json').write_text(json.dumps({'session_id':'synthetic','duration_s':0,'source_mode':'real',
        'manual_controls':config['manual_controls'], 'wheel_device_id':hardware['device_id'],
        'wheel_hardware_config':'hardware.json'}), encoding='utf-8')
    calls, handlers, clock = [], {}, [0.]
    class Lease:
        host_configuration = {'fixture':'NO_DEVICE'}
        def __init__(self,*args): pass
        def open(self): calls.append('lease_open')
        def close(self): calls.append('lease_close')
    class Journal:
        def __call__(self,record): calls.append(record['event'])
        def close(self): calls.append('journal_close')
        def status(self): return {'fixture':True}
    class Owner:
        def __init__(self, config, channel, journal, **kwargs):
            # Permission does not arm the default read-only lifecycle below.
            assert config['manual_controls']['arm_allowed'] is True
            self.channel, self.failure, self.steps = channel, None, 0
        def step(self):
            self.steps += 1; calls.append('step')
            assert self.steps <= 3
            if self.steps == 3:
                if failure: self.failure = 'synthetic feedback failure'
                else: handlers[wheel.signal.SIGINT](None,None)
        def fault(self, reason): self.failure = self.failure or reason
        def close(self): calls.append('owner_close')
        def status(self): return {'control_transmissions':0}
    journal = Journal()
    monkeypatch.setitem(sys.modules,'wc_runtime.cli',NS(ROOT=workspace,RUN=workspace/'run',target=lambda:None))
    monkeypatch.setattr(transport,'FeedbackSerialLease',Lease)
    monkeypatch.setattr(transport,'create_journal',lambda *args,**kwargs:journal)
    monkeypatch.setattr(wheel,'MappingWheel',Owner)
    monkeypatch.setattr(wheel,'WheelChannel',lambda *args:NS(control_transmissions=0))
    monkeypatch.setattr(wheel,'ManualSocket',lambda *args:NS(pump=lambda:None,close=lambda:calls.append('socket_close')))
    monkeypatch.setattr(wheel,'RosOutput',lambda:NS(publish=lambda *args:None,String=lambda **kwargs:object(),
        manual=NS(publish=lambda *args:None),close=lambda:calls.append('ros_close')))
    monkeypatch.setattr(wheel,'time',NS(monotonic=lambda:clock[0],sleep=lambda seconds:clock.__setitem__(0,clock[0]+50000)))
    monkeypatch.setattr(wheel.signal,'SIGHUP',1,raising=False)
    monkeypatch.setattr(wheel.signal,'signal',lambda number,handler:handlers.__setitem__(number,handler))
    assert wheel.main(['--session-root',str(session)]) == int(failure)
    assert calls.count('step') == 3 and clock[0] > 43200
    assert calls.index('socket_close') < calls.index('owner_close') < calls.index('lease_close') < calls.index('journal_close')
    final = json.loads((session/'wheel_status.json').read_text())
    assert final['state'] == ('FAILED' if failure else 'STOPPED')
