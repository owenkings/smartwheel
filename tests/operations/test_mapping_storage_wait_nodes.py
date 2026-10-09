"""SYNTHETIC stop-time persistence plumbing; no hardware or real ROS nodes."""
import copy
import json
from pathlib import Path
import shutil
import sys
import threading
from types import SimpleNamespace as NS
import uuid

import pytest

from wc_runtime import mapping_cameras as cameras
from wc_runtime import mapping_monitor as monitor
from wc_runtime import mapping_prior as prior


NODE_ARGS = [
    (cameras, ['--config', 'session/cameras_config.json', '--output-root', 'session/cameras', '--duration', '.5'], 10.),
    (monitor, ['--session-root', 'session'], 25.),
    (prior, ['--config', 'session/prior.json', '--session-id', 'synthetic', '--output-root', 'session/prior'], None),
]


@pytest.fixture
def workspace():
    parent = Path(__file__).absolute().parent
    path = parent/('.mapping_storage_wait_'+uuid.uuid4().hex)
    path.mkdir()
    try:
        yield path
    finally:
        resolved = path.resolve()
        assert resolved.parent == parent and resolved.name.startswith('.mapping_storage_wait_')
        shutil.rmtree(resolved)


@pytest.mark.parametrize('module,argv,old_default', NODE_ARGS)
def test_cli_keeps_old_default_and_accepts_mapping_wait(module, argv, old_default):
    assert module._parse_args(argv).close_timeout_s == old_default
    assert module._parse_args(argv+['--close-timeout-s', '90']).close_timeout_s == 90.
    assert module._parse_args(argv+['--close-timeout-s', '.001']).close_timeout_s == .001


@pytest.mark.parametrize('module,argv,old_default', NODE_ARGS)
@pytest.mark.parametrize('value', ['0', '-1', '90.001', 'nan', 'inf', 'not-a-number'])
def test_invalid_wait_is_rejected_before_ros_or_device_import(module, argv, old_default, value):
    with pytest.raises(SystemExit) as raised:
        module._parse_args(argv+['--close-timeout-s='+value])
    assert raised.value.code == 2


def test_camera_instance_passes_wait_to_status_writer_after_device_cleanup(workspace):
    session = workspace/'session'; session.mkdir()
    events = []
    class Writer:
        error = None
        def submit(self, value): pass
        def close(self, value, timeout_s):
            events.append(('persist', timeout_s))
            assert value['persistence_close_timeout_s'] == 90.
            return {'final_fsync_complete': True, 'error': None}
    lease = NS(close=lambda: events.append(('lease_close',)))
    value = cameras.CameraCompanion(session/'cameras_config.json', session/'cameras', workspace,
        workspace/'run', 'synthetic', 1., close_timeout_s=90., writer=Writer(), lease=lease,
        owner=NS(pid=123, start_ticks='synthetic'))
    def cleanup(*args, **kwargs):
        events.append(('devices_stopped',))
        return {'status': 'PASS', 'remaining': []}
    result = value.close(cleanup=cleanup)
    assert result['storage_status'] == 'PASS'
    assert events[:2] == [('devices_stopped',), ('persist', 90.)]
    assert cameras.STOP_PHASES == ((cameras.signal.SIGINT, 7.5),
                                  (cameras.signal.SIGTERM, .75), (cameras.KILL_SIGNAL, .75))


def test_camera_longer_wait_still_latches_blocked_writer_failure(monkeypatch):
    entered, release = threading.Event(), threading.Event()
    def blocked(*args):
        entered.set()
        assert release.wait(3)
    writer = cameras.StatusWriter(None, None, write=blocked)
    original_join = writer.thread.join
    observed = []
    try:
        writer.submit({'status': 'synthetic'})
        assert entered.wait(1)
        monkeypatch.setattr(writer.thread, 'join', lambda timeout: observed.append(timeout))
        result = writer.close({'status': 'stopped'}, timeout_s=90.)
        assert observed == [90.]
        assert result['error'] == 'CAMERA_STATUS_CLOSE_TIMEOUT'
        assert result['final_fsync_complete'] is False
    finally:
        release.set()
        original_join(2)
        assert not writer.thread.is_alive()


def test_camera_main_transmits_cli_wait_without_extending_capture(workspace, monkeypatch):
    session = workspace/'session'; session.mkdir()
    (session/'runtime_config.json').write_text(json.dumps({'session_id': 'synthetic'}), encoding='utf-8')
    received = {}
    class Companion:
        children = {}
        def __init__(self, *args, **kwargs):
            received['duration'] = args[5]
            received['close_timeout_s'] = kwargs['close_timeout_s']
        def start(self): pass
        def close(self, **kwargs): return {'status': 'PASS'}
        def status(self): return {'state': 'STOPPED'}
    monkeypatch.setattr(cameras, 'CameraCompanion', Companion)
    monkeypatch.setattr(cameras, 'arm_owner', lambda *_: None)
    monkeypatch.setattr(cameras.signal, 'signal', lambda *_: None)
    moments = iter([0., 1.])
    monkeypatch.setattr(cameras, 'time', NS(monotonic=lambda: next(moments)))
    monkeypatch.setitem(sys.modules, 'wc_runtime.cli', NS(ROOT=workspace, RUN=workspace/'run', target=lambda: None))
    assert cameras.main(['--config', str(session/'cameras_config.json'), '--output-root', str(session/'cameras'),
                         '--duration', '.5', '--close-timeout-s', '90']) == 0
    assert received == {'duration': .5, 'close_timeout_s': 90.}


@pytest.mark.parametrize('override,expected,source', [(None, 17., 'config_policy'), (90., 90., 'command_line')])
def test_prior_override_preserves_config_and_all_runtime_quality_thresholds(override, expected, source):
    policy = dict(prior.DEFAULT_POLICY, writer_close_timeout_s=17.)
    config = {'policy': policy}
    original_bytes = json.dumps(config, sort_keys=True).encode()
    state = NS(config=config, policy=policy)
    before = copy.deepcopy(policy)
    record = prior._apply_persistence_wait(state, override)
    assert state.policy['writer_close_timeout_s'] == expected
    assert json.dumps(config, sort_keys=True).encode() == original_bytes
    assert {k: v for k, v in state.policy.items() if k != 'writer_close_timeout_s'} == {
        k: v for k, v in before.items() if k != 'writer_close_timeout_s'}
    assert record == {'persistence_close_timeout_s': expected, 'persistence_close_timeout_source': source}
    assert prior.DEFAULT_POLICY['writer_close_timeout_s'] == 20.


def test_prior_selected_wait_still_reports_incomplete_close():
    observed = []
    state = NS(condition=threading.Condition(), closing=False, shutdown_error=None,
               thread=NS(join=lambda value: observed.append(value), is_alive=lambda: True))
    with pytest.raises(prior.PriorError, match='final persistence is incomplete'):
        prior.AsyncPriorJournal.close(state, timeout_s=90.)
    assert observed == [90.] and state.shutdown_error


def test_monitor_selected_wait_preserves_failure_and_tracking_limit(workspace):
    observed = []
    class Archive:
        def __init__(self, root): pass
        def storage(self): return {}
        def snapshot(self, *args): pass
        def close(self, value, *, timeout_s):
            observed.append(timeout_s)
            assert value['persistence_close_timeout_s'] == timeout_s == 90.
            raise RuntimeError('synthetic final persistence incomplete')
    state = monitor.Health(workspace/'health', 'synthetic', 'right', archive_factory=Archive)
    with pytest.raises(RuntimeError, match='persistence incomplete'):
        state.close(timeout_s=90.)
    assert observed == [90.] and state.failure == 'synthetic final persistence incomplete'


@pytest.mark.parametrize('close_fails', [False, True])
@pytest.mark.parametrize('continuous_mapping', [False, True])
def test_monitor_main_passes_cli_wait_and_failed_close_returns_nonzero(workspace, monkeypatch, close_fails, continuous_mapping):
    session = workspace/'session'; session.mkdir()
    (session/'runtime_config.json').write_text(json.dumps({'session_id': 'synthetic', 'mode': 'right',
        'continuous_mapping': continuous_mapping}), encoding='utf-8')
    observed = []
    profiles = []
    class Health:
        failure = None
        def __init__(self, *args, odometry_source, continuous_mapping):
            self.odometry_source = odometry_source
            profiles.append(continuous_mapping)
        def fail(self, reason): self.failure = str(reason)
        def close(self, *, timeout_s):
            observed.append(timeout_s)
            if close_fails: raise RuntimeError('synthetic close timeout')
        def receive_info(self, message): pass
        def receive_cloud(self, message): pass
        def receive_grid(self, message): pass
    node = NS(create_publisher=lambda *a: object(), create_subscription=lambda *a: object(),
              destroy_node=lambda: None, get_logger=lambda: NS(error=lambda message: None))
    monkeypatch.setattr(monitor, 'Health', Health)
    monkeypatch.setattr(monitor.signal, 'signal', lambda *_: None)
    monkeypatch.setitem(sys.modules, 'wc_runtime.cli', NS(ROOT=workspace, target=lambda: None))
    modules = {
        'rclpy': NS(init=lambda **kw: None, create_node=lambda *a: node, ok=lambda: False),
        'rclpy.signals': NS(SignalHandlerOptions=NS(NO=0)),
        'rclpy.qos': NS(QoSProfile=lambda **kw: object(), ReliabilityPolicy=NS(RELIABLE=1),
                        DurabilityPolicy=NS(TRANSIENT_LOCAL=1)),
        'nav_msgs.msg': NS(Odometry=object, OccupancyGrid=object),
        'sensor_msgs.msg': NS(PointCloud2=object), 'rtabmap_msgs.msg': NS(OdomInfo=object),
        'visualization_msgs.msg': NS(Marker=object, MarkerArray=object),
    }
    for name, module in modules.items(): monkeypatch.setitem(sys.modules, name, module)
    code = monitor.main(['--session-root', str(session), '--close-timeout-s', '90'])
    assert profiles == [continuous_mapping]
    assert observed == [90.] and code == int(close_fails)
