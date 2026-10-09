"""Interactive lifecycle/protocol tests use no ROS, hardware or real processes."""
from concurrent.futures import Future
import copy
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time
from types import SimpleNamespace
import uuid

import pytest

from wc_runtime.mapping_app import parse_request, prepare_request
from wc_runtime.mapping_interactive import InteractiveCoordinator, OwnedSave
from wc_runtime.mapping_interactive_control import WindowControl, atomic_document, read_document


@pytest.fixture
def project():
    parent = Path(__file__).absolute().parent
    path = parent/('.interactive_'+uuid.uuid4().hex)
    path.mkdir()
    (path/'config').mkdir()
    (path/'config/storage.json').write_text('{"schema_version":1,"enabled":false}', encoding='utf-8')
    try:
        yield path
    finally:
        assert path.resolve().parent == parent and path.name.startswith('.interactive_')
        shutil.rmtree(path)


def control(project):
    return WindowControl(project, 'interactive_fixture', owner_ticks='123')


def command(owner, requested_action, **overrides):
    status = owner.status
    value = {key: owner.owner[key] for key in ('schema_version', 'window_id', 'owner_pid', 'owner_start_ticks')}
    value.update(generation=status['generation'], active_session_id=(status['active_session'] or {}).get('session_id'),
                 command_token=status['command_token'], sequence=status['last_sequence']+1,
                 nonce=uuid.uuid4().hex, action=requested_action)
    value.update(overrides)
    atomic_document(owner.directory/'command.json', value)
    return value


@pytest.mark.parametrize('field,value,reason', [
    ('window_id','foreign','OWNER_IDENTITY_MISMATCH'), ('owner_pid',999999,'OWNER_IDENTITY_MISMATCH'),
    ('owner_start_ticks','999','OWNER_IDENTITY_MISMATCH'), ('generation',999,'STALE_GENERATION'),
    ('active_session_id','other','ACTIVE_SESSION_MISMATCH'), ('sequence',0,'STALE_SEQUENCE'),
    ('sequence',True,'STALE_SEQUENCE'), ('nonce','bad','INVALID_NONCE'),
    ('command_token','old','STALE_COMMAND_TOKEN'), ('action','delete','ACTION_NOT_AVAILABLE'),
])
def test_rejects_foreign_stale_or_invalid_command(project, field, value, reason):
    owner = control(project)
    owner.publish(transition=True, state='PREVIEW', can_start=True)
    command(owner, 'start', **{field:value})
    assert owner.receive() is None
    assert owner.status['last_command']['reason'] == reason
    assert owner.status['last_sequence'] == 0


def test_consumed_command_is_not_replayed_and_nonce_cannot_be_reused(project):
    owner = control(project)
    owner.publish(transition=True, state='PREVIEW', can_start=True)
    first = command(owner, 'start')
    assert owner.receive() == 'start'
    assert owner.receive() is None
    command(owner, 'start', nonce=first['nonce'])
    assert owner.receive() is None
    assert owner.status['last_command']['reason'] == 'REUSED_NONCE'
    assert owner.status['last_sequence'] == 1


@pytest.mark.skipif(os.name == 'nt', reason='POSIX symlink metadata guard')
def test_command_symlink_is_rejected_without_touching_target(project):
    owner = control(project)
    outside = project/'kept.json'
    outside.write_bytes(b'unchanged')
    (owner.directory/'command.json').symlink_to(outside)
    with pytest.raises(ValueError, match='链接'):
        owner.receive()
    assert outside.read_bytes() == b'unchanged'


class ImmediateExecutor:
    def submit(self, function, *arguments):
        future = Future()
        try:
            future.set_result(function(*arguments))
        except BaseException as error:
            future.set_exception(error)
        return future


class Backend:
    def __init__(self):
        self.events, self.handles, self.requests = [], {}, []
        self.stop_error = False
    def preflight(self, request):
        self.events.append(('preflight', request['session_id']))
    def start(self, request):
        self.requests.append(copy.deepcopy(request))
        handle = dict(session_id=request['session_id'], directory=request['output_dir'],
                      runtime=str(Path(request['project_root'])/'.phase1_runtime/sessions'/request['session_id']/'mapping_app'),
                      mapping_enabled=request['mapping_enabled'], mode=request['mode'])
        self.handles[handle['session_id']] = 'RUNNING'
        self.events.append(('start', handle['session_id'], handle['mapping_enabled']))
        return handle
    def stop(self, handle):
        self.events.append(('stop', handle['session_id']))
        if self.stop_error:
            raise RuntimeError('fixture stop failed')
        self.handles[handle['session_id']] = 'STOPPED'
        return dict(state='STOPPED', exit_code=0, cleanup_errors=[])
    def inspect(self, handle):
        return dict(state=self.handles[handle['session_id']])
    def discard(self, handle):
        assert self.handles[handle['session_id']] == 'STOPPED'
        assert not handle['mapping_enabled'], 'New mapping data must not be silently discarded'
        self.events.append(('discard', handle['session_id']))
        return dict(status='SESSION_DATA_DISCARDED')
    def rviz_command(self, handle):
        return ['fake-rviz', handle['session_id']]
    def rviz_environment(self, handle):
        return {'DISPLAY': ':7', 'XAUTHORITY': '/desktop/auth'}


class Gui:
    exit_code = None
    def poll(self):
        return self.exit_code


class Saver:
    def __init__(self, control, handle, job, *, gui_environment):
        self.handle = handle
        self.cancelled = False
        self.remaining = 2
        self.gui_environment = gui_environment
    def cancel(self):
        self.cancelled = True
    def is_running(self):
        return False
    def poll(self):
        self.remaining -= 1
        if self.remaining >= 0 and not self.cancelled:
            return None
        return dict(status='SAVE_PENDING', session_id=self.handle['session_id'],
                    session_output=self.handle['directory'], message='原地图待处理')


def fixture(project, drive, *, backend=None, saver_type=Saver, spawn_error=False):
    owner, backend, gui = control(project), backend or Backend(), Gui()
    calls = []
    request = prepare_request('all', name='request', project_root=project, mapping_enabled=True)
    request.update(estimator='five_state', motion_correction=True, process_noise='white_acceleration', geometry=True,
                   panel_layout='unified')
    def spawn(argv, requested):
        assert requested['rviz_environment']['WC_MAPPING_CONTROL_DIR'] == str(owner.directory)
        calls.append(requested)
        if spawn_error:
            raise RuntimeError('fixture GUI spawn failed')
        return gui
    def save_factory(*args, **kwargs):
        active = owner.status['active_session']
        assert active['mapping_enabled'] is False
        assert backend.handles[active['session_id']] == 'RUNNING'
        assert backend.handles[args[1]['session_id']] == 'STOPPED'
        return saver_type(*args, **kwargs)
    count = [0]
    def wait(_seconds):
        count[0] += 1
        assert count[0] < 100, 'Lifecycle failed to terminate within bounded fixture steps'
        drive(owner, backend, gui, count[0])
    runner = InteractiveCoordinator(request, backend, owner, spawn=spawn,
        close_gui=lambda child: calls.append('closed'), save_factory=save_factory,
        executor=ImmediateExecutor(), wait=wait)
    return runner, owner, backend, gui, calls


def test_two_maps_have_new_identity_and_save_pending_preserves_each(project):
    starts = [0]
    def drive(owner, backend, gui, count):
        state = owner.status['state']
        if state == 'PREVIEW':
            if starts[0] == 2:
                gui.exit_code = 0
            else:
                command(owner, 'start'); starts[0] += 1
        elif state == 'MAPPING':
            command(owner, 'stop')
    runner, owner, backend, gui, calls = fixture(project, drive)
    result, code = runner.run()
    assert code == 3 and result['status'] == 'SAVE_PENDING'
    assert len(result['saves']) == 2
    assert len(result['pending_sessions']) == 2
    assert all(Path(row['directory']).name == row['session_id'] for row in result['pending_sessions'])
    assert all(row['status'] == 'SAVE_PENDING' for row in result['saves'])
    maps = [r for r in backend.requests if r['mapping_enabled']]
    assert len(maps) == 2 and maps[0]['output_dir'] != maps[1]['output_dir']
    assert maps[0]['session_id'] != maps[1]['session_id']
    assert all(r['motion_correction'] and r['geometry'] and r['process_noise']=='white_acceleration' for r in maps)
    assert all(not r['motion_correction'] and not r['geometry'] for r in backend.requests if not r['mapping_enabled'])
    assert all(state == 'STOPPED' for state in backend.handles.values())
    assert len([call for call in calls if isinstance(call, dict)]) == 1, 'The RViz window is persistent'
    assert owner.status['state'] == 'CLOSED'


def test_cancel_during_start_cleans_owned_session_and_returns_preview(project):
    sent = [False, False]
    def drive(owner, backend, gui, count):
        if owner.status['state'] == 'PREVIEW':
            if sent[0]: gui.exit_code = 0
            else: command(owner,'start'); sent[0]=True
        elif owner.status['state'] == 'STARTING' and not sent[1]:
            command(owner,'stop'); sent[1]=True
    runner, owner, backend, _, _ = fixture(project, drive)
    result, code = runner.run()
    assert code in (0, 3) and sent == [True, True]
    assert all(state == 'STOPPED' for state in backend.handles.values())
    assert not [event for event in backend.events if event[0] == 'discard' and '_m' in event[1]]


def test_close_during_save_cancels_worker_and_stops_preview(project):
    savers = []
    class LongSave(Saver):
        def __init__(self, *args, **kwargs):
            super().__init__(*args, **kwargs); self.remaining=1000; savers.append(self)
    def drive(owner, backend, gui, count):
        if owner.status['state'] == 'PREVIEW': command(owner,'start')
        elif owner.status['state'] == 'MAPPING': command(owner,'stop')
        elif owner.status['state'] == 'SAVING': gui.exit_code=0
    runner, owner, backend, _, _ = fixture(project, drive, saver_type=LongSave)
    result, code = runner.run()
    assert code == 3 and savers[0].cancelled
    assert savers[0].gui_environment == {'DISPLAY':':7','XAUTHORITY':'/desktop/auth'}
    assert result['saves'][0]['status'] == 'SAVE_PENDING'
    assert all(state == 'STOPPED' for state in backend.handles.values())


def test_stop_failure_has_bounded_attempts_and_retains_ownership_evidence(project):
    backend = Backend(); backend.stop_error=True
    runner, owner, backend, _, _ = fixture(project, lambda o,b,g,n:setattr(g,'exit_code',0), backend=backend)
    result, code = runner.run()
    assert code == 2 and result['status'] == 'FAILED'
    assert len([event for event in backend.events if event[0]=='stop']) == 2
    assert len(result['unconfirmed_owned_sessions']) == 1
    assert result['unconfirmed_owned_sessions'][0]['stop_confirmed'] is False
    assert not any(event[0]=='discard' for event in backend.events)


def test_gui_spawn_failure_stops_first_preview(project):
    runner, owner, backend, _, _ = fixture(project, lambda *args:None, spawn_error=True)
    result, code = runner.run()
    assert code == 2
    assert all(state=='STOPPED' for state in backend.handles.values())


def test_keyboard_interrupt_while_start_future_pending_does_not_double_stop(project):
    runner, owner, backend, _, _ = fixture(project, lambda *args:(_ for _ in ()).throw(KeyboardInterrupt()))
    result, code = runner.run()
    stops=[event for event in backend.events if event[0]=='stop']
    assert code == 0 and len(stops)==1
    assert all(state=='STOPPED' for state in backend.handles.values())


def test_cli_interactive_requires_mapping_but_legacy_request_is_unchanged(project):
    assert 'interactive' not in parse_request(['right','--mapping','true'],project_root=project)
    assert parse_request(['right','--mapping','true','--interactive'],project_root=project)['interactive'] is True
    with pytest.raises(SystemExit):
        parse_request(['right','--interactive'],project_root=project)


def test_distinct_long_window_names_keep_random_session_suffix(project):
    request = prepare_request('all', name='a'*60, project_root=project, mapping_enabled=True)
    identifiers = []
    for suffix in ('11111111','22222222'):
        owner = WindowControl(project, 'a'*42+'_w'+suffix, owner_ticks='123')
        runner = InteractiveCoordinator(request, Backend(), owner, executor=ImmediateExecutor())
        identifiers.append(runner.session_request(True)['session_id'])
    assert identifiers[0] != identifiers[1]
    assert all(len(identity) <= 64 for identity in identifiers)


def test_status_publications_never_change_binding_with_same_generation(project, monkeypatch):
    from wc_runtime import mapping_interactive_control as protocol
    original = protocol.atomic_document
    observed = []
    def recording(path, value):
        if Path(path).name == 'status.json': observed.append(copy.deepcopy(value))
        original(path,value)
    monkeypatch.setattr(protocol,'atomic_document',recording)
    starts = [False]
    def drive(owner, backend, gui, count):
        if owner.status['state'] == 'PREVIEW':
            if starts[0]: gui.exit_code=0
            else: command(owner,'start'); starts[0]=True
        elif owner.status['state'] == 'MAPPING': command(owner,'stop')
    runner, *_ = fixture(project,drive)
    result, code = runner.run()
    assert code == 3
    for before, after in zip(observed,observed[1:]):
        assert after['generation'] >= before['generation']
        if after['generation'] == before['generation']:
            assert all(after[key] == before[key] for key in ('active_session','view_generation','state'))


def test_malformed_completed_save_does_not_starve_preview_stop(project):
    class BrokenResult(Saver):
        def poll(self): raise ValueError('fixture malformed result')
    def drive(owner, backend, gui, count):
        if owner.status['state'] == 'PREVIEW': command(owner,'start')
        elif owner.status['state'] == 'MAPPING': command(owner,'stop')
    runner, owner, backend, _, _ = fixture(project,drive,saver_type=BrokenResult)
    result, code = runner.run()
    assert code == 2 and result['status']=='FAILED'
    assert len(result['pending_sessions']) == 1
    assert all(state=='STOPPED' for state in backend.handles.values())


def test_failed_save_is_not_reported_as_success_when_window_closes(project):
    class Failure(Saver):
        def poll(self):
            return dict(status='FAILED',session_id=self.handle['session_id'],
                        session_output=self.handle['directory'],errors=['fixture export failed'])
    started = [False]
    def drive(owner, backend, gui, count):
        if owner.status['state']=='PREVIEW':
            if started[0]: gui.exit_code=0
            else: command(owner,'start');started[0]=True
        elif owner.status['state']=='MAPPING': command(owner,'stop')
    runner, *_ = fixture(project,drive,saver_type=Failure)
    result, code=runner.run()
    assert code==2 and result['status']=='FAILED'
    assert result['pending_sessions'][0]['status']=='FAILED'


def test_interrupt_before_pending_start_result_is_consumed_stops_once(project):
    class DeferredFirst(ImmediateExecutor):
        pending = None
        def submit(self, function, *arguments):
            if self.pending is None:
                future=Future();self.pending=(future,function,arguments);return future
            return super().submit(function,*arguments)
    executor=DeferredFirst()
    runner, owner, backend, _, _ = fixture(project,lambda *args:None)
    runner.executor=executor
    def interrupt(_seconds):
        future,function,arguments=executor.pending
        future.set_result(function(*arguments))
        raise KeyboardInterrupt
    runner.wait=interrupt
    result, code=runner.run()
    assert code==0
    assert len([row for row in backend.events if row[0]=='stop'])==1
    assert all(state=='STOPPED' for state in backend.handles.values())


def test_stop_before_start_does_not_launch_preview(project):
    runner, owner, backend, _, calls=fixture(project,lambda *args:None)
    runner.stop_requested=lambda:True
    result, code=runner.run()
    assert code==130 and result['status']=='CANCELLED_BEFORE_START'
    assert not backend.requests and not calls


def test_stop_during_preflight_does_not_launch_preview(project):
    stopped=[False]
    backend=Backend()
    backend.preflight=lambda request:stopped.__setitem__(0,True)
    runner, owner, backend, _, calls=fixture(project,lambda *args:None,backend=backend)
    runner.stop_requested=lambda:stopped[0]
    result, code=runner.run()
    assert not backend.requests and calls==['closed']
    assert not result['errors']


@pytest.mark.parametrize('mode,interactive', [('preview',False),('mapping',True)])
def test_panel_adds_interactive_only_to_mapping_and_preserves_options(project,mode,interactive):
    from wc_panel.backend import PanelBackend
    (project/'config/mapping_live.json').write_text('{"wheel_imu_estimator":"five_state"}',encoding='utf-8')
    backend=PanelBackend.__new__(PanelBackend)
    backend.project_root=project
    backend._identity=lambda kind:'live_fixture'
    backend._ros_command=lambda argv:list(map(str,argv))
    backend.policy=SimpleNamespace(check=lambda:None)
    backend.device_parameters=lambda:{}
    backend.capabilities=lambda:{key:{'by_sides':{'all':{'enabled':True}}}
                                 for key in ('live_mapping','live_motion_correction','live_geometry')}
    submitted=[]
    backend._manager=SimpleNamespace(submit=lambda *args,**kwargs:submitted.append((args,kwargs)))
    backend._start_live(dict(mode=mode,sides='all',cloud='raw',estimator='five_state',
                            motion_correction=interactive,geometry=interactive,
                            process_noise='white_acceleration' if interactive else 'legacy'))
    command_line=submitted[0][0][2][0]['commands'][0]
    assert ('--interactive' in command_line) is interactive
    assert command_line[command_line.index('--cloud')+1]=='raw'
    assert command_line[command_line.index('--geometry')+1]==('on' if interactive else 'off')
    assert command_line[command_line.index('--process-noise')+1]==('white_acceleration' if interactive else 'legacy')


@pytest.mark.skipif(sys.platform != 'linux', reason='Linux descendant-owning process wrapper')
def test_owned_save_cancellation_reaps_synthetic_dialog_descendant(project):
    from wc_runtime.component import process
    owner=control(project)
    script=project/'fake_save.py'
    marker=project/'descendant.json'
    script.write_text('''import json,subprocess,sys,time
from pathlib import Path
from wc_runtime.component import process
child=subprocess.Popen([sys.executable,'-c','import time;time.sleep(300)'])
record=process(child.pid)
Path(sys.argv[1]).write_text(json.dumps({'pid':record.pid,'start_ticks':record.start_ticks}))
time.sleep(300)
''',encoding='utf-8')
    worker=OwnedSave.__new__(OwnedSave)
    worker.control=owner;worker.handle=dict(session_id='old_map',directory=str(project/'old_map'))
    worker.job='1';worker.cancelled=False;worker.result_path=owner.directory/'never_written.json'
    worker.process=subprocess.Popen([sys.executable,'-m','wc_runtime.component','--parent',str(os.getpid()),
        '--sigint-grace-s','.3','--',sys.executable,str(script),str(marker)],
        stdin=subprocess.DEVNULL,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL,start_new_session=True)
    try:
        deadline=time.monotonic()+10
        while not marker.exists() and time.monotonic()<deadline:
            assert worker.process.poll() is None
            time.sleep(.02)
        assert marker.exists()
        identity=json.loads(marker.read_text())
        worker.cancel()
        worker.process.wait(timeout=10)
        assert worker.poll()['status']=='SAVE_PENDING'
        observed=process(identity['pid'])
        assert observed is None or observed.start_ticks!=identity['start_ticks']
    finally:
        if worker.process.poll() is None:
            worker.cancel();worker.process.wait(timeout=10)
