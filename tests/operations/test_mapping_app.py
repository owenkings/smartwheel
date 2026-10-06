"""Foreground lifecycle/path tests with fake owned processes; no ROS or devices."""
from datetime import datetime, timezone
from pathlib import Path
import shutil
from types import SimpleNamespace
import uuid

import pytest

from wc_runtime import mapping_app as app


@pytest.fixture
def tmp_path():
    # Normal-permission directories avoid mode-0700 ACL failures on this Windows workspace.
    parent = Path(__file__).absolute().parent
    path = parent / ('.mapping_app_test_' + uuid.uuid4().hex)
    path.mkdir()
    try:
        yield path
    finally:
        resolved = path.resolve()
        assert resolved.parent == parent and resolved.name.startswith('.mapping_app_test_')
        shutil.rmtree(resolved)


def request(tmp_path, mode='right', *, mapping_enabled=True):
    return app.prepare_request(mode, name='mapping_test_01', project_root=tmp_path, mapping_enabled=mapping_enabled)


class Backend:
    def __init__(self, *, preflight_error=None, start_error=None, states=None, stop_state=None, stop_error=None, export_error=None):
        self.events = []
        self.preflight_error, self.start_error = preflight_error, start_error
        self.states = list(states or [{'state': 'RUNNING'}])
        self.stop_state = stop_state or {'state': 'STOPPED', 'exit_code': 0, 'cleanup_errors': []}
        self.stop_error, self.export_error = stop_error, export_error

    def preflight(self, value):
        self.events.append(('preflight', value['mode']))
        if self.preflight_error:
            raise self.preflight_error

    def start(self, value):
        self.events.append(('start', value['mode']))
        if self.start_error:
            raise self.start_error
        return {'session_id': value['session_id']}

    def rviz_command(self, handle):
        self.events.append(('rviz_command', handle['session_id']))
        return ['rviz2', '-d', 'fixture.rviz']

    def rviz_environment(self, handle):
        return {'DISPLAY': ':7', 'XAUTHORITY': '/verified/authority'}

    def inspect(self, handle):
        self.events.append(('inspect', handle['session_id']))
        if len(self.states) > 1:
            return self.states.pop(0)
        return self.states[0]

    def stop(self, handle):
        self.events.append(('stop', handle['session_id']))
        if self.stop_error:
            raise self.stop_error
        return self.stop_state

    def export(self, handle):
        self.events.append(('export', handle['session_id']))
        if self.export_error:
            raise self.export_error
        return {'status': 'EXPORTED_EXPERIMENTAL_MAP', 'files': {'map.ply': {}}}

    def save(self, handle, destination):
        self.events.append(('save', handle['session_id'], destination))
        exported = self.export(handle)
        return {'status': 'SAVED_EXPERIMENTAL_MAP', 'output': destination, 'export': exported}

    def discard(self, handle):
        self.events.append(('discard', handle['session_id']))
        return {'status': 'SESSION_DATA_DISCARDED'}


def execute(value, backend, *, gui_code=0, stop_requested=lambda: False, wait=lambda seconds: None,
            spawn_error=None, close_error=None, emit=lambda value: None, clock=app.time.monotonic,
            choose_save=None):
    def spawn(command, gui_request):
        backend.events.append(('spawn', command, gui_request))
        if spawn_error:
            raise spawn_error
        return SimpleNamespace(poll=lambda: gui_code)
    def close(gui):
        backend.events.append(('close_gui', gui))
        if close_error:
            raise close_error
    def affirmative_choice(request, handle):
        backend.events.append(('choose_save', handle['session_id']))
        return {'decision': 'save', 'destination': str(Path(request['project_root'])/'saved'/'chosen_map')}
    return app.run_session(value, backend, spawn=spawn, close_gui=close, wait=wait,
                           stop_requested=stop_requested, emit=emit, clock=clock,
                           choose_save=affirmative_choice if choose_save is None else choose_save)


def names(backend):
    return [event[0] for event in backend.events]


def test_check_config_cli_never_starts_the_mapping_lifecycle(tmp_path,capsys):
    backend=Backend()
    def check(value):
        assert value['mode']=='all'
        return {'status':'CONFIG_VALID_EXPERIMENT','hardware_started':False}
    backend.check_configuration=check
    assert app.main(['all','--check-config'],project_root=tmp_path,backend=backend)==0
    assert backend.events==[]
    assert 'CONFIG_VALID_EXPERIMENT' in capsys.readouterr().out
    assert not (tmp_path/'reports').exists()


@pytest.mark.parametrize('mode', app.MODES)
def test_default_request_is_new_project_scoped_and_does_not_create_files(tmp_path, mode):
    value = app.prepare_request(mode, name='mapping_test_01', project_root=tmp_path)
    assert value['output_dir'] == str(tmp_path/'reports/maps/mapping_test_01')
    assert value['config_path'] == str(tmp_path/'config/mapping_live.json')
    assert value['mode'] == mode and value['owner_pid'] > 0
    assert value['duration_s'] == 0
    assert value['mapping_enabled'] is False
    assert list(tmp_path.iterdir()) == []


def test_generated_names_are_unique_and_custom_output_is_exact(tmp_path):
    now = datetime(2026, 9, 14, tzinfo=timezone.utc)
    first = app.prepare_request('left', project_root=tmp_path, now=now)
    second = app.prepare_request('left', project_root=tmp_path, now=now)
    assert first['session_id'] != second['session_id']
    custom = app.prepare_request('right', output='reports/custom/session 1', name='exact', project_root=tmp_path)
    assert custom['output_dir'] == str(tmp_path/'reports/custom/session 1')
    assert custom['session_id'] == 'exact'


@pytest.mark.parametrize('duration', [0, 1, 59, 60, 300, 2400, 2401, 86400])
def test_explicit_duration_is_passed_to_backend_request(tmp_path, duration):
    value = app.parse_request(['right', '--duration', str(duration)], project_root=tmp_path)
    assert value['duration_s'] == 0
    assert value['requested_duration_s'] == duration


@pytest.mark.parametrize('duration', [-1, True, False, 120.5])
def test_duration_rejects_invalid_finite_values_and_boolean_zero(tmp_path, duration):
    with pytest.raises(ValueError, match='非负整数'):
        app.prepare_request('right', project_root=tmp_path, duration_s=duration)


def test_until_close_still_exports_after_window_close_and_reports_no_preset_limit(tmp_path):
    value = app.prepare_request('right', project_root=tmp_path, duration_s=0, mapping_enabled=True)
    backend, events = Backend(), []
    result, code = execute(value, backend, emit=events.append)
    assert code == 0 and result['stop_reason'] == 'RVIZ_WINDOW_CLOSED'
    assert names(backend).count('start') == names(backend).count('stop') == names(backend).count('export') == 1
    started = next(row for row in events if row['status'] == 'MAPPING_STARTED_NOT_YET_VERIFIED')
    assert started['duration_s'] == 0 and '无总时长和归档帧数上限' in started['message']
    assert '无需启动静止等待' in started['message']
    assert '数据间断时等待恢复' in started['message']


@pytest.mark.parametrize('cause', ['component_failure', 'storage_limit'])
def test_until_close_keeps_runtime_failure_and_low_storage_stop(tmp_path, cause):
    value = app.prepare_request('right', project_root=tmp_path, duration_s=0, mapping_enabled=True)
    backend = Backend(states=[{'state':'FAILED','exit_code':1}] if cause == 'component_failure' else None)
    if cause == 'storage_limit':
        backend.storage_status = lambda handle: {'free_bytes':2*1024**3, 'stop_free_bytes':2*1024**3}
    result, code = execute(value, backend, gui_code=None)
    assert names(backend).count('stop') == 1
    if cause == 'component_failure':
        assert code != 0 and result['status'] == 'FAILED' and 'export' not in names(backend)
    else:
        assert code == 0 and result['stop_reason'] == 'STORAGE_LIMIT_REACHED'


@pytest.mark.parametrize('output', ['../outside', 'reports/../map', '.git/map', '.phase1_runtime/map', '.codex/map'])
def test_unsafe_output_is_rejected(tmp_path, output):
    with pytest.raises(ValueError):
        app.prepare_request('right', output=output, name='safe', project_root=tmp_path)


def test_existing_output_and_file_parent_are_not_overwritten(tmp_path):
    existing = tmp_path/'existing'; existing.mkdir()
    original = existing/'original.db'; original.write_bytes(b'untouched original')
    with pytest.raises(ValueError, match='已存在'):
        app.prepare_request('right', existing, 'safe', project_root=tmp_path)
    with pytest.raises(ValueError, match='父级'):
        app.prepare_request('right', original/'child', 'safe', project_root=tmp_path)
    assert original.read_bytes() == b'untouched original'


def test_outside_root_output_and_config_rejected(tmp_path):
    with pytest.raises(ValueError, match='工程目录内'):
        app.prepare_request('right', tmp_path.parent/'outside_new_map', 'safe', project_root=tmp_path)
    with pytest.raises(ValueError, match='配置'):
        app.prepare_request('right', name='safe', project_root=tmp_path, config=tmp_path.parent/'mapping.json')


def test_linked_parent_rejected_without_resolving_away_link(tmp_path, monkeypatch):
    original = Path.is_symlink
    linked = tmp_path/'redirect'
    monkeypatch.setattr(Path, 'is_symlink', lambda self: self == linked or original(self))
    with pytest.raises(ValueError, match='符号链接'):
        app.prepare_request('right', linked/'map', 'safe', project_root=tmp_path)


@pytest.mark.parametrize('argv,mode', [(['left'], 'left'), (['--mode', 'all'], 'all'),
                                     (['right', '--mode', 'right'], 'right')])
def test_cli_supports_flag_and_positional_mode(tmp_path, argv, mode):
    assert app.parse_request(argv, project_root=tmp_path)['mode'] == mode


@pytest.mark.parametrize('argv', [[], ['left', '--mode', 'right'], ['--mode', 'dual'], ['right', '--name', '../bad']])
def test_ambiguous_or_invalid_cli_does_not_start(tmp_path, argv):
    with pytest.raises(SystemExit) as raised:
        app.parse_request(argv, project_root=tmp_path)
    assert raised.value.code == 2
    assert list(tmp_path.iterdir()) == []


def test_rviz_close_stops_before_export_and_passes_verified_gui_environment(tmp_path):
    backend = Backend()
    result, code = execute(request(tmp_path), backend)
    assert code == 0 and result['status'] == 'SAVED'
    assert result['stop_reason'] == 'RVIZ_WINDOW_CLOSED'
    assert names(backend).index('close_gui') < names(backend).index('stop') < names(backend).index('export')
    assert names(backend).index('stop') < names(backend).index('choose_save') < names(backend).index('save')
    spawn = next(event for event in backend.events if event[0] == 'spawn')
    assert spawn[2]['rviz_environment']['DISPLAY'] == ':7'


def test_start_notice_explains_direct_wasd_and_hand_push_without_claiming_no_control(tmp_path):
    notices = []
    class HybridBackend(Backend):
        def start(self,value):
            return {**super().start(value),'manual_controls':{'arm_allowed':True,'interaction_policy':'hybrid_manual'}}
    result, code = execute(request(tmp_path), HybridBackend(), emit=notices.append)
    assert code == 0 and result['status'] == 'SAVED'
    started = next(row for row in notices if row['status'] == 'MAPPING_STARTED_NOT_YET_VERIFIED')
    assert '直接手推或按住 WASD' in started['message']
    assert '确认停稳后恢复手推' in started['message']
    assert '0.75' not in started['message']
    assert '此命令不控制车辆' not in started['message']


@pytest.mark.parametrize('controls',[None,{}, {'arm_allowed':False,'interaction_policy':'hybrid_manual'}])
def test_notice_does_not_claim_motion_authorization_for_readonly_handle(controls):
    assert '未授权轮控制' in app.manual_control_notice({'manual_controls':controls})


def test_ctrl_c_during_monitoring_stops_and_exports(tmp_path):
    backend = Backend()
    def interrupt(_seconds):
        raise KeyboardInterrupt
    result, code = execute(request(tmp_path), backend, gui_code=None, wait=interrupt)
    assert code == 0 and result['stop_reason'] == 'USER_INTERRUPT'
    assert names(backend)[-4:] == ['stop', 'choose_save', 'save', 'export']


@pytest.mark.parametrize('remaining', [2 * 1024**3, 2 * 1024**3 - 1])
def test_low_space_normally_stops_then_exports_without_claiming_full_duration(tmp_path, remaining):
    backend = Backend(); announcements = []
    backend.storage_status = lambda handle: {'free_bytes': remaining, 'stop_free_bytes': 2 * 1024**3}
    result, code = execute(request(tmp_path), backend, gui_code=None, emit=announcements.append)
    assert code == 0 and result['status'] == 'SAVED'
    assert result['stop_reason'] == 'STORAGE_LIMIT_REACHED'
    assert result['storage']['free_bytes'] == remaining
    assert '提前结束' in result['message']
    assert names(backend)[-5:] == ['close_gui', 'stop', 'choose_save', 'save', 'export']
    notice = next(row for row in announcements if row['status'] == 'STORAGE_LIMIT_REACHED')
    assert '不保证' in notice['message']


def test_storage_checks_are_once_per_second_while_gui_and_supervisor_still_poll(tmp_path):
    backend = Backend(); instant = [0.0]; checked = []; waits = []
    def disk(handle):
        checked.append(instant[0])
        return {'free_bytes': 3 * 1024**3 if len(checked) == 1 else 2 * 1024**3,
                'stop_free_bytes': 2 * 1024**3}
    def wait(seconds):
        waits.append(seconds); instant[0] = len(waits) * .2
        assert len(waits) <= 5
    backend.storage_status = disk
    result, code = execute(request(tmp_path), backend, gui_code=None, wait=wait, clock=lambda: instant[0])
    assert code == 0 and result['stop_reason'] == 'STORAGE_LIMIT_REACHED'
    assert checked == [0., 1.] and waits == [.2] * 5
    assert names(backend).count('inspect') == 6


@pytest.mark.parametrize('invalid', [None, {'free_bytes': -1, 'stop_free_bytes': 2 * 1024**3},
                                   {'free_bytes': True, 'stop_free_bytes': 2 * 1024**3}, 'raise'])
def test_failed_storage_read_stops_but_never_exports_or_claims_saved(tmp_path, invalid):
    backend = Backend()
    def disk(handle):
        if invalid == 'raise': raise OSError('synthetic statvfs failure')
        return invalid
    backend.storage_status = disk
    result, code = execute(request(tmp_path), backend, gui_code=None)
    assert code == 2 and result['status'] == 'FAILED'
    assert result['stop_reason'] == 'STORAGE_CHECK_FAILED'
    assert 'STORAGE_CHECK_FAILED' in result['errors'][0]
    assert names(backend)[-2:] == ['close_gui', 'stop'] and 'export' not in names(backend)


def test_low_space_does_not_override_abnormal_cleanup(tmp_path):
    backend = Backend(stop_state={'state': 'FAILED', 'exit_code': 1})
    backend.storage_status = lambda handle: {'free_bytes': 2 * 1024**3, 'stop_free_bytes': 2 * 1024**3}
    result, code = execute(request(tmp_path), backend, gui_code=None)
    assert code == 2 and result['stop_reason'] == 'STORAGE_LIMIT_REACHED'
    assert 'export' not in names(backend) and result['status'] == 'FAILED'


def test_preflight_capacity_estimate_is_announced_before_start_without_enforcing_prediction(tmp_path):
    backend = Backend(); events = []
    original = backend.preflight
    def preflight(value):
        original(value)
        return {'storage': {'warning': True, 'estimate_only': True, 'message': '估算不能保证运行时长'}}
    backend.preflight = preflight
    def emit(value):
        events.append(value)
        if value['status'] == 'STORAGE_CAPACITY_ESTIMATE': assert names(backend) == ['preflight']
    result, code = execute(request(tmp_path), backend, emit=emit)
    assert code == 0 and result['status'] == 'SAVED'
    assert events[0]['status'] == 'STORAGE_CAPACITY_ESTIMATE' and events[0]['warning']


def test_signal_request_after_start_skips_gui_but_stops_and_exports(tmp_path):
    backend = Backend()
    result, code = execute(request(tmp_path), backend, stop_requested=lambda: 'start' in names(backend))
    assert code == 0 and result['stop_reason'] == 'USER_STOP_REQUEST'
    assert 'spawn' not in names(backend)
    assert names(backend)[-4:] == ['stop', 'choose_save', 'save', 'export']


def test_cancel_before_start_has_no_device_or_gui_action(tmp_path):
    backend = Backend()
    result, code = execute(request(tmp_path), backend, stop_requested=lambda: True)
    assert code == 130 and result['hardware_started'] is False
    assert names(backend) == ['preflight']


def test_missing_all_calibration_fails_before_start_without_fallback(tmp_path):
    backend = Backend(preflight_error=RuntimeError('all requires verified T_left_right'))
    result, code = execute(request(tmp_path, 'all'), backend)
    assert code == 2 and result['mode'] == 'all'
    assert names(backend) == ['preflight']
    assert 'verified T_left_right' in result['errors'][0]


def test_failed_start_does_not_signal_a_guessed_or_existing_session(tmp_path):
    backend = Backend(start_error=RuntimeError('collision; no owner acquired'))
    result, code = execute(request(tmp_path), backend)
    assert code == 2
    assert names(backend) == ['preflight', 'start']


@pytest.mark.parametrize('kwargs', [dict(gui_code=1), dict(spawn_error=OSError('no display')),
                                  dict(close_error=RuntimeError('GUI owner timeout'))])
def test_gui_failure_still_stops_backend_but_never_claims_export(tmp_path, kwargs):
    backend = Backend()
    result, code = execute(request(tmp_path), backend, **kwargs)
    assert code == 2 and result['status'] == 'FAILED'
    assert 'stop' in names(backend) and 'export' not in names(backend)


@pytest.mark.parametrize('state', [{'state': 'FAILED', 'exit_code': 1}, {'state': 'MISSING'},
                                 {'state': 'STOPPED', 'exit_code': 0, 'cleanup_errors': ['timeout']}])
def test_pipeline_failure_is_preserved_even_if_subsequent_stop_is_normal(tmp_path, state):
    backend = Backend(states=[state])
    result, code = execute(request(tmp_path), backend)
    assert code == 2 and 'stop' in names(backend) and 'export' not in names(backend)


def test_supervisor_normal_deadline_closes_gui_then_exports(tmp_path):
    backend = Backend(states=[{'state': 'STOPPED', 'exit_code': 0}])
    result, code = execute(request(tmp_path), backend, gui_code=None)
    assert code == 0 and result['stop_reason'] == 'SESSION_NORMAL_STOP'
    assert names(backend)[-5:] == ['close_gui', 'stop', 'choose_save', 'save', 'export']


@pytest.mark.parametrize('kwargs', [dict(stop_state={'state': 'RUNNING'}),
                                  dict(stop_state={'state': 'STOPPED', 'exit_code': 1}),
                                  dict(stop_error=RuntimeError('owned cleanup failed'))])
def test_export_requires_normal_cleanup(tmp_path, kwargs):
    backend = Backend(**kwargs)
    result, code = execute(request(tmp_path), backend)
    assert code == 2 and result['export'] is None and 'export' not in names(backend)


def test_export_failure_keeps_closed_state_and_failure_visible(tmp_path):
    backend = Backend(export_error=RuntimeError('native DB not closed'))
    result, code = execute(request(tmp_path), backend)
    assert code == 2 and result['closed_session']['state'] == 'STOPPED'
    assert any('native DB not closed' in error for error in result['errors'])


@pytest.mark.parametrize('value', [{'status': 'PENDING'}, {'status': 'FAILED'}, None, 'invalid backend response', []])
def test_pending_or_absent_save_cannot_be_reported_saved(tmp_path, value):
    backend = Backend()
    backend.save = lambda handle, destination: value
    result, code = execute(request(tmp_path), backend)
    assert code == 2 and result['status'] == 'FAILED'


def test_broken_output_pipe_cannot_skip_session_cleanup(tmp_path):
    backend = Backend()
    def broken(_value):
        raise BrokenPipeError('terminal closed')
    result, code = execute(request(tmp_path), backend, emit=broken)
    assert code == 0 and names(backend)[-4:] == ['stop', 'choose_save', 'save', 'export']


def test_rviz_launcher_uses_owned_wrapper_argv_and_verified_environment(tmp_path, monkeypatch):
    value = request(tmp_path)
    Path(value['output_dir']).mkdir(parents=True)
    value['rviz_environment'] = {'DISPLAY': ':3'}
    captured = {}
    def popen(argv, **kwargs):
        captured.update(argv=argv, kwargs=kwargs)
        return 'owned GUI'
    monkeypatch.setattr(app.subprocess, 'Popen', popen)
    result = app.launch_rviz(['rviz2', '-d', '/reviewed config/map.rviz'], value)
    assert result == 'owned GUI'
    assert captured['argv'][1:3] == ['-m', 'wc_runtime.component']
    assert captured['argv'][-3:] == ['rviz2', '-d', '/reviewed config/map.rviz']
    assert captured['kwargs']['start_new_session'] is True
    assert captured['kwargs']['env']['DISPLAY'] == ':3'
    assert captured['kwargs']['env']['ROS_DOMAIN_ID'] == '83'


def test_close_rviz_signals_only_owned_popen_and_waits():
    events = []
    child = SimpleNamespace(poll=lambda: None, send_signal=lambda signum: events.append(('signal', signum)),
                            wait=lambda timeout: events.append(('wait', timeout)))
    app.close_rviz(child)
    assert events == [('signal', app.signal.SIGTERM), ('wait', 15)]
