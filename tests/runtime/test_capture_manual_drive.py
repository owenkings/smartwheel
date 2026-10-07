"""Independent capture ownership/lifecycle tests; no ROS, UI or device access."""
import copy
import json
import os
from pathlib import Path
import shutil
import signal
import sys
from types import ModuleType, SimpleNamespace
import uuid

import pytest

from wc_runtime import capture


PROJECT = Path(__file__).resolve().parents[2]


@pytest.fixture(autouse=True)
def posix_imports_for_pure_windows_tests(monkeypatch):
    if os.name == 'nt':
        module = ModuleType('fcntl')
        module.LOCK_EX, module.LOCK_NB, module.LOCK_UN = 2, 4, 8
        module.flock = lambda *args: None
        monkeypatch.setitem(sys.modules, 'fcntl', module)
        monkeypatch.setattr(signal, 'SIGHUP', 1, raising=False)


@pytest.fixture
def capture_temp():
    parent = Path(__file__).resolve().parent
    directory = parent/('.manual_capture_test_'+uuid.uuid4().hex)
    directory.mkdir()
    try:
        yield directory
    finally:
        assert directory.resolve().parent == parent
        assert directory.name.startswith('.manual_capture_test_')
        shutil.rmtree(directory)


def read_config(name):
    return json.loads((PROJECT/'config'/name).read_text(encoding='utf-8'))


def write_json(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value), encoding='utf-8')


def frozen_bindings(directory, *, serial='CAPTURE_TEST_A'):
    """Prepare the immutable session input required by source_commands."""
    from wc_runtime.device_bindings import validate_device_bindings
    value = validate_device_bindings(dict(schema_version=1,
        imu=dict(device='/dev/smartwheel_h30_imu',
            expected_by_id='/dev/serial/by-id/usb-1a86_USB_Single_Serial_'+serial+'-if00',
            hardware_serial=serial, sensor_id='H30-'+serial),
        network=dict(interface=None)))
    write_json(directory/'configuration/device_bindings.json', value)
    return value


def ready_evidence(session):
    return dict(session_id=session, ready=True, raw_recorder_discovered=True,
                manual_socket_ready=True, feedback_samples=1, source_type='HYBRID_MANUAL_CAPTURE')


def final_evidence(session, count=7):
    return dict(session_id=session, control_transmissions=count,
                control_write_attempts=count+1, control_bytes_observed=count*8,
                ready_observed=True, recorder_discovered=True,
                synchronized=True, closed_normally=True, journal={'final_fsync_complete': True})


def test_capacity_accounts_for_source_initialization_before_manual_window():
    default = capture.capacity('all_sensors',1,10**12)
    manual = capture.capacity('all_sensors',1,10**12,initialization_budget_s=capture.MANUAL_READY_TIMEOUT_S)
    equivalent = capture.capacity('all_sensors',21,10**12)
    assert default['estimated_source_window_s'] == 1 and default['source_initialization_budget_s'] == 0
    assert manual['requested_duration_s'] == 1 and manual['source_initialization_budget_s'] == 20
    assert manual['estimated_source_window_s'] == 21
    assert manual['estimated_recording_bytes'] == equivalent['estimated_recording_bytes']
    maximum = capture.capacity('mapping_core',3600,10**12,initialization_budget_s=20)
    assert maximum['requested_duration_s'] == 3600 and maximum['estimated_source_window_s'] == 3620


def test_manual_capture_uses_one_owner_and_preserves_read_only_default(capture_temp, monkeypatch):
    from wc_runtime import cli
    monkeypatch.setattr(cli, 'ros_command', lambda arguments: list(map(str, arguments)))
    frozen_bindings(capture_temp/'data')
    default = capture.source_commands(PROJECT, capture_temp/'run', capture_temp/'data', 'session', 'all_sensors')
    manual = capture.source_commands(PROJECT, capture_temp/'run', capture_temp/'data', 'session',
                                     'all_sensors', manual_drive=True)
    assert 'wc_motion.feedback_transport' in default['wheel']
    assert 'manual_ui' not in default
    assert 'wc_runtime.mapping_wheel' in manual['wheel']
    assert 'wc_motion.feedback_transport' not in json.dumps(manual)
    assert set(manual)-set(default) == {'manual_ui'}
    assert manual['manual_ui'] == [
        str(PROJECT/'install/main/wc_bringup/lib/wc_bringup/manual_capture_ui'),
        '--session-root', str(capture_temp/'data'), '--session-id', 'session']
    assert '--require-recorder' in manual['wheel']
    assert '--config' in manual['wheel'] and '--summary-path' in manual['wheel'] and '--raw-output' in manual['wheel']
    assert not any(value in json.dumps(manual) for value in ('mapping_prior', 'rtabmap', 'slam_backend'))


def test_manual_ui_uses_project_install_and_keeps_paths_as_separate_arguments(capture_temp, monkeypatch):
    from wc_runtime import cli
    monkeypatch.setattr(cli, 'ros_command', lambda arguments: list(map(str, arguments)))
    root = capture_temp/'project with spaces'
    directory = capture_temp/'capture with spaces'
    frozen_bindings(directory)
    commands = capture.source_commands(root, root/'run', directory, 'identified_session',
                                       'mapping_core', manual_drive=True)
    assert commands['manual_ui'] == [
        str(root/'install/main/wc_bringup/lib/wc_bringup/manual_capture_ui'),
        '--session-root', str(directory), '--session-id', 'identified_session']
    assert 'ros2' not in commands['manual_ui'] and 'run' not in commands['manual_ui']


def test_manual_ui_keeps_ros_environment_exec_wrapper(capture_temp):
    from wc_runtime import cli
    root = PROJECT
    directory = capture_temp/'session'
    frozen_bindings(directory)
    commands = capture.source_commands(root, root/'.phase1_runtime', directory, 'session',
                                       'mapping_core', manual_drive=True)
    expected = cli.ros_command([
        root/'install/main/wc_bringup/lib/wc_bringup/manual_capture_ui',
        '--session-root', directory, '--session-id', 'session'])
    assert commands['manual_ui'] == expected
    # Sourcing still supplies the installed Qt/ROS library environment, but
    # exec replaces the shell with the exact preflight-checked UI process.
    assert 'exec "$@"' in commands['manual_ui'][4]
    executable = str(root/'install/main/wc_bringup/lib/wc_bringup/manual_capture_ui')
    assert commands['manual_ui'].count(executable) == 1
    assert executable not in commands['manual_ui'][4]  # A positional argument, not shell code.


@pytest.mark.parametrize('manual_drive', [False, True])
def test_source_commands_keep_frozen_bindings_after_current_machine_changes(capture_temp, monkeypatch, manual_drive):
    from wc_runtime import cli
    monkeypatch.setattr(cli, 'ros_command', lambda arguments: list(map(str, arguments)))
    root, directory = capture_temp/'project', capture_temp/'capture'
    original = frozen_bindings(directory, serial='FROZEN_A')
    path = directory/'configuration/device_bindings.json'
    raw = path.read_bytes()
    before = capture.source_commands(root, root/'run', directory, 'frozen_session',
                                     'mapping_core', manual_drive=manual_drive)
    changed = frozen_bindings(capture_temp/'other_session', serial='CURRENT_B')
    changed['imu']['device'] = '/dev/current_machine_h30'
    write_json(root/'config/device_bindings.json', changed)
    write_json(root/'config/device_bindings.local.json', changed)
    monkeypatch.setattr(cli, 'DEVICE_BINDINGS', changed)
    monkeypatch.setattr(cli, 'H30_DEVICE', changed['imu']['device'])
    monkeypatch.setattr(cli, 'H30_BY_ID', changed['imu']['expected_by_id'])
    monkeypatch.setattr(cli, 'H30_SERIAL', changed['imu']['hardware_serial'])
    after = capture.source_commands(root, root/'run', directory, 'frozen_session',
                                    'mapping_core', manual_drive=manual_drive)
    assert after == before
    for option, field in (('--device', 'device'), ('--expected-by-id', 'expected_by_id'),
                          ('--hardware-serial', 'hardware_serial'), ('--sensor-id', 'sensor_id')):
        assert after['imu'][after['imu'].index(option)+1] == original['imu'][field]
    assert 'CURRENT_B' not in json.dumps(after)
    assert path.read_bytes() == raw
    # Losing the archive is an error even when current-machine bindings exist.
    path.unlink()
    with pytest.raises(FileNotFoundError):
        capture.source_commands(root, root/'run', directory, 'frozen_session',
                                'mapping_core', manual_drive=manual_drive)


def test_manual_runtime_authorization_does_not_fill_unknown_geometry(capture_temp):
    setup = json.loads((PROJECT/'tests/fixtures/v7_hardware_unknown.json').read_text(encoding='utf-8'))
    setup['manual_controls']['arm_allowed'] = False
    original = copy.deepcopy(setup)
    assert setup['assembly']['confirmed'] is False
    assert all(item['translation']['value_m'] is None for item in setup['data_transforms'])
    runtime = capture.manual_runtime_configuration(setup, read_config('wheel_feedback_current.json'),
                                                   capture_temp/'session', 'session')
    assert setup == original
    assert runtime['manual_controls']['arm_allowed'] is True
    assert runtime['manual_controls']['interaction_policy'] == 'hybrid_manual'
    assert runtime['manual_controls']['operation_evidence']['status'] == 'USER_REPORTED_LEGACY_OPERATION'
    assert runtime['manual_authorization']['source'] == 'EXPLICIT_CAPTURE_MANUAL_DRIVE_ARGUMENT'
    assert runtime['manual_authorization']['hardware_stop_validated'] is False
    assert runtime['session_id'] == 'session' and runtime['source_mode'] == 'real' and runtime['status'] == 'EXPERIMENT'
    assert runtime['continuous_mapping'] and runtime['duration_s'] == 0
    assert not runtime['motion_estimation'] and not runtime['slam']
    assert 'transforms' not in runtime and 'geometry_report' not in runtime
    assert runtime['wheel_candidate']['formal_odometry_eligible'] is False


def test_manual_runtime_requires_explicit_command_scale(capture_temp):
    setup = read_config('hardware_setup.json')
    del setup['manual_controls']['command_rpm_to_register_scale']
    with pytest.raises(ValueError, match='MANUAL_DRIVE_SETTINGS_UNAVAILABLE'):
        capture.manual_runtime_configuration(setup, read_config('wheel_feedback_current.json'),
                                             capture_temp/'session', 'session')


@pytest.mark.parametrize('change', [dict(session_id='other'), dict(ready=False),
    dict(raw_recorder_discovered=False), dict(manual_socket_ready=False), dict(feedback_samples=0),
    dict(feedback_samples=True), dict(source_type='READ_ONLY')])
def test_manual_readiness_requires_actual_feedback_and_matching_session(capture_temp, change):
    assert capture.wheel_ready(capture_temp, 'session') is False
    value = ready_evidence('session')
    value.update(change)
    write_json(capture_temp/'wheel_ready.json', value)
    with pytest.raises(ValueError, match='MANUAL_WHEEL_READY_INVALID'):
        capture.wheel_ready(capture_temp, 'session')


def test_manual_readiness_valid_and_non_object_marker(capture_temp):
    write_json(capture_temp/'wheel_ready.json', ready_evidence('session'))
    assert capture.wheel_ready(capture_temp, 'session') is True
    write_json(capture_temp/'wheel_ready.json', [])
    with pytest.raises(ValueError, match='MANUAL_WHEEL_READY_INVALID'):
        capture.wheel_ready(capture_temp, 'session')


def test_control_evidence_reports_actual_counts_after_persistence(capture_temp):
    write_json(capture_temp/'sources/wheel_summary.json', final_evidence(capture_temp.name))
    result = capture.final_control_evidence(capture_temp, manual_drive=True)
    assert result['control_transmissions'] == 7
    assert result['control_write_attempts'] == 8 and result['control_bytes_observed'] == 56
    assert result['control_count_status'] == 'SOURCE_OWNER_FINAL_SYNCHRONIZED_COUNTER'
    assert 'error' not in result
    result = capture.final_control_evidence(capture_temp)
    assert result['control_transmissions'] == 7 and 'read-only' in result['error']


@pytest.mark.parametrize('change', [dict(synchronized=False), dict(closed_normally=False),
    dict(journal=None), dict(journal={'final_fsync_complete':False}), dict(session_id='other'),
    dict(ready_observed=False), dict(recorder_discovered=False)])
def test_unfinished_or_mismatched_control_evidence_cannot_claim_final_sync(capture_temp, change):
    value = final_evidence(capture_temp.name)
    value.update(change)
    write_json(capture_temp/'sources/wheel_summary.json', value)
    result = capture.final_control_evidence(capture_temp, manual_drive=True)
    assert result['control_transmissions'] == 7  # Preserve observed count, not a guessed zero.
    assert result['control_count_status'] == 'SOURCE_OWNER_FINAL_COUNTER'
    assert result.get('error')


@pytest.mark.parametrize('value', [None, [], {}, {'control_transmissions':True}, {'control_transmissions':-1}])
def test_missing_invalid_control_evidence_is_unknown(capture_temp, value):
    if value is not None:
        write_json(capture_temp/'sources/wheel_summary.json', value)
    result = capture.final_control_evidence(capture_temp, manual_drive=True)
    assert result['control_transmissions'] is None
    assert result['control_count_status'] == 'UNVERIFIED' and result.get('error')


def test_stop_order_revokes_ui_and_owner_before_sources_and_recorder(monkeypatch):
    events = []
    def stop(children, grace=40):
        events.append((list(children), grace))
        return dict.fromkeys(children, 0)
    monkeypatch.setattr(capture, 'stop_children', stop)
    monkeypatch.setattr(capture.time, 'sleep', lambda seconds: events.append(('drain', seconds)))
    children = dict(lidar=object(), imu=object(), wheel=object(), manual_ui=object(), camera_left=object())
    results = capture.stop_capture_children(children, object(), manual_drive=True)
    assert events == [(['manual_ui'], 40), (['wheel'],45), (['lidar','imu','camera_left'],40),
                      ('drain',.5), (['recorder'],60)]
    assert set(results) == set(children)|{'recorder'}
    assert set(children) == {'lidar','imu','wheel','manual_ui','camera_left'}


def mock_capture_run(monkeypatch, capture_temp, *, mode='duration_reached', manual_drive=True, diagnostic=False):
    """Exercise the real capture supervisor with synthetic subprocesses/audit.

    Data-level source/bag reconciliation is independently tested by capture_audit;
    this fixture isolates launch authorization, owner identity and stop lifecycle.
    """
    from wc_runtime import cli, capture_audit, capture_contract, capture_support, runtime_health, sensor_viewer
    root = capture_temp/'project'
    root.mkdir()
    run = root/'run'
    session = 'synthetic_manual'
    directory = root/'data/experiments'/session
    events, handlers, processes = [], {}, {}
    checks = {name:dict(status='AVAILABLE') for name in ('lidar','imu','wheel','manual_drive_configuration','manual_ui')}
    checks['capacity'] = dict(sufficient=True)
    monkeypatch.setattr(cli,'ROOT',root)
    monkeypatch.setattr(cli,'RUN',run)
    monkeypatch.setattr(cli,'target',lambda: None)
    monkeypatch.setattr(cli,'environment',lambda: {})
    monkeypatch.setattr(cli,'ros_command',lambda arguments:list(map(str,arguments)))
    # Match the supervisor's frozen contract identity without real devices.
    bindings = read_config('device_bindings.json')
    monkeypatch.setattr(cli, 'DEVICE_BINDINGS', copy.deepcopy(bindings))
    monkeypatch.setattr(capture,'preflight',lambda *args,**kwargs:copy.deepcopy(checks))
    monkeypatch.setattr(capture.shutil,'disk_usage',lambda path:SimpleNamespace(free=10**12))
    monkeypatch.setattr(sensor_viewer,'desktop_environment',lambda:{'DISPLAY':':synthetic'})
    monkeypatch.setattr(sensor_viewer,'confirm_display',lambda env: None)
    monkeypatch.setattr(runtime_health,'write_health_report',lambda directory,**kwargs: None)

    def signal_handler(sig, handler):
        previous = handlers.get(sig)
        handlers[sig] = handler
        return previous
    monkeypatch.setattr(capture.signal,'signal',signal_handler)

    def snapshot(source, output, **kwargs):
        config = output/'configuration'
        config.mkdir()
        write_json(config/'hardware_setup.json',read_config('hardware_setup.json'))
        write_json(config/'wheel_feedback.json',read_config('wheel_feedback_current.json'))
        write_json(config/'device_bindings.json', bindings)
        for side in ('left','right'):
            path=config/'lidar'/(side+'.yaml');path.parent.mkdir(exist_ok=True)
            path.write_text('side: '+side+'\nexpected_serial: synthetic_'+side+
                            '\ndevice_ip: synthetic_'+side+'\nreceive_ip: synthetic\n',encoding='utf-8')
        (output/'records.jsonl').write_text('',encoding='utf-8')
        return {}
    monkeypatch.setattr(capture,'snapshot',snapshot)

    class Process:
        def __init__(self, name):
            self.name, self.code, self.interrupted = name, None, False
        def poll(self):
            if self.name == 'manual_ui':
                if mode == 'window_closed':
                    return 0
                if mode == 'ui_failed':
                    return 9
                if mode == 'user_stop' and not self.interrupted:
                    self.interrupted = True
                    handlers[signal.SIGINT](signal.SIGINT,None)
            if self.name == 'wheel' and mode == 'wheel_failed':
                return 8
            return self.code

    def launch(command, **kwargs):
        if 'wc_runtime.source_recorder' in command:
            name = 'recorder'
            write_json(directory/'recorder_ready.json',{'ready':True})
        elif 'wc_runtime.mapping_wheel' in command or 'wc_motion.feedback_transport' in command:
            name = 'wheel'
            if mode != 'no_readiness':
                write_json(directory/'wheel_ready.json',ready_evidence(session))
        elif any(Path(argument).name == 'manual_capture_ui' for argument in command):
            name = 'manual_ui'
            assert capture.wheel_ready(directory,session)
            manifest = json.loads((directory/'capture_manifest.json').read_text(encoding='utf-8'))
            assert manifest['status'] == 'RECORDING' and manifest['manual_drive'] is True
        elif 'wc_imu.ros_node' in command:
            name = 'imu'
        else:
            name = 'lidar'
        events.append(('start',name))
        process = Process(name)
        processes[name] = process
        return process
    monkeypatch.setattr(capture.subprocess,'Popen',launch)

    def stop(children, grace=40):
        events.append(('stop',list(children)))
        results = {}
        for name, process in children.items():
            code = process.poll()
            results[name] = code if code is not None else 0
            process.code = results[name]
            if name == 'wheel' and mode != 'missing_summary':
                write_json(directory/'sources/wheel_summary.json',final_evidence(session,7 if manual_drive else 0))
        return results
    monkeypatch.setattr(capture,'stop_children',stop)

    # Advance time without sleeping; missing readiness must terminate before UI.
    clock = [0.]
    def monotonic():
        clock[0] += .1
        return clock[0]
    monkeypatch.setattr(capture.time,'monotonic',monotonic)
    monkeypatch.setattr(capture.time,'monotonic_ns',lambda:int(clock[0]*1e9))
    monkeypatch.setattr(capture.time,'sleep',lambda seconds: None)
    def readiness(output,contract):
        # This supervisor fixture substitutes received samples; byte/window
        # evidence is exercised independently in test_capture_audit_contract.
        sources={logical:dict(schema_version=1,session_id=session,logical_source=logical,
            source_id=spec['source_id'],stream_epoch='synthetic-epoch',first_valid_monotonic_ns=1,
            last_valid_monotonic_ns=int((clock[0]+20)*1e9),valid_count=1,ready=True,error=None)
            for logical,spec in contract['sources'].items()}
        return dict(all_ready=True,ready=True,sources=sources,missing=[],invalid={},observed_common_ready_ns=1)
    monkeypatch.setattr(capture_contract,'read_source_readiness',readiness)
    def audit(*args):
        complete_window=mode not in ('window_closed','user_stop')
        issues=[] if complete_window else [dict(code='REQUESTED_WINDOW_NOT_COVERED',evidence='sources',
            detail='synthetic user interruption before requested window end')]
        return dict(status='PARTIAL',recording_complete=False,transport_complete=True,
                    window_complete=complete_window,durability_complete=None,
                    completeness_contract='IDENTITY_TRANSPORT_COMMON_WINDOW_DURABILITY',
                    completion_pending=['durability'],issues=issues,source_accounting={})
    monkeypatch.setattr(capture_audit,'audit_capture',audit)
    if os.name=='nt':
        # Windows cannot fsync directory descriptors; target Linux integration
        # exercises real fsync and dedicated finalization fault tests cover it.
        monkeypatch.setattr(capture_support,'synchronize_tree',lambda output:dict(complete=True,
            file_count=0,directory_count=0,completed_monotonic_ns=int(clock[0]*1e9),method='SYNTHETIC_TEST_ONLY'))
    args = SimpleNamespace(profile='mapping_core',session=session,duration=10,dry_run=False,
                           diagnostic=diagnostic,manual_drive=manual_drive)
    return args,directory,events,processes,checks


def test_user_window_close_retains_partial_window_after_owner_drain_with_actual_controls(capture_temp,monkeypatch):
    args,directory,events,_,_ = mock_capture_run(monkeypatch,capture_temp,mode='window_closed')
    assert capture.run_capture(args) == 2
    manifest = json.loads((directory/'capture_manifest.json').read_text(encoding='utf-8'))
    assert manifest['status'] == 'PARTIAL' and not manifest['recording_complete']
    assert manifest['stop_reason'] == 'MANUAL_UI_WINDOW_CLOSED'
    assert any(row['code']=='REQUESTED_WINDOW_NOT_COVERED' for row in manifest['issues'])
    assert manifest['control_transmissions'] == 7 and manifest['manual_drive'] is True
    assert manifest['control_count_status'] == 'SOURCE_OWNER_FINAL_SYNCHRONIZED_COUNTER'
    assert not manifest['motion_estimation'] and not manifest['slam']
    assert events == [('start','recorder'),('start','lidar'),('start','imu'),('start','wheel'),
                      ('start','manual_ui'),('stop',['manual_ui']),('stop',['wheel']),
                      ('stop',['lidar','imu']),('stop',['recorder'])]
    runtime = json.loads((directory/'configuration/manual_runtime.json').read_text(encoding='utf-8'))
    assert runtime['manual_controls']['interaction_policy'] == 'hybrid_manual'
    assert 'configuration/manual_runtime.json' in manifest['input_hashes']


@pytest.mark.parametrize('mode,error', [('ui_failed','MANUAL_UI_FAILED'),
    ('wheel_failed','SOURCE_EXITED: wheel'),('no_readiness','MANUAL_WHEEL_NOT_READY'),
    ('missing_summary','CONTROL_FINAL_EVIDENCE_INVALID')])
def test_startup_or_owner_failures_remain_partial_even_diagnostic(capture_temp,monkeypatch,mode,error):
    args,directory,events,_,_ = mock_capture_run(monkeypatch,capture_temp,mode=mode,diagnostic=True)
    assert capture.run_capture(args) == 2
    manifest = json.loads((directory/'capture_manifest.json').read_text(encoding='utf-8'))
    assert manifest['status'] == 'PARTIAL' and not manifest['recording_complete']
    assert any(error in item['detail'] or error == item['code'] for item in manifest['issues'])
    if mode == 'no_readiness':
        assert ('start','manual_ui') not in events
    if mode == 'missing_summary':
        assert manifest['control_transmissions'] is None
    assert events.index(('stop',['wheel'])) < events.index(('stop',['lidar','imu']))
    assert events[-1] == ('stop',['recorder'])


def test_manual_early_user_stop_retains_partial_requested_window(capture_temp,monkeypatch):
    args,directory,_,_,_ = mock_capture_run(monkeypatch,capture_temp,mode='user_stop')
    assert capture.run_capture(args) == 2
    manifest = json.loads((directory/'capture_manifest.json').read_text(encoding='utf-8'))
    assert manifest['stop_reason'] == 'USER_STOP_REQUEST' and not manifest['recording_complete']
    assert any('USER_INTERRUPTED_BEFORE_REQUESTED_DURATION' in item['detail'] for item in manifest['issues'])


def test_diagnostic_flag_cannot_authorize_unavailable_manual_owner(capture_temp,monkeypatch):
    args,directory,events,_,checks = mock_capture_run(monkeypatch,capture_temp,diagnostic=True)
    checks['manual_ui'] = dict(status='BLOCKED',reason='no desktop')
    with pytest.raises(RuntimeError,match='MANUAL_DRIVE_PREFLIGHT_BLOCKED'):
        capture.run_capture(args)
    assert events == [] and not directory.exists()


def test_direct_recording_starts_despite_duration_forecast_shortfall(capture_temp,monkeypatch):
    args,directory,events,_,checks = mock_capture_run(monkeypatch,capture_temp)
    checks['capacity']=capture.capacity('mapping_core',300,6*1024**3,initialization_budget_s=30)
    assert not checks['capacity']['sufficient'] and checks['capacity']['startup_allowed']
    assert capture.run_capture(args)==0
    assert ('start','recorder') in events and ('start','manual_ui') in events
    manifest=json.loads((directory/'capture_manifest.json').read_text(encoding='utf-8'))
    assert manifest['recording_complete'] and manifest['preflight']['capacity']['status']=='WARNING'


@pytest.mark.parametrize('headroom',[0,1,capture.SOFTWARE_SNAPSHOT_BUDGET_BYTES])
def test_actual_disk_reserve_shortfall_still_starts_no_process(capture_temp,monkeypatch,headroom):
    args,directory,events,_,checks = mock_capture_run(monkeypatch,capture_temp,diagnostic=True)
    checks['capacity']=capture.capacity('mapping_core',300,capture.RESERVE_BYTES+headroom)
    with pytest.raises(RuntimeError,match='CAPTURE_STORAGE_RESERVE_REACHED'):
        capture.run_capture(args)
    assert events==[] and not directory.exists()


def test_runtime_space_exhaustion_drains_and_preserves_partial(capture_temp,monkeypatch):
    from wc_runtime import capture_storage
    args,directory,events,_,_ = mock_capture_run(monkeypatch,capture_temp)
    original=capture_storage.StorageMonitor.check
    def low_space(self,**kwargs):
        if not kwargs.get('closing') and ('start','manual_ui') in events:
            raise RuntimeError('CAPTURE_STORAGE_RESERVE_REACHED: injected real space drop')
        return original(self,**kwargs)
    monkeypatch.setattr(capture_storage.StorageMonitor,'check',low_space)
    assert capture.run_capture(args)==2
    manifest=json.loads((directory/'capture_manifest.json').read_text(encoding='utf-8'))
    assert not manifest['recording_complete'] and manifest['status']=='PARTIAL'
    assert any('injected real space drop' in row['detail'] for row in manifest['issues'])
    assert events[-4:]==[('stop',['manual_ui']),('stop',['wheel']),('stop',['lidar','imu']),('stop',['recorder'])]
    assert (directory/'records.jsonl').exists()


def test_display_lost_after_preflight_does_not_create_recording_session(capture_temp,monkeypatch):
    from wc_runtime import sensor_viewer
    args,directory,events,_,_ = mock_capture_run(monkeypatch,capture_temp)
    def lost_display(env):
        raise RuntimeError('display lost after preflight')
    monkeypatch.setattr(sensor_viewer,'confirm_display',lost_display)
    with pytest.raises(RuntimeError,match='display lost after preflight'):
        capture.run_capture(args)
    assert events == [] and not directory.exists()


def assert_stopped_checkpoint(directory, events):
    value = json.loads((directory/'capture_manifest.json').read_text(encoding='utf-8'))
    assert value['status'] == 'PARTIAL' and value['recording_complete'] is False
    assert value['finalization_state'] == 'FINALIZING'
    assert value['control_transmissions'] == 7
    assert value['process_exit_codes'] == dict(recorder=0,lidar=0,imu=0,wheel=0,manual_ui=0)
    assert events[-1] == ('stop',['recorder'])


@pytest.mark.parametrize('stage,code', [('copy','CAPTURE_SOURCE_COPY_FAILED'),
    ('audit','CAPTURE_AUDIT_FAILED'), ('audit_schema','CAPTURE_AUDIT_FAILED'),
    ('hash','CAPTURE_HASH_FAILED'), ('scan','CAPTURE_HASH_SCAN_FAILED'),
    ('health','CAPTURE_HEALTH_REPORT_FAILED'), ('summary','CAPTURE_SUMMARY_WRITE_FAILED')])
def test_stopped_finalization_faults_persist_partial_and_keep_control_evidence(capture_temp,monkeypatch,stage,code):
    from wc_runtime import capture_audit, cli, runtime_health
    args,directory,events,_,_ = mock_capture_run(monkeypatch,capture_temp)
    health_calls = []
    def healthy(output, *, capture_manifest):
        assert_stopped_checkpoint(directory,events)
        health_calls.append(copy.deepcopy(capture_manifest))
        write_json(output/'health.json',{'lifecycle':capture_manifest['status'],
            'recording':{'recording_complete':capture_manifest['recording_complete']}})
    monkeypatch.setattr(runtime_health,'write_health_report',healthy)
    if stage == 'copy':
        write_json(cli.RUN/'sessions'/args.session/'left/epoch/source.json',{'retained_original':True})
        def fail_copy(*args,**kwargs):
            assert_stopped_checkpoint(directory,events)
            raise OSError('injected source copy failure')
        monkeypatch.setattr(capture.shutil,'copytree',fail_copy)
    elif stage in ('audit','audit_schema'):
        def fail_audit(*args):
            assert_stopped_checkpoint(directory,events)
            if stage == 'audit_schema':
                return dict(recording_complete=True,status='COMPLETE',issues=None,source_accounting={})
            raise ValueError('injected damaged source schema')
        monkeypatch.setattr(capture_audit,'audit_capture',fail_audit)
    elif stage == 'hash':
        original = capture.digest
        def fail_hash(path):
            if Path(path).name == 'lidar.log':
                assert_stopped_checkpoint(directory,events)
                raise OSError('injected artifact hash failure')
            return original(path)
        monkeypatch.setattr(capture,'digest',fail_hash)
    elif stage == 'scan':
        original = Path.rglob
        def fail_scan(path,*args,**kwargs):
            if path == directory:
                assert_stopped_checkpoint(directory,events)
                raise OSError('injected artifact traversal failure')
            return original(path,*args,**kwargs)
        monkeypatch.setattr(Path,'rglob',fail_scan)
    elif stage == 'health':
        def fail_health(output, *, capture_manifest):
            assert_stopped_checkpoint(directory,events)
            health_calls.append(copy.deepcopy(capture_manifest))
            # Simulate health.json being replaced before diagnosis failed.
            write_json(output/'health.json',{'lifecycle':capture_manifest['status'],
                'recording':{'recording_complete':capture_manifest['recording_complete']}})
            raise OSError('injected health diagnosis failure')
        monkeypatch.setattr(runtime_health,'write_health_report',fail_health)
    elif stage == 'summary':
        original = Path.write_text
        def fail_summary(path,*args,**kwargs):
            if path.name == 'summary_zh.md':
                assert_stopped_checkpoint(directory,events)
                raise OSError('injected summary write failure')
            return original(path,*args,**kwargs)
        monkeypatch.setattr(Path,'write_text',fail_summary)
    assert capture.run_capture(args) == 2
    manifest = json.loads((directory/'capture_manifest.json').read_text(encoding='utf-8'))
    assert manifest['status'] == 'PARTIAL' and manifest['recording_complete'] is False
    assert manifest['finalization_state'] == 'FINALIZED'
    assert manifest['control_transmissions'] == 7
    assert any(row['code']==code for row in manifest['issues'])
    assert (directory/'sources/wheel_summary.json').is_file()
    assert len([event for event in events if event[0]=='start']) == 5  # Never restart a source.
    if stage == 'copy':
        assert (cli.RUN/'sessions'/args.session/'left/epoch/source.json').is_file()
    if stage == 'hash':
        assert 'logs/lidar.log' not in manifest['input_hashes']
    if stage == 'health':
        assert len(health_calls) == 2
        assert health_calls[-1]['status'] == 'PARTIAL' and not (directory/'health.json').exists()
        assert len(list(directory.glob('health.failed_finalization_*.json'))) == 1
        assert '状态：PARTIAL' in (directory/'summary_zh.md').read_text(encoding='utf-8')
    else:
        health = json.loads((directory/'health.json').read_text(encoding='utf-8'))
        assert health['lifecycle']=='PARTIAL' and health['recording']['recording_complete'] is False


def test_health_uses_prospective_complete_before_durable_final_commit(capture_temp,monkeypatch):
    from wc_runtime import runtime_health
    args,directory,events,_,_ = mock_capture_run(monkeypatch,capture_temp)
    seen = []
    def health(output, *, capture_manifest):
        assert_stopped_checkpoint(directory,events)
        assert capture_manifest['status']=='COMPLETE' and capture_manifest['recording_complete'] is True
        seen.append(capture_manifest['session_id'])
    monkeypatch.setattr(runtime_health,'write_health_report',health)
    assert capture.run_capture(args)==0 and seen==[args.session]
    manifest = json.loads((directory/'capture_manifest.json').read_text(encoding='utf-8'))
    assert manifest['status']=='COMPLETE' and manifest['finalization_state']=='FINALIZED'


@pytest.mark.parametrize('after_rename',[False,True])
def test_final_manifest_commit_failure_downgrades_reports_and_returns_nonzero(capture_temp,monkeypatch,capsys,after_rename):
    from wc_runtime import runtime_health
    args,directory,events,_,_ = mock_capture_run(monkeypatch,capture_temp)
    original = capture.atomic_json
    health_statuses = []
    def health(output, *, capture_manifest):
        health_statuses.append(capture_manifest['status'])
        write_json(output/'health.json',{'lifecycle':capture_manifest['status'],
            'recording':{'recording_complete':capture_manifest['recording_complete']}})
    def fail_commit(path,value):
        if Path(path).name=='capture_manifest.json' and value['status']=='COMPLETE':
            assert_stopped_checkpoint(directory,events)
            if after_rename:
                original(path,value)
            raise OSError('injected final commit fsync failure')
        return original(path,value)
    monkeypatch.setattr(capture,'atomic_json',fail_commit)
    monkeypatch.setattr(runtime_health,'write_health_report',health)
    assert capture.run_capture(args)==2
    manifest = json.loads((directory/'capture_manifest.json').read_text(encoding='utf-8'))
    assert manifest['status']=='PARTIAL' and manifest['recording_complete'] is False
    assert manifest['manifest_persistence_failed'] and manifest['control_transmissions']==7
    assert any(row['code']=='CAPTURE_MANIFEST_WRITE_FAILED' for row in manifest['issues'])
    assert health_statuses==['COMPLETE','PARTIAL']
    health = json.loads((directory/'health.json').read_text(encoding='utf-8'))
    assert health['lifecycle']=='PARTIAL' and health['recording']['recording_complete'] is False
    assert '状态：PARTIAL' in (directory/'summary_zh.md').read_text(encoding='utf-8')
    assert 'CAPTURE_MANIFEST_WRITE_FAILED FINAL_RESULT' in capsys.readouterr().err


def test_checkpoint_storage_failure_returns_nonzero_without_copy_or_audit(capture_temp,monkeypatch,capsys):
    from wc_runtime import capture_audit
    args,directory,events,_,_ = mock_capture_run(monkeypatch,capture_temp)
    original = capture.atomic_json
    attempts=[]
    def failed_checkpoint(path,value):
        if Path(path).name=='capture_manifest.json' and value.get('finalization_state')=='FINALIZING':
            assert events[-1]==('stop',['recorder'])
            attempts.append(value['status'])
            raise OSError('injected unavailable manifest storage')
        return original(path,value)
    monkeypatch.setattr(capture,'atomic_json',failed_checkpoint)
    monkeypatch.setattr(capture_audit,'audit_capture',lambda *args:pytest.fail('audit before durable checkpoint'))
    assert capture.run_capture(args)==2 and attempts==['PARTIAL','PARTIAL']
    assert not (directory/'sources/lidar').exists()
    assert (directory/'sources/wheel_summary.json').is_file()
    assert 'CAPTURE_PARTIAL_MANIFEST_WRITE_FAILED' in capsys.readouterr().err
