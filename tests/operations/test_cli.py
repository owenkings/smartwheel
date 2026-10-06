"""CLI parsing/forwarding/evidence preservation without ROS or device dispatch."""

import json
import os
from pathlib import Path
import subprocess
import sys
import uuid

import pytest

from wc_runtime import cli


PROBE = Path(__file__).with_name('cli_probe.py').resolve()


def probe(tmp_path, *arguments):
    dispatch = tmp_path/(uuid.uuid4().hex+'-dispatch.jsonl')
    env = os.environ.copy()
    env['WC_OPS_DISPATCH_LOG'] = str(dispatch)
    env['PYTHONNOUSERSITE'] = '1'
    result = subprocess.run([sys.executable, str(PROBE), *map(str, arguments)], env=env,
                            capture_output=True, text=True, timeout=10)
    return result, dispatch


@pytest.mark.parametrize('arguments', [[], ['not-a-command'], ['drivers'],
    ['drivers', '--session', '../escape'], ['drivers', '--session', 'x', '--mode', 'triple'],
    ['drivers', '--session', 'x', '--duration', '0'], ['record', '--session', 'x', '--duration', '-1'],
    ['record', '--session', 'x', '--duration', '3601'], ['status', '--made-up-option'],
    ['calibrate', 'not-a-calibration-command']])
def test_invalid_cli_arguments_exit_nonzero_before_any_dispatch(tmp_path, arguments):
    result, dispatch = probe(tmp_path, *arguments)
    assert result.returncode != 0, result.stdout+result.stderr
    assert not dispatch.exists(), 'invalid CLI arguments reached a runtime/hardware launch boundary'


@pytest.mark.parametrize('arguments, expected', [(['--help'], 'drivers'),
    (['calibrate', '--help'], 'prepare-files'), (['calibrate', 'calibrate', '--help'], '--input'),
    (['save', '--help'], 'usage'), (['load', '--help'], 'usage')])
def test_help_is_forwarded_to_actual_subcommand_parser(tmp_path, arguments, expected):
    result, dispatch = probe(tmp_path, *arguments)
    assert result.returncode == 0, result.stdout+result.stderr
    assert expected.lower() in result.stdout.lower()
    assert not dispatch.exists(), '--help must not cross a launch or hardware boundary'


def test_repeated_session_role_preserves_original_plan_and_logs(tmp_path, monkeypatch):
    runtime = tmp_path/'runtime'
    directory = runtime/'sessions'/'existing-session'/'map'
    directory.mkdir(parents=True)
    original = {'plan.json': b'{"original": true}', 'supervisor.log': b'old supervisor evidence\n',
                'process-0.log': b'old component evidence\n'}
    for filename, payload in original.items():
        (directory/filename).write_bytes(payload)
    monkeypatch.setattr(cli, 'RUN', runtime)
    monkeypatch.setattr(cli.subprocess, 'Popen', lambda *a, **kw: pytest.fail('duplicate session attempted process launch'))
    with pytest.raises(RuntimeError, match='already exists|preserve'):
        cli.begin('existing-session', 'map', [[sys.executable, '-c', 'pass']], 1)
    assert {filename: (directory/filename).read_bytes() for filename in original} == original


def test_negative_mapping_duration_is_not_dispatched(tmp_path):
    # The CLI's source path guard remains active, so this input lives in a new
    # owned runtime folder. Only parser dispatch is recorded, never executed.
    directory = Path('/home/nvidia/wheelchair/.phase1_runtime/sessions')/('ops-cli-'+uuid.uuid4().hex)
    directory.mkdir(parents=True, exist_ok=False)
    config = directory/'synthetic-invalid-duration.json'
    config.write_text(json.dumps({'session_id': 'synthetic-invalid-duration', 'sensor_mode': 'dual',
                                 'source_mode': 'synthetic'}), encoding='utf-8')
    result, dispatch = probe(tmp_path, 'map', '--session-config', config, '--duration', '-1')
    assert result.returncode != 0, 'negative duration was reported as a successfully dispatched mapping session'
    assert not dispatch.exists(), 'negative duration reached supervisor launch'


def test_ros_command_keeps_user_arguments_out_of_shell_source():
    injection = "session-$(touch /tmp/WC_OPS_MUST_NEVER_EXIST);`echo data`"
    command = cli.ros_command(['echo', injection])
    shell_source=command[command.index('-c')+1]
    assert injection not in shell_source
    assert command[-1] == injection
    assert 'exec "$@"' in shell_source


def test_replay_allowlist_has_no_control_or_historical_pose_topics():
    assert '/wc_mapping/lidar_left/source_frame' in cli.TOPICS
    assert '/wc_mapping/lidar_right/source_frame' in cli.TOPICS
    assert '/wc_mapping/imu/source_frame' in cli.TOPICS
    assert not any(value in {'/tf', '/tf_static', '/cmd_vel', '/odom', '/wc_mapping/odom', '/navigate_to_pose'}
                   for value in cli.TOPICS)


def recording_fixture(tmp_path, monkeypatch):
    """Record launch arguments only; no process, ROS or device access occurs."""
    calls=[]
    monkeypatch.setattr(cli, 'ROOT', tmp_path)
    monkeypatch.setattr(cli, 'RUN', tmp_path/'.phase1_runtime')
    monkeypatch.setattr(cli, 'target', lambda: None)
    monkeypatch.setattr(cli, 'device_preflight', lambda: calls.append(('lidar_preflight',)))
    monkeypatch.setattr(cli.subprocess, 'Popen', lambda *a, **kw: pytest.fail('fixture must not start a process'))
    def capture(*args, **kwargs):
        calls.append(('begin', args, kwargs))
        return 0
    monkeypatch.setattr(cli, 'begin', capture)
    return calls


@pytest.mark.parametrize('failure', ['missing', 'identity_mismatch', 'occupied'])
def test_record_default_h30_failure_precedes_begin_and_all_sensor_scheduling(tmp_path, monkeypatch, capsys, failure):
    from wc_imu import ros_node as h30
    calls=recording_fixture(tmp_path, monkeypatch)
    observed=[]
    def identity(alias, by_id, serial):
        observed.append(('identity', alias, by_id, serial))
        if failure=='missing':raise FileNotFoundError('H30 expected by-id is missing')
        if failure=='identity_mismatch':raise h30.ImuAcquisitionError('H30 alias/by-id mismatch')
        return Path('/dev/TEST_ONLY_H30'), object()
    def occupancy(actual):
        observed.append(('occupancy', str(actual)))
        raise h30.ImuAcquisitionError('H30 device is already owned')
    monkeypatch.setattr(h30, 'verify_device_identity', identity)
    monkeypatch.setattr(h30, 'require_unoccupied', occupancy)
    monkeypatch.setattr(cli, 'driver_command', lambda *a, **kw: pytest.fail('sensor command constructed before H30 preflight passed'))
    assert cli.main(['record','--session','default-must-have-imu','--duration','5'])==2
    assert calls==[], 'no radar preflight or begin may occur after failed required H30 check'
    assert observed[0] == ('identity','/dev/smartwheel_h30_imu',
        '/dev/serial/by-id/usb-1a86_USB_Single_Serial_0000000015-if00','0000000015')
    assert len(observed)==(2 if failure=='occupied' else 1)
    assert not (tmp_path/'data/bags').exists()
    assert json.loads(capsys.readouterr().err)['status']=='ERROR'


def test_record_lidar_only_keeps_both_lidars_and_bag_without_any_serial_check(tmp_path, monkeypatch):
    from wc_imu import ros_node as h30
    calls=recording_fixture(tmp_path, monkeypatch)
    original_topics=list(cli.TOPICS)
    monkeypatch.setattr(h30, 'verify_device_identity', lambda *a: pytest.fail('lidar-only may not inspect an IMU alias'))
    monkeypatch.setattr(h30, 'require_unoccupied', lambda *a: pytest.fail('lidar-only may not inspect a serial owner'))
    monkeypatch.setattr(h30.ReadOnlySerialLease, 'open', lambda *a: pytest.fail('lidar-only may not open serial'))
    assert cli.main(['record','--session','explicit-dual-only','--duration','5','--lidar-only'])==0
    assert [row[0] for row in calls]==['lidar_preflight','begin']
    _, args, kwargs=calls[-1]
    commands=args[2]
    assert len(commands)==2, 'only a bag recorder and dual lidar launch are expected'
    assert 'source_mode:=dual' in commands[1] and 'dual_sources.launch.py' in commands[1]
    assert commands[0][commands[0].index('wc_phase1')+1:][:3]==['ros2','bag','record']
    assert all('wc_imu.ros_node' not in command for command in commands)
    flat=[token for command in commands for token in command]
    assert not any(token.startswith('/dev/') for token in flat)
    assert not any(token.startswith('/wc_mapping/imu/') for token in flat)
    for side in ('left','right'):
        assert '/wc_mapping/lidar_'+side+'/source_frame' in commands[0]
        assert '/wc_mapping/lidar_'+side+'/source_frame_filtered' in commands[0]
        assert '/wc_mapping/lidar_'+side+'/diagnostics' in commands[0]
    evidence=kwargs['recording']
    assert evidence['mode']=='lidar_only' and evidence['sensor_mode']=='dual'
    assert evidence['explicit_lidar_only'] is True and evidence['imu_included'] is False
    assert evidence['imu_preflight']['state']=='NOT_REQUESTED'
    assert evidence['topics']==[topic for topic in original_topics if not topic.startswith('/wc_mapping/imu/')]
    assert cli.TOPICS==original_topics, 'one explicit mode must not mutate later default recording behavior'


@pytest.mark.parametrize('mode',['single_left','single_right'])
def test_record_lidar_only_cannot_silently_become_single_lidar(tmp_path, monkeypatch, mode):
    calls=recording_fixture(tmp_path, monkeypatch)
    monkeypatch.setattr(cli, 'imu_preflight', lambda: pytest.fail('invalid combination should fail before any device lookup'))
    assert cli.main(['record','--session','invalid-single','--mode',mode,'--lidar-only'])==2
    assert calls==[]


def test_record_default_still_checks_and_starts_h30_with_exact_identity(tmp_path, monkeypatch):
    from wc_imu import ros_node as h30
    calls=recording_fixture(tmp_path, monkeypatch)
    actual=Path('/dev/TEST_ONLY_H30')
    def identity(alias, by_id, serial):
        assert (alias,by_id,serial)==(cli.H30_DEVICE,cli.H30_BY_ID,cli.H30_SERIAL)
        calls.append(('h30_identity',))
        return actual,object()
    def occupancy(value):
        assert value==actual
        calls.append(('h30_occupancy',))
    monkeypatch.setattr(h30, 'verify_device_identity', identity)
    monkeypatch.setattr(h30, 'require_unoccupied', occupancy)
    monkeypatch.setattr(h30.ReadOnlySerialLease, 'open', lambda *a: pytest.fail('preflight may not open the serial port'))
    assert cli.main(['record','--session','default-with-h30','--duration','5'])==0
    assert [row[0] for row in calls]==['h30_identity','h30_occupancy','lidar_preflight','begin']
    _,args,kwargs=calls[-1]
    commands=args[2]
    assert len(commands)==3
    assert 'source_mode:=dual' in commands[1]
    assert 'wc_imu.ros_node' in commands[2]
    for option,value in (('--device',cli.H30_DEVICE),('--expected-by-id',cli.H30_BY_ID),('--hardware-serial',cli.H30_SERIAL)):
        assert commands[2][commands[2].index(option)+1]==value
    assert all(topic in commands[0] for topic in cli.TOPICS)
    evidence=kwargs['recording']
    assert evidence['mode']=='lidar_with_h30' and evidence['imu_included'] is True
    assert evidence['explicit_lidar_only'] is False
    assert evidence['imu_preflight']['state']=='READ_ONLY_PREFLIGHT_PASS'
    assert evidence['imu_preflight']['resolved_device']==str(actual)
    assert evidence['imu_preflight']['serial_opened'] is False


def test_record_mode_evidence_is_written_before_supervisor_dispatch(tmp_path, monkeypatch):
    runtime=tmp_path/'.phase1_runtime'
    monkeypatch.setattr(cli,'RUN',runtime)
    monkeypatch.setattr(cli,'ROOT',tmp_path)
    evidence={'mode':'lidar_only','sensor_mode':'dual','imu_included':False,'explicit_lidar_only':True}
    def launch(*args, **kwargs):
        directory=runtime/'sessions'/'evidence-test'/'record'
        plan=json.loads((directory/'plan.json').read_text())
        assert plan['recording']==evidence
        (directory/'manifest.json').write_text(json.dumps({'state':'RUNNING','fixture':'no_actual_process'}))
        return type('FixtureSupervisor',(),{'poll':lambda self:None})()
    monkeypatch.setattr(cli.subprocess,'Popen',launch)
    assert cli.begin('evidence-test','record',[['TEST_BAG'],['TEST_DUAL_LIDAR']],8,
                     ['sensor_owner.lock','domain-83-source.lock'],True,recording=evidence)==0
