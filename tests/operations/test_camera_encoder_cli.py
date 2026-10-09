"""Dispatch tests: identities, independent roles and feedback opt-in stay explicit."""
import json
from pathlib import Path
import pytest
from wc_runtime import cli

PROJECT = Path(__file__).resolve().parents[2]


def dispatch_fixture(monkeypatch, tmp_path):
    (tmp_path/'config').mkdir()
    for filename in ('cameras.json', 'wheel_feedback_current.json'):
        (tmp_path/'config'/filename).write_bytes((PROJECT/'config'/filename).read_bytes())
    lidar=tmp_path/'src/wc_xt_driver/config'
    lidar.mkdir(parents=True)
    for filename in ('left.yaml','right.yaml','left.xtcfg','right.xtcfg',
                     'left-2026-09-11.xtcfg','right-2026-09-11.xtcfg'):
        (lidar/filename).write_text('synthetic: true')
    monkeypatch.setattr(cli, 'ROOT', tmp_path)
    monkeypatch.setattr(cli, 'RUN', tmp_path/'.phase1_runtime')
    monkeypatch.setattr(cli, 'target', lambda: None)
    monkeypatch.setattr(cli, 'scoped_path', lambda path: tmp_path/Path(path))
    calls=[]
    monkeypatch.setattr(cli, 'begin', lambda *a, **kw: calls.append((a,kw)) or 0)
    monkeypatch.setattr(cli.subprocess, 'Popen', lambda *a, **kw: pytest.fail('no process allowed in dispatch fixture'))
    return calls


def test_four_slots_each_have_their_own_camera_process(monkeypatch, tmp_path):
    calls=dispatch_fixture(monkeypatch,tmp_path)
    assert cli.main(['cameras','--session','test','--duration','60','--profile','detail_640'])==0
    args,_=calls[0]
    assert args[1]=='cameras' and len(args[2])==4
    roles=[]
    for command in args[2]:
        assert 'wc_cameras.node' in command
        roles.append(command[command.index('--role')+1])
        assert command[command.index('--profile')+1]=='detail_640'
        assert command[command.index('--run-root')+1]==str(tmp_path/'.phase1_runtime')
    assert roles==['left_front','right_front','left_side','right_side']


@pytest.mark.parametrize('command', [['encoder','--session','x'],
    ['encoder','--session','x','--allow-read-queries','--duration','301'],
    ['cameras','--session','x','--duration','0']])
def test_missing_optin_or_invalid_duration_cannot_start_a_device(monkeypatch,tmp_path,command):
    calls=dispatch_fixture(monkeypatch,tmp_path)
    assert cli.main(command)==2
    assert calls==[]


def test_encoder_preview_is_explicit_and_distinct_from_validated_wheel_odometry(monkeypatch,tmp_path):
    calls=dispatch_fixture(monkeypatch,tmp_path)
    assert cli.main(['encoder','--session','x','--allow-read-queries','--duration','20',
                     '--preview-history-calibration'])==0
    args,_=calls[0]
    command=args[2][0]
    assert args[1]=='encoder' and 'wc_motion.feedback_transport' in command
    assert '--preview-history-calibration' in command
    assert command[command.index('--samples')+1]=='200'
    assert not any('cmd_vel' in token or 'manual_teleop' in token for token in command)


def test_h30_poll_period_is_recorded_and_forwarded(monkeypatch,tmp_path):
    calls=dispatch_fixture(monkeypatch,tmp_path)
    monkeypatch.setattr(cli,'device_preflight',lambda:None)
    monkeypatch.setattr(cli,'imu_preflight',lambda:{'serial_opened':False})
    assert cli.main(['record','--session','x','--duration','20','--imu-poll-period-ms','2.5'])==0
    args,kwargs=calls[0]
    imu=next(command for command in args[2] if 'wc_imu.ros_node' in command)
    assert imu[imu.index('--poll-period-ms')+1]=='2.5'
    assert kwargs['recording']['imu_poll_period_ms']==2.5
    assert not any(topic.endswith('image_raw') for topic in kwargs['recording']['topics'])
