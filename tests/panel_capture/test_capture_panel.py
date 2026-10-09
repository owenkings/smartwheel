"""Pure capture-selection and closing-window tests; no ROS/device startup."""
import copy
import json
import os
from pathlib import Path
import sys
from types import ModuleType

import pytest

from wc_runtime import capture
from wc_runtime.capture_contract import build_capture_contract,user_stopped_window,GAP_NS
from wc_runtime.capture_preview import preview_plan
from wc_runtime.source_recorder import topics_for_profile

PROJECT=Path(__file__).resolve().parents[2]


@pytest.fixture(autouse=True)
def posix_import_stub(monkeypatch):
    if os.name=='nt':
        module=ModuleType('fcntl');module.LOCK_EX=2;module.LOCK_NB=4;module.LOCK_UN=8;module.flock=lambda *a:None
        monkeypatch.setitem(sys.modules,'fcntl',module)


def write(path,value):
    path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(json.dumps(value),encoding='utf-8')


@pytest.mark.parametrize('side,other',[('left','right'),('right','left')])
def test_single_lidar_selects_one_driver_contract_topic_and_preview(tmp_path,monkeypatch,side,other):
    from wc_runtime import cli
    monkeypatch.setattr(cli,'ros_command',lambda args:list(map(str,args)))
    cfg=tmp_path/'configuration'
    bindings=json.loads((PROJECT/'config/device_bindings.json').read_text(encoding='utf-8'))
    write(cfg/'device_bindings.json',bindings)
    write(cfg/'wheel_feedback.json',json.loads((PROJECT/'config/wheel_feedback_current.json').read_text()))
    (cfg/'lidar').mkdir()
    # The unselected side has no YAML at all: it may not be accessed.
    (cfg/'lidar'/(side+'.yaml')).write_text('side: '+side+'\nexpected_serial: SN_'+side+'\nframe_id: lidar_'+side+'\ndevice_ip: 192.0.2.10\nreceive_ip: 192.0.2.11\n')
    selected=capture.source_selection('mapping_core',side)
    assert 'lidar_'+side in selected['selected_sources']
    assert 'lidar_'+other in selected['excluded_sources']
    commands=capture.source_commands(PROJECT,PROJECT/'.phase1_runtime',tmp_path,'capture_test','mapping_core',sides=side)
    assert 'source_mode:=single_'+side in commands['lidar']
    contract=build_capture_contract(cfg,'mapping_core','capture_test',imu_sensor_id='test_imu',sides=side)
    assert set(contract['sources'])=={'lidar_'+side,'imu','wheel_left','wheel_right'}
    topics=topics_for_profile('mapping_core',side)
    assert '/wc_mapping/lidar_'+side+'/source_frame' in topics
    assert not any('lidar_'+other in topic for topic in topics)
    write(cfg/'source_selection.json',selected)
    write(tmp_path/'capture_manifest.json',dict(session_id='capture_test',status='RECORDING',started_wall_ns=100,started_monotonic_ns=100))
    write(tmp_path/'recorder_ready.json',dict(ready=True,host_monotonic_ns=200))
    plan=preview_plan(tmp_path,'capture_test')
    assert set(plan['lidars'])=={side}


def test_unified_manual_capture_uses_one_reviewed_control_owner(tmp_path,monkeypatch):
    from wc_runtime import cli
    monkeypatch.setattr(cli,'ros_command',lambda args:list(map(str,args)))
    write(tmp_path/'configuration/device_bindings.json',json.loads((PROJECT/'config/device_bindings.json').read_text()))
    commands=capture.source_commands(PROJECT,PROJECT/'.phase1_runtime',tmp_path,'capture_test','mapping_core',manual_drive=True,preview_layout='unified')
    assert 'manual_ui' not in commands
    assert 'wc_runtime.mapping_wheel' in commands['wheel']
    assert not any('wc_motion.feedback_transport' in command for command in commands.values())


def contract_and_readiness():
    contract=dict(schema_version=1,session_id='test',profile='mapping_core',sources={
        'imu':dict(kind='imu',source_id='imu',max_gap_ns=GAP_NS['imu'],expected_identity={}),
        'lidar_left':dict(kind='lidar',source_id='left',max_gap_ns=GAP_NS['lidar'],expected_identity={})})
    ready=dict(all_ready=True,sources={
        'imu':dict(last_valid_monotonic_ns=9_900_000_000),
        'lidar_left':dict(last_valid_monotonic_ns=9_800_000_000)})
    return contract,ready


def test_unlimited_window_freezes_observed_end_with_tail_evidence():
    contract,ready=contract_and_readiness()
    value=user_stopped_window(contract,ready,1_000_000_000,10_000_000_000)
    assert value['end_monotonic_ns']==9_800_000_000
    assert value['tail_trim_ns']==200_000_000
    assert value['requested_duration_ns']==8_800_000_000
    assert value['policy']=='UNTIL_USER_STOP_OBSERVED_COMMON_WINDOW'


@pytest.mark.parametrize('last',[1_100_000_000,10_100_000_000])
def test_unlimited_stop_rejects_stalled_or_future_source(last):
    contract,ready=contract_and_readiness()
    ready['sources']['imu']['last_valid_monotonic_ns']=last
    with pytest.raises(ValueError,match='SOURCE_STALE_AT_USER_STOP'):
        user_stopped_window(contract,ready,1_000_000_000,10_000_000_000)


def test_unlimited_capacity_is_explicit_startup_estimate_and_single_lidar_rate():
    dual=capture.capacity('mapping_core',0,10**12,initialization_budget_s=60)
    single=capture.capacity('mapping_core',0,10**12,initialization_budget_s=60,sides='left')
    assert dual['user_stopped_duration']
    assert dual['estimate_scope']=='STARTUP_ONLY_UNBOUNDED_CAPTURE'
    assert single['estimated_recording_bytes']<dual['estimated_recording_bytes']
    with pytest.raises(ValueError):capture.capacity('mapping_core',-1,10**12)
