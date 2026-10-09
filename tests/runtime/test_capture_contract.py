"""Identity/readiness/window tests without ROS, serial, CV2 or real hardware."""
import copy
import json
from pathlib import Path
import shutil
import uuid

import pytest

from wc_runtime.capture_contract import (GAP_NS,build_capture_contract,collect_source_evidence,
    evaluate_capture_window,identity_matches,read_source_readiness,validate_contract)
from wc_cameras.node import camera_readiness_record,camera_stream_epoch


@pytest.fixture
def contract_root():
    parent=Path(__file__).resolve().parent
    root=parent/('.contract_test_'+uuid.uuid4().hex)
    root.mkdir()
    try: yield root
    finally:
        assert root.resolve().parent==parent and root.name.startswith('.contract_test_')
        shutil.rmtree(root)


def contract(kind='imu',logical='imu',source_id='imu'):
    return dict(schema_version=1,session_id='synthetic',profile='mapping_core',sources={logical:
        dict(kind=kind,source_id=source_id,max_gap_ns=GAP_NS[kind],expected_identity={})})


def dump(path,value):
    path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(json.dumps(value),encoding='utf-8')


def test_build_uses_frozen_serials_camera_usb_ports_and_separate_motor_addresses(contract_root):
    cfg=contract_root/'configuration'
    for side,serial in [('left','LEFT_SERIAL'),('right','RIGHT_SERIAL')]:
        path=cfg/'lidar'/(side+'.yaml');path.parent.mkdir(parents=True,exist_ok=True)
        path.write_text(f'side: {side}\nexpected_serial: {serial}\ndevice_ip: synthetic_{side}\nreceive_ip: synthetic\n',encoding='utf-8')
    dump(cfg/'wheel_feedback.json',dict(device_id='wheel',device='/synthetic',expected_by_id='/synthetic/id',
                                      hardware_serial='serial',usb_vid='vid',usb_pid='pid'))
    dump(cfg/'cameras.json',dict(mapping_status='USER_CONFIRMED',cameras=[dict(role=role,port=port,
        vid='vid',pid='pid',serial='SHARED_CAMERA_SERIAL') for role,port in
        [('left_front',1),('right_front',4),('left_side',2),('right_side',3)]]))
    dump(cfg/'ultrasonic.json',dict(device='/synthetic/ultra',expected_by_id='/synthetic/ultra-id',
        expected_usb_path='synthetic-usb',usb_vid='vid',usb_pid='pid',addresses=[1,2,3,4]))
    result=build_capture_contract(cfg,'all_sensors','synthetic',imu_sensor_id='imu')
    assert len(result['sources'])==13
    assert result['sources']['lidar_left']['source_id']=='LEFT_SERIAL'
    assert result['sources']['lidar_right']['source_id']=='RIGHT_SERIAL'
    assert result['sources']['wheel_left']['register_address']==0x20AB
    assert result['sources']['wheel_right']['register_address']==0x20AC
    assert result['sources']['wheel_left']['physical_role_status']=='LEGACY_MAPPING_UNVALIDATED'
    assert result['sources']['camera_right_front']['port']==4
    assert result['sources']['ultrasonic_address_4']['physical_role_status']=='UNKNOWN'


def test_readiness_reads_small_markers_and_checks_current_session_identity(contract_root):
    spec=contract()
    assert not read_source_readiness(contract_root,spec)['all_ready']
    value=dict(schema_version=1,session_id='synthetic',logical_source='imu',source_id='imu',stream_epoch='epoch',
               first_valid_monotonic_ns=1,last_valid_monotonic_ns=2,valid_count=2,ready=True,error=None)
    dump(contract_root/'ready/imu.json',value)
    assert read_source_readiness(contract_root,spec)['all_ready']
    for key,bad in [('session_id','other'),('source_id','other'),('valid_count',0),('ready',False),
                    ('last_valid_monotonic_ns',0),('error','failure')]:
        changed=dict(value);changed[key]=bad;dump(contract_root/'ready/imu.json',changed)
        assert read_source_readiness(contract_root,spec)['invalid']


@pytest.mark.parametrize('kind',list(GAP_NS))
def test_window_exact_gap_budget_and_common_coverage(kind):
    spec=contract(kind)
    if kind=='wheel':spec['sources']['imu']['register_index']=0
    if kind=='ultrasonic':spec['sources']['imu']['address']=1
    limit=GAP_NS[kind]
    evidence=dict(sources={'imu':dict(count=3,samples_monotonic_ns=[1,1+limit,1+2*limit],
        first_valid_monotonic_ns=1,last_valid_monotonic_ns=1+2*limit,issues=[])})
    result=evaluate_capture_window(evidence,spec,1,1+2*limit)
    assert result['window_complete'] and not result['physical_synchronization_verified']
    evidence['sources']['imu']['samples_monotonic_ns']=[1,2+limit,2+2*limit]
    result=evaluate_capture_window(evidence,spec,1,1+2*limit)
    assert not result['window_complete']
    assert any(row['code']=='MAX_SOURCE_GAP_EXCEEDED' for row in result['issues'])


def test_late_start_or_tail_before_window_end_never_passes():
    spec=contract()
    for stamps in ([2,3],[1,2],[]):
        evidence=dict(sources={'imu':dict(count=len(stamps),samples_monotonic_ns=stamps,issues=[])})
        assert not evaluate_capture_window(evidence,spec,1,3)['window_complete']


def test_source_epoch_identity_sequence_and_time_reorder_are_not_hidden(contract_root):
    directory=contract_root/'sources/imu';directory.mkdir(parents=True)
    rows=[dict(event='frame',sensor_id='imu',session_id='synthetic',stream_epoch='epoch',sequence=1,host_monotonic_ns=10),
          dict(event='frame',sensor_id='WRONG',session_id='foreign',stream_epoch='other',sequence=3,host_monotonic_ns=9)]
    (directory/'events.jsonl').write_text(''.join(json.dumps(row)+'\n' for row in rows),encoding='utf-8')
    evidence=collect_source_evidence(contract_root,contract())
    assert set(evidence['sources']['imu']['issues'])=={'SOURCE_IDENTITY_MISMATCH','SOURCE_SESSION_MISMATCH',
        'SOURCE_EPOCH_CHANGED','SOURCE_SEQUENCE_GAP_OR_REORDER','SOURCE_MONOTONIC_REORDER'}
    assert not evaluate_capture_window(evidence,contract(),9,10)['window_complete']


def test_identity_recursive_subset_retains_transport_metadata():
    expected=dict(device='/by-path/port1',identity=dict(ID_PATH='port1',ID_SERIAL_SHORT='shared'))
    actual=dict(expected,resolved_node='/dev/video8',st_rdev=1)
    assert identity_matches(expected,actual)
    changed=copy.deepcopy(actual);changed['identity']['ID_PATH']='port2'
    assert not identity_matches(expected,changed)


def test_camera_marker_uses_explicit_stream_epoch_and_native_sequence_not_8hz_preview():
    epoch='synthetic-'+'a'*32
    row=camera_readiness_record('synthetic','left_front',42,
        dict(capture_sequence=30,host_monotonic_ns=100),10,dict(device='/synthetic'),stream_epoch=epoch)
    assert row['stream_epoch']==epoch and row['worker_pid']==42 and row['valid_count']==30
    assert row['logical_source']=='camera_left_front' and row['source_id']=='left_front'
    assert row['ready'] and not row['persistence_verified']
    bad=camera_readiness_record('synthetic','left_front',42,
        dict(capture_sequence=31,host_monotonic_ns=110),10,dict(device='/synthetic'),
        stream_epoch=epoch,error='source failure')
    assert not bad['ready'] and bad['error']


@pytest.mark.parametrize('epoch',[None,'','foreign-'+'a'*32,'synthetic-','synthetic-../foreign',True])
def test_camera_marker_rejects_missing_foreign_or_malformed_actual_epoch(epoch):
    from wc_cameras.config import CameraError
    with pytest.raises(CameraError,match='readiness'):
        camera_readiness_record('synthetic','left_front',42,
            dict(capture_sequence=30,host_monotonic_ns=100),10,dict(device='/synthetic'),stream_epoch=epoch)


def test_camera_marker_requires_actual_epoch_instead_of_fallback_to_worker_pid():
    with pytest.raises(TypeError,match='stream_epoch'):
        camera_readiness_record('synthetic','left_front',42,
            dict(capture_sequence=30,host_monotonic_ns=100),10,dict(device='/synthetic'))


def test_camera_worker_ready_marker_frames_and_summary_share_actual_epoch(contract_root,monkeypatch):
    """One inert decoded frame exercises the real worker/archive; no V4L2/ROS."""
    from contextlib import nullcontext
    import sys
    import threading
    from types import SimpleNamespace
    from wc_cameras import capture as module
    from wc_runtime.source_archive import read_camera_frame
    stop=threading.Event();events=[];identity=dict(device='/synthetic',resolved_node='/synthetic')
    payload=b'abcdef'
    frame=SimpleNamespace(size=6,dtype=SimpleNamespace(name='uint8'),shape=(1,2,3),tobytes=lambda:payload)
    def read():stop.set();return True,frame
    cap=SimpleNamespace(isOpened=lambda:True,read=read,release=lambda:None)
    cv=SimpleNamespace(CAP_V4L2=200,VideoCapture=lambda *args:cap,setNumThreads=lambda count:None,
                       __version__='SYNTHETIC_NO_DEVICE')
    monkeypatch.setitem(sys.modules,'cv2',cv)
    monkeypatch.setattr(module,'arm_parent_death',lambda pid:None)
    monkeypatch.setattr(module,'CameraLease',lambda *args:nullcontext(identity))
    monkeypatch.setattr(module,'verify_device',lambda camera:identity)
    monkeypatch.setattr(module,'configure_capture',lambda *args:dict(width=2,height=1,fps=30))
    monkeypatch.setattr(module.os,'getpid',lambda:42)
    shared=SimpleNamespace(put=lambda *args:None)
    status=SimpleNamespace(send=events.append,close=lambda:None)
    root=contract_root/'sources/cameras/left_front'
    module.capture_worker(dict(role='left_front',rotate_deg=0),dict(width=2,height=1),contract_root,
                          'synthetic',shared,stop,status,1,str(root))
    ready=next(event for event in events if event['event']=='READY')
    archive=next(event['archive'] for event in events if event['event']=='ARCHIVE_COMPLETE')
    stored=json.loads((root/'frames.jsonl').read_text(encoding='utf-8'))
    diagnostic_epoch=camera_stream_epoch('synthetic',ready['stream_epoch'])
    marker=camera_readiness_record('synthetic','left_front',42,
        dict(capture_sequence=1,host_monotonic_ns=stored['host_monotonic_ns']),stored['host_monotonic_ns'],identity,
        stream_epoch=diagnostic_epoch)
    assert marker['stream_epoch']==diagnostic_epoch==stored['stream_epoch']==archive['stream_epoch']=='synthetic-42'
    assert marker['worker_pid']==42 and read_camera_frame(root,stored)==payload


def test_unreviewed_larger_gap_does_not_silently_relax_acceptance():
    spec=contract();spec['sources']['imu']['max_gap_ns']+=1
    with pytest.raises(ValueError):validate_contract(spec)


@pytest.mark.parametrize('fault',['all_nan','nan_tail','nan_gap','missing_count'])
def test_lidar_retains_every_sequence_but_nan_frames_do_not_establish_window(contract_root,fault):
    spec=contract('lidar','lidar_left','lidar');spec['sources']['lidar_left']['side']='left'
    stamps=[1_000_000_000+index*300_000_000 for index in range(4)]
    rows=[dict(sensor_id='lidar',stream_epoch='epoch',sequence=index,host_monotonic_ns=stamp,
               raw_valid_count=10,filtered_valid_count=5) for index,stamp in enumerate(stamps)]
    if fault=='all_nan':
        for row in rows:row['raw_valid_count']=row['filtered_valid_count']=0
    elif fault=='nan_tail':rows[-1]['filtered_valid_count']=0
    elif fault=='nan_gap':rows[1]['filtered_valid_count']=0
    else:rows[1].pop('raw_valid_count')
    path=contract_root/'sources/lidar/left/epoch/source_frames.jsonl';path.parent.mkdir(parents=True)
    path.write_text(''.join(json.dumps(row)+'\n' for row in rows),encoding='utf-8')
    evidence=collect_source_evidence(contract_root,spec);source=evidence['sources']['lidar_left']
    assert source['observed_count']==4 and source['unavailable_count']>0
    assert 'SOURCE_SEQUENCE_GAP_OR_REORDER' not in source['issues']
    result=evaluate_capture_window(evidence,spec,stamps[0]+1,stamps[-1])
    assert not result['window_complete']
    if fault=='nan_gap':assert any(row['code']=='MAX_SOURCE_GAP_EXCEEDED' for row in result['issues'])


def test_short_lidar_nan_warmup_is_retained_and_does_not_invent_transport_loss(contract_root):
    spec=contract('lidar','lidar_left','lidar');spec['sources']['lidar_left']['side']='left'
    stamps=[1_000_000_000+index*50_000_000 for index in range(4)]
    rows=[dict(sensor_id='lidar',stream_epoch='epoch',sequence=index,host_monotonic_ns=stamp,
               raw_valid_count=10,filtered_valid_count=0 if index==1 else 5) for index,stamp in enumerate(stamps)]
    path=contract_root/'sources/lidar/left/epoch/source_frames.jsonl';path.parent.mkdir(parents=True)
    path.write_text(''.join(json.dumps(row)+'\n' for row in rows),encoding='utf-8')
    evidence=collect_source_evidence(contract_root,spec)
    assert evidence['sources']['lidar_left']['observed_count']==4 and evidence['sources']['lidar_left']['count']==3
    assert evaluate_capture_window(evidence,spec,stamps[0]+1,stamps[-1])['window_complete']
