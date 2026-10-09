"""Independent source/index/bag + strict identity/drop/window/durability audit."""
from contextlib import closing
import hashlib
import json
from pathlib import Path
import shutil
import sqlite3
import uuid

import pytest

from wc_runtime.capture_audit import audit_capture
from wc_runtime.capture_contract import GAP_NS
from wc_runtime.source_archive import CameraArchive,atomic_json,digest


@pytest.fixture
def audit_root():
    parent=Path(__file__).resolve().parent
    root=parent/('.audit_contract_test_'+uuid.uuid4().hex)
    root.mkdir()
    try: yield root
    finally:
        assert root.resolve().parent==parent and root.name.startswith('.audit_contract_test_')
        shutil.rmtree(root)


def write_rows(path,rows):
    path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(''.join(json.dumps(row)+'\n' for row in rows),encoding='utf-8')


def fixture_capture(root):
    records=[];stamps=[1_000_000_000+i*50_000_000 for i in range(6)]
    specs={}
    for side in ('left','right'):
        specs['lidar_'+side]=dict(kind='lidar',source_id=side,side=side,max_gap_ns=GAP_NS['lidar'],expected_identity={})
        rows=[]
        for sequence,stamp in enumerate(stamps):
            rows.append(dict(sensor_id=side,stream_epoch='epoch',sequence=sequence,
                             host_monotonic_ns=stamp,raw_valid_count=1,filtered_valid_count=1,
                             raw_sha256='raw'+side,filtered_sha256='filtered'+side))
            for representation in ('raw','filtered'):
                records.append(dict(source_id=side,stream_epoch='epoch',sequence=sequence,representation=representation,
                    payload_sha256=representation+side,topic='/lidar_'+side+'/'+representation))
        directory=root/'sources/lidar'/side/'epoch'
        write_rows(directory/'source_frames.jsonl',rows)
        pipeline=dict(callback_received=6,accepted_frames=6,rejected_frames=0,rejected_bytes=0,
                      pending_frames=0,pending_bytes=0,worker_processed=6,worker_journaled=6,worker_published=6,
                      failure_code=None,shutdown_deadline_exceeded=False,publication_acknowledged=True,
                      raw_publication_acknowledged=True,filtered_publication_acknowledged=True)
        atomic_json(directory/'source_summary.json',dict(sensor_id=side,side=side,synchronized=True,closed_normally=True,
            published_frames=6,journal_frames=6,pipeline=pipeline,
            sdk_queues=dict(raw_dropped=0,image_dropped=0,raw_pending_frames=0,raw_pending_bytes=0,
                            image_pending_frames=0,image_pending_bytes=0,strict_delivery=True)))
    specs['imu']=dict(kind='imu',source_id='imu',max_gap_ns=GAP_NS['imu'],expected_identity={})
    events=[dict(event='byte_batch',batch_id=1,bytes_hex='01',sha256=hashlib.sha256(b'\x01').hexdigest())]
    for sequence,stamp in enumerate(stamps):
        events.append(dict(event='frame',batch_id=1,sensor_id='imu',stream_epoch='epoch',sequence=sequence,
                           host_monotonic_ns=stamp,packet_sha256='packet'))
        records.append(dict(source_id='imu',stream_epoch='epoch',sequence=sequence,representation='packet',
                            payload_sha256='packet',topic='/imu'))
    directory=root/'sources/imu'
    write_rows(directory/'events.jsonl',events)
    atomic_json(directory/'summary.json',dict(synchronized=True,closed_normally=True,event_counts=dict(frame=6),
                                             events_sha256=digest(directory/'events.jsonl')))
    wheel_hash=hashlib.sha256(bytes.fromhex('01030400000000fa33')).hexdigest()
    transactions=[]
    for sequence,stamp in enumerate(stamps):
        transactions.append(dict(event='transaction_complete',device_id='wheel',stream_epoch='epoch',sequence=sequence,
            receive_monotonic_ns=stamp,response_sha256=wheel_hash,status='RESPONSE_VALID',request_hex='010320ab0002be2b',
            response_hex='01030400000000fa33',register_words_u16=[0,0]))
        records.append(dict(source_id='wheel',stream_epoch='epoch',sequence=sequence,representation='transaction',
                            payload_sha256=wheel_hash,topic='/wheel'))
    for index,side in enumerate(('left','right')):
        specs['wheel_'+side]=dict(kind='wheel',source_id='wheel',side=side,register_index=index,register_address=0x20AB+index,
                                 max_gap_ns=GAP_NS['wheel'],expected_identity={})
    write_rows(root/'sources/wheel_feedback.jsonl',transactions+[dict(event='capture_complete',completed=6),
        dict(event='lease_closed'),dict(event='publication_ack',acknowledged=True,timeout_s=5.0)])
    atomic_json(root/'sources/wheel_summary.json',dict(synchronized=True,closed_normally=True,completed=6,
        control_transmissions=0,journal=dict(final_fsync_complete=True),events_sha256=digest(root/'sources/wheel_feedback.jsonl')))
    (root/'bag').mkdir()
    (root/'bag/metadata.yaml').write_text('closed',encoding='utf-8')
    with closing(sqlite3.connect(root/'bag/bag_0.db3')) as conn,conn:
        conn.execute('CREATE TABLE topics(id INTEGER,name TEXT)')
        conn.execute('CREATE TABLE messages(topic_id INTEGER,timestamp INTEGER,data BLOB)')
        for index,row in enumerate(records):
            cdr=index.to_bytes(4,'little')
            row.update(storage_stamp_ns=index,cdr_sha256=hashlib.sha256(cdr).hexdigest())
            conn.execute('INSERT INTO topics VALUES (?,?)',(index,row['topic']))
            conn.execute('INSERT INTO messages VALUES (?,?,?)',(index,index,cdr))
    write_rows(root/'records.jsonl',records)
    atomic_json(root/'recorder_summary.json',dict(synchronized=True,closed_normally=True,
        received=len(records),persisted=len(records),index_sha256=digest(root/'records.jsonl')))
    contract=dict(schema_version=1,session_id='synthetic',profile='mapping_core',sources=specs)
    (root/'configuration').mkdir()
    atomic_json(root/'configuration/capture_contract.json',contract)
    manifest=dict(session_id='synthetic',profile='mapping_core',capture_contract_required=True,
        acquisition_window=dict(start_monotonic_ns=stamps[1],end_monotonic_ns=stamps[-2],requested_duration_ns=150_000_000),
        input_hashes={'configuration/capture_contract.json':digest(root/'configuration/capture_contract.json')})
    atomic_json(root/'capture_manifest.json',manifest)
    return dict.fromkeys(('lidar','imu','wheel','recorder'),0)


def test_new_contract_requires_transport_window_and_final_durability(audit_root):
    codes=fixture_capture(audit_root)
    pending=audit_capture(audit_root,'mapping_core',codes)
    assert pending['transport_complete'] and pending['window_complete']
    assert not pending['recording_complete'] and pending['completion_pending']==['durability']
    assert pending['frequency_diagnostics']['map_consumption_hz'] is None and pending['frequency_diagnostics']['display_hz'] is None
    assert pending['frequency_diagnostics']['display_rate_basis']=='NOT_MEASURED'
    assert pending['frequency_diagnostics']['preview_status_path']=='capture_preview_status.json'
    assert 'starts no SLAM' in pending['frequency_diagnostics']['inactive_reason']
    assert audit_capture(audit_root,'mapping_core',codes,durability_complete=True)['recording_complete']
    assert not audit_capture(audit_root,'mapping_core',codes,durability_complete=False)['recording_complete']


@pytest.mark.parametrize('missing_window',['absent','null','empty'])
def test_pre_readiness_manual_failure_has_no_established_window(audit_root,monkeypatch,missing_window):
    """The failed manual startup manifest has no frozen acquisition endpoints."""
    codes=fixture_capture(audit_root)
    codes.update(wheel=1,manual_ui='NOT_STARTED')
    path=audit_root/'capture_manifest.json'
    manifest=json.loads(path.read_text(encoding='utf-8'))
    manifest.pop('acquisition_window')
    manifest.update(schema_version=2,status='PARTIAL',recording_complete=False,
        manual_drive=True,diagnostic_capture=False,requested_source_duration_s=300,
        process_exit_codes=codes,stop_reason='CAPTURE_FAILURE',
        control_authorization='EXPLICIT_CAPTURE_MANUAL_DRIVE_ARGUMENT',
        issues=[dict(code='CAPTURE_INTERRUPTED',evidence='logs',
                     detail='RuntimeError: MANUAL_WHEEL_NOT_READY; UI not started')])
    if missing_window!='absent':
        manifest['acquisition_window']=None if missing_window=='null' else {}
    atomic_json(path,manifest)
    wheel_log=audit_root/'logs/wheel.log'
    wheel_log.parent.mkdir()
    wheel_log.write_text('PermissionError: manual wheel setup failed before readiness\n',encoding='utf-8')
    from wc_runtime import capture_contract
    def forbidden(*args,**kwargs):
        raise AssertionError('a common window that was never established cannot be evaluated')
    monkeypatch.setattr(capture_contract,'collect_source_evidence',forbidden)
    result=audit_capture(audit_root,'mapping_core',codes,durability_complete=True)
    assert result['status']=='PARTIAL' and not result['recording_complete']
    assert result['window_complete'] is False and result['full_window_verified'] is False
    failures=[row for row in result['issues'] if row['code'].startswith('CAPTURE_WINDOW_')]
    assert [row['code'] for row in failures]==['CAPTURE_WINDOW_NOT_ESTABLISHED']
    assert 'verification was not run' in failures[0]['detail']
    assert 'duration differs' not in failures[0]['detail']
    assert result['window_diagnostics']['evaluated'] is False
    assert result['window_diagnostics']['requested_source_duration_s']==300
    assert result['window_diagnostics']['stop_reason']=='CAPTURE_FAILURE'
    assert 'start_monotonic_ns' not in result['window_diagnostics']
    assert json.loads(path.read_text(encoding='utf-8'))==manifest
    assert wheel_log.read_text(encoding='utf-8').startswith('PermissionError')


@pytest.mark.parametrize('fault',['non_object','missing_endpoint','bool_endpoint','duration_mismatch',
                                 'requested_duration_mismatch','zero_start','empty_interval'])
def test_established_window_invalid_format_or_endpoints_remain_rejected(audit_root,fault):
    codes=fixture_capture(audit_root)
    path=audit_root/'capture_manifest.json'
    manifest=json.loads(path.read_text(encoding='utf-8'))
    window=manifest['acquisition_window']
    if fault=='non_object': manifest['acquisition_window']=[]
    elif fault=='missing_endpoint':window.pop('start_monotonic_ns')
    elif fault=='bool_endpoint':window['start_monotonic_ns']=True
    elif fault=='duration_mismatch':window['requested_duration_ns']+=1
    elif fault=='requested_duration_mismatch':manifest['requested_source_duration_s']=1
    elif fault=='zero_start':
        window['start_monotonic_ns']=0
        window['requested_duration_ns']=window['end_monotonic_ns']
    elif fault=='empty_interval':
        window['end_monotonic_ns']=window['start_monotonic_ns']
        window['requested_duration_ns']=0
    atomic_json(path,manifest)
    result=audit_capture(audit_root,'mapping_core',codes,durability_complete=True)
    assert result['status']=='PARTIAL' and result['window_complete'] is False
    assert not result['recording_complete'] and not result['full_window_verified']
    failures=[row for row in result['issues'] if row['code'].startswith('CAPTURE_WINDOW_')]
    assert [row['code'] for row in failures]==['CAPTURE_WINDOW_EVIDENCE_INVALID']
    if fault=='duration_mismatch':
        assert failures[0]['detail']=='requested window duration differs from frozen endpoints'
    elif fault=='requested_duration_mismatch':
        assert failures[0]['detail']=='acquisition window differs from original requested duration'


@pytest.mark.parametrize('fault',['sdk_drop','sdk_pending','publication_ack','pipeline_reject','pipeline_count','side','sensor_id','epoch',
                                 'window_tail','window_gap','missing_contract','hash_changed','missing_logical'])
def test_faults_cannot_pass_even_when_bag_bytes_match(audit_root,fault):
    codes=fixture_capture(audit_root)
    summary_path=audit_root/'sources/lidar/left/epoch/source_summary.json'
    if fault in ('sdk_drop','sdk_pending','publication_ack','pipeline_reject','pipeline_count','side','sensor_id'):
        value=json.loads(summary_path.read_text(encoding='utf-8'))
        if fault=='sdk_drop':value['sdk_queues']['raw_dropped']=1
        elif fault=='sdk_pending':value['sdk_queues']['image_pending_frames']=1
        elif fault=='publication_ack':value['pipeline']['filtered_publication_acknowledged']=False
        elif fault=='pipeline_reject':value['pipeline']['rejected_frames']=1
        elif fault=='pipeline_count':value['pipeline']['worker_processed']=5
        elif fault=='side':value['side']='right'
        else:value['sensor_id']='foreign'
        atomic_json(summary_path,value)
    elif fault=='epoch':
        path=audit_root/'sources/lidar/left/epoch/source_frames.jsonl'
        values=[json.loads(line) for line in path.read_text(encoding='utf-8').splitlines()]
        values[2]['stream_epoch']='other'
        write_rows(path,values)
    elif fault.startswith('window_'):
        path=audit_root/'capture_manifest.json';value=json.loads(path.read_text(encoding='utf-8'))
        if fault=='window_tail':
            value['acquisition_window']['end_monotonic_ns']=2_000_000_000
            value['acquisition_window']['requested_duration_ns']=2_000_000_000-value['acquisition_window']['start_monotonic_ns']
        else:
            path2=audit_root/'sources/imu/events.jsonl'
            values=[json.loads(line) for line in path2.read_text(encoding='utf-8').splitlines()]
            values[3]['host_monotonic_ns']+=200_000_000
            write_rows(path2,values)
            summary=json.loads((audit_root/'sources/imu/summary.json').read_text(encoding='utf-8'))
            summary['events_sha256']=digest(path2);atomic_json(audit_root/'sources/imu/summary.json',summary)
        atomic_json(path,value)
    elif fault=='missing_contract':(audit_root/'configuration/capture_contract.json').unlink()
    else:
        path=audit_root/'configuration/capture_contract.json';value=json.loads(path.read_text(encoding='utf-8'))
        if fault=='missing_logical':value['sources'].pop('wheel_right')
        else:value['sources']['imu']['max_gap_ns']-=1
        atomic_json(path,value)
    result=audit_capture(audit_root,'mapping_core',codes,durability_complete=True)
    assert not result['recording_complete'] and result['issues']


def test_optional_renderer_exit_does_not_invalidate_sources(audit_root):
    codes=fixture_capture(audit_root);codes['preview']=1
    assert audit_capture(audit_root,'mapping_core',codes,durability_complete=True)['recording_complete']


def test_legacy_archive_never_claims_new_full_window_verification(audit_root):
    codes=fixture_capture(audit_root)
    (audit_root/'configuration/capture_contract.json').unlink()
    atomic_json(audit_root/'capture_manifest.json',dict(session_id='synthetic',profile='mapping_core'))
    result=audit_capture(audit_root,'mapping_core',codes)
    assert result['recording_complete'] and result['transport_complete']
    assert result['window_complete'] is None and not result['full_window_verified']
    assert result['completeness_contract']=='LEGACY_CAPTURED_BYTES_ONLY'


def fixture_cameras(root):
    codes=fixture_capture(root)
    contract_path=root/'configuration/capture_contract.json'
    contract=json.loads(contract_path.read_text(encoding='utf-8'));contract['profile']='mapping_cameras'
    for role in ('left_front','right_front','left_side','right_side'):
        directory=root/'sources/cameras'/role
        archive=CameraArchive(directory,role,'epoch-camera')
        for sequence in range(1,7):
            archive.append(bytes([sequence])*12,width=2,height=2,sequence=sequence,
                           host_monotonic_ns=1_000_000_000+(sequence-1)*50_000_000)
        archive.close()
        transport=dict(device='/synthetic/'+role,identity=dict(ID_PATH=role))
        atomic_json(directory/'capture_configuration.json',dict(camera=dict(role=role),identity=transport))
        contract['sources']['camera_'+role]=dict(kind='camera',role=role,source_id=role,
            max_gap_ns=GAP_NS['camera'],expected_identity=transport)
        codes['camera_'+role]=0
    atomic_json(contract_path,contract)
    manifest_path=root/'capture_manifest.json';manifest=json.loads(manifest_path.read_text(encoding='utf-8'))
    manifest.update(profile='mapping_cameras',input_hashes={'configuration/capture_contract.json':digest(contract_path)})
    atomic_json(manifest_path,manifest)
    return codes


def test_compressed_camera_audit_decodes_original_bytes_and_matches_transport(audit_root):
    result=audit_capture(audit_root,'mapping_cameras',fixture_cameras(audit_root),durability_complete=True)
    assert result['recording_complete'] and result['window_complete'] and result['transport_complete']
    assert result['source_accounting']['camera_left_front']['retained']==6
    assert result['source_accounting']['camera_left_front']['exposure_completeness']=='NOT_VERIFIED'


@pytest.mark.parametrize('fault',['camera_port','camera_role','camera_source','camera_epoch','compressed_bytes','decoded_hash'])
def test_compressed_camera_or_role_corruption_never_passes(audit_root,fault):
    codes=fixture_cameras(audit_root);directory=audit_root/'sources/cameras/left_front'
    if fault in ('camera_port','camera_role'):
        path=directory/'capture_configuration.json';value=json.loads(path.read_text(encoding='utf-8'))
        if fault=='camera_port':value['identity']['identity']['ID_PATH']='foreign-port'
        else:value['camera']['role']='right_front'
        atomic_json(path,value)
    else:
        path=directory/'frames.jsonl';values=[json.loads(line) for line in path.read_text(encoding='utf-8').splitlines()]
        summary=json.loads((directory/'summary.json').read_text(encoding='utf-8'))
        if fault=='camera_source':values[0]['source_id']='right_front'
        elif fault=='camera_epoch':values[0]['stream_epoch']='foreign-epoch'
        elif fault=='decoded_hash':values[0]['payload_sha256']='0'*64
        else:
            chunk=directory/values[0]['file'];data=bytearray(chunk.read_bytes());data[0]^=1;chunk.write_bytes(data)
            summary['chunks'][0]['sha256']=digest(chunk)
        write_rows(path,values)
        summary['index_sha256']=digest(path);atomic_json(directory/'summary.json',summary)
    result=audit_capture(audit_root,'mapping_cameras',codes,durability_complete=True)
    assert not result['recording_complete'] and result['issues']


def fixture_ultrasonic(root):
    from wc_runtime.ultrasonic_capture import request,crc16
    codes=fixture_cameras(root);codes['ultrasonic']=0
    contract_path=root/'configuration/capture_contract.json'
    contract=json.loads(contract_path.read_text(encoding='utf-8'));contract['profile']='all_sensors'
    expected=dict(device='/synthetic/ultra',expected_by_id='/synthetic/by-id/ultra',expected_usb_path='synthetic-usb',
                  usb_vid='1a86',usb_pid='7523')
    for address in (1,2,3,4):
        contract['sources']['ultrasonic_address_'+str(address)]=dict(kind='ultrasonic',source_id='ultrasonic',
            address=address,max_gap_ns=GAP_NS['ultrasonic'],expected_identity=expected,physical_role_status='UNKNOWN')
    events=[];new_records=[]
    for batch in range(6):
        for address in (1,2,3,4):
            sequence=batch*4+address-1
            body=bytes([address,3,2,0,42]);response=body+crc16(body).to_bytes(2,'little')
            response_hash=hashlib.sha256(response).hexdigest()
            events.append(dict(event='transaction',sensor_id='ultrasonic',session_id='synthetic',stream_epoch='ultra-epoch',
                sequence=sequence,address=address,host_monotonic_ns=1_000_000_000+batch*50_000_000,
                request_hex=request(address).hex(),response_hex=response.hex(),response_sha256=response_hash,
                status='VALID_RESPONSE',response_value_u16=42))
            new_records.append(dict(source_id='ultrasonic',stream_epoch='ultra-epoch',sequence=sequence,
                representation='transaction',payload_sha256=response_hash,topic='/ultrasonic'))
    events.append(dict(event='source_finalization',session_id='synthetic',stream_epoch='ultra-epoch',
        completed_transactions=24,published_transactions=24,
        publication_ack=dict(status='PASS',acknowledged=True,timeout_s=5.),lease_closed=True,cleanup_errors=[]))
    directory=root/'sources/ultrasonic';write_rows(directory/'events.jsonl',events)
    atomic_json(directory/'summary.json',dict(synchronized=True,closed_normally=True,
        accepted_records=25,persisted_records=25,rejected_records=0,event_counts=dict(transaction=24,source_finalization=1),
        events_sha256=digest(directory/'events.jsonl')))
    atomic_json(directory/'identity.json',dict(expected,schema_version=1,source_id='ultrasonic',session_id='synthetic',
        opened_identity=dict(st_rdev=1,st_dev=2,st_ino=3),verified_monotonic_ns=1,
        evidence_basis='CURRENT_UDEV_AND_OPEN_DESCRIPTOR_IDENTITY'))
    records=[json.loads(line) for line in (root/'records.jsonl').read_text(encoding='utf-8').splitlines()]
    with closing(sqlite3.connect(root/'bag/bag_0.db3')) as conn,conn:
        for index,row in enumerate(new_records,len(records)):
            cdr=index.to_bytes(4,'little');row.update(storage_stamp_ns=index,cdr_sha256=hashlib.sha256(cdr).hexdigest())
            conn.execute('INSERT INTO topics VALUES (?,?)',(index,row['topic']))
            conn.execute('INSERT INTO messages VALUES (?,?,?)',(index,index,cdr))
    records.extend(new_records);write_rows(root/'records.jsonl',records)
    atomic_json(root/'recorder_summary.json',dict(synchronized=True,closed_normally=True,received=len(records),
        persisted=len(records),index_sha256=digest(root/'records.jsonl')))
    atomic_json(contract_path,contract)
    manifest_path=root/'capture_manifest.json';manifest=json.loads(manifest_path.read_text(encoding='utf-8'))
    manifest.update(profile='all_sensors',input_hashes={'configuration/capture_contract.json':digest(contract_path)})
    atomic_json(manifest_path,manifest)
    return codes


@pytest.mark.parametrize('fault',[None,'missing_actual','wrong_bus','foreign_session'])
def test_all_sensors_requires_actual_ultrasonic_identity_and_distinct_address_windows(audit_root,fault):
    codes=fixture_ultrasonic(audit_root)
    path=audit_root/'sources/ultrasonic/identity.json'
    if fault=='missing_actual':path.unlink()
    elif fault:
        value=json.loads(path.read_text(encoding='utf-8'))
        value['expected_usb_path' if fault=='wrong_bus' else 'session_id']='foreign'
        atomic_json(path,value)
    result=audit_capture(audit_root,'all_sensors',codes,durability_complete=True)
    assert result['recording_complete']==(fault is None)
    if fault is None:
        for address in (1,2,3,4):
            assert result['window_diagnostics']['sources']['ultrasonic_address_'+str(address)]['count']==6
    else:assert result['issues']


@pytest.mark.parametrize('fault',['missing','duplicate','not_tail','completed','published','lease','ack',
                                 'cleanup','epoch','session','journal_counts'])
def test_ultrasonic_finalization_is_separate_from_transactions_and_required_for_new_contract(audit_root,fault):
    codes=fixture_ultrasonic(audit_root);directory=audit_root/'sources/ultrasonic'
    events=[json.loads(line) for line in (directory/'events.jsonl').read_text(encoding='utf-8').splitlines()]
    final=events[-1]
    if fault=='missing':events.pop()
    elif fault=='duplicate':events.append(dict(final))
    elif fault=='not_tail':events.insert(0,events.pop())
    elif fault in ('completed','published'):final[fault+'_transactions']-=1
    elif fault=='lease':final['lease_closed']=False
    elif fault=='ack':final['publication_ack']['acknowledged']=False
    elif fault=='cleanup':final['cleanup_errors']=['injected close failure']
    elif fault=='epoch':final['stream_epoch']='foreign'
    elif fault=='session':final['session_id']='foreign'
    write_rows(directory/'events.jsonl',events)
    summary=json.loads((directory/'summary.json').read_text(encoding='utf-8'))
    summary['events_sha256']=digest(directory/'events.jsonl')
    if fault=='journal_counts':summary['event_counts']['transaction']-=1
    atomic_json(directory/'summary.json',summary)
    result=audit_capture(audit_root,'all_sensors',codes,durability_complete=True)
    assert not result['transport_complete'] and not result['recording_complete']
    assert any(row['code']=='ULTRASONIC_EVIDENCE_INVALID' for row in result['issues'])


def test_legacy_ultrasonic_transaction_only_journal_keeps_captured_byte_compatibility(audit_root):
    codes=fixture_ultrasonic(audit_root)
    (audit_root/'configuration/capture_contract.json').unlink()
    atomic_json(audit_root/'capture_manifest.json',dict(profile='all_sensors'))
    directory=audit_root/'sources/ultrasonic'
    events=[json.loads(line) for line in (directory/'events.jsonl').read_text(encoding='utf-8').splitlines()][:-1]
    write_rows(directory/'events.jsonl',events)
    summary=json.loads((directory/'summary.json').read_text(encoding='utf-8'))
    summary.update(accepted_records=24,persisted_records=24,event_counts=dict(transaction=24),
                   events_sha256=digest(directory/'events.jsonl'))
    atomic_json(directory/'summary.json',summary)
    result=audit_capture(audit_root,'all_sensors',codes)
    assert result['recording_complete'] and result['window_complete'] is None
    assert result['completeness_contract']=='LEGACY_CAPTURED_BYTES_ONLY'


@pytest.mark.parametrize('source_shape',['read_only_feedback','manual_mapping'])
def test_new_wheel_journal_lease_then_ack_then_async_footer_is_accepted(audit_root,source_shape):
    codes=fixture_capture(audit_root)
    path=audit_root/'sources/wheel_feedback.jsonl'
    events=[json.loads(line) for line in path.read_text(encoding='utf-8').splitlines()]
    if source_shape=='manual_mapping':
        for row in events:
            row.update(session_id='synthetic',control_context='user_manual_mapping')
        runtime=dict(session_id='synthetic',manual_controls=dict(arm_allowed=True,interaction_policy='hybrid_manual'),
                     manual_authorization=dict(source='EXPLICIT_CAPTURE_MANUAL_DRIVE_ARGUMENT'))
        atomic_json(audit_root/'configuration/manual_runtime.json',runtime)
        manifest_path=audit_root/'capture_manifest.json'
        manifest=json.loads(manifest_path.read_text(encoding='utf-8'));manifest['manual_drive']=True
        manifest['input_hashes']['configuration/manual_runtime.json']=digest(audit_root/'configuration/manual_runtime.json')
        atomic_json(manifest_path,manifest)
    events.append(dict(event='journal_finalization',final_fsync_complete=False))
    write_rows(path,events)
    summary_path=audit_root/'sources/wheel_summary.json'
    summary=json.loads(summary_path.read_text(encoding='utf-8'))
    summary.update(events_sha256=digest(path),interaction_policy='hybrid_manual')
    atomic_json(summary_path,summary)
    assert audit_capture(audit_root,'mapping_core',codes,durability_complete=True)['recording_complete']


@pytest.mark.parametrize('fault',['missing','false','duplicate','before_lease','timeout','after_ack'])
def test_new_wheel_source_requires_one_successful_ack_after_lease_close(audit_root,fault):
    codes=fixture_capture(audit_root);path=audit_root/'sources/wheel_feedback.jsonl'
    events=[json.loads(line) for line in path.read_text(encoding='utf-8').splitlines()]
    if fault=='missing':events.pop()
    elif fault=='false':events[-1]['acknowledged']=False
    elif fault=='duplicate':events.append(dict(events[-1]))
    elif fault=='before_lease':events[-2],events[-1]=events[-1],events[-2]
    elif fault=='timeout':events[-1]['timeout_s']=None
    else:events.append(dict(event='unclosed_source_event'))
    write_rows(path,events)
    summary_path=audit_root/'sources/wheel_summary.json';summary=json.loads(summary_path.read_text(encoding='utf-8'))
    summary['events_sha256']=digest(path);atomic_json(summary_path,summary)
    result=audit_capture(audit_root,'mapping_core',codes,durability_complete=True)
    assert not result['transport_complete'] and not result['recording_complete']
    assert any(issue['code']=='WHEEL_EVIDENCE_INVALID' for issue in result['issues'])


def test_legacy_wheel_source_without_dds_ack_does_not_claim_full_window(audit_root):
    codes=fixture_capture(audit_root);(audit_root/'configuration/capture_contract.json').unlink()
    atomic_json(audit_root/'capture_manifest.json',dict(profile='mapping_core'))
    path=audit_root/'sources/wheel_feedback.jsonl'
    events=[json.loads(line) for line in path.read_text(encoding='utf-8').splitlines()][:-1]
    write_rows(path,events)
    summary_path=audit_root/'sources/wheel_summary.json';summary=json.loads(summary_path.read_text(encoding='utf-8'))
    summary['events_sha256']=digest(path);atomic_json(summary_path,summary)
    result=audit_capture(audit_root,'mapping_core',codes)
    assert result['recording_complete'] and result['window_complete'] is None
