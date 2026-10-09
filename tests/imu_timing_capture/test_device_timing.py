"""Hardware-free timing provenance, discontinuity and original-arrival checks."""
import copy
import hashlib
import json
from pathlib import Path
import struct
import sys
from types import SimpleNamespace as NS

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'src'))
from wc_imu.device_time import DeviceTimeValidator, load_timing_profile, message_timing_metadata, advance_device_counter
from wc_imu.ros_node import frame_record, assign_ros_frame
from wc_runtime.capture_audit import audit_imu_device_timing
from wc_runtime.source_recorder import message_index, SourceReadiness
from wc_sensors.h30 import H30Parser, h30_checksum


def write_profile(base):
    base.mkdir(parents=True, exist_ok=True)
    evidence = base/'calibration/synthetic_timing/evidence.json'
    evidence.parent.mkdir(parents=True, exist_ok=True)
    evidence.write_bytes(b'{"scope":"SYNTHETIC_TEST_ONLY"}\n')
    value = dict(schema_version=1, status='VERIFIED_DEVICE_RELATIVE_TIME', sensor_id='H30-SYN',
        hardware_serial='SYN', product_version='SYNTHETIC-NOT-HARDWARE', device_timestamp_unit='us', tick_ns=1000,
        required_fields=['sample_timestamp_raw','dataready_timestamp_raw'], backward_policy='reject',
        duplicate_policy='reject', read_mode='event', evidence=[dict(path='calibration/synthetic_timing/evidence.json',
        sha256=hashlib.sha256(evidence.read_bytes()).hexdigest())])
    path = base/'imu_timing.json'
    path.write_text(json.dumps(value), encoding='utf-8')
    return path, value


def packet(sample=0, ready=600, tid=1):
    payload = b'\x10\x0c'+struct.pack('<iii',0,0,9810000)+b'\x20\x0c'+struct.pack('<iii',0,0,1)
    if sample is not None: payload += b'\x51\x04'+struct.pack('<I',sample)
    if ready is not None: payload += b'\x52\x04'+struct.pack('<I',ready)
    body = struct.pack('<HB',tid,len(payload))+payload
    return b'YS'+body+h30_checksum(body)


def frame(sample=0, ready=600, tid=1):
    return H30Parser().feed(packet(sample, ready, tid))[0]


def message():
    return NS(header=NS(stamp=NS(sec=0,nanosec=0),frame_id=''), imu=NS(header=None,
        orientation=NS(x=0.,y=0.,z=0.,w=0.),angular_velocity=NS(x=0.,y=0.,z=0.),
        linear_acceleration=NS(x=0.,y=0.,z=0.)))


def test_profile_checks_real_evidence_identity_and_does_not_infer_units(tmp_path):
    path, _ = write_profile(tmp_path)
    profile = load_timing_profile(path, sensor_id='H30-SYN')
    assert profile['_profile_sha256'] == hashlib.sha256(path.read_bytes()).hexdigest()
    assert profile['_profile_path'] == str(path.absolute())
    with pytest.raises(ValueError,match='IDENTITY'):
        load_timing_profile(path,sensor_id='H30-OTHER')
    (path.parent/profile['evidence'][0]['path']).write_bytes(b'changed')
    with pytest.raises(ValueError,match='HASH_MISMATCH'):
        load_timing_profile(path)


@pytest.mark.parametrize('key,value', [('status','CANDIDATE'),('device_timestamp_unit','UNKNOWN'),
    ('tick_ns',100000),('required_fields',[]),('backward_policy','unwrap'),('duplicate_policy','ignore'),
    ('read_mode','auto'),('max_gap_ticks',True),('evidence',[])])
def test_unverified_or_ambiguous_profile_rejected(tmp_path,key,value):
    path, policy = write_profile(tmp_path)
    policy[key] = value
    path.write_text(json.dumps(policy),encoding='utf-8')
    with pytest.raises(ValueError): load_timing_profile(path)


@pytest.mark.parametrize('sample,ready', [(None,600),(0,None),(0,600),(2**32-1,2**32-1)])
def test_missing_zero_and_maximum_are_distinct(tmp_path,sample,ready):
    path,_ = write_profile(tmp_path)
    guard = DeviceTimeValidator(load_timing_profile(path))
    if sample is None or ready is None:
        with pytest.raises(ValueError,match='MISSING'): guard.observe(frame(sample,ready))
    else:
        timing = guard.observe(frame(sample,ready))
        assert timing['sample_timestamp_valid'] and timing['sample_timestamp_raw'] == sample


@pytest.mark.parametrize('first,second,reason', [(100,100,'DUPLICATE'),(100,90,'BACKWARD'),(2**32-5000,0,'BACKWARD')])
def test_clock_fault_latches_and_never_silently_unwraps(tmp_path,first,second,reason):
    path,_ = write_profile(tmp_path)
    guard = DeviceTimeValidator(load_timing_profile(path))
    guard.observe(frame(first,first))
    with pytest.raises(ValueError,match=reason): guard.observe(frame(second,second))
    with pytest.raises(ValueError,match='LATCHED'): guard.observe(frame(200,200))


def test_device_times_differ_with_shared_host_batch_and_tid_wrap(tmp_path):
    path,_ = write_profile(tmp_path)
    guard = DeviceTimeValidator(load_timing_profile(path))
    outputs = []
    for sequence, decoded in enumerate((frame(0,600,60000),frame(5000,5600,1)),1):
        timing = guard.observe(decoded)
        record = frame_record(decoded,session_id='SYN',sensor_id='H30-SYN',stream_epoch='epoch',
            frame_sequence=sequence,host_receive_ns=1700000000000000001,host_monotonic_ns=123456,timing=timing)
        msg = assign_ros_frame(record,message())
        assert msg.raw_packet == decoded.raw_packet
        assert msg.header.stamp == msg.imu.header.stamp == msg.host_receive_time
        assert (msg.header.stamp.sec,msg.header.stamp.nanosec) == (1700000000,1)
        assert msg.host_monotonic_ns == 123456 and msg.common_time_valid is False and msg.common_time_ns == 0
        assert msg.time_source == 'arrival_only'
        assert message_timing_metadata(msg) == timing
        index = message_index('/wc_mapping/imu/source_frame',msg)
        assert index['timing_profile_sha256'] == guard.profile['_profile_sha256']
        outputs.append(msg)
    assert outputs[1].sample_timestamp_raw-outputs[0].sample_timestamp_raw == 5000


def build_journal(root):
    path,_ = write_profile(root/'configuration')
    policy = load_timing_profile(path)
    guard = DeviceTimeValidator(policy)
    raw = packet(0,600,1)+packet(5000,5600,2)
    stale=packet(1000000,1000600,99)
    events=[dict(event='byte_batch',batch_id=1,bytes_hex=stale.hex(),sha256=hashlib.sha256(stale).hexdigest(),
                 host_receive_ns=100000,host_monotonic_ns=200000),
        dict(event='startup_quarantine',batch_id=1,sensor_id='H30-SYN',stream_epoch='epoch',
            packet_sha256=hashlib.sha256(stale).hexdigest(),host_receive_ns=100000,host_monotonic_ns=200000,
            tid=99,reason='BOUNDED_STARTUP_BUFFER_QUARANTINE',quarantine_started_monotonic_ns=200000,
            startup_quarantine_ns=250000000,timing_profile_sha256=policy['_profile_sha256']),
        dict(event='byte_batch',batch_id=2,bytes_hex=raw.hex(),sha256=hashlib.sha256(raw).hexdigest(),
                 host_receive_ns=250100000,host_monotonic_ns=250200000)]
    for sequence,decoded in enumerate(H30Parser().feed(raw),1):
        events.append(dict(event='frame',batch_id=2,sensor_id='H30-SYN',stream_epoch='epoch',sequence=sequence,
            packet_sha256=hashlib.sha256(decoded.raw_packet).hexdigest(),host_receive_ns=250100000,
            host_monotonic_ns=250200000,**guard.observe(decoded)))
    output = root/'sources/imu/events.jsonl'
    output.parent.mkdir(parents=True)
    output.write_text(''.join(json.dumps(row)+'\n' for row in events),encoding='utf-8')
    manifest=dict(imu_sensor_id='H30-SYN',imu_timing_profile_path='configuration/imu_timing.json',
                  input_hashes={'configuration/imu_timing.json':policy['_profile_sha256']})
    return output, events, manifest


def test_audit_reparses_batches_and_accepts_original_arrival(tmp_path):
    _,_,manifest=build_journal(tmp_path)
    expected, report=audit_imu_device_timing(tmp_path,manifest)
    assert len(expected)==2 and report['accepted_frames']==2
    assert expected[('H30-SYN','epoch',2,'packet')]['sample_timestamp_raw']==5000
    assert report['common_measurement_time_validated'] is False
    assert report['startup_quarantine_frames']==1 and report['parser']['frames']==3


@pytest.mark.parametrize('mutation', ['counter','unit','arrival','raw','tail','fault'])
def test_audit_rejects_metadata_forgery_or_incomplete_timing(tmp_path,mutation):
    path,events,manifest=build_journal(tmp_path)
    if mutation=='counter': events[-1]['sample_timestamp_raw']=5001
    elif mutation=='unit': events[-1]['device_timestamp_unit']='100us'
    elif mutation=='arrival': events[-1]['host_receive_ns']+=1
    elif mutation=='raw': events[0]['bytes_hex']='00'
    elif mutation=='tail': events.pop()
    else: events.append(dict(event='device_time_fault',error='reset'))
    path.write_text(''.join(json.dumps(row)+'\n' for row in events),encoding='utf-8')
    with pytest.raises(ValueError): audit_imu_device_timing(tmp_path,manifest)


def test_legacy_audit_works_without_timing_but_lost_profile_is_rejected(tmp_path):
    assert audit_imu_device_timing(tmp_path,{})==({},None)
    with pytest.raises(ValueError,match='PROFILE_MISSING'):
        audit_imu_device_timing(tmp_path,{'imu_timing_profile_path':'configuration/imu_timing.json'})


@pytest.mark.parametrize('field,value',[('tid',100),('reason','skip_anything'),
    ('startup_quarantine_ns',999999999),('quarantine_started_monotonic_ns',0),('timing_profile_sha256','0'*64)])
def test_quarantine_metadata_tampering_is_rejected(tmp_path,field,value):
    path,events,manifest=build_journal(tmp_path)
    events[1][field]=value
    path.write_text(''.join(json.dumps(row)+'\n' for row in events),encoding='utf-8')
    with pytest.raises(ValueError,match='QUARANTINE'):
        audit_imu_device_timing(tmp_path,manifest)


def test_published_frame_cannot_be_relabelled_as_late_quarantine(tmp_path):
    path,events,manifest=build_journal(tmp_path)
    late=dict(events[1],batch_id=2,host_receive_ns=250100000,host_monotonic_ns=250200000,
              packet_sha256=events[-1]['packet_sha256'],tid=2)
    events[-1]=late
    path.write_text(''.join(json.dumps(row)+'\n' for row in events),encoding='utf-8')
    with pytest.raises(ValueError,match='QUARANTINE'):
        audit_imu_device_timing(tmp_path,manifest)


def test_capture_uses_frozen_profile_and_event_default(tmp_path,monkeypatch):
    from wc_runtime import capture
    monkeypatch.setitem(sys.modules,'wc_runtime.cli',NS(ros_command=lambda args:list(map(str,args))))
    monkeypatch.setattr(capture,'source_selection',lambda *args: {})
    config=tmp_path/'configuration';config.mkdir()
    binding=dict(device='/dev/SYN',expected_by_id='/dev/serial/by-id/usb-SYN',hardware_serial='SYN',sensor_id='H30-SYN')
    (config/'device_bindings.json').write_text(json.dumps({'imu':binding}))
    legacy=capture.source_commands(ROOT,ROOT/'.phase1_runtime',tmp_path,'SYN','mapping_core')['imu']
    assert legacy[legacy.index('--read-mode')+1]=='event' and '--timing-profile' not in legacy
    path,_=write_profile(config)
    current=capture.source_commands(ROOT,ROOT/'.phase1_runtime',tmp_path,'SYN','mapping_core')['imu']
    assert current[current.index('--timing-profile')+1]==str(path)
    assert '--write' not in current and '--configure' not in current


def test_readiness_requires_frozen_timing_not_just_messages(tmp_path):
    path,_=write_profile(tmp_path/'configuration')
    profile=load_timing_profile(path)
    ready=SourceReadiness(tmp_path,dict(session_id='SYN',sources={'imu':{'source_id':'H30-SYN'}}))
    row=dict(topic='/wc_mapping/imu/source_frame',source_id='H30-SYN',session_id='SYN',stream_epoch='epoch',host_monotonic_ns=1)
    with pytest.raises(ValueError,match='timing'): ready.observe(row)
    row.update(DeviceTimeValidator(profile).observe(frame()))
    ready.observe(row)
    assert ready.values['imu']['ready'] is True


def guarded_profile(tmp_path):
    path,policy=write_profile(tmp_path)
    policy.update(backward_policy='guarded_uint32_wrap',max_gap_ticks=2000000,wrap_host_tolerance_ns=100000000)
    path.write_text(json.dumps(policy),encoding='utf-8')
    return load_timing_profile(path)


def test_guarded_wrap_counters_may_cross_on_different_frames(tmp_path):
    guard=DeviceTimeValidator(guarded_profile(tmp_path))
    modulus=2**32
    guard.observe(frame(modulus-5200,modulus-4600),host_monotonic_ns=1000000000)
    first=guard.observe(frame(modulus-200,400),host_monotonic_ns=1005000000)
    second=guard.observe(frame(4800,5400),host_monotonic_ns=1010000000)
    assert first['dataready_timestamp_raw']==400 and second['sample_timestamp_raw']==4800
    assert guard.report()['wrap_counts']=={'sample_timestamp_raw':1,'dataready_timestamp_raw':1}
    assert guard.report()['elapsed_ticks']=={'sample_timestamp_raw':10000,'dataready_timestamp_raw':10000}


@pytest.mark.parametrize('previous,current,host_delta',[(10000000,0,5000000),
    (2**32-5000,0,500000000),(2**32-3000000,0,5000000),(2**32-5000,3000000,5000000)])
def test_reset_or_unproven_wrap_rejected(tmp_path,previous,current,host_delta):
    with pytest.raises(ValueError,match='WRAP_OR_RESET'):
        advance_device_counter(previous,current,host_delta,guarded_profile(tmp_path))


def test_guarded_wrap_requires_real_host_counter_and_accepts_zero(tmp_path):
    profile=guarded_profile(tmp_path)
    assert advance_device_counter(2**32-5000,0,5000000,profile)==(5000,True)
    assert advance_device_counter(2**32-5000,0,0,profile)==(5000,True) # same received batch
    with pytest.raises(ValueError,match='HOST'): DeviceTimeValidator(profile).observe(frame())
    with pytest.raises(ValueError,match='GAP'): advance_device_counter(1,3000000,5000000,profile)


def test_snapshot_preserves_profile_and_evidence_bytes(tmp_path,monkeypatch):
    from wc_runtime import capture, capture_support
    project=tmp_path/'project';configuration=project/'config'
    path,_=write_profile(configuration)
    original=path.read_bytes()
    for name in ('mapping_live.json','hardware_setup.json','wheel_feedback_current.json'):
        (configuration/name).write_text('{}')
    lidar=project/'install/main/wc_xt_driver/share/wc_xt_driver/config'
    lidar.mkdir(parents=True);(lidar/'left.yaml').write_text('synthetic: true')
    binding=dict(device='/dev/SYN',expected_by_id='/dev/serial/by-id/usb-SYN',hardware_serial='SYN',sensor_id='H30-SYN')
    monkeypatch.setitem(sys.modules,'wc_runtime.cli',NS(require_device_bindings=lambda:{'imu':binding}))
    monkeypatch.setitem(sys.modules,'wc_runtime.hardware_setup',NS(resolve_hardware_setup=lambda value:{'geometry_report':{}}))
    monkeypatch.setattr(capture,'source_selection',lambda *args: {'selected_sources':['imu']})
    monkeypatch.setattr(capture_support,'software_snapshot',lambda *args:{})
    output=tmp_path/'session';output.mkdir()
    hashes=capture.snapshot(project,output,profile='mapping_core',storage_policy=NS(snapshot_configuration=lambda cfg:{}))
    frozen=output/'configuration/imu_timing.json'
    assert frozen.read_bytes()==original
    assert hashes['configuration/imu_timing.json']==hashlib.sha256(original).hexdigest()
    path.write_text('current machine configuration changed')
    frozen_policy=load_timing_profile(frozen,sensor_id='H30-SYN')
    assert frozen_policy['_profile_sha256']==hashes['configuration/imu_timing.json']
    assert str(Path('configuration/calibration/synthetic_timing/evidence.json')) in hashes
