"""Synthetic fixtures for read-only pairing audit; no ROS/device execution."""
import copy
import hashlib
import importlib.util
import json
from pathlib import Path
import sqlite3
from types import SimpleNamespace as N
import numpy as np
import pytest
import yaml

SCRIPT=Path(__file__).with_name('check_filtered_bag.py')
spec=importlib.util.spec_from_file_location('wc_filtered_bag_auditor',SCRIPT)
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)


def write(path,text):
    path.parent.mkdir(parents=True,exist_ok=True)
    path.write_bytes(text.encode())


def configuration(root,side):
    source=b'[Setting]\nimgType=4\npclFilterOn=1\n\n[Filters]\nspatialEnable=1\n'
    config=root/'src/wc_xt_driver/config'/(side+'.xtcfg')
    config.parent.mkdir(parents=True,exist_ok=True);config.write_bytes(source)
    directory=root/'.phase1_runtime/sessions/test-session'/side/('epoch-'+side)
    write(directory/'device_before.txt','SYNTHETIC before\n')
    after=b'SYNTHETIC after\n';write(directory/'device_after.txt',after.decode())
    write(directory/'configuration_commands.txt','exposures_cmd8=READBACK_MATCH_NO_WRITE\npersistent_save=NOT_REQUESTED\nnetwork_clock_firmware_motor=DENIED\n')
    text='actual_device_readback_sha256='+m.digest(after)+'\n'
    text+='serial='+m.EXPECTED[side]+'\nrequested_xtcfg_sha256='+m.digest(source)+'\n'
    text+='configuration_status=PARTIAL_XTCFG\nSDK_filter_flags=253\n'
    text+='Setting.imgType=4;disposition=APPLIED_SDK\nSetting.pclFilterOn=1;disposition=UNSUPPORTED\nFilters.spatialEnable=1;disposition=APPLIED_SDK\n'
    raw=m.digest((text+'representation=before_host_filter\n').encode())
    filtered=m.digest((text+'representation=host_filtered\n').encode())
    write(directory/'device_readback.txt',text+'effective_source_hash='+raw+'\nfiltered_source_hash='+filtered+'\n')
    return raw,filtered


def frame(side,config_hash,filtered=False):
    header=N(frame_id='lidar_'+side,stamp=N(sec=1,nanosec=23))
    values=np.array([[1,2,3,10],[4,5,6,20],[np.nan,1,2,30]],dtype='<f4')
    if filtered:values[0,0]=1.2
    cloud=N(header=copy.deepcopy(header),width=3,height=1,point_step=16,row_step=48,
        is_bigendian=False,fields=[N(name=name,datatype=7,count=1,offset=i*4) for i,name in enumerate(('x','y','z','intensity'))],data=values.tobytes())
    return N(header=header,cloud=cloud,session_id='test-session',side=side,sensor_id=m.EXPECTED[side],stream_epoch='epoch-'+side,frame_sequence=0,
      source_config_hash=config_hash,coordinate_convention='FLU',units='m',device_timestamp_valid=True,device_timestamp_raw=991,
      device_timestamp_unit='sdk_v3_ms',device_time_components_valid=True,device_timestamp_seconds=0,device_timestamp_nanoseconds=991,
      device_time_type='sdk_type_0',device_sync_state='sdk_state_0',host_receive_time=N(sec=1,nanosec=23),host_monotonic_ns=1000023,
      common_time_valid=False,common_time_ns=1000000023,clock_model_id='',time_source='arrival_only',uncertainty_valid=False,uncertainty_ns=0,
      raw_count=3,valid_count=2,diagnostic_flags=['representation='+('host_filtered' if filtered else 'before_host_filter'),
        'host_filter_and_dispatch_elapsed_ns=123456','configuration_status=PARTIAL_XTCFG','host_elapsed_is_not_measurement_latency'])


def create_project_fixture(root):
    root.mkdir(parents=True,exist_ok=True)
    frames={}
    for side in ('left','right'):
        raw_hash,filtered_hash=configuration(root,side)
        raw=frame(side,raw_hash);filtered=frame(side,filtered_hash,True)
        filtered.diagnostic_flags += ['raw_source_config_hash='+raw_hash,'before_host_filter_cloud_sha256='+m.digest(raw.cloud.data)]
        frames[f'/wc_mapping/lidar_{side}/source_frame']=raw
        frames[f'/wc_mapping/lidar_{side}/source_frame_filtered']=filtered
    return frames


def review(root,frames):
    audit=m.Audit(root,root/'.phase1_runtime',root/'src/wc_xt_driver/config')
    for topic,message in frames.items():audit.consume(topic,message,1000000999,topic.encode())
    return audit


def test_exact_join_uses_sensor_identity_and_preserves_descriptive_stats(tmp_path):
    root=tmp_path/'project';frames=create_project_fixture(root);audit=review(root,frames);result=audit.finish()
    assert not result['issues']
    assert len(result['configuration_records'])==2
    for side in ('left','right'):
        value=result['sides'][side]
        assert value['counts']['verified_raw_filtered_pairs']==1
        assert value['counts']['different_cloud_payloads']==1
        assert value['raw_valid_points_per_frame']['min']==2
        assert value['filtered_valid_points_per_frame']['min']==2
        assert value['host_filter_and_dispatch_elapsed_ns']['p50']==123456


@pytest.mark.parametrize('fault',('content_hash','raw_config_hash','acquisition','duplicate','unpaired','declared_count','duplicate_flag','original_config','effective_config'))
def test_integrity_faults_never_receive_clean_pairing_result(tmp_path,fault):
    root=tmp_path/'project';frames=create_project_fixture(root)
    topic='/wc_mapping/lidar_left/source_frame_filtered';filtered=frames[topic]
    if fault=='content_hash':filtered.diagnostic_flags[-1]='before_host_filter_cloud_sha256='+'0'*64
    if fault=='raw_config_hash':filtered.diagnostic_flags[-2]='raw_source_config_hash='+'0'*64
    if fault=='acquisition':filtered.host_monotonic_ns+=1
    if fault=='unpaired':del frames[topic]
    if fault=='declared_count':filtered.valid_count=3
    if fault=='duplicate_flag':filtered.diagnostic_flags.append('representation=host_filtered')
    if fault=='original_config':(root/'src/wc_xt_driver/config/left.xtcfg').write_bytes(b'CHANGED')
    if fault=='effective_config':
        p=root/'.phase1_runtime/sessions/test-session/left/epoch-left/device_readback.txt'
        p.write_bytes(p.read_bytes().replace(b'SDK_filter_flags=253',b'SDK_filter_flags=0'))
    try:audit=review(root,frames)
    except ValueError:
        assert fault in ('declared_count','duplicate_flag','original_config','effective_config');return
    if fault=='duplicate':audit.consume(topic,filtered,1000000999,b'conflicting bytes')
    assert audit.finish()['issues']


def build_bag(root,frames):
    bag=root/'data/bags/fixture';bag.mkdir(parents=True)
    database=bag/'fixture_0.db3';conn=sqlite3.connect(database)
    conn.executescript('CREATE TABLE topics(id INTEGER PRIMARY KEY,name TEXT,type TEXT,serialization_format TEXT); CREATE TABLE messages(id INTEGER PRIMARY KEY,topic_id INTEGER,timestamp INTEGER,data BLOB);')
    decoder={};metadata_topics=[]
    for i,(topic,message) in enumerate(frames.items(),1):
        blob=('INERT_TEST_MESSAGE_'+str(i)).encode();decoder[blob]=message
        conn.execute('INSERT INTO topics VALUES (?,?,?,?)',(i,topic,m.TYPE,'cdr'))
        conn.execute('INSERT INTO messages VALUES (?,?,?,?)',(i,i,1000000999+i,blob))
        metadata_topics.append({'topic_metadata':{'name':topic,'type':m.TYPE,'serialization_format':'cdr'},'message_count':1})
    conn.commit();conn.close()
    info={'storage_identifier':'sqlite3','relative_file_paths':['fixture_0.db3'],'message_count':len(frames),'topics_with_message_count':metadata_topics,'compression_format':'','compression_mode':'none'}
    write(bag/'metadata.yaml',yaml.safe_dump({'rosbag2_bagfile_information':info}))
    return bag,decoder


def test_finalized_sqlite_read_leaves_bytes_and_mtime_unchanged(tmp_path):
    root=tmp_path/'project';frames=create_project_fixture(root);bag,decoder=build_bag(root,frames)
    before={str(p):m.file_state(p) for p in root.rglob('*') if p.is_file()}
    result=m.audit_bag(bag,root,decoder=decoder.__getitem__)
    assert result['status']=='PAIRING_REVIEW_OK' and result['decoding']=='TEST_DECODER'
    assert result['storage']['files_unchanged']
    assert result['acceptance']['filter_quality']=='NOT_EVALUATED'
    assert result['acceptance']['measurement_latency']=='NOT_MEASURED'
    assert before=={str(p):m.file_state(p) for p in root.rglob('*') if p.is_file()}


def test_active_sidecar_and_memory_bound_refuse(tmp_path):
    root=tmp_path/'project';frames=create_project_fixture(root);bag,decoder=build_bag(root,frames)
    with pytest.raises(ValueError,match='memory bound'):
        m.audit_bag(bag,root,max_envelopes=3,decoder=decoder.__getitem__)
    (bag/'fixture_0.db3-wal').write_bytes(b'ACTIVE')
    with pytest.raises(ValueError,match='active SQLite sidecar'):
        m.audit_bag(bag,root,decoder=decoder.__getitem__)


def test_corrupt_metadata_counts_are_reported(tmp_path):
    root=tmp_path/'project';frames=create_project_fixture(root);bag,decoder=build_bag(root,frames)
    p=bag/'metadata.yaml';metadata=yaml.safe_load(p.read_bytes());metadata['rosbag2_bagfile_information']['message_count']=999
    p.write_bytes(yaml.safe_dump(metadata).encode())
    result=m.audit_bag(bag,root,decoder=decoder.__getitem__)
    assert result['status']=='PAIRING_REVIEW_ISSUES'
    assert 'metadata_message_count_mismatch' in result['issues']
