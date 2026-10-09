import json
from pathlib import Path

import pytest

from wc_runtime.source_recorder import SourceReadiness,drain_received_callbacks
from wc_runtime.capture_support import synchronize_tree


def lidar(sequence=0, representation='raw', epoch='epoch', source='serial-left', side='left'):
    return dict(topic='/wc_mapping/lidar_left/source_frame',source_id=source,side=side,
                stream_epoch=epoch,sequence=sequence,representation=representation,valid_point_count=10,
                host_monotonic_ns=100+sequence,session_id='session')


def readiness(tmp_path):
    return SourceReadiness(tmp_path,dict(session_id='session',sources={
        'lidar_left':dict(source_id='serial-left',side='left'),
        'wheel_left':dict(source_id='wheel'), 'wheel_right':dict(source_id='wheel'),
        'ultrasonic_address_1':dict(source_id='ultrasonic')}))


def test_lidar_waits_for_both_representations_and_freezes_epoch(tmp_path):
    state=readiness(tmp_path)
    state.observe(lidar())
    assert not state.values
    with pytest.raises(ValueError,match='epoch'):
        state.observe(lidar(representation='filtered',epoch='different'))
    state.observe(lidar(representation='filtered'))
    state.flush()
    assert json.loads((tmp_path/'ready/lidar_left.json').read_text())['ready']


@pytest.mark.parametrize('updates',[{'source':'serial-right'},{'side':'right'}])
def test_lidar_wrong_identity_never_ready(tmp_path,updates):
    with pytest.raises(ValueError,match='identity'):
        readiness(tmp_path).observe(lidar(**updates))


def test_timeouts_do_not_make_wheel_or_ultrasound_ready(tmp_path):
    state=readiness(tmp_path)
    wheel=dict(topic='/wc_mapping/wheel/feedback_raw',status='RESPONSE_TIMEOUT',
        source_id='wheel',stream_epoch='w',host_monotonic_ns=100)
    ultra=dict(topic='/wc_mapping/ultrasonic/feedback_raw',status='RESPONSE_TIMEOUT',
        source_id='ultrasonic',stream_epoch='u',host_monotonic_ns=100,address=1)
    state.observe(wheel);state.observe(ultra)
    assert not state.values
    state.observe(dict(wheel,status='RESPONSE_VALID',register_words_u16=[1,2]))
    state.observe(dict(ultra,status='VALID_RESPONSE'))
    assert set(state.values)=={'wheel_left','wheel_right','ultrasonic_address_1'}


def test_durability_failure_is_propagated(tmp_path,monkeypatch):
    (tmp_path/'source').mkdir();(tmp_path/'source/tail').write_bytes(b'last frame')
    def failed(_): raise OSError('injected fsync failure')
    monkeypatch.setattr('wc_runtime.capture_support.os.fsync',failed)
    with pytest.raises(OSError,match='injected'):
        synchronize_tree(tmp_path)


def test_durability_synchronizes_added_files_before_directories(tmp_path,monkeypatch):
    (tmp_path/'source').mkdir();(tmp_path/'source/tail').write_bytes(b'last frame')
    calls=[]
    monkeypatch.setattr('wc_runtime.capture_support.os.fsync',lambda fd:calls.append(fd))
    if __import__('os').name=='nt':
        pytest.skip('directory descriptors use the Linux target API')
    result=synchronize_tree(tmp_path)
    assert result['complete'] and result['file_count']==1 and result['directory_count']==2
    assert result['ancestor_directory_count']==len(tmp_path.absolute().parents)
    assert len(calls)==3+result['ancestor_directory_count']


def test_durability_parent_entry_failure_cannot_be_complete(tmp_path,monkeypatch):
    import os
    if os.name=='nt':
        pytest.skip('directory descriptors use the Linux target API')
    parent_identity=tmp_path.parent.stat()
    def fail_parent(fd):
        identity=os.fstat(fd)
        if (identity.st_dev,identity.st_ino)==(parent_identity.st_dev,parent_identity.st_ino):
            raise OSError('injected session parent fsync failure')
    monkeypatch.setattr('wc_runtime.capture_support.os.fsync',fail_parent)
    with pytest.raises(OSError,match='session parent'):
        synchronize_tree(tmp_path)


def test_middleware_tail_is_drained_before_quiet_return():
    now,count=[0.],[0]
    def spin():
        now[0]+=.05
        if now[0]<.7: count[0]+=1
    assert drain_received_callbacks(spin,lambda:count[0],clock=lambda:now[0])
    assert count[0]>10 and now[0]>=1.15


def test_middleware_drain_has_a_deadline():
    now,count=[0.],[0]
    def spin(): now[0]+=.05;count[0]+=1
    assert not drain_received_callbacks(spin,lambda:count[0],clock=lambda:now[0])
    assert 3<=now[0]<3.1
