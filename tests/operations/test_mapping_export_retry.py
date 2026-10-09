"""Native export retry with real local SQLite files and a fake exporter; no devices."""
from contextlib import closing
import json
import os
from pathlib import Path
import shutil
import sqlite3
import sys
from types import SimpleNamespace
import uuid

import pytest
import yaml


@pytest.fixture
def tmp_path():
    parent=Path(__file__).absolute().parent
    path=parent/('.export_retry_test_'+uuid.uuid4().hex)
    path.mkdir()
    try:
        yield path
    finally:
        resolved=path.resolve()
        assert resolved.parent==parent and resolved.name.startswith('.export_retry_test_')
        shutil.rmtree(resolved)


@pytest.fixture
def prepared(tmp_path,monkeypatch):
    if os.name=='nt':
        monkeypatch.setitem(sys.modules,'fcntl',SimpleNamespace(LOCK_EX=1,LOCK_NB=2,LOCK_UN=4,flock=lambda *a:None))
    from wc_runtime import mapping_controller as c
    from wc_maps import store
    if os.name=='nt':
        def rename(source,destination):
            assert source.resolve().is_relative_to(tmp_path.resolve())
            assert destination.resolve().is_relative_to(tmp_path.resolve())
            assert source.parent==destination.parent and not destination.exists()
            source.rename(destination)
        monkeypatch.setattr(store,'_rename_no_replace',rename)
        monkeypatch.setattr(store,'_sync_directory',lambda path:None)
    monkeypatch.setattr(c,'ROOT',tmp_path)
    directory=tmp_path/'work';runtime=tmp_path/'runtime'
    for folder in (directory/'slam',directory/'bag',directory/'health',runtime):folder.mkdir(parents=True)
    handle={'directory':str(directory),'runtime':str(runtime),'session_id':'retry_fixture','mode':'right',
            'mapping_enabled':True}
    counts={name:1 for name in ('odom','prior_odom','cloud_map','grid_map')}
    (directory/'runtime_config.json').write_text(json.dumps({'odometry_source':'wheel_imu'}))
    (directory/'health/status.json').write_text(json.dumps({'odometry_source':'wheel_imu','counts':counts,'failure':None}))
    db=directory/'slam/rtabmap.db'
    with closing(sqlite3.connect(db)) as connection:
        connection.execute('CREATE TABLE Node(id INTEGER)')
        connection.execute('INSERT INTO Node VALUES(1)');connection.commit()
    with closing(sqlite3.connect(directory/'bag/fixture.db3')) as connection:
        connection.execute('CREATE TABLE fixture(id INTEGER)')
    topics=['/wc_mapping/imu/source_frame','/wc_mapping/wheel/feedback_raw',
            '/wc_mapping/lidar_right/source_frame','/wc_mapping/lidar_right/source_frame_filtered']
    topics+=['/wc_mapping/app/'+name for name in counts]
    (directory/'bag/metadata.yaml').write_text(yaml.safe_dump({'rosbag2_bagfile_information':{
        'relative_file_paths':['fixture.db3'],
        'topics_with_message_count':[{'topic_metadata':{'name':topic},'message_count':1} for topic in topics]}}))
    (runtime/'process-1.log').write_text(str(db)+' Saving database/long-term memory...done!\n')
    monkeypatch.setattr(c,'inspect',lambda h:{'state':'STOPPED','exit_code':0})
    monkeypatch.setattr(c,'input_archive_summary',lambda h:{'status':'FINAL_CHECKPOINT_VERIFIED'})
    monkeypatch.setattr(c,'storage_status',lambda h:{'free_bytes':10*1024**3})
    calls=[];codes=[]
    def native(command,**kwargs):
        calls.append(command)
        output=directory/'export'
        (output/'map_3d.ply').write_bytes(b'ply\nformat ascii 1.0\nelement vertex 1\nend_header\n0 0 0\n')
        (output/'map.pgm').write_bytes(b'P5\n4 4\n255\n'+bytes([205])*16)
        (output/'map.yaml').write_text(yaml.safe_dump({'image':'map.pgm','resolution':.03,'origin':[0.,0.,0.]}))
        kwargs['stdout'].write(b'native attempt '+str(len(calls)).encode());kwargs['stdout'].flush()
        return SimpleNamespace(returncode=codes.pop(0) if codes else 0)
    monkeypatch.setattr(c.subprocess,'run',native)
    return c,handle,directory,db,calls,codes


def test_completed_export_reused_after_save_destination_failure(prepared,monkeypatch):
    c,handle,directory,db,calls,codes=prepared
    first=c.export(handle);before=db.read_bytes()
    saved={p.name:p.read_bytes() for p in (directory/'export').iterdir()}
    monkeypatch.setattr(c,'storage_status',lambda h:pytest.fail('reused export needs no new native-export reserve'))
    second=c.export(handle)
    assert len(calls)==1 and second['export_reused']
    assert first['files']==second['files'] and db.read_bytes()==before
    assert {p.name:p.read_bytes() for p in (directory/'export').iterdir()}==saved


def test_failed_native_attempt_is_retained_then_retried(prepared):
    c,handle,directory,db,calls,codes=prepared
    codes.extend([7,0]);before=db.read_bytes()
    with pytest.raises(RuntimeError,match='导出失败'):c.export(handle)
    partial={p.name:p.read_bytes() for p in (directory/'export').iterdir()}
    result=c.export(handle)
    retained=Path(result['previous_partial_export'])
    assert retained.parent==directory and retained.name.startswith('export_failed_')
    assert {p.name:p.read_bytes() for p in retained.iterdir()}==partial
    assert (directory/'export/result.json').is_file() and len(calls)==2
    assert db.read_bytes()==before


@pytest.mark.parametrize('marker',['missing','truncated','failed_status'])
def test_incomplete_manifest_is_preserved_for_retry(prepared,marker):
    c,handle,directory,db,calls,codes=prepared
    output=directory/'export';output.mkdir();(output/'export.log').write_bytes(b'prior partial attempt')
    if marker=='truncated':(output/'result.json').write_bytes(b'{"status":')
    elif marker=='failed_status':(output/'result.json').write_text('{"status":"FAIL"}')
    result=c.export(handle)
    assert (Path(result['previous_partial_export'])/'export.log').read_bytes()==b'prior partial attempt'
    assert len(calls)==1


@pytest.mark.parametrize('defect',['session','database_hash','file_hash','missing_file','extra_file','bad_inventory'])
def test_invalid_completed_export_is_not_silently_replaced(prepared,defect):
    c,handle,directory,db,calls,codes=prepared
    c.export(handle);output=directory/'export'
    path=output/'result.json';report=json.loads(path.read_text())
    if defect=='session':report['session_id']='different_session'
    elif defect=='database_hash':report['database_sha256']='0'*64
    elif defect=='file_hash':(output/'map_3d.ply').write_bytes(b'changed cloud')
    elif defect=='missing_file':(output/'map.pgm').unlink()
    elif defect=='extra_file':(output/'unexpected.txt').write_text('not in original export')
    elif defect=='bad_inventory':report['files']={}
    path.write_text(json.dumps(report))
    before={p.name:p.read_bytes() for p in output.iterdir()}
    with pytest.raises(RuntimeError,match='既有成功导出'):c.export(handle)
    assert len(calls)==1 and not list(directory.glob('export_failed_*'))
    assert {p.name:p.read_bytes() for p in output.iterdir()}==before


def test_reuse_still_checks_original_closed_session(prepared,monkeypatch):
    c,handle,directory,db,calls,codes=prepared
    c.export(handle)
    monkeypatch.setattr(c,'inspect',lambda h:{'state':'RUNNING'})
    with pytest.raises(RuntimeError,match='异常退出'):c.export(handle)
    assert len(calls)==1


def test_all_sqlite_handles_explicitly_closed(prepared,monkeypatch):
    c,handle,directory,db,calls,codes=prepared
    connect=sqlite3.connect;opened=[]
    class Tracked(sqlite3.Connection):
        def close(self):
            self.was_closed=True
            return super().close()
    def tracked(*args,**kwargs):
        connection=connect(*args,**kwargs,factory=Tracked)
        connection.was_closed=False;opened.append(connection)
        return connection
    monkeypatch.setattr(c.sqlite3,'connect',tracked)
    c.export(handle);c.export(handle)
    assert len(opened)>=6 and all(connection.was_closed for connection in opened)
