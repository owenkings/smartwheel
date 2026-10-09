"""Completed offline results may have an empty supervisor log and nonempty native logs."""
import os
from pathlib import Path
from unittest.mock import patch

import pytest

from wc_panel.jobs import JobManager


@pytest.fixture
def logs(tmp_path):
    project=tmp_path/'project';project.mkdir()
    output=tmp_path/'result';stage=output/'official_filtered'
    native=stage/'native'/'official_hz5';native.mkdir(parents=True)
    (native/'native.log').write_text('原生建图启动\nmap complete\n',encoding='utf-8')
    manager=JobManager(project,tmp_path/'jobs',detached_offline=False)
    with patch('wc_panel.jobs.threading.Thread.start'):
        row=manager.submit('offline','fixture',[dict(id='official_filtered',label='官方 filtered',
            output=str(stage),commands=[['not-executed']])],output=output)
    Path(row['log_path']).write_bytes(b'')
    manager._jobs[row['id']]['status']='COMPLETE'
    return manager,row,native


def test_nonempty_stage_selectable_when_main_log_empty_and_reopen_keeps_it(logs):
    manager,row,native=logs
    options=manager.list_logs(row['id'])
    assert options[0]['id']=='runtime' and options[0]['bytes']==0
    [stage]=[item for item in options if item['id']!='runtime']
    assert stage['available'] and stage['bytes']>0
    assert 'official_filtered' not in stage['id']  # opaque, not a path argument
    assert 'map complete' in manager.read_log(row['id'],log_id=stage['id'])['text']
    assert manager.read_log(row['id'])['text']==''
    manager._save(manager._jobs[row['id']])
    reopened=JobManager(manager.project,manager.job_root,detached_offline=False)
    assert reopened.read_log(row['id'],log_id=stage['id'])['text'].startswith('原生建图')
    with pytest.raises(ValueError,match='日志选项'):
        manager.read_log(row['id'],log_id=str(native/'native.log'))


def test_log_offsets_bound_utf8_and_cap_at_existing_length(logs):
    manager,row,native=logs
    stage=manager.list_logs(row['id'])[1]
    first=manager.read_log(row['id'],log_id=stage['id'])
    assert first['offset']==len((native/'native.log').read_bytes())
    assert manager.read_log(row['id'],first['offset'],stage['id'])['text']==''
    with pytest.raises(ValueError):manager.read_log(row['id'],-1,stage['id'])


def test_nested_mount_is_reported_without_hiding_good_log(logs):
    manager,row,native=logs
    mounted=native.parent/'mounted';mounted.mkdir();(mounted/'hidden.log').write_text('no')
    with patch('wc_panel.storage._mount_points',return_value={mounted}):
        options=manager.list_logs(row['id'])
    assert any(item['available'] and item['path'].endswith('native.log') for item in options)
    assert any(not item['available'] and '挂载' in item['error'] for item in options)
    assert not any(item['path'].endswith('hidden.log') for item in options)


def test_listing_and_reading_logs_never_open_native_cloud_or_result_payload(logs):
    manager,row,native=logs
    payloads={native/'cloud.pcd',native/'rtabmap.db',native.parent/'result.json'}
    for path in payloads:path.write_bytes(b'fixture payload must not be read')
    original=Path.open
    def guarded(path,*args,**kwargs):
        assert path not in payloads, 'log discovery must only stat non-log payloads'
        return original(path,*args,**kwargs)
    with patch.object(Path,'open',guarded):
        options=manager.list_logs(row['id'])
        log=next(item for item in options if item['id']!='runtime' and item['available'])
        assert 'map complete' in manager.read_log(row['id'],log_id=log['id'])['text']


def test_stage_directory_enumeration_is_bounded_before_materialization(logs):
    manager,row,native=logs
    ordinary=native/'not_a_log.pcd';ordinary.write_bytes(b'x')
    class HugeDirectory:
        count=0
        def __enter__(self):return self
        def __exit__(self,*unused):pass
        def __iter__(self):return self
        def __next__(self):
            self.count+=1
            assert self.count<=8193, 'must not materialize the complete huge directory'
            return type('Entry',(),{'path':str(ordinary)})()
    listing=HugeDirectory()
    with patch('wc_panel.jobs.os.scandir',return_value=listing):
        options=manager.list_logs(row['id'])
    assert listing.count==8193
    assert any('扫描条目限制' in item['error'] for item in options)


def test_failure_is_not_published_before_child_states_and_logs_are_final(logs):
    manager,row,native=logs
    manager._jobs[row['id']]['status']='QUEUED'
    manager._jobs[row['id']]['task_states'].append(dict(id='later',label='later',status='QUEUED',error=''))
    observed=[]
    class FailedCommand:
        pid=1234
        returncode=7
        def poll(self):return self.returncode
    def observe_log(*unused):observed.append(manager.get(row['id']))
    with patch('wc_panel.jobs.subprocess.Popen',return_value=FailedCommand()), \
            patch.object(manager,'_failure_log',side_effect=observe_log):
        manager._run(row['id'],[dict(label='failure fixture',commands=[['not-executed']])],[])
    assert observed[0]['status']=='RUNNING'
    final=manager.get(row['id'])
    assert final['status']=='FAILED' and final['ended_at'] is not None
    assert [state['status'] for state in final['task_states']]==['FAILED','SKIPPED']


@pytest.mark.skipif(os.name=='nt',reason='symlink fixture requires POSIX')
def test_symlink_cannot_turn_registered_stage_log_into_arbitrary_file(logs,tmp_path):
    manager,row,native=logs
    chosen=manager.list_logs(row['id'])[1]['id']
    secret=tmp_path/'other.log';secret.write_text('not allowed')
    (native/'native.log').unlink();(native/'native.log').symlink_to(secret)
    response=manager.read_log(row['id'],log_id=chosen)
    assert not response['available'] and not response['text']
