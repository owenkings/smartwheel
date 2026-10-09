"""Submission and stop outcomes must agree with the owned supervisor evidence."""
import json
from pathlib import Path
import time
from unittest.mock import patch

import pytest

from wc_panel.jobs import JobManager
from wc_panel.storage import atomic_json


class BadGuard:
    def check(self):
        raise ValueError('fixture destination identity changed')


def one_task(**extra):
    return dict(id='step',label='fixture step',commands=[['not-executed']],**extra)


def test_unavailable_log_guard_fails_before_creating_a_fallback_directory(tmp_path):
    project=tmp_path/'project';project.mkdir()
    directory=tmp_path/'missing_mount/jobs'
    manager=JobManager(project,directory,detached_offline=False,log_guard=BadGuard())
    with pytest.raises(ValueError,match='identity changed'):
        manager.submit('live','fixture',[one_task()])
    assert not directory.exists()
    assert manager.jobs()==[]


@pytest.mark.parametrize('failure',['guard','launch_file','spawn'])
def test_submission_failure_is_terminal_logged_and_releases_data(tmp_path,failure):
    project=tmp_path/'project';project.mkdir()
    output=tmp_path/'result';output.mkdir()
    atomic_json(output/'panel_result.json',{'status':'QUEUED'})
    manager=JobManager(project,tmp_path/'jobs',detached_offline=True)
    def write(path,value):
        if failure=='launch_file' and Path(path).name=='launch.json':
            raise OSError('fixture launch write failure')
        return atomic_json(path,value)
    with patch('wc_panel.jobs.atomic_json',side_effect=write), \
            patch('wc_panel.jobs.subprocess.Popen',side_effect=OSError('fixture spawn failure')) as spawn:
        with pytest.raises(RuntimeError,match='任务未启动'):
            manager.submit('offline','fixture',[one_task()],output=output,protected_paths=[output],
                           guards=[BadGuard()] if failure=='guard' else [])
    assert spawn.call_count==(1 if failure=='spawn' else 0)
    [row]=manager.jobs()
    assert row['status']=='FAILED'
    assert row['ended_at'] is not None
    assert row['task_states'][0]['status']=='SKIPPED'
    assert manager.active_paths()==[]
    assert 'fixture' in manager.read_log(row['id'])['text']
    assert json.loads((output/'panel_result.json').read_text(encoding='utf-8'))['status']=='FAILED'
    reopened=JobManager(project,tmp_path/'jobs',detached_offline=False)
    assert reopened.get(row['id'])['status']=='FAILED'


class ExitedSupervisor:
    pid=987654321
    def __init__(self,code):self.returncode=code
    def poll(self):return self.returncode


@pytest.mark.parametrize('code,expected',[(0,'COMPLETE'),(3,'PENDING'),(2,'FAILED'),(-2,'FAILED')])
def test_live_user_stop_keeps_exit_failure_and_save_pending(tmp_path,code,expected):
    project=tmp_path/'project';project.mkdir()
    manager=JobManager(project,tmp_path/'jobs',detached_offline=False)
    tasks=[one_task(accepted_exit_codes=[0,3])]
    with patch('wc_panel.jobs.threading.Thread.start'):
        row=manager.submit('live','live fixture',tasks)
    def start(*args,**kwargs):
        # User stop arrives after the command begins, before its final receipt.
        manager._cancel.add(row['id'])
        return ExitedSupervisor(code)
    with patch('wc_panel.jobs.subprocess.Popen',side_effect=start):
        manager._run(row['id'],tasks,())
    done=manager.get(row['id'])
    assert done['status']==done['task_states'][0]['status']==expected
    assert done['task_states'][0]['exit_code']==code
    assert done['progress']==(1.0 if expected=='COMPLETE' else 0.0)
    if expected=='FAILED':assert '退出码' in manager.read_log(row['id'])['text']


@pytest.mark.parametrize('valid,expected',[(True,'COMPLETE'),(False,'FAILED')])
def test_capture_zero_exit_requires_final_durable_manifest(tmp_path,valid,expected):
    project=tmp_path/'project';project.mkdir()
    output=tmp_path/'recording';output.mkdir()
    atomic_json(output/'capture_manifest.json',{'status':'COMPLETE' if valid else 'PARTIAL','recording_complete':valid})
    manager=JobManager(project,tmp_path/'jobs',detached_offline=False)
    tasks=[one_task()]
    with patch('wc_panel.jobs.threading.Thread.start'):
        row=manager.submit('capture','capture fixture',tasks,output=output)
    with patch('wc_panel.jobs.subprocess.Popen',return_value=ExitedSupervisor(0)):
        manager._run(row['id'],tasks,())
    done=manager.get(row['id'])
    assert done['status']==done['task_states'][0]['status']==expected
    assert done['phase']==expected


def test_normal_capture_window_close_is_stopping_during_transfer(tmp_path):
    project=tmp_path/'project';project.mkdir()
    manager=JobManager(project,tmp_path/'jobs',detached_offline=False)
    staged=tmp_path/'staged/progress.json'
    final=tmp_path/'final/progress.json'
    task=one_task(progress_paths=[str(staged),str(final)])
    with patch('wc_panel.jobs.threading.Thread.start'):
        initial=manager.submit('capture','capture fixture',[task])
    row=manager._jobs[initial['id']];row['status']='RUNNING'
    atomic_json(staged,{'stage':'STOPPING_SOURCES','host_monotonic_ns':1})
    manager._capture_progress(row,task)
    assert row['status']=='STOPPING' and row['phase']=='STOPPING_SOURCES'
    atomic_json(staged,{'stage':'TRANSFERRING','host_monotonic_ns':2})
    atomic_json(final,{'stage':'VERIFYING','host_monotonic_ns':3})
    manager._capture_progress(row,task)
    assert row['status']=='STOPPING' and row['phase']=='VERIFYING'
    assert row['stage_message']=='正在核验完整性与落盘'


def test_detached_supervisor_failure_uses_the_gui_log(tmp_path):
    project=tmp_path/'project';project.mkdir()
    manager=JobManager(project,tmp_path/'jobs',detached_offline=True)
    def spawn(*args,**kwargs):
        kwargs['stdout'].write(b'fixture supervisor import failed\n')
        return ExitedSupervisor(1)
    with patch('wc_panel.jobs.subprocess.Popen',side_effect=spawn):
        row=manager.submit('offline','offline fixture',[one_task()])
    done=manager.get(row['id'])
    assert done['status']=='FAILED'
    assert done['task_states'][0]['status']=='SKIPPED'
    assert 'supervisor import failed' in manager.read_log(row['id'])['text']


def test_lost_log_directory_is_not_silently_recreated(tmp_path):
    project=tmp_path/'project';project.mkdir()
    manager=JobManager(project,tmp_path/'jobs',detached_offline=False)
    with patch('wc_panel.jobs.threading.Thread.start'):
        row=manager.submit('live','fixture',[one_task()])
    directory=Path(row['log_path']).parent
    directory.rename(directory.with_name(directory.name+'_retained'))
    with pytest.raises(OSError):manager._save(row)
    manager._failure_log(row,'destination lost')
    assert not directory.exists()


class StartingSupervisor:
    pid=987654321
    returncode=None
    def poll(self):return self.returncode
    def send_signal(self,value):self.returncode=-int(value)
    def wait(self):self.waited=True;return self.returncode


def test_launch_receipt_reattaches_before_worker_bootstrap_without_overwriting_state(tmp_path):
    project=tmp_path/'project';project.mkdir()
    jobs=tmp_path/'jobs'
    manager=JobManager(project,jobs,detached_offline=True)
    identity=dict(pid=StartingSupervisor.pid,start_ticks='fixture',boot_id='fixture_boot')
    with patch('wc_panel.jobs.subprocess.Popen',return_value=StartingSupervisor()), \
            patch('wc_panel.jobs.process_identity',return_value=identity):
        row=manager.submit('offline','fixture',[one_task()])
        directory=Path(row['log_path']).parent
        receipt=json.loads((directory/'worker_launch.json').read_text(encoding='utf-8'))
        assert receipt['worker_identity']==identity
        # A bootstrap can outlast a timing grace: PID/start/boot evidence still
        # identifies it without depending on the launcher process remaining.
        row['submitted_at_ns']=time.time_ns()-60_000_000_000
        atomic_json(directory/'job.json',row)
        reopened=JobManager(project,jobs,detached_offline=True)
        assert reopened.get(row['id'])['status']=='QUEUED'
        assert reopened.jobs()[0]['supervision']=='DETACHED_WORKER'
        assert json.loads((directory/'job.json').read_text(encoding='utf-8'))['status']=='QUEUED'
        assert 'worker_identity' not in row  # receipt, not a stale parent row rewrite


def test_reopen_before_launch_receipt_never_persists_interrupted_startup(tmp_path):
    project=tmp_path/'project';project.mkdir()
    manager=JobManager(project,tmp_path/'jobs',detached_offline=False)
    with patch('wc_panel.jobs.threading.Thread.start'):
        row=manager.submit('offline','fixture',[one_task()])
    row['execution_mode']='detached_offline'
    manager._save(row)
    reopened=JobManager(project,tmp_path/'jobs',detached_offline=True)
    assert reopened.get(row['id'])['status']=='QUEUED'
    assert reopened.jobs()[0]['supervision']=='STARTING_WORKER'
    path=Path(row['log_path']).parent/'job.json'
    assert json.loads(path.read_text(encoding='utf-8'))['status']=='QUEUED'
    identity=dict(pid=987654321,start_ticks='new_worker',boot_id='fixture_boot')
    row.update(worker_identity=identity,status='RUNNING',supervision='DETACHED_WORKER')
    atomic_json(path,row)
    with patch('wc_panel.jobs.process_identity',return_value=identity):
        assert reopened.get(row['id'])['status']=='RUNNING'


def test_expired_unclaimed_startup_is_interrupted_not_queued_forever(tmp_path):
    project=tmp_path/'project';project.mkdir()
    manager=JobManager(project,tmp_path/'jobs',detached_offline=False)
    with patch('wc_panel.jobs.threading.Thread.start'):
        row=manager.submit('offline','fixture',[one_task()])
    row.update(execution_mode='detached_offline',submitted_at_ns=time.time_ns()-60_000_000_000)
    manager._save(row)
    reopened=JobManager(project,tmp_path/'jobs',detached_offline=True)
    assert reopened.get(row['id'])['status']=='INTERRUPTED'
    assert reopened.active_paths()==[]


def test_failed_launch_receipt_stops_owned_worker_before_reporting_failure(tmp_path):
    project=tmp_path/'project';project.mkdir()
    manager=JobManager(project,tmp_path/'jobs',detached_offline=True)
    worker=StartingSupervisor()
    def write(path,value):
        if Path(path).name=='worker_launch.json':raise OSError('fixture launch receipt write failed')
        return atomic_json(path,value)
    with patch('wc_panel.jobs.subprocess.Popen',return_value=worker),patch('wc_panel.jobs.atomic_json',side_effect=write):
        with pytest.raises(RuntimeError,match='receipt write failed'):
            manager.submit('offline','fixture',[one_task()])
    assert worker.returncode is not None and worker.waited
    assert manager.jobs()[0]['status']=='FAILED'
