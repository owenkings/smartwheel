"""Detached offline queue supervisor; survives closing/reopening the GUI."""
import argparse
import json
import os
from pathlib import Path
import signal
import threading
import time
from .jobs import JobManager,process_identity
from .storage import atomic_json,ordinary,resolve_user_destination


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--project-root',type=Path,required=True)
    parser.add_argument('--job-file',type=Path,required=True)
    args=parser.parse_args(argv)
    project=ordinary(args.project_root)
    job_file=ordinary(args.job_file)
    row=json.loads(job_file.read_text(encoding='utf-8'))
    if row.get('kind')!='offline' or row.get('execution_mode')!='detached_offline' or row['status']!='QUEUED':
        raise ValueError('Only a new explicitly queued offline job may start')
    if Path(row['log_path']).parent!=job_file.parent or row['id']!=job_file.parent.name:
        raise ValueError('Offline queue identity/path mismatch')
    launch=json.loads((job_file.parent/'launch.json').read_text(encoding='utf-8'))
    manager=JobManager(project,job_file.parent.parent,detached_offline=False)
    row.update(worker_identity=process_identity(os.getpid()),supervision='DETACHED_WORKER')
    manager._jobs={row['id']:row};manager._save(row)
    identifier=row['id'];cancel_path=job_file.parent/'cancel.json'
    stopped=threading.Event()
    def request_cancel(*unused):
        manager._cancel.add(identifier)
        process=manager._processes.get(identifier)
        if process is not None:manager._signal(process)
        row['status']='STOPPING' if row['status']=='RUNNING' else row['status']
    for sig in (signal.SIGINT,signal.SIGTERM):signal.signal(sig,request_cancel)
    def observe_cancel():
        while not stopped.wait(.2):
            if cancel_path.is_file():
                try:
                    value=json.loads(cancel_path.read_text(encoding='utf-8'))
                    if value.get('job_id')==identifier:request_cancel();return
                except (OSError,ValueError):pass
    watcher=threading.Thread(target=observe_cancel,daemon=True);watcher.start()
    lock_stream=None
    try:
        guards=[]
        for spec in launch.get('guard_specs',[]):
            value=spec.get('output_root')
            if not value:raise ValueError('Persistent offline queue requires an explicit guarded root')
            _,guard=resolve_user_destination(project,value)
            actual=guard.check()
            for key in ('required_uuid','mount_point','device','mount_id'):
                if spec.get(key)!=actual.get(key):raise ValueError('离线队列等待期间存储身份改变: '+key)
            guards.append(guard)
        # Kernel flock serializes workers across GUI instances of this project;
        # it is released automatically if the supervisor dies.
        import fcntl
        lock_root=project/'.phase1_runtime';lock_root.mkdir(exist_ok=True)
        lock_stream=(lock_root/'panel-offline.lock').open('a')
        while identifier not in manager._cancel:
            earlier=False
            for other in job_file.parent.parent.glob('*/job.json'):
                if other==job_file:continue
                try:
                    previous=json.loads(other.read_text(encoding='utf-8'))
                    if (previous.get('execution_mode')!='detached_offline' or previous.get('status')!='QUEUED'
                            or previous.get('submitted_at_ns',0)>=row.get('submitted_at_ns',0)):continue
                    alive,starting=manager._detached_liveness(previous)
                    if alive or starting:
                        earlier=True;break
                except (OSError,ValueError,KeyError):continue
            if earlier:time.sleep(.2);continue
            try:
                fcntl.flock(lock_stream.fileno(),fcntl.LOCK_EX|fcntl.LOCK_NB);break
            except BlockingIOError:time.sleep(.2)
        if identifier in manager._cancel:
            row.update(status='CANCELLED',ended_at=time.time())
            for state in row['task_states']:state['status']='CANCELLED'
            manager._save(row)
        else:manager._run(identifier,launch['tasks'],guards)
    except BaseException as error:
        row.update(status='FAILED',ended_at=time.time(),error=str(error))
        for state in row['task_states']:
            if state['status'] in ('QUEUED','RUNNING'):state.update(status='FAILED',error=str(error))
        manager._save(row)
        manager._failure_log(row,error)
        manager._save_result(row)
        raise
    finally:
        stopped.set();watcher.join(timeout=1)
        if lock_stream is not None:lock_stream.close()
    return 0 if row['status'] in ('COMPLETE','CANCELLED') else 2


if __name__=='__main__':raise SystemExit(main())
