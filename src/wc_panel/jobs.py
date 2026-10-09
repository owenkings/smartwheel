"""Owned child processes, serialized offline work, durable logs and status."""
import copy
import hashlib
from itertools import islice
from datetime import datetime
import json
import os
from pathlib import Path
import signal
import stat
import subprocess
import sys
import threading
import time
import uuid
from .storage import atomic_json

ACTIVE = {'QUEUED', 'RUNNING', 'STOPPING'}
WORKER_START_GRACE_NS = 10_000_000_000


def process_identity(pid):
    if os.name=='nt': return None
    try:
        proc=Path('/proc')/str(pid)
        if proc.stat().st_uid!=os.getuid():return None
        fields=(proc/'stat').read_text().rsplit(')',1)[1].split()
        if fields[0]=='Z':return None
        return dict(pid=pid,start_ticks=fields[19],boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip())
    except (OSError,IndexError):return None


class JobManager:
    def __init__(self, project_root, job_root, *, detached_offline=None, log_guard=None):
        self.project = Path(project_root)
        self.job_root = Path(job_root)
        self._jobs = {}
        self._processes = {}
        self._cancel = set()
        self._lock = threading.RLock()
        self._offline_lock = threading.Lock()
        self.detached_offline=(os.name!='nt') if detached_offline is None else detached_offline
        self._workers={}
        self.log_guard=log_guard
        self._load_history()

    def _detached_liveness(self, row):
        """Recognize a spawned worker before its Python bootstrap writes state.

        The launch receipt is separate from job.json so the parent never
        overwrites a worker's newer RUNNING state with its old QUEUED copy.
        """
        identity=row.get('worker_identity')
        receipt_present=False
        if not identity:
            path=Path(row['log_path']).parent/'worker_launch.json'
            try:
                receipt=json.loads(path.read_text(encoding='utf-8'))
                if (receipt.get('job_id')==row['id'] and
                        receipt.get('log_directory_identity')==row.get('log_directory_identity')):
                    identity=receipt.get('worker_identity')
                    receipt_present=bool(identity)
            except (OSError,ValueError):pass
        alive=bool(identity and process_identity(identity['pid'])==identity)
        owned=self._workers.get(row['id'])
        if owned is not None:return owned.poll() is None,False
        if identity or receipt_present:return alive,False
        # A GUI can attach while Popen is still returning, or just after a hard
        # launcher exit. Only this bounded, still-QUEUED bootstrap window is
        # provisional; it must never be persisted as an interrupted job.
        age=time.time_ns()-row.get('submitted_at_ns',0)
        starting=row['status']=='QUEUED' and 0<=age<WORKER_START_GRACE_NS
        return False,starting

    def _load_history(self):
        if not self.job_root.exists(): return
        for path in sorted(self.job_root.glob('*/job.json'))[-200:]:
            try:
                row = json.loads(path.read_text(encoding='utf-8'))
                if row['status'] in ACTIVE:
                    detached=row.get('execution_mode')=='detached_offline'
                    if detached:alive,starting=self._detached_liveness(row)
                    else:
                        identity=row.get('process_identity')
                        alive=bool(identity and process_identity(identity['pid'])==identity);starting=False
                    if alive or starting:
                        if detached:row['supervision']='STARTING_WORKER' if starting else 'DETACHED_WORKER'
                        else:row.update(status='RUNNING',supervision='REATTACHED_OBSERVER',
                                        error='已核实原任务进程仍运行；可查看日志或正常停止，剩余队列不会自动重启')
                    else:
                        row['status'] = 'INTERRUPTED'
                        row['error'] = '面板已重新启动；原任务进程已不在，未自动重启历史任务'
                self._jobs[row['id']] = row
            except (ValueError, OSError, KeyError):
                continue

    def active_paths(self):
        with self._lock:
            self._refresh_detached()
            return [p for row in self._jobs.values() if row['status'] in ACTIVE
                    for p in row.get('protected_paths', [])]

    def _refresh_detached(self):
        for identifier,old in list(self._jobs.items()):
            if old.get('execution_mode')!='detached_offline':continue
            path=Path(old['log_path']).parent/'job.json'
            try:
                row=json.loads(path.read_text(encoding='utf-8'))
                if row.get('id')!=identifier or row.get('log_path')!=old['log_path']:continue
                owned=self._workers.get(identifier)
                alive,starting=self._detached_liveness(row)
                if starting:row['supervision']='STARTING_WORKER'
                if row['status'] in ACTIVE and not alive and not starting:
                    failed=owned is not None and owned.returncode not in (None,0)
                    message=('离线任务监督进程异常退出 '+str(owned.returncode)+'；详情见任务日志' if failed else
                             '离线任务监督进程已退出；保留已有结果，未自动重跑')
                    row.update(status='FAILED' if failed else 'INTERRUPTED',error=message,ended_at=time.time())
                    for state in row['task_states']:
                        if state['status'] in ACTIVE:
                            state.update(status='SKIPPED' if state['status']=='QUEUED' else row['status'],error=message)
                    self._save_result(row)
                    try:self._save(row)
                    except (OSError,ValueError):pass
                self._jobs[identifier]=row
            except (OSError,ValueError,KeyError):continue

    def jobs(self):
        with self._lock:
            self._refresh_detached()
            for row in self._jobs.values():
                if row.get('supervision')=='REATTACHED_OBSERVER' and row['status'] in ACTIVE:
                    identity=row['process_identity']
                    if process_identity(identity['pid'])!=identity:
                        row.update(status='INTERRUPTED',ended_at=time.time(),error='重启前的进程已结束；请核查产物完整性，剩余队列未执行')
            return copy.deepcopy(list(self._jobs.values()))

    def get(self, identifier):
        with self._lock:
            self._refresh_detached()
            if identifier not in self._jobs: raise ValueError('未知任务')
            return copy.deepcopy(self._jobs[identifier])

    def _log_directory(self, row):
        directory=Path(row['log_path']).parent
        identity=row.get('log_directory_identity')
        if identity is not None:
            info=directory.stat()
            if (directory.is_symlink() or getattr(directory,'is_junction',lambda:False)()
                    or [info.st_dev,info.st_ino]!=identity):
                raise OSError('任务日志目录身份改变，拒绝写入替代位置')
        return directory

    def _save(self, row):
        directory=self._log_directory(row)
        atomic_json(directory/'job.json', row)

    def _save_result(self, row):
        if row['kind']!='offline' or not row.get('output'): return
        try:
            metadata_path=Path(row['output'])/'panel_result.json'
            metadata=json.loads(metadata_path.read_text(encoding='utf-8'))
            metadata.update(status=row['status'],job_id=row['id'],task_states=row['task_states'],error=row['error'])
            atomic_json(metadata_path,metadata)
        except (OSError,ValueError): pass

    def _failure_log(self, row, message):
        # This is also the GUI-visible log for supervisor/launch failures. Never
        # fall back to a different disk when the selected log disk is missing.
        try:
            self._log_directory(row)
            with Path(row['log_path']).open('ab',buffering=0) as stream:
                stream.write(('\nPANEL_ERROR: '+str(message)+'\n').encode('utf-8'))
        except OSError: pass

    def submit(self, kind, label, tasks, *, output=None, protected_paths=(), guards=(), snapshot=None):
        with self._lock:
            if not tasks: raise ValueError('任务必须包含至少一个执行步骤')
            if self.log_guard is not None:self.log_guard.check()
            if kind in ('capture', 'live') and any(row['kind'] in ('capture', 'live') and row['status'] in ACTIVE for row in self._jobs.values()):
                raise ValueError('已有实时或录制任务使用设备，请先正常结束当前任务')
            identifier = datetime.now().strftime('%Y%m%d_%H%M%S') + '_' + uuid.uuid4().hex[:8]
            directory = self.job_root/identifier
            directory.mkdir(parents=True, exist_ok=False)
            info=directory.stat()
            row = dict(id=identifier, kind=kind, label=label, status='QUEUED', progress=0.0, step=0,
                       submitted_at_ns=time.time_ns(),
                       log_directory_identity=[info.st_dev,info.st_ino],
                       total_steps=len(tasks), log_path=str(directory/'runtime.log'), output=str(output) if output else None,
                       started_at=None, ended_at=None, error='', protected_paths=list(map(str, protected_paths)),
                       task_states=[dict(id=t['id'], label=t['label'], status='QUEUED', error='', output=t.get('output')) for t in tasks])
            self._jobs[identifier] = row
            try:
                self._save(row)
                guard_specs=[guard.check() for guard in guards]
                atomic_json(directory/'launch.json', dict(tasks=tasks, snapshot=snapshot,guard_specs=guard_specs))
                if kind=='offline' and self.detached_offline:
                    row.update(execution_mode='detached_offline',supervision='DETACHED_WORKER')
                    self._save(row)
                    env=os.environ.copy()
                    env.update(PYTHONUNBUFFERED='1',PYTHONDONTWRITEBYTECODE='1',
                               PYTHONPATH=str(self.project/'src')+os.pathsep+env.get('PYTHONPATH',''))
                    command=[sys.executable,'-s','-m','wc_panel.job_worker','--project-root',str(self.project),
                             '--job-file',str(directory/'job.json')]
                    # Both supervision and command output are visible through
                    # the same monotonic byte-offset log exposed to the GUI.
                    with Path(row['log_path']).open('ab',buffering=0) as log:
                        self._workers[identifier]=subprocess.Popen(command,cwd=self.project,env=env,stdin=subprocess.DEVNULL,
                                                                  stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
                    self._log_directory(row)
                    atomic_json(directory/'worker_launch.json',dict(job_id=identifier,
                        log_directory_identity=row['log_directory_identity'],
                        worker_identity=process_identity(self._workers[identifier].pid)))
                else:
                    threading.Thread(target=self._run, args=(identifier,tasks,tuple(guards)), daemon=True, name='panel-'+identifier).start()
            except Exception as error:
                worker=self._workers.get(identifier)
                if worker is not None and worker.poll() is None:
                    self._signal(worker)
                    worker.wait()  # A failed receipt cannot leave an unowned queue running.
                row.update(status='FAILED',error='任务未启动: '+str(error),ended_at=time.time())
                for state in row['task_states']:state.update(status='SKIPPED',error=row['error'])
                self._failure_log(row,row['error'])
                self._save_result(row)
                try:self._save(row)
                except (OSError,ValueError):pass
                raise RuntimeError(row['error']+'；任务记录 '+identifier+'，详情见日志') from error
            return copy.deepcopy(row)

    def _capture_progress(self, row, task):
        if row['kind']!='capture': return
        values=[]
        for value in task.get('progress_paths',[]):
            path=Path(value)
            try:
                from .storage import ordinary
                ordinary(path)
                if path.stat().st_size>16384:continue
                progress=json.loads(path.read_text(encoding='utf-8'))
                if isinstance(progress,dict) and type(progress.get('host_monotonic_ns')) is int:
                    values.append(progress)
            except (OSError,ValueError):continue
        if not values:return
        progress=max(values,key=lambda value:value['host_monotonic_ns'])
        stage=progress.get('stage')
        messages={'PREPARING':'准备录制','STARTING_SOURCES':'启动数据来源','WAITING_FOR_WHEEL':'等待轮反馈',
                  'WAITING_FOR_ALL_SOURCES':'等待所有选定来源','RECORDING':'录制中',
                  'STOPPING_SOURCES':'正在停止来源并排空录包','TRANSFERRING':'正在写入最终保存目录',
                  'VERIFYING':'正在核验完整性与落盘'}
        if stage not in messages or stage==row.get('phase'):return
        row.update(phase=stage,stage_message=messages[stage])
        if stage in ('STOPPING_SOURCES','TRANSFERRING','VERIFYING'):
            row['status']='STOPPING'
        self._save(row)

    def _run(self, identifier, tasks, guards):
        row = self._jobs[identifier]
        locked = False
        final_status = None
        try:
            if row['kind'] == 'offline':
                while not self._offline_lock.acquire(timeout=.2):
                    if identifier in self._cancel: return
                locked = True
            if identifier in self._cancel: return
            row.update(status='RUNNING', started_at=time.time())
            self._save(row)
            env = os.environ.copy()
            env.update(WHEELCHAIR_PROJECT_ROOT=str(self.project), PYTHONNOUSERSITE='1', PYTHONDONTWRITEBYTECODE='1',
                       PYTHONUNBUFFERED='1', PYTHONPATH=str(self.project/'src')+os.pathsep+env.get('PYTHONPATH',''))
            with Path(row['log_path']).open('ab', buffering=0) as log:
                for index, task in enumerate(tasks):
                    if identifier in self._cancel: break
                    for guard in guards: guard.check()
                    for filename,expected in task.get('input_configuration_hashes',{}).items():
                        path=Path(filename)
                        if path.is_symlink() or hashlib.sha256(path.read_bytes()).hexdigest()!=expected:
                            raise ValueError('排队期间原录包配置改变，已停止任务: '+filename)
                    state = row['task_states'][index]
                    state['status'] = 'RUNNING'
                    pending=False
                    completed_commands=0
                    row['step'] = index+1
                    self._save(row)
                    log.write(('\n=== '+task['label']+' ===\n').encode('utf-8'))
                    for command in task['commands']:
                        if identifier in self._cancel: break
                        log.write((json.dumps(command, ensure_ascii=False)+'\n').encode('utf-8'))
                        with self._lock:
                            if identifier in self._cancel: break
                            process = subprocess.Popen(command, cwd=self.project, env=env, stdin=subprocess.DEVNULL,
                                                       stdout=log, stderr=subprocess.STDOUT, start_new_session=os.name!='nt')
                            self._processes[identifier] = process
                            row['process_identity']=process_identity(process.pid)
                            self._save(row)
                        guard_error = None
                        while process.poll() is None:
                            time.sleep(.4)
                            self._capture_progress(row,task)
                            if guard_error is not None: continue
                            try:
                                for guard in guards: guard.check()
                            except Exception as error:
                                guard_error = error
                                row.update(status='STOPPING',error='存储不可用，等待任务正常收尾: '+str(error))
                                self._signal(process)
                        with self._lock: self._processes.pop(identifier, None)
                        if guard_error is not None: raise guard_error
                        state['exit_code']=process.returncode
                        cancelled_offline=identifier in self._cancel and row['kind']=='offline'
                        if process.returncode not in task.get('accepted_exit_codes',[0]) and not cancelled_offline:
                            raise RuntimeError('命令退出码 '+str(process.returncode)+'；详情见任务日志')
                        if process.returncode==3 and 3 in task.get('accepted_exit_codes',[]):pending=True
                        completed_commands+=1
                    if row['kind']=='capture' and completed_commands:
                        manifest_path=Path(row['output'])/'capture_manifest.json'
                        manifest=json.loads(manifest_path.read_text(encoding='utf-8')) if manifest_path.is_file() else {}
                        if manifest.get('status')!='COMPLETE' or manifest.get('recording_complete') is not True:
                            raise RuntimeError('录制已停止，但完整性审计未通过；已保留原数据，请查看日志')
                    cancelled=(identifier in self._cancel and
                               (row['kind']=='offline' or completed_commands<len(task['commands'])))
                    state['status'] = 'PENDING' if pending else 'CANCELLED' if cancelled else 'COMPLETE'
                    row['progress'] = (index+1 if state['status']=='COMPLETE' else index)/len(tasks)
                    self._save(row)
            if any(state['status']=='PENDING' for state in row['task_states']):final_status='PENDING'
            elif all(state['status']=='COMPLETE' for state in row['task_states']):final_status='COMPLETE'
            else:final_status='CANCELLED' if identifier in self._cancel else 'FAILED'
        except Exception as error:
            process=self._processes.get(identifier)
            if process is not None and process.poll() is None:
                row.update(status='STOPPING',error=str(error))
                self._signal(process)
                process.wait()  # Keep its data protected until owned cleanup ends.
            with self._lock:self._processes.pop(identifier,None)
            final_status='FAILED'
            row['error']=str(error)
            self._failure_log(row,error)
            for state in row['task_states']:
                if state['status'] == 'RUNNING': state.update(status='FAILED', error=str(error))
        finally:
            # Publish terminal status only after command/log cleanup and final
            # task states. Observers must not see FAILED with RUNNING children.
            with self._lock:
                if final_status is not None:row['status']=final_status
                elif identifier in self._cancel:row['status']='CANCELLED'
                for state in row['task_states']:
                    if state['status'] == 'QUEUED': state['status'] = 'CANCELLED' if identifier in self._cancel else 'SKIPPED'
                row['ended_at'] = time.time()
                if row['kind']=='capture':
                    row.update(last_phase=row.get('phase'),phase=row['status'],stage_message={
                        'COMPLETE':'录制完成，完整性与最终落盘已确认',
                        'FAILED':'录制未完整结束，已保留可用数据；请查看日志',
                        'CANCELLED':'录制启动已取消'}.get(row['status'],row['status']))
                self._save_result(row)
                try: self._save(row)
                except OSError: pass  # A detached destination cannot silently move the log elsewhere.
            if locked: self._offline_lock.release()

    @staticmethod
    def _signal(process):
        if process.poll() is None:
            # Signal only the owned supervisor. It handles device and rosbag
            # finalization in dependency order; a group kill would bypass it.
            process.send_signal(signal.SIGINT if os.name != 'nt' else signal.SIGTERM)

    def cancel(self, identifier):
        with self._lock:
            self._refresh_detached()
            row = self._jobs.get(identifier)
            if row is None: raise ValueError('未知任务')
            if row['status'] not in ACTIVE: return copy.deepcopy(row)
            if row.get('execution_mode')=='detached_offline':
                # A durable request survives a GUI crash and closes the race
                # before the detached supervisor installs signal handlers.
                atomic_json(Path(row['log_path']).parent/'cancel.json',dict(job_id=identifier,requested_at=time.time()))
                return copy.deepcopy(row)
            self._cancel.add(identifier)
            if row['status'] == 'RUNNING': row['status'] = 'STOPPING'
            process = self._processes.get(identifier)
            if process is not None: self._signal(process)
            elif row.get('supervision')=='REATTACHED_OBSERVER':
                identity=row.get('process_identity')
                if identity and process_identity(identity['pid'])==identity:
                    os.kill(identity['pid'],signal.SIGINT)
                    row['status']='STOPPING'
            self._save(row)
            return copy.deepcopy(row)

    def list_logs(self, identifier):
        """List only supervisor logs and logs beneath registered task outputs.

        IDs are opaque selectors, never caller-supplied filesystem paths. A
        missing or unsafe stage remains visible without hiding other logs.
        """
        from .storage import ordinary, _mount_points, resolve_user_destination
        row = self.get(identifier)
        entries=[]
        def add(log_id, label, path, error=''):
            item=dict(id=log_id,label=label,path=str(path),bytes=None,available=False,error=error)
            if not error:
                try:
                    selected=ordinary(path)
                    info=selected.stat()
                    if not stat.S_ISREG(info.st_mode):raise ValueError('日志不是普通文件')
                    item.update(bytes=info.st_size,available=True)
                except (OSError,ValueError) as error:item['error']=str(error)
            entries.append(item)
        runtime_error=''
        try:
            if self.log_guard is not None:self.log_guard.check()
            self._log_directory(row)
            ordinary(row['log_path'])
        except (OSError,ValueError) as error:runtime_error=str(error)
        add('runtime','任务主日志',Path(row['log_path']),runtime_error)
        # Only offline commands currently write separate native-stage logs.
        if row.get('kind')!='offline':return entries
        try:
            directory=ordinary(self._log_directory(row))
            launch_path=ordinary(directory/'launch.json')
            launch=json.loads(launch_path.read_text(encoding='utf-8'))
            specs=launch.get('guard_specs',[])
            mounts=_mount_points()
        except (OSError,ValueError) as error:
            add('stages_unavailable','阶段日志暂不可用',Path(row['log_path']).parent,str(error));return entries
        for task in row.get('task_states',[]):
            if not task.get('output'):continue
            root=Path(task['output'])
            prefix=hashlib.sha256(str(task['id']).encode()).hexdigest()[:16]
            try:
                ordinary(root)
                if not row.get('output') or not root.is_relative_to(ordinary(row['output'])):
                    raise ValueError('阶段输出不在本任务结果目录中')
                for spec in specs:
                    if not spec.get('output_root') or not root.is_relative_to(Path(spec['output_root'])):continue
                    _,guard=resolve_user_destination(self.project,spec['output_root'])
                    actual=guard.check()
                    for key in ('required_uuid','mount_point','device','mount_id'):
                        if actual.get(key)!=spec.get(key):raise ValueError('阶段日志存储身份改变: '+key)
                if not root.exists():continue  # queued stage has no output yet
                if root in mounts:raise ValueError('阶段结果目录变为挂载点，拒绝读取替代内容')
                device=root.stat().st_dev
                pending=[(root,0)];visited=0
                while pending:
                    parent,depth=pending.pop()
                    if depth>8:raise ValueError('阶段日志目录超过扫描深度限制')
                    try:
                        ordinary(parent)
                        if parent.stat().st_dev!=device:raise ValueError('阶段日志目录跨挂载')
                        # Do not materialize an unbounded directory before the
                        # stage's entry limit is checked (large native exports).
                        with os.scandir(parent) as iterator:
                            children=[Path(entry.path) for entry in islice(iterator,8193-visited)]
                    except (OSError,ValueError) as error:
                        add(prefix+'_unreadable_'+str(visited),task['label']+' / '+parent.name,parent,str(error));continue
                    for path in children:
                        visited+=1
                        if visited>8192:raise ValueError('阶段日志目录超过扫描条目限制')
                        relative=path.relative_to(root).as_posix()
                        log_id=hashlib.sha256((str(task['id'])+'\0'+relative).encode()).hexdigest()
                        label=task['label']+' / '+relative
                        try:
                            info=path.lstat()
                            if (stat.S_ISLNK(info.st_mode) or getattr(path,'is_junction',lambda:False)()
                                    or info.st_dev!=device or path in mounts):
                                raise ValueError('不读取链接或跨挂载阶段日志')
                            if stat.S_ISDIR(info.st_mode):pending.append((path,depth+1))
                            elif path.suffix.lower()=='.log':add(log_id,label,path)
                        except (OSError,ValueError) as error:add(log_id,label,path,str(error))
            except (OSError,ValueError) as error:add(prefix+'_unavailable',task['label']+' / 阶段日志',root,str(error))
        return entries

    def read_log(self, identifier, offset=0, log_id=None):
        row = self.get(identifier)
        if type(offset) is not int or offset < 0: raise ValueError('日志偏移必须为非负整数')
        selected_id='runtime' if log_id is None else log_id
        matches=[item for item in self.list_logs(identifier) if item['id']==selected_id]
        if not matches:raise ValueError('未知或不属于本任务的日志选项')
        selected=matches[0]
        if not selected['available']:
            return dict(text='',offset=offset,status=row['status'],log_id=selected_id,available=False,error=selected['error'])
        from .storage import ordinary
        path=ordinary(selected['path'])
        descriptor=os.open(path,os.O_RDONLY|getattr(os,'O_NOFOLLOW',0)|getattr(os,'O_BINARY',0))
        with os.fdopen(descriptor,'rb') as stream:
            if not stat.S_ISREG(os.fstat(stream.fileno()).st_mode):raise ValueError('日志不是普通文件')
            stream.seek(offset)
            data = stream.read(512*1024)
            # Keep an incomplete UTF-8 suffix for the next read.
            end = len(data)
            while end and end > len(data)-4:
                try:
                    text = data[:end].decode('utf-8')
                    break
                except UnicodeDecodeError as exc:
                    if exc.reason == 'unexpected end of data': end -= 1
                    else: text = data.decode('utf-8', errors='replace'); end=len(data); break
            else: text = ''
            return dict(text=text, offset=offset+end, status=row['status'],log_id=selected_id,available=True,error='')
