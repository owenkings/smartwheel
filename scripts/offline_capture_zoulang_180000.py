#!/usr/bin/env python3
"""One offline acquisition attempt. Uses the existing recorder; never drives."""
import argparse
import datetime as dt
import fcntl
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time


def load_config(path):
    value=json.loads(Path(path).read_text())
    required={'schema_version','job_id','start_unix','latest_start_unix','duration_seconds',
              'folder_name','project_root','output_root','required_uuid','capture_sha256'}
    if set(value)!=required or value['schema_version']!=1:
        raise ValueError('Unsupported offline job configuration')
    if value['job_id']!='zoulang_20261011_180000' or value['folder_name']!='zoulang':
        raise ValueError('Unexpected job or folder identity')
    expected=int(dt.datetime(2026,10,11,18,0,tzinfo=dt.timezone(dt.timedelta(hours=8))).timestamp())
    if value['start_unix']!=expected or value['latest_start_unix']!=expected+120:
        raise ValueError('Unexpected start window')
    if value['duration_seconds']!=600:
        raise ValueError('This job requires a 600-second common data window')
    if value['project_root']!='/home/nvidia/wheelchair':
        raise ValueError('Unexpected project root')
    if value['output_root']!='/media/nvidia/tiany-USB/wheelchair/data/experiments' or value['required_uuid']!='EFFA-F777':
        raise ValueError('Unexpected archive identity')
    if not isinstance(value['capture_sha256'],str) or len(value['capture_sha256'])!=64:
        raise ValueError('Missing reviewed recorder hash')
    return value


def time_gate(config, now):
    if now<config['start_unix']:
        raise RuntimeError('NOT_DUE: refusing early acquisition')
    if now>config['latest_start_unix']:
        raise RuntimeError('START_WINDOW_EXPIRED: no delayed acquisition')


def command(config, dry_run=False):
    result=['/usr/bin/python3','-u','-s',config['project_root']+'/scripts/capture_usb',
            '--profile','mapping_core','--sides','all','--session',config['job_id'],
            '--folder-name',config['folder_name'],'--duration',str(config['duration_seconds']),
            '--staging','memory','--output-root',config['output_root'],
            '--required-output-uuid',config['required_uuid']]
    if dry_run: result.append('--dry-run')
    return result


def atomic_json(path, value):
    temp=path.with_name(path.name+'.tmp')
    with temp.open('w',encoding='utf-8') as stream:
        json.dump(value,stream,ensure_ascii=False,indent=2)
        stream.write('\n');stream.flush();os.fsync(stream.fileno())
    os.replace(temp,path)
    fd=os.open(path.parent,os.O_RDONLY|os.O_DIRECTORY)
    try: os.fsync(fd)
    finally: os.close(fd)


def read_json(path):
    try: return json.loads(path.read_text())
    except (OSError,ValueError): return {}


def require_persistent_user():
    # logind RemoveIPC otherwise deletes this user's /dev/shm data on logout.
    if os.getuid()!=1000:
        raise RuntimeError('UNEXPECTED_CAPTURE_USER: expected uid 1000')
    probe=subprocess.run(['loginctl','show-user','1000','-p','Linger','--value'],
                         text=True,capture_output=True,timeout=10)
    if probe.returncode!=0 or probe.stdout.strip()!='yes':
        raise RuntimeError('OFFLINE_CACHE_NOT_PROTECTED: enable linger for nvidia before RAM capture')


def run(config_path, check_only=False):
    config=load_config(config_path)
    root=Path(config['project_root'])
    actual=hashlib.sha256((root/'src/wc_runtime/capture.py').read_bytes()).hexdigest()
    if actual!=config['capture_sha256']:
        raise RuntimeError('Recorder changed since deployment; review required')
    if check_only:
        print(json.dumps({'status':'CONFIG_VALID','command':command(config),
                          'start_unix':config['start_unix'],'motor_control':False}))
        return 0
    directory=Path(config_path).parent
    with (directory/'runner.lock').open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        state={'job_id':config['job_id'],'scheduled_start_unix':config['start_unix'],
               'duration_seconds':600,'motor_control':False,'status':'WAITING_FOR_DEVICES'}
        def status(stage, **details):
            state.update(status=stage,updated_unix=time.time(),**details)
            atomic_json(directory/'status.json',state)
            print(json.dumps(state,ensure_ascii=False),flush=True)
        stopped=[False]
        for sig in (signal.SIGINT,signal.SIGTERM,signal.SIGHUP):
            signal.signal(sig,lambda *_:stopped.__setitem__(0,True))
        child=None
        try:
            if (directory/'attempt.json').exists():
                previous=read_json(directory/'status.json')
                print(json.dumps({'status':'ALREADY_ATTEMPTED','previous_status':previous.get('status'),
                                  'acquisition_restarted':False}),flush=True)
                return 0 if previous.get('status')=='COMPLETE' else 2
            time_gate(config,time.time())
            require_persistent_user()
            env=dict(os.environ,PYTHONNOUSERSITE='1',PYTHONDONTWRITEBYTECODE='1',
                     PYTHONUNBUFFERED='1',WHEELCHAIR_PROJECT_ROOT=str(root),
                     OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',LC_ALL='C.UTF-8')
            while not stopped[0]:
                time_gate(config,time.time())
                if (Path(config['output_root'])/config['folder_name']).exists():
                    raise RuntimeError('DESTINATION_EXISTS: existing zoulang data will not be overwritten')
                probe=subprocess.run(command(config,True),cwd=root,env=env,text=True,
                                     capture_output=True,timeout=75)
                try: evidence=json.loads(probe.stdout)
                except ValueError: evidence={'error':probe.stderr[-2500:],'stdout':probe.stdout[-1500:]}
                atomic_json(directory/'preflight.json',evidence)
                checks=evidence.get('preflight',{})
                blocked={k:v for k,v in checks.items() if v.get('status')=='BLOCKED'}
                sufficient=all(checks.get(k,{}).get('sufficient') is True for k in ('capacity','memory_capacity'))
                if probe.returncode==0 and checks and not blocked and sufficient:
                    break
                status('WAITING_FOR_DEVICES',blocked=blocked,preflight_code=probe.returncode,
                       capacity_forecast_sufficient=sufficient)
                time.sleep(5)
            if stopped[0]: raise RuntimeError('CANCELLED_BEFORE_ACQUISITION')
            time_gate(config,time.time())
            with (directory/'attempt.json').open('x') as stream:
                json.dump({'started_unix':time.time(),'boot_id':Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
                           'command':command(config)},stream)
                stream.flush();os.fsync(stream.fileno())
            fd=os.open(directory,os.O_RDONLY|os.O_DIRECTORY)
            try: os.fsync(fd)
            finally: os.close(fd)
            status('STARTING',blocked={},final_directory=str(Path(config['output_root'])/config['folder_name']))
            with (directory/'capture.log').open('xb') as log:
                child=subprocess.Popen(command(config),cwd=root,env=env,stdin=subprocess.DEVNULL,
                                       stdout=log,stderr=subprocess.STDOUT)
                status('STARTING',capture_pid=child.pid)
                forwarded=False;last_stage=None
                staging=Path('/dev/shm/wc_capture_1000')/config['job_id']
                final=Path(config['output_root'])/config['folder_name']
                while child.poll() is None:
                    if stopped[0] and not forwarded:
                        child.send_signal(signal.SIGINT);forwarded=True
                        status('STOP_REQUESTED')
                    progress=read_json(staging/'progress.json') or read_json(final/'progress.json')
                    stage=progress.get('stage')
                    if stage and stage!=last_stage:
                        status(stage,progress=progress);last_stage=stage
                    time.sleep(1)
                result=read_json(final/'capture_manifest.json')
                complete=(child.returncode==0 and result.get('status')=='COMPLETE'
                          and result.get('recording_complete') is True
                          and result.get('storage_staging',{}).get('durable_archive') is True)
                status('COMPLETE' if complete else 'FAILED',exit_code=child.returncode,
                       archive_status=result.get('status'),recording_complete=result.get('recording_complete',False),
                       stop_reason=result.get('stop_reason'),completed_unix=time.time())
                return 0 if complete else 2
        except BaseException as error:
            if child is not None and child.poll() is None:
                child.send_signal(signal.SIGINT);child.wait()
            status('FAILED',error=type(error).__name__+': '+str(error))
            return 2


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config',type=Path,required=True)
    parser.add_argument('--check-config',action='store_true')
    args=parser.parse_args()
    raise SystemExit(run(args.config,args.check_config))
