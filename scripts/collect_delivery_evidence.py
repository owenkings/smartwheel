#!/usr/bin/env python3
"""Read project evidence and verify owned process identities; never stop a process."""
import datetime
import errno
import getpass
import fcntl
import hashlib
import json
import os
from pathlib import Path
import platform
import socket
import subprocess
import sys

# Locate this checkout from either scripts/ or its in-tree ROS install.
# Never select a project merely because it is the caller's current directory.
sys.dont_write_bytecode = True
_candidates = list(Path(__file__).resolve().parents)
for _candidate in _candidates:
    if ((_candidate/'src/wc_runtime').is_dir() and
            (_candidate/'config/storage.json').is_file() and
            (_candidate/'scripts/wc_phase1').is_file()):
        sys.path.insert(0, str(_candidate/'src'))
        break
else:
    raise RuntimeError('Cannot locate the wheelchair checkout; set WHEELCHAIR_PROJECT_ROOT')
from wc_runtime.project_paths import project_root
ROOT = project_root(start=__file__)
from wc_runtime.storage_policy import StoragePolicy
from wc_runtime.project_paths import require_linux_runtime


def digest(path):
    h = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024*1024), b''):
            h.update(block)
    return h.hexdigest()


def process_state(pid, ticks, proc_root=Path('/proc')):
    try:
        if type(pid) is not int or pid <= 0 or not str(ticks).isdigit():
            return 'UNKNOWN'
        value = (proc_root/str(pid)/'stat').read_text()
        columns = value[value.rfind(')')+2:].split()
        if str(columns[19]) != str(ticks):
            return 'REPLACED'
        return 'ZOMBIE' if columns[0] == 'Z' else 'ALIVE'
    except OSError as error:
        return 'DEAD' if error.errno in (errno.ENOENT,errno.ESRCH) else 'UNKNOWN'
    except (ValueError, TypeError, IndexError):
        return 'UNKNOWN'


def main():
    require_linux_runtime()
    stamp = datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%SZ')
    storage = StoragePolicy(ROOT)
    from wc_runtime.development_archive import create_task
    output = create_task(ROOT, 'delivery_evidence') / 'reports'
    evidence = {}
    for folder, patterns in {
        'reports/test_runs': ['*.json','*.xml'],
        'dev_archive': ['*/validation/run.json', '*/validation/junit.xml',
                        '*/validation/builds/*.log', '*/validation/builds/*/logger_all.log'],
        'reports/synthetic': ['*/ros_pipeline.json','*/resources.json','*/tf_native.json'],
        'reports/live_static': ['*-bag_audit.json','*/sha256.json'],
        'reports/real_bag_replay': ['*/result.json'],
        'reports/saved_maps': ['*/result.json'],
        'reports/operator_cli': ['*/result.json'],
        'reports/rviz': ['*/result.json','*/3d.png','*/2d.png'],
        'reports/ablation': ['*.json'],
        'reports/performance': ['*.json'],
        'reports/ros_contracts': ['*/full.log','*/h30_cdr.json','*/fusion_faults/*.json'],
        'reports/builds': ['*/full.log','*/exit_code','native_tests.log','shutdown_repeat10.log'],
        'reports/final_validation': ['*/full.log','*/operator_build.log','*/synthetic_exit_code'],
        'reports/graph_delivery': ['*.log','*exit_code','*/*.log','*/*exit_code'],
    }.items():
        for pattern in patterns:
            for path in sorted(storage.resolve(folder).glob(pattern)):
                if path.is_file() and not path.is_symlink():
                    evidence[storage.logical(path)] = dict(bytes=path.stat().st_size,sha256=digest(path))
    sessions = []
    for path in sorted((ROOT/'.phase1_runtime/sessions').glob('*/*/manifest.json')):
        data = json.loads(path.read_text())
        registered = []
        registered.append(dict(role='supervisor',pid=data['supervisor_pid'],start_ticks=data['supervisor_start_ticks']))
        for entry in data.get('children',[]):
            registered.append(dict(role='component',pid=entry.get('pid'),start_ticks=entry.get('start_ticks')))
        for item in registered:
            item['inspection_state'] = process_state(item['pid'],item['start_ticks'])
            item['same_process_alive'] = item['inspection_state'] == 'ALIVE'
            item['verified_not_alive'] = item['inspection_state'] in ('DEAD','REPLACED','ZOMBIE')
        sessions.append(dict(manifest=str(path.relative_to(ROOT)),state=data.get('state'),
                             exit_code=data.get('exit_code'),registered=registered))
    locks = {}
    for path in sorted((ROOT/'.phase1_runtime/locks').glob('*.lock')):
        if path.is_symlink():
            locks[path.name] = 'UNVERIFIED_SYMLINK'
            continue
        with path.open('rb') as stream:
            try:
                fcntl.flock(stream,fcntl.LOCK_EX|fcntl.LOCK_NB)
                locks[path.name] = 'FREE_AT_OBSERVATION'
                fcntl.flock(stream,fcntl.LOCK_UN)
            except BlockingIOError:
                locks[path.name] = 'OWNED_AT_OBSERVATION'
    udp = subprocess.check_output(['ss','-H','-u','-l','-n'],text=True)
    udp = [line for line in udp.splitlines() if ':7687 ' in line]
    ancestors = set()
    pid = os.getpid()
    while pid > 1 and pid not in ancestors:
        ancestors.add(pid)
        try:
            value = Path(f'/proc/{pid}/stat').read_text()
            pid = int(value[value.rfind(')')+2:].split()[1])
        except (OSError,ValueError):
            break
    project_cwd_processes = []
    for directory in Path('/proc').glob('[0-9]*'):
        try:
            pid = int(directory.name)
            if pid in ancestors or directory.stat().st_uid != os.getuid():
                continue
            cwd = Path(os.readlink(directory/'cwd'))
            if cwd.is_relative_to(ROOT):
                value = (directory/'stat').read_text()
                columns = value[value.rfind(')')+2:].split()
                project_cwd_processes.append(dict(pid=pid,start_ticks=columns[19],state=columns[0],
                                                   comm=(directory/'comm').read_text().strip(),cwd=str(cwd)))
        except (OSError,ValueError):
            continue
    sources = {str(path.relative_to(ROOT)):digest(path) for folder in ('src','tests','config','scripts','vendor_patches')
               for path in sorted((ROOT/folder).rglob('*')) if path.is_file() and not path.is_symlink()
               and not {'__pycache__','.pytest_cache'} & set(path.parts)}
    installed = {}
    for relative in ('install/main/wc_xt_driver/lib/wc_xt_driver/xt_source_node',
                     'install/main/wc_xt_driver/share/wc_xt_driver/provenance/vendor_provenance.json',
                     'install/main/wc_slam/lib/wc_slam/wc_graph_core_node',
                     'install/main/wc_bringup/local/lib/python3.10/dist-packages/wc_fusion/ros_node.py'):
        path = ROOT/relative
        installed[relative] = dict(sha256=digest(path),bytes=path.stat().st_size) if path.is_file() else 'MISSING'
    identity = dict(hostname=socket.gethostname(),user=getpass.getuser(),
                    uid=os.getuid(),architecture=platform.machine(),root=str(ROOT))
    report = dict(created_utc=stamp,target=identity,formal_phase1_acceptance='NOT_PASSED',
                  evidence=evidence,source_sha256=sources,installed_artifacts=installed,sessions=sessions,locks=locks,udp_7687_listeners=udp,
                  no_registered_process_alive=all(item['verified_not_alive'] for s in sessions for item in s['registered']),
                  unknown_registered_processes=[item for s in sessions for item in s['registered'] if item['inspection_state']=='UNKNOWN'],
                  no_running_session_manifest=not any(s['state']=='RUNNING' for s in sessions),
                  starting_session_manifests=[s['manifest'] for s in sessions if s['state']=='STARTING'],
                  historical_running_manifests_without_live_registered_process=[s['manifest'] for s in sessions
                      if s['state']=='RUNNING' and all(item['verified_not_alive'] for item in s['registered'])],
                  other_project_cwd_processes=project_cwd_processes,
                  observations_only='Historical evidence is bound to its own source hashes, not automatically to current source/installed hashes. Lock checks cover existing lock files only; ss covers listening UDP sockets only; PID checks cover registered identities. These observations do not prove future ownership, every descendant process, or physical vehicle state.')
    storage.check()
    (output/'evidence_manifest.json').write_text(json.dumps(report,indent=2))
    latest = ROOT/'.phase1_runtime/state/latest_delivery.txt'
    latest.parent.mkdir(parents=True, exist_ok=True)
    latest.write_text(storage.logical(output)+'\n')
    print(json.dumps(dict(output=str(output),evidence_count=len(evidence),
                          no_registered_process_alive=report['no_registered_process_alive'],
                          no_running_session_manifest=report['no_running_session_manifest'],udp_7687_listeners=udp)))


if __name__ == '__main__':
    main()
