"""Foreground parent for a single managed session, with durable PID identity."""
import argparse
import fcntl
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

from .component import DEFAULT_SIGINT_GRACE_S, RECORD_SIGINT_GRACE_S, TERM_GRACE_S, KILL_GRACE_S, validate_sigint_grace
from .runtime_locks import acquire_resource_lock


def shutdown_policy(plan):
    grace = validate_sigint_grace(plan.get('sigint_grace_s',
        RECORD_SIGINT_GRACE_S if plan.get('role') in ('record', 'drivers') else DEFAULT_SIGINT_GRACE_S))
    # All components receive stop together. Deadlines are shared, so more
    # components do not multiply the allowed cleanup time.
    component_wait = grace + TERM_GRACE_S + KILL_GRACE_S + 5.0
    manifest_wait = 30.0 if plan.get('role') == 'mapping_app' else 1.0
    return {'sigint_grace_s': grace, 'component_wait_s': component_wait,
            'supervisor_term_s': 3.0, 'supervisor_kill_s': 3.0,
            'final_manifest_wait_s': manifest_wait,
            'cli_stop_wait_s': component_wait + 3.0 + 3.0 + manifest_wait}


def finish_children(children, policy):
    """Wait for ownership wrappers to finish/reap; every escalation is failure."""
    failure = 0
    reasons = []
    for child, _ in reversed(children):
        if child.poll() is None:
            try:
                child.send_signal(signal.SIGINT)
            except ProcessLookupError:
                pass  # Exit raced the identity-owned Popen signal.
    pending = list(reversed(children))
    for wait_s, escalation in ((policy['component_wait_s'], signal.SIGTERM),
                               (policy['supervisor_term_s'], signal.SIGKILL),
                               (policy['supervisor_kill_s'], None)):
        deadline = time.monotonic() + wait_s
        survivors = []
        for child, log in pending:
            try:
                rc = child.wait(timeout=max(0, deadline-time.monotonic()))
                if rc:
                    if not failure:
                        failure = rc
                    reasons.append(f'component_cleanup_failure:{child.pid}:{rc}')
            except subprocess.TimeoutExpired:
                failure = failure or 1
                reasons.append(f'component_cleanup_timeout:{child.pid}:{wait_s}')
                if escalation is not None:
                    try:
                        os.killpg(child.pid, escalation)
                    except ProcessLookupError:
                        pass
                survivors.append((child, log))
        pending = survivors
        if not pending:
            break
    for child, _ in pending:
        reasons.append(f'component_cleanup_incomplete:{child.pid}')
    for _, log in children:
        log.close()
    return failure, reasons


def ticks(pid):
    stat = Path(f'/proc/{pid}/stat').read_text()
    return stat[stat.rindex(')')+2:].split()[19]


def write_json(path, data):
    temporary = path.with_name(path.name+'.tmp')
    with temporary.open('w', encoding='utf-8') as stream:
        json.dump(data, stream, indent=2)
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temporary, path)


def stop_registered(manifest_path):
    manifest_path = Path(manifest_path)
    data = json.loads(manifest_path.read_text())
    if data.get('state') != 'RUNNING':
        return False
    pid = data['supervisor_pid']
    pidfd = None
    try:
        pidfd = os.pidfd_open(pid)
        if ticks(pid) != data['supervisor_start_ticks']:
            raise RuntimeError('PID reused: refusing to signal another process')
        command = Path(f'/proc/{pid}/cmdline').read_bytes().rstrip(b'\0').split(b'\0')
        expected = [os.fsencode(sys.executable),b'-m',b'wc_runtime.supervisor',b'--plan',os.fsencode(manifest_path.parent/'plan.json')]
        if command != expected:
            raise RuntimeError('process command does not match session owner')
        signal.pidfd_send_signal(pidfd,signal.SIGTERM)
        return True
    except (FileNotFoundError,ProcessLookupError):
        return False
    finally:
        if pidfd is not None:os.close(pidfd)


def main(argv=None, *, project_root=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--plan', required=True, type=Path)
    args = parser.parse_args(argv)
    plan_path = args.plan.resolve()
    from .project_paths import project_root as discover_project
    project = Path(project_root).resolve() if project_root is not None else discover_project()
    if not plan_path.is_relative_to(project/'.phase1_runtime'):
        raise RuntimeError('plan outside project runtime root')
    plan = json.loads(plan_path.read_text())
    duration = plan.get('duration_s',0)
    if isinstance(duration,bool) or not isinstance(duration,(int,float)) or not math.isfinite(duration) or duration<0:
        raise ValueError('duration must be finite and nonnegative')
    if not plan.get('commands') or any(not isinstance(c,list) or not c or any(not isinstance(v,str) for v in c) for c in plan['commands']):
        raise ValueError('plan requires nonempty argv command lists')
    shutdown = shutdown_policy(plan)
    root = plan_path.parent
    lock_streams = []
    children = []
    stopping = False
    manifest = dict(session_id=plan['session_id'], role=plan['role'], state='STARTING',
                    supervisor_pid=os.getpid(), supervisor_start_ticks=ticks(os.getpid()),
                    started_ns=time.time_ns(), children=[],
                    sigint_grace_s=shutdown['sigint_grace_s'], shutdown=shutdown)
    def signal_stop(signum, frame):
        nonlocal stopping
        stopping = True
    signal.signal(signal.SIGTERM, signal_stop)
    signal.signal(signal.SIGINT, signal_stop)
    result = 0
    try:
        for name in plan['locks']:
            stream = acquire_resource_lock(name)
            lock_streams.append(stream)
        environment = os.environ.copy()
        environment.update(plan['environment'])
        for index, command in enumerate(plan['commands']):
            log_path = root/f'process-{index}.log'
            log = log_path.open('ab', buffering=0)
            wrapped = [sys.executable,'-m','wc_runtime.component','--parent',str(os.getpid()),
                       '--sigint-grace-s',str(shutdown['sigint_grace_s']),'--',*command]
            child = subprocess.Popen(wrapped, cwd=project, env=environment,
                                     stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT,
                                     pass_fds=tuple(stream.fileno() for stream in lock_streams),
                                     start_new_session=True)
            children.append((child, log))
            manifest['children'].append(dict(pid=child.pid,start_ticks=ticks(child.pid),
                                            argv=command,log=str(log_path)))
        manifest['state'] = 'RUNNING'
        write_json(root/'manifest.json',manifest)
        end = time.monotonic()+plan['duration_s'] if plan.get('duration_s') else None
        while not stopping:
            for child, log in children:
                rc = child.poll()
                if rc is not None:
                    failure = rc if rc != 0 else (0 if plan.get('allow_component_exit') else 1)
                    if failure and result==0:result=failure
                    # Every long-running component is necessary. Even an early
                    # zero exit terminates this mapping session visibly.
                    manifest['stop_reason'] = f'component_exit:{child.pid}:{rc}'
                    stopping = True
            if end is not None and time.monotonic() >= end:
                manifest['stop_reason'] = 'duration_reached'
                stopping = True
            time.sleep(0.1)
    except Exception as exc:
        manifest['stop_reason'] = type(exc).__name__+': '+str(exc)
        result = 1
    finally:
        # Only wrappers/process groups created above are signalled. They keep
        # ownership of descendants throughout graceful bag flush and reaping.
        cleanup_rc, cleanup_reasons = finish_children(children, shutdown)
        if cleanup_rc and result == 0:
            result = cleanup_rc
            manifest['stop_reason'] = cleanup_reasons[0]
        if cleanup_reasons:
            manifest['cleanup_errors'] = cleanup_reasons
        manifest['state'] = 'STOPPED' if result==0 else 'FAILED'
        manifest['exit_code'] = result
        manifest['ended_ns'] = time.time_ns()
        write_json(root/'manifest.json',manifest)
        for stream in reversed(lock_streams):
            stream.close()
    return result


if __name__ == '__main__':
    raise SystemExit(main())
