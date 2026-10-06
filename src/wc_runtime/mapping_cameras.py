"""Optional four-camera preview companion; never records images or controls motion.

Camera failures degrade this companion while healthy slots keep running. Only
owned processes are stopped. The mapping estimator has no dependency on camera
images, calibration or availability.
"""
import argparse
from collections import deque
import copy
import ctypes
import json
import os
from pathlib import Path
import re
import signal
import stat
import subprocess
import sys
import threading
import time

from wc_cameras.config import CameraError, ROLES, load_config, number, topic
from .prepare_picker_input import project_path, read_json
from . import component
from .mapping_shutdown import persistence_wait


CAMERA_LOCK = 'domain-83-cameras.lock'
KILL_SIGNAL = getattr(signal, 'SIGKILL', 9)  # Pure Windows fixtures; real CLI requires Linux.
STOP_PHASES = ((signal.SIGINT, 7.5), (signal.SIGTERM, .75), (KILL_SIGNAL, .75))
# Shared camera stop waits total 9 s; reserve a further 10 s for the independent
# metadata writer by default. The mapping app explicitly selects a longer
# persistence-only wait inside its own larger component shutdown budget.
STATUS_CLOSE_S = 10.
MAX_RESTARTS_PER_SLOT = 2
RESTART_DELAY_S = 1.
READ_DEADLINE_ERROR = 'camera read deadline exceeded; capture child will be stopped'


def restart_eligibility(row):
    """Only a known read deadline with affirmative normal-release evidence.

    Exit alone does not prove that a capture worker released its device. The
    replacement node must still reacquire CameraLease and repeat identity and
    occupancy checks before opening V4L2; this function never touches a device.
    """
    if row['exit_code'] != 1 or not row['reader_done'] or row.get('reader_error'):
        return False, 'node exit/final output not confirmed'
    stopped = row['capture_stop']
    if not row['capture_ready'] or not isinstance(stopped, dict) or stopped.get('failure_latched') is not True \
            or stopped.get('error') != READ_DEADLINE_ERROR:
        return False, 'failure is not the reviewed read-deadline recovery case'
    cleanup = stopped.get('cleanup', {})
    if not isinstance(cleanup, dict) or cleanup.get('status') != 'PASS' or cleanup.get('termination') != 'normal' \
            or cleanup.get('alive_after_cleanup') is not False or cleanup.get('final_exit_code') != 0 \
            or cleanup.get('errors') != [] or cleanup.get('late_event_drain') != 'DRAINED_AFTER_CHILD_EXIT':
        return False, 'normal capture exit and complete cleanup not confirmed'
    reaped = cleanup.get('reap_observed_host_monotonic_ns')
    if type(cleanup.get('owned_pid')) is not int or cleanup['owned_pid'] <= 0 or type(reaped) is not int or reaped <= 0:
        return False, 'owned capture reap identity/time missing'
    events = cleanup.get('late_events')
    if not isinstance(events, list):
        return False, 'complete release event list missing'
    for event in events:
        if not isinstance(event, dict) or event.get('event') != 'RELEASE_COMPLETE':
            continue
        begin, end, elapsed = (event.get(key) for key in ('host_release_start_monotonic_ns',
                            'host_release_complete_monotonic_ns', 'host_release_elapsed_ns'))
        if all(type(value) is int for value in (begin, end, elapsed)) and 0 < begin <= end <= reaped and elapsed == end-begin:
            return True, 'normal release/reap confirmed; replacement must recheck identity and exclusive lease'
    return False, 'complete device release timing missing'


def child_commands(config_path, config, session_id, run_root, duration, *, executable=sys.executable):
    """The outer ros_command already supplies the reviewed ROS environment."""
    profile = config['default_profile']
    cameras = {row['role']: row for row in config['cameras']}
    return [{'role': role, 'port': cameras[role]['port'], 'topic': topic(role), 'profile': profile,
             'argv': [str(executable), '-s', '-m', 'wc_cameras.node', '--config', str(config_path),
                      '--role', role, '--profile', profile, '--session-id', session_id,
                      '--run-root', str(run_root), '--duration', str(duration)]} for role in ROLES]


class DomainLease:
    """The same flock used by wc_phase1 cameras; per-device leases remain in node."""
    def __init__(self, project_root, run_root):
        self.root, self.run_root, self.stream = Path(project_root), Path(run_root), None

    def acquire(self):
        import fcntl
        directory = project_path(self.root, self.run_root/'locks')
        directory.mkdir(parents=True, exist_ok=True)
        path = project_path(self.root, directory/CAMERA_LOCK)
        descriptor = os.open(path, os.O_RDWR | os.O_CREAT | os.O_CLOEXEC | os.O_NOFOLLOW, 0o600)
        stream = os.fdopen(descriptor, 'a+')
        try:
            if not stat.S_ISREG(os.fstat(stream.fileno()).st_mode):
                raise CameraError('camera domain lock is not a regular file')
            fcntl.flock(stream.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BaseException:
            stream.close()
            raise
        self.stream = stream

    def pass_fds(self):
        return (self.stream.fileno(),) if self.stream is not None else ()

    def close(self):
        if self.stream is not None:
            self.stream.close(); self.stream = None


class StatusWriter:
    """One replaceable metadata snapshot; no camera thread ever waits for disk."""
    def __init__(self, root, output, *, write=None):
        self.root, self.output = root, output
        self.write = write or self._write
        self.condition = threading.Condition()
        self.latest = None
        self.closing = self.synced = False
        self.worker_exited = False
        self.error = None
        self.written = self.max_write_ns = 0
        self.thread = threading.Thread(target=self._run, name='camera-preview-status', daemon=True)
        self.thread.start()

    def _write(self, value, final):
        destination = project_path(self.root, self.output/'status.json')
        temporary = project_path(self.root, self.output/'status.json.partial')
        with temporary.open('wb') as stream:
            stream.write((json.dumps(value, ensure_ascii=False, allow_nan=False)+'\n').encode())
            stream.flush()
            if final:
                os.fsync(stream.fileno())
        os.replace(temporary, destination)
        if final and os.name == 'posix':
            descriptor = os.open(self.output, os.O_RDONLY | os.O_DIRECTORY)
            try: os.fsync(descriptor)
            finally: os.close(descriptor)

    def submit(self, value):
        snapshot = copy.deepcopy(value)
        with self.condition:
            if not self.closing:
                self.latest = snapshot
                self.condition.notify_all()

    def _run(self):
        try:
            while True:
                with self.condition:
                    while self.latest is None and not self.closing:
                        self.condition.wait()
                    value, self.latest, final = self.latest, None, self.closing
                if value is not None:
                    started = time.monotonic_ns()
                    self.write(value, final)
                    with self.condition:
                        self.written += 1
                        self.max_write_ns = max(self.max_write_ns, time.monotonic_ns()-started)
                        self.synced = final
                if final:
                    return
        except Exception as error:
            with self.condition:
                self.error = self.error or 'CAMERA_STATUS_WRITE_FAILED: '+str(error)
        finally:
            with self.condition:
                self.worker_exited = True
                self.condition.notify_all()

    def snapshot(self):
        with self.condition:
            return {'final_fsync_complete': self.synced, 'error': self.error,
                    'worker_exited': self.worker_exited, 'pending_status': self.latest is not None,
                    'snapshots_written': self.written, 'max_write_ns': self.max_write_ns}

    def close(self, final_status, timeout_s=STATUS_CLOSE_S):
        timeout_s = persistence_wait(timeout_s)
        with self.condition:
            if not self.closing:
                self.latest = copy.deepcopy(final_status)
                self.closing = True
            self.condition.notify_all()
        self.thread.join(timeout_s)
        with self.condition:
            if self.thread.is_alive():
                self.error = self.error or 'CAMERA_STATUS_CLOSE_TIMEOUT'
            return self.snapshot()


def arm_owner(parent_pid):
    """Reap orphaned capture children and stop when our direct owner disappears."""
    if not hasattr(os, 'pidfd_open') or not hasattr(signal, 'pidfd_send_signal'):
        raise CameraError('camera companion requires Linux pidfd ownership')
    libc = ctypes.CDLL(None, use_errno=True)
    if libc.prctl(1, signal.SIGTERM, 0, 0, 0) != 0 or libc.prctl(36, 1, 0, 0, 0) != 0:
        raise CameraError('camera companion could not arm parent-death/subreaper handling')
    if os.getppid() != parent_pid:
        raise CameraError('camera companion parent changed before startup')


def reap_owned(children):
    """Preserve direct node exit codes if waitpid wins a race with Popen.poll."""
    by_pid = {child.pid: child for child in children}
    for child in children:
        child.poll()
    while True:
        try:
            pid, status = os.waitpid(-1, os.WNOHANG)
        except ChildProcessError:
            return
        if pid == 0:
            return
        if pid in by_pid:
            by_pid[pid].returncode = os.waitstatus_to_exitcode(status)


def close_children(owner, children, *, descendants=component.descendants,
                   send=component.signal_descendant, clock=time.monotonic, wait=time.sleep,
                   reap_orphans=None):
    """Shared deadlines for all four slots, never four sequential stop budgets."""
    direct = {child.pid for child in children}
    observed = []
    forced = False
    errors = []
    def reap():
        for child in children:
            child.poll()  # Preserve Popen return codes before reaping adopted orphans.
        if reap_orphans:
            reap_orphans()
    started = clock()
    for signum, seconds in STOP_PHASES:
        deadline = clock()+seconds
        sent = set()
        while True:
            reap()
            remaining = descendants(owner)
            if not remaining:
                return {'status': 'FAIL' if forced or errors else 'PASS', 'remaining': [],
                        'forced': forced, 'errors': errors, 'signals': observed, 'elapsed_s': clock()-started}
            for record in remaining:
                key = (record.pid, record.start_ticks)
                if record.state in ('Z', 'X') or key in sent or (signum == signal.SIGINT and record.pid not in direct):
                    continue
                try:
                    if send(record, owner, signum):
                        sent.add(key)
                        forced = forced or signum != signal.SIGINT
                        observed.append({'pid': record.pid, 'start_ticks': record.start_ticks, 'signal': int(signum)})
                except OSError as error:
                    errors.append(str(error))
            if clock() >= deadline:
                break
            wait(min(.05, max(0, deadline-clock())))
    reap()
    remaining = descendants(owner)
    return {'status': 'FAIL' if remaining or forced or errors else 'PASS',
            'remaining': [{'pid': row.pid, 'start_ticks': row.start_ticks, 'state': row.state} for row in remaining],
            'forced': forced, 'errors': errors, 'signals': observed, 'elapsed_s': clock()-started}


class CameraCompanion:
    def __init__(self, config_path, output, project_root, run_root, session_id, duration, *,
                 owner=None, launch=subprocess.Popen, lease=None, writer=None, clock=time.monotonic,
                 close_timeout_s=STATUS_CLOSE_S):
        self.close_timeout_s = persistence_wait(close_timeout_s)
        self.root = Path(project_root).absolute()
        self.config_path = project_path(self.root, config_path)
        self.output = project_path(self.root, output)
        self.run_root = project_path(self.root, run_root)
        self.session_id, self.duration = session_id, duration
        if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_.-]{0,119}', session_id):
            raise CameraError('explicit camera session identity required')
        number(duration, 'duration', 0, 43200)
        if 0 < duration < .5:
            raise CameraError('duration must be 0 (until stopped) or in [0.5, 43200] seconds')
        if self.config_path.parent != self.output.parent or self.config_path.name != 'cameras_config.json' or self.output.name != 'cameras':
            raise CameraError('camera config/output must belong to the same owned session')
        self.output.mkdir(exist_ok=False)
        self.owner = owner or component.process(os.getpid())
        self.launch = launch
        self.clock = clock
        self.deadline = None
        self.lease = lease or DomainLease(self.root, self.run_root)
        self.writer = writer or StatusWriter(self.root, self.output)
        self.lock = threading.Lock()
        self.children, self.readers = {}, []
        self.config = self.config_hash = None
        self.commands = []
        self.problem = None
        self.closed = False
        self.stopping = False
        self.cleanup = None
        self.rows = {role: {'role': role, 'state': 'NOT_STARTED', 'pid': None, 'exit_code': None,
                            'capture_ready': None, 'capture_stop': None, 'log_tail': deque(maxlen=12),
                            'log_lines': 0, 'reader_done': False, 'reader_error': None, 'error': None,
                            'attempt': 1, 'restart_attempts': 0, 'restart_count': 0, 'failure_count': 0,
                            'restart_state': 'NOT_NEEDED', 'restart_reason': None,
                            'next_restart_monotonic_s': None, 'attempt_history': [],
                            'attempt_recorded': False} for role in ROLES}

    def start(self):
        self.deadline = self.clock()+self.duration if self.duration else None
        try:
            self.config, self.config_hash = load_config(self.config_path)
            self.commands = child_commands(self.config_path, self.config, self.session_id, self.run_root, self.duration)
            self.lease.acquire()
        except Exception as error:
            self.problem = 'CAMERA_PREVIEW_UNAVAILABLE: '+str(error)
            for row in self.rows.values():
                row.update(state='UNAVAILABLE', error=self.problem)
            self.persist()
            return
        for command in self.commands:
            self._launch_slot(command)
        self.persist()

    def _launch_slot(self, command, *, restart=False):
        if self.stopping or self.closed:
            return
        role = command['role']
        argv = list(command['argv'])
        if restart and self.deadline is not None:
            remaining = self.deadline-self.clock()
            if remaining < self.config['timeouts']['open_s']+self.config['timeouts']['read_s']:
                with self.lock:
                    self.rows[role].update(restart_state='SUPPRESSED_SESSION_END', next_restart_monotonic_s=None)
                return
            argv[argv.index('--duration')+1] = str(remaining)
        environment = dict(os.environ, PYTHONNOUSERSITE='1', OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1')
        try:
            if restart:
                # Do not restart a slot from a changed configuration snapshot.
                _, current_hash = load_config(self.config_path)
                if current_hash != self.config_hash:
                    raise CameraError('camera configuration changed after session startup')
                if self.stopping:
                    return
                with self.lock:
                    row = self.rows[role]
                    row.update(attempt=row['attempt']+1, restart_attempts=row['restart_attempts']+1,
                               state='RESTARTING', pid=None, exit_code=None, capture_ready=None, capture_stop=None,
                               log_tail=deque(maxlen=12), log_lines=0, reader_done=False, reader_error=None, error=None,
                               attempt_recorded=False, next_restart_monotonic_s=None,
                               restart_state='RESTARTING_WITH_IDENTITY_AND_LEASE_CHECK')
                del self.children[role]  # Prior node and capture are positively reaped.
            child = self.launch(argv, cwd=self.root, env=environment, stdin=subprocess.DEVNULL,
                                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, start_new_session=True,
                                pass_fds=self.lease.pass_fds())
            self.children[role] = child
            with self.lock:
                row = self.rows[role]
                row.update(state='PROCESS_RUNNING', pid=child.pid, active_argv=argv)
                if restart:
                    row['restart_count'] += 1
            reader = threading.Thread(target=self._read_output, args=(role, child.stdout),
                                      name='camera-output-'+role, daemon=True)
            self.readers.append(reader); reader.start()
        except Exception as error:
            with self.lock:
                self.rows[role].update(state='UNAVAILABLE', error='CAMERA_START_FAILED: '+str(error),
                                       restart_state='RESTART_FAILED_NO_FURTHER_ATTEMPT', next_restart_monotonic_s=None)
                if restart:
                    self.rows[role]['failure_count'] += 1

    def request_stop(self):
        self.stopping = True

    def _read_output(self, role, stream):
        try:
            while True:
                raw = stream.readline(16385)
                if not raw:
                    break
                line = raw.decode('utf-8', errors='replace').rstrip()
                event = None
                if len(raw) <= 16384 and '{' in line:
                    try: event = json.loads(line[line.index('{'):])
                    except (ValueError, TypeError): pass
                with self.lock:
                    row = self.rows[role]
                    row['log_lines'] += 1
                    row['log_tail'].append(line[-2048:])
                    if isinstance(event, dict) and event.get('logical_slot') == role:
                        if event.get('event') == 'CAMERA_READY':
                            row['capture_ready'] = event
                            if row['restart_count']:
                                row['restart_state'] = 'REOPENED_PREVIEW_RATE_NOT_VERIFIED'
                        elif event.get('event') == 'CAMERA_STOPPED':
                            row['capture_stop'] = event
        except Exception as error:
            with self.lock:
                self.rows[role]['error'] = 'CAMERA_LOG_READER_FAILED: '+str(error)
                self.rows[role]['reader_error'] = str(error)
        finally:
            stream.close()
            with self.lock:
                self.rows[role]['reader_done'] = True

    def tick(self):
        if self.stopping or self.closed:
            return
        now = self.clock()
        for role, child in list(self.children.items()):
            code = child.poll()
            if code is not None:
                with self.lock:
                    row = self.rows[role]
                    if row['exit_code'] is None:
                        row.update(exit_code=code, state='EXITED', error='Camera exited during preview',
                                   failure_count=row['failure_count']+1)
                    if row['reader_done'] and not row['attempt_recorded']:
                        row['attempt_history'].append({key: copy.deepcopy(row[key]) for key in
                            ('attempt', 'pid', 'exit_code', 'capture_ready', 'capture_stop', 'log_lines', 'error')})
                        row['attempt_history'][-1].update(observed_exit_monotonic_s=now, log_tail=list(row['log_tail']))
                        row['attempt_recorded'] = True
                        eligible, reason = restart_eligibility(row)
                        row['restart_reason'] = reason
                        if not eligible:
                            row['restart_state'] = 'NOT_ELIGIBLE'
                        elif row['restart_attempts'] >= MAX_RESTARTS_PER_SLOT:
                            row['restart_state'] = 'RESTART_LIMIT_REACHED'
                        elif self.deadline is not None and now+RESTART_DELAY_S+self.config['timeouts']['open_s']+self.config['timeouts']['read_s'] > self.deadline:
                            row['restart_state'] = 'SUPPRESSED_SESSION_END'
                        else:
                            row.update(restart_state='WAITING_RESTART', next_restart_monotonic_s=now+RESTART_DELAY_S)
                    due = row['next_restart_monotonic_s']
                if due is not None and now >= due:
                    self._launch_slot(next(command for command in self.commands if command['role'] == role), restart=True)
        self.persist()

    def status(self):
        with self.lock:
            rows = {role: {**copy.deepcopy(row), 'log_tail': list(row['log_tail'])} for role, row in self.rows.items()}
        storage_error = getattr(self.writer, 'error', None)
        degraded = bool(self.problem or storage_error) or any(row['error'] or row['failure_count'] for row in rows.values())
        state = ('STOPPED_WITH_CAMERA_ERRORS' if degraded else 'STOPPED') if self.closed else ('DEGRADED' if degraded else 'RUNNING')
        if self.cleanup and self.cleanup['status'] != 'PASS':
            state = 'FAILED'
        elif self.closed and self.cleanup and self.cleanup.get('storage_status') == 'FAIL':
            state = 'FAILED_STORAGE'
        return {'schema_version': 1, 'kind': 'MAPPING_CAMERA_PREVIEW', 'session_id': self.session_id,
                'state': state,
                'source_mode': 'real', 'optional_for_mapping': True, 'problem': self.problem,
                'config_file': str(self.config_path), 'config_sha256': self.config_hash,
                'profile': self.config['default_profile'] if self.config else None, 'commands': self.commands,
                'slots': rows, 'cleanup': self.cleanup, 'images_saved': False, 'images_recorded_to_bag': False,
                'status_writer_error': storage_error,
                'persistence_close_timeout_s': self.close_timeout_s,
                'restart_policy': {'max_restarts_per_slot': MAX_RESTARTS_PER_SLOT, 'delay_s': RESTART_DELAY_S,
                                   'only_after_normal_read_deadline_release': True,
                                   'fresh_identity_and_exclusive_lease_required': True,
                                   'session_deadline_monotonic_s': self.deadline},
                'publishes_tf': False, 'calibration_used_by_mapping': False, 'time_source': 'arrival_only',
                'live_image_rate_verified_by_companion': False,
                'availability_note': 'PROCESS_RUNNING is process presence, not an image-rate assertion. RViz marks stale images independently.',
                'updated_monotonic_ns': time.monotonic_ns()}

    def persist(self):
        self.writer.submit(self.status())

    def close(self, *, cleanup=close_children, reap_orphans=None):
        if self.closed:
            return self.cleanup
        self.request_stop()
        self.cleanup = cleanup(self.owner, list(self.children.values()), reap_orphans=reap_orphans)
        for role, child in self.children.items():
            code = child.poll()
            with self.lock:
                row = self.rows[role]
                row['exit_code'] = code
                if row['error'] is None:
                    row['state'] = 'STOPPED' if code == 0 else 'FAILED'
                    if code != 0:
                        row['error'] = 'Camera failed during shutdown: '+str(code)
        # Pipes are drained by their owner threads. Do not close a pipe while a
        # reader is blocked on an unreaped child still holding its write end.
        for reader in self.readers:
            reader.join(.02)
        # A capture worker can require escalation inside its node's reviewed
        # 5+1+1-second close before this companion has to escalate the node.
        # Preserve that cleanup failure independently of preview availability.
        with self.lock:
            camera_cleanup_errors = {
                role: row['capture_stop']['cleanup'] for role, row in self.rows.items()
                if row['capture_stop'] and isinstance(row['capture_stop'].get('cleanup'), dict)
                and (row['capture_stop']['cleanup'].get('alive_after_cleanup')
                     or row['capture_stop']['cleanup'].get('termination') in ('terminate', 'kill'))}
        if camera_cleanup_errors:
            self.cleanup['status'] = 'FAIL'
            self.cleanup['camera_cleanup_errors'] = camera_cleanup_errors
        self.closed = True
        final = self.status()
        final['state'] = 'FAILED' if self.cleanup['status'] != 'PASS' else final['state']
        storage = self.writer.close(final, timeout_s=self.close_timeout_s)
        self.cleanup['status_storage'] = storage
        # Device cleanup is distinct from optional preview metadata persistence.
        # Returning a nonzero component exit for this optional storage failure
        # would reject an otherwise valid core map in the shared supervisor.
        self.cleanup['storage_status'] = 'FAIL' if storage.get('error') or not storage.get('final_fsync_complete') else 'PASS'
        if not self.cleanup['remaining']:
            self.lease.close()
        return self.cleanup


def _parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--output-root', type=Path, required=True)
    parser.add_argument('--duration', type=float, required=True, help='0 means until stopped; otherwise 0.5..43200 seconds')
    parser.add_argument('--close-timeout-s', type=persistence_wait, default=STATUS_CLOSE_S,
                        help='Stop-time metadata persistence wait in (0,90] seconds; device stop limits unchanged')
    return parser.parse_args(argv)


def main(argv=None):
    args = _parse_args(argv)
    from .cli import ROOT, RUN, target
    target()
    config = project_path(ROOT, args.config)
    runtime = read_json(ROOT, config.parent/'runtime_config.json', limit=1_000_000)
    stopped = False
    companion = None
    def stop(*_):
        nonlocal stopped
        stopped = True
        if companion is not None:
            companion.request_stop()
    for signum in (signal.SIGINT, signal.SIGTERM):
        signal.signal(signum, stop)
    arm_owner(os.getppid())
    companion = CameraCompanion(config, args.output_root, ROOT, RUN, runtime['session_id'], args.duration,
                                close_timeout_s=args.close_timeout_s)
    started = time.monotonic()
    try:
        if not stopped:
            companion.start()
        while not stopped and (args.duration == 0 or time.monotonic()-started < args.duration):
            companion.tick()
            time.sleep(.2)
    finally:
        result = companion.close(reap_orphans=lambda: reap_owned(list(companion.children.values())))
        print(json.dumps({'event': 'MAPPING_CAMERAS_STOPPED', 'session_id': runtime['session_id'],
                          'cleanup': result, 'preview_state': companion.status()['state'], 'images_saved': False},
                         ensure_ascii=False, allow_nan=False), flush=True)
    return 0 if result['status'] == 'PASS' else 1


if __name__ == '__main__':
    raise SystemExit(main())
