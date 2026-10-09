"""Actual Linux process/lock acceptance. No ROS, GUI, devices or heavy builds.

Every test owns a new UUID role under the production runtime root. Plans/logs
remain as evidence. Failure cleanup signals only helper-recorded PID identities.
"""

import fcntl
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import uuid

import pytest

from wc_runtime.supervisor import stop_registered, ticks
from wc_runtime.project_paths import shared_lock_root, project_root


PROJECT = project_root()
RUNTIME = PROJECT/'.phase1_runtime'
HELPER = Path(__file__).with_name('process_helper.py').resolve()


def identity(pid):
    try:
        text = Path(f'/proc/{pid}/stat').read_text()
        fields = text[text.rindex(')')+2:].split()
        return fields[19], fields[0]
    except FileNotFoundError:
        return None


def alive(record):
    observed = identity(record['pid'])
    return observed is not None and observed[0] == str(record['start_ticks']) and observed[1] not in {'Z', 'X'}


def wait_for(predicate, timeout=8, message='condition timed out'):
    end = time.monotonic()+timeout
    while time.monotonic() < end:
        if predicate():
            return
        time.sleep(.03)
    raise AssertionError(message)


def signal_verified(record, signum):
    """Tests use pidfds so a PID reuse between checking and signaling is harmless."""
    if not alive(record):
        return
    try:
        descriptor = os.pidfd_open(record['pid'])
    except ProcessLookupError:
        return
    try:
        observed = identity(record['pid'])
        if observed is not None and observed[0] == str(record['start_ticks']):
            signal.pidfd_send_signal(descriptor, signum)
    finally:
        os.close(descriptor)


def kill_verified(record):
    signal_verified(record, signal.SIGKILL)


class Harness:
    def __init__(self):
        assert sys.platform == 'linux' and PROJECT.is_dir(), 'run these operations tests in the selected Linux project'
        self.token = 'ops-test-'+uuid.uuid4().hex
        self.root = RUNTIME/'sessions'/self.token/'operations'
        self.root.mkdir(parents=True, exist_ok=False)
        self.pid_log = self.root/'helper-pids.jsonl'
        self.process = None
        self.output_stream = None
        self.others = []
        self.supervisor_record = None

    def command(self, mode='stay', *extra):
        return [sys.executable, str(HELPER), '--mode', mode, '--pid-log', str(self.pid_log), '--token', self.token, *extra]

    def records(self):
        if not self.pid_log.exists():
            return []
        try:
            records = [json.loads(line) for line in self.pid_log.read_text().splitlines() if line]
        except json.JSONDecodeError:
            return []  # Writer is still appending; caller waits for complete evidence.
        assert all(record['token'] == self.token for record in records)
        return records

    def owned_records(self):
        records = self.records()
        if (self.root/'manifest.json').exists():
            records += self.manifest().get('children', [])
        return records

    def launch(self, commands=None, duration=0, locks=None, allow_component_exit=False):
        plan = {'session_id': self.token, 'role': 'operations', 'commands': commands or [self.command()],
                'environment': {'PYTHONNOUSERSITE': '1'}, 'duration_s': duration,
                'locks': locks or [], 'allow_component_exit': allow_component_exit,
                'data_source': 'SYNTHETIC_PROCESS_TEST_NO_ROS_NO_HARDWARE'}
        path = self.root/'plan.json'
        path.write_text(json.dumps(plan), encoding='utf-8')
        self.output_stream = (self.root/'supervisor.log').open('w', encoding='utf-8')
        self.process = subprocess.Popen([sys.executable, '-m', 'wc_runtime.supervisor', '--plan', str(path)],
                                        stdin=subprocess.DEVNULL, stdout=self.output_stream, stderr=subprocess.STDOUT,
                                        start_new_session=True, cwd=PROJECT, env=os.environ.copy())
        self.supervisor_record = {'pid': self.process.pid, 'start_ticks': ticks(self.process.pid)}
        wait_for(lambda: (self.root/'manifest.json').exists() or self.process.poll() is not None,
                 message='supervisor did not create a manifest')
        return self

    def manifest(self):
        return json.loads((self.root/'manifest.json').read_text())

    def started(self, count=1):
        wait_for(lambda: len(self.records()) >= count, message='helper process tree did not start')
        assert self.manifest()['state'] == 'RUNNING'

    def finished(self, timeout=23):
        self.process.wait(timeout=timeout)
        return self.manifest()

    def unrelated(self, *extra):
        command = [sys.executable, '-c', 'import time; time.sleep(90)', *map(str, extra)]
        process = subprocess.Popen(command, start_new_session=True)
        record = {'pid': process.pid, 'start_ticks': ticks(process.pid)}
        self.others.append((process, record))
        return process, record

    def cleanup(self):
        # Cleanup is deliberately stricter than the product under test and is
        # never counted as a product cleanup PASS.
        for record in reversed(self.owned_records()):
            kill_verified(record)
        if self.supervisor_record:
            kill_verified(self.supervisor_record)
        if self.process is not None:
            self.process.wait(timeout=5)
        for process, record in self.others:
            kill_verified(record)
            process.wait(timeout=5)
        if self.output_stream is not None:
            self.output_stream.close()


@pytest.fixture
def harness():
    value = Harness()
    try:
        yield value
    finally:
        value.cleanup()


def assert_no_owned_live_processes(harness, timeout=3):
    wait_for(lambda: not any(alive(record) for record in harness.owned_records()), timeout,
             message='production supervisor left a recorded live descendant; test cleanup will remove it')


def test_stop_only_affects_the_registered_session(harness):
    _, unrelated = harness.unrelated()
    harness.launch()
    harness.started()
    assert stop_registered(harness.root/'manifest.json') is True
    result = harness.finished()
    assert result['state'] == 'STOPPED' and result['exit_code'] == 0
    assert alive(unrelated), 'registered stop signalled an unrelated process'
    assert_no_owned_live_processes(harness)


def test_duration_stops_normally_and_cleans_children(harness):
    harness.launch(duration=.7)
    harness.started()
    result = harness.finished()
    assert result['state'] == 'STOPPED' and result['exit_code'] == 0
    assert result['stop_reason'] == 'duration_reached'
    assert_no_owned_live_processes(harness)


def test_graceful_stop_does_not_deliver_sigint_twice_through_a_launcher(harness):
    signal_log = harness.root/'forwarded-signal-events.jsonl'
    harness.launch([harness.command('forwarder', '--signal-log', str(signal_log))])
    harness.started(2)
    stop_registered(harness.root/'manifest.json')
    result = harness.finished()
    assert result['state'] == 'STOPPED' and result['exit_code'] == 0
    events = [json.loads(line) for line in signal_log.read_text().splitlines()]
    assert len(events) == 1 and events[0]['token'] == harness.token and events[0]['sigint_count'] == 1, \
        'launcher and descendant received duplicate SIGINT; graceful cleanup could be interrupted'
    assert_no_owned_live_processes(harness)


@pytest.mark.parametrize('new_session', [False, True])
def test_component_exit_does_not_leave_its_grandchild(harness, new_session):
    extra = ['--grandchild-new-session'] if new_session else []
    harness.launch([harness.command('spawn-exit', '--exit-code', '17', *extra)])
    wait_for(lambda: any(record['role'] == 'grandchild' for record in harness.records()))
    result = harness.finished()
    assert result['state'] == 'FAILED' and result['exit_code'] != 0
    assert_no_owned_live_processes(harness)


def test_component_leader_exit_during_stop_does_not_skip_stubborn_grandchild(harness):
    harness.launch([harness.command('spawn-stay', '--grandchild-new-session')])
    harness.started(2)
    stop_registered(harness.root/'manifest.json')
    harness.finished()
    assert_no_owned_live_processes(harness)


def test_supervisor_sigkill_does_not_orphan_its_process_tree(harness):
    harness.launch([harness.command('spawn-stay', '--grandchild-new-session')])
    harness.started(2)
    kill_verified(harness.supervisor_record)
    harness.process.wait(timeout=5)
    # A separate cleanup guardian/lifeline must survive the parent death; the
    # test itself does not call stop or clean descendants before this assertion.
    assert_no_owned_live_processes(harness, timeout=20)


def test_resource_lock_survives_supervisor_death_until_descendant_cleanup(harness):
    lock_name = harness.token+'-held-through-cleanup.lock'
    lock_path = shared_lock_root()/lock_name
    harness.launch([harness.command('spawn-stay', '--grandchild-new-session')], locks=[lock_name])
    harness.started(2)
    kill_verified(harness.supervisor_record)
    harness.process.wait(timeout=5)
    assert any(alive(record) and record['role'] == 'grandchild' for record in harness.records())
    with lock_path.open('a+') as contender:
        with pytest.raises(BlockingIOError):
            fcntl.flock(contender, fcntl.LOCK_EX | fcntl.LOCK_NB)
    assert_no_owned_live_processes(harness, timeout=20)
    with lock_path.open('a+') as next_owner:
        fcntl.flock(next_owner, fcntl.LOCK_EX | fcntl.LOCK_NB)


def test_component_cleanup_rejects_an_unrelated_live_pid(harness):
    from wc_runtime.component import process, signal_descendant
    _, unrelated = harness.unrelated()
    harness.launch()
    harness.started()
    owner = process(harness.manifest()['children'][0]['pid'])
    assert signal_descendant(process(unrelated['pid']), owner, signal.SIGTERM) is False
    assert alive(unrelated), 'component cleanup signalled a process outside its own descendant tree'
    stop_registered(harness.root/'manifest.json')
    harness.finished()


def test_pid_start_ticks_mismatch_refuses_to_signal(harness):
    harness.launch()
    harness.started()
    original = harness.manifest()
    changed = {**original, 'supervisor_start_ticks': str(int(original['supervisor_start_ticks'])+1)}
    path = harness.root/'manifest.json'
    path.write_text(json.dumps(changed), encoding='utf-8')
    with pytest.raises(RuntimeError, match='PID|identity|reuse'):
        stop_registered(path)
    assert alive(harness.supervisor_record)
    path.write_text(json.dumps(original), encoding='utf-8')
    stop_registered(path)
    harness.finished()


def test_command_substrings_do_not_make_an_unrelated_process_a_supervisor(harness):
    _, unrelated = harness.unrelated('wc_runtime.supervisor', harness.root/'plan.json')
    data = {'state': 'RUNNING', 'supervisor_pid': unrelated['pid'], 'supervisor_start_ticks': unrelated['start_ticks']}
    path = harness.root/'manifest.json'
    path.write_text(json.dumps(data), encoding='utf-8')
    with pytest.raises(RuntimeError, match='command|identity|owner|supervisor'):
        stop_registered(path)
    assert alive(unrelated), 'substring spoof caused stop to signal an unrelated process'


def test_lock_conflict_fails_before_component_start(harness):
    lock_name = harness.token+'.lock'
    lock_path = shared_lock_root()/lock_name
    lock_path.parent.mkdir(parents=True, exist_ok=True)
    with lock_path.open('a+') as owner:
        fcntl.flock(owner.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        harness.launch(locks=[lock_name])
        result = harness.finished()
        assert result['state'] == 'FAILED' and result['exit_code'] != 0
        assert harness.records() == [], 'contended lock allowed a component to start'


def test_nonzero_component_exit_cannot_be_overwritten_by_later_zero_exit(harness):
    release = harness.root/'release-components'
    commands = [harness.command('exit-on-release', '--release', str(release), '--exit-code', str(code)) for code in (17, 0)]
    harness.launch(commands, allow_component_exit=True)
    harness.started(2)
    time.sleep(.4)
    # Make both exits visible in the same poll cycle, not a scheduler-dependent
    # coincidence. SIGSTOP affects only this test's verified supervisor PID.
    signal_verified(harness.supervisor_record, signal.SIGSTOP)
    release.write_text('go', encoding='utf-8')
    wait_for(lambda: not any(alive(record) for record in harness.records()), timeout=3)
    time.sleep(.2)  # Let the ownership wrappers reap both primary exit codes.
    signal_verified(harness.supervisor_record, signal.SIGCONT)
    result = harness.finished()
    assert result['state'] == 'FAILED' and result['exit_code'] != 0, 'successfully exiting peer masked the failed component'


@pytest.mark.parametrize('duration', [-1, float('nan'), float('inf')])
def test_invalid_supervisor_duration_is_rejected_without_starting_components(harness, duration):
    harness.launch(duration=duration)
    assert harness.process.wait(timeout=5) != 0
    if (harness.root/'manifest.json').exists():
        result = harness.manifest()
        assert result['state'] == 'FAILED' and result['exit_code'] != 0
    assert not harness.records(), 'invalid duration reached component launch'


@pytest.mark.parametrize('stop_before_first_component', [True, False])
def test_stop_during_startup_skips_remaining_components_and_cleans_started_children(
        tmp_path, monkeypatch, stop_before_first_component):
    from wc_runtime import supervisor
    project = tmp_path/'project'
    runtime = project/'.phase1_runtime/sessions/startup/operations'
    runtime.mkdir(parents=True)
    plan = {'session_id': 'startup', 'role': 'operations', 'duration_s': 0,
            'commands': [['synthetic-first'], ['synthetic-second'], ['synthetic-third']],
            'locks': ['synthetic-startup.lock'], 'environment': {}}
    plan_path = runtime/'plan.json'
    plan_path.write_text(json.dumps(plan))
    handlers, children, observed_states = {}, [], []
    resource = (runtime/'synthetic-startup.lock').open('a+')
    write_manifest = supervisor.write_json

    def acquire(_):
        if stop_before_first_component:
            handlers[signal.SIGTERM](signal.SIGTERM, None)
        return resource

    class Child:
        pid = 400001

        def __init__(self):
            self.returncode = None
            self.signals = []
            self.reaped = False

        def poll(self):
            return self.returncode

        def send_signal(self, signum):
            self.signals.append(signum)
            self.returncode = 0

        def wait(self, timeout):
            assert self.returncode == 0, 'started child was not stopped before reap'
            self.reaped = True
            return self.returncode

    def spawn(argv, **kwargs):
        assert argv[-1] == 'synthetic-first', 'component started after stop request'
        child = Child()
        children.append(child)
        handlers[signal.SIGTERM](signal.SIGTERM, None)
        return child

    def record(path, value):
        observed_states.append(value['state'])
        return write_manifest(path, value)

    monkeypatch.setattr(supervisor.signal, 'signal', lambda signum, handler: handlers.__setitem__(signum, handler))
    monkeypatch.setattr(supervisor, 'acquire_resource_lock', acquire)
    monkeypatch.setattr(supervisor, 'ticks', lambda _: 'synthetic-start-ticks')
    monkeypatch.setattr(supervisor.subprocess, 'Popen', spawn)
    monkeypatch.setattr(supervisor, 'write_json', record)
    try:
        assert supervisor.main(['--plan', str(plan_path)], project_root=project) == 0
        assert resource.closed, 'startup cancellation did not release its resource lock'
    finally:
        if not resource.closed:
            resource.close()
    assert len(children) == (0 if stop_before_first_component else 1)
    assert all(child.signals == [signal.SIGINT] and child.reaped for child in children)
    assert observed_states == ['RUNNING', 'STOPPED']
    final = json.loads((runtime/'manifest.json').read_text())
    assert final['stop_reason'] == 'startup_stop_request' and final['exit_code'] == 0
    assert len(final['children']) == len(children)


def launch_mapping_owner(harness, *, startup_gate):
    """An actual parent starts the supervisor before any controller owner-watch."""
    plan = {'session_id': harness.token, 'role': 'mapping_app',
            'commands': [harness.command()] * (2 if startup_gate else 1),
            'environment': {'PYTHONNOUSERSITE': '1'}, 'duration_s': 0, 'locks': [],
            'data_source': 'SYNTHETIC_PROCESS_TEST_NO_ROS_NO_HARDWARE'}
    # Hold the first real Popen return so owner death between components is
    # deterministic, without inserting a delay into production startup.
    bootstrap = harness.root/'gated-supervisor.py'
    bootstrap.write_text('''from pathlib import Path
import subprocess,sys,time
from wc_runtime import supervisor
root = Path(sys.argv[1]).parent
original = subprocess.Popen
count = 0
def spawn(*args, **kwargs):
    global count
    child = original(*args, **kwargs)
    count += 1
    if count == 1:
        (root/'first-component-created').write_text('ready')
        while not (root/'continue-startup').exists():
            time.sleep(.01)
    return child
supervisor.subprocess.Popen = spawn
raise SystemExit(supervisor.main(['--plan', sys.argv[1]]))
''', encoding='utf-8')
    plan_path = harness.root/'plan.json'
    command = ([sys.executable, str(bootstrap), str(plan_path)] if startup_gate else
               [sys.executable, '-m', 'wc_runtime.supervisor', '--plan', str(plan_path)])
    owner_code = '''import json,os,subprocess,sys,time
from pathlib import Path
from wc_runtime.supervisor import ticks
root = Path(sys.argv[1])
plan = json.loads(sys.argv[2])
plan.update(owner_pid=os.getpid(), owner_start_ticks=ticks(os.getpid()))
(root/'plan.json').write_text(json.dumps(plan))
with (root/'supervisor.log').open('w') as log:
    child = subprocess.Popen(json.loads(sys.argv[3]), stdin=subprocess.DEVNULL,
        stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
(root/'supervisor-identity.json').write_text(json.dumps({'pid':child.pid,'start_ticks':ticks(child.pid)}))
while True: time.sleep(1)
'''
    with (harness.root/'owner.log').open('w') as log:
        owner = subprocess.Popen([sys.executable, '-c', owner_code, str(harness.root),
                                  json.dumps(plan), json.dumps(command)], cwd=PROJECT,
                                 stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    record = {'pid': owner.pid, 'start_ticks': ticks(owner.pid)}
    harness.others.append((owner, record))
    identity_path = harness.root/'supervisor-identity.json'
    wait_for(identity_path.exists, message='synthetic owner did not launch supervisor')
    harness.supervisor_record = json.loads(identity_path.read_text())
    if startup_gate:
        wait_for(lambda: (harness.root/'first-component-created').exists())
    else:
        wait_for(lambda: (harness.root/'manifest.json').exists())
    wait_for(lambda: len(harness.records()) >= 1)
    return owner, record


@pytest.mark.parametrize('phase', ['running_zombie', 'running_reaped', 'startup_reaped'])
def test_mapping_owner_death_stops_without_owner_watch(harness, phase):
    startup_gate = phase == 'startup_reaped'
    owner, record = launch_mapping_owner(harness, startup_gate=startup_gate)
    assert not (harness.root/'owner-watch.log').exists()
    kill_verified(record)
    if phase == 'running_zombie':
        wait_for(lambda: (identity(record['pid']) or ('', ''))[1] == 'Z')
    else:
        owner.wait(timeout=5)
    if startup_gate:
        (harness.root/'continue-startup').write_text('resume', encoding='utf-8')
    wait_for(lambda: (harness.root/'manifest.json').exists() and
             harness.manifest().get('state') == 'STOPPED', timeout=8,
             message='owner died before owner-watch existed; supervisor did not stop itself')
    result = harness.manifest()
    assert result['exit_code'] == 0 and not result.get('cleanup_errors')
    assert result['stop_reason'] == ('owner_process_dead:Z' if phase == 'running_zombie' else 'owner_process_missing')
    assert len(result['children']) == 1, 'owner death allowed a later component to start'
    assert len(harness.records()) == 1
    assert_no_owned_live_processes(harness)


@pytest.mark.parametrize('fields', [
    {'owner_pid': 123}, {'owner_start_ticks': '123'},
    {'owner_pid': True, 'owner_start_ticks': '123'},
    {'owner_pid': 0, 'owner_start_ticks': '123'},
    {'owner_pid': -1, 'owner_start_ticks': '123'},
    {'owner_pid': '123', 'owner_start_ticks': '123'},
    {'owner_pid': 123, 'owner_start_ticks': 123},
    {'owner_pid': 123, 'owner_start_ticks': ''},
    {'owner_pid': 123, 'owner_start_ticks': '../stat'},
])
def test_invalid_mapping_owner_is_rejected_before_locks_or_components(tmp_path, monkeypatch, fields):
    from wc_runtime import supervisor
    root = tmp_path/'.phase1_runtime/mapping-owner-invalid'
    root.mkdir(parents=True)
    plan = dict(session_id='invalid-owner', role='mapping_app', commands=[['must-not-start']],
                duration_s=0, locks=['must-not-lock'], environment={}, **fields)
    path = root/'plan.json'
    path.write_text(json.dumps(plan))
    def forbidden(*args, **kwargs):
        pytest.fail('Invalid owner reached a resource or process operation')
    monkeypatch.setattr(supervisor, 'acquire_resource_lock', forbidden)
    monkeypatch.setattr(supervisor.subprocess, 'Popen', forbidden)
    with pytest.raises(ValueError, match='mapping_app owner'):
        supervisor.main(['--plan', str(path)], project_root=tmp_path)
    assert not (root/'manifest.json').exists()


@pytest.mark.parametrize('observation, reason', [
    (None, 'owner_process_missing'),
    (('124', 'S'), 'owner_pid_reused'),
    (('123', 'Z'), 'owner_process_dead:Z'),
    (('123', 'X'), 'owner_process_dead:X'),
    (PermissionError(), 'owner_identity_unavailable:PermissionError'),
])
def test_mapping_owner_lost_before_lock_never_starts_components(tmp_path, monkeypatch, observation, reason):
    from types import SimpleNamespace
    from wc_runtime import supervisor
    root = tmp_path/'.phase1_runtime/mapping-owner-lost'
    root.mkdir(parents=True)
    plan = dict(session_id='lost-owner', role='mapping_app', owner_pid=123, owner_start_ticks='123',
                commands=[['must-not-start']], duration_s=0, locks=['must-not-lock'], environment={})
    path = root/'plan.json'
    path.write_text(json.dumps(plan))
    def inspect_owner(pid):
        assert pid == 123
        if isinstance(observation, Exception):
            raise observation
        return None if observation is None else SimpleNamespace(start_ticks=observation[0], state=observation[1])
    def forbidden(*args, **kwargs):
        pytest.fail('Lost owner reached a resource or process operation')
    monkeypatch.setattr(supervisor, 'process_identity', inspect_owner)
    monkeypatch.setattr(supervisor, 'acquire_resource_lock', forbidden)
    monkeypatch.setattr(supervisor.subprocess, 'Popen', forbidden)
    monkeypatch.setattr(supervisor.signal, 'signal', lambda *args: None)
    assert supervisor.main(['--plan', str(path)], project_root=tmp_path) == 0
    result = json.loads((root/'manifest.json').read_text())
    assert result['state'] == 'STOPPED' and result['exit_code'] == 0
    assert result['stop_reason'] == reason and result['children'] == []


@pytest.mark.parametrize('plan', [
    {'role': 'mapping_app'}, {'role': 'operations', 'owner_pid': True, 'owner_start_ticks': None},
    {'role': 'record', 'owner_pid': -1},
])
def test_owner_guard_does_not_change_unbound_or_other_roles(plan):
    from wc_runtime.supervisor import mapping_owner
    assert mapping_owner(plan) is None
