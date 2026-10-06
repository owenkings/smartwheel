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


PROJECT = Path('/home/nvidia/wheelchair')
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
        assert sys.platform == 'linux' and PROJECT.is_dir(), 'run these operations tests on the confirmed target Linux project'
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
    lock_path = RUNTIME/'locks'/lock_name
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
    lock_path = RUNTIME/'locks'/lock_name
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
