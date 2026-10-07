"""Bounded shutdown fixtures; Linux-only cases run synthetic writers, never ROS/devices."""

import io
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import uuid

import pytest

from wc_runtime import cli, component, supervisor


class Clock:
    def __init__(self):
        self.now = 0.0
        self.after_sleep = lambda: None

    def monotonic(self):
        return self.now

    def sleep(self, seconds):
        assert seconds >= 0
        self.now += seconds
        self.after_sleep()


@pytest.fixture
def clock(monkeypatch):
    value = Clock()
    monkeypatch.setattr(time, 'monotonic', value.monotonic)
    monkeypatch.setattr(time, 'sleep', value.sleep)
    # Windows fixtures never dispatch signals; POSIX values are fixture data.
    monkeypatch.setattr(signal, 'SIGKILL', getattr(signal, 'SIGKILL', 9), raising=False)
    return value


@pytest.mark.parametrize('role, grace, total', [('record', 30, 47), ('map', 5, 22), ('drivers', 30, 47)])
def test_role_shutdown_budget_covers_all_stages_below_one_minute(role, grace, total):
    policy = supervisor.shutdown_policy({'role': role})
    assert policy['sigint_grace_s'] == grace
    assert policy['component_wait_s'] >= grace + component.TERM_GRACE_S + component.KILL_GRACE_S + 5
    assert policy['cli_stop_wait_s'] == total < 60
    assert total > sum(policy[key] for key in ('component_wait_s', 'supervisor_term_s', 'supervisor_kill_s'))


@pytest.mark.parametrize('bad', [-1, float('nan'), float('inf'), float('-inf'), 121, True, '30'])
def test_invalid_grace_cannot_reach_supervisor_launch(tmp_path, monkeypatch, bad):
    root = tmp_path/'.phase1_runtime'/'sessions'/'invalid'/'record'
    root.mkdir(parents=True)
    plan = {'role': 'record', 'session_id': 'invalid', 'commands': [['fixture-only']],
            'duration_s': 0, 'sigint_grace_s': bad}
    path = root/'plan.json'
    path.write_text(json.dumps(plan), encoding='utf-8')
    monkeypatch.setattr(supervisor.subprocess, 'Popen', lambda *a, **kw: pytest.fail('invalid grace launched a process'))
    with pytest.raises(ValueError, match='SIGINT grace'):
        supervisor.main(['--plan', str(path)], project_root=tmp_path)
    with pytest.raises(ValueError, match='SIGINT grace'):
        component.validate_sigint_grace(bad)
    assert not (root/'manifest.json').exists()


@pytest.mark.parametrize('argument', ['-1', 'nan', 'inf', '121'])
def test_component_cli_rejects_invalid_grace_before_platform_or_process_setup(monkeypatch, argument):
    monkeypatch.setattr(component.subprocess, 'Popen', lambda *a, **kw: pytest.fail('invalid grace launched a process'))
    with pytest.raises(ValueError, match='SIGINT grace'):
        component.main(['--parent', '1', '--sigint-grace-s', argument, '--', 'fixture-only'])


@pytest.mark.parametrize('grace, expected_termination', [(30, False), (5, True)])
def test_writer_needing_six_seconds_gets_record_grace_but_default_is_unchanged(clock, monkeypatch, grace, expected_termination):
    owner = component.Process(1, 0, 'owner', 'S')
    record = component.Process(2, 1, 'writer', 'S')
    child = type('Writer', (), {'pid': 2, 'returncode': None})()
    events = []

    def reap(_):
        if child.returncode is None and clock.now >= 6:
            child.returncode = 0

    def send(candidate, parent, signum):
        assert candidate == record and parent == owner
        events.append((signum, clock.now))
        if signum != signal.SIGINT:
            child.returncode = -int(signum)
        return True

    monkeypatch.setattr(component, 'reap', reap)
    monkeypatch.setattr(component, 'descendants', lambda _: [record] if child.returncode is None else [])
    monkeypatch.setattr(component, 'signal_descendant', send)
    assert component.cleanup(owner, child, grace)
    assert events[0] == (signal.SIGINT, 0)
    assert any(signum == signal.SIGTERM for signum, _ in events) is expected_termination
    if not expected_termination:
        assert child.returncode == 0 and 6 <= clock.now < 7


def test_stuck_descendant_is_still_failed_after_all_bounded_component_stages(clock, monkeypatch):
    owner = component.Process(1, 0, 'owner', 'S')
    stuck = component.Process(2, 1, 'writer', 'D')
    events = []
    monkeypatch.setattr(component, 'reap', lambda _: None)
    monkeypatch.setattr(component, 'descendants', lambda _: [stuck])
    monkeypatch.setattr(component, 'signal_descendant', lambda p, o, sig: events.append((sig, clock.now)) or True)
    assert not component.cleanup(owner, type('Child', (), {'pid': 2})(), 30)
    assert [sig for sig, _ in events] == [signal.SIGINT, signal.SIGTERM, signal.SIGKILL]
    assert 35 <= clock.now < 35.2


@pytest.mark.parametrize('exit_code', [0, 17, -15])
def test_component_preserves_exit_status_observed_during_cleanup(monkeypatch, exit_code):
    handlers = {}
    child = type('Writer', (), {'returncode': None})()
    monkeypatch.setattr(signal, 'signal', lambda sig, handler: handlers.__setitem__(sig, handler))
    monkeypatch.setattr(os, 'pidfd_open', lambda *a: pytest.fail('fixture has no real PID'), raising=False)
    monkeypatch.setattr(signal, 'pidfd_send_signal', lambda *a: pytest.fail('fixture sends no signals'), raising=False)
    monkeypatch.setattr(os, 'getppid', lambda: 10)
    monkeypatch.setattr(os, 'getpid', lambda: 20)
    monkeypatch.setattr(os, 'getpgrp', lambda: 20, raising=False)
    monkeypatch.setattr(component.ctypes, 'CDLL', lambda *a, **kw: type('Libc', (), {'prctl': lambda *a: 0})())
    monkeypatch.setattr(component, 'process', lambda _: component.Process(20, 10, 'fixture', 'S'))
    monkeypatch.setattr(component.subprocess, 'Popen', lambda *a, **kw: child)
    monkeypatch.setattr(component, 'reap', lambda _: handlers[signal.SIGINT](signal.SIGINT, None))
    monkeypatch.setattr(component.time, 'sleep', lambda _: None)

    def finish(owner, observed, grace):
        assert observed is child and grace == 30
        observed.returncode = exit_code
        return True

    monkeypatch.setattr(component, 'cleanup', finish)
    assert component.main(['--parent', '10', '--sigint-grace-s', '30', '--', 'fixture-only']) == exit_code


class Child:
    def __init__(self, clock, pid, finish_at=None, exit_code=0):
        self.clock, self.pid, self.finish_at, self.exit_code = clock, pid, finish_at, exit_code
        self.returncode = None
        self.signals = []

    def poll(self):
        if self.finish_at is not None and self.clock.now >= self.finish_at:
            self.returncode = self.exit_code
        return self.returncode

    def send_signal(self, signum):
        self.signals.append((signum, self.clock.now))

    def wait(self, timeout):
        assert timeout >= 0
        if self.finish_at is not None and self.finish_at <= self.clock.now + timeout:
            self.clock.sleep(max(0, self.finish_at-self.clock.now))
            return self.poll()
        self.clock.sleep(timeout)
        raise subprocess.TimeoutExpired('fixture-only', timeout)


def test_supervisor_allows_twenty_second_flush_without_escalation(clock, monkeypatch):
    child = Child(clock, 100, finish_at=20)
    monkeypatch.setattr(os, 'killpg', lambda *a: pytest.fail('premature escalation interrupted writer'), raising=False)
    result, errors = supervisor.finish_children([(child, io.StringIO())], supervisor.shutdown_policy({'role': 'record'}))
    assert result == 0 and errors == []
    assert child.signals == [(signal.SIGINT, 0)] and clock.now == 20


def test_supervisor_multiple_stuck_wrappers_share_deadlines_and_fail(clock, monkeypatch):
    children = [(Child(clock, pid), io.StringIO()) for pid in (100, 101, 102)]
    group_signals = []
    monkeypatch.setattr(os, 'killpg', lambda pid, sig: group_signals.append((pid, sig, clock.now)), raising=False)
    result, errors = supervisor.finish_children(children, supervisor.shutdown_policy({'role': 'record'}))
    assert result != 0 and clock.now == 46
    assert all(child.signals == [(signal.SIGINT, 0)] and log.closed for child, log in children)
    assert {when for _, sig, when in group_signals if sig == signal.SIGTERM} == {40}
    assert {when for _, sig, when in group_signals if sig == signal.SIGKILL} == {43}
    assert sum('cleanup_incomplete' in reason for reason in errors) == 3


def test_supervisor_preserves_nonzero_flush_exit_beside_successful_peer(clock):
    children = [(Child(clock, 100, 6, 17), io.StringIO()), (Child(clock, 101, 6, 0), io.StringIO())]
    result, errors = supervisor.finish_children(children, supervisor.shutdown_policy({'role': 'record'}))
    assert result == 17 and errors == ['component_cleanup_failure:100:17']


@pytest.mark.parametrize('state, code, expected', [('STOPPED', 0, 0), ('FAILED', 17, 1)])
def test_cli_waits_for_record_flush_and_reports_final_failure(tmp_path, monkeypatch, clock, state, code, expected):
    path = tmp_path/'sessions'/'record-fixture'/'record'/'manifest.json'
    path.parent.mkdir(parents=True)
    data = {'role': 'record', 'state': 'RUNNING', 'sigint_grace_s': 30}
    path.write_text(json.dumps(data), encoding='utf-8')
    calls = []
    monkeypatch.setattr(cli, 'RUN', tmp_path)
    monkeypatch.setattr(cli, 'target', lambda: None)
    monkeypatch.setattr(cli, 'stop_registered', lambda observed: calls.append(observed))

    def complete():
        if clock.now >= 35:
            path.write_text(json.dumps({**data, 'state': state, 'exit_code': code}), encoding='utf-8')

    clock.after_sleep = complete
    assert cli.main(['stop', '--session', 'record-fixture']) == expected
    assert calls == [path] and 35 <= clock.now < 36


@pytest.mark.skipif(sys.platform != 'linux', reason='Actual synthetic process shutdown requires Linux pidfd/subreaping')
@pytest.mark.parametrize('exit_code', [0, 17])
def test_linux_record_writer_flushes_six_seconds_then_preserves_actual_exit(exit_code):
    from wc_runtime.project_paths import project_root
    project = project_root()
    assert project.is_dir(), 'Run this process acceptance in the selected Linux project'
    session = 'ops-record-flush-'+uuid.uuid4().hex
    root = project/'.phase1_runtime'/'sessions'/session/'record'
    root.mkdir(parents=True, exist_ok=False)
    ready, evidence = root/'writer-ready.json', root/'flush-complete.json'
    helper = root/'synthetic-writer.py'
    helper.write_text('''import json, os, signal, sys, time
from pathlib import Path
stopped = False
def stop(sig, frame):
    global stopped
    stopped = True
signal.signal(signal.SIGINT, stop)
signal.signal(signal.SIGTERM, lambda sig, frame: sys.exit(23))
stat = Path('/proc/self/stat').read_text()
Path(sys.argv[1]).write_text(json.dumps({'pid': os.getpid(), 'start_ticks': stat[stat.rindex(')')+2:].split()[19]}))
deadline = time.monotonic()+55
while not stopped and time.monotonic() < deadline:
    time.sleep(.02)
if not stopped:
    sys.exit(24)
start = time.monotonic()
time.sleep(6)
Path(sys.argv[2]).write_text(json.dumps({'data_source': 'SYNTHETIC_PROCESS_NO_ROS_NO_HARDWARE', 'flush_elapsed_s': time.monotonic()-start}))
sys.exit(int(sys.argv[3]))
''', encoding='utf-8')
    plan = {'session_id': session, 'role': 'record', 'commands': [[sys.executable, str(helper), str(ready), str(evidence), str(exit_code)]],
            'duration_s': 0, 'locks': [], 'environment': {'PYTHONNOUSERSITE': '1'}, 'allow_component_exit': True}
    path = root/'plan.json'
    path.write_text(json.dumps(plan), encoding='utf-8')
    with (root/'supervisor.log').open('w', encoding='utf-8') as log:
        proc = subprocess.Popen([sys.executable, '-m', 'wc_runtime.supervisor', '--plan', str(path)],
                                cwd=project, stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            deadline = time.monotonic()+8
            while time.monotonic() < deadline and not (ready.exists() and (root/'manifest.json').exists()):
                assert proc.poll() is None
                time.sleep(.03)
            assert ready.exists() and (root/'manifest.json').exists()
            assert supervisor.stop_registered(root/'manifest.json')
            assert proc.wait(timeout=47) == exit_code
            manifest = json.loads((root/'manifest.json').read_text())
            assert manifest['state'] == ('STOPPED' if exit_code == 0 else 'FAILED')
            assert manifest['exit_code'] == exit_code
            assert json.loads(evidence.read_text())['flush_elapsed_s'] >= 6
            identity = json.loads(ready.read_text())
            assert component.process(identity['pid']) is None, 'writer must be fully reaped, including zombies'
            assert all(component.process(child['pid']) is None for child in manifest['children'])
        finally:
            if proc.poll() is None:
                # Test-owned supervisor PID only; its component wrapper retains
                # the existing parent-death cleanup. Never signal by name.
                proc.kill()
                proc.wait(timeout=5)
