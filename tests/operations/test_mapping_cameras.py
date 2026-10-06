"""Synthetic optional-camera ownership/lifecycle tests; no ROS, camera or display."""
import copy
import io
import json
import os
from pathlib import Path
import shutil
import signal
import sys
import threading
import time
from types import SimpleNamespace as NS
import uuid

import pytest

from wc_cameras.config import ROLES, load_config
from wc_runtime import mapping_cameras as m


PROJECT = Path(__file__).resolve().parents[2]


@pytest.fixture
def tmp_path():
    parent = Path(__file__).absolute().parent
    path = parent/('.mapping_cameras_test_'+uuid.uuid4().hex)
    path.mkdir()
    try:
        yield path
    finally:
        resolved = path.resolve()
        assert resolved.parent == parent and resolved.name.startswith('.mapping_cameras_test_')
        shutil.rmtree(resolved)


def wait_until(predicate):
    deadline = time.monotonic()+2
    while not predicate():
        assert time.monotonic() < deadline
        time.sleep(.002)


class MemoryWriter:
    error = None
    def __init__(self): self.latest = self.final = None
    def submit(self, value): self.latest = copy.deepcopy(value)
    def close(self, value, timeout_s=m.STATUS_CLOSE_S):
        self.final = copy.deepcopy(value)
        self.close_timeout_s = timeout_s
        return {'final_fsync_complete': True, 'error': None}


class Lease:
    def __init__(self, failure=False): self.failure, self.acquired, self.closed = failure, False, False
    def acquire(self):
        if self.failure: raise BlockingIOError('camera domain already owned')
        self.acquired = True
    def pass_fds(self): return (99,)
    def close(self): self.closed = True


class Child:
    def __init__(self, pid, role):
        self.pid, self.returncode = pid, None
        self.stdout = io.BytesIO(('[INFO] '+json.dumps({'event':'CAMERA_READY','logical_slot':role})+'\n').encode())
    def poll(self): return self.returncode


@pytest.fixture
def companion(tmp_path):
    session = tmp_path/'session';session.mkdir()
    config = session/'cameras_config.json'
    config.write_bytes((PROJECT/'config/cameras.json').read_bytes())
    created = []
    def make(*, unavailable=None, lock_busy=False, writer=None, clock=time.monotonic, duration=60):
        launched = []
        def launch(argv, **kwargs):
            role = argv[argv.index('--role')+1]
            if role == unavailable: raise OSError('synthetic executable failure')
            child = Child(100+len(launched), role)
            launched.append((child, argv, kwargs))
            return child
        lease = Lease(lock_busy)
        value = m.CameraCompanion(config, session/'cameras', tmp_path, tmp_path/'.phase1_runtime', 'fixture', duration,
            owner=NS(pid=90, start_ticks='owner'), launch=launch, lease=lease, writer=writer or MemoryWriter(), clock=clock)
        created.append(value)
        return value, launched, lease
    yield make
    for value in created:
        if not value.closed:
            value.close(cleanup=clean_close)


def clean_close(owner, children, **kwargs):
    for child in children:
        if child.returncode is None: child.returncode = 0
    return {'status':'PASS','remaining':[],'forced':False,'errors':[],'signals':[],'elapsed_s':.1}


def test_commands_bind_same_snapshot_profiles_ports_topics_and_never_record_images():
    config, _ = load_config(PROJECT/'config/cameras.json')
    commands = m.child_commands('/session/cameras_config.json', config, 'same_session', '/shared/runtime', 600,
                                executable='/usr/bin/python3')
    assert [row['role'] for row in commands] == list(ROLES)
    assert [row['port'] for row in commands] == [1,4,2,3]
    assert {row['profile'] for row in commands} == {'monitor_320'}
    for row in commands:
        argv = row['argv']
        assert argv[:4] == ['/usr/bin/python3','-s','-m','wc_cameras.node']
        assert argv[argv.index('--config')+1] == '/session/cameras_config.json'
        assert argv[argv.index('--session-id')+1] == 'same_session'
        assert argv[argv.index('--run-root')+1] == '/shared/runtime'
        assert row['topic'] == '/wc_mapping/cameras/'+row['role']+'/image_raw'
        assert not any(value in ('record','bag','--output') for value in argv)


def test_four_independent_children_keep_images_out_of_status_and_do_not_claim_live_rate(companion):
    value, launched, lease = companion()
    value.start()
    wait_until(lambda: all(row['reader_done'] for row in value.rows.values()))
    state = value.status()
    assert len(launched) == 4 and lease.acquired
    assert state['state'] == 'RUNNING' and state['optional_for_mapping'] is True
    assert state['live_image_rate_verified_by_companion'] is False
    assert state['images_saved'] is state['images_recorded_to_bag'] is False
    assert all(row['state'] == 'PROCESS_RUNNING' and row['capture_ready'] for row in state['slots'].values())
    for child, argv, kwargs in launched:
        assert kwargs['stdout'] == m.subprocess.PIPE and kwargs['stderr'] == m.subprocess.STDOUT
        assert kwargs['pass_fds'] == (99,) and kwargs['start_new_session'] is True
    assert not list(value.output.glob('*.png')) and not (value.output/'bag').exists()


@pytest.mark.parametrize('exit_code', [0,1,75])
def test_one_camera_exit_degrades_without_stopping_or_restarting_others(companion,exit_code):
    value, launched, lease = companion()
    value.start();launched[1][0].returncode = exit_code
    for _ in range(4): value.tick()
    status = value.status()
    assert len(launched) == 4 and status['state'] == 'DEGRADED'
    assert status['slots']['right_front']['exit_code'] == exit_code
    assert all(child.returncode is None for child, _, _ in [launched[0],launched[2],launched[3]])
    assert not lease.closed and not value.closed
    result = value.close(cleanup=clean_close)
    assert result['status'] == 'PASS' and value.status()['state'] == 'STOPPED_WITH_CAMERA_ERRORS'
    assert lease.closed


def test_unavailable_one_slot_does_not_prevent_other_launches(companion):
    value, launched, lease = companion(unavailable='left_side')
    value.start()
    assert len(launched) == 3 and value.status()['slots']['left_side']['state'] == 'UNAVAILABLE'
    assert set(value.children) == {'left_front','right_front','right_side'}
    assert value.status()['state'] == 'DEGRADED' and lease.acquired


def test_busy_shared_domain_degrades_and_never_launches_or_kills_existing_owner(companion):
    value, launched, lease = companion(lock_busy=True)
    value.start();value.tick()
    assert launched == [] and not lease.acquired
    assert value.status()['state'] == 'DEGRADED' and 'already owned' in value.status()['problem']
    assert all(row['state'] == 'UNAVAILABLE' for row in value.status()['slots'].values())
    assert value.close(cleanup=clean_close)['status'] == 'PASS'


def test_failed_cleanup_retains_lock_and_reports_remaining_owned_process(companion):
    value, launched, lease = companion()
    value.start()
    def incomplete(owner, children, **kwargs):
        return {'status':'FAIL','remaining':[{'pid':children[0].pid,'start_ticks':'owned','state':'D'}],
                'forced':True,'errors':['kernel wait'],'signals':[],'elapsed_s':9.0}
    result = value.close(cleanup=incomplete)
    assert result['status'] == 'FAIL' and not lease.closed
    assert value.status()['state'] == 'FAILED' and value.writer.final['state'] == 'FAILED'


def test_capture_worker_forced_shutdown_cannot_be_reported_as_normal_close(companion):
    value, launched, lease = companion()
    value.start()
    wait_until(lambda: all(row['reader_done'] for row in value.rows.values()))
    with value.lock:
        value.rows['left_front']['capture_stop'] = {
            'event':'CAMERA_STOPPED', 'cleanup':{'status':'FAIL','termination':'terminate',
                                               'alive_after_cleanup':False}}
    result = value.close(cleanup=clean_close)
    assert result['status'] == 'FAIL' and lease.closed
    assert result['camera_cleanup_errors']['left_front']['termination'] == 'terminate'
    assert value.status()['state'] == 'FAILED'


def test_reaper_preserves_direct_node_failure_when_exit_races_popen_poll(monkeypatch):
    child = Child(20, 'left_front')
    events = iter([(20, 7 << 8), (21, 0), (0, 0)])
    monkeypatch.setattr(m.os, 'WNOHANG', 1, raising=False)
    monkeypatch.setattr(m.os, 'waitpid', lambda *_: next(events))
    # Windows encodes its wait status differently; this models the Linux CLI.
    monkeypatch.setattr(m.os, 'waitstatus_to_exitcode', lambda status: status >> 8)
    m.reap_owned([child])
    assert child.returncode == 7


def test_output_and_snapshot_must_share_new_owned_session(tmp_path):
    session = tmp_path/'session';session.mkdir()
    other = tmp_path/'other';other.mkdir()
    config = session/'cameras_config.json';config.write_bytes((PROJECT/'config/cameras.json').read_bytes())
    with pytest.raises(m.CameraError,match='same owned session'):
        m.CameraCompanion(config,other/'cameras',tmp_path,tmp_path/'run','fixture',60,writer=MemoryWriter())
    output = session/'cameras';output.mkdir();(output/'original').write_bytes(b'unchanged')
    with pytest.raises(FileExistsError):
        m.CameraCompanion(config,output,tmp_path,tmp_path/'run','fixture',60,writer=MemoryWriter())
    assert (output/'original').read_bytes() == b'unchanged'


def test_status_writer_stall_coalesces_without_blocking_camera_lifecycle(companion):
    entered, release = threading.Event(), threading.Event()
    writes = []
    def blocked(value, final):
        entered.set();assert release.wait(3)
        writes.append((value,final,threading.current_thread().name))
    writer = m.StatusWriter(None,None,write=blocked)
    value, launched, lease = companion(writer=writer)
    try:
        value.start();assert entered.wait(1)
        launched[0][0].returncode = 1
        for _ in range(10): value.tick()
        assert value.status()['state'] == 'DEGRADED' and len(launched) == 4
        assert writer.latest['slots']['left_front']['exit_code'] == 1
        assert len(writes) == 0
    finally:
        release.set();value.close(cleanup=clean_close)
    assert writes[-1][1] is True and all(row[2]=='camera-preview-status' for row in writes)
    assert len(writes) <= 3 and writer.synced


def test_status_writer_timeout_does_not_wait_indefinitely_or_claim_synced():
    entered, release = threading.Event(), threading.Event()
    def blocked(value,final): entered.set();assert release.wait(3)
    writer = m.StatusWriter(None,None,write=blocked)
    writer.submit({'state':'RUNNING'});assert entered.wait(1)
    try:
        result = writer.close({'state':'STOPPED'},timeout_s=.01)
        assert result['error']=='CAMERA_STATUS_CLOSE_TIMEOUT' and result['final_fsync_complete'] is False
        assert writer.thread.is_alive() and result['worker_exited'] is False
    finally:
        release.set();writer.thread.join(2)
    assert writer.snapshot()['worker_exited'] is True and writer.snapshot()['final_fsync_complete'] is True
    assert writer.snapshot()['error']=='CAMERA_STATUS_CLOSE_TIMEOUT'


def test_status_writer_real_final_json_contains_no_images(tmp_path):
    output=tmp_path/'cameras';output.mkdir()
    writer=m.StatusWriter(tmp_path,output)
    final={'state':'STOPPED','images_saved':False,'slots':{role:{'exit_code':0} for role in ROLES}}
    result=writer.close(final)
    assert result['final_fsync_complete'] is True and result['error'] is None
    assert json.loads((output/'status.json').read_text())==final
    assert {path.name for path in output.iterdir()}=={'status.json'}


def test_status_write_failure_is_explicit_without_stopping_healthy_preview(companion):
    def fail(value, final): raise OSError('synthetic full metadata filesystem')
    writer = m.StatusWriter(None, None, write=fail)
    value, launched, lease = companion(writer=writer)
    value.start()
    wait_until(lambda: writer.error is not None)
    value.tick()
    assert value.status()['state'] == 'DEGRADED'
    assert all(child.returncode is None for child, _, _ in launched)
    result = value.close(cleanup=clean_close)
    assert result['status'] == 'PASS' and lease.closed
    assert result['storage_status'] == 'FAIL' and value.status()['state'] == 'FAILED_STORAGE'
    assert result['status_storage']['final_fsync_complete'] is False
    assert 'synthetic full metadata filesystem' in result['status_storage']['error']


def test_camera_status_default_close_waits_for_final_owner_write_and_fits_component_budget(tmp_path,monkeypatch):
    output=tmp_path/'cameras'; output.mkdir()
    entered,release=threading.Event(),threading.Event()
    writes,joins=[],[]
    writer=m.StatusWriter(tmp_path,output)
    original_write=writer.write
    def write(value,final):
        entered.set(); assert release.wait(3)
        writes.append((value['state'],final,threading.current_thread().name))
        original_write(value,final)
    writer.write=write
    writer.submit({'state':'RUNNING'}); assert entered.wait(1)
    original_join=writer.thread.join
    def join(timeout):
        joins.append(timeout); release.set()
        return original_join(timeout)
    monkeypatch.setattr(writer.thread,'join',join)
    try:
        result=writer.close({'state':'STOPPED'})
    finally:
        release.set(); original_join(2)
    assert joins==[10.] and m.STATUS_CLOSE_S==10.
    assert sum(seconds for _,seconds in m.STOP_PHASES)==9.
    # Four slots, at most an initial reader plus two restart readers each.
    assert 9.+len(ROLES)*(1+m.MAX_RESTARTS_PER_SLOT)*.02+m.STATUS_CLOSE_S < 30.
    assert writes==[('RUNNING',False,'camera-preview-status'),('STOPPED',True,'camera-preview-status')]
    assert result['final_fsync_complete'] is True and result['worker_exited'] is True
    assert result['error'] is None and result['pending_status'] is False
    assert json.loads((output/'status.json').read_text())=={'state':'STOPPED'}


def test_camera_final_sync_error_does_not_replace_previous_status_with_false_success(tmp_path,monkeypatch):
    output=tmp_path/'cameras'; output.mkdir()
    writer=m.StatusWriter(tmp_path,output)
    writer.submit({'state':'RUNNING'})
    wait_until(lambda: writer.snapshot()['snapshots_written']==1)
    def fail_sync(_): raise OSError('synthetic final fsync failure')
    monkeypatch.setattr(m.os,'fsync',fail_sync)
    result=writer.close({'state':'STOPPED'},timeout_s=1.)
    assert result['final_fsync_complete'] is False and result['worker_exited'] is True
    assert 'synthetic final fsync failure' in result['error']
    assert json.loads((output/'status.json').read_text())=={'state':'RUNNING'}


def test_camera_late_completion_keeps_timeout_and_first_final_request(tmp_path):
    output=tmp_path/'cameras'; output.mkdir()
    writer=m.StatusWriter(tmp_path,output)
    writer.submit({'state':'RUNNING'})
    wait_until(lambda: writer.snapshot()['snapshots_written']==1)
    entered,release=threading.Event(),threading.Event()
    original=writer.write
    def blocked(value,final):
        entered.set(); assert release.wait(3)
        original(value,final)
    writer.write=blocked
    try:
        result=writer.close({'state':'STOPPED','original_final':True},timeout_s=.01)
        assert entered.is_set() and result['error']=='CAMERA_STATUS_CLOSE_TIMEOUT'
        assert json.loads((output/'status.json').read_text())=={'state':'RUNNING'}
    finally:
        release.set(); writer.thread.join(2)
    again=writer.close({'state':'WRONG_SECOND_CLOSE'},timeout_s=1.)
    assert again['worker_exited'] and again['final_fsync_complete']
    assert again['error']=='CAMERA_STATUS_CLOSE_TIMEOUT' and not again['pending_status']
    assert json.loads((output/'status.json').read_text())=={'state':'STOPPED','original_final':True}


def test_optional_camera_final_storage_timeout_preserves_cleanup_success_but_reports_failure(companion):
    class TimedOutWriter(MemoryWriter):
        def close(self,value,timeout_s=m.STATUS_CLOSE_S):
            self.final=copy.deepcopy(value)
            self.error='CAMERA_STATUS_CLOSE_TIMEOUT'
            return {'final_fsync_complete':False,'error':self.error,'worker_exited':False}
    value, launched, lease=companion(writer=TimedOutWriter())
    value.start()
    wait_until(lambda: all(row['reader_done'] for row in value.rows.values()))
    result=value.close(cleanup=clean_close)
    assert result['status']=='PASS' and lease.closed  # Optional preview cannot invalidate core-map cleanup.
    assert result['storage_status']=='FAIL' and result['status_storage']['error']=='CAMERA_STATUS_CLOSE_TIMEOUT'
    assert value.status()['state']=='FAILED_STORAGE'
    assert all(row['state']=='STOPPED' and row['exit_code']==0 for row in value.status()['slots'].values())


class FakeClock:
    def __init__(self): self.now=0.
    def clock(self): return self.now
    def wait(self,seconds): self.now+=seconds


def stopped_after_read_deadline():
    return {'event': 'CAMERA_STOPPED', 'logical_slot': 'right_front', 'published_frames': 266,
            'failure_latched': True, 'error': m.READ_DEADLINE_ERROR,
            'cleanup': {'status': 'PASS', 'termination': 'normal', 'alive_after_cleanup': False,
                        'final_exit_code': 0, 'errors': [], 'owned_pid': 900,
                        'reap_observed_host_monotonic_ns': 5_000_000_000,
                        'late_event_drain': 'DRAINED_AFTER_CHILD_EXIT',
                        'late_events': [{'event': 'RELEASE_COMPLETE',
                            'host_release_start_monotonic_ns': 3_000_000_000,
                            'host_release_complete_monotonic_ns': 4_900_000_000,
                            'host_release_elapsed_ns': 1_900_000_000}]}}


def stop_slot_for_restart(value, role='right_front'):
    wait_until(lambda: value.rows[role]['reader_done'])
    with value.lock:
        value.rows[role]['capture_stop'] = stopped_after_read_deadline()
    value.children[role].returncode = 1
    value.tick()


def test_read_deadline_restarts_only_reaped_slot_and_preserves_disruption_history(companion):
    clock = FakeClock()
    value, launched, lease = companion(clock=clock.clock)
    value.start()
    originals = dict(value.children)
    stop_slot_for_restart(value)
    status = value.status()['slots']['right_front']
    assert status['failure_count'] == 1 and status['restart_state'] == 'WAITING_RESTART'
    assert len(launched) == 4
    clock.now = .99; value.tick(); assert len(launched) == 4
    clock.now = 1.; value.tick()
    wait_until(lambda: value.rows['right_front']['reader_done'])
    assert len(launched) == 5 and value.children['right_front'] is not originals['right_front']
    assert all(value.children[role] is originals[role] for role in ROLES if role != 'right_front')
    _, argv, options = launched[-1]
    assert argv[argv.index('--duration')+1] == '59.0'
    assert argv[argv.index('--config')+1] == str(value.config_path)
    assert argv[argv.index('--session-id')+1] == 'fixture' and options['pass_fds'] == (99,)
    row = value.status()['slots']['right_front']
    assert row['restart_attempts'] == row['restart_count'] == 1
    assert row['restart_state'] == 'REOPENED_PREVIEW_RATE_NOT_VERIFIED'
    assert row['capture_stop'] is None and row['state'] == 'PROCESS_RUNNING'
    assert row['attempt_history'][0]['pid'] == originals['right_front'].pid
    assert row['attempt_history'][0]['capture_stop']['published_frames'] == 266
    assert row['attempt_history'][0]['capture_stop']['cleanup']['late_events'][0]['host_release_elapsed_ns'] == 1_900_000_000
    assert value.status()['state'] == 'DEGRADED'  # A successful reopen does not erase a disruption.
    assert value.close(cleanup=clean_close)['status'] == 'PASS' and lease.closed
    final = value.writer.final
    assert final['state'] == 'STOPPED_WITH_CAMERA_ERRORS'
    assert final['slots']['right_front']['state'] == 'STOPPED'
    assert final['slots']['right_front']['exit_code'] == 0
    assert final['slots']['right_front']['failure_count'] == 1
    assert final['slots']['right_front']['restart_count'] == 1
    assert final['slots']['right_front']['attempt_history'][0]['capture_stop']['error'] == m.READ_DEADLINE_ERROR


def test_until_close_camera_restart_has_no_hidden_deadline_and_keeps_restart_budget(companion):
    clock = FakeClock()
    value, launched, lease = companion(clock=clock.clock, duration=0)
    value.start()
    assert value.deadline is None
    for index in range(m.MAX_RESTARTS_PER_SLOT):
        clock.now = 50000. * (index+1)
        stop_slot_for_restart(value)
        assert value.rows['right_front']['restart_state'] == 'WAITING_RESTART'
        clock.now += m.RESTART_DELAY_S
        value.tick()
        assert launched[-1][1][launched[-1][1].index('--duration')+1] == '0'
    stop_slot_for_restart(value)
    assert value.rows['right_front']['restart_state'] == 'RESTART_LIMIT_REACHED'
    assert len(launched) == 4+m.MAX_RESTARTS_PER_SLOT
    value.close(cleanup=clean_close)
    assert value.closed and value.stopping and lease.closed


def test_three_read_failures_exhaust_two_restarts_without_affecting_other_slots(companion):
    clock = FakeClock()
    value, launched, lease = companion(clock=clock.clock)
    value.start()
    for attempt in range(3):
        clock.now = attempt*5.
        stop_slot_for_restart(value)
        clock.now += m.RESTART_DELAY_S
        value.tick()
    row = value.status()['slots']['right_front']
    assert len(launched) == 6 and row['restart_count'] == row['restart_attempts'] == 2
    assert row['failure_count'] == 3 and len(row['attempt_history']) == 3
    assert row['restart_state'] == 'RESTART_LIMIT_REACHED'
    for _ in range(10):
        clock.now += 1; value.tick()
    assert len(launched) == 6
    assert all(value.children[role].returncode is None for role in ROLES if role != 'right_front')
    assert not value.closed and not lease.closed


@pytest.mark.parametrize('change', [
    {'status': 'FAIL'}, {'termination': 'terminate'}, {'termination': 'kill'},
    {'alive_after_cleanup': True}, {'final_exit_code': 1}, {'errors': ['read failed during close']},
    {'late_event_drain': 'SKIPPED_UNREAPED_CHILD'}, {'owned_pid': None},
    {'reap_observed_host_monotonic_ns': None}, {'reap_observed_host_monotonic_ns': 4_000_000_000},
    {'late_events': []}, {'late_events': None},
])
def test_restart_requires_positive_normal_release_and_reap_evidence(change):
    stopped = stopped_after_read_deadline()
    stopped['cleanup'].update(change)
    row = {'exit_code': 1, 'reader_done': True, 'capture_ready': {'event': 'CAMERA_READY'}, 'capture_stop': stopped}
    allowed, reason = m.restart_eligibility(row)
    assert allowed is False and reason


@pytest.mark.parametrize('change', [
    {'exit_code': None}, {'exit_code': 0}, {'reader_done': False}, {'reader_error': 'incomplete read'},
    {'capture_ready': None}, {'capture_stop': None},
])
def test_restart_rejects_unconfirmed_exit_or_incomplete_output(change):
    row = {'exit_code': 1, 'reader_done': True, 'capture_ready': {'event': 'CAMERA_READY'},
           'capture_stop': stopped_after_read_deadline()}
    row.update(change)
    assert m.restart_eligibility(row)[0] is False


def test_restarted_slot_with_identity_or_lease_failure_is_not_retried(companion):
    clock = FakeClock()
    value, launched, _ = companion(clock=clock.clock)
    value.start(); stop_slot_for_restart(value)
    clock.now = 1.; value.tick()
    wait_until(lambda: value.rows['right_front']['reader_done'])
    with value.lock:
        value.rows['right_front']['capture_ready'] = None
        value.rows['right_front']['capture_stop'] = {**stopped_after_read_deadline(),
            'error': 'USB port identity or capture capability mismatch'}
    value.children['right_front'].returncode = 1
    value.tick(); clock.now = 10.; value.tick()
    row = value.status()['slots']['right_front']
    assert len(launched) == 5 and row['restart_state'] == 'NOT_ELIGIBLE'
    assert row['failure_count'] == 2 and 'identity' in row['attempt_history'][-1]['capture_stop']['error']


def test_restart_never_extends_session_and_stop_cancels_pending_restart(companion):
    clock = FakeClock()
    value, launched, _ = companion(clock=clock.clock)
    value.start(); stop_slot_for_restart(value)
    value.request_stop(); clock.now = 1.; value.tick()
    assert len(launched) == 4 and value.deadline == 60.
    value.close(cleanup=clean_close); value.tick()
    assert len(launched) == 4


def test_read_failure_near_session_end_is_not_restarted(companion):
    clock = FakeClock()
    value, launched, _ = companion(clock=clock.clock)
    value.start(); clock.now = 45.; stop_slot_for_restart(value)
    clock.now = 46.; value.tick()
    assert len(launched) == 4
    assert value.status()['slots']['right_front']['restart_state'] == 'SUPPRESSED_SESSION_END'


def test_changed_snapshot_blocks_restart_and_preserves_remaining_cameras(companion):
    clock = FakeClock()
    value, launched, _ = companion(clock=clock.clock)
    value.start(); stop_slot_for_restart(value)
    changed = json.loads(value.config_path.read_text())
    changed['mapping_note'] += ' synthetic change'
    value.config_path.write_text(json.dumps(changed))
    clock.now = 1.; value.tick(); clock.now = 5.; value.tick()
    assert len(launched) == 4
    row = value.status()['slots']['right_front']
    assert row['restart_state'] == 'RESTART_FAILED_NO_FURTHER_ATTEMPT'
    assert 'configuration changed' in row['error']
    assert all(value.children[role].returncode is None for role in ROLES if role != 'right_front')


def test_restart_spawn_failure_stays_bounded_and_keeps_original_failure_evidence(companion):
    clock = FakeClock()
    value, launched, _ = companion(clock=clock.clock)
    value.start(); stop_slot_for_restart(value)
    calls = []
    def failed_spawn(*args, **kwargs):
        calls.append(args)
        raise OSError('synthetic no process slots')
    value.launch = failed_spawn
    clock.now = 1.; value.tick()
    for _ in range(10):
        clock.now += 1; value.tick()
    row = value.status()['slots']['right_front']
    assert len(calls) == 1 and len(launched) == 4 and 'right_front' not in value.children
    assert row['failure_count'] == 2 and row['restart_attempts'] == 1 and row['restart_count'] == 0
    assert row['restart_state'] == 'RESTART_FAILED_NO_FURTHER_ATTEMPT'
    assert 'no process slots' in row['error'] and row['attempt_history'][0]['capture_stop']['published_frames'] == 266


def test_cleanup_signals_four_direct_children_with_one_shared_normal_deadline():
    owner=NS(pid=10,start_ticks='owner');children=[Child(i,ROLES[i-20]) for i in range(20,24)]
    rows=[NS(pid=child.pid,start_ticks=str(child.pid),state='S') for child in children]
    clock=FakeClock();sent=[]
    def remaining(_): return rows if clock.now<1 else []
    def send(record,actual_owner,signum):
        assert actual_owner is owner;sent.append((record.pid,signum));return True
    result=m.close_children(owner,children,descendants=remaining,send=send,clock=clock.clock,wait=clock.wait)
    assert result['status']=='PASS' and result['forced'] is False
    assert sent==[(child.pid,signal.SIGINT) for child in children]
    assert result['elapsed_s']<1.1


def test_cleanup_escalates_only_owned_descendants_and_is_bounded_across_all_slots():
    owner=NS(pid=10,start_ticks='owner');child=Child(20,'left_front')
    rows=[NS(pid=20,start_ticks='node',state='S'),NS(pid=21,start_ticks='capture',state='D')]
    clock=FakeClock();sent=[]
    def remaining(_): return rows if clock.now<8.5 else []
    def send(record,actual_owner,signum):
        assert record in rows and actual_owner is owner
        sent.append((record.pid,signum));return True
    result=m.close_children(owner,[child],descendants=remaining,send=send,clock=clock.clock,wait=clock.wait)
    assert sent[0]==(20,signal.SIGINT) and (21,signal.SIGINT) not in sent
    assert (21,signal.SIGTERM) in sent and (21,m.KILL_SIGNAL) in sent
    assert result['status']=='FAIL' and result['forced'] is True and result['elapsed_s']<=9.


def test_cleanup_retains_unreaped_identity_and_does_not_signal_pid_after_ownership_rejection():
    owner=NS(pid=10,start_ticks='owner');child=Child(20,'left_front')
    row=NS(pid=20,start_ticks='original_identity',state='D')
    clock=FakeClock();attempts=[]
    def send(record,actual_owner,signum):
        # Models the existing pidfd/start_ticks/ancestry gate rejecting a race.
        attempts.append((record.pid,record.start_ticks));return False
    result=m.close_children(owner,[child],descendants=lambda _:[row],send=send,clock=clock.clock,wait=clock.wait)
    assert result['status']=='FAIL' and result['signals']==[] and result['elapsed_s']<=9.
    assert result['remaining']==[{'pid':20,'start_ticks':'original_identity','state':'D'}]
    assert set(attempts)=={(20,'original_identity')}


@pytest.mark.skipif(sys.platform!='linux',reason='Linux flock on temporary files only')
def test_domain_lease_conflicts_with_existing_reviewed_camera_lock(tmp_path):
    import fcntl
    run=tmp_path/'runtime';locks=run/'locks';locks.mkdir(parents=True)
    with (locks/m.CAMERA_LOCK).open('a+') as existing:
        fcntl.flock(existing.fileno(),fcntl.LOCK_EX|fcntl.LOCK_NB)
        lease=m.DomainLease(tmp_path,run)
        with pytest.raises(BlockingIOError):lease.acquire()
        assert lease.stream is None
    lease.acquire();assert lease.pass_fds();lease.close()
