"""Offline serial transaction fixtures; PTY cases run only on Linux, never hardware."""

import copy
import json
import os
from pathlib import Path
import shutil
import stat
import struct
import sys
import threading
import time
from types import SimpleNamespace
import uuid

import pytest

from wc_motion import feedback_transport as ft
from wc_motion.protocol import FeedbackError, crc16, read_request
from wc_motion.ros_node import FeedbackProcessor


CONFIG_PATH = Path(__file__).resolve().parents[2]/'config/wheel_feedback_current.json'


@pytest.fixture
def tmp_path():
    parent=Path(__file__).absolute().parent
    directory=parent/('.feedback_transport_test_'+uuid.uuid4().hex)
    directory.mkdir()
    try:
        yield directory
    finally:
        resolved=directory.resolve()
        assert resolved.parent==parent and resolved.name.startswith('.feedback_transport_test_')
        shutil.rmtree(resolved)


@pytest.fixture
def config():
    return json.loads(CONFIG_PATH.read_text(encoding='utf-8'))


def response(left=0, right=0, slave=1, function=3):
    payload = bytes((slave, function, 4))+struct.pack('>HH', left & 0xffff, right & 0xffff)
    return payload+crc16(payload)


@pytest.mark.parametrize('change', [{'request_hex': read_request(1, 0x2088, 2).hex()},
    {'request_hex': read_request(2, 0x20ab, 2).hex()}, {'baud': 9600}, {'hardware_serial': '0000000015'},
    {'usb_pid': '0000'}, {'device': '/dev/ttyACM0'}, {'read_request_reviewed': False},
    {'timeout_s': float('nan')}, {'timeout_s': -1}, {'timeout_s': 1}])
def test_config_rejects_other_devices_queries_and_unbounded_time(config, change):
    with pytest.raises(FeedbackError):
        ft.validate_config({**config, **change})


def test_exact_reviewed_query_and_no_control_function_api(config):
    assert ft.QUERY.hex(' ') == '01 03 20 ab 00 02 be 2b'
    assert ft.validate_config(config) is config
    assert not any(hasattr(ft.FeedbackSerialLease, name) for name in ('write', 'write_register', 'enable', 'stop_motor'))


def termios_fixture():
    names = ('IGNBRK BRKINT PARMRK ISTRIP INLCR IGNCR ICRNL IXON IXOFF IXANY OPOST '
             'PARENB CSTOPB CRTSCTS HUPCL CS8 CLOCAL CREAD ECHO ECHONL ICANON ISIG IEXTEN').split()
    values = {name: 1 << index for index, name in enumerate(names)}
    values.update(CSIZE=values['CS8'] | (1 << 25), CBAUD=1 << 26, B115200=4098, VMIN=0, VTIME=1)
    return SimpleNamespace(**values)


def test_effective_tty_readback_accepts_only_kernel_baud_flag_normalization():
    t = termios_fixture()
    attrs = ft.raw_attributes([0, 0, 0, 0, 0, 0, [0, 0]], t)
    normalized = copy.deepcopy(attrs)
    normalized[2] |= t.CBAUD
    normalized[6] = [b'\x00', b'\x00']
    ft.validate_raw_attributes(normalized, t)


@pytest.mark.parametrize('fault', ['baud_input', 'baud_output', 'hardware_flow', 'software_flow', 'parity', 'stopbits', 'hupcl', 'echo', 'canonical', 'vmin', 'cread', 'cs8'])
def test_effective_tty_readback_rejects_incorrect_baud_raw_and_flow_modes(fault):
    t = termios_fixture()
    attrs = ft.raw_attributes([0, 0, 0, 0, 0, 0, [0, 0]], t)
    if fault == 'baud_input': attrs[4] = 9600
    elif fault == 'baud_output': attrs[5] = 9600
    elif fault == 'hardware_flow': attrs[2] |= t.CRTSCTS
    elif fault == 'software_flow': attrs[0] |= t.IXON
    elif fault == 'parity': attrs[2] |= t.PARENB
    elif fault == 'stopbits': attrs[2] |= t.CSTOPB
    elif fault == 'hupcl': attrs[2] |= t.HUPCL
    elif fault == 'echo': attrs[3] |= t.ECHO
    elif fault == 'canonical': attrs[3] |= t.ICANON
    elif fault == 'vmin': attrs[6][t.VMIN] = 1
    elif fault == 'cread': attrs[2] &= ~t.CREAD
    elif fault == 'cs8': attrs[2] &= ~t.CS8
    with pytest.raises(FeedbackError):
        ft.validate_raw_attributes(attrs, t)


def io_fixture(monkeypatch, config, chunks, *, write_count=8, unsolicited=False):
    lease = ft.FeedbackSerialLease(config, '/fixture')
    lease.fd = 123
    writes = []
    reads = list(chunks)
    first = [True]

    def select(readable, writable, error, timeout):
        if first[0]:
            first[0] = False
            return ([123] if unsolicited else []), [], []
        return ([123] if readable and reads else []), ([123] if writable else []), []

    monkeypatch.setattr(ft.select, 'select', select)
    monkeypatch.setattr(ft.os, 'read', lambda fd, count: reads.pop(0))
    monkeypatch.setattr(ft.os, 'write', lambda fd, payload: writes.append(payload) or write_count)
    return lease, writes


def test_fragmented_reply_retains_both_raw_registers(config, monkeypatch):
    reply = response(-123, 456)
    lease, writes = io_fixture(monkeypatch, config, [reply[:1], reply[1:4], reply[4:]])
    assert lease.exchange(ft.QUERY, .2) == reply
    assert writes == [ft.QUERY]


@pytest.mark.parametrize('bad_reply', [b'\x01\x03', response()[:-1]+b'\xff', response(slave=2),
    response(function=4), response()+b'extra', b'\x01\x83\x02'+crc16(b'\x01\x83\x02'), b''])
def test_invalid_reply_latches_failure_without_retry(config, monkeypatch, bad_reply):
    lease, writes = io_fixture(monkeypatch, config, [bad_reply])
    with pytest.raises(ft.QueryFailure):
        lease.exchange(ft.QUERY, .2)
    with pytest.raises(ft.QueryFailure, match='failed lease'):
        lease.exchange(ft.QUERY, .2)
    assert lease.failed and writes == [ft.QUERY]


def test_partial_write_and_unsolicited_bytes_never_trigger_retry(config, monkeypatch):
    lease, writes = io_fixture(monkeypatch, config, [], write_count=3)
    with pytest.raises(ft.QueryFailure) as caught:
        lease.exchange(ft.QUERY, .2)
    assert caught.value.transmitted_bytes == 3 and writes == [ft.QUERY]
    lease, writes = io_fixture(monkeypatch, config, [response()], unsolicited=True)
    with pytest.raises(ft.QueryFailure, match='unsolicited'):
        lease.exchange(ft.QUERY, .2)
    assert writes == []


def test_non_allowlisted_bytes_cannot_reach_write(config, monkeypatch):
    lease, writes = io_fixture(monkeypatch, config, [])
    with pytest.raises(ft.QueryFailure):
        lease.exchange(read_request(1, 0x2088, 2), .2)
    assert writes == []


def test_raw_publication_schema_is_compatible_but_not_formal_odometry(config):
    journal = []
    lease = type('Lease', (), {'exchange': lambda self, query, timeout: response(-10, 10)})()
    poller = ft.FeedbackPoller(config, lease, journal.append, epoch='fixture')
    record = poller.sample()
    assert record['register_words_u16'] == [65526, 10]
    assert record['schema'] == 'wc_wheel_feedback_v1'
    assert record['time_valid'] is False and record['time_source'] == 'arrival_only'
    assert record['uncertainty_ns'] is None and record['formal_odometry_eligible'] is False
    assert [event['event'] for event in journal] == ['request_planned', 'transaction_complete']
    raw, odometry = FeedbackProcessor({'mode': 'raw_only'}).process(json.dumps(record))
    assert raw == record and odometry is None
    assert record['request_monotonic_ns']<=record['exchange_started_monotonic_ns']
    assert record['exchange_started_monotonic_ns']<=record['exchange_completed_monotonic_ns']<=record['receive_monotonic_ns']
    from wc_motion.history_preview import HistoryPreview
    from wc_runtime.mapping_prior import MotionPrior
    candidate=json.loads(CONFIG_PATH.with_name('wheel_history_calibration.json').read_text())
    assert HistoryPreview(candidate).update(record)['stamp_ns']==record['stamp_ns']
    prior=MotionPrior({'schema_version':1,'status':'EXPERIMENT','source_mode':'real',
        'base_frame':'lidar_left','reference_frame':'mapping_reference','guess_frame_id':'prior_odom',
        'imu_sensor_id':'H30-0000000015','R_base_imu':[[1.,0.,0.],[0.,1.,0.],[0.,0.,1.]],
        'T_base_axle':[[1.,0.,0.,0.],[0.,1.,0.,0.],[0.,0.,1.,0.],[0.,0.,0.,1.]],
        'wheel_candidate':candidate,'input_cloud_topic':'/wc_mapping/app/input_cloud',
        'output_cloud_topic':'/wc_mapping/app/scan_cloud','private_tf_topic':'/wc_mapping/app/tf'},'fixture')
    prior.add_wheel(record)
    assert prior.failure is None and prior.wheel[-1]['stamp']==record['stamp_ns']


def test_transaction_error_keeps_sent_and_received_bytes(config):
    journal = []
    def fail(*args):
        raise ft.QueryFailure('timeout', b'\x01\x03', 8)
    lease = type('Lease', (), {'exchange': fail})()
    poller = ft.FeedbackPoller(config, lease, journal.append)
    with pytest.raises(ft.QueryFailure):
        poller.sample()
    with pytest.raises(FeedbackError, match='failed stream'):
        poller.sample()
    assert journal[-1]['response_hex'] == '0103' and journal[-1]['transmitted_bytes'] == 8
    assert journal[-1]['event'] == 'transaction_failed'


def test_cli_requires_opt_in_before_opening_device_or_output(tmp_path, monkeypatch):
    monkeypatch.setattr(ft.FeedbackSerialLease, 'open', lambda _: pytest.fail('no hardware in fixture'))
    path = tmp_path/'must-not-exist.jsonl'
    with pytest.raises(FeedbackError, match='allow-read-queries'):
        ft.main(['--config', str(CONFIG_PATH), '--output', str(path), '--run-root', str(tmp_path)])
    assert not path.exists()


@pytest.mark.skipif(sys.platform != 'linux', reason='Linux PTY/termios acceptance, no real serial device')
def test_linux_exclusive_lease_queries_only_fc03_and_start_stop_emit_no_bytes(tmp_path, config):
    import pty
    import select
    master, slave = pty.openpty()
    device = Path(os.ttyname(slave))
    identity = device.stat()
    events = []
    def check(_):
        events.append('identity')
        return device, identity
    lease = ft.FeedbackSerialLease(config, tmp_path, identity_checker=check,
        occupancy_checker=lambda _: events.append('fuser'), exclusive_checker=lambda _: events.append('post-open'))
    try:
        lease.open()
        assert not select.select([master], [], [], .03)[0], 'startup wrote to motor bus'
        with pytest.raises(OSError):
            unexpected = os.open(device, os.O_RDWR | os.O_NOCTTY)
            os.close(unexpected)
        seen = []
        def reply():
            if select.select([master], [], [], 1)[0]:
                seen.append(os.read(master, 8))
                os.write(master, response(1, 2))
        thread = threading.Thread(target=reply)
        thread.start()
        assert lease.exchange(ft.QUERY, .2) == response(1, 2)
        thread.join(timeout=2)
        assert not thread.is_alive() and seen == [ft.QUERY]
        lease.close()
        assert not select.select([master], [], [], .03)[0], 'shutdown wrote to motor bus'
        assert events == ['identity', 'fuser', 'identity', 'post-open']
    finally:
        lease.close()
        os.close(master)
        os.close(slave)


@pytest.mark.skipif(sys.platform != 'linux', reason='Linux character-device/lock fixture, no hardware')
@pytest.mark.parametrize('fault', ['occupied', 'identity_changed', 'raced_owner'])
def test_linux_ownership_failure_occurs_before_any_bus_transmission(tmp_path, config, fault):
    import pty
    import select
    master, slave = pty.openpty()
    device = Path(os.ttyname(slave))
    identity = device.stat()
    calls = []
    def check(_):
        calls.append('identity')
        if fault == 'identity_changed' and len(calls) > 1:
            raise FeedbackError('changed')
        return device, identity
    def occupancy(_):
        if fault == 'occupied':
            raise FeedbackError('occupied')
    def exclusive(_):
        if fault == 'raced_owner':
            raise FeedbackError('raced owner')
    lease = ft.FeedbackSerialLease(config, tmp_path, identity_checker=check,
        occupancy_checker=occupancy, exclusive_checker=exclusive)
    try:
        with pytest.raises(FeedbackError):
            lease.open()
        assert lease.fd is None and lease.lock is None
        assert not select.select([master], [], [], .03)[0]
    finally:
        lease.close()
        os.close(master)
        os.close(slave)


class ObservedOutput:
    """A real fixture file with deterministic write blocking/failure hooks."""
    def __init__(self,path,*,block=False,fail=False):
        self.path=path;self.stream=path.open('x',encoding='utf-8')
        self.entered=threading.Event();self.release=threading.Event()
        if not block:self.release.set()
        self.fail=fail;self.write_threads=[];self.close_threads=[]
    @property
    def closed(self):return self.stream.closed
    def write(self,line):
        self.write_threads.append(threading.current_thread().name)
        self.entered.set()
        assert self.release.wait(3),'test must release the blocked writer'
        if self.fail:raise OSError('simulated disk write failure')
        return self.stream.write(line)
    def flush(self):return self.stream.flush()
    def fileno(self):return self.stream.fileno()
    def close(self):
        self.close_threads.append(threading.current_thread().name)
        return self.stream.close()


def wait_until(predicate):
    deadline=time.monotonic()+2
    while not predicate():
        assert time.monotonic()<deadline,'journal worker did not reach expected state'
        time.sleep(.002)


def test_blocked_journal_allows_multiple_mock_exchanges_then_preserves_fifo(config,tmp_path):
    output=ObservedOutput(tmp_path/'blocked.jsonl',block=True)
    journal=ft.AsyncJournal(output)
    calls=[]
    def exchange(query,timeout):
        calls.append((query,timeout,threading.current_thread().name))
        return response()
    poller=ft.FeedbackPoller(config,SimpleNamespace(exchange=exchange),journal,epoch='ordered-fixture')
    records=[]
    try:
        for _ in range(4):records.append(poller.sample())
        assert output.entered.wait(1)
        assert len(calls)==4 and all(query==ft.QUERY and timeout==.2 for query,timeout,_ in calls)
        assert all(name==threading.current_thread().name for _,_,name in calls)
        state=journal.status()
        assert state['enqueued_events']==state['pending_events']==8 and state['written_events']==0
        assert state['final_fsync_complete'] is False
        assert [record['sequence'] for record in records]==[0,1,2,3]
        assert all(record['time_source']=='arrival_only' and record['control_transmissions']==0 for record in records)
    finally:
        output.release.set();journal.close()
    rows=[json.loads(line) for line in output.path.read_text().splitlines()]
    events=rows[:-1]
    assert [row['event'] for row in events]==['request_planned','transaction_complete']*4
    assert [row['journal_event_index'] for row in events]==list(range(1,9))
    assert [row['sequence'] for row in events]==[0,0,1,1,2,2,3,3]
    assert [row['stamp_ns'] for row in events if row['event']=='transaction_complete']==[r['stamp_ns'] for r in records]
    assert rows[-1]['event']=='journal_finalization' and rows[-1]['final_fsync_complete'] is False
    state=journal.status()
    assert state['pending_events']==state['pending_bytes']==0 and state['written_events']==8
    assert state['final_fsync_complete'] is True and state['worker_exited'] is True
    assert state['max_write_flush_ns']>0
    assert set(output.write_threads)==set(output.close_threads)=={'wheel-feedback-journal'}


@pytest.mark.parametrize('preloaded',[False,True])
def test_full_async_journal_latches_poller_and_never_retries(config,tmp_path,preloaded):
    output=ObservedOutput(tmp_path/'full.jsonl',block=True)
    journal=ft.AsyncJournal(output,max_events=1)
    calls=[]
    poller=ft.FeedbackPoller(config,SimpleNamespace(exchange=lambda query,timeout:calls.append(query) or response()),journal)
    if preloaded:
        journal({'event':'already_accepted'})
        assert output.entered.wait(1)
    try:
        with pytest.raises(FeedbackError,match='ASYNC_JOURNAL_QUEUE_FULL'):poller.sample()
        with pytest.raises(FeedbackError,match='failed stream'):poller.sample()
        assert calls==([] if preloaded else [ft.QUERY])
        assert poller.failed and journal.status()['rejected_events']==1
        assert journal.status()['pending_events']==1
    finally:
        output.release.set()
        with pytest.raises(FeedbackError,match='ASYNC_JOURNAL_QUEUE_FULL'):journal.close()
    assert journal.status()['written_events']==1 and journal.status()['final_fsync_complete'] is True
    assert output.closed


def test_async_journal_byte_bound_rejects_before_any_write(tmp_path):
    output=ObservedOutput(tmp_path/'byte_bound.jsonl')
    journal=ft.AsyncJournal(output,max_bytes=32)
    with pytest.raises(FeedbackError,match='ASYNC_JOURNAL_QUEUE_FULL'):
        journal({'event':'does_not_fit'})
    with pytest.raises(FeedbackError,match='ASYNC_JOURNAL_QUEUE_FULL'):journal.close()
    assert journal.status()['enqueued_events']==journal.status()['written_events']==0
    assert output.closed


def test_async_write_failure_is_seen_before_the_next_serial_exchange(config,tmp_path):
    output=ObservedOutput(tmp_path/'failed.jsonl',fail=True)
    journal=ft.AsyncJournal(output)
    journal({'event':'lease_open'})
    wait_until(lambda:journal.status()['error'] is not None)
    calls=[]
    poller=ft.FeedbackPoller(config,SimpleNamespace(exchange=lambda *args:calls.append(args)),journal)
    with pytest.raises(FeedbackError,match='ASYNC_JOURNAL_WRITE_FAILED'):poller.sample()
    assert calls==[] and poller.failed
    with pytest.raises(FeedbackError,match='ASYNC_JOURNAL_WRITE_FAILED'):journal.close()
    state=journal.status()
    assert state['pending_events']==1 and state['written_events']==0 and state['final_fsync_complete'] is False
    assert state['max_write_flush_ns']>0 and output.closed


def test_async_close_timeout_never_closes_the_workers_active_file(tmp_path):
    output=ObservedOutput(tmp_path/'timeout.jsonl',block=True)
    journal=ft.AsyncJournal(output,close_timeout_s=.01)
    journal({'event':'accepted'})
    try:
        assert output.entered.wait(1)
        with pytest.raises(FeedbackError,match='ASYNC_JOURNAL_CLOSE_TIMEOUT'):journal.close()
        assert not output.closed and output.close_threads==[] and journal.thread.is_alive()
        assert journal.status()['pending_events']==1 and journal.status()['final_fsync_complete'] is False
    finally:
        output.release.set();journal.thread.join(2)
    assert output.closed and output.close_threads==['wheel-feedback-journal']
    with pytest.raises(FeedbackError,match='ASYNC_JOURNAL_CLOSE_TIMEOUT'):journal.close()


def test_normal_async_close_fsyncs_on_writer_and_preserves_enqueued_snapshot(tmp_path,monkeypatch):
    output=ObservedOutput(tmp_path/'sync.jsonl',block=True)
    calls=[];original=ft.os.fsync
    def synced(fd):
        calls.append((fd,threading.current_thread().name));return original(fd)
    monkeypatch.setattr(ft.os,'fsync',synced)
    journal=ft.AsyncJournal(output)
    record={'event':'original','source_stamp_ns':123,'nested':{'raw':[1,2]}}
    journal(record);record['source_stamp_ns']=999;record['nested']['raw'].append(3)
    output.release.set();journal.close()
    first=json.loads(output.path.read_text().splitlines()[0])
    assert first['source_stamp_ns']==123 and first['nested']['raw']==[1,2]
    assert len(calls)==1 and calls[0][1]=='wheel-feedback-journal'
    assert journal.status()['final_fsync_complete'] is True and output.closed


def test_batching_limits_bytes_and_events_and_flushes_pending_prefix_in_fifo(tmp_path):
    output=ObservedOutput(tmp_path/'batched.jsonl',block=True)
    journal=ft.AsyncJournal(output)
    originals=[]
    try:
        journal({'event':'first'}); assert output.entered.wait(1)
        for index in range(130):
            record={'event':'raw', 'source_stamp_ns':123_000_000+index, 'sequence':index,
                    'payload':'x'*5000, 'request_hex':ft.QUERY.hex(), 'response_hex':response().hex()}
            originals.append(copy.deepcopy(record)); journal(record)
            record['source_stamp_ns']=0
        state=journal.status()
        assert state['pending_events']==131 and state['written_events']==0
        assert state['pending_bytes']<ft.JOURNAL_PENDING_BYTES
    finally:
        output.release.set(); journal.close()
    rows=[json.loads(line) for line in output.path.read_text().splitlines()]
    for actual, expected in zip(rows[1:-1],originals):
        assert all(actual[key]==value for key,value in expected.items())
    assert [row['journal_event_index'] for row in rows[:-1]]==list(range(1,132))
    state=journal.status()
    assert state['pending_events']==state['pending_bytes']==0
    assert state['written_events']==state['enqueued_events']==131 and state['final_fsync_complete']
    assert 1 < state['max_event_batch_events'] <= 64
    assert state['max_event_batch_bytes'] <= 65536
    assert state['completed_event_batches']<25  # 131 original records were not flushed one by one.
    assert state['write_flush_count']==state['completed_event_batches']+1


def test_larger_event_capacity_still_enforces_byte_budget_including_inflight_batch(tmp_path):
    output=ObservedOutput(tmp_path/'bytes_still_bounded.jsonl',block=True)
    journal=ft.AsyncJournal(output,max_bytes=2048)
    accepted=0
    try:
        journal({'event':'first'}); accepted+=1; assert output.entered.wait(1)
        with pytest.raises(FeedbackError,match='ASYNC_JOURNAL_QUEUE_FULL'):
            for index in range(ft.JOURNAL_PENDING_EVENTS):
                journal({'event':'raw', 'sequence':index, 'payload':'x'*512}); accepted+=1
        state=journal.status()
        assert state['max_pending_events']==1024
        assert state['pending_events']==accepted < 10
        assert state['pending_bytes']<=2048 and state['written_events']==0
    finally:
        output.release.set()
        with pytest.raises(FeedbackError,match='ASYNC_JOURNAL_QUEUE_FULL'): journal.close()
    assert journal.status()['written_events']==accepted and journal.status()['final_fsync_complete']


def test_short_batch_write_cannot_declare_events_written_or_finally_synced(tmp_path):
    output=ObservedOutput(tmp_path/'short_write.jsonl',block=True)
    original=output.write
    def short(line):
        original(line[:len(line)//2])
        return len(line)//2
    output.write=short
    journal=ft.AsyncJournal(output)
    try:
        for index in range(4): journal({'event':'accepted', 'sequence':index})
        assert output.entered.wait(1)
    finally:
        output.release.set()
        with pytest.raises(FeedbackError,match='journal short text write'): journal.close()
    state=journal.status()
    assert state['written_events']==0 and state['pending_events']==4
    assert not state['final_fsync_complete'] and state['worker_exited'] and output.closed


def mock_main_lease(monkeypatch):
    calls=[]
    class Lease:
        host_configuration={'fixture':True}
        def __init__(self,*args):pass
        def open(self):calls.append('open')
        def exchange(self,query,timeout):
            assert query==ft.QUERY and timeout==.2
            calls.append('query');return response()
        def close(self):calls.append('close')
    monkeypatch.setattr(ft,'FeedbackSerialLease',Lease)
    monkeypatch.setattr(ft.signal,'signal',lambda *args:None)
    return calls


@pytest.mark.parametrize('asynchronous',[False,True])
def test_cli_async_journal_requires_flag_and_default_remains_synchronous(tmp_path,monkeypatch,capsys,asynchronous):
    calls=mock_main_lease(monkeypatch)
    output=tmp_path/'capture.jsonl'
    args=['--config',str(CONFIG_PATH),'--output',str(output),'--run-root',str(tmp_path),
          '--allow-read-queries','--samples','2','--rate-hz','50','--max-duration','1']
    if asynchronous:args.append('--async-journal')
    assert ft.main(args)==0 and calls==['open','query','query','close']
    summary=json.loads(capsys.readouterr().out.splitlines()[-1])
    assert summary['journal']['mode']==('async' if asynchronous else 'synchronous')
    assert summary['journal']['final_fsync_complete'] is True
    rows=[json.loads(line) for line in output.read_text().splitlines()]
    assert [row['event'] for row in rows]==['lease_open','request_planned','transaction_complete',
        'request_planned','transaction_complete','capture_complete','lease_closed']+(['journal_finalization'] if asynchronous else [])


@pytest.mark.parametrize('failure', [False, True])
def test_until_stopped_feedback_keeps_fc03_after_old_limit_and_drains_on_stop(tmp_path, monkeypatch, capsys, failure):
    handlers, calls, clock = {}, [], [0.]
    class Lease:
        host_configuration = {'fixture': 'NO_DEVICE'}
        def __init__(self, *args): pass
        def open(self): calls.append('open')
        def exchange(self, query, timeout):
            assert query == ft.QUERY and query[1] == 3 and timeout == .2
            calls.append('query')
            if calls.count('query') == 3:
                if failure: raise ft.QueryFailure('synthetic read failure', transmitted_bytes=8)
                handlers[ft.signal.SIGINT](ft.signal.SIGINT, None)
            return response()
        def close(self): calls.append('close')
    monkeypatch.setattr(ft, 'FeedbackSerialLease', Lease)
    monkeypatch.setattr(ft.signal, 'signal', lambda number, handler: handlers.__setitem__(number, handler))
    monkeypatch.setattr(ft.time, 'monotonic', lambda: clock[0])
    monkeypatch.setattr(ft.time, 'sleep', lambda seconds: clock.__setitem__(0, clock[0]+50000))
    output = tmp_path/'until_stopped.jsonl'
    code = ft.main(['--config', str(CONFIG_PATH), '--output', str(output), '--run-root', str(tmp_path),
                    '--samples', '0', '--max-duration', '0', '--allow-read-queries', '--async-journal'])
    assert code == int(failure) and calls == ['open', 'query', 'query', 'query', 'close']
    final = json.loads(capsys.readouterr().out.splitlines()[-1])
    assert final['journal']['final_fsync_complete'] and final['control_transmissions'] == 0
    rows = [json.loads(line) for line in output.read_text().splitlines()]
    assert all(row.get('control_transmissions', 0) == 0 for row in rows)
    if not failure:
        complete = next(row for row in rows if row['event'] == 'capture_complete')
        assert complete['completed'] == 3 and complete['requested'] == 0 and complete['until_stopped']


@pytest.mark.parametrize('samples,duration', [('0','1'), ('1','0'), ('0','nan'), ('0','-1')])
def test_only_paired_explicit_zeros_select_unbounded_feedback(tmp_path, monkeypatch, samples, duration):
    calls = mock_main_lease(monkeypatch)
    with pytest.raises(FeedbackError):
        ft.main(['--config', str(CONFIG_PATH), '--output', str(tmp_path/'invalid.jsonl'), '--run-root', str(tmp_path),
                 '--samples', samples, '--max-duration', duration, '--allow-read-queries'])
    assert not calls and not (tmp_path/'invalid.jsonl').exists()


@pytest.mark.parametrize('fault',['write','close_timeout','fsync'])
def test_main_returns_nonzero_for_async_journal_failure_and_releases_serial_first(tmp_path,monkeypatch,capsys,fault):
    calls=mock_main_lease(monkeypatch)
    output=ObservedOutput(tmp_path/'bad_capture.jsonl',block=fault=='close_timeout',fail=fault=='write')
    journal=ft.AsyncJournal(output,close_timeout_s=.01 if fault=='close_timeout' else 1.)
    def make_journal(*args, close_timeout_s):
        assert close_timeout_s == ft.JOURNAL_CLOSE_TIMEOUT_S
        return journal
    monkeypatch.setattr(ft,'create_journal',make_journal)
    if fault=='fsync':
        def fail_sync(fd):raise OSError('simulated fsync failure')
        monkeypatch.setattr(ft.os,'fsync',fail_sync)
    try:
        result=ft.main(['--config',str(CONFIG_PATH),'--output',str(output.path),'--run-root',str(tmp_path),
            '--allow-read-queries','--async-journal','--samples','1','--rate-hz','50','--max-duration','1'])
        assert result==1 and calls[-1]=='close'
        summary=json.loads(capsys.readouterr().out.splitlines()[-1])
        assert summary['state']=='FAILED' and summary['journal']['error']
        assert summary['journal']['final_fsync_complete'] is False
        if fault=='close_timeout':assert not output.closed and journal.thread.is_alive()
    finally:
        output.release.set();journal.thread.join(2)
    assert output.closed and output.close_threads==['wheel-feedback-journal']


@pytest.mark.parametrize('ack_mode',['pass','false','throw'])
def test_read_only_capture_dds_ack_follows_lease_close_and_failure_preserves_failed_source(tmp_path,monkeypatch,capsys,ack_mode):
    from types import ModuleType
    calls=mock_main_lease(monkeypatch)
    class Publisher:
        def get_subscription_count(self):return 1
        def publish(self,message):
            assert json.loads(message.data)['status']=='RESPONSE_VALID'
            calls.append('publish')
        def wait_for_all_acked(self,timeout):
            calls.append('recording_ack');assert timeout.seconds==5.0
            if ack_mode=='throw':raise RuntimeError('synthetic DDS ACK exception')
            return ack_mode=='pass'
    class Node:
        def create_publisher(self,message_type,topic,qos):
            assert topic=='/wc_mapping/wheel/feedback_raw' and qos.reliability=='RELIABLE'
            return Publisher()
        def destroy_node(self):calls.append('node_destroy')
    ros=ModuleType('rclpy');ros.init=lambda **kwargs:None
    ros.create_node=lambda name:Node();ros.spin_once=lambda *args,**kwargs:None
    ros.ok=lambda:True;ros.shutdown=lambda:calls.append('ros_shutdown')
    qos=ModuleType('rclpy.qos');qos.QoSProfile=lambda **kwargs:SimpleNamespace(**kwargs)
    qos.ReliabilityPolicy=SimpleNamespace(RELIABLE='RELIABLE')
    signals=ModuleType('rclpy.signals');signals.SignalHandlerOptions=SimpleNamespace(NO=0)
    duration=ModuleType('rclpy.duration');duration.Duration=lambda **kwargs:SimpleNamespace(**kwargs)
    messages=ModuleType('std_msgs.msg');messages.String=lambda **kwargs:SimpleNamespace(**kwargs)
    for name,module in [('rclpy',ros),('rclpy.qos',qos),('rclpy.signals',signals),('rclpy.duration',duration),
                        ('std_msgs',ModuleType('std_msgs')),('std_msgs.msg',messages)]:
        monkeypatch.setitem(sys.modules,name,module)
    original_create_journal=ft.create_journal
    def create_journal(*args,**kwargs):
        journal=original_create_journal(*args,**kwargs);original_close=journal.close
        def close():calls.append('journal_close');return original_close()
        journal.close=close
        return journal
    monkeypatch.setattr(ft,'create_journal',create_journal)
    raw=tmp_path/'capture.jsonl';summary_path=tmp_path/'summary.json'
    code=ft.main(['--config',str(CONFIG_PATH),'--output',str(raw),'--summary-path',str(summary_path),
        '--run-root',str(tmp_path),'--allow-read-queries','--async-journal','--samples','1',
        '--rate-hz','50','--max-duration','1','--publish-ros','--require-recorder'])
    assert code==(0 if ack_mode=='pass' else 1)
    assert calls.index('close')<calls.index('recording_ack')<calls.index('journal_close')<calls.index('node_destroy')<calls.index('ros_shutdown')
    summary=json.loads(summary_path.read_text(encoding='utf-8'))
    assert summary['state']==('RAW_FEEDBACK_CAPTURED' if ack_mode=='pass' else 'FAILED')
    assert summary['closed_normally'] is (ack_mode=='pass') and summary['synchronized'] is (ack_mode=='pass')
    assert summary['journal']['final_fsync_complete'] and summary['completed']==1 and summary['control_transmissions']==0
    assert summary['events_sha256']==__import__('hashlib').sha256(raw.read_bytes()).hexdigest()
    rows=[json.loads(line) for line in raw.read_text(encoding='utf-8').splitlines()]
    ack_rows=[row for row in rows if row.get('event')=='publication_ack']
    if ack_mode=='throw':assert ack_rows==[]
    else:
        assert len(ack_rows)==1 and ack_rows[0]['acknowledged'] is (ack_mode=='pass')
        assert rows.index(next(row for row in rows if row['event']=='lease_closed'))<rows.index(ack_rows[0])
    if ack_mode!='pass':
        expected='acknowledgement timeout' if ack_mode=='false' else 'synthetic DDS ACK exception'
        assert expected in capsys.readouterr().err
