"""Synthetic slow-disk/health tests; no ROS node, hardware, or real maps."""
import json
import struct
import threading
from pathlib import Path
import shutil
import uuid
from types import SimpleNamespace as NS

import numpy as np
import pytest

from wc_runtime import mapping_monitor as monitor


@pytest.fixture
def tmp_path():
    parent = Path(__file__).absolute().parent
    directory = parent/('.mapping_monitor_'+uuid.uuid4().hex)
    directory.mkdir()
    try: yield directory
    finally:
        assert directory.resolve().parent == parent and directory.name.startswith('.mapping_monitor_')
        shutil.rmtree(directory)


def info(sequence=0, *, lost=False):
    return NS(header=NS(frame_id='mapping_odom', stamp=NS(sec=1, nanosec=sequence)), lost=lost,
              guess=NS(translation=NS(x=0., y=0., z=0.), rotation=NS(x=0., y=0., z=0., w=1.)),
              icp_inliers_ratio=.8)


def cloud(value=1.):
    return NS(header=NS(frame_id='mapping_map'), height=1, width=2, point_step=12, row_step=24,
              is_bigendian=False, data=struct.pack('<6f', value, 2, 3, 4, 5, 6),
              fields=[NS(name=name, offset=i*4, datatype=7, count=1) for i, name in enumerate(('x','y','z'))])


def grid(value=100):
    return NS(header=NS(frame_id='mapping_map'), data=[-1, 0, value, 0, 0, -1],
              info=NS(width=3, height=2, resolution=.03,
                      origin=NS(position=NS(x=0., y=0., z=0.), orientation=NS(x=0., y=0., z=0., w=1.))))


def test_wheel_imu_mapping_requires_real_outputs_without_icp_and_keeps_freshness_gate(tmp_path):
    now=[0.]
    state=monitor.Health(tmp_path/'health','wheel_fixture','right',clock=lambda:now[0],odometry_source='wheel_imu')
    try:
        now[0]=31.
        for key in ('odom','prior_odom'):state.receive(key)
        state.receive_cloud(cloud());state.receive_grid(grid())
        assert state.tick()
        status=state.snapshot()
        assert status['odometry_source']=='wheel_imu' and not status['icp_tracking_required']
        assert status['consecutive_lost'] is None and status['total_lost'] is None
        assert 'odom_info' not in status['counts']
        with pytest.raises(ValueError,match='not part'):state.receive_info(info(lost=True))
        now[0]+=3.01
        assert not state.tick() and '未更新' in state.failure
    finally:state.close(timeout_s=3)


@pytest.mark.parametrize('missing',['odom','prior_odom','cloud_map','grid_map'])
def test_wheel_imu_missing_output_still_prevents_mapping_success(tmp_path,missing):
    now=[0.]
    state=monitor.Health(tmp_path/'health','wheel_fixture','left',clock=lambda:now[0],odometry_source='wheel_imu')
    try:
        now[0]=31.
        for key in monitor.required_outputs('wheel_imu'):
            if key!=missing:state.receive(key)
        assert not state.tick() and missing in state.failure
    finally:state.close(timeout_s=3)


def test_icp_mode_still_fails_after_five_lost_frames(tmp_path):
    state=monitor.Health(tmp_path/'health','icp_fixture','right',odometry_source='icp')
    try:
        for i in range(4):state.receive_info(info(i,lost=True))
        assert state.failure is None
        state.receive_info(info(4,lost=True))
        assert state.failure=='ICP 连续五帧失去跟踪'
    finally:state.close(timeout_s=3)


def test_continuous_mapping_waits_through_long_silence_then_accepts_outputs(tmp_path):
    now = [0.]
    state = monitor.Health(tmp_path/'health', 'continuous_fixture', 'right',
                           clock=lambda:now[0], odometry_source='wheel_imu', continuous_mapping=True)
    try:
        now[0] = 12*3600.
        assert state.tick() and state.failure is None
        assert set(state.snapshot()['waiting_outputs']) == set(monitor.required_outputs('wheel_imu'))
        for key in ('odom','prior_odom'): state.receive(key)
        state.receive_cloud(cloud()); state.receive_grid(grid())
        assert state.tick() and state.snapshot()['waiting_outputs'] == []
        now[0] += 24*3600.
        assert state.tick() and state.failure is None
        for key in ('odom','prior_odom'): state.receive(key)
        state.receive_cloud(cloud(2.)); state.receive_grid(grid(50))
        assert state.tick()
        assert all(value == 2 for value in state.counts.values())
        assert state.snapshot()['data_freshness_shutdown'] is False
    finally: state.close(timeout_s=3)
    saved = json.loads((state.root/'status.json').read_text())
    assert saved['status'] == 'STOPPED' and saved['failure'] is None


def test_blocked_disk_keeps_health_callbacks_current_and_only_latest_snapshots(tmp_path):
    entered, release = threading.Event(), threading.Event()
    now = [0.]
    def blocked(root, kind, value, **kwargs):
        if kind == 'status' and not entered.is_set():
            entered.set()
            assert release.wait(5)
        monitor._write_snapshot(root, kind, value, **kwargs)
    state = monitor.Health(tmp_path/'health', 'synthetic', 'all', clock=lambda: now[0],
                           archive_factory=lambda root: monitor.AsyncHealthArchive(root, write_snapshot=blocked))
    try:
        assert entered.wait(2)
        for sequence in range(25):
            now[0] = .2*(sequence+1)
            state.receive('odom');state.receive('prior_odom')
            state.receive_info(info(sequence))
            state.receive_cloud(cloud(float(sequence)))
            state.receive_grid(grid(sequence))
            assert state.tick()
        assert now[0] == 5.  # elapsed health time exceeds unchanged 3 s freshness gate
        assert state.counts == {key:25 for key in ('odom','prior_odom','odom_info','cloud_map','grid_map')}
        assert state.last == {key:5. for key in state.counts}
        storage = state.archive.storage()
        assert storage['pending_tracking'] == 25
        assert storage['pending_snapshots'] == ['cloud','grid','status']
        assert storage['write_in_flight'] == 'status'
        assert state.failure is None
    finally:
        release.set()
        state.close(timeout_s=3)
    saved = json.loads((state.root/'status.json').read_text(encoding='utf-8'))
    assert saved['status'] == 'STOPPED' and saved['failure'] is None
    assert saved['storage']['final'] is True
    assert saved['storage']['tracking_written'] == saved['storage']['tracking_accepted'] == 25
    assert saved['storage']['pending_tracking'] == 0 and saved['storage']['pending_snapshots'] == []
    assert saved['storage']['snapshot_generations_written']['cloud'] == 25
    assert saved['storage']['snapshot_generations_written']['grid'] == 25
    rows = [json.loads(row) for row in (state.root/'tracking.jsonl').read_text().splitlines()]
    assert [row['stamp_ns'] for row in rows] == [1000000000+i for i in range(25)]
    with np.load(state.root/'latest_cloud.npz', allow_pickle=False) as data:
        assert data['xyz'][0,0] == 24
    with np.load(state.root/'latest_grid.npz', allow_pickle=False) as data:
        assert data['data'][0,2] == 24


def test_async_snapshot_owns_copies_and_tracking_overflow_is_explicit(tmp_path):
    entered, release = threading.Event(), threading.Event()
    def blocked(root, kind, value, **kwargs):
        entered.set()
        assert release.wait(5)
        monitor._write_snapshot(root, kind, value, **kwargs)
    archive = monitor.AsyncHealthArchive(tmp_path, max_tracking=2, write_snapshot=blocked)
    try:
        archive.snapshot('status', {'status':'RUNNING'})
        assert entered.wait(2)
        xyz = np.array([[1.,2.,3.]])
        archive.snapshot('cloud', xyz); xyz[:]=999
        record = {'stamp_ns':1, 'value':[2]}
        archive.record(record); record['value'][0]=999
        archive.record({'stamp_ns':2})
        with pytest.raises(RuntimeError, match='FIFO full'):
            archive.record({'stamp_ns':3})
        assert archive.storage()['pending_tracking'] == 2
    finally:
        release.set()
        archive.close({'status':'FAILED','failure':'synthetic overflow'},timeout_s=3)
    with np.load(tmp_path/'latest_cloud.npz',allow_pickle=False) as saved:
        assert np.array_equal(saved['xyz'], [[1.,2.,3.]])
    records = [json.loads(row) for row in (tmp_path/'tracking.jsonl').read_text().splitlines()]
    assert records == [{'stamp_ns':1,'value':[2]}, {'stamp_ns':2}]


def test_writer_failure_surfaces_to_tick_and_close(tmp_path):
    entered, release = threading.Event(), threading.Event()
    def fail(*_, **__):
        entered.set()
        assert release.wait(5)
        raise OSError('synthetic eMMC error')
    state = monitor.Health(tmp_path/'health', 'synthetic', 'left',
                           archive_factory=lambda root: monitor.AsyncHealthArchive(root, write_snapshot=fail))
    try:
        assert entered.wait(2)
    finally:
        release.set()
    state.archive.thread.join(2)
    assert not state.archive.thread.is_alive()
    with pytest.raises(RuntimeError, match='synthetic eMMC error'):
        state.tick()
    with pytest.raises(RuntimeError, match='synthetic eMMC error'):
        state.close(timeout_s=2)
    assert 'synthetic eMMC error' in state.failure


def test_close_timeout_does_not_close_writer_descriptor_from_main_thread(tmp_path):
    entered, release = threading.Event(), threading.Event()
    def blocked(root, kind, value, **kwargs):
        entered.set()
        assert release.wait(5)
        monitor._write_snapshot(root,kind,value,**kwargs)
    archive = monitor.AsyncHealthArchive(tmp_path, write_snapshot=blocked)
    try:
        archive.snapshot('status',{'status':'RUNNING'})
        assert entered.wait(2)
        archive.record({'stamp_ns':42})
        with pytest.raises(RuntimeError,match='persistence incomplete'):
            archive.close({'status':'STOPPED'},timeout_s=.01)
        assert archive.thread.is_alive() and not archive.storage()['final']
    finally:
        release.set()
        archive.close({'status':'STOPPED'},timeout_s=3)
    # A parent-side premature close would have caused an I/O-on-closed-file
    # error here when the still-running writer resumed its queued tracking.
    assert json.loads((tmp_path/'tracking.jsonl').read_text()) == {'stamp_ns':42}
    assert archive.storage()['final']


def test_final_status_is_written_after_tracking_and_map_fsync(tmp_path,monkeypatch):
    events=[]
    actual_fsync=monitor.os.fsync
    def observe_fsync(fd):
        events.append(('fsync',threading.current_thread().name))
        return actual_fsync(fd)
    def observe_write(root,kind,value,**kwargs):
        if kwargs.get('durable'):
            events.append(('final_status',threading.current_thread().name))
        monitor._write_snapshot(root,kind,value,**kwargs)
    monkeypatch.setattr(monitor.os,'fsync',observe_fsync)
    archive=monitor.AsyncHealthArchive(tmp_path,write_snapshot=observe_write)
    archive.record({'stamp_ns':1})
    archive.snapshot('cloud',np.array([[1.,2.,3.]]))
    archive.snapshot('grid',(np.array([[0]],dtype=np.int16),{'resolution':.03}))
    archive.close({'status':'STOPPED'},timeout_s=3)
    final_index=next(i for i,event in enumerate(events) if event[0]=='final_status')
    assert sum(kind=='fsync' for kind,_ in events[:final_index]) == 3  # tracking, cloud, grid
    assert all(name=='mapping-health-archive' for _,name in events)


def test_final_fsync_failure_cannot_certify_stopped_archive(tmp_path,monkeypatch):
    def fail_sync(_):
        raise OSError('synthetic final fsync error')
    monkeypatch.setattr(monitor.os,'fsync',fail_sync)
    state=monitor.Health(tmp_path/'health','synthetic','left')
    state.receive_info(info())
    state.receive_cloud(cloud())
    with pytest.raises(RuntimeError,match='final fsync error'):
        state.close(timeout_s=3)
    assert not state.archive.thread.is_alive()
    assert state.failure and not state.archive.storage()['final']
    path=state.root/'status.json'
    if path.exists():
        saved=json.loads(path.read_text(encoding='utf-8'))
        assert saved['status']!='STOPPED' and saved['storage']['final'] is False


def test_health_three_second_and_thirty_second_gates_remain_unchanged(tmp_path):
    now=[0.]
    state=monitor.Health(tmp_path/'freshness','synthetic','left',clock=lambda:now[0])
    try:
        state.receive('odom');state.receive('odom_info');state.receive('prior_odom')
        now[0]=3.
        assert state.tick()
        now[0]=3.0001
        assert not state.tick()
        assert '超过 3 秒' in state.failure
    finally:state.close(timeout_s=3)
    now[0]=0.
    state=monitor.Health(tmp_path/'startup','synthetic','right',clock=lambda:now[0])
    try:
        now[0]=30.
        assert state.tick()
        now[0]=30.0001
        assert not state.tick()
        assert '等待真实建图输出超时' in state.failure
    finally:state.close(timeout_s=3)


def test_consecutive_five_lost_frames_gate_and_nonconsecutive_recovery(tmp_path):
    state=monitor.Health(tmp_path/'health','synthetic','all')
    try:
        for sequence in range(4):state.receive_info(info(sequence,lost=True))
        assert state.failure is None and state.lost==4
        state.receive_info(info(4,lost=False))
        assert state.lost==0 and state.total_lost==4
        for sequence in range(5,10):state.receive_info(info(sequence,lost=True))
        assert state.lost==5 and state.total_lost==9
        assert state.failure=='ICP 连续五帧失去跟踪'
    finally:state.close(timeout_s=3)
    saved=json.loads((state.root/'status.json').read_text(encoding='utf-8'))
    assert saved['status']=='FAILED' and saved['storage']['tracking_written']==10
