"""Bounded monitor state, saved evidence and loopback HTTP; no ROS or hardware."""
import hashlib
import http.client
import json
from pathlib import Path
import sys
import threading

import numpy as np
import pytest

from wc_runtime.single_mapping_monitor import (MappingState, MonitorServer, OfflineState,
    MAX_DISPLAY_POINTS, MAX_RAW_POINTS, main)


class Clock:
    now = 0.
    def __call__(self): return self.now
    def wall(self): return 1700000000000000000 + int(self.now*1000000000)


@pytest.fixture
def state(tmp_path):
    clock = Clock()
    value = MappingState('SYN_TEST.01', 'right', tmp_path/'monitor', clock=clock, wall_clock=clock.wall)
    yield value, clock
    value.close()


def odom(state, stamp, position=(1., 2., 3.), quaternion=(0., 0., 0., 1.)):
    state.receive_odom('single_right_odom', 'lidar_right', stamp, position, quaternion)


def info(state, stamp, lost=False):
    state.receive_info('single_right_odom', stamp, lost, {'inliers': 80})


def cloud(state, stamp=1):
    state.receive_map('single_right_map', stamp, np.array([[1., 2., 3.], [4., 5., 6.]]))


def test_unknown_is_explicit_and_no_existing_directory_is_reused(state):
    value, _ = state
    status = value.status_snapshot()
    assert status['status'] == 'UNKNOWN'
    assert status['tracking_lost'] is None
    assert status['age_s'] == {'odom': None, 'odom_info': None, 'cloud_map': None}
    assert not json.loads(value.map_bytes())['available']
    assert status['calibration_validated'] is False and status['navigation_validated'] is False
    before = (value.output/'status.json').read_bytes()
    with pytest.raises(FileExistsError):
        MappingState('OTHER', 'right', value.output)
    assert (value.output/'status.json').read_bytes() == before


def test_same_stamp_tracking_confirmation_handles_both_callback_orders(state):
    value, _ = state
    odom(value, 1)
    assert value.status_snapshot()['trajectory'] == []
    info(value, 1)
    assert len(value.status_snapshot()['trajectory']) == 1
    info(value, 2)
    odom(value, 2)
    cloud(value)
    assert value.status_snapshot()['status'] == 'RUNNING'
    assert [p['source_stamp_ns'] for p in value.status_snapshot()['trajectory']] == [1, 2]
    copy = value.status_snapshot()
    copy['trajectory'][0]['xyz'][0] = 999
    assert value.status_snapshot()['trajectory'][0]['xyz'][0] == 1.


def test_single_lost_null_pose_is_recorded_and_recovery_starts_new_segment(state):
    value, _ = state
    info(value, 1); odom(value, 1)
    odom(value, 2, quaternion=(0., 0., 0., 0.))
    info(value, 2, True)
    assert value.failure is None
    assert value.status_snapshot()['status'] == 'TRACKING_LOST'
    assert value.status_snapshot()['odometry']['pose_valid'] is False
    assert len(value.status_snapshot()['trajectory']) == 1
    odom(value, 3); info(value, 3)
    assert [p['segment'] for p in value.status_snapshot()['trajectory']] == [0, 1]
    events = [json.loads(line) for line in (value.output/'events.jsonl').read_text().splitlines()]
    assert [row['event'] for row in events] == ['ODOMETRY_LOST', 'ODOMETRY_RECOVERED_NEW_SEGMENT']
    assert len((value.output/'odometry.jsonl').read_text().splitlines()) == 3
    assert len((value.output/'trajectory.jsonl').read_text().splitlines()) == 2


def test_five_consecutive_losses_fail_without_reset_and_persist(state):
    value, _ = state
    for stamp in range(1, 5): info(value, stamp, True)
    assert value.failure is None
    info(value, 5, True)
    status = json.loads((value.output/'status.json').read_text())
    assert status['status'] == 'FAILED'
    assert status['failure']['code'] == 'ODOMETRY_LOST_FIVE_CONSECUTIVE'
    assert not value.tick()
    info(value, 6, False)
    assert value.status_snapshot()['consecutive_lost'] == 5


def test_recovery_resets_consecutive_counter_without_erasing_total(state):
    value, _ = state
    for stamp in range(1, 5): info(value, stamp, True)
    info(value, 5, False)
    for stamp in range(6, 10): info(value, stamp, True)
    assert value.failure is None
    assert value.status_snapshot()['consecutive_lost'] == 4
    assert value.status_snapshot()['total_lost'] == 8


@pytest.mark.parametrize('stream', ['odom', 'odom_info'])
def test_first_received_stream_times_out_after_three_seconds(state, stream):
    value, clock = state
    (odom if stream == 'odom' else info)(value, 1)
    clock.now = 3.
    assert value.tick()
    clock.now = 3.01
    assert not value.tick()
    assert value.failure['code'] == 'STREAM_TIMEOUT'
    assert stream in value.failure['detail']


def test_initial_missing_stream_fails_after_thirty_seconds(state):
    value, clock = state
    clock.now = 30.
    assert value.tick()
    clock.now = 30.01
    assert not value.tick()
    assert value.failure['code'] == 'INITIAL_STREAM_TIMEOUT'


def test_static_unchanged_map_does_not_timeout_when_tracking_remains_fresh(state):
    value, clock = state
    cloud(value)
    for stamp in range(1, 35):
        clock.now = float(stamp)
        info(value, stamp); odom(value, stamp)
        assert value.tick()
    status = value.status_snapshot()
    assert status['status'] == 'RUNNING' and status['cloud_map_stale'] is True
    assert status['counts']['cloud_map'] == 1


def test_raw_map_rows_preserved_and_display_sampling_is_bounded(state):
    value, _ = state
    xyz = np.arange(180009, dtype=float).reshape(-1, 3)
    xyz[5, 0] = np.nan
    original = xyz.copy()
    value.receive_map('single_right_map', 7, xyz)
    xyz[:] = 0
    with np.load(value.output/'latest_map.npz', allow_pickle=False) as saved:
        assert np.array_equal(saved['xyz'], original, equal_nan=True)
        assert saved['session_id'].item() == 'SYN_TEST.01'
        assert saved['side'].item() == 'right'
    preview = json.loads(value.map_bytes())
    assert preview['raw_point_count'] == len(original)
    assert preview['finite_point_count'] == len(original)-1
    assert preview['display_point_count'] == MAX_DISPLAY_POINTS
    assert preview['points'][0] == original[0].tolist()
    assert preview['points'][-1] == original[-1].tolist()
    assert np.isfinite(preview['points']).all()


def test_bad_map_or_pose_reference_and_nonunit_quaternion_are_rejected(state):
    value, _ = state
    with pytest.raises(ValueError): value.receive_map('map', 1, np.ones((3, 3)))
    with pytest.raises(ValueError): value.receive_odom('odom', 'lidar_right', 1, [0, 0, 0], [0, 0, 0, 1])
    with pytest.raises(ValueError): value.receive_info('lidar_right', 1, False)
    with pytest.raises(ValueError): odom(value, 1, quaternion=(0, 0, 0, .5))
    with pytest.raises(ValueError): odom(value, 1, position=(float('nan'), 0, 0))
    with pytest.raises(ValueError): value.receive_map('single_right_map', 1, np.broadcast_to(np.zeros(3), (MAX_RAW_POINTS+1, 3)))
    assert value.counts == {'odom': 0, 'odom_info': 0, 'cloud_map': 0}


def test_backward_source_time_does_not_connect_new_trajectory(state):
    value, _ = state
    info(value, 9); odom(value, 9)
    with pytest.raises(ValueError): odom(value, 8)
    assert len(value.status_snapshot()['trajectory']) == 1


def test_pending_tracking_and_trajectory_snapshot_memory_are_bounded(state):
    value, _ = state
    for stamp in range(1, 2200):
        odom(value, stamp)
        info(value, stamp)
    assert len(value.trajectory) == 2000
    assert len(value.tracking_by_stamp) == 128
    assert len(value.pending_poses) == 0
    for stamp in range(2200, 2400): odom(value, stamp)
    assert len(value.pending_poses) == 128


def fingerprints(root):
    return {p.name: (p.stat().st_mtime_ns, hashlib.sha256(p.read_bytes()).hexdigest()) for p in root.iterdir() if p.is_file()}


def test_offline_load_is_read_only_identified_and_never_imports_ros(state):
    value, _ = state
    info(value, 1); odom(value, 1); cloud(value); value.close()
    before = fingerprints(value.output)
    ros_before = {name for name in sys.modules if name.startswith(('rclpy', 'nav_msgs', 'rtabmap_msgs'))}
    offline = OfflineState(value.output)
    status = offline.status_snapshot()
    assert status['offline'] is True and status['status'] == 'STOPPED'
    assert status['session_id'] == value.session_id
    assert json.loads(offline.map_bytes())['points'] == [[1., 2., 3.], [4., 5., 6.]]
    assert fingerprints(value.output) == before
    assert {name for name in sys.modules if name.startswith(('rclpy', 'nav_msgs', 'rtabmap_msgs'))} == ros_before


@pytest.mark.parametrize('fault', ['session', 'side', 'source', 'navigation', 'missing_map'])
def test_offline_refuses_misidentified_or_missing_saved_data(state, fault):
    value, _ = state
    cloud(value); value.close()
    path = value.output/'status.json'
    status = json.loads(path.read_text())
    if fault == 'session': status['session_id'] = 'OTHER'
    elif fault == 'side': status['side'] = 'left'
    elif fault == 'source': status['source_mode'] = 'synthetic'
    elif fault == 'navigation': status['navigation_validated'] = True
    elif fault == 'missing_map': (value.output/'latest_map.npz').rename(value.output/'saved_elsewhere.npz')
    path.write_text(json.dumps(status), encoding='utf-8')
    with pytest.raises(ValueError): OfflineState(value.output)


def test_http_is_loopback_readonly_and_unknown_paths_do_not_read_files(state):
    value, _ = state
    server = MonitorServer(value, 0)
    assert server.server_address[0] == '127.0.0.1'
    thread = threading.Thread(target=server.serve_forever, kwargs={'poll_interval': .01}, daemon=True)
    thread.start()
    try:
        for method, path, host, expected in [('GET', '/api/status', None, 200), ('GET', '/api/map', None, 200),
                ('GET', '/', None, 200), ('GET', '/viewer.js', None, 200), ('GET', '/viewer.css', None, 200),
                ('GET', '/status.json', None, 404), ('GET', '/../status.json', None, 404),
                ('GET', '/api/status', 'example.org', 403), ('POST', '/api/status', None, 405)]:
            connection = http.client.HTTPConnection('127.0.0.1', server.server_port, timeout=3)
            connection.request(method, path, headers={} if host is None else {'Host': host})
            response = connection.getresponse()
            assert response.status == expected
            assert response.getheader('Cache-Control') == 'no-store'
            response.read(); connection.close()
    finally:
        server.shutdown(); server.server_close(); thread.join(timeout=2)


@pytest.mark.parametrize('args', [[], ['--offline', 'saved', '--side', 'right'],
                                  ['--session-id', 'x', '--side', 'right'], ['--offline', 'saved', '--port', '0']])
def test_cli_requires_separate_complete_live_or_offline_arguments(args):
    with pytest.raises(SystemExit): main(args)
