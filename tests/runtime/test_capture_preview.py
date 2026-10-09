"""Session isolation, bounded latest-frame preview and owned-window cleanup."""
import copy
import json
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace as NS

import pytest

from wc_runtime import capture_preview as preview


def write(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value), encoding='utf-8')


@pytest.fixture
def session(tmp_path):
    manifest = dict(session_id='capture.test-1', status='RECORDING', started_wall_ns=1_000_000_000, started_monotonic_ns=100)
    write(tmp_path/'capture_manifest.json', manifest)
    write(tmp_path/'recorder_ready.json', dict(ready=True, host_monotonic_ns=200))
    write(tmp_path/'configuration/source_selection.json', dict(selected_sources=['lidar_left', 'lidar_right']+
        ['camera_'+role for role in preview.ROLES]))
    write(tmp_path/'configuration/cameras.json', dict(cameras=[dict(role=role, port=port) for role, port in
        [('left_front', 1), ('right_front', 4), ('left_side', 2), ('right_side', 3)]]))
    for side in preview.SIDES:
        path = tmp_path/'configuration/lidar'/(side+'.yaml'); path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text('ros__parameters:\n  expected_serial: SN_'+side+'\n  frame_id: lidar_'+side+'\n', encoding='utf-8')
    return tmp_path


def plan(session):
    return preview.preview_plan(session, 'capture.test-1')


def stamp(ns):
    return NS(sec=ns//1_000_000_000, nanosec=ns%1_000_000_000)


def lidar_message(side='left', sequence=0, session_id='capture.test-1'):
    header = NS(frame_id='lidar_'+side, stamp=stamp(1_000_000_200+sequence))
    return NS(session_id=session_id, side=side, sensor_id='SN_'+side, header=header,
        cloud=NS(header=copy.deepcopy(header), height=1, width=2, point_step=16, row_step=32,
                 fields=[NS(name=name, offset=offset, datatype=7, count=1)
                         for name, offset in [('x', 0), ('y', 4), ('z', 8)]],
                 data=bytes(range(32)), is_bigendian=False, is_dense=False),
        host_receive_time=copy.deepcopy(header.stamp),
        host_monotonic_ns=300+sequence, stream_epoch='source_epoch', frame_sequence=sequence)


def camera_diagnostic(role, *, session_id='capture.test-1', state='ACTIVE'):
    values = dict(session_id=session_id, state=state, stream_epoch=session_id+'-epoch')
    return NS(status=[NS(name='/wc_mapping/cameras/'+role,
        values=[NS(key=key, value=json.dumps(value)) for key, value in values.items()])])


def test_preview_plan_is_native_session_scoped_and_uses_all_selected_cameras(session):
    config = plan(session)
    assert len(config['cameras']) == 4
    assert config['cloud_source'] == 'raw' and config['preview_hz'] == 3
    assert not config['hardware_started'] and not config['controls_enabled'] and not config['tf_published']
    for side in preview.SIDES:
        view = preview.rviz_configuration(config, side)
        manager = view['Visualization Manager']
        assert manager['Global Options']['Fixed Frame'] == 'lidar_'+side
        assert len(manager['Displays']) == 3
        assert manager['Displays'][0]['Decay Time'] == 0
        assert all(display['Topic']['Reliability Policy'] == 'Best Effort' for display in manager['Displays'])
        assert all(display['Topic']['Value'].startswith(config['namespace']+'/') for display in manager['Displays'])
        assert not any('RobotModel' in display['Class'] for display in manager['Displays'])


@pytest.mark.parametrize('name, mutation', [
    ('capture_manifest.json', lambda s: s.update(session_id='OTHER')),
    ('capture_manifest.json', lambda s: s.update(status='COMPLETE')),
    ('recorder_ready.json', lambda s: s.update(ready=False)),
    ('recorder_ready.json', lambda s: s.update(host_monotonic_ns=99)),
    ('recorder_ready.json', lambda s: s.update(session_id='OTHER')),
])
def test_preview_never_attaches_to_wrong_or_unready_capture(session, name, mutation):
    path = session/name; value = json.loads(path.read_text()); mutation(value); write(path, value)
    with pytest.raises(ValueError): plan(session)


def test_single_latest_cloud_retains_original_native_frame_stamp_and_rejects_other_sessions(session):
    cache = preview.PreviewCache(plan(session))
    first = lidar_message()
    assert cache.lidar('left', first, 500)
    assert not cache.lidar('left', first, 501)
    assert not cache.lidar('left', lidar_message(sequence=1, session_id='OTHER'), 502)
    for sequence in range(1, 101):
        assert cache.lidar('left', lidar_message(sequence=sequence), 503+sequence)
    assert len(cache.latest) == 1
    output = list(cache.pending(604))
    assert len(output) == 1 and output[0][1].header.frame_id == 'lidar_left'
    assert preview._stamp(output[0][1].header.stamp) == 1_000_000_300
    assert list(cache.pending(605)) == []
    assert cache.lidar('right', lidar_message(side='right'), 606)
    assert list(cache.pending(2_000_000_000)) == []


def test_camera_needs_current_session_diagnostics_and_never_republishes_stale_or_old_stamps(session):
    cache = preview.PreviewCache(plan(session)); role = 'left_front'
    image = NS(header=NS(frame_id='camera_usb3_1', stamp=stamp(1_000_000_500)))
    assert not cache.camera(role, image, 500)
    assert cache.camera_diagnostic(role, camera_diagnostic(role))
    assert cache.camera(role, image, 501)
    assert not cache.camera(role, image, 502)
    assert len(list(cache.pending(503))) == 1
    assert not cache.camera_diagnostic(role, camera_diagnostic(role, session_id='OTHER'))
    assert role not in cache.latest
    assert not cache.camera(role, image, 504)
    assert not cache.camera_diagnostic(role, camera_diagnostic(role, state='STALE'))


def test_lidar_disconnect_clears_display_once_and_new_sample_recovers_without_repeating_old_data(session):
    cache = preview.PreviewCache(plan(session))
    received = 10_000_000_000
    before = cache.source_status(received)
    assert all(row['status'] == 'WAITING' and row['age_ns'] is None
               and row['last_received_monotonic_ns'] is None for row in before.values())
    message = lidar_message()
    message.cloud.fields = [NS(name=name, offset=offset, datatype=7, count=1)
                            for name, offset in [('x', 0), ('y', 4), ('z', 8)]]
    message.cloud.height, message.cloud.width = 1, 2
    message.cloud.is_bigendian, message.cloud.is_dense = False, False
    message.cloud.point_step, message.cloud.row_step = 16, 32
    message.cloud.data = bytes(range(32))
    assert cache.lidar('left', message, received)
    assert cache.source_status(received)['lidar_left']['status'] == 'FRESH'
    assert list(cache.pending(received)) == [('left', message.cloud)]
    assert list(cache.pending(received+1)) == []
    assert list(cache.stale_lidar_clears(received+preview.PREVIEW_STALE_NS)) == []

    now = received+preview.PREVIEW_STALE_NS+1
    row = cache.source_status(now)['lidar_left']
    assert row['status'] == 'STALE' and row['last_received_monotonic_ns'] == received
    assert row['age_ns'] == preview.PREVIEW_STALE_NS+1
    assert list(cache.pending(now)) == []
    clears = list(cache.stale_lidar_clears(now))
    assert len(clears) == 1 and clears[0][0] == 'left'
    empty = clears[0][1]
    assert empty.header == message.cloud.header and empty.header is not message.cloud.header
    assert empty.fields == message.cloud.fields and empty.fields is not message.cloud.fields
    assert empty.width == 0 and empty.height == 1 and empty.row_step == 0
    assert empty.point_step == 16 and empty.data == b'' and empty.is_dense is True
    assert message.cloud.width == 2 and message.cloud.data == bytes(range(32))
    assert list(cache.stale_lidar_clears(now+1)) == []

    recovered = lidar_message(sequence=1)
    assert cache.lidar('left', recovered, now+2)
    fresh = cache.source_status(now+3)['lidar_left']
    assert fresh['status'] == 'FRESH' and fresh['last_received_monotonic_ns'] == now+2
    assert list(cache.pending(now+3)) == [('left', recovered.cloud)]
    assert list(cache.pending(now+4)) == []
    assert list(cache.stale_lidar_clears(now+4)) == []


def test_rejected_session_cannot_refresh_lidar_age_or_other_sources(session):
    cache = preview.PreviewCache(plan(session)); received = 10_000_000_000
    assert cache.lidar('left', lidar_message(), received)
    now = received+preview.PREVIEW_STALE_NS+1
    assert not cache.lidar('left', lidar_message(sequence=1, session_id='OTHER'), now)
    sources = cache.source_status(now)
    assert sources['lidar_left']['status'] == 'STALE'
    assert sources['lidar_left']['last_received_monotonic_ns'] == received
    assert all(row['status'] == 'WAITING' for name, row in sources.items() if name != 'lidar_left')


def test_camera_disconnect_and_inactive_diagnostics_report_stale_until_new_valid_image(session):
    cache = preview.PreviewCache(plan(session)); role = 'left_front'; received = 10_000_000_000
    image = NS(header=NS(frame_id='camera_usb3_1', stamp=stamp(1_000_000_500)))
    assert cache.camera_diagnostic(role, camera_diagnostic(role))
    assert cache.camera(role, image, received)
    assert cache.source_status(received)['camera_'+role]['status'] == 'FRESH'
    assert len(list(cache.pending(received))) == 1
    assert cache.source_status(received+preview.PREVIEW_STALE_NS+1)['camera_'+role]['status'] == 'STALE'
    assert list(cache.pending(received+preview.PREVIEW_STALE_NS+1)) == []
    assert not cache.camera_diagnostic(role, camera_diagnostic(role, state='STALE'))
    assert cache.source_status(received+1)['camera_'+role]['status'] == 'STALE'
    assert cache.camera_diagnostic(role, camera_diagnostic(role))
    assert cache.source_status(received+2)['camera_'+role]['status'] == 'STALE'
    new_image = NS(header=NS(frame_id='camera_usb3_1', stamp=stamp(1_000_000_501)))
    assert cache.camera(role, new_image, received+3)
    assert cache.source_status(received+4)['camera_'+role]['status'] == 'FRESH'
    assert list(cache.pending(received+4)) == [(role, new_image)]
    assert list(cache.pending(received+5)) == []


def test_non_camera_capture_does_not_claim_four_camera_sources(session):
    write(session/'configuration/source_selection.json', dict(selected_sources=['lidar_left', 'lidar_right', 'imu', 'wheel']))
    config = plan(session)
    assert config['cameras'] == {}
    assert len(preview.rviz_configuration(config, 'left')['Visualization Manager']['Displays']) == 1


def test_software_render_selection_is_owned_and_explicit_and_does_not_change_desktop():
    desktop = {'DISPLAY': ':test', 'PRIVATE_COOKIE': 'not logged', '__GLX_VENDOR_LIBRARY_NAME': 'nvidia',
               'LIBGL_ALWAYS_SOFTWARE': '0', 'MESA_LOADER_DRIVER_OVERRIDE': 'zink',
               '__NV_PRIME_RENDER_OFFLOAD': '1', '__NV_PRIME_RENDER_OFFLOAD_PROVIDER': 'NVIDIA-G0'}
    original = dict(desktop)
    env, selection = preview.rendering_environment(desktop)
    assert desktop == original and env is not desktop
    assert env['LIBGL_ALWAYS_SOFTWARE'] == '1' and env['GALLIUM_DRIVER'] == 'llvmpipe'
    assert env['__GLX_VENDOR_LIBRARY_NAME'] == 'mesa'
    assert not {'MESA_LOADER_DRIVER_OVERRIDE', '__NV_PRIME_RENDER_OFFLOAD',
                '__NV_PRIME_RENDER_OFFLOAD_PROVIDER'} & env.keys()
    assert selection['selected_backend'] == 'software' and not selection['actual_backend_verified']
    assert not selection['visible_pixels_verified'] and 'PRIVATE_COOKIE' not in selection['environment']
    system, report = preview.rendering_environment(desktop, 'system')
    assert system == original and report['selected_backend'] == 'system'
    with pytest.raises(ValueError): preview.rendering_environment(desktop, 'automatic')


def test_render_log_reports_version_without_assuming_pixels_or_actual_backend_and_preserves_failure(tmp_path):
    path = tmp_path/'right.log'; monitor = preview.RenderLogMonitor(path)
    assert monitor.observe()['status'] == 'WAITING_LOG'
    path.write_bytes(b'[rviz2] OpenGl version: 4.6 (GLSL 4.6).\n')
    observed = monitor.observe()
    assert observed['status'] == 'INITIALIZATION_REPORTED'
    assert observed['observed_backend'] == 'UNKNOWN' and not observed['actual_backend_verified']
    assert not observed['visible_pixels_verified']
    with path.open('ab') as stream: stream.write(b'Failed to create an OpenGL con')
    assert monitor.observe()['status'] != 'FAILED'
    with path.open('ab') as stream: stream.write(b'text. GLXBadDrawable\n')
    failed = monitor.observe()
    assert failed['status'] == 'FAILED' and 'GLXBadDrawable' in failed['errors'][0]
    with path.open('ab') as stream: stream.write(b'OpenGl version: 4.6\n')
    assert monitor.observe()['status'] == 'FAILED'  # Later output cannot heal a context failure.


def test_render_log_actual_renderer_is_evidence_not_the_launch_request(tmp_path):
    path = tmp_path/'left.log'
    path.write_text('Actual Ogre renderer: device=llvmpipe (LLVM 14), vendor_category=Mesa\n', encoding='utf-8')
    row = preview.RenderLogMonitor(path).observe()
    assert row['observed_backend'] == 'MESA_SOFTWARE' and row['actual_backend_verified']
    assert row['actual_renderer_evidence'] and not row['visible_pixels_verified']


def test_render_log_bounded_read_eventually_checks_errors_after_large_prefix_and_rejects_truncation(tmp_path):
    path = tmp_path/'right.log'
    path.write_bytes(b'x\n'*(preview.RENDER_LOG_READ_BYTES//2)+b'GLXBadDrawable\n')
    monitor = preview.RenderLogMonitor(path)
    first = monitor.observe()
    assert first['bytes_observed'] == preview.RENDER_LOG_READ_BYTES and first['unread_bytes'] > 0
    assert first['status'] != 'FAILED'
    assert monitor.observe()['status'] == 'FAILED'
    path.write_bytes(b'new')
    assert 'shrank' in monitor.observe()['errors'][-1]


@pytest.mark.parametrize('exercise_stream, render_failure', [(False, None), (True, None),
    (True, 'runtime'), (False, 'cleanup')])
def test_preview_close_cleans_only_owned_windows_and_subscriptions(monkeypatch, session, capsys, exercise_stream, render_failure):
    events, subscriptions, windows, callbacks, published, timers = [], [], [], {}, {}, []
    now_ns = [10_000_000_000]
    if exercise_stream:
        monkeypatch.setattr(preview.time, 'monotonic_ns', lambda: now_ns[0])
        monkeypatch.setattr(preview.time, 'monotonic', lambda: now_ns[0]/1_000_000_000)
    from wc_runtime import sensor_viewer
    monkeypatch.setattr(sensor_viewer, 'desktop_environment', lambda: {'DISPLAY': ':test'})
    monkeypatch.setattr(sensor_viewer, 'confirm_display', lambda env: None)
    monkeypatch.setattr(preview.sys, 'platform', 'linux')
    monkeypatch.setattr(preview.signal, 'SIGHUP', 999, raising=False)
    monkeypatch.setattr(preview.signal, 'signal', lambda *args: None)
    class Window:
        def __init__(self, side, command, *args):
            self.side = side; self.command = command; self.env = dict(args[0]); self.log_path = args[2]
            windows.append(self)
        def start(self):
            events.append('start_'+self.side)
            self.log_path.write_text('OpenGl version: 4.6\n', encoding='utf-8')
        def metadata(self): return {'side': self.side}
        def poll(self): return 0 if events.count('spin') >= (5 if exercise_stream else 1) and self.side == 'left' else None
        def request_stop(self): events.append('stop_'+self.side)
        def finish(self):
            events.append('finish_'+self.side)
            if render_failure == 'cleanup' and self.side == 'right':
                with self.log_path.open('a', encoding='utf-8') as stream: stream.write('GLXBadDrawable\n')
            return {'status': 'PASS', 'side': self.side}
    monkeypatch.setattr(sensor_viewer, 'OwnedRviz', Window)
    ros = ModuleType('rclpy')
    class Node:
        def create_publisher(self, kind, topic, qos):
            published[topic] = []
            return NS(publish=published[topic].append)
        def create_subscription(self, message_type, topic, callback, qos):
            subscriptions.append((topic, qos)); callbacks[topic] = callback; return object()
        def create_timer(self, period, callback): timers.append(callback); return object()
        def destroy_node(self): events.append('node_destroy')
    ros.init = lambda **kwargs: events.append('ros_init')
    ros.create_node = lambda *args: Node()
    def spin_once(*args, **kwargs):
        events.append('spin')
        if exercise_stream:
            count = events.count('spin')
            if count == 2: now_ns[0] += preview.PREVIEW_STALE_NS+1
            elif count > 2: now_ns[0] += 1
            if count in (1, 4):
                callbacks['/wc_mapping/lidar_left/source_frame'](lidar_message(sequence=0 if count == 1 else 1))
            for callback in timers: callback()
        if render_failure == 'runtime':
            with windows[1].log_path.open('a', encoding='utf-8') as stream:
                stream.write('Failed to create an OpenGL context. GLXBadDrawable\n')
    ros.spin_once = spin_once
    ros.ok = lambda: True
    ros.shutdown = lambda: events.append('ros_shutdown')
    signals = ModuleType('rclpy.signals'); signals.SignalHandlerOptions = NS(NO=0)
    qos = ModuleType('rclpy.qos'); qos.QoSProfile = lambda **kwargs: NS(**kwargs)
    qos.ReliabilityPolicy = NS(BEST_EFFORT='best_effort'); qos.HistoryPolicy = NS(KEEP_LAST='keep_last')
    for name, module in [('rclpy', ros), ('rclpy.signals', signals), ('rclpy.qos', qos)]:
        monkeypatch.setitem(sys.modules, name, module)
    for package, names in [('sensor_msgs', ('PointCloud2', 'Image')), ('wc_interfaces', ('SourceFrame',)),
                            ('diagnostic_msgs', ('DiagnosticArray',))]:
        module = ModuleType(package+'.msg')
        for name in names: setattr(module, name, type(name, (), {}))
        monkeypatch.setitem(sys.modules, package, ModuleType(package))
        monkeypatch.setitem(sys.modules, package+'.msg', module)
    assert preview.main(['--session-root', str(session), '--session-id', 'capture.test-1']) == (2 if render_failure else 0)
    assert len(windows) == 2 and all(window.command[0] == 'rviz2' for window in windows)
    assert len(subscriptions) == 10  # 2 SourceFrame + 4 Image + 4 camera identity diagnostics.
    assert all(qos.depth == 1 and qos.reliability == 'best_effort' for _, qos in subscriptions)
    assert all(window.env['LIBGL_ALWAYS_SOFTWARE'] == '1' and window.env['GALLIUM_DRIVER'] == 'llvmpipe'
               and window.env['__GLX_VENDOR_LIBRARY_NAME'] == 'mesa' for window in windows)
    assert events.index('stop_left') < events.index('finish_right')
    assert 'node_destroy' in events and 'ros_shutdown' in events
    status = json.loads((session/'capture_preview_status.json').read_text())
    assert status['status'] == ('FAILED' if render_failure else 'CLOSED')
    assert status['stop_reason'] == ('PREVIEW_RENDERING_FAILURE' if render_failure else 'PREVIEW_WINDOW_CLOSED')
    assert not status['recording_affected_by_preview_exit']
    assert json.loads((session/'capture_manifest.json').read_text())['status'] == 'RECORDING'
    assert json.loads((session/'capture_preview_ready.json').read_text())['ready'] is False
    assert status['rendering']['selected_backend'] == 'software'
    assert not status['rendering']['actual_backend_verified'] and not status['rendering']['visible_pixels_verified']
    assert len(status['source_status']) == 6
    assert all(row['status'] == 'WAITING' for name, row in status['source_status'].items()
               if name != 'lidar_left' or not exercise_stream)
    epoch_status = json.loads((session/'preview'/status['preview_epoch']/'status.json').read_text())
    assert epoch_status == status
    emitted = capsys.readouterr().out.splitlines()
    render_changes = [json.loads(line) for line in emitted if 'PREVIEW_RENDER_STATE_CHANGED' in line]
    assert render_changes and all(not change['recording_affected'] for change in render_changes)
    if render_failure:
        assert status['rendering']['windows']['right']['status'] == 'FAILED'
        assert 'GLXBadDrawable' in status['rendering']['windows']['right']['errors'][0]
        assert any(event['windows'].get('right', {}).get('status') == 'FAILED' for event in render_changes)
        if exercise_stream:
            assert status['source_status']['lidar_left']['status'] == 'FRESH'
        return
    changes = [json.loads(line) for line in emitted
               if 'PREVIEW_SOURCE_STATE_CHANGED' in line]
    assert all(not change['recording_affected'] for change in changes)
    if exercise_stream:
        assert [change['sources']['lidar_left']['status'] for change in changes] == ['WAITING','FRESH','STALE','FRESH']
        output = published[status['lidars']['left']['output_topic']]
        assert len(output) == 3 and [cloud.width for cloud in output] == [2, 0, 2]
        assert status['published_counts']['left'] == 2 and status['stale_clear_counts']['left'] == 1
        assert status['source_status']['lidar_left']['status'] == 'FRESH'
    else:
        assert len(changes) == 1
