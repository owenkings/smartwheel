#!/usr/bin/env python3
"""Reload audited real maps in the complete dashboard layout, without hardware.

Run in the sourced target ROS environment. Domain 84 is held exclusively by
the existing offline-review lock. The new screenshot still needs visual review;
successful subscriptions and normal window close do not prove a nonblack image.
"""
import argparse
import copy
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time


DEFAULT_SESSION = 'reports/maps/map_dashboard_left_20260914_04'
DEFAULT_OUTPUT = 'reports/map_six_issues_20260914/saved_render_check_01'
SYNTHETIC_OUTPUT = 'reports/map_six_issues_20260914/saved_render_synthetic_cameras_01'
SYNTHETIC_TITLE = 'SYNTHETIC CAMERA DISPLAY DIAGNOSTIC'
RVIZ_NODE = 'mapping_saved_dashboard_validation'


def offline_dashboard_view(original):
    """Preserve original panels/styles/views; no substitute geometry or TF."""
    view = copy.deepcopy(original)
    if not isinstance(view, dict) or not isinstance(view.get('Panels'), list):
        raise ValueError('Saved dashboard config has no panel list')
    allowed_panels = {'rviz_common/Displays', 'rviz_common/Views', 'rviz_common/Tool Properties',
                      'wc_camera_panel/CameraPanel'}
    if any(panel.get('Class') not in allowed_panels for panel in view['Panels']):
        raise ValueError('Unreviewed panel class in saved dashboard; no plugin fallback')
    manager = view['Visualization Manager']
    manager['Global Options']['Fixed Frame'] = 'mapping_map'
    changes, disabled = [], []
    for display in manager['Displays']:
        topic = display.get('Topic', {}).get('Value')
        if display.get('Class') == 'rviz_default_plugins/PointCloud2' and topic == '/wc_mapping/app/cloud_map':
            target = '/wc_mapping/offline/cloud_map'
        elif display.get('Class') == 'rviz_default_plugins/Map' and topic in (
                '/wc_mapping/app/view_grid', '/wc_mapping/app/grid_map'):
            target = '/wc_mapping/offline/grid_map'
        else:
            display['Enabled'] = display['Value'] = False
            disabled.append(display.get('Name'))
            continue
        display['Topic'].update(Value=target, Depth=1, **{
            'History Policy': 'Keep Last', 'Reliability Policy': 'Reliable', 'Durability Policy': 'Transient Local'})
        display['Enabled'] = display['Value'] = True
        changes.append({'display': display.get('Name'), 'source_topic': topic, 'offline_topic': target})
    if sorted(row['offline_topic'] for row in changes) != ['/wc_mapping/offline/cloud_map', '/wc_mapping/offline/grid_map']:
        raise ValueError('Exactly one saved 3D and one saved 2D dashboard display required')
    return view, {'topic_changes': changes, 'disabled_unreplayed_displays': disabled,
                  'panels_preserved': view['Panels'], 'saved_native_grid_height_used': True,
                  'live_candidate_ground_projection_replayed': False}


def stop_owned(child):
    """The session component owns/reaps its descendants within its own grace."""
    if child is None:
        return None
    if child.poll() is None:
        child.send_signal(signal.SIGINT)
        child.wait(timeout=15)  # Component: 5s INT + 3s TERM + 2s KILL.
    return child.returncode


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session-root', type=Path, default=Path(DEFAULT_SESSION))
    parser.add_argument('--output-root', type=Path)
    parser.add_argument('--synthetic-camera-images', action='store_true',
                        help='Add artificial 320x240 BGR colour bars at 8Hz; no camera hardware')
    args = parser.parse_args(argv)
    if args.output_root is None:
        args.output_root = Path(SYNTHETIC_OUTPUT if args.synthetic_camera_images else DEFAULT_OUTPUT)
    # Keep --help and the config-copy helper usable without ROS or Linux imports.
    import fcntl
    import yaml
    from wc_runtime.cli import ROOT, target
    from wc_runtime.sensor_viewer import desktop_environment
    from check_mapping_app_export import contained, fingerprint, read_json
    from check_mapping_app_live import close_owned_window
    from check_rviz_display import capture_owned_window
    target()
    os.chdir(ROOT)
    session, out = contained(ROOT, args.session_root), contained(ROOT, args.output_root)
    if not session.is_dir() or out.exists():
        raise ValueError('Existing source session and NEW report directory required')
    out.mkdir(parents=True, exist_ok=False)
    result = {'test': 'AUDITED_REAL_SAVED_MAPS_IN_NATIVE_DASHBOARD', 'status': 'FAIL',
              'validation_level': 'OFFLINE_REAL_FILES_GUI_CAPTURE', 'session_root': str(session),
              'report_root': str(out), 'domain_id': 84, 'hardware_started': False,
              'teleop_socket_started': False, 'synthetic_tf_published': False,
              'camera_images_published': False, 'live_camera_validation': False,
              'synthetic_camera_images_requested': args.synthetic_camera_images,
              'geometry_accuracy_validated': False, 'visual_review_status': 'PENDING',
              'pass_meaning': 'Original export audit passed; saved maps published; owned native dashboard captured and closed. Visual rendering requires separate screenshot inspection.'}
    stopped = [False]
    previous_signals = {number: signal.signal(number, lambda *_: stopped.__setitem__(0, True))
                        for number in (signal.SIGINT, signal.SIGTERM)}
    gui = publisher = cameras = node = executor = lock = None
    logs, ros_started = [], False
    started = time.monotonic()
    try:
        lock_path = contained(ROOT, '.phase1_runtime/locks/domain-84-offline-review.lock')
        lock = lock_path.open('a')
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        env = desktop_environment()
        env.update(ROS_DOMAIN_ID='84', ROS_LOCALHOST_ONLY='1', PYTHONNOUSERSITE='1')
        os.environ.update(ROS_DOMAIN_ID='84', ROS_LOCALHOST_ONLY='1')
        binary = contained(ROOT, 'install/main/wc_bringup/lib/wc_bringup/mapping_rviz')
        result['wrapper_binary'] = {'file': str(binary), **fingerprint(binary)}
        source_view = contained(session, 'view.rviz')
        if source_view.stat().st_size > 2_000_000:
            raise ValueError('Saved dashboard config exceeds size bound')
        result['source_view'] = {'file': str(source_view), **fingerprint(source_view)}
        view, changes = offline_dashboard_view(yaml.safe_load(source_view.read_text(encoding='utf-8')))
        result['view_changes'] = changes
        camera_ports = None
        if args.synthetic_camera_images:
            from synthetic_dashboard_cameras import reviewed_ports, ROLES, TOPICS, NODE_NAME
            camera_ports = reviewed_ports(view)
            camera_library = contained(ROOT, 'install/main/wc_camera_panel/lib/libwc_camera_panel.so')
            result['camera_plugin_binary'] = {'file': str(camera_library), **fingerprint(camera_library)}
            result['validation_level'] = 'OFFLINE_REAL_MAPS_SYNTHETIC_CAMERA_GUI_CAPTURE'
            result['synthetic_camera_display'] = {
                'synthetic': True, 'window_title': SYNTHETIC_TITLE, 'ports': camera_ports,
                'duration_limit_s': 90, 'requested_hz_per_stream': 8, 'width': 320, 'height': 240,
                'encoding': 'bgr8', 'scope': 'Artificial camera pixels with audited real saved maps; not live camera validation'}
        # The C++ panel accepts ONLY a session's view.rviz with matching runtime
        # identities. This deliberately different name never attempts manual.sock.
        offline_view = out/'offline_dashboard.rviz'
        with offline_view.open('x', encoding='utf-8') as stream:
            yaml.safe_dump(view, stream, allow_unicode=True, sort_keys=False)
        result['offline_view'] = {'file': str(offline_view), **fingerprint(offline_view)}
        component = [sys.executable, '-s', '-m', 'wc_runtime.component', '--parent', str(os.getpid()),
                     '--sigint-grace-s', '5', '--']
        log = (out/'publisher.log').open('xb'); logs.append(log)
        publisher = subprocess.Popen(component+[sys.executable, '-s',
            str(ROOT/'tests/integration/check_mapping_app_export.py'), '--project-root', str(ROOT),
            '--session-root', str(session), '--output', str(out/'audit.json'), '--publish', '--duration', '90'],
            cwd=ROOT, env=env, stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        deadline = time.monotonic()+30
        while not (out/'audit.rviz').exists():
            # audit.json is an exclusive new file, not an atomic replacement;
            # inspect a rejection only after its writer has exited.
            if publisher.poll() is not None and (out/'audit.json').exists():
                audit = read_json(out/'audit.json')
                if audit.get('status') != 'PASS':
                    result['audit_rejection'] = audit
                    raise RuntimeError('Original export audit rejected this session; no bypass and no RViz launch')
            if stopped[0] or publisher.poll() is not None or time.monotonic() >= deadline:
                raise RuntimeError('Saved-file audit/publisher did not become ready within 30s')
            time.sleep(.1)
        audit = read_json(out/'audit.json')
        if audit.get('status') != 'PASS' or audit.get('session_root') != str(session):
            raise RuntimeError('Audit status/source-session mismatch')
        result['audit'] = {'file': str(out/'audit.json'), 'status': 'PASS', 'session_id': audit['session_id'],
                           'source_clouds': audit['clouds'], 'source_maps': audit['maps']}
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.signals import SignalHandlerOptions
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        from sensor_msgs.msg import PointCloud2
        from nav_msgs.msg import OccupancyGrid
        rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO); ros_started = True
        node = rclpy.create_node('saved_dashboard_render_observer')
        executor = SingleThreadedExecutor(); executor.add_node(node)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        messages = {'cloud_map': 0, 'grid_map': 0}

        def received(name, message):
            if message.header.frame_id != 'mapping_map':
                raise RuntimeError('Saved-map publication has unexpected frame')
            messages[name] += 1

        subscriptions = [node.create_subscription(kind, '/wc_mapping/offline/'+name,
            lambda message, name=name: received(name, message), qos)
            for name, kind in (('cloud_map', PointCloud2), ('grid_map', OccupancyGrid))]
        camera_status_path = out/'synthetic_cameras.json'
        if args.synthetic_camera_images:
            camera_script = ROOT/'tests/integration/synthetic_dashboard_cameras.py'
            result['synthetic_camera_script'] = {'file': str(camera_script), **fingerprint(camera_script)}
            log = (out/'synthetic_cameras.log').open('xb'); logs.append(log)
            cameras = subprocess.Popen(component+[sys.executable, '-s', str(camera_script),
                '--output', str(camera_status_path), '--ports', *map(str, camera_ports), '--duration', '90'],
                cwd=ROOT, env=env, stdin=subprocess.DEVNULL, stdout=log,
                stderr=subprocess.STDOUT, start_new_session=True)
        log = (out/'rviz.log').open('xb'); logs.append(log)
        # Native Humble VisualizerApp supports -t/--display-title-format.
        title_args = ['-t', SYNTHETIC_TITLE] if args.synthetic_camera_images else []
        gui = subprocess.Popen(component+[str(binary), '-d', str(offline_view)]+title_args+['--ros-args',
            '-r', '__node:='+RVIZ_NODE, '-r', '/tf:=/wc_mapping/offline/tf',
            '-r', '/tf_static:=/wc_mapping/offline/tf_static'], cwd=ROOT, env=env,
            stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        deadline, ready_since = time.monotonic()+30, None
        while time.monotonic() < deadline:
            if stopped[0] or gui.poll() is not None or publisher.poll() is not None or (cameras is not None and cameras.poll() is not None):
                raise RuntimeError('Owned GUI or saved publisher stopped before screenshot')
            executor.spin_once(timeout_sec=.1)
            endpoints = {topic: [info.node_name for info in node.get_subscriptions_info_by_topic('/wc_mapping/offline/'+topic)
                                 if info.node_name == RVIZ_NODE] for topic in messages}
            camera_ready = True
            if args.synthetic_camera_images:
                camera_endpoints = {role: [info.node_name for info in node.get_subscriptions_info_by_topic(TOPICS[role])
                    if info.node_namespace == '/wc_mapping/ui' and info.node_name.startswith('wc_camera_panel_')
                    and info.topic_type == 'sensor_msgs/msg/Image'] for role in ROLES}
                camera_publishers = {role: [info.node_name for info in node.get_publishers_info_by_topic(TOPICS[role])
                    if info.node_namespace == '/wc_mapping/offline' and info.node_name == NODE_NAME
                    and info.topic_type == 'sensor_msgs/msg/Image'] for role in ROLES}
                state = read_json(camera_status_path) if camera_status_path.exists() else {}
                camera_ready = (all(camera_endpoints.values()) and all(camera_publishers.values())
                    and state.get('status') == 'RUNNING' and set(state.get('streams', {})) == set(ROLES)
                    and all(row['published'] >= 8 and row['matched_subscribers'] >= 1
                        and 0 <= time.monotonic_ns()-row['last_published_monotonic_ns'] <= 1_000_000_000
                        for row in state['streams'].values()))
                result['synthetic_camera_subscriptions'] = camera_endpoints
                result['synthetic_camera_publishers'] = camera_publishers
                result['synthetic_camera_progress'] = state
            if all(messages.values()) and all(endpoints.values()) and camera_ready:
                ready_since = ready_since or time.monotonic()
                if time.monotonic()-ready_since >= 5:
                    break
            elif args.synthetic_camera_images:
                ready_since = None
        else:
            raise RuntimeError('Required saved-map'+('/synthetic-camera' if args.synthetic_camera_images else '')+
                               ' messages and native subscriptions did not become ready within 30s')
        result['observed_saved_messages'] = messages
        result['native_subscriptions'] = endpoints
        result['window'] = capture_owned_window(gui, out/'saved_dashboard.png', env, activate_owned=True)
        if args.synthetic_camera_images:
            result['camera_images_published'] = True
            title = subprocess.check_output(['xprop', '-id', result['window']['window_id'],
                '_NET_WM_NAME', 'WM_NAME'], env=env, text=True, timeout=5)
            result['observed_window_title'] = title
            if SYNTHETIC_TITLE not in title:
                raise RuntimeError('Synthetic diagnostic title missing from owned native window')
        close_owned_window(gui, result['window'], env)
        result['close_exit_code'] = gui.wait(timeout=15)
        result['close_method'] = 'WM_DELETE_WINDOW_ONLY'
        if gui.returncode != 0:
            raise RuntimeError('Owned native dashboard did not close normally')
        if fingerprint(source_view) != {key: result['source_view'][key] for key in ('bytes', 'sha256')}:
            raise RuntimeError('Original dashboard configuration changed during review')
        if args.synthetic_camera_images:
            result['same_wrapper_binary_after_capture'] = fingerprint(binary) == {
                key: result['wrapper_binary'][key] for key in ('bytes', 'sha256')}
            if not result['same_wrapper_binary_after_capture']:
                raise RuntimeError('Native wrapper binary changed during camera diagnostic')
            result['same_camera_plugin_after_capture'] = fingerprint(camera_library) == {
                key: result['camera_plugin_binary'][key] for key in ('bytes', 'sha256')}
            if not result['same_camera_plugin_after_capture']:
                raise RuntimeError('Camera plugin binary changed during diagnostic')
        result['status'] = 'PASS'
    except Exception as error:
        result['error'] = type(error).__name__+': '+str(error)
    finally:
        for name, child in (('gui', gui), ('publisher', publisher), ('synthetic_cameras', cameras)):
            try:
                code = stop_owned(child)
                result[name+'_final_returncode'] = code
                if code not in (0, None):
                    result.setdefault('cleanup_errors', []).append(name+' exited '+str(code))
                    result['status'] = 'FAIL'
            except Exception as error:
                result.setdefault('cleanup_errors', []).append(name+': '+str(error))
                result['status'] = 'FAIL'
        if cameras is not None:
            try:
                camera_final = read_json(out/'synthetic_cameras.json')
                result['synthetic_camera_final'] = camera_final
                result['camera_images_published'] = any(row['published'] > 0 for row in camera_final['streams'].values())
                if (camera_final.get('status') != 'PASS' or camera_final.get('final') is not True
                        or any(row['published'] < 8 or row['max_matched_subscribers'] < 1
                               or row['published_span_s'] > 90 for row in camera_final['streams'].values())):
                    raise RuntimeError('Synthetic cameras did not close with complete bounded publication evidence')
            except Exception as error:
                result.setdefault('cleanup_errors', []).append('synthetic camera evidence: '+str(error))
                result['status'] = 'FAIL'
        if executor is not None:
            if node is not None:
                executor.remove_node(node)
            executor.shutdown(timeout_sec=1.)
        if node is not None:
            node.destroy_node()
        if ros_started and rclpy.ok():
            rclpy.shutdown()
        for log in logs:
            log.close()
        if lock is not None:
            lock.close()
        for number, handler in previous_signals.items():
            signal.signal(number, handler)
        result['elapsed_s'] = time.monotonic()-started
        if (out/'rviz.log').exists():
            result['rviz_log_tail'] = (out/'rviz.log').read_text(errors='replace')[-6000:]
        with (out/'result.json').open('x', encoding='utf-8') as stream:
            json.dump(result, stream, ensure_ascii=False, indent=2, allow_nan=False)
        print(json.dumps(result, ensure_ascii=False, allow_nan=False))
    return 0 if result['status'] == 'PASS' else 1


if __name__ == '__main__':
    raise SystemExit(main())
