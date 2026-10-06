#!/usr/bin/env python3
"""Bounded real-bag display isolation in the current native dashboard, no hardware.

Replays only recorded map/TF/odometry/path/status messages in localhost domain 84.
Original timestamps and geometry are preserved; /clock drives RViz simulated time.
Unrecorded wheelchair markers and the non-whitelisted input scan stay disabled.
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
DEFAULT_OUTPUT = 'reports/map_six_issues_20260914/recorded_scene_check_02'
PREFIX = '/wc_mapping/app/'
TYPES = {'cloud_map': 'sensor_msgs/msg/PointCloud2', 'grid_map': 'nav_msgs/msg/OccupancyGrid',
         'tf': 'tf2_msgs/msg/TFMessage', 'tf_static': 'tf2_msgs/msg/TFMessage',
         'odom': 'nav_msgs/msg/Odometry', 'prior_odom': 'nav_msgs/msg/Odometry',
         'mapPath': 'nav_msgs/msg/Path', 'status_markers': 'visualization_msgs/msg/MarkerArray'}
RVIZ_NODE, PLAYER_NODE = 'recorded_scene_rviz', 'recorded_scene_player'
TITLE = 'RECORDED SCENE DISPLAY DIAGNOSTIC - NO HARDWARE'


def playback_qos(view, topics):
    # Offer transient-local for the retained map/status displays. A volatile
    # observer alone cannot establish compatibility with a latched RViz display.
    retained = {PREFIX+role for role in ('cloud_map', 'grid_map', 'status_markers', 'tf_static')}
    retained.update(display.get('Topic', {}).get('Value')
        for display in view['Visualization Manager']['Displays']
        if display.get('Enabled') and display.get('Topic', {}).get('Durability Policy') == 'Transient Local')
    return {topic: {'history': 'keep_last', 'depth': 100, 'reliability': 'reliable',
                   'durability': 'transient_local' if topic in retained else 'volatile'} for topic in topics}


def qos_fields(profile):
    result = {name: getattr(getattr(profile, name), 'name', str(getattr(profile, name)))
              for name in ('history', 'reliability', 'durability', 'liveliness')}
    result['depth'] = profile.depth
    for name in ('deadline', 'lifespan', 'liveliness_lease_duration'):
        result[name+'_ns'] = getattr(profile, name).nanoseconds
    return result


def native_qos_evidence(node, topics, check_compatible, ok_value, error_value):
    """Inspect real graph endpoint QoS, never infer RViz receipt from our observer."""
    endpoints, evidence, errors = {}, {}, []
    ready = True
    for topic in topics:
        publishers = [info for info in node.get_publishers_info_by_topic(topic) if info.node_name == PLAYER_NODE]
        subscribers = [info for info in node.get_subscriptions_info_by_topic(topic)
            if info.node_name == RVIZ_NODE or (topic in (PREFIX+'tf', PREFIX+'tf_static')
                and info.node_name.startswith('transform_listener_impl_'))]
        endpoints[topic] = [info.node_name for info in subscribers]
        rows = evidence[topic] = []
        ready = ready and len(publishers) == 1 and bool(subscribers)
        for publisher in publishers:
            for subscriber in subscribers:
                compatibility, reason = check_compatible(publisher.qos_profile, subscriber.qos_profile)
                rows.append({'publisher_node': publisher.node_name, 'subscriber_node': subscriber.node_name,
                    'offered_qos': qos_fields(publisher.qos_profile), 'requested_qos': qos_fields(subscriber.qos_profile),
                    'compatibility': getattr(compatibility, 'name', str(compatibility)), 'reason': reason})
                if compatibility == error_value:
                    errors.append(topic+': '+reason)
                # Unknown/system-default profiles are not proof of compatibility.
                ready = ready and compatibility == ok_value
    return endpoints, evidence, ready, errors


def recorded_view(original):
    from check_dashboard_saved_render import offline_dashboard_view
    offline_dashboard_view(original)  # Reuse the reviewed panel/config contract.
    view = copy.deepcopy(original)
    manager = view['Visualization Manager']
    manager['Global Options']['Fixed Frame'] = 'mapping_map'
    enabled, disabled = [], []
    classes = {'cloud_map': 'rviz_default_plugins/PointCloud2', 'grid_map': 'rviz_default_plugins/Map',
               'odom': 'rviz_default_plugins/Odometry', 'prior_odom': 'rviz_default_plugins/Odometry',
               'mapPath': 'rviz_default_plugins/Path', 'status_markers': 'rviz_default_plugins/MarkerArray'}
    for display in manager['Displays']:
        topic = display.get('Topic', {}).get('Value')
        if topic == PREFIX+'view_grid':
            display['Topic']['Value'] = topic = PREFIX+'grid_map'
        role = topic[len(PREFIX):] if isinstance(topic, str) and topic.startswith(PREFIX) else None
        active = ((role in classes and display.get('Class') == classes[role]) or
                  display.get('Class') == 'rviz_default_plugins/TF')
        display['Enabled'] = display['Value'] = active
        (enabled if active else disabled).append({'name': display.get('Name'), 'topic': topic,
                                                  'class': display.get('Class')})
    if {item['topic'] for item in enabled if item['topic']} != {PREFIX+key for key in classes}:
        raise ValueError('The recorded dashboard must include all six reviewed topic displays')
    if sum(item['class'] == 'rviz_default_plugins/TF' for item in enabled) != 1:
        raise ValueError('Exactly one native TF display is required')
    return view, {'enabled_displays': enabled, 'disabled_displays': disabled,
                  'wheelchair_view_markers_rebuilt': False, 'camera_images_published': False,
                  'native_grid_used_without_candidate_ground_shift': True}


def selected_topics(metadata):
    rows = metadata['rosbag2_bagfile_information']['topics_with_message_count']
    actual = {row['topic_metadata']['name']: row for row in rows}
    selected, missing = [], []
    for role, message_type in TYPES.items():
        topic = PREFIX+role
        row = actual.get(topic)
        if row is None or int(row['message_count']) == 0:
            if role != 'tf_static':
                raise ValueError('Required recorded display topic is absent: '+topic)
            missing.append(topic)
        else:
            if row['topic_metadata']['type'] != message_type:
                raise ValueError('Unexpected recorded display type: '+topic)
            selected.append(topic)
    return selected, missing


def stop_components(children, deadline):
    """Notify all owned components first; share one bounded cleanup allowance."""
    errors = []
    for name, child in children:
        if child is not None and child.poll() is None:
            try:
                child.send_signal(signal.SIGINT)
            except ProcessLookupError:
                pass
    codes = {}
    for name, child in children:
        if child is None:
            continue
        try:
            codes[name] = child.wait(timeout=max(.01, deadline-time.monotonic()))
            if codes[name] != 0:
                errors.append(name+' exited '+str(codes[name]))
        except subprocess.TimeoutExpired:
            errors.append(name+' did not finish owned cleanup within total deadline')
    return codes, errors


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session-root', type=Path, default=Path(DEFAULT_SESSION))
    parser.add_argument('--output-root', type=Path, default=Path(DEFAULT_OUTPUT))
    args = parser.parse_args(argv)
    import fcntl
    import yaml
    from wc_runtime.cli import ROOT, target
    from wc_runtime.sensor_viewer import desktop_environment
    from check_mapping_app_export import contained, fingerprint, read_json
    from check_mapping_app_live import close_owned_window
    from check_rviz_display import capture_owned_window
    target(); os.chdir(ROOT)
    session, out = contained(ROOT, args.session_root), contained(ROOT, args.output_root)
    if not session.is_dir() or out.exists():
        raise ValueError('An existing source session and new report directory are required')
    out.mkdir(parents=True, exist_ok=False)
    started = time.monotonic(); deadline = started+90.; work_deadline = started+60.
    result = {'test': 'RECORDED_SCENE_NATIVE_DASHBOARD_ISOLATION', 'status': 'FAIL',
              'validation_level': 'REAL_BAG_GUI_CAPTURE', 'session_root': str(session),
              'report_root': str(out), 'domain_id': 84, 'rate': 1., 'total_budget_s': 90.,
              'post_ready_observation_s': 10., 'hardware_started': False,
              'teleop_socket_started': False, 'source_or_control_topics_replayed': False,
              'synthetic_tf_or_geometry_published': False, 'use_sim_time': True,
              'clock_generated_by_rosbag': True, 'visual_review_status': 'PENDING',
              'native_message_receipt_verified': False,
              'observed_recorded_messages_scope': 'Observer callbacks only; these counts do not prove RViz received or displayed a message.',
              'pass_meaning': 'Original audit passed; actual player/native endpoint QoS compatible; observer received recorded messages; owned screenshot/close completed. Native visible content requires separate screenshot inspection.',
              'prior_run_diagnostic': {'report': 'recorded_scene_check_01', 'status': 'FAIL',
                  'scope': 'QOS_INCOMPATIBLE_NATIVE_MAP_DELIVERY',
                  'reason': 'Volatile playback offered cloud/grid to transient-local RViz subscriptions. Observer counts cannot validate native map delivery; prior result is not a successful map-display test.'}}
    stopped = [False]
    previous = {number: signal.signal(number, lambda *_: stopped.__setitem__(0, True))
                for number in (signal.SIGINT, signal.SIGTERM)}
    audit_process = gui = player = node = executor = lock = None
    logs, ros_started = [], False
    try:
        lock = contained(ROOT, '.phase1_runtime/locks/domain-84-offline-review.lock').open('a')
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        env = desktop_environment()
        env.update(ROS_DOMAIN_ID='84', ROS_LOCALHOST_ONLY='1', PYTHONNOUSERSITE='1')
        os.environ.update(ROS_DOMAIN_ID='84', ROS_LOCALHOST_ONLY='1')
        component = [sys.executable, '-s', '-m', 'wc_runtime.component', '--parent', str(os.getpid()),
                     '--sigint-grace-s', '5', '--']

        def launch(command, logfile):
            log = (out/logfile).open('xb'); logs.append(log)
            return subprocess.Popen(component+command, cwd=ROOT, env=env, stdin=subprocess.DEVNULL,
                stdout=log, stderr=subprocess.STDOUT, start_new_session=True)

        def require_running(until):
            if stopped[0] or time.monotonic() >= min(until, work_deadline):
                raise RuntimeError('Recorded-scene observation interrupted or reached its bounded deadline')
            if gui is not None and gui.poll() is not None:
                raise RuntimeError('Owned native dashboard exited before capture')
            if player is not None and player.poll() is not None:
                raise RuntimeError('Owned bag player exited before display observation completed')

        audit_process = launch([sys.executable, '-s', str(ROOT/'tests/integration/check_mapping_app_export.py'),
            '--project-root', str(ROOT), '--session-root', str(session), '--output', str(out/'audit.json')], 'audit.log')
        until = time.monotonic()+25
        while audit_process.poll() is None:
            require_running(until); time.sleep(.05)
        audit = read_json(out/'audit.json')
        if audit_process.returncode != 0 or audit.get('status') != 'PASS' or audit.get('session_root') != str(session):
            result['audit_rejection'] = audit
            raise RuntimeError('Original closed-session export audit rejected the source; replay refused')
        result['audit'] = {'file': str(out/'audit.json'), 'session_id': audit['session_id'], 'status': 'PASS'}
        bag = contained(session, 'bag'); meta_path = contained(bag, 'metadata.yaml')
        result['bag_metadata'] = {'file': str(meta_path), **fingerprint(meta_path)}
        metadata = yaml.safe_load(meta_path.read_text(encoding='utf-8'))
        topics, missing = selected_topics(metadata)
        result['replay_whitelist'], result['missing_unfabricated_topics'] = topics, missing
        result['bag_files'] = audit['bag_files']
        source_view = contained(session, 'view.rviz')
        if source_view.stat().st_size > 2_000_000:
            raise ValueError('Source RViz config exceeds size bound')
        view, changes = recorded_view(yaml.safe_load(source_view.read_text(encoding='utf-8')))
        result['view_changes'] = changes
        result['source_view'] = {'file': str(source_view), **fingerprint(source_view)}
        # This filename cannot satisfy the live WASD panel session identity gate.
        view_path = out/'recorded_scene.rviz'
        view_path.write_text(yaml.safe_dump(view, sort_keys=False, allow_unicode=True), encoding='utf-8')
        binary = contained(ROOT, 'install/main/wc_bringup/lib/wc_bringup/mapping_rviz')
        library = contained(ROOT, 'install/main/wc_camera_panel/lib/libwc_camera_panel.so')
        result['wrapper_binary'] = {'file': str(binary), **fingerprint(binary)}
        result['camera_plugin_binary'] = {'file': str(library), **fingerprint(library)}
        overrides = playback_qos(view, topics)
        result['playback_qos_overrides'] = overrides
        qos_path = out/'playback_qos.yaml'
        qos_path.write_text(yaml.safe_dump(overrides), encoding='utf-8')
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.signals import SignalHandlerOptions
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_check_compatible, QoSCompatibility
        from rclpy.parameter import Parameter
        from rosbag2_interfaces.srv import Resume
        from rosidl_runtime_py.utilities import get_message
        from rosgraph_msgs.msg import Clock
        rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO); ros_started = True
        node = rclpy.create_node('recorded_scene_observer', parameter_overrides=[Parameter('use_sim_time', value=True)])
        executor = SingleThreadedExecutor(); executor.add_node(node)
        messages = {topic: {'count': 0, 'first_receipt_monotonic_ns': None,
                           'last_receipt_monotonic_ns': None} for topic in topics}
        clocks = {'count': 0, 'first_stamp_ns': None, 'last_stamp_ns': None}
        result['observed_recorded_messages'] = messages; result['observed_clock'] = clocks

        def received(topic, message):
            row = messages[topic]; now_ns = time.monotonic_ns()
            row['count'] += 1
            row['first_receipt_monotonic_ns'] = row['first_receipt_monotonic_ns'] or now_ns
            row['last_receipt_monotonic_ns'] = now_ns

        def clock_received(message):
            stamp = message.clock.sec*1_000_000_000+message.clock.nanosec
            clocks['count'] += 1; clocks['last_stamp_ns'] = stamp
            clocks['first_stamp_ns'] = clocks['first_stamp_ns'] or stamp

        subscriptions = [node.create_subscription(get_message(TYPES[topic[len(PREFIX):]]), topic,
            lambda message, topic=topic: received(topic, message), QoSProfile(depth=100,
                reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL
                if overrides[topic]['durability'] == 'transient_local' else DurabilityPolicy.VOLATILE)) for topic in topics]
        subscriptions.append(node.create_subscription(Clock, '/clock', clock_received,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)))
        gui = launch([str(binary), '-d', str(view_path), '-t', TITLE, '--ros-args',
            '-r', '__node:='+RVIZ_NODE, '-r', '/tf:='+PREFIX+'tf', '-r', '/tf_static:='+PREFIX+'tf_static',
            '-p', 'use_sim_time:=true'], 'rviz.log')
        command = ['ros2', 'bag', 'play', str(bag), '--storage', 'sqlite3', '--start-paused',
            '--disable-keyboard-controls', '--read-ahead-queue-size', '100', '--rate', '1', '--clock',
            '--qos-profile-overrides-path', str(qos_path), '--remap', '__node:='+PLAYER_NODE, '--topics', *topics]
        result['player_command'] = command
        player = launch(command, 'player.log')
        client = node.create_client(Resume, '/'+PLAYER_NODE+'/resume')

        def compatible_native_endpoints():
            endpoints, evidence, compatible, errors = native_qos_evidence(
                node, topics, qos_check_compatible, QoSCompatibility.OK, QoSCompatibility.ERROR)
            result['latest_native_subscriptions'] = endpoints
            result['latest_native_qos_compatibility'] = evidence
            result['all_native_qos_confirmed_compatible'] = compatible
            if errors:
                result['failure_scope'] = 'QOS_INCOMPATIBLE_NATIVE_DISPLAY_DELIVERY'
                raise RuntimeError('NATIVE_QOS_INCOMPATIBLE: '+'; '.join(errors))
            return endpoints, compatible

        until = time.monotonic()+20; stable = None
        while True:
            require_running(until); executor.spin_once(timeout_sec=.05)
            endpoints, compatible = compatible_native_endpoints()
            ready = client.service_is_ready() and compatible
            stable = (stable or time.monotonic()) if ready else None
            if stable is not None and time.monotonic()-stable >= 1:
                break
        if any(row['count'] for row in messages.values()):
            raise RuntimeError('Bag published historical display data before explicit resume')
        advertised = [topic for topic, _ in node.get_topic_names_and_types()
            if any(info.node_name == PLAYER_NODE for info in node.get_publishers_info_by_topic(topic))]
        allowed = set(topics) | {'/clock', '/rosout', '/parameter_events', '/events/read_split'}
        if not set(advertised) <= allowed:
            raise RuntimeError('Owned player advertised a topic outside the explicit display whitelist')
        result['player_publishers_before_resume'] = advertised
        result['native_subscriptions_before_resume'] = endpoints
        result['native_qos_before_resume'] = copy.deepcopy(result['latest_native_qos_compatibility'])
        resumed = client.call_async(Resume.Request()); until = time.monotonic()+5
        while not resumed.done():
            require_running(until); executor.spin_once(timeout_sec=.05)
        if resumed.exception() is not None:
            raise RuntimeError('Owned paused player Resume service failed')
        result['replay_started_after_native_subscription_discovery'] = True
        ready_since = None; next_qos_check = 0.
        while True:
            require_running(work_deadline); executor.spin_once(timeout_sec=.05)
            if time.monotonic() >= next_qos_check:
                _, compatible = compatible_native_endpoints()
                next_qos_check = time.monotonic()+.5
            if compatible and all(row['count'] for row in messages.values()) and clocks['count'] > 0:
                ready_since = ready_since or time.monotonic()
                if time.monotonic()-ready_since >= 10:
                    break
            else:
                ready_since = None
        _, compatible = compatible_native_endpoints()
        if not compatible:
            raise RuntimeError('Native display QoS is no longer confirmed compatible before capture')
        result['observed_recorded_messages'] = messages; result['observed_clock'] = clocks
        result['window'] = capture_owned_window(gui, out/'recorded_scene.png', env, activate_owned=True)
        close_owned_window(gui, result['window'], env)
        result['close_exit_code'] = gui.wait(timeout=max(.01, min(10, deadline-time.monotonic()-12)))
        result['close_method'] = 'WM_DELETE_WINDOW_ONLY'
        if gui.returncode != 0:
            raise RuntimeError('Recorded-scene native window did not close normally')
        for path, field in ((source_view, 'source_view'), (binary, 'wrapper_binary'), (library, 'camera_plugin_binary'), (meta_path, 'bag_metadata')):
            if fingerprint(path) != {key: result[field][key] for key in ('bytes', 'sha256')}:
                raise RuntimeError('Source configuration/binary/bag metadata changed during recorded-scene review')
        result['same_current_binaries_after_capture'] = True
        result['status'] = 'PASS'
    except Exception as error:
        result['error'] = type(error).__name__+': '+str(error)
    finally:
        codes, errors = stop_components((('gui', gui), ('player', player), ('audit', audit_process)), deadline)
        result['final_returncodes'] = codes
        if errors:
            result['cleanup_errors'] = errors; result['status'] = 'FAIL'
        if executor is not None:
            executor.remove_node(node); executor.shutdown(timeout_sec=1.)
        if node is not None:
            node.destroy_node()
        if ros_started and rclpy.ok():
            rclpy.shutdown()
        for log in logs:
            log.close()
        if lock is not None:
            lock.close()
        for number, handler in previous.items():
            signal.signal(number, handler)
        result['elapsed_s'] = time.monotonic()-started
        for name in ('rviz', 'player'):
            if (out/(name+'.log')).exists():
                result[name+'_log_tail'] = (out/(name+'.log')).read_text(errors='replace')[-5000:]
        with (out/'result.json').open('x', encoding='utf-8') as stream:
            json.dump(result, stream, ensure_ascii=False, indent=2, allow_nan=False)
        print(json.dumps(result, ensure_ascii=False, allow_nan=False))
    return 0 if result['status'] == 'PASS' else 1


if __name__ == '__main__':
    raise SystemExit(main())
