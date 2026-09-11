#!/usr/bin/env python3
"""Isolated, bounded regression of FAST-LIO wheel aiding on an existing bag.

Source ROS + workspace setup, then run this script. It NEVER starts hardware,
RViz, a motor driver, or a command-velocity publisher. Only the four allowlisted
bag topics are replayed (plus rosbag's clock). The capture's diagnostic layout
is explicit; this is not a physical calibration or trajectory acceptance.
"""
import argparse
import json
import math
import os
from pathlib import Path
import re
import signal
import sqlite3
import subprocess
import time
import shutil
import hashlib

import yaml

ROOT = Path('/home/nvidia/smartwheel')
DEFAULT_BAG = ROOT / 'bags/diag/right_vertical_jump_20260910_210202'
ALLOWED = ['/xtm60/right/points', '/imu/data', '/wheel/odom',
           '/base/wheel_feedback_healthy']


def inspect_bag(bag):
    from rclpy.serialization import deserialize_message
    from nav_msgs.msg import Odometry
    from std_msgs.msg import Bool
    meta = yaml.safe_load((bag / 'metadata.yaml').read_text())['rosbag2_bagfile_information']
    counts = {t['topic_metadata']['name']: t['message_count']
              for t in meta['topics_with_message_count']}
    missing = [topic for topic in ALLOWED if counts.get(topic, 0) == 0]
    if missing:
        raise ValueError(f'Missing required bag topics: {missing}')
    wheel = []
    healthy = 0
    health_total = 0
    for relative in meta['relative_file_paths']:
        db = (bag / relative).resolve()
        if db.parent != bag.resolve():
            raise ValueError('Unexpected bag data path')
        with sqlite3.connect(f'file:{db}?mode=ro', uri=True) as con:
            rows = con.execute('SELECT topics.name,messages.data FROM messages '
                               'JOIN topics ON topics.id=messages.topic_id '
                               'WHERE topics.name IN (?,?) ORDER BY messages.timestamp',
                               ('/wheel/odom', '/base/wheel_feedback_healthy'))
            for topic, blob in rows:
                if topic == '/wheel/odom':
                    msg = deserialize_message(blob, Odometry)
                    stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                    wheel.append([stamp, msg.twist.twist.linear.x,
                                  msg.twist.twist.angular.z, msg.twist.covariance[0],
                                  msg.child_frame_id])
                else:
                    healthy += int(deserialize_message(blob, Bool).data)
                    health_total += 1
    finite = all(math.isfinite(value) for row in wheel for value in row[:4])
    bag_start = meta['starting_time']['nanoseconds_since_epoch'] * 1e-9
    segments = []
    active_start = None
    for row in wheel:
        if abs(row[1]) > 0.02 and active_start is None:
            active_start = row[0]
        elif abs(row[1]) <= 0.02 and active_start is not None:
            if row[0] - active_start >= 0.5:
                segments.append([active_start - bag_start, row[0] - bag_start])
            active_start = None
    if active_start is not None:
        segments.append([active_start - bag_start, wheel[-1][0] - bag_start])
    return {
        'bag': str(bag), 'duration_sec': meta['duration']['nanoseconds'] * 1e-9,
        'bag_start_time_sec': bag_start,
        'forward_motion_segments_relative_sec': segments,
        'allowed_topic_counts': {name: counts[name] for name in ALLOWED},
        'wheel_max_abs_linear_mps': max(abs(row[1]) for row in wheel),
        'wheel_max_abs_angular_rps': max(abs(row[2]) for row in wheel),
        'wheel_nonzero_linear_samples': sum(abs(row[1]) > 0.02 for row in wheel),
        'wheel_nonzero_motion_samples': sum(abs(row[1]) > 0.02 or abs(row[2]) > 0.02
                                            for row in wheel),
        'wheel_positive_variance_samples': int(sum(row[3] > 0.0 for row in wheel)),
        'wheel_child_frames': sorted(set(row[4] for row in wheel)),
        'wheel_samples_finite': finite,
        'healthy_samples': healthy, 'health_samples': health_total,
    }, wheel


def inspect_headers(bag, start_offset, bag_start):
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import Imu, PointCloud2
    from nav_msgs.msg import Odometry
    meta = yaml.safe_load((bag / 'metadata.yaml').read_text())['rosbag2_bagfile_information']
    result = {}
    lower = int((bag_start + start_offset) * 1e9)
    for relative in meta['relative_file_paths']:
        with sqlite3.connect(f'file:{bag / relative}?mode=ro', uri=True) as con:
            for topic, kind in [(ALLOWED[0], PointCloud2), (ALLOWED[1], Imu), (ALLOWED[2], Odometry)]:
                row = con.execute('SELECT messages.timestamp,messages.data FROM messages '
                                  'JOIN topics ON topics.id=messages.topic_id '
                                  'WHERE topics.name=? AND messages.timestamp>=? '
                                  'ORDER BY messages.timestamp LIMIT 1', (topic, lower)).fetchone()
                if row is not None and topic not in result:
                    msg = deserialize_message(row[1], kind)
                    stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                    result[topic] = {'record_sec': row[0] * 1e-9, 'header_sec': stamp,
                                     'record_minus_header_sec': row[0] * 1e-9 - stamp}
                    if topic == '/imu/data':
                        a, w = msg.linear_acceleration, msg.angular_velocity
                        result[topic]['accel_norm_mps2'] = math.sqrt(a.x*a.x + a.y*a.y + a.z*a.z)
                        result[topic]['gyro_norm_rps'] = math.sqrt(w.x*w.x + w.y*w.y + w.z*w.z)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', type=Path, default=DEFAULT_BAG)
    parser.add_argument('--domain', type=int, choices=[91, 92], default=92)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--inspect-only', action='store_true')
    parser.add_argument('--start-offset', type=float, default=0.0)
    parser.add_argument('--play-seconds', type=float)
    parser.add_argument('--disable-wheel-aiding', action='store_true',
                        help='Matched A/B control; leave every other parameter unchanged')
    parser.add_argument('--disable-zupt-only', action='store_true')
    args = parser.parse_args()
    # Global read-only precondition protects hardcoded FAST Log outputs and hardware.
    ps = subprocess.check_output(['ps', '-eo', 'pid,args'], text=True)
    conflicting = [line for line in ps.splitlines() if any(token in line for token in
                   ('/fastlio_mapping ', '/xtm60_adapter_node ', '/imu_adapter_node ',
                    '/zlac8030_driver_node ', 'ros2 bag play '))]
    if conflicting:
        raise RuntimeError('Existing mapping/hardware/player process: '+repr(conflicting))
    bag = args.bag.resolve()
    preflight, bag_wheel = inspect_bag(bag)
    preflight['first_headers_at_requested_offset'] = inspect_headers(
        bag, args.start_offset, preflight['bag_start_time_sec'])
    print(json.dumps(preflight, ensure_ascii=False, indent=2), flush=True)
    if args.inspect_only:
        return 0
    if not math.isfinite(args.start_offset) or args.start_offset < 0.0:
        raise ValueError('start-offset must be finite and nonnegative')
    remaining = preflight['duration_sec'] - args.start_offset
    play_seconds = args.play_seconds if args.play_seconds is not None else remaining
    if not math.isfinite(play_seconds) or not 0.0 < play_seconds <= min(180.0, remaining):
        raise ValueError('Select a nonempty <=180-second recording window')
    output = (args.output or ROOT / 'auto_test' /
              time.strftime('wheel_aiding_replay_%Y%m%d_%H%M%S')).resolve()
    if output.exists():
        raise ValueError('Evidence output already exists; refusing overwrite')
    output.mkdir(parents=True)
    log_root = ROOT / 'src/third_party/FAST_LIO_ROS2/Log'
    original_logs = output / 'original_fast_logs'
    shutil.copytree(log_root, original_logs)
    original_hashes = {p.name: hashlib.sha256(p.read_bytes()).hexdigest()
                       for p in log_root.iterdir() if p.is_file()}
    (output / 'original_log_hashes.json').write_text(json.dumps(original_hashes, indent=2))
    os.environ['ROS_DOMAIN_ID'] = str(args.domain)
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    os.environ['ROS_LOG_DIR'] = str(output / 'ros_logs')
    os.environ['RCUTILS_COLORIZED_OUTPUT'] = '0'
    from ament_index_python.packages import get_package_prefix
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Imu, PointCloud2
    from nav_msgs.msg import Odometry
    from std_msgs.msg import Bool
    from rosgraph_msgs.msg import Clock
    from wheelchair_3d_mapping.diagnostic_layout import load_diagnostic_layout

    config_path = ROOT / 'src/wheelchair_3d_mapping/config/xtm60_right_lio.yaml'
    params = yaml.safe_load(config_path.read_text())
    mapping_params = params['/**']['ros__parameters']
    layout = load_diagnostic_layout(
        ROOT / 'src/wheelchair_description/config/diagnostic_measured_layout.yaml')
    mapping_params['use_sim_time'] = True
    mapping_params['wheel_update'] = {
        'enabled': not args.disable_wheel_aiding, 'base_to_imu_R': layout['base_to_imu_R'],
        'base_to_imu_T': layout['base_to_imu_T'],
    }
    # NEW capture uses this exact same diagnostic measured layout.
    mapping_params['mapping']['extrinsic_R'] = layout['extrinsic_R']
    mapping_params['mapping']['extrinsic_T'] = layout['extrinsic_T']
    mapping_params['zupt']['enabled'] = not (args.disable_wheel_aiding or args.disable_zupt_only)
    if max(abs(a-b) for a,b in zip(layout['extrinsic_T'], [.449686,-.297159,.288450])) > 1e-6:
        raise RuntimeError('Current layout differs from NEW capture log')
    (output / 'exact_layout.json').write_text(json.dumps(layout, indent=2))
    param_file = output / 'replay_fastlio_params.yaml'
    param_file.write_text(yaml.safe_dump(params, sort_keys=False), encoding='utf-8')
    adapter_params = {'/**': {'ros__parameters': {
        'use_sim_time': True, 'input_topic': ALLOWED[0],
        'output_topic': '/lio/cloud_in', 'restamp_to_now': False,
        'add_zero_time_field': False, 'min_range': 0.3, 'max_range': 12.0,
        'output_qos': 'best_effort',
    }}}
    adapter_file = output / 'replay_adapter_params.yaml'
    adapter_file.write_text(yaml.safe_dump(adapter_params), encoding='utf-8')
    (output / 'preflight.json').write_text(json.dumps(preflight, indent=2), encoding='utf-8')
    stats = {'topic_counts': {}, 'odom_nonfinite': 0, 'odom_first_stamp': None,
             'odom_last_stamp': None, 'odom_first_position': None,
             'odom_last_position': None, 'odom_min_position': None,
             'odom_max_position': None, 'odom_max_inter_message_gap_sec': 0.0}
    odom_samples = []
    processes = []
    clock_samples = []
    handles = []
    exit_before_cleanup = {}
    player_window_stopped = False
    error = None
    started = time.monotonic()
    rclpy.init(args=[])
    node = rclpy.create_node('wheel_aiding_replay_observer')

    def count(topic):
        stats['topic_counts'][topic] = stats['topic_counts'].get(topic, 0) + 1

    def observe_odom(msg):
        count('/Odometry')
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        values = [p.x, p.y, p.z, q.x, q.y, q.z, q.w,
                  msg.twist.twist.linear.x, msg.twist.twist.linear.y,
                  msg.twist.twist.linear.z] + list(msg.pose.covariance)
        if not all(math.isfinite(v) for v in values):
            stats['odom_nonfinite'] += 1
        position = [p.x, p.y, p.z]
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        tw = msg.twist.twist
        odom_samples.append([time.time(),stamp,p.x,p.y,p.z,q.x,q.y,q.z,q.w,
                             tw.linear.x,tw.linear.y,tw.linear.z,
                             tw.angular.x,tw.angular.y,tw.angular.z])
        if stats['odom_first_stamp'] is None:
            stats['odom_first_stamp'] = stamp
            stats['odom_first_position'] = position
            stats['odom_min_position'] = position[:]
            stats['odom_max_position'] = position[:]
        if stats['odom_last_stamp'] is not None:
            stats['odom_max_inter_message_gap_sec'] = max(
                stats['odom_max_inter_message_gap_sec'], stamp - stats['odom_last_stamp'])
        stats['odom_last_stamp'] = stamp
        stats['odom_last_position'] = position
        stats['odom_min_position'] = list(map(min, stats['odom_min_position'], position))
        stats['odom_max_position'] = list(map(max, stats['odom_max_position'], position))

    def observe_clock(msg):
        count('/clock')
        stats['last_clock_sec'] = msg.clock.sec + msg.clock.nanosec * 1e-9
        clock_samples.append((time.time(), stats['last_clock_sec']))

    def observe_input(topic, msg):
        count(topic)
        if hasattr(msg, 'header'):
            stats.setdefault('last_input_stamps', {})[topic] = (
                msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)

    subscriptions = [node.create_subscription(Odometry, '/Odometry', observe_odom,
                                               qos_profile_sensor_data)]
    subscriptions.append(node.create_subscription(Clock, '/clock', observe_clock,
                                                   qos_profile_sensor_data))
    for topic, kind in [(ALLOWED[0], PointCloud2), (ALLOWED[1], Imu),
                        (ALLOWED[2], Odometry), (ALLOWED[3], Bool),
                        ('/lio/cloud_in', PointCloud2), ('/cloud_registered', PointCloud2)]:
        subscriptions.append(node.create_subscription(
            kind, topic, lambda msg, topic=topic: observe_input(topic, msg), qos_profile_sensor_data))

    def launch(label, command):
        handle = (output / f'{label}.log').open('w', encoding='utf-8')
        handles.append(handle)
        proc = subprocess.Popen(command, stdout=handle, stderr=subprocess.STDOUT,
                                start_new_session=True, env=os.environ.copy())
        processes.append((label, proc))
        print(f'Started own {label} PID={proc.pid}', flush=True)
        return proc

    def spin_for(seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)

    try:
        spin_for(1.0)
        existing = [(name, ns) for name, ns in node.get_node_names_and_namespaces()
                    if name != node.get_name()]
        if existing:
            raise RuntimeError(f'Isolated domain is occupied: {existing}')
        fast = Path(get_package_prefix('fast_lio')) / 'lib/fast_lio/fastlio_mapping'
        adapter = Path(get_package_prefix('wheelchair_3d_mapping')) / 'lib/wheelchair_3d_mapping/lio_cloud_adapter'
        launch('adapter', [str(adapter), '--ros-args', '--params-file', str(adapter_file)])
        launch('fastlio', [str(fast), '--ros-args', '--params-file', str(param_file)])
        spin_for(2.0)
        stats['fastlio_subscriptions_at_start'] = node.get_subscriber_names_and_types_by_node(
            'laser_mapping', '/')
        play_command = ['ros2', 'bag', 'play', str(bag), '--clock', '100',
                        '--disable-keyboard-controls', '--read-ahead-queue-size', '2000',
                        '--start-offset', str(args.start_offset),
                        '--topics'] + ALLOWED
        (output / 'player_command.json').write_text(json.dumps(play_command, indent=2))
        player = launch('player', play_command)
        playback_started = time.monotonic()
        next_report = time.monotonic() + 10.0
        while player.poll() is None:
            if time.monotonic() - started > play_seconds + 25.0:
                raise TimeoutError('Bounded playback deadline; stop only owned children')
            for label, proc in processes:
                if label != 'player' and proc.poll() is not None:
                    raise RuntimeError(f'{label} exited unexpectedly: {proc.returncode}')
            if args.play_seconds is not None and time.monotonic() - playback_started >= play_seconds:
                # Intentional bounded slice of a longer bag, not an error and
                # not a claim that the complete route was replayed.
                os.killpg(player.pid, signal.SIGINT)
                player_window_stopped = True
                player.wait(timeout=3.0)
                break
            rclpy.spin_once(node, timeout_sec=0.1)
            if time.monotonic() >= next_report:
                print(f'Elapsed={time.monotonic()-started:.1f}s counts={stats["topic_counts"]}', flush=True)
                next_report += 10.0
        if player.returncode != 0 and not player_window_stopped:
            raise RuntimeError(f'Bag player failed: {player.returncode}')
        spin_for(3.0)
    except (Exception, KeyboardInterrupt) as exc:
        error = f'{type(exc).__name__}: {exc}'
    finally:
        # These process groups were created by this script; never pkill or touch
        # a hardware node, another domain, another launch, or a user's terminal.
        for label, proc in reversed(processes):
            exit_before_cleanup[label] = proc.poll()
            if proc.poll() is None:
                os.killpg(proc.pid, signal.SIGINT)
        deadline = time.monotonic() + 5.0
        for label, proc in reversed(processes):
            try:
                proc.wait(timeout=max(0.1, deadline - time.monotonic()))
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGKILL)
                proc.wait(timeout=2.0)
        for handle in handles:
            handle.close()
        node.destroy_node()
        rclpy.shutdown()

    log = (output / 'fastlio.log').read_text(errors='replace') if (output / 'fastlio.log').exists() else ''
    (output / 'odom_samples.json').write_text(json.dumps(odom_samples))
    generated_logs = output / 'generated_fast_logs'
    shutil.copytree(log_root, generated_logs)
    generated_hashes = {p.name: hashlib.sha256(p.read_bytes()).hexdigest()
                        for p in log_root.iterdir() if p.is_file()}
    # No live FAST process may appear while restoring originals. Never erase new files.
    ps_after = subprocess.check_output(['ps','-eo','pid,args'],text=True)
    if any('/fastlio_mapping ' in line for line in ps_after.splitlines()):
        raise RuntimeError('Another FAST process appeared; originals retained in backup only')
    for name, old_hash in original_hashes.items():
        current = log_root / name
        if hashlib.sha256(current.read_bytes()).hexdigest() != generated_hashes[name]:
            raise RuntimeError('Concurrent Log change; refusing original restoration')
        shutil.copy2(original_logs / name, current)
    (output / 'log_restoration.json').write_text(json.dumps({
        'original_hashes_restored': all(hashlib.sha256((log_root/n).read_bytes()).hexdigest()==v
                                        for n,v in original_hashes.items()),
        'new_run_logs_preserved': str(generated_logs)},indent=2))
    (output / 'clock_wall_to_bag_time.json').write_text(json.dumps(clock_samples))
    wall_counters = [(float(stamp), int(applied), int(rejected)) for stamp, applied, rejected in
                re.findall(r'\[INFO\] \[([0-9.]+)\].*Wheel aiding: applied=(\d+) rejected=(\d+)', log)]
    # rcutils log headers use wall time even under use_sim_time. Convert only
    # logs adjacent to an actually observed /clock tick; never compare the 2026
    # wall log timestamp directly to the historical bag's message headers.
    counters = []
    for wall_stamp, applied, rejected in wall_counters:
        if not clock_samples:
            continue
        wall_ref, ros_ref = min(clock_samples, key=lambda item: abs(item[0] - wall_stamp))
        if abs(wall_ref - wall_stamp) <= 0.2:
            counters.append((ros_ref + wall_stamp - wall_ref, applied, rejected))
    # A count increase entirely inside an interval of nonzero recorded wheel
    # forward velocity demonstrates actual dynamic integration, not just launch.
    dynamic_increments = 0
    dynamic_intervals = []
    for before, after in zip(counters, counters[1:]):
        # Keep a 150ms margin around both log boundaries so clock delivery jitter
        # cannot turn a stationary-to-moving transition into a false positive.
        rows = [row for row in bag_wheel if before[0] - 0.15 <= row[0] <= after[0] + 0.15]
        if rows and all(abs(row[1]) > 0.02 for row in rows):
            added = max(0, after[1] - before[1])
            dynamic_increments += added
            if added:
                dynamic_intervals.append({'bag_time_start': before[0], 'bag_time_end': after[0],
                                          'applied_increase': added,
                                          'minimum_abs_wheel_mps': min(abs(row[1]) for row in rows)})
    lidar_counters = [(int(accepted), int(rejected)) for accepted, rejected in
                      re.findall(r'accepted=(\d+) rejected=(\d+)', log)]
    last_cloud = stats.get('last_input_stamps', {}).get('/lio/cloud_in')
    output_lag = (last_cloud - stats['odom_last_stamp']
                  if last_cloud is not None and stats['odom_last_stamp'] is not None else None)
    result = {
        'preflight': preflight, 'observations': stats, 'ros_domain_id': args.domain,
        'replayed_topics': ALLOWED, 'published_clock': True,
        'historical_lidar_extrinsics_retained': False,
        'start_offset_sec': args.start_offset,
        'requested_play_seconds': play_seconds,
        'player_stopped_at_requested_window': player_window_stopped,
        'new_layout_supplies_exact_capture_lidar_and_base_imu_extrinsics': True,
        'zupt_enabled': mapping_params['zupt']['enabled'],
        'wheel_aiding_enabled': not args.disable_wheel_aiding,
        'error': error, 'exit_before_cleanup': exit_before_cleanup,
        'owned_process_exit_codes': {name: proc.returncode for name, proc in processes},
        'wheel_applied_max': max((item[1] for item in wall_counters), default=0),
        'wheel_rejected_max': max((item[2] for item in wall_counters), default=0),
        'wheel_applied_during_nonzero_forward_intervals': dynamic_increments,
        'dynamic_intervals_with_clock_mapping_and_150ms_margin': dynamic_intervals,
        'wheel_counter_log_times_mapped_to_bag_clock': counters,
        'lidar_accepted_max': max((row[0] for row in lidar_counters), default=0),
        'lidar_rejected_max': max((row[1] for row in lidar_counters), default=0),
        'last_cloud_minus_last_odom_sec': output_lag,
        'finite_odometry_produced': stats['topic_counts'].get('/Odometry', 0) > 0 and
                                    stats['odom_nonfinite'] == 0,
        'hardware_started': False, 'physical_motion_or_accuracy_validated': False,
        'wall_duration_sec': time.monotonic() - started,
    }
    result['finite_output_smoke_pass'] = (error is None and result['finite_odometry_produced'] and
                                         stats['topic_counts'].get('/cloud_registered', 0) > 0)
    # Merely producing one finite pose does not establish a healthy pipeline.
    # Catch the originally reported "initially updates, then freezes" failure.
    result['continuous_output_pass'] = (result['finite_output_smoke_pass'] and
                                        output_lag is not None and output_lag <= 2.0 and
                                        stats['odom_max_inter_message_gap_sec'] <= 2.0)
    result['regression_pass'] = result['continuous_output_pass']
    result['dynamic_wheel_aiding_proven'] = dynamic_increments > 0
    (output / 'result.json').write_text(json.dumps(result, ensure_ascii=False, indent=2))
    print(json.dumps(result, ensure_ascii=False, indent=2), flush=True)
    print(f'EVIDENCE_OUTPUT={output}', flush=True)
    return 0 if result['regression_pass'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
