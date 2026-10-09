#!/usr/bin/env python3
"""Read-only bounded four-topic ROS observation. No device open or image storage."""
import argparse
import json
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'src'))
from wc_cameras.config import load_config, number, topic
from wc_cameras.node import diagnostic_level_number


def diagnostic_identity_matches(values, config, digest, camera):
    role = camera['role']
    expected = {'mapping_status': config['mapping_status'], 'mapping_note': config['mapping_note'],
                'config_sha256': digest, 'logical_slot': role, 'topic': topic(role),
                'frame_id': 'camera_usb3_'+str(camera['port']),
                'physical_direction': role if config['mapping_status'] == 'USER_CONFIRMED' else 'UNKNOWN',
                'rotation_status': 'CONFIG_ONLY', 'time_source': 'arrival_only',
                'common_time_valid': False, 'tf_published': False, 'camera_info': 'UNAVAILABLE'}
    return all(key in values and values[key] == value for key, value in expected.items())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=ROOT / 'config/cameras.json')
    parser.add_argument('--duration', type=float, default=15)
    parser.add_argument('--min-frames', type=int, default=20)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    number(args.duration, 'duration', 2, 120)
    number(args.min_frames, 'min-frames', 1, 1800)
    config, digest = load_config(args.config)
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from sensor_msgs.msg import Image
    from diagnostic_msgs.msg import DiagnosticArray

    report = {'verification_level': 'LIVE_STATIC_ROS_OBSERVATION', 'images_saved': False,
              'config_sha256': digest, 'mapping_status': config['mapping_status'],
              'mapping_note': config['mapping_note'],
              'physical_directions_user_confirmed': config['mapping_status'] == 'USER_CONFIRMED',
              'physical_directions_verified': False, 'visual_quality_verified': False,
              'synchronization_verified': False, 'measurement_latency_measured': False,
              'started_ns': time.time_ns(), 'cameras': {}, 'errors': []}
    rclpy.init(args=[])
    node = Node('camera_observation_check', namespace='/wc_mapping/tests')
    subscriptions = []
    started_mono = time.monotonic_ns()
    for camera in config['cameras']:
        role = camera['role']
        row = report['cameras'][role] = {'port': camera['port'], 'topic': topic(role), 'frames': 0,
            'expected_frame_id': 'camera_usb3_'+str(camera['port']), 'first_receive_mono_ns': None,
            'last_receive_mono_ns': None, 'last_image_stamp_ns': None, 'last_shape': None,
            'latest_diagnostic': None, 'max_host_arrival_to_observer_ns': None}

        def on_image(message, row=row, role=role):
            now_wall, now_mono = time.time_ns(), time.monotonic_ns()
            stamp = message.header.stamp.sec*1_000_000_000+message.header.stamp.nanosec
            errors = []
            if message.header.frame_id != row['expected_frame_id']:
                errors.append('port_frame_id_mismatch')
            if message.encoding != 'bgr8' or message.step != message.width*3 or len(message.data) != message.step*message.height:
                errors.append('invalid_bgr8_payload')
            if row['last_image_stamp_ns'] is not None and stamp <= row['last_image_stamp_ns']:
                errors.append('duplicate_or_backwards_arrival_stamp')
            if now_wall-stamp < -100_000_000 or now_wall-stamp > 3_000_000_000:
                errors.append('stale_or_future_arrival_stamp')
            for error in errors:
                label = role+':'+error
                if label not in report['errors']:
                    report['errors'].append(label)
            row['frames'] += 1
            row['first_receive_mono_ns'] = row['first_receive_mono_ns'] or now_mono
            row['last_receive_mono_ns'] = now_mono
            row['last_image_stamp_ns'] = stamp
            row['last_shape'] = [message.height, message.width, 3]
            row['max_host_arrival_to_observer_ns'] = max(row['max_host_arrival_to_observer_ns'] or 0, now_wall-stamp)

        def on_diagnostic(message, row=row):
            if message.status:
                status = message.status[0]
                values = {}
                for item in status.values:
                    try:
                        values[item.key] = json.loads(item.value)
                    except ValueError:
                        values[item.key] = item.value
                row['latest_diagnostic'] = {'level': diagnostic_level_number(status.level), 'message': status.message, 'values': values}

        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=2, reliability=ReliabilityPolicy.BEST_EFFORT)
        subscriptions.append(node.create_subscription(Image, topic(role), on_image, qos))
        subscriptions.append(node.create_subscription(DiagnosticArray, '/wc_mapping/cameras/'+role+'/diagnostics', on_diagnostic, 10))
    try:
        while time.monotonic_ns()-started_mono < args.duration*1e9:
            rclpy.spin_once(node, timeout_sec=.1)
        finished_mono = time.monotonic_ns()
        for role, row in report['cameras'].items():
            if row['frames'] < args.min_frames:
                report['errors'].append(role+':insufficient_frames')
            if row['last_receive_mono_ns'] is None or finished_mono-row['last_receive_mono_ns'] > 2e9:
                report['errors'].append(role+':not_fresh_at_end')
            diag = row['latest_diagnostic']
            if diag is None or diag['values'].get('failure_latched') is not False:
                report['errors'].append(role+':missing_or_failed_diagnostics')
            elif not diagnostic_identity_matches(diag['values'], config, digest,
                                                 next(camera for camera in config['cameras'] if camera['role'] == role)):
                report['errors'].append(role+':diagnostic_identity_or_config_mismatch')
            span = (row['last_receive_mono_ns'] or 0)-(row['first_receive_mono_ns'] or 0)
            row['observed_hz'] = (row['frames']-1)*1e9/span if span > 0 else None
        report['finished_ns'] = time.time_ns()
        report['status'] = 'PASS' if not report['errors'] else 'FAIL'
        report['simultaneous_four_topics_observed'] = not report['errors']
        text = json.dumps(report, indent=2, allow_nan=False)
        if args.output:
            with args.output.open('x', encoding='utf-8') as stream:
                stream.write(text+'\n')
        print(text)
        return 0 if report['status'] == 'PASS' else 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
