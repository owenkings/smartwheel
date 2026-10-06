#!/usr/bin/env python3
"""SYNTHETIC four-camera display load, never a device or sensor-record source.

Only the four allowlisted Image topics in localhost domain 84 are published.
The owning offline dashboard harness holds the domain lock and component lease.
"""
import argparse
from array import array
import json
import math
import os
from pathlib import Path
import signal
import time


ROLES = ('left_front', 'right_front', 'left_side', 'right_side')
TOPICS = {role: '/wc_mapping/cameras/'+role+'/image_raw' for role in ROLES}
NODE_NAME = 'synthetic_dashboard_cameras'
WIDTH, HEIGHT, HZ = 320, 240, 8.


def reviewed_ports(view):
    panels = [item for item in view['Panels'] if item.get('Class') == 'wc_camera_panel/CameraPanel']
    if len(panels) != 1 or panels[0].get('Mapping Status') != 'USER_CONFIRMED':
        raise ValueError('Exactly one reviewed camera panel is required for synthetic display load')
    ports = [panels[0].get(role+' USB Port') for role in ROLES]
    if any(type(port) is not int for port in ports) or sorted(ports) != [1, 2, 3, 4]:
        raise ValueError('Camera display bindings must be a permutation of USB1..USB4')
    return ports


def artificial_bgr(role_index, phase):
    """Hard-edged colour bars/checkers, with alternating white timing stripes."""
    colors = ((0, 0, 255), (0, 255, 0), (255, 0, 0), (0, 255, 255))
    pixels = bytearray(WIDTH*HEIGHT*3)
    for y in range(HEIGHT):
        for x in range(WIDTH):
            color = colors[(x//80+role_index) % 4]
            if ((x//20)+(y//20)) % 2:
                color = tuple(value//3 for value in color)
            if phase*160+60 <= x < phase*160+80:
                color = (255, 255, 255)
            offset = (y*WIDTH+x)*3
            pixels[offset:offset+3] = bytes(color)
    return array('B', pixels)


def write_status(path, value):
    value['status_written_monotonic_ns'] = time.monotonic_ns()
    temporary = path.with_suffix('.partial')
    with temporary.open('w', encoding='utf-8') as stream:
        json.dump(value, stream, ensure_ascii=False, allow_nan=False)
    temporary.replace(path)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--ports', required=True, type=int, nargs=4)
    parser.add_argument('--duration', type=float, default=90.)
    args = parser.parse_args(argv)
    if os.name != 'posix' or os.environ.get('ROS_DOMAIN_ID') != '84' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        parser.error('Requires the target Linux environment and explicit localhost ROS domain 84')
    if sorted(args.ports) != [1, 2, 3, 4] or not math.isfinite(args.duration) or not 0 < args.duration <= 90:
        parser.error('Requires four unique display ports and a duration in (0, 90] seconds')
    from wc_runtime.cli import ROOT
    from check_mapping_app_export import contained
    output = contained(ROOT, args.output)
    if not output.is_relative_to(ROOT/'reports') or not output.parent.is_dir() or output.exists() or output.with_suffix('.partial').exists():
        raise ValueError('Synthetic evidence requires a new JSON path inside an existing reports directory')
    result = {'validation_level': 'SYNTHETIC_CAMERA_DISPLAY_ONLY', 'status': 'STARTING',
              'synthetic': True, 'domain_id': 84, 'pid': os.getpid(), 'node_name': NODE_NAME,
              'duration_limit_s': args.duration, 'requested_hz_per_stream': HZ,
              'width': WIDTH, 'height': HEIGHT, 'encoding': 'bgr8',
              'hardware_started': False, 'source_frames_published': False,
              'tf_published': False, 'control_published': False,
              'streams': {role: {'topic': TOPICS[role], 'frame_id': 'camera_usb3_'+str(port),
                  'published': 0, 'matched_subscribers': 0, 'max_matched_subscribers': 0,
                  'first_published_monotonic_ns': None, 'last_published_monotonic_ns': None}
                  for role, port in zip(ROLES, args.ports)}}
    stopped = [False]
    for number in (signal.SIGINT, signal.SIGTERM):
        signal.signal(number, lambda *_: stopped.__setitem__(0, True))
    node = executor = None
    ros_started = False
    try:
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.signals import SignalHandlerOptions
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        from sensor_msgs.msg import Image
        rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO); ros_started = True
        node = rclpy.create_node(NODE_NAME, namespace='/wc_mapping/offline')
        executor = SingleThreadedExecutor(); executor.add_node(node)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)
        publishers = {role: node.create_publisher(Image, TOPICS[role], qos) for role in ROLES}
        images = {}
        for index, role in enumerate(ROLES):
            images[role] = []
            for phase in range(2):
                message = Image()
                message.header.frame_id = result['streams'][role]['frame_id']
                message.width, message.height, message.step = WIDTH, HEIGHT, WIDTH*3
                message.encoding, message.is_bigendian = 'bgr8', 0
                message.data = artificial_bgr(index, phase)
                images[role].append(message)
        start = time.monotonic()
        deadline = start+args.duration

        def publish():
            for role in ROLES:
                if stopped[0] or time.monotonic() >= deadline:
                    return
                row = result['streams'][role]
                message = images[role][(row['published']//4) % 2]
                message.header.stamp = node.get_clock().now().to_msg()
                now_ns = time.monotonic_ns()
                publishers[role].publish(message)
                row['published'] += 1
                row['first_published_monotonic_ns'] = row['first_published_monotonic_ns'] or now_ns
                row['last_published_monotonic_ns'] = now_ns

        timer = node.create_timer(1./HZ, publish)
        result['status'] = 'RUNNING'
        last_report = 0.
        while not stopped[0] and rclpy.ok() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.02)
            if time.monotonic()-last_report >= .5:
                for role, publisher in publishers.items():
                    row = result['streams'][role]
                    row['matched_subscribers'] = publisher.get_subscription_count()
                    row['max_matched_subscribers'] = max(row['max_matched_subscribers'], row['matched_subscribers'])
                write_status(output, result)
                last_report = time.monotonic()
        node.destroy_timer(timer)
        result['elapsed_publication_loop_s'] = time.monotonic()-start
        result['stopped_by_signal'] = stopped[0]
        result['status'] = 'PASS'
    except Exception as error:
        result['status'] = 'FAIL'
        result['error'] = type(error).__name__+': '+str(error)
    finally:
        try:
            if executor is not None:
                executor.remove_node(node); executor.shutdown(timeout_sec=1.)
            if node is not None:
                node.destroy_node()
            if ros_started and rclpy.ok():
                rclpy.shutdown()
        except Exception as error:
            result['status'] = 'FAIL'; result['cleanup_error'] = str(error)
        for row in result['streams'].values():
            row['published_span_s'] = 0. if row['published'] < 2 else (
                row['last_published_monotonic_ns']-row['first_published_monotonic_ns'])/1e9
            row['observed_publish_hz'] = ((row['published']-1)/row['published_span_s']
                if row['published_span_s'] > 0 else None)
        result['final'] = True
        write_status(output, result)
        print(json.dumps(result, ensure_ascii=False, allow_nan=False), flush=True)
    return 0 if result['status'] == 'PASS' else 1


if __name__ == '__main__':
    raise SystemExit(main())
