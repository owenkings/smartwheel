#!/usr/bin/env python3
"""Open actual Orin RViz, verify four panel subscriptions, capture owned window."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'tests/integration'))
from check_rviz_display import desktop_environment, capture_owned_window


def main():
    from wc_runtime.cli import target
    from wc_cameras.config import ROLES, load_config, topic
    import rclpy
    from rclpy.node import Node
    target()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', type=Path, required=True)
    args = parser.parse_args()
    output = args.output_dir.resolve()
    if not output.is_relative_to(ROOT/'reports') or output.exists():
        raise ValueError('New project reports directory required')
    output.mkdir(parents=True)
    if os.environ.get('ROS_DOMAIN_ID') != '83':
        raise RuntimeError('Camera preview uses project domain 83')
    env = desktop_environment()
    config, digest = load_config(ROOT/'config/cameras.json')
    result = {'status': 'FAIL', 'source_mode': 'real', 'mapping_status': config['mapping_status'],
              'mapping_note': config['mapping_note'], 'config_sha256': digest,
              'physical_directions_user_confirmed': config['mapping_status'] == 'USER_CONFIRMED',
              'physical_directions_verified': False, 'visual_review': 'PENDING',
              'map_quality_tested': False, 'subscriptions': {}}
    child = None
    fd = None
    rclpy.init(args=[])
    node = Node('wc_camera_panel_observer')
    with (output/'rviz.log').open('x') as log:
        try:
            child = subprocess.Popen([sys.executable, '-m', 'wc_runtime.component', '--parent', str(os.getpid()),
                '--', 'rviz2', '-d', str(ROOT/'config/rviz/cameras.rviz'),
                '--ros-args', '-r', '__node:=wc_camera_rviz_test'], env=env,
                stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            fd = os.pidfd_open(child.pid)
            ready = None
            end = time.monotonic()+35
            while time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=.1)
                if child.poll() is not None:
                    raise RuntimeError('RViz exited before preview verification')
                for role in ROLES:
                    result['subscriptions'][role] = [i.node_name for i in node.get_subscriptions_info_by_topic(topic(role))
                        if i.node_name.startswith('wc_camera_panel_') and i.node_namespace == '/wc_mapping/ui']
                if all(result['subscriptions'].values()):
                    ready = ready or time.monotonic()
                    if time.monotonic()-ready > 7:
                        break
            if not all(result['subscriptions'].values()):
                raise RuntimeError('Native panel did not subscribe to all four camera topics')
            result['capture'] = capture_owned_window(child, output/'four_cameras.png', env)
            result['status'] = 'DDS_AND_WINDOW_CAPTURE_PASS'
        except Exception as error:
            result['error'] = str(error)
        finally:
            if child is not None:
                if child.poll() is None:
                    if fd is not None:
                        signal.pidfd_send_signal(fd, signal.SIGINT)
                    else:
                        child.send_signal(signal.SIGINT)
                try:
                    child.wait(timeout=18)
                    result['rviz_exit_code'] = child.returncode
                    if child.returncode != 0:
                        result['status'] = 'FAIL'
                finally:
                    if fd is not None:
                        os.close(fd)
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
            (output/'result.json').write_text(json.dumps(result, indent=2))
    print(json.dumps(result, indent=2))
    return 0 if result['status'] == 'DDS_AND_WINDOW_CAPTURE_PASS' else 1


if __name__ == '__main__':
    raise SystemExit(main())
