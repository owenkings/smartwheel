#!/usr/bin/env python3
"""Exercise real RViz windows using ROS messages only; never starts hardware.

Run under the sourced target install and an owned Xvfb/desktop display:
  python3 tests/panel_rviz/run_synthetic.py --project-root /home/nvidia/wheelchair \
    --output-root <new configured-storage directory>
Outputs include real screenshots, subscriber/display observations and a report.
"""
from __future__ import annotations
import argparse
import json
import os
from pathlib import Path
import re
import struct
import subprocess
import sys
import threading
import time


def cloud(frame, count=1200):
    from sensor_msgs.msg import PointCloud2, PointField
    value = PointCloud2()
    value.header.frame_id = frame
    value.height = 1
    value.width = count
    value.fields = [PointField(name=name, offset=i * 4, datatype=PointField.FLOAT32, count=1)
                    for i, name in enumerate(('x', 'y', 'z'))]
    value.point_step = 12
    value.row_step = count * 12
    value.is_dense = True
    value.data = b''.join(struct.pack('<fff', (i % 40) * .09, (i // 40) * .09 - 1.3,
                                     .2 + (i % 6) * .12) for i in range(count))
    return value


def verify_layout(report, mode):
    widgets = {x['name']: x for x in report['widgets']}
    expected = (['mapping_left_column', 'mapping_center_column', 'mapping_right_column']
                if mode == 'mapping' else
                ['capture_status_column', 'capture_left_column', 'capture_right_column'])
    rows = [widgets[name] for name in expected]
    assert all(row['visible'] and row['width'] >= 180 for row in rows), rows
    assert rows[0]['x'] + rows[0]['width'] <= rows[1]['x'], rows
    assert rows[1]['x'] + rows[1]['width'] <= rows[2]['x'], rows
    runtime = report['runtime']
    if mode == 'capture':
        assert all(x['received'] >= 3 and x['points'] > 0 for x in runtime['lidars']), runtime
        assert all(x['frames'] > 0 for x in runtime['views']), runtime
        assert len(runtime['views']) == 2, runtime
        assert all(x['independent_view'] for x in runtime['views']), runtime
        assert [x['color_materials_retained'] for x in runtime['views']] == [0, 8], runtime
        native_views = runtime['views']
        display_sets = [view['displays'] for view in runtime['views']]
    else:
        assert runtime['frames'] > 0, runtime
        assert len(runtime['secondary_views']) == 1, runtime
        assert runtime['secondary_views'][0]['frames'] > 0, runtime
        assert runtime['secondary_views'][0]['independent_view'], runtime
        assert runtime['secondary_views'][0]['color_materials_retained'] == 8, runtime
        fit = runtime['secondary_views'][0]['grid_fit']
        assert fit['auto_fit_done'] and fit['bounds_valid'] and fit['fit_count'] == 1, fit
        assert abs(fit['center_x'] - 4) < 1e-4 and abs(fit['center_y'] - 2.5) < 1e-4, fit
        assert abs(fit['width_m'] - 8) < 1e-4 and abs(fit['height_m'] - 5) < 1e-4, fit
        map_native = runtime['secondary_views'][0]['native_window']
        assert fit['width_m'] * fit['scale'] * 1.1 <= map_native['ogre_width'] + 1, (fit, map_native)
        assert fit['height_m'] * fit['scale'] * 1.1 <= map_native['ogre_height'] + 1, (fit, map_native)
        assert len(runtime['camera_states']) == 4, runtime
        for status in runtime['camera_states']:
            match = re.search(r'有效\s+(\d+)', status['text'])
            assert match and int(match[1]) > 0, status
        for side in ('left', 'right'):
            a, b = widgets['camera_' + side + '_front'], widgets['camera_' + side + '_side']
            assert a['visible'] and b['visible'] and a['y'] < b['y'], (a, b)
        display_sets = [runtime['displays'], runtime['secondary_views'][0]['displays']]
        native_views = runtime['secondary_views']
    for view in native_views:
        native = view['native_window']
        assert native['initialized'] and native['visible'] and native['exposed'], native
        assert native['window_id_stable'] and native['parent_id_stable'], native
        assert native['width'] > 0 and native['height'] > 0, native
        assert native['ogre_width'] > 0 and native['ogre_height'] > 0, native
    # RViz status proves that the display itself processed a message, in addition
    # to the independent point counters. A renderer screenshot still needs review.
    for displays in display_sets:
        active = [display for display in displays if display['enabled'] and display['class'] in
                  ('rviz_default_plugins/PointCloud2', 'rviz_default_plugins/Map')]
        assert active, displays
        for display in active:
            text = ' '.join(item['value'] for item in display['status'])
            assert display['status'], display
            assert all(item['level'] == 0 for item in display['status']), display
            assert 'No messages received' not in text and 'Could not' not in text, display


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--project-root', type=Path, required=True)
    parser.add_argument('--install-root', type=Path, help='Explicit isolated install; defaults to project install/main')
    parser.add_argument('--output-root', type=Path, required=True)
    parser.add_argument('--domain', type=int, default=217)
    args = parser.parse_args()
    assert 1 <= args.domain <= 232
    root = args.project_root.resolve(strict=True)
    output = args.output_root.absolute()
    if output.exists() or output.is_symlink():
        raise ValueError('Use a new test output directory')
    output.mkdir(parents=True)
    os.environ['ROS_DOMAIN_ID'] = str(args.domain)
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    import yaml
    import rclpy
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
    from sensor_msgs.msg import Image, PointCloud2
    from nav_msgs.msg import OccupancyGrid
    rclpy.init()
    node = rclpy.create_node('wc_panel_synthetic_only')
    reliable = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                          reliability=ReliabilityPolicy.RELIABLE)
    cloud_publishers = []
    for side in ('left', 'right'):
        topic = '/wc_mapping/lidar_' + side + '/points_filtered'
        cloud_publishers.append((node.create_publisher(PointCloud2, topic, 2), cloud('lidar_' + side)))
    for topic in ('/wc_mapping/app/scan_cloud', '/wc_mapping/app/cloud_map'):
        cloud_publishers.append((node.create_publisher(PointCloud2, topic, reliable), cloud('mapping_map')))
    grid = OccupancyGrid()
    grid.header.frame_id = 'mapping_map'
    grid.info.resolution = .05
    grid.info.width, grid.info.height = 160, 100
    grid.info.origin.orientation.w = 1.0
    grid.data = [100 if x < 3 or y < 3 or x > 155 or y > 95 else
                 (100 if 70 < x < 75 and y < 75 else 0) for y in range(100) for x in range(160)]
    grid_publisher = node.create_publisher(OccupancyGrid, '/wc_mapping/app/grid_map', reliable)
    images = []
    roles = ('left_front', 'right_front', 'left_side', 'right_side')
    for index, role in enumerate(roles):
        message = Image()
        message.header.frame_id = 'camera_usb3_' + str(index + 1)
        message.width, message.height = 320, 240
        message.encoding, message.step = 'rgb8', 960
        color = ((180, 50, 40), (30, 150, 70), (40, 70, 200), (180, 130, 30))[index]
        message.data = bytes(channel for y in range(240) for x in range(320)
                             for channel in (color if (x // 32 + y // 24) % 2 else (220, 220, 220)))
        images.append((node.create_publisher(Image, '/wc_mapping/cameras/' + role + '/image_raw', 1), message))
    stopping = threading.Event()
    def publish():
        while not stopping.is_set():
            stamp = node.get_clock().now().to_msg()
            for publisher, message in cloud_publishers + images:
                message.header.stamp = stamp
                publisher.publish(message)
            grid.header.stamp = stamp
            grid_publisher.publish(grid)
            stopping.wait(.15)
    worker = threading.Thread(target=publish)
    worker.start()
    outcomes = []
    try:
        mapping = yaml.safe_load((root / 'config/rviz/mapping_live.rviz').read_text())
        manager = mapping['Visualization Manager']
        manager['Displays'] = [d for d in manager['Displays'] if d.get('Class') in
                               ('rviz_default_plugins/PointCloud2', 'rviz_default_plugins/Map')]
        for display in manager['Displays']:
            if display['Class'] == 'rviz_default_plugins/Map':
                display['Topic']['Value'] = '/wc_mapping/app/grid_map'
        camera_panel = {'Class': 'wc_camera_panel/CameraPanel', 'Name': 'Cameras',
                        'Compact Layout': True, 'Mapping Status': 'USER_CONFIRMED'}
        camera_panel.update({role + ' USB Port': i + 1 for i, role in enumerate(roles)})
        mapping['Panels'].append(camera_panel)
        (output / 'view.rviz').write_text(yaml.safe_dump(mapping, allow_unicode=True), encoding='utf-8')
        (output / 'session.json').write_text(json.dumps({'session_id': 'synthetic_panel_test',
                                                       'source_mode': 'synthetic'}))
        (output / 'runtime_config.json').write_text(json.dumps({'mapping_enabled': True, 'mode': 'all',
            'cloud_source': 'filtered', 'odometry_source': 'synthetic', 'motion_model': 'synthetic'}))
        binary = (args.install_root or root/'install/main')/'wc_bringup/lib/wc_bringup'
        calls = [('capture', [str(binary / 'unified_capture_rviz'), '--read-only', '--left-config',
                  str(root / 'config/rviz/sensor_left.rviz'), '--right-config',
                  str(root / 'config/rviz/sensor_right.rviz')]),
                 ('mapping', [str(binary / 'mapping_rviz'), '-d', str(output / 'view.rviz'),
                              '--ros-args', '-r', '/tf:=/wc_mapping/app/tf', '-r',
                              '/tf_static:=/wc_mapping/app/tf_static'])]
        for mode, command in calls:
            env = dict(os.environ, WC_PANEL_LAYOUT='unified', WC_PANEL_TEST_EXIT_MS='8000',
                       WC_PANEL_SCREENSHOT=str(output / (mode + '.png')),
                       WC_PANEL_STATE_JSON=str(output / (mode + '.json')))
            with (output / (mode + '.log')).open('wb') as logfile:
                completed = subprocess.run(command, env=env, stdout=logfile, stderr=subprocess.STDOUT, timeout=45)
            assert completed.returncode == 0, (mode, completed.returncode)
            log_text = (output / (mode + '.log')).read_text(errors='replace')
            assert 'failed to create drawable' not in log_text, (mode, 'GLX drawable creation failed')
            assert 'GL_INVALID_FRAMEBUFFER_OPERATION' not in log_text, (mode, 'Invalid framebuffer')
            observation = json.loads((output / (mode + '.json')).read_text())
            verify_layout(observation, mode)
            assert (output / (mode + '.png')).stat().st_size > 10000
            outcomes.append({'mode': mode, 'status': 'PASS', 'level': 'SYNTHETIC',
                             'screenshot': str(output / (mode + '.png')), 'visual_review': 'PENDING'})
    finally:
        stopping.set()
        worker.join(3)
        node.destroy_node()
        rclpy.shutdown()
        (output / 'result.json').write_text(json.dumps(outcomes, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps(outcomes, ensure_ascii=False))


if __name__ == '__main__':
    main()
