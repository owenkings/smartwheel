"""Current lidar frames in configured axle coordinates, without mapping or recording."""
import argparse
import signal
import time
from pathlib import Path

import numpy as np

from .mapping_visuals import build_scene, marker_array, _rigid, _quaternion


def preview_geometry(config):
    if config.get('mapping_enabled') is not False:
        raise ValueError('Preview requires mapping_enabled=false')
    sides = ('left', 'right') if config['mode'] == 'all' else (config['mode'],)
    if any(side not in config.get('mounts', {}) for side in sides):
        return {'transforms': [], 'scene': None, 'world_tracking': False,
                'reference_basis': 'NATIVE_SENSOR_FRAMES; choose lidar_left/right Fixed Frame; no cross-sensor transform is asserted'}
    transforms = []
    for side in (('left', 'right') if config['mode'] == 'all' else (config['mode'],)):
        mount = config['mounts'][side]
        pose = np.eye(4)
        pose[:3, :3] = mount['R_axle_lidar']; pose[:3, 3] = mount['t_axle_lidar_m']
        pose = _rigid(pose, 'T_axle_'+side)
        transforms.append({'parent': 'mapping_reference', 'child': 'lidar_'+side,
            'translation': pose[:3, 3].tolist(), 'rotation_xyzw': _quaternion(pose[:3, :3])})
    scene = None
    if config.get('visualization'):
        initialization = {'session_id': config['session_id'], 'status': 'EXPERIMENT', 'source_mode': 'real',
            'gravity_input_base_frame': 'lidar_right' if config['mode'] == 'right' else 'lidar_left',
            'T_reference_axle': np.eye(4).tolist()}
        scene = build_scene(config, initialization)
    return {'transforms': transforms, 'scene': scene, 'world_tracking': False,
            'reference_basis': 'configured axle frame; preview follows the chair, no odometry or measured leveling'}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session-root', required=True, type=Path)
    args = parser.parse_args(argv)
    from .cli import ROOT, target
    from .prepare_picker_input import project_path, read_json
    target()
    directory = project_path(ROOT, args.session_root)
    geometry = preview_geometry(read_json(ROOT, directory/'runtime_config.json'))
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from geometry_msgs.msg import TransformStamped
    from tf2_msgs.msg import TFMessage
    from visualization_msgs.msg import Marker, MarkerArray
    stopped = [False]
    previous = {}
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        previous[sig] = signal.signal(sig, lambda *_: stopped.__setitem__(0, True))
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('mapping_preview_reference')
    latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
    tf_publisher = node.create_publisher(TFMessage, '/wc_mapping/app/tf_static', latched)
    markers = node.create_publisher(MarkerArray, '/wc_mapping/app/view_markers', 1)
    transforms = []
    for item in geometry['transforms']:
        transform = TransformStamped()
        transform.header.stamp = node.get_clock().now().to_msg()
        transform.header.frame_id = item['parent']; transform.child_frame_id = item['child']
        t, q = transform.transform.translation, transform.transform.rotation
        t.x, t.y, t.z = item['translation']; q.x, q.y, q.z, q.w = item['rotation_xyzw']
        transforms.append(transform)
    tf_publisher.publish(TFMessage(transforms=transforms))
    last_markers = -1.
    try:
        while not stopped[0] and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=.1)
            now = time.monotonic()
            if geometry['scene'] is not None and now-last_markers >= .2:
                markers.publish(marker_array(geometry['scene'], node.get_clock().now().to_msg(), Marker, MarkerArray))
                last_markers = now
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
        for sig, handler in previous.items(): signal.signal(sig, handler)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
