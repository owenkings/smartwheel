"""Display-only wheelchair schematic and candidate-height 2D projection.

Neither the native cloud, occupancy grid, TF tree nor saved maps are changed.
All geometry comes from the frozen session configuration and initialization.
This is a placement aid, not a measured body mesh or collision model.
"""
import argparse
import copy
import math
from pathlib import Path
import signal
import time

import numpy as np


MARKERS_TOPIC = '/wc_mapping/app/view_markers'
GRID_TOPIC = '/wc_mapping/app/view_grid'
NATIVE_GRID_TOPIC = '/wc_mapping/app/grid_map'
REFERENCE_FRAME = 'mapping_reference'
MAP_FRAME = 'mapping_map'
# Rendering thicknesses only; no physical chair thickness is inferred.
PANEL_THICKNESS_M = .03
SYMBOL_DIAMETER_M = .06


def _positive(value, label, maximum):
    if type(value) not in (int, float) or not math.isfinite(value) or not 0 < value <= maximum:
        raise ValueError(label+' must be a finite positive dimension within '+str(maximum)+' m')
    return float(value)


def validate_visualization(value):
    """Validate the dedicated schematic options without importing mount code."""
    if not isinstance(value, dict) or set(value) != {'wheelchair', 'display_ground_projection', 'note'}:
        raise ValueError('visualization requires exactly wheelchair, display_ground_projection and note')
    if type(value['display_ground_projection']) is not bool:
        raise ValueError('display_ground_projection must be boolean')
    if not isinstance(value['note'], str) or not value['note'].strip() or len(value['note']) > 2000:
        raise ValueError('visualization.note must describe the schematic, within 2000 characters')
    chair = value['wheelchair']
    dimensions = {'seat_width_m', 'seat_depth_m', 'seat_height_above_axle_m', 'back_height_m', 'wheel_width_m'}
    if not isinstance(chair, dict) or set(chair) != dimensions | {'schematic'} or chair['schematic'] is not True:
        raise ValueError('wheelchair must explicitly be schematic with exactly its five display dimensions')
    result = copy.deepcopy(value)
    for key in dimensions:
        result['wheelchair'][key] = _positive(chair[key], key, .5 if key == 'wheel_width_m' else 2.)
    return result


def _rigid(value, label):
    matrix = np.asarray(value, dtype=float)
    if matrix.shape != (4, 4) or not np.isfinite(matrix).all() or \
            not np.allclose(matrix[3], [0, 0, 0, 1], atol=1e-8, rtol=0):
        raise ValueError(label+' must be a finite homogeneous 4x4 transform')
    rotation = matrix[:3, :3]
    if not np.allclose(rotation.T@rotation, np.eye(3), atol=1e-6, rtol=0) or \
            not math.isclose(float(np.linalg.det(rotation)), 1., abs_tol=1e-6, rel_tol=0):
        raise ValueError(label+' must contain a proper rotation, without reflection or scaling')
    return matrix.copy()


def _quaternion(rotation):
    """Proper rotation matrix to normalized XYZW, including 180-degree poses."""
    r = rotation
    trace = float(np.trace(r))
    if trace > 0:
        s = 2*math.sqrt(trace+1.)
        q = np.array([(r[2, 1]-r[1, 2])/s, (r[0, 2]-r[2, 0])/s, (r[1, 0]-r[0, 1])/s, s/4])
    else:
        i = int(np.argmax(np.diag(r))); j = (i+1) % 3; k = (i+2) % 3
        s = 2*math.sqrt(max(0., 1.+r[i, i]-r[j, j]-r[k, k]))
        q = np.zeros(4)
        q[i] = s/4; q[j] = (r[j, i]+r[i, j])/s; q[k] = (r[k, i]+r[i, k])/s
        q[3] = (r[k, j]-r[j, k])/s
    return (q/np.linalg.norm(q)).tolist()


def _marker(identifier, shape, parent, position, scale, color, *, rotation=None, text=''):
    local = np.eye(4)
    local[:3, 3] = position
    if rotation is not None:
        local[:3, :3] = rotation
    pose = parent@local
    return {'id': identifier, 'shape': shape, 'frame_id': REFERENCE_FRAME, 'frame_locked': True,
            'position_m': pose[:3, 3].tolist(), 'orientation_xyzw': _quaternion(pose[:3, :3]),
            'scale_m': list(map(float, scale)), 'color_rgba': list(map(float, color)), 'text': text}


def build_scene(config, initialization):
    """Return immutable-data marker specifications, with no ROS or file access."""
    settings = validate_visualization(config['visualization'])
    if config.get('mode') not in ('left', 'right', 'all') or not isinstance(config.get('session_id'), str) or \
            not config['session_id'] or config.get('source_mode') != 'real':
        raise ValueError('Explicit real session identity and selected lidar mode required')
    if initialization.get('session_id') != config['session_id'] or initialization.get('source_mode') != 'real' or \
            initialization.get('status') != 'EXPERIMENT':
        raise ValueError('Initialization must belong to this experimental session')
    expected_base = 'lidar_right' if config['mode'] == 'right' else 'lidar_left'
    if initialization.get('gravity_input_base_frame') != expected_base:
        raise ValueError('Initialization base does not match selected lidar mode')
    transform = _rigid(initialization['T_reference_axle'], 'T_reference_axle')
    wheel = config['wheel_candidate']
    radius = _positive(wheel['wheel_radius_m'], 'wheel radius candidate', 1.)
    track = _positive(wheel['track_width_m'], 'wheel track candidate', 3.)
    chair = settings['wheelchair']
    width, depth, height, back = [chair[key] for key in
        ('seat_width_m', 'seat_depth_m', 'seat_height_above_axle_m', 'back_height_m')]
    # The rear edge of the schematic seat is placed at axle X=0. This is a
    # display convention, not an independently measured seat mounting pose.
    wheel_rotation = np.array([[1., 0., 0.], [0., 0., -1.], [0., 1., 0.]])
    markers = [_marker(0, 'CYLINDER', transform, [0, track/2, 0],
                       [2*radius, 2*radius, chair['wheel_width_m']], [.18, .22, .28, 1.], rotation=wheel_rotation),
               _marker(1, 'CYLINDER', transform, [0, -track/2, 0],
                       [2*radius, 2*radius, chair['wheel_width_m']], [.18, .22, .28, 1.], rotation=wheel_rotation),
               _marker(2, 'CUBE', transform, [0, 0, 0], [PANEL_THICKNESS_M, track, PANEL_THICKNESS_M], [.6, .65, .7, 1.]),
               _marker(3, 'CUBE', transform, [depth/2, 0, height],
                       [depth, width, PANEL_THICKNESS_M], [.25, .55, .9, .75]),
               _marker(4, 'CUBE', transform, [0, 0, height+back/2],
                       [PANEL_THICKNESS_M, width, back], [.25, .55, .9, .75]),
               _marker(5, 'ARROW', transform, [0, 0, height+back+.1], [.35, .025, .06], [.95, .8, .15, 1.])]
    for index, side in enumerate(('left', 'right')):
        mount = config['mounts'][side]
        axle_lidar = np.eye(4)
        axle_lidar[:3, :3] = np.asarray(mount['R_axle_lidar'], dtype=float)
        axle_lidar[:3, 3] = np.asarray(mount['t_axle_lidar_m'], dtype=float)
        axle_lidar = _rigid(axle_lidar, 'T_axle_'+side)
        active = config['mode'] in (side, 'all')
        color = [.15, .85, .9, 1. if active else .35] if side == 'left' else [1., .6, .2, 1. if active else .35]
        markers.append(_marker(6+index, 'SPHERE', transform@axle_lidar, [0, 0, 0], [SYMBOL_DIAMETER_M]*3, color))
        markers.append(_marker(8+index, 'TEXT_VIEW_FACING', transform@axle_lidar, [0, 0, .1],
                               [0, 0, .07], color, text=side.upper()+' lidar'+(' selected' if active else ' inactive')))
    # This is the requested initial horizontal reference candidate. It is not
    # plane fitting, ground truth, a map correction, or a live floor tracker.
    ground_z = float((transform@np.array([0., 0., -radius, 1.]))[2])
    label = 'Wheelchair schematic\nNOT a collision model\nGround display candidate Z={:.3f} m'.format(ground_z)
    markers.append(_marker(10, 'TEXT_VIEW_FACING', transform, [0, 0, height+back+.35],
                           [0, 0, .085], [.95, .95, .95, 1.], text=label))
    return {'markers': markers, 'ground_z_m': ground_z,
            'display_ground_projection': settings['display_ground_projection'],
            'session_id': config['session_id'], 'ground_height_validated': False,
            'collision_model': False, 'native_maps_modified': False,
            'ground_basis': 'Initial T_reference_axle*[0,0,-wheel_radius,1]; horizontal reference candidate, not a fitted floor'}


def display_grid(message, ground_z_m):
    """Independent display copy; preserve stamp, cells, XY, orientation and scale."""
    if message.header.frame_id != MAP_FRAME:
        raise ValueError('Unexpected native occupancy grid frame')
    if type(ground_z_m) not in (int, float) or not math.isfinite(ground_z_m):
        raise ValueError('Finite candidate display height required')
    width, height = message.info.width, message.info.height
    if type(width) is not int or type(height) is not int or min(width, height) <= 0 or \
            width*height != len(message.data) or width*height > 10_000_000:
        raise ValueError('Invalid bounded occupancy grid dimensions')
    if not math.isfinite(message.info.resolution) or message.info.resolution <= 0:
        raise ValueError('Invalid grid resolution')
    values = np.asarray(message.data)
    if values.dtype.kind not in 'iu' or not ((values >= -1) & (values <= 100)).all():
        raise ValueError('Invalid grid occupancy values')
    p, q = message.info.origin.position, message.info.origin.orientation
    if not all(math.isfinite(value) for value in (p.x, p.y, p.z, q.x, q.y, q.z, q.w)) or \
            not math.isclose(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w, 1., abs_tol=1e-6, rel_tol=0):
        raise ValueError('Invalid grid origin')
    result = copy.deepcopy(message)
    result.info.origin.position.z = float(ground_z_m)
    return result


def marker_array(scene, stamp, marker_type, array_type):
    """Adapt tested geometry specifications to native ROS Marker messages."""
    markers = []
    for spec in scene['markers']:
        marker = marker_type()
        marker.header.frame_id = spec['frame_id']; marker.header.stamp = copy.deepcopy(stamp)
        marker.ns = 'wheelchair_schematic'; marker.id = spec['id']
        marker.type = getattr(marker_type, spec['shape']); marker.action = marker_type.ADD
        marker.frame_locked = True
        marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = spec['position_m']
        marker.pose.orientation.x, marker.pose.orientation.y, marker.pose.orientation.z, marker.pose.orientation.w = spec['orientation_xyzw']
        marker.scale.x, marker.scale.y, marker.scale.z = spec['scale_m']
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = spec['color_rgba']
        marker.text = spec['text']
        marker.lifetime.sec = 2
        markers.append(marker)
    return array_type(markers=markers)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session-root', type=Path, required=True)
    args = parser.parse_args(argv)
    from .cli import ROOT, target
    from .prepare_picker_input import project_path, read_json
    target()
    directory = project_path(ROOT, args.session_root)
    config = read_json(ROOT, directory/'runtime_config.json', limit=1_000_000)
    validate_visualization(config['visualization'])
    initialization_path = project_path(ROOT, directory/'prior/initialization.json')
    # All ROS imports follow argument parsing; offline geometry tests need no ROS.
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from nav_msgs.msg import OccupancyGrid
    from visualization_msgs.msg import Marker, MarkerArray
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('mapping_app_visuals')
    stopped = [False]
    previous_signals = {sig: signal.signal(sig, lambda *_: stopped.__setitem__(0, True))
                        for sig in (signal.SIGINT, signal.SIGTERM)}
    latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
    marker_publisher = node.create_publisher(MarkerArray, MARKERS_TOPIC, latched)
    grid_publisher = node.create_publisher(OccupancyGrid, GRID_TOPIC, latched)
    scene = None
    latest_grid = None
    last_warning = -math.inf

    def warn(reason):
        nonlocal last_warning
        now = time.monotonic()
        if now-last_warning >= 5:
            node.get_logger().warning('Display layer waiting: '+str(reason)); last_warning = now

    def receive_grid(message):
        nonlocal latest_grid
        # A single latched native grid may arrive before initialization exists.
        # Keep one bounded original message and publish it after initialization.
        try:
            latest_grid = display_grid(message, message.info.origin.position.z)
            if scene is not None and scene['display_ground_projection']:
                grid_publisher.publish(display_grid(latest_grid, scene['ground_z_m']))
        except Exception as error:
            warn(error)

    subscription = node.create_subscription(OccupancyGrid, NATIVE_GRID_TOPIC, receive_grid, latched)
    last_init_check = last_markers = -math.inf
    try:
        while not stopped[0] and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=.1)
            now = time.monotonic()
            if scene is None and now-last_init_check >= 1:
                last_init_check = now
                try:
                    if initialization_path.exists():
                        scene = build_scene(config, read_json(ROOT, initialization_path, limit=2_000_000))
                        if latest_grid is not None and scene['display_ground_projection']:
                            grid_publisher.publish(display_grid(latest_grid, scene['ground_z_m']))
                    else:
                        warn('prior initialization is not available yet')
                except Exception as error:
                    warn(error)
            if scene is not None and now-last_markers >= .2:
                try:
                    marker_publisher.publish(marker_array(scene, node.get_clock().now().to_msg(), Marker, MarkerArray))
                except Exception as error:
                    warn(error)
                last_markers = now
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for sig, handler in previous_signals.items():
            signal.signal(sig, handler)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
