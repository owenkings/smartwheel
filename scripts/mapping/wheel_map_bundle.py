#!/usr/bin/env python3
"""Export/reload provisional wheel/IMU maps without starting any hardware.

The exporter opens the source SQLite database read-only, backs it up, and runs
RTAB-Map on that private snapshot in localhost ROS domain 93. A completed bundle
means files are valid, NOT that calibration, loop closure or map accuracy passed.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import sqlite3
import subprocess
import time

import numpy as np
import yaml


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def write_json(path, value):
    Path(path).write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')


def validate_grid(grid):
    width, height = int(grid.info.width), int(grid.info.height)
    if grid.header.frame_id != 'map' or width <= 0 or height <= 0:
        raise ValueError('grid must be a nonempty map-frame grid')
    if len(grid.data) != width * height:
        raise ValueError('grid dimensions do not match data length')
    if not math.isfinite(grid.info.resolution) or grid.info.resolution <= 0:
        raise ValueError('invalid grid resolution')
    cells = np.asarray(grid.data, dtype=np.int16).reshape(height, width)
    if np.any((cells < -1) | (cells > 100)) or np.all(cells == -1):
        raise ValueError('invalid or entirely unknown occupancy grid')
    pose = grid.info.origin
    q = np.array([pose.orientation.x, pose.orientation.y,
                  pose.orientation.z, pose.orientation.w], dtype=float)
    if not np.isfinite(q).all() or np.linalg.norm(q) < 1e-9:
        raise ValueError('invalid grid origin quaternion')
    q /= np.linalg.norm(q)
    if abs(q[0]) > 1e-6 or abs(q[1]) > 1e-6:
        raise ValueError('cannot export tilted occupancy origin to Nav2 YAML')
    if not all(math.isfinite(v) for v in (pose.position.x, pose.position.y, pose.position.z)):
        raise ValueError('nonfinite grid origin')
    if abs(pose.position.z) > 1e-6:
        raise ValueError('cannot silently discard nonzero grid origin z')
    yaw = 2.0 * math.atan2(q[2], q[3])
    return cells, yaw


def write_grid(directory, grid):
    cells, yaw = validate_grid(grid)
    # Nav2 trinary thresholds, explicitly recorded; unknown remains 205.
    pixels = np.full(cells.shape, 205, dtype=np.uint8)
    pixels[(cells >= 0) & (cells <= 25)] = 254
    pixels[cells >= 65] = 0
    pgm = b'P5\n%d %d\n255\n' % (grid.info.width, grid.info.height)
    (directory / 'map_2d.pgm').write_bytes(pgm + np.flipud(pixels).tobytes())
    (directory / 'map_2d.yaml').write_text(yaml.safe_dump({
        'image': 'map_2d.pgm', 'mode': 'trinary',
        'resolution': float(grid.info.resolution),
        'origin': [float(grid.info.origin.position.x), float(grid.info.origin.position.y), yaw],
        'negate': 0, 'occupied_thresh': 0.65, 'free_thresh': 0.25,
    }), encoding='utf-8')


def verify_bundle(directory):
    directory = Path(directory).resolve()
    if (directory / '.incomplete').exists():
        raise ValueError('bundle is incomplete')
    manifest = json.loads((directory / 'manifest.json').read_text(encoding='utf-8'))
    if manifest.get('schema_version') != 1 or manifest.get('complete') is not True:
        raise ValueError('unsupported or incomplete bundle manifest')
    entries = manifest.get('files', {})
    required = {'map_data.cdr', 'grid.cdr', 'map_xyzi.pcd', 'map_xyzi.ply',
                'map_2d.pgm', 'map_2d.yaml', 'trajectory.csv', 'backend.db'}
    if not required.issubset(entries):
        raise ValueError('bundle is missing required products')
    for name, record in entries.items():
        path = (directory / name).resolve()
        if path.parent != directory or not path.is_file():
            raise ValueError('invalid bundle file path: ' + name)
        if path.stat().st_size != record['size'] or sha256(path) != record['sha256']:
            raise ValueError('bundle checksum mismatch: ' + name)
    return manifest


def stop_owned(process):
    if process is None or process.poll() is not None:
        return
    for sig, timeout in ((signal.SIGINT, 8), (signal.SIGTERM, 3), (signal.SIGKILL, 2)):
        try:
            os.killpg(process.pid, sig)
            process.wait(timeout=timeout)
            return
        except ProcessLookupError:
            return
        except subprocess.TimeoutExpired:
            pass
    raise RuntimeError('owned offline backend failed to stop')


def call_service(node, service_type, name, request):
    import rclpy
    client = node.create_client(service_type, name)
    if not client.wait_for_service(timeout_sec=25):
        raise RuntimeError('offline service unavailable: ' + name)
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=30)
    if not future.done() or future.exception() is not None:
        raise RuntimeError('offline service failed or timed out: ' + name)
    return future.result()


def products_from_snapshot(map_data):
    from smartwheel_global_mapping.rtabmap_optimized_cloud_node import (
        assemble_optimized_cloud_with_intensity, assemble_optimized_trajectory)
    xyz, intensity, count = assemble_optimized_cloud_with_intensity(map_data, 0.05)
    trajectory = assemble_optimized_trajectory(map_data)
    if count < 2 or len(trajectory) < 2 or intensity is None:
        raise ValueError('need >=2 optimized scan/pose nodes with genuine intensity')
    return xyz, intensity, count, trajectory


def export_bundle(database, output):
    import rclpy
    from rclpy.serialization import serialize_message
    from rtabmap_msgs.srv import GetMap
    from nav_msgs.srv import GetMap as GetGrid
    from smartwheel_map_products.writers import write_pcd, write_ply

    source, directory = Path(database).resolve(strict=True), Path(output).resolve()
    if directory.exists():
        raise ValueError('output already exists; choose a new directory, never overwrite')
    source_hash = sha256(source)
    # Refuse empty/unrelated databases before creating an output directory.
    with sqlite3.connect(source.as_uri() + '?mode=ro', uri=True) as reader:
        if reader.execute('SELECT count(*) FROM Node').fetchone()[0] < 2:
            raise ValueError('database has fewer than two nodes; mapping not established')
        directory.mkdir(parents=True, exist_ok=False)
        (directory / '.incomplete').touch()
        with sqlite3.connect(directory / 'backend.db') as writer:
            reader.backup(writer)
    backend = None
    node = None
    log = (directory / 'backend.log').open('w')
    try:
        command = ['ros2', 'run', 'rtabmap_slam', 'rtabmap', '--ros-args',
                   '-r', '__node:=rtabmap', '-p', 'database_path:=' + str(directory / 'backend.db'),
                   '-p', 'subscribe_depth:=false', '-p', 'subscribe_rgb:=false',
                   '-p', 'subscribe_scan_cloud:=true', '-p', 'publish_tf:=false',
                   '-p', "Mem/IncrementalMemory:='false'", '-p', "Mem/InitWMWithAllNodes:='true'",
                   '-p', 'frame_id:=base_link', '-p', 'map_frame_id:=map',
                   '-p', "Grid/CellSize:='0.05'", '-p', "Grid/FromDepth:='false'",
                   '-p', "Grid/Sensor:='0'", '-p', "Reg/Force3DoF:='true'"]
        backend = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        rclpy.init()
        node = rclpy.create_node('wheel_bundle_export')
        request = GetMap.Request()
        request.global_map, request.optimized, request.graph_only = True, True, False
        map_data = call_service(node, GetMap, '/rtabmap/get_map_data', request).data
        grid = call_service(node, GetGrid, '/rtabmap/get_map', GetGrid.Request()).map
        xyz, intensity, count, trajectory = products_from_snapshot(map_data)
        write_grid(directory, grid)
        write_pcd(directory / 'map_xyzi.pcd', xyz, intensity)
        write_ply(directory / 'map_xyzi.ply', xyz, intensity=intensity)
        (directory / 'map_data.cdr').write_bytes(serialize_message(map_data))
        (directory / 'grid.cdr').write_bytes(serialize_message(grid))
        with (directory / 'trajectory.csv').open('w', newline='') as stream:
            writer = csv.writer(stream)
            writer.writerow(['node_id', 'stamp', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'])
            for node_id, stamp, pose in trajectory:
                p, q = pose.position, pose.orientation
                writer.writerow([node_id, stamp, p.x, p.y, p.z, q.x, q.y, q.z, q.w])
        stop_owned(backend)
        backend = None
        if sha256(source) != source_hash:
            raise RuntimeError('source database changed during export; retry after session stops')
        # A snapshot may contain additional nearby closures; report type counts,
        # never equate those counts or successful export with verified loop quality.
        link_counts = {}
        for link in map_data.graph.links:
            key = str(link.type)
            link_counts[key] = link_counts.get(key, 0) + 1
        manifest = {
            'schema_version': 1, 'complete': True,
            'stage': 'PROVISIONAL_PLANAR_INDOOR_MAPPING', 'hardware_validated': False,
            'accuracy_validated': False, 'extrinsics_validated': False,
            'loop_closure_validated': False, 'map_frame': 'map',
            'pose_source': 'wheel_speed_and_H30_yaw_rate', 'backend': 'RTAB-Map',
            'source_database': str(source), 'source_database_sha256': source_hash,
            'voxel_size_m': 0.05, 'point_count': len(xyz), 'scan_nodes': count,
            'trajectory_poses': len(trajectory), 'link_type_counts': link_counts,
            'grid_size': [grid.info.width, grid.info.height],
            'limitations': ['Planar vehicle pose, not slope tracking',
                            'Mount origins/time synchronization/wheel scale not calibrated',
                            'Export success is not real-route map acceptance'],
        }
        log.flush()
        manifest['files'] = {p.name: {'size': p.stat().st_size, 'sha256': sha256(p)}
                             for p in sorted(directory.iterdir())
                             if p.is_file() and p.name not in ('.incomplete', 'manifest.json')}
        write_json(directory / 'manifest.json', manifest)
        (directory / '.incomplete').unlink()
        verify_bundle(directory)
        print(json.dumps({'export_complete': True, 'directory': str(directory),
                          'points': len(xyz), 'hardware_validated': False}))
    finally:
        stop_owned(backend)
        log.close()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def preview_bundle(directory, seconds, gui):
    import rclpy
    from rclpy.serialization import deserialize_message
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from rtabmap_msgs.msg import MapData
    from nav_msgs.msg import OccupancyGrid, Path as NavPath
    from geometry_msgs.msg import PoseStamped
    from std_msgs.msg import Header
    from sensor_msgs.msg import PointCloud2
    from smartwheel_sensor_api import pointcloud2_from_xyz

    directory = Path(directory).resolve()
    manifest = verify_bundle(directory)
    data = deserialize_message((directory / 'map_data.cdr').read_bytes(), MapData)
    grid = deserialize_message((directory / 'grid.cdr').read_bytes(), OccupancyGrid)
    xyz, intensity, count, trajectory = products_from_snapshot(data)
    validate_grid(grid)
    if len(xyz) != manifest['point_count'] or count != manifest['scan_nodes']:
        raise ValueError('reconstructed geometry does not match manifest')
    rclpy.init()
    node = rclpy.create_node('wheel_bundle_preview')
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    cloud_pub = node.create_publisher(PointCloud2, '/wheel_preview/cloud', qos)
    grid_pub = node.create_publisher(OccupancyGrid, '/wheel_preview/grid', qos)
    path_pub = node.create_publisher(NavPath, '/wheel_preview/path', qos)
    rviz = None
    try:
        if gui:
            config = Path(__file__).with_name('wheel_map_preview.rviz')
            rviz = subprocess.Popen(['rviz2', '-d', str(config)], start_new_session=True)
        deadline = time.monotonic() + seconds if seconds > 0 else math.inf
        print(json.dumps({'reload_verified': True, 'points': len(xyz),
                          'hardware_started': False, 'hardware_validated': False}), flush=True)
        while time.monotonic() < deadline and (rviz is None or rviz.poll() is None):
            header = Header(stamp=node.get_clock().now().to_msg(), frame_id='map')
            cloud = pointcloud2_from_xyz(header, xyz, intensity=intensity)
            grid.header = header
            path = NavPath(header=header)
            for _, stamp, pose in trajectory:
                pose_header = Header(frame_id='map')
                pose_header.stamp.sec = int(stamp)
                pose_header.stamp.nanosec = int(round((stamp - int(stamp)) * 1e9))
                if pose_header.stamp.nanosec == 1000000000:
                    pose_header.stamp.sec += 1
                    pose_header.stamp.nanosec = 0
                path.poses.append(PoseStamped(header=pose_header, pose=pose))
            cloud_pub.publish(cloud)
            grid_pub.publish(grid)
            path_pub.publish(path)
            rclpy.spin_once(node, timeout_sec=0.5)
    except KeyboardInterrupt:
        pass
    finally:
        stop_owned(rviz)
        node.destroy_node()
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest='command', required=True)
    export = commands.add_parser('export')
    export.add_argument('--database', required=True)
    export.add_argument('--output', required=True)
    preview = commands.add_parser('preview')
    preview.add_argument('--bundle', required=True)
    preview.add_argument('--seconds', type=float, default=0)
    preview.add_argument('--no-gui', action='store_true')
    args = parser.parse_args()
    # Prevent exporting/previewing in the live motor/sensor DDS graph.
    os.environ['ROS_DOMAIN_ID'] = '93'
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    import fcntl
    with open('/tmp/smartwheel_wheel_bundle.lock', 'a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        if args.command == 'export':
            export_bundle(args.database, args.output)
        else:
            preview_bundle(args.bundle, args.seconds, not args.no_gui)


if __name__ == '__main__':
    main()
