#!/usr/bin/env python3
"""Read-only recorded geometry audit; no ROS nodes, device access or map mutation."""
import argparse
from bisect import bisect_right
import hashlib
import json
from pathlib import Path
import sqlite3
import struct

import numpy as np
from scipy.spatial.transform import Rotation


def stamp(value):
    return int(value.sec)*10**9+int(value.nanosec)


def stats(values):
    a = np.asarray(values, dtype=float)
    return {'min': np.min(a, axis=0).tolist(), 'median': np.median(a, axis=0).tolist(),
            'max': np.max(a, axis=0).tolist(), 'mean': np.mean(a, axis=0).tolist(),
            'std': np.std(a, axis=0).tolist()} if len(a) else None


def plane(points, *, horizontal, rng, threshold=.035, iterations=350):
    """Independent robust planes, with only a broad horizontal/vertical category."""
    points = points[np.isfinite(points).all(axis=1)]
    if len(points) < 100:
        return None
    if len(points) > 18000:
        points = points[rng.choice(len(points), 18000, replace=False)]
    best = None
    for _ in range(iterations):
        a, b, c = points[rng.choice(len(points), 3, replace=False)]
        n = np.cross(b-a, c-a)
        length = np.linalg.norm(n)
        if length < 1e-8:
            continue
        n /= length
        if (horizontal and abs(n[2]) < .7) or (not horizontal and abs(n[2]) > .7):
            continue
        d = -n@a
        mask = abs(points@n+d) < threshold
        if best is None or int(mask.sum()) > best[0]:
            best = (int(mask.sum()), mask)
    if best is None or best[0] < 100:
        return None
    mask = best[1]
    for _ in range(3):
        selected = points[mask]
        center = selected.mean(axis=0)
        _, _, vh = np.linalg.svd(selected-center, full_matrices=False)
        n = vh[-1]
        if n[2] < 0: n = -n
        d = -n@center
        mask = abs(points@n+d) < threshold
    residual = points[mask]@n+d
    return {'normal': n.tolist(), 'd_m': float(d), 'candidate_points': len(points), 'inliers': int(mask.sum()),
            'inlier_ratio': float(mask.mean()), 'normal_tilt_from_z_deg': float(np.rad2deg(np.arccos(np.clip(abs(n[2]), 0, 1)))),
            'inlier_abs_residual_p50_p95_m': np.quantile(abs(residual), [.5, .95]).tolist(),
            'centroid_m': points[mask].mean(axis=0).tolist(),
            'inlier_xyz_p05_p95_m': np.quantile(points[mask], [.05, .95], axis=0).tolist(), 'threshold_m': threshold}


def bag_topics(source):
    result = []
    for path in sorted((source/'bag').glob('*.db3')):
        with sqlite3.connect(path.as_uri()+'?mode=ro', uri=True) as db:
            result.append((path, {name: (i, kind) for i, name, kind in db.execute('SELECT id,name,type FROM topics')}))
    return result


def messages(parts, name, *, stride=1):
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    index = 0
    for path, topics in parts:
        if name not in topics: continue
        topic_id, kind = topics[name]
        message_type = get_message(kind)
        with sqlite3.connect(path.as_uri()+'?mode=ro', uri=True) as db:
            for received, raw in db.execute('SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp,id', (topic_id,)):
                if index % stride == 0:
                    yield received, deserialize_message(raw, message_type), hashlib.sha256(raw).hexdigest()
                index += 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session-root', type=Path, required=True)
    args = parser.parse_args()
    from wc_sensors.pointcloud import decode_pointcloud2
    from wc_motion.protocol import parse_exchange
    source = args.session_root.resolve()
    config = json.loads((source/'runtime_config.json').read_text())
    initial = json.loads((source/'prior/initialization.json').read_text())
    prior_status = json.loads((source/'prior/status.json').read_text())
    health = json.loads((source/'health/status.json').read_text())
    wheel_status = json.loads((source/'wheel_status.json').read_text())
    guesses = [json.loads(line) for line in (source/'prior/guesses.jsonl').read_text().splitlines() if line]
    poses = np.asarray([row['T_prior_reference'] for row in guesses])
    times = np.asarray([row['stamp_ns'] for row in guesses], dtype=np.int64)
    relative = (times-times[0])*1e-9
    euler = Rotation.from_matrix(poses[:, :3, :3]).as_euler('xyz', degrees=True)
    euler[:, 2] = np.rad2deg(np.unwrap(np.deg2rad(euler[:, 2])))
    level = np.asarray(initial['R_reference_base'])
    r_ref_imu = np.asarray(initial['R_reference_imu'])
    report = {'session_id': config['session_id'], 'validation_level': 'REAL_BAG_GEOMETRY_DIAGNOSTIC',
              'hardware_accessed': False, 'ros_nodes_started': False, 'original_data_modified': False,
              'source_root': str(source), 'initialization': initial,
              'source_summary': {'duration_s': float(relative[-1]), 'prior_count': len(guesses),
                  'prior_failure': prior_status['failure'], 'prior_coverage_gap_count': prior_status['coverage_gap_count'],
                  'manual_control_transmissions': wheel_status['manual']['control_transmissions'],
                  'health_counts': health['counts'], 'final_map_age_s': health['age_s']['cloud_map']},
              'trajectory': {'xyz_m': stats(poses[:, :3, 3]), 'roll_pitch_unwrapped_yaw_deg': stats(euler),
                  'last_xyz_m': poses[-1, :3, 3].tolist(), 'last_rpy_deg': euler[-1].tolist(),
                  'xy_path_length_m': float(np.linalg.norm(np.diff(poses[:, :2, 3], axis=0), axis=1).sum()),
                  'samples': [{'t_s': float(t), 'xyz_m': p[:3, 3].tolist(), 'rpy_deg': r.tolist()}
                              for t, p, r in zip(relative, poses, euler)]},
              'limitations': ['No surveyed floor/wall angle or external motion ground truth.',
                  'Plane selection is RANSAC in broad floor/wall ROIs; supports are reported and furniture may contaminate candidates.',
                  'Zero wheel feedback does not prove the IMU or chassis is physically motionless.',
                  'A rigid mounting rotation preserves within-frame plane angles; it can change leveling and accumulated trajectory axes.']}
    report['mount_geometry'] = {'right_rotation_rpy_deg': Rotation.from_matrix(level).as_euler('xyz', degrees=True).tolist(),
        'reference_imu_matches_config_max_error': float(np.max(abs(r_ref_imu-np.asarray(config['imu_mount']['R_axle_imu'])))),
        'reference_axle_rotation_identity_max_error': float(np.max(abs(np.asarray(initial['T_reference_axle'])[:3,:3]-np.eye(3)))),
        'expected_reference_axle_translation_m': (-np.asarray(config['mounts']['right']['t_axle_lidar_m'])).tolist()}
    parts = bag_topics(source)
    wheel = []
    for _, msg, _ in messages(parts, '/wc_mapping/wheel/feedback_raw'):
        row = json.loads(msg.data)
        words = parse_exchange(bytes.fromhex(row['request_hex']), bytes.fromhex(row['response_hex']))[2]
        wheel.append((row['stamp_ns'], words == [0, 0] or tuple(words) == (0, 0)))
    wheel.sort()
    wheel_times = [row[0] for row in wheel]
    imu_rows = []
    for _, msg, _ in messages(parts, '/wc_mapping/imu/source_frame', stride=5):
        t = stamp(msg.host_receive_time)
        a, g = msg.imu.linear_acceleration, msg.imu.angular_velocity
        accel, gyro = np.array([a.x, a.y, a.z]), np.array([g.x, g.y, g.z])
        index = bisect_right(wheel_times, t)-1
        zero = index >= 0 and wheel[index][1] and t-wheel[index][0] < 200_000_000
        imu_rows.append((t, gyro, accel, zero))
    windows, run = [], []
    def finish_run():
        if len(run) >= 60 and run[-1][0]-run[0][0] >= 2_000_000_000:
            gyros, accels = np.asarray([row[1] for row in run]), np.asarray([row[2] for row in run])
            mean = gyros.mean(axis=0)
            gravity = r_ref_imu@accels.mean(axis=0)
            start = int(np.argmin(abs(times-run[0][0]))); end = int(np.argmin(abs(times-run[-1][0])))
            rotation_change = Rotation.from_matrix(poses[start, :3, :3].T@poses[end, :3, :3]).as_rotvec()
            windows.append({'start_s': (run[0][0]-int(times[0]))*1e-9, 'end_s': (run[-1][0]-int(times[0]))*1e-9,
                'samples': len(run), 'gyro_native_mean_rad_s': mean.tolist(), 'gyro_native_std_rad_s': gyros.std(axis=0).tolist(),
                'gyro_reference_mean_deg_s': np.rad2deg(r_ref_imu@mean).tolist(),
                'gyro_reference_if_constant_over_session_deg': (np.rad2deg(r_ref_imu@mean)*relative[-1]).tolist(),
                'mean_acceleration_reference_m_s2': gravity.tolist(),
                'acceleration_tilt_from_reference_z_deg': float(np.rad2deg(np.arctan2(np.linalg.norm(gravity[:2]), gravity[2]))),
                'prior_rotation_change_vector_deg': np.rad2deg(rotation_change).tolist(),
                'prior_xyz_change_m': (poses[end, :3, 3]-poses[start, :3, 3]).tolist()})
        run.clear()
    for row in imu_rows:
        good = row[3] and np.linalg.norm(row[1]) < .03 and 8 < np.linalg.norm(row[2]) < 12
        if not good or (run and row[0]-run[-1][0] > 200_000_000): finish_run()
        if good: run.append(row)
    finish_run()
    report['zero_wheel_low_gyro_windows'] = windows
    with sqlite3.connect((source/'slam/rtabmap.db').as_uri()+'?mode=ro', uri=True) as db:
        rows = db.execute('SELECT id,stamp,pose,weight FROM Node ORDER BY id').fetchall()
        raw_nodes = []
        errors = []
        for identifier, when, blob, weight in rows:
            if len(blob) != 48: raise RuntimeError('Unexpected RTAB node pose encoding')
            matrix = np.eye(4); matrix[:3] = np.asarray(struct.unpack('<12f', blob)).reshape(3, 4)
            if not np.allclose(matrix[:3, :3].T@matrix[:3, :3], np.eye(3), atol=1e-5):
                raise RuntimeError('RTAB node pose is not a rigid 3x4 float32 transform')
            j = int(np.argmin(abs(times-round(when*1e9))))
            errors.append(np.linalg.norm(matrix[:3, 3]-poses[j, :3, 3]))
            raw_nodes.append({'id': identifier, 'stamp_ns': round(when*1e9), 'weight': weight, 'pose': matrix.tolist()})
        report['rtab_database'] = {'node_count': len(rows), 'links_by_type': db.execute('SELECT type,count(*) FROM Link GROUP BY type').fetchall(),
            'node_translation_difference_from_nearest_prior_m': stats(errors), 'nodes': raw_nodes}
    rng = np.random.default_rng(20260915)
    clouds = list(messages(parts, '/wc_mapping/lidar_right/source_frame', stride=120))
    frames = []
    for received, msg, digest in clouds:
        raw = decode_pointcloud2(msg.cloud)
        raw = raw[np.isfinite(raw).all(axis=1)]
        xyz = raw@level.T
        distance = np.linalg.norm(xyz, axis=1)
        xyz = xyz[(distance > .8)&(distance < 8)&(abs(xyz[:, 0]) < 6)&(abs(xyz[:, 1]) < 6)]
        floor = plane(xyz[(xyz[:, 2] > -1.4)&(xyz[:, 2] < -.15)], horizontal=True, rng=rng)
        walls = []
        candidates = xyz[(xyz[:, 2] > -.6)&(xyz[:, 2] < 2.4)]
        if floor:
            candidates = candidates[abs(candidates@np.array(floor['normal'])+floor['d_m']) > .12]
        for _ in range(2):
            wall = plane(candidates, horizontal=False, rng=rng)
            if wall is None: break
            if floor:
                wall['acute_angle_to_floor_deg'] = float(np.rad2deg(np.arccos(np.clip(abs(np.dot(floor['normal'], wall['normal'])), 0, 1))))
                wall['deviation_from_90_deg'] = 90-wall['acute_angle_to_floor_deg']
                extent = np.asarray(wall['inlier_xyz_p05_p95_m'])
                wall['usable_for_angle_summary'] = bool(wall['inliers'] >= 600 and abs(wall['normal'][2]) < .35
                    and extent[1,2]-extent[0,2] >= .7 and floor['inliers'] >= 1000)
            walls.append(wall)
            candidates = candidates[abs(candidates@np.array(wall['normal'])+wall['d_m']) > .12]
        t = stamp(msg.header.stamp)
        j = int(np.argmin(abs(times-t)))
        entry = {'source_stamp_ns': t, 'time_s': (t-int(times[0]))*1e-9, 'cdr_sha256': digest,
                 'raw_finite_points': len(raw), 'nearby_prior_delta_s': abs(int(times[j])-t)*1e-9,
                 'floor_in_fixed_reference': floor, 'walls_in_same_frame': walls,
                 'cloud_fields': [{'name': field.name, 'datatype': field.datatype, 'count': field.count} for field in msg.cloud.fields]}
        if len(frames) in (0, 1, len(clouds)-2, len(clouds)-1):
            entry['sample_points_fixed_reference_m'] = xyz[rng.choice(len(xyz), min(2500, len(xyz)), replace=False)].tolist()
        if floor:
            n = poses[j, :3, :3]@np.asarray(floor['normal'])
            d = floor['d_m']-n@poses[j, :3, 3]
            entry['floor_after_recorded_prior'] = {'normal': n.tolist(), 'd_m': float(d),
                'tilt_from_world_z_deg': float(np.rad2deg(np.arccos(np.clip(abs(n[2]), 0, 1)))),
                'plane_height_at_world_origin_m': float(-d/n[2])}
        frames.append(entry)
    report['single_frame_planes'] = frames
    last_graph = None
    for _, message, digest in messages(parts, '/wc_mapping/app/mapGraph'):
        last_graph = message, digest
    if last_graph:
        graph, digest = last_graph
        node_by_id = {row['id']: np.asarray(row['pose']) for row in raw_nodes}
        translation_difference, rotation_difference = [], []
        for identifier, value in zip(graph.poses_id, graph.poses):
            if identifier not in node_by_id: continue
            original = node_by_id[identifier]
            p, q = value.position, value.orientation
            rotation = Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix()
            translation_difference.append(float(np.linalg.norm(np.asarray([p.x,p.y,p.z])-original[:3,3])))
            rotation_difference.append(float(np.rad2deg(Rotation.from_matrix(original[:3,:3].T@rotation).magnitude())))
        report['last_recorded_graph'] = {'cdr_sha256': digest, 'pose_count': len(graph.poses_id),
            'translation_difference_from_db_node_m': stats(translation_difference),
            'rotation_difference_from_db_node_deg': stats(rotation_difference)}
    latest_map = None
    for _, msg, digest in messages(parts, '/wc_mapping/app/cloud_map'):
        latest_map = (msg, digest)
    if latest_map:
        msg, digest = latest_map
        xyz = decode_pointcloud2(msg); xyz = xyz[np.isfinite(xyz).all(axis=1)]
        map_floor = plane(xyz[(xyz[:, 2] > -2.)&(xyz[:, 2] < .2)], horizontal=True, rng=rng, iterations=500)
        chosen = xyz[rng.choice(len(xyz), min(6000, len(xyz)), replace=False)]
        report['saved_map_snapshot'] = {'stamp_ns': stamp(msg.header.stamp), 'frame_id': msg.header.frame_id,
            'point_count': len(xyz), 'cdr_sha256': digest, 'xyz_m': stats(xyz), 'dominant_floor_plane': map_floor,
            'scatter_xyz_m': chosen.tolist()}
    # Output is data only; the caller saves it outside the original session.
    print(json.dumps(report, ensure_ascii=False, allow_nan=False, separators=(',', ':')))


if __name__ == '__main__': main()
