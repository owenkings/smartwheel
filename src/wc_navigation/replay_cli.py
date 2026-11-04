"""Build a training-only wall submap and localize a separate recording interval."""
import argparse
import csv
from dataclasses import asdict
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import uuid

import numpy as np


def number(value):
    value = float(value)
    if not math.isfinite(value):
        raise argparse.ArgumentTypeError('finite number required')
    return value


def arguments(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--project-root', type=Path, required=True)
    parser.add_argument('--source-cell', type=Path, required=True)
    parser.add_argument('--training-window', type=number, nargs=2, default=[110., 180.])
    parser.add_argument('--query-window', type=number, nargs=2, default=[70., 105.])
    parser.add_argument('--training-interval', type=number, default=.6)
    parser.add_argument('--query-interval', type=number, default=.45)
    parser.add_argument('--initial-offset', type=number, nargs=3, default=[.15, .10, 3.],
                        metavar=('X_M', 'Y_M', 'YAW_DEG'))
    parser.add_argument('--max-gap', type=number, default=1.)
    args = parser.parse_args(argv)
    for window in [args.training_window, args.query_window]:
        if not 0 <= window[0] < window[1]:
            parser.error('ordered nonnegative time windows required')
    a, b = args.training_window, args.query_window
    if not (a[1] < b[0] or b[1] < a[0]):
        parser.error('training and query windows must be separated by a temporal gap')
    if not 0 < args.training_interval <= 10 or not 0 < args.query_interval < args.max_gap <= 10:
        parser.error('positive bounded sampling intervals and query interval < max gap required')
    if math.hypot(*args.initial_offset[:2]) > .3 or abs(args.initial_offset[2]) > 7:
        parser.error('initial perturbation must remain within the declared local matcher capture range')
    return args


def digest(path):
    result = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            result.update(chunk)
    return result.hexdigest()


def write(path, value):
    Path(path).write_text(json.dumps(value, ensure_ascii=False, allow_nan=False, indent=2) + '\n', encoding='utf-8')


def select_rows(rows, epoch, window, interval):
    result = []
    last = None
    for row in rows:
        relative = (row['stamp_ns'] - epoch) * 1e-9
        if window[0] <= relative <= window[1] and (last is None or row['stamp_ns'] - last >= interval * 1e9):
            result.append(row)
            last = row['stamp_ns']
    return result


def perturbed_initial_pose(pose, offset):
    """Apply a map-frame XY offset and orientation offset about the pose origin."""
    from wc_runtime.offline_planar_registration import rigid, se2
    result = rigid(pose)
    result[:2, 3] += np.asarray(offset[:2], dtype=float)
    result[:3, :3] = se2(0., 0., math.radians(offset[2]))[:3, :3] @ result[:3, :3]
    return rigid(result)


def checked_frames(source, rows, runtime, hashes):
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import PointCloud2
    from nav_msgs.msg import Odometry
    from scipy.spatial.transform import Rotation
    from wc_runtime.offline_planar_registration import rigid
    from .rebuild import select_left_cloud, pointcloud_xyz
    frames, counts = [], []
    for row in rows:
        raw = {}
        for kind in ['cloud', 'odom']:
            name = row[kind + '_cdr_file']
            if Path(name).name != name:
                raise ValueError('source CDR path must be a single filename')
            path = source / 'frontend' / name
            if path.is_symlink() or path.stat().st_size > 64 * 1024**2:
                raise ValueError('linked or oversized source CDR')
            raw[kind] = path.read_bytes()
            if hashlib.sha256(raw[kind]).hexdigest() != row[kind + '_cdr_sha256']:
                raise ValueError('source CDR identity mismatch')
            hashes[str(path)] = row[kind + '_cdr_sha256']
        cloud = deserialize_message(raw['cloud'], PointCloud2)
        odom = deserialize_message(raw['odom'], Odometry)
        stamp = lambda msg: int(msg.header.stamp.sec) * 10**9 + int(msg.header.stamp.nanosec)
        if (stamp(cloud) != row['stamp_ns'] or stamp(odom) != row['stamp_ns'] or
                cloud.header.frame_id != 'mapping_reference' or odom.child_frame_id != 'mapping_reference' or
                odom.header.frame_id != 'mapping_odom'):
            raise ValueError('source message clock/frame differs from index')
        if hashlib.sha256(bytes(cloud.data)).hexdigest() != row['cloud_data_sha256']:
            raise ValueError('source cloud data hash differs from index')
        cloud, selection = select_left_cloud(cloud, source_config=runtime)
        points = pointcloud_xyz(cloud)
        finite = np.isfinite(points).all(axis=1)
        pose = rigid(row['T_prior_reference'])
        q = odom.pose.pose.orientation
        rotation = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        p = odom.pose.pose.position
        if not np.allclose(pose[:3, :3], rotation, atol=1e-7) or not np.allclose(pose[:3, 3], [p.x, p.y, p.z], atol=1e-7):
            raise ValueError('indexed pose does not match hashed odometry CDR')
        frames.append({'stamp_ns': row['stamp_ns'], 'points': points[finite], 'odometry_T': pose})
        counts.append({'stamp_ns': row['stamp_ns'], 'rows': len(points), 'finite_rows': int(finite.sum()),
                       'nonfinite_rows_excluded': int((~finite).sum()), 'selection': selection['selection']})
    return frames, counts


def render(path, wall, training, queries, result):
    import matplotlib
    matplotlib.use('Agg')
    from matplotlib import pyplot as plt
    fig, axes = plt.subplots(2, 1, figsize=(13, 8), gridspec_kw={'height_ratios': [3, 1]}, constrained_layout=True)
    ax = axes[0]
    shown = wall[::max(1, len(wall) // 50000)]
    ax.scatter(shown[:, 0], shown[:, 1], s=.6, c='#78818a', alpha=.35, label='Training-only wall observations')
    train_xy = np.asarray([f['odometry_T'][:2, 3] for f in training])
    query_xy = np.asarray([f['odometry_T'][:2, 3] for f in queries])
    ax.plot(train_xy[:, 0], train_xy[:, 1], color='#758392', linewidth=1, label='Training odometry')
    ax.plot(query_xy[:, 0], query_xy[:, 1], color='#1678bb', linestyle='--', label='Holdout causal odometry (prediction + comparison)')
    for status, color in [('MATCHED', '#21865a'), ('ODOM_ONLY', '#cf8533')]:
        xy = np.asarray([np.asarray(row['estimated_T'])[:2, 3] for row in result['frames'] if row['status'] == status])
        if xy.size:
            ax.scatter(xy[:, 0], xy[:, 1], s=10, color=color, label=status)
    ax.set_aspect('equal')
    ax.set_xlabel('replay_map x (m)')
    ax.set_ylabel('replay_map y (m)')
    ax.set_title('Single-left lidar: fixed training wall map and separate temporal holdout\nRecorded-data consistency test; no external truth or physical motion')
    ax.legend(fontsize=8, ncol=2)
    times = [(row['stamp_ns'] - queries[0]['stamp_ns']) * 1e-9 for row in result['frames']]
    values = [{'MATCHED': 2, 'ODOM_ONLY': 1, 'LOST': 0}[row['status']] for row in result['frames']]
    axes[1].step(times, values, where='post', color='#435567')
    axes[1].set_yticks([0, 1, 2], ['LOST', 'ODOM_ONLY', 'MATCHED'])
    axes[1].set_xlabel('Holdout elapsed time (s)')
    axes[1].set_ylim(-.2, 2.2)
    axes[1].grid(alpha=.2)
    fig.savefig(path, dpi=160)
    plt.close(fig)


def main(argv=None):
    args = arguments(argv)
    from wc_runtime.storage_policy import StoragePolicy
    from .rebuild import validate_frontend_row
    from .replay_localization import build_wall_map, localize_replay
    policy = StoragePolicy(args.project_root)
    policy.check()
    source = args.source_cell.absolute()
    runtime_raw = (source / 'runtime_config.json').read_bytes()
    index_raw = (source / 'frontend/index.jsonl').read_bytes()
    if len(runtime_raw) > 2 * 1024**2 or len(index_raw) > 128 * 1024**2:
        raise ValueError('source metadata resource bound exceeded')
    runtime = json.loads(runtime_raw)
    if runtime.get('mode') != 'left' or runtime.get('offline_source_selection', {}).get('selected_sides') != ['left']:
        raise ValueError('exact single-left source provenance is required')
    rows = [json.loads(line) for line in index_raw.decode('utf-8').splitlines()]
    stamps = [validate_frontend_row(row) for row in rows]
    if not stamps or len(rows) > 200000 or any(a >= b for a, b in zip(stamps, stamps[1:])):
        raise ValueError('source frames must have unique increasing stamps')
    epoch = rows[0]['stamp_ns']
    training_rows = select_rows(rows, epoch, args.training_window, args.training_interval)
    query_rows = select_rows(rows, epoch, args.query_window, args.query_interval)
    if len(training_rows) < 10 or len(query_rows) < 5:
        raise ValueError('insufficient frames in declared training/holdout windows')
    run = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ') + '_' + uuid.uuid4().hex[:8]
    output = policy.resolve('reports/navigation_replay/' + run)
    output.mkdir(parents=True, exist_ok=False)
    hashes = {str(source / 'runtime_config.json'): hashlib.sha256(runtime_raw).hexdigest(),
              str(source / 'frontend/index.jsonl'): hashlib.sha256(index_raw).hexdigest()}
    report = {'status': 'RUNNING', 'output': str(output), 'source_cell': str(source),
              'hardware_started': False, 'navigation_validated': False, 'external_truth': False,
              'coordinate_frame': 'replay_map', 'time_basis': 'ORIGINAL_CANONICAL_ARRIVAL_TIME',
              'source_motion_authority': 'FROZEN_CAUSAL_WHEEL_IMU_ODOMETRY',
              'training_window_s': args.training_window, 'query_window_s': args.query_window,
              'initial_offset_x_y_yaw_deg': args.initial_offset,
              'same_recording_shared_odometry': True,
              'interpretation': 'Temporal-holdout consistency; not independent absolute localization accuracy'}
    try:
        write(output / 'progress.json', report)
        training, train_counts = checked_frames(source, training_rows, runtime, hashes)
        queries, query_counts = checked_frames(source, query_rows, runtime, hashes)
        ground = -(float(runtime['mounts']['left']['t_axle_lidar_m'][2]) + float(runtime['wheel_candidate']['wheel_radius_m']))
        built = build_wall_map(training, ground_z=ground)
        np.savez_compressed(output / 'training_wall_map.npz', points=built['points'], training_stamps_ns=built['training_stamps_ns'])
        write(output / 'training_map.json', built['summary'])
        write(output / 'split.json', {'training_stamps_ns': built['training_stamps_ns'],
                  'query_stamps_ns': [f['stamp_ns'] for f in queries], 'epoch_ns': epoch,
                  'training_frame_counts': train_counts, 'query_frame_counts': query_counts,
                  'source_sha256': hashes, 'query_added_to_map': False})
        initial = perturbed_initial_pose(queries[0]['odometry_T'], args.initial_offset)
        result = localize_replay(built['points'], built['training_stamps_ns'], queries, initial,
                                max_gap_s=args.max_gap,
                                reference_poses={f['stamp_ns']: f['odometry_T'] for f in queries})
        result['reference_evaluation']['reference_label'] = 'SAME_FROZEN_CAUSAL_ODOMETRY_NOT_TRUTH'
        write(output / 'localization.json', result)
        with (output / 'trajectory.csv').open('w', newline='') as stream:
            writer = csv.writer(stream)
            writer.writerow(['stamp_ns', 'status', 'x_m', 'y_m', 'yaw_rad'])
            for row in result['frames']:
                pose = np.asarray(row['estimated_T']) if row['estimated_T'] is not None else None
                writer.writerow([row['stamp_ns'], row['status'], *([] if pose is None else [pose[0, 3], pose[1, 3], math.atan2(pose[1, 0], pose[0, 0])])])
        render(output / 'preview.png', built['points'], training, queries, result)
        report.update(status='COMPLETE', outcome=result['outcome'], summary=result['summary'],
                      reference_evaluation=result['reference_evaluation'],
                      wall_map=built['summary'], preview=str(output / 'preview.png'),
                      source_sha256=hashes, sources_unchanged=all(digest(p) == h for p, h in hashes.items()))
        if not report['sources_unchanged']:
            raise ValueError('source changed during replay')
    except Exception as error:
        import traceback
        report.update(status='FAIL', error=type(error).__name__ + ': ' + str(error), traceback=traceback.format_exc())
    finally:
        policy.check()
        write(output / 'result.json', report)
        print(json.dumps({k: report.get(k) for k in ['status', 'output', 'outcome', 'summary', 'error', 'preview']}, ensure_ascii=True), flush=True)
    return 0 if report['status'] == 'COMPLETE' else 1


if __name__ == '__main__':
    raise SystemExit(main())
