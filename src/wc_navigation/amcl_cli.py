"""AMCL interface check on a rebuilt map using the same frozen recording."""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import uuid

import numpy as np

from .replay_cli import checked_frames, digest, number, perturbed_initial_pose, select_rows, write


def arguments(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--project-root', required=True, type=Path)
    parser.add_argument('--source-cell', required=True, type=Path)
    parser.add_argument('--map', dest='map_yaml', required=True, type=Path)
    parser.add_argument('--window', type=number, nargs=2, default=[62., 180.])
    parser.add_argument('--interval', type=number, default=0., help='0 preserves every frame; positive seconds explicitly decimate')
    parser.add_argument('--initial-offset', type=number, nargs=3, default=[.15, .10, 3.])
    parser.add_argument('--timeout', type=number, default=180.)
    parser.add_argument('--scan-height', type=number, nargs=2, default=[-.50, .50])
    parser.add_argument('--axle-reference', action='store_true', default=True,
                        help='use the frozen planar axle-to-lidar lever arm for the differential model')
    parser.add_argument('--lidar-reference-diagnostic', action='store_false', dest='axle_reference',
                        help='reproduce the earlier lidar-centered motion-model diagnostic')
    parser.add_argument('--laser-model', choices=['likelihood_field', 'likelihood_field_prob'], default='likelihood_field')
    args = parser.parse_args(argv)
    if not 0 <= args.window[0] < args.window[1] or not 0 <= args.interval <= 5 or not 10 <= args.timeout <= 900:
        parser.error('valid nonnegative window, interval [0,5], timeout [10,900] required')
    if math.hypot(*args.initial_offset[:2]) > .3 or abs(args.initial_offset[2]) > 7:
        parser.error('initial offset must remain within the stated local initialization range')
    if not -2 < args.scan_height[0] < args.scan_height[1] < 2:
        parser.error('ordered finite scan height band inside (-2,2) meters required')
    return args


def render(path, map_yaml, frames, result, base_from_lidar=None):
    import matplotlib
    matplotlib.use('Agg')
    from matplotlib import pyplot as plt
    from wc_runtime.map_viewer import inspect_grid
    stats, data, meta = inspect_grid(map_yaml)
    image = np.full(data.shape, .55)
    image[data == 0] = 1.
    image[data == 100] = .08
    ox, oy = meta['origin'][:2]
    r = meta['resolution']
    fig, ax = plt.subplots(figsize=(14, 5), constrained_layout=True)
    ax.imshow(image, origin='lower', cmap='gray', vmin=0, vmax=1,
              extent=[ox, ox + data.shape[1] * r, oy, oy + data.shape[0] * r])
    to_base = np.linalg.inv(base_from_lidar) if base_from_lidar is not None else np.eye(4)
    prior = np.asarray([(f['odometry_T'] @ to_base)[:2, 3] for f in frames])
    ax.plot(prior[:, 0], prior[:, 1], '--', color='#257bb2', linewidth=1, label='Frozen wheel/IMU prior')
    poses = result.get('trajectory', [])
    if poses:
        label = 'AMCL candidate axle-reference pose' if base_from_lidar is not None else 'AMCL lidar-reference pose'
        ax.plot([p['x_m'] for p in poses], [p['y_m'] for p in poses], color='#a33da1', linewidth=1.4, label=label)
    scope = result.get('evaluation_scope', 'UNSPECIFIED')
    ax.set_title('AMCL replay: ' + scope.replace('_', ' ') + '\nPhysical navigation remains unvalidated', fontsize=10)
    ax.set_xlabel('map x (m)')
    ax.set_ylabel('map y (m)')
    ax.set_aspect('equal')
    ax.legend(fontsize=8)
    fig.savefig(path, dpi=160)
    plt.close(fig)


def prior_disagreement(frames, result, base_from_lidar=None):
    """Flag shared-prior disagreement for review; neither input is physical truth."""
    from wc_runtime.offline_planar_registration import Policy
    policy = Policy()
    to_base = np.eye(4) if base_from_lidar is None else np.linalg.inv(base_from_lidar)
    prior = {f['stamp_ns']: f['odometry_T'] @ to_base for f in frames}
    translation, yaw, stamps = [], [], []
    for row in result.get('trajectory', []):
        if row['stamp_ns'] not in prior:
            raise ValueError('AMCL trajectory stamp absent from input prior')
        pose = prior[row['stamp_ns']]
        delta = row['yaw_rad'] - math.atan2(pose[1, 0], pose[0, 0])
        translation.append(math.hypot(row['x_m'] - pose[0, 3], row['y_m'] - pose[1, 3]))
        yaw.append(abs(math.degrees(math.atan2(math.sin(delta), math.cos(delta)))))
        stamps.append(row['stamp_ns'])
    flagged = [stamp for stamp, xy, angle in zip(stamps, translation, yaw)
               if xy > policy.max_correction_translation_m or angle > policy.max_correction_yaw_deg]
    stats = lambda values: ({'median': float(np.median(values)), 'p95': float(np.quantile(values, .95)),
                             'max': float(max(values))} if values else None)
    return {'status': 'REVIEW_REQUIRED_ODOMETRY_DISAGREEMENT' if flagged else 'UNVALIDATED',
            'compared_frames': len(stamps), 'flagged_frames': len(flagged),
            'first_flagged_stamp_ns': flagged[0] if flagged else None,
            'translation_m': stats(translation), 'yaw_deg': stats(yaw),
            'review_translation_m': policy.max_correction_translation_m,
            'review_yaw_deg': policy.max_correction_yaw_deg,
            'threshold_source': 'EXISTING_PLANAR_POLICY_USED_AS_CUMULATIVE_REVIEW_TRIGGER_NOT_ACCURACY_GATE',
            'interpretation': 'SHARED_ODOMETRY_DISAGREEMENT_NOT_GROUND_TRUTH_ERROR',
            'physical_navigation_validated': False}


def replay_map_coverage(map_yaml, frames, result, base_from_lidar=None):
    """Diagnose coverage/translation ambiguity at returned poses, without truth feedback."""
    from wc_runtime.map_viewer import inspect_grid
    from .geometry import Grid
    from .amcl_replay import build_scan, planar_base_from_lidar
    from .localization_coverage import CoverageAnalyzer
    _, data, meta = inspect_grid(Path(map_yaml))
    analyzer = CoverageAnalyzer(Grid(data, meta['resolution'], tuple(meta['origin'])))
    fixed = np.eye(4) if base_from_lidar is None else planar_base_from_lidar(base_from_lidar)
    by_stamp = {f['stamp_ns']: f for f in frames}
    settings = result['scan_settings']
    angles = settings['angle_min_rad'] + np.arange(settings['beams']) * settings['angle_increment_rad']
    reports = []
    for row in result.get('trajectory', []):
        frame = by_stamp[row['stamp_ns']]
        yaw = row['yaw_rad']
        c, s = math.cos(yaw), math.sin(yaw)
        base = np.eye(4)
        base[:2, :2] = [[c, -s], [s, c]]
        base[:2, 3] = [row['x_m'], row['y_m']]
        laser = base @ fixed
        ranges, _ = build_scan(frame['points'], settings['height_min_m'], settings['height_max_m'])
        report = analyzer.analyze(ranges, angles,
                                  [laser[0, 3], laser[1, 3], math.atan2(laser[1, 0], laser[0, 0])],
                                  heading_yaw_rad=yaw)
        reports.append({'stamp_ns': row['stamp_ns'], **report})
    incomplete = [r['stamp_ns'] for r in reports if r['coverage_insufficient']]
    ambiguous = [r['stamp_ns'] for r in reports if r['along_heading_ambiguity_evidence']]
    summary = {'status': 'REVIEW_REQUIRED' if incomplete or ambiguous or not reports else 'UNVALIDATED',
               'analyzed_frames': len(reports), 'insufficient_coverage_frames': len(incomplete),
               'along_heading_ambiguity_frames': len(ambiguous),
               'first_insufficient_coverage_stamp_ns': incomplete[0] if incomplete else None,
               'first_ambiguity_stamp_ns': ambiguous[0] if ambiguous else None,
               'interpretation': 'POSE_DEPENDENT_ENDPOINT_DIAGNOSTICS_NOT_TRUTH_OR_VISIBILITY_TEST',
               'physical_navigation_validated': False}
    return {'summary': summary, 'frames': reports}


def main(argv=None):
    args = arguments(argv)
    from wc_runtime.storage_policy import StoragePolicy
    from .rebuild import validate_frontend_row
    from .amcl_replay import run_amcl_replay
    policy = StoragePolicy(args.project_root)
    policy.check()
    source = args.source_cell.absolute()
    runtime_raw = (source / 'runtime_config.json').read_bytes()
    index_raw = (source / 'frontend/index.jsonl').read_bytes()
    runtime = json.loads(runtime_raw)
    if runtime.get('mode') != 'left' or runtime.get('offline_source_selection', {}).get('selected_sides') != ['left']:
        raise ValueError('exact frozen single-left source is required')
    rows = [json.loads(line) for line in index_raw.decode('utf-8').splitlines()]
    stamps = [validate_frontend_row(row) for row in rows]
    if not stamps or any(a >= b for a, b in zip(stamps, stamps[1:])):
        raise ValueError('unique increasing source timestamps required')
    available = select_rows(rows, stamps[0], args.window, 0.)
    selected = select_rows(rows, stamps[0], args.window, args.interval)
    if len(selected) < 5:
        raise ValueError('at least five replay frames required')
    hashes = {str(source / 'runtime_config.json'): hashlib.sha256(runtime_raw).hexdigest(),
              str(source / 'frontend/index.jsonl'): hashlib.sha256(index_raw).hexdigest()}
    frames, counts = checked_frames(source, selected, runtime, hashes)
    initial = perturbed_initial_pose(frames[0]['odometry_T'], args.initial_offset)
    base_from_lidar = None
    if args.axle_reference:
        if runtime.get('motion_model') != 'planar_ekf':
            raise ValueError('frozen planar FLU reference is required for the axle lever arm')
        base_from_lidar = np.eye(4)
        base_from_lidar[:2, 3] = np.asarray(runtime['mounts']['left']['t_axle_lidar_m'][:2], dtype=float)
        if not np.isfinite(base_from_lidar).all():
            raise ValueError('finite frozen planar lever arm required')
    run = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ') + '_' + uuid.uuid4().hex[:8]
    output = policy.resolve('reports/navigation_amcl/' + run)
    output.mkdir(parents=True, exist_ok=False)
    write(output / 'input_contract.json', {'source_cell': str(source), 'source_sha256': hashes,
          'available_window_frames': len(available), 'omitted_window_frames': len(available)-len(selected),
          'laser_model': args.laser_model, 'frame_counts': counts, 'window_s': args.window, 'interval_s': args.interval,
          'initial_offset_x_y_yaw_deg': args.initial_offset,
          'scan_height_m': args.scan_height, 'candidate_axle_reference': args.axle_reference,
          'base_from_lidar': base_from_lidar.tolist() if base_from_lidar is not None else None,
          'map_reuses_same_recording': True, 'independent_holdout': False,
          'hardware_started': False, 'navigation_validated': False})
    result = run_amcl_replay(args.map_yaml.absolute(), frames, initial, output, args.timeout,
                             scan_height_m=tuple(args.scan_height), base_from_lidar=base_from_lidar, laser_model=args.laser_model)
    summary = {k: result.get(k) for k in ['status', 'input_frames', 'published_frames', 'acknowledged_frames',
               'pose_frames', 'evaluation_scope', 'global_relocalization_performed', 'error', 'wall_elapsed_s']}
    poses = result.get('trajectory', [])
    if poses:
        for name, index in [('x_std_m', 0), ('y_std_m', 7), ('yaw_std_deg', 35)]:
            values = np.sqrt(np.maximum(0., [p['covariance'][index] for p in poses]))
            if index == 35:
                values = np.degrees(values)
            summary[name] = {'median': float(np.median(values)), 'p95': float(np.quantile(values, .95)), 'max': float(values.max())}
    summary.update(output=str(output), source_sha256=hashes,
                   sources_unchanged=all(digest(p) == h for p, h in hashes.items()),
                   hardware_started=False, navigation_validated=False,
                   covariance_interpretation='Particle-model covariance, not calibrated physical accuracy',
                   candidate_axle_reference=args.axle_reference, scan_height_m=args.scan_height,
                   laser_model=args.laser_model, available_window_frames=len(available),
                   omitted_window_frames=len(available)-len(selected),
                   parameter_readback_verified=bool(result.get('parameter_readback')) and result['status']=='AMCL_REPLAY_COMPLETE')
    summary['prior_disagreement'] = prior_disagreement(frames, result, base_from_lidar)
    summary['localization_status'] = summary['prior_disagreement']['status']
    coverage = replay_map_coverage(args.map_yaml, frames, result, base_from_lidar)
    write(output / 'coverage.json', coverage)
    summary['map_coverage'] = coverage['summary']
    summary['map_coverage_report'] = str(output / 'coverage.json')
    if coverage['summary']['status'] == 'REVIEW_REQUIRED':
        summary['localization_status'] = 'REVIEW_REQUIRED_MAP_COVERAGE_OR_AMBIGUITY'
    try:
        render(output / 'preview.png', args.map_yaml.absolute(), frames, result, base_from_lidar)
        summary['preview'] = str(output / 'preview.png')
    except Exception as error:
        summary['preview_error'] = str(error)
    policy.check()
    write(output / 'summary.json', summary)
    print(json.dumps(summary, ensure_ascii=True, allow_nan=False), flush=True)
    return 0 if result['status'] == 'AMCL_REPLAY_COMPLETE' and summary['sources_unchanged'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
