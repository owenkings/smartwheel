"""Plan on a saved map with Nav2 and track the path in a numerical model."""
import argparse
import csv
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import sys
import uuid


def positive(value):
    number = float(value)
    if not math.isfinite(number) or number <= 0:
        raise argparse.ArgumentTypeError('expected a finite positive number')
    return number


def nonnegative(value):
    number = float(value)
    if not math.isfinite(number) or number < 0:
        raise argparse.ArgumentTypeError('expected a finite nonnegative number')
    return number


def arguments(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--project-root', type=Path, required=True)
    parser.add_argument('--map', dest='map_yaml', type=Path, required=True)
    parser.add_argument('--map-kind', choices=['recorded', 'synthetic'], default='recorded')
    parser.add_argument('--assumed-radius', type=positive, required=True,
                        help='explicit hypothetical circle radius, meters; not a measured footprint')
    parser.add_argument('--margin', type=nonnegative, default=0.10)
    parser.add_argument('--demo', action='store_true', help='choose virtual start/goal in the largest free component')
    parser.add_argument('--start', nargs=2, type=float, metavar=('X', 'Y'))
    parser.add_argument('--goal', nargs=2, type=float, metavar=('X', 'Y'))
    parser.add_argument('--max-duration', type=positive, default=300)
    parser.add_argument('--planner-timeout', type=positive, default=75)
    args = parser.parse_args(argv)
    if args.demo:
        if args.start is not None or args.goal is not None:
            parser.error('--demo cannot be combined with --start/--goal')
    elif args.start is None or args.goal is None:
        parser.error('provide --demo or both --start X Y and --goal X Y')
    for values in (args.start, args.goal):
        if values is not None and not all(math.isfinite(v) for v in values):
            parser.error('start and goal must be finite map coordinates')
    return args


def fingerprint(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def write_json(path, value):
    Path(path).write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + '\n', encoding='utf-8')


def render_preview(destination, inflated, plan, simulation, map_kind='recorded'):
    import matplotlib
    matplotlib.use('Agg')
    from matplotlib import pyplot as plt
    import numpy as np
    grid = inflated.grid
    data = np.asarray(grid.data)
    image = np.full(data.shape, 0.55)
    image[data == 0] = 1.0
    image[data == 100] = 0.06
    figure_size = (9, 7) if grid.width / grid.height < 2 else (15, 5)
    fig, ax = plt.subplots(figsize=figure_size, constrained_layout=True)
    ax.imshow(image, origin='lower', cmap='gray', vmin=0, vmax=1)
    overlay = np.zeros((*data.shape, 4))
    overlay[(data == 0) & inflated.blocked] = [0.96, 0.56, 0.16, 0.45]
    ax.imshow(overlay, origin='lower')
    def pixels(points):
        points = [grid.world_to_grid(p) for p in points]
        return np.asarray(points) - 0.5
    if plan:
        xy = pixels(plan)
        ax.plot(xy[:, 0], xy[:, 1], color='#1678bb', linewidth=2.2, label='Nav2 planned path')
        ax.scatter(*xy[0], c='#1d985b', s=55, zorder=5, label='Virtual start')
        ax.scatter(*xy[-1], c='#c74638', s=65, marker='*', zorder=5, label='Virtual goal')
    poses = simulation.get('poses', [])
    if poses:
        xy = pixels([(p[1], p[2]) for p in poses])
        ax.plot(xy[:, 0], xy[:, 1], color='#9a43a6', linewidth=1.4, linestyle='--', label='Numerical differential-drive track')
    title = 'Synthetic L-shaped corridor' if map_kind == 'synthetic' else 'Recorded corridor map'
    ax.set_title(title + ' | Nav2 planning + numerical tracking\n'
                 'Hypothetical circle footprint; software simulation only')
    ax.set_xlabel('Map grid column')
    ax.set_ylabel('Map grid row')
    ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.15), ncol=3, fontsize=8)
    ax.set_aspect('equal')
    fig.savefig(destination, dpi=170)
    plt.close(fig)


def main(argv=None):
    args = arguments(argv)
    from wc_runtime.map_viewer import inspect_grid
    from wc_runtime.storage_policy import StoragePolicy
    from .geometry import Grid, inflate_grid, demo_endpoints, simulate_path, connected_components
    from .nav2_planner import plan_nav2
    storage = StoragePolicy(args.project_root)
    storage.check()
    run_id = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ') + '_' + uuid.uuid4().hex[:8]
    output = storage.resolve('reports/navigation/' + run_id)
    output.mkdir(parents=True, exist_ok=False)
    report = {
        'schema_version': 1, 'run_id': run_id, 'status': 'STARTING',
        'mode': 'OFFLINE_NAV2_AND_NUMERICAL_SIMULATION',
        'hardware_control': False, 'navigation_validated': False,
        'localization_source': 'VIRTUAL_POSE_NO_REAL_LOCALIZATION',
        'physical_test_performed': False,
        'footprint': {'model': 'assumed_circle', 'radius_m': args.assumed_radius,
                      'margin_m': args.margin, 'measured': False},
        'unknown_space': 'BLOCKED', 'map_boundary': 'BLOCKED',
        'output': str(output), 'source_map': str(args.map_yaml.absolute()),
        'map_kind': args.map_kind,
        'ros_domain_id': os.environ.get('ROS_DOMAIN_ID'),
        'ros_localhost_only': os.environ.get('ROS_LOCALHOST_ONLY'),
    }
    plan = []
    simulation = {}
    inflated = None
    try:
        stats, data, meta = inspect_grid(args.map_yaml.absolute())
        report['map'] = stats
        source_paths = [args.map_yaml.absolute(), Path(stats['image'])]
        source_hashes = {str(p): fingerprint(p) for p in source_paths}
        report['source_sha256'] = source_hashes
        grid = Grid(data, meta['resolution'], tuple(meta['origin']))
        inflated = inflate_grid(grid, args.assumed_radius, args.margin)
        components = connected_components(inflated)
        report['traversability'] = {
            'free_cells_after_inflation': int(inflated.free.sum()),
            'connected_components': len(components),
            'largest_component_cells': len(components[0]) if components else 0,
            'largest_component_area_m2': (len(components[0]) if components else 0) * grid.resolution ** 2,
            'map_validated_for_real_navigation': False,
        }
        if args.demo:
            endpoint_grid = inflate_grid(grid, args.assumed_radius + math.sqrt(2) * grid.resolution, args.margin)
            start, goal = demo_endpoints(endpoint_grid)
        else:
            start, goal = tuple(args.start), tuple(args.goal)
        if not inflated.is_free_world(start) or not inflated.is_free_world(goal):
            raise ValueError('start/goal is outside inflated free space')
        report.update(start=list(start), goal=list(goal), endpoint_source='AUTO_DEMO_VIRTUAL' if args.demo else 'EXPLICIT_VIRTUAL')
        report['endpoint_distance_m'] = math.dist(start, goal)
        write_json(output / 'preflight.json', report)
        result = plan_nav2(args.map_yaml.absolute(), start, goal, args.assumed_radius,
                           args.margin, output, timeout_s=args.planner_timeout)
        plan = result['path']
        report['planner'] = result
        if (len(plan) < 2 or not inflated.is_free_world(plan[0]) or
                any(not inflated.segment_is_free(a, b) for a, b in zip(plan, plan[1:]))):
            raise ValueError('Nav2 path failed independent swept-footprint validation')
        simulation = simulate_path(inflated, plan, max_duration_s=args.max_duration)
        with (output / 'trajectory.csv').open('w', newline='', encoding='utf-8') as stream:
            writer = csv.writer(stream)
            writer.writerow(['time_s', 'x_m', 'y_m', 'yaw_rad', 'v_mps', 'w_radps'])
            writer.writerows(simulation.get('poses', []))
        write_json(output / 'simulation.json', simulation)
        report['simulation'] = {k: v for k, v in simulation.items() if k != 'poses'}
        report['path_length_m'] = sum(math.dist(a, b) for a, b in zip(plan, plan[1:]))
        report['status'] = 'PASS' if simulation.get('status') == 'REACHED' else 'FAIL'
        report['sources_unchanged'] = all(fingerprint(p) == source_hashes[str(p)] for p in source_paths)
        if not report['sources_unchanged']:
            raise ValueError('source map changed while planning')
    except Exception as exc:
        report.update(status='FAIL', error=type(exc).__name__ + ': ' + str(exc))
    finally:
        storage.check()
        if inflated is not None:
            try:
                render_preview(output / 'preview.png', inflated, plan, simulation, args.map_kind)
                report['preview'] = str(output / 'preview.png')
            except Exception as exc:
                report['preview_error'] = str(exc)
        write_json(output / 'result.json', report)
        print(json.dumps(report, ensure_ascii=True, allow_nan=False), flush=True)
    return 0 if report['status'] == 'PASS' else 1


if __name__ == '__main__':
    raise SystemExit(main())
