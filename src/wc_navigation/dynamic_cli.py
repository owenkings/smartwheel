"""Numerical stop/wait scenarios on a frozen global path; no vehicle interface."""
import argparse
from datetime import datetime, timezone
import json
import hashlib
import math
from pathlib import Path
import uuid

import numpy as np

from .cli import fingerprint, write_json


SCENARIOS = ('clear', 'persistent_block', 'temporary_block', 'crossing',
             'perception_dropout', 'localization_dropout', 'sudden_close_obstacle')


def arguments(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--project-root', required=True, type=Path)
    parser.add_argument('--plan-result', required=True, type=Path)
    parser.add_argument('--scenario', choices=('all', *SCENARIOS), default='all')
    return parser.parse_args(argv)


def path_point(path, distance):
    points = np.asarray(path, dtype=float)
    segments = np.linalg.norm(np.diff(points, axis=0), axis=1)
    cumulative = np.r_[0., np.cumsum(segments)]
    if not 0 <= distance <= cumulative[-1]:
        raise ValueError('scenario requires a sufficiently long frozen path')
    at = min(len(segments) - 1, int(np.searchsorted(cumulative, distance, side='right')) - 1)
    at = max(0, at)
    while segments[at] <= 1e-10:
        at += 1
    tangent = (points[at + 1] - points[at]) / segments[at]
    point = points[at] + tangent * (distance - cumulative[at])
    return point, tangent


def scenario_inputs(path, name, nominal_poses=None):
    """World schedules for a synthetic sensor adapter; never controller inputs."""
    settings = {'max_duration_s': 60., 'max_wait_s': 20.}
    obstacles = []
    if name in ('persistent_block', 'temporary_block'):
        point, _ = path_point(path, 2.5)
        obstacles = [{'id': name, 'center_xy': point.tolist(), 'velocity_xy': [0., 0.],
                      'radius_m': .30, 'active_from_s': 0.,
                      'active_until_s': 15. if name == 'temporary_block' else 120.}]
    elif name == 'crossing':
        point, tangent = path_point(path, 3.)
        normal = np.array([-tangent[1], tangent[0]])
        obstacles = [{'id': name, 'center_xy': (point - 3. * normal).tolist(),
                      'velocity_xy': (.25 * normal).tolist(), 'radius_m': .25,
                      'active_from_s': 0., 'active_until_s': 120.}]
    elif name == 'perception_dropout':
        settings['sensor_dropouts'] = [(4., 60.)]
    elif name == 'localization_dropout':
        settings['localization_dropouts'] = [(4., 60.)]
    elif name == 'sudden_close_obstacle':
        if nominal_poses is None:
            raise ValueError('the negative scenario requires the declared clear-world reference')
        poses = np.asarray(nominal_poses)
        at = min(range(len(poses)), key=lambda i: abs(poses[i, 0] - 3.))
        # World construction only: appear just beyond the assumed envelope,
        # with less clearance than the moving model can brake over. This
        # reference is never given to the online control decision.
        nominal = poses[at]
        point = nominal[1:3] + .85 * np.array([math.cos(nominal[3]), math.sin(nominal[3])])
        obstacles = [{'id': name, 'center_xy': point.tolist(), 'velocity_xy': [0., 0.],
                      'radius_m': .15, 'active_from_s': 3., 'active_until_s': 120.}]
    elif name != 'clear':
        raise ValueError('unknown scenario')
    return obstacles, settings


def assess_scenario(name, result):
    audit = result['audit']
    clear = audit['status'] == 'CLEAR_UNDER_STATED_MODEL'
    no_discard = result['collision_step_discarded'] is False
    events = result['events']
    dynamic_events = [e for e in events if 'DYNAMIC' in e['reason'] or e['reason'] == 'POST_STOP_MOVING_OBSTACLE']
    if name == 'sudden_close_obstacle':
        expected = 'Detect the declared close-appearance failure; do not assert safe stopping'
        satisfied = result['status'] == 'FAIL' and audit['violation_count'] > 0 and no_discard
    elif name == 'clear':
        expected = 'Reach goal with certified model sweeps'
        satisfied = result['status'] == 'REACHED' and clear and no_discard
    elif name in ('temporary_block', 'crossing'):
        expected = 'Brake/wait for the observed hazard, then resume and reach'
        satisfied = (result['status'] == 'REACHED' and clear and no_discard and bool(dynamic_events)
                     and result['stop_count'] >= 1 and result['resume_count'] >= 1)
    else:
        reason = {'persistent_block': None, 'perception_dropout': 'OBSERVATIONS_STALE',
                  'localization_dropout': 'LOCALIZATION_UNTRUSTED'}[name]
        expected = 'Stop and remain waiting through the declared audit horizon'
        satisfied = (result['status'] == 'WAITING' and clear and no_discard
                     and abs(result['final_speed_mps']) < 1e-9
                     and abs(result['final_angular_speed_radps']) < 1e-9
                     and (bool(dynamic_events) if reason is None else any(e['reason'] == reason for e in events))
                     and (result['stop_count'] == 1 and result['resume_count'] == 0 if name == 'persistent_block' else True))
    return {'expected': expected, 'satisfied': bool(satisfied),
            'meaning': 'NUMERICAL_SCENARIO_CHECK_NOT_PHYSICAL_ACCEPTANCE'}


def render(destination, inflated, path, results):
    import matplotlib
    matplotlib.use('Agg')
    from matplotlib import pyplot as plt
    fig, axes = plt.subplots(2, 1, figsize=(13, 8), constrained_layout=True)
    grid = inflated.grid
    background = np.full(grid.data.shape, .55)
    background[grid.data == 0] = 1.
    background[grid.data == 100] = .08
    ox, oy = grid.origin[:2]
    # Frozen experiment maps are axis aligned; rotated input is still simulated,
    # but must not be rendered as an axis-aligned map.
    if abs(grid.origin[2]) > 1e-10:
        raise ValueError('preview requires an axis-aligned map')
    axes[0].imshow(background, origin='lower', cmap='gray', vmin=0, vmax=1,
                   extent=[ox, ox+grid.width*grid.resolution, oy, oy+grid.height*grid.resolution])
    points = np.asarray(path)
    axes[0].plot(points[:, 0], points[:, 1], '--', color='#303030', label='Frozen Nav2 path')
    for name, result in results.items():
        poses = np.asarray(result.get('poses', []))
        if poses.size:
            axes[0].plot(poses[:, 1], poses[:, 2], linewidth=1.2, label=name)
            axes[1].plot(poses[:, 0], poses[:, 4], linewidth=1.2, label=name)
    stop_point, _ = path_point(path, 2.5)
    axes[0].scatter(*stop_point, marker='x', s=60, color='#ba493d', label='Synthetic blocking-disc center')
    axes[0].set_xlim(points[:, 0].min()-1., points[:, 0].max()+1.)
    axes[0].set_ylim(points[:, 1].min()-1.2, points[:, 1].max()+1.2)
    axes[0].legend(fontsize=8, ncol=2)
    axes[0].set_aspect('equal'); axes[0].set_xlabel('Map x (m)'); axes[0].set_ylabel('Map y (m)')
    axes[0].set_title('Frozen recorded map + synthetic dynamic scenarios')
    axes[1].set_xlabel('Simulated time (s)'); axes[1].set_ylabel('Linear speed (m/s)')
    axes[1].grid(alpha=.2); axes[1].legend(fontsize=8, ncol=3)
    fig.suptitle('Numerical stop / wait / resume only\nAssumed footprint and braking; no physical localization, sensor or vehicle validation')
    fig.savefig(destination, dpi=150); plt.close(fig)


def main(argv=None):
    args = arguments(argv)
    from wc_runtime.map_viewer import inspect_grid
    from wc_runtime.storage_policy import StoragePolicy
    from .geometry import Grid, inflate_grid, _validated_path
    from .dynamic_simulation import simulate_dynamic_path
    policy = StoragePolicy(args.project_root); policy.check()
    plan_file = args.plan_result.absolute()
    raw = plan_file.read_bytes()
    saved = json.loads(raw)
    if saved.get('status') != 'PASS' or saved.get('mode') != 'OFFLINE_NAV2_AND_NUMERICAL_SIMULATION':
        raise ValueError('a completed offline Nav2 plan result is required')
    path = _validated_path(saved['planner']['path']).tolist()
    if sum(math.dist(a, b) for a, b in zip(path, path[1:])) < 4.:
        raise ValueError('dynamic demonstration needs at least a four-meter frozen path')
    map_yaml = Path(saved['source_map'])
    stats, data, meta = inspect_grid(map_yaml)
    hashes = saved['source_sha256']
    if (set(hashes) != {str(map_yaml), stats['image']} or
            any(fingerprint(p) != h for p, h in hashes.items())):
        raise ValueError('frozen source map identity differs from the successful plan')
    grid = Grid(data, meta['resolution'], tuple(meta['origin']))
    footprint = saved['footprint']
    if footprint.get('model') != 'assumed_circle' or footprint.get('measured') is not False:
        raise ValueError('explicit numerical circular footprint assumption required')
    inflated = inflate_grid(grid, footprint['radius_m'], footprint['margin_m'])
    run = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ') + '_' + uuid.uuid4().hex[:8]
    output = policy.resolve('reports/navigation_dynamic/' + run); output.mkdir(parents=True, exist_ok=False)
    names = SCENARIOS if args.scenario == 'all' else (args.scenario,)
    report = {'schema_version': 1, 'mode': 'NUMERICAL_DYNAMIC_STOP_WAIT', 'output': str(output),
              'plan_result': str(plan_file), 'plan_sha256': hashlib.sha256(raw).hexdigest(),
              'source_sha256': hashes, 'footprint': footprint, 'scenarios': {},
              'localization_source': 'VIRTUAL_MODEL_POSE_NO_REAL_LOCALIZATION',
              'obstacle_source': 'SYNTHETIC_DISCS_WITH_ASSUMED_OBSERVATIONS_AND_VELOCITIES',
              'hardware_started': False, 'navigation_validated': False,
              'physical_test_performed': False, 'rerouting_implemented': False}
    results = {}
    for name in names:
        reference = None
        if name == 'sudden_close_obstacle':
            reference = results.get('clear')
            if reference is None:
                reference = simulate_dynamic_path(inflated, path, (), max_duration_s=60., max_wait_s=20.)
            reference = reference['poses']
        obstacles, settings = scenario_inputs(path, name, reference)
        write_json(output / (name + '_world.json'), {'obstacles': obstacles, 'settings': settings})
        result = simulate_dynamic_path(inflated, path, obstacles, **settings)
        results[name] = result
        write_json(output / (name + '.json'), result)
        report['scenarios'][name] = {k: v for k, v in result.items() if k not in ('poses', 'decisions')}
        report['scenarios'][name]['expectation_check'] = assess_scenario(name, result)
    report['status'] = 'SCENARIOS_EXECUTED_REQUIRES_REVIEW'
    report['scenario_contracts_satisfied'] = all(v['expectation_check']['satisfied'] for v in report['scenarios'].values())
    report['contact_interpretation'] = 'ASSUMED_RADIUS_PLUS_MARGIN_ENVELOPE_CONTACT_NOT_MEASURED_BODY_CONTACT'
    report['stop_resume_count_note'] = 'MOTION_STOP_RESUME_EXCLUDE_INITIAL_READINESS_SEPARATE_HOLD_RELEASE_COUNT'
    report['sources_unchanged'] = (fingerprint(plan_file) == report['plan_sha256'] and
                                   all(fingerprint(p) == h for p, h in hashes.items()))
    try:
        render(output / 'preview.png', inflated, path, results)
        report['preview'] = str(output / 'preview.png')
    except Exception as exc:
        report['preview_error'] = str(exc)
    policy.check(); write_json(output / 'result.json', report)
    print(json.dumps(report, ensure_ascii=True, allow_nan=False), flush=True)
    return 0 if report['sources_unchanged'] and report['scenario_contracts_satisfied'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
