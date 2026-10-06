#!/usr/bin/env python3
"""Render immutable native route1 A/B exports with identical bounds and scales.

No ROS, fitting, geometric filtering or map regeneration. Uniform original-row
stride is used only for point display, identically for both map exports.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import sys


NAMES = ('se3_no_filter', 'planar_filter')
LABELS = {'se3_no_filter': 'SE3 baseline', 'planar_filter': 'Planar EKF + filters + native grid profile'}
COLORS = {'se3_no_filter': '#ca5f1b', 'planar_filter': '#1677a3'}


def require(condition, message):
    if not condition:
        raise ValueError(message)


def digest(path):
    result = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024*1024), b''):
            result.update(block)
    return result.hexdigest()


def original_file(native_root, value):
    path = Path(value)
    if not path.is_absolute():
        path = native_root/path
    require('..' not in path.parts and all(not p.is_symlink() for p in (path, *path.parents)), 'Linked/traversing native artifact')
    require(path.resolve().is_relative_to(native_root) and path.is_file(), 'Native artifact outside result directory')
    return path.resolve()


def cloud_display(path, stride, expected_count, ply_blocks, np):
    """Audit all rows; select the same row modulo stride without spatial masks."""
    count = finite = 0
    low, high, blocks = np.full(3, np.inf), np.full(3, -np.inf), []
    for xyz in ply_blocks(path):
        mask = np.isfinite(xyz).all(axis=1)
        valid = xyz[mask]
        finite += len(valid)
        if len(valid):
            low, high = np.minimum(low, valid.min(axis=0)), np.maximum(high, valid.max(axis=0))
        selected = ((np.arange(len(xyz))+count) % stride == 0) & mask
        blocks.append(xyz[selected])
        count += len(xyz)
    require(count == expected_count and finite > 0, 'PLY row count differs from native export evidence')
    shown = np.concatenate(blocks, axis=0)
    require(len(shown) > 0, 'Uniform display sample has no finite point; do not invent substitute rows')
    return shown, {'vertices': count, 'finite_xyz': finite, 'nonfinite_xyz': count-finite,
                   'min_m': low.tolist(), 'max_m': high.tolist(), 'extent_m': (high-low).tolist(),
                   'display_points': len(shown), 'display_stride': stride, 'display_row_rule': 'original_row_index % stride == 0'}


def final_graph(rows, np):
    require(rows, 'No native graph snapshot')
    row = rows[-1]
    ids, points = np.asarray(row['ids']), np.asarray(row['positions'], dtype=float)
    require(ids.ndim == 1 and points.shape == (len(ids), 3) and len(ids) > 0
            and len(set(ids.tolist())) == len(ids) and np.isfinite(points).all(), 'Invalid final native graph')
    order = np.argsort(ids)
    return ids[order], points[order], row['stamp_ns']


def padded_bounds(low, high, np):
    span = np.maximum(high-low, .1)
    return low-.03*span, high+.03*span


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--native', type=Path, required=True, help='Native A/B top-level result.json')
    parser.add_argument('--output', type=Path, required=True, help='New project directory for PNG and summary.json')
    parser.add_argument('--project-root', type=Path, default=Path(__file__).resolve().parents[2])
    parser.add_argument('--max-display-points', type=int, default=100000, help='Per-map upper bound through one shared row stride')
    args = parser.parse_args(argv)
    root, source = args.project_root.resolve(), args.native.resolve()
    require(source.is_file() and source.is_relative_to(root), 'Native result must exist inside the project')
    require(1000 <= args.max_display_points <= 1000000, 'Display point bound must be 1000..1000000')
    output = args.output if args.output.is_absolute() else root/args.output
    require('..' not in output.parts and all(not p.is_symlink() for p in (output, *output.parents)), 'Unsafe output directory')
    require(output.is_relative_to(root) and output != root and not output.exists(), 'Output must be a new project subdirectory')
    require(not source.is_relative_to(output), 'Output cannot contain the input result')
    for key in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS'):
        os.environ[key] = '1'
    import numpy as np
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.colors import ListedColormap, BoundaryNorm, Normalize
    from matplotlib.transforms import Affine2D
    sys.path.insert(0, str(root/'src'))
    from wc_runtime.map_viewer import ply_blocks, inspect_grid
    report = json.loads(source.read_text(encoding='utf-8'))
    require(report['status'] == 'NATIVE_AB_COMPLETE_REQUIRES_VISUAL_REVIEW', 'Native A/B did not finish')
    require(all(name in report['cells'] for name in NAMES), 'Both baseline and route1 native cells are required')
    hashes = {str(source): digest(source)}
    maximum = max(report['cells'][name]['map_export']['clouds'][0]['vertices'] for name in NAMES)
    stride = max(1, int(math.ceil(maximum/args.max_display_points)))
    cells = {}
    for name in NAMES:
        native = report['cells'][name]
        require(native['status'] == 'NATIVE_REPLAY_AND_EXPORT_COMPLETE_REQUIRES_VISUAL_REVIEW', 'Incomplete native cell')
        exported = native['map_export']
        require(len(exported['clouds']) == len(exported['maps']) == 1, 'Exactly one original PLY/PGM map per cell required')
        old_cloud, old_grid = exported['clouds'][0], exported['maps'][0]
        cloud_file = original_file(source.parent, old_cloud['file'])
        yaml_file = original_file(source.parent, old_grid['yaml'])
        require(digest(cloud_file) == old_cloud['sha256'], 'PLY hash differs from native export')
        grid_report, grid, meta = inspect_grid(yaml_file)
        require(grid_report['yaml_fingerprint'] == old_grid['yaml_fingerprint']
                and grid_report['image_fingerprint'] == old_grid['image_fingerprint'], 'Grid differs from native export')
        image_file = original_file(source.parent, grid_report['image'])
        for path in (cloud_file, yaml_file, image_file):
            hashes[str(path)] = digest(path)
        points, cloud_report = cloud_display(cloud_file, stride, old_cloud['vertices'], ply_blocks, np)
        ids, trajectory, graph_stamp = final_graph(native['graphs'], np)
        cells[name] = {'points': points, 'cloud': cloud_report, 'grid': grid, 'grid_report': grid_report,
                       'meta': meta, 'trajectory': trajectory, 'node_ids': ids, 'graph_stamp_ns': graph_stamp}
    low = np.min([cell['cloud']['min_m'] for cell in cells.values()], axis=0)
    high = np.max([cell['cloud']['max_m'] for cell in cells.values()], axis=0)
    low = np.minimum(low, np.min(np.concatenate([cell['trajectory'] for cell in cells.values()]), axis=0))
    high = np.maximum(high, np.max(np.concatenate([cell['trajectory'] for cell in cells.values()]), axis=0))
    limits_low, limits_high = padded_bounds(low, high, np)
    z_norm = Normalize(vmin=float(low[2]), vmax=float(max(high[2], low[2]+.001)))
    grid_low = np.min([cell['grid_report']['world_xy_min_m'] for cell in cells.values()], axis=0)
    grid_high = np.max([cell['grid_report']['world_xy_max_m'] for cell in cells.values()], axis=0)
    trajectory_all = np.concatenate([cell['trajectory'] for cell in cells.values()])
    grid_low, grid_high = padded_bounds(np.minimum(grid_low, trajectory_all[:, :2].min(axis=0)),
                                        np.maximum(grid_high, trajectory_all[:, :2].max(axis=0)), np)
    output.mkdir(parents=True)
    plt.rcParams.update({'font.size': 10, 'axes.titlesize': 11, 'savefig.facecolor': 'white'})
    files = []

    def save(figure, filename):
        path = output/filename
        figure.savefig(path, dpi=180)
        plt.close(figure)
        files.append({'file': filename, 'sha256': digest(path)})

    cmap = ListedColormap(['#c6c9cd', '#ffffff', '#16191d'])
    discrete = BoundaryNorm([-.5, .5, 1.5, 2.5], cmap.N)
    fig, axes = plt.subplots(1, 2, figsize=(13, 6), constrained_layout=True)
    for ax, name in zip(axes, NAMES):
        cell = cells[name]
        grid, meta = cell['grid'], cell['meta']
        display = np.where(grid < 0, 0, np.where(grid == 0, 1, 2))
        h, w = grid.shape
        x, y, yaw = meta['origin']
        transform = Affine2D().rotate(yaw).translate(x, y)+ax.transData
        ax.imshow(display, origin='lower', interpolation='nearest', cmap=cmap, norm=discrete,
                  extent=(0, w*meta['resolution'], 0, h*meta['resolution']), transform=transform)
        trajectory = cell['trajectory']
        ax.plot(trajectory[:, 0], trajectory[:, 1], color='#d95b20', linewidth=1., label='final native graph')
        ax.set(xlim=(grid_low[0], grid_high[0]), ylim=(grid_low[1], grid_high[1]), xlabel='X [m]', ylabel='Y [m]',
               title=LABELS[name]+'\nOriginal grid: '+str(meta['resolution'])+' m/cell')
        ax.set_aspect('equal', adjustable='box')
        ax.legend(loc='best', fontsize=8)
    fig.suptitle('Native occupancy maps | identical world bounds and metric scale\nBlack occupied; white free; gray unknown. Original resolution and origin retained.')
    save(fig, 'occupancy_comparison.png')

    fig, axes = plt.subplots(1, 2, figsize=(13, 5.5), constrained_layout=True)
    for name, cell in cells.items():
        points = cell['trajectory']
        axes[0].plot(points[:, 0], points[:, 1], '.-', markersize=2, linewidth=1., color=COLORS[name], label=LABELS[name])
        axes[1].plot(cell['node_ids'], points[:, 2], '.-', markersize=2, linewidth=1., color=COLORS[name], label=LABELS[name])
    axes[0].set(xlim=(grid_low[0], grid_high[0]), ylim=(grid_low[1], grid_high[1]), xlabel='X [m]', ylabel='Y [m]', title='Final native graph trajectory')
    axes[0].set_aspect('equal', adjustable='box')
    axes[1].set(xlabel='Native node ID (node selection can differ)', ylabel='Node Z [m]', title='Model output, not surveyed height error')
    for ax in axes:
        ax.grid(alpha=.2)
        ax.legend(fontsize=8)
    fig.suptitle('Trajectory comparison | unchanged exported coordinates; no alignment or refit')
    save(fig, 'trajectory_comparison.png')

    note = 'Display only: identical row stride '+str(stride)+'; no voxel, crop, refit, or geometric rejection.'
    fig, axes = plt.subplots(2, 2, figsize=(13, 9), constrained_layout=True)
    for row, name in enumerate(NAMES):
        points = cells[name]['points']
        for col, axis in enumerate((0, 1)):
            ax = axes[row, col]
            scatter = ax.scatter(points[:, axis], points[:, 2], c=points[:, 2], norm=z_norm, cmap='viridis',
                                 s=.35, alpha=1., linewidths=0, rasterized=True)
            ax.set(xlim=(limits_low[axis], limits_high[axis]), ylim=(limits_low[2], limits_high[2]),
                   xlabel=('X' if axis == 0 else 'Y')+' [m]', ylabel='Z [m]',
                   title=LABELS[name]+' | '+('XZ' if axis == 0 else 'YZ'))
            ax.set_aspect('equal', adjustable='box')
            ax.grid(alpha=.15)
    fig.colorbar(scatter, ax=axes, label='Original map Z [m]', shrink=.75)
    fig.suptitle('Native 3D map side views | shared full-data bounds and Z colors\n'+note)
    save(fig, 'map_side_views.png')

    fig = plt.figure(figsize=(13, 6), constrained_layout=True)
    axes = []
    for i, name in enumerate(NAMES):
        ax = fig.add_subplot(1, 2, i+1, projection='3d')
        axes.append(ax)
        points = cells[name]['points']
        scatter = ax.scatter(points[:, 0], points[:, 1], points[:, 2], c=points[:, 2], norm=z_norm,
                             cmap='viridis', s=.35, alpha=1., linewidths=0, depthshade=False, rasterized=True)
        trajectory = cells[name]['trajectory']
        ax.plot(trajectory[:, 0], trajectory[:, 1], trajectory[:, 2], color='#d95b20', linewidth=1.)
        ax.set(xlim=(limits_low[0], limits_high[0]), ylim=(limits_low[1], limits_high[1]),
               zlim=(limits_low[2], limits_high[2]), xlabel='X [m]', ylabel='Y [m]', zlabel='Z [m]', title=LABELS[name])
        ax.set_box_aspect(limits_high-limits_low)
        ax.set_proj_type('ortho')
        ax.view_init(elev=35.26438968, azim=-45)
    fig.colorbar(scatter, ax=axes, label='Original map Z [m]', shrink=.65)
    fig.suptitle('Native map comparison | identical view, point size, full-data axes and colors\n'+note)
    save(fig, 'map_isometric.png')

    require(all(digest(Path(path)) == expected for path, expected in hashes.items()), 'Original native artifact changed while rendering')
    summary = {'status': 'RENDERED_REQUIRES_VISUAL_REVIEW', 'native_result': str(source), 'original_sha256': hashes,
               'display_stride': stride, 'display_max_points_per_map': args.max_display_points,
               'shared_xyz_limits_m': {'min': limits_low.tolist(), 'max': limits_high.tolist()},
               'shared_grid_xy_limits_m': {'min': grid_low.tolist(), 'max': grid_high.tolist()},
               'isometric_view': {'elevation_deg': 35.26438968, 'azimuth_deg': -45, 'projection': 'orthographic'},
               'files': files, 'cells': {},
               'geometry_changed': False, 'originals_unchanged': True, 'hardware_started': False,
               'ground_thickness': {'available': False, 'reason': 'NO_INDEPENDENT_FIXED_PLANE_ROI'},
               'limitations': ['Point row subsampling is display only; full finite bounds/counts are audited.',
                              'Planar model Z/roll/pitch constraints are not measured accuracy.',
                              'Final graph node selection can differ between native profiles.',
                              'Complete pipeline comparison includes EKF, filtering and native grid parameter differences.',
                              'No fitted plane, registration, crop, residual-based rejection, or map regeneration performed.']}
    for name, cell in cells.items():
        trajectory = cell['trajectory']
        summary['cells'][name] = {'cloud': cell['cloud'], 'grid': cell['grid_report'],
            'native_database': report['cells'][name]['database'], 'final_graph_stamp_ns': cell['graph_stamp_ns'],
            'final_graph_nodes': len(trajectory), 'graph_xyz_min_m': trajectory.min(axis=0).tolist(),
            'graph_xyz_max_m': trajectory.max(axis=0).tolist(),
            'graph_path_length_m': float(np.linalg.norm(np.diff(trajectory, axis=0), axis=1).sum())}
    with (output/'summary.json').open('x', encoding='utf-8') as stream:
        json.dump(summary, stream, ensure_ascii=False, allow_nan=False, indent=2)
        stream.write('\n')
    print(json.dumps({'status': summary['status'], 'output': str(output), 'figures': [row['file'] for row in files]}))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
