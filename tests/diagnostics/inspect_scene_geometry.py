"""Offline descriptive geometry audit; no sensor, calibration or UI mutations."""
import argparse
import hashlib
import json
from pathlib import Path
import warnings

import numpy as np


def clean(value):
    if isinstance(value, dict):
        return {str(k): clean(v) for k, v in value.items()}
    if isinstance(value, (list, tuple, np.ndarray)):
        return [clean(v) for v in value]
    if isinstance(value, (np.integer,)):
        return int(value)
    if isinstance(value, (float, np.floating)):
        return float(value) if np.isfinite(value) else None
    return value


def plane_patches(points):
    """Unlabelled 3-cm RANSAC patches, not ground truth or full-wall validation."""
    rng = np.random.default_rng(913)
    remaining = points.copy()
    patches = []
    for _ in range(5):
        if len(remaining) < 300:
            break
        sample = remaining[rng.choice(len(remaining), min(len(remaining), 3500), replace=False)]
        best, model = 0, None
        for _ in range(320):
            a, b, c = sample[rng.choice(len(sample), 3, replace=False)]
            normal = np.cross(b-a, c-a)
            size = np.linalg.norm(normal)
            if size < 1e-8:
                continue
            normal /= size
            offset = -a @ normal
            count = np.count_nonzero(np.abs(sample @ normal + offset) < .03)
            if count > best:
                best, model = count, (normal, offset)
        if model is None:
            break
        mask = np.abs(remaining @ model[0] + model[1]) < .03
        if mask.sum() < 250:
            break
        for _ in range(2):
            inliers = remaining[mask]
            center = inliers.mean(axis=0)
            _, _, vt = np.linalg.svd(inliers-center, full_matrices=False)
            normal = vt[-1]
            if normal[np.argmax(np.abs(normal))] < 0:
                normal = -normal
            offset = -center @ normal
            mask = np.abs(remaining @ normal + offset) < .03
        residual = np.abs(remaining[mask] @ normal + offset)
        patches.append({'count': int(mask.sum()), 'normal': normal, 'offset_m': offset,
                        'centroid_m': remaining[mask].mean(axis=0),
                        'rmse_m': np.sqrt(np.mean(residual**2)), 'p95_m': np.quantile(residual, .95)})
        remaining = remaining[~mask]
    return patches


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--study', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    root = Path('/home/nvidia/wheelchair')
    study, output = args.study.resolve(), args.output.resolve()
    if study.parent != root/'reports/lidar_stability' or not output.is_relative_to(root/'reports'):
        raise ValueError('Expected explicit project report paths')
    output.mkdir(parents=True, exist_ok=False)
    prepared_path = root/'reports/calibration'/study.name/'prepared.json'
    prepared = json.loads(prepared_path.read_text())
    scene = prepared['training'][0]
    result = {'status': 'DESCRIPTIVE_OFFLINE_AUDIT_NOT_CALIBRATION', 'study': str(study),
              'scene_id': scene['id'], 'prepared_sha256': hashlib.sha256(prepared_path.read_bytes()).hexdigest(),
              'limitations': ['No surveyed dimensions or user-labelled ground/wall ROI.',
                              'RANSAC patches use a 3 cm selection threshold; their residuals do not validate a complete surface.',
                              'Raw already includes vendor decoding and lens projection.',
                              'Sequential single-lidar input; no IMU acquisition in this recording procedure.'],
              'sides': {}, 'dual_single_repeatability': {}}
    plot_clouds = {}
    for side in ['left', 'right']:
        meta = scene['input_files'][side]
        index = meta['frame_index']
        npz_path = study/(side+'_single')/(side+'.npz')
        arrays = np.load(npz_path, allow_pickle=False)
        item = {'selected_frame': index, 'npz_sha256': hashlib.sha256(npz_path.read_bytes()).hexdigest(),
                'source_npz_hash_matches_preparation': hashlib.sha256(npz_path.read_bytes()).hexdigest() == meta['sources']['npz']['sha256'],
                'representations': {}}
        selected = {}
        for rep in ['raw', 'filtered']:
            xyz = arrays[rep+'_xyz'][index].astype(float)
            selected[rep] = xyz
            ranges = np.linalg.norm(xyz, axis=1)
            valid = np.isfinite(xyz).all(axis=1) & (ranges > .2) & (ranges < 8) & (xyz[:, 0] > .05)
            cloud = xyz[valid]
            slopes = np.column_stack([np.rad2deg(np.arctan2(cloud[:, 1], cloud[:, 0])),
                                      np.rad2deg(np.arctan2(cloud[:, 2], np.hypot(cloud[:, 0], cloud[:, 1])))])
            item['representations'][rep] = {'analysis_points_within_0_2_to_8m': int(valid.sum()),
                'xyz_q01_q50_q99_m': np.quantile(cloud, [.01, .5, .99], axis=0),
                'angular_q01_q50_q99_deg': np.quantile(slopes, [.01, .5, .99], axis=0),
                'unlabelled_plane_patches': plane_patches(cloud)}
            plot_clouds[side+'_'+rep] = cloud
        raw, filtered = selected['raw'], selected['filtered']
        common = np.isfinite(raw).all(axis=1) & np.isfinite(filtered).all(axis=1)
        common &= (np.linalg.norm(raw, axis=1) > .2) & (np.linalg.norm(filtered, axis=1) > .2)
        common &= (np.linalg.norm(raw, axis=1) < 8) & (np.linalg.norm(filtered, axis=1) < 8)
        displacement = np.linalg.norm(filtered[common]-raw[common], axis=1)
        ratios = np.linalg.norm(filtered[common], axis=1)/np.linalg.norm(raw[common], axis=1)
        unit_raw = raw[common]/np.linalg.norm(raw[common], axis=1)[:, None]
        unit_filtered = filtered[common]/np.linalg.norm(filtered[common], axis=1)[:, None]
        angles = np.rad2deg(np.arccos(np.clip(np.sum(unit_raw*unit_filtered, axis=1), -1, 1)))
        item['paired_host_filter_effect'] = {'common_pixels': int(common.sum()),
            'displacement_q50_q95_q99_m': np.quantile(displacement, [.5, .95, .99]),
            'range_ratio_q01_q50_q99': np.quantile(ratios, [.01, .5, .99]),
            'ray_angle_q50_q95_q99_deg': np.quantile(angles, [.5, .95, .99])}
        arrays.close()
        result['sides'][side] = item
        temporal = {}
        for label in ['dual_A', side+'_single', 'dual_B']:
            with np.load(study/label/(side+'.npz'), allow_pickle=False) as data:
                temporal[label] = {}
                for rep in ['raw', 'filtered']:
                    values = data[rep+'_xyz']
                    distance = np.linalg.norm(values, axis=2)
                    valid = np.isfinite(distance) & (distance > .2) & (distance < 8)
                    stable = np.mean(valid, axis=0) >= .95
                    distance = np.where(valid[:, stable], distance[:, stable], np.nan)
                    with warnings.catch_warnings():
                        warnings.simplefilter('ignore', RuntimeWarning)
                        span = np.nanquantile(distance, .95, axis=0)-np.nanquantile(distance, .05, axis=0)
                    temporal[label][rep] = {'frames': len(values), 'stable_pixels': int(stable.sum()),
                        'median_temporal_p95_p05_m': np.nanmedian(span), 'p95_temporal_p95_p05_m': np.nanquantile(span, .95),
                        'mask_note': 'Mask per stream; descriptive, not a controlled identical-pixel causal comparison.'}
        result['dual_single_repeatability'][side] = temporal
    (output/'result.json').write_text(json.dumps(clean(result), indent=2, allow_nan=False)+'\n')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(2, 3, figsize=(14, 8), constrained_layout=True)
    for row, side in enumerate(['left', 'right']):
        for column, dims in enumerate([(0, 1), (0, 2), (1, 2)]):
            ax = axes[row, column]
            for rep, color in [('raw', '#708090'), ('filtered', '#087f8c' if side == 'left' else '#b66a14')]:
                p = plot_clouds[side+'_'+rep]
                ax.scatter(p[:, dims[0]], p[:, dims[1]], s=.55, c=color, alpha=.55, label=rep, rasterized=True)
            ax.set_aspect('equal', adjustable='box')
            ax.set_xlabel('XYZ'[dims[0]]+' (m)'); ax.set_ylabel('XYZ'[dims[1]]+' (m)')
            ax.set_title(side+' sensor | '+['top XY', 'side XZ', 'front YZ'][column])
            ax.grid(alpha=.2)
            ax.set_xlim((.0, 7) if dims[0] == 0 else (-2, 5))
            ax.set_ylim((-2, 5) if dims[1] == 1 else (-1.3, 2.5))
            if column == 0: ax.legend(markerscale=6)
    fig.suptitle('Original sensor coordinates, no alignment | raw and host-filtered | equal metric axes\n'+scene['id'])
    fig.savefig(output/'raw_filtered_views.png', dpi=150)
    plt.close(fig)
    compact={side:{'filter_effect':v['paired_host_filter_effect'], 'planes':{r:[{'count':p['count'],'normal':p['normal'],'centroid':p['centroid_m'],'p95_m':p['p95_m']} for p in d['unlabelled_plane_patches']] for r,d in v['representations'].items()}} for side,v in result['sides'].items()}
    print(json.dumps(clean({'output':str(output),'summary':compact,'repeatability':result['dual_single_repeatability']}),allow_nan=False), flush=True)


if __name__ == '__main__':
    main()
