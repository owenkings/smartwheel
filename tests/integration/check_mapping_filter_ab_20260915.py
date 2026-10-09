#!/usr/bin/env python3
"""Read-only, identity-bound same-pixel raw/filtered bag comparison. No ROS nodes."""
import argparse
from contextlib import ExitStack, closing
import hashlib
import json
import os
from pathlib import Path
import sqlite3
from types import SimpleNamespace

# Bound CPU use before importing numerical libraries, including when run remotely.
for _name in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS', 'NUMEXPR_NUM_THREADS'):
    os.environ[_name] = '1'
import numpy as np


def sha(data):
    return hashlib.sha256(data).hexdigest()


def stamp(value):
    return int(value.sec)*10**9 + int(value.nanosec)


def key(msg):
    return (msg.session_id, msg.side, msg.sensor_id, msg.stream_epoch, int(msg.frame_sequence))


def flags(msg):
    return dict(value.split('=', 1) for value in msg.diagnostic_flags if '=' in value)


def verify_pair(raw, filtered, session_id):
    assert key(raw) == key(filtered) and raw.session_id == session_id, 'SOURCE_IDENTITY_MISMATCH'
    for field in ('source_config_hash', 'sensor_id', 'stream_epoch'):
        assert getattr(raw, field) and getattr(filtered, field), 'MISSING_IDENTITY_FIELD'
    for field in ('raw_count', 'host_monotonic_ns', 'time_source', 'common_time_valid',
                  'device_timestamp_valid', 'device_timestamp_raw', 'device_timestamp_unit',
                  'device_time_components_valid', 'device_timestamp_seconds',
                  'device_timestamp_nanoseconds', 'device_time_type', 'device_sync_state',
                  'coordinate_convention', 'units'):
        assert getattr(raw, field) == getattr(filtered, field), 'SOURCE_METADATA_MISMATCH:'+field
    assert stamp(raw.header.stamp) == stamp(filtered.header.stamp), 'SOURCE_STAMP_MISMATCH'
    assert stamp(raw.host_receive_time) == stamp(filtered.host_receive_time), 'HOST_STAMP_MISMATCH'
    assert raw.header.frame_id == filtered.header.frame_id, 'SOURCE_FRAME_MISMATCH'
    assert raw.coordinate_convention == 'FLU' and raw.units == 'm', 'COORDINATE_CONTRACT'
    a, b = flags(raw), flags(filtered)
    assert a.get('representation') == 'before_host_filter', 'RAW_REPRESENTATION_MISMATCH'
    assert b.get('representation') == 'host_filtered', 'FILTERED_REPRESENTATION_MISMATCH'
    assert b.get('raw_source_config_hash') == raw.source_config_hash, 'CONFIG_BINDING_MISMATCH'
    assert b.get('before_host_filter_cloud_sha256') == sha(bytes(raw.cloud.data)), 'RAW_CLOUD_HASH_MISMATCH'
    for field in ('width', 'height', 'point_step', 'row_step', 'is_bigendian'):
        assert getattr(raw.cloud, field) == getattr(filtered.cloud, field), 'CLOUD_LAYOUT_MISMATCH:'+field
    layout = lambda msg: [(f.name, f.offset, f.datatype, f.count) for f in msg.cloud.fields]
    assert layout(raw) == layout(filtered), 'POINT_FIELDS_MISMATCH'
    assert stamp(raw.cloud.header.stamp) == stamp(raw.header.stamp), 'RAW_CLOUD_STAMP_MISMATCH'
    assert stamp(filtered.cloud.header.stamp) == stamp(filtered.header.stamp), 'FILTERED_CLOUD_STAMP_MISMATCH'
    return {'identity': list(key(raw)), 'raw_cloud_sha256': sha(bytes(raw.cloud.data)),
            'raw_source_config_hash': raw.source_config_hash,
            'filtered_source_config_hash': filtered.source_config_hash,
            'binding': 'filtered diagnostic SHA-256 binds raw.cloud.data; CDR hashes recorded separately'}


def quantiles(values):
    values = np.asarray(values, dtype=float)
    values = values[np.isfinite(values)]
    return {'count': int(len(values)), 'p50': float(np.median(values)),
            'p95': float(np.quantile(values, .95)), 'max': float(np.max(values))} if len(values) else {'count': 0}


def normal_angle(a, b):
    return float(np.degrees(np.arccos(np.clip(abs(np.dot(a, b)), 0, 1))))


def fit_fixed(points):
    """One TLS fit on immutable pixel support: no branch-specific rejection/refit."""
    if len(points) < 80:
        return None
    center = points.mean(axis=0)
    _, singular, vectors = np.linalg.svd(points-center, full_matrices=False)
    if singular[1] < .03:
        return None
    normal = vectors[-1]
    if normal[np.argmax(abs(normal))] < 0:
        normal = -normal
    residual = (points-center)@normal
    in_plane = (points-center)@vectors[:2].T
    spans = np.diff(np.quantile(in_plane, [.05, .95], axis=0), axis=0)[0]
    return {'normal': normal.tolist(), 'd_m': float(-normal@center), 'count': int(len(points)),
            'thickness_abs_residual_m': quantiles(abs(residual)),
            'singular_values_m': singular.tolist(), 'normal_to_second_axis_ratio': float(singular[2]/singular[1]),
            'in_plane_axis_spans_p05_p95_m': spans.tolist(),
            'xyz_extent_p05_p95_m': np.quantile(points, [.05, .95], axis=0).tolist()}


def raw_seed(points, horizontal, rng):
    """Choose a candidate once on raw; both branches then use a wider fixed support."""
    if len(points) < 150:
        return None
    best_count, best = 0, None
    for _ in range(300):
        a, b, c = points[rng.choice(len(points), 3, replace=False)]
        normal = np.cross(b-a, c-a)
        length = np.linalg.norm(normal)
        if length < 1e-8:
            continue
        normal /= length
        if (horizontal and abs(normal[2]) < .85) or (not horizontal and abs(normal[2]) > .35):
            continue
        d = -normal@a
        support = abs(points@normal+d) < .035
        if support.sum() > best_count:
            best_count, best = int(support.sum()), fit_fixed(points[support])
    if best_count < 150 or best is None:
        return None
    return best


def boundary_mask(raw, common, width=160, height=60):
    if len(raw) != width*height:
        return None
    distance = np.linalg.norm(raw, axis=1).reshape(height, width)
    valid = common.reshape(height, width)
    edge = np.zeros((height, width), dtype=bool)
    for axis in (0, 1):
        one, two = [slice(None), slice(None)], [slice(None), slice(None)]
        one[axis], two[axis] = slice(None, -1), slice(1, None)
        one, two = tuple(one), tuple(two)
        pair = valid[one] & valid[two]
        jump = pair & (abs(distance[one]-distance[two]) > .2)
        invalid = valid[one] ^ valid[two]
        edge[one] |= jump | invalid
        edge[two] |= jump | invalid
    edge[[0, -1], :] = True
    edge[:, [0, -1]] = True
    return edge.ravel() & common


def compare_clouds(raw, filtered, rotation, rng):
    assert raw.shape == filtered.shape and raw.shape[1] == 3
    a, b = raw@rotation.T, filtered@rotation.T
    raw_finite = np.isfinite(a).all(axis=1)
    filtered_finite = np.isfinite(b).all(axis=1)
    common = raw_finite & filtered_finite
    distance = np.linalg.norm(a, axis=1)
    bounded = common & (distance >= .3) & (distance < 8)
    edge = boundary_mask(raw, common)
    segments = {'all_0.3_to_8m': bounded, 'near_0.3_to_2m': bounded & (distance < 2),
                'mid_2_to_4m': bounded & (distance >= 2) & (distance < 4),
                'far_4_to_8m': bounded & (distance >= 4)}
    if edge is not None:
        segments.update(boundary=bounded & edge, interior=bounded & ~edge)
    displacement = np.linalg.norm(b-a, axis=1)
    result = {'point_count': len(a), 'raw_finite': int(raw_finite.sum()),
              'filtered_finite': int(filtered_finite.sum()), 'shared_finite': int(common.sum()),
              'raw_finite_removed': int((raw_finite & ~filtered_finite).sum()),
              'filtered_finite_added': int((~raw_finite & filtered_finite).sum()),
              'displacement_m': {name: quantiles(displacement[mask]) for name, mask in segments.items()},
              'radial_change_m': {name: quantiles(abs(np.linalg.norm(b[mask], axis=1)-distance[mask]))
                                  for name, mask in segments.items()}, 'planes': {}}
    # Same fixed coordinate ROI on raw, not different ROIs for each branch.
    base = bounded & (a[:, 0] > .5) & (a[:, 0] < 6) & (abs(a[:, 1]) < 3)
    floor_roi = base & (a[:, 2] > -1.4) & (a[:, 2] < -.15)
    floor = raw_seed(a[floor_roi], True, rng)
    wall_roi = base & (a[:, 2] > -.3) & (a[:, 2] < 2.4)
    if floor:
        wall_roi &= abs(a@np.asarray(floor['normal'])+floor['d_m']) > .15
    seeds = [('floor', floor_roi, floor), ('wall', wall_roi, raw_seed(a[wall_roi], False, rng))]
    for name, roi, seed in seeds:
        if seed is None:
            result['planes'][name] = {'status': 'NO_SUPPORTED_CANDIDATE'}
            continue
        support = roi & (abs(a@np.asarray(seed['normal'])+seed['d_m']) < .12)
        ids = np.flatnonzero(support)
        first, second = fit_fixed(a[support]), fit_fixed(b[support])
        if first is None or second is None:
            result['planes'][name] = {'status': 'DEGENERATE_SUPPORT'}
            continue
        subsets = {}
        for label, mask in segments.items():
            subset = support & mask
            left, right = fit_fixed(a[subset]), fit_fixed(b[subset])
            subsets[label] = {'count': int(subset.sum()), 'raw': left, 'filtered': right}
        result['planes'][name] = {'status': 'FIXED_SHARED_PIXEL_SUPPORT', 'roi_count': int(roi.sum()),
            'support_count': len(ids), 'support_pixel_indices_sha256': sha(ids.astype('<u4').tobytes()),
            'support_pixel_indices': ids.tolist(), 'raw': first, 'filtered': second,
            'normal_change_deg': normal_angle(first['normal'], second['normal']),
            'filtered_residual_to_raw_fit_m': quantiles(abs(b[support]@np.asarray(first['normal'])+first['d_m'])),
            'subsets': subsets}
        # This tests the raw support only. Large filtered changes stay visible.
        raw_quality = (len(ids) >= 500 and first['normal_to_second_axis_ratio'] <= .25
                       and min(first['in_plane_axis_spans_p05_p95_m']) >= .5
                       and ((name == 'floor' and abs(first['normal'][2]) >= .85)
                            or (name == 'wall' and abs(first['normal'][2]) <= .35)))
        result['planes'][name]['raw_support_geometrically_usable'] = bool(raw_quality)
    f, w = result['planes']['floor'], result['planes']['wall']
    if f.get('raw') and w.get('raw'):
        result['candidate_wall_floor_angle_deg'] = {
            branch: normal_angle(f[branch]['normal'], w[branch]['normal']) for branch in ('raw', 'filtered')}
        result['angle_raw_support_geometrically_usable'] = bool(f['raw_support_geometrically_usable'] and w['raw_support_geometrically_usable'])
    # Real displacement image, retained for three representative frames only by caller.
    heat = np.full(len(a), np.nan)
    heat[bounded] = displacement[bounded]
    return result, heat


def motion_label(row, initial):
    angular = np.asarray(row['angular_velocity_reference_rad_s'])
    linear = np.asarray(row['linear_velocity_reference_m_s'])
    transform = np.asarray(initial['T_reference_axle'])
    axle = transform[:3, :3].T @ (linear + np.cross(angular, transform[:3, 3]))
    speed, spin = float(abs(axle[0])), float(np.linalg.norm(angular))
    label = 'static_candidate' if speed < .005 and spin < .02 else ('moving' if speed > .03 or spin > .04 else 'transition')
    return {'label': label, 'wheel_longitudinal_abs_m_s': speed, 'imu_angular_norm_rad_s': spin}


def source_stat(path):
    s = path.stat()
    return {'path': str(path), 'bytes': s.st_size, 'mtime_ns': s.st_mtime_ns}


def aggregate(frames):
    result = {'frames': len(frames), 'motion_counts': {label: sum(f['motion']['label'] == label for f in frames)
               for label in ('static_candidate', 'moving', 'transition')}, 'displacement_m': {}, 'planes': {}}
    for segment in frames[0]['geometry']['displacement_m']:
        result['displacement_m'][segment] = {'frame_p50': quantiles([f['geometry']['displacement_m'][segment].get('p50', np.nan) for f in frames]),
                                           'frame_p95': quantiles([f['geometry']['displacement_m'][segment].get('p95', np.nan) for f in frames])}
    for name in ('floor', 'wall'):
        planes = [f['geometry']['planes'][name] for f in frames if f['geometry']['planes'][name].get('raw')]
        result['planes'][name] = {'paired_candidates': len(planes), 'normal_change_deg': quantiles([p['normal_change_deg'] for p in planes]),
            'raw_p95_thickness_m': quantiles([p['raw']['thickness_abs_residual_m']['p95'] for p in planes]),
            'filtered_p95_thickness_m': quantiles([p['filtered']['thickness_abs_residual_m']['p95'] for p in planes]),
            'filtered_minus_raw_p95_m': quantiles([p['filtered']['thickness_abs_residual_m']['p95']-p['raw']['thickness_abs_residual_m']['p95'] for p in planes])}
        usable = [p for p in planes if p['raw_support_geometrically_usable']]
        result['planes'][name]['usable_raw_support_count'] = len(usable)
        result['planes'][name]['usable_raw_support_normal_change_deg'] = quantiles([p['normal_change_deg'] for p in usable])
    usable_angles = [f['geometry']['candidate_wall_floor_angle_deg'] for f in frames if f['geometry'].get('angle_raw_support_geometrically_usable')]
    result['usable_candidate_wall_floor_angles_deg'] = {branch: quantiles([row[branch] for row in usable_angles]) for branch in ('raw', 'filtered')}
    result['usable_candidate_angle_filtered_minus_raw_deg'] = quantiles([row['filtered']-row['raw'] for row in usable_angles])
    return result


def analyze(source, output, count=36):
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    from wc_sensors.pointcloud import decode_pointcloud2
    assert 20 <= count <= 40
    source, output = source.resolve(), output.resolve()
    assert source not in output.parents and not output.exists(), 'OUTPUT_MUST_BE_NEW_AND_OUTSIDE_SOURCE'
    config = json.loads((source/'runtime_config.json').read_text())
    initial = json.loads((source/'prior/initialization.json').read_text())
    guesses = [json.loads(line) for line in (source/'prior/guesses.jsonl').read_text().splitlines() if line]
    guess_times = np.asarray([row['stamp_ns'] for row in guesses], dtype=np.int64)
    paths = sorted((source/'bag').glob('*.db3'))
    assert paths, 'NO_RECORDED_BAG'
    before = [source_stat(path) for path in paths]
    indices = {'raw': {}, 'filtered': {}}
    topic_names = {'/wc_mapping/lidar_right/source_frame': 'raw', '/wc_mapping/lidar_right/source_frame_filtered': 'filtered'}
    with ExitStack() as stack:
        databases = {path: stack.enter_context(closing(sqlite3.connect(path.as_uri()+'?mode=ro', uri=True))) for path in paths}
        for path, db in databases.items():
            db.execute('PRAGMA query_only=ON')
            for topic_id, name, kind in db.execute('SELECT id,name,type FROM topics'):
                if name not in topic_names:
                    continue
                branch = topic_names[name]
                cls = get_message(kind)
                for message_id, received, cdr in db.execute('SELECT id,timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp,id', (topic_id,)):
                    msg = deserialize_message(cdr, cls)
                    identity = key(msg)
                    assert identity not in indices[branch], 'DUPLICATE_RECORDED_IDENTITY'
                    indices[branch][identity] = (path, message_id, int(received), stamp(msg.header.stamp), cls)
            print('Indexed '+path.name, flush=True)
        matched = sorted(indices['raw'].keys() & indices['filtered'].keys(), key=lambda k: indices['raw'][k][3])
        assert len(matched) >= count, 'INSUFFICIENT_BOUND_PAIRS'
        times = np.asarray([indices['raw'][k][3] for k in matched], dtype=np.int64)
        motion = []
        for value in times:
            j = int(np.argmin(abs(guess_times-value)))
            row = motion_label(guesses[j], initial)
            row['prior_delta_s'] = abs(int(guess_times[j])-int(value))*1e-9
            if row['prior_delta_s'] > .5:
                row['label'] = 'transition'
            motion.append(row)
        selected = {int(np.argmin(abs(times-t))) for t in np.linspace(int(times[0]), int(times[-1]), count-8)}
        for label in ('static_candidate', 'moving'):
            available = [i for i, value in enumerate(motion) if value['label'] == label and i not in selected]
            for j in np.linspace(0, len(available)-1, min(4, len(available)), dtype=int) if available else []:
                selected.add(available[int(j)])
        for i in np.linspace(0, len(matched)-1, count*4, dtype=int):
            if len(selected) >= count:
                break
            selected.add(int(i))
        frames, images = [], []
        ordered = sorted(selected)
        image_indices = {ordered[0], ordered[len(ordered)//2], ordered[-1]}
        for i in ordered:
            pair, hashes = {}, {}
            for branch in ('raw', 'filtered'):
                path, message_id, received, _, cls = indices[branch][matched[i]]
                cdr = databases[path].execute('SELECT data FROM messages WHERE id=?', (message_id,)).fetchone()[0]
                pair[branch] = deserialize_message(cdr, cls)
                hashes[branch] = {'bag': path.name, 'message_id': message_id, 'bag_received_ns': received, 'cdr_sha256': sha(cdr)}
            binding = verify_pair(pair['raw'], pair['filtered'], config['session_id'])
            geometry, heat = compare_clouds(decode_pointcloud2(pair['raw'].cloud), decode_pointcloud2(pair['filtered'].cloud),
                                            np.asarray(initial['R_reference_base']), np.random.default_rng(20260915+i))
            frame = {'time_s': (int(times[i])-int(times[0]))*1e-9, 'source_stamp_ns': int(times[i]),
                     'binding': binding, 'cdr_records': hashes, 'motion': motion[i], 'geometry': geometry}
            frames.append(frame)
            if i in image_indices:
                images.append({'time_s': frame['time_s'], 'shape': [60, 160] if len(heat) == 9600 else [1, len(heat)],
                               'displacement_m': [float(x) if np.isfinite(x) else None for x in heat]})
            print('Compared sequence %d: %s' % (matched[i][-1], motion[i]['label']), flush=True)
    after = [source_stat(path) for path in paths]
    assert before == after, 'SOURCE_BAG_STAT_CHANGED'
    report = {'validation_level': 'REAL_BAG', 'session_id': config['session_id'], 'source_root': str(source),
              'hardware_accessed': False, 'ros_nodes_started': False, 'source_access': 'SQLite mode=ro and query_only; every connection closed',
              'bag_stats_before_after_identical': True, 'bag_stats': before,
              'recorded_counts': {branch: len(rows) for branch, rows in indices.items()}, 'matched_identity_count': len(matched),
              'selection': 'Time-uniform source stamps plus static/moving candidates, all compared on identical pixel support',
              'policy': {'range_bins_m': [[.3, 2], [2, 4], [4, 8]], 'raw_seed_ransac_threshold_m': .035,
                         'fixed_shared_support_band_m': .12, 'boundary_raw_neighbor_jump_m': .2,
                         'pixel_layout': '160x60 only when raw_count=9600; original message row order',
                         'plane_fit': 'One raw RANSAC seed; fixed raw ROI and 12 cm support; branch TLS fits on exactly the same pixel indices; no branch-specific rejection',
                         'angle_support_diagnostic_gate': 'Raw support only: count>=500, minor/middle singular value<=0.25, both in-plane p90 spans>=0.5m, floor |nz|>=0.85, wall |nz|<=0.35. Failed candidates remain in the report, not treated as surveyed planes.',
                         'coordinates': 'Fixed recorded R_reference_base applied equally to raw/filtered; no trajectory or gyro-bias correction'},
              'limitations': ['Candidate planes are not surveyed ground truth; raw-seeded support selection can favor raw residuals.',
                  'Reported plane thickness is residual spread, not absolute range accuracy; support and normals are retained.',
                  'User describes the route as mainly flat without ramps; this is route context, not a measured plane/angle reference. Furniture and thresholds may remain in candidate support.',
                  'Static candidates use recorded wheel and gyro estimates, not independent proof of no chassis motion.',
                  'before_host_filter is after SDK decode and lens projection, not raw optical phase/depth hardware truth.',
                  'This compares the whole configured host filter chain against its bypass, not individual filter ablations.',
                  'Disabling only temporal filters requires replayable SDK raw depth/amplitude history or new controlled capture; XYZ pairs cannot emulate that ablation.',
                  'Flash global exposure must not be interpreted as a mechanically swept scanner; HDR exposure timing remains unvalidated.',
                  'Shared-finite analysis excludes pixels removed by filtering; removed/added counts are reported separately.',
                  'No runtime, driver, calibration or original map modifications. Source file stat checks are not whole-file cryptographic hashes.'],
              'summary': aggregate(frames), 'by_motion': {label: aggregate(rows) for label in ('static_candidate', 'moving', 'transition')
                  if (rows := [f for f in frames if f['motion']['label'] == label])}, 'frames': frames, 'images': images}
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open('x') as stream:
        json.dump(report, stream, ensure_ascii=False, allow_nan=False, indent=2)
    return report


def plot(source, output):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    data = json.loads(source.read_text())
    frames = data['frames']
    fig, axes = plt.subplots(2, 3, figsize=(14, 8), constrained_layout=True)
    t = [f['time_s'] for f in frames]
    for segment, label in (('all_0.3_to_8m', 'all'), ('boundary', 'boundary'), ('interior', 'interior')):
        axes[0, 0].plot(t, [100*f['geometry']['displacement_m'][segment].get('p95', np.nan) for f in frames], '.-', label=label)
    axes[0, 0].set(xlabel='Source time (s)', ylabel='Same-pixel displacement P95 (cm)', title='Raw to full host-filter chain')
    axes[0, 0].legend()
    for name, ax in zip(('floor', 'wall'), axes[0, 1:]):
        rows = [f for f in frames if f['geometry']['planes'][name].get('raw')]
        for branch in ('raw', 'filtered'):
            ax.plot([f['time_s'] for f in rows], [100*f['geometry']['planes'][name][branch]['thickness_abs_residual_m']['p95'] for f in rows], '.-', label=branch)
        ax.set(xlabel='Source time (s)', ylabel='Fixed-support plane thickness P95 (cm)', title=name.title()+' candidate: identical pixels')
        ax.legend()
    for ax, item in zip(axes[1], data['images']):
        values = np.asarray(item['displacement_m'], dtype=float).reshape(item['shape'])*100
        im = ax.imshow(values, vmin=0, vmax=30, cmap='magma', interpolation='nearest', aspect='auto')
        ax.set(title='Same-pixel change, t=%.1f s' % item['time_s'], xlabel='Original pixel column', ylabel='Original pixel row')
    fig.colorbar(im, ax=list(axes[1]), label='Displacement (cm; display clipped at 30)', shrink=.8)
    fig.suptitle('Recorded right M60: %d identity/hash-verified pairs; candidate geometry, no ground truth' % len(frames), fontsize=13)
    assert not output.exists(), 'PLOT_OUTPUT_EXISTS'
    fig.savefig(output, dpi=160)
    plt.close(fig)


def self_test():
    # Synthetic plane translation must preserve thickness and angle on frozen pixels.
    rng = np.random.default_rng(14)
    p = np.column_stack([rng.uniform(.5, 4, 1200), rng.uniform(-2, 2, 1200), rng.normal(-.6, .005, 1200)])
    a, b = fit_fixed(p), fit_fixed(p+np.array([0., 0., .07]))
    assert abs(a['thickness_abs_residual_m']['p95']-b['thickness_abs_residual_m']['p95']) < 1e-12
    assert normal_angle(a['normal'], b['normal']) < 1e-5
    assert abs(a['d_m']-b['d_m']) > .069
    # Deliberate large shifted pixels must remain in the filtered TLS support.
    q = p.copy(); q[:100, 2] += .2
    result, _ = compare_clouds(p, q, np.eye(3), np.random.default_rng(8))
    floor = result['planes']['floor']
    assert floor['raw']['count'] == floor['filtered']['count'] == floor['support_count']
    assert floor['filtered']['thickness_abs_residual_m']['p95'] > .1
    # Hash binding rejects changed raw bytes even with matching frame identity.
    s = SimpleNamespace(sec=1, nanosec=2)
    h = SimpleNamespace(stamp=s, frame_id='lidar_right')
    c = SimpleNamespace(data=b'1234', header=h, width=1, height=1, point_step=4, row_step=4,
                        is_bigendian=False, fields=[])
    values = dict(session_id='session', side='right', sensor_id='sensor', stream_epoch='epoch', frame_sequence=3,
        source_config_hash='rawhash', header=h, host_receive_time=s, cloud=c, raw_count=1, host_monotonic_ns=4,
        time_source='arrival_only', common_time_valid=False, device_timestamp_valid=True, device_timestamp_raw=5,
        device_timestamp_unit='ns', device_time_components_valid=True, device_timestamp_seconds=0,
        device_timestamp_nanoseconds=5, device_time_type='sdk', device_sync_state='unknown', coordinate_convention='FLU', units='m')
    raw = SimpleNamespace(**values, diagnostic_flags=['representation=before_host_filter'])
    values['source_config_hash'] = 'filteredhash'
    filtered = SimpleNamespace(**values, diagnostic_flags=['representation=host_filtered', 'raw_source_config_hash=rawhash',
        'before_host_filter_cloud_sha256='+sha(c.data)])
    verify_pair(raw, filtered, 'session')
    c.data = b'bad!'
    try:
        verify_pair(raw, filtered, 'session')
    except AssertionError as error:
        assert str(error) == 'RAW_CLOUD_HASH_MISMATCH'
    else:
        raise AssertionError('corrupt raw binding accepted')
    print('SYNTHETIC_PASS: plane invariance, immutable shared support, raw-cloud hash rejection')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session-root', type=Path)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--count', type=int, default=36)
    parser.add_argument('--plot-json', type=Path)
    parser.add_argument('--self-test', action='store_true')
    args = parser.parse_args()
    if args.self_test:
        self_test()
    elif args.plot_json:
        assert args.output
        plot(args.plot_json, args.output)
    else:
        assert args.session_root and args.output
        result = analyze(args.session_root, args.output, args.count)
        print(json.dumps(result['summary'], ensure_ascii=False, allow_nan=False))


if __name__ == '__main__':
    main()
