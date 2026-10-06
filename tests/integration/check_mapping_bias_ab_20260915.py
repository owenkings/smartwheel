#!/usr/bin/env python3
"""Read-only real-CDR bias A/B integration; no ROS node or device is started.

Writes new diagnostic files outside the source session. Candidate variants are
offline experiments, not accepted calibrations. All variants share the same
original source samples, mounting matrices and fully covered intervals.
"""
import argparse
from bisect import bisect_right
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import sqlite3

import numpy as np
from scipy.spatial.transform import Rotation

from wc_runtime.mapping_bias import CausalBiasEstimator
from wc_runtime.mapping_prior import DEFAULT_POLICY, integrate_body_twist
from wc_motion.history_preview import HistoryPreview


def stamp(value):
    return int(value.sec)*1_000_000_000+int(value.nanosec)


def load_sources(source, config, session_id):
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    names = {config['imu_topic']: 'imu', config['wheel_topic']: 'wheel'}
    rows = {'imu': [], 'wheel': []}
    hashes, identities = {}, {}
    for path in sorted((source/'bag').glob('*.db3')):
        identities[str(path)] = [path.stat().st_size, path.stat().st_mtime_ns]
        with sqlite3.connect(path.as_uri()+'?mode=ro', uri=True) as db:
            topics = {i: (name, get_message(kind)) for i, name, kind in db.execute('SELECT id,name,type FROM topics') if name in names}
            if not topics:
                continue
            query = 'SELECT topic_id,data FROM messages WHERE topic_id IN ('+','.join('?' for _ in topics)+') ORDER BY timestamp,id'
            for topic_id, raw in db.execute(query, tuple(topics)):
                name, kind = topics[topic_id]
                hashes.setdefault(name, hashlib.sha256()).update(raw)
                msg = deserialize_message(raw, kind)
                if names[name] == 'wheel':
                    record = json.loads(msg.data)
                    rows['wheel'].append({'stamp': int(record['stamp_ns']), 'mono': int(record['receive_monotonic_ns']),
                                          'sequence': int(record['sequence']), 'epoch': record['stream_epoch'], 'record': record})
                else:
                    if (not msg.angular_velocity_valid or not msg.linear_acceleration_valid or msg.uncertainty_valid or
                            msg.sensor_id != config['imu_sensor_id'] or msg.session_id != session_id or
                            msg.header.frame_id != 'imu_h30_native' or msg.imu.header.frame_id != 'imu_h30_native' or
                            msg.coordinate_convention != 'H30_NATIVE_UNVALIDATED' or msg.time_source != 'arrival_only' or msg.common_time_valid):
                        raise ValueError('original H30 identity/native-axis/time classification rejected')
                    g, a = msg.imu.angular_velocity, msg.imu.linear_acceleration
                    row = {'stamp': stamp(msg.host_receive_time), 'mono': int(msg.host_monotonic_ns),
                           'sequence': int(msg.frame_sequence), 'epoch': msg.stream_epoch,
                           'gyro': np.array([g.x, g.y, g.z]), 'accel': np.array([a.x, a.y, a.z])}
                    if not np.isfinite(np.r_[row['gyro'], row['accel']]).all():
                        raise ValueError('original IMU contains nonfinite SI data')
                    if np.linalg.norm(row['gyro']) > DEFAULT_POLICY['max_gyro_rad_s'] or np.linalg.norm(row['accel']) > DEFAULT_POLICY['max_accel_m_s2']:
                        raise ValueError('original IMU exceeds experimental SI bounds')
                    rows['imu'].append(row)
    if not all(rows.values()):
        raise ValueError('recording has no original IMU/wheel data')
    policy = {**DEFAULT_POLICY, **config.get('policy', {})}
    decoder = HistoryPreview(config['wheel_candidate'], continuous=True)
    gaps = []
    for name, values in rows.items():
        previous = None
        for row in values:
            row['gap_before'] = False
            if previous is not None:
                ds, dm = row['stamp']-previous['stamp'], row['mono']-previous['mono']
                if row['epoch'] != previous['epoch'] or ds < 0 or dm < 0 or row['sequence'] <= previous['sequence']:
                    raise ValueError(name+' original identity or clock reversal')
                limit = policy['max_imu_age_s'] if name == 'imu' else min(policy['max_wheel_age_s'], config['wheel_candidate']['max_gap_s'])
                row['gap_before'] = (row['sequence'] != previous['sequence']+1 or ds > limit*1e9 or
                                     abs(ds-dm) > policy['max_arrival_clock_delta_s']*1e9)
                if row['gap_before']:
                    gaps.append({'source': name, 'start_stamp_ns': previous['stamp'], 'end_stamp_ns': row['stamp'],
                                 'duration_s': ds*1e-9, 'missing_sequences': row['sequence']-previous['sequence']-1})
            if name == 'wheel':
                sample = decoder.update(row.pop('record'))
                factor = 2*np.pi*config['wheel_candidate']['wheel_radius_m']/60
                row['speeds'] = np.array([sample['left_wheel_rpm_candidate'], sample['right_wheel_rpm_candidate']])*factor
                row['v'] = sample['linear_velocity_m_s']
            previous = row
    return rows, policy, {'bag_file_identities_before': identities,
                         'original_cdr_sha256': {name: h.hexdigest() for name, h in hashes.items()},
                         'recorded_source_counts': {name: len(values) for name, values in rows.items()},
                         'distinct_sequence_packets_sharing_stamp': {
                             name: sum(a['stamp'] == b['stamp'] for a, b in zip(values, values[1:]))
                             for name, values in rows.items()},
                         'recording_gaps': gaps}


def candidates_and_profiles(rows, windows, origin):
    estimator = CausalBiasEstimator()
    wheels = rows['wheel']; wtimes = [r['stamp'] for r in wheels]
    heldout = [(origin+round(w['start_s']*1e9), origin+round(w['end_s']*1e9)) for w in windows[1:]]
    candidates, usable, robust = [], [], []
    last_wheel_index = None
    for im in rows['imu']:
        j = bisect_right(wtimes, im['stamp'])-1
        if j < 0:
            continue
        if im['gap_before']:
            estimator.reset_observations('RECORDED_SOURCE_GAP')
        wh = wheels[j]
        if j != last_wheel_index and wh['gap_before']:
            estimator.reset_observations('RECORDED_WHEEL_GAP')
        last_wheel_index = j
        candidate = estimator.observe(im['stamp'], im['gyro'], im['accel'], wh['stamp'], wh['speeds'])
        if candidate is None:
            continue
        candidate['intersects_heldout_window'] = any(candidate['start_stamp_ns'] <= b and candidate['end_stamp_ns'] >= a for a, b in heldout)
        candidates.append(candidate)
        if candidate['screening_passed'] and not candidate['intersects_heldout_window']:
            usable.append(candidate)
            if len(usable) >= 2:
                bias = np.median([r['bias_native_rad_s'] for r in usable[-estimator.policy.robust_windows:]], axis=0)
                robust.append((candidate['end_stamp_ns'], bias))
    assert len(estimator.versions) == 1, 'Unconfirmed candidates must not change the production estimator'
    a, b = origin+round(windows[0]['start_s']*1e9), origin+round(windows[0]['end_s']*1e9)
    training = [r['gyro'] for r in rows['imu'] if a <= r['stamp'] <= b]
    if not training:
        raise ValueError('No high-frequency samples in first fit window')
    fixed = np.mean(training, axis=0)
    return {'zero': [], 'fixed_first_retrospective': [(0, fixed)],
            'fixed_first_causal': [(b, fixed)], 'causal_robust_holdout': robust}, {
                'first_fit_interval_ns': [a, b], 'first_fit_high_frequency_samples': len(training),
                'fixed_first_native_rad_s': fixed.tolist(), 'heldout_intervals_ns': heldout,
                'screened_windows': candidates, 'usable_training_windows': len(usable),
                'causal_robust_versions': [{'effective_stamp_ns': t, 'bias_native_rad_s': v.tolist()} for t, v in robust],
                'production_estimator_status_without_independent_evidence': estimator.status(),
                'screening_policy': asdict(estimator.policy)}


def reintegrate(rows, policy, initial, profiles, extra_stamps):
    imu, wheel = rows['imu'], rows['wheel']
    its, wts = [r['stamp'] for r in imu], [r['stamp'] for r in wheel]
    first = max(its[0], wts[0], initial['stamp_ns'])
    last = min(its[-1], wts[-1])
    events = sorted(set([first, last]+[t for t in its+wts+extra_stamps if first <= t <= last]+
                        [t for profile in profiles.values() for t, _ in profile if first <= t <= last]))
    r = np.asarray(initial['R_reference_imu']); axle = np.asarray(initial['T_reference_axle'])
    poses = {name: np.empty((len(events), 4, 4)) for name in profiles}
    for value in poses.values():
        value[0] = np.eye(4)
    profile_times = {name: [t for t, _ in values] for name, values in profiles.items()}
    observed, moving = [], []
    for index, (begin, end) in enumerate(zip(events, events[1:])):
        ii, wi = bisect_right(its, begin)-1, bisect_right(wts, begin)-1
        im, wh = imu[ii], wheel[wi]
        good = ((end-im['stamp'])*1e-9 <= policy['max_imu_age_s'] and
                (end-wh['stamp'])*1e-9 <= policy['max_wheel_age_s'] and
                not (ii+1 < len(imu) and imu[ii+1]['gap_before'] and begin < imu[ii+1]['stamp']) and
                not (wi+1 < len(wheel) and wheel[wi+1]['gap_before'] and begin < wheel[wi+1]['stamp']))
        observed.append(good)
        moving.append(bool(np.max(np.abs(wh['speeds'])) > .002))
        for name, profile in profiles.items():
            pose = poses[name][index]
            if good:
                bi = bisect_right(profile_times[name], begin)-1
                bias = profile[bi][1] if bi >= 0 else np.zeros(3)
                angular = r@(im['gyro']-bias)
                linear = axle[:3, :3]@np.array([wh['v'], 0., 0.])-np.cross(angular, axle[:3, 3])
                pose = integrate_body_twist(pose, linear, angular, (end-begin)*1e-9)
            poses[name][index+1] = pose
    events = np.asarray(events, dtype=np.int64)
    observed, moving = np.asarray(observed, dtype=bool), np.asarray(moving, dtype=bool)
    segments, start = [], None
    for i, good in enumerate(observed):
        if good and start is None:
            start = int(events[i])
        if start is not None and (not good or i == len(observed)-1):
            end = int(events[i+1] if good else events[i])
            segments.append({'start_stamp_ns': start, 'end_stamp_ns': end, 'duration_s': (end-start)*1e-9})
            start = None
    assert all(np.array_equal(p[1:][~observed], p[:-1][~observed]) for p in poses.values())
    return events, poses, observed, moving, segments


def evaluate(times, poses, observed, moving, windows, origin):
    evaluation = {'window_results': [], 'motion_sensitivity_not_accuracy': {}}
    dt = np.diff(times)*1e-9
    for wi, window in enumerate(windows):
        start, end = origin+round(window['start_s']*1e9), origin+round(window['end_s']*1e9)
        left, right = np.searchsorted(times, [max(start, int(times[0])), min(end, int(times[-1]))])
        left, right = int(left), int(right)
        used = dt[left:right]*observed[left:right]
        entry = {'window_index': wi, 'role': 'fit' if wi == 0 else 'held_out_of_all_candidate_profiles',
                 'actual_start_stamp_ns': int(times[left]), 'actual_end_stamp_ns': int(times[right]),
                 'observed_duration_s': float(used.sum()), 'uncovered_duration_s': float(dt[left:right].sum()-used.sum()), 'variants': {}}
        for name, values in poses.items():
            first, last = values[left], values[right]
            rotation = Rotation.from_matrix(first[:3, :3].T@last[:3, :3])
            entry['variants'][name] = {'relative_rotation_vector_deg': np.rad2deg(rotation.as_rotvec()).tolist(),
                                       'rotation_magnitude_deg': float(np.rad2deg(rotation.magnitude())),
                                       'translation_change_m': (last[:3, 3]-first[:3, 3]).tolist()}
        evaluation['window_results'].append(entry)
    selected = np.r_[False, observed&moving]
    baseline = poses['zero']
    for name, values in poses.items():
        angles = np.rad2deg(Rotation.from_matrix(np.swapaxes(baseline[selected, :3, :3], 1, 2)@values[selected, :3, :3]).magnitude())
        distances = np.linalg.norm(values[selected, :3, 3]-baseline[selected, :3, 3], axis=1)
        evaluation['motion_sensitivity_not_accuracy'][name] = {
            'common_observed_wheel_motion_duration_s': float(dt[observed&moving].sum()),
            'rotation_difference_from_zero_mean_p95_max_deg': [float(np.mean(angles)), float(np.quantile(angles, .95)), float(np.max(angles))] if len(angles) else None,
            'position_difference_from_zero_mean_p95_max_m': [float(np.mean(distances)), float(np.quantile(distances, .95)), float(np.max(distances))] if len(distances) else None,
            'final_xyz_m': values[-1, :3, 3].tolist(),
            'z_min_max_m': [float(values[:, 2, 3].min()), float(values[:, 2, 3].max())]}
    return evaluation


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session-root', required=True, type=Path)
    summaries = parser.add_mutually_exclusive_group(required=True)
    summaries.add_argument('--geometry-summary', type=Path)
    summaries.add_argument('--geometry-summary-json')
    parser.add_argument('--query-stamps', type=Path)
    parser.add_argument('--output-prefix', required=True, type=Path)
    args = parser.parse_args()
    source, prefix = args.session_root.resolve(), args.output_prefix.resolve()
    if source == prefix or source in prefix.parents:
        raise ValueError('Diagnostic output must be outside the original session')
    if prefix.with_suffix('.json').exists() or prefix.with_suffix('.npz').exists():
        raise ValueError('Use a new output prefix; never replace prior diagnostic outputs')
    config = json.loads((source/'prior_config.json').read_text())
    initial = json.loads((source/'prior/initialization.json').read_text())
    summary = json.loads(args.geometry_summary.read_text() if args.geometry_summary else args.geometry_summary_json)
    if summary['session_id'] != initial['session_id']:
        raise ValueError('geometry summary session mismatch')
    guesses = [json.loads(line) for line in (source/'prior/guesses.jsonl').read_text().splitlines() if line]
    origin = guesses[0]['stamp_ns']
    rows, policy, recording = load_sources(source, config, initial['session_id'])
    windows = summary['zero_wheel_low_gyro_windows']
    profiles, candidates = candidates_and_profiles(rows, windows, origin)
    extra = [origin+round(w[key]*1e9) for w in windows for key in ('start_s', 'end_s')]
    extra.extend(row['stamp_ns'] for row in guesses)
    query_stamps = [int(row['stamp_ns']) for row in json.loads(args.query_stamps.read_text())] if args.query_stamps else []
    extra.extend(query_stamps)
    times, poses, observed, moving, segments = reintegrate(rows, policy, initial, profiles, extra)
    report = {'validation_level': 'REAL_BAG_HIGH_FREQUENCY_COMMON_COVERAGE_REINTEGRATION',
              'session_id': initial['session_id'], 'hardware_accessed': False, 'ros_nodes_started': False,
              'original_data_modified': False, 'calibration_applied': False, 'status': 'PASS',
              'time_basis': 'original arrival_only timestamps; synchronization unvalidated',
              'route': 'user-reported level route; no forced z=0 or externally measured ground plane',
              'candidates': candidates, 'recording': recording,
              'integration': {'method': 'causal zero-order-hold original gyro/wheel plus fixed axle lever arm; exact SE3 step',
                              'boundary_count': len(times), 'fully_observed_segments': segments,
                              'covered_duration_s': float((np.diff(times)*1e-9)[observed].sum()),
                              'uncovered_duration_s': float((np.diff(times)*1e-9)[~observed].sum()),
                              'pose_held_identically_across_uncovered_intervals': True},
              'evaluation': evaluate(times, poses, observed, moving, windows, origin),
              'limitations': [
                  'Offline bias candidates are not accepted calibration: there is no independently verified stationary LiDAR evidence in this experiment.',
                  'First-window retrospective variant intentionally uses future fit data before its end and cannot be deployed as a causal startup estimator.',
                  'Causal robust variant is the median of at most five completed screened training windows, requires two windows, and excludes both held-out windows. Its hypothetical candidate updates do not bypass production confirmation.',
                  'Wheel-zero held-out windows are disjoint evaluation samples, not external stationary truth.',
                  'Motion differences from the zero-bias baseline measure sensitivity, not motion accuracy.',
                  'The recording contains missing source intervals absent from the original live prior. Every variant holds pose identically there; resulting full-session pose is not a reconstruction of unrecorded motion.',
                  'No gravity orientation correction, mounting change, height constraint or point-cloud map optimization is performed.']}
    report['recording']['source_bag_file_identities_unchanged'] = all(
        [Path(p).stat().st_size, Path(p).stat().st_mtime_ns] == values for p, values in recording['bag_file_identities_before'].items())
    if not report['recording']['source_bag_file_identities_unchanged']:
        raise ValueError('source recording changed during read-only analysis')
    # Only compare with original live output before the first recording gap;
    # the live estimator possessed samples absent from the later bag segments.
    baseline_errors = []
    for row in guesses:
        query = row['stamp_ns']
        if not segments[0]['start_stamp_ns'] <= query <= segments[0]['end_stamp_ns']:
            continue
        index = int(np.searchsorted(times, query))
        original, replayed = np.asarray(row['T_prior_reference']), poses['zero'][index]
        baseline_errors.append([float(np.linalg.norm(original[:3, 3]-replayed[:3, 3])),
                                float(np.rad2deg(Rotation.from_matrix(original[:3, :3].T@replayed[:3, :3]).magnitude()))])
    report['baseline_comparison_before_first_recording_gap'] = {
        'compared_original_prior_stamps': len(baseline_errors),
        'max_translation_m_rotation_deg': np.max(baseline_errors, axis=0).tolist() if baseline_errors else None}
    prefix.parent.mkdir(parents=True, exist_ok=True)
    if query_stamps:
        usable_queries, missing_queries = [], []
        for query in query_stamps:
            index = int(np.searchsorted(times, query))
            good = (0 < index < len(times)-1 and times[index] == query and observed[index-1] and observed[index])
            (usable_queries if good else missing_queries).append((query, index))
        report['query_stamps'] = {'source_file': str(args.query_stamps), 'requested': len(query_stamps),
                                  'covered': len(usable_queries), 'unsupported_stamps_ns': [q for q, _ in missing_queries]}
        for name, value in poses.items():
            output = prefix.parent/(prefix.name+'_'+name+'_prior.jsonl')
            with output.open('x') as stream:
                for query, index in usable_queries:
                    stream.write(json.dumps({'stamp_ns': query, 'T_prior_reference': value[index].tolist()})+'\n')
    np.savez_compressed(prefix.with_suffix('.npz'), stamps_ns=times, interval_observed=observed,
                        interval_wheel_motion=moving, **{'poses_'+name: value for name, value in poses.items()})
    with prefix.with_suffix('.json').open('x') as output:
        json.dump(report, output, ensure_ascii=False, allow_nan=False, indent=2)
    print(json.dumps({'status': 'PASS', 'output_prefix': str(prefix), 'covered_duration_s': report['integration']['covered_duration_s'],
                      'screened_windows': len(candidates['screened_windows']), 'usable_training_windows': candidates['usable_training_windows']}))


if __name__ == '__main__':
    main()
