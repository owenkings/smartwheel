"""Reproduce the exact-stamp trust ledger from frozen saved diagnostics.

Run with --coverage-dir (coverage_*.json), --witnesses (remote exported original
AMCL results), and --output. No ROS import, publishers, map edits, or pose feedback.
"""
from __future__ import annotations

import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import statistics

from .localization_trust import evaluate_localization_evidence, POLICY


def index(rows):
    result = {}
    for row in rows:
        stamp = row['stamp_ns']
        if isinstance(stamp, bool) or not isinstance(stamp, int) or not 0 <= stamp < 2**63:
            raise ValueError('invalid capture stamp')
        if stamp in result:
            raise ValueError('duplicate capture stamp')
        result[stamp] = row
    if list(result) != sorted(result):
        raise ValueError('capture stamps must increase')
    return result


def common_axle_pose(coverage_row, mount):
    # CoverageAnalyzer's "base" is its unshifted laser hypothesis, not axle.
    x, y, yaw = coverage_row['base']['pose_x_y_yaw']
    c, s = math.cos(yaw), math.sin(yaw)
    return [x - c * mount[0] + s * mount[1],
            y - s * mount[0] - c * mount[1], yaw]


def coverage_packet(row):
    if row is None:
        return None
    return {'stamp_ns': row['stamp_ns'],
            **{key: row['base'][key] for key in ['observed_endpoints', 'known_fraction', 'supported_fraction']},
            'along_heading_ambiguity_evidence': row['along_heading_ambiguity_evidence']}


def stats(values):
    values = [v for v in values if v is not None]
    return {'count': len(values), 'median': statistics.median(values), 'max': max(values)} if values else None


def summary(rows):
    return {'frames': len(rows), 'not_trusted_frames': sum(r['trust_status'] == 'NOT_TRUSTED' for r in rows),
            'near_witness_exact_frames': sum(r['near_return_witness']['status'] == 'EXACT_STAMP_WITNESS' for r in rows),
            'near_witness_missing_frames': sum(r['near_return_witness']['status'] == 'NOT_EVALUATED_MISSING_WITNESSES' for r in rows),
            'near_raw_overlap_frames': sum((r['near_return_witness']['raw_inside_circle_count'] or 0) > 0 for r in rows),
            'near_voxel_overlap_frames': sum((r['near_return_witness']['overlap_voxel_count'] or 0) > 0 for r in rows),
            'diagnostic_flag_counts': dict(Counter(f for r in rows for f in r['diagnostic_flags'])),
            'reference_category_counts': dict(Counter(r['predeclared_reference_category'] for r in rows)),
            'translation_disagreement_m': stats([r['translation_disagreement_m'] for r in rows]),
            'yaw_disagreement_deg': stats([math.degrees(r['yaw_disagreement_rad'])
                                           if r['yaw_disagreement_rad'] is not None else None for r in rows]),
            'small_covariance_with_conflict_frames': sum(
                'SMALL_PARTICLE_COVARIANCE_WITH_REFERENCE_CONFLICT' in r['diagnostic_flags'] for r in rows),
            'good_endpoint_fit_with_conflict_frames': sum(
                'GOOD_ENDPOINT_FIT_WITH_REFERENCE_CONFLICT' in r['diagnostic_flags'] for r in rows)}


def evaluate(coverage_dir, witnesses_path):
    blobs = {}
    def read(path):
        path = Path(path)
        raw = path.read_bytes()
        blobs[str(path.resolve())] = hashlib.sha256(raw).hexdigest()
        return json.loads(raw)
    original = read(witnesses_path)
    if not original['sources_unchanged']:
        raise ValueError('source export did not verify unchanged inputs')
    epoch = original['source_epoch_ns']
    reference_rows = read(coverage_dir / 'coverage_frozen_prior.json')['frames']
    reference = index(reference_rows)
    near_witnesses = index(original['frames']) if isinstance(original.get('frames'), list) else {}
    contract = read(coverage_dir / 'coverage_evaluation.json')
    if len(reference) != contract['query_frames']:
        raise ValueError('reference ledger lost query frames')
    source_mount = original['base_from_lidar']
    expected = [[1., 0., 0., .323], [0., 1., 0., .22875], [0., 0., 1., 0.], [0., 0., 0., 1.]]
    if source_mount != expected:
        raise ValueError('explicit parallel-axis planar mount adapter does not match export')
    mount = [.323, .22875]
    report = {'schema_version': 1, 'status': 'LOCALIZATION_NOT_TRUSTED',
              'query_window_s': contract['query_window_s'], 'training_window_s': contract['training_window_s'],
              'segments_predeclared_s': contract['segments_predeclared_s'],
              'expected_query_frames': len(reference), 'scope': 'FROZEN_OFFLINE_EXACT_STAMP_EVIDENCE_LEDGER',
              'coverage_selection_source': 'DECLARED_FROZEN_PRIOR_ONLY_NEVER_AMCL_POSE',
              'coverage_selection_is_physical_truth': False,
              'reference_category_definition': 'INSUFFICIENT_IF_BEAMS_LT20_OR_KNOWN_LT0_8_OR_SUPPORT_LT0_7;_THEN_AMBIGUITY;_ELSE_UNVALIDATED',
              'coverage_policy_delta': 'Prior coverage_insufficient checks known>=0.8 and beams>=20; this ledger additionally requires support>=0.7. This is a stronger review criterion, not a change to the original diagnostic or calibration gate.',
              'reference_independence': 'SHARED_WITH_MAP_ODOMETRY_NOT_INDEPENDENT_PHYSICAL_TRUTH',
              'coordinate_reference': 'CONFIGURED_PLANAR_AXLE_UNVALIDATED',
              'base_from_lidar': source_mount, 'timestamp_basis': original['source_time'],
              'receipt_time_available': False, 'all_query_frames_retained': True,
              'near_return_join': 'EXACT_CAPTURE_STAMP_ONLY_NO_NEAREST_TIME_JOIN',
              'policy': dict(POLICY), 'cases': {}, 'physical_navigation_validated': False,
              'control_topics_published': False, 'source_export_sha256': original['source_sha256']}
    for case_name in ['likelihood_field', 'likelihood_field_prob']:
        case = original['localization_results'][case_name]
        if case['base_from_lidar'] != source_mount:
            raise ValueError('AMCL base transform differs from fixed source transform')
        estimates = index(case['trajectory'])
        coverage_rows = index(read(coverage_dir / ('coverage_' + case_name + '.json'))['frames'])
        if set(estimates) - set(reference) or set(coverage_rows) - set(reference):
            raise ValueError('unexpected query stamps in case')
        ledger = []
        for stamp, ref in reference.items():
            actual, fitted = estimates.get(stamp), coverage_rows.get(stamp)
            packet = None
            if actual is not None:
                pose = [actual['x_m'], actual['y_m'], actual['yaw_rad']]
                cov = actual['covariance']
                if len(cov) != 36 or not all(math.isfinite(v) for v in cov):
                    raise ValueError('invalid original particle covariance')
                variances = [cov[i] for i in [0, 7, 35]]
                if min(variances) < 0:
                    raise ValueError('negative original particle variance')
                packet = {'stamp_ns': stamp, 'pose_x_y_yaw': pose,
                          'particle_std_x_y_yaw': [math.sqrt(v) for v in variances]}
                if fitted is not None:
                    recovered = common_axle_pose(fitted, mount)
                    if max(abs(a - b) for a, b in zip(pose, recovered)) > 1e-9:
                        raise ValueError('coverage estimate and original axle pose do not match')
            row = evaluate_localization_evidence(stamp, stamp, estimate=packet,
                reference={'stamp_ns': stamp, 'pose_x_y_yaw': common_axle_pose(ref, mount)},
                reference_coverage=coverage_packet(ref), estimated_coverage=coverage_packet(fitted),
                acquisition_time_verified=False, coordinate_reference_verified=False)
            row['relative_time_s'] = (stamp - epoch) * 1e-9
            near = near_witnesses.get(stamp)
            if near is None or any(key not in near for key in ['raw_inside_circle_count', 'overlap_voxel_count']):
                row['near_return_witness'] = {'status': 'NOT_EVALUATED_MISSING_WITNESSES',
                    'raw_inside_circle_count': None, 'overlap_voxel_count': None}
            else:
                counts = {key: near[key] for key in ['raw_inside_circle_count', 'overlap_voxel_count']}
                if any(isinstance(v, bool) or not isinstance(v, int) or v < 0 for v in counts.values()):
                    raise ValueError('near-return counts must be nonnegative integers')
                row['near_return_witness'] = {'status': 'EXACT_STAMP_WITNESS', **counts}
            ledger.append(row)
        categories = sorted({r['predeclared_reference_category'] for r in ledger})
        report['cases'][case_name] = {'summary': summary(ledger),
            'by_predeclared_reference_category': {c: summary([r for r in ledger if r['predeclared_reference_category'] == c])
                                                  for c in categories},
            'by_predeclared_time_segment': [
                {'window_s': [a, b], **summary([r for r in ledger if a <= r['relative_time_s'] < b])}
                for a, b in report['segments_predeclared_s']], 'frames': ledger}
    report['input_sha256'] = blobs
    report['inputs_unchanged'] = all(hashlib.sha256(Path(p).read_bytes()).hexdigest() == h for p, h in blobs.items())
    if not report['inputs_unchanged']:
        raise ValueError('an input changed during evaluation')
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--project-root', type=Path, required=True)
    parser.add_argument('--coverage-dir', type=Path, required=True)
    parser.add_argument('--witnesses', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args(argv)
    from wc_runtime.storage_policy import StoragePolicy
    policy = StoragePolicy(args.project_root)
    policy.check()
    output = policy.resolve(args.output)
    if not Path(policy.logical(output)).parts or Path(policy.logical(output)).parts[0] != 'reports':
        raise ValueError('localization evidence outputs must stay under configured reports storage')
    if output.exists():
        raise ValueError('refusing to overwrite an existing localization evidence report')
    result = evaluate(args.coverage_dir, args.witnesses)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, indent=2, allow_nan=False), encoding='utf-8')
    policy.check()
    print(json.dumps({n: c['summary'] for n, c in result['cases'].items()}, indent=2))


if __name__ == "__main__":
    main()
