"""Offline range repeatability statistics; no accuracy or interference verdict.

One NPZ is one sensor/run. Preserve all 9600 pixel slots and NaN invalid points.
raw_xyz/filtered_xyz, when both present, must be paired by the producer using
the exact source frame identity, not arrival proximity. XYZ units are metres.
"""

import argparse
import hashlib
import json
from pathlib import Path
import warnings

import numpy as np


PIXELS = 9600


def _require(condition, message):
    if not condition:
        raise ValueError(message)


def _quantile(values, q, axis=None):
    with warnings.catch_warnings():
        warnings.simplefilter('ignore', RuntimeWarning)  # Empty pixels remain NaN, never zero.
        return np.nanquantile(values, q, axis=axis)


def _summary(values):
    finite = np.asarray(values, dtype=float)
    finite = finite[np.isfinite(finite)]
    if not finite.size:
        return {'count': 0, 'median': None, 'p05': None, 'p95': None, 'max': None}
    return {'count': int(finite.size), 'median': float(np.median(finite)),
            'p05': float(np.quantile(finite, .05)), 'p95': float(np.quantile(finite, .95)),
            'max': float(np.max(finite))}


def _json_safe(value):
    if isinstance(value, np.ndarray):
        return _json_safe(value.tolist())
    if isinstance(value, dict):
        return {key: _json_safe(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return [_json_safe(item) for item in value]
    if isinstance(value, (float, np.floating)):
        return float(value) if np.isfinite(value) else None
    if isinstance(value, np.integer):
        return int(value)
    return value


def _integer_vector(value, count, name, minimum=0):
    value = np.asarray(value)
    _require(value.shape == (count,) and value.dtype.kind in 'iu', name + ' must be an integer T-vector')
    rows = [int(item) for item in value]
    _require(all(item >= minimum for item in rows), name + ' contains values below its minimum')
    return rows


def _differences(values):
    return [right-left for left, right in zip(values, values[1:])]


def _correlation(first, second):
    finite = np.isfinite(first) & np.isfinite(second)
    if np.count_nonzero(finite) < 3:
        return np.nan
    x, y = first[finite], second[finite]
    x, y = x-np.mean(x), y-np.mean(y)
    denominator = np.linalg.norm(x)*np.linalg.norm(y)
    return float(np.clip(np.dot(x, y)/denominator, -1, 1)) if denominator > 1e-20 else np.nan


def _rows(delta, height, width):
    data = delta.reshape(-1, height, width)
    medians = _quantile(data, .5, axis=2)
    row_span = _quantile(medians, .95, axis=1)-_quantile(medians, .05, axis=1)
    horizontal, vertical, explained = [], [], []
    for frame in data:
        horizontal.append(_correlation(frame[:, :-1].ravel(), frame[:, 1:].ravel()))
        vertical.append(_correlation(frame[:-1, :].ravel(), frame[1:, :].ravel()))
        valid = np.isfinite(frame)
        if not np.any(valid):
            explained.append(np.nan)
            continue
        counts = valid.sum(axis=1)
        sums = np.nansum(frame, axis=1)
        means = np.divide(sums, counts, out=np.full(height, np.nan), where=counts > 0)
        overall = np.mean(frame[valid])
        total = np.sum((frame[valid]-overall)**2)
        between = np.nansum(counts*(means-overall)**2)
        explained.append(float(np.clip(between/total, 0, 1)) if total > 1e-20 else np.nan)
    return {'state': 'COMPUTED_FOR_EXPLICIT_PIXEL_LAYOUT', 'height': height, 'width': width,
        'row_median_adjacent_delta_m': medians, 'row_median_delta_p95_minus_p05_m': row_span,
        'row_explained_delta_variance_fraction': explained,
        'horizontal_neighbor_delta_correlation': horizontal, 'vertical_neighbor_delta_correlation': vertical,
        'row_span_summary_m': _summary(row_span), 'row_explained_variance_summary': _summary(explained),
        'horizontal_correlation_summary': _summary(horizontal), 'vertical_correlation_summary': _summary(vertical),
        'interpretation': 'Descriptive coherence only; constant/no-data correlations are null, not perfect stability.'}


def _representation(xyz, adjacent, layout):
    finite = np.isfinite(xyz).all(axis=2)
    with np.errstate(over='ignore', invalid='ignore'):
        ranges = np.linalg.norm(xyz.astype(np.float64), axis=2)
    # Finite zero-length vectors are an explicit invalid-depth sentinel.
    valid = finite & np.isfinite(ranges) & (ranges > 0)
    ranges[~valid] = np.nan
    observations = valid.sum(axis=0)
    temporal_eligible = observations >= 3
    median = _quantile(ranges, .5, axis=0)
    mad = _quantile(np.abs(ranges-median), .5, axis=0)
    span = _quantile(ranges, .95, axis=0)-_quantile(ranges, .05, axis=0)
    for statistic in (median, mad, span):
        statistic[~temporal_eligible] = np.nan
    delta = np.diff(ranges, axis=0)
    delta[~adjacent] = np.nan
    pair_counts = np.isfinite(delta).sum(axis=0)
    delta_median = _quantile(np.abs(delta), .5, axis=0)
    delta_p95 = _quantile(np.abs(delta), .95, axis=0)
    delta_median[pair_counts < 2] = np.nan
    delta_p95[pair_counts < 2] = np.nan
    common = valid.all(axis=0)
    common_median = _quantile(ranges[:, common], .5, axis=1) if np.any(common) else np.full(len(xyz), np.nan)
    return {'range_definition': 'sqrt(x*x+y*y+z*z), metres; not axial distance or ground-truth error',
        'pixel_valid_observations': observations, 'pixel_valid_fraction': observations/len(xyz),
        'pixel_temporal_stat_eligible': temporal_eligible,
        'pixel_range_median_m': median, 'pixel_range_mad_m_unscaled': mad,
        'pixel_range_p95_minus_p05_m': span, 'pixel_adjacent_valid_pairs': pair_counts,
        'pixel_adjacent_abs_delta_median_m': delta_median, 'pixel_adjacent_abs_delta_p95_m': delta_p95,
        'frame_valid_fraction': valid.mean(axis=1), 'frame_median_range_m': _quantile(ranges, .5, axis=1),
        'common_valid_pixel_count': int(common.sum()), 'frame_common_pixel_median_range_m': common_median,
        'frame_adjacent_median_delta_m': _quantile(delta, .5, axis=1),
        'summary': {'range_mad_m_unscaled': _summary(mad), 'range_p95_minus_p05_m': _summary(span),
            'adjacent_abs_delta_median_m': _summary(delta_median), 'adjacent_abs_delta_p95_m': _summary(delta_p95),
            'valid_fraction': _summary(observations/len(xyz))},
        'rowwise': _rows(delta, *layout) if layout else {'state': 'DISABLED_PIXEL_LAYOUT_NOT_PROVIDED'},
        'cautions': ['Finite/nonzero validity is observable coverage, not measurement accuracy.',
            'Frame medians may change when the valid-pixel mask changes; compare common-pixel medians too.',
            'Per-pixel temporal stats require >=3 valid frames; adjacent stats require >=2 eligible pairs.',
            'Temporal filtering can lower variation while adding bias or lag; no quality pass threshold is applied.']}


def analyze(arrays, *, height=None, width=None, scene_static_source=None, run_label=None):
    keys = [key for key in ('raw_xyz', 'filtered_xyz') if key in arrays]
    _require(bool(keys), 'raw_xyz and/or filtered_xyz required')
    layout = None
    if height is not None or width is not None:
        _require(type(height) is int and type(width) is int and height > 1 and width > 1
                 and height*width == PIXELS, 'explicit layout must be H>1, W>1 and H*W=9600')
        layout = (height, width)
    streams = {key: np.asarray(arrays[key]) for key in keys}
    first = streams[keys[0]]
    _require(first.ndim == 3 and first.shape[1:] == (PIXELS, 3) and first.shape[0] >= 3,
             'XYZ requires T>=3 and shape T,9600,3; keep all pixel slots')
    count = first.shape[0]
    for key, value in streams.items():
        _require(value.shape == first.shape and value.dtype.kind == 'f', key + ' requires equal T,9600,3 floating XYZ in metres')
    sequence = _integer_vector(arrays['frame_sequence'], count, 'frame_sequence')
    host = _integer_vector(arrays['host_ns'], count, 'host_ns', 1)
    sequence_delta, host_delta = _differences(sequence), _differences(host)
    adjacent = np.asarray([seq == 1 and dt > 0 for seq, dt in zip(sequence_delta, host_delta)])
    for key in ('raw_frame_sequence', 'filtered_frame_sequence'):
        if key in arrays:
            _require(_integer_vector(arrays[key], count, key) == sequence, key + ' differs from shared exact sequence')
    device = None
    if 'device_raw' in arrays:
        values = _integer_vector(arrays['device_raw'], count, 'device_raw')
        differences = _differences(values)
        device = {'values': values, 'difference_raw_units': differences,
            'nonincreasing_indices': [index+1 for index, delta in enumerate(differences) if delta <= 0],
            'unit': 'UNSPECIFIED_RAW_DEVICE_UNIT', 'common_time_validated': False}
    if scene_static_source is not None:
        _require(isinstance(scene_static_source, str) and bool(scene_static_source.strip()), 'static-scene source must be a nonempty label')
    result = {'schema': 'wc_lidar_range_stability_v1', 'status': 'COMPUTED_NO_QUALITY_VERDICT',
        'run_label': run_label, 'scene_assumption': 'DECLARED_STATIC' if scene_static_source else 'UNVERIFIED',
        'scene_static_source': scene_static_source, 'ground_truth_available': False,
        'accuracy_validated': False, 'optical_interference_diagnosed': False, 'units': 'm',
        'frames': count, 'pixels_per_frame': PIXELS, 'frame_sequence': sequence, 'host_ns': host,
        'arrival': {'interval_ns': host_delta, 'positive_interval_summary_s': _summary([delta*1e-9 for delta in host_delta if delta > 0]),
            'nonincreasing_indices': [index+1 for index, delta in enumerate(host_delta) if delta <= 0],
            'interpretation': 'Host arrivals only; not exposure duration, measurement timestamp or ranging quality.'},
        'sequence': {'delta': sequence_delta, 'gap_indices': [index+1 for index, delta in enumerate(sequence_delta) if delta > 1],
            'missing_frames_between_samples': sum(max(delta-1, 0) for delta in sequence_delta),
            'duplicate_indices': [index+1 for index, delta in enumerate(sequence_delta) if delta == 0],
            'backward_indices': [index+1 for index, delta in enumerate(sequence_delta) if delta < 0],
            'scope': 'Input source sequence only; internal SDK latest-only drops before publication may not create a gap.'},
        'adjacent_pair_eligible': adjacent, 'device_time': device,
        'pairing_evidence': 'Input producer must match exact frame/epoch identity; matching array length is not independent pairing proof.',
        'representations': {key[:-4]: _representation(value, adjacent, layout) for key, value in streams.items()}}
    result['has_usable_temporal_pixels'] = any(
        representation['summary']['range_mad_m_unscaled']['count'] > 0 for representation in result['representations'].values())
    if not result['has_usable_temporal_pixels']:
        result['status'] = 'INSUFFICIENT_VALID_DATA'
    return _json_safe(result)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--height', type=int)
    parser.add_argument('--width', type=int)
    parser.add_argument('--scene-static-source', help='declaration provenance only, never ground-truth evidence')
    parser.add_argument('--run-label')
    args = parser.parse_args(argv)
    try:
        with np.load(args.input, allow_pickle=False) as arrays:
            result = analyze(arrays, height=args.height, width=args.width,
                scene_static_source=args.scene_static_source, run_label=args.run_label)
        result['input'] = str(args.input.resolve())
        digest = hashlib.sha256()
        with args.input.open('rb') as source:
            for chunk in iter(lambda: source.read(1024*1024), b''):
                digest.update(chunk)
        result['input_sha256'] = digest.hexdigest()
    except Exception as error:
        result = {'schema': 'wc_lidar_range_stability_v1', 'status': 'ERROR',
            'error': type(error).__name__ + ': ' + str(error), 'accuracy_validated': False}
    with args.output.open('x', encoding='utf-8') as output:
        json.dump(result, output, indent=2, ensure_ascii=False, allow_nan=False)
        output.write('\n')
    print(json.dumps({'status': result['status'], 'output': str(args.output)}))
    return 0 if result['status'] == 'COMPUTED_NO_QUALITY_VERDICT' else 1


if __name__ == '__main__':
    raise SystemExit(main())
