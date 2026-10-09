"""Explicit geometric masks for mapping, without changing source records.

Invalidated XYZ rows become NaN in an owning derived cloud. Row positions,
other fields, source identity and original SourceFrame CDR remain unchanged.
Pixel adjacency requires an organized cloud or explicit producer dimensions;
point count alone is never interpreted as a sensor image layout.
"""
import copy
import math

import numpy as np

from wc_sensors.pointcloud import decode_pointcloud2, validate_pointcloud2


DEFAULT_CLOUD_FILTER = {
    'enabled': False,
    'range': {'enabled': True, 'min_m': .3, 'max_m': 15.},
    'organized_support': {'enabled': True, 'min_neighbors': 2,
                          'distance_offset_m': .03, 'distance_scale': .02},
    'ground_height': {'enabled': True, 'min_m': -.1, 'max_m': 2.5},
}
GROUND_BASIS = 'CONFIGURED_AXLE_PLANE_PLUS_WHEEL_RADIUS_UNVALIDATED'


def _require(condition, message):
    if not condition:
        raise ValueError('CLOUD_FILTER_' + message)


def _number(value):
    return type(value) in (int, float) and math.isfinite(value)


def resolve_cloud_filter(config):
    """Validate optional overrides; a missing block preserves old behavior.

The returned settings retain the requested switch. effective_enabled() also
requires explicit mapping_enabled, so ordinary raw/filtered preview is intact.
"""
    supplied = config.get('cloud_filter', {})
    _require(isinstance(supplied, dict), 'OBJECT_REQUIRED')
    _require(not set(supplied)-set(DEFAULT_CLOUD_FILTER), 'UNKNOWN_FIELDS')
    result = copy.deepcopy(DEFAULT_CLOUD_FILTER)
    for key, value in supplied.items():
        if key == 'enabled':
            result[key] = value
        else:
            _require(isinstance(value, dict) and not set(value)-set(result[key]), 'INVALID_'+key.upper())
            result[key].update(value)
    _require(type(result['enabled']) is bool, 'ENABLED_MUST_BE_BOOL')
    for key in ('range', 'organized_support', 'ground_height'):
        _require(type(result[key]['enabled']) is bool, key.upper()+'_ENABLED_MUST_BE_BOOL')
    for key in ('range', 'ground_height'):
        low, high = result[key]['min_m'], result[key]['max_m']
        _require(_number(low) and _number(high) and low < high, 'INVALID_'+key.upper()+'_BOUNDS')
    _require(result['range']['min_m'] >= 0, 'NEGATIVE_RANGE')
    support = result['organized_support']
    _require(type(support['min_neighbors']) is int and 0 <= support['min_neighbors'] <= 8,
             'INVALID_MIN_NEIGHBORS')
    for name in ('distance_offset_m', 'distance_scale'):
        _require(_number(support[name]) and support[name] >= 0, 'INVALID_'+name.upper())
    if effective_enabled(config, result) and result['ground_height']['enabled']:
        radius = config.get('wheel_candidate', {}).get('wheel_radius_m')
        _require(_number(radius) and 0 < radius <= 1, 'EXPLICIT_WHEEL_RADIUS_REQUIRED')
    return result


def effective_enabled(config, settings):
    return settings['enabled'] and config.get('mapping_enabled') is True


def _organized_shape(message):
    layout = validate_pointcloud2(message.cloud)
    if layout['height'] > 1:
        return (layout['height'], layout['width']), 'POINTCLOUD2_ORGANIZED'
    flags = getattr(message, 'diagnostic_flags', [])
    values = {key: [flag.split('=', 1)[1] for flag in flags if flag.startswith(key+'=')]
              for key in ('sdk_width', 'sdk_height', 'sdk_point_order')}
    if not any(values.values()):
        return None, 'LAYOUT_UNAVAILABLE'
    if any(len(value) != 1 for value in values.values()) or values['sdk_point_order'] != ['row_major']:
        return None, 'LAYOUT_METADATA_INVALID'
    width, height = values['sdk_width'][0], values['sdk_height'][0]
    if not width.isascii() or not height.isascii() or not width.isdecimal() or not height.isdecimal():
        return None, 'LAYOUT_METADATA_INVALID'
    # Bound strings before int conversion and shape dimensions before allocation.
    if len(width) > 6 or len(height) > 6:
        return None, 'LAYOUT_METADATA_INVALID'
    width, height = int(width), int(height)
    if width < 2 or height < 2 or width*height != layout['point_count']:
        return None, 'LAYOUT_METADATA_INVALID'
    return (height, width), 'SDK_REPORTED_ROW_MAJOR'


def support_mask(xyz, valid, shape, settings):
    """Eight pixel neighbours, no edge wrapping and no iterative erosion."""
    grid = xyz.reshape(*shape, 3)
    mask = valid.reshape(shape)
    ranges = np.linalg.norm(grid, axis=2)
    support = np.zeros(shape, dtype=np.uint8)
    height, width = shape
    for dy in (-1, 0, 1):
        for dx in (-1, 0, 1):
            if dx == dy == 0:
                continue
            y0, y1 = max(0, -dy), min(height, height-dy)
            x0, x1 = max(0, -dx), min(width, width-dx)
            here = grid[y0:y1, x0:x1]
            other = grid[y0+dy:y1+dy, x0+dx:x1+dx]
            pair = mask[y0:y1, x0:x1] & mask[y0+dy:y1+dy, x0+dx:x1+dx]
            limit = settings['distance_offset_m'] + settings['distance_scale'] * np.maximum(
                ranges[y0:y1, x0:x1], ranges[y0+dy:y1+dy, x0+dx:x1+dx])
            with np.errstate(invalid='ignore', over='ignore'):
                supported = pair & (np.sum((here-other)**2, axis=2) <= limit**2)
            support[y0:y1, x0:x1] += supported.astype(np.uint8)
    return (mask & (support >= settings['min_neighbors'])).reshape(-1)


def _invalidate_rows(cloud, keep):
    """Only overwrite XYZ bytes, preserving layout, padding and all other fields."""
    result = copy.deepcopy(cloud)
    layout = validate_pointcloud2(cloud)
    storage = bytearray(cloud.data)
    reject = ~keep.reshape(layout['height'], layout['width'])
    for name in ('x', 'y', 'z'):
        field = layout['fields'][name]
        dtype = np.dtype(('>' if layout['is_bigendian'] else '<') + field['dtype'])
        values = np.ndarray(reject.shape, dtype=dtype, buffer=storage, offset=field['offset'],
                            strides=(layout['row_step'], layout['point_step']))
        values[reject] = np.nan
    result.data = bytes(storage)
    result.is_dense = bool(keep.all())
    return result


def filter_cloud(cloud, members, transforms, config):
    """Mask a build_cloud result using original per-side coordinates.

Returns (derived_cloud, JSON report). Every side must retain an observation
before a dual pair is publishable; rejected pairs still retain original CDRs.
Ground height is a configured axle-plane assumption, not a fitted ground truth.
"""
    settings = resolve_cloud_filter(config)
    active = effective_enabled(config, settings)
    report = {'schema_version': 1, 'enabled': active, 'requested_enabled': settings['enabled'],
              'settings': settings, 'row_policy': 'PRESERVE_ROWS_INVALIDATE_REJECTED_XYZ_ONLY',
              'source_records_modified': False, 'publishable': True, 'skip_reason': None,
              'empty_sides': [], 'sources': []}
    if not active:
        report['disabled_reason'] = 'SWITCH_OFF' if not settings['enabled'] else 'PREVIEW_PRESERVES_SOURCE'
        return cloud, report
    output_xyz = decode_pointcloud2(cloud)
    _require(len(output_xyz) == sum(source.raw_count for source, _ in members), 'OUTPUT_ROW_COUNT_MISMATCH')
    height_settings = settings['ground_height']
    if height_settings['enabled']:
        base_axle = np.asarray(transforms['T_base_axle'], dtype=float)
        axle_base = np.linalg.inv(base_axle)
        radius = config['wheel_candidate']['wheel_radius_m']
        ground_z = (output_xyz @ axle_base[:3, :3].T + axle_base[:3, 3])[:, 2] + radius
        report['ground_reference'] = {'basis': GROUND_BASIS, 'ground_truth_validated': False,
            'wheel_radius_m': radius, 'ground_z_in_axle_m': -radius,
            'hardware_setup_provenance': copy.deepcopy(config.get('hardware_setup_provenance')),
            'wheel_parameter_provenance': config['wheel_candidate'].get('provenance'),
            'limitation': 'Assumes ground parallel to configured axle XY plane; not live floor fitting.'}
    keeps, offset = [], 0
    for source, metadata in members:
        xyz = decode_pointcloud2(source.cloud)
        finite = np.isfinite(xyz).all(axis=1)
        distance = np.linalg.norm(xyz, axis=1)
        keep = finite & (distance > 0)
        row = {'side': source.side, 'raw_key': copy.deepcopy(metadata.get('raw_key')),
               'output_row_start': offset, 'input_rows': len(xyz), 'finite_points': int(finite.sum()),
               'nonzero_points': int(keep.sum())}
        band = settings['range']
        if band['enabled']:
            keep &= (distance >= band['min_m']) & (distance <= band['max_m'])
        row['after_range'] = int(keep.sum())
        shape, layout_status = _organized_shape(source)
        if settings['organized_support']['enabled'] and shape is not None:
            keep = support_mask(xyz, keep, shape, settings['organized_support'])
            row['support_status'] = 'APPLIED'
        else:
            row['support_status'] = layout_status if settings['organized_support']['enabled'] else 'DISABLED'
        row['organized_shape'] = list(shape) if shape is not None else None
        row['layout_evidence'] = layout_status
        row['after_support'] = int(keep.sum())
        if height_settings['enabled']:
            heights = ground_z[offset:offset+len(xyz)]
            keep &= (heights >= height_settings['min_m']) & (heights <= height_settings['max_m'])
        row['after_height'] = row['retained_points'] = int(keep.sum())
        row['rejected_points'] = len(xyz)-row['retained_points']
        if not keep.any():
            report['empty_sides'].append(source.side)
        report['sources'].append(row)
        keeps.append(keep)
        offset += len(xyz)
    report['input_rows'] = offset
    report['retained_points'] = sum(row['retained_points'] for row in report['sources'])
    report['publishable'] = not report['empty_sides']
    if not report['publishable']:
        report['skip_reason'] = 'FILTERED_SOURCE_EMPTY'
    return _invalidate_rows(cloud, np.concatenate(keeps)), report
