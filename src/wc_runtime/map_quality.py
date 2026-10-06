"""Read-only RTABMap 0.23.7 file, moving-map and scan evidence.

These are predeclared experimental qualification limits, not a calibration or
an independent geometry accuracy test. A static map can remain a valid file.
Native encodings were checked against actual target databases and upstream:
https://github.com/introlab/rtabmap/blob/0.23.7/corelib/src/Compression.cpp
https://github.com/introlab/rtabmap/blob/0.23.7/corelib/src/DBDriverSqlite3.cpp
"""
from contextlib import closing
from dataclasses import asdict, dataclass
import hashlib
import math
from pathlib import Path
import sqlite3
import struct
import zlib

import numpy as np


@dataclass(frozen=True)
class MapQualityPolicy:
    schema_version: int = 1
    status: str = 'EXPERIMENTAL_NOT_CALIBRATED'
    min_nodes: int = 3
    min_path_length_m: float = 1.0
    min_translation_extent_m: float = .5
    min_valid_scan_points: int = 100
    min_valid_scan_fraction: float = .9
    require_single_connected_component: bool = True
    max_nodes: int = 100000
    max_links: int = 1000000
    max_uncompressed_scan_bytes: int = 64*1024*1024

    def __post_init__(self):
        if type(self.schema_version) is not int or self.schema_version != 1 or self.status != 'EXPERIMENTAL_NOT_CALIBRATED':
            raise ValueError('map quality policy must remain schema 1 / EXPERIMENTAL_NOT_CALIBRATED')
        for key in ('min_nodes', 'min_valid_scan_points', 'max_nodes', 'max_links', 'max_uncompressed_scan_bytes'):
            if type(getattr(self, key)) is not int or getattr(self, key) <= 0:
                raise ValueError('positive integer map quality policy required: '+key)
        if self.min_nodes < 2 or self.min_nodes > self.max_nodes:
            raise ValueError('moving-map minimum must be at least two nodes within the inspection bound')
        for key in ('min_path_length_m', 'min_translation_extent_m', 'min_valid_scan_fraction'):
            value = getattr(self, key)
            if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
                raise ValueError('positive finite map quality policy required: '+key)
        if self.min_valid_scan_fraction > 1 or type(self.require_single_connected_component) is not bool:
            raise ValueError('valid scan fraction and explicit graph connectivity policy required')


def validate_map_quality_policy(value=None):
    if isinstance(value, MapQualityPolicy):
        return asdict(value)
    if value is not None and not isinstance(value, dict):
        raise ValueError('map_quality_policy must be an explicit object')
    try:
        return asdict(MapQualityPolicy(**(value or {})))
    except TypeError as error:
        raise ValueError('unsupported map quality policy field: '+str(error)) from error


def _digest(path):
    checksum = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024*1024), b''):
            checksum.update(chunk)
    return checksum.hexdigest()


def _pose(blob):
    if not isinstance(blob, bytes) or len(blob) != 48:
        raise ValueError('expected native 3x4 little-endian float32 pose')
    pose = np.eye(4)
    pose[:3] = np.frombuffer(blob, dtype='<f4').reshape(3, 4)
    rotation = pose[:3, :3]
    if not np.isfinite(pose).all() or not np.allclose(rotation.T@rotation, np.eye(3), atol=1e-3, rtol=0) or not np.isclose(np.linalg.det(rotation), 1., atol=1e-3, rtol=0):
        raise ValueError('native pose is not a finite proper rigid transform')
    return pose


def _compressed_matrix(blob, *, max_bytes):
    """Native compressData: zlib payload followed by rows, cols, cv::Mat type."""
    if not isinstance(blob, bytes) or len(blob) <= 12:
        raise ValueError('missing native compressed matrix')
    rows, cols, cv_type = struct.unpack('<iii', blob[-12:])
    depth, channels = cv_type & 7, (cv_type >> 3)+1
    if rows <= 0 or cols <= 0 or cv_type < 0 or depth not in (4, 5) or not 1 <= channels <= 12:
        raise ValueError('unsupported native matrix dimensions/type')
    expected = rows*cols*channels*4
    if expected > max_bytes:
        raise ValueError('native matrix exceeds declared decompression budget')
    decoder = zlib.decompressobj()
    raw = decoder.decompress(blob[:-12], expected+1)
    if len(raw) != expected or not decoder.eof or decoder.unconsumed_tail or decoder.unused_data:
        raise ValueError('native matrix size/checksum/stream mismatch')
    values = np.frombuffer(raw, dtype='<i4' if depth == 4 else '<f4').reshape(rows, cols, channels)
    return values, {'rows': rows, 'cols': cols, 'cv_type': cv_type, 'channels': channels, 'uncompressed_bytes': expected}


# LaserScan::Format -> (channels, spatial coordinate dimensions). RGB is packed
# into a float: only coordinates, not attribute bits, determine finite points.
_FORMATS = {1: (2, 2), 2: (3, 2), 3: (5, 2), 4: (6, 2),
            5: (3, 3), 6: (4, 3), 7: (4, 3), 8: (6, 3),
            9: (7, 3), 10: (7, 3), 11: (5, 3), 12: (6, 3)}


def _scan(blob, info, policy):
    result = {'status': 'FAIL', 'scan_present': bool(blob), 'scan_info_present': bool(info),
              'compressed_bytes': len(blob or b''), 'compressed_sha256': hashlib.sha256(blob).hexdigest() if blob else None,
              'reasons': []}
    if not blob:
        result['reasons'] = ['SCAN_MISSING_OR_EMPTY']
        return result
    if not info:
        result.update(status='UNKNOWN', reasons=['SCAN_INFO_MISSING'])
        return result
    try:
        if len(info) != 76:
            result.update(status='UNKNOWN', reasons=['UNSUPPORTED_SCAN_INFO_ENCODING'])
            return result
        fields = np.frombuffer(info, dtype='<f4')
        if not np.isfinite(fields).all() or int(fields[0]) != fields[0]:
            raise ValueError('nonfinite/invalid scan metadata')
        scan_format = int(fields[0])
        if scan_format not in _FORMATS:
            result.update(status='UNKNOWN', reasons=['UNSUPPORTED_SCAN_FORMAT'], scan_format=scan_format)
            return result
        local_pose = _pose(info[28:])
        values, metadata = _compressed_matrix(blob, max_bytes=policy.max_uncompressed_scan_bytes)
        expected_channels, dimensions = _FORMATS[scan_format]
        if metadata['cv_type'] & 7 != 5 or metadata['channels'] != expected_channels:
            raise ValueError('scan format and native matrix type/channels disagree')
        points = values.reshape(-1, expected_channels)[:, :dimensions]
        finite = np.isfinite(points).all(axis=1)
        nonzero = finite & np.any(points != 0, axis=1)
        count = int(nonzero.sum())
        fraction = count/len(points)
        result.update(matrix=metadata, scan_format=scan_format, coordinate_dimensions=dimensions,
                      local_transform=local_pose.tolist(), point_rows=len(points), finite_coordinate_points=int(finite.sum()),
                      valid_nonzero_points=count, valid_point_fraction=fraction,
                      validity_definition='finite nonzero XY/XYZ coordinates; not registration or geometry accuracy')
        if count < policy.min_valid_scan_points:
            result['reasons'].append('INSUFFICIENT_VALID_SCAN_POINTS')
        if fraction < policy.min_valid_scan_fraction:
            result['reasons'].append('INSUFFICIENT_VALID_SCAN_FRACTION')
        result['status'] = 'PASS' if not result['reasons'] else 'FAIL'
    except (ValueError, zlib.error, OverflowError) as error:
        result['reasons'] = ['SCAN_DECODE_OR_METADATA_INVALID: '+str(error)]
    return result


def _motion(nodes, poses):
    ordered = sorted(nodes, key=lambda row: (row['stamp'], row['id']))
    xyz = np.asarray([poses[row['id']][:3, 3] for row in ordered])
    span = np.ptp(xyz, axis=0) if len(xyz) else np.zeros(3)
    return {'ordered_node_ids': [row['id'] for row in ordered],
            'path_length_m': float(np.linalg.norm(np.diff(xyz, axis=0), axis=1).sum()) if len(xyz) > 1 else 0.,
            'translation_extent_m': float(np.linalg.norm(span)), 'axis_extents_m': span.tolist(),
            'first_to_last_translation_m': float(np.linalg.norm(xyz[-1]-xyz[0])) if len(xyz) else 0.,
            'interpretation': 'stored pose motion; not independently measured travel or geometry precision'}


def inspect_map_quality(database, policy=None):
    """Return evidence; only malformed policy/path raises, never promotes accuracy.

    Controller API: inspect_map_quality(db, runtime_config.get('map_quality_policy')).
    The caller can save a static file while retaining NOT_QUALIFIED in this report.
    No writes or exports occur here. Unsupported native schema stays UNKNOWN.
    """
    resolved = validate_map_quality_policy(policy)
    candidate = MapQualityPolicy(**resolved)
    database = Path(database).resolve(strict=True)
    if not database.is_file():
        raise ValueError('map quality requires a regular SQLite database')
    checksum = _digest(database)
    report = {'schema_version': 1, 'status': 'MAP_QUALITY_EVIDENCE_ONLY', 'database': str(database),
              'database_sha256': checksum, 'policy': resolved,
              'policy_source': 'EXPLICIT_CALLER_POLICY' if policy is not None else 'BUILTIN_PREDECLARED_EXPERIMENTAL_CANDIDATE',
              'file_integrity': {'status': 'UNKNOWN'}, 'movement_map_qualification': {'status': 'UNKNOWN', 'reasons': []},
              'geometry_accuracy': {'status': 'UNKNOWN_NO_INDEPENDENT_GEOMETRY_REFERENCE', 'formal_acceptance': False,
                                    'navigation_validated': False}, 'per_node_scans': [],
              'hardware_started': False, 'database_modified': False,
              'encoding_evidence': 'RTABMap 0.23.7 native SQLite and upstream Compression/DBDriverSqlite3'}
    try:
        sidecars = [suffix for suffix in ('-wal', '-journal') if Path(str(database)+suffix).exists()]
        if sidecars:
            raise ValueError('closed native database required; live sidecars: '+','.join(sidecars))
        with closing(sqlite3.connect(database.as_uri()+'?mode=ro', uri=True)) as db:
            db.execute('PRAGMA query_only=ON')
            integrity = db.execute('PRAGMA integrity_check').fetchall()
            report['file_integrity'] = {'status': 'PASS' if integrity == [('ok',)] else 'FAIL',
                                        'sqlite_integrity_check': [row[0] for row in integrity]}
            if report['file_integrity']['status'] != 'PASS':
                return report
            tables = {row[0] for row in db.execute("SELECT name FROM sqlite_master WHERE type='table'")}
            required = {'Node': {'id', 'stamp', 'pose', 'weight', 'map_id'}, 'Link': {'from_id', 'to_id', 'type', 'transform'},
                        'Data': {'id', 'scan', 'scan_info'}, 'Admin': {'version'}}
            missing = {name: sorted(columns-{row[1] for row in db.execute('PRAGMA table_info('+name+')')})
                       for name, columns in required.items() if name not in tables or columns-{row[1] for row in db.execute('PRAGMA table_info('+name+')')}}
            versions = [row[0] for row in db.execute('SELECT version FROM Admin')] if 'Admin' in tables and 'version' not in missing.get('Admin', []) else []
            report['native_schema'] = {'versions': versions, 'supported_version': '0.23.7', 'missing_fields': missing}
            if missing or versions != ['0.23.7']:
                report['movement_map_qualification']['reasons'] = ['UNSUPPORTED_OR_INCOMPLETE_NATIVE_SCHEMA']
                return report
            rows = db.execute('SELECT id,stamp,pose,weight,map_id FROM Node ORDER BY id LIMIT ?', (candidate.max_nodes+1,)).fetchall()
            if len(rows) > candidate.max_nodes:
                report['movement_map_qualification']['reasons'] = ['NODE_INSPECTION_BUDGET_EXCEEDED']
                return report
            nodes, poses, invalid = [], {}, []
            for identifier, stamp, blob, weight, map_id in rows:
                try:
                    if type(identifier) is not int or identifier <= 0 or type(stamp) not in (int, float) or not math.isfinite(stamp) or type(weight) is not int or type(map_id) is not int:
                        raise ValueError('positive node ID, finite stamp and explicit weight/map identity required')
                    poses[identifier] = _pose(blob)
                    nodes.append({'id': identifier, 'stamp': stamp, 'weight': weight, 'map_id': map_id})
                except ValueError as error:
                    invalid.append({'node_id': identifier, 'reason': str(error)})
            report['nodes'] = {'total': len(rows), 'valid_poses': len(nodes), 'invalid': invalid}
            report['motion'] = _motion(nodes, poses)
            report['motion']['pose_source'] = 'Node.pose stored odometry (unoptimized); not fitted or independently measured'
            ids = {row['id'] for row in nodes}
            adjacency = {identifier: set() for identifier in ids}
            links = db.execute('SELECT from_id,to_id,type,transform FROM Link LIMIT ?', (candidate.max_links+1,)).fetchall()
            if len(links) > candidate.max_links:
                report['movement_map_qualification']['reasons'] = ['LINK_INSPECTION_BUDGET_EXCEEDED']
                return report
            types, dangling, bad_links, ignored, edges = {}, [], [], 0, set()
            for start, end, kind, blob in links:
                types[str(kind)] = types.get(str(kind), 0)+1
                if kind in (5, 7, 8):
                    ignored += 1  # Virtual links, pose priors and landmarks are not scan graph connectivity.
                    continue
                if kind not in (0, 1, 2, 3, 4, 6):
                    bad_links.append({'from': start, 'to': end, 'type': kind, 'reason': 'unsupported physical link type'})
                    continue
                if start not in ids or end not in ids:
                    dangling.append({'from': start, 'to': end, 'type': kind})
                    continue
                try:
                    _pose(blob)
                except ValueError as error:
                    bad_links.append({'from': start, 'to': end, 'type': kind, 'reason': str(error)})
                    continue
                if start != end:
                    edges.add(tuple(sorted((start, end))))
                    adjacency[start].add(end); adjacency[end].add(start)
            remaining, components = set(ids), []
            while remaining:
                pending, seen = [min(remaining)], set()
                while pending:
                    current = pending.pop()
                    if current not in seen:
                        seen.add(current); pending.extend(adjacency[current]-seen)
                remaining -= seen; components.append(sorted(seen))
            report['graph'] = {'components': components, 'component_count': len(components), 'unique_physical_edges': len(edges),
                               'links_by_type': types, 'ignored_virtual_prior_landmark_links': ignored,
                               'dangling_links': dangling, 'invalid_links': bad_links,
                               'status': 'PASS' if nodes and not invalid and not dangling and not bad_links and
                                         (not candidate.require_single_connected_component or len(components) == 1) else 'FAIL'}
            for identifier, _stamp, _pose_blob, weight, _map_id in rows:
                sizes = db.execute('SELECT length(scan),length(scan_info) FROM Data WHERE id=?', (identifier,)).fetchone()
                if sizes is None:
                    scan = {'status': 'FAIL', 'scan_present': False, 'reasons': ['DATA_ROW_MISSING']}
                elif (sizes[0] or 0)>candidate.max_uncompressed_scan_bytes or (sizes[1] or 0)>76:
                    scan = {'status': 'UNKNOWN', 'scan_present': bool(sizes[0]), 'compressed_bytes': sizes[0],
                            'reasons': ['SCAN_INSPECTION_BUDGET_OR_UNSUPPORTED_METADATA_SIZE']}
                else:
                    data = db.execute('SELECT scan,scan_info FROM Data WHERE id=?', (identifier,)).fetchone()
                    scan = _scan(*data, candidate)
                scan.update(node_id=identifier, node_weight=weight,
                            required_for_moving_qualification=weight != -1)
                # RTAB intermediate weight=-1 nodes may intentionally lack data;
                # still inspect and report each one, never claim its scan passed.
                report['per_node_scans'].append(scan)
            required_scans = [row for row in report['per_node_scans'] if row['required_for_moving_qualification']]
            report['scan_summary'] = {'nodes_inspected': len(report['per_node_scans']),
                                      'required_data_nodes': len(required_scans),
                                      'intermediate_nodes_exempt_from_scan_gate': len(rows)-len(required_scans),
                                      'pass': sum(row['status'] == 'PASS' for row in required_scans),
                                      'fail': sum(row['status'] == 'FAIL' for row in required_scans),
                                      'unknown': sum(row['status'] == 'UNKNOWN' for row in required_scans)}
            reasons = []
            if len(required_scans) < candidate.min_nodes: reasons.append('INSUFFICIENT_DATA_NODES')
            if report['motion']['path_length_m'] < candidate.min_path_length_m: reasons.append('INSUFFICIENT_STORED_PATH_LENGTH')
            if report['motion']['translation_extent_m'] < candidate.min_translation_extent_m: reasons.append('INSUFFICIENT_STORED_TRANSLATION_EXTENT')
            if report['graph']['status'] != 'PASS': reasons.append('GRAPH_NOT_CONNECTED_OR_INVALID')
            if not required_scans or any(row['status'] != 'PASS' for row in required_scans): reasons.append('REQUIRED_NODE_SCAN_CHECK_NOT_PASSED')
            report['movement_map_qualification'] = {'status': 'QUALIFIED_EXPERIMENTAL_CANDIDATE' if not reasons else 'NOT_QUALIFIED',
                'reasons': reasons, 'is_absolute_accuracy_acceptance': False,
                'static_map_save_allowed_if_file_and_export_integrity_pass': True}
    except (sqlite3.Error, ValueError, OSError, OverflowError) as error:
        report['inspection_error'] = type(error).__name__+': '+str(error)
        if report['file_integrity']['status'] == 'UNKNOWN':
            report['file_integrity'] = {'status': 'FAIL', 'reason': str(error)}
    finally:
        report['database_sha256_after'] = _digest(database)
        report['database_modified'] = report['database_sha256_after'] != checksum
        if report['database_modified']:
            report['file_integrity'] = {'status': 'FAIL', 'reason': 'DATABASE_CHANGED_DURING_INSPECTION'}
            report['movement_map_qualification'] = {'status': 'UNKNOWN', 'reasons': ['DATABASE_CHANGED_DURING_INSPECTION']}
    return report
