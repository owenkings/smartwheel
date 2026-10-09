#!/usr/bin/env python3
"""Read-only statistics for a CLOSED single-lidar experiment; stdout JSON only.

Example: python3 -B tests/diagnostics/check_single_mapping_session.py \
    --session-root reports/single_mapping/single_right_static_20260914_01 --stationary

Uses only the named session's known artifacts, without ROS, CDR deserialization,
hardware access, map modification, or automatic success/acceptance decisions.
"""
import argparse
from contextlib import contextmanager
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import sqlite3
import sys


def require(condition, message):
    if not condition:
        raise ValueError(message)


def safe_path(path):
    path = Path(path).absolute()
    require('..' not in path.parts, 'Parent traversal is not allowed')
    require(not any(p.is_symlink() or (hasattr(p, 'is_junction') and p.is_junction())
                    for p in (path, *path.parents)), 'Links/junctions are not allowed: ' + str(path))
    return path


def finite_vector(value, size):
    require(isinstance(value, list) and len(value) == size and
            all(type(x) in (int, float) and math.isfinite(x) for x in value),
            'Expected finite numeric vector of length ' + str(size))
    return value


def integer(value, label):
    require(type(value) is int and value >= 0, label + ' must be a nonnegative integer')
    return value


class Reader:
    def __init__(self, root):
        self.root = safe_path(root)
        require(self.root.is_dir(), 'Session directory does not exist')
        self.read_files = []
        self.missing_files = []

    def file(self, relative, limit=None):
        path = safe_path(self.root / relative)
        require(path.is_relative_to(self.root), 'File escaped session directory')
        if not path.exists():
            self.missing_files.append(relative)
            return None
        require(path.is_file(), 'Expected regular file: ' + relative)
        require(limit is None or path.stat().st_size <= limit, 'Artifact exceeds read bound: ' + relative)
        self.read_files.append(relative)
        return path

    def json(self, relative):
        path = self.file(relative, 8_000_000)
        if path is None:
            return None
        value = json.loads(path.read_text(encoding='utf-8'))
        require(isinstance(value, dict), 'Expected JSON object: ' + relative)
        return value

    def jsonl(self, relative):
        path = self.file(relative, 128_000_000)
        rows = []
        if path is not None:
            with path.open(encoding='utf-8') as stream:
                for line in stream:
                    if not line.strip():
                        continue
                    require(len(line) <= 1_000_000 and len(rows) < 100_000,
                            'JSONL row/read bound exceeded: ' + relative)
                    value = json.loads(line)
                    require(isinstance(value, dict), 'Expected JSONL object: ' + relative)
                    rows.append(value)
        return rows


@contextmanager
def closed_sqlite(path):
    """Immutable read-only URI avoids creating locks/sidecars in evidence folders."""
    def snapshot():
        for suffix in ('-wal', '-journal'):
            require(not safe_path(Path(str(path) + suffix)).exists(),
                    'Database has journal/WAL; close it before diagnosis: ' + str(path))
        stat = path.stat()
        return stat.st_size, stat.st_mtime_ns
    before = snapshot()
    connection = sqlite3.connect(path.as_uri() + '?mode=ro&immutable=1', uri=True)
    try:
        connection.execute('PRAGMA query_only=ON')
        yield connection
    finally:
        connection.close()
    require(snapshot() == before, 'Database changed during diagnosis: ' + str(path))


def bag_counts(reader, side):
    directory = safe_path(reader.root / 'bag')
    if not directory.exists():
        reader.missing_files.append('bag/*.db3')
        return None
    require(directory.is_dir(), 'bag must be a directory')
    paths = sorted(directory.glob('*.db3'))
    require(len(paths) <= 128, 'Bag split count exceeds diagnostic bound')
    if not paths:
        reader.missing_files.append('bag/*.db3')
        return None
    topics = {}
    for candidate in paths:
        path = reader.file('bag/' + candidate.name)
        with closed_sqlite(path) as connection:
            rows = connection.execute('SELECT t.name, t.type, COUNT(m.id), MIN(m.timestamp), MAX(m.timestamp) '
                                      'FROM topics t LEFT JOIN messages m ON m.topic_id=t.id '
                                      'GROUP BY t.id').fetchall()
        for name, kind, count, first, last in rows:
            item = topics.setdefault(name, {'type': kind, 'messages': 0,
                                            'first_bag_timestamp_ns': None, 'last_bag_timestamp_ns': None})
            require(item['type'] == kind, 'Bag topic type changed between splits: ' + name)
            item['messages'] += count
            if count:
                old_first, old_last = item['first_bag_timestamp_ns'], item['last_bag_timestamp_ns']
                item['first_bag_timestamp_ns'] = first if old_first is None else min(first, old_first)
                item['last_bag_timestamp_ns'] = last if old_last is None else max(last, old_last)
    source = '/wc_mapping/lidar_' + side
    return {'sqlite_files': len(paths), 'topics': topics,
            'raw_source_messages': topics.get(source + '/source_frame', {}).get('messages'),
            'filtered_source_messages': topics.get(source + '/source_frame_filtered', {}).get('messages'),
            'note': 'Counts are recorded ROS messages, not UDP packets or proof of loss-free acquisition; '
                    'bag timestamps are recording timestamps, not validated measurement synchronization.'}


def publication_statistics(status, rows):
    keys = ('received_frames', 'validated_frames', 'throttled_frames', 'archived_frames',
            'published_frames', 'sequence_gaps')
    result = {key: integer(status[key], key) for key in keys}
    source_times, guard_times, sequences = [], [], []
    for row in rows:
        require(row.get('side') == status['side'] and row.get('sensor_id') == status['sensor_id'],
                'Archive index sensor identity mismatch')
        require(row.get('raw_key', [None])[0] == status['session_id'], 'Archive index session mismatch')
        source_times.append(integer(row['host_monotonic_ns'], 'host_monotonic_ns'))
        guard_times.append(integer(row['guard_receive_monotonic_ns'], 'guard_receive_monotonic_ns'))
        sequences.append(integer(row['frame_sequence'], 'frame_sequence'))
    for values in (source_times, guard_times, sequences):
        require(all(a < b for a, b in zip(values, values[1:])), 'Archive index order is not strictly increasing')
    source_span = (source_times[-1] - source_times[0]) / 1e9 if len(rows) > 1 else None
    guard_span = (guard_times[-1] - guard_times[0]) / 1e9 if len(rows) > 1 else None
    exact_count_match = len(rows) == result['archived_frames'] == result['published_frames']
    rate_ready = exact_count_match and len(rows) > 1
    result.update(configured_limit_hz=status.get('rate_hz'), archived_index_rows=len(rows),
                  archive_and_publication_counts_match=exact_count_match,
                  archived_first_to_last_source_host_span_s=source_span,
                  archived_first_to_last_guard_arrival_span_s=guard_span,
                  publication_rate_hz=(len(rows)-1)/source_span if rate_ready else None,
                  publication_rate_by_guard_arrival_hz=(len(rows)-1)/guard_span if rate_ready else None,
                  archived_sequence_skips_including_intentional_throttle=
                  sum(b-a-1 for a, b in zip(sequences, sequences[1:])),
                  rate_method='(published_frames-1)/(last-first archived host monotonic timestamp); '
                              'only available when every archived frame is accounted for as published',
                  rate_limitation='Host-arrival-span estimate, not a measured publish-call clock rate or sensor specification.',
                  sequence_gap_interpretation='sequence_gaps counts discontinuities observed at the guard before throttle. '
                      'Recorded-message/guard-count differences and sequence gaps do not localize a port or hardware fault; '
                      'startup/shutdown windows, DDS delivery and scheduling need separate evidence.')
    return result


def translation_statistics(rows):
    if not rows:
        return {'samples': 0, 'max_from_start_m': None, 'final_from_start_m': None}
    start = rows[0]['xyz']
    deltas = [[p-c for p, c in zip(row['xyz'], start)] for row in rows]
    distances = [math.sqrt(sum(x*x for x in delta)) for delta in deltas]
    return {'samples': len(rows), 'start_xyz_m': start, 'final_xyz_m': rows[-1]['xyz'],
            'final_delta_xyz_in_odometry_frame_m': deltas[-1],
            'max_from_start_m': max(distances), 'final_from_start_m': distances[-1],
            'source_stamp_span_s': (rows[-1]['source_stamp_ns']-rows[0]['source_stamp_ns']) / 1e9}


def orientation_statistics(rows):
    if not rows:
        return {'samples': 0, 'max_relative_rotation_deg': None, 'final_relative_rotation_deg': None}
    quaternions = []
    for row in rows:
        quaternion = finite_vector(row['orientation_xyzw'], 4)
        norm = math.sqrt(sum(x*x for x in quaternion))
        require(abs(norm-1) <= .01, 'Odometry quaternion is not unit length')
        quaternions.append([x/norm for x in quaternion])
    angles = [math.degrees(2*math.acos(min(1., abs(sum(a*b for a, b in zip(quaternions[0], q))))))
              for q in quaternions]
    return {'samples': len(rows), 'max_relative_rotation_deg': max(angles),
            'final_relative_rotation_deg': angles[-1],
            'reference_source_stamp_ns': rows[0]['source_stamp_ns'],
            'final_source_stamp_ns': rows[-1]['source_stamp_ns'],
            'method': 'Shortest relative quaternion angle; q and -q describe the same orientation.'}


def pose_statistics(trajectory, odometry, side, stationary):
    by_segment, by_stamp = {}, {}
    previous = -1
    for row in trajectory:
        finite_vector(row['xyz'], 3)
        stamp = integer(row['source_stamp_ns'], 'trajectory source stamp')
        segment = integer(row['segment'], 'trajectory segment')
        require(stamp > previous, 'Trajectory source stamps are not strictly increasing')
        previous = stamp
        by_segment.setdefault(segment, []).append(row)
        by_stamp[stamp] = row
    orientations = {}
    for row in odometry:
        stamp = row.get('source_stamp_ns')
        if stamp not in by_stamp:
            continue
        require(row.get('frame_id') == 'single_' + side + '_odom' and row.get('child_frame_id') == 'lidar_' + side,
                'Odometry frame mismatch')
        require(row.get('pose_valid') is True, 'Confirmed trajectory has invalid odometry pose')
        require(stamp not in orientations, 'Duplicate matched odometry source stamp')
        require(finite_vector(row['position'], 3) == by_stamp[stamp]['xyz'], 'Trajectory/odometry position mismatch')
        orientations[stamp] = row
    matched = [orientations[stamp] for stamp in by_stamp if stamp in orientations]
    return {'frame_id': 'single_' + side + '_odom', 'stationary_declared_by_operator': stationary,
            'interpretation': 'Static relative-pose drift observation; no ground-truth accuracy established.' if stationary else
                              'Relative pose changes include intentional motion; these values are not accuracy or drift measurements.',
            'continuous_single_segment': len(by_segment) == 1,
            'translation_relative_to_first_sample': translation_statistics(trajectory),
            'orientation_relative_to_first_matched_sample': orientation_statistics(matched),
            'orientation_missing_for_confirmed_trajectory_samples': len(trajectory)-len(matched),
            'segments': {str(segment): {'translation': translation_statistics(rows),
                'orientation': orientation_statistics([orientations[r['source_stamp_ns']] for r in rows
                                                       if r['source_stamp_ns'] in orientations])}
                         for segment, rows in by_segment.items()},
            'continuity_note': 'Multiple segments follow tracking loss. Cross-segment numeric offsets do not establish '
                               'a continuous physical route. XYZ axes are not a validated horizontal reference.'}


def export_statistics(reader, exported, side):
    result = {'declared_status': exported.get('status') if exported else None,
              'declared_retained_nodes': exported.get('retained_nodes') if exported else None}
    path = reader.file('slam/rtabmap.db')
    if path is not None:
        with closed_sqlite(path) as connection:
            result['native_database_node_rows'] = connection.execute('SELECT COUNT(*) FROM Node').fetchone()[0]
    pose_name = 'single_' + side + '_map_poses.txt'
    path = reader.file('export/' + pose_name, 64_000_000)
    if path is not None:
        with path.open(encoding='utf-8') as stream:
            result['exported_pose_nonempty_noncomment_lines'] = sum(1 for line in stream
                if line.strip() and not line.lstrip().startswith('#'))
        result['pose_file'] = 'export/' + pose_name
        result['pose_file_listed_in_export_result'] = bool(exported and pose_name in exported.get('files', {}))
    result['interpretation'] = ('Node table rows are retained database records, not a count of spatially distinct keyframes. '
        'The optimized exported pose set can be smaller; a static run may contain many Node rows and one exported pose. '
        'No dynamic-route completion or geometric acceptance follows from either count. Pose rows are counted, not parsed.')
    return result


def diagnose(root, stationary=False):
    reader = Reader(root)
    status = reader.json('input/status.json')
    require(status is not None, 'input/status.json is required to identify the session')
    require(status.get('kind') == 'SINGLE_LIDAR_EXPERIMENT' and status.get('source_mode') == 'real',
            'Not an identified real single-lidar experiment')
    side, session = status.get('side'), status.get('session_id')
    require(side in ('left', 'right') and isinstance(session, str) and bool(session), 'Invalid session identity')
    require(status.get('state') in ('STOPPED', 'FAILED'), 'Input is not closed; diagnose after session cleanup')
    monitor = reader.json('monitor/status.json')
    exported = reader.json('export/result.json')
    for label, value in (('monitor', monitor), ('export', exported)):
        if value is not None:
            require(value.get('session_id') == session, label + ' session identity mismatch')
            require(value.get('sensor_mode') == 'single_' + side, label + ' sensor mode mismatch')
    if monitor is not None:
        require(monitor.get('status') in ('STOPPED', 'FAILED'), 'Monitor is not closed')
    inputs = publication_statistics(status, reader.jsonl('input/frames.jsonl'))
    bag = bag_counts(reader, side)
    if bag and bag['filtered_source_messages'] is not None:
        inputs['bag_filtered_minus_guard_received'] = bag['filtered_source_messages']-inputs['received_frames']
    pose = pose_statistics(reader.jsonl('monitor/trajectory.jsonl'), reader.jsonl('monitor/odometry.jsonl'), side, stationary)
    export = export_statistics(reader, exported, side)
    return {'schema_version': 1, 'status': 'DIAGNOSTIC_COMPLETE_WITH_MISSING_ARTIFACTS' if reader.missing_files else 'DIAGNOSTIC_COMPLETE',
            'generated_utc': datetime.now(timezone.utc).isoformat(), 'session_id': session,
            'session_root': str(reader.root), 'sensor_mode': 'single_' + side,
            'experiment': 'SINGLE_LIDAR_EXPERIMENT', 'formal_acceptance': False,
            'input_final_state': status['state'], 'input_failure': status.get('failure_reason'),
            'monitor': {key: monitor.get(key) for key in ('status', 'counts', 'total_lost', 'failure', 'elapsed_s')}
                       if monitor else None,
            'bag': bag, 'input': inputs, 'relative_pose': pose, 'export': export,
            'read_files': reader.read_files, 'missing_files': reader.missing_files,
            'known_limitations': [
                'Read-only closed-file statistics. DIAGNOSTIC_COMPLETE means the script completed, not that the map passed.',
                'SourceFrame CDR contents and source sequence numbers in bag payloads are not independently decoded here.',
                'Host arrival_only time is not validated measurement synchronization; no IMU gravity or wheel constraints.',
                'No left-right/IMU extrinsic, ground reference, metric accuracy, dynamic-route or navigation validation.',
                'No hardware-port diagnosis can be inferred from a message-count difference or guard sequence gap alone.',
                'No map point clouds are changed or re-exported; the export status is reported, not independently revalidated.',
                'Stationary is an operator assertion. Without it, relative pose changes include real intended motion.']}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session-root', required=True, type=Path)
    parser.add_argument('--stationary', action='store_true', help='Operator declares the full capture was stationary')
    args = parser.parse_args(argv)
    try:
        report = diagnose(args.session_root, args.stationary)
        print(json.dumps(report, ensure_ascii=False, allow_nan=False, indent=2))
        return 0
    except (OSError, ValueError, TypeError, KeyError, IndexError, sqlite3.Error) as error:
        print(json.dumps({'status': 'DIAGNOSTIC_FAILED', 'error': type(error).__name__, 'reason': str(error)},
                         ensure_ascii=False, allow_nan=False, indent=2))
        return 2


if __name__ == '__main__':
    sys.exit(main())
