#!/usr/bin/env python3
"""Read-only raw/filtered pairing audit for a finalized SQLite rosbag2.

No ROS node, playback, sensor access, map or configuration write is performed.
The sole write is a new explicit JSON report. Host processing/dispatch elapsed
values are descriptive and are NOT measurement latency or a filter-quality test.
"""
from __future__ import annotations
import argparse
from collections import Counter, defaultdict
import configparser
import hashlib
import json
from pathlib import Path
import re
import sqlite3
import time
import numpy as np
import yaml

TYPE = 'wc_interfaces/msg/SourceFrame'
TOPICS = {f'/wc_mapping/lidar_{side}/{name}': (side, branch)
          for side in ('left', 'right')
          for name, branch in (('source_frame', 'raw'), ('source_frame_filtered', 'filtered'))}
EXPECTED = {'left': 'XTM60B00000000000012', 'right': 'XTM60B00000000000013'}
HASH = re.compile(r'[0-9a-f]{64}\Z')
TOKEN = re.compile(r'[A-Za-z0-9_.-]+\Z')


def digest(data):
    return hashlib.sha256(data).hexdigest()


def file_state(path):
    stat = path.stat()
    return {'size': stat.st_size, 'mtime_ns': stat.st_mtime_ns,
            'sha256': digest(path.read_bytes())}


def inside(path, parent):
    parent = Path(parent).resolve(strict=True)
    path = Path(path).absolute()
    resolved = path.resolve(strict=True)
    if not resolved.is_relative_to(parent):
        raise ValueError('evidence path outside project: '+str(path))
    while path != parent and path != path.parent:
        if path.is_symlink():
            raise ValueError('symlink evidence path refused: '+str(path))
        path = path.parent
    return resolved


def ns(stamp):
    sec, nano = int(stamp.sec), int(stamp.nanosec)
    if not 0 <= nano < 1_000_000_000:
        raise ValueError('noncanonical timestamp')
    return sec*1_000_000_000+nano


def stats(values):
    if not values:
        return {'count': 0}
    data = np.asarray(values, dtype=np.float64)
    return {'count': len(values), 'min': float(data.min()), 'max': float(data.max()),
            'mean': float(data.mean()), 'p50': float(np.percentile(data, 50)),
            'p95': float(np.percentile(data, 95))}


def flag_value(flags, name, required=True):
    values = [item[len(name)+1:] for item in flags if item.startswith(name+'=')]
    if len(values) != 1:
        if not required and not values:
            return None
        raise ValueError('missing/duplicate flag: '+name)
    return values[0]


def cloud_stats(cloud):
    width, height, step, row = map(int, (cloud.width, cloud.height, cloud.point_step, cloud.row_step))
    raw = bytes(cloud.data)
    if width < 1 or height < 1 or step < 12 or row < width*step or len(raw) != row*height:
        raise ValueError('malformed cloud dimensions/stride/storage')
    fields = {field.name: field for field in cloud.fields}
    if len(fields) != len(cloud.fields):
        raise ValueError('duplicate point fields')
    xyz = []
    for axis in ('x', 'y', 'z'):
        f = fields.get(axis)
        if f is None or int(f.datatype) != 7 or int(f.count) != 1 or not 0 <= int(f.offset) <= step-4:
            raise ValueError('XYZ must be float32 scalar fields')
        xyz.append(np.ndarray((height, width), dtype=('>' if cloud.is_bigendian else '<')+'f4',
                              buffer=raw, offset=int(f.offset), strides=(row, step)))
    finite = np.isfinite(xyz[0]) & np.isfinite(xyz[1]) & np.isfinite(xyz[2])
    return {'count': width*height, 'valid': int(finite.sum()), 'data_sha256': digest(raw)}


def acquisition(message):
    return {'header_frame': message.header.frame_id, 'header_stamp_ns': ns(message.header.stamp),
            'cloud_frame': message.cloud.header.frame_id, 'cloud_stamp_ns': ns(message.cloud.header.stamp),
            'host_receive_ns': ns(message.host_receive_time), 'host_monotonic_ns': int(message.host_monotonic_ns),
            **{name: getattr(message, name) for name in (
                'coordinate_convention', 'units', 'device_timestamp_valid', 'device_timestamp_raw',
                'device_timestamp_unit', 'device_time_components_valid', 'device_timestamp_seconds',
                'device_timestamp_nanoseconds', 'device_time_type', 'device_sync_state',
                'common_time_valid', 'common_time_ns', 'clock_model_id', 'time_source',
                'uncertainty_valid', 'uncertainty_ns')}}


class Audit:
    def __init__(self, project_root, run_root, config_dir, expected=None):
        self.root = Path(project_root).resolve(strict=True)
        self.run_root, self.config_dir = Path(run_root), Path(config_dir)
        self.expected = expected or EXPECTED
        self.messages = {'raw': {}, 'filtered': {}}
        self.issues, self.examples, self.seen = Counter(), defaultdict(list), Counter()
        self.configs, self.inputs, self.sides = {}, {}, {}
        for side in ('left', 'right'):
            self.sides[side] = {'counts': Counter(), 'raw_valid': [], 'filtered_valid': [],
                                'raw_total': [], 'filtered_total': [], 'elapsed_ns': [],
                                'config_statuses': Counter(), 'flags': Counter()}

    def issue(self, code, detail=''):
        self.issues[code] += 1
        if len(self.examples[code]) < 5:
            self.examples[code].append(str(detail))

    def input(self, path):
        path = inside(path, self.root)
        observed = file_state(path)
        if str(path) in self.inputs and self.inputs[str(path)] != observed:
            raise ValueError('input changed during audit: '+str(path))
        self.inputs[str(path)] = observed
        return path.read_bytes()

    def config(self, session, side, epoch):
        key = (session, side, epoch)
        if key in self.configs:
            return self.configs[key]
        directory = self.run_root/'sessions'/session/side/epoch
        text = self.input(directory/'device_readback.txt').decode('utf-8')
        values = {}
        for line in text.splitlines():
            name, sep, value = line.partition('=')
            if not sep or name in values:
                raise ValueError('malformed/duplicate run configuration record')
            values[name] = value
        prefix, sep, suffix = text.partition('effective_source_hash=')
        if not sep or suffix != values['effective_source_hash']+'\nfiltered_source_hash='+values['filtered_source_hash']+'\n':
            raise ValueError('source hash trailer differs from driver record format')
        raw_hash = digest((prefix+'representation=before_host_filter\n').encode())
        filtered_hash = digest((prefix+'representation=host_filtered\n').encode())
        if raw_hash != values['effective_source_hash'] or filtered_hash != values['filtered_source_hash']:
            raise ValueError('effective source hash cannot be recomputed from run configuration')
        source = self.input(self.config_dir/(side+'.xtcfg'))
        if digest(source) != values['requested_xtcfg_sha256']:
            raise ValueError('recorded requested hash differs from corresponding original xtcfg')
        parser = configparser.ConfigParser(interpolation=None, strict=True)
        parser.optionxform = str
        parser.read_string(source.decode('utf-8-sig'))
        dispositions = {}
        for section in parser.sections():
            for name, requested in parser[section].items():
                item = section+'.'+name
                actual, marker, disposition = values.get(item, '').partition(';disposition=')
                if not marker or actual != requested:
                    raise ValueError('requested field/disposition missing or changed: '+item)
                dispositions[item] = disposition
        before = self.input(directory/'device_before.txt')
        after = self.input(directory/'device_after.txt')
        commands = self.input(directory/'configuration_commands.txt').decode('utf-8')
        if digest(after) != values['actual_device_readback_sha256']:
            raise ValueError('actual device-after file hash differs from source configuration')
        if values['serial'] != self.expected[side] or ';' in values['serial']:
            raise ValueError('run configuration serial differs from requested physical source')
        if '=FAILED\n' in commands:
            raise ValueError('run configuration contains a failed imaging command')
        if 'persistent_save=NOT_REQUESTED\n' not in commands or 'network_clock_firmware_motor=DENIED\n' not in commands:
            raise ValueError('imaging command audit policy footer missing')
        result = {'session_id': session, 'side': side, 'stream_epoch': epoch,
                  'directory': str(directory), 'raw_config_hash': raw_hash,
                  'filtered_config_hash': filtered_hash, 'requested_xtcfg_sha256': digest(source),
                  'requested_field_count': len(dispositions), 'field_dispositions': dispositions,
                  'configuration_status': values.get('configuration_status'),
                  'SDK_filter_flags': values.get('SDK_filter_flags'),
                  'device_before_sha256': digest(before), 'device_after_sha256': digest(after),
                  'commands_sha256': digest(commands.encode()), 'validated_against_original_and_run_record': True}
        self.configs[key] = result
        return result

    def consume(self, topic, message, recorded_ns, cdr_bytes):
        side, branch = TOPICS[topic]
        self.seen[topic] += 1
        if message.side != side or message.sensor_id != self.expected[side]:
            raise ValueError('topic/side/expected device identity mismatch')
        for token in (message.session_id, message.stream_epoch, message.sensor_id):
            if not TOKEN.fullmatch(token) or token in ('.', '..'):
                raise ValueError('invalid identity token')
        key = (message.session_id, side, message.sensor_id, message.stream_epoch, int(message.frame_sequence))
        if key in self.messages[branch]:
            self.issue('duplicate_'+branch+'_key', key)
            if self.messages[branch][key]['cdr_sha256'] != digest(cdr_bytes):
                self.issue('conflicting_duplicate_'+branch+'_key', key)
            return
        view = acquisition(message)
        if view['header_frame'] != view['cloud_frame'] or view['header_stamp_ns'] != view['cloud_stamp_ns']:
            raise ValueError('cloud and envelope frame or stamp mismatch')
        if view['coordinate_convention'] != 'FLU' or view['units'] != 'm':
            raise ValueError('unexpected geometry units or convention')
        if view['time_source'] == 'synthetic':
            raise ValueError('synthetic source in requested real acquisition bag')
        if view['time_source'] == 'arrival_only' and (view['common_time_valid'] or view['uncertainty_valid'] or view['host_receive_ns'] != view['header_stamp_ns']):
            raise ValueError('arrival-only time claim inconsistent')
        cloud = cloud_stats(message.cloud)
        if cloud['count'] != int(message.raw_count) or cloud['valid'] != int(message.valid_count):
            raise ValueError('declared cloud count differs from actual XYZ')
        config = self.config(message.session_id, side, message.stream_epoch)
        expected_hash = config['raw_config_hash' if branch == 'raw' else 'filtered_config_hash']
        if message.source_config_hash != expected_hash:
            raise ValueError('message config hash differs from matching run log')
        flags = list(message.diagnostic_flags)
        representation = flag_value(flags, 'representation')
        if representation != ('before_host_filter' if branch == 'raw' else 'host_filtered'):
            raise ValueError('wrong raw/processed representation flag')
        elapsed = flag_value(flags, 'host_filter_and_dispatch_elapsed_ns')
        if not elapsed.isdigit():
            raise ValueError('processing/dispatch elapsed must be nonnegative integer')
        descriptor = {'acquisition': view, 'config_hash': message.source_config_hash,
                      'cloud': cloud, 'cdr_sha256': digest(cdr_bytes), 'bag_recorded_ns': int(recorded_ns),
                      'elapsed_ns': int(elapsed)}
        if branch == 'filtered':
            descriptor['raw_config_hash'] = flag_value(flags, 'raw_source_config_hash')
            descriptor['before_cloud_hash'] = flag_value(flags, 'before_host_filter_cloud_sha256')
            if not HASH.fullmatch(descriptor['before_cloud_hash']):
                raise ValueError('malformed before-host-filter content hash')
        self.messages[branch][key] = descriptor
        info = self.sides[side]
        info['counts'][branch+'_frames'] += 1
        info[branch+'_valid'].append(cloud['valid'])
        info[branch+'_total'].append(cloud['count'])
        info['flags'].update(flags)
        status = flag_value(flags, 'configuration_status')
        info['config_statuses'][status] += 1
        if status != config['configuration_status']:
            self.issue('message_configuration_status_differs_from_run_log', key)

    def finish(self):
        raw_keys, filtered_keys = set(self.messages['raw']), set(self.messages['filtered'])
        for key in raw_keys-filtered_keys:
            self.sides[key[1]]['counts']['raw_without_filtered'] += 1
        for key in filtered_keys-raw_keys:
            self.sides[key[1]]['counts']['filtered_without_raw'] += 1
        for key in raw_keys & filtered_keys:
            raw, filtered = self.messages['raw'][key], self.messages['filtered'][key]
            info = self.sides[key[1]]
            info['counts']['matched_identity_frames'] += 1
            faults = []
            if raw['acquisition'] != filtered['acquisition']:
                faults.append('pair_acquisition_metadata_mismatch')
            if raw['elapsed_ns'] != filtered['elapsed_ns']:
                faults.append('pair_host_elapsed_metadata_mismatch')
            if raw['config_hash'] != filtered['raw_config_hash']:
                faults.append('pair_raw_source_config_hash_mismatch')
            if raw['cloud']['data_sha256'] != filtered['before_cloud_hash']:
                faults.append('pair_raw_cloud_hash_mismatch')
            for fault in faults:
                self.issue(fault, key)
            if not faults:
                info['counts']['verified_raw_filtered_pairs'] += 1
            info['elapsed_ns'].append(filtered['elapsed_ns'])
            if raw['cloud']['data_sha256'] == filtered['cloud']['data_sha256']:
                info['counts']['equal_cloud_payloads'] += 1
            else:
                info['counts']['different_cloud_payloads'] += 1
        for topic in TOPICS:
            if not self.seen[topic]:
                self.issue('required_topic_absent', topic)
        sides = {}
        for side, info in self.sides.items():
            if info['counts']['raw_without_filtered'] or info['counts']['filtered_without_raw']:
                self.issue('unpaired_recorded_frames', side)
            sides[side] = {'counts': {name: info['counts'][name] for name in (
                    'raw_frames', 'filtered_frames', 'matched_identity_frames', 'verified_raw_filtered_pairs',
                    'raw_without_filtered', 'filtered_without_raw', 'equal_cloud_payloads', 'different_cloud_payloads')},
                'raw_total_points_per_frame': stats(info['raw_total']),
                'raw_valid_points_per_frame': stats(info['raw_valid']),
                'filtered_total_points_per_frame': stats(info['filtered_total']),
                'filtered_valid_points_per_frame': stats(info['filtered_valid']),
                'host_filter_and_dispatch_elapsed_ns': stats(info['elapsed_ns']),
                'configuration_status_counts': dict(info['config_statuses']),
                'diagnostic_flag_counts': dict(info['flags'])}
        return {'sides': sides, 'observed_topics': dict(self.seen),
                'configuration_records': list(self.configs.values()),
                'issues': dict(self.issues), 'issue_examples': dict(self.examples)}


def audit_bag(bag, project_root, run_root=None, config_dir=None, max_envelopes=20000, decoder=None):
    root = Path(project_root).resolve(strict=True)
    bag = inside(bag, root/'data/bags')
    review = Audit(root, run_root or root/'.phase1_runtime', config_dir or root/'src/wc_xt_driver/config')
    metadata = review.input(bag/'metadata.yaml')
    info = yaml.safe_load(metadata)['rosbag2_bagfile_information']
    if info.get('storage_identifier') != 'sqlite3' or info.get('compression_format') or info.get('compression_mode') not in (None, '', 'none'):
        raise ValueError('only uncompressed finalized SQLite bags supported')
    relative = info.get('relative_file_paths', [])
    if not relative or len(set(relative)) != len(relative):
        raise ValueError('missing/duplicate finalized database list')
    databases = []
    for name in relative:
        rel = Path(name)
        if rel.is_absolute() or '..' in rel.parts:
            raise ValueError('unsafe database path in metadata')
        path = inside(bag/rel, bag)
        if path.suffix != '.db3':
            raise ValueError('non-db3 metadata entry')
        for suffix in ('-wal', '-shm', '-journal'):
            sidecar = Path(str(path)+suffix)
            if sidecar.exists() and sidecar.stat().st_size:
                raise ValueError('active SQLite sidecar; stop recorder first')
        review.inputs[str(path)] = file_state(path)
        databases.append(path)
    if set(bag.glob('*.db3')) != set(databases):
        raise ValueError('bag database files differ from finalized metadata')
    decode_kind = 'TEST_DECODER' if decoder is not None else 'ROS2_CDR'
    if decoder is None:
        from rclpy.serialization import deserialize_message
        from wc_interfaces.msg import SourceFrame
        decoder = lambda blob: deserialize_message(blob, SourceFrame)
    totals, types, db_reports = Counter(), {}, []
    for database in databases:
        conn = sqlite3.connect(database.as_uri()+'?mode=ro&immutable=1', uri=True)
        try:
            conn.execute('PRAGMA query_only=ON')
            if [r[0] for r in conn.execute('PRAGMA quick_check')] != ['ok']:
                raise ValueError('SQLite integrity check failed')
            topics = {row[0]: row[1:] for row in conn.execute('SELECT id,name,type,serialization_format FROM topics')}
            count = 0
            for tid, amount in conn.execute('SELECT topic_id,COUNT(*) FROM messages GROUP BY topic_id'):
                if tid not in topics:
                    raise ValueError('unknown topic id referenced by a message')
                totals[topics[tid][0]] += amount
                count += amount
            if sum(totals[t] for t in TOPICS) > max_envelopes:
                raise ValueError('explicit envelope memory bound exceeded')
            ids = []
            for tid, (name, typ, fmt) in topics.items():
                if name in types and types[name] != (typ, fmt):
                    raise ValueError('topic type/serialization changed between splits')
                types[name] = (typ, fmt)
                if name in TOPICS:
                    if typ != TYPE or fmt != 'cdr':
                        raise ValueError('SourceFrame topic type/serialization mismatch')
                    ids.append(tid)
            if ids:
                sql = 'SELECT id,topic_id,timestamp,data FROM messages WHERE topic_id IN ('+','.join('?' for _ in ids)+') ORDER BY id'
                for row_id, tid, stamp, blob in conn.execute(sql, ids):
                    try:
                        review.consume(topics[tid][0], decoder(blob), stamp, blob)
                    except (ValueError, KeyError, TypeError, AttributeError, RuntimeError, IndexError) as error:
                        review.issue('envelope_or_configuration_validation_failed', str(database.name)+':'+str(row_id)+': '+str(error))
            db_reports.append({'path': str(database), 'messages': count, 'quick_check': 'ok'})
        finally:
            conn.close()
    if sum(totals.values()) != int(info.get('message_count', -1)):
        review.issue('metadata_message_count_mismatch')
    metadata_topics = {}
    for entry in info.get('topics_with_message_count', []):
        item = entry['topic_metadata'];name = item['name']
        if name in metadata_topics:
            review.issue('metadata_duplicate_topic', name)
        metadata_topics[name] = (item['type'], item.get('serialization_format'), int(entry['message_count']))
    for name in set(metadata_topics) | set(totals):
        expected = metadata_topics.get(name)
        if expected is None or expected[2] != totals[name] or (name in types and expected[:2] != types[name]):
            review.issue('metadata_topic_count_or_type_mismatch', name)
    unchanged = all(file_state(Path(path)) == state for path, state in review.inputs.items())
    unchanged = unchanged and set(bag.glob('*.db3')) == set(databases)
    for path in databases:
        for suffix in ('-wal', '-shm', '-journal'):
            sidecar = Path(str(path)+suffix)
            if sidecar.exists() and sidecar.stat().st_size:
                unchanged = False
    if not unchanged:
        review.issue('inputs_changed_during_read')
    result = review.finish()
    result.update(schema_version=1, audit_kind='RECORDED_RAW_FILTERED_PAIRING_ONLY',
        status='PAIRING_REVIEW_OK' if not review.issues else 'PAIRING_REVIEW_ISSUES',
        bag=str(bag), generated_ns=time.time_ns(), decoding=decode_kind,
        storage={'read_only_immutable': True, 'finalized_metadata_present': True,
                 'files_unchanged': unchanged, 'files': review.inputs,
                 'actual_topic_counts': dict(totals), 'databases': db_reports,
                 'graceful_recorder_exit_independently_proven': False},
        acceptance={'filter_quality': 'NOT_EVALUATED', 'measurement_latency': 'NOT_MEASURED',
                    'common_sample_synchronization': 'NOT_ESTABLISHED',
                    'real_static_map': 'NOT_TESTED', 'dynamic_map': 'NOT_TESTED',
                    'note': 'Host elapsed includes processing and callback dispatch; counts/hash correspondence do not establish point accuracy, temporal response, physical stillness or map quality.'})
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--project-root', type=Path, default=Path('/home/nvidia/wheelchair'))
    parser.add_argument('--run-root', type=Path)
    parser.add_argument('--config-dir', type=Path)
    parser.add_argument('--max-envelopes', type=int, default=20000)
    args = parser.parse_args(argv)
    if args.max_envelopes < 1:
        parser.error('max-envelopes must be positive')
    root = args.project_root.resolve(strict=True)
    output = args.output.absolute()
    inside(output.parent, root)
    if output.exists() or output.is_symlink() or output.resolve().is_relative_to((root/'data/bags').resolve()):
        parser.error('output must be a new report outside the input bag area')
    try:
        report = audit_bag(args.bag, root, args.run_root, args.config_dir, args.max_envelopes)
    except Exception as error:
        report = {'schema_version': 1, 'audit_kind': 'RECORDED_RAW_FILTERED_PAIRING_ONLY',
                  'status': 'INCOMPLETE', 'error': type(error).__name__+': '+str(error),
                  'acceptance': {'filter_quality': 'NOT_EVALUATED', 'measurement_latency': 'NOT_MEASURED'}}
    with output.open('x', encoding='utf-8') as stream:
        json.dump(report, stream, indent=2, ensure_ascii=False, allow_nan=False)
        stream.write('\n')
    print(json.dumps({'status': report['status'], 'output': str(output), 'issues': report.get('issues', {}),
                      'error': report.get('error')}, ensure_ascii=False))
    return 0 if report['status'] == 'PAIRING_REVIEW_OK' else 2 if report['status'] == 'INCOMPLETE' else 1


if __name__ == '__main__':
    raise SystemExit(main())
