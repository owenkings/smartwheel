"""Real SQLite source selection through dataset loading and the replay index.

ROS deserialization alone is replaced by a JSON fixture codec on developer
machines; source selection, identity validation, hashing and indexing use the
production implementations. No nodes, devices or fusion workers are started.
"""
import hashlib
import json
from pathlib import Path
import sqlite3
import sys
from types import ModuleType, SimpleNamespace

import pytest

from wc_runtime import mapping_compare, mapping_refine
from wc_runtime.compare_replay import RecordedPackets


def namespace(data):
    return json.loads(data, object_hook=lambda row: SimpleNamespace(**row))


@pytest.fixture
def source(tmp_path, monkeypatch):
    serialization = ModuleType('rclpy.serialization')
    serialization.deserialize_message = lambda data, kind: namespace(data)
    utilities = ModuleType('rosidl_runtime_py.utilities')
    utilities.get_message = lambda kind: kind
    monkeypatch.setitem(sys.modules, 'rclpy.serialization', serialization)
    monkeypatch.setitem(sys.modules, 'rosidl_runtime_py.utilities', utilities)
    source = tmp_path/'original'; (source/'bag').mkdir(parents=True)
    stamp = {'sec': 100, 'nanosec': 0}
    with sqlite3.connect(source/'bag/source.db3') as db:
        db.execute('CREATE TABLE topics(id INTEGER PRIMARY KEY,name TEXT,type TEXT)')
        db.execute('CREATE TABLE messages(id INTEGER PRIMARY KEY,topic_id INTEGER,timestamp INTEGER,data BLOB)')
        streams = [('/wc_mapping/wheel/feedback_raw', 'std_msgs/msg/String', 'wheel')]
        streams += [('/wc_mapping/imu/source_frame', 'wc_interfaces/msg/H30Frame', 'imu')]
        streams += [('/wc_mapping/lidar_'+side+'/source_frame'+suffix, 'wc_interfaces/msg/SourceFrame', side)
                    for side in ('left', 'right') for suffix in ('', '_filtered')]
        for topic_id, (name, kind, side) in enumerate(streams, 1):
            db.execute('INSERT INTO topics VALUES(?,?,?)', (topic_id, name, kind))
            if side == 'wheel':
                msg = {'data': json.dumps({'stamp_ns': 10**11, 'receive_monotonic_ns': 1000,
                       'sequence': 1, 'device_id': 'recorded-wheel', 'session_id': source.name})}
            else:
                msg = {'side': side if side != 'imu' else '', 'sensor_id': 'recorded-'+side,
                       'session_id': source.name, 'host_receive_time': stamp, 'host_monotonic_ns': 1000+topic_id,
                       'frame_sequence': 1}
            db.execute('INSERT INTO messages VALUES(?,?,?,?)',
                       (topic_id, topic_id, 10**11+topic_id, json.dumps(msg).encode()))
    db.close()
    eye = [[1., 0., 0.], [0., 1., 0.], [0., 0., 1.]]
    runtime = {'mode': 'all', 'mount_model': 'fixed_hardware_setup_v1',
               'mounts': {side: {'R_axle_lidar': eye, 't_axle_lidar_m': [0., offset, 0.]}
                          for side, offset in (('left', .2), ('right', -.2))},
               'imu_mount': {'R_axle_imu': eye}, 'wheel_candidate': {'wheel_radius_m': .185},
               'wheel_imu_covariance': {}, 'max_pair_delta_ns': 50_000_000, 'input_rate_hz': 5.}
    (source/'runtime_config.json').write_text(json.dumps(runtime), encoding='utf-8')
    return source


@pytest.mark.parametrize('mode,cloud', [(mode, cloud) for mode in ('all', 'left', 'right') for cloud in ('raw', 'filtered')])
def test_selected_lidar_alone_enters_the_replay_stream_without_losing_wheel_or_imu(source, mode, cloud):
    files = [path for path in source.rglob('*') if path.is_file()]
    before = {str(path): hashlib.sha256(path.read_bytes()).hexdigest() for path in files}
    runtime, manifest, hashes = mapping_compare.load_dataset(source, sides=mode, cloud=cloud)
    assert runtime['mode'] == mode and manifest is None and hashes == before
    assert runtime['prior_template']['require_dual_time_offsets'] == (mode == 'all')
    assert runtime['offline_source_selection']['single_lidar_diagnostic'] == (mode != 'all')
    expected = ['left', 'right'] if mode == 'all' else [mode]
    packets = RecordedPackets(source, runtime)
    try:
        sources = [row for row in packets.events if row['category'] == 'source']
        assert sorted(row['side'] for row in sources) == expected
        assert all(row['topic'].endswith('_filtered') == (cloud == 'filtered') for row in sources)
        assert sorted(row['category'] for row in packets.events if row['category'] != 'source') == ['imu', 'wheel']
        assert runtime['offline_source_selection']['selected_sides'] == expected
    finally:
        packets.close()
    assert all(hashlib.sha256(path.read_bytes()).hexdigest() == before[str(path)] for path in files)


def test_requested_missing_side_rejected_before_geometry_or_replay(source):
    with sqlite3.connect(source/'bag/source.db3') as db:
        db.execute("DELETE FROM messages WHERE topic_id IN (SELECT id FROM topics WHERE name LIKE '%lidar_right%')")
    db.close()
    with pytest.raises(ValueError, match='没有所选雷达'):
        mapping_compare.load_dataset(source, sides='all', cloud='raw')
    runtime, _, _ = mapping_compare.load_dataset(source, sides='left', cloud='raw')
    assert runtime['mode'] == 'left'


def test_deployed_lidar_keyword_and_new_sides_keyword_are_equivalent(source):
    old, _, _ = mapping_compare.load_dataset(source, lidar='left', cloud='raw')
    new, _, _ = mapping_compare.load_dataset(source, sides='left', cloud='raw')
    both, _, _ = mapping_compare.load_dataset(source, lidar='left', sides='left', cloud='raw')
    assert old == new == both
    assert old['offline_lidar_selection'] == mapping_compare.select_lidar('all', old['sensor_ids'], 'left')
    assert old['offline_lidar_selection']['clock_reference_sides'] == ['left', 'right']
    with pytest.raises(ValueError, match='冲突'):
        mapping_compare.load_dataset(source, lidar='left', sides='right', cloud='raw')


def test_excluded_lidar_preserves_deployed_clock_but_never_enters_input_or_counts(source):
    # Right raw/filtered frames establish different epochs before the left,
    # wheel and IMU. The formal pipeline must retain both epochs when selecting
    # only left; per-cloud candidates remain necessary and must not be merged.
    with sqlite3.connect(source/'bag/source.db3') as db:
        for topic_id, origin in ((5, 100), (6, 200)):
            data = json.loads(db.execute('SELECT data FROM messages WHERE topic_id=?', (topic_id,)).fetchone()[0])
            data['host_monotonic_ns'] = origin
            db.execute('UPDATE messages SET data=? WHERE topic_id=?', (json.dumps(data).encode(), topic_id))
    db.close()
    clock_by_cloud = {}
    for cloud in ('raw', 'filtered'):
        clocks = []
        for side in ('all', 'left', 'right'):
            runtime, _, _ = mapping_compare.load_dataset(source, sides=side, cloud=cloud)
            packets = RecordedPackets(source, runtime)
            try:
                clocks.append(packets.clock.report())
                expected = {'left', 'right'} if side == 'all' else {side}
                assert {event['side'] for event in packets.events if event['category'] == 'source'} == expected
                assert sum(packets.counts.values()) == len(expected)+2
                if side == 'left':
                    assert not any('lidar_right' in name for name in packets.counts)
            finally:
                packets.close()
        assert clocks[0] == clocks[1] == clocks[2]
        clock_by_cloud[cloud] = clocks[0]
    assert clock_by_cloud['raw']['monotonic_origin_ns'] == 100
    assert clock_by_cloud['filtered']['monotonic_origin_ns'] == 200
    assert clock_by_cloud['raw'] != clock_by_cloud['filtered']


@pytest.mark.parametrize('mode', ['all', 'left', 'right'])
def test_refine_cli_accepts_and_forwards_source_choice(monkeypatch, mode):
    seen = {}
    monkeypatch.setattr(mapping_refine, 'run', lambda args: seen.update(vars(args)) or 0)
    assert mapping_refine.main(['--dataset', 'original', '--output', 'new', '--sides', mode]) == 0
    assert seen['sides'] == mode


def test_compare_cli_passes_selection_into_verified_dataset_loading(source, monkeypatch):
    class BoundaryReached(Exception): pass
    def inspect(path, **options):
        assert options['sides'] == 'right' and options['cloud'] == 'filtered'
        raise BoundaryReached()
    monkeypatch.setattr(mapping_compare, 'load_dataset', inspect)
    with pytest.raises(BoundaryReached):
        mapping_compare.main(['--project-root', str(source.parent), '--dataset', str(source),
                              '--output', str(source.parent/'new-result'), '--sides', 'right', '--cloud', 'filtered'])


def test_compare_cli_retains_lidar_alias(source, monkeypatch):
    class BoundaryReached(Exception): pass
    def inspect(path, **options):
        assert options['lidar'] == 'right' and options['sides'] is None
        raise BoundaryReached()
    monkeypatch.setattr(mapping_compare, 'load_dataset', inspect)
    with pytest.raises(BoundaryReached):
        mapping_compare.main(['--project-root', str(source.parent), '--dataset', str(source),
                              '--output', str(source.parent/'new-result'), '--lidar', 'right'])


def test_refine_cli_retains_lidar_alias(monkeypatch):
    seen = {}
    monkeypatch.setattr(mapping_refine, 'run', lambda args: seen.update(vars(args)) or 0)
    assert mapping_refine.main(['--dataset', 'original', '--output', 'new', '--lidar', 'left']) == 0
    assert seen['lidar'] == 'left' and seen['sides'] is None


@pytest.mark.parametrize('entrypoint', [mapping_compare.main, mapping_refine.main])
def test_conflicting_cli_aliases_fail_before_opening_any_recording(entrypoint):
    with pytest.raises(SystemExit) as error:
        entrypoint(['--dataset', 'not-opened', '--output', 'not-created', '--lidar', 'left', '--sides', 'right'])
    assert error.value.code == 2
