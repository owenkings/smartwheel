"""Offline side selection preserves original bag bytes and motion-candidate inputs.

The SQLite fixtures contain actual ROS-serialized messages, not decoder stubs.
No rclpy node, publisher, device driver or hardware access is started.
"""
import copy
import hashlib
import json
import sqlite3
import struct

import pytest

from wc_runtime import mapping_compare, mapping_refine
from wc_runtime.compare_replay import RecordedPackets
from wc_runtime.offline_refinement_inputs import selected_message_evidence


IMU_TOPIC = '/wc_mapping/imu/source_frame'
WHEEL_TOPIC = '/wc_mapping/wheel/feedback_raw'
SENSOR_IDS = {'left': 'fixture-left', 'right': 'fixture-right'}


@pytest.mark.parametrize('mode,expected', [
    ('all', ['left', 'right']), ('left', ['left']), ('right', ['right'])])
def test_omitted_selection_preserves_recorded_mode_and_input_identity(mode, expected):
    identities = {side: SENSOR_IDS[side] for side in expected}
    before = copy.deepcopy(identities)
    selected = mapping_compare.select_lidar(mode, identities)
    assert selected['recorded_mode'] == selected['selected_mode'] == mode
    assert list(selected['selected_sides']) == expected
    assert list(selected['clock_reference_sides']) == expected
    assert selected['selected_sensor_ids'] == identities == before
    assert not selected['excluded_sides']
    assert selected['selection_basis']


@pytest.mark.parametrize('side,excluded', [('left', 'right'), ('right', 'left')])
def test_explicit_selection_retains_recorded_dual_clock_scope(side, excluded):
    identities = copy.deepcopy(SENSOR_IDS)
    selected = mapping_compare.select_lidar('all', identities, side)
    assert selected['recorded_mode'] == 'all'
    assert selected['selected_mode'] == side
    assert list(selected['selected_sides']) == [side]
    assert list(selected['excluded_sides']) == [excluded]
    assert list(selected['clock_reference_sides']) == ['left', 'right']
    assert selected['selected_sensor_ids'] == {side: SENSOR_IDS[side]}
    assert identities == SENSOR_IDS


@pytest.mark.parametrize('mode', [None, '', 'dual', 'single_left', 'LEFT'])
def test_invalid_recorded_mode_is_rejected(mode):
    with pytest.raises(ValueError):
        mapping_compare.select_lidar(mode, SENSOR_IDS, 'left')


@pytest.mark.parametrize('requested', ['', 'dual', 'single_right', 'RIGHT'])
def test_invalid_requested_selection_is_rejected(requested):
    with pytest.raises(ValueError):
        mapping_compare.select_lidar('all', SENSOR_IDS, requested)


@pytest.mark.parametrize('requested', ['right', 'all'])
def test_missing_selected_identity_is_rejected(requested):
    with pytest.raises(ValueError):
        mapping_compare.select_lidar('left', {'left': SENSOR_IDS['left']}, requested)


@pytest.mark.parametrize('entrypoint', [mapping_compare.main, mapping_refine.main])
def test_cli_documents_explicit_lidar_selection(entrypoint, capsys):
    with pytest.raises(SystemExit) as error:
        entrypoint(['--help'])
    assert error.value.code == 0
    output = capsys.readouterr().out
    assert '--lidar' in output and 'left' in output and 'right' in output and 'all' in output


def _set_stamp(target, ns):
    target.sec, target.nanosec = divmod(ns, 10**9)


@pytest.fixture
def bag_case(tmp_path):
    from rclpy.serialization import serialize_message
    from sensor_msgs.msg import PointField
    from std_msgs.msg import String
    from wc_interfaces.msg import SourceFrame, H30Frame

    source = tmp_path / 'single_lidar_fixture'
    bag = source / 'bag'
    bag.mkdir(parents=True)
    database = bag / 'bag_0.db3'
    topics = [
        (1, '/wc_mapping/lidar_left/source_frame', 'wc_interfaces/msg/SourceFrame'),
        (2, '/wc_mapping/lidar_right/source_frame', 'wc_interfaces/msg/SourceFrame'),
        (3, '/wc_mapping/lidar_left/source_frame_filtered', 'wc_interfaces/msg/SourceFrame'),
        (4, '/wc_mapping/lidar_right/source_frame_filtered', 'wc_interfaces/msg/SourceFrame'),
        (5, IMU_TOPIC, 'wc_interfaces/msg/H30Frame'),
        (6, WHEEL_TOPIC, 'std_msgs/msg/String'),
    ]
    with sqlite3.connect(database) as db:
        db.executescript('CREATE TABLE topics(id INTEGER PRIMARY KEY,name TEXT,type TEXT);'
                        'CREATE TABLE messages(id INTEGER PRIMARY KEY,topic_id INTEGER,timestamp INTEGER,data BLOB);')
        db.executemany('INSERT INTO topics VALUES(?,?,?)', topics)
        # Right lidar owns the earliest original event and is excluded by a left run.
        events = [('right', 1, 1_000_000), ('left', 1, 2_000_000),
                  ('imu', 1, 3_000_000), ('wheel', 1, 4_000_000),
                  ('right', 2, 5_000_000), ('left', 2, 6_000_000),
                  ('imu', 2, 7_000_000), ('wheel', 2, 8_000_000),
                  ('imu', 3, 9_000_000)]
        for category, sequence, monotonic in events:
            wall_ns = 100_000_000_000 + monotonic
            if category == 'wheel':
                message = String(data=json.dumps({'stamp_ns': wall_ns,
                    'receive_monotonic_ns': monotonic, 'sequence': sequence,
                    'session_id': source.name, 'device_id': 'fixture-wheel'}))
                topic_ids = [6]
            else:
                message = SourceFrame() if category in SENSOR_IDS else H30Frame()
                message.session_id = source.name
                message.sensor_id = SENSOR_IDS.get(category, 'fixture-imu')
                message.stream_epoch = 'fixture-' + category
                message.frame_sequence = sequence
                message.host_monotonic_ns = monotonic
                message.time_source = 'arrival_only'
                message.common_time_valid = False
                frame = 'lidar_' + category if category in SENSOR_IDS else 'imu_h30_native'
                message.header.frame_id = frame
                _set_stamp(message.header.stamp, wall_ns)
                _set_stamp(message.host_receive_time, wall_ns)
                if category in SENSOR_IDS:
                    message.side = category
                    message.common_time_ns = wall_ns
                    message.cloud.header.frame_id = frame
                    _set_stamp(message.cloud.header.stamp, wall_ns)
                    message.cloud.height = 1
                    message.cloud.width = 1
                    message.cloud.point_step = message.cloud.row_step = 12
                    message.cloud.fields = [PointField(name=name, offset=i*4,
                        datatype=PointField.FLOAT32, count=1) for i, name in enumerate(('x', 'y', 'z'))]
                    message.cloud.data = struct.pack('<fff', 1., 0.2, 0.5)
                    message.raw_count = message.valid_count = 1
                    topic_ids = [1, 3] if category == 'left' else [2, 4]
                else:
                    message.imu.header.frame_id = frame
                    _set_stamp(message.imu.header.stamp, wall_ns)
                    message.angular_velocity_valid = message.linear_acceleration_valid = True
                    message.imu.linear_acceleration.z = 9.81
                    topic_ids = [5]
            for topic_id in topic_ids:
                db.execute('INSERT INTO messages(topic_id,timestamp,data) VALUES(?,?,?)',
                           (topic_id, wall_ns, serialize_message(message)))
    return source, database


def _config(lidar=None, cloud='raw'):
    selection = mapping_compare.select_lidar('all', SENSOR_IDS, lidar)
    return {'mode': selection['selected_mode'], 'cloud_source': cloud,
            'offline_lidar_selection': selection,
            'prior_template': {'imu_topic': IMU_TOPIC, 'wheel_topic': WHEEL_TOPIC}}


@pytest.mark.parametrize('cloud', ['raw', 'filtered'])
@pytest.mark.parametrize('side', ['left', 'right'])
def test_single_replay_preserves_dual_epoch_motion_evidence_and_original_bytes(bag_case, side, cloud):
    source, database = bag_case
    original = hashlib.sha256(database.read_bytes()).hexdigest()
    dual = RecordedPackets(source, _config(cloud=cloud))
    single = RecordedPackets(source, _config(side, cloud))
    try:
        assert single.clock.report() == dual.clock.report()
        assert single.clock.monotonic_origin_ns == 1_000_000
        assert single.clock.ros_origin_ns == 100_001_000_000
        assert set(single.sides) == {side}
        rows = [row for row in single.events if row['category'] == 'source']
        assert len(rows) == 2 and {row['side'] for row in rows} == {side}
        suffix = '_filtered' if cloud == 'filtered' else ''
        selected_topic = '/wc_mapping/lidar_' + side + '/source_frame' + suffix
        assert {row['topic'] for row in rows} == {selected_topic}
        assert dict(single.counts) == {selected_topic: 2, IMU_TOPIC: 3, WHEEL_TOPIC: 2}
        single_motion = [row for row in single.events if row['category'] in ('imu', 'wheel')]
        dual_motion = [row for row in dual.events if row['category'] in ('imu', 'wheel')]
        assert single_motion == dual_motion
        single_evidence, _, counts = selected_message_evidence(source, packets=single)
        dual_evidence, _, _ = selected_message_evidence(source, packets=dual)
        assert single_evidence == dual_evidence
        assert counts == {IMU_TOPIC: 3, WHEEL_TOPIC: 2}
        for row in single.events:
            message = single.get(row)
            if row['category'] == 'source':
                assert message.side == side
            elif row['category'] == 'wheel':
                assert json.loads(message.data)['sequence'] == row['sequence']
            else:
                assert message.frame_sequence == row['sequence']
    finally:
        single.close()
        dual.close()
    assert hashlib.sha256(database.read_bytes()).hexdigest() == original


@pytest.mark.parametrize('side,topic_ids', [('left', (1, 3)), ('right', (2, 4))])
def test_declared_side_without_actual_source_messages_is_rejected(bag_case, side, topic_ids):
    source, database = bag_case
    with sqlite3.connect(database) as db:
        db.execute('DELETE FROM messages WHERE topic_id IN (?,?)', topic_ids)
    with pytest.raises(ValueError):
        RecordedPackets(source, _config(side))
