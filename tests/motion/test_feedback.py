"""Offline fixture tests. Addresses/scales/geometry below are invented TEST data."""

import copy
import json
import math
from pathlib import Path
import struct
import sys
from types import SimpleNamespace as NS

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'src'))
from wc_motion.model import Geometry, WheelObservation, WheelOdometry, axle_twist_at_rig
from wc_motion.protocol import FeedbackError, FeedbackProfile, ReadOnlyFeedbackClient, crc16, parse_exchange, read_request
from wc_motion.ros_node import FeedbackProcessor, assign_odometry


def fixture_profile():
    return {'model': 'ZLAC8030D', 'protocol_verified': True,
            'protocol_evidence': 'SYNTHETIC_TEST_ONLY_not_a_device_register_map',
            'device_id': 'TEST_CONTROLLER', 'slave_id': 7, 'allowed_reads': [[0x1000, 2]],
            'fields': {side + '_velocity': {'address': 0x1000 + index, 'words': 1, 'signed': True,
                       'word_order': 'big', 'scale': .25, 'unit': 'motor_rpm'}
                       for index, side in enumerate(('left', 'right'))}}


def fixture_geometry(**updates):
    value = dict(left_radius_m=.1, right_radius_m=.1, track_width_m=.5, left_sign=1, right_sign=1,
                 left_motor_turns_per_wheel_turn=1., right_motor_turns_per_wheel_turn=1.,
                 counts_per_motor_turn=1000, max_wheel_speed_m_s=2., max_gap_s=2.,
                 linear_velocity_variance=.01, angular_velocity_variance=.02)
    value.update(updates)
    return Geometry(**value)


def response(values, slave=7):
    payload = bytes([slave, 3, len(values) * 2]) + b''.join(struct.pack('>H', v & 0xffff) for v in values)
    return payload + crc16(payload)


def record(sequence=1, stamp=1_000_000_000, values=(240, 240)):
    return {'schema': 'wc_wheel_feedback_v1', 'request_hex': read_request(7, 0x1000, 2).hex(),
            'response_hex': response(values).hex(), 'device_id': 'TEST_CONTROLLER',
            'stream_epoch': 'TEST_EPOCH', 'sequence': sequence, 'stamp_ns': stamp,
            'time_valid': True, 'time_source': 'external_common_time', 'uncertainty_ns': 1000}


def observation(sequence, seconds, *, left_rpm=60., right_rpm=60., **updates):
    value = dict(stamp_ns=int(seconds * 1e9), time_valid=True, time_source='external_common_time',
                 uncertainty_ns=1000, stream_epoch='TEST_EPOCH', sequence=sequence,
                 left_velocity_rpm=left_rpm, right_velocity_rpm=right_rpm)
    value.update(updates)
    return WheelObservation(**value)


def test_historical_stationary_frame_crc_only_does_not_prove_units():
    request = bytes.fromhex('01 03 20 AB 00 01 FE 2A')
    reply = bytes.fromhex('01 03 02 00 00 B8 44')
    assert parse_exchange(request, reply) == (1, 0x20ab, (0,))
    with pytest.raises(FeedbackError):
        parse_exchange(request, request)  # Exact local adapter echo is not feedback.


def test_signed_velocity_and_explicit_scale():
    profile = FeedbackProfile(fixture_profile())
    assert profile.decode(record(values=(-240, 120))) == {'left_velocity': -60., 'right_velocity': 30.}


@pytest.mark.parametrize('mutation', [
    lambda d: d.update(protocol_verified=False),
    lambda d: d.update(protocol_evidence=None),
    lambda d: d.update(model='ZLAC8015D'),
    lambda d: d['fields']['left_velocity'].update(scale=None),
    lambda d: d['fields']['left_velocity'].update(unit='UNKNOWN'),
    lambda d: d.update(allowed_reads=[]),
])
def test_unverified_or_unknown_profile_refuses(mutation):
    config = fixture_profile()
    mutation(config)
    with pytest.raises((FeedbackError, TypeError)):
        FeedbackProfile(config)


@pytest.mark.parametrize('mutation', [
    lambda d: d.update(device_id='OTHER_DEVICE'),
    lambda d: d.update(request_hex=read_request(7, 0x1001, 2).hex()),
    lambda d: d.update(response_hex=response((1,), slave=7).hex()),
    lambda d: d.update(response_hex=response((1, 2), slave=8).hex()),
    lambda d: d.update(response_hex='0000000000'),
])
def test_exchange_identity_integrity_and_whitelist(mutation):
    data = record()
    mutation(data)
    with pytest.raises(FeedbackError):
        FeedbackProfile(fixture_profile()).decode(data)


def test_fc03_optional_client_never_sends_control_or_implicit_queries():
    class Transport:
        def __init__(self):
            self.writes = []
            self.data = bytearray(response((240, -240)))
        def write(self, value):
            self.writes.append(value)
            return len(value)
        def read(self, maximum):
            result = self.data[:min(maximum, 2)]
            del self.data[:len(result)]
            return result
    transport = Transport()
    profile = FeedbackProfile(fixture_profile())
    client = ReadOnlyFeedbackClient(profile, transport)
    with pytest.raises(FeedbackError, match='disabled'):
        client.read_feedback(0x1000, 2)
    assert transport.writes == []
    client = ReadOnlyFeedbackClient(profile, transport, allow_read_queries=True)
    with pytest.raises(FeedbackError, match='whitelist'):
        client.read_feedback(0x200e, 1)
    assert transport.writes == []
    request, reply = client.read_feedback(0x1000, 2)
    assert transport.writes == [read_request(7, 0x1000, 2)]
    assert parse_exchange(request, reply)[2] == (240, 0xff10)
    control = bytes.fromhex('07 06 20 0E 00 08')
    with pytest.raises(FeedbackError, match='FC03'):
        profile.validated_query(control + crc16(control))


def test_exception_and_short_client_response_refused():
    request = read_request(7, 0x1000, 2)
    exception = b'\x07\x83\x02'
    with pytest.raises(FeedbackError, match='exception'):
        parse_exchange(request, exception + crc16(exception))


def test_position_word_order_and_signed_decode():
    from wc_motion.protocol import Field
    high_first = Field.from_dict(dict(address=0x1234, words=2, signed=True, word_order='big', scale=1, unit='motor_counts'))
    low_first = Field.from_dict(dict(address=0x1234, words=2, signed=True, word_order='little', scale=1, unit='motor_counts'))
    assert high_first.decode(0x1234, (0xffff, 0xfffe)) == -2
    assert low_first.decode(0x1234, (0xfffe, 0xffff)) == -2
    with pytest.raises(FeedbackError, match='both complete'):
        high_first.decode(0x1235, (0, 0))


def test_query_client_short_io_is_not_retried():
    class Transport:
        writes = 0
        def write(self, data):
            self.writes += 1
            return len(data)
        def read(self, count):
            return b''
    transport = Transport()
    client = ReadOnlyFeedbackClient(FeedbackProfile(fixture_profile()), transport, allow_read_queries=True)
    with pytest.raises(FeedbackError, match='timeout'):
        client.read_feedback(0x1000, 2)
    assert transport.writes == 1


def test_straight_reverse_and_exact_curve():
    model = WheelOdometry(fixture_geometry())
    first = model.update(observation(1, 1))
    assert first.x_m == 0 and first.linear_velocity_m_s == pytest.approx(.2 * math.pi)
    second = model.update(observation(2, 2))
    assert second.x_m == pytest.approx(.2 * math.pi) and second.y_m == pytest.approx(0)
    curve = WheelOdometry(fixture_geometry())
    curve.update(observation(1, 1, left_rpm=0., right_rpm=60.))
    end = curve.update(observation(2, 2, left_rpm=0., right_rpm=60.))
    angle = .2 * math.pi / .5
    assert end.x_m == pytest.approx(.25 * math.sin(angle))
    assert end.y_m == pytest.approx(.25 * (1 - math.cos(angle)))
    assert end.angular_velocity_rad_s == pytest.approx(angle)
    reverse = WheelOdometry(fixture_geometry())
    reverse.update(observation(1, 1, left_rpm=-60., right_rpm=-60.))
    assert reverse.update(observation(2, 2, left_rpm=-60., right_rpm=-60.)).x_m < 0


def test_in_place_turn_has_no_translation_and_actual_forward_signs():
    model = WheelOdometry(fixture_geometry(left_sign=-1))
    model.update(observation(1, 1))
    end = model.update(observation(2, 2))
    assert end.x_m == 0 and end.y_m == 0 and end.angular_velocity_rad_s > 0


def test_count_rollover_integration_and_first_sample_no_fake_velocity():
    model = WheelOdometry(fixture_geometry())
    first = observation(1, 1, left_rpm=None, right_rpm=None, left_position_counts=32760, right_position_counts=32760, counter_bits=16)
    assert model.update(first) is None
    end = model.update(observation(2, 2, left_rpm=None, right_rpm=None, left_position_counts=-32760, right_position_counts=-32760, counter_bits=16))
    assert end.x_m == pytest.approx(16 / 1000 * .2 * math.pi)
    assert end.linear_velocity_m_s == pytest.approx(end.x_m)


@pytest.mark.parametrize('change', [
    dict(time_valid=False), dict(time_source='arrival_only'), dict(uncertainty_ns=None),
    dict(stamp_ns=1_000_000_000), dict(stream_epoch='RESTART'), dict(sequence=1),
    dict(stamp_ns=9_000_000_000), dict(left_velocity_rpm=float('nan')),
    dict(left_velocity_rpm=10000.),
])
def test_bad_time_or_motion_latches_without_moving(change):
    model = WheelOdometry(fixture_geometry())
    model.update(observation(1, 1))
    data = observation(2, 2).__dict__.copy()
    data.update(change)
    with pytest.raises(FeedbackError):
        model.update(WheelObservation(**data))
    assert model.x == 0 and model.previous.sequence == 1
    with pytest.raises(FeedbackError, match='blocked'):
        model.update(observation(3, 3))


@pytest.mark.parametrize('name', ['left_radius_m', 'track_width_m', 'right_motor_turns_per_wheel_turn', 'left_sign'])
def test_unknown_geometry_rejected(name):
    with pytest.raises(FeedbackError):
        fixture_geometry(**{name: None})


def test_counts_require_resolution_and_ambiguous_wrap_fails():
    model = WheelOdometry(fixture_geometry(counts_per_motor_turn=None))
    with pytest.raises(FeedbackError, match='counts per'):
        model.update(observation(1, 1, left_rpm=None, right_rpm=None, left_position_counts=0, right_position_counts=0, counter_bits=16))
    model = WheelOdometry(fixture_geometry(counts_per_motor_turn=1000000))
    model.update(observation(1, 1, left_rpm=None, right_rpm=None, left_position_counts=0, right_position_counts=0, counter_bits=16))
    with pytest.raises(FeedbackError, match='ambiguous counter wrap'):
        model.update(observation(2, 2, left_rpm=None, right_rpm=None, left_position_counts=1, right_position_counts=1, counter_bits=16))


def test_rig_origin_velocity_includes_lever_arm():
    model = WheelOdometry(fixture_geometry())
    estimate = model.update(observation(1, 1, left_rpm=-60., right_rpm=60.))
    vx, vy, wz = axle_twist_at_rig(estimate, rig_origin_in_axle_m=(.2, .3), yaw_axle_from_rig_rad=0.)
    assert vx == pytest.approx(-wz * .3) and vy == pytest.approx(wz * .2)


def test_processor_raw_only_never_needs_protocol_or_geometry():
    processor = FeedbackProcessor({'mode': 'raw_only'})
    raw, estimate = processor.process(json.dumps({'bytes': '00ff', 'time_valid': False}))
    assert raw['bytes'] == '00ff' and estimate is None
    assert processor.status()['state'] == 'RAW_ONLY'
    assert processor.status()['time_valid'] is False


def test_processor_common_time_status_and_no_tf_ros_assignment():
    processor = FeedbackProcessor({'mode': 'wheel_odometry', 'protocol': fixture_profile(), 'geometry': fixture_geometry().__dict__})
    _, estimate = processor.process(json.dumps(record()))
    assert processor.status()['time_valid'] and not processor.status(stale=True)['time_valid']
    assert processor.status()['publishes_tf'] is False
    message = NS(header=NS(stamp=NS()), pose=NS(pose=NS(position=NS(), orientation=NS()), covariance=[0.] * 36),
                 twist=NS(twist=NS(linear=NS(), angular=NS()), covariance=[0.] * 36))
    assign_odometry(estimate, message)
    assert (message.header.frame_id, message.child_frame_id) == ('wheel_odom', 'axle_link')
    assert message.header.stamp.sec == 1 and message.twist.twist.linear.x == pytest.approx(.2 * math.pi)
    assert message.pose.covariance[0] == 1e6 and message.twist.covariance[0] == .01


def test_invalid_json_and_bound_refused():
    processor = FeedbackProcessor({'mode': 'raw_only'})
    for text in ('{"a":NaN}', '[]', '{"x":"' + 'a' * 16384 + '"}'):
        with pytest.raises(FeedbackError):
            processor.process(text)


def test_offline_replay_outputs_real_computed_displacement_and_preserves_input(tmp_path, capsys):
    from wc_motion.replay import main
    cfg, source, output = (tmp_path / name for name in ('config.json', 'input.jsonl', 'result.jsonl'))
    cfg.write_text(json.dumps({'mode': 'wheel_odometry', 'protocol': fixture_profile(), 'geometry': fixture_geometry().__dict__}), encoding='utf-8')
    original = '\n'.join(json.dumps(record(sequence=i, stamp=i * 1_000_000_000)) for i in (1, 2)) + '\n'
    source.write_text(original, encoding='utf-8')
    assert main(['--config', str(cfg), '--input', str(source), '--output', str(output)]) == 0
    results = [json.loads(line) for line in output.read_text(encoding='utf-8').splitlines()]
    assert results[1]['estimate']['x_m'] == pytest.approx(.2 * math.pi)
    assert source.read_text(encoding='utf-8') == original
    with pytest.raises(FileExistsError):
        main(['--config', str(cfg), '--input', str(source), '--output', str(output)])


def test_passive_serial_source_has_no_transmission_or_tty_change():
    import ast
    source = (Path(__file__).resolve().parents[2] / 'src/wc_motion/passive_serial.py').read_text(encoding='utf-8')
    tree = ast.parse(source)
    calls = {ast.unparse(node.func) for node in ast.walk(tree) if isinstance(node, ast.Call)}
    assert 'os.read' in calls and 'os.O_RDONLY' in source
    assert not {'os.write', 'termios.tcsetattr', 'termios.tcsendbreak', 'termios.tcflush'} & calls
    ros_source = (Path(__file__).resolve().parents[2] / 'src/wc_motion/ros_node.py').read_text(encoding='utf-8')
    assert 'TransformBroadcaster' not in ros_source and 'ReadOnlyFeedbackClient(' not in ros_source
