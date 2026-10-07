"""Historical parameters are preview-only; formal odometry remains gated."""

import copy
import json
import math
from pathlib import Path
import shutil
import struct
from types import SimpleNamespace as NS
import uuid

import pytest

from wc_motion.history_preview import HistoryPreview, assign_preview_odometry
from wc_motion import feedback_transport as transport
from wc_motion.feedback_transport import QUERY
from wc_motion.protocol import FeedbackError, crc16
from wc_motion.model import WheelObservation


@pytest.fixture
def tmp_path():
    # Match the workspace-local fixtures used by the mapping tests. Pytest's
    # mode-0700 temporary directories are inaccessible in the Windows sandbox.
    parent = Path(__file__).resolve().parent
    path = parent/('.history_preview_test_'+uuid.uuid4().hex)
    path.mkdir()
    try:
        yield path
    finally:
        assert path.resolve().parent == parent and path.name.startswith('.history_preview_test_')
        shutil.rmtree(path)


@pytest.fixture
def config():
    return json.loads((Path(__file__).resolve().parents[2]/'config/wheel_history_calibration.json').read_text())


def sample(sequence=0, left=-100, right=100, *, stamp=None):
    payload = b'\x01\x03\x04'+struct.pack('>HH', left & 65535, right & 65535)
    stamp = stamp or 1_000_000_000+sequence*100_000_000
    return {'schema': 'wc_wheel_feedback_v1', 'device_id': json.loads((Path(__file__).resolve().parents[2]/'config/wheel_feedback_current.json').read_text())['device_id'],
        'status': 'RESPONSE_VALID', 'request_hex': QUERY.hex(), 'response_hex': (payload+crc16(payload)).hex(),
        'stamp_ns': stamp, 'receive_monotonic_ns': stamp, 'sequence': sequence, 'stream_epoch': 'synthetic-fixture',
        'time_valid': False, 'time_source': 'arrival_only', 'uncertainty_ns': None}


def test_history_preview_forward_distance_and_straight_pose_match_explicit_candidate(config):
    preview = HistoryPreview(config)
    first = preview.update(sample())
    second = preview.update(sample(1))
    expected = 100*.112*2*math.pi*.165/60
    assert first['x_m'] == first['y_m'] == first['yaw_rad'] == 0
    assert second['linear_velocity_m_s'] == pytest.approx(expected)
    assert second['x_m'] == second['signed_distance_m'] == second['travel_distance_m'] == pytest.approx(expected*.1)
    assert second['state'] == 'UNVALIDATED' and not second['formal_odometry_eligible']
    assert not second['time_valid'] and second['uncertainty_ns'] is None and second['precision_evidence'] == 'NONE'
    assert second['frame_id'] == 'wheel_odom_preview' and second['child_frame_id'] == 'axle_preview'


def test_preview_turn_and_reverse_preserve_signed_distance(config):
    preview = HistoryPreview(config)
    preview.update(sample(0, left=100, right=-100))
    reverse = preview.update(sample(1, left=100, right=-100))
    assert reverse['signed_distance_m'] < 0 < reverse['travel_distance_m']
    preview = HistoryPreview(config)
    preview.update(sample(0, left=100, right=100))
    turn = preview.update(sample(1, left=100, right=100))
    assert turn['x_m'] == 0 and turn['signed_distance_m'] == 0
    assert turn['angular_velocity_rad_s'] > 0 and turn['yaw_rad'] > 0


@pytest.mark.parametrize('change', [{'stamp_ns': 1}, {'receive_monotonic_ns': 1}, {'sequence': 3},
    {'stream_epoch': 'restarted'}, {'device_id': 'another-device'}, {'status': 'ERROR'},
    {'time_valid': True}, {'response_hex': '010304000000000000'}])
def test_preview_invalid_input_latches_and_preserves_pose(config, change):
    preview = HistoryPreview(config)
    preview.update(sample())
    with pytest.raises(FeedbackError):
        preview.update({**sample(1), **change})
    assert preview.blocked and preview.x == 0 and preview.distance == 0
    with pytest.raises(FeedbackError, match='BLOCKED'):
        preview.update(sample(2))


def test_preview_long_gap_and_transport_error_stop_integration(config):
    preview = HistoryPreview(config)
    preview.update(sample())
    with pytest.raises(FeedbackError, match='gap'):
        preview.update(sample(1, stamp=2_000_000_000))
    assert preview.x == preview.travel == 0
    preview = HistoryPreview(config)
    preview.update(sample())
    preview.block('transport timeout')
    with pytest.raises(FeedbackError, match='transport timeout'):
        preview.update(sample(1))


@pytest.mark.parametrize('sequence,stamp,missing', [(2, 1_200_000_000, 1), (1, 9_000_000_000, 0), (9, 9_000_000_000, 8)])
def test_continuous_preview_skips_uncovered_gap_and_resumes_real_sample_integration(config, sequence, stamp, missing):
    preview = HistoryPreview(config, continuous=True)
    initial = preview.update(sample())
    gap = preview.update(sample(sequence, stamp=stamp))
    assert preview.blocked is None and gap['continuous_mapping'] is True
    assert gap['integration_gap'] and gap['integration_interval_s'] is None
    assert gap['x_m'] == initial['x_m'] == gap['travel_distance_m'] == 0.
    assert gap['integration_gap_count'] == 1 and gap['missing_sequences'] == missing
    assert gap['uncovered_duration_s'] == pytest.approx((stamp-1_000_000_000)/1e9)
    assert not gap['motion_coverage_complete'] and gap['stamp_ns'] == stamp
    resumed = preview.update(sample(sequence+1, stamp=stamp+100_000_000))
    assert resumed['integration_interval_s'] == pytest.approx(.1) and not resumed['integration_gap']
    assert resumed['x_m'] == pytest.approx(resumed['linear_velocity_m_s']*.1)
    assert resumed['integration_gap_count'] == 1 and not resumed['motion_coverage_complete']
    assert not resumed['time_valid'] and not resumed['formal_odometry_eligible']


@pytest.mark.parametrize('change', [{'stamp_ns': 1}, {'receive_monotonic_ns': 1}, {'sequence': 0},
    {'stream_epoch': 'restarted'}, {'device_id': 'another-device'}, {'status': 'ERROR'},
    {'response_hex': '010304000000000000'}])
def test_continuous_preview_keeps_identity_crc_and_backward_time_failure(config, change):
    preview = HistoryPreview(config, continuous=True)
    preview.update(sample())
    with pytest.raises(FeedbackError):
        preview.update({**sample(1), **change})
    assert preview.blocked and preview.x == preview.travel == 0


def test_continuous_preview_keeps_wheel_speed_bound(config):
    preview = HistoryPreview(config, continuous=True)
    with pytest.raises(FeedbackError, match='speed'):
        preview.update(sample(left=-30000, right=30000))


def test_preview_ros_adapter_uses_different_frames_and_unknown_covariance(config):
    candidate = HistoryPreview(config).update(sample())
    pose = NS(position=NS(x=0., y=0., z=0.), orientation=NS(x=0., y=0., z=0., w=0.))
    twist = NS(linear=NS(x=0., y=0., z=0.), angular=NS(x=0., y=0., z=0.))
    message = NS(header=NS(stamp=NS(sec=0, nanosec=0), frame_id=''), child_frame_id='',
        pose=NS(pose=pose, covariance=[0.]*36), twist=NS(twist=twist, covariance=[0.]*36))
    assign_preview_odometry(candidate, message)
    assert message.header.frame_id == 'wheel_odom_preview' and message.child_frame_id == 'axle_preview'
    assert message.pose.pose.orientation.w == 1
    assert all(message.pose.covariance[i] == message.twist.covariance[i] == 1e6 for i in (0, 7, 14, 21, 28, 35))
    with pytest.raises(FeedbackError, match='common measurement time'):
        WheelObservation(stamp_ns=candidate['stamp_ns'], time_valid=candidate['time_valid'],
            time_source=candidate['time_source'], uncertainty_ns=candidate['uncertainty_ns'], stream_epoch='fixture',
            sequence=0, left_velocity_rpm=1, right_velocity_rpm=1).validate()


@pytest.mark.parametrize('change', [{'state': 'VALIDATED'}, {'formal_odometry_eligible': True},
    {'wheel_radius_m': float('nan')}, {'track_width_m': 0}, {'left_sign': True}])
def test_preview_configuration_cannot_claim_validation(config, change):
    with pytest.raises(FeedbackError):
        HistoryPreview({**config, **change})


@pytest.mark.parametrize('preview_enabled, fail_second, continuous_preview, gap_second', [
    (False, False, False, False), (True, False, False, False), (True, True, False, False),
    (True, False, True, False), (True, True, True, False),
    (True, False, False, True), (True, False, True, True)])
def test_cli_preview_is_explicit_and_transport_failure_ends_integration(
        tmp_path, monkeypatch, preview_enabled, fail_second, continuous_preview, gap_second):
    config_path = Path(__file__).resolve().parents[2]/'config/wheel_feedback_current.json'
    clock = [10.]
    events = []
    monkeypatch.setattr(transport.signal, 'signal', lambda *args: None)
    monkeypatch.setattr(transport.time, 'monotonic', lambda: clock[0])
    monkeypatch.setattr(transport.time, 'time_ns', lambda: int(clock[0]*1e9))
    monkeypatch.setattr(transport.time, 'monotonic_ns', lambda: int(clock[0]*1e9))
    monkeypatch.setattr(transport.time, 'sleep', lambda seconds: clock.__setitem__(0, clock[0]+max(seconds, 1e-8)))
    class Lease:
        host_configuration = {'fixture': 'NO_REAL_DEVICE'}
        def open(self):
            events.append('open')
        def close(self):
            events.append('close')
        def exchange(self, query, timeout):
            events.append(query)
            if fail_second and events.count(QUERY) == 2:
                raise transport.QueryFailure('fixture response timeout', b'\x01', 8)
            if gap_second and events.count(QUERY) == 2:
                clock[0] += 1.  # Valid next response after a synthetic scheduling gap.
            if gap_second and events.count(QUERY) == 3:
                clock[0] += .1  # The resumed exchange has a distinct arrival time.
            return bytes.fromhex(sample()['response_hex'])
    monkeypatch.setattr(transport, 'FeedbackSerialLease', lambda *args: Lease())
    output = tmp_path/'fixture.jsonl'
    args = ['--config', str(config_path), '--output', str(output), '--run-root', str(tmp_path),
            '--samples', '3', '--rate-hz', '10', '--allow-read-queries']
    if preview_enabled:
        args.append('--preview-history-calibration')
    if continuous_preview:
        args.append('--continuous-preview')
    should_fail = fail_second or (gap_second and not continuous_preview)
    assert transport.main(args) == int(should_fail)
    records = [json.loads(line) for line in output.read_text().splitlines()]
    previews = [row for row in records if row['event'] == 'history_preview']
    assert len(previews) == (1 if should_fail else 3) if preview_enabled else not previews
    assert events[0] == 'open' and events[-1] == 'close'
    assert events.count(QUERY) == (2 if should_fail else 3)
    if preview_enabled:
        assert all(row['state'] == 'UNVALIDATED' and row['formal_odometry_eligible'] is False for row in previews)
        assert all(row['continuous_mapping'] is continuous_preview for row in previews)
        assert [row for row in records if row['event'] == 'history_preview_end'][0]['state'] == ('BLOCKED' if should_fail else 'STOPPED')
        if gap_second and continuous_preview:
            assert previews[1]['integration_gap'] and previews[1]['integration_interval_s'] is None
            assert previews[1]['x_m'] == previews[0]['x_m'] == 0.
            assert not previews[2]['integration_gap'] and previews[2]['integration_interval_s'] == pytest.approx(.1)
            assert previews[2]['x_m'] > previews[1]['x_m']
            assert not previews[2]['motion_coverage_complete']


def test_cli_continuous_preview_requires_explicit_preview_mode_before_opening_lease(tmp_path, monkeypatch):
    config_path = Path(__file__).resolve().parents[2]/'config/wheel_feedback_current.json'
    def forbidden(*args):
        pytest.fail('Invalid preview flags must never acquire the serial lease')
    monkeypatch.setattr(transport, 'FeedbackSerialLease', forbidden)
    output = tmp_path/'not_created.jsonl'
    with pytest.raises(FeedbackError, match='--continuous-preview requires explicit --preview-history-calibration'):
        transport.main(['--config', str(config_path), '--output', str(output),
                        '--run-root', str(tmp_path), '--allow-read-queries', '--continuous-preview'])
    assert not output.exists()
