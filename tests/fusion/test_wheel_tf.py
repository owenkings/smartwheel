"""Synthetic private guess / native public TF separation; no ROS or hardware."""

from dataclasses import replace
from pathlib import Path
import sys
from types import SimpleNamespace as NS

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'src'))
from wc_fusion.core import FusionError, Policy
from wc_fusion.wheel_tf import WheelTfBridge, assign_transform


def config():
    mount = np.eye(4)
    mount[:3, :3] = Rotation.from_euler('xyz', [.04, -.08, .03]).as_matrix()
    mount[:3, 3] = [.2, .1, .3]
    return {'coordinate_status': 'SYNTHETIC', 'time_status': 'SYNTHETIC',
            'T_rig_axle': mount.tolist(), 'max_time_uncertainty_ns': 1_000_000}


def message(stamp, x=0., yaw=0., *, parent='wheel_odom', child='axle_link'):
    q = Rotation.from_euler('z', yaw).as_quat()
    return NS(header=NS(frame_id=parent, stamp=NS(sec=stamp // 1_000_000_000, nanosec=stamp % 1_000_000_000)),
              child_frame_id=child,
              pose=NS(pose=NS(position=NS(x=x, y=0., z=0.),
                              orientation=NS(x=q[0], y=q[1], z=q[2], w=q[3]))))


def diagnostic(stamp, sequence):
    return {'schema': 'wc_wheel_diagnostics_v1', 'time_valid': True, 'state': 'FEEDBACK',
            'time_source': 'external_common_time', 'stamp_ns': stamp,
            'uncertainty_ns': 1000, 'stream_epoch': 'FIXTURE_ONLY', 'sequence': sequence}


def initialized():
    bridge = WheelTfBridge(Policy(), config(), 'synthetic')
    for i in (0, 1):
        stamp = 1_000_000_000 + i * 100_000_000
        bridge.observe_wheel(message(stamp, x=i * .01), diagnostic(stamp, i),
                             now_ns=stamp + 1_000_000, now_monotonic_ns=stamp)
    return bridge


def prepare(bridge, stamp=1_150_000_000):
    return bridge.prepare_guess(stamp, now_ns=stamp + 1_000_000, now_monotonic_ns=stamp)


def test_tilted_mount_and_wheel_motion_reach_private_guess_only():
    bridge = initialized()
    record = prepare(bridge)
    expected = np.eye(4)
    expected[0, 3] = .015
    expected = expected @ np.linalg.inv(config()['T_rig_axle'])
    np.testing.assert_allclose(record.transform, expected, atol=1e-12)
    assert (record.parent_frame, record.child_frame) == ('wheel_odom', 'rig_link')
    assert bridge.private_tf_topic != '/tf'
    assert bridge.status()['last_relayed_ns'] is None


def test_native_relay_copies_native_pose_instead_of_wheel_guess():
    bridge = initialized()
    frozen = prepare(bridge)
    bridge.register_guess(frozen)
    # Deliberately distinct native answer proves the relay does not substitute
    # wheel pose or apply the wheel-to-rig extrinsic a second time.
    native = message(frozen.stamp_ns, x=3., yaw=.3, parent='odom', child='rig_link')
    output = bridge.relay_native_odometry(native)
    assert (output.parent_frame, output.child_frame) == ('odom', 'rig_link')
    assert output.transform[0, 3] == 3.
    np.testing.assert_allclose(output.transform[:3, :3], Rotation.from_euler('z', .3).as_matrix())
    assert bridge.history.poses[-1][0] == 1_100_000_000
    assert bridge.status()['pending_native_stamps'] == []
    with pytest.raises(FusionError, match='UNEXPECTED'):
        bridge.relay_native_odometry(native)


def test_archive_delay_does_not_resample_frozen_guess():
    bridge = initialized()
    frozen = prepare(bridge)
    original = frozen.transform.copy()
    for i in range(2, 7):
        stamp = 1_000_000_000 + i * 100_000_000
        bridge.observe_wheel(message(stamp, x=i * .01), diagnostic(stamp, i),
                             now_ns=stamp + 1_000_000, now_monotonic_ns=stamp)
    registered = bridge.register_guess(frozen)
    assert registered.stamp_ns == 1_150_000_000
    np.testing.assert_array_equal(registered.transform, original)


def test_frozen_guess_mutation_rejected():
    bridge = initialized()
    frozen = prepare(bridge)
    frozen.transform[0, 3] += .001
    with pytest.raises(FusionError, match='FROZEN_GUESS_CHANGED'):
        bridge.register_guess(frozen)
    assert bridge.status()['pending_native_stamps'] == []


def test_only_one_native_input_and_prepared_order():
    bridge = initialized()
    first, second = prepare(bridge), prepare(bridge, 1_160_000_000)
    with pytest.raises(FusionError, match='REORDERED'):
        bridge.register_guess(second)
    bridge.register_guess(first)
    with pytest.raises(FusionError, match='IN_FLIGHT'):
        bridge.register_guess(second)
    bridge.relay_native_odometry(message(first.stamp_ns, parent='odom', child='rig_link'))
    bridge.register_guess(second)


@pytest.mark.parametrize('change', [
    {'time_valid': False}, {'time_source': 'arrival_only'}, {'stamp_ns': 1_099_999_999},
    {'uncertainty_ns': None}, {'uncertainty_ns': 1_000_001}, {'state': 'STALE'},
    {'stream_epoch': 'RESTART'}, {'sequence': 1},
])
def test_missing_or_mismatched_wheel_truth_blocks(change):
    bridge = initialized()
    stamp = 1_200_000_000
    diag = diagnostic(stamp, 2)
    diag.update(change)
    with pytest.raises(FusionError):
        bridge.observe_wheel(message(stamp, x=.02), diag, now_ns=stamp + 1000, now_monotonic_ns=stamp)
    assert bridge.status()['blocked']
    with pytest.raises(FusionError, match='BLOCKED'):
        prepare(bridge)


@pytest.mark.parametrize('change', [
    {'T_rig_axle': None}, {'coordinate_status': 'UNKNOWN'}, {'time_status': 'UNKNOWN'},
    {'max_time_uncertainty_ns': None},
])
def test_unknown_mount_and_time_are_never_identity_defaults(change):
    cfg = config()
    cfg.update(change)
    with pytest.raises(FusionError):
        WheelTfBridge(Policy(), cfg, 'synthetic')
    with pytest.raises(FusionError):
        WheelTfBridge(Policy(), config(), 'real')


def test_stale_measurement_and_monotonic_freshness_fail():
    bridge = initialized()
    with pytest.raises(FusionError, match='STALE'):
        bridge.prepare_guess(1_150_000_000, now_ns=1_500_000_000, now_monotonic_ns=1_500_000_000)
    with pytest.raises(FusionError, match='STALE'):
        bridge.prepare_guess(1_150_000_000, now_ns=1_150_000_000, now_monotonic_ns=1_050_000_000)
    assert not bridge.status()['prepared_stamps']


def test_one_sample_is_not_a_motion_guess():
    bridge = WheelTfBridge(Policy(), config(), 'synthetic')
    stamp = 1_000_000_000
    bridge.observe_wheel(message(stamp), diagnostic(stamp, 0), now_ns=stamp, now_monotonic_ns=stamp)
    with pytest.raises(FusionError, match='HISTORY'):
        bridge.prepare_guess(stamp, now_ns=stamp, now_monotonic_ns=stamp)


def test_native_unregistered_stamp_bad_frame_and_null_pose_are_rejected():
    bridge = initialized()
    frozen = bridge.register_guess(prepare(bridge))
    with pytest.raises(FusionError, match='UNEXPECTED'):
        bridge.relay_native_odometry(message(frozen.stamp_ns + 1, parent='odom', child='rig_link'))
    with pytest.raises(FusionError, match='FRAME'):
        bridge.relay_native_odometry(message(frozen.stamp_ns))
    bad = message(frozen.stamp_ns, parent='odom', child='rig_link')
    bad.pose.pose.orientation.w = 0.
    with pytest.raises(FusionError, match='QUATERNION'):
        bridge.relay_native_odometry(bad)
    assert bridge.status()['pending_native_stamps'] == [frozen.stamp_ns]


def test_tf_assignment_retains_exact_nanoseconds():
    bridge = initialized()
    record = prepare(bridge, 1_150_000_123)
    output = NS(header=NS(stamp=NS()), transform=NS(translation=NS(), rotation=NS()))
    assign_transform(record, output)
    assert output.header.stamp.sec == 1 and output.header.stamp.nanosec == 150_000_123
    assert output.header.frame_id == 'wheel_odom' and output.child_frame_id == 'rig_link'


def test_prediction_queue_is_bounded():
    bridge = initialized()
    for i in range(bridge.policy.max_queue):
        prepare(bridge, 1_110_000_000 + i)
    with pytest.raises(FusionError, match='QUEUE_FULL'):
        prepare(bridge, 1_110_000_100)
    assert len(bridge.status()['prepared_stamps']) == bridge.policy.max_queue
