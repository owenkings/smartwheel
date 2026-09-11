import math

import pytest

from wheelchair_3d_mapping.lio_consistency import (
    MonitorConfig,
    MotionConsistencyMonitor,
    Pose,
    TimedPose,
    compose_pose,
    inverse_pose,
    lio_imu_pose_to_base_pose,
    relative_pose,
)


def yaw(angle):
    return (0.0, 0.0, math.sin(angle / 2.0), math.cos(angle / 2.0))


def pose(x=0.0, y=0.0, z=0.0, angle=0.0):
    return Pose((x, y, z), yaw(angle))


def config(**overrides):
    values = dict(
        window_sec=1.0,
        source_timeout_sec=1.0,
        max_alignment_age_sec=0.20,
        max_interpolation_gap_sec=0.60,
        max_translation_error_m=0.10,
        max_rotation_error_rad=0.15,
        retention_sec=4.0,
    )
    values.update(overrides)
    return MonitorConfig(**values)


def add_pair(monitor, stamp, lio_imu, reference_base, receipt=None):
    receipt = stamp if receipt is None else receipt
    monitor.add_lio_imu_pose(TimedPose(stamp, lio_imu), receipt)
    monitor.add_reference_base_pose(TimedPose(stamp, reference_base), receipt)


def test_pose_composition_and_inverse_round_trip():
    original = pose(1.0, -2.0, 0.4, 0.7)
    identity = compose_pose(original, inverse_pose(original))
    assert identity.translation == pytest.approx((0.0, 0.0, 0.0), abs=1e-12)
    assert identity.rotation == pytest.approx((0.0, 0.0, 0.0, 1.0), abs=1e-12)


def test_base_to_imu_lever_arm_is_removed_before_relative_comparison():
    # IMU is 1 m ahead of base. A 90-degree base turn moves the IMU through an
    # arc even though the base origin only rotates. The conversion must remove
    # that apparent translation.
    base_to_imu = pose(1.0, 0.0, 0.0)
    world_base_start = pose()
    world_base_end = pose(angle=math.pi / 2.0)
    world_imu_start = compose_pose(world_base_start, base_to_imu)
    world_imu_end = compose_pose(world_base_end, base_to_imu)
    recovered_start = lio_imu_pose_to_base_pose(world_imu_start, base_to_imu)
    recovered_end = lio_imu_pose_to_base_pose(world_imu_end, base_to_imu)
    increment = relative_pose(recovered_start, recovered_end)
    assert increment.translation == pytest.approx((0.0, 0.0, 0.0), abs=1e-12)


def test_unavailable_is_false_until_reference_has_started():
    monitor = MotionConsistencyMonitor(pose(), config())
    monitor.add_lio_imu_pose(TimedPose(10.0, pose()), 10.0)
    result = monitor.evaluate(10.1)
    assert result.state == "UNAVAILABLE"
    assert not result.consistent
    assert "reference_not_started" in result.reasons


def test_recording_startup_sequence_stays_unavailable_between_lio_and_wheel_start():
    monitor = MotionConsistencyMonitor(pose(), config(source_timeout_sec=60.0))
    # Mirrors the new bag's source ordering: shadow LIO begins around bag+7 s,
    # while wheel/IMU odometry does not exist until about bag+41 s.
    monitor.add_lio_imu_pose(TimedPose(1007.0, pose()), 7.0)
    monitor.add_lio_imu_pose(TimedPose(1008.0, pose()), 8.0)
    result = monitor.evaluate(40.9)
    assert result.state == "UNAVAILABLE"
    assert not result.consistent
    assert result.reasons == ("reference_not_started",)


def test_matching_relative_motion_passes_despite_different_world_origins():
    monitor = MotionConsistencyMonitor(pose(), config())
    # Arbitrary and different world origins. Only each stream's increment is
    # comparable.
    add_pair(monitor, 10.0, pose(100.0, 50.0, 2.0, 1.2), pose(-4.0, 8.0, 0.0, -0.5))
    lio_end = compose_pose(pose(100.0, 50.0, 2.0, 1.2), pose(0.08, 0.0, 0.0, 0.1))
    ref_end = compose_pose(pose(-4.0, 8.0, 0.0, -0.5), pose(0.08, 0.0, 0.0, 0.1))
    add_pair(monitor, 11.0, lio_end, ref_end)
    result = monitor.evaluate(11.05)
    assert result.state == "OK"
    assert result.consistent
    assert result.values["translation_error_m"] == pytest.approx(0.0, abs=1e-12)
    assert result.values["rotation_error_rad"] == pytest.approx(0.0, abs=1e-12)


def test_translation_and_rotation_divergence_are_both_reported():
    monitor = MotionConsistencyMonitor(pose(), config())
    add_pair(monitor, 10.0, pose(), pose())
    add_pair(monitor, 11.0, pose(0.5, 0.0, 0.2, 0.5), pose(0.08, 0.0, 0.0, 0.1))
    result = monitor.evaluate(11.05)
    assert result.state == "DIVERGENCE"
    assert not result.consistent
    assert set(result.reasons) == {
        "translation_increment_mismatch", "rotation_increment_mismatch"
    }
    assert result.values["thresholds_provisional"] is True


def test_reference_interpolation_uses_exact_lio_window_stamps():
    monitor = MotionConsistencyMonitor(pose(), config())
    monitor.add_lio_imu_pose(TimedPose(10.25, pose(0.0)), 10.25)
    monitor.add_lio_imu_pose(TimedPose(11.25, pose(1.0)), 11.25)
    # Reference samples bracket both LIO stamps; linear interpolation produces
    # exactly the same 1 m increment.
    for stamp, x in ((10.2, -0.05), (10.3, 0.05), (11.2, 0.95), (11.3, 1.05)):
        monitor.add_reference_base_pose(TimedPose(stamp, pose(x)), stamp)
    result = monitor.evaluate(11.55)
    assert result.state == "OK"
    assert result.values["reference_translation_increment_m"] == pytest.approx(
        (1.0, 0.0, 0.0)
    )


def test_large_interpolation_gap_is_invalid_not_pass():
    monitor = MotionConsistencyMonitor(
        pose(), config(max_interpolation_gap_sec=0.20)
    )
    monitor.add_lio_imu_pose(TimedPose(10.0, pose()), 10.0)
    monitor.add_lio_imu_pose(TimedPose(11.0, pose(0.1)), 11.0)
    monitor.add_reference_base_pose(TimedPose(9.9, pose()), 9.9)
    monitor.add_reference_base_pose(TimedPose(11.1, pose(0.1)), 11.1)
    result = monitor.evaluate(11.15)
    assert result.state == "INVALID"
    assert not result.consistent
    assert any("interpolation_gap_exceeded" in reason for reason in result.reasons)


def test_source_timeout_is_false_and_explicit():
    monitor = MotionConsistencyMonitor(pose(), config(source_timeout_sec=0.5))
    add_pair(monitor, 10.0, pose(), pose(), receipt=20.0)
    add_pair(monitor, 11.0, pose(0.1), pose(0.1), receipt=20.1)
    result = monitor.evaluate(20.7)
    assert result.state == "STALE"
    assert not result.consistent
    assert "lio_timeout" in result.reasons
    assert "reference_timeout" in result.reasons


def test_fresh_republication_of_duplicate_old_poses_is_timestamp_stale():
    monitor = MotionConsistencyMonitor(pose(), config(source_timeout_sec=0.5))
    add_pair(monitor, 10.0, pose(), pose(), receipt=10.0)
    add_pair(monitor, 11.0, pose(0.1), pose(0.1), receipt=11.0)
    # A publisher can remain alive while repeatedly sending the same old state.
    # Fresh callback receipt must not turn that into an OK motion source.
    add_pair(monitor, 11.0, pose(0.1), pose(0.1), receipt=11.8)
    result = monitor.evaluate(11.8)
    assert result.state == "STALE"
    assert not result.consistent
    assert "lio_timestamp_stale" in result.reasons
    assert "reference_timestamp_stale" in result.reasons


def test_stamp_regression_resets_history_and_never_false_passes():
    monitor = MotionConsistencyMonitor(pose(), config())
    add_pair(monitor, 10.0, pose(), pose())
    add_pair(monitor, 11.0, pose(0.1), pose(0.1))
    monitor.add_lio_imu_pose(TimedPose(9.0, pose()), 11.1)
    result = monitor.evaluate(11.1)
    assert result.state == "INVALID"
    assert not result.consistent
    assert "lio_stamp_regression" in result.reasons
