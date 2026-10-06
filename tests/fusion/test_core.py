import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from wc_fusion import DualFusion, FusionError, Gate, MotionHistory, Policy, transform


def make_fusion(**kwargs):
    t = np.eye(4)
    t[1, 3] = -0.6
    calibration = dict(calibration_id='synthetic-cal', sensor_ids={'left': 'L', 'right': 'R'},
                       T_left_right=t, status='CANDIDATE', source_mode='synthetic')
    return DualFusion('test', {'left': 'L', 'right': 'R'}, calibration,
                      {'left': 'cl', 'right': 'cr'}, source_mode='synthetic', **kwargs)


def frame(side, seq=1, stamp=1_000_000_000, epoch='boot1'):
    # Independent visible structures, not a copied cloud with another frame name.
    rng = np.random.default_rng(17 if side == 'left' else 28)
    points = rng.uniform(1, 3, (40, 3))
    points[:, 0 if side == 'left' else 1] = 2
    return dict(session_id='test', side=side, sensor_id='L' if side == 'left' else 'R',
                stream_epoch=epoch, frame_sequence=seq, points=points,
                common_time_ns=stamp, clock_model_id='cl' if side == 'left' else 'cr',
                time_valid=True, uncertainty_ns=100_000, time_source='synthetic',
                host_monotonic_ns=100_000_000, frame_id='lidar_' + side)


def bundle(fusion=None):
    f = fusion or make_fusion()
    assert f.push(frame('left'), 100_000_000) is None
    return f.push(frame('right'), 100_000_000)


def test_dual_transform_original_origins_and_ownership():
    f = make_fusion()
    l, r = frame('left'), frame('right')
    l_original = l['points'].copy()
    assert f.push(l, 100_000_000) is None
    l['points'][:] = 0
    b = f.push(r, 100_000_000)
    np.testing.assert_allclose(b['points'][:40], l_original)
    np.testing.assert_allclose(b['points'][40:], r['points'] + [0, -0.6, 0])
    assert b['source_counts'] == {'left': 40, 'right': 40}
    assert set(b['source_codes']) == {1, 2}
    np.testing.assert_allclose(b['observations'][1]['origin_in_ref'], [0, -0.6, 0])
    assert b['observations'][0]['raw_key'] != b['observations'][1]['raw_key']


def test_wrong_transform_direction_is_detectable():
    f = make_fusion()
    b = bundle(f)
    wrong = transform(np.linalg.inv(f.extrinsics['right']), frame('right')['points'])
    assert np.max(np.linalg.norm(wrong - b['points'][40:], axis=1)) > 1.19


def test_exact_once_and_bounded_queues():
    f = make_fusion()
    b = bundle(f)
    with pytest.raises(FusionError, match='DUPLICATE'):
        f.push(frame('left'), 100_000_000)
    for seq in range(2, 200):
        f.push(frame('left', seq, 1_000_000_000 + seq * 100_000_000), 100_000_000)
    assert len(f.queues['left']) == f.policy.max_queue
    assert len(f.seen) <= f.policy.max_queue * 16
    assert f.counts['BUNDLE_DUAL'] == 1
    assert f.counts['QUEUE_OVERFLOW'] > 0


@pytest.mark.parametrize('mutate,reason', [
    (lambda x: x.update(uncertainty_ns=None), 'TIME_UNVALIDATED'),
    (lambda x: x.update(sensor_id='L'), 'SOURCE_IDENTITY'),
    (lambda x: x.update(clock_model_id='wrong'), 'CLOCK_MODEL'),
    (lambda x: x.update(points=np.full((50, 3), np.nan)), 'TOO_FEW'),
    (lambda x: x.update(common_time_ns=1.0), 'INTEGER_NS'),
])
def test_invalid_source_latches_without_single_fallback(mutate, reason):
    f = make_fusion()
    f.push(frame('left'), 100_000_000)
    r = frame('right')
    mutate(r)
    with pytest.raises(FusionError, match=reason):
        f.push(r, 100_000_000)
    assert f.bundle_counter == 0 and f.latched_reason
    with pytest.raises(FusionError, match='PAUSED'):
        f.push(frame('right', 2), 100_000_000)


def test_epoch_change_requires_explicit_continuous_resume():
    f = make_fusion()
    bundle(f)
    with pytest.raises(FusionError, match='EPOCH_CHANGED'):
        f.push(frame('right', 0, epoch='boot2'), 100_000_000)
    with pytest.raises(FusionError, match='PRECONDITIONS'):
        f.resume(explicit=False, stable=True, continuity_verified=True,
                 calibration_id='synthetic-cal', clock_model_ids=f.clock_model_ids)
    assert f.latched_reason


def test_nanosecond_precision_and_latest_reference():
    f = make_fusion()
    t = 1_789_100_000_000_000_000
    f.push(frame('left', stamp=t), 100_000_000)
    b = f.push(frame('right', stamp=t + 7), 100_000_000)
    assert b['t_ref_ns'] == t + 7
    assert b['observations'][1]['stamp_ns'] - b['observations'][0]['stamp_ns'] == 7


def test_prediction_includes_translation_rotation_and_lever_arm():
    policy = Policy(max_time_error_m=0.004, prediction_error_bound_m=0.001)
    f = make_fusion(policy=policy)
    p0, p1 = np.eye(4), np.eye(4)
    p1[:3, 3] = [0.01, 0, 0]
    p1[:3, :3] = Rotation.from_rotvec([0, 0, 0.01]).as_matrix()
    f.history.add(800_000_000, p0, 'odom1')
    f.history.add(900_000_000, p1, 'odom1')
    l, r = frame('left', stamp=990_000_000), frame('right', stamp=1_000_000_000)
    f.push(l, 100_000_000)
    b = f.push(r, 100_000_000)
    expected = np.linalg.inv(f.history.predict(1_000_000_000)) @ f.history.predict(990_000_000)
    np.testing.assert_allclose(b['points'][:40], transform(expected, l['points']))
    assert b['compensation_mode'] == 'history_constant_velocity'
    with pytest.raises(FusionError, match='EXPIRED'):
        f.history.predict(2_000_000_000)


def test_no_prediction_never_means_zero_motion():
    f = make_fusion(policy=Policy(max_time_error_m=0.004))
    f.push(frame('left'), 100_000_000)
    with pytest.raises(FusionError, match='NEEDS_COMPLETED_HISTORY'):
        f.push(frame('right', stamp=1_010_000_000), 100_000_000)


def assessment(b, **changes):
    result = dict(session_id='test', bundle_id=b['bundle_id'], stamp_ns=b['t_ref_ns'],
                  tracking_valid=True, T_odom_rig=np.eye(4), odom_epoch='odom1',
                  actual_source_participation=b['source_counts'], residual=None, inliers=None)
    result.update(changes)
    return result


def test_gate_id_time_and_failure_latch():
    b = bundle()
    gate = Gate('test')
    gate.submit(b, 1)
    with pytest.raises(FusionError, match='ID_TIME_MISMATCH'):
        gate.assess(assessment(b, stamp_ns=b['t_ref_ns'] + 1), 2)
    assert gate.accepted == 0
    with pytest.raises(FusionError, match='PAUSED'):
        gate.submit(b, 3)


def test_gate_provenance_and_expiry():
    b = bundle()
    gate = Gate('test')
    gate.submit(b, 1)
    accepted = gate.assess(assessment(b), 2)
    assert gate.accepted == 1
    assert accepted['assessment']['inliers'] is None
    gate = Gate('test', timeout_ns=5)
    gate.submit(b, 1)
    assert not gate.expire(7)


def test_synthetic_calibration_cannot_enter_live():
    with pytest.raises(FusionError, match='LIVE_CALIBRATION'):
        DualFusion('live', {'left': 'L', 'right': 'R'},
                   dict(sensor_ids={'left': 'L', 'right': 'R'}, T_left_right=np.eye(4),
                        status='VALIDATED', source_mode='synthetic'),
                   {'left': 'cl', 'right': 'cr'})


def test_missing_source_watchdog_latches_without_more_callbacks():
    f = make_fusion()
    f.push(frame('left'), 100_000_000)
    assert not f.check_freshness(500_000_001)
    assert f.latched_reason.startswith('SOURCE_TIMEOUT')


@pytest.mark.parametrize('reordered_side', ['left', 'right'])
def test_allowed_reordering_preserves_latest_source_freshness(reordered_side):
    f = make_fusion(policy=Policy(max_age_ns=1_000_000_000))
    first = bundle(f)  # Both sources acquired/delivered sequence 1 at 0.1 s.
    assert first['bundle_id'] == 1
    other_side = 'right' if reordered_side == 'left' else 'left'

    def sample(side, sequence, acquired):
        # One monotonic/common-time offset for all samples; only delivery is
        # reordered. Sequence 2 was never delivered before sequence 3.
        result = frame(side, seq=sequence, stamp=acquired + 900_000_000)
        result['host_monotonic_ns'] = acquired
        return result

    # The first side receives 3 before the unseen 2; both pass _check's
    # actual bounded-reordering, identity, sequence and individual age rules.
    assert f.push(sample(reordered_side, 3, 900_000_000), 900_000_000) is None
    assert f.push(sample(reordered_side, 2, 500_000_000), 1_000_000_000) is None
    second = f.push(sample(other_side, 2, 500_000_000), 1_000_000_000)
    third = f.push(sample(other_side, 3, 900_000_000), 1_000_000_000)
    assert [second['t_ref_ns'], third['t_ref_ns']] == [1_400_000_000, 1_800_000_000]
    assert [second['bundle_id'], third['bundle_id']] == [2, 3]
    assert [observation['raw_key'] for observation in second['observations']] == ['L/boot1/2', 'R/boot1/2']
    assert [observation['raw_key'] for observation in third['observations']] == ['L/boot1/3', 'R/boot1/3']
    assert all(not queue for queue in f.queues.values())

    # Original code times out the reordered side using acquisition 0.5 s,
    # despite both sides having valid sequence 3 acquired at 0.9 s.
    assert f.check_freshness(1_500_000_001)
    assert f.latched_reason is None
    assert f.check_freshness(1_900_000_000)  # Exact existing deadline remains valid.
    assert not f.check_freshness(1_900_000_001)
    assert f.latched_reason == 'SOURCE_TIMEOUT:left'
    # Delivery was 1.0 s: timing out at 1.9 s also proves that the fix did
    # not replace source timestamps with callback arrival timestamps.


@pytest.mark.parametrize('acquired', [0, 1_100_000_001])
def test_freshness_highwater_does_not_admit_stale_or_future_late_frame(acquired):
    f = make_fusion(policy=Policy(max_age_ns=1_000_000_000))
    bundle(f)
    latest = frame('left', seq=3, stamp=1_800_000_000)
    latest['host_monotonic_ns'] = 900_000_000
    assert f.push(latest, 900_000_000) is None
    late = frame('left', seq=2, stamp=1_400_000_000)
    late['host_monotonic_ns'] = acquired
    with pytest.raises(FusionError, match='SOURCE_STALE_OR_MONOTONIC_DOMAIN_MISMATCH'):
        f.push(late, 1_100_000_000)
    assert f.latched_reason == 'SOURCE_STALE_OR_MONOTONIC_DOMAIN_MISMATCH'
    assert f.bundle_counter == 1
    assert all(not queue for queue in f.queues.values())


def test_same_epoch_new_sequence_time_backwards_is_not_reordering():
    f = make_fusion()
    f.push(frame('left'), 100_000_000)
    with pytest.raises(FusionError, match='SOURCE_TIME_JUMP'):
        f.push(frame('left', seq=2, stamp=999_000_000), 100_000_000)


def test_gate_rejects_geometrically_valid_but_impossible_pose_jump():
    f, g = make_fusion(), Gate('test')
    b = bundle(f)
    g.submit(b, 1)
    g.assess(assessment(b), 2)
    f.push(frame('left', seq=2, stamp=1_100_000_000), 100_000_000)
    b2 = f.push(frame('right', seq=2, stamp=1_100_000_000), 100_000_000)
    g.submit(b2, 3)
    impossible = np.eye(4)
    impossible[0, 3] = 100
    with pytest.raises(FusionError, match='ODOM_MOTION_JUMP'):
        g.assess(assessment(b2, T_odom_rig=impossible), 4)
    assert g.accepted == 1


def test_resume_clears_old_timeout_but_keeps_pose_continuity():
    g = Gate('test', timeout_ns=5)
    b = bundle()
    g.submit(b, 1)
    g.assess(assessment(b), 2)
    g.pause('EXPLICIT_PAUSE')
    g.resume(explicit=True, stable=True, continuity_verified=True, odom_epoch='odom1')
    assert g.expire(1000)
    assert g.last_pose is not None and g.last_stamp_ns == b['t_ref_ns']


def test_prediction_motion_bound_cannot_exceed_time_budget_assumption():
    p = Policy(velocity_bound_mps=0.2, max_motion_speed_mps=0.5)
    history = MotionHistory(p)
    first, second = np.eye(4), np.eye(4)
    second[0, 3] = 0.04
    history.add(0, first, 'epoch')
    history.add(100_000_000, second, 'epoch')
    with pytest.raises(FusionError, match='SPEED_EXCEEDED'):
        history.predict(110_000_000)
