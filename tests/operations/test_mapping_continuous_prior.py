"""Continuous mapping uses real samples without startup stillness or gap extrapolation."""
from collections import deque
import copy
import math
import threading

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from wc_runtime import mapping_prior as m
from test_mapping_prior import config, imu, wheel, cloud, tmp_path


def continuous(**policy):
    cfg = config()
    cfg.update(continuous_mapping=True, policy=policy)
    p = m.MotionPrior(cfg, 'synthetic', clock=lambda: 100000.)
    p.enable_startup_recovery(0.)
    return p


def sources(p, sequence, stamp, *, raw=100, gyro=(0, 0, 0)):
    imu(p, sequence, stamp, gyro=gyro)
    p.add_wheel(wheel(sequence, stamp, -raw, raw))


def test_first_moving_samples_initialize_only_configured_axes_without_gravity_or_bias_fit(monkeypatch):
    cfg = config(); cfg['continuous_mapping'] = True
    rotation = Rotation.from_euler('xyz', [.3, -.2, .1]).as_matrix()
    cfg['T_base_axle'] = np.eye(4).tolist()
    for index in range(3):
        cfg['T_base_axle'][index][:3] = rotation[index].tolist()
    monkeypatch.setattr(m, 'estimate_gravity_level', lambda *args, **kwargs: pytest.fail('moving startup cannot fit gravity'))
    p = m.MotionPrior(cfg, 'synthetic', clock=lambda: 1000.)
    p.enable_startup_recovery(0.)
    imu(p, 0, 1_000_000_000, gyro=(.2, -.1, .4), accel=(5., 1., 8.))
    assert p.initialization is None
    p.add_wheel(wheel(0, 1_000_000_000, -100, 100))
    assert p.initialization is not None and p.startup_deadline is None
    assert np.allclose(p.L, rotation.T) and np.allclose(p.T_reference_axle[:3, :3], np.eye(3))
    assert p.initialization['gravity'] is None and p.initialization['gravity_estimated'] is False
    assert p.initialization['gyro_bias_status'] == 'UNKNOWN_UNESTIMATED_ZERO_CANDIDATE'
    assert np.array_equal(p.bias, np.zeros(3))
    assert p.initialization['time_valid'] is p.initialization['formal_odometry_eligible'] is False
    assert p.imu[0]['stamp'] == p.wheel[0]['stamp'] == 1_000_000_000


@pytest.mark.parametrize('mode', ['left', 'right', 'all'])
def test_moving_bootstrap_mount_axes_and_visual_scene_share_one_reference(mode):
    from wc_runtime.mapping_input import mount_transforms
    from wc_runtime.mapping_visuals import build_scene
    from test_mapping_visuals import fixture
    display, _ = fixture(mode)
    display.update(session_id='synthetic', continuous_mapping=True)
    r_axle_imu = Rotation.from_euler('xyz', [.12, -.23, .31]).as_matrix()
    display['imu_mount']['R_axle_imu'] = r_axle_imu.tolist()
    for side, angles in (('left', [.3, -.2, .1]), ('right', [-.15, .25, -.2])):
        display['mounts'][side]['R_axle_lidar'] = Rotation.from_euler('xyz', angles).as_matrix().tolist()
    geometry = mount_transforms(display, None)
    cfg = config()
    cfg.update(continuous_mapping=True, **{key: geometry[key] for key in ('base_frame', 'R_base_imu', 'T_base_axle')})
    prior = m.MotionPrior(cfg, 'synthetic', clock=lambda: 1000.)
    gyro_imu = r_axle_imu.T @ [0., 0., .7]
    imu(prior, 0, 1_000_000_000, gyro=gyro_imu, accel=(4., 2., 9.))
    prior.add_wheel(wheel(0, 1_000_000_000, -100, 100))
    initialization = prior.initialization
    assert initialization['gravity_input_base_frame'] == geometry['base_frame']
    assert initialization['gravity'] is None and initialization['gravity_estimated'] is False
    assert np.allclose(prior.R_reference_imu, r_axle_imu, atol=1e-12)
    selected = 'right' if mode == 'right' else 'left'
    offset = np.asarray(display['mounts'][selected]['t_axle_lidar_m'])
    assert np.allclose(prior.T_reference_axle[:3, :3], np.eye(3), atol=1e-12)
    assert np.allclose(prior.T_reference_axle[:3, 3], -offset, atol=1e-12)
    linear, angular = prior._twist({'gyro': gyro_imu}, {'v': .1})
    assert np.allclose(angular, [0, 0, .7], atol=1e-12)
    assert np.allclose(linear, [.1, 0, 0]+np.cross(angular, offset), atol=1e-12)
    scene = build_scene(display, initialization)
    sensor_id = 7 if selected == 'right' else 6
    marker = next(row for row in scene['markers'] if row['id'] == sensor_id)
    assert np.allclose(marker['position_m'], [0, 0, 0], atol=1e-12)
    assert scene['ground_height_validated'] is False


@pytest.mark.parametrize('first', ['imu', 'wheel'])
def test_sources_may_arrive_after_minutes_and_stale_callback_ages_do_not_close_session(first):
    p = continuous()
    if first == 'imu': imu(p, 0, 1_000_000_000)
    else: p.add_wheel(wheel(0, 1_000_000_000))
    ages = m.check_source_freshness(p, {'imu': 1., 'wheel': 1., 'cloud': None}, 600.)
    assert p.failure is None and p.initialization is None and p.startup_deadline is None
    assert ages['latest_source_age_s']['imu'] == 599.
    if first == 'imu': p.add_wheel(wheel(0, 60_000_000_000, -100, 100))
    else: imu(p, 0, 60_000_000_000, gyro=(0, 0, .5))
    assert p.initialization['stamp_ns'] == 60_000_000_000
    with pytest.raises(m.NotReady, match='watermarks'):
        p.pose_at(60_000_000_000)
    if first == 'imu': imu(p, 1, 60_000_000_000)
    else: p.add_wheel(wheel(1, 60_000_000_000, -100, 100))
    assert np.array_equal(p.pose_at(60_000_000_000), np.eye(4))
    assert p.coverage_report()['motion_coverage_complete'] is False and p.failure is None


def test_long_gap_holds_pose_then_actual_new_samples_resume_without_fake_displacement():
    p = continuous()
    for sequence in range(4):
        sources(p, sequence, 1_000_000_000+sequence*100_000_000)
    before = p.pose_at(1_300_000_000)
    p.record_forwarded(1_300_000_000, before)
    sources(p, 4, 61_300_000_000)
    assert np.allclose(p.pose_at(61_300_000_000), before, atol=1e-12)
    sources(p, 5, 61_400_000_000)
    after = p.pose_at(61_400_000_000)
    speed = 100*.112*2*math.pi*.165/60
    assert after[0, 3]-before[0, 3] == pytest.approx(speed*.1)
    assert p.failure is None and p.forwarded == 1
    assert p.coverage_gap_count >= 2
    assert all(not row['motion_integrated_across_gap'] for row in p.coverage_events)
    report = p.sample_report(61_400_000_000, after)
    assert report['imu_stamp_ns'] == report['wheel_stamp_ns'] == 61_400_000_000
    assert report['motion_coverage_complete'] is False


@pytest.mark.parametrize('silent', ['imu', 'wheel'])
def test_missing_peer_and_absent_clouds_keep_history_bounded_and_restore_from_real_samples(silent):
    p = continuous(max_imu_samples=12, max_wheel_samples=12, history_s=.3)
    sources(p, 0, 1_000_000_000)
    sources(p, 1, 1_100_000_000)
    last_pose = p.pose_at(1_100_000_000)
    for index in range(2, 502):
        stamp = 1_000_000_000+index*100_000_000
        if silent == 'imu': p.add_wheel(wheel(index, stamp, -100, 100))
        else: imu(p, index, stamp)
        assert len(p.imu) <= 12 and len(p.wheel) <= 12 and len(p.checkpoints) <= 2
    assert p.forwarded == 0 and p.failure is None
    assert p.history_discarded_samples['wheel' if silent == 'imu' else 'imu'] > 400
    assert len(p.coverage_events) <= 64
    with pytest.raises(m.DroppedCloud): p.pose_at(1_100_000_000)
    resume_stamp = 51_100_000_000
    if silent == 'imu': imu(p, 2, resume_stamp)
    else: p.add_wheel(wheel(2, resume_stamp, -100, 100))
    assert np.allclose(p.pose_at(resume_stamp), last_pose, atol=1e-12)


def test_continuous_dense_history_retires_checkpoints_without_losing_observed_motion():
    p = continuous(max_imu_samples=10, max_wheel_samples=10, history_s=.3)
    for index in range(501):
        sources(p, index, 1_000_000_000+index*100_000_000)
        assert len(p.imu) <= 10 and len(p.wheel) <= 10 and len(p.checkpoints) <= 2
    speed = 100*.112*2*math.pi*.165/60
    assert p.pose_at(51_000_000_000)[0, 3] == pytest.approx(speed*50.)
    assert p.coverage_report()['motion_coverage_complete'] is True


def test_old_pending_clouds_are_dropped_and_later_covered_scan_recovers_after_source_gap():
    p = continuous(max_cloud_queue=2, history_s=.5)
    sources(p, 0, 1_000_000_000)
    queued, counters = deque(), {'warmup_clouds_dropped': 0}
    for stamp in (1_100_000_000, 1_200_000_000, 1_300_000_000):
        m.enqueue_cloud(p, queued, counters, 1., cloud([[1., 0, 0]], stamp=stamp))
    assert len(queued) == 2 and counters['cloud_queue_dropped'] == 1
    assert m.next_prepared_cloud(p, queued, counters, 600.) is None  # No causal watermark yet; no timer failure.
    sources(p, 1, 10_000_000_000)
    sources(p, 2, 10_100_000_000)
    assert m.next_prepared_cloud(p, queued, counters, 601.) is None
    assert not queued and counters['uncovered_clouds_dropped'] == 2
    m.enqueue_cloud(p, queued, counters, 602., cloud([[1., 0, 0]], stamp=10_100_000_000))
    transformed, pose, report = m.next_prepared_cloud(p, queued, counters, 603.)
    assert transformed.header.stamp.sec == 10 and report['stamp_ns'] == 10_100_000_000
    assert p.failure is None and report['motion_coverage_complete'] is False


def test_cloud_in_known_gap_is_dropped_without_fabricated_odometry():
    p = continuous(history_s=10.)
    sources(p, 0, 1_000_000_000)
    sources(p, 1, 3_000_000_000)
    with pytest.raises(m.DroppedCloud, match='unobserved'):
        m.prepare_cloud(cloud([[1., 0, 0]], stamp=2_000_000_000), p)
    assert np.array_equal(p.pose_at(2_000_000_000), np.eye(4))
    assert p.failure is None


def test_forward_wall_clock_jump_records_uncovered_interval_without_integrating_it():
    p = continuous(history_s=10.)
    sources(p, 0, 1_000_000_000)
    imu(p, 1, 2_100_000_000, monotonic_ns=1_100_000_000)
    record = wheel(1, 2_100_000_000, -100, 100)
    record['receive_monotonic_ns'] = 1_100_000_000
    p.add_wheel(record)
    assert p.failure is None and p.coverage_gap_count == 2
    assert np.array_equal(p.pose_at(2_100_000_000), np.eye(4))
    imu(p, 2, 2_200_000_000, monotonic_ns=1_200_000_000)
    record = wheel(2, 2_200_000_000, -100, 100)
    record['receive_monotonic_ns'] = 1_200_000_000
    p.add_wheel(record)
    assert p.pose_at(2_200_000_000)[0, 3] > 0
    with pytest.raises(m.PriorError, match='discontinuity'):
        imu(p, 3, 2_150_000_000, monotonic_ns=1_300_000_000)


@pytest.mark.parametrize('invalid', ['identity', 'negative_time', 'bad_acceleration', 'bad_wheel_identity'])
def test_continuous_mode_preserves_malformed_identity_and_backwards_time_rejection(invalid):
    p = continuous(); sources(p, 0, 1_000_000_000)
    with pytest.raises(m.PriorError):
        if invalid == 'identity': imu(p, 1, 1_100_000_000, sensor_id='other')
        elif invalid == 'negative_time': imu(p, 1, 900_000_000)
        elif invalid == 'bad_acceleration': imu(p, 1, 1_100_000_000, accel=(float('nan'), 0, 0))
        else:
            record = wheel(1, 1_100_000_000); record['device_id'] = 'other'; p.add_wheel(record)
    assert p.failure


def test_wait_config_zero_can_outlast_old_limit_and_exit_after_real_config_arrives(tmp_path, monkeypatch):
    config_path = tmp_path/'late.json'
    now, calls = [0.], [0]
    monkeypatch.setattr(m.time, 'monotonic', lambda: now[0])
    def wait(_):
        now[0] += 60.
        calls[0] += 1
        if calls[0] == 3: config_path.write_text('{}')
    monkeypatch.setattr(m.time, 'sleep', wait)
    assert m._wait_for_config(config_path, 0) is True and now[0] == 180.


def test_journal_capacity_probe_allows_backpressure_without_discarding_accepted_guesses(tmp_path):
    entered, release = threading.Event(), threading.Event()
    def blocked_write(path, value):
        entered.set(); assert release.wait(3)
        m._write_json(path, value)
    journal = m.AsyncPriorJournal(tmp_path, max_pending=1, write_json=blocked_write)
    try:
        journal.initialization({'fixture': True}); assert entered.wait(1)
        assert journal.has_capacity()
        journal.guess({'stamp_ns': 1})
        assert journal.has_capacity() is False and journal.snapshot()['failure'] is None
    finally:
        release.set(); journal.close(2)


def test_continuous_flag_is_explicit_and_does_not_change_strict_default():
    assert m.validate_config(config())['continuous_mapping'] is False
    bad = copy.deepcopy(config()); bad['continuous_mapping'] = 'true'
    with pytest.raises(m.PriorError, match='boolean'): m.validate_config(bad)
