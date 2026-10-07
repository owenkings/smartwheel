"""Synthetic geometry/timing checks, never hardware accuracy evidence."""
import copy
import json
import math
from pathlib import Path
import signal
import shutil
import struct
import threading
import uuid
from types import SimpleNamespace as NS

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from wc_motion.feedback_transport import QUERY
from wc_motion.protocol import crc16
from wc_runtime.mapping_prior import (MotionPrior, NotReady, PriorError,
    integrate_body_twist, prepare_cloud, validate_config, _owned_output)
from wc_runtime import mapping_prior as prior_module
from wc_sensors.pointcloud import decode_pointcloud2


@pytest.fixture
def tmp_path():
    parent = Path(__file__).absolute().parent
    path = parent/('.mapping_prior_test_'+uuid.uuid4().hex)
    path.mkdir()
    try:
        yield path
    finally:
        resolved = path.resolve()
        assert resolved.parent == parent and resolved.name.startswith('.mapping_prior_test_')
        shutil.rmtree(resolved)


def config():
    return {'schema_version': 1, 'status': 'EXPERIMENT', 'source_mode': 'real',
            'base_frame': 'lidar_left', 'reference_frame': 'mapping_reference', 'guess_frame_id': 'prior_odom',
            'imu_sensor_id': 'H30-0000000015', 'R_base_imu': np.eye(3).tolist(), 'T_base_axle': np.eye(4).tolist(),
            'wheel_candidate': json.loads((Path(__file__).resolve().parents[2]/'config/wheel_history_calibration.json').read_text()),
            'input_cloud_topic': '/wc_mapping/app/input_cloud', 'output_cloud_topic': '/wc_mapping/app/scan_cloud',
            'private_tf_topic': '/wc_mapping/app/tf'}


def authority_weights():
    return {'status': 'UNVALIDATED_MODEL_WEIGHTS',
            'pose_diagonal': [.25, .25, 1., .09, .09, .09],
            'twist_diagonal': [.04, .25, .25, .01, .01, .04]}


def test_authority_mode_requires_explicit_weights_without_changing_legacy_policy():
    legacy = validate_config(config())
    assert legacy['odometry_source'] == 'icp' and 'wheel_imu_covariance' not in legacy
    cfg = config(); cfg['odometry_source'] = 'wheel_imu'
    with pytest.raises(PriorError, match='UNVALIDATED_MODEL_WEIGHTS'):
        validate_config(cfg)
    cfg['wheel_imu_covariance'] = authority_weights()
    authority = validate_config(cfg)
    assert authority['policy'] == legacy['policy']
    assert authority['wheel_imu_covariance'] == authority_weights()
    assert authority['T_base_axle'] == legacy['T_base_axle']
    cfg['prior_odom_topic'] = '/wc_mapping/app/odom'
    with pytest.raises(PriorError, match='distinct'):
        validate_config(cfg)


def test_unknown_odometry_source_is_rejected():
    cfg = config(); cfg['odometry_source'] = 'automatic_fallback'
    with pytest.raises(PriorError, match='odometry_source'):
        validate_config(cfg)


def wheel(seq, stamp, left=0, right=0):
    payload = b'\x01\x03\x04'+struct.pack('>HH', left & 65535, right & 65535)
    return {'schema': 'wc_wheel_feedback_v1', 'device_id': json.loads((Path(__file__).resolve().parents[2]/'config/wheel_feedback_current.json').read_text())['device_id'],
            'status': 'RESPONSE_VALID', 'request_hex': QUERY.hex(), 'response_hex': (payload+crc16(payload)).hex(),
            'stamp_ns': stamp, 'receive_monotonic_ns': stamp, 'sequence': seq, 'stream_epoch': 'wheel-fixture',
            'time_valid': False, 'time_source': 'arrival_only', 'uncertainty_ns': None}


def imu(prior, seq, stamp, gyro=(0, 0, 0), accel=(0, 0, 9.80665), **extra):
    data = dict(stamp_ns=stamp, monotonic_ns=stamp, acceleration=accel, angular_velocity=gyro,
                sensor_id='H30-0000000015', session_id='synthetic', stream_epoch='imu-fixture', sequence=seq)
    data.update(extra)
    prior.add_imu(**data)


def initialized(cfg=None, accel=(0, 0, 9.80665), bias=(0, 0, 0)):
    p = MotionPrior(cfg or config(), 'synthetic')
    for i in range(201):
        stamp = 1000000000+i*10000000
        imu(p, i, stamp, gyro=bias, accel=accel)
        if i % 10 == 0:
            p.add_wheel(wheel(i//10, stamp))
    assert p.initialization is not None
    return p


def connecting_prior():
    now = [1_000_000_000]
    p = MotionPrior(config(), 'synthetic', clock=lambda: now[0]*1e-9)
    p.enable_startup_recovery(1.)
    return p, now


def connecting_tick(p, now, index, *, with_wheel=True):
    now[0] = 1_000_000_000+index*10_000_000
    imu(p, index, now[0])
    if with_wheel and index % 10 == 0:
        p.add_wheel(wheel(index//10, now[0]))


def test_connecting_delayed_wheel_and_imu_pause_require_new_full_static_window():
    p, now = connecting_prior()
    for index in range(81):
        connecting_tick(p, now, index, with_wheel=False)
    now[0] = 2_200_000_000
    prior_module.check_source_freshness(p, {'imu': 1.8, 'wheel': None, 'cloud': None}, 2.2)
    assert not p.imu and not p.wheel and p.last_seen['imu']['sequence'] == 80
    assert p.startup_report()['discarded_samples'] == {'imu': 81, 'wheel': 0}
    # IMU resumes before the independently discovered wheel subscription.
    for index in range(121, 350):
        connecting_tick(p, now, index, with_wheel=index >= 150)
    assert p.initialization is None  # Only 1.99 s since both streams began.
    connecting_tick(p, now, 350)
    assert p.initialization['window_start_ns'] == 2_500_000_000
    assert p.initialization['window_end_ns'] == 4_500_000_000
    assert p.initialization['imu_sequence_start'] == 150
    assert p.startup_report()['discarded_candidates'] == 1
    assert p.startup_report()['recovery_closed'] is True
    assert p.forwarded == 0 and p.failure is None


@pytest.mark.parametrize('source', ['imu', 'wheel'])
def test_connecting_positive_sample_gap_discards_both_source_candidates(source):
    p, now = connecting_prior()
    for index in range(51):
        connecting_tick(p, now, index)
    now[0] = 2_100_000_000
    if source == 'imu':
        imu(p, 110, now[0])
    else:
        p.add_wheel(wheel(11, now[0]))
    # The peer's original time is stale as well, so it cannot seed a new window.
    assert not p.imu and not p.wheel
    report = p.startup_report()
    assert report['discarded_samples']['imu'] >= 51
    assert report['discarded_samples']['wheel'] >= 6
    assert any('gap' in reason for reason in report['reasons'])
    assert p.initialization is None and p.failure is None


def test_connecting_wheel_silence_discards_entire_window_without_erasing_baselines():
    p, now = connecting_prior()
    for index in range(81):
        connecting_tick(p, now, index, with_wheel=index <= 20)
    # Wheel last seen at 1.2 s; continuous IMU does not make it fresh.
    assert not p.imu and not p.wheel
    assert p.last_seen['wheel']['sequence'] == 2
    ages = prior_module.source_ages(p, {'imu': 1.8, 'wheel': 1.2, 'cloud': None}, 1.8)
    assert ages['latest_producer_age_s']['wheel'] == pytest.approx(.6)
    assert ages['last_source_sequence']['wheel'] == 2
    assert p.initialization is None


def test_connecting_deadline_never_restarts_on_absence_or_candidate_discard():
    p, now = connecting_prior()
    connecting_tick(p, now, 0)
    for stamp in (2., 12., 30.99):
        now[0] = round(stamp*1e9)
        prior_module.check_source_freshness(p, {'imu': 1., 'wheel': 1., 'cloud': None}, stamp)
        assert p.startup_report()['deadline_monotonic_s'] == 31.
    now[0] = 31_000_000_001
    with pytest.raises(PriorError, match='startup timeout'):
        prior_module.check_source_freshness(p, {'imu': None, 'wheel': None, 'cloud': None}, now[0]*1e-9)
    with pytest.raises(PriorError, match='enabled once'):
        p.enable_startup_recovery(31.)


@pytest.mark.parametrize('source', ['imu', 'wheel'])
def test_connecting_recovery_is_closed_immediately_on_initialization(source):
    p, now = connecting_prior()
    for index in range(201):
        connecting_tick(p, now, index)
    assert p.initialization is not None and p.forwarded == 0
    now[0] = 3_600_000_000
    with pytest.raises(PriorError, match='gap'):
        if source == 'imu':
            imu(p, 260, now[0])
        else:
            p.add_wheel(wheel(21, now[0]))
    assert p.failure is not None and p.startup_discards == 0


def test_connecting_commit_seeds_strict_wheel_continuity_and_matches_default_estimator():
    p, now = connecting_prior()
    reference = initialized()
    for index in range(201):
        connecting_tick(p, now, index)
    assert p.wheel_decoder.previous['sequence'] == 20
    for index in range(201, 221):
        connecting_tick(p, now, index)
    extend(reference, end=220)
    assert p.wheel_decoder.previous['sequence'] == 22
    assert np.allclose(p.pose_at(3_200_000_000), reference.pose_at(3_200_000_000), atol=1e-12)
    # Even a quiet source with no new callback is fatal immediately after commit.
    now[0] = 3_451_000_000
    with pytest.raises(PriorError, match='imu source disconnected/stale'):
        prior_module.check_source_freshness(p, {'imu': 3.2, 'wheel': 3.2, 'cloud': None}, now[0]*1e-9)
    assert p.initialization is not None and p.startup_discards == 0


def test_connecting_stale_queued_window_never_initializes_or_rewrites_source_time():
    p, now = connecting_prior()
    now[0] = 4_000_000_000  # A fresh callback receipt cannot rehabilitate old samples.
    for index in range(201):
        stamp = 1_000_000_000+index*10_000_000
        imu(p, index, stamp)
        if index % 10 == 0:
            p.add_wheel(wheel(index//10, stamp))
    assert p.initialization is None and not p.imu and not p.wheel
    assert p.startup_report()['discarded_samples'] == {'imu': 201, 'wheel': 21}
    assert p.last_seen['imu']['mono'] == 3_000_000_000
    assert p.last_seen['wheel']['stamp'] == 3_000_000_000


def test_connecting_fitting_delay_rechecks_original_age_before_commit(monkeypatch):
    p, now = connecting_prior()
    original = prior_module.estimate_gravity_level
    def delayed_fit(*args, **kwargs):
        result = original(*args, **kwargs)
        now[0] += 260_000_000
        return result
    monkeypatch.setattr(prior_module, 'estimate_gravity_level', delayed_fit)
    for index in range(201):
        connecting_tick(p, now, index)
    assert p.initialization is None and not p.imu and not p.wheel
    assert p.startup_discards == 1
    assert p.wheel_decoder.previous is None  # Strict decoder was never reset/seeded prematurely.


@pytest.mark.parametrize('change', [
    {'stream_epoch': 'new-epoch'}, {'sequence': 0}, {'stamp_ns': 999_999_999},
    {'monotonic_ns': 999_999_999}, {'stamp_ns': 1_400_000_000},
    {'sensor_id': 'unknown'}, {'angular_velocity': (4., 0., 0.)},
])
def test_connecting_imu_fatal_checks_survive_candidate_discard(change):
    p, now = connecting_prior()
    connecting_tick(p, now, 0)
    now[0] = 1_300_000_000
    prior_module.check_source_freshness(p, {'imu': 1., 'wheel': 1., 'cloud': None}, 1.3)
    data = dict(stamp_ns=now[0], monotonic_ns=now[0], acceleration=(0., 0., 9.80665),
                angular_velocity=(0., 0., 0.), sensor_id='H30-0000000015', session_id='synthetic',
                stream_epoch='imu-fixture', sequence=1)
    data.update(change)
    with pytest.raises(PriorError):
        p.add_imu(**data)
    assert p.failure is not None and p.last_seen['imu']['sequence'] == 0
    with pytest.raises(PriorError, match='FAILED'):
        imu(p, 2, now[0])


@pytest.mark.parametrize('change', [
    {'stream_epoch': 'new-epoch'}, {'sequence': 0}, {'stamp_ns': 999_999_999},
    {'receive_monotonic_ns': 999_999_999}, {'stamp_ns': 1_700_000_000},
    {'device_id': 'unknown'}, {'response_hex': '010304000000000000'},
])
def test_connecting_wheel_fatal_checks_survive_candidate_discard(change):
    p, now = connecting_prior()
    connecting_tick(p, now, 0)
    now[0] = 1_600_000_000
    prior_module.check_source_freshness(p, {'imu': 1., 'wheel': 1., 'cloud': None}, 1.6)
    record = wheel(1, now[0]); record.update(change)
    with pytest.raises(PriorError):
        p.add_wheel(record)
    assert p.failure is not None and p.last_seen['wheel']['sequence'] == 0
    assert p.wheel_decoder.previous is None and p.wheel_decoder.blocked is None


def test_connecting_future_source_time_is_fatal_even_without_a_candidate():
    p, now = connecting_prior()
    with pytest.raises(PriorError, match='stale/future'):
        imu(p, 0, now[0]+1)
    assert p.failure is not None and p.initialization is None


def extend(p, end=320, raw=0, gyro=(0, 0, 0), accel=(0, 0, 9.80665)):
    start = p.imu[-1]['sequence']+1
    for i in range(start, end+1):
        stamp = 1000000000+i*10000000
        imu(p, i, stamp, gyro=gyro, accel=accel)
        if i % 10 == 0:
            p.add_wheel(wheel(i//10, stamp, -raw, raw))


def cloud(xyz, stamp=3100000000, *, dual=False, offsets=None, bigendian=False):
    prefix = '>' if bigendian else '<'
    fields = [NS(name=n, offset=i*4, datatype=7, count=1) for i, n in enumerate(('x', 'y', 'z'))]
    step = 24 if dual else 16
    fields.append(NS(name='intensity', offset=12, datatype=7, count=1))
    if dual:
        # Deliberately unaligned FLOAT64 proves the decoder does not assume packed XYZ.
        fields[-1] = NS(name='source_code', offset=12, datatype=2, count=1)
        fields.append(NS(name='time_offset_s', offset=16, datatype=8, count=1))
    data = bytearray(step*len(xyz)+7)  # row padding preserved
    for i, point in enumerate(xyz):
        struct.pack_into(prefix+'fff', data, i*step, *point)
        if dual:
            data[i*step+12] = 1 if i == 0 else 2
            struct.pack_into(prefix+'d', data, i*step+16, offsets[i])
        else:
            struct.pack_into(prefix+'f', data, i*step+12, 17+i)
    data[-7:] = b'padding'
    sec, nano = divmod(stamp, 1000000000)
    return NS(header=NS(stamp=NS(sec=sec, nanosec=nano), frame_id='lidar_left'), fields=fields,
              width=len(xyz), height=1, point_step=step, row_step=len(data), is_bigendian=bigendian,
              is_dense=False, data=data)


@pytest.mark.parametrize('change', [ {'R_base_imu': None}, {'T_base_axle': None}, {'R_base_imu': np.diag([1, 1, -1]).tolist()},
    {'status': 'VALIDATED'}, {'time_valid': True}, {'base_frame': 'base_link'}, {'imu_sensor_id': 'unknown'},
    {'input_cloud_topic': '/scan'}, {'output_cloud_topic': '/wc_mapping/app/input_cloud'}])
def test_required_geometry_identity_and_private_frames(change):
    with pytest.raises((ValueError, TypeError)):
        validate_config({**config(), **change})


def test_gravity_level_bias_and_forward_model_preserve_geometric_definitions():
    cfg = config(); del cfg['T_base_axle']
    cfg.update(mount_model='forward_aligned_axle', same_forward=True, t_axle_base_m=[.34, .3, .52])
    tilt = Rotation.from_euler('xy', [10, -7], degrees=True).as_matrix()
    up = tilt.T @ np.array([0, 0, 9.80665])
    p = initialized(cfg, accel=up, bias=(.01, -.02, .005))
    assert np.allclose(p.L @ up, [0, 0, 9.80665], atol=1e-10)
    assert np.allclose(p.T_reference_axle[:3, :3], np.eye(3))
    assert np.allclose(p.T_reference_axle[:3, 3], [-.34, -.3, -.52])
    assert np.allclose(p.R_reference_imu, p.L)
    linear, angular = p._twist(p.imu[-1], p.wheel[-1])
    assert np.linalg.norm(angular) < 1e-15 and np.linalg.norm(linear) < 1e-15
    assert p.initialization['time_valid'] is False and p.initialization['formal_odometry_eligible'] is False
    assert p.initialization['mounting_basis'] == 'user_reported_forward_aligned_unvalidated'
    cfg.pop('t_axle_base_m')
    with pytest.raises(PriorError):
        MotionPrior(cfg, 'synthetic')


def test_initialization_needs_real_duration_and_both_wheels_static():
    p = MotionPrior(config(), 'synthetic')
    for i in range(201):
        stamp = 1000000000+i*10000000
        imu(p, i, stamp)
        if i % 10 == 0:
            p.add_wheel(wheel(i//10, stamp, 100, 100))  # zero mean, turning wheels
    assert p.initialization is None
    assert p.static_wait_reason == 'wheel_motion_during_gravity_window'
    with pytest.raises(NotReady):
        p.pose_at(3000000000)


def test_stationary_acceleration_and_gyro_magnitude_rejection():
    p = MotionPrior(config(), 'synthetic')
    for i in range(201):
        stamp = 1000000000+i*10000000
        imu(p, i, stamp, accel=(0, 0, 1))
        if i % 10 == 0:
            p.add_wheel(wheel(i//10, stamp))
    assert p.initialization is None
    assert 'NOT_NEAR_GRAVITY' in p.static_wait_reason
    with pytest.raises(PriorError, match='SI magnitude'):
        imu(p, 202, 3020000000, gyro=(0, 0, 90))
    with pytest.raises(PriorError, match='FAILED'):
        imu(p, 203, 3030000000)


def test_actual_wheel_speed_drives_translation_and_gyro_drives_rotation():
    p = initialized()
    extend(p, end=410, raw=100)
    pose = p.pose_at(5100000000)
    speed = 100*.112*2*math.pi*.165/60
    assert np.allclose(pose[:3, 3], [speed*2, 0, 0], atol=1e-10)  # speed starts at 3.1 s
    q = initialized()
    extend(q, end=301, gyro=(0, 0, 1))
    yaw = Rotation.from_matrix(q.pose_at(4000000000)[:3, :3]).as_rotvec()[2]
    assert yaw == pytest.approx(.99)  # gyro starts at 3.01 s
    assert np.allclose(q.pose_at(4000000000)[:3, 3], 0)


def test_axle_lever_arm_pure_spin_orbit_and_se3_translation():
    cfg = config(); cfg['T_base_axle'][0][3] = -.34
    p = initialized(cfg)
    v, w = p._twist({'gyro': np.array([0., 0., 1.])}, {'v': 0.})
    assert np.allclose(v, [0, .34, 0])
    pose = integrate_body_twist(np.eye(4), v, w, math.pi/2)
    assert np.allclose(pose[:3, 3], [-.34, .34, 0], atol=1e-12)
    assert np.allclose(pose[:3, :3] @ [1, 0, 0], [0, 1, 0], atol=1e-12)


def test_wheel_authority_keeps_causal_lever_arm_pose_and_exact_cloud_timestamp():
    from wc_runtime.mapping_odometry import adapt_prior_odometry
    cfg = config()
    cfg.update(odometry_source='wheel_imu', wheel_imu_covariance=authority_weights())
    cfg['T_base_axle'][0][3] = -.34
    prior = initialized(cfg)
    extend(prior, end=301, gyro=(0, 0, 1))
    source = cloud([[1., 2., 3.]], stamp=4_000_000_000)
    output_cloud, pose, report = prepare_cloud(source, prior)
    q = list(map(float, Rotation.from_matrix(pose[:3, :3]).as_quat()))
    diagnostic = NS(header=NS(stamp=copy.deepcopy(source.header.stamp), frame_id='prior_odom'),
        child_frame_id='mapping_reference',
        pose=NS(pose=NS(position=NS(**dict(zip('xyz', map(float, pose[:3, 3])))),
                       orientation=NS(**dict(zip('xyzw', q)))), covariance=[1e6]*36),
        twist=NS(twist=NS(linear=NS(**dict(zip('xyz', report['linear_velocity_reference_m_s']))),
                         angular=NS(**dict(zip('xyz', report['angular_velocity_reference_rad_s'])))),
                 covariance=[1e6]*36))
    odom = adapt_prior_odometry(diagnostic, cfg['wheel_imu_covariance'])
    # A sensor 0.34 m ahead of a rotating axle travels an arc even when the
    # axle's longitudinal velocity is zero; authority must keep this motion.
    assert odom.pose.pose.position.x == pytest.approx(.34*(math.cos(.99)-1))
    assert odom.pose.pose.position.y == pytest.approx(.34*math.sin(.99))
    assert odom.twist.twist.linear.y == pytest.approx(.34)
    assert odom.twist.twist.angular.z == pytest.approx(1.)
    assert odom.header.stamp == output_cloud.header.stamp == source.header.stamp
    assert odom.child_frame_id == output_cloud.header.frame_id == 'mapping_reference'
    assert np.array_equal(decode_pointcloud2(output_cloud), decode_pointcloud2(source))
    assert prior.initialization['formal_odometry_eligible'] is False
    assert prior.initialization['wheel_imu_covariance']['status'] == 'UNVALIDATED_MODEL_WEIGHTS'
    # Authority selection cannot soften a real post-initialization source gap.
    with pytest.raises(PriorError, match='IMU sample gap'):
        imu(prior, 500, 4_500_000_000)
    assert prior.failure


def test_causal_watermark_does_not_use_future_values_or_extrapolate():
    p = initialized()
    with pytest.raises(NotReady):
        p.pose_at(3050000000)
    extend(p, end=210, raw=100, gyro=(0, 0, 0))
    assert np.allclose(p.pose_at(3050000000), np.eye(4))  # next wheel speed at 3.1 is future
    with pytest.raises(NotReady):
        p.pose_at(3110000000)
    with pytest.raises(PriorError, match='predates'):
        p.pose_at(2990000000)


@pytest.mark.parametrize('change', [ {'stream_epoch': 'restarted'}, {'session_id': 'other'},
    {'sensor_id': 'wrong'}, {'monotonic_ns': 2900000000}, {'coordinate_convention': 'FLU'},
    {'common_time_valid': True}, {'monotonic_ns': 3100000000}])
def test_imu_discontinuities_latch_failure(change):
    p = initialized()
    with pytest.raises(PriorError):
        imu(p, 201, 3010000000, **change)
    assert p.failure


def test_repeated_host_stamp_preserves_sequence_and_does_not_invent_time():
    p = initialized()
    imu(p, 201, 3000000000)
    assert p.imu[-1]['stamp'] == p.imu[-2]['stamp']
    assert p.imu[-1]['sequence'] == 201


@pytest.mark.parametrize('bigendian', [False, True])
def test_cloud_rotation_preserves_count_invalid_bytes_fields_and_row_padding(bigendian):
    tilt = Rotation.from_euler('x', 12, degrees=True).as_matrix()
    p = initialized(accel=tilt.T @ [0, 0, 9.80665])
    extend(p, end=210)
    source = cloud([[1, 2, 3], [float('nan'), 4, 5]], bigendian=bigendian)
    original = bytes(source.data)
    result, pose, report = prepare_cloud(source, p)
    assert result.header.frame_id == 'mapping_reference'
    assert np.allclose(decode_pointcloud2(result)[0], p.L @ [1, 2, 3], atol=1e-6)
    assert bytes(source.data) == original
    assert bytes(result.data)[12:16] == original[12:16]
    assert bytes(result.data)[16:] == original[16:]
    assert report['finite_point_count'] == 1 and report['point_count'] == 2
    assert np.allclose(pose, np.eye(4))


def test_dual_arrival_compensation_matches_static_world_point():
    p = initialized(); extend(p, end=230, raw=100)
    speed = 100*.112*2*math.pi*.165/60
    source = cloud([[1, 0, 0], [1-speed*.05, 0, 0]], stamp=3200000000, dual=True, offsets=[-.05, 0.])
    result, pose, report = prepare_cloud(source, p)
    assert np.allclose(decode_pointcloud2(result)[0], decode_pointcloud2(result)[1], atol=1e-7)
    assert report['arrival_motion_compensated'] is True
    assert report['source_offsets_ns'] == [-50000000, 0]
    assert bytes(result.data)[12:24] == bytes(source.data)[12:24]
    assert pose[0, 3] == pytest.approx(speed*.1)


def test_all_mode_cannot_silently_forward_a_cloud_without_offsets():
    cfg = config(); cfg['require_dual_time_offsets'] = True
    p = initialized(cfg); extend(p, end=220)
    with pytest.raises(PriorError, match='all mode requires'):
        prepare_cloud(cloud([[1, 0, 0]]), p)


def test_explicit_imu_axes_map_gyro_and_axle_forward_before_integration():
    cfg = config()
    mount = Rotation.from_euler('x', 90, degrees=True).as_matrix()
    cfg['R_base_imu'] = mount.tolist()
    p = initialized(cfg, accel=mount.T @ [0, 0, 9.80665])
    assert np.allclose(p.L, np.eye(3), atol=1e-12)
    linear, angular = p._twist({'gyro': mount.T @ [0, 0, .3]}, {'v': .2})
    assert np.allclose(linear, [.2, 0, 0])
    assert np.allclose(angular, [0, 0, .3])


@pytest.mark.parametrize('offsets', [[-.051, 0.], [.01, 0.], [-.02, -.01], [float('nan'), 0.]])
def test_dual_offsets_reject_unknown_or_invalid_acquisition_assumptions(offsets):
    p = initialized(); extend(p, end=220)
    with pytest.raises(PriorError):
        prepare_cloud(cloud([[1, 0, 0], [1, 0, 0]], dual=True, offsets=offsets), p)


def test_history_pruning_retains_recent_offset_pose_and_bounds_memory():
    p = initialized()
    for end in range(210, 701, 10):
        extend(p, end=end, raw=10)
        stamp = 1000000000+end*10000000
        pose = p.pose_at(stamp)
        p.record_forwarded(stamp, pose)
        assert np.isfinite(p.pose_at(stamp-50000000)).all()
    assert len(p.imu) < 250 and len(p.wheel) < 30 and len(p.checkpoints) < 30
    with pytest.raises(PriorError, match='predates'):
        p.pose_at(3000000000)


def test_output_directory_is_exclusive(tmp_path):
    output = _owned_output(tmp_path/'prior')
    assert output.is_dir()
    with pytest.raises(FileExistsError):
        _owned_output(output)


@pytest.mark.parametrize('signum', [signal.SIGINT, signal.SIGTERM])
def test_stop_while_waiting_for_bootstrap_config_returns_zero_and_restores_handlers(tmp_path, monkeypatch, signum):
    before = {number: signal.getsignal(number) for number in (signal.SIGINT, signal.SIGTERM)}
    monkeypatch.setattr(prior_module.time, 'sleep', lambda _: signal.raise_signal(signum))
    assert prior_module.main(['--config', str(tmp_path/'missing.json'), '--session-id', 'synthetic',
                              '--output-root', str(tmp_path/'unused')]) == 0
    assert not (tmp_path/'unused').exists()
    assert {number: signal.getsignal(number) for number in before} == before


def test_wait_for_config_keyboard_interrupt_is_clean_and_timeout_still_fails(tmp_path, monkeypatch):
    def interrupt(_):
        raise KeyboardInterrupt()
    monkeypatch.setattr(prior_module.time, 'sleep', interrupt)
    assert prior_module._wait_for_config(tmp_path/'missing.json', 15) is False
    assert prior_module._wait_for_config(tmp_path/'missing.json', 0) is False
    now = iter([1., 2.])
    monkeypatch.setattr(prior_module.time, 'monotonic', lambda: next(now))
    with pytest.raises(PriorError, match='timed out'):
        prior_module._wait_for_config(tmp_path/'missing.json', .1)


def test_blocked_disk_writer_does_not_block_motion_samples_and_drains_final_records(tmp_path):
    entered, release = threading.Event(), threading.Event()
    def blocked_write(path, value):
        if path.name == 'initialization.json':
            entered.set()
            assert release.wait(5)
        prior_module._write_json(path, value)
    p = initialized()
    journal = prior_module.AsyncPriorJournal(tmp_path, max_pending=4, write_json=blocked_write)
    try:
        journal.initialization(p.initialization)
        assert entered.wait(2)
        # No filesystem completion is possible, yet the ROS-side estimator can
        # consume fresh IMU/wheel observations and queue its actual predictions.
        extend(p, end=230, raw=100)
        for stamp_ns in (3100000000, 3200000000, 3300000000):
            pose = p.pose_at(stamp_ns)
            report = p.sample_report(stamp_ns, pose)
            journal.guess(report)
            report['T_prior_reference'][0][3] = 999  # queued data owns its snapshot
        for sequence in range(1000):
            journal.status({'status': 'RUNNING', 'sequence': sequence})
        assert p.imu[-1]['sequence'] == 230
        assert journal.snapshot()['pending_records'] == 3
        assert journal.snapshot()['pending_status'] is True
        assert journal.snapshot()['write_in_flight'] is True
        journal.status({'status': 'STOPPED', 'sequence': 1000})
    finally:
        release.set()
        journal.close(2)
    assert json.loads((tmp_path/'initialization.json').read_text())['stamp_ns'] == 3000000000
    guesses = [json.loads(row) for row in (tmp_path/'guesses.jsonl').read_text().splitlines()]
    assert [row['stamp_ns'] for row in guesses] == [3100000000, 3200000000, 3300000000]
    assert all(row['T_prior_reference'][0][3] != 999 for row in guesses)
    assert json.loads((tmp_path/'status.json').read_text()) == {'status': 'STOPPED', 'sequence': 1000}


def test_async_journal_queue_bound_fails_without_discarding_accepted_guesses(tmp_path):
    entered, release = threading.Event(), threading.Event()
    def blocked_write(path, value):
        entered.set()
        assert release.wait(5)
        prior_module._write_json(path, value)
    journal = prior_module.AsyncPriorJournal(tmp_path, max_pending=2, write_json=blocked_write)
    try:
        journal.initialization({'synthetic': True})
        assert entered.wait(2)
        journal.guess({'stamp_ns': 1})
        journal.guess({'stamp_ns': 2})
        with pytest.raises(PriorError, match='queue is full'):
            journal.guess({'stamp_ns': 3})
        assert journal.snapshot()['pending_records'] == 2
    finally:
        release.set()
        journal.close(2)
    assert [json.loads(x)['stamp_ns'] for x in (tmp_path/'guesses.jsonl').read_text().splitlines()] == [1, 2]


def test_async_journal_write_errors_are_visible_to_main_and_shutdown(tmp_path):
    def fail(*_):
        raise OSError('synthetic disk write failed')
    journal = prior_module.AsyncPriorJournal(tmp_path, write_json=fail)
    journal.initialization({'synthetic': True})
    journal.thread.join(2)
    assert not journal.thread.is_alive()
    assert journal.snapshot()['failure'] == 'synthetic disk write failed'
    assert journal.snapshot()['worker_exited'] is True and journal.snapshot()['drain_complete'] is False
    with pytest.raises(PriorError, match='disk write failed'):
        journal.check()
    with pytest.raises(PriorError, match='disk write failed'):
        journal.close(2)


def test_async_journal_shutdown_wait_is_bounded_and_cannot_claim_persistence(tmp_path):
    entered, release = threading.Event(), threading.Event()
    def blocked_write(path, value):
        entered.set()
        assert release.wait(5)
        prior_module._write_json(path, value)
    journal = prior_module.AsyncPriorJournal(tmp_path, write_json=blocked_write)
    try:
        journal.status({'status': 'STOPPED'})
        assert entered.wait(2)
        with pytest.raises(PriorError, match='persistence is incomplete'):
            journal.close(.01)
        state = journal.snapshot()
        assert state['worker_exited'] is False and state['drain_complete'] is False
        assert state['write_in_flight'] is True and state['shutdown_error']
    finally:
        release.set()
        with pytest.raises(PriorError, match='persistence is incomplete'):
            journal.close(2)
    # A late disk completion is distinguishable from meeting the stop budget.
    assert journal.snapshot()['worker_exited'] is True and journal.snapshot()['drain_complete'] is True
    assert journal.snapshot()['failure'] == journal.snapshot()['shutdown_error']


def test_prior_default_close_budget_waits_for_owner_to_drain_final_status(tmp_path, monkeypatch):
    entered, release = threading.Event(), threading.Event()
    writes, joins = [], []
    def write(path, value):
        writes.append((path.name, threading.current_thread().name))
        entered.set(); assert release.wait(3)
        prior_module._write_json(path, value)
    journal = prior_module.AsyncPriorJournal(tmp_path, write_json=write)
    journal.initialization({'synthetic': True})
    assert entered.wait(1)
    journal.guess({'stamp_ns': 123})
    journal.status({'status': 'STOPPED'})
    original_join = journal.thread.join
    def join(timeout):
        joins.append(timeout)
        release.set()  # Disk completes only once the caller starts its bounded wait.
        return original_join(timeout)
    monkeypatch.setattr(journal.thread, 'join', join)
    try:
        journal.close()
    finally:
        release.set(); original_join(2)
    assert joins == [20.] and prior_module.DEFAULT_POLICY['writer_close_timeout_s'] == 20.
    assert validate_config(config())['policy']['writer_close_timeout_s'] == 20.
    assert json.loads((tmp_path/'status.json').read_text()) == {'status': 'STOPPED'}
    assert json.loads((tmp_path/'guesses.jsonl').read_text()) == {'stamp_ns': 123}
    assert writes == [('initialization.json', 'mapping-prior-journal'), ('status.json', 'mapping-prior-journal')]
    state = journal.snapshot()
    assert state['drain_complete'] and state['worker_exited'] and not state['write_in_flight']
    assert state['pending_records'] == 0 and not state['pending_status'] and state['failure'] is None
    assert 'does not certify fsync' in state['storage_policy']


def freshness_fixture(stamp=1.8):
    sample = {'stamp': int(stamp*1e9), 'mono': int(stamp*1e9), 'sequence': 0}
    prior = NS(policy=copy.deepcopy(prior_module.DEFAULT_POLICY), imu=[dict(sample)], wheel=[dict(sample)])
    receipts = {'imu': stamp, 'wheel': stamp, 'cloud': None}
    return prior, receipts


def test_ready_imu_queue_is_consumed_before_one_other_callback_can_cause_false_disconnect():
    prior, receipts = freshness_fixture()
    now = [2.0]; callbacks = [0]; timeouts = []
    pending = [('wheel', 2.060)] + [('imu', 1.8+i*.005) for i in range(1, 53)]
    def spin(*, timeout_sec):
        timeouts.append(timeout_sec)
        if not pending:
            return
        name, original = pending.pop(0)
        now[0] += .061 if name == 'wheel' else .00001
        getattr(prior, name).append({'stamp': round(original*1e9), 'mono': round(original*1e9),
                                    'sequence': len(getattr(prior, name))})
        receipts[name] = now[0]; callbacks[0] += 1
        if callbacks[0] == 1:
            # The previous main loop would fail here, although 52 consecutive
            # valid 5-ms IMU samples are already ready in the executor.
            with pytest.raises(PriorError, match='imu source disconnected/stale'):
                prior_module.check_source_freshness(prior, receipts, now[0])
    report = prior_module.drain_ready_callbacks(spin, lambda: callbacks[0], clock=lambda: now[0])
    ages = prior_module.check_source_freshness(prior, receipts, now[0])
    assert callbacks[0] == 53 and report['ready_callbacks'] == 52
    assert report['ready_budget_exhausted'] is False and pending == []
    assert timeouts[0] == .02 and set(timeouts[1:]) == {0.}
    assert ages['latest_producer_age_s']['imu'] < .01
    assert prior.imu[-1]['mono'] == 2060000000
    assert max(b['mono']-a['mono'] for a,b in zip(prior.imu,prior.imu[1:])) == 5000000


def test_ready_drain_does_not_reset_a_truly_disconnected_imu():
    prior, receipts = freshness_fixture()
    now = [2.061]; callbacks = [0]
    def spin(*, timeout_sec):
        if callbacks[0] == 0:
            prior.wheel.append({'stamp': 2060000000, 'mono': 2060000000, 'sequence': 1})
            receipts['wheel'] = now[0]; callbacks[0] += 1
    prior_module.drain_ready_callbacks(spin, lambda: callbacks[0], clock=lambda: now[0])
    with pytest.raises(PriorError, match='imu source disconnected/stale'):
        prior_module.check_source_freshness(prior, receipts, now[0])
    assert prior.imu[-1]['mono'] == 1800000000 and receipts['imu'] == 1.8


@pytest.mark.parametrize('producer', [1.0, 2.5])
def test_new_callback_receipt_cannot_hide_old_or_future_original_source_time(producer):
    prior, receipts = freshness_fixture(2.0)
    receipts['imu'] = 2.01
    prior.imu[-1]['mono'] = int(producer*1e9)
    with pytest.raises(PriorError, match='imu original source time stale/future'):
        prior_module.check_source_freshness(prior, receipts, 2.02)
    assert prior.imu[-1]['mono'] == int(producer*1e9)


@pytest.mark.parametrize('name,limit', [('imu', .25), ('wheel', .5)])
def test_source_age_policy_still_accepts_fresh_samples_and_rejects_real_limit_crossing(name, limit):
    prior, receipts = freshness_fixture(2.0)
    now = 2.0+limit+.001
    for other in ('imu', 'wheel'):
        receipts[other] = now
        getattr(prior, other)[-1]['mono'] = round(now*1e9)
    prior_module.check_source_freshness(prior, receipts, now)
    getattr(prior, name)[-1]['mono'] = 2000000000
    with pytest.raises(PriorError, match=name+' original source time stale/future'):
        prior_module.check_source_freshness(prior, receipts, now)


@pytest.mark.parametrize('count_limit,time_limit', [(7, 1.), (256, .004)])
def test_busy_executor_ready_drain_has_count_and_elapsed_time_bounds(count_limit, time_limit):
    now = [0.]; callbacks = [0]
    def spin(*, timeout_sec):
        now[0] += .001; callbacks[0] += 1
    report = prior_module.drain_ready_callbacks(spin, lambda: callbacks[0], clock=lambda: now[0],
        max_ready_callbacks=count_limit, max_ready_seconds=time_limit)
    assert report['ready_budget_exhausted'] is True
    assert report['ready_callbacks'] <= count_limit
    assert report['ready_drain_s'] <= time_limit+.0011
    assert callbacks[0] < 10


def test_empty_executor_waits_once_and_callback_errors_propagate():
    timeouts = []
    report = prior_module.drain_ready_callbacks(lambda **kwargs: timeouts.append(kwargs['timeout_sec']), lambda: 0)
    assert timeouts == [.02] and report['initial_callbacks'] == report['ready_callbacks'] == 0
    def failed(**_): raise PriorError('original source epoch changed')
    with pytest.raises(PriorError, match='original source epoch changed'):
        prior_module.drain_ready_callbacks(failed, lambda: 0)


def test_initialization_timing_and_source_diagnostics_preserve_actual_sample_clocks():
    prior = initialized()
    rows = prior.initialization_timings['try_initialize']
    assert rows['calls'] > 200 and rows['total_s'] >= rows['max_s'] >= rows['last_s'] >= 0
    assert prior.initialization['stamp_ns'] == 3000000000
    assert prior.initialization['window_start_ns'] == 1000000000
    ages = prior_module.source_ages(prior, {'imu': 3.02, 'wheel': 3.01, 'cloud': None}, 3.025)
    assert ages['latest_producer_age_s'] == pytest.approx({'imu': .025, 'wheel': .025})
    assert ages['latest_source_age_s']['imu'] == pytest.approx(.005)
    assert ages['last_source_monotonic_ns'] == {'imu': 3000000000, 'wheel': 3000000000}
    assert ages['last_source_sequence'] == {'imu': 200, 'wheel': 20}
