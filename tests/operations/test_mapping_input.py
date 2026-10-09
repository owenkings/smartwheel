"""Device-free bootstrap, geometry and strict/queued publication contract checks."""
import copy
from collections import deque
import hashlib
import json
import math
from pathlib import Path
import shutil
import struct
import threading
import time
from types import SimpleNamespace as NS
import uuid

import numpy as np
import pytest

from wc_motion.protocol import crc16
from wc_runtime import mapping_input as m
from wc_sensors.pointcloud import decode_pointcloud2, validate_pointcloud2


START = 10_000_000_000
WALL = 1_700_000_000_000_000_000


@pytest.fixture
def tmp_path():
    parent = Path(__file__).absolute().parent
    directory = parent / ('.mapping_input_test_' + uuid.uuid4().hex)
    directory.mkdir()
    try:
        yield directory
    finally:
        resolved = directory.resolve()
        assert resolved.parent == parent and resolved.name.startswith('.mapping_input_test_')
        shutil.rmtree(resolved)


def config(mode='left'):
    roll, pitch = math.radians(3), math.radians(5)
    rx = np.array([[1, 0, 0], [0, math.cos(roll), -math.sin(roll)], [0, math.sin(roll), math.cos(roll)]])
    ry = np.array([[math.cos(pitch), 0, math.sin(pitch)], [0, 1, 0], [-math.sin(pitch), 0, math.cos(pitch)]])
    return {'schema_version': 1, 'status': 'EXPERIMENT', 'source_mode': 'real', 'mode': mode, 'session_id': 'mapping_input_test',
            'sensor_ids': {'left': 'lidar-L', 'right': 'lidar-R'}, 'imu_sensor_id': 'H30-fixture', 'wheel_device_id': 'wheel-fixture',
            'mount_model': 'fixed_hardware_setup_v1', 'imu_mount': {'R_axle_imu': np.eye(3).tolist()},
            'mounts': {'left': {'t_axle_lidar_m': [.34, .3125, .52], 'R_axle_lidar': np.eye(3).tolist()},
                       'right': {'same_forward': True, 't_axle_lidar_m': [.34, -.3125, .52], 'R_axle_lidar': (ry@rx).tolist()}},
            'input_rate_hz': 5., 'max_pair_delta_ns': 50_000_000,
            'prior_template': {'schema_version': 1, 'status': 'EXPERIMENT', 'source_mode': 'real',
                               'reference_frame': 'mapping_reference', 'guess_frame_id': 'prior_odom'},
            'formal_acceptance': False, 'navigation_validated': False, 'common_measurement_time_validated': False}


def stamp(value):
    return NS(sec=value//1_000_000_000, nanosec=value%1_000_000_000)


def imu(sequence=0, host=START, acceleration=(0., 0., 9.81), gyro=(0., 0., 0.)):
    header = NS(stamp=stamp(WALL+host), frame_id='imu_h30_native')
    return NS(session_id='mapping_input_test', sensor_id='H30-fixture', header=header,
              imu=NS(header=copy.deepcopy(header), linear_acceleration=NS(x=acceleration[0], y=acceleration[1], z=acceleration[2]),
                     angular_velocity=NS(x=gyro[0], y=gyro[1], z=gyro[2])),
              stream_epoch='imu-epoch', frame_sequence=sequence, coordinate_convention='H30_NATIVE_UNVALIDATED',
              time_source='arrival_only', common_time_valid=False, uncertainty_valid=False, common_time_ns=0,
              host_receive_time=stamp(WALL+host), host_monotonic_ns=host,
              angular_velocity_valid=True, linear_acceleration_valid=True, raw_packet=b'preserved-imu-packet')


def wheel(sequence=0, host=START, words=(0, 0)):
    payload = b'\x01\x03\x04' + struct.pack('>HH', *words)
    response = payload + crc16(payload)
    return {'schema': 'wc_wheel_feedback_v1', 'device_id': 'wheel-fixture', 'status': 'RESPONSE_VALID',
            'time_source': 'arrival_only', 'time_valid': False, 'formal_odometry_eligible': False, 'control_transmissions': 0,
            'request_hex': m.QUERY_HEX, 'response_hex': response.hex(), 'response_sha256': hashlib.sha256(response).hexdigest(),
            'register_words_u16': list(words), 'receive_monotonic_ns': host, 'stamp_ns': WALL+host,
            'sequence': sequence, 'stream_epoch': 'wheel-epoch'}


def source(side='left', sequence=0, host=START, *, invalid=True):
    points = [1., 2., 3., 7., 4., 5., 6., 9., float('nan') if invalid else 7., 8., 9., 11.]
    header = NS(stamp=stamp(WALL+host), frame_id='lidar_'+side)
    cloud = NS(header=copy.deepcopy(header), width=3, height=1, point_step=16, row_step=48,
               is_bigendian=False, is_dense=not invalid,
               fields=[NS(name=name, offset=4*i, count=1, datatype=7) for i, name in enumerate(('x', 'y', 'z', 'intensity'))],
               data=struct.pack('<12f', *points))
    return NS(header=header, cloud=cloud, session_id='mapping_input_test', side=side,
              sensor_id='lidar-L' if side=='left' else 'lidar-R', stream_epoch=side+'-epoch', frame_sequence=sequence,
              units='m', coordinate_convention='FLU', time_source='arrival_only', common_time_valid=False,
              uncertainty_valid=False, clock_model_id='', common_time_ns=WALL+host,
              host_receive_time=stamp(WALL+host), host_monotonic_ns=host, source_config_hash='a'*64,
              raw_count=3, valid_count=2 if invalid else 3)


def finish_bootstrap(value, *, acceleration=(0., 0., 9.81)):
    completed = None
    for index in range(21):
        now = START+index*100_000_000
        if isinstance(value, m.MappingInput):
            value.receive_wheel(wheel(index, now), now)
            completed = value.receive_imu(imu(index, now, acceleration), now)
        else:
            value.wheel(wheel(index, now), now)
            completed = value.imu(imu(index, now, acceleration), now)
    assert completed
    if isinstance(value, m.MappingInput):
        settle(value, now)
    return START+2_000_000_000


def wait_until(predicate, timeout=3):
    deadline = time.monotonic()+timeout
    while not predicate():
        assert time.monotonic() < deadline, 'worker did not complete within test timeout'
        time.sleep(.002)


def settle(value, now):
    wait_until(lambda: value.worker.jobs.unfinished_tasks == 0 or value.worker.error is not None)
    value.poll_io(now, clock=lambda: now)


@pytest.fixture
def owner(tmp_path):
    owners = []
    def make(mode='left', *, publication_mode=None, checkpoint_policy=None, continuous_mapping=False,
             offline_experiment=False, input_rate_hz=5.):
        session = tmp_path/('session_'+mode)
        session.mkdir()
        candidate = config(mode)
        candidate.update(offline_experiment=offline_experiment, input_rate_hz=input_rate_hz)
        if continuous_mapping:
            candidate['continuous_mapping'] = True
        if publication_mode is not None:
            candidate['archive_publication_mode'] = publication_mode
        if checkpoint_policy is not None:
            candidate['archive_checkpoint_policy'] = checkpoint_policy
        value = m.MappingInput(candidate, tmp_path, session/'input', started_ns=START)
        owners.append(value)
        wait_until(lambda: value.timings.get('status_write', {}).get('count', 0) >= 1)
        return value
    yield make
    for value in owners:
        value.close()


def serialize(message):
    return b'cdr:' + message.side.encode() + b':' + bytes(message.cloud.data)


def deliver(owner, message, published, *, at=None, **kwargs):
    now = message.host_monotonic_ns if at is None else at
    accepted = owner.receive_source(message, now, serialize, published.append, clock=lambda: now, **kwargs)
    if accepted:
        settle(owner, now)
    return accepted


@pytest.mark.parametrize('mode', ['left', 'right', 'all'])
def test_explicit_candidate_config_modes(mode):
    assert m.validate_config(config(mode)) == (('left', 'right') if mode=='all' else (mode,))


@pytest.mark.parametrize('offline,rate,accepted', [(False,0,False), (True,0,True),
                                                 (True,10,True), (True,11,False)])
def test_unthrottled_policy_is_offline_only(offline, rate, accepted):
    candidate=config('all')
    candidate.update(offline_experiment=offline, input_rate_hz=rate)
    if accepted:
        assert m.validate_config(candidate) == ('left','right')
    else:
        with pytest.raises(m.InputFailure, match='INPUT_RATE'):
            m.validate_config(candidate)


@pytest.mark.parametrize('rate,expected_pairs', [(0,12), (5,4), (10,6)])
def test_offline_all_pairs_preserves_jittered_frames_and_capped_baselines(owner, rate, expected_pairs):
    value=owner('all', offline_experiment=True, input_rate_hz=rate)
    start=finish_bootstrap(value)+100_000_000
    published=[]
    for index in range(12):
        # A 99 ms source interval is just below the old 100 ms threshold.
        host=start+index*99_000_000
        assert not deliver(value, source('left',index,host), published)
        deliver(value, source('right',index,host+2_000_000), published)
    assert len(published) == value.enqueued_outputs == expected_pairs
    status=value.status(start+12*99_000_000)
    assert status['rate_policy'] == ('ALL_VALID_PAIRS_NOT_THROTTLED' if rate == 0
                                    else 'MINIMUM_SOURCE_AND_PAIR_INTERVAL')
    for gate in value.gates.values():
        assert gate.validated_frames == 12 and gate.sequence_gaps == 0
        assert gate.throttled_frames == 12-expected_pairs
    rows=[json.loads(line) for line in (value.output/'pairs.jsonl').read_text().splitlines()]
    raw_keys=[tuple(member['raw_key']) for row in rows for member in row['sources']]
    assert len(raw_keys) == len(set(raw_keys)) == 2*expected_pairs


def test_offline_unthrottled_still_requires_unique_pair_within_time_bound(owner):
    value=owner('all', offline_experiment=True, input_rate_hz=0)
    start=finish_bootstrap(value)+100_000_000
    published=[]
    assert not deliver(value,source('left',0,start),published)
    too_late=start+value.config['max_pair_delta_ns']+1
    assert not deliver(value,source('right',0,too_late),published)
    assert value.unpaired_discarded == 1 and not published
    assert deliver(value,source('left',1,too_late+1),published)
    assert not deliver(value,source('right',1,too_late+2),published)
    assert len(published) == value.enqueued_outputs == 1
    with pytest.raises(m.InputFailure,match='NONMONOTONIC_SOURCE_SEQUENCE'):
        deliver(value,source('left',1,too_late+3),published)


@pytest.mark.parametrize('allowed,context,session,accepted',[
    (False,'user_manual_mapping','mapping_input_test',False),
    (True,'wrong','mapping_input_test',False),
    (True,'user_manual_mapping','other_session',False),
    (True,'user_manual_mapping','mapping_input_test',True)])
def test_controlled_feedback_requires_opt_in_and_current_session(allowed,context,session,accepted):
    candidate=config();candidate['manual_controls']={'arm_allowed':allowed}
    bootstrap=m.GravityBootstrap(candidate,started_ns=START)
    record=wheel();record.update(control_transmissions=4,control_context=context,session_id=session)
    if accepted:
        bootstrap.wheel(record,START)
        assert bootstrap.last_wheel['control_transmissions']==4
    else:
        with pytest.raises(m.InputFailure,match='WHEEL_NOT_READ_ONLY'):
            bootstrap.wheel(record,START)


@pytest.mark.parametrize('field,value', [('source_mode', 'synthetic'), ('status', 'VERIFIED'), ('mode', 'dual'),
                                      ('input_rate_hz', 6), ('max_pair_delta_ns', 50_000_001),
                                      ('formal_acceptance', True), ('common_measurement_time_validated', True)])
def test_bad_configuration_never_creates_an_input(owner, field, value):
    candidate = config(); candidate[field] = value
    with pytest.raises(m.InputFailure):
        m.validate_config(candidate)


def test_unknown_or_scaled_mount_rotation_is_not_invented():
    candidate = config('all'); del candidate['mounts']['right']['R_axle_lidar']
    with pytest.raises(KeyError):
        m.validate_config(candidate)
    candidate = config('right'); candidate['mounts']['right']['R_axle_lidar'] = (2*np.eye(3)).tolist()
    with pytest.raises(m.InputFailure):
        m.validate_config(candidate)
    candidate = config(); candidate['imu_mount']['R_axle_imu'][0][0] = -1
    with pytest.raises(m.InputFailure):
        m.validate_config(candidate)


def test_parking_slope_cannot_change_fixed_mounts_or_pair_geometry():
    acceleration = np.array([1.2, -2.1, 9.5])
    result = m.mount_transforms(config('all'), acceleration.tolist())
    r = np.asarray(result['R_axle_imu'])
    assert np.array_equal(r, np.eye(3))  # Static mount, not a gravity-leveling rotation.
    flat = m.mount_transforms(config('all'), [0., 0., 9.81])
    for key in ('T_base_axle', 'R_base_imu', 'R_axle_imu', 'T_base_side'):
        assert result[key] == flat[key]
    assert result['imu_roll_pitch_yaw_rad'] != flat['imu_roll_pitch_yaw_rad']
    assert result['gravity_used_to_define_mounts'] is False
    assert np.allclose(r.T@r, np.eye(3), atol=1e-12) and np.isclose(np.linalg.det(r), 1)
    left, right = [np.asarray(result['T_base_side'][side]) for side in ('left', 'right')]
    assert np.array_equal(left, np.eye(4))
    base_axle = np.asarray(result['T_base_axle'])
    axle_right = np.eye(4); axle_right[:3,:3] = config('all')['mounts']['right']['R_axle_lidar']
    axle_right[:3,3] = config('all')['mounts']['right']['t_axle_lidar_m']
    assert np.allclose(np.linalg.inv(base_axle)@right, axle_right)
    assert np.allclose(np.asarray(result['R_base_imu']), np.eye(3))


def test_right_base_stays_original_right_and_imu_has_actual_relative_rotation():
    result = m.mount_transforms(config('right'), [0., 0., 9.81])
    assert result['base_frame']=='lidar_right'
    assert np.array_equal(result['T_base_side']['right'], np.eye(4))
    assert np.allclose(result['R_base_imu'], np.asarray(config('right')['mounts']['right']['R_axle_lidar']).T)


def test_bootstrap_requires_full_continuous_two_seconds_and_records_exact_sources():
    boot = m.GravityBootstrap(config(), started_ns=START)
    finish_bootstrap(boot)
    assert len(boot.ready['imu_samples_used'])==21
    assert len(boot.ready['wheel_zero_records_used'])==21
    assert boot.ready['window_host_span_ns']==2_000_000_000
    assert boot.ready['stationary_ground_truth_validated'] is False
    assert boot.ready['common_measurement_time_validated'] is False
    assert boot.ready['yaw_reference']=='fixed_hardware_setup_mount_yaw; gravity_does_not_observe_heading'


def test_h30_serial_read_batches_preserve_repeated_arrivals_without_faking_duration():
    boot = m.GravityBootstrap(config(), started_ns=START)
    # Producer assigns a single pair of host clocks to every decoded packet
    # from one read. Four packets per read are four distinct sequence numbers.
    for batch in range(21):
        host = START+batch*100_000_000
        boot.wheel(wheel(batch, host), host)
        for packet in range(4):
            sequence = batch*4+packet
            boot.imu(imu(sequence, host), host+packet*1_000_000)
            if batch < 20:
                assert boot.ready is None
    assert boot.ready['window_host_span_ns'] == 2_000_000_000
    samples = boot.ready['imu_samples_used']
    assert [item['sequence'] for item in samples] == list(range(81))
    assert [item['host_monotonic_ns'] for item in samples[:4]] == [START]*4
    assert [item['source_stamp_ns'] for item in samples[:4]] == [WALL+START]*4
    assert boot.last_imu['sequence'] == 83
    assert len(boot.ready['wheel_zero_records_used']) == 21


@pytest.mark.parametrize('violation', ['repeat_sequence', 'reverse_sequence', 'epoch', 'host_reverse', 'stamp_reverse'])
def test_equal_h30_arrival_times_do_not_weaken_identity_or_reverse_time_checks(violation):
    boot = m.GravityBootstrap(config(), started_ns=START)
    boot.wheel(wheel(), START)
    boot.imu(imu(2, START), START)
    message = imu(3, START)
    if violation == 'repeat_sequence':
        message.frame_sequence = 2
    elif violation == 'reverse_sequence':
        message.frame_sequence = 1
    elif violation == 'epoch':
        message.stream_epoch = 'restarted'
    elif violation == 'host_reverse':
        message.host_monotonic_ns -= 1
    else:
        earlier = stamp(WALL+START-1)
        message.header.stamp = copy.deepcopy(earlier)
        message.imu.header.stamp = copy.deepcopy(earlier)
        message.host_receive_time = copy.deepcopy(earlier)
    with pytest.raises(m.InputFailure, match='IMU_EPOCH_OR_TIME_CHANGED'):
        boot.imu(message, START)


def test_nonzero_wheel_or_gyro_resets_incomplete_static_window():
    boot = m.GravityBootstrap(config(), started_ns=START)
    for index in range(11):
        now=START+index*100_000_000
        boot.wheel(wheel(index,now),now); boot.imu(imu(index,now),now)
    now+=100_000_000
    boot.wheel(wheel(11,now,(0,3)),now); assert boot.window==[]
    boot.imu(imu(11,now),now); assert boot.ready is None
    now+=100_000_000
    boot.wheel(wheel(12,now),now); boot.imu(imu(12,now,gyro=(.1,0,0)),now)
    assert boot.window==[] and boot.ready is None


def test_stale_wheel_cannot_be_used_to_confirm_static_imu():
    boot = m.GravityBootstrap(config(), started_ns=START)
    boot.wheel(wheel(),START)
    for index in range(22):
        now=START+index*100_000_000
        boot.imu(imu(index,now),now)
    assert boot.ready is None
    with pytest.raises(m.InputFailure, match='BOOTSTRAP_STARTUP_TIMEOUT'):
        boot.check_stale(START+15_000_000_001)


@pytest.mark.parametrize('field,value', [('session_id','other'), ('sensor_id','other'),
                                      ('angular_velocity_valid',False), ('common_time_valid',True),
                                      ('coordinate_convention','FLU')])
def test_invalid_imu_identity_or_metadata_latches_failure(owner, field, value):
    value_owner=owner(); message=imu(); setattr(message,field,value)
    with pytest.raises(m.InputFailure):
        value_owner.receive_imu(message,START)
    assert value_owner.state=='FAILED'
    with pytest.raises(m.InputFailure):
        value_owner.receive_wheel(wheel(),START)


@pytest.mark.parametrize('field,value', [('device_id','other'), ('control_transmissions',1),
                                      ('response_sha256','0'*64), ('register_words_u16',[0,1]),
                                      ('time_valid',True), ('request_hex','0106000000000000')])
def test_wheel_zero_metadata_cannot_override_actual_crc_response(owner, field, value):
    value_owner=owner(); record=wheel(); record[field]=value
    with pytest.raises(m.InputFailure):
        value_owner.receive_wheel(record,START)
    assert value_owner.state=='FAILED'


def test_imu_or_wheel_epoch_changes_are_not_stitched():
    for sensor in ('imu','wheel'):
        boot=m.GravityBootstrap(config(),started_ns=START)
        if sensor=='imu':
            boot.imu(imu(),START); next_msg=imu(1,START+100_000_000); next_msg.stream_epoch='restarted'
        else:
            boot.wheel(wheel(),START); next_msg=wheel(1,START+100_000_000); next_msg['stream_epoch']='restarted'
        with pytest.raises(m.InputFailure,match='EPOCH'):
            getattr(boot,sensor)(next_msg,START+100_000_000)


def test_prior_config_is_complete_atomic_and_exclusive(owner):
    value=owner(); finish_bootstrap(value)
    prior_path=value.output.parent/'prior_config.json'
    prior=json.loads(prior_path.read_text())
    assert prior['base_frame']=='lidar_left'
    assert prior['bootstrap_sha256']==hashlib.sha256((value.output/'bootstrap.json').read_bytes()).hexdigest()
    assert not (value.output.parent/'prior_config.json.partial').exists()
    before=prior_path.read_bytes()
    with pytest.raises(FileExistsError):
        m.write_exclusive_json(value.project_root,prior_path,{'wrong':'replacement'})
    assert prior_path.read_bytes()==before


def test_single_source_raw_bytes_archive_and_index_exist_before_publication(owner):
    value=owner(); now=finish_bootstrap(value)+10_000_000
    message=source(host=now); before=copy.deepcopy(message)
    observed=[]
    def publish(cloud):
        row=json.loads((value.output/'pairs.jsonl').read_text())
        path=value.output/row['sources'][0]['cdr_file']
        assert path.read_bytes()==serialize(message)
        assert row['sources'][0]['time_offset_ns']==0
        assert cloud.data==before.cloud.data and cloud.header.frame_id=='lidar_left'
        observed.append(cloud)
    assert value.receive_source(message,now,serialize,publish,clock=lambda:now)
    settle(value,now)
    assert message.cloud.data==before.cloud.data
    assert len(observed)==1 and value.published_frames==1


def test_bootstrap_discards_lidar_until_mount_config_is_written(owner):
    value=owner(); published=[]
    assert not deliver(value,source(host=START),published)
    assert published==[] and value.bootstrap_discarded==1
    assert not (value.output.parent/'prior_config.json').exists()


def test_after_boot_nonzero_wheel_and_gyro_allow_motion_without_releveling(owner):
    value=owner(); now=finish_bootstrap(value)
    original=copy.deepcopy(value.transforms)
    now+=100_000_000
    value.receive_wheel(wheel(21,now,(42,65501)),now)
    assert not value.receive_imu(imu(21,now,acceleration=(1.,2.,8.),gyro=(.5,.3,.2)),now)
    published=[]
    assert deliver(value,source(host=now),published)
    assert value.transforms==original and value.state=='RUNNING'


def test_single_throttle_is_host_monotonic_and_preserves_stamp(owner):
    value=owner(); now=finish_bootstrap(value)+10_000_000; published=[]
    assert deliver(value,source(sequence=0,host=now),published)
    assert not deliver(value,source(sequence=1,host=now+90_000_000),published)
    final=source(sequence=2,host=now+210_000_000)
    assert deliver(value,final,published)
    assert m._stamp(published[-1].header.stamp)==m._stamp(final.header.stamp)
    assert value.gates['left'].throttled_frames==1 and len(published)==2


def test_dual_pair_keeps_offsets_source_codes_raw_row_order_and_rigid_geometry(owner):
    value=owner('all'); now=finish_bootstrap(value)+10_000_000; published=[]
    left,right=source('left',host=now),source('right',host=now+30_000_000)
    original_left,original_right=bytes(left.cloud.data),bytes(right.cloud.data)
    assert not deliver(value,left,published)
    assert deliver(value,right,published)
    cloud=published[0]; layout=validate_pointcloud2(cloud)
    assert layout['point_count']==6 and layout['fields']['time_offset_s']['datatype']==8
    assert layout['fields']['source_code']['datatype']==2
    offsets=np.ndarray((6,),dtype='<f8',buffer=cloud.data,offset=16,strides=(32,))
    codes=np.ndarray((6,),dtype='u1',buffer=cloud.data,offset=24,strides=(32,))
    assert np.array_equal(codes,[1,1,1,2,2,2])
    assert np.array_equal(offsets,[-.03,-.03,-.03,0,0,0])
    assert m._stamp(cloud.header.stamp)==WALL+now+30_000_000
    xyz=decode_pointcloud2(cloud); right_xyz=decode_pointcloud2(right.cloud)
    transform=np.asarray(value.transforms['T_base_side']['right'])
    assert np.allclose(xyz[3:5],(right_xyz@transform[:3,:3].T+transform[:3,3])[:2],atol=1e-6)
    assert np.isnan(xyz[2]).any() and np.isnan(xyz[5]).any()
    assert bytes(left.cloud.data)==original_left and bytes(right.cloud.data)==original_right
    index=json.loads((value.output/'pairs.jsonl').read_text())
    assert [r['output_row_start'] for r in index['sources']]==[0,3]
    assert [r['time_offset_ns'] for r in index['sources']]==[-30_000_000,0]
    for row in index['sources']:
        assert hashlib.sha256((value.output/row['cdr_file']).read_bytes()).hexdigest()==row['cdr_sha256']
    assert value.status(now+30_000_000)['archived_frames']==2


def test_dual_matching_chooses_nearest_available_without_reusing_source(owner):
    value=owner('all'); now=finish_bootstrap(value)+10_000_000; published=[]
    deliver(value,source('left',0,now),published)
    deliver(value,source('left',1,now+20_000_000),published)
    assert deliver(value,source('right',0,now+25_000_000),published)
    row=json.loads((value.output/'pairs.jsonl').read_text())
    assert row['sources'][0]['raw_key'][-1]==1
    assert not deliver(value,source('right',1,now+100_000_000),published)
    assert len(published)==1


def test_dual_excess_delta_drops_unmatched_without_single_fallback(owner):
    value=owner('all'); now=finish_bootstrap(value)+10_000_000; published=[]
    assert not deliver(value,source('left',host=now),published)
    assert not deliver(value,source('right',host=now+60_000_000),published)
    assert published==[] and value.unpaired_discarded==1


def test_missing_required_side_eventually_fails_no_degraded_single_output(owner):
    value=owner('all'); now=finish_bootstrap(value)+10_000_000; published=[]
    deliver(value,source('left',host=now),published); deliver(value,source('right',host=now),published)
    later=now+3_000_000_001
    value.receive_wheel(wheel(22,later,(1,1)),later); value.receive_imu(imu(22,later,gyro=(.2,0,0)),later)
    assert not deliver(value,source('left',1,later),published)
    with pytest.raises(m.InputFailure,match='SOURCE_STALE'):
        value.check_stale(later)
    assert len(published)==1 and value.state=='FAILED'


def test_failed_second_cdr_preserves_first_but_publishes_nothing(owner,monkeypatch):
    value=owner('all'); now=finish_bootstrap(value)+10_000_000; published=[]
    def failed(*args):raise OSError('disk failure')
    monkeypatch.setattr(value.archives['right'],'append',failed)
    deliver(value,source('left',host=now),published)
    with pytest.raises(m.InputFailure,match='disk failure'):
        deliver(value,source('right',host=now),published)
    assert published==[] and value.published_frames==0
    assert value.gates['left'].archived_frames==1 and value.state=='FAILED'
    assert (value.output/'left/frames/00000001.cdr').is_file()


def test_publication_exception_latches_failure_after_flushed_pair_index(owner):
    value=owner(); now=finish_bootstrap(value)+10_000_000
    def failed(cloud):raise RuntimeError('publisher unavailable')
    assert value.receive_source(source(host=now),now,serialize,failed,clock=lambda:now)
    with pytest.raises(m.InputFailure,match='publisher unavailable'):
        settle(value,now)
    assert value.archived_outputs==1 and value.published_frames==0 and value.state=='FAILED'
    assert json.loads((value.output/'pairs.jsonl').read_text())['publication_state'].startswith('WRITTEN_AND_FLUSHED')


def test_source_identity_failure_blocks_all_future_callbacks(owner):
    value=owner(); now=finish_bootstrap(value)+10_000_000; message=source(host=now); message.sensor_id='wrong'
    with pytest.raises(m.InputFailure,match='SOURCE_IDENTITY'):
        deliver(value,message,[])
    with pytest.raises(m.InputFailure):
        deliver(value,source(host=now),[])


def test_existing_input_evidence_is_never_overwritten(owner):
    value=owner(); before=(value.output/'status.json').read_bytes()
    with pytest.raises(FileExistsError):
        m.MappingInput(config(),value.project_root,value.output,started_ns=START)
    assert (value.output/'status.json').read_bytes()==before


def health_qos():
    return m.qos_profiles(NS, NS(BEST_EFFORT='best_effort', RELIABLE='reliable'), NS(VOLATILE='volatile'))


def test_health_qos_keeps_latest_while_lidar_and_output_contracts_stay_bounded():
    qos = health_qos()
    assert qos['imu'].depth == qos['wheel'].depth == 1
    assert qos['imu'].reliability == 'best_effort' and qos['wheel'].reliability == 'reliable'
    assert qos['lidar'].depth == qos['output'].depth == 2
    assert qos['output'].reliability == 'reliable'


def test_latest_health_samples_survive_simulated_one_second_durable_io_stall(owner):
    value = owner(); ready_time = finish_bootstrap(value)
    # Model KEEP_LAST using the actual production subscription depth. The source
    # keeps producing at 200 Hz while this consumer cannot run callbacks.
    queue = deque(maxlen=health_qos()['imu'].depth)
    original = []
    for index in range(1, 201):
        message = imu(20+index, ready_time+index*5_000_000)
        original.append((message.frame_sequence, message.host_monotonic_ns, m._stamp(message.header.stamp)))
        queue.append(message)
    now = ready_time+1_000_000_000
    value.receive_wheel(wheel(30, now, (4, 4)), now)
    value.receive_imu(queue.popleft(), now)
    assert value.failure_reason is None and value.bootstrap.imu_seq == 220
    assert value.bootstrap.last_imu['host_monotonic_ns'] == now
    assert value.bootstrap.last_imu['source_stamp_ns'] == WALL+now
    assert value.bootstrap.imu_sequence_gaps == 199
    assert original[-1] == (220, now, WALL+now)
    assert value.status(now)['last_health_observations']['imu']['callback_source_age_ns'] == 0
    # The obsolete first queued sample is still rejected by the exact old gate;
    # the fix does not relabel that timestamp or widen the freshness threshold.
    check = m.GravityBootstrap(config(), started_ns=START)
    with pytest.raises(m.InputFailure, match='IMU_STALE_OR_FUTURE'):
        check.imu(imu(21, ready_time+5_000_000), now)
    assert check.last_imu_observation['callback_source_age_ns'] == 995_000_000


@pytest.mark.parametrize('age_ns,valid', [(500_000_000, True), (500_000_001, False), (-1, False)])
def test_imu_exact_original_freshness_boundary_and_future_rejection(age_ns, valid):
    boot = m.GravityBootstrap(config(), started_ns=START)
    message = imu(host=START+1_000_000_000)
    if valid:
        boot.imu(message, message.host_monotonic_ns+age_ns)
        assert boot.last_imu['host_monotonic_ns'] == message.host_monotonic_ns
    else:
        with pytest.raises(m.InputFailure, match='IMU_STALE_OR_FUTURE'):
            boot.imu(message, message.host_monotonic_ns+age_ns)


def test_latest_sample_bootstrap_preserves_real_span_and_resets_unobserved_gap():
    boot = m.GravityBootstrap(config(), started_ns=START)
    boot.wheel(wheel(), START); boot.imu(imu(), START)
    # A consumer gap above 0.5 s breaks the static window even if the next
    # sample is fresh. Subsequent source sequence gaps do not manufacture time.
    origin = START+600_000_000
    for index in range(21):
        host = origin+index*100_000_000
        boot.wheel(wheel(index+1, host), host)
        boot.imu(imu(20*(index+1), host), host)
        if index < 20:
            assert boot.ready is None
    assert boot.ready['imu_samples_used'][0]['host_monotonic_ns'] == origin
    assert boot.ready['window_host_span_ns'] == 2_000_000_000
    assert len(boot.ready['imu_samples_used']) == 21 and boot.resets == 1
    assert boot.imu_sequence_gaps == 21*19


def test_timing_evidence_records_slow_failed_operation_without_changing_input_time(owner):
    value = owner()
    ticks = iter([100, 800_000_100])
    def failed():
        raise OSError('simulated durability failure')
    with pytest.raises(OSError, match='durability'):
        value.timed('archive', failed, clock=lambda: next(ticks))
    assert value.timings['archive'] == {'count': 1, 'last_duration_ns': 800_000_000, 'max_duration_ns': 800_000_000}


def test_blocked_archive_does_not_block_health_and_publish_waits_for_all_writes(owner, monkeypatch):
    value = owner('all'); now = finish_bootstrap(value)+10_000_000; published = []
    entered, release = threading.Event(), threading.Event()
    original = value.archives['right'].append
    def blocked(*args):
        entered.set()
        assert release.wait(3)
        return original(*args)
    monkeypatch.setattr(value.archives['right'], 'append', blocked)
    left, right = source('left', host=now), source('right', host=now+20_000_000)
    assert not value.receive_source(left, now, serialize, published.append)
    assert value.receive_source(right, now+20_000_000, serialize, published.append)
    try:
        assert entered.wait(1)
        fresh = now+200_000_000
        value.receive_wheel(wheel(21, fresh, (5, 5)), fresh)
        value.receive_imu(imu(21, fresh, gyro=(.2, 0, 0)), fresh)
        value.poll_io(fresh, clock=lambda: fresh); value.check_stale(fresh)
        assert value.failure_reason is None and value.bootstrap.imu_seq == 21
        assert published == [] and value.published_frames == 0
        state = value.status(fresh)
        assert state['pending_outputs'] == 1 and state['archived_frames'] == 1
        assert state['storage']['checkpoint'] is None
        assert (value.output/'pairs.jsonl').read_bytes() == b''
    finally:
        release.set()
    settle(value, fresh)
    assert len(published) == 1 and value.status(fresh)['pending_outputs'] == 0
    assert m._stamp(published[0].header.stamp) == WALL+right.host_monotonic_ns


def test_bootstrap_config_io_runs_off_callback_and_motion_does_not_relevel(owner, monkeypatch):
    value = owner(); entered, release = threading.Event(), threading.Event()
    original = m.write_exclusive_json
    def blocked(*args, **kwargs):
        entered.set(); assert release.wait(3)
        return original(*args, **kwargs)
    monkeypatch.setattr(m, 'write_exclusive_json', blocked)
    try:
        for index in range(21):
            now = START+index*100_000_000
            value.receive_wheel(wheel(index, now), now)
            value.receive_imu(imu(index, now), now)
        assert entered.wait(1) and value.transforms is None
        now += 100_000_000
        value.receive_wheel(wheel(21, now, (10, 10)), now)
        value.receive_imu(imu(21, now, gyro=(.3, 0, 0)), now)
        assert value.bootstrap.ready is not None and value.failure_reason is None
        assert not value.receive_source(source(host=now), now, serialize, lambda cloud: pytest.fail('premature publish'))
    finally:
        release.set()
    settle(value, now)
    assert value.transforms['base_frame'] == 'lidar_left'
    assert value.bootstrap.ready['imu_samples_used'][-1]['sequence'] == 20


def test_status_requests_coalesce_while_storage_is_blocked(owner, monkeypatch):
    value = owner(); entered, release = threading.Event(), threading.Event()
    original = m.atomic_status
    writes = []
    def blocked(project, output, payload, **kwargs):
        entered.set(); assert release.wait(3)
        writes.append(payload['updated_monotonic_ns'])
        return original(project, output, payload, **kwargs)
    monkeypatch.setattr(m, 'atomic_status', blocked)
    try:
        value.write_status(START+1)
        assert entered.wait(1)
        for index in range(2, 100):
            value.write_status(START+index)
        assert value.status_writer.latest_status['updated_monotonic_ns'] == START+99
        assert value.worker.jobs.qsize() == 0
        value.receive_wheel(wheel(), START); value.receive_imu(imu(), START)
        assert value.failure_reason is None
    finally:
        release.set()
    wait_until(lambda: len(writes) >= 2)
    assert writes == [START+1, START+99]


def test_blocked_status_cannot_fill_cloud_archive_queue(owner, monkeypatch):
    value = owner(); now = finish_bootstrap(value); published = []
    entered, release = threading.Event(), threading.Event()
    original = m.atomic_status
    def blocked(*args, **kwargs):
        entered.set()
        assert release.wait(5)
        return original(*args, **kwargs)
    monkeypatch.setattr(m, 'atomic_status', blocked)
    try:
        value.write_status(now)
        assert entered.wait(1)
        # Six seconds of sensor time: three times the old 10-frame queue
        # capacity. Disk status remains blocked throughout, not merely slow.
        for index in range(30):
            at = now+(index+1)*200_000_000
            value.receive_wheel(wheel(21+index, at), at)
            value.receive_imu(imu(21+index, at), at)
            assert deliver(value, source(sequence=index, host=at), published)
            value.check_stale(at)
            value.write_status(at)
            assert value.failure_reason is None
        assert len(published) == 30
        assert value.worker.outputs == 30
        assert value.worker.jobs.qsize() == 0
        assert value.status_writer.thread.is_alive()
    finally:
        release.set()
    value.close()
    final = json.loads((value.output/'status.json').read_text())
    assert final['state'] == 'STOPPED'
    assert final['status_write_isolated_from_archive'] is True
    assert final['published_frames'] == final['archived_outputs'] == 30


def test_status_write_failure_is_reported_without_losing_completed_archive(owner, monkeypatch):
    value = owner(); now = finish_bootstrap(value)
    assert deliver(value, source(host=now), [])
    def fail(*args, **kwargs):
        raise OSError('synthetic preview write failure')
    with monkeypatch.context() as patch:
        patch.setattr(m, 'atomic_status', fail)
        value.write_status(now)
        wait_until(lambda: value.status_writer.error is not None)
        with pytest.raises(m.InputFailure, match='STATUS_WRITE_FAILED'):
            value.poll_io(now, clock=lambda: now)
        assert value.failure_reason.startswith('STATUS_WRITE_FAILED')
    value.close()
    final = json.loads((value.output/'status.json').read_text())
    assert final['state'] == 'FAILED'
    assert final['archived_outputs'] == 1
    assert json.loads((value.output/'checkpoint.json').read_text())['final'] is True


def test_blocked_checkpoint_leaves_archive_running_and_certifies_only_captured_prefix(owner, monkeypatch):
    # Hold the periodic thread idle so the controlled checkpoint thread tests
    # a real fsync boundary deterministically rather than racing a timer.
    monkeypatch.setattr(m, 'CHECKPOINT_INTERVAL_S', 3600.)
    value = owner(); now = finish_bootstrap(value)+10_000_000; published = []
    assert deliver(value, source(sequence=0, host=now), published)
    prefix = value.checkpoints.snapshot()['written_prefix']
    entered, release = threading.Event(), threading.Event()
    original = value.checkpoints._fsync_path
    def blocked(path):
        if Path(path).suffix == '.cdr' and not entered.is_set():
            entered.set(); assert release.wait(3)
        return original(path)
    monkeypatch.setattr(value.checkpoints, '_fsync_path', blocked)
    failure = []
    def commit():
        try:
            value.checkpoints._commit(prefix, False)
        except Exception as error:
            failure.append(error)
    thread = threading.Thread(target=commit)
    thread.start()
    try:
        assert entered.wait(1)
        assert deliver(value, source(sequence=1, host=now+210_000_000), published)
        assert len(published) == 2 and value.checkpoints.snapshot()['checkpoint'] is None
    finally:
        release.set(); thread.join(3)
    assert not thread.is_alive() and failure == []
    checkpoint = json.loads((value.output/'checkpoint.json').read_text())
    assert checkpoint['archived_outputs'] == 1 and checkpoint['sources']['left']['frames'] == 1
    assert checkpoint['pairs_index_bytes'] < (value.output/'pairs.jsonl').stat().st_size
    assert checkpoint['crash_loss_bound_s'] is None and checkpoint['final'] is False
    value.close()
    final = json.loads((value.output/'checkpoint.json').read_text())
    assert final['final'] is True and final['archived_outputs'] == 2
    assert final['pairs_index_bytes'] == (value.output/'pairs.jsonl').stat().st_size


def test_checkpoint_failure_latches_failure_and_close_never_claims_stopped(owner, monkeypatch):
    monkeypatch.setattr(m, 'CHECKPOINT_INTERVAL_S', 3600.)
    value = owner(); now = finish_bootstrap(value)+10_000_000
    assert deliver(value, source(host=now), [])
    def failed(path):
        raise OSError('simulated eMMC fsync failure')
    monkeypatch.setattr(value.checkpoints, '_fsync_path', failed)
    value.close()
    status = json.loads((value.output/'status.json').read_text())
    assert value.failure_reason.startswith('STORAGE_CHECKPOINT_FAILED')
    assert status['state'] == 'FAILED' and status['storage']['checkpoint'] is None
    assert (value.output/'left/frames/00000001.cdr').is_file()


def checkpoint_call(operation):
    errors = []
    def run():
        try: operation()
        except BaseException as error: errors.append(error)
    thread = threading.Thread(target=run, name='synthetic-checkpoint-coordinator', daemon=True)
    thread.start()
    return thread, errors


def test_checkpoint_data_fsync_really_overlaps_with_four_daemons_and_lazy_dispatch(owner, monkeypatch):
    monkeypatch.setattr(m, 'CHECKPOINT_INTERVAL_S', 3600.)
    value = owner(); journal = value.checkpoints
    paths = [value.output/('parallel_%02d.bin' % number) for number in range(12)]
    for path in paths: path.write_bytes(b'independent synthetic data')
    categories = ('cdr_file', 'source_index', 'pairs_index', 'immutable_artifact')
    yielded, entered_threads, completed = [], set(), []
    lock, all_four, release = threading.Lock(), threading.Event(), threading.Event()
    original = journal._fsync_path
    def jobs():
        for number, path in enumerate(paths):
            yielded.append(path)
            yield categories[number % 4], path
    def blocked(path):
        assert threading.current_thread().daemon
        with lock:
            entered_threads.add(threading.get_ident())
            if len(entered_threads) == 4: all_four.set()
        assert release.wait(3)
        original(path)  # Real file descriptor fsync, not a mocked success.
        with lock: completed.append(path)
    with monkeypatch.context() as patch:
        patch.setattr(journal, '_fsync_path', blocked)
        thread, errors = checkpoint_call(lambda: journal._sync_data_files(jobs()))
        try:
            assert all_four.wait(2)
            assert len(yielded) == 4  # No pre-materialized list of jobs/futures.
            stats = journal.snapshot()['data_sync_workers']
            assert stats['limit'] == stats['active'] == stats['max_active'] == 4
            assert stats['jobs_started'] == 4 and stats['jobs_completed'] == 0
            assert journal.thread.daemon
        finally:
            release.set(); thread.join(3)
        assert not thread.is_alive() and not errors
    assert sorted(completed) == sorted(paths) and len(yielded) == 12
    stats = journal.snapshot()['data_sync_workers']
    assert stats['active'] == stats['threads_alive'] == 0
    assert stats['max_active'] == 4 and stats['jobs_started'] == stats['jobs_completed'] == 12
    assert set(journal.snapshot()['sync_operation_timings']) == set(categories)


def test_parallel_data_barrier_blocks_all_namespace_and_manifest_work(owner, monkeypatch):
    monkeypatch.setattr(m, 'CHECKPOINT_INTERVAL_S', 3600.)
    value = owner(); journal = value.checkpoints
    journal._commit(journal.snapshot()['written_prefix'], False)
    previous = (value.output/'checkpoint.json').read_bytes()
    now = finish_bootstrap(value)+10_000_000
    assert deliver(value, source(host=now), [])
    prefix = journal.snapshot()['written_prefix']
    entered, release, namespace_operations = threading.Event(), threading.Event(), []
    original_file, original_operation = journal._fsync_path, journal._sync_operation
    def blocked(path):
        if Path(path).suffix == '.cdr':
            entered.set(); assert release.wait(3)
        return original_file(path)
    def operation(category, action):
        if category.endswith('directory') or category.startswith('manifest_'):
            namespace_operations.append(category)
        return original_operation(category, action)
    with monkeypatch.context() as patch:
        patch.setattr(journal, '_fsync_path', blocked)
        patch.setattr(journal, '_sync_operation', operation)
        thread, errors = checkpoint_call(lambda: journal._commit(prefix, False))
        try:
            assert entered.wait(2)
            assert thread.is_alive() and namespace_operations == []
            assert (value.output/'checkpoint.json').read_bytes() == previous
            assert not (value.output/'checkpoint.json.partial').exists()
        finally:
            release.set(); thread.join(3)
        assert not thread.is_alive() and not errors
    assert journal.snapshot()['checkpoint']['archived_outputs'] == 1
    assert namespace_operations[-3:] == ['manifest_file', 'manifest_replace', 'manifest_directory']


def test_parallel_data_failure_joins_started_jobs_and_preserves_manifest(owner, monkeypatch):
    monkeypatch.setattr(m, 'CHECKPOINT_INTERVAL_S', 3600.)
    value = owner(); journal = value.checkpoints
    journal._commit(journal.snapshot()['written_prefix'], False)
    previous = (value.output/'checkpoint.json').read_bytes()
    now = finish_bootstrap(value)+10_000_000
    assert deliver(value, source(host=now), [])
    prefix = journal.snapshot()['written_prefix']
    blocked, failed, release = threading.Event(), threading.Event(), threading.Event()
    original = journal._fsync_path
    def operation(path):
        if Path(path).name == 'frames.jsonl':
            assert blocked.wait(2)
            failed.set(); raise OSError('synthetic source-index fsync failure')
        if Path(path).name == 'pairs.jsonl':
            blocked.set(); assert release.wait(3)
        return original(path)
    with monkeypatch.context() as patch:
        patch.setattr(journal, '_fsync_path', operation)
        thread, errors = checkpoint_call(lambda: journal._commit(prefix, False))
        try:
            assert failed.wait(2)
            # Returning an error before joining would leak the still-owned FD.
            thread.join(.03)
            assert thread.is_alive() and not errors
            assert (value.output/'checkpoint.json').read_bytes() == previous
            assert not (value.output/'checkpoint.json.partial').exists()
        finally:
            release.set(); thread.join(3)
        assert not thread.is_alive() and len(errors) == 1
        assert 'source-index fsync failure' in str(errors[0])
    assert (value.output/'checkpoint.json').read_bytes() == previous
    assert journal.snapshot()['checkpoint']['archived_outputs'] == 0
    assert journal.snapshot()['data_sync_workers']['threads_alive'] == 0


def test_close_timeout_cancels_late_parallel_data_commit_and_keeps_old_manifest(owner, monkeypatch):
    monkeypatch.setattr(m, 'CHECKPOINT_INTERVAL_S', 3600.)
    value = owner(); journal = value.checkpoints
    journal._commit(journal.snapshot()['written_prefix'], False)
    previous = (value.output/'checkpoint.json').read_bytes()
    now = finish_bootstrap(value)+10_000_000
    assert deliver(value, source(host=now), [])
    entered, release = threading.Event(), threading.Event()
    original = journal._fsync_path
    def blocked(path):
        if Path(path).suffix == '.cdr':
            entered.set(); assert release.wait(3)
        return original(path)
    with monkeypatch.context() as patch:
        patch.setattr(journal, '_fsync_path', blocked)
        # Start the real final-checkpoint coordinator before measuring close's
        # short timeout. Its four initial index/artifact fsyncs can legitimately
        # take longer than 150 ms, cancelling dispatch before a CDR ever starts.
        # This case specifically exercises cancellation with a CDR in flight.
        with journal.condition:
            journal.closing = True
            journal.condition.notify_all()
        try:
            assert entered.wait(2)
            closer, errors = checkpoint_call(lambda: journal.close(.15))
            closer.join(1)
            assert not closer.is_alive() and len(errors) == 1
            assert 'STORAGE_CHECKPOINT_CLOSE_TIMEOUT' in str(errors[0])
            assert journal.thread.is_alive() and journal.thread.daemon
            assert all(thread.daemon for thread in journal.data_threads)
            assert journal.snapshot()['data_sync_workers']['cancelled']
            assert (value.output/'checkpoint.json').read_bytes() == previous
        finally:
            release.set(); journal.thread.join(3)
        assert not journal.thread.is_alive()
    assert (value.output/'checkpoint.json').read_bytes() == previous
    assert not (value.output/'checkpoint.json.partial').exists()
    assert journal.snapshot()['checkpoint']['archived_outputs'] == 0
    assert journal.snapshot()['error'] == 'STORAGE_CHECKPOINT_CLOSE_TIMEOUT'
    assert journal.snapshot()['data_sync_workers']['threads_alive'] == 0
    value.close()
    assert value.state == 'FAILED' and 'STORAGE_CHECKPOINT_CLOSE_TIMEOUT' in value.failure_reason


def test_checkpoint_unchanged_skips_io_but_final_still_commits(owner, monkeypatch):
    monkeypatch.setattr(m, 'CHECKPOINT_INTERVAL_S', 3600.)
    value = owner(); now = finish_bootstrap(value)+10_000_000
    assert deliver(value, source(host=now), [])
    journal = value.checkpoints
    prefix = journal.snapshot()['written_prefix']
    assert journal._commit(prefix, False) is True
    before = journal.snapshot(); contents = (value.output/'checkpoint.json').read_bytes()
    assert journal._commit(copy.deepcopy(prefix), False) is False
    skipped = journal.snapshot()
    assert skipped['checkpoint'] == before['checkpoint']
    assert skipped['sync_operation_timings'] == before['sync_operation_timings']
    assert skipped['skipped_unchanged'] == 1
    assert (value.output/'checkpoint.json').read_bytes() == contents
    assert journal._commit(prefix, True) is True
    final = journal.snapshot()
    assert final['checkpoint']['final'] is True and final['checkpoint']['generation'] == 2
    for category, row in before['sync_operation_timings'].items():
        increase = 1 if category in ('manifest_file', 'manifest_replace', 'manifest_directory') else 0
        assert final['sync_operation_timings'][category]['count'] == row['count']+increase


def test_checkpoint_steady_append_syncs_only_new_cdr_changed_indexes_and_frame_names(owner, monkeypatch):
    monkeypatch.setattr(m, 'CHECKPOINT_INTERVAL_S', 3600.)
    value = owner('all'); now = finish_bootstrap(value)+10_000_000; published = []
    for sequence in range(2):
        host = now+sequence*210_000_000
        assert not deliver(value, source('left', sequence, host), published)
        assert deliver(value, source('right', sequence, host+10_000_000), published)
        if sequence == 0:
            value.checkpoints._commit(value.checkpoints.snapshot()['written_prefix'], False)
            before = value.checkpoints.snapshot()['sync_operation_timings']
    value.checkpoints._commit(value.checkpoints.snapshot()['written_prefix'], False)
    after = value.checkpoints.snapshot()['sync_operation_timings']
    increments = {key: after[key]['count']-row['count'] for key, row in before.items()}
    assert increments == {'cdr_file': 2, 'source_index': 2, 'frames_directory': 2,
                          'source_directory': 0, 'pairs_index': 1, 'immutable_artifact': 0,
                          'new_entry_directory': 0, 'manifest_file': 1,
                          'manifest_replace': 1, 'manifest_directory': 1}


def test_new_artifact_names_sync_before_manifest_and_output_sync_after_rename(owner, monkeypatch):
    monkeypatch.setattr(m, 'CHECKPOINT_INTERVAL_S', 3600.)
    value = owner(); journal = value.checkpoints
    journal._commit(journal.snapshot()['written_prefix'], False)  # Initial empty namespace is durable.
    finish_bootstrap(value)
    events = []
    original_file, original_dir, original_operation = journal._fsync_path, m._sync_dir, journal._sync_operation
    def sync_file(path): events.append(('file', Path(path))); return original_file(path)
    def sync_dir(path): events.append(('directory', Path(path))); return original_dir(path)
    def operation(category, action):
        if category in ('manifest_file', 'manifest_replace', 'manifest_directory'):
            events.append((category, None))
        return original_operation(category, action)
    with monkeypatch.context() as patch:
        patch.setattr(journal, '_fsync_path', sync_file)
        patch.setattr(m, '_sync_dir', sync_dir)
        patch.setattr(journal, '_sync_operation', operation)
        journal._commit(journal.snapshot()['written_prefix'], False)
    for artifact in (value.output/'bootstrap.json', value.output.parent/'prior_config.json'):
        assert events.index(('file', artifact)) < events.index(('directory', artifact.parent))
        assert events.index(('directory', artifact.parent)) < events.index(('manifest_file', None))
    assert events.count(('directory', value.output)) == 2  # New bootstrap name, then manifest rename.
    assert events.count(('directory', value.output.parent)) == 1
    assert events.index(('manifest_file', None)) < events.index(('manifest_replace', None))
    assert events[-2:] == [('manifest_directory', None), ('directory', value.output)]
    assert all(path not in (value.output/'left', value.output/'left/frames') for kind, path in events if kind == 'directory')


@pytest.mark.parametrize('failed_phase', ['immutable_artifact', 'new_entry_directory',
                                        'manifest_file', 'manifest_replace', 'manifest_directory'])
def test_checkpoint_failure_barriers_never_advance_committed_prefix(owner, monkeypatch, failed_phase):
    monkeypatch.setattr(m, 'CHECKPOINT_INTERVAL_S', 3600.)
    value = owner(); journal = value.checkpoints
    journal._commit(journal.snapshot()['written_prefix'], False)
    previous = journal.snapshot()['checkpoint']
    now = finish_bootstrap(value)+10_000_000
    assert deliver(value, source(host=now), [])
    original = journal._sync_operation
    def failed(): raise OSError('synthetic failure at '+failed_phase)
    def operation(category, action):
        return original(category, failed if category == failed_phase else action)
    monkeypatch.setattr(journal, '_sync_operation', operation)
    value.close()
    snapshot = journal.snapshot()
    assert snapshot['checkpoint'] == previous and journal.generation == previous['generation']
    assert snapshot['error'].startswith('STORAGE_CHECKPOINT_FAILED')
    assert snapshot['sync_operation_timings'][failed_phase]['failures'] == 1
    final = json.loads((value.output/'status.json').read_text())
    assert final['state'] == 'FAILED' and final['storage']['checkpoint']['final'] is False
    # Before replacement failure, the previous on-disk manifest is preserved.
    # After rename a failed directory fsync does NOT justify advancing the
    # in-memory committed prefix, even if the new name happens to be visible.
    if failed_phase != 'manifest_directory':
        assert json.loads((value.output/'checkpoint.json').read_text()) == previous


def test_archive_queue_is_bounded_with_honest_pending_and_no_false_publication(owner, monkeypatch):
    value = owner(); now = finish_bootstrap(value)+10_000_000; published = []
    entered, release = threading.Event(), threading.Event()
    original = value.archives['left'].append
    def blocked(*args):
        entered.set(); assert release.wait(3)
        return original(*args)
    monkeypatch.setattr(value.archives['left'], 'append', blocked)
    try:
        for index in range(m.MAX_PENDING_OUTPUTS):
            host = now+index*210_000_000
            assert value.receive_source(source(sequence=index, host=host), host, serialize, published.append)
        assert entered.wait(1)
        with pytest.raises(m.InputFailure, match='ARCHIVE_QUEUE_FULL'):
            host += 210_000_000
            value.receive_source(source(sequence=10, host=host), host, serialize, published.append)
        state = value.status(host)
        assert state['pending_outputs'] == m.MAX_PENDING_OUTPUTS
        assert state['pending_reserved_bytes'] <= m.MAX_PENDING_BYTES and published == []
    finally:
        release.set()
    value.close()
    status = json.loads((value.output/'status.json').read_text())
    assert status['state'] == 'FAILED' and status['archived_outputs'] == m.MAX_PENDING_OUTPUTS
    assert status['published_frames'] == 0 and status['pending_outputs'] == 0
    assert status['storage']['checkpoint']['final'] is True


def test_normal_close_archives_accepted_pending_outputs_without_publishing_after_shutdown(owner):
    value = owner(); now = finish_bootstrap(value)+10_000_000
    published = []
    assert value.receive_source(source(host=now), now, serialize, published.append)
    value.close()
    status = json.loads((value.output/'status.json').read_text())
    assert status['state'] == 'STOPPED' and status['failure_reason'] is None
    assert status['archived_outputs'] == 1 and status['published_frames'] == 0 and published == []
    assert status['outputs_discarded_on_close'] == 1 and status['pending_outputs'] == 0
    assert status['storage']['checkpoint']['final'] is True
    assert status['storage']['accepted_outputs_not_checkpointed'] == 0
    assert not value.worker.thread.is_alive() and not value.checkpoints.thread.is_alive()


def test_stale_archived_result_is_not_redated_or_published_even_with_fresh_sources(owner):
    value = owner(); now = finish_bootstrap(value)+10_000_000; published = []
    message = source(host=now)
    assert value.receive_source(message, now, serialize, published.append)
    wait_until(lambda: value.worker.jobs.unfinished_tasks == 0)
    later = now+3_000_000_001
    value.receive_wheel(wheel(21, later, (5, 5)), later)
    value.receive_imu(imu(21, later, gyro=(.3, 0, 0)), later)
    # Fresh input prevents a source-disconnect classification, while the old
    # completed result must still fail its own unchanged source-time age gate.
    assert value.receive_source(source(sequence=1, host=later), later, serialize, published.append)
    with pytest.raises(m.InputFailure, match='ARCHIVED_OUTPUT_STALE_OR_FUTURE'):
        value.poll_io(later, clock=lambda: later)
    assert published == [] and m._stamp(message.header.stamp) == WALL+now


def test_each_pending_result_uses_fresh_clock_after_previous_publish(owner):
    value = owner(); first_host = finish_bootstrap(value)+10_000_000
    second_host = first_host+210_000_000
    current = [second_host]
    published = []
    messages = [source(sequence=0, host=first_host), source(sequence=1, host=second_host)]

    def delayed_publish(cloud):
        published.append(cloud)
        # Model time spent in the first publish without sleeping. Refresh
        # health separately so the second result fails its own age check.
        current[0] += m.STALE_NS+1
        later = current[0]
        value.receive_wheel(wheel(21, later, (5, 5)), later)
        value.receive_imu(imu(21, later, gyro=(.3, 0, 0)), later)
        assert value.receive_source(source(sequence=2, host=later), later, serialize, published.append)

    for message in messages:
        assert value.receive_source(message, message.host_monotonic_ns, serialize, delayed_publish)
    wait_until(lambda: value.worker.jobs.unfinished_tasks == 0)
    with pytest.raises(m.InputFailure, match='ARCHIVED_OUTPUT_STALE_OR_FUTURE'):
        value.poll_io(second_host, clock=lambda: current[0])
    assert len(published) == value.published_frames == 1
    assert m._stamp(published[0].header.stamp) == WALL+first_host
    assert [m._stamp(message.header.stamp) for message in messages] == [WALL+first_host, WALL+second_host]
    assert value.last_published_stamp_ns == WALL+first_host
    assert value.status(current[0])['callback_and_io_timings']['archive_to_publish_wait']['last_duration_ns'] == 210_000_000


def test_archive_path_uses_no_fsync_before_publish_and_close_syncs_all(owner, monkeypatch):
    monkeypatch.setattr(m, 'CHECKPOINT_INTERVAL_S', 3600.)
    value = owner(); now = finish_bootstrap(value)+10_000_000; published = []
    synced_threads = []
    original = m.os.fsync
    def record(descriptor):
        synced_threads.append(threading.current_thread().name)
        return original(descriptor)
    monkeypatch.setattr(m.os, 'fsync', record)
    assert deliver(value, source(host=now), published)
    assert len(published) == 1 and synced_threads == []
    value.close()
    assert 'mapping-checkpoint' in synced_threads
    assert 'mapping-archive' not in synced_threads
    assert value.failure_reason is None


@pytest.mark.parametrize('mode', [None, '', True, 'memory_only'])
def test_queued_publication_requires_explicit_recognized_mode(mode):
    candidate = config(); candidate['archive_publication_mode'] = mode
    with pytest.raises(m.InputFailure, match='INVALID_ARCHIVE_PUBLICATION_MODE'):
        m.validate_config(candidate)


def test_queued_disk_stall_beyond_old_queue_keeps_fresh_publication_then_drains_all(owner, monkeypatch):
    monkeypatch.setattr(m, 'CHECKPOINT_INTERVAL_S', 3600.)
    value = owner(publication_mode='queued_realtime'); now = finish_bootstrap(value)+10_000_000
    entered, release = threading.Event(), threading.Event()
    original = value.archives['left'].append
    def blocked(*args):
        entered.set(); assert release.wait(5)
        return original(*args)
    monkeypatch.setattr(value.archives['left'], 'append', blocked)
    published = []; expected = []
    try:
        for index in range(25):
            host = now+index*200_000_000
            value.receive_wheel(wheel(21+index, host, (5, 5)), host)
            value.receive_imu(imu(21+index, host, gyro=(.3, 0, 0)), host)
            message = source(sequence=index, host=host); expected.append(serialize(message))
            assert value.receive_source(message, host, serialize, published.append, clock=lambda: host)
            assert len(published) == index+1
            value.poll_io(host, clock=lambda: host); value.check_stale(host)
        assert entered.wait(1)
        status = value.status(host)
        assert status['pending_outputs'] == status['published_frames'] == 25
        assert status['archived_outputs'] == 0 and status['outputs_discarded_on_close'] == 0
        assert status['published_outputs_not_confirmed_archived'] == 25
        assert status['published_outputs_not_checkpointed'] == 25
        assert status['pending_reserved_bytes'] < 25*m.MAX_CDR_BYTES
        assert status['archive_queue_limits']['outputs'] == 100
        assert status['storage']['checkpoint'] is None
        assert 'QUEUED_BEFORE_PUBLICATION_NOT_YET_DURABLE' in status['storage']['publication_contract']
        assert [m._stamp(cloud.header.stamp) for cloud in published] == [WALL+now+i*200_000_000 for i in range(25)]
    finally:
        release.set()
    value.close()
    status = json.loads((value.output/'status.json').read_text())
    assert status['state'] == 'STOPPED' and status['failure_reason'] is None
    assert status['enqueued_outputs'] == status['published_frames'] == status['archived_outputs'] == 25
    assert status['pending_outputs'] == status['pending_reserved_bytes'] == status['outputs_discarded_on_close'] == 0
    assert status['published_outputs_not_confirmed_archived'] == status['published_outputs_not_checkpointed'] == 0
    assert status['storage']['checkpoint']['final'] is True
    assert status['storage']['checkpoint']['archived_outputs'] == 25
    rows = [json.loads(line) for line in (value.output/'left/frames.jsonl').read_text().splitlines()]
    for index, (row, cdr) in enumerate(zip(rows, expected)):
        actual = (value.output/'left'/row['file']).read_bytes()
        assert actual == cdr and hashlib.sha256(actual).hexdigest() == row['cdr_sha256']
        assert row['header_stamp_ns'] == WALL+now+index*200_000_000
    assert len(rows) == len(published) == 25  # Archive completion/close never publishes.


def test_queued_all_freezes_originals_offsets_and_publishes_exactly_once(owner, monkeypatch):
    value = owner('all', publication_mode='queued_realtime'); now = finish_bootstrap(value)+10_000_000
    entered, release = threading.Event(), threading.Event()
    original = value.archives['left'].append
    def blocked(*args):
        entered.set(); assert release.wait(3)
        return original(*args)
    monkeypatch.setattr(value.archives['left'], 'append', blocked)
    left, right = source(host=now), source('right', host=now+30_000_000)
    originals = [serialize(left), serialize(right)]; stamps = [WALL+now, WALL+now+30_000_000]
    published = []
    try:
        assert not value.receive_source(left, now, serialize, published.append, clock=lambda: now)
        assert value.receive_source(right, now+30_000_000, serialize, published.append, clock=lambda: now+30_000_000)
        assert entered.wait(1) and len(published) == 1
        output_bytes = bytes(published[0].data)
        layout = validate_pointcloud2(published[0])
        assert m._stamp(published[0].header.stamp) == stamps[1]
        offsets = np.frombuffer(output_bytes, dtype=np.dtype({'names':['t','side'], 'formats':['<f8','u1'],
                                    'offsets':[16,24], 'itemsize':32}))
        assert layout['point_count'] == 6
        np.testing.assert_allclose(offsets['t'], [-.03]*3+[0.]*3)
        assert offsets['side'].tolist() == [1]*3+[2]*3
        # Neither later caller mutation nor a mutable publish argument may
        # rewrite the already accepted source CDR or pair-output hash.
        left.cloud.data = right.cloud.data = b'mutated by caller'
        left.header.stamp.sec = right.header.stamp.sec = 0
        left.frame_sequence = right.frame_sequence = 999
        published[0].data = b'mutated display copy'
        value.transforms['T_base_side']['right'][0][3] = 999.
    finally:
        release.set()
    settle(value, now+30_000_000)
    for _ in range(3): value.poll_io(now+30_000_000, clock=lambda: now+30_000_000)
    value.close()
    assert len(published) == value.published_frames == value.archived_outputs == 1
    pair = json.loads((value.output/'pairs.jsonl').read_text())
    assert pair['archive_publication_mode'] == 'queued_realtime'
    assert 'QUEUED_BEFORE_PUBLICATION_NOT_YET_DURABLE' in pair['publication_state']
    assert pair['output_data_sha256'] == hashlib.sha256(output_bytes).hexdigest()
    assert [row['source_stamp_ns'] for row in pair['sources']] == stamps
    assert pair['sources'][1]['T_base_side'][0][3] != 999.
    for side, raw, expected_stamp in zip(('left', 'right'), originals, stamps):
        row = json.loads((value.output/side/'frames.jsonl').read_text())
        assert row['header_stamp_ns'] == expected_stamp and row['frame_sequence'] == 0
        assert (value.output/side/row['file']).read_bytes() == raw


@pytest.mark.parametrize('elapsed,reason', [(500_000_001, 'IMU_STALE_OR_FUTURE'), (-1, 'REALTIME_OUTPUT_STALE_OR_FUTURE')])
def test_queued_rechecks_actual_clock_after_freezing_without_redating(owner, elapsed, reason):
    value = owner(publication_mode='queued_realtime'); now = finish_bootstrap(value)+10_000_000
    if elapsed > 0:
        value.receive_wheel(wheel(21, now), now); value.receive_imu(imu(21, now), now)
    message = source(host=now); published = []; current = [now]
    def slow_serialize(source_message):
        result = serialize(source_message); current[0] += elapsed
        return result
    with pytest.raises(m.InputFailure, match=reason):
        value.receive_source(message, now, slow_serialize, published.append, clock=lambda: current[0])
    assert not published and m._stamp(message.header.stamp) == WALL+now
    value.close()
    assert value.state == 'FAILED' and value.enqueued_outputs == value.archived_outputs == 1
    assert value.published_frames == 0 and value.outputs_discarded_on_close == 1
    assert value.pending_bytes == 0 and not value.pending


def test_queued_stale_original_rejected_before_queue_acceptance(owner):
    value = owner(publication_mode='queued_realtime'); now = finish_bootstrap(value)+10_000_000
    with pytest.raises(m.InputFailure, match='SOURCE_STALE_OR_FUTURE'):
        value.receive_source(source(host=now), now+m.STALE_NS+1, serialize, lambda _: pytest.fail('stale publish'))
    assert value.enqueued_outputs == value.published_frames == 0


def test_queued_fast_archive_completion_before_publish_returns_does_not_discard_or_republish(owner):
    value = owner(publication_mode='queued_realtime'); now = finish_bootstrap(value)+10_000_000
    published = []
    def publish(cloud):
        wait_until(lambda: value.worker.jobs.unfinished_tasks == 0)
        # This is deliberately more aggressive than the real single-threaded
        # ROS loop: consume the early completion before publish returns.
        value.poll_io(now, clock=lambda: now)
        assert value.archived_outputs == 1 and value.published_frames == 0
        published.append(cloud)
    assert value.receive_source(source(host=now), now, serialize, publish, clock=lambda: now)
    value.close()
    assert len(published) == value.published_frames == value.archived_outputs == 1
    assert value.outputs_discarded_on_close == 0
    assert value.state == 'STOPPED' and value.failure_reason is None


def test_queued_freeze_cannot_publish_with_stale_wheel_even_when_imu_is_fresh(owner):
    value = owner(publication_mode='queued_realtime'); now = finish_bootstrap(value)+10_000_000
    value.receive_wheel(wheel(21, now), now); value.receive_imu(imu(21, now), now)
    current = [now]
    def delayed(source_message):
        current[0] += m.BOOT_GAP_NS+1
        value.receive_imu(imu(22, current[0]), current[0])
        return serialize(source_message)
    with pytest.raises(m.InputFailure, match='WHEEL_STALE_OR_FUTURE'):
        value.receive_source(source(host=now), now, delayed, lambda _: pytest.fail('stale wheel publish'),
                             clock=lambda: current[0])
    assert value.enqueued_outputs == 1 and value.published_frames == 0
    value.close()
    assert value.archived_outputs == value.outputs_discarded_on_close == 1


@pytest.mark.parametrize('kind', ['empty', 'mutable', 'oversized'])
def test_queued_invalid_cdr_envelope_is_rejected_before_acceptance(owner, kind):
    value = owner(publication_mode='queued_realtime'); now = finish_bootstrap(value)+10_000_000
    cdr = b'' if kind == 'empty' else bytearray(b'not immutable') if kind == 'mutable' else b'x'*(m.MAX_CDR_BYTES+1)
    with pytest.raises(m.InputFailure, match='INVALID_CDR_ENVELOPE'):
        value.receive_source(source(host=now), now, lambda _: cdr, lambda _: pytest.fail('invalid CDR publish'),
                             clock=lambda: now)
    assert value.enqueued_outputs == value.published_frames == value.pending_bytes == 0


@pytest.mark.parametrize('failure', ['publish', 'cdr_write', 'checkpoint'])
def test_queued_real_failure_preserves_honest_publication_and_archive_counts(owner, monkeypatch, failure):
    monkeypatch.setattr(m, 'CHECKPOINT_INTERVAL_S', 3600.)
    value = owner(publication_mode='queued_realtime'); now = finish_bootstrap(value)+10_000_000
    entered, release = threading.Event(), threading.Event()
    original = value.archives['left'].append; published = []
    def archive(*args):
        entered.set(); assert release.wait(3)
        if failure == 'cdr_write': raise OSError('synthetic queued CDR write error')
        return original(*args)
    monkeypatch.setattr(value.archives['left'], 'append', archive)
    def publish(cloud):
        if failure == 'publish': raise RuntimeError('synthetic publisher unavailable')
        published.append(cloud)
    try:
        if failure == 'publish':
            with pytest.raises(m.InputFailure, match='synthetic publisher unavailable'):
                value.receive_source(source(host=now), now, serialize, publish, clock=lambda: now)
        else:
            assert value.receive_source(source(host=now), now, serialize, publish, clock=lambda: now)
        assert entered.wait(1)
    finally:
        release.set()
    if failure == 'cdr_write':
        wait_until(lambda: value.worker.error is not None)
        with pytest.raises(m.InputFailure, match='synthetic queued CDR write error'):
            value.poll_io(now, clock=lambda: now)
        with pytest.raises(m.InputFailure):
            value.receive_source(source(sequence=1, host=now+200_000_000), now+200_000_000, serialize, publish)
    if failure == 'checkpoint':
        def fail_sync(path): raise OSError('synthetic queued checkpoint failure')
        monkeypatch.setattr(value.checkpoints, '_fsync_path', fail_sync)
    value.close()
    status = json.loads((value.output/'status.json').read_text())
    assert status['state'] == 'FAILED' and status['failure_reason']
    assert status['enqueued_outputs'] == 1
    assert status['published_frames'] == len(published) == (0 if failure == 'publish' else 1)
    assert status['outputs_discarded_on_close'] == (1 if failure == 'publish' else 0)
    assert status['archived_outputs'] == (0 if failure == 'cdr_write' else 1)
    if failure == 'checkpoint': assert status['storage']['checkpoint'] is None


@pytest.mark.parametrize('limit', ['count', 'bytes', 'checkpoint'])
def test_queued_limits_fail_before_accepting_or_publishing_an_extra_output(owner, monkeypatch, limit):
    monkeypatch.setattr(m, 'CHECKPOINT_INTERVAL_S', 3600.)
    value = owner(publication_mode='queued_realtime'); now = finish_bootstrap(value)+10_000_000
    entered, release = threading.Event(), threading.Event()
    original = value.archives['left'].append
    def blocked(*args): entered.set(); assert release.wait(5); return original(*args)
    monkeypatch.setattr(value.archives['left'], 'append', blocked)
    if limit == 'bytes': monkeypatch.setattr(m, 'MAX_PENDING_BYTES', 1)
    if limit == 'checkpoint': monkeypatch.setattr(m, 'MAX_UNCHECKPOINTED_OUTPUTS', 1)
    count = 100 if limit == 'count' else (1 if limit == 'checkpoint' else 0)
    published = []
    try:
        for index in range(count):
            host = now+index*200_000_000
            value.receive_wheel(wheel(21+index, host), host); value.receive_imu(imu(21+index, host), host)
            assert value.receive_source(source(sequence=index, host=host), host, serialize, published.append, clock=lambda: host)
        if count: assert entered.wait(1)
        host = now+count*200_000_000
        value.receive_wheel(wheel(21+count, host), host); value.receive_imu(imu(21+count, host), host)
        reason = 'STORAGE_CHECKPOINT_BACKLOG' if limit == 'checkpoint' else 'ARCHIVE_QUEUE_FULL'
        with pytest.raises(m.InputFailure, match=reason):
            value.receive_source(source(sequence=count, host=host), host, serialize, published.append, clock=lambda: host)
        assert len(published) == value.enqueued_outputs == count
        assert value.published_frames == count and value.outputs_discarded_on_close == 0
        assert len(value.pending) == count and value.pending_bytes <= m.MAX_PENDING_BYTES
    finally:
        release.set()
    value.close()
    assert value.archived_outputs == count and value.published_frames == count
    assert value.outputs_discarded_on_close == 0 and len(published) == count


def test_queued_written_unsynced_prefix_can_exceed_100_but_stops_at_256(owner, monkeypatch):
    # An idle sync worker models a complete storage-sync stall; the separate
    # writer still flushes every original CDR and the live path checks freshness.
    monkeypatch.setattr(m, 'CHECKPOINT_INTERVAL_S', 3600.)
    assert m.MAX_UNCHECKPOINTED_OUTPUTS == 256
    assert m.MAX_REALTIME_PENDING_OUTPUTS == 100 and m.MAX_PENDING_BYTES == 128*1024*1024
    value = owner(publication_mode='queued_realtime'); now = finish_bootstrap(value)+10_000_000
    published = []
    for index in range(256):
        host = now+index*200_000_000
        value.receive_wheel(wheel(21+index, host), host); value.receive_imu(imu(21+index, host), host)
        assert deliver(value, source(sequence=index, host=host), published)
        value.check_stale(host)
        assert value.pending_bytes == 0 and not value.pending
        assert value.archived_outputs == index+1
        assert value.checkpoints.snapshot()['checkpoint'] is None
    assert len(published) == 256
    assert [m._stamp(cloud.header.stamp) for cloud in published] == [WALL+now+i*200_000_000 for i in range(256)]
    host = now+256*200_000_000
    value.receive_wheel(wheel(277, host), host); value.receive_imu(imu(277, host), host)
    with pytest.raises(m.InputFailure, match='STORAGE_CHECKPOINT_BACKLOG'):
        value.receive_source(source(sequence=256, host=host), host, serialize, published.append, clock=lambda: host)
    assert value.enqueued_outputs == value.published_frames == value.archived_outputs == len(published) == 256
    value.close()
    final = json.loads((value.output/'status.json').read_text())
    assert final['state'] == 'FAILED' and final['failure_reason'] == 'STORAGE_CHECKPOINT_BACKLOG'
    assert final['storage']['checkpoint']['final'] is True
    assert final['storage']['checkpoint']['archived_outputs'] == 256
    assert final['storage']['accepted_outputs_not_checkpointed'] == final['pending_reserved_bytes'] == 0


@pytest.mark.parametrize('mode', ['left', 'right', 'all'])
def test_on_close_writes_over_old_checkpoint_count_without_runtime_sync(owner, monkeypatch, mode):
    # Real file writes/flushes, with a very short legacy interval to ensure this
    # policy really waits for close instead of repeatedly scheduling fsync.
    monkeypatch.setattr(m, 'CHECKPOINT_INTERVAL_S', .001)
    value = owner(mode, publication_mode='queued_realtime', checkpoint_policy='on_close')
    now = finish_bootstrap(value)+10_000_000; published = []; hashes = []
    for index in range(270):
        host = now+index*200_000_000
        value.receive_wheel(wheel(21+index, host), host)
        value.receive_imu(imu(21+index, host), host)
        for side in value.sides:
            message = source(side=side, sequence=index, host=host)
            hashes.append((side, index+1, hashlib.sha256(serialize(message)).hexdigest()))
            deliver(value, message, published)
        assert value.archived_outputs == index+1 and value.pending_bytes == 0
        value.check_stale(host)
    before = value.status(host)
    assert len(published) == 270 and before['storage']['checkpoint'] is None
    assert before['storage']['checkpoint_interval_s'] is None
    assert before['storage']['sync_operation_timings'] == {}
    assert before['storage']['data_sync_workers']['jobs_started'] == 0
    assert before['archive_queue_limits']['accepted_not_checkpointed'] is None
    assert before['storage']['accepted_outputs_not_checkpointed'] == 270
    assert not (value.output/'checkpoint.json').exists()
    value.close()
    final = json.loads((value.output/'status.json').read_text())
    assert final['state'] == 'STOPPED' and final['failure_reason'] is None
    assert final['storage']['checkpoint']['final'] is True
    assert final['storage']['checkpoint']['checkpoint_policy'] == 'on_close'
    assert final['storage']['checkpoint']['archived_outputs'] == 270
    assert final['storage']['accepted_outputs_not_checkpointed'] == final['pending_reserved_bytes'] == 0
    assert final['storage']['sync_operation_timings']['cdr_file']['count'] == 270*len(value.sides)
    for side, index, sha in hashes:
        assert hashlib.sha256((value.output/side/'frames'/('%08d.cdr' % index)).read_bytes()).hexdigest() == sha


@pytest.mark.parametrize('phase', ['write', 'sync'])
def test_on_close_storage_failure_never_claims_success(owner, monkeypatch, phase):
    value = owner(publication_mode='queued_realtime', checkpoint_policy='on_close')
    now = finish_bootstrap(value)+10_000_000
    def failed(*args):
        raise OSError('synthetic on-close '+phase+' failure')
    if phase == 'write':
        monkeypatch.setattr(value.archives['left'], 'append', failed)
        assert value.receive_source(source(host=now), now, serialize, lambda _: None, clock=lambda: now)
        wait_until(lambda: value.worker.error is not None)
        with pytest.raises(m.InputFailure, match='ARCHIVE_WRITE_FAILED'):
            value.poll_io(now)
    else:
        assert deliver(value, source(host=now), [])
        monkeypatch.setattr(value.checkpoints, '_fsync_path', failed)
    value.close()
    final = json.loads((value.output/'status.json').read_text())
    assert final['state'] == 'FAILED' and final['failure_reason']
    if phase == 'sync':
        assert final['storage']['checkpoint'] is None
        assert final['storage']['accepted_outputs_not_checkpointed'] == 1


@pytest.mark.parametrize('bad_policy', [None, '', 'off', False])
def test_unknown_checkpoint_policy_rejected_before_files_created(tmp_path, bad_policy):
    candidate = config(); candidate['archive_checkpoint_policy'] = bad_policy
    with pytest.raises(m.InputFailure, match='INVALID_ARCHIVE_CHECKPOINT_POLICY'):
        m.MappingInput(candidate, tmp_path, tmp_path/'never_created', started_ns=START)
    assert not (tmp_path/'never_created').exists()


@pytest.mark.parametrize('phase', ['freeze', 'queue_accept', 'publication_clock'])
def test_stop_inside_source_callback_never_publishes_or_checks_stale_but_drains_accepted(owner, monkeypatch, phase):
    value = owner(publication_mode='queued_realtime'); now = finish_bootstrap(value)+10_000_000
    stopped = [False]; published = []
    original_submit = value.worker.submit

    def freeze(message):
        result = serialize(message)
        if phase == 'freeze': stopped[0] = True
        return result

    def submit(job):
        original_submit(job)
        if phase == 'queue_accept': stopped[0] = True

    def clock():
        if phase == 'publication_clock': stopped[0] = True
        return now+2_000_000_000  # Would fail the unchanged freshness checks.

    monkeypatch.setattr(value.worker, 'submit', submit)
    accepted = value.receive_source(source(host=now), now, freeze, published.append,
        clock=clock, should_stop=lambda: stopped[0])
    assert accepted == (phase != 'freeze')
    assert stopped[0] and published == [] and value.failure_reason is None
    assert value.enqueued_outputs == int(accepted)
    value.close()
    final = json.loads((value.output/'status.json').read_text())
    assert final['state'] == 'STOPPED' and final['failure_reason'] is None
    assert final['archived_outputs'] == int(accepted) and final['published_frames'] == 0
    assert final['outputs_discarded_on_close'] == int(accepted)
    assert final['storage']['checkpoint']['final'] is True
    assert final['storage']['checkpoint']['archived_outputs'] == int(accepted)


def test_stop_during_strict_result_publish_discards_remaining_results_without_stale(owner):
    value = owner(); now = finish_bootstrap(value)+10_000_000
    stopped = [False]; published = []

    def publish(cloud):
        published.append(cloud)
        stopped[0] = True

    for index in range(2):
        host = now+index*200_000_000
        value.receive_wheel(wheel(21+index, host), host); value.receive_imu(imu(21+index, host), host)
        assert value.receive_source(source(sequence=index, host=host), host, serialize, publish)
    wait_until(lambda: value.worker.jobs.unfinished_tasks == 0)
    clocks = iter((now+200_000_000, now+20_000_000_000))
    value.poll_io(now+200_000_000, clock=lambda: next(clocks), should_stop=lambda: stopped[0])
    assert value.failure_reason is None and len(published) == value.published_frames == 1
    assert value.archived_outputs == 2 and value.outputs_discarded_on_close == 1
    value.close()
    assert value.state == 'STOPPED' and value.checkpoints.snapshot()['checkpoint']['archived_outputs'] == 2


@pytest.mark.parametrize('scenario', ['spin_stop', 'callback_stop', 'poll_stop',
                                    'callback_error', 'callback_error_after_stop', 'spin_error'])
def test_main_owns_signals_drains_before_ros_shutdown_and_keeps_real_errors(tmp_path, monkeypatch, capsys, scenario):
    import signal
    import sys
    from types import ModuleType
    events, callbacks, logs = [], {}, []
    context = {'active': False}
    previous = {number: signal.getsignal(number) for number in (signal.SIGINT, signal.SIGTERM)}

    def request_stop():
        signal.getsignal(signal.SIGINT)(signal.SIGINT, None)
        assert context['active']  # The handler must never shut down ROS.

    class Owner:
        sides = ('left',)
        failure_reason = None
        def timed(self, label, operation): return operation()
        def fail(self, reason): self.failure_reason = self.failure_reason or str(reason)
        def receive_source(self, *args, should_stop, **kwargs):
            events.append('source')
            if scenario in ('callback_stop', 'callback_error_after_stop'):
                request_stop(); assert should_stop()
            if scenario in ('callback_error', 'callback_error_after_stop'):
                raise RuntimeError('synthetic callback argument conversion failure')
        def receive_imu(self, *args): events.append('unexpected_imu_after_stop')
        def receive_wheel(self, *args): events.append('unexpected_wheel_after_stop')
        def poll_io(self, now, should_stop):
            events.append('poll')
            assert scenario == 'poll_stop'
            request_stop(); assert should_stop()
        def check_stale(self, now): raise AssertionError('stale checked after requested stop')
        def write_status(self, now): events.append('status')
        def close(self):
            assert context['active']
            assert 'destroy_node' not in events and 'shutdown' not in events
            events.append('archive_close_complete')

    owner = Owner()
    class Node:
        def __init__(self, name): events.append('node')
        def create_publisher(self, *args): return NS(publish=lambda _: events.append('unexpected_publish'))
        def create_subscription(self, kind, topic, callback, qos): callbacks[topic] = callback
        def get_logger(self): return NS(error=logs.append)
        def destroy_node(self):
            assert 'archive_close_complete' in events and context['active']
            events.append('destroy_node')

    no_signals = object()
    def initialize(*, args, signal_handler_options):
        assert args == [] and signal_handler_options is no_signals
        context['active'] = True; events.append('init_no_signals')
    def shutdown():
        assert events[-1] == 'destroy_node'
        context['active'] = False; events.append('shutdown')
    def spin_once(node, timeout_sec):
        events.append('spin')
        if scenario == 'spin_error': raise RuntimeError('synthetic executor failure')
        if scenario == 'spin_stop': request_stop()
        if scenario.startswith('callback_'):
            callbacks['/wc_mapping/lidar_left/source_frame_filtered'](NS(side='left'))
        if scenario in ('spin_stop', 'callback_stop', 'callback_error_after_stop'):
            callbacks[m.IMU_TOPIC](NS())  # A callback already ready in this spin.

    def module(name, **attributes):
        value = ModuleType(name); value.__dict__.update(attributes)
        monkeypatch.setitem(sys.modules, name, value)
    module('wc_runtime.cli', ROOT=tmp_path, target=lambda: None)
    module('rclpy', init=initialize, ok=lambda: context['active'], shutdown=shutdown, spin_once=spin_once)
    module('rclpy.node', Node=Node)
    module('rclpy.qos', QoSProfile=lambda **kwargs: NS(**kwargs),
           ReliabilityPolicy=NS(RELIABLE=1, BEST_EFFORT=2), DurabilityPolicy=NS(VOLATILE=2))
    module('rclpy.serialization', serialize_message=serialize)
    module('rclpy.signals', SignalHandlerOptions=NS(NO=no_signals))
    module('sensor_msgs.msg', PointCloud2=NS, PointField=NS)
    module('std_msgs.msg', String=NS)
    module('wc_interfaces.msg', SourceFrame=NS, H30Frame=NS)
    def make_owner(*args, close_timeout_s):
        assert close_timeout_s == m.CLOSE_TIMEOUT_S
        return owner
    monkeypatch.setattr(m, 'MappingInput', make_owner)
    path = tmp_path/'runtime_config.json'; path.write_text(json.dumps(config()), encoding='utf-8')
    result = m.main(['--config', str(path), '--output-root', str(tmp_path/'input')])
    failed = 'error' in scenario
    assert result == int(failed) and bool(owner.failure_reason) == failed
    assert events[-3:] == ['archive_close_complete', 'destroy_node', 'shutdown']
    assert not context['active']
    assert not any(event.startswith('unexpected_') for event in events)
    assert ('poll' in events) == (scenario == 'poll_stop')
    assert {number: signal.getsignal(number) for number in previous} == previous
    stderr = capsys.readouterr().err
    assert ('Traceback (most recent call last)' in stderr) == failed
    if failed: assert 'synthetic' in stderr
