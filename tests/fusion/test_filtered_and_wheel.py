from copy import deepcopy
import hashlib
from types import SimpleNamespace as NS
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from wc_fusion import FusionError, Policy
from wc_fusion.source_join import RawFilteredJoin
from wc_fusion.wheel_prior import WheelMotionHistory
from test_core import frame, make_fusion


def message(sequence=1):
    stamp=NS(sec=1,nanosec=0)
    header=NS(frame_id='lidar_left',stamp=stamp)
    return NS(session_id='s',side='left',sensor_id='L',stream_epoch='boot',frame_sequence=sequence,
              header=header,cloud=NS(header=header,data=b'raw'),source_config_hash='cfg',
              diagnostic_flags=['raw_source_config_hash=cfg', 'before_host_filter_cloud_sha256='+hashlib.sha256(b'raw').hexdigest()],
              device_timestamp_valid=True,device_timestamp_raw=100,
              device_timestamp_unit='raw',device_time_components_valid=True,device_timestamp_seconds=1,
              device_timestamp_nanoseconds=0,device_time_type='relative',device_sync_state='unknown',
              host_receive_time=stamp,host_monotonic_ns=1,common_time_valid=False,common_time_ns=0,
              clock_model_id='',time_source='arrival_only',uncertainty_valid=False,uncertainty_ns=0,
              coordinate_convention='FLU',units='m')


def test_exact_raw_filtered_pairing_reorders_without_crossing_frames():
    join=RawFilteredJoin(capacity=2,timeout_ns=100)
    raw,filtered=message(),message()
    assert join.push('filtered',filtered,1) is None
    assert join.push('raw',message(2),2) is None
    assert join.push('raw',raw,3)==(raw,filtered)
    assert join.push('filtered',filtered,4) is None
    assert len(join.pending)==1
    changed=deepcopy(filtered);changed.cloud.data=b'changed'
    with pytest.raises(FusionError,match='CONTENT_CHANGED'):join.push('filtered',changed,5)


def test_filtered_reference_must_match_original_content():
    join=RawFilteredJoin()
    raw,filtered=message(),message()
    raw.cloud.data=b'other'
    join.push('raw',raw,1)
    with pytest.raises(FusionError,match='CONTENT_REFERENCE_MISMATCH'):join.push('filtered',filtered,2)


def test_filter_pair_rejects_mismatched_capture_time_and_bounds():
    join=RawFilteredJoin(capacity=1,timeout_ns=10)
    join.push('raw',message(),1)
    wrong=message();wrong.host_monotonic_ns=2
    with pytest.raises(FusionError,match='ACQUISITION_MISMATCH'):join.push('filtered',wrong,2)
    join.push('raw',message(2),3)
    with pytest.raises(FusionError,match='QUEUE_FULL'):join.push('raw',message(3),4)
    with pytest.raises(FusionError,match='TIMEOUT'):join.expire(14)


def test_filtered_icp_geometry_is_separate_from_measured_map_rays():
    fusion=make_fusion()
    left,right=frame('left'),frame('right')
    original=left['points'].copy()
    left['original_points']=original.copy()
    left['points'][:,0]+=0.25
    left['filter_config_hash']='filtered-cfg'
    left['original_config_hash']='raw-cfg'
    assert fusion.push(left,100_000_000) is None
    output=fusion.push(right,100_000_000)
    np.testing.assert_allclose(output['points'][:40],left['points'])
    np.testing.assert_allclose(output['observations'][0]['points'],original)
    assert output['observations'][0]['source_config_hash']=='raw-cfg'
    assert output['observations'][0]['filter_config_hash']=='filtered-cfg'
    assert set(output['source_codes'])=={1,2}


def test_filtered_count_changes_keep_original_indices_and_require_raw():
    f=make_fusion()
    left,right=frame('left'),frame('right')
    original=np.vstack((left['points'], [np.nan,0,0], [100.,0,0]))
    left.update(original_points=original, points=left['points'][:25], representation='host_filtered')
    f.push(left,100_000_000)
    b=f.push(right,100_000_000)
    observation=b['observations'][0]
    assert b['source_counts']['left']==25
    assert observation['raw_count']==42 and observation['valid_count']==40
    np.testing.assert_array_equal(observation['valid_indices'],np.arange(40))
    np.testing.assert_allclose(observation['points'],original[:40])
    del left['original_points']
    with pytest.raises(FusionError,match='REQUIRES_ORIGINAL_RAYS'):make_fusion().push(left,100_000_000)


def wheel_config(offset=0.0):
    t=np.eye(4);t[1,3]=offset
    return dict(coordinate_status='SYNTHETIC',time_status='SYNTHETIC',T_rig_axle=t)


def test_wheel_prior_requires_geometry_time_and_fresh_samples():
    with pytest.raises(FusionError,match='UNVERIFIED'):
        WheelMotionHistory(Policy(),wheel_config(),'real')
    history=WheelMotionHistory(Policy(),wheel_config(),'synthetic')
    with pytest.raises(FusionError,match='TWO_SAMPLES'):history.predict(1_000_000_000)
    history.add_wheel(1_000_000_000,np.eye(4))
    p=np.eye(4);p[0,3]=0.01
    history.add_wheel(1_100_000_000,p)
    np.testing.assert_allclose(history.predict(1_150_000_000)[0,3],0.015)
    with pytest.raises(FusionError,match='EXPIRED'):history.predict(1_300_000_000)
    history.add(1_100_000_000,np.eye(4),'icp')
    np.testing.assert_allclose(history.predict(1_150_000_000)[0,3],0.015)


def test_wheel_prediction_transforms_axle_motion_to_offset_lidar():
    h=WheelMotionHistory(Policy(),wheel_config(0.3),'synthetic')
    h.add_wheel(1_000_000_000,np.eye(4))
    pose=np.eye(4);pose[:3,:3]=Rotation.from_euler('z',0.01).as_matrix()
    h.add_wheel(1_100_000_000,pose)
    prediction=h.predict(1_100_000_000)
    np.testing.assert_allclose(prediction,pose @ np.linalg.inv(h.T_rig_axle))
    assert abs(prediction[0,3])>0.0029
    assert not h.status()['weighted_estimator_constraint']


def test_dual_fusion_uses_wheel_prediction_and_keeps_both_sources():
    f=make_fusion()
    f.history=WheelMotionHistory(f.policy,wheel_config(),'synthetic')
    f.history.add_wheel(900_000_000,np.eye(4))
    p=np.eye(4);p[0,3]=0.01
    f.history.add_wheel(1_000_000_000,p)
    assert f.push(frame('left',stamp=1_000_000_000),100_000_000) is None
    b=f.push(frame('right',stamp=1_010_000_000),100_000_000)
    assert b['compensation_mode']=='wheel_odometry_prediction'
    np.testing.assert_allclose(b['observations'][0]['T_ref_rig_at_source'][0,3],-0.001,atol=1e-12)
    assert b['source_counts']=={'left':40,'right':40}
