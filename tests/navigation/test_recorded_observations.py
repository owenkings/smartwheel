import json
import math

import numpy as np
import pytest

from wc_navigation.recorded_observations import project_observation, CausalObservationBuffer


def fixed():
    matrix=np.eye(4);matrix[:2,3]=[.323,.22875];return matrix


def observation(stamp, points=None):
    return project_observation(np.array([[1.,0.,0.]]) if points is None else points,
                               stamp,fixed(),-.7635)


def packet(stamp,received=None,points=None):
    return {'stamp_ns':stamp,'received_time_ns':stamp if received is None else received,
            'observation':observation(stamp,points)}


def test_planar_lever_arm_preserves_points_and_height_without_self_return_removal():
    source=np.array([[.3,0.,-.70],[1.,0.,0.],[2.,0.,-.75],[2.,0.,1.],
                     [.1,0.,0.],[9.,0.,0.],[np.nan,0.,0.]])
    before=source.copy();result=project_observation(source,10,fixed(),-.7635)
    np.testing.assert_allclose(result['points_body_xy_lidar_z'],[[.623,.22875,-.70],[1.323,.22875,0.]])
    np.testing.assert_array_equal(source,before)
    assert result['summary']['retained_rows']==2
    assert result['summary']['vertical_coordinate']=='PREPARED_LIDAR_Z_NOT_AXLE_Z'
    assert result['summary']['nonfinite_rows_excluded']==1
    assert result['summary']['self_return_mask_applied'] is False
    assert result['summary']['in_envelope_returns_removed'] is False
    assert result['summary']['object_velocity_known'] is False
    json.dumps(result['summary'],allow_nan=False)


def test_explicit_yaw_rotates_endpoints_and_sensor_axes():
    transform=fixed();transform[:2,:2]=[[0.,-1.],[1.,0.]]
    result=project_observation([[1,0,0],[0,1,0]],10,transform,-.7)
    np.testing.assert_allclose(result['points_body_xy_lidar_z'],[[.323,1.22875,0.],[-.677,.22875,0.]])
    assert result['sensor_yaw_body_rad']==pytest.approx(math.pi/2)
    assert result['sensor_origin_body_xy']==[.323,.22875]


def test_every_retained_endpoint_is_covered_by_a_voxel_disc_including_negative_cells():
    points=np.array([[.31,-.011,0.],[.39,-.099,0.],[-.31,.011,0.],[-.39,.099,0.]])
    result=project_observation(points,10,np.eye(4),-.7,voxel_m=.1)
    centers=result['voxel_centers_body_xy'];radius=result['voxel_radius_m']
    distances=np.linalg.norm(result['points_body_xy_lidar_z'][:,:2,None]-centers.T[None,:,:],axis=1)
    assert np.all(distances.min(axis=1)<=radius+1e-12)
    assert result['voxel_counts'].sum()==len(points)
    assert radius==pytest.approx(.1/math.sqrt(2))
    assert not result['points_body_xy_lidar_z'].flags.writeable
    assert not result['voxel_centers_body_xy'].flags.writeable


def test_height_and_range_boundary_observations_retained_inclusive():
    result=project_observation([[.3,0,-.65],[8.,0,.9],[8.001,0,0.]],10,np.eye(4),-.7)
    assert len(result['points_body_xy_lidar_z'])==2


def test_empty_projection_never_claims_clear_or_free_space_or_inferred_fov():
    result=project_observation(np.empty((0,3)),10,fixed(),-.7635)
    assert result['points_body_xy_lidar_z'].shape==(0,3)
    assert result['voxel_centers_body_xy'].shape==(0,2)
    assert result['horizontal_fov_rad'] is None
    assert result['summary']['empty_observation_is_clear'] is False
    assert result['summary']['missing_returns_are_free'] is False


def test_fov_is_declared_metadata_not_a_return_filter():
    result=project_observation([[-1.,0.,0.],[1.,0.,0.]],10,np.eye(4),-.7,
                               horizontal_fov_rad=(-math.pi/3,math.pi/3))
    assert len(result['points_body_xy_lidar_z'])==2
    assert result['horizontal_fov_rad']==[-math.pi/3,math.pi/3]
    assert result['summary']['observed_azimuth_is_free_space_or_fov_evidence'] is False


@pytest.mark.parametrize('change',[('z',.5),('roll',.1),('scale',2.),('reflection',-1.)])
def test_nonplanar_or_nonrigid_transform_never_silently_projected(change):
    matrix=fixed();kind,value=change
    if kind=='z':matrix[2,3]=value
    elif kind=='roll':matrix[1:3,1:3]=[[math.cos(value),-math.sin(value)],[math.sin(value),math.cos(value)]]
    elif kind=='scale':matrix[0,0]=value
    else:matrix[0,0]=value
    with pytest.raises(ValueError,match='planar'):
        project_observation([[1,0,0]],10,matrix,-.7)


@pytest.mark.parametrize('kwargs',[{'stamp_ns':True},{'stamp_ns':-1},{'ground_z_lidar':float('nan')},
 {'voxel_m':0},{'obstacle_height_m':(.5,.1)},{'horizontal_fov_rad':(1.,-1.)}])
def test_invalid_configuration_is_rejected(kwargs):
    request={'points_lidar':[[1,0,0]],'stamp_ns':10,'base_from_lidar':fixed(),'ground_z_lidar':-.7}
    request.update(kwargs)
    with pytest.raises(ValueError):project_observation(**request)


@pytest.mark.parametrize('points',[[[1,2]],np.zeros((100001,3)),[['bad',0,0]]])
def test_oversized_or_malformed_endpoint_arrays_rejected(points):
    with pytest.raises(ValueError):observation(10,points)


def test_delayed_delivery_never_reads_future_snapshot_and_age_is_capture_based():
    buffer=CausalObservationBuffer([packet(100,200),packet(300,400)])
    assert buffer.latest(150,100)['status']=='UNAVAILABLE'
    first=buffer.latest(200,100)
    assert first['stamp_ns']==100 and first['age_ns']==100 and first['usable']
    second=buffer.latest(350,100)
    assert second['stamp_ns']==100 and second['status']=='STALE' and not second['usable']
    assert second['observation'] is not None and second['dropout_is_clear'] is False
    third=buffer.latest(400,100)
    assert third['stamp_ns']==300 and third['delivery_latency_ns']==100


def test_late_old_packet_does_not_replace_newer_capture():
    buffer=CausalObservationBuffer([packet(100,500),packet(200,300)])
    assert buffer.latest(300,300)['stamp_ns']==200
    result=buffer.latest(500,1000)
    assert result['stamp_ns']==200 and result['received_time_ns']==300


def test_inputs_and_returned_payloads_cannot_mutate_buffer_or_future_observations():
    packets=[packet(100),packet(200)];buffer=CausalObservationBuffer(packets)
    packets[1]['observation']['summary']['retained_rows']=99
    result=buffer.latest(100,1000)
    result['observation']['summary']['retained_rows']=88
    result['observation']['points_body_xy_lidar_z'].setflags(write=True)
    result['observation']['points_body_xy_lidar_z'][0,0]=999
    later=buffer.latest(101,1000)
    assert later['observation']['summary']['retained_rows']==1
    assert later['observation']['points_body_xy_lidar_z'][0,0]==pytest.approx(1.323)
    assert buffer.latest(200,1000)['observation']['summary']['retained_rows']==1


def test_empty_delivered_scan_is_fresh_data_but_does_not_certify_clearance():
    buffer=CausalObservationBuffer([packet(100,100,np.empty((0,3)))])
    result=buffer.latest(100,100)
    assert result['status']=='FRESH' and result['usable']
    assert result['observation']['summary']['empty_observation_is_clear'] is False


def test_buffer_rejects_nonmonotonic_capture_future_at_receipt_and_bad_clock():
    with pytest.raises(ValueError,match='strictly increase'):CausalObservationBuffer([packet(200),packet(100)])
    with pytest.raises(ValueError,match='future'):CausalObservationBuffer([packet(200,100)])
    with pytest.raises(ValueError,match='match'):
        CausalObservationBuffer([{'stamp_ns':100,'received_time_ns':100,'observation':observation(200)}])
    buffer=CausalObservationBuffer([])
    assert buffer.latest(100,10)['observation'] is None
    with pytest.raises(ValueError,match='strictly increase'):buffer.latest(100,10)
    with pytest.raises(ValueError):CausalObservationBuffer([{'stamp_ns':True,'received_time_ns':100,'observation':{}}])


def test_buffer_resource_budget_rejects_before_copying_oversized_array():
    huge=np.zeros(10_000_001,dtype=np.uint8)
    with pytest.raises(ValueError,match='resource bound'):
        CausalObservationBuffer([{'stamp_ns':1,'received_time_ns':1,'observation':{'stamp_ns':1,'huge':huge}}])


def test_near_actual_height_returns_remain_available_for_review_not_free_blind_zone():
    result=project_observation([[.1,0.,0.],[.2,0.,-.7],[1.,0.,0.]],10,fixed(),-.7)
    assert len(result['points_body_xy_lidar_z'])==1
    np.testing.assert_allclose(result['near_range_points_body_xy_lidar_z'],[[.423,.22875,0.]])
    assert result['summary']['near_range_excluded_points']==1
    assert result['summary']['near_field_clearance_validated'] is False
    assert 'NOT_VERIFIED_SENSOR_BLIND_ZONE' in result['summary']['range_limits_basis']
    assert not result['near_range_points_body_xy_lidar_z'].flags.writeable
