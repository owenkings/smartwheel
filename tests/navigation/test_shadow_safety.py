import copy
import math

import numpy as np
import pytest

from wc_navigation.shadow_safety import evaluate_shadow_stop


def snapshot(points=((4., 0.),), *, age_s=0., usable=True, fov=None, voxel_radius=.05):
    stamp=1_000_000_000
    observation={'stamp_ns':stamp,'voxel_centers_body_xy':np.asarray(points,dtype=float).reshape(-1,2),
                 'voxel_radius_m':voxel_radius,'sensor_origin_body_xy':[0.,0.],
                 'sensor_yaw_body_rad':0.,'horizontal_fov_rad':fov,'summary':{}}
    now=stamp+round(age_s*1e9)
    return {'observation':observation,'stamp_ns':stamp,'received_time_ns':now,'usable':usable},now


def evaluate(data,now,v=.25,w=0.,**kwargs):
    return evaluate_shadow_stop(data,v,w,now,localization_trusted=True,lateral_velocity_mps=0.,**kwargs)


def test_initial_envelope_overlap_is_not_removed_as_self_return():
    data,now=snapshot(((.4,0.),(4.,0.)))
    before=data['observation']['voxel_centers_body_xy'].copy()
    report=evaluate(data,now)
    assert report['status']=='STOP_REQUIRED'
    assert report['capture_envelope_overlap_voxels']==1
    assert 'CAPTURE_ENVELOPE_OVERLAP_SELF_RETURN_OR_OBSTACLE' in report['stop_reasons']
    np.testing.assert_array_equal(data['observation']['voxel_centers_body_xy'],before)
    assert report['control_topics_published'] is False


def test_finite_braking_sweeps_an_obstacle_beyond_initial_envelope():
    data,now=snapshot(((1.,0.),))
    report=evaluate(data,now,v=.5)
    assert report['capture_envelope_overlap_voxels']==0
    assert report['braking_sweep_trigger_voxels']==1
    assert report['status']=='STOP_REQUIRED'
    assert report['modeled_capture_to_stop_distance_m']==pytest.approx(.5*.15+.5*.5/(2*.25))
    poses=np.asarray(report['poses'])
    assert poses[-1,4]==pytest.approx(0.,abs=1e-10)
    assert np.max(np.abs(np.diff(poses[:,4])/np.diff(poses[:,0])))<=.25+1e-9
    assert poses[1,4]==.5  # reaction interval, not an instant stop


def test_age_extends_capture_frame_hold_without_claiming_ego_compensation():
    data,now=snapshot(age_s=.2)
    report=evaluate(data,now,v=.5)
    assert report['modeled_capture_to_stop_distance_m']==pytest.approx(.5*(.2+.15)+.5)
    assert 'OLD_CAPTURE_WITHOUT_VERIFIED_EGO_COMPENSATION' in report['review_reasons']
    assert report['coordinate_basis']=='CAPTURE_BODY_NO_VERIFIED_EGO_MOTION_COMPENSATION'
    assert report['closed_loop_replay_performed'] is False


def test_turning_arc_is_checked_instead_of_only_forward_centerline():
    data,now=snapshot(((.8,.7),),voxel_radius=.02)
    turning=evaluate(data,now,v=1.,w=1.,radius_m=.2,margin_m=0.,
                     linear_deceleration_m_s2=.5,angular_deceleration_rad_s2=.5)
    straight=evaluate(data,now,v=1.,w=0.,radius_m=.2,margin_m=0.,linear_deceleration_m_s2=.5)
    assert turning['braking_sweep_trigger_voxels']==1
    assert straight['braking_sweep_trigger_voxels']==0
    assert turning['poses'][-1][3]>1.


def test_stale_and_unusable_snapshot_requests_stop_even_without_obstacle():
    data,now=snapshot(age_s=.5,usable=False)
    report=evaluate(data,now)
    assert report['status']=='STOP_REQUIRED'
    assert 'OBSERVATIONS_STALE' in report['stop_reasons']
    assert 'OBSERVATIONS_NOT_USABLE' in report['stop_reasons']


def test_future_capture_or_delivery_never_used():
    data,now=snapshot()
    data['received_time_ns']=now+1
    report=evaluate(data,now)
    assert report['status']=='STOP_REQUIRED'
    assert report['poses']==[]
    assert 'FUTURE_OR_INCONSISTENT_OBSERVATION_TIME' in report['stop_reasons']


def test_localization_gate_requests_stop():
    data,now=snapshot()
    report=evaluate_shadow_stop(data,.25,0.,now,localization_trusted=False,lateral_velocity_mps=0.)
    assert report['status']=='STOP_REQUIRED'
    assert 'LOCALIZATION_UNTRUSTED' in report['stop_reasons']


def test_missing_observation_is_not_an_empty_clear_scan():
    report=evaluate(None,1_000_000_000)
    assert report['status']=='STOP_REQUIRED'
    assert 'OBSERVATIONS_UNAVAILABLE' in report['stop_reasons']


def test_unknown_velocity_has_no_fabricated_rollout():
    data,now=snapshot()
    report=evaluate(data,now,v=None)
    assert report['status']=='REVIEW_REQUIRED'
    assert report['poses']==[] and report['velocity']['forward_mps'] is None
    assert 'RECORDED_VELOCITY_UNKNOWN' in report['review_reasons']


def test_default_unknown_fov_is_review_even_if_no_endpoint_trigger():
    data,now=snapshot()
    report=evaluate(data,now)
    assert report['status']=='REVIEW_REQUIRED'
    assert 'SENSOR_FOV_UNKNOWN' in report['review_reasons']
    assert report['physical_navigation_validated'] is False


def test_configured_front_fov_does_not_cover_reverse_or_full_circle():
    data,now=snapshot(fov=(-math.pi/3,math.pi/3))
    report=evaluate(data,now,v=-.5)
    assert 'SWEPT_ENVELOPE_OUTSIDE_CONFIGURED_FOV' in report['review_reasons']
    assert report['coverage']['outside_fov_points']>0
    assert report['poses'][-1][1]<0
    assert report['status']=='REVIEW_REQUIRED'


def test_near_range_blind_region_is_explicit_even_with_full_fov():
    data,now=snapshot(fov=(-math.pi,math.pi))
    report=evaluate(data,now,coverage_validated=True)
    assert 'SWEPT_ENVELOPE_INTERSECTS_CONFIGURED_NEAR_BLIND_ZONE' in report['review_reasons']
    assert report['coverage']['blind_zone_points']>0
    assert report['status']=='REVIEW_REQUIRED'


def test_excluded_near_points_cannot_silently_look_clear():
    data,now=snapshot(fov=(-math.pi,math.pi))
    data['observation']['summary']['near_range_height_candidate_rows']=2
    report=evaluate(data,now,coverage_validated=True,sensor_range_m=(0.,8.))
    assert 'NEAR_RANGE_POINTS_EXCLUDED_REVIEW' in report['review_reasons']
    assert report['status']=='REVIEW_REQUIRED'


def test_only_no_trigger_label_for_explicit_synthetic_coverage_case():
    data,now=snapshot(fov=(-math.pi,math.pi))
    report=evaluate(data,now,coverage_validated=True,sensor_range_m=(0.,8.))
    assert report['status']=='NO_STOP_TRIGGER_UNVALIDATED'
    assert report['physical_navigation_validated'] is False
    assert report['obstacle_velocity_known'] is False
    assert report['missing_returns_are_free'] is False


def test_empty_endpoints_do_not_prove_clear_space():
    data,now=snapshot((),fov=(-math.pi,math.pi))
    report=evaluate(data,now,coverage_validated=True,sensor_range_m=(0.,8.))
    assert report['status']=='REVIEW_REQUIRED'
    assert 'NO_OBSTACLE_ENDPOINTS_DOES_NOT_PROVE_FREE_SPACE' in report['review_reasons']


def test_lateral_motion_outside_unicycle_model_requires_review():
    data,now=snapshot()
    report=evaluate_shadow_stop(data,.25,0.,now,localization_trusted=True,lateral_velocity_mps=.1)
    assert 'LATERAL_MOTION_OUTSIDE_UNICYCLE_MODEL' in report['review_reasons']


def test_braking_resource_limit_does_not_truncate_and_claim_success():
    data,now=snapshot()
    report=evaluate(data,now,v=1.,linear_deceleration_m_s2=.001)
    assert report['status']=='REVIEW_REQUIRED'
    assert report['poses']==[]
    assert 'BRAKING_HORIZON_EXCEEDS_DIAGNOSTIC_RESOURCE_BOUND' in report['review_reasons']


@pytest.mark.parametrize('kwargs',[{'radius_m':0.},{'prediction_dt_s':.3},
                                    {'linear_deceleration_m_s2':float('nan')},
                                    {'margin_m':-1.}])
def test_invalid_model_parameters_rejected(kwargs):
    data,now=snapshot()
    with pytest.raises(ValueError):evaluate(data,now,**kwargs)


def test_observation_timestamp_mismatch_rejected():
    data,now=snapshot();data['observation']['stamp_ns']+=1
    with pytest.raises(ValueError,match='differs'):evaluate(data,now)
