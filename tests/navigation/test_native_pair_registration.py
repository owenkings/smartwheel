import math
import numpy as np
import pytest

from wc_navigation.native_pair_registration import analyze_native_pair


def corner_scene():
    rng=np.random.default_rng(821)
    a=rng.uniform(-1.,1.,(350,2));b=rng.uniform(-1.,1.,(350,2));c=rng.uniform(-1.,1.,(350,2))
    return np.vstack([np.column_stack([np.full(len(a),2.),a]),
                      np.column_stack([b[:,0],np.full(len(b),1.7),b[:,1]]),
                      np.column_stack([c,np.full(len(c),-1.2)])])


def test_rigid_transform_direction_and_reverse_cycle_are_correct():
    target=corner_scene()
    angle=.025;r=np.array([[math.cos(angle),-math.sin(angle),0],[math.sin(angle),math.cos(angle),0],[0,0,1.]])
    t=np.array([.07,-.035,.02])
    # Target = R * source + t. This is cloud alignment, not a body pose.
    source=(target-t)@r
    report=analyze_native_pair(source,target)
    transform=np.asarray(report['T_target_source_candidate'])
    assert transform[:3,:3]==pytest.approx(r,abs=.01)
    assert transform[:3,3]==pytest.approx(t,abs=.015)
    assert report['cycle_computed'] and report['cycle_translation_m']<.02
    assert report['cycle_rotation_deg']<1.
    assert report['trust_status']=='NOT_TRUSTED'
    assert report['observed_cloud_alignment_is_vehicle_motion'] is False


def test_a_plane_has_unobservable_local_degrees_despite_zero_fit_error():
    yz=np.array(np.meshgrid(np.linspace(-1,1,24),np.linspace(-1,1,24))).reshape(2,-1).T
    points=np.column_stack([np.full(len(yz),2.),yz])
    report=analyze_native_pair(points,points)
    assert report['forward_quality']['nearest_neighbor_rmse_m']<1e-9
    assert report['local_information']['rank']<6
    assert 'LOCAL_GEOMETRIC_WEAKNESS' in report['diagnostic_flags']
    assert report['local_information']['rotation_length_scale_m']==1.
    assert report['trust_status']=='NOT_TRUSTED'


def test_repeated_planar_grid_exposes_distinct_comparable_cloud_alignment():
    y,z=np.meshgrid(np.arange(-1.5,1.51,.1),np.arange(-1.5,1.51,.1))
    target=np.column_stack([y.ravel(),z.ravel(),np.full(y.size,2.)])
    source=target[(np.abs(target[:,0])<.9)&(np.abs(target[:,1])<.9)]
    report=analyze_native_pair(source,target,{'voxel_m':.05})
    assert 'COMPARABLE_FIT_DISTINCT_TRANSFORM_HYPOTHESES' in report['diagnostic_flags']
    assert report['authority_trajectory_written'] is False


def test_insufficient_overlap_does_not_report_identity_cycle_as_evidence():
    a=corner_scene();b=a+np.array([4.,0,0])
    report=analyze_native_pair(a,b)
    assert report['cycle_computed'] is False
    assert report['cycle_translation_m'] is None
    assert report['T_target_source_candidate'] is None
    assert report['diagnostic_flags']
    assert report['trust_status']=='NOT_TRUSTED'


def test_empty_native_data_remains_unknown_without_fabricated_motion():
    report=analyze_native_pair(np.empty((0,3)),np.empty((0,3)))
    assert report['forward_quality'] is None
    assert report['T_target_source_candidate'] is None
    assert report['T_source_target_reverse_candidate'] is None
    assert all(h['T_target_source_candidate'] is None for h in report['translation_hypotheses'])
    assert report['cycle_translation_m'] is None
    assert report['local_information']['rank'] is None
    assert report['status']=='NATIVE_CLOUD_CONSISTENCY_REVIEW_REQUIRED'


def test_preprocessing_preserves_input_and_never_applies_body_or_floor_mask():
    a=np.vstack([corner_scene(),[[float('nan'),0,0],[0,0,.1],[0,0,9.]]]);before=a.copy()
    report=analyze_native_pair(a,a)
    assert np.array_equal(a,before,equal_nan=True)
    assert report['source_counts']['nonfinite_rows_excluded']==1
    assert report['source_counts']['range_rows_excluded']==2
    assert report['body_mount_used'] is False and report['floor_model_used'] is False
    assert report['imu_or_odometry_used'] is False
    assert report['physical_navigation_validated'] is False


@pytest.mark.parametrize('policy',[{'max_points':True},{'range_min_m':2.,'range_max_m':1.},{'normal_neighbors':2},{'correspondence_limit_m':float('nan')}])
def test_invalid_diagnostic_policy_is_rejected(policy):
    with pytest.raises(ValueError):analyze_native_pair(np.empty((0,3)),np.empty((0,3)),policy)
