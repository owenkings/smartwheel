"""Acceptance of source-time, gate, strict holdout and ICP safety boundaries."""
import copy
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from wc_runtime.mapping_planar import PlanarEKF
from wc_runtime.measurement import prepare_gyro
from wc_runtime.mapping_bias import calibrate_stationary,StationaryCalibrationPolicy,BiasPolicy,CausalBiasEstimator
from wc_runtime.source_time import SourceClock,event_key,select_stamps
from wc_runtime.icp_shadow import point_to_plane
from wc_runtime.mapping_compare import compare_cells

def test_native_bias_subtracts_before_installed_axis_rotation_and_rotates_covariance():
    rotation=Rotation.from_euler('xyz',[.3,-.2,.5]).as_matrix()
    raw=np.array([.04,-.03,.08]); bias=np.array([.001,.002,-.003]); cov=np.diag([.001,.002,.003])
    report=prepare_gyro(raw,bias,rotation,cov)
    assert np.allclose(report['target_rad_s'],rotation@(raw-bias))
    assert np.allclose(report['target_covariance'],rotation@cov@rotation.T)
    with pytest.raises(ValueError): prepare_gyro(raw,bias,None)

def test_gate_rejection_consumes_sequence_preserves_state_and_traces_covariance():
    model=PlanarEKF({}); model.wheel(0,.1,0.)
    before=model.x.copy(),model.P.copy()
    assert model.gyro(1,10.) is False
    assert np.array_equal(model.x,before[0]) and np.array_equal(model.P,before[1])
    row=model.report()['last_update']['gyro']
    assert row['nis']>row['threshold_nis']==25.
    assert row['sequence']==1 and row['innovation_covariance'][0][0]>0
    assert model.gyro(1,0.) is None and model.rejected_counts['gyro']==1
    assert model.gyro(2,0.) is True

def stationary_rows():
    return [dict(stamp_ns=1_000_000_000+i*10_000_000,sequence=i,
        gyro_native=[.0005,.0003,-.0005],acceleration_native=[0.,0.,9.80665],
        wheel_velocity_m_s=0.,wheel_age_s=.01) for i in range(3001)]

def evidence():
    return dict(physically_stationary=True,source='operator_visual_confirmation',evidence_id='physical-observation',
        evidence_sha256='a'*64,start_stamp_ns=1_000_000_000,end_stamp_ns=31_000_000_000)

def test_strict_calibration_uses_middle_only_and_unseen_last_segment_rejects_drift():
    rows=stationary_rows(); report=calibrate_stationary(rows,evidence())
    assert report['status']=='HOLDOUT_PASSED'
    assert [report['phases'][k]['sample_count'] for k in ('warmup','calibration','holdout')]==[1000]*3
    for row in rows[:1000]: row['gyro_native']=[.005,.005,.005] # Warmup never biases estimate.
    assert np.allclose(calibrate_stationary(rows,evidence())['bias_native_rad_s'],[.0005,.0003,-.0005])
    for row in rows[2000:]: row['gyro_native']=[.003,.0005,-.001]
    failed=calibrate_stationary(rows,evidence())
    assert failed['status']=='REJECTED' and 'HOLDOUT_MEAN_RESIDUAL' in failed['rejection_reasons']
    with pytest.raises(ValueError): calibrate_stationary(rows,{**evidence(),'physically_stationary':False})

def test_holdout_preserves_existing_stationary_limits_and_unknown_zero_is_not_calibration():
    old,new=BiasPolicy(),StationaryCalibrationPolicy()
    for key in ('max_bias_norm_rad_s','max_gyro_sample_rad_s','max_gyro_mean_rad_s',
                'max_wheel_age_s','max_wheel_speed_m_s','max_accel_norm_error_m_s2',
                'max_accel_axis_std_m_s2','max_accel_direction_p95_deg','gravity_m_s2'):
        assert getattr(new,key)==getattr(old,key)
    assert new.max_gap_s==old.max_imu_gap_s
    assert new.min_rate_hz==old.min_samples/old.window_s
    monitor=CausalBiasEstimator()
    assert monitor.status()['calibration_status']=='UNKNOWN_UNESTIMATED_ZERO_CANDIDATE'
    assert monitor.bias_metadata_at(1)['version']==0
    rows=stationary_rows()
    for row in rows: row['gyro_native']=[.003,0.,0.]
    rejected=calibrate_stationary(rows,evidence())
    assert rejected['status']=='REJECTED' and 'BIAS_NORM' in rejected['rejection_reasons']

def test_source_monotonic_epoch_ignores_wall_clock_jump_without_mutating_input():
    rows=[dict(stamp_ns=1_000_000_000,monotonic_ns=20_000_000,sequence=0,category='wheel'),
          dict(stamp_ns=900_000_000,monotonic_ns=30_000_000,sequence=0,category='imu'),
          dict(stamp_ns=2_000_000_000,monotonic_ns=30_000_000,sequence=1,category='imu')]
    original=copy.deepcopy(rows); clock=SourceClock(rows)
    mapped=[clock.canonicalize(row) for row in rows]
    assert rows==original
    assert [r['stamp_ns'] for r in sorted(mapped,key=event_key)]==[1_000_000_000,1_010_000_000,1_010_000_000]
    assert mapped[1]['original_source_stamp_ns']==900_000_000
    assert select_stamps([0,100_000_000,200_000_000],5.)==[0,200_000_000]

def test_shadow_rejects_unobservable_plane_and_never_changes_trajectory():
    x,y=np.meshgrid(np.linspace(-1,1,30),np.linspace(-1,1,30))
    plane=np.column_stack([x.ravel(),y.ravel(),np.zeros(x.size)])
    result=point_to_plane(plane,plane,np.eye(4))
    assert not result['accepted'] and 'GEOMETRIC_DEGENERACY' in result['rejection_reasons']
    assert result['trajectory_changed'] is False and result['final_quality']['rank']<6

def test_shadow_recovers_small_translation_with_three_independent_planes():
    rng=np.random.default_rng(8); uv=rng.uniform(-1,1,(500,2))
    target=np.vstack([np.column_stack([uv,np.zeros(500)]),
        np.column_stack([uv[:,0],np.zeros(500),uv[:,1]]),
        np.column_stack([np.zeros(500),uv])])
    source=target-np.array([.02,-.015,.01])
    result=point_to_plane(source,target,np.eye(4))
    assert result['accepted'],result['rejection_reasons']
    assert np.allclose(np.asarray(result['T_target_source_shadow'])[:3,3],[.02,-.015,.01],atol=2e-3)

def test_factor_comparisons_do_not_call_multi_factor_changes_isolated():
    cells={}
    for name,estimator,rate,filt in [('a','five_state',5,False),('b','robot_localization',5,False),('c','robot_localization',10,True)]:
        cells[name]={'summary':dict(estimator=estimator,input_rate_hz=rate,filter_enabled=filt),
            'frames':{1:{'T_prior_reference':np.eye(4).tolist()}},
            'pairs':{1:{'sources':[dict(side='left',raw_key='raw-a',source_stamp_ns=1)]}}}
    result=compare_cells(cells)
    assert len(result['single_factor_comparisons'])==1
    assert result['single_factor_comparisons'][0]['changed_factor']=='estimator'
