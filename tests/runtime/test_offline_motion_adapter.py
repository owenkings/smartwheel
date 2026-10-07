import copy
import sys
from pathlib import Path
import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'operations'))
from test_mapping_prior import config, imu, wheel, authority_weights
from wc_runtime.mapping_prior import MotionPrior, PriorError
from wc_runtime.offline_motion_adapter import OfflineCorrectedPrior, corrected_wheel_observation
from wc_runtime import offline_motion_calibration as calibration


def settings():
    c=config()
    c.update(continuous_mapping=True, motion_model='planar_ekf', estimator='five_state',
             offline_experiment=True, wheel_device_id='ZLAC8030D-0000000014',
             odometry_source='wheel_imu', wheel_imu_covariance=authority_weights(), policy={'history_s':5.})
    return c


def candidate(c, *, bias=(0.,0.,0.), a=1., b=0.):
    # Synthetic identity/covariance fixture, never saved as a hardware candidate.
    d={'schema':calibration.SCHEMA, 'status':calibration.VALIDATED,
       'validation':{'passed':True,'reasons':[]}, 'policy':copy.deepcopy(calibration.DEFAULT_POLICY),
       'provenance':{'session_id':'synthetic','imu_device_id':c['imu_sensor_id'],
                     'wheel_device_id':c['wheel_device_id'], 'sources':[{'path':'synthetic','sha256':'a'*64}],
                     'R_reference_imu':c['R_base_imu']},
       'windows':{'warmup':[0,10],'bias_train':[10,20],'bias_validation':[100,120],
                  'wheel_train':[25,60],'wheel_validation':[60,95]},
       'bias_native_rad_s':list(bias),'wheel_yaw_scale':a,'wheel_yaw_speed_coefficient':b}
    d['candidate_id']=calibration._candidate_digest(d)
    return d


def corrected(c=None, **kwargs):
    c=settings() if c is None else c
    return OfflineCorrectedPrior(c, 'synthetic', candidate=candidate(c,**kwargs),
                                  candidate_sha256='b'*64, clock=lambda:1000.)


def feed(prior, *, extra_queries=False, bias=(0.,0.,0.)):
    for i in range(101):
        t=1_000_000_000+i*10_000_000
        prior.add_wheel(wheel(i,t,-100,110))
        imu(prior,i,t,gyro=tuple(np.array([0.,0.,.025])+bias))
        if extra_queries and i>2:
            prior.pose_at(t-5_000_000)
            prior.pose_at(t)
    assert prior.failure is None
    return t


def test_identity_correction_preserves_complete_filter_state_and_covariance():
    c=settings(); ordinary=MotionPrior(c,'synthetic',clock=lambda:1000.); adapted=corrected(c)
    t=feed(ordinary);feed(adapted,extra_queries=True)
    x,y=ordinary._planar_state_at(t),adapted._planar_state_at(t)
    np.testing.assert_allclose(x.x,y.x,atol=1e-14,rtol=0)
    np.testing.assert_allclose(x.P,y.P,atol=1e-14,rtol=0)
    assert x.counts==y.counts and x.rejected_counts==y.rejected_counts and x.last_sequence==y.last_sequence


def test_nonzero_native_bias_removed_exactly_once_and_queries_are_pure():
    plain=corrected(); bias=np.array([.0003,-.0002,.0008]); adjusted=corrected(bias=bias)
    t=feed(plain);feed(adjusted,bias=bias,extra_queries=True)
    x,y=plain._planar_state_at(t),adjusted._planar_state_at(t)
    np.testing.assert_allclose(x.x,y.x,atol=1e-13,rtol=0)
    np.testing.assert_allclose(x.P,y.P,atol=1e-13,rtol=0)
    r=adjusted.sample_report(t,adjusted.pose_at(t))
    assert r['gyro_bias_applied_native_rad_s']==bias.tolist()
    assert adjusted.initialization['gyro_bias_status']=='OFFLINE_CANDIDATE_APPLIED'
    assert adjusted.bias_report()['formal_calibration'] is False


def test_wheel_transform_retains_cross_covariance_and_positive_definiteness():
    c={'wheel_v_variance':.0025,'wheel_w_variance':.0025}
    z,R=corrected_wheel_observation(.4,.2,c,.9,-.014)
    assert z[1]==pytest.approx(.9*.2-.014*.4)
    assert R[0,1]==R[1,0]==pytest.approx(-.014*.0025)
    assert R[1,1]==pytest.approx(.9**2*.0025+.014**2*.0025)
    assert np.linalg.eigvalsh(R).min()>0


def test_same_stamp_multiple_imu_sequences_are_used_once_each():
    p=corrected();t=feed(p)
    state=p._planar_state_at(t); before=state.counts.copy()
    rows=[{'stamp':t,'gyro':np.array([0.,0.,.025]),'sequence':101+i,'epoch':'synthetic'} for i in range(2)]
    group={'wheel':{},'imu':{t:rows}}
    p._planar_observe(state,t,grouped=group)
    counts=state.counts.copy();P=state.P.copy();x=state.x.copy()
    assert counts['gyro']==before['gyro']+2
    p._planar_observe(state,t,grouped=group)
    assert counts==state.counts
    assert np.array_equal(P,state.P) and np.array_equal(x,state.x)


@pytest.mark.parametrize('change',[{'offline_experiment':False},{'estimator':'robot_localization'},
    {'motion_model':'se3_gyro'},{'confirmed_gyro_bias':{}},{'offline_motion_correction':{}}])
def test_incompatible_or_double_application_refused(change):
    c=settings();c.update(change)
    with pytest.raises(ValueError): corrected(c)


def test_candidate_axis_change_cannot_be_silently_applied():
    c=settings();d=candidate(c)
    c['R_base_imu']=[[0.,-1.,0.],[1.,0.,0.],[0.,0.,1.]]
    p=OfflineCorrectedPrior(c,'synthetic',candidate=d,candidate_sha256='b'*64,clock=lambda:1000.)
    with pytest.raises((ValueError,PriorError,AssertionError)):
        feed(p)


@pytest.mark.parametrize('bad',[float('nan'),float('inf'),float('-inf')])
def test_nonfinite_measurement_refused(bad):
    with pytest.raises(ValueError):
        corrected_wheel_observation(.4,bad,{'wheel_v_variance':.1,'wheel_w_variance':.1},1.,0.)
