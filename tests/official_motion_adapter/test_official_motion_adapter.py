"""Real official-library regressions for the shared offline measurement adapter."""
import sys
from pathlib import Path
import numpy as np
import pytest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'runtime'))
from test_offline_motion_adapter import settings,candidate,feed,imu,wheel
from wc_runtime.mapping_prior import MotionPrior
from wc_runtime.offline_motion_adapter import OfflineCorrectedPrior,corrected_wheel_observation
from wc_runtime.robot_localization_provider import RobotLocalizationEKF,worker_path


@pytest.fixture(autouse=True)
def require_actual_worker():
    # Fail, rather than simulate an official library when target build is absent.
    assert worker_path().is_file()


def official_settings():
    c=settings();c['estimator']='robot_localization'
    return c


def adapted(c=None,**kwargs):
    c=official_settings() if c is None else c
    return OfflineCorrectedPrior(c,'synthetic',candidate=candidate(c,**kwargs),
        candidate_sha256='b'*64,clock=lambda:1000.)


def test_identity_candidate_preserves_official_baseline_and_pure_queries():
    c=official_settings();plain=MotionPrior(c,'synthetic',clock=lambda:1000.);corrected=adapted(c)
    t=feed(plain);feed(corrected,extra_queries=True)
    a=plain._planar_state_at(t);b=corrected._planar_state_at(t)
    ax,ap=a.state();bx,bp=b.state()
    assert np.array_equal(ax,bx) and np.array_equal(ap,bp)
    assert a.counts==b.counts and a.rejected_counts==b.rejected_counts


def test_bias_is_subtracted_once_with_official_library():
    bias=np.array([.0003,-.0002,.0008]);plain=adapted();corrected=adapted(bias=bias)
    t=feed(plain);feed(corrected,bias=bias,extra_queries=True)
    a=plain._planar_state_at(t);b=corrected._planar_state_at(t)
    np.testing.assert_allclose(a.x,b.x,rtol=0,atol=1e-13)
    np.testing.assert_allclose(a.P,b.P,rtol=0,atol=1e-13)
    last=b.report()['last_update']['gyro']['preparation']
    np.testing.assert_allclose(last['target_rad_s'],[0.,0.,.025],atol=1e-16)
    assert corrected.sample_report(t,corrected.pose_at(t))['gyro_bias_applied_native_rad_s']==bias.tolist()


def test_native_bias_precedes_nonidentity_installation_rotation():
    c=official_settings();R=np.array([[0.,0.,1.],[0.,1.,0.],[-1.,0.,0.]])
    c['R_base_imu']=R.tolist();bias=np.array([.0003,-.0002,.0008])
    rotated=adapted(c,bias=bias);reference=adapted();t=feed(reference)
    raw=R.T@np.array([0.,0.,.025])+bias
    for i in range(101):
        stamp=1_000_000_000+i*10_000_000
        rotated.add_wheel(wheel(i,stamp,-100,110));imu(rotated,i,stamp,gyro=tuple(raw))
    np.testing.assert_allclose(reference._planar_state_at(t).x,rotated._planar_state_at(t).x,atol=1e-13,rtol=0)
    np.testing.assert_allclose(reference._planar_state_at(t).P,rotated._planar_state_at(t).P,atol=1e-13,rtol=0)


def test_shared_measurements_include_identical_full_covariance_for_both_ekfs():
    config=settings();five=OfflineCorrectedPrior(config,'synthetic',candidate=candidate(config,a=.9012,b=-.013),
        candidate_sha256='b'*64,clock=lambda:1000.)
    official=adapted(a=.9012,b=-.013);t=feed(five);feed(official)
    for kind in ('wheel','gyro'):
        a=five._planar_state_at(t).last_update[kind]
        b=official._planar_state_at(t).last_update[kind]
        assert a['measurement']==b['measurement']
        assert a['stamp_ns']==b['stamp_ns'] and a['sequence']==b['sequence']
        if kind=='wheel':
            assert a['measurement_covariance']==b['measurement_covariance']
            assert b['measurement_covariance'][0][1] != 0
            assert b['covariance_protocol']=='wheel_cov_v1'


def test_official_startup_retains_cross_covariance_and_diagonal_path_is_identical():
    a=RobotLocalizationEKF({});b=RobotLocalizationEKF({})
    R=np.diag([a.config['wheel_v_variance'],a.config['wheel_w_variance']])
    a.wheel(0,.3,.1);b.wheel_covariance(0,.3,.1,R)
    assert all(np.array_equal(x,y) for x,y in zip(a.state(),b.state()))
    c=RobotLocalizationEKF({})
    z,R=corrected_wheel_observation(.3,.1,c.config,.9,-.013)
    c.wheel_covariance(0,*z,R)
    np.testing.assert_allclose(c.state()[1][np.ix_([6,11],[6,11])],R,rtol=0,atol=0)
    before=c.state();assert c.wheel_covariance(0,*z,R) is None
    assert all(np.array_equal(x,y) for x,y in zip(before,c.state()))
    assert c.wheel_covariance(1,50.,20.,R) is False
    assert all(np.array_equal(x,y) for x,y in zip(before,c.state()))


@pytest.mark.parametrize('R',[[[1.,2.],[2.,1.]],[[1.,.2],[.3,1.]],[[0.,0.],[0.,1.]],
    [[float('nan'),0.],[0.,1.]],[[1.,0.,0.],[0.,1.,0.]]])
def test_invalid_covariance_refused_before_native_mutation(R):
    c=RobotLocalizationEKF({});before=c.state()
    with pytest.raises(ValueError,match='positive definite'):
        c.wheel_covariance(0,.3,.1,R)
    assert all(np.array_equal(x,y) for x,y in zip(before,c.state()))


def test_official_refuses_five_state_continuous_process_noise():
    c=official_settings()
    with pytest.raises(ValueError,match='continuous process noise'):
        OfflineCorrectedPrior(c,'synthetic',candidate=candidate(c),candidate_sha256='b'*64,
            clock=lambda:1000.,process_noise={})
