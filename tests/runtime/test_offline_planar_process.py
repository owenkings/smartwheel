import copy
import importlib.util
import math
import os
from pathlib import Path
import sys
import numpy as np
import pytest

from wc_runtime.mapping_planar import PlanarEKF


def load(name,path):
    spec=importlib.util.spec_from_file_location(name,path);m=importlib.util.module_from_spec(spec)
    sys.modules[name]=m;spec.loader.exec_module(m);return m


candidate_dir=os.environ.get('WC_PROCESS_CANDIDATE_DIR')
if candidate_dir:
    p=Path(candidate_dir)
    process=load('wc_runtime.offline_planar_process',p/'offline_planar_process.py')
    adapter=load('wc_runtime.offline_motion_adapter_psd_test',p/'offline_motion_adapter_psd_candidate.py')
else:
    from wc_runtime import offline_planar_process as process
    from wc_runtime import offline_motion_adapter as adapter
sys.path.insert(0,str(Path(__file__).resolve().parent))
from test_offline_motion_adapter import settings,candidate,feed

PSD={'model':'continuous_white_acceleration','status':'EXPERIMENTAL_NOT_CALIBRATED',
     'linear_acceleration_psd_m2_s3':.25,'angular_acceleration_psd_rad2_s3':.25}


def fresh():return process.ContinuousPlanarEKF({},process_noise=PSD)
def adapted(*,psd=None):
    c=settings()
    return adapter.OfflineCorrectedPrior(c,'synthetic',candidate=candidate(c),candidate_sha256='b'*64,clock=lambda:1000.,process_noise=psd)


def test_none_preserves_legacy_process_choice():
    assert process.validate_process_noise(None) is None
    p=adapted();t=feed(p)
    assert type(p.planar_anchor[1]) is PlanarEKF
    assert 'offline_process_noise' not in p.initialization
    assert 'process_noise' not in p._planar_state_at(t).report()


@pytest.mark.parametrize('value',[0.,-1.,float('nan'),float('inf'),float('-inf'),True,'0.25'])
def test_invalid_psd_rejected(value):
    for key in ('linear_acceleration_psd_m2_s3','angular_acceleration_psd_rad2_s3'):
        d=dict(PSD);d[key]=value
        with pytest.raises(ValueError):process.validate_process_noise(d)


@pytest.mark.parametrize('change',[{'model':'legacy'}, {'status':'CALIBRATED'}, {'unknown':1}])
def test_model_units_and_status_are_explicit(change):
    d=dict(PSD);d.update(change)
    with pytest.raises(ValueError):process.validate_process_noise(d)
    missing=dict(PSD);missing.pop('angular_acceleration_psd_rad2_s3')
    with pytest.raises(ValueError):process.validate_process_noise(missing)


def test_initial_anchor_copied_without_any_second_update():
    old=PlanarEKF({},(.3,-.2));old.wheel(2,.4,.2);old.gyro(8,.21)
    old_report=copy.deepcopy(old.report())
    new=process.ContinuousPlanarEKF.from_state(old,process_noise=PSD)
    np.testing.assert_array_equal(new.x,old.x);np.testing.assert_array_equal(new.P,old.P)
    for key in ('measurement_updates','last_sequence','measurement_rejections','last_update'):
        assert new.report()[key]==old_report[key]
    new.x[0]=100;assert old.x[0]!=100
    with pytest.raises(ValueError):process.ContinuousPlanarEKF.from_state(new,process_noise=PSD)


def test_same_initial_covariance_produces_identical_measurement_gate_and_update():
    old=PlanarEKF({});new=process.ContinuousPlanarEKF.from_state(old,process_noise=PSD)
    for k,z in enumerate((.1,.12,.15,3.)):
        assert old.gyro(k,z)==new.gyro(k,z)
        np.testing.assert_array_equal(new.x,old.x);np.testing.assert_array_equal(new.P,old.P)
        assert new.last_update==old.last_update


@pytest.mark.parametrize('parts',[1,10,50,100,200,400])
def test_white_yaw_covariance_matches_continuous_analytic_model(parts):
    p=fresh();p.P[:]=0.;p.x[3:]=[.5,.3]
    for _ in range(parts):p.predict(1/parts)
    assert p.P[4,4]==pytest.approx(.25,abs=1e-11)
    assert p.P[2,4]==pytest.approx(.25/2,abs=1e-11)
    assert p.P[2,2]==pytest.approx(.25/3,abs=1e-11)
    assert np.linalg.eigvalsh(p.P).min()>-1e-12


def test_full_state_prediction_is_approximately_partition_invariant():
    a=fresh();b=fresh();a.x[3:]=b.x[3:]=[.5,.3]
    a.predict(1.)
    for _ in range(100):b.predict(.01)
    np.testing.assert_allclose(a.x,b.x,atol=1e-12,rtol=0)
    np.testing.assert_allclose(a.P,b.P,atol=2e-7,rtol=0)


def test_gap_holds_pose_and_prevents_cross_gap_measurement_motion():
    p=fresh();p.wheel(0,.6,.2);p.predict(.2)
    old=p.x.copy();p.predict(.7,observed=False)
    np.testing.assert_array_equal(p.x,old)
    np.testing.assert_array_equal(p.P[:3,3:],np.zeros((3,2)))
    p.wheel(1,0.,0.)
    np.testing.assert_array_equal(p.x[:3],old[:3])
    assert p.gap_intervals==1


def test_gap_uncertainty_does_not_depend_on_missing_interval_partitions():
    a=fresh();b=fresh();a.P[:]=b.P[:]=0.
    a.predict(.7,observed=False)
    for _ in range(70):b.predict(.01,observed=False)
    np.testing.assert_allclose(a.P,b.P,atol=1e-14,rtol=0)
    assert a.P[2,2]==pytest.approx(.25*.7**3/3)
    assert b.process_noise_report()['current_continuous_gap_s']==pytest.approx(.7)


def test_gap_budget_survives_measurement_and_copy_but_resets_on_observed_interval():
    a=fresh();a.P[:]=0.;a.predict(.2,observed=False)
    a.predict(0.,observed=True)
    b=a.copy();b.gyro(0,0.);b.predict(.3,observed=False)
    assert b.P[2,2]==pytest.approx(.25*.5**3/3)
    assert a.process_noise_report()['current_continuous_gap_s']==pytest.approx(.2)
    b.predict(.01);pose_var=b.P[2,2]
    b.predict(.1,observed=False)
    assert b.P[2,2]-pose_var==pytest.approx(.25*.1**3/3)
    assert b.process_noise_report()['current_continuous_gap_s']==pytest.approx(.1)


def test_deepcopy_retains_subclass_and_has_independent_psd_dictionary():
    p=fresh();q=p.copy();assert isinstance(q,process.ContinuousPlanarEKF)
    q.process_noise['angular_acceleration_psd_rad2_s3']=.5
    assert p.process_noise['angular_acceleration_psd_rad2_s3']==.25
    assert q.process_noise_report()['legacy_variance_parameters_reinterpreted'] is False


def test_adapter_psd_is_explicit_and_pose_queries_are_pure():
    p=adapted(psd=PSD);q=adapted(psd=PSD);t=feed(p);feed(q,extra_queries=True)
    assert isinstance(p.planar_anchor[1],process.ContinuousPlanarEKF)
    x,y=p._planar_state_at(t),q._planar_state_at(t)
    np.testing.assert_allclose(x.x,y.x,atol=1e-13,rtol=0)
    np.testing.assert_allclose(x.P,y.P,atol=1e-13,rtol=0)
    assert p.initialization['offline_process_noise']['units']['angular_acceleration_psd_rad2_s3']=='rad^2/s^3'
    assert x.report()['process_noise']['measurement_noise_and_innovation_gate_changed'] is False
    assert x.config['innovation_gate_sigma']==5.


def test_legacy_none_adapter_matches_existing_no_psd_behavior():
    from test_offline_motion_adapter import corrected
    old=corrected();new=adapted();t=feed(old);feed(new)
    x,y=old._planar_state_at(t),new._planar_state_at(t)
    np.testing.assert_array_equal(x.x,y.x);np.testing.assert_array_equal(x.P,y.P)
    assert x.report()==y.report()


def test_psd_cannot_enable_live_or_other_estimator():
    for change in ({'offline_experiment':False},{'estimator':'robot_localization'},{'motion_model':'se3_gyro'}):
        c=settings();c.update(change)
        with pytest.raises(ValueError):adapter.OfflineCorrectedPrior(c,'synthetic',candidate=candidate(c),candidate_sha256='b'*64,clock=lambda:1000.,process_noise=PSD)


@pytest.mark.parametrize('dt',[-1.,float('nan'),float('inf')])
def test_invalid_prediction_interval_rejected(dt):
    with pytest.raises(ValueError):fresh().predict(dt)
