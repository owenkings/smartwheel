import math
import numpy as np
import pytest

from wc_navigation.geometry import Grid, inflate_grid
from wc_navigation.dynamic_simulation import simulate_dynamic_path, audit_dynamic_trajectory


def field():
    return inflate_grid(Grid(np.zeros((120, 260), dtype=np.int8), .05), .55, .10)


def disc(center, velocity=(0.,0.), start=0., end=100.):
    return {'id':'person','center_xy':center,'velocity_xy':velocity,'radius_m':.25,
            'active_from_s':start,'active_until_s':end}


def run(obstacles=(), **kwargs):
    return simulate_dynamic_path(field(), [(2.,2.),(8.,2.)], obstacles, **kwargs)


def assert_acceleration_bounded(result):
    poses=np.asarray(result['poses']);dt=np.diff(poses[:,0])
    assert np.all(np.abs(np.diff(poses[:,4])) <= result['settings']['linear_acceleration']*dt+1e-10)
    assert np.all(np.abs(np.diff(poses[:,5])) <= result['settings']['angular_acceleration']*dt+1e-10)


def test_clear_path_reaches_without_hardware_and_preserves_limits():
    r=run(max_duration_s=35.)
    assert r['status']=='REACHED'
    assert r['audit']['violation_count']==0
    assert r['final_speed_mps']==0
    assert r['hardware_started'] is False and r['navigation_validated'] is False
    assert r['collision_step_discarded'] is False
    assert_acceleration_bounded(r)


def test_persistent_obstacle_brakes_and_audits_wait_until_duration_limit():
    r=run([disc((5.,2.))],max_duration_s=25.,max_wait_s=3.)
    assert r['status']=='WAITING'
    assert r['max_wait_latched']
    assert r['audit']['violation_count']==0
    assert r['elapsed_s']==pytest.approx(25.)
    assert r['final_speed_mps']==0 and r['poses'][-1][1]<4.1
    assert any(x['state']=='BRAKING' for x in r['events'])
    assert_acceleration_bounded(r)


def test_persistent_obstacle_waits_without_micro_creep_or_false_resume_for_60_seconds():
    r=run([disc((5.,2.))],max_duration_s=60.,max_wait_s=20.)
    assert r['status']=='WAITING' and r['audit']['violation_count']==0
    assert r['stop_count']==1 and r['resume_count']==0
    rows=np.asarray(r['poses'])
    stopped=np.flatnonzero((rows[:,0]>3.) & (np.abs(rows[:,4])<1e-10))
    assert len(stopped)
    tail=rows[stopped[0]:]
    np.testing.assert_allclose(tail[:,1:3],np.tile(tail[0,1:3],(len(tail),1)),atol=1e-12)
    assert np.all(tail[:,4:6]==0)


def test_temporary_block_only_resumes_after_fresh_stable_clear_observation():
    r=run([disc((5.,2.),end=15.)],max_duration_s=50.,max_wait_s=20.)
    assert r['status']=='REACHED' and r['audit']['violation_count']==0
    assert r['stop_count']==1 and r['resume_count']==1
    rows=np.asarray(r['poses']);stopped=np.flatnonzero((rows[:,0]>3.) & (rows[:,4]==0))[0]
    moving=np.flatnonzero((np.arange(len(rows))>stopped) & (rows[:,4]>1e-10))[0]
    # Removal at15 is delivered no earlier than15.15, then must stay clear .5s.
    assert rows[moving,0]>=15.65-1e-9
    tail=rows[stopped:moving]
    np.testing.assert_allclose(tail[:,1:3],np.tile(tail[0,1:3],(len(tail),1)),atol=1e-12)
    assert_acceleration_bounded(r)


def test_departure_probe_has_progress_requirement_even_with_low_acceleration():
    # With this acceleration a fixed one-second probe moves less than a mm.
    # It cannot silently certify a real departure; its explicit time budget is
    # exhausted while trying to establish the configured 0.3m progress.
    r=run(max_duration_s=3.,linear_acceleration=.001,resume_probe_time_limit_s=1.)
    assert r['status']=='WAITING' and r['audit']['violation_count']==0
    assert all(p[4]==0 for p in r['poses'])
    assert any(e['reason']=='DEPARTURE_PROBE_PROGRESS_NOT_CERTIFIED' for e in r['events'])


def test_crossing_obstacle_waits_then_resumes_and_reaches():
    r=run([disc((5.,-1.),(0.,.25))],max_duration_s=50.,max_wait_s=20.)
    assert r['status']=='REACHED'
    assert r['audit']['violation_count']==0
    assert any(x['state']=='BRAKING' and x['reason'] in ['DYNAMIC_BRAKING_SWEEP','POST_STOP_MOVING_OBSTACLE'] for x in r['events'])
    assert r['resume_count']>=1
    assert r['hold_release_count'] > r['resume_count']  # Startup release is separate.
    assert not r['max_wait_latched']
    assert_acceleration_bounded(r)


@pytest.mark.parametrize('fault', ['sensor_dropouts','localization_dropouts'])
def test_fault_at_speed_has_finite_braking_distance_and_recovers(fault):
    r=run(max_duration_s=40.,**{fault:[(5.,9.)]})
    assert r['status']=='REACHED'
    assert r['audit']['violation_count']==0
    rows=np.asarray(r['poses']);after=rows[(rows[:,0]>=5.)&(rows[:,0]<=7.)]
    assert np.max(after[:,4])>.2 and np.min(after[:,4])==0
    assert after[-1,1]-after[0,1]>.05
    assert_acceleration_bounded(r)


def test_unseen_obstacle_inside_braking_distance_is_failure_with_executed_step():
    r=run([disc((3.53,2.),start=3.)],max_duration_s=10.,perception_latency_s=0.)
    assert r['status']=='FAIL'
    assert r['audit']['violation_count']>0
    assert r['audit']['contact_count']>0
    assert r['poses'][-1][0]>=3.
    assert len(r['poses'])>2 and not r['collision_step_discarded']
    assert_acceleration_bounded(r)


def test_initial_overlap_is_reported_even_before_sensor_warmup():
    r=run([disc((2.,2.))],max_duration_s=3.)
    assert r['status']=='FAIL' and len(r['poses'])==1
    assert r['audit']['contact_count']>0


def test_object_can_hit_stationary_robot_during_observation_failure():
    r=run([disc((5.,2.),(-1.,0.))],max_duration_s=5.,sensor_dropouts=[(0.,10.)])
    assert r['status']=='FAIL' and r['audit']['contact_count']>0
    assert all(p[4]==0 for p in r['poses'])


def test_mid_step_brief_appearance_is_not_missed():
    r=run([disc((2.,2.),start=.025,end=.035)],max_duration_s=1.)
    assert r['status']=='FAIL' and r['audit']['contact_count']>0
    assert r['poses'][-1][0]==pytest.approx(.05)


def test_exact_final_endpoint_appearance_is_checked():
    obstacle=disc((2.,2.),start=.05,end=1.)
    audit=audit_dynamic_trajectory(field(),[[0.,2.,2.,0.,0.,0.],[.05,2.,2.,0.,0.,0.]],[obstacle])
    assert audit['violation_count']>0 and audit['contact_count']>0
    assert 'ENVELOPE' in audit['contact_meaning']


def test_future_events_do_not_change_control_prefix():
    empty=run(max_duration_s=4.)
    future=run([disc((5.,2.),start=8.)],max_duration_s=4.)
    np.testing.assert_allclose(empty['poses'],future['poses'],atol=0,rtol=0)
    assert empty['decisions']==future['decisions']


def test_audit_rejects_teleportation_instead_of_accepting_zero_speed():
    audit=audit_dynamic_trajectory(field(),[[0.,2.,2.,0.,0.,0.],[1.,3.,2.,0.,0.,0.]])
    assert audit['violation_count']==1
    assert audit['violations'][0]['kind']=='TRAJECTORY_INTEGRATION_MISMATCH'


def test_unknown_static_path_is_not_made_traversable():
    data=np.zeros((120,260),dtype=np.int8);data[:,100:105]=-1
    r=simulate_dynamic_path(inflate_grid(Grid(data,.05),.55,.1),[(2.,2.),(8.,2.)],max_duration_s=5.)
    assert r['status']=='INVALID_PATH' and len(r['poses'])==1


@pytest.mark.parametrize('kwargs',[{'dt':.5},{'linear_acceleration':0.},{'reaction_delay_s':-1.},
                                   {'post_stop_horizon_s':float('nan')},{'sensor_dropouts':[(2.,1.)]}])
def test_malformed_scenario_rejected_before_simulation(kwargs):
    with pytest.raises(ValueError):run(**kwargs)
