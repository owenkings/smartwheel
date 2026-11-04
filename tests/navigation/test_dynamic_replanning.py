import math
import numpy as np
import pytest

from wc_navigation.geometry import Grid, inflate_grid
from wc_navigation.dynamic_simulation import simulate_dynamic_path


def field():
    return inflate_grid(Grid(np.zeros((120,260),dtype=np.int8),.05),.55,.10)


def obstacle(name='block',center=(5.,2.),start=0.,end=100.):
    return {'id':name,'center_xy':center,'velocity_xy':(0.,0.),'radius_m':.25,
            'active_from_s':start,'active_until_s':end}


def path_for(snapshot):
    start=snapshot['pose_x_y_yaw'][:2]
    return [start,[start[0],4.],[6.5,4.],[6.5,2.],snapshot['goal_xy']]


def run(callback,**settings):
    defaults={'max_duration_s':70.,'max_wait_s':2.,'replan_wait_s':.5,
              'planning_delay_s':.5,'replan_cooldown_s':2.}
    defaults.update(settings)
    return simulate_dynamic_path(field(),[(2.,2.),(8.,2.)],[obstacle()],
                                 replan_callback=callback,**defaults)


def assert_continuous(result):
    rows=np.asarray(result['poses']);dt=np.diff(rows[:,0])
    assert np.all(dt>0)
    assert np.all(np.abs(np.diff(rows[:,4]))<=result['settings']['linear_acceleration']*dt+1e-9)
    assert np.all(np.abs(np.diff(rows[:,5]))<=result['settings']['angular_acceleration']*dt+1e-9)
    assert result['collision_step_discarded'] is False
    assert result['audit']['violation_count']==0


def test_detour_installs_at_current_stopped_pose_and_preserves_world_clock():
    calls=[]
    def callback(snapshot):
        calls.append(snapshot)
        assert set(snapshot)=={'elapsed_s','pose_x_y_yaw','goal_xy','observation_stamp_s','tracks'}
        assert snapshot['observation_stamp_s']<=snapshot['elapsed_s']-.15+1e-8
        assert all(set(t)=={'id','center_xy','velocity_xy','radius_m','stamp_s'} for t in snapshot['tracks'])
        return {'status':'PATH','path':path_for(snapshot),'metadata':{'algorithm':'declared-test-detour'}}
    result=run(callback)
    assert result['status']=='REACHED'
    assert len(calls)==1 and result['replans'][0]['status']=='INSTALLED'
    installation=result['path_segments'][1]
    rows=np.asarray(result['poses'])
    at=np.argmin(abs(rows[:,0]-installation['installed_time_s']))
    np.testing.assert_allclose(rows[at,1:3],installation['path'][0],atol=1e-10)
    np.testing.assert_allclose(rows[at,4:6],0.,atol=1e-10)
    assert installation['installed_time_s']>=calls[0]['elapsed_s']+.5-1e-8
    assert result['stop_count']==1 and result['resume_count']==1
    assert result['navigation_validated'] is False
    assert_continuous(result)


def test_same_geometry_no_path_is_only_requested_once_and_remains_still():
    calls=[]
    result=run(lambda s:calls.append(s) or {'status':'NO_PATH','reason':'no free route'},max_duration_s=25.)
    assert result['status']=='WAITING' and len(calls)==1
    assert result['replans'][0]['status']=='NO_PATH'
    rows=np.asarray(result['poses']);at=np.argmin(abs(rows[:,0]-calls[0]['elapsed_s']))
    tail=rows[at:]
    np.testing.assert_allclose(tail[:,1:3],np.tile(tail[0,1:3],(len(tail),1)),atol=1e-10)
    assert np.all(tail[:,4:6]==0.)
    assert_continuous(result)


@pytest.mark.parametrize('fault',['sensor_dropouts','localization_dropouts'])
def test_fault_does_not_trigger_replanning_even_with_current_static_obstacle(fault):
    calls=[]
    result=run(lambda s:calls.append(s) or {'status':'NO_PATH'},max_duration_s=18.,**{fault:[(5.,30.)]})
    assert result['status']=='WAITING' and not calls and not result['replans']
    assert_continuous(result)


@pytest.mark.parametrize('problem',['start','goal','static','nan','exception','resource'])
def test_invalid_or_failed_callback_is_rejected_without_pose_discontinuity(problem):
    def callback(snapshot):
        path=path_for(snapshot)
        if problem=='start':path[0]=[path[0][0]+.2,path[0][1]]
        if problem=='goal':path[-1]=[7.5,2.]
        if problem=='static':path.insert(1,[1.,.1])
        if problem=='nan':path[1][0]=float('nan')
        if problem=='resource':path[1]=[1e10,1e10]
        if problem=='exception':raise RuntimeError('test planner unavailable')
        return {'status':'PATH','path':path}
    result=run(callback,max_duration_s=18.,max_replans=1)
    assert result['status']=='WAITING' and len(result['path_segments'])==1
    assert len(result['replans'])==1 and result['replans'][0]['status']!='INSTALLED'
    assert_continuous(result)


def request_time():
    result=run(lambda s:{'status':'NO_PATH'},max_duration_s=15.,planning_delay_s=1.)
    return result['replans'][0]['requested_time_s']


@pytest.mark.parametrize('fault',['sensor_dropouts','localization_dropouts'])
def test_pending_result_rejected_when_delivery_is_untrusted(fault):
    when=request_time()
    result=run(lambda s:{'status':'PATH','path':path_for(s)},max_duration_s=when+2.,
               planning_delay_s=1.,max_replans=1,**{fault:[(when+.1,when+3.)]})
    assert result['replans'][0]['status']=='REJECTED_UNTRUSTED_OR_NOT_STOPPED'
    assert len(result['path_segments'])==1
    assert_continuous(result)


def test_pending_path_is_checked_against_new_current_obstacle():
    when=request_time()
    result=simulate_dynamic_path(field(),[(2.,2.),(8.,2.)],
        [obstacle(),obstacle('new',(5.5,4.),start=when+.1)],
        replan_callback=lambda s:{'status':'PATH','path':path_for(s)},max_duration_s=when+2.,
        max_wait_s=2.,replan_wait_s=.5,planning_delay_s=1.,max_replans=1)
    assert result['replans'][0]['status']=='PATH_LATEST_OBSERVATION_BLOCKED'
    assert len(result['path_segments'])==1
    assert_continuous(result)


def test_transient_fault_rejection_can_retry_fresh_result_without_resetting_state():
    when=request_time()
    result=run(lambda s:{'status':'PATH','path':path_for(s)},max_duration_s=70.,
               planning_delay_s=1.,max_replans=2,
               localization_dropouts=[(when+.1,when+2.5)])
    assert [r['status'] for r in result['replans']]==['REJECTED_UNTRUSTED_OR_NOT_STOPPED','INSTALLED']
    assert result['status']=='REACHED' and result['stop_count']==1 and result['resume_count']==1
    assert result['replans'][1]['requested_time_s']>=when+2.5
    assert_continuous(result)


def test_replanning_attempt_budget_is_bounded():
    result=run(lambda s:{'status':'PLANNER_ERROR','reason':'offline planner failed'},max_duration_s=25.)
    assert len(result['replans'])==3 and result['status']=='WAITING'
    assert_continuous(result)
    with pytest.raises(ValueError,match='max_replans'):
        run(lambda s:{'status':'NO_PATH'},max_replans=4)


def test_temporary_block_clear_before_persistence_threshold_does_not_replan():
    calls=[]
    result=simulate_dynamic_path(field(),[(2.,2.),(8.,2.)],[obstacle(end=10.)],
        replan_callback=lambda s:calls.append(s) or {'status':'NO_PATH'},max_duration_s=40.,
        max_wait_s=20.,replan_wait_s=3.)
    assert result['status']=='REACHED' and not calls
    assert_continuous(result)


def test_extra_replan_reserve_stops_earlier_without_changing_observed_or_audited_radius():
    no_path=lambda s:{'status':'NO_PATH'}
    baseline=run(no_path,max_duration_s=15.)
    reserved=run(no_path,max_duration_s=15.,replan_stop_reserve_m=.15)
    assert reserved['poses'][-1][1] < baseline['poses'][-1][1]-.12
    assert reserved['settings']['replan_stop_reserve_m']==.15
    assert reserved['world_obstacles'][0]['radius_m']==.25
    assert reserved['replans'][0]['snapshot']['tracks'][0]['radius_m']==.25
    assert reserved['audit']['contact_meaning']=='ASSUMED_CIRCLE_PLUS_MARGIN_ENVELOPE_NOT_MEASURED_PHYSICAL_CONTACT'
    assert any(d['replan_stop_reserve_applied_ids']==['block'] for d in reserved['decisions'])
    assert_continuous(reserved)


def test_replan_reserve_releases_for_installed_detour_and_allows_recovery():
    result=run(lambda s:{'status':'PATH','path':path_for(s)},replan_stop_reserve_m=.15)
    assert result['status']=='REACHED' and result['stop_count']==1 and result['resume_count']==1
    installed=result['path_segments'][1]['installed_time_s']
    after=[d for d in result['decisions'] if d['time_s']>installed+1e-8]
    assert after and all(not d['replan_stop_reserve_applied_ids'] for d in after)
    assert_continuous(result)


def test_reserve_does_not_apply_without_callback_or_to_unrelated_static_tracks():
    disabled=run(None,max_duration_s=12.)
    disabled_reserved=run(None,max_duration_s=12.,replan_stop_reserve_m=.15)
    assert disabled['poses']==disabled_reserved['poses']
    assert all(not d['replan_stop_reserve_applied_ids'] for d in disabled_reserved['decisions'])
    args=(field(),[(2.,2.),(8.,2.)],[obstacle(center=(5.,4.))])
    baseline=simulate_dynamic_path(*args,replan_callback=lambda s:{'status':'NO_PATH'},max_duration_s=12.)
    reserved=simulate_dynamic_path(*args,replan_callback=lambda s:{'status':'NO_PATH'},max_duration_s=12.,replan_stop_reserve_m=.15)
    assert baseline['poses']==reserved['poses']
    assert all(not d['replan_stop_reserve_applied_ids'] for d in reserved['decisions'])

