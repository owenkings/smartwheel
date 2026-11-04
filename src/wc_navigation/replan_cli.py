"""Observed-obstacle overlay, isolated Nav2 replanning and numerical execution."""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import time
import uuid
import subprocess
import sys

import numpy as np

from .cli import fingerprint, write_json
from .dynamic_cli import path_point
from .geometry import Grid, inflate_grid, _validated_path
from .replanning import overlay_observed_discs, find_path


SCENARIOS = ('side_detour', 'center_no_route', 'localization_untrusted', 'perception_dropout')


def _plan_nav2_isolated(map_yaml,start_xy,goal_xy,radius_m,margin_m,output_dir,timeout_s=75):
    """Destroy the DDS participant after every request rather than reusing contexts."""
    from .nav2_planner import Nav2PlanningError
    directory=Path(output_dir).parent
    request_file=directory/'worker_request.json';result_file=directory/'worker_result.json'
    write_json(request_file,{'map_yaml':str(map_yaml),'start_xy':start_xy,'goal_xy':goal_xy,
                            'radius_m':radius_m,'margin_m':margin_m,'output_dir':str(output_dir),
                            'timeout_s':timeout_s})
    with (directory/'worker_stdout.log').open('x') as stdout,(directory/'worker_stderr.log').open('x') as stderr:
        process=subprocess.Popen([sys.executable,'-B','-s','-m','wc_navigation.replan_worker',
                                  '--request',str(request_file),'--result',str(result_file)],
                                 stdout=stdout,stderr=stderr,start_new_session=True)
        expired=False
        try:
            process.wait(timeout=timeout_s+30)
        except subprocess.TimeoutExpired:
            expired=True
        finally:
            if process.poll() is None:
                process.terminate()
                try:process.wait(timeout=20)
                except subprocess.TimeoutExpired:process.kill();process.wait(timeout=5)
        if expired:raise Nav2PlanningError('isolated planning worker exceeded its wall deadline')
    if not result_file.is_file():
        raise Nav2PlanningError('isolated planning worker returned no result; see worker logs')
    response=json.loads(result_file.read_text(encoding='utf-8'))
    if response.get('status')!='PATH':raise Nav2PlanningError(response.get('error','invalid worker response'))
    if process.returncode!=0:raise Nav2PlanningError('planning worker returned PATH but exited unsuccessfully')
    result=response['result'];result['worker']={'pid':process.pid,'exit_code':process.returncode,
                                               'request_file':str(request_file),'result_file':str(result_file),
                                               'one_ros_participant_per_process':True}
    return result


def arguments(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--project-root', type=Path, required=True)
    parser.add_argument('--plan-result', type=Path, required=True)
    parser.add_argument('--scenario', choices=('all', *SCENARIOS), default='all')
    return parser.parse_args(argv)


class ObservedMapPlanner:
    """No world schedule, future disappearance or actual vehicle interface."""
    def __init__(self, grid, radius_m, margin_m, directory):
        self.grid = grid
        self.radius_m, self.margin_m = radius_m, margin_m
        self.directory = Path(directory)
        self.calls = []

    def __call__(self, snapshot):
        from .map_guard import write_guarded_map
        from .nav2_planner import Nav2PlanningError
        directory = self.directory / ('request_%02d' % (len(self.calls)+1))
        directory.mkdir(parents=True, exist_ok=False)
        now = snapshot['elapsed_s']
        observed = []
        for track in snapshot['tracks']:
            age = now - track['stamp_s']
            if not 0 <= age <= .4 + 1e-8:
                return {'status': 'PLANNER_ERROR', 'reason': 'STALE_OR_FUTURE_OBSERVATION'}
            center = np.asarray(track['center_xy']) + age*np.asarray(track['velocity_xy'])
            observed.append({'id': track['id'], 'center_xy': center.tolist(),
                             'radius_m': track['radius_m']})
        overlay, protection = overlay_observed_discs(self.grid, observed)
        inflated = inflate_grid(overlay, self.radius_m, self.margin_m)
        start, goal = snapshot['pose_x_y_yaw'][:2], snapshot['goal_xy']
        export = write_guarded_map(overlay, directory / 'observed_map', {
            'purpose': 'EPHEMERAL_CURRENT_OBSERVATION_OVERLAY_NOT_BASE_MAP_UPDATE',
            'snapshot': snapshot, 'projected_observed_discs': observed, 'protection': protection})
        diagnostic = find_path(inflated, start, goal)
        metadata = {'snapshot_time_s': now, 'overlay': protection, 'export': export,
                    'discrete_connectivity': {k:v for k,v in diagnostic.items() if k!='path'},
                    'assumed_radius_m': self.radius_m, 'margin_m': self.margin_m,
                    'base_map_mutated': False, 'primary_planner': 'Nav2 Navfn A*',
                    'virtual_delivery_delay_is_not_wall_latency': True}
        before = time.monotonic()
        try:
            nav2 = _plan_nav2_isolated(export['map_yaml'], start, goal, self.radius_m, self.margin_m,
                                      directory/'nav2', timeout_s=75)
            # Navfn returns cell-centre endpoints. Preserve the exact stopped
            # pose and original goal using independently certified connectors.
            path = _validated_path([start, *nav2['path'], goal]).tolist()
            metadata['nav2'] = nav2
            rejected = [i for i,(a,b) in enumerate(zip(path,path[1:])) if not inflated.segment_is_free(a,b)]
            if rejected:
                metadata['nav2_outcome']='RETURNED_PATH_REJECTED_BY_CONTINUOUS_SWEEP_CHECK'
                metadata['rejected_nav2_segments']=rejected
                write_json(directory/'rejected_nav2_path.json',{'path':path,'rejected_segments':rejected})
                fallback = diagnostic.get('path',[])
                if (diagnostic['status']=='PATH' and fallback and
                        math.dist(fallback[0],start)<1e-6 and math.dist(fallback[-1],goal)<1e-6 and
                        all(inflated.segment_is_free(a,b) for a,b in zip(fallback,fallback[1:]))):
                    metadata['selected_planner']='CONSERVATIVE_GRID_ASTAR_AFTER_NAV2_SWEEP_REJECTION'
                    metadata['path_length_m']=sum(math.dist(a,b) for a,b in zip(fallback,fallback[1:]))
                    response={'status':'PATH','path':fallback}
                else:
                    response = {'status':'PLANNER_ERROR', 'reason':'NAV2_PATH_CONTINUOUS_SWEEP_REJECTED'}
            else:
                metadata['nav2_outcome']='CONTINUOUS_SWEEPS_CERTIFIED'
                metadata['selected_planner']='NAV2_NAVFN_ASTAR'
                metadata['path_length_m'] = sum(math.dist(a,b) for a,b in zip(path,path[1:]))
                response = {'status':'PATH', 'path':path}
        except Nav2PlanningError as exc:
            reason = str(exc)
            metadata['nav2_error'] = reason
            if 'Nav2 did not find a path (action status ' in reason and diagnostic['status']=='NO_PATH':
                response = {'status':'NO_PATH', 'reason':'CONSERVATIVE_GRID_NO_PATH_AND_NAV2_ACTION_UNSUCCESSFUL'}
            else:
                response = {'status':'PLANNER_ERROR', 'reason':'NAV2_UNAVAILABLE_OR_INVALID_OUTPUT'}
        metadata['planner_wall_elapsed_s'] = time.monotonic()-before
        response['metadata'] = metadata
        self.calls.append({'snapshot':snapshot,'response':response})
        write_json(directory/'request_result.json', self.calls[-1])
        return response


def scenario_inputs(path, name):
    point, tangent = path_point(path, 2. if name!='center_no_route' else 2.5)
    if name!='center_no_route':
        point = point+.7*np.array([-tangent[1],tangent[0]])
    obstacle = {'id':'observed_block','center_xy':point.tolist(),'velocity_xy':[0.,0.],
                'radius_m':.3 if name=='center_no_route' else .15,
                'active_from_s':0.,'active_until_s':180.}
    settings = {'max_duration_s':70.,'max_wait_s':25.,'replan_wait_s':3.,'planning_delay_s':.5,
                'replan_cooldown_s':5.,'max_replans':3,'replan_stop_reserve_m':.15}
    if name=='localization_untrusted': settings['localization_dropouts']=[(4.,70.)]
    if name=='perception_dropout': settings['sensor_dropouts']=[(4.,70.)]
    return [obstacle],settings


def assess(name, result):
    clear = result['audit']['status']=='CLEAR_UNDER_STATED_MODEL'
    installed = [p for p in result['replans'] if p['status']=='INSTALLED']
    if name=='side_detour':
        satisfied = result['status']=='REACHED' and len(installed)==1 and len(result['path_segments'])==2 and result['stop_count']==1 and result['resume_count']==1 and clear
        expected = 'Stop once, install a certified detour and reach the original goal'
    elif name=='center_no_route':
        satisfied = (result['status']=='WAITING' and len(result['replans'])==1
                     and result['replans'][0]['status']=='NO_PATH' and not installed
                     and result['stop_count']==1 and result['resume_count']==0 and clear)
        expected = 'One no-path request for unchanged geometry, then stable waiting'
    else:
        satisfied = result['status']=='WAITING' and not result['replans'] and clear
        expected = 'No replanning/restart while localization or observations are untrusted'
    return {'satisfied':bool(satisfied),'expected':expected,'physical_acceptance':False}


def render(path, grid, original, results, worlds, radius_m, margin_m):
    import matplotlib
    matplotlib.use('Agg')
    from matplotlib import pyplot as plt
    from matplotlib.patches import Circle
    fig, axes = plt.subplots(2,2,figsize=(14,8),constrained_layout=True)
    image=np.full(grid.data.shape,.55);image[grid.data==0]=1;image[grid.data==100]=.08
    if abs(grid.origin[2])>1e-10:raise ValueError('preview requires an axis-aligned map')
    ox,oy=grid.origin[:2];extent=[ox,ox+grid.width*grid.resolution,oy,oy+grid.height*grid.resolution]
    old=np.asarray(original)
    for ax,(name,result) in zip(axes.flat,results.items()):
        ax.imshow(image,origin='lower',cmap='gray',vmin=0,vmax=1,extent=extent)
        ax.plot(old[:,0],old[:,1],'--',color='#7b7b7b',label='Original Nav2 path')
        for segment in result['path_segments'][1:]:
            new=np.asarray(segment['path']);ax.plot(new[:,0],new[:,1],color='#2277ba',linewidth=2,label='Replanned path')
        poses=np.asarray(result['poses']);ax.plot(poses[:,1],poses[:,2],color='#21955b',linewidth=1.2,label='Numerical execution')
        for obstacle in worlds[name]:
            ax.add_patch(Circle(obstacle['center_xy'],obstacle['radius_m'],color='#c44c40',alpha=.8))
            ax.add_patch(Circle(obstacle['center_xy'],obstacle['radius_m']+radius_m+margin_m,fill=False,color='#c44c40',linestyle=':'))
        ax.set_xlim(old[:,0].min()-.7,old[:,0].max()+.7);ax.set_ylim(old[:,1].min()-1.3,old[:,1].max()+1.3)
        ax.set_aspect('equal');ax.set_title(name+': '+result['status']);ax.set_xlabel('Map x (m)');ax.set_ylabel('Map y (m)')
        ax.legend(fontsize=7)
    fig.suptitle('Observed permanent-obstacle replanning on the protected recorded map\nAssumed radius %.2f m + margin %.2f m; virtual localization and synthetic observations' % (radius_m,margin_m))
    fig.savefig(path,dpi=150);plt.close(fig)


def main(argv=None):
    args=arguments(argv)
    from wc_runtime.map_viewer import inspect_grid
    from wc_runtime.storage_policy import StoragePolicy
    from .dynamic_simulation import simulate_dynamic_path
    policy=StoragePolicy(args.project_root);policy.check()
    raw=args.plan_result.read_bytes();saved=json.loads(raw)
    if saved.get('status')!='PASS' or saved.get('mode')!='OFFLINE_NAV2_AND_NUMERICAL_SIMULATION':
        raise ValueError('successful frozen offline Nav2 result required')
    original=_validated_path(saved['planner']['path']).tolist()
    map_yaml=Path(saved['source_map']);stats,data,meta=inspect_grid(map_yaml)
    hashes=saved['source_sha256']
    if set(hashes)!={str(map_yaml),stats['image']} or any(fingerprint(p)!=h for p,h in hashes.items()):
        raise ValueError('source map differs from the frozen successful path')
    footprint=saved['footprint'];grid=Grid(data,meta['resolution'],tuple(meta['origin']))
    if footprint['model']!='assumed_circle' or footprint['measured'] is not False:
        raise ValueError('explicit simulation footprint assumption required')
    inflated=inflate_grid(grid,footprint['radius_m'],footprint['margin_m'])
    run=datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')+'_'+uuid.uuid4().hex[:8]
    output=policy.resolve('reports/navigation_replanning/'+run);output.mkdir(parents=True,exist_ok=False)
    report={'schema_version':1,'mode':'OBSERVED_OVERLAY_NAV2_REPLAN_NUMERICAL_EXECUTION',
            'output':str(output),'plan_result':str(args.plan_result.absolute()),'plan_sha256':hashlib.sha256(raw).hexdigest(),
            'source_sha256':hashes,'footprint':footprint,'scenarios':{},'hardware_started':False,
            'navigation_validated':False,'physical_test_performed':False,'localization_source':'VIRTUAL_TRUST_SIGNAL_NOT_REAL_AMCL',
            'obstacle_source':'IDEAL_DELAYED_CURRENT_SYNTHETIC_DISC_OBSERVATIONS'}
    names=SCENARIOS if args.scenario=='all' else (args.scenario,);results={};worlds={}
    for name in names:
        obstacles,settings=scenario_inputs(original,name);worlds[name]=obstacles
        planner=ObservedMapPlanner(grid,footprint['radius_m'],footprint['margin_m'],output/name/'planning')
        write_json(output/(name+'_world.json'),{'obstacles':obstacles,'settings':settings})
        result=simulate_dynamic_path(inflated,original,obstacles,replan_callback=planner,**settings)
        results[name]=result;write_json(output/(name+'.json'),result)
        report['scenarios'][name]={k:v for k,v in result.items() if k not in ('poses','decisions','world_obstacles')}
        report['scenarios'][name]['expectation_check']=assess(name,result)
    report['scenario_contracts_satisfied']=all(s['expectation_check']['satisfied'] for s in report['scenarios'].values())
    report['sources_unchanged']=(fingerprint(args.plan_result)==report['plan_sha256'] and all(fingerprint(p)==h for p,h in hashes.items()))
    report['status']='SCENARIOS_EXECUTED_REQUIRES_REVIEW'
    try:render(output/'preview.png',grid,original,results,worlds,footprint['radius_m'],footprint['margin_m']);report['preview']=str(output/'preview.png')
    except Exception as exc:report['preview_error']=str(exc)
    policy.check();write_json(output/'result.json',report);print(json.dumps(report,ensure_ascii=True,allow_nan=False),flush=True)
    return 0 if report['scenario_contracts_satisfied'] and report['sources_unchanged'] else 1


if __name__=='__main__':raise SystemExit(main())
