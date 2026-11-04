"""Causal recorded-lidar shadow diagnostics, never counterfactual sensor control."""
import argparse
from collections import Counter
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import uuid

import numpy as np

from .replay_cli import checked_frames, select_rows, digest, write, number


def axle_twist(linear_reference, angular_reference, base_from_lidar):
    """Transform an explicitly child-reference twist; do not discard side motion."""
    from .amcl_replay import planar_base_from_lidar
    fixed=planar_base_from_lidar(base_from_lidar)
    v=np.asarray(linear_reference,dtype=float);w=np.asarray(angular_reference,dtype=float)
    if (v.shape!=(3,) or w.shape!=(3,) or not np.isfinite(v).all() or not np.isfinite(w).all()
            or np.max(np.abs(v))>100 or np.max(np.abs(w))>100):
        return {'status':'UNKNOWN','forward_mps':None,'yaw_radps':None,'lateral_mps':None,
                'reason':'INVALID_RECORDED_TWIST'}
    angular=fixed[:3,:3]@w
    body=fixed[:3,:3]@v-np.cross(angular,fixed[:3,3])
    compatible=(max(abs(body[1]),abs(body[2]),abs(angular[0]),abs(angular[1]))<=1e-6
                and abs(body[0])<=2 and abs(angular[2])<=3)
    return {'status':'PLANAR_MODEL_COMPATIBLE_UNVALIDATED' if compatible else 'UNKNOWN',
            'forward_mps':float(body[0]) if compatible else None,
            'yaw_radps':float(angular[2]) if compatible else None,
            'lateral_mps':float(body[1]),'full_axle_linear_xyz_mps':body.tolist(),
            'full_axle_angular_xyz_radps':angular.tolist(),
            'source':'SHA_VERIFIED_FROZEN_ODOMETRY_CHILD_TWIST_WITH_PLANAR_LEVER_ARM',
            'model_compatibility_tolerance':1e-6,'physical_accuracy_validated':False}


def arguments(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--project-root',type=Path,required=True)
    parser.add_argument('--source-cell',type=Path,required=True)
    parser.add_argument('--window',type=number,nargs=2,default=[62.,180.])
    args=parser.parse_args(argv)
    if not 0<=args.window[0]<args.window[1]:parser.error('ordered nonnegative window required')
    return args


def summarize(rows):
    return {'frames':len(rows),'status_counts':dict(Counter(r['status'] for r in rows)),
            'stop_reason_counts':dict(Counter(x for r in rows for x in r['stop_reasons'])),
            'review_reason_counts':dict(Counter(x for r in rows for x in r['review_reasons'])),
            'maximum_modeled_capture_to_stop_distance_m':max((r.get('modeled_capture_to_stop_distance_m',0.) for r in rows),default=0.),
            'physical_navigation_validated':False,'closed_loop_replay_performed':False}


def render(directory, frames, velocities, cases, examples):
    import matplotlib
    matplotlib.use('Agg')
    from matplotlib import pyplot as plt
    from matplotlib.patches import Circle
    epoch=frames[0]['stamp_ns'];times=np.array([(f['stamp_ns']-epoch)*1e-9 for f in frames])
    fig,axes=plt.subplots(2,2,figsize=(14,8),constrained_layout=True)
    fixed=examples['fixed'];inverse=np.linalg.inv(fixed)
    xy=np.array([(f['odometry_T']@inverse)[:2,3] for f in frames])
    axes[0,0].plot(xy[:,0],xy[:,1],color='#287ba5');axes[0,0].set_title('Recorded shared-odometry path (not ground truth)')
    axes[0,0].set_aspect('equal');axes[0,0].set_xlabel('Recorded map x (m)');axes[0,0].set_ylabel('Recorded map y (m)')
    axes[0,1].plot(times,[velocities[f['stamp_ns']]['forward_mps'] if velocities[f['stamp_ns']]['forward_mps'] is not None else np.nan for f in frames],label='Recorded axle velocity candidate')
    axes[0,1].axhline(.25,linestyle='--',color='#bb6d34',label='Earlier simulation speed assumption')
    axes[0,1].set_title('Recorded speed, not a commanded or accepted limit');axes[0,1].set_ylabel('m/s');axes[0,1].legend(fontsize=8)
    for name in ['diagnostic_no_added_delay','diagnostic_added_250ms','diagnostic_added_600ms','diagnostic_delivery_dropout']:
        rows=cases[name];axes[1,0].plot(times,[r['observation_age_s'] if r['observation_age_s'] is not None else np.nan for r in rows],label=name.replace('diagnostic_',''))
    axes[1,0].axhline(.4,color='black',linestyle=':',label='Declared stale threshold');axes[1,0].set_ylabel('Source-stamp age (s)');axes[1,0].legend(fontsize=7)
    labels=list(cases);bottom=np.zeros(len(labels))
    for status,color in [('STOP_REQUIRED','#c06c59'),('REVIEW_REQUIRED','#d3b35f'),('NO_STOP_TRIGGER_UNVALIDATED','#6aa28a')]:
        counts=np.array([sum(r['status']==status for r in cases[n]) for n in labels]);axes[1,1].bar(np.arange(len(labels)),counts,bottom=bottom,label=status,color=color);bottom+=counts
    axes[1,1].set_xticks(np.arange(len(labels)),[n.replace('diagnostic_','diag_').replace('_','\n') for n in labels],fontsize=7);axes[1,1].set_ylabel('Shadow evaluations');axes[1,1].legend(fontsize=7)
    for ax in [axes[0,1],axes[1,0]]:ax.set_xlabel('Window elapsed time (s)');ax.grid(alpha=.2)
    fig.suptitle('Actual recorded lidar endpoints + causal delivery faults\nShadow triggers only: no vehicle commands, no counterfactual avoidance or physical acceptance')
    fig.savefig(directory/'preview.png',dpi=150);plt.close(fig)
    observation,decision=examples['observation'],examples['decision']
    fig,ax=plt.subplots(figsize=(9,6),constrained_layout=True)
    centers=observation['voxel_centers_body_xy'];ax.scatter(centers[:,0],centers[:,1],s=4,color='#6086a5',label='Recorded conservative voxel centers')
    poses=np.asarray(decision.get('poses',[]))
    if len(poses):
        ax.plot(poses[:,1],poses[:,2],color='#2b9969',label='Hypothetical braking centerline')
        for row in poses[::max(1,len(poses)//10)]:ax.add_patch(Circle(row[1:3],.65,fill=False,color='#2b9969',alpha=.25))
    ax.add_patch(Circle([0,0],.65,fill=False,color='#ba5b4e',label='Assumed radius + margin'))
    ax.scatter(*observation['sensor_origin_body_xy'],marker='x',c='black',label='Configured lidar origin')
    ax.set_aspect('equal');ax.set_xlim(-1,4);ax.set_ylim(-2.5,2.5);ax.set_xlabel('Capture axle x (m)');ax.set_ylabel('Capture axle y (m)');ax.legend(fontsize=8)
    ax.set_title('Longest modeled stopping-distance example\nRecorded endpoints held fixed for geometry; obstacle motion and ego timing unverified')
    fig.savefig(directory/'braking_example.png',dpi=150);plt.close(fig)


def main(argv=None):
    args=arguments(argv)
    from wc_runtime.storage_policy import StoragePolicy
    from rclpy.serialization import deserialize_message
    from nav_msgs.msg import Odometry
    from .recorded_observations import project_observation,CausalObservationBuffer
    from .shadow_safety import evaluate_shadow_stop
    policy=StoragePolicy(args.project_root);policy.check();source=args.source_cell.absolute()
    raw_runtime=(source/'runtime_config.json').read_bytes();raw_index=(source/'frontend/index.jsonl').read_bytes()
    runtime=json.loads(raw_runtime);rows=[json.loads(x) for x in raw_index.decode().splitlines()]
    if (runtime.get('mode')!='left' or runtime.get('motion_model')!='planar_ekf'
            or runtime.get('offline_source_selection',{}).get('selected_sides')!=['left']):
        raise ValueError('exact planar prepared single-left frontend required')
    from .rebuild import validate_frontend_row
    stamps=[validate_frontend_row(row) for row in rows]
    if not stamps or any(a>=b for a,b in zip(stamps,stamps[1:])):
        raise ValueError('strictly increasing frozen frontend timestamps required')
    epoch=stamps[0];selected=select_rows(rows,epoch,args.window,0)
    if len(selected)<5 or len(selected)>2000:raise ValueError('bounded nonempty recording window required')
    hashes={str(source/'runtime_config.json'):hashlib.sha256(raw_runtime).hexdigest(),str(source/'frontend/index.jsonl'):hashlib.sha256(raw_index).hexdigest()}
    frames,counts=checked_frames(source,selected,runtime,hashes)
    fixed=np.eye(4);fixed[:2,3]=runtime['mounts']['left']['t_axle_lidar_m'][:2]
    ground=-(runtime['mounts']['left']['t_axle_lidar_m'][2]+runtime['wheel_candidate']['wheel_radius_m'])
    observations=[];velocities={};projection_rows=[];largest_example=None;largest_distance=-1.
    for row,frame in zip(selected,frames):
        raw=(source/'frontend'/row['odom_cdr_file']).read_bytes()
        if hashlib.sha256(raw).hexdigest()!=row['odom_cdr_sha256']:raise ValueError('odometry changed during decoding')
        odom=deserialize_message(raw,Odometry);twist=odom.twist.twist
        velocity=axle_twist([twist.linear.x,twist.linear.y,twist.linear.z],
                            [twist.angular.x,twist.angular.y,twist.angular.z],fixed)
        velocities[frame['stamp_ns']]=velocity
        observation=project_observation(frame['points'],frame['stamp_ns'],fixed,ground,horizontal_fov_rad=None)
        observation['summary'].update(time_source='FRONTEND_ARRIVAL_REFERENCE_NOT_SENSOR_ACQUISITION',time_validated=False)
        # Keep only the diagnostic payload in the causal buffer. The immutable
        # original full XYZ and CDR remain in the frozen source, not copied here.
        packet={k:v for k,v in observation.items() if k not in ('points_body_xy_lidar_z','voxel_counts')}
        observations.append(packet)
        body=observation['points_body_xy_lidar_z'];near=body[np.linalg.norm(body[:,:2],axis=1)<=.65]
        projection_rows.append({'stamp_ns':frame['stamp_ns'],'summary':observation['summary'],
                                'raw_retained_points_inside_assumed_circle':len(near),
                                'inside_circle_low_height_rows':int(np.sum((near[:,2]-ground<=.15))) if len(near) else 0,
                                'velocity':velocity})
    run=datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')+'_'+uuid.uuid4().hex[:8]
    output=policy.resolve('reports/navigation_recorded_shadow/'+run);output.mkdir(parents=True,exist_ok=False)
    cases={};case_specs=[('policy_gate_localization_untrusted',0.,False,None),
                         ('diagnostic_no_added_delay',0.,True,None),
                         ('diagnostic_added_250ms',.25,True,None),
                         ('diagnostic_added_600ms',.6,True,None),
                         ('diagnostic_delivery_dropout',0.,True,(85.,90.))]
    for name,delay,assumed_trust,dropout in case_specs:
        schedule=[]
        for observation in observations:
            received=observation['stamp_ns']+round(delay*1e9);relative=(received-epoch)*1e-9
            if dropout and dropout[0]<=relative<dropout[1]:continue
            schedule.append({'stamp_ns':observation['stamp_ns'],'received_time_ns':received,'observation':observation})
        buffer=CausalObservationBuffer(schedule);decisions=[]
        for frame in frames:
            now=frame['stamp_ns'];snapshot=buffer.latest(now,400_000_000)
            velocity=velocities.get(snapshot['stamp_ns'],{'forward_mps':None,'yaw_radps':None,'lateral_mps':None})
            decision=evaluate_shadow_stop(snapshot,velocity['forward_mps'],velocity['yaw_radps'],now,
                                          lateral_velocity_mps=velocity['lateral_mps'],
                                          localization_trusted=assumed_trust,coverage_validated=False,
                                          horizontal_fov_rad=None,
                                          velocity_source='FROZEN_CHILD_TWIST_TRANSFORMED_TO_AXLE_UNVALIDATED')
            decision['assumed_trust_for_diagnostic_only']=assumed_trust
            decision['elapsed_from_source_start_s']=(now-epoch)*1e-9
            distance=decision.get('modeled_capture_to_stop_distance_m',0.)
            if name=='diagnostic_no_added_delay' and distance>largest_distance:
                largest_example=(snapshot['observation'],decision);largest_distance=distance
            decisions.append({k:v for k,v in decision.items() if k!='poses'})
        cases[name]=decisions
        write(output/(name+'.json'),{'extra_latency_s':delay,'delivery_dropout_s':dropout,
                                    'localization_trust_is_hypothetical':assumed_trust,
                                    'summary':summarize(decisions),'frames':decisions})
    summary={'schema_version':1,'status':'SHADOW_REPLAY_COMPLETE_NO_CONTROL_EXECUTED','output':str(output),
             'input_frames':len(frames),'source_cell':str(source),'source_sha256':hashes,
             'sources_unchanged':all(digest(n)==h for n,h in hashes.items()),
             'window_s':args.window,'base_from_lidar_xy':fixed[:2,3].tolist(),'ground_z_lidar_candidate':ground,
             'case_summaries':{n:summarize(rows) for n,rows in cases.items()},
             'recorded_speed_max_abs_mps':max((abs(v['forward_mps']) for v in velocities.values() if v['forward_mps'] is not None),default=None),
             'velocity_unknown_frames':sum(v['status']=='UNKNOWN' for v in velocities.values()),
             'max_lateral_velocity_residual_mps':max((abs(v['lateral_mps']) for v in velocities.values() if v.get('lateral_mps') is not None),default=None),
             'projection_summary':{'retained_rows':sum(r['summary']['retained_rows'] for r in projection_rows),
                                   'frames_with_raw_circle_overlap':sum(r['raw_retained_points_inside_assumed_circle']>0 for r in projection_rows),
                                   'source_points_modified':False,'self_returns_removed':False},
             'ground_model_measured':False,'sensor_fov_validated':False,'source_time_validated':False,
             'obstacle_velocities_known':False,'localization_validated':False,'physical_navigation_validated':False,
             'hardware_started':False,'control_topics_published':False,'closed_loop_replay_performed':False,
             'counterfactual_detour_sensor_replay_performed':False}
    assert summary['sources_unchanged']
    write(output/'projection.json',projection_rows);write(output/'input_contract.json',{'frame_counts':counts,'source_sha256':hashes,'time_semantics':'FRONTEND_ARRIVAL_REFERENCE_NOT_VERIFIED_SENSOR_CAPTURE'})
    try:render(output,frames,velocities,cases,{'fixed':fixed,'observation':largest_example[0],'decision':largest_example[1]});summary['preview']=str(output/'preview.png')
    except Exception as exc:summary['preview_error']=str(exc)
    policy.check();write(output/'result.json',summary)
    print(json.dumps({k:v for k,v in summary.items() if k!='source_sha256'},ensure_ascii=True,allow_nan=False),flush=True)
    return 0


if __name__=='__main__':raise SystemExit(main())
