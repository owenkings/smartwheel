"""Causal stop/wait/resume simulation, with independent continuous-sweep audit.

This is an ideal numerical model, not a physical controller or a detour planner.
Sensor adapters expose only delayed current/past snapshots. Obstacle schedules
belong to world truth and are never passed into the control prediction.
"""
from __future__ import annotations

import math
import copy
import json
from numbers import Real
import numpy as np

from .geometry import (InflatedGrid, _angle, _approach, _number, _point,
                       _target, _validated_path)


def _intervals(values, label):
    result = []
    for item in values:
        if len(item) != 2:
            raise ValueError(label + ' requires start/end pairs')
        a, b = (_number(v, label, nonnegative=True) for v in item)
        if b <= a:
            raise ValueError(label + ' requires ordered intervals')
        result.append((a, b))
    if len(result) > 1000:
        raise ValueError(label + ' resource bound')
    return tuple(result)


def _discs(values):
    if not isinstance(values, (list, tuple)) or len(values) > 100:
        raise ValueError('at most 100 explicit obstacle dictionaries required')
    result, names = [], set()
    for item in values:
        if not isinstance(item, dict) or set(item) - {'id', 'center_xy', 'velocity_xy', 'radius_m', 'active_from_s', 'active_until_s'}:
            raise ValueError('unknown obstacle fields')
        name = item.get('id')
        if not isinstance(name, str) or not name or name in names:
            raise ValueError('unique nonempty obstacle id required')
        names.add(name)
        center, velocity = _point(item['center_xy']), _point(item.get('velocity_xy', (0., 0.)))
        if max(map(abs, center)) > 10000 or math.hypot(*velocity) > 5:
            raise ValueError('obstacle position/velocity resource bound')
        radius = _number(item['radius_m'], 'obstacle radius', positive=True)
        begin = _number(item.get('active_from_s', 0.), 'active start', nonnegative=True)
        end = _number(item.get('active_until_s', 1_000_000.), 'active end', nonnegative=True)
        if radius > 5 or not begin < end <= 1_000_000:
            raise ValueError('obstacle radius/active interval bound')
        result.append({'id': name, 'center_xy': center, 'velocity_xy': velocity,
                       'radius_m': radius, 'active_from_s': begin, 'active_until_s': end})
    return result


def _inside(t, intervals):
    return any(a <= t < b for a, b in intervals)


def _center(obstacle, t):
    return np.asarray(obstacle['center_xy']) + np.asarray(obstacle['velocity_xy']) * t


def _motion(position, yaw, v, w, duration):
    if abs(w) < 1e-10:
        new = position + v * duration * np.array([math.cos(yaw), math.sin(yaw)])
    else:
        new = position + (v / w) * np.array([math.sin(yaw + w * duration) - math.sin(yaw),
                                           math.cos(yaw) - math.cos(yaw + w * duration)])
    return new, _angle(yaw + w * duration)


def _closest_segment(a, b):
    delta = b - a
    squared = float(delta @ delta)
    fraction = float(np.clip(-float(a @ delta) / squared, 0., 1.)) if squared > 1e-24 else 0.
    return float(np.linalg.norm(a + fraction * delta)), fraction


def _dynamic_bound(position, yaw, v, w, start, end, obstacle, required):
    """Lower bound to a moving disc over one constant-midpoint-speed arc."""
    duration = end - start
    new, _ = _motion(position, yaw, v, w, duration)
    relative_start = position - _center(obstacle, start)
    relative_end = new - _center(obstacle, end)
    distance, fraction = _closest_segment(relative_start, relative_end)
    arc_bound = abs(v * w) * duration * duration / 8.
    sample, _ = _motion(position, yaw, v, w, duration * fraction)
    actual_sample_distance = float(np.linalg.norm(sample - _center(obstacle, start + duration * fraction)))
    minimum_sample = min(float(np.linalg.norm(relative_start)), float(np.linalg.norm(relative_end)), actual_sample_distance)
    return distance - arc_bound - required, minimum_sample <= required + 1e-12


def audit_dynamic_trajectory(inflated, poses, obstacles=()):
    """Audit executed midpoint-integrated poses, including world event boundaries.

    A nonpositive lower bound is never counted as certified clear. Arc cases
    without a sampled contact are explicitly uncertified, rather than asserted
    collisions. Obstacle appearance inside a step is checked at its true time.
    """
    if not isinstance(inflated, InflatedGrid):
        raise ValueError('InflatedGrid required')
    world = _discs(obstacles)
    rows = np.asarray(poses, dtype=float)
    if rows.ndim != 2 or rows.shape[1] != 6 or not 1 <= len(rows) <= 200001 or not np.isfinite(rows).all():
        raise ValueError('finite bounded [t,x,y,yaw,v,w] poses required')
    if rows[0, 0] < 0 or np.any(np.diff(rows[:, 0]) <= 0):
        raise ValueError('increasing nonnegative trajectory times required')
    violations = []
    lower = math.inf
    first = rows[0]
    if not inflated.is_free_world(first[1:3]):
        violations.append({'time_s': float(first[0]), 'kind': 'INITIAL_STATIC_SWEEP_UNCERTIFIED'})
    for obstacle in world:
        if obstacle['active_from_s'] <= first[0] < obstacle['active_until_s']:
            gap = float(np.linalg.norm(first[1:3] - _center(obstacle, first[0]))) - inflated.required_clearance_m - obstacle['radius_m']
            lower = min(lower, gap)
            if gap <= 0:
                violations.append({'time_s': float(first[0]), 'kind': 'INITIAL_DYNAMIC_SAFETY_ENVELOPE_CONTACT', 'obstacle_id': obstacle['id']})
    for a, b in zip(rows[:-1], rows[1:]):
        t0, t1 = float(a[0]), float(b[0]); duration = t1 - t0
        v, w = float((a[4] + b[4]) * .5), float((a[5] + b[5]) * .5)
        expected, yaw = _motion(a[1:3], a[3], v, w, duration)
        if np.linalg.norm(expected - b[1:3]) > 1e-7 or abs(_angle(yaw - b[3])) > 1e-7:
            violations.append({'time_s': t1, 'kind': 'TRAJECTORY_INTEGRATION_MISMATCH'})
            continue
        if not inflated.segment_is_free(a[1:3], b[1:3], extra_margin_m=abs(v*w)*duration*duration/8.):
            violations.append({'time_s': t1, 'kind': 'STATIC_SWEEP_UNCERTIFIED'})
        for obstacle in world:
            # Active intervals are half-open; an appearance exactly at the final
            # pose must still be checked even without a following time interval.
            if obstacle['active_from_s'] <= t1 < obstacle['active_until_s']:
                endpoint_gap = float(np.linalg.norm(b[1:3] - _center(obstacle, t1))) - inflated.required_clearance_m - obstacle['radius_m']
                lower = min(lower, endpoint_gap)
                if endpoint_gap <= 0:
                    violations.append({'time_s': t1, 'obstacle_id': obstacle['id'],
                                       'kind': 'ENDPOINT_DYNAMIC_SAFETY_ENVELOPE_CONTACT'})
            start, end = max(t0, obstacle['active_from_s']), min(t1, obstacle['active_until_s'])
            if start >= end:
                continue
            p, angle = _motion(a[1:3], a[3], v, w, start-t0)
            gap, contact = _dynamic_bound(p, angle, v, w, start, end, obstacle,
                                           inflated.required_clearance_m + obstacle['radius_m'])
            lower = min(lower, gap)
            if gap <= 1e-12:
                violations.append({'time_s': start, 'interval_end_s': end, 'obstacle_id': obstacle['id'],
                                   'kind': 'DYNAMIC_SAFETY_ENVELOPE_CONTACT' if contact else 'CURVED_DYNAMIC_SWEEP_UNCERTIFIED'})
    return {'status': 'CLEAR_UNDER_STATED_MODEL' if not violations else 'VIOLATION_OR_UNCERTIFIED',
            'violations': violations, 'violation_count': len(violations),
            'contact_count': sum('CONTACT' in v['kind'] for v in violations),
            'uncertified_count': sum('UNCERTIFIED' in v['kind'] or 'MISMATCH' in v['kind'] for v in violations),
            'minimum_dynamic_safety_gap_lower_bound_m': lower if math.isfinite(lower) else None,
            'contact_meaning': 'ASSUMED_CIRCLE_PLUS_MARGIN_ENVELOPE_NOT_MEASURED_PHYSICAL_CONTACT',
            'audited_until_s': float(rows[-1,0]), 'physical_navigation_validated': False}


def simulate_dynamic_path(inflated, world_points, obstacles=(), *, start_yaw=None,
                          max_duration_s=90., dt=.05, max_linear_speed=.25,
                          max_angular_speed=.6, linear_acceleration=.25,
                          angular_acceleration=1., sensor_period_s=.1,
                          perception_latency_s=.15, reaction_delay_s=.15,
                          observation_timeout_s=.4, clear_stable_s=.5,
                          sensor_dropouts=(), localization_dropouts=(),
                          max_wait_s=15., post_stop_horizon_s=5.,
                          goal_tolerance=.12, lookahead_m=.3,
                          resume_probe_distance_m=.3, resume_probe_time_limit_s=10.,
                          replan_callback=None, replan_wait_s=3., planning_delay_s=.5,
                          replan_cooldown_s=5., max_replans=3, static_track_speed_mps=.02,
                          replan_stop_reserve_m=0.):
    """Track a fixed path, stop/wait for hazards, and resume after stable clearance.

    Obstacles are world-truth dictionaries: id, center_xy at t=0, velocity_xy,
    radius_m, and optional active_from_s/active_until_s. The perfect synthetic
    sensor reports all currently existing discs after latency; known velocities
    are exact constant-velocity assumptions. Dropout intervals suppress fresh
    deliveries or localization trust at the current time only. A max-wait latch
    holds zero commands but continues world auditing until max_duration_s.
    """
    if not isinstance(inflated, InflatedGrid):
        raise ValueError('InflatedGrid required')
    points = _validated_path(world_points); world = _discs(obstacles)
    original_goal = points[-1].copy()
    if replan_callback is not None and not callable(replan_callback):
        raise ValueError('replan_callback must be callable or None')
    if type(max_replans) is not int or not 1 <= max_replans <= 3:
        raise ValueError('max_replans must be an integer in [1,3]')
    sensor_dropouts = _intervals(sensor_dropouts, 'sensor dropout')
    localization_dropouts = _intervals(localization_dropouts, 'localization dropout')
    positive = {'max_duration_s':max_duration_s,'dt':dt,'max_linear_speed':max_linear_speed,
                'max_angular_speed':max_angular_speed,'linear_acceleration':linear_acceleration,
                'angular_acceleration':angular_acceleration,'sensor_period_s':sensor_period_s,
                'observation_timeout_s':observation_timeout_s,'clear_stable_s':clear_stable_s,
                'max_wait_s':max_wait_s,'post_stop_horizon_s':post_stop_horizon_s,
                'goal_tolerance':goal_tolerance,'lookahead_m':lookahead_m,
                'resume_probe_distance_m':resume_probe_distance_m,
                'resume_probe_time_limit_s':resume_probe_time_limit_s,
                'replan_wait_s':replan_wait_s,'planning_delay_s':planning_delay_s,
                'replan_cooldown_s':replan_cooldown_s}
    settings = {k:_number(v,k,positive=True) for k,v in positive.items()}
    settings.update(perception_latency_s=_number(perception_latency_s,'latency',nonnegative=True),
                    reaction_delay_s=_number(reaction_delay_s,'reaction delay',nonnegative=True),
                    static_track_speed_mps=_number(static_track_speed_mps,'static track speed',nonnegative=True),
                    replan_stop_reserve_m=_number(replan_stop_reserve_m,'replan stop reserve',nonnegative=True))
    if max(replan_wait_s, planning_delay_s, replan_cooldown_s)>300 or static_track_speed_mps>.1 or replan_stop_reserve_m>2.:
        raise ValueError('replanning time/speed resource bound')
    settings['max_replans']=max_replans
    if dt > .2 or math.ceil(max_duration_s/dt)>100000 or max_linear_speed>2 or max_angular_speed>3:
        raise ValueError('simulation speed/time resource bound')
    if (reaction_delay_s>2 or perception_latency_s>2 or post_stop_horizon_s>30
            or resume_probe_distance_m>2 or resume_probe_time_limit_s>30):
        raise ValueError('prediction horizon resource bound')
    dt=settings['dt'];position=points[0].copy()
    direction=points[1]-points[0] if len(points)>1 else np.array([1.,0.])
    yaw=math.atan2(direction[1],direction[0]) if start_yaw is None else _angle(_number(start_yaw,'start yaw'))
    velocity=angular=progress=elapsed=0.
    cumulative=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(points,axis=0),axis=1))]
    poses=[[0.,float(position[0]),float(position[1]),yaw,0.,0.]]
    result={'schema_version':1,'status':'STARTING','poses':poses,'events':[],'decisions':[],
            'settings':settings,'world_obstacles':world,'sensor_dropouts':sensor_dropouts,
            'localization_dropouts':localization_dropouts,'mode':'STOP_WAIT_RESUME_WITH_CAUSAL_REPLANNING' if replan_callback is not None else 'STOP_WAIT_RESUME_NO_DETOUR',
            'footprint':{'radius_m':inflated.radius_m,'margin_m':inflated.margin_m,'source':'EXPLICIT_SIMULATION_ASSUMPTION'},
            'hardware_started':False,'control_topics_published':False,'navigation_validated':False,
            'localization_source':'VIRTUAL_TRUST_SIGNAL_NOT_VALIDATED_AMCL',
            'obstacle_velocity_model':'EXACT_KNOWN_CONSTANT_VELOCITY_SYNTHETIC_SENSOR',
            'resume_probe_model':'FINITE_ACCELERATION_PATH_FEEDBACK_TO_MINIMUM_PROGRESS_THEN_BRAKING_AND_POST_STOP',
            'collision_step_discarded':False,'max_wait_latched':False,
            'replanning_enabled':replan_callback is not None, 'replans':[],
            'path_segments':[{'installed_time_s':0.,'source':'INITIAL','path':points.tolist()}],
            'replan_stop_reserve_meaning':'EXTRA_PREDICTION_CLEARANCE_ONLY_FOR_CURRENT_STATIC_PATH_BLOCKERS_NOT_WORLD_RADIUS'}
    last_capture=None; tracks=[];holding=False;clear_since=None;stopped_since=None;latched=False
    previous_state=None; stopped_count=resume_count=hold_release_count=0; ever_moved=False
    pending_replan=None; attempted_geometry=set(); last_request_s=-math.inf
    stationary_geometry=None; stationary_since=None
    reserve_track_ids=set()

    def event(state,reason):
        nonlocal previous_state
        if (state,reason)!=previous_state:
            result['events'].append({'time_s':elapsed,'state':state,'reason':reason})
            previous_state=(state,reason)

    def finish(status,reason):
        audit=audit_dynamic_trajectory(inflated,poses,world)
        if audit['violation_count']:
            status,reason='FAIL','EXECUTED_SWEEP_VIOLATION_OR_UNCERTIFIED'
        result.update(status=status,reason=reason,audit=audit,elapsed_s=poses[-1][0],
                      final_goal_distance_m=float(np.linalg.norm(position-points[-1])),
                      stop_count=stopped_count,resume_count=resume_count,
                      hold_release_count=hold_release_count,
                      initial_readiness_wait_counted_as_stop=False,
                      final_speed_mps=velocity,final_angular_speed_radps=angular,
                      simulated_distance_m=sum(math.dist(a[1:3],b[1:3]) for a,b in zip(poses[:-1],poses[1:])),
                      continuation_beyond_audited_horizon_validated=False)
        return result

    def predictions_clear(next_v,next_w,cruise_target=None):
        # Predict one candidate step, a reserved reaction interval, finite braking,
        # then a stationary protection horizon. Only current sensor tracks enter.
        p=position.copy();angle=yaw;v0=velocity;w0=angular;t=elapsed
        stages=[(step,next_v,next_w)]
        hold_v,hold_w=next_v,next_w
        if cruise_target is not None:
            # Releasing a hold requires space for a real finite-acceleration
            # departure, not just one tiny command from zero speed. Otherwise
            # a persistent obstacle can cause repeated stop/creep/resume cycles.
            probe_p,probe_yaw=_motion(position,yaw,(velocity+next_v)*.5,(angular+next_w)*.5,step)
            probe_progress=progress;travel=abs((velocity+next_v)*.5)*step;probe_time=step
            for _ in range(1000):
                goal_distance=float(np.linalg.norm(points[-1]-probe_p))
                if travel>=resume_probe_distance_m or goal_distance<=goal_tolerance:break
                if probe_time>=resume_probe_time_limit_s-1e-10:
                    return False,'DEPARTURE_PROBE_PROGRESS_NOT_CERTIFIED'
                probe_target,probe_progress=_target(points,cumulative,probe_p,probe_progress,lookahead_m)
                probe_error=_angle(math.atan2(*(probe_target-probe_p)[::-1])-probe_yaw)
                target_w=float(np.clip(2*probe_error,-max_angular_speed,max_angular_speed))
                target_v=min(max_linear_speed,math.sqrt(2*linear_acceleration*max(0.,goal_distance-goal_tolerance*.5)))
                target_v*=max(0.,math.cos(probe_error)) if abs(probe_error)<=.6 else 0.
                h=min(dt,resume_probe_time_limit_s-probe_time)
                next_probe_v=_approach(hold_v,target_v,linear_acceleration*h)
                next_probe_w=_approach(hold_w,target_w,angular_acceleration*h)
                v,w=(hold_v+next_probe_v)*.5,(hold_w+next_probe_w)*.5
                probe_p,probe_yaw=_motion(probe_p,probe_yaw,v,w,h)
                travel+=abs(v)*h;probe_time+=h
                hold_v,hold_w=next_probe_v,next_probe_w
                stages.append((h,hold_v,hold_w))
            else:return False,'DEPARTURE_PROBE_PROGRESS_NOT_CERTIFIED'
        if settings['reaction_delay_s']>0:
            remaining=settings['reaction_delay_s']
            while remaining>1e-10:
                h=min(dt,remaining);stages.append((h,hold_v,hold_w));remaining-=h
        bv,bw=hold_v,hold_w
        for _ in range(1000):
            if abs(bv)<1e-10 and abs(bw)<1e-10:break
            h=min(dt,max(abs(bv)/linear_acceleration,abs(bw)/angular_acceleration))
            bv=_approach(bv,0.,linear_acceleration*h);bw=_approach(bw,0.,angular_acceleration*h)
            stages.append((h,bv,bw))
        else:raise ValueError('braking rollout budget exceeded')
        for h,v1,w1 in stages:
            v,w=(v0+v1)*.5,(w0+w1)*.5
            new,new_angle=_motion(p,angle,v,w,h)
            if not inflated.segment_is_free(p,new,extra_margin_m=abs(v*w)*h*h/8.):return False,'STATIC_BRAKING_SWEEP'
            for track in tracks:
                shifted={**track,'center_xy':np.asarray(track['center_xy'])-np.asarray(track['velocity_xy'])*track['stamp_s']}
                gap,_=_dynamic_bound(p,angle,v,w,t,t+h,shifted,prediction_required(track))
                if gap<=1e-10:return False,'DYNAMIC_BRAKING_SWEEP'
            p,angle,v0,w0,t=new,new_angle,v1,w1,t+h
        for track in tracks:
            center=np.asarray(track['center_xy'])+np.asarray(track['velocity_xy'])*(t-track['stamp_s'])
            relative=p-center
            future=relative-np.asarray(track['velocity_xy'])*post_stop_horizon_s
            distance,_=_closest_segment(relative,future)
            if distance <= prediction_required(track)+1e-10:
                return False,'POST_STOP_MOVING_OBSTACLE'
        return True,'CLEAR_PREDICTED_BRAKING_AND_POST_STOP_HORIZON'

    def current_center(track):
        return np.asarray(track['center_xy']) + np.asarray(track['velocity_xy']) * (elapsed-track['stamp_s'])

    def intersects_tracks(path, selected_tracks):
        segments = list(zip(path[:-1],path[1:])) or [(path[0],path[0])]
        for track in selected_tracks:
            center=current_center(track)
            required=inflated.required_clearance_m+track['radius_m']
            if any(_closest_segment(a-center,b-center)[0] <= required+1e-10 for a,b in segments):
                return True
        return False

    def static_path_blockers():
        remaining=np.vstack([position,points[cumulative>=progress-1e-9]])
        return [track for track in tracks if math.hypot(*track['velocity_xy']) <= static_track_speed_mps
                and intersects_tracks(remaining,[track])]

    def prediction_required(track):
        reserve=replan_stop_reserve_m if track['id'] in reserve_track_ids else 0.
        return inflated.required_clearance_m+track['radius_m']+reserve

    def replan_geometry():
        # Quantization throttles repeat requests; no observation timestamps or
        # future disappearance times participate in the retry identity.
        blocking=[]
        for track in static_path_blockers():
            center=current_center(track)
            blocking.append((track['id'],round(float(center[0])/.05),round(float(center[1])/.05),
                             round(float(track['radius_m']),6)))
        return tuple(sorted(blocking))

    def inspect_delivered_path(response):
        if not isinstance(response,dict) or response.get('status') not in ('PATH','NO_PATH','PLANNER_ERROR'):
            return None,'INVALID_PLANNER_RESPONSE'
        if response['status']!='PATH':return None,response['status']
        try:
            proposed=_validated_path(response['path'])
        except (KeyError,TypeError,ValueError,OverflowError):return None,'INVALID_PLANNER_PATH'
        if np.linalg.norm(proposed[0]-position)>1e-6:
            return None,'PATH_START_DISCONTINUITY'
        if np.linalg.norm(proposed[-1]-original_goal)>1e-6:
            return None,'PATH_CHANGED_GOAL'
        try:
            if (not inflated.is_free_world(proposed[0]) or
                    any(not inflated.segment_is_free(a,b) for a,b in zip(proposed[:-1],proposed[1:]))):
                return None,'PATH_STATIC_SWEEP_REJECTED'
            if intersects_tracks(proposed,tracks):return None,'PATH_LATEST_OBSERVATION_BLOCKED'
        except (ValueError,OverflowError,FloatingPointError):
            return None,'INVALID_PLANNER_PATH_RESOURCE_BOUND'
        return proposed,'INSTALLED'

    def coordinate_replan(fault,hazard):
        nonlocal pending_replan,points,cumulative,progress,latched,clear_since,holding,stopped_since
        nonlocal last_request_s,stationary_geometry,stationary_since
        if replan_callback is None:return hazard
        stationary=abs(velocity)<1e-9 and abs(angular)<1e-9
        if pending_replan is not None and elapsed+1e-10>=pending_replan['due_s']:
            item=pending_replan;pending_replan=None;entry=item['entry']
            entry['delivered_time_s']=elapsed
            if fault or not stationary:
                entry['status']='REJECTED_UNTRUSTED_OR_NOT_STOPPED'
            else:
                proposed,reason=inspect_delivered_path(item['response'])
                entry['status']=reason
                if proposed is not None:
                    points=proposed;cumulative=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(points,axis=0),axis=1))]
                    progress=0.;latched=False;result['max_wait_latched']=False
                    holding=True;clear_since=None;stopped_since=elapsed
                    result['path_segments'].append({'installed_time_s':elapsed,'source':'REPLAN',
                                                    'request_index':entry['request_index'],'path':points.tolist()})
                    stationary_geometry=None;stationary_since=None
                    # Recompute path feedback on the next step while remaining
                    # stopped; never execute an old-path candidate on delivery.
                    return 'REPLAN_INSTALLED_RECHECK'
            if entry['status']=='NO_PATH':attempted_geometry.add(item['geometry'])
            event('WAITING',entry['status'])
        if pending_replan is not None:return fault or 'REPLAN_PENDING'
        if fault or not stationary or not ever_moved or not holding:
            stationary_geometry=None;stationary_since=None
            return hazard
        geometry=replan_geometry()
        if not geometry:
            stationary_geometry=None;stationary_since=None
            return hazard
        if geometry!=stationary_geometry:
            stationary_geometry=geometry;stationary_since=elapsed
        if (elapsed-stationary_since<replan_wait_s-1e-10 or geometry in attempted_geometry
                or len(result['replans'])>=max_replans or elapsed-last_request_s<replan_cooldown_s-1e-10):
            return hazard
        snapshot={'elapsed_s':elapsed,'pose_x_y_yaw':[float(position[0]),float(position[1]),yaw],
                  'goal_xy':original_goal.tolist(),'observation_stamp_s':last_capture,
                  'tracks':copy.deepcopy(tracks)}
        entry={'request_index':len(result['replans'])+1,'requested_time_s':elapsed,
               'due_time_s':elapsed+planning_delay_s,'status':'PENDING','snapshot':copy.deepcopy(snapshot),
               'geometry_key':[list(x) for x in geometry]}
        try:
            response=replan_callback(copy.deepcopy(snapshot))
            # Snapshot callbacks have a serializable, finite evidence contract.
            response=json.loads(json.dumps(response,allow_nan=False))
        except Exception as exc:
            response={'status':'PLANNER_ERROR','reason':type(exc).__name__+': '+str(exc)}
        entry['planner_metadata']=response.get('metadata') if isinstance(response,dict) else None
        if isinstance(response,dict) and 'reason' in response:entry['planner_reason']=response['reason']
        result['replans'].append(entry);last_request_s=elapsed
        pending_replan={'due_s':elapsed+planning_delay_s,'response':response,'entry':entry,'geometry':geometry}
        return 'REPLAN_PENDING'

    initial=audit_dynamic_trajectory(inflated,poses,world)
    if initial['violation_count']:return finish('FAIL','INITIAL_OVERLAP_OR_UNCERTIFIED')
    if any(not inflated.segment_is_free(a,b) for a,b in zip(points[:-1],points[1:])):
        return finish('INVALID_PATH','STATIC_PATH_NOT_CERTIFIED')
    if len(points)==1:return finish('REACHED','GOAL_REACHED')
    while elapsed < max_duration_s-1e-10:
        step=min(dt,max_duration_s-elapsed)
        capture=math.floor((elapsed-perception_latency_s+1e-10)/sensor_period_s)*sensor_period_s
        if capture>=0 and not _inside(elapsed,sensor_dropouts) and (last_capture is None or capture>last_capture+1e-10):
            tracks=[{'id':o['id'],'center_xy':_center(o,capture).tolist(),'velocity_xy':o['velocity_xy'],
                     'radius_m':o['radius_m'],'stamp_s':capture} for o in world if o['active_from_s']<=capture<o['active_until_s']]
            last_capture=capture
        age=elapsed-last_capture if last_capture is not None else None
        fault=('OBSERVATIONS_UNAVAILABLE' if age is None else 'OBSERVATIONS_STALE' if age>observation_timeout_s+1e-10 else
               'LOCALIZATION_UNTRUSTED' if _inside(elapsed,localization_dropouts) else None)
        distance=float(np.linalg.norm(points[-1]-position))
        if distance<=goal_tolerance and abs(velocity)<1e-9 and abs(angular)<1e-9:return finish('REACHED','GOAL_REACHED')
        target,progress=_target(points,cumulative,position,progress,lookahead_m)
        error=_angle(math.atan2(*(target-position)[::-1])-yaw)
        desired_w=float(np.clip(2*error,-max_angular_speed,max_angular_speed))
        desired_v=min(max_linear_speed,math.sqrt(2*linear_acceleration*max(0.,distance-goal_tolerance*.5)))
        desired_v*=max(0.,math.cos(error)) if abs(error)<=.6 else 0.
        if distance<=goal_tolerance:desired_v=desired_w=0.
        candidate_v=_approach(velocity,desired_v,linear_acceleration*step)
        candidate_w=_approach(angular,desired_w,angular_acceleration*step)
        reserve_track_ids=({track['id'] for track in static_path_blockers()}
                           if replan_callback is not None and replan_stop_reserve_m>0 else set())
        full_departure_probe=holding
        safe,reason=predictions_clear(candidate_v,candidate_w,
                                     cruise_target=(desired_v,desired_w) if full_departure_probe else None)
        hazard=fault or (None if safe else reason)
        hazard=coordinate_replan(fault,hazard)
        if hazard:
            holding=True;clear_since=None
        if holding and not hazard and not latched:
            if abs(velocity)<1e-9 and abs(angular)<1e-9:
                clear_since=elapsed if clear_since is None else clear_since
                if elapsed-clear_since>=clear_stable_s-1e-10:
                    holding=False;clear_since=None;stopped_since=None;hold_release_count+=1
                    if ever_moved:resume_count+=1
            else:clear_since=None
        stopping=holding or latched or distance<=goal_tolerance
        next_v=_approach(velocity,0. if stopping else desired_v,linear_acceleration*step)
        next_w=_approach(angular,0. if stopping else desired_w,angular_acceleration*step)
        if stopping and abs(velocity)<1e-9 and abs(angular)<1e-9:
            if stopped_since is None:
                stopped_since=elapsed
                if ever_moved:stopped_count+=1
            if elapsed-stopped_since>=max_wait_s and distance>goal_tolerance:
                latched=True;result['max_wait_latched']=True
            state='WAITING_LATCHED' if latched else 'WAITING'
        elif stopping:state='BRAKING'
        else:state='TRACKING';stopped_since=None
        event(state,hazard or ('CLEARANCE_STABILITY_WAIT' if holding else 'GOAL_BRAKING' if distance<=goal_tolerance else 'TRACKING'))
        result['decisions'].append({'time_s':elapsed,'sensor_capture_s':last_capture,'sensor_age_s':age,
                                    'observed_discs':len(tracks),'state':state,'reason':hazard,
                                    'desired_v_mps':0. if stopping else desired_v,'desired_w_radps':0. if stopping else desired_w,
                                    'prediction_safe':safe,
                                    'full_departure_probe_used':full_departure_probe,
                                    'replan_stop_reserve_applied_ids':sorted(reserve_track_ids)})
        # Integrate even when the finite braking command cannot avoid contact.
        v,w=(velocity+next_v)*.5,(angular+next_w)*.5
        position,yaw=_motion(position,yaw,v,w,step)
        velocity,angular=next_v,next_w;elapsed+=step
        ever_moved=ever_moved or abs(velocity)>1e-9 or abs(angular)>1e-9
        poses.append([float(elapsed),float(position[0]),float(position[1]),yaw,velocity,angular])
        step_audit=audit_dynamic_trajectory(inflated,poses[-2:],world)
        if step_audit['violation_count']:return finish('FAIL','EXECUTED_SWEEP_VIOLATION_OR_UNCERTIFIED')
    return finish('WAITING' if abs(velocity)<1e-9 and abs(angular)<1e-9 and holding else 'TIMEOUT',
                  'STOPPED_WITHIN_AUDITED_HORIZON' if holding else 'DURATION_LIMIT')
