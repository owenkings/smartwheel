"""Recorded-observation stop triggers in the observation's capture body frame.

This module never executes control, tracks a counterfactual robot, or certifies
free space. Endpoints are held fixed only for a stated geometric diagnostic;
their actual velocities and the historical-to-current ego transform are unknown.
"""
from __future__ import annotations

import math
from numbers import Real
import numpy as np


MAX_VOXELS = 100_000
MAX_PREDICTION_STEPS = 2000
PERIMETER_SAMPLES = 64


def _number(value, label, *, positive=False, nonnegative=False):
    if isinstance(value, (bool, np.bool_)) or not isinstance(value, Real) or not math.isfinite(value):
        raise ValueError(label + ' must be finite numeric')
    value = float(value)
    if (positive and value <= 0) or (nonnegative and value < 0):
        raise ValueError(label + ' outside its range')
    return value


def _time(value, label):
    if type(value) is not int or not 0 <= value < 2**63:
        raise ValueError(label + ' must be bounded integer nanoseconds')
    return value


def _velocity(value, label, bound):
    if value is None:
        return None
    if isinstance(value, (bool, np.bool_)) or not isinstance(value, Real):
        raise ValueError(label + ' must be numeric or None')
    if not math.isfinite(value):
        return None
    if abs(value) > bound:
        raise ValueError(label + ' exceeds shadow-model bound')
    return float(value)


def _approach_zero(value, amount):
    return math.copysign(max(0., abs(value)-amount), value)


def _motion(position, yaw, v, w, duration):
    if abs(w) < 1e-10:
        return position + v*duration*np.array([math.cos(yaw), math.sin(yaw)]), yaw+w*duration
    return position + v/w*np.array([math.sin(yaw+w*duration)-math.sin(yaw),
                                    math.cos(yaw)-math.cos(yaw+w*duration)]), yaw+w*duration


def _point_segment_distances(points, start, end):
    vector = end-start
    squared = float(vector @ vector)
    fraction = np.clip(((points-start) @ vector)/squared, 0., 1.) if squared > 1e-24 else np.zeros(len(points))
    return np.linalg.norm(points-(start+fraction[:, None]*vector), axis=1)


def _geometric_coverage(poses, radius, observation, sensor_range_m, horizontal_fov_rad):
    origin = np.asarray(observation.get('sensor_origin_body_xy', [0., 0.]), dtype=float)
    if origin.shape != (2,) or not np.isfinite(origin).all() or np.max(np.abs(origin)) > 10:
        raise ValueError('sensor origin must be bounded finite body xy')
    sensor_yaw = _number(observation.get('sensor_yaw_body_rad', 0.), 'sensor yaw')
    try:
        near, far = [_number(v, 'sensor range', nonnegative=True) for v in sensor_range_m]
    except (TypeError, ValueError) as exc:
        raise ValueError('sensor range needs two finite bounds') from exc
    if not 0 <= near < far <= 100:
        raise ValueError('invalid configured sensor range')
    low = high = None
    if horizontal_fov_rad is not None:
        try:
            low, high = [_number(v, 'FOV angle') for v in horizontal_fov_rad]
        except (TypeError, ValueError) as exc:
            raise ValueError('FOV needs two finite bounds') from exc
        if not -math.pi <= low < high <= math.pi:
            raise ValueError('invalid configured FOV')
    angles = np.arange(PERIMETER_SAMPLES)*2*math.pi/PERIMETER_SAMPLES
    perimeter = radius*np.column_stack([np.cos(angles), np.sin(angles)])
    # Full circular envelope, not just centerline: unobserved side/rear and the
    # near-range blind region cannot silently become clear space.
    centers = np.asarray([p[1:3] for p in poses])
    points = np.concatenate([(c+perimeter) for c in centers]+[centers], axis=0)
    relative = points-origin
    ranges = np.linalg.norm(relative, axis=1)
    bearing = np.arctan2(np.sin(np.arctan2(relative[:, 1], relative[:, 0])-sensor_yaw),
                         np.cos(np.arctan2(relative[:, 1], relative[:, 0])-sensor_yaw))
    outside = ((bearing < low-1e-12) | (bearing > high+1e-12)) if low is not None else None
    blind = ranges < near-1e-12
    distant = ranges > far+1e-12
    return {'configured': low is not None, 'fov_min_max_rad': [low, high] if low is not None else None, 'range_min_max_m': [near, far],
            'sampled_envelope_points': len(points), 'perimeter_samples_per_pose': PERIMETER_SAMPLES,
            'outside_fov_points': int(outside.sum()) if outside is not None else None, 'blind_zone_points': int(blind.sum()),
            'beyond_range_points': int(distant.sum()),
            'potentially_covered_fraction': float(np.mean(~(outside | blind | distant))) if outside is not None else None,
            'free_ray_visibility_tested': False,
            'meaning': 'CONFIGURED_GEOMETRIC_POTENTIAL_ONLY_NOT_ACTUAL_FREE_SPACE_COVERAGE'}


def evaluate_shadow_stop(buffer_snapshot, v_mps, w_radps, now_ns, *, localization_trusted,
                         lateral_velocity_mps=None, velocity_source='FROZEN_ODOMETRY_TWIST_UNVALIDATED',
                         radius_m=.55, margin_m=.10, linear_deceleration_m_s2=.25,
                         angular_deceleration_rad_s2=1., reaction_delay_s=.15,
                         observation_timeout_s=.4, prediction_dt_s=.05,
                         max_prediction_s=20., horizontal_fov_rad=None,
                         sensor_range_m=(.3, 8.), coverage_validated=False):
    """Evaluate stop/review triggers; return no acceptance or vehicle commands.

    ``buffer_snapshot`` is CausalObservationBuffer.latest(now): observation,
    stamp_ns, received_time_ns, usable. Observation geometry is body-frame at
    capture, not at delivery. The modeled capture-to-stop path reserves age plus
    reaction with the same recorded twist, then uses bounded deceleration.
    """
    now_ns = _time(now_ns, 'now')
    if type(localization_trusted) is not bool or type(coverage_validated) is not bool:
        raise ValueError('trust/coverage flags must be explicit booleans')
    radius = _number(radius_m, 'radius', positive=True)
    margin = _number(margin_m, 'margin', nonnegative=True)
    deceleration = _number(linear_deceleration_m_s2, 'linear deceleration', positive=True)
    angular_deceleration = _number(angular_deceleration_rad_s2, 'angular deceleration', positive=True)
    reaction = _number(reaction_delay_s, 'reaction delay', nonnegative=True)
    timeout = _number(observation_timeout_s, 'observation timeout', positive=True)
    dt = _number(prediction_dt_s, 'prediction dt', positive=True)
    limit = _number(max_prediction_s, 'prediction limit', positive=True)
    if radius > 5 or margin > 2 or dt > .2 or dt < .005 or limit > 30 or reaction > 2 or timeout > 10:
        raise ValueError('shadow resource or model bounds exceeded')
    if not isinstance(velocity_source, str) or not velocity_source or len(velocity_source) > 200:
        raise ValueError('velocity source must be a bounded explicit label')
    velocity = _velocity(v_mps, 'forward velocity', 2.)
    angular = _velocity(w_radps, 'yaw velocity', 3.)
    lateral = _velocity(lateral_velocity_mps, 'lateral velocity', 2.)
    stop_reasons, review_reasons = [], []
    if not localization_trusted:
        stop_reasons.append('LOCALIZATION_UNTRUSTED')
    if velocity is None or angular is None:
        review_reasons.append('RECORDED_VELOCITY_UNKNOWN')
    if lateral is None:
        review_reasons.append('LATERAL_VELOCITY_UNVERIFIED')
    elif abs(lateral) > .02:
        review_reasons.append('LATERAL_MOTION_OUTSIDE_UNICYCLE_MODEL')
    if not coverage_validated:
        review_reasons.append('SENSOR_COVERAGE_UNVALIDATED')
    result = {'schema_version': 1, 'status': None, 'stop_reasons': stop_reasons,
              'review_reasons': review_reasons, 'now_ns': now_ns,
              'velocity': {'forward_mps': velocity, 'yaw_radps': angular, 'lateral_mps': lateral,
                           'source': velocity_source, 'physical_accuracy_validated': False},
              'footprint': {'radius_m': radius, 'margin_m': margin, 'measured': False},
              'settings': {'linear_deceleration_m_s2': deceleration,
                           'angular_deceleration_rad_s2': angular_deceleration,
                           'reaction_delay_s': reaction, 'observation_timeout_s': timeout,
                           'prediction_dt_s': dt, 'max_prediction_s': limit},
              'poses': [], 'coverage': None, 'observation_age_s': None,
              'capture_envelope_overlap_voxels': 0, 'braking_sweep_trigger_voxels': 0,
              'minimum_shadow_gap_lower_bound_m': None,
              'coordinate_basis': 'CAPTURE_BODY_NO_VERIFIED_EGO_MOTION_COMPENSATION',
              'stamp_semantics': 'SUPPLIED_STAMP_NOT_VALIDATED_SENSOR_ACQUISITION_TIME',
              'prediction_model': 'CAPTURE_FRAME_RECORDED_TWIST_HOLD_FOR_AGE_PLUS_REACTION_THEN_FINITE_BRAKING',
              'obstacle_velocity_known': False, 'single_scan_proves_static': False,
              'missing_returns_are_free': False, 'control_topics_published': False,
              'hardware_started': False, 'physical_navigation_validated': False,
              'closed_loop_replay_performed': False,
              'interpretation': 'SHADOW_STOP_TRIGGER_NOT_ACTUAL_STOP_OR_COUNTERFACTUAL_AVOIDANCE_SUCCESS',
              'limitations': ['endpoint_voxels_held_fixed_for_geometry_only_actual_motion_unknown',
                              'near_returns_may_be_self_returns_or_true_obstacles_not_removed',
                              'configured_FOV_does_not_prove_free_ray_visibility',
                              'capture_to_delivery_twist_hold_is_unvalidated',
                              'recorded_path_is_not_changed_or_executed']}

    def finish():
        result['status'] = ('STOP_REQUIRED' if stop_reasons else 'REVIEW_REQUIRED' if review_reasons
                            else 'NO_STOP_TRIGGER_UNVALIDATED')
        return result

    if not isinstance(buffer_snapshot, dict) or buffer_snapshot.get('observation') is None:
        stop_reasons.append('OBSERVATIONS_UNAVAILABLE')
        return finish()
    observation = buffer_snapshot['observation']
    if not isinstance(observation, dict):
        raise ValueError('observation must be a dictionary')
    stamp = _time(buffer_snapshot.get('stamp_ns'), 'capture stamp')
    received = _time(buffer_snapshot.get('received_time_ns'), 'received stamp')
    if observation.get('stamp_ns') != stamp:
        raise ValueError('wrapper capture stamp differs from observation')
    result.update(observation_stamp_ns=stamp, received_time_ns=received)
    if stamp > now_ns or received > now_ns or received < stamp:
        stop_reasons.append('FUTURE_OR_INCONSISTENT_OBSERVATION_TIME')
        return finish()
    age = (now_ns-stamp)*1e-9
    result['observation_age_s'] = age
    if age > timeout+1e-12:
        stop_reasons.append('OBSERVATIONS_STALE')
    if buffer_snapshot.get('usable') is not True:
        stop_reasons.append('OBSERVATIONS_NOT_USABLE')
    if age > 0:
        review_reasons.append('OLD_CAPTURE_WITHOUT_VERIFIED_EGO_COMPENSATION')
    centers = np.asarray(observation.get('voxel_centers_body_xy'), dtype=float)
    if centers.ndim != 2 or centers.shape[1] != 2 or len(centers) > MAX_VOXELS or not np.isfinite(centers).all():
        raise ValueError('bounded finite Nx2 obstacle voxel centers required')
    if centers.size and np.max(np.abs(centers)) > 100:
        raise ValueError('voxel metric bound exceeded')
    voxel_radius = _number(observation.get('voxel_radius_m'), 'voxel radius', nonnegative=True)
    if voxel_radius > 1:
        raise ValueError('voxel radius exceeds model bound')
    summary=observation.get('summary',{})
    if not isinstance(summary,dict):raise ValueError('observation summary must be a dictionary')
    near_excluded=summary.get('near_range_height_candidate_rows',0)
    if type(near_excluded) is not int or not 0<=near_excluded<=MAX_VOXELS:
        raise ValueError('near-range exclusion count must be bounded nonnegative integer')
    result['near_range_height_candidate_rows']=near_excluded
    if near_excluded:
        review_reasons.append('NEAR_RANGE_POINTS_EXCLUDED_REVIEW')
    required = radius+margin+voxel_radius
    initial_gap = np.linalg.norm(centers, axis=1)-required
    result['capture_envelope_overlap_voxels'] = int(np.sum(initial_gap <= 1e-12))
    if result['capture_envelope_overlap_voxels']:
        stop_reasons.append('CAPTURE_ENVELOPE_OVERLAP_SELF_RETURN_OR_OBSTACLE')
    if not len(centers):
        review_reasons.append('NO_OBSTACLE_ENDPOINTS_DOES_NOT_PROVE_FREE_SPACE')
    minimum = float(initial_gap.min()) if len(centers) else math.inf
    # No rollout can be calculated from missing velocities. Do not fill either
    # component with zero or silently substitute a future finite difference.
    if velocity is None or angular is None:
        return finish()
    hold = age+reaction
    braking_time = max(abs(velocity)/deceleration, abs(angular)/angular_deceleration)
    duration = hold+braking_time
    if (duration > limit or math.ceil(duration/dt)+2 > MAX_PREDICTION_STEPS
            or (math.ceil(duration/dt)+1)*len(centers) > 20_000_000):
        review_reasons.append('BRAKING_HORIZON_EXCEEDS_DIAGNOSTIC_RESOURCE_BOUND')
        return finish()
    p = np.zeros(2); yaw = elapsed = 0.;v = velocity;w = angular
    poses = [[0.,0.,0.,0.,v,w]]
    triggered = np.zeros(len(centers), dtype=bool)
    while elapsed < duration-1e-12:
        h = min(dt,duration-elapsed)
        if elapsed < hold-1e-12:
            h = min(h,hold-elapsed);next_v,next_w=v,w
        else:
            next_v = _approach_zero(v,deceleration*h)
            next_w = _approach_zero(w,angular_deceleration*h)
        mid_v,mid_w=(v+next_v)*.5,(w+next_w)*.5
        new,new_yaw=_motion(p,yaw,mid_v,mid_w,h)
        if len(centers):
            arc_bound=abs(mid_v*mid_w)*h*h/8.
            gap=_point_segment_distances(centers,p,new)-arc_bound-required
            triggered |= gap<=1e-12
            minimum=min(minimum,float(gap.min()))
        elapsed+=h;p,yaw,v,w=new,new_yaw,next_v,next_w
        poses.append([float(elapsed),float(p[0]),float(p[1]),float(yaw),v,w])
    result['poses']=poses
    result['modeled_capture_to_stop_time_s']=duration
    result['modeled_capture_to_stop_distance_m']=sum(math.dist(a[1:3],b[1:3]) for a,b in zip(poses,poses[1:]))
    result['braking_sweep_trigger_voxels']=int(triggered.sum())
    result['minimum_shadow_gap_lower_bound_m']=minimum if math.isfinite(minimum) else None
    if triggered.any():
        stop_reasons.append('MODELED_BRAKING_SWEEP_MEETS_RECORDED_ENDPOINT_ENVELOPE')
    fov = horizontal_fov_rad if horizontal_fov_rad is not None else observation.get('horizontal_fov_rad')
    coverage=_geometric_coverage(poses,radius+margin,observation,sensor_range_m,fov)
    result['coverage']=coverage
    if not coverage['configured']:
        review_reasons.append('SENSOR_FOV_UNKNOWN')
    if coverage['outside_fov_points']:
        review_reasons.append('SWEPT_ENVELOPE_OUTSIDE_CONFIGURED_FOV')
    if coverage['blind_zone_points']:
        review_reasons.append('SWEPT_ENVELOPE_INTERSECTS_CONFIGURED_NEAR_BLIND_ZONE')
    if coverage['beyond_range_points']:
        review_reasons.append('SWEPT_ENVELOPE_BEYOND_CONFIGURED_SENSOR_RANGE')
    return finish()
