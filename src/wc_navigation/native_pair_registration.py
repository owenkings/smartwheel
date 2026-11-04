"""Native-lidar cloud consistency diagnostics, never vehicle localization.

Each transform maps an observed source cloud to a target cloud in their native
sensor coordinates. Object stationarity and sensor/body mounting are unknown.
Even a unique, reversible low-residual cloud alignment remains untrusted.
"""
from dataclasses import asdict, dataclass
import math

import numpy as np
from scipy.spatial import cKDTree


@dataclass(frozen=True)
class NativePairPolicy:
    voxel_m: float = .10
    range_min_m: float = .30
    range_max_m: float = 8.
    max_points: int = 2000
    max_iterations: int = 100
    correspondence_limit_m: float = .30
    minimum_correspondences: int = 50
    minimum_inlier_fraction: float = .20
    normal_neighbors: int = 12
    normal_radius_max_m: float = .50
    normal_curvature_max: float = .10
    rotation_length_scale_m: float = 1.
    weak_information_eigen_fraction: float = .001
    cycle_translation_review_m: float = .05
    cycle_rotation_review_deg: float = 2.
    hypothesis_translation_m: float = .15
    distinct_translation_m: float = .05
    distinct_rotation_deg: float = 2.
    comparable_rmse_ratio: float = 1.10
    comparable_rmse_addition_m: float = .005

    def __post_init__(self):
        for name,value in asdict(self).items():
            if isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value) or value<=0:
                raise ValueError('positive finite native-pair policy required: '+name)
        for name in ['max_points','max_iterations','minimum_correspondences','normal_neighbors']:
            if type(getattr(self,name)) is not int:
                raise ValueError('integer count required: '+name)
        if (not 0<self.range_min_m<self.range_max_m<=100 or self.max_points>5000
                or self.max_iterations>100 or self.normal_neighbors<3
                or self.normal_neighbors>self.max_points
                or self.minimum_correspondences>self.max_points
                or self.minimum_inlier_fraction>1 or self.normal_curvature_max>1):
            raise ValueError('native-pair policy exceeds bounded diagnostic domain')


def _points(value, policy):
    a=np.asarray(value)
    if a.ndim!=2 or a.shape[1]!=3 or a.dtype.kind not in 'fiu' or len(a)>100000:
        raise ValueError('bounded numeric Nx3 native points required')
    a=a.astype(float,copy=True)
    finite=np.isfinite(a).all(axis=1)
    b=a[finite];distance=np.linalg.norm(b,axis=1)
    keep=(distance>=policy.range_min_m)&(distance<=policy.range_max_m)
    b=b[keep]
    if len(b):
        cells=np.floor(b/policy.voxel_m).astype(np.int64)
        _,inverse,counts=np.unique(cells,axis=0,return_inverse=True,return_counts=True)
        c=np.column_stack([np.bincount(inverse,weights=b[:,i])/counts for i in range(3)])
    else:c=np.empty((0,3))
    voxel_rows=len(c)
    if len(c)>policy.max_points:
        c=c[np.linspace(0,len(c)-1,policy.max_points,dtype=int)]
    return c,{'input_rows':len(a),'nonfinite_rows_excluded':int((~finite).sum()),
              'range_rows_excluded':int((~keep).sum()),'voxel_centroids':voxel_rows,
              'points_used':len(c),'body_or_floor_mask_applied':False,
              'range_is_processing_bound_not_verified_sensor_blind_zone':True}


def _angle_deg(rotation):
    return math.degrees(math.acos(float(np.clip((np.trace(rotation)-1)/2,-1,1))))


def _quality(source,target,transform,policy):
    moved=source@transform[:3,:3].T+transform[:3,3]
    distance,index=cKDTree(target).query(moved,k=1)
    selected=distance<=policy.correspondence_limit_m
    count=int(selected.sum());fraction=count/len(source) if len(source) else 0.
    return {'correspondences':count,'inlier_fraction':fraction,
            'nearest_neighbor_rmse_m':float(np.sqrt(np.mean(distance[selected]**2))) if count else None,
            'nearest_neighbor_p95_m':float(np.quantile(distance[selected],.95)) if count else None},moved,index,selected


def _icp(source,target,initial,policy):
    transform=initial.copy();history=[];reasons=[]
    if min(len(source),len(target))<policy.minimum_correspondences:
        return {'transform':transform,'quality':None,'iterations':[],
                'reasons':['INSUFFICIENT_FINITE_RANGE_FILTERED_POINTS'],'converged':False}
    converged=False
    for _ in range(policy.max_iterations):
        q,moved,index,selected=_quality(source,target,transform,policy)
        history.append(q)
        if q['correspondences']<policy.minimum_correspondences or q['inlier_fraction']<policy.minimum_inlier_fraction:
            reasons.append('INSUFFICIENT_CORRESPONDENCE_OR_OVERLAP');break
        a=moved[selected];b=target[index[selected]]
        ac=a.mean(axis=0);bc=b.mean(axis=0)
        u,_,vt=np.linalg.svd((a-ac).T@(b-bc))
        rotation=vt.T@u.T
        if np.linalg.det(rotation)<0:
            vt[-1]*=-1;rotation=vt.T@u.T
        translation=bc-rotation@ac
        delta=np.eye(4);delta[:3,:3]=rotation;delta[:3,3]=translation
        transform=delta@transform
        if np.linalg.norm(translation)<1e-5 and _angle_deg(rotation)<.001:
            converged=True;break
    final,_,_,_=_quality(source,target,transform,policy)
    if final['correspondences']<policy.minimum_correspondences or final['inlier_fraction']<policy.minimum_inlier_fraction:
        if 'INSUFFICIENT_CORRESPONDENCE_OR_OVERLAP' not in reasons:reasons.append('INSUFFICIENT_CORRESPONDENCE_OR_OVERLAP')
    if not converged:reasons.append('ITERATION_LIMIT_OR_NO_CONVERGENCE')
    return {'transform':transform,'quality':final,'iterations':history,'reasons':reasons,'converged':converged}


def _information(source,target,transform,policy):
    if len(target)<policy.normal_neighbors or len(source)<policy.minimum_correspondences:
        return {'status':'INSUFFICIENT_NORMAL_SUPPORT','correspondences':0,
                'rank':None,'eigenvalues':None,'minimum_eigen_fraction':None}
    tree=cKDTree(target)
    distances,neighbors=tree.query(target,k=policy.normal_neighbors)
    patches=target[neighbors];centered=patches-patches.mean(axis=1,keepdims=True)
    vals,vecs=np.linalg.eigh(np.einsum('nki,nkj->nij',centered,centered)/policy.normal_neighbors)
    curvature=vals[:,0]/np.maximum(vals.sum(axis=1),1e-15)
    valid=(curvature<=policy.normal_curvature_max)&(distances[:,-1]<=policy.normal_radius_max_m)&(vals[:,1]>=1e-6)
    _,moved,index,selected=_quality(source,target,transform,policy)
    selected&=valid[index]
    count=int(selected.sum())
    if count<policy.minimum_correspondences:
        return {'status':'INSUFFICIENT_NORMAL_SUPPORT','correspondences':count,
                'rank':None,'eigenvalues':None,'minimum_eigen_fraction':None}
    p=moved[selected];n=vecs[index[selected],:,0]
    jacobian=np.column_stack([np.cross(p,n)/policy.rotation_length_scale_m,n])
    information=jacobian.T@jacobian/count
    eigenvalues,eigenvectors=np.linalg.eigh(information)
    eigenvalues=np.maximum(eigenvalues,0)
    maximum=float(eigenvalues[-1]);fraction=float(eigenvalues[0]/maximum) if maximum else 0.
    rank=int(np.linalg.matrix_rank(jacobian))
    return {'status':'GEOMETRIC_WEAKNESS_REVIEW' if rank<6 or fraction<policy.weak_information_eigen_fraction else 'NO_WEAKNESS_TRIGGER_IN_LOCAL_FIXED_CORRESPONDENCES_UNVALIDATED',
            'correspondences':count,'rank':rank,'eigenvalues':eigenvalues.tolist(),
            'minimum_eigen_fraction':fraction,'weakest_eigenvector':eigenvectors[:,0].tolist(),
            'rotation_length_scale_m':policy.rotation_length_scale_m,
            'parameter_order':['L_times_rotation_x','L_times_rotation_y','L_times_rotation_z','translation_x','translation_y','translation_z'],
            'interpretation':'LOCAL_POINT_TO_PLANE_LINEARIZATION_WITH_FIXED_NEAREST_NEIGHBORS_NOT_GLOBAL_POSE_UNIQUENESS'}


def analyze_native_pair(source_points,target_points,policy=None):
    """Return inspectable cloud-alignment candidates without accepting a pose."""
    policy=policy if isinstance(policy,NativePairPolicy) else NativePairPolicy(**(policy or {}))
    source,source_counts=_points(source_points,policy);target,target_counts=_points(target_points,policy)
    base=_icp(source,target,np.eye(4),policy)
    reverse=_icp(target,source,np.eye(4),policy)
    transform=base['transform'];cycle=reverse['transform']@transform
    def supported(fit):
        q=fit['quality']
        return q is not None and q['correspondences']>=policy.minimum_correspondences and q['inlier_fraction']>=policy.minimum_inlier_fraction
    forward_available=supported(base);reverse_available=supported(reverse)
    cycle_valid=base['quality'] is not None and reverse['quality'] is not None and not base['reasons'] and not reverse['reasons']
    info=_information(source,target,transform,policy)
    hypotheses=[];ambiguity=False
    for sign in [1.,-1.]:
        initial=np.eye(4);initial[0,3]=sign*policy.hypothesis_translation_m
        alternative=_icp(source,target,initial,policy)
        available=supported(alternative)
        difference=alternative['transform']@np.linalg.inv(transform)
        separation=float(np.linalg.norm(difference[:3,3])) if available and forward_available else None
        angle=_angle_deg(difference[:3,:3]) if available and forward_available else None
        q=alternative['quality'];b=base['quality']
        comparable=(q is not None and b is not None and q['nearest_neighbor_rmse_m'] is not None
            and b['nearest_neighbor_rmse_m'] is not None
            and q['correspondences']>=policy.minimum_correspondences
            and q['inlier_fraction']>=max(policy.minimum_inlier_fraction,b['inlier_fraction']-.05)
            and q['nearest_neighbor_rmse_m']<=b['nearest_neighbor_rmse_m']*policy.comparable_rmse_ratio+policy.comparable_rmse_addition_m)
        distinct=separation is not None and (separation>policy.distinct_translation_m or angle>policy.distinct_rotation_deg)
        ambiguity|=bool(comparable and distinct)
        hypotheses.append({'initial_native_x_translation_m':sign*policy.hypothesis_translation_m,
            'T_target_source_candidate':alternative['transform'].tolist() if available else None,'quality':q,'fit_reasons':alternative['reasons'],
            'separation_translation_m':separation,'separation_rotation_deg':angle,
            'comparable_fit':bool(comparable),'distinct_transform':bool(distinct)})
    flags=list(base['reasons'])
    if reverse['reasons']:flags.append('REVERSE_ALIGNMENT_REVIEW')
    if info['status']=='GEOMETRIC_WEAKNESS_REVIEW':flags.append('LOCAL_GEOMETRIC_WEAKNESS')
    elif info['status']=='INSUFFICIENT_NORMAL_SUPPORT':flags.append('NORMAL_SUPPORT_UNKNOWN')
    translation=float(np.linalg.norm(cycle[:3,3])) if cycle_valid else None
    rotation=_angle_deg(cycle[:3,:3]) if cycle_valid else None
    if cycle_valid and (translation>policy.cycle_translation_review_m or rotation>policy.cycle_rotation_review_deg):flags.append('BIDIRECTIONAL_INCONSISTENCY')
    if ambiguity:flags.append('COMPARABLE_FIT_DISTINCT_TRANSFORM_HYPOTHESES')
    return {'schema_version':1,'status':'NATIVE_CLOUD_CONSISTENCY_REVIEW_REQUIRED' if flags else 'NO_TRIGGER_IN_TESTED_NATIVE_CLOUD_HYPOTHESES_UNVALIDATED',
        'trust_status':'NOT_TRUSTED','policy':asdict(policy),'source_counts':source_counts,'target_counts':target_counts,
        'T_target_source_candidate':transform.tolist() if forward_available else None,'forward_quality':base['quality'],'forward_fit_reasons':base['reasons'],
        'T_source_target_reverse_candidate':reverse['transform'].tolist() if reverse_available else None,'reverse_quality':reverse['quality'],
        'cycle_translation_m':translation,'cycle_rotation_deg':rotation,'cycle_computed':cycle_valid,
        'local_information':info,'translation_hypotheses':hypotheses,'diagnostic_flags':flags,
        'thresholds_calibrated':False,'observed_cloud_alignment_is_vehicle_motion':False,
        'stationary_world_assumed':False,'ego_motion_truth_available':False,
        'body_mount_used':False,'floor_model_used':False,'imu_or_odometry_used':False,
        'source_points_modified':False,'authority_trajectory_written':False,
        'physical_navigation_validated':False,'control_topics_published':False}
