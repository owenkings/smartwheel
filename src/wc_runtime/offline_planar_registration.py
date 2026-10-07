"""Offline bounded SE(2) registration of already-FLU scan clouds.

T_target_source maps source coordinates into target coordinates. This module
never opens hardware, changes a calibration or forces a room shape/loop closure.
Thresholds are frozen experimental availability gates, not calibrated accuracy.
"""
from collections import deque
from dataclasses import dataclass, asdict
import math
import numpy as np
from scipy.spatial import cKDTree


@dataclass(frozen=True)
class Policy:
    voxel_m: float = .08
    min_range_m: float = .35
    max_range_m: float = 8.
    min_height_m: float = -.50
    max_height_m: float = .50
    max_source_points: int = 4000
    max_target_points: int = 10000
    normal_neighbors: int = 14
    normal_radius_m: float = .35
    min_normal_neighbors: int = 6
    max_normal_curvature: float = .10
    min_horizontal_normal_length: float = .70
    min_normal_spread_m2: float = .002
    stable_patch_grid_m: float = .40
    stable_patch_radius_m: float = .75
    stable_patch_neighbors: int = 128
    stable_patch_min_points: int = 32
    stable_patch_min_spread_m2: float = .008
    stable_patch_max_rmse_m: float = .05
    stable_patch_min_matches: int = 16
    stable_patch_min_count: int = 6
    correspondence_distances_m: tuple = (.40, .25, .15)
    iterations_per_scale: int = 12
    huber_m: float = .06
    min_correspondences: int = 80
    min_source_overlap: float = .35
    min_normal_eigen_fraction: float = .005
    max_information_condition: float = 1000.
    rotation_length_scale_m: float = 2.
    max_correction_translation_m: float = .35
    max_correction_yaw_deg: float = 8.
    max_final_rmse_m: float = .06
    max_final_p95_m: float = .10
    max_iteration_translation_m: float = .10
    max_iteration_yaw_deg: float = 2.
    keyframe_translation_m: float = .15
    keyframe_rotation_deg: float = 4.
    keyframe_interval_s: float = 1.
    submap_keyframes: int = 8
    submap_max_age_s: float = 8.
    reset_after_rejection_s: float = 2.
    reset_after_input_gap_s: float = 1.

    def __post_init__(self):
        for k,v in asdict(self).items():
            if k in ('min_height_m', 'max_height_m', 'correspondence_distances_m'): continue
            if isinstance(v,bool) or not isinstance(v,(int,float)) or not math.isfinite(v) or v<=0:
                raise ValueError('positive finite policy required: '+k)
        counts=('max_source_points','max_target_points','normal_neighbors','min_normal_neighbors',
                'min_correspondences','iterations_per_scale','submap_keyframes',
                'stable_patch_neighbors','stable_patch_min_points','stable_patch_min_matches','stable_patch_min_count')
        if any(type(getattr(self,k)) is not int for k in counts): raise ValueError('integer policy counts required')
        if not math.isfinite(self.min_height_m) or not math.isfinite(self.max_height_m) or self.min_height_m>=self.max_height_m:
            raise ValueError('invalid height interval')
        if self.min_range_m>=self.max_range_m or not 0<self.min_source_overlap<=1: raise ValueError('invalid range/overlap')
        if self.min_normal_neighbors<3 or self.normal_neighbors<self.min_normal_neighbors: raise ValueError('invalid normal neighborhood')
        if not 0<self.max_normal_curvature<.5 or not 0<self.min_normal_eigen_fraction<1 or not 0<self.min_horizontal_normal_length<=1: raise ValueError('invalid observability gate')
        values=self.correspondence_distances_m
        if len(values)<1 or any(not isinstance(v,(int,float)) or not math.isfinite(v) or v<=0 for v in values):
            raise ValueError('invalid correspondence distances')
        if any(a<b for a,b in zip(values,values[1:])): raise ValueError('coarse-to-fine distances required')


def rigid(value):
    T=np.asarray(value,dtype=float)
    if T.shape!=(4,4) or not np.isfinite(T).all() or not np.allclose(T[3],[0,0,0,1],atol=1e-9):
        raise ValueError('finite homogeneous transform required')
    if not np.allclose(T[:3,:3].T@T[:3,:3],np.eye(3),atol=1e-7) or not np.isclose(np.linalg.det(T[:3,:3]),1.,atol=1e-7):
        raise ValueError('proper rotation required, reflection is not an extrinsic')
    if not np.allclose(T[2],[0,0,1,0],atol=1e-7) or not np.allclose(T[:2,2],0,atol=1e-7):
        raise ValueError('SE2 transform required; nonplanar motion must use a different experiment')
    return T.copy()


def transform_xy(T,points): return points@T[:2,:2].T+T[:2,3]

def transform_points(T,points): return points@T[:3,:3].T+T[:3,3]

def se2(x,y,yaw):
    c,s=math.cos(yaw),math.sin(yaw)
    T=np.eye(4);T[:2,:2]=[[c,-s],[s,c]];T[:2,3]=[x,y];return T

def yaw(T): return math.atan2(T[1,0],T[0,0])

def rotation_difference_deg(A,B): return abs(math.degrees(math.atan2(math.sin(yaw(A)-yaw(B)),math.cos(yaw(A)-yaw(B)))))


def _voxel_xy(xy,voxel,limit):
    if not len(xy):return np.empty((0,xy.shape[1]))
    # Sort by voxel key, average each cell, then spatially spread a bounded cap.
    keys=np.floor(xy/voxel).astype(np.int64)
    _,inv,counts=np.unique(keys,axis=0,return_inverse=True,return_counts=True)
    result=np.column_stack([np.bincount(inv,weights=xy[:,i])/counts for i in range(xy.shape[1])])
    if len(result)>limit:result=result[np.linspace(0,len(result)-1,limit,dtype=int)]
    return result


def preprocess(points,policy=None,*,target=False,preprocessed=False):
    policy=policy if isinstance(policy,Policy) else Policy(**(policy or {}))
    a=np.asarray(points,dtype=float)
    if a.ndim!=2 or a.shape[1] not in (2,3):raise ValueError('Nx2 or Nx3 points required')
    if len(a)>200000:raise ValueError('input point budget exceeded; bound local submap before registration')
    a=a[np.isfinite(a).all(axis=1)]
    if a.shape[1]==2:a=np.column_stack([a,np.zeros(len(a))])
    if not preprocessed:
        distance=np.linalg.norm(a[:,:2],axis=1)
        mask=(distance>=policy.min_range_m)&(distance<=policy.max_range_m)
        if a.shape[1]==3:mask&=(a[:,2]>=policy.min_height_m)&(a[:,2]<=policy.max_height_m)
        a=a[mask]
    return _voxel_xy(a,policy.voxel_m,policy.max_target_points if target else policy.max_source_points)


def _normal_model(target,p):
    tree=cKDTree(target)
    distances,indices=tree.query(target,k=min(p.normal_neighbors,len(target)),workers=1)
    mask=distances<=p.normal_radius_m
    patches=target[indices]
    count=mask.sum(axis=1)
    mean=np.sum(patches*mask[:,:,None],axis=1)/np.maximum(count[:,None],1)
    centered=(patches-mean[:,None,:])*mask[:,:,None]
    covariance=np.einsum('nki,nkj->nij',centered,centered)/np.maximum(count[:,None,None],1)
    eigen,vectors=np.linalg.eigh(covariance)
    normals=vectors[:,:,0]
    curvature=eigen[:,0]/np.maximum(eigen.sum(axis=1),1e-15)
    valid=(count>=p.min_normal_neighbors)&(eigen[:,1]>=p.min_normal_spread_m2)&(curvature<=p.max_normal_curvature)
    valid&=np.linalg.norm(normals[:,:2],axis=1)>=p.min_horizontal_normal_length
    return tree,normals,valid


def _stable_patch_model(target,p):
    # A separate, wider spatial support model guards against pointwise noisy
    # normals manufacturing a fictitious observable direction in a corridor.
    # Equal patch weight prevents dense points or repeated correspondences from
    # creating extra information. It does not impose any preferred wall angle.
    centers=_voxel_xy(target,p.stable_patch_grid_m,1000)
    tree=cKDTree(target)
    distances,indices=tree.query(centers,k=min(p.stable_patch_neighbors,len(target)),workers=1)
    valid=distances<=p.stable_patch_radius_m
    patches=target[indices];count=valid.sum(axis=1)
    mean=np.sum(patches*valid[:,:,None],axis=1)/np.maximum(count[:,None],1)
    centered=(patches-mean[:,None,:])*valid[:,:,None]
    covariance=np.einsum('nki,nkj->nij',centered,centered)/np.maximum(count[:,None,None],1)
    eigen,vectors=np.linalg.eigh(covariance);normal=vectors[:,:,0]
    rms=np.sqrt(np.maximum(eigen[:,0],0))
    eligible=(count>=p.stable_patch_min_points)&(eigen[:,1]>=p.stable_patch_min_spread_m2)
    eligible&=(eigen[:,0]/np.maximum(eigen.sum(axis=1),1e-15)<=p.max_normal_curvature)
    eligible&=(rms<=p.stable_patch_max_rmse_m)&(np.linalg.norm(normal[:,:2],axis=1)>=p.min_horizontal_normal_length)
    return {'indices':indices[eligible], 'valid':valid[eligible], 'normals':normal[eligible],
            'centers':mean[eligible], 'rmse':rms[eligible], 'count':count[eligible],
            'candidate_patch_count':len(centers),'target_points':len(target)}


def _stable_patch_quality(nearest,model,p):
    matched=np.zeros(model['target_points'],dtype=bool);matched[nearest]=True
    count=np.sum(matched[model['indices']]&model['valid'],axis=1)
    selected=count>=p.stable_patch_min_matches
    centers=model['centers'][selected];n=model['normals'][selected]
    A=np.column_stack([n[:,0],n[:,1],(-n[:,0]*centers[:,1]+n[:,1]*centers[:,0])/p.rotation_length_scale_m])
    H=A.T@A/max(1,len(A));eigen=np.linalg.eigvalsh(H)
    fraction=float(eigen[0]/max(eigen[-1],1e-30))
    return {'candidate_patch_count':model['candidate_patch_count'],
       'stable_patch_count':len(model['centers']),'matched_stable_patch_count':len(A),
       'matched_patch_rmse_p95_m':float(np.quantile(model['rmse'][selected],.95)) if len(A) else None,
       'minimum_eigen_fraction':fraction,
       'information_eigenvalues_normalized':eigen.tolist(),
       'information_condition':float(eigen[-1]/max(eigen[0],1e-30)),
       'weighting':'Equal spatial patch weight; unique target membership, no preferred room angles'}


def _correspondences(source,target,T,tree,normals,normal_valid,limit,p,patch_model=None):
    moved=transform_points(T,source);distance,nearest=tree.query(moved,k=1,workers=1)
    good=(distance<=limit)&normal_valid[nearest]
    a,b,n=moved[good],target[nearest[good]],normals[nearest[good]]
    residual=np.einsum('ij,ij->i',n,a-b)
    A=np.column_stack([n[:,0],n[:,1],-n[:,0]*a[:,1]+n[:,1]*a[:,0]])
    weights=np.minimum(1.,p.huber_m/np.maximum(np.abs(residual),1e-15))
    scaled=A.copy();scaled[:,2]/=p.rotation_length_scale_m
    H=scaled.T@(weights[:,None]*scaled)/max(1,len(a))
    eigen=np.linalg.eigvalsh(H)
    minimum=float(eigen[0]/max(eigen[-1],1e-30));condition=float(eigen[-1]/max(eigen[0],1e-30))
    quality={'correspondences':len(a),'source_overlap':len(a)/len(source),
      'rmse_m':float(np.sqrt(np.mean(residual**2))) if len(a) else None,
      'absolute_residual_p95_m':float(np.quantile(np.abs(residual),.95)) if len(a) else None,
      'nearest_distance_p95_m':float(np.quantile(distance[good],.95)) if len(a) else None,
      'information_eigenvalues_normalized':eigen.tolist(),'minimum_eigen_fraction':minimum,
      'information_condition':condition,'observable_degrees_of_freedom':['x','y','yaw'],
      'rotation_length_scale_m':p.rotation_length_scale_m,
      'effective_robust_weight':float(weights.sum()),
      'unique_target_correspondences':int(len(np.unique(nearest[good]))),
      'unique_target_per_correspondence':float(len(np.unique(nearest[good]))/max(len(a),1)),
      'source_overlap_meaning':'fraction of prepared source points with accepted nearest target plane; not semantic physical overlap'}
    if patch_model is not None:quality['stable_patch_observability']=_stable_patch_quality(nearest[good],patch_model,p)
    return A,residual,weights,quality


def _quality_reasons(q,p,*,final=False):
    reasons=[]
    if q['correspondences']<p.min_correspondences:reasons.append('INSUFFICIENT_CORRESPONDENCES')
    if q['source_overlap']<p.min_source_overlap:reasons.append('INSUFFICIENT_SOURCE_OVERLAP')
    if q['minimum_eigen_fraction']<p.min_normal_eigen_fraction or q['information_condition']>p.max_information_condition:
        reasons.append('SE2_GEOMETRIC_DEGENERACY')
    if 'stable_patch_observability' in q:
        patch=q['stable_patch_observability']
        if patch['matched_stable_patch_count']<p.stable_patch_min_count:reasons.append('INSUFFICIENT_STABLE_PLANE_PATCHES')
        elif patch['minimum_eigen_fraction']<p.min_normal_eigen_fraction or patch['information_condition']>p.max_information_condition:
            reasons.append('STABLE_PATCH_SE2_DEGENERACY')
    if final and (q['rmse_m'] is None or q['rmse_m']>p.max_final_rmse_m):reasons.append('FINAL_RESIDUAL_RMSE')
    if final and (q['absolute_residual_p95_m'] is None or q['absolute_residual_p95_m']>p.max_final_p95_m):reasons.append('FINAL_RESIDUAL_P95')
    return reasons


def align(source,target,initial_T,policy=None,*,preprocessed=False):
    """Return T_target_source; rejected result always returns initial_T unchanged.

    Internal candidate is retained separately for diagnosis. Coordinates are
    physical FLU metres; no sensor-axis reflection is performed here.
    """
    p=policy if isinstance(policy,Policy) else Policy(**(policy or {}));initial=rigid(initial_T)
    source=preprocess(source,p,preprocessed=preprocessed)
    target=preprocess(target,p,target=True,preprocessed=preprocessed)
    prepared_source_before_support=len(source)
    source_support_valid_count=0
    if len(source)>=p.min_normal_neighbors:
        _,_,source_valid=_normal_model(source,p)
        source_support_valid_count=int(source_valid.sum())
        source=source[source_valid]
    prepared_source_count=len(source)
    held_out=(np.arange(len(source))%5)==0
    validation_source=source[held_out].copy()
    source=source[~held_out]
    report={'schema_version':1,'mode':'OFFLINE_PLANAR_GEOMETRIC_CORRECTION','accepted':False,
      'threshold_status':'EXPERIMENTAL_FROZEN_POLICY_V1_GATES_WITH_XYZ_SYMMETRIC_PLANES_AND_STABLE_PATCH_OBSERVABILITY_V5',
      'registration_geometry':'SE2_POINT_TO_PLANE_SYMMETRIC_PLANAR_SUPPORT','policy':asdict(p),
      'source_points':len(source),'source_points_before_holdout':prepared_source_count,
      'source_points_before_planar_support':prepared_source_before_support,
      'source_planar_support_points':source_support_valid_count,
      'source_support_definition':'Same XYZ local-plane quality and horizontal-normal gate as target; overlap denominator is eligible geometric support',
      'validation_source_points':len(validation_source),'target_points':len(target),'initial_T':initial.tolist(),
      'source_holdout_policy':'Every fifth lexicographically ordered prepared voxel excluded from optimizer; same scan correlated diagnostic, not independent pose truth',
      'T_target_source':initial.tolist(),'candidate_T':initial.tolist(),'iterations':[],
      'rejection_reasons':[],'correction_translation_m':0.,'correction_yaw_deg':0.}
    if min(len(source),len(target))<p.min_correspondences:
        report['rejection_reasons']=['INSUFFICIENT_FILTERED_POINTS'];return report
    tree,normals,valid=_normal_model(target,p);report['target_normal_valid_count']=int(valid.sum())
    target=target[valid];normals=normals[valid];valid=np.ones(len(target),dtype=bool)
    if len(target)<p.min_correspondences:
        report['rejection_reasons']=['INSUFFICIENT_TARGET_PLANAR_SUPPORT'];return report
    tree=cKDTree(target);patch_model=_stable_patch_model(target,p)
    _,_,_,validation_initial=_correspondences(validation_source,target,initial,tree,normals,valid,p.correspondence_distances_m[-1],p,patch_model)
    T=initial.copy();failed=[];last=None
    for limit in p.correspondence_distances_m:
        for iteration in range(p.iterations_per_scale):
            A,residual,w,q=_correspondences(source,target,T,tree,normals,valid,limit,p,patch_model)
            report['iterations'].append(dict(scale_distance_m=limit,iteration=iteration,**q));last=q
            failed=_quality_reasons(q,p)
            if failed:break
            step=np.linalg.lstsq(A*np.sqrt(w[:,None]),-residual*np.sqrt(w),rcond=None)[0]
            norm=float(np.linalg.norm(step[:2]));angle=abs(step[2]);factor=min(1.,p.max_iteration_translation_m/max(norm,1e-15),math.radians(p.max_iteration_yaw_deg)/max(angle,1e-15))
            step*=factor;T=se2(*step)@T
            if np.linalg.norm(step[:2])<1e-5 and abs(step[2])<1e-5:break
        if failed:break
    _,_,_,final=_correspondences(source,target,T,tree,normals,valid,p.correspondence_distances_m[-1],p,patch_model)
    reasons=list(dict.fromkeys(failed+_quality_reasons(final,p,final=True)))
    correction=T@np.linalg.inv(initial)
    translation=float(np.linalg.norm(T[:2,3]-initial[:2,3]))
    rotation=rotation_difference_deg(T,initial)
    if translation>p.max_correction_translation_m:reasons.append('EXCESSIVE_PRIOR_TRANSLATION_CORRECTION')
    if rotation>p.max_correction_yaw_deg:reasons.append('EXCESSIVE_PRIOR_YAW_CORRECTION')
    _,_,_,validation_candidate=_correspondences(validation_source,target,T,tree,normals,valid,p.correspondence_distances_m[-1],p,patch_model)
    report['heldout_validation']={'not_used_in_optimizer':True,'initial':validation_initial,'candidate':validation_candidate,
      'applied':validation_candidate if not reasons else validation_initial,'is_independent_trajectory_truth':False}
    report.update(candidate_T=T.tolist(),final_quality=final,rejection_reasons=reasons,
      correction_translation_m=translation,correction_yaw_deg=rotation,
      correction_matrix=correction.tolist(),accepted=not reasons)
    if not reasons:report['T_target_source']=T.tolist()
    return report


class TrajectoryRefiner:
    """Local registration with bounded recent keyframes, prior fallback and segments.

    update(stamp_ns, points, prior_T_world_scan) -> result. Input scans already
    contain the caller's declared within-pair compensation; no sensor is read.
    No history or optimized trajectory is silently rewritten.
    """
    def __init__(self,policy=None):
        self.policy=policy if isinstance(policy,Policy) else Policy(**(policy or {}))
        self.history=deque(maxlen=self.policy.submap_keyframes)
        self.last_stamp=None;self.last_prior=None;self.last_pose=None;self.last_accept_stamp=None
        self.segment=0;self.rejections=0;self.accepted=0;self.count=0

    def _seed(self,stamp,xy,prior,pose,reason):
        self.history.clear();self.history.append((stamp,xy.copy(),pose.copy()))
        self.last_accept_stamp=stamp
        return {'accepted':False,'status':reason,'rejection_reasons':[],
          'T_world_scan':pose.tolist(),'prior_T_world_scan':prior.tolist(),'segment_id':self.segment,
          'segment_start':True,'segment_start_reason':reason,'geometry_constraint_added':False}

    def update(self,stamp_ns,points,prior_T_world_scan):
        if type(stamp_ns) is not int or stamp_ns<0 or (self.last_stamp is not None and stamp_ns<=self.last_stamp):
            raise ValueError('strictly increasing integer nanosecond stamps required')
        p=self.policy;prior=rigid(prior_T_world_scan);xy=preprocess(points,p)
        if self.last_pose is None:
            predicted=prior.copy()
        else:
            predicted=self.last_pose@np.linalg.inv(self.last_prior)@prior
        reset=self.last_stamp is not None and (stamp_ns-self.last_stamp)*1e-9>p.reset_after_input_gap_s
        if self.last_stamp is None or reset or not self.history:
            if self.last_stamp is not None:self.segment+=1
            result=self._seed(stamp_ns,xy,prior,predicted,'INITIAL_REFERENCE' if self.last_stamp is None else 'INPUT_GAP_NEW_SEGMENT')
        else:
            while len(self.history)>1 and (stamp_ns-self.history[0][0])*1e-9>p.submap_max_age_s:self.history.popleft()
            anchor=self.last_pose;inv_anchor=np.linalg.inv(anchor)
            target=np.concatenate([transform_points(inv_anchor@T,a) for _,a,T in self.history])
            registration=align(xy,target,inv_anchor@predicted,p,preprocessed=True)
            pose=anchor@np.asarray(registration['T_target_source'])
            result={'accepted':registration['accepted'],'status':'REGISTERED' if registration['accepted'] else 'PRIOR_FALLBACK',
              'rejection_reasons':registration['rejection_reasons'],'T_world_scan':pose.tolist(),
              'prior_T_world_scan':prior.tolist(),'predicted_T_world_scan':predicted.tolist(),
              'segment_id':self.segment,'segment_start':False,'geometry_constraint_added':registration['accepted'],
              'registration':registration,'submap_keyframes':len(self.history)}
            if registration['accepted']:
                self.accepted+=1;self.last_accept_stamp=stamp_ns
                ts,_,last=self.history[-1]
                if np.linalg.norm(pose[:2,3]-last[:2,3])>=p.keyframe_translation_m or rotation_difference_deg(pose,last)>=p.keyframe_rotation_deg or (stamp_ns-ts)*1e-9>=p.keyframe_interval_s:
                    self.history.append((stamp_ns,xy.copy(),pose.copy()))
            else:
                self.rejections+=1
                if (stamp_ns-self.last_accept_stamp)*1e-9>=p.reset_after_rejection_s:
                    self.segment+=1
                    previous_reasons=list(result['rejection_reasons'])
                    previous_registration=result['registration']
                    result=self._seed(stamp_ns,xy,prior,predicted,'GEOMETRY_REJECTION_NEW_SEGMENT')
                    result.update(rejection_reasons=previous_reasons,registration=previous_registration)
        self.last_stamp=stamp_ns;self.last_prior=prior.copy();self.last_pose=rigid(result['T_world_scan']);self.count+=1
        result.update(stamp_ns=stamp_ns,source_points_after_preparation=len(xy),prior_fallback_preserves_increment=True,
                      loop_closure_applied=False,room_shape_constraint_applied=False)
        return result

    def summary(self):
        return {'frames':self.count,'accepted_registrations':self.accepted,'rejected_registrations':self.rejections,
          'segments':self.segment+int(self.count>0),'policy':asdict(self.policy),
          'physical_accuracy_validated':False,'room_shape_constraint_applied':False,'loop_closure_applied':False}


def refine_trajectory(frames,policy=None):
    """Yield per-frame reports for iterable dicts: stamp_ns, points, prior_T.

    This streaming API has bounded geometry memory. The caller writes reports,
    binds source hashes and decides whether a separate output is publishable.
    """
    engine=TrajectoryRefiner(policy)
    for frame in frames:yield engine.update(frame['stamp_ns'],frame['points'],frame['prior_T'])
