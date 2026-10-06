"""Offline point-to-plane ICP diagnostics. Never writes an authority trajectory."""
from dataclasses import asdict, dataclass
import math
import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

@dataclass(frozen=True)
class ICPPolicy:
    max_points: int = 5000
    normal_neighbors: int = 15
    max_iterations: int = 20
    max_correspondence_m: float = .30
    min_correspondences: int = 80
    min_inlier_ratio: float = .30
    max_condition: float = 1e8
    min_information_eigenvalue: float = 1e-6
    max_correction_translation_m: float = .25
    max_correction_rotation_deg: float = 10.
    max_residual_m: float = .05
    max_normal_curvature: float = .10

    def __post_init__(self):
        for name,value in asdict(self).items():
            if isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value) or value<=0:
                raise ValueError('positive finite ICP policy required: '+name)
        for name in ('max_points','normal_neighbors','max_iterations','min_correspondences'):
            if type(getattr(self,name)) is not int: raise ValueError('integer ICP count required: '+name)
        if self.normal_neighbors<3 or self.min_inlier_ratio>1: raise ValueError('invalid ICP neighborhood/ratio')

def _points(value,limit):
    points=np.asarray(value,dtype=float)
    if points.ndim!=2 or points.shape[1]!=3: raise ValueError('Nx3 points required')
    points=points[np.isfinite(points).all(axis=1)]
    if len(points)>limit: points=points[np.linspace(0,len(points)-1,limit,dtype=int)]
    return points

def point_to_plane(source,target,T_target_source_initial,policy=None):
    policy=policy if isinstance(policy,ICPPolicy) else ICPPolicy(**(policy or {}))
    initial=np.asarray(T_target_source_initial,dtype=float)
    if initial.shape!=(4,4) or not np.isfinite(initial).all() or not np.allclose(initial[3],[0,0,0,1]) or \
            not np.allclose(initial[:3,:3].T@initial[:3,:3],np.eye(3),atol=1e-6) or np.linalg.det(initial[:3,:3])<.999999:
        raise ValueError('proper explicit T_target_source_initial required')
    source,target=_points(source,policy.max_points),_points(target,policy.max_points)
    report={'schema_version':1,'mode':'SHADOW_ONLY','trajectory_changed':False,
            'policy':asdict(policy),'threshold_status':'EXPERIMENTAL_NOT_CALIBRATED',
            'source_points':len(source),'target_points':len(target),'iterations':[],
            'T_target_source_initial':initial.tolist(),'accepted':False,'rejection_reasons':[]}
    if min(len(source),len(target))<max(policy.min_correspondences,policy.normal_neighbors):
        report['rejection_reasons']=['INSUFFICIENT_FINITE_POINTS']; return report
    tree=cKDTree(target)
    neighbors=tree.query(target,k=policy.normal_neighbors)[1]
    patches=target[neighbors]; centered=patches-patches.mean(axis=1,keepdims=True)
    eigenvalues,eigenvectors=np.linalg.eigh(np.einsum('nki,nkj->nij',centered,centered)/policy.normal_neighbors)
    curvature=eigenvalues[:,0]/np.maximum(eigenvalues.sum(axis=1),1e-15)
    normal_valid=curvature<=policy.max_normal_curvature
    normals=eigenvectors[:,:,0]
    transform=initial.copy(); final=None
    for iteration in range(policy.max_iterations):
        moved=source@transform[:3,:3].T+transform[:3,3]
        distance,index=tree.query(moved,k=1)
        selected=(distance<=policy.max_correspondence_m)&normal_valid[index]
        count=int(selected.sum()); ratio=count/len(source)
        if count<policy.min_correspondences or ratio<policy.min_inlier_ratio:
            report['rejection_reasons'].append('INSUFFICIENT_CORRESPONDENCE_OR_OVERLAP'); break
        p,q,n=moved[selected],target[index[selected]],normals[index[selected]]
        residual=np.einsum('ij,ij->i',n,p-q)
        A=np.column_stack((np.cross(p,n),n))
        information=A.T@A
        values=np.linalg.eigvalsh(information)
        rank=int(np.linalg.matrix_rank(A)); condition=float(values[-1]/max(values[0],1e-30))
        row={'iteration':iteration,'correspondences':count,'inlier_ratio':ratio,
             'point_to_plane_rmse_m':float(np.sqrt(np.mean(residual**2))),
             'information_eigenvalues':values.tolist(),'rank':rank,'condition':condition,
             'nearest_neighbor_p95_m':float(np.quantile(distance[selected],.95))}
        report['iterations'].append(row); final=row
        if rank<6 or values[0]<policy.min_information_eigenvalue or condition>policy.max_condition:
            report['rejection_reasons'].append('GEOMETRIC_DEGENERACY'); break
        correction=np.linalg.lstsq(A,-residual,rcond=None)[0]
        delta=np.eye(4); delta[:3,:3]=Rotation.from_rotvec(correction[:3]).as_matrix(); delta[:3,3]=correction[3:]
        transform=delta@transform
        if np.linalg.norm(correction)<1e-7: break
    difference=transform@np.linalg.inv(initial)
    translation=float(np.linalg.norm(difference[:3,3])); angle=float(np.rad2deg(Rotation.from_matrix(difference[:3,:3]).magnitude()))
    if translation>policy.max_correction_translation_m: report['rejection_reasons'].append('EXCESSIVE_PRIOR_TRANSLATION_CORRECTION')
    if angle>policy.max_correction_rotation_deg: report['rejection_reasons'].append('EXCESSIVE_PRIOR_ROTATION_CORRECTION')
    if final is None or final['point_to_plane_rmse_m']>policy.max_residual_m: report['rejection_reasons'].append('RESIDUAL_LIMIT')
    report.update(T_target_source_shadow=transform.tolist(),correction_translation_m=translation,
                  correction_rotation_deg=angle,final_quality=final,
                  accepted=not report['rejection_reasons'])
    return report
