"""Synthetic geometry checks for the offline delay score; no ROS or devices."""
from pathlib import Path
import sys

import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'integration'))
from check_mapping_time_ab_20260915 import normal_model, score_pair


def corner():
    u,v=np.meshgrid(np.linspace(-1,1,30),np.linspace(-1,1,30))
    return np.vstack([np.c_[u.ravel(),v.ravel(),np.zeros(u.size)],
                      np.c_[np.ones(u.size),u.ravel(),v.ravel()]])


def test_rejected_correspondences_cannot_fake_a_better_fit():
    cloud=corner();tree,normals,planar=normal_model(cloud)
    match=score_pair(cloud,cloud,tree,normals,planar,np.eye(4))
    wrong=np.eye(4);wrong[0,3]=10
    mismatch=score_pair(cloud,cloud,tree,normals,planar,wrong)
    assert match['accepted_ratio']>.7
    assert mismatch['accepted_count']==0 and mismatch['inlier_p50_m'] is None
    assert mismatch['penalized_rmse_m']==.25 and mismatch['penalized_rmse_m']>match['penalized_rmse_m']


def test_known_accelerating_rotation_prefers_true_delay_over_zero():
    world=corner()
    def pose(t):
        result=np.eye(4);result[:3,:3]=Rotation.from_euler('z',.8*t*t).as_matrix()
        result[:3,3]=[.1*t*t,0,0]
        return result
    a,b,delay=.4,.65,.12
    pa,pb=pose(a+delay),pose(b+delay)
    target=(world-pa[:3,3])@pa[:3,:3]
    source=(world-pb[:3,3])@pb[:3,:3]
    tree,normals,planar=normal_model(target)
    scores=[]
    for offset in [0.,delay,-delay]:
        relative=np.linalg.inv(pose(a+offset))@pose(b+offset)
        scores.append(score_pair(source,target,tree,normals,planar,relative))
    assert scores[1]['inlier_p95_m']<1e-12
    assert scores[0]['inlier_p95_m']>.001 and scores[2]['inlier_p95_m']>.001
    assert scores[1]['penalized_rmse_m']<scores[0]['penalized_rmse_m']
