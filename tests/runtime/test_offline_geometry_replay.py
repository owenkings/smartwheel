import math
import numpy as np
from scipy.spatial.transform import Rotation
from wc_runtime.offline_geometry_replay import correction_at,redeskew_points

def T(x=0,y=0,yaw=0):
    a=np.eye(4);a[:3,:3]=Rotation.from_euler('z',yaw).as_matrix();a[:2,3]=[x,y];return a

def apply(t,p):return np.array(p)@t[:3,:3].T+t[:3,3]

def test_identity_correction_never_changes_compensated_cloud():
    points=np.array([[1.,2.,3.],[2.,1.,0.],[np.nan,2,3]])
    pose=T(3,-2,.5);stamps=np.array([100,200],dtype=np.int64)
    out,delta=redeskew_points(points,np.array([-10,0,0]),200,pose,pose,stamps,[np.eye(4),np.eye(4)])
    np.testing.assert_allclose(out,points,atol=1e-14);assert delta<1e-14

def test_redeskew_matches_reprojection_from_original_two_source_poses():
    stamps=np.array([1000,2000],dtype=np.int64);cs=[T(.2,.1,.3),T(.5,-.3,.7)]
    oldref=T(1,2,.6);oldsrc=T(.8,1.8,.55);newref=cs[-1]@oldref
    source_points=np.array([[1,2,3],[2,1,0]],dtype=float)
    oldcloud=apply(np.linalg.inv(oldref)@oldsrc,source_points)
    corrected,_=redeskew_points(oldcloud,np.array([-100,-100]),2000,oldref,newref,stamps,cs)
    expected=apply(np.linalg.inv(newref)@correction_at(stamps,cs,1900)@oldsrc,source_points)
    np.testing.assert_allclose(corrected,expected,atol=1e-14)

def test_shortest_yaw_and_boundary_hold():
    stamps=np.array([10,20],dtype=np.int64);cs=[T(yaw=math.radians(179)),T(yaw=math.radians(-179))]
    middle=correction_at(stamps,cs,15)
    assert abs(abs(math.atan2(middle[1,0],middle[0,0]))-math.pi)<1e-14
    np.testing.assert_array_equal(correction_at(stamps,cs,0),cs[0])
    np.testing.assert_array_equal(correction_at(stamps,cs,30),cs[-1])

def test_final_reference_side_is_unchanged_by_redeskew():
    stamps=np.array([1,2],dtype=np.int64);cs=[T(),T(1,2,.8)];old=T(.4,.2,.3)
    points=np.array([[1.,0,2.]])
    result,delta=redeskew_points(points,np.array([0]),2,old,cs[-1]@old,stamps,cs)
    np.testing.assert_allclose(result,points,atol=1e-14)
