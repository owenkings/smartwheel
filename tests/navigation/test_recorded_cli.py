import math
import numpy as np

from wc_navigation.recorded_cli import axle_twist


def test_turning_reference_velocity_recovers_axle_without_lever_arm_error():
    fixed=np.eye(4);fixed[:2,3]=[.323,.22875]
    result=axle_twist([.885625,.1615,0.],[0.,0.,.5],fixed)
    assert result['status']=='PLANAR_MODEL_COMPATIBLE_UNVALIDATED'
    assert abs(result['forward_mps']-1.)<1e-12
    assert abs(result['lateral_mps'])<1e-12
    assert result['yaw_radps']==.5


def test_lateral_nonplanar_and_invalid_twist_are_not_zero_filled():
    fixed=np.eye(4)
    for linear,angular in [([1.,.2,0.],[0.,0.,0.]),([1.,0.,.2],[0.,0.,0.]),
                           ([1.,0.,0.],[.1,0.,0.]),([math.nan,0.,0.],[0.,0.,0.])]:
        result=axle_twist(linear,angular,fixed)
        assert result['status']=='UNKNOWN'
        assert result['forward_mps'] is None and result['yaw_radps'] is None
