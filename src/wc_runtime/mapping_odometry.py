"""Pure adapter for experimental wheel/IMU odometry, with no ROS startup.

The prior and authoritative odometry origins coincide by definition. This does
not supply a missing physical mounting transform or calibrate sensor noise.
"""
import copy
import math


AUTHORITY_FRAME = 'mapping_odom'
PRIOR_FRAME = 'prior_odom'
REFERENCE_FRAME = 'mapping_reference'
AUTHORITY_TOPIC = '/wc_mapping/app/odom'
COVARIANCE_STATUS = 'UNVALIDATED_MODEL_WEIGHTS'


def _valid_weight(value):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return False
    try:
        return math.isfinite(value) and 0 < value < 9999
    except OverflowError:
        return False

# RTAB-Map ROS 0.23.7-humble CoreWrapper.cpp L73, L1087, L1127-1145:
# both pose[0] and twist[0] >= 9999 imply an odometry reset; positive twist
# covariance is preferred, and x variance == 1 is ignored. The old diagnostic
# 1e6 matrices must therefore never be passed through as authority weights.
# https://github.com/introlab/rtabmap_ros/blob/0.23.7-humble/rtabmap_slam/src/CoreWrapper.cpp
# No defaults here: a wheel_imu session must archive explicitly chosen weights.
def validate_covariance(value):
    """Validate numerical model weights, not measured uncertainty or accuracy."""
    keys = {'status', 'pose_diagonal', 'twist_diagonal'}
    if not isinstance(value, dict) or set(value) != keys or value['status'] != COVARIANCE_STATUS:
        raise ValueError('wheel_imu_covariance requires explicit UNVALIDATED_MODEL_WEIGHTS and two diagonals')
    result = {'status': COVARIANCE_STATUS}
    for name in ('pose_diagonal', 'twist_diagonal'):
        diagonal = value[name]
        if not isinstance(diagonal, (list, tuple)) or len(diagonal) != 6 or any(
                not _valid_weight(v) for v in diagonal):
            raise ValueError(name+' must contain six finite positive weights below RTAB-Map reset sentinel 9999')
        if diagonal[0] == 1.0:
            raise ValueError(name+' x weight must differ from the RTAB-Map ignored value 1.0')
        result[name] = [float(v) for v in diagonal]
    return result


def _validate_prior(message):
    try:
        if message.header.frame_id != PRIOR_FRAME or message.child_frame_id != REFERENCE_FRAME:
            raise ValueError('expected prior_odom -> mapping_reference odometry')
        sec, ns = message.header.stamp.sec, message.header.stamp.nanosec
        if type(sec) is not int or type(ns) is not int or not 0 <= sec < 2**31 or \
                not 0 <= ns < 1_000_000_000 or sec*1_000_000_000+ns <= 0:
            raise ValueError('prior odometry requires a valid positive original ROS timestamp')
        p, q = message.pose.pose.position, message.pose.pose.orientation
        v, w = message.twist.twist.linear, message.twist.twist.angular
        numbers = [p.x, p.y, p.z, q.x, q.y, q.z, q.w, v.x, v.y, v.z, w.x, w.y, w.z]
        if any(isinstance(x, bool) or not isinstance(x, (int, float)) or not math.isfinite(x) for x in numbers):
            raise ValueError('prior pose and twist must be finite numeric values')
        if not math.isclose(sum(x*x for x in (q.x, q.y, q.z, q.w)), 1.0, abs_tol=1e-6, rel_tol=0):
            raise ValueError('prior odometry quaternion must already be unit length')
        for matrix in (message.pose.covariance, message.twist.covariance):
            if len(matrix) != 36 or any(not math.isfinite(x) for x in matrix):
                raise ValueError('prior diagnostic covariance must have 36 finite entries')
    except (AttributeError, TypeError, OverflowError) as error:
        raise ValueError('malformed prior Odometry message: '+str(error)) from error


def adapt_prior_odometry(message, covariance):
    """Return an independent message; original pose, body twist and stamp survive.

    Pose is T_prior_reference. Since mapping_odom == prior_odom by definition
    in wheel_imu mode, it is also T_mapping_odom_reference. Twist remains in
    the unchanged child mapping_reference frame. No interpolation or ICP runs.
    """
    weights = validate_covariance(covariance)
    _validate_prior(message)
    result = copy.deepcopy(message)
    result.header.frame_id = AUTHORITY_FRAME
    for field, name in ((result.pose, 'pose_diagonal'), (result.twist, 'twist_diagonal')):
        matrix = [0.0]*36
        for index, value in zip((0, 7, 14, 21, 28, 35), weights[name]):
            matrix[index] = value
        field.covariance = matrix
    return result


def authority_identity_transform(message, *, transform_type=None):
    """Identity mapping_odom -> prior_odom at the original scan/odom stamp.

    An injectable message constructor permits ROS-free tests. The real ROS
    message is imported only when this function is used in an adapter.
    """
    _validate_prior(message)
    if transform_type is None:
        from geometry_msgs.msg import TransformStamped
        transform_type = TransformStamped
    result = transform_type()
    result.header = copy.deepcopy(message.header)
    result.header.frame_id = AUTHORITY_FRAME
    result.child_frame_id = PRIOR_FRAME
    result.transform.translation.x = result.transform.translation.y = result.transform.translation.z = 0.0
    result.transform.rotation.x = result.transform.rotation.y = result.transform.rotation.z = 0.0
    result.transform.rotation.w = 1.0
    return result
