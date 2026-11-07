"""Small ROS helpers shared by the autonomy nodes (imports ROS lazily)."""
from __future__ import annotations

import json
import math
import os


# 96 is the default. 97-101 let parallel runs, including separate development
# checkouts (git worktrees), each use their own localhost domain. Domain locks
# live under each checkout, so parallel checkouts must be given distinct domains.
SIMULATION_DOMAINS = ("96", "97", "98", "99", "100", "101")


def simulation_domain():
    """Dedicated domain of this run: 96 by default, 97-101 for parallel runs."""
    domain = os.environ.get("WC_AUTONOMY_ROS_DOMAIN", "96")
    if domain not in SIMULATION_DOMAINS:
        raise RuntimeError("WC_AUTONOMY_ROS_DOMAIN must be one of " + ", ".join(SIMULATION_DOMAINS))
    return domain


def require_isolated_domain(expected=None):
    """Refuse to start outside the dedicated localhost-only simulation domain."""
    expected = expected or simulation_domain()
    if os.name != "posix":
        raise RuntimeError("autonomy simulation requires the supported Linux target")
    if os.environ.get("ROS_DOMAIN_ID") != expected or os.environ.get("ROS_LOCALHOST_ONLY") != "1":
        raise RuntimeError(f"set ROS_DOMAIN_ID={expected} and ROS_LOCALHOST_ONLY=1 for the simulation")
    if os.environ.get("ROS_DISCOVERY_SERVER"):
        raise RuntimeError("autonomy simulation must not use an external discovery server")


def yaw_to_quaternion(yaw):
    return 0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5)


def quaternion_to_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def compose(a, b):
    """SE(2) composition a * b of (x, y, yaw) tuples."""
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1],
            math.atan2(math.sin(a[2] + b[2]), math.cos(a[2] + b[2])))


def inverse(a):
    c, s = math.cos(a[2]), math.sin(a[2])
    return (-c * a[0] - s * a[1], s * a[0] - c * a[1], -a[2])


def stamp_seconds(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def json_message(value):
    from std_msgs.msg import String
    message = String()
    message.data = json.dumps(value, ensure_ascii=True, allow_nan=False, separators=(",", ":"))
    return message


def latched_qos(depth=1):
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
    return QoSProfile(depth=depth, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                      reliability=ReliabilityPolicy.RELIABLE)


def sensor_qos(depth=5):
    from rclpy.qos import qos_profile_sensor_data
    return qos_profile_sensor_data


def pose_stamped(frame, stamp, x, y, yaw):
    from geometry_msgs.msg import PoseStamped
    message = PoseStamped()
    message.header.frame_id = frame
    message.header.stamp = stamp
    message.pose.position.x, message.pose.position.y = float(x), float(y)
    qx, qy, qz, qw = yaw_to_quaternion(yaw)
    message.pose.orientation.x, message.pose.orientation.y = qx, qy
    message.pose.orientation.z, message.pose.orientation.w = qz, qw
    return message


def lookup_pose(buffer, target, source):
    """Latest ``target <- source`` planar pose and its stamp, or None."""
    from rclpy.time import Time
    try:
        transform = buffer.lookup_transform(target, source, Time())
    except Exception:  # tf2 raises several lookup/extrapolation types
        return None
    t = transform.transform
    return (t.translation.x, t.translation.y, quaternion_to_yaw(t.rotation)), stamp_seconds(transform.header.stamp)


def load_config(path):
    with open(path, encoding="utf-8") as stream:
        value = json.load(stream)
    if not isinstance(value, dict):
        raise ValueError("node configuration must be a JSON object")
    return value
