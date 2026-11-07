"""Wheel feedback + H30 gyro -> planar axle odometry, through the production motion prior.

``wc_runtime.mapping_prior.MotionPrior`` (planar EKF, robot_localization
worker) is used unchanged, with the same input checks as the live mapping
prior node: wheel JSON records and H30 frames go in with their arrival stamps,
and poses come out only up to the common IMU/wheel watermark (no
extrapolation). The prior's clock is the newest host monotonic time seen in the
messages, so a bag replayed later behaves as when it was recorded; source
staleness is watched separately by the node. Poses are emitted on a fixed grid
(default 50 Hz) once the watermark has passed, and each emitted pose becomes the
prior's checkpoint so its history stays bounded. The prior pose is the lidar
reference (``mapping_reference``) in its odometry frame; base_link is the axle,
the reference moved back by the planar lever arm.
"""
from __future__ import annotations

import json
import math

import numpy as np


class OdometryNotReady(RuntimeError):
    pass


def axle_from_reference(pose, lever_xy):
    """Planar (x, y, yaw) of the axle from a 4x4 reference pose and the lidar lever arm in the axle frame."""
    yaw = math.atan2(pose[1, 0], pose[0, 0])
    c, s = math.cos(yaw), math.sin(yaw)
    lx, ly = lever_xy
    return float(pose[0, 3] - c * lx + s * ly), float(pose[1, 3] - s * lx - c * ly), yaw


class LiveOdometry:
    def __init__(self, prior_config, lever_xy, period_s=0.02, prior_factory=None):
        self.config = prior_config
        self.lever = [float(v) for v in lever_xy]
        self.period_ns = int(round(float(period_s) * 1e9))
        if not 1_000_000 <= self.period_ns <= 200_000_000:
            raise ValueError("odometry period must be within 1..200 ms")
        self.factory = prior_factory
        self.prior = None
        self.session_id = None
        self.mono_ns = 0
        self.last_emitted = None
        self.counts = {"imu": 0, "wheel": 0, "emitted": 0, "rejected_imu": 0}

    def _ensure(self, session_id):
        if self.prior is None:
            from wc_runtime.mapping_prior import MotionPrior
            factory = self.factory or MotionPrior
            self.session_id = session_id
            self.prior = factory(self.config, session_id, clock=lambda: self.mono_ns * 1e-9)
        elif session_id != self.session_id:
            raise ValueError("session changed from %s to %s" % (self.session_id, session_id))

    def add_imu(self, message):
        """One H30Frame, with the checks of the live mapping prior node."""
        if not message.angular_velocity_valid or not message.linear_acceleration_valid or message.uncertainty_valid:
            self.counts["rejected_imu"] += 1
            return
        if message.imu.header.frame_id != "imu_h30_native" or message.header.frame_id != "imu_h30_native":
            raise ValueError("unexpected native H30 frame id")
        self._ensure(message.session_id)
        stamp = int(message.host_receive_time.sec) * 1_000_000_000 + int(message.host_receive_time.nanosec)
        self.mono_ns = max(self.mono_ns, int(message.host_monotonic_ns))
        a, g = message.imu.linear_acceleration, message.imu.angular_velocity
        self.prior.add_imu(stamp_ns=stamp, monotonic_ns=int(message.host_monotonic_ns),
                           acceleration=[a.x, a.y, a.z], angular_velocity=[g.x, g.y, g.z],
                           sensor_id=message.sensor_id, session_id=message.session_id,
                           stream_epoch=message.stream_epoch, sequence=int(message.frame_sequence),
                           coordinate_convention=message.coordinate_convention, time_source=message.time_source,
                           common_time_valid=message.common_time_valid)
        self.counts["imu"] += 1

    def add_wheel(self, data):
        """One wheel feedback JSON record (std_msgs/String data)."""
        if self.prior is None:
            return False  # the session is learned from the first IMU frame
        record = json.loads(data)
        self.mono_ns = max(self.mono_ns, int(record.get("receive_monotonic_ns", 0)))
        self.prior.add_wheel(record)
        self.counts["wheel"] += 1
        return True

    def watermark(self):
        prior = self.prior
        if prior is None or prior.initialization is None or not prior.imu or not prior.wheel:
            return None
        return min(prior.imu[-1]["stamp"], prior.wheel[-1]["stamp"])

    def poses(self):
        """New (stamp_ns, x, y, yaw) axle poses on the emission grid up to the watermark."""
        mark = self.watermark()
        if mark is None:
            return []
        if self.last_emitted is None:
            start = max(int(self.prior.initialization["stamp_ns"]), int(self.prior.checkpoints[0][0]))
            queries = [start] if start <= mark else []
        else:
            queries = list(range(self.last_emitted + self.period_ns, mark + 1, self.period_ns))
        result = []
        for stamp in queries:
            try:
                pose = self.prior.pose_at(int(stamp))
            except Exception as error:  # NotReady / DroppedCloud: the watermark or history moved
                if type(error).__name__ in ("NotReady", "DroppedCloud"):
                    break
                raise
            self.prior.record_forwarded(int(stamp), pose)
            self.last_emitted = int(stamp)
            result.append((int(stamp), *axle_from_reference(pose, self.lever)))
        self.counts["emitted"] += len(result)
        return result


def odometry_message(stamp, pose, twist, frame_id="odom", child_frame_id="base_link"):
    from nav_msgs.msg import Odometry
    from ..autonomy.ros_util import yaw_to_quaternion
    message = Odometry()
    message.header.stamp = stamp
    message.header.frame_id, message.child_frame_id = frame_id, child_frame_id
    message.pose.pose.position.x, message.pose.pose.position.y = float(pose[0]), float(pose[1])
    qx, qy, qz, qw = yaw_to_quaternion(float(pose[2]))
    o = message.pose.pose.orientation
    o.x, o.y, o.z, o.w = qx, qy, qz, qw
    message.twist.twist.linear.x, message.twist.twist.angular.z = float(twist[0]), float(twist[1])
    return message


def body_twist(previous, current, dt):
    """Forward speed and yaw rate between two planar poses."""
    if previous is None or dt <= 0:
        return 0.0, 0.0
    turn = math.atan2(math.sin(current[2] - previous[2]), math.cos(current[2] - previous[2]))
    heading = previous[2] + 0.5 * turn
    dx, dy = current[0] - previous[0], current[1] - previous[1]
    return (dx * math.cos(heading) + dy * math.sin(heading)) / dt, turn / dt
