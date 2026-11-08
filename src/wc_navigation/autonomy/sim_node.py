"""Simulated wheelchair base, laser and clock for the isolated autonomy domain.

Publishes ``/clock``, ``/odom`` + ``odom->base_link``, a static
``base_link->laser``, ``/scan``, and ground truth on ``/wc_sim/ground_truth``
(frame ``wc_sim_world``). It accepts motion only from the safety gate output
topic. There is no device access and no connection to the wheel owner.
Collisions use the configured body: the assumed circle, or the convex outline
of the navigation profile, checked along every step including turns. With
``map_to_odom_from_truth`` it also publishes ``map->odom`` from ground truth,
so navigation can be judged without AMCL.
"""
from __future__ import annotations

import argparse
import math
import time

import numpy as np

from .kinematics import BaseLimits, DiffDriveSim, OdometryErrorModel
from .nav2_config import CMD_GATED
from .ros_util import (compose, inverse, json_message, latched_qos, load_config, require_isolated_domain,
                       yaw_to_quaternion)
from .world import LaserModel, load_world


def collision_body(config):
    """Simulated body: ``collision_body`` (circle or the NavProfile outline), else the legacy radius."""
    from ..planning.body import Body
    if config.get("collision_body"):
        return Body.from_config(config["collision_body"])
    return Body(radius_m=float(config["collision_radius_m"]))


def _time_msg(seconds):
    from builtin_interfaces.msg import Time
    nanoseconds = int(round(seconds * 1e9))
    message = Time()
    message.sec, message.nanosec = divmod(nanoseconds, 1_000_000_000)
    return message


class Simulator:
    def __init__(self, node, config):
        from geometry_msgs.msg import TransformStamped, Twist
        from nav_msgs.msg import Odometry
        from rosgraph_msgs.msg import Clock
        from sensor_msgs.msg import LaserScan
        from std_msgs.msg import String
        from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
        self.node = node
        self.world = load_world(config["world"])
        self.laser = LaserModel(**config.get("laser", {}))
        self.body = collision_body(config)
        self.radius = self.body.radius_m
        start = tuple(float(v) for v in config["start_pose"])
        if self.world.body_collides(start, self.body):
            raise ValueError("start pose collides with the simulated world")
        self.base = DiffDriveSim(start, BaseLimits(**config.get("base_limits", {})),
                                 OdometryErrorModel(**config.get("odometry", {})))
        self.rng = np.random.default_rng(int(config.get("seed", 0)))
        self.rtf = float(config.get("real_time_factor", 1.0))
        if not 0.1 <= self.rtf <= 4.0:
            raise ValueError("real_time_factor must be within 0.1..4")
        self.physics_hz = float(config.get("physics_hz", 100.0))
        self.dt = self.rtf / self.physics_hz
        self.odom_period = 1.0 / float(config.get("odom_hz", 50.0))
        self.scan_period = 1.0 / float(config.get("scan_hz", 10.0))
        self.sim_time = self.start_time = float(config.get("start_time_s", 100.0))
        self.next_odom = self.next_scan = self.next_status = self.sim_time
        self.contact = False
        self.contacts = []
        self.Clock, self.Odometry, self.LaserScan, self.TransformStamped = Clock, Odometry, LaserScan, TransformStamped
        self.clock_pub = node.create_publisher(Clock, "/clock", 10)
        self.odom_pub = node.create_publisher(Odometry, "/odom", 20)
        self.truth_pub = node.create_publisher(Odometry, "/wc_sim/ground_truth", 20)
        self.scan_pub = node.create_publisher(LaserScan, "/scan", 10)
        self.status_pub = node.create_publisher(String, "/wc_sim/status", latched_qos())
        self.collision_pub = node.create_publisher(String, "/wc_sim/collision", latched_qos(depth=20))
        self.tf = TransformBroadcaster(node)
        static = TransformStamped()
        static.header.stamp = _time_msg(self.sim_time)
        static.header.frame_id, static.child_frame_id = "base_link", "laser"
        static.transform.translation.x = self.laser.mount_x
        static.transform.translation.y = self.laser.mount_y
        qx, qy, qz, qw = yaw_to_quaternion(self.laser.mount_yaw)
        static.transform.rotation.x, static.transform.rotation.y = qx, qy
        static.transform.rotation.z, static.transform.rotation.w = qz, qw
        self.static_tf = StaticTransformBroadcaster(node)
        self.static_tf.sendTransform(static)
        node.create_subscription(Twist, CMD_GATED, self.on_command, 10)
        # Optional: map->odom from ground truth (navigation without AMCL, to judge planning and control alone).
        truth = config.get("map_to_odom_from_truth")
        self.map_T_world = None if not truth else inverse(tuple(float(v) for v in truth["world_T_map"]))
        self.wall_start = time.monotonic()
        node.create_timer(1.0 / self.physics_hz, self.tick)

    def on_command(self, message):
        self.base.set_command(message.linear.x, message.angular.z, self.sim_time)

    def _collides(self, a, b):
        return self.world.swept_body_collides(a, b, self.body)

    def tick(self):
        moved = self.base.step(self.dt, self.sim_time, self._collides)
        self.sim_time += self.dt
        if not moved and not self.contact:
            x, y, yaw = self.base.pose
            event = {"sim_time_s": round(self.sim_time, 3), "x": x, "y": y, "yaw": yaw,
                     "clearance_m": self.world.clearance(x, y),
                     "body_gap_m": self.world.body_clearance(self.base.pose, self.body),
                     "body": self.body.source, "count": self.base.collisions}
            self.contacts.append(event)
            self.collision_pub.publish(json_message(event))
        self.contact = not moved
        clock = self.Clock()
        clock.clock = _time_msg(self.sim_time)
        self.clock_pub.publish(clock)
        if self.sim_time + 1e-9 >= self.next_odom:
            self.next_odom += self.odom_period
            self.publish_odometry()
        if self.sim_time + 1e-9 >= self.next_scan:
            self.next_scan += self.scan_period
            self.publish_scan()
        if self.sim_time + 1e-9 >= self.next_status:
            self.next_status += 1.0
            x, y, yaw = self.base.pose
            self.status_pub.publish(json_message({
                "sim_time_s": round(self.sim_time, 3), "pose": [x, y, yaw],
                "distance_m": self.base.distance_m, "collisions": self.base.collisions,
                "real_time_factor_measured": (self.sim_time - self.start_time) / max(1e-6, time.monotonic() - self.wall_start),
                "hardware_control": False}))

    def _odometry(self, frame, child, pose, v, w, stamp):
        message = self.Odometry()
        message.header.stamp = stamp
        message.header.frame_id, message.child_frame_id = frame, child
        message.pose.pose.position.x, message.pose.pose.position.y = pose[0], pose[1]
        qx, qy, qz, qw = yaw_to_quaternion(pose[2])
        o = message.pose.pose.orientation
        o.x, o.y, o.z, o.w = qx, qy, qz, qw
        message.twist.twist.linear.x, message.twist.twist.angular.z = v, w
        return message

    def publish_odometry(self):
        stamp = _time_msg(self.sim_time)
        b = self.base
        self.odom_pub.publish(self._odometry("odom", "base_link", b.odom_pose, b.odom_v, b.odom_w, stamp))
        self.truth_pub.publish(self._odometry("wc_sim_world", "wc_sim_truth", b.pose, b.v, b.w, stamp))
        transforms = [self._transform("odom", "base_link", b.odom_pose, stamp)]
        if self.map_T_world is not None:
            map_T_odom = compose(compose(self.map_T_world, b.pose), inverse(b.odom_pose))
            transforms.append(self._transform("map", "odom", map_T_odom, stamp))
        self.tf.sendTransform(transforms)

    def _transform(self, parent, child, pose, stamp):
        t = self.TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id, t.child_frame_id = parent, child
        t.transform.translation.x, t.transform.translation.y = pose[0], pose[1]
        qx, qy, qz, qw = yaw_to_quaternion(pose[2])
        r = t.transform.rotation
        r.x, r.y, r.z, r.w = qx, qy, qz, qw
        return t

    def publish_scan(self):
        ranges = self.world.scan(self.base.pose, self.laser, self.rng)
        message = self.LaserScan()
        message.header.stamp = _time_msg(self.sim_time)
        message.header.frame_id = "laser"
        message.angle_min, message.angle_max = self.laser.angle_min, self.laser.angle_max
        message.angle_increment = self.laser.angle_increment
        message.time_increment = 0.0
        message.scan_time = self.scan_period
        message.range_min, message.range_max = self.laser.range_min, self.laser.range_max
        message.ranges = [float(r) for r in ranges]
        self.scan_pub.publish(message)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    args, ros_args = parser.parse_known_args(argv)
    require_isolated_domain()
    config = load_config(args.config)
    import rclpy
    from rclpy.executors import ExternalShutdownException
    rclpy.init(args=ros_args)
    node = rclpy.create_node("wc_sim")
    try:
        Simulator(node, config)
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        if rclpy.ok():
            raise  # a real failure while running; shutdown-time publish errors are expected
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
