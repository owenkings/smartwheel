"""Odometry-anchored 2-D mapper: integrates /scan at the odometry-predicted pose.

Two pose sources:

* ``odom_grid``: wheel + IMU odometry only. ``map`` and ``odom`` coincide (a
  static identity), mirroring the project's mapping choice in ADR 0001.
* ``odom_icp``: the odometry prediction is refined against the map being
  built by a bounded, prior-regularized point-to-line match (scan_match.py).
  The accumulated correction is published as ``map -> odom``; rejected
  matches keep the prediction. There is no loop closure.

Publishes /map (latched, at most once per period when it changed) and a status
JSON with match statistics. Map quality is reported against simulated truth.
"""
from __future__ import annotations

import argparse
import array
from collections import Counter, deque

import numpy as np

from .occupancy import LogOddsGrid
from .ros_util import (compose, inverse, json_message, latched_qos, load_config, quaternion_to_yaw,
                       require_isolated_domain, yaw_to_quaternion)
from .scan_match import IcpConfig, correct_pose, map_points, scan_points

TF_LEAD_S = 0.2  # map->odom is stamped ahead, as slam_toolbox and AMCL do


class MapperNode:
    def __init__(self, node, config):
        from geometry_msgs.msg import TransformStamped
        from nav_msgs.msg import OccupancyGrid
        from sensor_msgs.msg import LaserScan
        from std_msgs.msg import String
        from tf2_ros import Buffer, StaticTransformBroadcaster, TransformBroadcaster, TransformListener
        self.node = node
        self.grid = LogOddsGrid(**dict(config.get("grid", {})))
        self.scan_matching = bool(config.get("scan_matching", False))
        self.icp = IcpConfig(**dict(config.get("icp", {}))) if self.scan_matching else None
        self.publish_period = float(config.get("publish_period_s", 1.0))
        self.max_scan_age = float(config.get("max_scan_age_s", 1.0))
        self.OccupancyGrid, self.TransformStamped = OccupancyGrid, TransformStamped
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, node)
        self.T_map_odom = (0.0, 0.0, 0.0)
        self.last_tf_time = None
        if self.scan_matching:
            self.dynamic = TransformBroadcaster(node)
        else:
            self.static = StaticTransformBroadcaster(node)
            identity = TransformStamped()
            identity.header.stamp = node.get_clock().now().to_msg()
            identity.header.frame_id, identity.child_frame_id = "map", "odom"
            identity.transform.rotation.w = 1.0
            self.static.sendTransform(identity)
        self.map_pub = node.create_publisher(OccupancyGrid, "/map", latched_qos())
        self.status_pub = node.create_publisher(String, "/wc_nav/mapper_status", latched_qos())
        self.pending = deque(maxlen=50)
        self.counts = {"integrated": 0, "dropped_no_tf": 0, "dropped_overflow": 0}
        self.matches = Counter()
        self.largest_correction = [0.0, 0.0]
        self.changed = False
        self.last_publish = -1e9
        self.last_status = -1e9
        node.create_subscription(LaserScan, "/scan", self.on_scan, 10)
        node.create_timer(0.05, self.process)

    def now(self):
        return self.node.get_clock().now().nanoseconds * 1e-9

    def on_scan(self, message):
        if len(self.pending) == self.pending.maxlen:
            self.counts["dropped_overflow"] += 1
        self.pending.append(message)

    def integrate(self, message, odom_sensor):
        count = len(message.ranges)
        angles = message.angle_min + message.angle_increment * np.arange(count)
        ranges = np.asarray(message.ranges, dtype=float)
        pose = compose(self.T_map_odom, odom_sensor)
        if self.scan_matching:
            points = scan_points(angles, ranges, message.range_min, message.range_max, self.icp.max_points)
            nearby = map_points(self.grid, pose[:2], message.range_max + 0.5)
            result = correct_pose(nearby, pose, points, self.icp)
            self.matches[result["status"]] += 1
            if result["status"] == "CORRECTED":
                dx, dy, dyaw = result["delta"]
                self.largest_correction = [max(self.largest_correction[0], float(np.hypot(dx, dy))),
                                           max(self.largest_correction[1], abs(float(dyaw)))]
                pose = tuple(result["pose"])
                self.T_map_odom = compose(pose, inverse(odom_sensor))
        self.grid.integrate(pose, angles, ranges, message.range_min, message.range_max)
        self.counts["integrated"] += 1
        self.changed = True

    def process(self):
        from rclpy.time import Time
        now = self.now()
        while self.pending:
            message = self.pending[0]
            stamp = Time.from_msg(message.header.stamp)
            frame = message.header.frame_id
            if self.buffer.can_transform("odom", frame, stamp):
                t = self.buffer.lookup_transform("odom", frame, stamp).transform
                self.integrate(message, (t.translation.x, t.translation.y, quaternion_to_yaw(t.rotation)))
                self.pending.popleft()
            elif now - stamp.nanoseconds * 1e-9 > self.max_scan_age:
                self.counts["dropped_no_tf"] += 1
                self.pending.popleft()
            else:
                break  # wait for the odometry transform at this stamp
        if self.scan_matching:
            self.publish_correction(now)
        if self.changed and now - self.last_publish >= self.publish_period:
            self.publish_map()
        if now - self.last_status >= 10.0:
            self.last_status = now
            self.status_pub.publish(json_message({
                "time_s": round(now, 3), "mode": "ODOMETRY_ICP" if self.scan_matching else "ODOMETRY_ONLY",
                "counts": self.counts, "matches": dict(self.matches),
                "largest_applied_correction_m_rad": self.largest_correction,
                "map_to_odom": list(self.T_map_odom) if self.scan_matching else "STATIC_IDENTITY",
                "grid": self.grid.parameters()}))

    def publish_correction(self, now):
        if now <= 0 or (self.last_tf_time is not None and now <= self.last_tf_time):
            return
        from rclpy.duration import Duration
        stamp = self.node.get_clock().now() + Duration(seconds=TF_LEAD_S)
        message = self.TransformStamped()
        message.header.stamp = stamp.to_msg()
        message.header.frame_id, message.child_frame_id = "map", "odom"
        message.transform.translation.x, message.transform.translation.y = self.T_map_odom[0], self.T_map_odom[1]
        qx, qy, qz, qw = yaw_to_quaternion(self.T_map_odom[2])
        r = message.transform.rotation
        r.x, r.y, r.z, r.w = qx, qy, qz, qw
        self.dynamic.sendTransform(message)
        self.last_tf_time = now

    def publish_map(self):
        cropped = self.grid.cropped()
        if cropped is None:
            return
        data, origin = cropped
        message = self.OccupancyGrid()
        message.header.stamp = self.node.get_clock().now().to_msg()
        message.header.frame_id = "map"
        message.info.map_load_time = message.header.stamp
        message.info.resolution = float(self.grid.resolution)
        message.info.height, message.info.width = int(data.shape[0]), int(data.shape[1])
        message.info.origin.position.x, message.info.origin.position.y = float(origin[0]), float(origin[1])
        message.info.origin.orientation.w = 1.0
        message.data = array.array("b", np.ascontiguousarray(data).tobytes())
        self.map_pub.publish(message)
        self.changed = False
        self.last_publish = self.now()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    args, ros_args = parser.parse_known_args(argv)
    require_isolated_domain()
    config = load_config(args.config)
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.parameter import Parameter
    rclpy.init(args=ros_args)
    node = rclpy.create_node("wc_odom_grid_mapper", parameter_overrides=[Parameter("use_sim_time", value=True)])
    try:
        MapperNode(node, config)
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
