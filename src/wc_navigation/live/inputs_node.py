"""Live inputs for Nav2: /scan, /odom, odom->base_link and static base_link->laser from sensor topics.

Subscribes only to the left lidar host-filtered SourceFrame, the H30 frames and
the wheel feedback records (as recorded in rosbag2 or relayed from the
vehicle's sensor domain); publishes the Nav2 inputs listed above and a status
JSON on ``/wc_live/inputs_status``. It opens no device, subscribes to no
command and publishes no velocity. When an input stops for longer than
``stale_after_s`` the status reports it; odometry is then simply not extended,
so the TF ages and the safety gate stops on stale localization.
"""
from __future__ import annotations

import argparse
import json
import time

from ..autonomy.ros_util import json_message, latched_qos, load_config, yaw_to_quaternion
from .config import IMU_TOPIC, LEFT_FILTERED_TOPIC, WHEEL_TOPIC
from .odometry_adapter import LiveOdometry, body_twist, odometry_message
from .scan_adapter import (LIVE_LIDAR_TIME_OFFSET_MS, ScanRejected, check_source, laser_scan_message,
                           scan_from_source, scan_stamp_ns)


def _stamp_ns(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def _time_msg(stamp_ns):
    from builtin_interfaces.msg import Time
    sec, nanosec = divmod(int(stamp_ns), 1_000_000_000)
    return Time(sec=sec, nanosec=nanosec)


class LiveInputs:
    def __init__(self, node, config):
        from geometry_msgs.msg import TransformStamped
        from nav_msgs.msg import Odometry
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        from sensor_msgs.msg import LaserScan
        from std_msgs.msg import String
        from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
        from wc_interfaces.msg import H30Frame, SourceFrame
        self.node = node
        live = config["live"]
        prefix = config.get("source_prefix", "")
        # "scan" and "odometry" run the two halves in separate processes, so lidar frames (the heavy part)
        # never delay the wheel/IMU callbacks; "all" keeps both in one process.
        self.role = config.get("role", "all")
        if self.role not in ("all", "scan", "odometry"):
            raise ValueError("role must be all, scan or odometry")
        scan_role, odometry_role = self.role in ("all", "scan"), self.role in ("all", "odometry")
        self.filter_config = live["filter"]
        self.transforms = {"T_base_axle": live["T_base_axle"]}
        self.rotation = live["R_reference_base"]
        self.lidar_id = live["identities"]["lidar_left"]
        self.lever = live["lever_xy"]
        self.odometry = LiveOdometry(live["prior"], self.lever, float(config.get("odom_period_s", 0.02)))
        rate = float(config.get("scan_rate_hz", 10.0))
        self.scan_period_ns = int(1e9 / rate) if rate > 0 else 0
        # Lidar time offset (live default 75 ms since 2026-11-08); see scan_adapter.scan_stamp_ns.
        self.lidar_time_offset_ms = float(config.get("lidar_time_offset_ms", LIVE_LIDAR_TIME_OFFSET_MS))
        scan_stamp_ns(0, self.lidar_time_offset_ms)
        self.stale_after_s = float(config.get("stale_after_s", 1.0))
        self.TransformStamped = TransformStamped
        status_topic = "/wc_live/inputs_status" + ("" if self.role == "all" else "/" + self.role)
        self.status_pub = node.create_publisher(String, status_topic, latched_qos())
        # Same reliability as the live mapping prior node: sensors best effort, wheel records reliable.
        if scan_role:
            self.scan_pub = node.create_publisher(LaserScan, "/scan", 10)
            node.create_subscription(SourceFrame, prefix + LEFT_FILTERED_TOPIC, self.on_cloud,
                                     QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT))
        if odometry_role:
            self.odom_pub = node.create_publisher(Odometry, "/odom", 20)
            self.tf = TransformBroadcaster(node)
            self.static_tf = StaticTransformBroadcaster(node)
            laser = TransformStamped()
            laser.header.stamp = node.get_clock().now().to_msg()
            laser.header.frame_id, laser.child_frame_id = "base_link", "laser"
            laser.transform.translation.x, laser.transform.translation.y = float(self.lever[0]), float(self.lever[1])
            laser.transform.rotation.w = 1.0
            self.static_tf.sendTransform(laser)
            node.create_subscription(H30Frame, prefix + IMU_TOPIC, self.on_imu, QoSProfile(
                depth=400, reliability=ReliabilityPolicy.BEST_EFFORT))
            node.create_subscription(String, prefix + WHEEL_TOPIC, self.on_wheel,
                                     QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE))
        self.last_scan_ns = None
        self.previous_pose = None
        self.received = {name: None for name, used in (("cloud", scan_role), ("imu", odometry_role),
                                                        ("wheel", odometry_role)) if used}
        self.counts = {"scans": 0, "clouds_skipped_by_rate": 0, "clouds_rejected": 0, "odom": 0}
        self.failure = None
        self.reasons = {}
        # Steady clock: the status must appear before a replayed /clock starts (sim time stands still until then).
        from rclpy.clock import Clock, ClockType
        node.create_timer(0.5, self.publish_status, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def now(self):
        return self.node.get_clock().now().nanoseconds * 1e-9

    def _fail(self, where, error):
        self.failure = where + ": " + type(error).__name__ + ": " + str(error)
        self.node.get_logger().error(self.failure)

    def on_cloud(self, message):
        self.received["cloud"] = self.now()
        try:
            check_source(message, self.lidar_id)
            stamp = _stamp_ns(message.header.stamp)
            if self.scan_period_ns and self.last_scan_ns is not None and stamp - self.last_scan_ns < self.scan_period_ns * 0.9:
                self.counts["clouds_skipped_by_rate"] += 1
                return
            ranges, _ = scan_from_source(message, self.filter_config, self.transforms, self.rotation)
        except ScanRejected as error:
            self.counts["clouds_rejected"] += 1
            self.reasons[str(error)[:80]] = self.reasons.get(str(error)[:80], 0) + 1
            return
        except Exception as error:
            self.counts["clouds_rejected"] += 1
            self._fail("scan", error)
            return
        self.last_scan_ns = stamp
        published = (message.header.stamp if self.lidar_time_offset_ms == 0.0
                     else _time_msg(scan_stamp_ns(stamp, self.lidar_time_offset_ms)))
        self.scan_pub.publish(laser_scan_message(ranges, published))
        self.counts["scans"] += 1

    def on_imu(self, message):
        self.received["imu"] = self.now()
        if self.failure and self.failure.startswith("odometry"):
            return
        try:
            self.odometry.add_imu(message)
            self.emit()
        except Exception as error:
            self._fail("odometry", error)

    def on_wheel(self, message):
        self.received["wheel"] = self.now()
        if self.failure and self.failure.startswith("odometry"):
            return
        try:
            self.odometry.add_wheel(message.data)
            self.emit()
        except Exception as error:
            self._fail("odometry", error)

    def emit(self):
        for stamp, x, y, yaw in self.odometry.poses():
            pose = (x, y, yaw)
            dt = (stamp - self.previous_pose[0]) * 1e-9 if self.previous_pose else 0.0
            twist = body_twist(self.previous_pose[1] if self.previous_pose else None, pose, dt)
            self.previous_pose = (stamp, pose)
            ros_stamp = _time_msg(stamp)
            self.odom_pub.publish(odometry_message(ros_stamp, pose, twist))
            transform = self.TransformStamped()
            transform.header.stamp = ros_stamp
            transform.header.frame_id, transform.child_frame_id = "odom", "base_link"
            transform.transform.translation.x, transform.transform.translation.y = x, y
            qx, qy, qz, qw = yaw_to_quaternion(yaw)
            r = transform.transform.rotation
            r.x, r.y, r.z, r.w = qx, qy, qz, qw
            self.tf.sendTransform(transform)
            self.counts["odom"] += 1

    def publish_status(self):
        now = self.now()
        stale = [name for name, last in self.received.items()
                 if last is None or now - last > self.stale_after_s or now < last]
        state = "FAILED" if self.failure else ("STALE" if stale else "RUNNING")
        prior = self.odometry.prior
        self.status_pub.publish(json_message({
            "state": state, "role": self.role, "time_s": round(now, 3), "stale_inputs": stale, "failure": self.failure,
            "lidar_time_offset_ms": self.lidar_time_offset_ms,
            "counts": {**self.counts, **self.odometry.counts}, "rejections": self.reasons,
            "odometry_initialized": bool(prior is not None and prior.initialization is not None),
            "session_id": self.odometry.session_id, "wall_time": time.time(),
            "commands_subscribed": False, "hardware_control": False}))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    args, ros_args = parser.parse_known_args(argv)
    config = load_config(args.config)
    from .supervisor import require_live_domain
    require_live_domain(config)
    if isinstance(config.get("live"), str):
        config["live"] = json.loads(open(config["live"], encoding="utf-8").read())
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.parameter import Parameter
    rclpy.init(args=ros_args)
    node = rclpy.create_node("wc_live_inputs", parameter_overrides=[
        Parameter("use_sim_time", value=bool(config.get("use_sim_time", False)))])
    try:
        LiveInputs(node, config)
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
