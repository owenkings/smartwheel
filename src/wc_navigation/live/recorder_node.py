"""Recorder of a live (or bag-replayed) shadow run: what the framework would do and how the chair moved.

Writes rows as they arrive (flushed), so a stopped or crashed run keeps its
record: every velocity-chain stage (``commands.csv``), the chair's odometry
(``odom.csv``), AMCL poses and the sampled ``map -> odom`` (``localization.csv``),
the scans (``scans.csv``, ranges optional), global plans (``plans.jsonl``) and
state changes of the gate, the inputs and the goals (``events.jsonl``). It only
subscribes; it publishes nothing.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path

from ..autonomy import nav2_config
from ..autonomy.ros_util import latched_qos, load_config, lookup_pose, quaternion_to_yaw, stamp_seconds

STAGES = (("nav", nav2_config.CMD_NAV), ("smoothed", nav2_config.CMD_SMOOTHED),
          ("monitored", nav2_config.CMD_MONITORED), ("gated", nav2_config.CMD_GATED))
BASE_STATUS_TOPIC = "/wc_nav/base_status"        # closed loop: window A's base bridge (wc_base_status_relay_v1)
BASE_STATUS_KEYS = ("connected", "base_auto", "reason", "source", "auto_state", "enable_source", "enable_request",
                    "estop_request", "dead_man", "estop", "enable")


class _Writer:
    def __init__(self, path, header):
        self.stream = open(path, "x", newline="", encoding="utf-8")
        self.csv = csv.writer(self.stream)
        self.csv.writerow(header)
        self.stream.flush()

    def row(self, values):
        self.csv.writerow([("%.6f" % v) if isinstance(v, float) and math.isfinite(v) else v for v in values])
        self.stream.flush()

    def close(self):
        self.stream.close()


class ShadowRecorder:
    def __init__(self, node, config):
        from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
        from nav_msgs.msg import Odometry, Path as PathMessage
        from sensor_msgs.msg import LaserScan
        from std_msgs.msg import String
        from tf2_ros import Buffer, TransformListener
        self.node = node
        out = Path(config["output_dir"])
        self.ranges = bool(config.get("record_scan_ranges", False))
        self.commands = _Writer(out / "commands.csv", ["time_s", "stage", "linear", "angular"])
        self.odom = _Writer(out / "odom.csv", ["time_s", "stamp_s", "x", "y", "yaw", "linear", "angular"])
        self.localization = _Writer(out / "localization.csv",
                                    ["kind", "time_s", "stamp_s", "x", "y", "yaw", "cov_xx", "cov_xy", "cov_yy",
                                     "cov_yawyaw"])
        self.scans = _Writer(out / "scans.csv", ["time_s", "stamp_s", "observed_bins", "nearest_m"]
                             + (["r%03d" % k for k in range(161)] if self.ranges else []))
        self.events = open(out / "events.jsonl", "x", encoding="utf-8")
        self.plans = open(out / "plans.jsonl", "x", encoding="utf-8")
        self.last = {}
        for stage, topic in STAGES:
            node.create_subscription(Twist, topic, lambda m, s=stage: self.commands.row(
                [self.now(), s, float(m.linear.x), float(m.angular.z)]), 50)
        node.create_subscription(Odometry, "/odom", self.on_odom, 50)
        node.create_subscription(LaserScan, "/scan", self.on_scan, 10)
        node.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self.on_pose, latched_qos())
        node.create_subscription(PathMessage, "/plan", self.on_plan, 10)
        for topic in ("/wc_nav/gate_status", "/wc_live/inputs_status", "/wc_live/inputs_status/scan",
                      "/wc_live/inputs_status/odometry", "/wc_live/goal_status", "/wc_live/relay_status",
                      "/wc_live/inputs_status/obstacles"):          # window C's 3D obstacle input, when started
            node.create_subscription(String, topic, lambda m, t=topic: self.on_status(t, m), latched_qos())
        # closed loop: window A's base bridge publishes it reliable and volatile at 10 Hz; changes are recorded
        node.create_subscription(String, BASE_STATUS_TOPIC, self.on_base_status, 10)
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, node)
        self.last_map_odom = None
        node.create_timer(0.1, self.sample_map_odom)

    def now(self):
        return self.node.get_clock().now().nanoseconds * 1e-9

    def on_odom(self, m):
        p = m.pose.pose
        self.odom.row([self.now(), stamp_seconds(m.header.stamp), p.position.x, p.position.y,
                       quaternion_to_yaw(p.orientation), m.twist.twist.linear.x, m.twist.twist.angular.z])

    def on_scan(self, m):
        finite = [r for r in m.ranges if math.isfinite(r)]
        self.scans.row([self.now(), stamp_seconds(m.header.stamp), len(finite), min(finite) if finite else ""]
                       + ([("%.5f" % r) if math.isfinite(r) else "inf" for r in m.ranges] if self.ranges else []))

    def on_pose(self, m):
        p, c = m.pose.pose, m.pose.covariance
        self.localization.row(["amcl_pose", self.now(), stamp_seconds(m.header.stamp), p.position.x, p.position.y,
                               quaternion_to_yaw(p.orientation), float(c[0]), float(c[1]), float(c[7]), float(c[35])])

    def sample_map_odom(self):
        found = lookup_pose(self.buffer, "map", "odom")
        if found is None or found[1] == self.last_map_odom:
            return
        self.last_map_odom = found[1]
        (x, y, yaw), stamp = found
        self.localization.row(["map_odom", self.now(), stamp, x, y, yaw, "", "", "", ""])

    def on_plan(self, m):
        points, last = [], None
        for pose in m.poses:
            x, y = pose.pose.position.x, pose.pose.position.y
            if last is None or math.hypot(x - last[0], y - last[1]) >= 0.1:
                points.append([round(x, 3), round(y, 3)])
                last = (x, y)
        self.plans.write(json.dumps({"time_s": round(self.now(), 3), "frame": m.header.frame_id, "xy": points}) + "\n")
        self.plans.flush()

    def on_status(self, topic, message):
        try:
            value = json.loads(message.data)
        except ValueError:
            value = {"raw": message.data}
        preview = value.get("preview") if isinstance(value.get("preview"), dict) else {}
        # the gate's preview (closed loop, before "start automatic") counts as a change too
        key = (value.get("state"), tuple(value.get("reasons") or ()), value.get("stale_inputs") and tuple(value["stale_inputs"]),
               preview.get("state"), tuple(preview.get("reasons") or ()))
        if self.last.get(topic) == key:
            return
        self.last[topic] = key
        self.events.write(json.dumps({"time_s": round(self.now(), 3), "topic": topic, "value": value}) + "\n")
        self.events.flush()

    def on_base_status(self, message):
        """The bridge's status when what the gate and the operator act on changes (not every 10 Hz copy)."""
        try:
            value = json.loads(message.data)
        except ValueError:
            value = {"raw": message.data}
        key = tuple(value.get(k) for k in BASE_STATUS_KEYS) if isinstance(value, dict) else ("INVALID",)
        if self.last.get(BASE_STATUS_TOPIC) == key:
            return
        self.last[BASE_STATUS_TOPIC] = key
        self.events.write(json.dumps({"time_s": round(self.now(), 3), "topic": BASE_STATUS_TOPIC, "value": value})
                          + "\n")
        self.events.flush()

    def close(self):
        for writer in (self.commands, self.odom, self.localization, self.scans):
            writer.close()
        self.events.close()
        self.plans.close()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    args, ros_args = parser.parse_known_args(argv)
    config = load_config(args.config)
    from .supervisor import require_live_domain
    require_live_domain(config)
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.parameter import Parameter
    rclpy.init(args=ros_args)
    node = rclpy.create_node("wc_live_recorder", parameter_overrides=[
        Parameter("use_sim_time", value=bool(config.get("use_sim_time", False)))])
    recorder = None
    try:
        recorder = ShadowRecorder(node, config)
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        if rclpy.ok():
            raise
    finally:
        if recorder is not None:
            recorder.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
