"""Live 3D obstacle input node (window C): both lidars' host-filtered SourceFrames + /odom -> obstacle clouds.

Off by default; ``navigation_live_shadow inputs|shadow`` starts it with --obstacle-input, --obstacle-scan or
--obstacle-monitor (processing in ``live_obstacles``, offline check in ``live_check``). It subscribes to the two
lidars' SourceFrames (under the run's source prefix) and /odom, and publishes the shadow runs' obstacle topics, the
static base_link -> obstacles_lidar_<side> transforms and its status on /wc_live/inputs_status/obstacles; one line
per processed frame goes to obstacles.jsonl in the run directory. It opens no device, subscribes to no command and
publishes no velocity; it runs only in the isolated autonomy domains.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import time

from ..autonomy.ros_util import json_message, latched_qos, load_config
from . import live_obstacles as lo
from .nav2_obstacles import pointcloud2
from .obstacle_cloud import WRAP_DELETE_NOT_WIRED, ObstacleFilter


def _stamp_ns(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def _time_msg(stamp_ns):
    from builtin_interfaces.msg import Time
    sec, nanosec = divmod(int(stamp_ns), 1_000_000_000)
    return Time(sec=sec, nanosec=nanosec)


def _yaw(q):
    import math
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class ObstacleInput:
    def __init__(self, node, config):
        entries = config["obstacles"]
        settings = ObstacleFilter(**entries["filter"])
        if settings.wrap_delete:            # the commands refuse it; a hand-edited run_config must not slip through
            raise ValueError(WRAP_DELETE_NOT_WIRED)
        from geometry_msgs.msg import TransformStamped
        from nav_msgs.msg import Odometry
        from rclpy.clock import Clock, ClockType
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        from sensor_msgs.msg import LaserScan, PointCloud2, PointField
        from std_msgs.msg import String
        from tf2_ros import StaticTransformBroadcaster
        from wc_interfaces.msg import SourceFrame
        self.node = node
        live = config["live"]
        self.plan, self.cfg, self.filter = entries["plan"], lo.LiveObstacleConfig(**entries["config"]), settings
        self.body = [tuple(p) for p in entries["body"]]
        self.geometry = entries["geometry"]
        self.host_filter = live["filter"]
        # the offset the command resolved once for /scan and for these clouds (run_config, D 11-09 15:45)
        self.offset_ms = float(entries["lidar_time_offset_ms"])
        self.selector = lo.CycleSelector(self.plan["read"], self.cfg)
        self.gate = lo.PoseGate(self.offset_ms, self.cfg)
        self.history = lo.PoseHistory(self.cfg.odom_history_s)
        self.histories = lo.persistence_histories(self.plan["published"], self.filter)
        self.status = lo.LiveStatus(self.plan["read"], self.cfg, self.offset_ms, entries["lidar_time_offset_source"])
        self.PointCloud2, self.PointField = PointCloud2, PointField
        self.log = (Path(config["output_dir"]) / lo.LOG_NAME).open("a", encoding="utf-8")
        self.points_pub = {s: node.create_publisher(PointCloud2, t, 10) for s, t in self.plan["topics"].items()}
        self.clear_pub = {s: node.create_publisher(PointCloud2, t, 10) for s, t in self.plan["clearing_topics"].items()}
        self.scan_pub = (node.create_publisher(LaserScan, self.plan["filtered_scan_topic"], 10)
                         if self.plan["filtered_scan"] else None)
        self.status_pub = node.create_publisher(String, lo.STATUS_TOPIC, latched_qos())
        self.static_tf = StaticTransformBroadcaster(node)
        statics = []
        for side, frame in self.plan["frames"].items():
            t = TransformStamped()
            t.header.stamp = node.get_clock().now().to_msg()
            t.header.frame_id, t.child_frame_id = "base_link", frame
            lever = self.geometry[side]["lever"]
            t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = map(float, lever)
            t.transform.rotation.w = 1.0
            statics.append(t)
        self.static_tf.sendTransform(statics)
        prefix = config.get("source_prefix", "")
        for side in self.plan["read"]:
            node.create_subscription(SourceFrame, prefix + self.plan["source_topics"][side],
                                     lambda m, s=side: self.on_frame(s, m),
                                     QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT))
        node.create_subscription(Odometry, "/odom", self.on_odom, QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE))
        node.create_timer(0.02, self.on_timer)
        # Steady clock: the status must appear before a replayed /clock starts (sim time stands still until then).
        node.create_timer(0.5, self.publish_status, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def now_ns(self):
        return self.node.get_clock().now().nanoseconds

    def _fail(self, where, error):
        if self.status.failure is None:
            self.status.fail(where, error)
            self.node.get_logger().error(self.status.failure)

    def on_frame(self, side, message):
        now = self.now_ns()
        self.status.mark(side, now * 1e-9)
        if self.status.failure:
            return
        try:
            self.prepare(self.selector.offer(side, _stamp_ns(message.header.stamp), now, message), now)
            self.release(now)
        except Exception as error:
            self._fail("frame", error)

    def on_odom(self, message):
        now = self.now_ns()
        self.status.mark("odom", now * 1e-9)
        p = message.pose.pose
        self.history.add(_stamp_ns(message.header.stamp), p.position.x, p.position.y, _yaw(p.orientation))
        if not self.status.failure:
            try:
                self.release(now)
            except Exception as error:
                self._fail("processing", error)

    def on_timer(self):
        if self.status.failure:
            return
        now = self.now_ns()
        try:
            self.prepare(self.selector.poll(now), now)
            self.release(now)
        except Exception as error:
            self._fail("processing", error)

    def prepare(self, cycles, now_ns):
        """Host filter of every frame of the kept cycles, then into the pose gate."""
        from ..live.scan_adapter import ScanRejected, check_source, reference_points
        for cycle in cycles:
            clouds, rejected, host_ms = {}, {}, {}
            for side, frame in cycle["frames"].items():
                g = self.geometry[side]
                started = time.perf_counter()
                try:
                    check_source(frame["payload"], g["sensor_id"], side)
                    clouds[side] = reference_points(frame["payload"], self.host_filter,
                                                    {"T_base_axle": g["T_base_axle"]}, g["R_reference_base"])[0]
                except ScanRejected as error:
                    rejected[side] = str(error)[:80]
                    self.status.reject(side, error)
                host_ms[side] = (time.perf_counter() - started) * 1e3
                self.status.timing["host_filter_ms"].append(host_ms[side])
                frame["payload"] = None                   # the message is not needed any more
            cycle["clouds"], cycle["rejected"], cycle["host_ms"] = clouds, rejected, host_ms
            self.gate.add(cycle, now_ns)

    def release(self, now_ns):
        for released in self.gate.release(self.history, now_ns):
            cycle = released["cycle"]
            products = lo.process_cycle(released, cycle["clouds"], self.geometry, self.plan, self.filter, self.body,
                                        self.histories)
            published = self.now_ns()
            timing = {}
            for side, frame in cycle["frames"].items():
                timing[side] = {"host_filter": cycle["host_ms"][side], "latency": (published - frame["arrival_ns"]) * 1e-6}
                if side in products:
                    timing[side]["filter"] = products[side]["filter_ms"]
                    self.publish(side, frame, products[side])
                    self.status.timing["filter_ms"].append(products[side]["filter_ms"])
                    self.status.timing["latency_ms"].append(timing[side]["latency"])
            self.status.timing["pose_wait_ms"].append(released["waited_ns"] * 1e-6)
            for line in lo.frame_records(released, cycle["clouds"], products, self.offset_ms, timing, cycle["rejected"]):
                self.log.write(json.dumps(line) + "\n")
            self.log.flush()

    def publish(self, side, frame, product):
        stamp = _time_msg(lo.published_stamp_ns(frame["stamp_ns"], self.offset_ms))
        frame_id = self.plan["frames"][side]
        self.points_pub[side].publish(pointcloud2(self.PointCloud2, self.PointField, product["points"], frame_id, stamp))
        if side in self.clear_pub and product["rays"] is not None:
            self.clear_pub[side].publish(pointcloud2(self.PointCloud2, self.PointField, product["rays"], frame_id, stamp))
        if self.scan_pub is not None and product["scan"] is not None:
            from ..live.scan_adapter import laser_scan_message
            self.scan_pub.publish(laser_scan_message(product["scan"], stamp))
        self.status.published[side] += 1

    def publish_status(self):
        counts = {"lidars": self.selector.counts, "cycles": self.selector.cycles, "pose_gate": self.gate.counts,
                  "odom": self.history.counts, "queued": len(self.gate.queue)}
        self.status_pub.publish(json_message({**self.status.status(self.now_ns() * 1e-9, counts),
                                              "wall_time": time.time()}))

    def close(self):
        self.log.close()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    args, ros_args = parser.parse_known_args(argv)
    config = load_config(args.config)
    from ..live.supervisor import require_live_domain
    require_live_domain(config)
    if isinstance(config.get("live"), str):
        config["live"] = json.loads(Path(config["live"]).read_text(encoding="utf-8"))
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.parameter import Parameter
    rclpy.init(args=ros_args)
    node = rclpy.create_node("wc_live_obstacles", parameter_overrides=[
        Parameter("use_sim_time", value=bool(config.get("use_sim_time", False)))])
    worker = None
    try:
        worker = ObstacleInput(node, config)
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        if rclpy.ok():
            raise
    finally:
        if worker is not None:
            worker.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
