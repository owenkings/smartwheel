"""Velocity-smoother feedback from the gate's output (``--smoother-feedback gated``; explicit option, default off).

The velocity smoother runs open loop in this stack: the speed it ramps from is
its own last output. While the safety gate or the collision monitor holds the
chair, that output stays at the commanded speed, so the release jumps from
zero to it in one step (docs/architecture/velocity_channel.md, 9.2 item 7).

With ``gated`` the smoother runs CLOSED_LOOP on ``stack.FEEDBACK_TOPIC``, which
this node fills once per smoother output (``feedback``):

- the smoother's own last output, i.e. what OPEN_LOOP uses, while the gate
  passes a command;
- zero while the gate's latest output is exactly zero (gate stop, monitor stop
  zone) or the gate has published nothing for ``stale_s``.

So after a stop the smoother starts again from rest and ramps at its own
acceleration limit. Slowdown zones keep the open-loop behaviour on purpose:
feeding back the scaled speed itself would let the smoother's output settle at
accel/rate/(1 - ratio) (0.025 m/s at 0.3 m/s^2, 20 Hz, ratio 0.4) and the chair
would crawl through every slowdown zone. The step when a slowdown zone clears
(0.12 -> 0.30 m/s) therefore stays; only a ramp after the monitor (the gate or
the wheel owner) removes it.

The node cannot speed anything up or bypass a stage: it only tells the smoother
which speed to ramp from, and the monitor and the gate still act on every
command. The supervisor ends the run if the node exits. ``log_csv`` records
each decision for the validation runs.
"""
from __future__ import annotations

import argparse
import csv
import math

STALE_S = 0.25          # the gate publishes every 0.05 s; older than this counts as no output (chair held)
ZERO = 1e-9             # the gate and the monitor's stop zone publish exact zeros
LOG_FIELDS = ["time_s", "own_linear", "own_angular", "gated_linear", "gated_angular", "gated_age_s",
              "feedback_linear", "feedback_angular"]


def feedback(own, gated, gated_age_s, stale_s=STALE_S):
    """(linear, angular) the smoother should ramp from, given its own last output and the gate's latest output.

    Zero when the gate has no recent output or its output is exactly zero (the
    chair is held), else the smoother's own output. A gate output that is only
    smaller (slowdown zone, approach model, gate speed limit) keeps ``own``.
    """
    held = (gated is None or gated_age_s is None or not math.isfinite(gated_age_s) or gated_age_s > stale_s
            or (abs(gated[0]) < ZERO and abs(gated[1]) < ZERO))
    if held:
        return (0.0, 0.0)
    return (float(own[0]), float(own[1]))


def node_config(output_dir, *, use_sim_time=True):
    """Config of one relay process; the log goes into the run directory."""
    from pathlib import Path
    return {"use_sim_time": bool(use_sim_time), "stale_s": STALE_S,
            "log_csv": str(Path(output_dir) / "velocity_feedback.csv")}


class Relay:
    def __init__(self, node, config):
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry
        from ..autonomy.nav2_config import CMD_GATED, CMD_SMOOTHED
        from .stack import FEEDBACK_TOPIC
        self.node = node
        self.Odometry = Odometry
        self.stale_s = float(config.get("stale_s", STALE_S))
        self.gated = None
        self.gated_time = None
        self.log = self.writer = None
        if config.get("log_csv"):
            self.log = open(config["log_csv"], "x", newline="", encoding="utf-8")
            self.writer = csv.writer(self.log)
            self.writer.writerow(LOG_FIELDS)
        self.pub = node.create_publisher(Odometry, FEEDBACK_TOPIC, 10)
        node.create_subscription(Twist, CMD_GATED, self.on_gated, 10)
        node.create_subscription(Twist, CMD_SMOOTHED, self.on_smoothed, 10)

    def now(self):
        return self.node.get_clock().now().nanoseconds * 1e-9

    def on_gated(self, message):
        self.gated = (float(message.linear.x), float(message.angular.z))
        self.gated_time = self.now()

    def on_smoothed(self, message):
        now = self.now()
        own = (float(message.linear.x), float(message.angular.z))
        age = None if self.gated_time is None else now - self.gated_time
        linear, angular = feedback(own, self.gated, age, self.stale_s)
        out = self.Odometry()
        out.header.stamp = self.node.get_clock().now().to_msg()
        out.header.frame_id = "odom"
        out.child_frame_id = "base_link"
        out.twist.twist.linear.x = linear
        out.twist.twist.angular.z = angular
        self.pub.publish(out)
        if self.writer is not None:
            gated = self.gated or (math.nan, math.nan)
            self.writer.writerow(["%.3f" % now, "%.4f" % own[0], "%.4f" % own[1], "%.4f" % gated[0], "%.4f" % gated[1],
                                  "" if age is None else "%.3f" % age, "%.4f" % linear, "%.4f" % angular])
            self.log.flush()

    def close(self):
        if self.log is not None:
            self.log.close()


def main(argv=None):
    from ..autonomy.ros_util import load_config, require_isolated_domain
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    args, ros_args = parser.parse_known_args(argv)
    require_isolated_domain()
    config = load_config(args.config)
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.parameter import Parameter
    rclpy.init(args=ros_args)
    node = rclpy.create_node("wc_velocity_feedback", parameter_overrides=[
        Parameter("use_sim_time", value=bool(config.get("use_sim_time", True)))])
    relay = Relay(node, config)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        relay.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
