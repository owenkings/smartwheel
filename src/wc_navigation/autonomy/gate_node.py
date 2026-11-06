"""ROS wrapper of the safety gate: monitored command in, gated command out.

Publishes an explicit zero twist whenever the gate is not ACTIVE, at a fixed
rate, so the base watchdog always sees a fresh decision. Status JSON goes to
``/wc_nav/gate_status`` at 2 Hz and on every state change.
"""
from __future__ import annotations

import argparse

from .nav2_config import CMD_GATED, CMD_MONITORED
from .ros_util import json_message, latched_qos, load_config, lookup_pose, require_isolated_domain
from .safety_gate import CorrectionMonitor, GateConfig, SafetyGate


class GateNode:
    def __init__(self, node, config):
        from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
        from std_msgs.msg import Bool, String
        from tf2_ros import Buffer, TransformListener
        self.node = node
        self.gate = SafetyGate(GateConfig(**config.get("gate", {})))
        self.correction = CorrectionMonitor(**config.get("correction_monitor", {}))
        self.Twist = Twist
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, node)
        self.out = node.create_publisher(Twist, CMD_GATED, 10)
        self.status = node.create_publisher(String, "/wc_nav/gate_status", latched_qos())
        node.create_subscription(Twist, CMD_MONITORED, self.on_command, 10)
        node.create_subscription(Bool, "/wc_nav/enable", lambda m: self.gate.on_enable(m.data, self.now()), 10)
        node.create_subscription(Bool, "/wc_nav/estop", lambda m: self.gate.on_estop(m.data, self.now()), 10)
        self.covariance_ok, self.covariance_reason = (not self.gate.config.require_covariance), "NO_COVARIANCE_YET"
        if self.gate.config.require_covariance:
            node.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self.on_pose, latched_qos())
        self.last_state = None
        self.counts = {}
        self.last_status = -1e9
        node.create_timer(0.05, self.tick)

    def now(self):
        return self.node.get_clock().now().nanoseconds * 1e-9

    def on_command(self, message):
        self.gate.on_command(message.linear.x, message.angular.z, self.now())

    def on_pose(self, message):
        self.covariance_ok, self.covariance_reason = self.gate.check_covariance(message.pose.covariance)

    def tick(self):
        now = self.now()
        if now <= 0:
            return  # no simulated clock yet
        found = lookup_pose(self.buffer, "map", "base_link")
        correction = lookup_pose(self.buffer, "map", "odom")
        correction_ok, correction_reason = (True, "")
        if correction is not None:
            correction_ok, correction_reason = self.correction.update(now, correction[0])
        if found is None:
            self.gate.on_localization(False, "MAP_TO_BASE_TF_UNAVAILABLE")
        else:
            age = now - found[1]
            if age > self.gate.config.localization_max_age_s:
                self.gate.on_localization(False, "MAP_TO_BASE_TF_STALE")
            elif not self.covariance_ok:
                self.gate.on_localization(False, self.covariance_reason)
            elif not correction_ok:
                self.gate.on_localization(False, correction_reason)
            else:
                self.gate.on_localization(True)
        decision = self.gate.evaluate(now)
        message = self.Twist()
        message.linear.x, message.angular.z = decision.linear, decision.angular
        self.out.publish(message)
        self.counts[decision.state] = self.counts.get(decision.state, 0) + 1
        for reason in decision.reasons:
            self.counts[reason] = self.counts.get(reason, 0) + 1
        if decision.state != self.last_state or now - self.last_status >= 0.5:
            self.status.publish(json_message({
                "time_s": round(now, 3), "state": decision.state, "reasons": list(decision.reasons),
                "previous_state": self.last_state, "counts": self.counts,
                "map_correction_events": self.correction.events,
                "linear": decision.linear, "angular": decision.angular}))
            self.last_state, self.last_status = decision.state, now


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    args, ros_args = parser.parse_known_args(argv)
    require_isolated_domain()
    config = load_config(args.config)
    import rclpy
    from rclpy.parameter import Parameter
    from rclpy.executors import ExternalShutdownException
    rclpy.init(args=ros_args)
    node = rclpy.create_node("wc_safety_gate", parameter_overrides=[Parameter("use_sim_time", value=True)])
    try:
        GateNode(node, config)
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
