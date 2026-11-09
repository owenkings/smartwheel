"""ROS wrapper of the safety gate: monitored command in, gated command out.

Publishes an explicit zero twist whenever the gate is not ACTIVE, at a fixed
rate, so the base watchdog always sees a fresh decision. Status JSON goes to
``/wc_nav/gate_status`` at 2 Hz and on every state change. With AMCL (the
covariance condition on), a robot held for its covariance gets a bounded number
of AMCL no-motion updates (``StationaryRelocalization``); the gate's own
conditions and limits do not change. The covariance condition's mode
(``GateConfig.covariance_mode``: lateral by default since 2026-11-09, xy on request) is assessed
on every AMCL pose with its heading.
"""
from __future__ import annotations

import argparse

from .nav2_config import CMD_GATED, CMD_MONITORED
from .ros_util import json_message, latched_qos, load_config, lookup_pose, quaternion_to_yaw, require_isolated_domain
from .safety_gate import COVARIANCE_REASONS, CorrectionMonitor, GateConfig, SafetyGate, StationaryRelocalization

NOMOTION_SERVICE = "/request_nomotion_update"


class GateNode:
    def __init__(self, node, config):
        from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
        from std_msgs.msg import Bool, String
        from tf2_ros import Buffer, TransformListener
        self.node = node
        self.gate = SafetyGate(GateConfig(**config.get("gate", {})))
        self.correction = CorrectionMonitor(**config.get("correction_monitor", {}))
        # With AMCL (covariance required) the gate asks for in-place filter updates while it holds a standing
        # robot for its covariance; see StationaryRelocalization. Off without AMCL.
        relocalization = dict(config.get("stationary_relocalization", {}))
        service = relocalization.pop("service", NOMOTION_SERVICE)
        self.relocalization = None
        if self.gate.config.require_covariance:
            self.relocalization = StationaryRelocalization(**relocalization)
            from std_srvs.srv import Empty
            self.Empty = Empty
            self.nomotion = node.create_client(Empty, service)
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
        # The heading is used by the lateral covariance mode only; the xy mode ignores it.
        self.covariance_ok, self.covariance_reason = self.gate.on_covariance(
            message.pose.covariance, quaternion_to_yaw(message.pose.pose.orientation))

    def tick(self):
        now = self.now()
        if now <= 0:
            return  # no simulated clock yet
        found = lookup_pose(self.buffer, "map", "base_link")
        correction = lookup_pose(self.buffer, "map", "odom")
        robot = lookup_pose(self.buffer, "odom", "base_link")
        correction_ok, correction_reason = (True, "")
        if correction is not None:
            if self.correction.reference == "robot":
                if robot is None:
                    correction_ok, correction_reason = False, "ODOM_TO_BASE_TF_UNAVAILABLE"
                else:
                    correction_ok, correction_reason = self.correction.update(now, correction[0], robot[0])
            else:
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
        if self.relocalization is not None:
            holding = decision.state == "LOCALIZATION_UNTRUSTED" and any(r in COVARIANCE_REASONS for r in decision.reasons)
            if self.relocalization.update(now, holding, None if robot is None else robot[0],
                                          self.nomotion.service_is_ready()):
                self.nomotion.call_async(self.Empty.Request())
                self.node.get_logger().info("covariance hold on a standing robot: AMCL no-motion update %d/%d" % (
                    self.relocalization.requests_this_stop, self.relocalization.max_requests))
        self.counts[decision.state] = self.counts.get(decision.state, 0) + 1
        for reason in decision.reasons:
            self.counts[reason] = self.counts.get(reason, 0) + 1
        if decision.state != self.last_state or now - self.last_status >= 0.5:
            status = {"time_s": round(now, 3), "state": decision.state, "reasons": list(decision.reasons),
                      "previous_state": self.last_state, "counts": self.counts,
                      "map_correction_events": self.correction.events,
                      "linear": decision.linear, "angular": decision.angular}
            if self.relocalization is not None:
                status["stationary_relocalization"] = self.relocalization.status()
            if self.gate.config.require_covariance:
                status["covariance"] = self.gate.covariance_status()
            self.status.publish(json_message(status))
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
    # Simulated time by default (simulation and recorded shadow runs); the live shadow sets use_sim_time false.
    use_sim_time = config.get("use_sim_time", True)
    if type(use_sim_time) is not bool:
        raise ValueError("use_sim_time must be a boolean")
    node = rclpy.create_node("wc_safety_gate", parameter_overrides=[Parameter("use_sim_time", value=use_sim_time)])
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
