"""ROS wrapper of the safety gate: monitored command in, gated command out.

Publishes an explicit zero twist whenever the gate is not ACTIVE, at a fixed
rate, so the base watchdog always sees a fresh decision. The same decision goes
to ``/wc_nav/gate_command`` as JSON for window A's base bridge (schema
wc_gate_command_v1, velocity_channel.md 9.2 item 1). Status JSON goes to
``/wc_nav/gate_status`` at 2 Hz and on every state change. With AMCL (the
covariance condition on), a robot held for its covariance gets a bounded number
of AMCL no-motion updates (``StationaryRelocalization``); the gate's own
conditions and limits do not change. The covariance condition's mode
(``GateConfig.covariance_mode``: lateral by default since 2026-11-09, xy on request) is assessed
on every AMCL pose with its heading.

Vehicle closed loop (D.md 2026-11-10 16:45; all off by default, simulation and shadow runs unchanged):
- ``clock: monotonic`` in the node configuration: the enable and command ages and the cycle timer run on the
  monotonic clock. TF stamps are node-clock times, so the map->base_link age stays on the node clock, and a stamp
  more than 0.05 s ahead of it counts as stale too.
- ``GateConfig.require_base``: /wc_nav/base_status (window A's bridge) must report automatic control within
  ``base_status_timeout_s``, judged by the bridge's own monotonic stamp, else BASE_NOT_READY.
- ``GateConfig.enable_source_node``: /wc_nav/enable and /wc_nav/estop must each have exactly that node as their
  only publisher, else DISABLED with ENABLE_SOURCE_AMBIGUOUS; /wc_nav/estop is then read transient local, like the
  bridge publishes it, so a pressed emergency stop is known at once.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import time
import uuid

from .nav2_config import CMD_GATED, CMD_MONITORED
from .ros_util import json_message, latched_qos, lookup_pose, quaternion_to_yaw, require_isolated_domain
from .safety_gate import (COVARIANCE_REASONS, GATE_COMMAND_TOPIC, CorrectionMonitor, GateConfig, SafetyGate,
                          StationaryRelocalization, gate_command)

NOMOTION_SERVICE = "/request_nomotion_update"
BASE_STATUS_TOPIC = "/wc_nav/base_status"
BASE_STATUS_SCHEMA = "wc_base_status_relay_v1"     # window A's base bridge (wc_motion.auto_bridge.RELAY_SCHEMA)
SOURCE_TOPICS = ("/wc_nav/enable", "/wc_nav/estop")
CLOCKS = ("ros", "monotonic")
TF_FUTURE_TOLERANCE_S = 0.05       # monotonic clock mode: a map->base_link stamp further ahead of now is stale
BASE_FUTURE_TOLERANCE_NS = 50_000_000   # a base status stamped further ahead is not on this monotonic clock


def base_status_verdict(status, mono_ns, timeout_s):
    """(ok, reason) of the last /wc_nav/base_status for GateConfig.require_base.

    ``status`` is the parsed JSON (None before the first one). Its ``stamp_mono_ns`` is the bridge's
    time.monotonic_ns() when it sent the status, comparable with ``mono_ns`` on the same machine.
    """
    if status is None:
        return False, "BASE_STATUS_STALE"
    if not isinstance(status, dict) or status.get("schema") != BASE_STATUS_SCHEMA:
        return False, "BASE_STATUS_INVALID"
    stamp, auto = status.get("stamp_mono_ns"), status.get("base_auto")
    if type(stamp) is not int or type(auto) is not bool or stamp - mono_ns > BASE_FUTURE_TOLERANCE_NS:
        return False, "BASE_STATUS_INVALID"
    if (mono_ns - stamp) * 1e-9 > timeout_s:
        return False, "BASE_STATUS_STALE"
    if not auto:
        reason = status.get("reason")
        return False, reason if reason in ("BASE_NOT_AUTO", "BASE_STATUS_STALE") else "BASE_NOT_AUTO"
    return True, ""


def endpoint_name(info):
    """Fully qualified node name of a graph endpoint (TopicEndpointInfo)."""
    return info.node_namespace.rstrip("/") + "/" + info.node_name


def single_source(publishers, node_name):
    """Exactly one publisher, and it is ``node_name``."""
    return list(publishers) == [node_name]


class GateNode:
    def __init__(self, node, config, config_sha256=None):
        from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from std_msgs.msg import Bool, String
        from tf2_ros import Buffer, TransformListener
        self.node = node
        self.gate = SafetyGate(GateConfig(**config.get("gate", {})))
        cfg = self.gate.config
        self.clock = config.get("clock", "ros")
        if self.clock not in CLOCKS:
            raise ValueError("clock must be one of " + ", ".join(CLOCKS))
        self.correction = CorrectionMonitor(**config.get("correction_monitor", {}))
        # With AMCL (covariance required) the gate asks for in-place filter updates while it holds a standing
        # robot for its covariance; see StationaryRelocalization. Off without AMCL.
        relocalization = dict(config.get("stationary_relocalization", {}))
        service = relocalization.pop("service", NOMOTION_SERVICE)
        self.relocalization = None
        if cfg.require_covariance:
            self.relocalization = StationaryRelocalization(**relocalization)
            from std_srvs.srv import Empty
            self.Empty = Empty
            self.nomotion = node.create_client(Empty, service)
        self.Twist = Twist
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, node)
        self.out = node.create_publisher(Twist, CMD_GATED, 10)
        # The base bridge reads it reliable, keep last 1, volatile; a late decision is dropped there, not queued.
        self.command_out = node.create_publisher(String, GATE_COMMAND_TOPIC, QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE))
        self.gate_id = "gate_" + uuid.uuid4().hex[:12]     # one source for the whole run; a restart is a new one
        self.seq = 0
        self.config_sha256 = config_sha256
        self.status = node.create_publisher(String, "/wc_nav/gate_status", latched_qos())
        node.create_subscription(Twist, CMD_MONITORED, self.on_command, 10)
        node.create_subscription(Bool, "/wc_nav/enable", lambda m: self.gate.on_enable(m.data, self.age_now()), 10)
        node.create_subscription(Bool, "/wc_nav/estop", lambda m: self.gate.on_estop(m.data, self.age_now()),
                                 latched_qos() if cfg.enable_source_node else 10)
        self.base_status, self.base_messages = None, 0
        if cfg.require_base:
            node.create_subscription(String, BASE_STATUS_TOPIC, self.on_base_status, 10)
        self.sources = {}
        self.covariance_ok, self.covariance_reason = (not cfg.require_covariance), "NO_COVARIANCE_YET"
        if cfg.require_covariance:
            node.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self.on_pose, latched_qos())
        self.last_state = None
        self.counts = {}
        self.last_status = -1e9
        if self.clock == "monotonic":
            from rclpy.clock import Clock, ClockType
            node.create_timer(0.05, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
        else:
            node.create_timer(0.05, self.tick)

    def now(self):
        return self.node.get_clock().now().nanoseconds * 1e-9

    def age_now(self):
        """Clock of the enable and command ages: monotonic in the closed loop, else the node clock."""
        return time.monotonic() if self.clock == "monotonic" else self.now()

    def on_command(self, message):
        self.gate.on_command(message.linear.x, message.angular.z, self.age_now())

    def on_pose(self, message):
        # The heading is used by the lateral covariance mode only; the xy mode ignores it.
        self.covariance_ok, self.covariance_reason = self.gate.on_covariance(
            message.pose.covariance, quaternion_to_yaw(message.pose.pose.orientation))

    def on_base_status(self, message):
        self.base_messages += 1
        try:
            self.base_status = json.loads(message.data)
        except ValueError:
            self.base_status = {"schema": None}

    def tick(self):
        stamp_ns = self.node.get_clock().now().nanoseconds
        now = stamp_ns * 1e-9
        if now <= 0:
            return  # no simulated clock yet
        age_now = self.age_now()
        cfg = self.gate.config
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
            if age > cfg.localization_max_age_s or (self.clock == "monotonic" and age < -TF_FUTURE_TOLERANCE_S):
                self.gate.on_localization(False, "MAP_TO_BASE_TF_STALE")
            elif not self.covariance_ok:
                self.gate.on_localization(False, self.covariance_reason)
            elif not correction_ok:
                self.gate.on_localization(False, correction_reason)
            else:
                self.gate.on_localization(True)
        if cfg.require_base:
            self.gate.on_base(*base_status_verdict(self.base_status, time.monotonic_ns(), cfg.base_status_timeout_s))
        if cfg.enable_source_node:
            self.sources = {topic: [endpoint_name(e) for e in self.node.get_publishers_info_by_topic(topic)]
                            for topic in SOURCE_TOPICS}
            self.gate.on_sources(all(single_source(names, cfg.enable_source_node) for names in self.sources.values()))
        decision = self.gate.evaluate(age_now)
        message = self.Twist()
        message.linear.x, message.angular.z = decision.linear, decision.angular
        self.out.publish(message)
        self.seq += 1
        self.command_out.publish(json_message(gate_command(decision, cfg, self.gate_id, self.seq, stamp_ns,
                                                           time.monotonic_ns(), self.config_sha256)))
        if self.relocalization is not None:
            holding = decision.state == "LOCALIZATION_UNTRUSTED" and any(r in COVARIANCE_REASONS for r in decision.reasons)
            if self.relocalization.update(age_now, holding, None if robot is None else robot[0],
                                          self.nomotion.service_is_ready()):
                self.nomotion.call_async(self.Empty.Request())
                self.node.get_logger().info("covariance hold on a standing robot: AMCL no-motion update %d/%d" % (
                    self.relocalization.requests_this_stop, self.relocalization.max_requests))
        self.counts[decision.state] = self.counts.get(decision.state, 0) + 1
        for reason in decision.reasons:
            self.counts[reason] = self.counts.get(reason, 0) + 1
        if decision.state != self.last_state or age_now - self.last_status >= 0.5:
            # 2 Hz and on every state change; the preview's own changes show at the next 2 Hz status
            status = {"time_s": round(now, 3), "state": decision.state, "reasons": list(decision.reasons),
                      "previous_state": self.last_state, "counts": self.counts,
                      "map_correction_events": self.correction.events,
                      "linear": decision.linear, "angular": decision.angular,
                      "linear_ceiling": decision.linear_ceiling, "gate_id": self.gate_id, "seq": self.seq,
                      "clock": self.clock, "conditions": self.gate.conditions(),
                      "command": self.gate.command_status(age_now)}
            # What it would pass with the operator's enable, the base and the sources in order: in the closed
            # loop the standing chair before "start automatic" shows the shadow's view (BOARD 2026-11-10 14:40).
            preview = self.gate.preview(age_now)
            status["preview"] = {"state": preview.state, "reasons": list(preview.reasons),
                                 "linear": preview.linear, "angular": preview.angular}
            if self.relocalization is not None:
                status["stationary_relocalization"] = self.relocalization.status()
            if cfg.require_covariance:
                status["covariance"] = self.gate.covariance_status()
            if cfg.require_base:
                base = self.base_status if isinstance(self.base_status, dict) else {}
                status["base"] = {"messages": self.base_messages, "auto_state": base.get("auto_state"),
                                  "enable": base.get("enable"), "estop": base.get("estop"),
                                  "enable_source": base.get("enable_source")}
            if cfg.enable_source_node:
                status["sources"] = {"node": cfg.enable_source_node, "publishers": self.sources}
            self.status.publish(json_message(status))
            self.last_state, self.last_status = decision.state, age_now


def read_config(path):
    """Node configuration and the sha256 of its file, read once (gate_command's config_sha256)."""
    raw = Path(path).read_bytes()
    value = json.loads(raw.decode("utf-8"))
    if not isinstance(value, dict):
        raise ValueError("node configuration must be a JSON object")
    return value, hashlib.sha256(raw).hexdigest()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    args, ros_args = parser.parse_known_args(argv)
    require_isolated_domain()
    config, config_sha256 = read_config(args.config)
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
        GateNode(node, config, config_sha256=config_sha256)
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
