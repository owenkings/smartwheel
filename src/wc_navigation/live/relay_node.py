"""One-way relay of the raw sensor topics from a sensor domain into this (isolated) domain.

The relay process holds two ROS contexts. In the sensor (source) domain it has
one read-only node that subscribes, best effort and as serialized bytes, to
exactly three topics: the left lidar host-filtered frames, the H30 frames and
the wheel feedback records. In its own isolated domain a second node
republishes those bytes unchanged, plus a status JSON on
``/wc_live/relay_status``. Nothing is ever published in the source domain:

* the source node's class cannot create publishers, services or clients; the
  ``/parameter_events`` publisher rclpy creates internally is replaced by a
  stand-in that never reaches DDS; rosout logging and parameter services are
  off and global ROS arguments (remaps) do not apply to it;
* best-effort readers send no acknowledgements and cannot slow the panel's
  publishers down;
* after subscribing, the node checks its own graph entry: no publisher, no
  service, no client and exactly the three subscriptions, or the relay exits.

On the vehicle the source is the panel's sensor domain and the relay needs
the user's dated approval in the environment; in tests the source is this
same domain with a topic prefix.
"""
from __future__ import annotations

import argparse
import os
import time

from ..autonomy.ros_util import SIMULATION_DOMAINS, json_message, latched_qos, load_config
from .config import IMU_TOPIC, LEFT_FILTERED_TOPIC, WHEEL_TOPIC
from .supervisor import VEHICLE_APPROVAL_ENV, require_live_domain, vehicle_approved

RELAYED_TOPICS = (LEFT_FILTERED_TOPIC, IMU_TOPIC, WHEEL_TOPIC)
DEPTH = {LEFT_FILTERED_TOPIC: 5, IMU_TOPIC: 400, WHEEL_TOPIC: 50}


def check_relay_config(config, own_domain):
    source = int(config["source_domain"])
    prefix = config.get("source_prefix", "")
    if not 0 <= source <= 232:
        raise ValueError("source_domain must be a ROS domain id")
    if str(source) == str(own_domain) and not prefix:
        raise ValueError("a same-domain relay needs a source topic prefix (it would loop)")
    if str(source) not in SIMULATION_DOMAINS and not vehicle_approved():
        raise ValueError("relaying from domain %d needs %s=APPROVED_BY_USER_<date>" % (source, VEHICLE_APPROVAL_ENV))
    if prefix and not prefix.startswith("/"):
        raise ValueError("source_prefix must start with /")
    return source, prefix


def read_only_node_class():
    """rclpy Node subclass that can subscribe and nothing else (see module docstring)."""
    from rcl_interfaces.msg import ParameterEvent
    from rclpy.node import Node

    class DiscardedParameterEvents:
        """Stand-in for the /parameter_events publisher rclpy creates in every node."""

        def publish(self, message):
            return None

        def destroy(self):
            return None

    class ReadOnlyNode(Node):
        def create_publisher(self, msg_type, topic, *args, **kwargs):
            if topic == "/parameter_events" and msg_type is ParameterEvent:
                return DiscardedParameterEvents()
            raise PermissionError("read-only relay node: no publisher allowed (" + str(topic) + ")")

        def create_service(self, *args, **kwargs):
            raise PermissionError("read-only relay node: no service allowed")

        def create_client(self, *args, **kwargs):
            raise PermissionError("read-only relay node: no client allowed")

    return ReadOnlyNode


def graph_check(publishers, subscribers, services, clients, expected):
    """PASS only for zero publishers, services and clients and exactly the expected subscriptions."""
    topics = sorted(name for name, _ in subscribers)
    ok = not publishers and not services and not clients and topics == sorted(expected)
    return {"status": "PASS" if ok else "FAIL", "publishers": [name for name, _ in publishers],
            "subscriptions": topics, "services": [name for name, _ in services],
            "clients": [name for name, _ in clients], "expected_subscriptions": sorted(expected)}


def self_check(node, expected, timeout_s=3.0):
    name, namespace = node.get_name(), node.get_namespace()
    result = {"status": "FAIL", "error": "graph entry of the source node not found"}
    end = time.monotonic() + timeout_s
    while time.monotonic() < end:
        try:
            result = graph_check(node.get_publisher_names_and_types_by_node(name, namespace),
                                 node.get_subscriber_names_and_types_by_node(name, namespace),
                                 node.get_service_names_and_types_by_node(name, namespace),
                                 node.get_client_names_and_types_by_node(name, namespace), expected)
            if result["status"] == "PASS":
                return result
        except Exception as error:  # the graph may not list the node yet
            result = {"status": "FAIL", "error": type(error).__name__ + ": " + str(error)}
        time.sleep(0.1)
    return result


class Relay:
    def __init__(self, source_node, target_node, prefix):
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        from std_msgs.msg import String
        from wc_interfaces.msg import H30Frame, SourceFrame
        types = {LEFT_FILTERED_TOPIC: SourceFrame, IMU_TOPIC: H30Frame, WHEEL_TOPIC: String}
        self.counts = {topic: 0 for topic in types}
        self.publishers = {}
        for topic, kind in types.items():
            self.publishers[topic] = target_node.create_publisher(
                kind, topic, QoSProfile(depth=DEPTH[topic], reliability=ReliabilityPolicy.RELIABLE))
            source_node.create_subscription(kind, prefix + topic, lambda data, t=topic: self.forward(t, data),
                                            QoSProfile(depth=DEPTH[topic], reliability=ReliabilityPolicy.BEST_EFFORT),
                                            raw=True)

    def forward(self, topic, data):
        self.publishers[topic].publish(data)
        self.counts[topic] += 1


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    args, ros_args = parser.parse_known_args(argv)
    config = load_config(args.config)
    require_live_domain(config)
    source, prefix = check_relay_config(config, os.environ["ROS_DOMAIN_ID"])
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
    from std_msgs.msg import String
    source_context = Context()
    rclpy.init(args=ros_args)
    rclpy.init(args=[], context=source_context, domain_id=source)
    target_node = rclpy.create_node("wc_live_relay_out")
    status_pub = target_node.create_publisher(String, "/wc_live/relay_status", latched_qos())
    source_node = read_only_node_class()("wc_live_relay_in", context=source_context, enable_rosout=False,
                                         start_parameter_services=False, use_global_arguments=False)
    executor = SingleThreadedExecutor(context=source_context)
    executor.add_node(source_node)
    code = 0
    try:
        relay = Relay(source_node, target_node, prefix)
        check = self_check(source_node, [prefix + topic for topic in RELAYED_TOPICS])
        state = "RUNNING" if check["status"] == "PASS" else "FAILED"
        last_status = 0.0
        while rclpy.ok() and source_context.ok():
            if time.monotonic() - last_status >= 1.0:
                status_pub.publish(json_message({"state": state, "source_domain": source, "source_prefix": prefix,
                                                 "self_check": check, "counts": relay.counts,
                                                 "publishes_in_source_domain": False}))
                rclpy.spin_once(target_node, timeout_sec=0.0)
                last_status = time.monotonic()
                if state == "FAILED":
                    code = 3
                    break
            executor.spin_once(timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown(timeout_sec=1.0)
        source_node.destroy_node()
        target_node.destroy_node()
        if source_context.ok():
            source_context.shutdown()
        if rclpy.ok():
            rclpy.shutdown()
    return code


if __name__ == "__main__":
    raise SystemExit(main())
