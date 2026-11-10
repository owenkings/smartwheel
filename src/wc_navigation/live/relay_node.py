"""One-way relay of the raw sensor topics from a sensor domain into this (isolated) domain.

The relay process holds two ROS contexts. In the sensor (source) domain it has
one read-only node that subscribes, as serialized bytes, to exactly three
topics: the left lidar host-filtered frames, the H30 frames and the wheel
feedback records (best effort, volatile, keep last 3 / 100 / 20). In its own
isolated domain a second node republishes those bytes unchanged under
``/wc_live/relayed``, plus a status JSON on ``/wc_live/relay_status``. On the
vehicle the subscriptions are created only once every topic already has a
publisher and another subscriber (the panel's recorder or live mapping input):
the drivers start publishing as soon as any subscriber exists, so the relay
must not be the first one. Nothing is ever published in the source domain:

* the source node's class cannot create publishers, services or clients; the
  ``/parameter_events`` publisher rclpy creates internally is replaced by a
  stand-in that never reaches DDS; rosout logging and parameter services are
  off and global ROS arguments (remaps) do not apply to it;
* best-effort readers send no acknowledgements, take no part in the drivers'
  end-of-recording wait for all reliable readers and cannot slow the panel's
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
# Source side (the panel's domain), per window A 2026-11-08: best effort, volatile, keep last. Best-effort readers
# take no part in the drivers' end-of-recording acknowledgement wait, so the relay cannot make a recording PARTIAL.
SOURCE_DEPTH = {LEFT_FILTERED_TOPIC: 3, IMU_TOPIC: 100, WHEEL_TOPIC: 20}
TARGET_DEPTH = {LEFT_FILTERED_TOPIC: 5, IMU_TOPIC: 400, WHEEL_TOPIC: 50}
OUTPUT_PREFIX = "/wc_live/relayed"


def source_qos(topic):
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    return QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=SOURCE_DEPTH[topic],
                      reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE)


def check_relay_config(config, own_domain):
    source = int(config["source_domain"])
    prefix = config.get("source_prefix", "")
    output = config.get("output_prefix", OUTPUT_PREFIX)
    if not 0 <= source <= 232:
        raise ValueError("source_domain must be a ROS domain id")
    if str(source) == str(own_domain) and (not prefix or prefix == output):
        raise ValueError("a same-domain relay needs a source topic prefix other than the output prefix (it would loop)")
    if str(source) not in SIMULATION_DOMAINS and not vehicle_approved():
        raise ValueError("relaying from domain %d needs %s=APPROVED_BY_USER_<date>" % (source, VEHICLE_APPROVAL_ENV))
    if (prefix and not prefix.startswith("/")) or not output.startswith("/") or output == "/":
        raise ValueError("source_prefix and output_prefix must start with / (output_prefix is required)")
    if type(config.get("wait_for_panel_consumers", False)) is not bool:
        raise ValueError("wait_for_panel_consumers must be a boolean")
    return source, prefix, output


def panel_ready(counts):
    """Every relayed topic has a publisher and at least one subscriber that is not the relay.

    The drivers start publishing once any subscriber exists; waiting for the panel's own consumer (recorder or
    live mapping input) keeps the relay from being the one that releases them before the recorder has joined.
    """
    return all(publishers >= 1 and subscribers >= 1 for publishers, subscribers in counts.values())


def source_counts(node, topics):
    return {topic: (node.count_publishers(topic), node.count_subscribers(topic)) for topic in topics}


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


ENTRY_NODE = "wc_live_relay_in"
QUERY_NODE = "wc_live_relay_graph"


def _endpoints(infos):
    """Node and QoS of each endpoint (discovery carries no history depth)."""
    return sorted(({"node": (info.node_namespace.rstrip("/") + "/" + info.node_name),
                    "reliability": info.qos_profile.reliability.name, "durability": info.qos_profile.durability.name}
                   for info in infos), key=lambda e: e["node"])


def _node_graph(node, name, namespace, expected):
    return graph_check(node.get_publisher_names_and_types_by_node(name, namespace),
                       node.get_subscriber_names_and_types_by_node(name, namespace),
                       node.get_service_names_and_types_by_node(name, namespace),
                       node.get_client_names_and_types_by_node(name, namespace), expected)


def entry_graph(sensor_domain, source_prefix="", wait_s=3.0):
    """The relay entry node's graph entry and the relayed topics' endpoints, as seen in the sensor domain.

    One participant joins the sensor domain. Its node has no endpoint at all: the relay's read-only class (no
    publisher, service or client; /parameter_events stays out of DDS), no rosout, no parameter services and no
    subscriptions; everything reported comes from the local discovery cache, and the query node's own graph
    entry is checked to be empty. ``ros2 node info`` is not a read-only query: it starts the ros2 daemon, a
    node with publishers and services that stays in the domain after the command.
    """
    import rclpy
    from rclpy.context import Context
    context = Context()
    rclpy.init(args=[], context=context, domain_id=int(sensor_domain))
    node = read_only_node_class()(QUERY_NODE, context=context, enable_rosout=False, start_parameter_services=False,
                                  use_global_arguments=False)
    topics = [source_prefix + topic for topic in RELAYED_TOPICS]
    try:
        end = time.monotonic() + float(wait_s)
        entry, names = None, []
        while True:
            names = node.get_node_names_and_namespaces()
            if (ENTRY_NODE, "/") in names:
                try:
                    entry = _node_graph(node, ENTRY_NODE, "/", topics)
                except Exception as error:  # listed, endpoints not discovered yet
                    entry = {"status": "FAIL", "error": type(error).__name__ + ": " + str(error)}
                if entry["status"] == "PASS":
                    break
            if time.monotonic() >= end:
                break
            time.sleep(0.2)
        own = _node_graph(node, QUERY_NODE, "/", [])
        report = {"sensor_domain": int(sensor_domain), "source_prefix": source_prefix, "entry_node": "/" + ENTRY_NODE,
                  "entry": entry, "query_node": "/" + QUERY_NODE, "query_node_graph": own,
                  "nodes": sorted(ns.rstrip("/") + "/" + n for n, ns in names),
                  "topics": {topic: {"publishers": _endpoints(node.get_publishers_info_by_topic(topic)),
                                     "subscribers": _endpoints(node.get_subscriptions_info_by_topic(topic))}
                             for topic in topics}}
        entry_readers = [e for t in topics for e in report["topics"][t]["subscribers"] if e["node"] == "/" + ENTRY_NODE]
        best_effort = len(entry_readers) == len(topics) and all(e["reliability"] == "BEST_EFFORT" and
                                                                 e["durability"] == "VOLATILE" for e in entry_readers)
        report["entry_reads_best_effort_volatile"] = best_effort
        if entry is None:
            report["status"] = "NOT_FOUND"
        else:
            report["status"] = "PASS" if entry["status"] == "PASS" and own["status"] == "PASS" and best_effort else "FAIL"
        return report
    finally:
        node.destroy_node()
        if context.ok():
            context.shutdown()


class Relay:
    def __init__(self, source_node, target_node, prefix, output_prefix):
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        from std_msgs.msg import String
        from wc_interfaces.msg import H30Frame, SourceFrame
        types = {LEFT_FILTERED_TOPIC: SourceFrame, IMU_TOPIC: H30Frame, WHEEL_TOPIC: String}
        self.counts = {topic: 0 for topic in types}
        self.publishers = {}
        for topic, kind in types.items():
            # Renamed in the shadow domain, so the relayed copies can never be mistaken for the panel's topics.
            self.publishers[topic] = target_node.create_publisher(
                kind, output_prefix + topic, QoSProfile(depth=TARGET_DEPTH[topic], reliability=ReliabilityPolicy.RELIABLE))
            source_node.create_subscription(kind, prefix + topic, lambda data, t=topic: self.forward(t, data),
                                            source_qos(topic), raw=True)

    def forward(self, topic, data):
        self.publishers[topic].publish(data)
        self.counts[topic] += 1


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    args, ros_args = parser.parse_known_args(argv)
    config = load_config(args.config)
    require_live_domain(config)
    source, prefix, output_prefix = check_relay_config(config, os.environ["ROS_DOMAIN_ID"])
    wait_for_panel = config.get("wait_for_panel_consumers", False)
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
    topics = [prefix + topic for topic in RELAYED_TOPICS]
    base = {"source_domain": source, "source_prefix": prefix, "output_prefix": output_prefix,
            "wait_for_panel_consumers": wait_for_panel, "publishes_in_source_domain": False}
    try:
        # Vehicle: subscribe only once the panel's own consumers are there (graph queries create nothing).
        waited = time.monotonic()
        while wait_for_panel and rclpy.ok() and source_context.ok():
            counts = source_counts(source_node, topics)
            if panel_ready(counts):
                break
            status_pub.publish(json_message({**base, "state": "WAITING_FOR_PANEL_SOURCES",
                                             "waited_s": round(time.monotonic() - waited, 1),
                                             "publishers_subscribers": {t: list(c) for t, c in counts.items()}}))
            rclpy.spin_once(target_node, timeout_sec=0.0)
            time.sleep(1.0)
        relay = Relay(source_node, target_node, prefix, output_prefix)
        check = self_check(source_node, topics)
        state = "RUNNING" if check["status"] == "PASS" else "FAILED"
        last_status = 0.0
        while rclpy.ok() and source_context.ok():
            if time.monotonic() - last_status >= 1.0:
                status_pub.publish(json_message({**base, "state": state, "self_check": check, "counts": relay.counts}))
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
