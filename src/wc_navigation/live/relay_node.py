"""One-way relay of the raw sensor topics from a sensor domain into this (isolated) domain.

Subscribes to the left lidar host-filtered frames, the H30 frames and the wheel
records as serialized messages in ``source_domain`` and republishes them, bytes
unchanged, in the node's own domain. Nothing is ever published in the source
domain and no other topic crosses. On the vehicle the source is the panel's
sensor domain and the relay needs the user's dated approval in the
environment; in tests the source is this same domain with a topic prefix.
"""
from __future__ import annotations

import argparse
import os

from ..autonomy.ros_util import SIMULATION_DOMAINS, load_config
from .config import IMU_TOPIC, LEFT_FILTERED_TOPIC, WHEEL_TOPIC
from .supervisor import VEHICLE_APPROVAL_ENV, require_live_domain, vehicle_approved

QOS = {LEFT_FILTERED_TOPIC: ("best_effort", 5), IMU_TOPIC: ("best_effort", 400), WHEEL_TOPIC: ("reliable", 50)}


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


class Relay:
    def __init__(self, source_node, target_node, prefix):
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        from std_msgs.msg import String
        from wc_interfaces.msg import H30Frame, SourceFrame
        types = {LEFT_FILTERED_TOPIC: SourceFrame, IMU_TOPIC: H30Frame, WHEEL_TOPIC: String}
        self.counts = {topic: 0 for topic in types}
        self.publishers = {}
        for topic, kind in types.items():
            reliability, depth = QOS[topic]
            subscribe = QoSProfile(depth=depth, reliability=(ReliabilityPolicy.BEST_EFFORT if reliability == "best_effort"
                                                             else ReliabilityPolicy.RELIABLE))
            publisher = target_node.create_publisher(kind, topic, QoSProfile(depth=depth,
                                                                              reliability=ReliabilityPolicy.RELIABLE))
            self.publishers[topic] = publisher
            source_node.create_subscription(kind, prefix + topic, lambda data, t=topic: self.forward(t, data),
                                            subscribe, raw=True)

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
    source_context = Context()
    rclpy.init(args=ros_args)
    rclpy.init(args=[], context=source_context, domain_id=source)
    target_node = rclpy.create_node("wc_live_relay_out")
    source_node = rclpy.create_node("wc_live_relay_in", context=source_context)
    executor = SingleThreadedExecutor(context=source_context)
    executor.add_node(source_node)
    try:
        Relay(source_node, target_node, prefix)
        while rclpy.ok() and source_context.ok():
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
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
