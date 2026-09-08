from __future__ import annotations

from typing import Iterable

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from tf2_msgs.msg import TFMessage


def _frame_key(transform) -> tuple[str, str]:
    parent = str(transform.header.frame_id).lstrip("/")
    child = str(transform.child_frame_id).lstrip("/")
    return parent, child


def merge_static_transforms(existing: Iterable, incoming: Iterable) -> list:
    """Merge static transforms by parent/child while preserving insertion order."""
    merged = {}
    for transform in existing:
        merged[_frame_key(transform)] = transform
    for transform in incoming:
        merged[_frame_key(transform)] = transform
    return list(merged.values())


class OfflineStaticTfRelayNode(Node):
    """Re-publish bagged static TF with the transient-local contract TF2 expects."""

    def __init__(self) -> None:
        super().__init__("offline_static_tf_relay")
        self.declare_parameter("input_topic", "/offline/recorded_tf_static")
        self.declare_parameter("output_topic", "/tf_static")
        self.declare_parameter("republish_period_sec", 0.5)

        input_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        output_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._publisher = self.create_publisher(
            TFMessage, str(self.get_parameter("output_topic").value), output_qos
        )
        self._transforms = {}
        self.create_subscription(
            TFMessage,
            str(self.get_parameter("input_topic").value),
            self._on_message,
            input_qos,
        )
        self._timer = self.create_timer(
            float(self.get_parameter("republish_period_sec").value),
            self._republish,
        )

    def _on_message(self, message: TFMessage) -> None:
        for transform in message.transforms:
            self._transforms[_frame_key(transform)] = transform
        self._republish()

    def _republish(self) -> None:
        if not self._transforms:
            return
        self._publisher.publish(TFMessage(transforms=list(self._transforms.values())))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OfflineStaticTfRelayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
