import json
import math
import time
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from wheelchair_navigation.named_goal_store import NamedGoalStore, yaw_to_quaternion
from wheelchair_navigation.startup_localization import (
    StartupPoseTarget,
    fixed_target,
    named_goal_target,
    normalize_mode,
    pose_matches,
    validated_covariance,
)


def default_named_goals_path() -> str:
    source_candidate = Path(__file__).resolve().parents[1] / "config" / "named_goals.yaml"
    if source_candidate.exists():
        return str(source_candidate)
    return str(
        Path(get_package_share_directory("wheelchair_navigation"))
        / "config"
        / "named_goals.yaml"
    )


def quaternion_to_yaw(orientation) -> float:
    values = (
        float(orientation.x),
        float(orientation.y),
        float(orientation.z),
        float(orientation.w),
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError("orientation must contain finite values")
    norm = math.sqrt(sum(value * value for value in values))
    if norm <= 1e-9:
        raise ValueError("orientation quaternion must be non-zero")
    x, y, z, w = (value / norm for value in values)
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny, cosy)


class StartupLocalizationNode(Node):
    def __init__(self):
        super().__init__("startup_localization_node")
        self.declare_parameter("mode", "disabled")
        self.declare_parameter("named_goals_path", default_named_goals_path())
        self.declare_parameter("named_goal_name", "charging")
        self.declare_parameter("fixed_x", 0.0)
        self.declare_parameter("fixed_y", 0.0)
        self.declare_parameter("fixed_yaw", 0.0)
        self.declare_parameter("anchor_topic", "/localization/anchor_pose")
        self.declare_parameter("initialpose_topic", "/initialpose")
        self.declare_parameter("amcl_pose_topic", "/amcl_pose")
        self.declare_parameter("status_topic", "/localization/startup_status")
        self.declare_parameter("startup_delay_sec", 1.0)
        self.declare_parameter("retry_period_sec", 1.0)
        self.declare_parameter("max_attempts", 8)
        self.declare_parameter("ack_tolerance_m", 0.75)
        self.declare_parameter("ack_tolerance_yaw", 1.0)
        self.declare_parameter("covariance_x", 0.25)
        self.declare_parameter("covariance_y", 0.25)
        self.declare_parameter("covariance_yaw", 0.0685)

        self.mode = str(self.get_parameter("mode").value).strip().lower()
        self.retry_period_sec = max(
            0.1, float(self.get_parameter("retry_period_sec").value)
        )
        self.max_attempts = max(1, int(self.get_parameter("max_attempts").value))
        self.ack_tolerance_m = max(
            0.0, float(self.get_parameter("ack_tolerance_m").value)
        )
        self.ack_tolerance_yaw = max(
            0.0, float(self.get_parameter("ack_tolerance_yaw").value)
        )
        self.default_covariance = [0.0] * 36
        self.default_covariance[0] = self._nonnegative_parameter("covariance_x")
        self.default_covariance[7] = self._nonnegative_parameter("covariance_y")
        self.default_covariance[35] = self._nonnegative_parameter("covariance_yaw")
        self.target = None
        self.state = "STARTING"
        self.reason = "initializing"
        self.attempts = 0
        self.last_distance_m = None
        self.last_yaw_error_rad = None
        self.next_attempt_at = time.monotonic() + max(
            0.0, float(self.get_parameter("startup_delay_sec").value)
        )

        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.initialpose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            str(self.get_parameter("initialpose_topic").value),
            10,
        )
        self.status_pub = self.create_publisher(
            String,
            str(self.get_parameter("status_topic").value),
            latched_qos,
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            str(self.get_parameter("amcl_pose_topic").value),
            self.on_amcl_pose,
            10,
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            str(self.get_parameter("anchor_topic").value),
            self.on_external_anchor,
            10,
        )
        self._configure_target()
        self.create_timer(0.1, self.on_timer)

    def _nonnegative_parameter(self, name: str) -> float:
        value = float(self.get_parameter(name).value)
        if not math.isfinite(value) or value < 0.0:
            raise ValueError(f"{name} must be finite and non-negative")
        return value

    def _configure_target(self):
        try:
            self.mode = normalize_mode(self.mode)
            if self.mode == "disabled":
                self.set_state("DISABLED", "startup localization is disabled")
                return
            if self.mode == "external_anchor":
                self.set_state("WAITING_FOR_ANCHOR", "waiting for a map-frame anchor pose")
                return
            if self.mode == "fixed":
                self.target = fixed_target(
                    self.get_parameter("fixed_x").value,
                    self.get_parameter("fixed_y").value,
                    self.get_parameter("fixed_yaw").value,
                )
            else:
                name = str(self.get_parameter("named_goal_name").value)
                store = NamedGoalStore(
                    str(self.get_parameter("named_goals_path").value)
                )
                goal = store.get_goal(name)
                if goal is None:
                    raise ValueError(f"named goal {name!r} was not found")
                self.target = named_goal_target(name, goal)
            self.set_state("READY", f"target loaded from {self.target.source}")
        except Exception as exc:
            self.target = None
            self.set_state("ERROR", str(exc))
            self.get_logger().error(f"startup localization configuration failed: {exc}")

    def set_state(self, state: str, reason: str):
        self.state = state
        self.reason = reason
        self.publish_status()

    def publish_status(self):
        payload = {
            "state": self.state,
            "mode": self.mode,
            "reason": self.reason,
            "attempts": self.attempts,
            "max_attempts": self.max_attempts,
            "acknowledged": self.state == "LOCALIZED",
            "target": None,
            "distance_m": self.last_distance_m,
            "yaw_error_rad": self.last_yaw_error_rad,
        }
        if self.target is not None:
            payload["target"] = {
                "x": self.target.x,
                "y": self.target.y,
                "yaw": self.target.yaw,
                "frame_id": self.target.frame_id,
                "source": self.target.source,
            }
        message = String()
        message.data = json.dumps(payload, sort_keys=True)
        self.status_pub.publish(message)

    def _initial_pose_message(self):
        message = PoseWithCovarianceStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.target.frame_id
        message.pose.pose.position.x = self.target.x
        message.pose.pose.position.y = self.target.y
        quaternion = yaw_to_quaternion(self.target.yaw)
        message.pose.pose.orientation.x = quaternion["x"]
        message.pose.pose.orientation.y = quaternion["y"]
        message.pose.pose.orientation.z = quaternion["z"]
        message.pose.pose.orientation.w = quaternion["w"]
        message.pose.covariance = list(
            self.target.covariance or tuple(self.default_covariance)
        )
        return message

    def on_timer(self):
        if self.state not in ("READY", "INITIALIZING") or self.target is None:
            return
        now = time.monotonic()
        if now < self.next_attempt_at:
            return
        if self.attempts >= self.max_attempts:
            self.set_state(
                "FAILED",
                "AMCL did not acknowledge the startup pose within the retry limit",
            )
            return
        self.initialpose_pub.publish(self._initial_pose_message())
        self.attempts += 1
        self.next_attempt_at = now + self.retry_period_sec
        self.set_state(
            "INITIALIZING",
            f"published startup pose attempt {self.attempts}/{self.max_attempts}",
        )

    def on_amcl_pose(self, message):
        if self.target is None or self.attempts == 0 or self.state == "LOCALIZED":
            return
        frame_id = str(message.header.frame_id).strip()
        if frame_id and frame_id != "map":
            return
        try:
            yaw = quaternion_to_yaw(message.pose.pose.orientation)
            matches, distance, yaw_error = pose_matches(
                self.target,
                message.pose.pose.position.x,
                message.pose.pose.position.y,
                yaw,
                self.ack_tolerance_m,
                self.ack_tolerance_yaw,
            )
        except ValueError as exc:
            self.get_logger().warning(f"ignoring invalid AMCL pose: {exc}")
            return
        self.last_distance_m = distance
        self.last_yaw_error_rad = yaw_error
        if matches:
            self.set_state("LOCALIZED", "AMCL acknowledged the startup pose")

    def on_external_anchor(self, message):
        if self.mode != "external_anchor":
            return
        try:
            frame_id = str(message.header.frame_id).strip()
            if frame_id != "map":
                raise ValueError("external anchor must use frame_id 'map'")
            covariance = validated_covariance(message.pose.covariance)
            target = fixed_target(
                message.pose.pose.position.x,
                message.pose.pose.position.y,
                quaternion_to_yaw(message.pose.pose.orientation),
                source="external_anchor",
            )
            self.target = StartupPoseTarget(
                x=target.x,
                y=target.y,
                yaw=target.yaw,
                source=target.source,
                frame_id="map",
                covariance=covariance,
            )
        except ValueError as exc:
            self.target = None
            self.set_state("WAITING_FOR_ANCHOR", f"rejected anchor: {exc}")
            return
        self.attempts = 0
        self.last_distance_m = None
        self.last_yaw_error_rad = None
        self.next_attempt_at = time.monotonic()
        self.set_state("READY", "accepted external anchor pose")


def main(args=None):
    rclpy.init(args=args)
    node = StartupLocalizationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
