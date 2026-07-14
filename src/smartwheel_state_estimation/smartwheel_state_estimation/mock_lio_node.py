import math
from dataclasses import dataclass

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from smartwheel_interfaces.msg import SimMotion


@dataclass(frozen=True)
class LocalPose:
    x: float
    y: float
    yaw: float
    linear: float
    angular: float


@dataclass(frozen=True)
class MotionRates:
    stamp_sec: float
    linear_mps: float
    angular_rps: float


class CumulativeMotionDecoder:
    """Recover motion rates without corrupting distance when messages are dropped."""

    def __init__(self) -> None:
        self._sequence = None
        self._stamp = None
        self._distance = None
        self._yaw = None

    def update(
        self,
        sequence: int,
        stamp_sec: float,
        cumulative_distance_m: float,
        cumulative_yaw_rad: float,
    ) -> MotionRates | None:
        values = (stamp_sec, cumulative_distance_m, cumulative_yaw_rad)
        if not all(math.isfinite(value) for value in values):
            raise ValueError("mock LIO cumulative motion must be finite")
        if self._sequence is None:
            self._sequence = sequence
            self._stamp = stamp_sec
            self._distance = cumulative_distance_m
            self._yaw = cumulative_yaw_rad
            return None
        if sequence <= self._sequence:
            raise ValueError("mock LIO motion sequence must be strictly increasing")
        dt = stamp_sec - self._stamp
        if dt <= 0.0:
            raise ValueError("mock LIO timestamps must be strictly increasing")
        result = MotionRates(
            stamp_sec,
            (cumulative_distance_m - self._distance) / dt,
            (cumulative_yaw_rad - self._yaw) / dt,
        )
        self._sequence = sequence
        self._stamp = stamp_sec
        self._distance = cumulative_distance_m
        self._yaw = cumulative_yaw_rad
        return result


class MockLioIntegrator:
    """Deterministic local-odometry error model driven by synthetic motion increments."""

    def __init__(
        self,
        seed: int,
        linear_scale: float,
        angular_scale: float,
        linear_bias_mps: float,
        yaw_bias_rps: float,
        linear_noise_stddev_mps: float,
        angular_noise_stddev_rps: float,
    ) -> None:
        if linear_scale <= 0.0 or angular_scale <= 0.0:
            raise ValueError("mock LIO scales must be positive")
        if linear_noise_stddev_mps < 0.0 or angular_noise_stddev_rps < 0.0:
            raise ValueError("mock LIO noise standard deviations must be non-negative")
        self._rng = np.random.default_rng(seed)
        self._linear_scale = linear_scale
        self._angular_scale = angular_scale
        self._linear_bias = linear_bias_mps
        self._yaw_bias = yaw_bias_rps
        self._linear_noise = linear_noise_stddev_mps
        self._angular_noise = angular_noise_stddev_rps
        self._stamp = None
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0

    def update(self, stamp_sec: float, linear: float, angular: float) -> LocalPose:
        if not all(math.isfinite(value) for value in (stamp_sec, linear, angular)):
            raise ValueError("mock LIO inputs must be finite")
        if self._stamp is None:
            self._stamp = stamp_sec
            return LocalPose(self._x, self._y, self._yaw, 0.0, 0.0)
        dt = stamp_sec - self._stamp
        if dt <= 0.0:
            raise ValueError("mock LIO timestamps must be strictly increasing")
        measured_linear = (
            linear * self._linear_scale
            + self._linear_bias
            + float(self._rng.normal(0.0, self._linear_noise))
        )
        measured_angular = (
            angular * self._angular_scale
            + self._yaw_bias
            + float(self._rng.normal(0.0, self._angular_noise))
        )
        dyaw = measured_angular * dt
        distance = measured_linear * dt
        heading_mid = self._yaw + 0.5 * dyaw
        self._x += distance * math.cos(heading_mid)
        self._y += distance * math.sin(heading_mid)
        self._yaw = math.atan2(math.sin(self._yaw + dyaw), math.cos(self._yaw + dyaw))
        self._stamp = stamp_sec
        return LocalPose(self._x, self._y, self._yaw, measured_linear, measured_angular)


class MockLioNode(Node):
    def __init__(self) -> None:
        super().__init__("mock_lio")
        self.declare_parameter("input_topic", "/sim/mock_lio/motion")
        self.declare_parameter("output_topic", "/lio/odom")
        self.declare_parameter("path_topic", "/lio/path")
        self.declare_parameter("odom_frame_id", "odom")
        self.declare_parameter("base_frame_id", "base_link")
        self.declare_parameter("max_path_poses", 5000)
        self.declare_parameter("seed", 20260714)
        self.declare_parameter("linear_scale", 0.997)
        self.declare_parameter("angular_scale", 1.002)
        self.declare_parameter("linear_bias_mps", 0.0005)
        self.declare_parameter("yaw_bias_rps", 0.0008)
        self.declare_parameter("linear_noise_stddev_mps", 0.002)
        self.declare_parameter("angular_noise_stddev_rps", 0.003)
        self.declare_parameter("drop_every_n", 0)
        self._frame = str(self.get_parameter("odom_frame_id").value)
        self._base_frame = str(self.get_parameter("base_frame_id").value)
        self._max_path = int(self.get_parameter("max_path_poses").value)
        self._drop_every = int(self.get_parameter("drop_every_n").value)
        self._input_count = 0
        self._motion_decoder = CumulativeMotionDecoder()
        self._integrator = MockLioIntegrator(
            seed=int(self.get_parameter("seed").value),
            linear_scale=float(self.get_parameter("linear_scale").value),
            angular_scale=float(self.get_parameter("angular_scale").value),
            linear_bias_mps=float(self.get_parameter("linear_bias_mps").value),
            yaw_bias_rps=float(self.get_parameter("yaw_bias_rps").value),
            linear_noise_stddev_mps=float(self.get_parameter("linear_noise_stddev_mps").value),
            angular_noise_stddev_rps=float(self.get_parameter("angular_noise_stddev_rps").value),
        )
        self._odom_pub = self.create_publisher(
            Odometry, str(self.get_parameter("output_topic").value), 20
        )
        self._path_pub = self.create_publisher(Path, str(self.get_parameter("path_topic").value), 5)
        self._path = Path()
        self._path.header.frame_id = self._frame
        self.create_subscription(
            SimMotion, str(self.get_parameter("input_topic").value), self._on_motion, 20
        )
        self.get_logger().warning(
            "mock-LIO integrates synthetic local motion with deterministic drift; "
            "it is not FAST-LIO2 performance evidence"
        )

    @staticmethod
    def _stamp_seconds(stamp) -> float:
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    def _on_motion(self, source: SimMotion) -> None:
        self._input_count += 1
        try:
            stamp_sec = self._stamp_seconds(source.stamp)
            rates = self._motion_decoder.update(
                int(source.sequence),
                stamp_sec,
                float(source.cumulative_distance_m),
                float(source.cumulative_yaw_rad),
            )
            if rates is None:
                pose = self._integrator.update(stamp_sec, 0.0, 0.0)
            else:
                pose = self._integrator.update(
                    rates.stamp_sec,
                    rates.linear_mps,
                    rates.angular_rps,
                )
        except ValueError as exc:
            self.get_logger().error(str(exc))
            return
        if self._drop_every > 0 and self._input_count % self._drop_every == 0:
            return
        odom = Odometry()
        odom.header.stamp = source.stamp
        odom.header.frame_id = self._frame
        odom.child_frame_id = self._base_frame
        odom.pose.pose.position.x = pose.x
        odom.pose.pose.position.y = pose.y
        odom.pose.pose.orientation.z = math.sin(pose.yaw * 0.5)
        odom.pose.pose.orientation.w = math.cos(pose.yaw * 0.5)
        odom.twist.twist.linear.x = pose.linear
        odom.twist.twist.angular.z = pose.angular
        self._odom_pub.publish(odom)
        stamped_pose = PoseStamped()
        stamped_pose.header = odom.header
        stamped_pose.pose = odom.pose.pose
        self._path.poses.append(stamped_pose)
        if len(self._path.poses) > self._max_path:
            self._path.poses = self._path.poses[-self._max_path :]
        self._path.header.stamp = odom.header.stamp
        self._path_pub.publish(self._path)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MockLioNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
