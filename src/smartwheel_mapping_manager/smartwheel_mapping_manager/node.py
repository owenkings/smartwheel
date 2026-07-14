import shutil
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Imu, PointCloud2
from smartwheel_interfaces.msg import MappingStatus, WheelEncoder
from smartwheel_interfaces.srv import MapTask
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener

from smartwheel_mapping_manager.state_machine import MappingState, MappingStateMachine
from smartwheel_sensor_api import TimestampMonitor, pointcloud2_to_xyz, validate_pointcloud_fields


class MappingManagerNode(Node):
    def __init__(self) -> None:
        super().__init__("smartwheel_mapping_manager")
        self.declare_parameter("map_name", "stage_a_sim")
        self.declare_parameter("auto_start", True)
        self.declare_parameter("checking_timeout_sec", 8.0)
        self.declare_parameter("minimum_free_disk_gb", 1.0)
        self.declare_parameter("record_bag", False)
        self.declare_parameter("bag_path", "")
        self.declare_parameter("minimum_lidar_rate_hz", 2.0)
        self.declare_parameter("minimum_imu_rate_hz", 20.0)
        self.declare_parameter("minimum_wheel_rate_hz", 5.0)
        self.declare_parameter("maximum_lidar_imu_delta_sec", 0.25)
        self.declare_parameter("maximum_dual_lidar_delta_sec", 0.03)
        self.declare_parameter("maximum_point_coordinate_m", 100.0)
        self._map_name = str(self.get_parameter("map_name").value)
        self._timeout = float(self.get_parameter("checking_timeout_sec").value)
        self._minimum_disk = float(self.get_parameter("minimum_free_disk_gb").value)
        self._record_bag = bool(self.get_parameter("record_bag").value)
        self._bag_path = Path(str(self.get_parameter("bag_path").value)).expanduser()
        self._minimum_rates = {
            "left": float(self.get_parameter("minimum_lidar_rate_hz").value),
            "right": float(self.get_parameter("minimum_lidar_rate_hz").value),
            "imu": float(self.get_parameter("minimum_imu_rate_hz").value),
            "wheel": float(self.get_parameter("minimum_wheel_rate_hz").value),
        }
        self._max_lidar_imu_delta = float(self.get_parameter("maximum_lidar_imu_delta_sec").value)
        self._max_dual_delta = float(self.get_parameter("maximum_dual_lidar_delta_sec").value)
        self._max_coordinate = float(self.get_parameter("maximum_point_coordinate_m").value)
        self._machine = MappingStateMachine()
        self._checks = {}
        self._seen = {"left": 0, "right": 0, "imu": 0, "wheel": 0}
        self._monitors = {name: TimestampMonitor() for name in self._seen}
        self._first_stamp = {name: None for name in self._seen}
        self._latest_stamp = {name: None for name in self._seen}
        self._cloud_units_valid = {"left": False, "right": False}
        self._checking_started = None
        self._sim_complete = False
        self._export_path = ""
        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._publisher = self.create_publisher(MappingStatus, "/mapping/status", latched)
        self.create_service(MapTask, "/mapping/task", self._on_task)
        self._export_client = self.create_client(Trigger, "/map_export/export")
        self.create_subscription(PointCloud2, "/lidar/left/points_raw", lambda msg: self._on_cloud("left", msg), qos_profile_sensor_data)
        self.create_subscription(PointCloud2, "/lidar/right/points_raw", lambda msg: self._on_cloud("right", msg), qos_profile_sensor_data)
        self.create_subscription(Imu, "/imu/data_raw", self._on_imu, qos_profile_sensor_data)
        self.create_subscription(WheelEncoder, "/wheel/encoder_counts", self._on_wheel, 20)
        self.create_subscription(Bool, "/sim/completed", self._on_sim_complete, latched)
        self.create_subscription(String, "/map_export/completed", self._on_export, latched)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self.create_timer(0.25, self._tick)
        if bool(self.get_parameter("auto_start").value):
            self._machine.advance()
            self._checking_started = time.monotonic()
            self._publish()

    @staticmethod
    def _stamp(stamp) -> float:
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    def _observe(self, name: str, stamp) -> None:
        try:
            seconds = self._stamp(stamp)
            self._monitors[name].observe(seconds)
            if self._first_stamp[name] is None:
                self._first_stamp[name] = seconds
            self._latest_stamp[name] = seconds
            self._seen[name] += 1
        except ValueError as exc:
            self._machine.fail(f"{name} timestamp check failed: {exc}")

    def _on_cloud(self, name: str, message: PointCloud2) -> None:
        try:
            validate_pointcloud_fields(message)
        except ValueError as exc:
            self._machine.fail(f"{name} point field check failed: {exc}")
            return
        try:
            points, _ = pointcloud2_to_xyz(message)
            self._cloud_units_valid[name] = bool(
                points.size > 0
                and np.isfinite(points).all()
                and float(np.max(np.abs(points))) <= self._max_coordinate
            )
        except ValueError as exc:
            self._machine.fail(f"{name} point data check failed: {exc}")
            return
        self._observe(name, message.header.stamp)

    def _on_imu(self, message: Imu) -> None:
        self._observe("imu", message.header.stamp)

    def _on_wheel(self, message: WheelEncoder) -> None:
        if not message.valid:
            self._machine.fail("wheel encoder marked unavailable")
            return
        self._observe("wheel", message.stamp)

    def _on_sim_complete(self, message: Bool) -> None:
        self._sim_complete = bool(message.data)

    def _on_export(self, message: String) -> None:
        self._export_path = message.data

    def _preflight(self) -> bool:
        for name, count in self._seen.items():
            self._checks[f"topic_{name}"] = count >= 2
            first = self._first_stamp[name]
            latest = self._latest_stamp[name]
            elapsed = 0.0 if first is None or latest is None else latest - first
            rate = 0.0 if elapsed <= 0.0 else (count - 1) / elapsed
            self._checks[f"rate_{name}"] = rate >= self._minimum_rates[name]
        self._checks["point_units_left"] = self._cloud_units_valid["left"]
        self._checks["point_units_right"] = self._cloud_units_valid["right"]
        left_stamp = self._latest_stamp["left"]
        right_stamp = self._latest_stamp["right"]
        imu_stamp = self._latest_stamp["imu"]
        self._checks["dual_lidar_time_delta"] = (
            left_stamp is not None
            and right_stamp is not None
            and abs(left_stamp - right_stamp) <= self._max_dual_delta
        )
        self._checks["lidar_imu_time_delta"] = (
            imu_stamp is not None
            and left_stamp is not None
            and abs(left_stamp - imu_stamp) <= self._max_lidar_imu_delta
        )
        free_gb = shutil.disk_usage(".").free / (1024**3)
        self._checks["disk_space"] = free_gb >= self._minimum_disk
        frames = (
            "imu_link",
            "xtm60_left_link",
            "xtm60_right_link",
            "camera_front_link",
            "camera_left_link",
            "camera_right_link",
            "camera_rear_link",
        )
        self._checks["tf_static_complete"] = all(
            self._tf_buffer.can_transform("base_link", frame, Time(), timeout=Duration(seconds=0.01))
            for frame in frames
        )
        self._checks["bag_policy"] = not self._record_bag or (
            self._bag_path.is_dir()
            and any(path.stat().st_size > 0 for path in self._bag_path.glob("*.db3"))
        )
        return all(self._checks.values())

    def _tick(self) -> None:
        state = self._machine.state
        if state is MappingState.FAILED:
            self._publish()
            return
        if state is MappingState.CHECKING:
            if self._preflight():
                self._machine.advance()
            elif self._checking_started is not None and time.monotonic() - self._checking_started > self._timeout:
                missing = [name for name, passed in self._checks.items() if not passed]
                self._machine.fail(f"preflight timeout; failed checks: {', '.join(missing)}")
        elif state is MappingState.RECORDING:
            self._machine.advance()
        elif state is MappingState.MAPPING and self._sim_complete:
            self._machine.advance()
        elif state in (MappingState.LOOP_CLOSING, MappingState.OPTIMIZING):
            self._machine.advance()
        elif state is MappingState.EXPORTING and self._export_path:
            self._machine.advance()
        elif state is MappingState.QUALITY_CHECK:
            self._machine.advance()
        self._publish()

    def _on_task(self, request, response):
        if request.command == MapTask.Request.RESET:
            self._machine.reset()
            response.accepted = True
            response.reason = "reset"
        elif request.command == MapTask.Request.START and self._machine.state is MappingState.IDLE:
            if request.map_name:
                self._map_name = request.map_name
            self._machine.advance()
            self._checking_started = time.monotonic()
            response.accepted = True
            response.reason = "started"
        elif request.command == MapTask.Request.STOP and self._machine.state is MappingState.MAPPING:
            self._sim_complete = True
            response.accepted = True
            response.reason = "mapping stop requested"
        elif request.command == MapTask.Request.EXPORT and self._machine.state in (
            MappingState.MAPPING,
            MappingState.LOOP_CLOSING,
            MappingState.OPTIMIZING,
            MappingState.EXPORTING,
        ):
            while self._machine.state is not MappingState.EXPORTING:
                self._machine.advance()
            if self._export_client.service_is_ready():
                self._export_client.call_async(Trigger.Request())
                response.accepted = True
                response.reason = "export requested"
            else:
                response.accepted = False
                response.reason = "map export service is unavailable"
        else:
            response.accepted = False
            response.reason = f"command not valid in {self._machine.state.value}"
        self._publish()
        return response

    def _publish(self) -> None:
        message = MappingStatus()
        message.stamp = self.get_clock().now().to_msg()
        message.state = self._machine.state.value
        message.progress = float(self._machine.progress)
        message.map_name = self._map_name
        message.failure_reason = self._machine.failure_reason
        message.checks = [f"{name}={'PASS' if value else 'FAIL'}" for name, value in sorted(self._checks.items())]
        self._publisher.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MappingManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
