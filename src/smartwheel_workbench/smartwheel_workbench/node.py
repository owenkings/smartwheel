import json
import math
import os
import resource
import shutil
import signal
import subprocess
import threading
import time
from collections import deque
from datetime import datetime
from pathlib import Path

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Path as PathMessage
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rtabmap_msgs.msg import Info
from sensor_msgs.msg import PointCloud2
from slam_toolbox.srv import Pause
from smartwheel_interfaces.msg import HardwareStatus, MappingStatus, WorkbenchStatus
from smartwheel_interfaces.srv import MapProductTask, MapTask, WorkbenchCommand
from std_msgs.msg import Header
from std_srvs.srv import Empty

from smartwheel_workbench.cloud_color import colors_for_cloud, normalize_cloud, pack_rgb, xyzrgb_message
from smartwheel_workbench.products import load_map_version, list_map_versions
from smartwheel_workbench.state import WorkbenchSession


CAMERAS = ("front", "left", "right", "rear")
BAG_TOPICS = (
    "/lidar/left/points_raw",
    "/lidar/right/points_raw",
    "/imu/data_raw",
    "/wheel/encoder_counts",
    "/wheel/odom",
    "/lio/odom",
    "/lio/path",
    "/odom/fused",
    "/map",
    "/mapping/status",
    "/workbench/status",
    "/hardware/status",
    "/diagnostics",
    "/teleop/cmd_vel",
    "/cmd_vel_safe",
    "/tf",
    "/tf_static",
) + tuple(f"/camera/{name}/{suffix}" for name in CAMERAS for suffix in ("image_raw", "camera_info"))


def _latched_qos() -> QoSProfile:
    return QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


def _stamp_seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def _yaw(quaternion) -> float:
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    )


def _path_length(poses) -> float:
    return float(sum(
        math.hypot(
            current.pose.position.x - previous.pose.position.x,
            current.pose.position.y - previous.pose.position.y,
        )
        for previous, current in zip(poses, poses[1:])
    ))


def _directory_size_bytes(directory: str) -> int:
    if not directory:
        return 0
    root = Path(directory)
    if not root.is_dir():
        return 0
    size = 0
    for path in root.rglob("*"):
        if path.is_file():
            try:
                size += path.stat().st_size
            except FileNotFoundError:
                continue
    return size


class WorkbenchNode(Node):
    def __init__(self) -> None:
        super().__init__("smartwheel_workbench")
        self.declare_parameter("mode", "mock")
        self.declare_parameter("hardware_enabled", False)
        self.declare_parameter("mapping_backend", "rtabmap")
        self.declare_parameter("map_root", "maps/versions")
        self.declare_parameter("experiment_root", "experiments")
        self.declare_parameter("map_cloud_source", "/rtabmap/optimized_cloud")
        self.declare_parameter("map_source", "/rtabmap/map")
        self.declare_parameter("path_source", "/lio/path")
        self.declare_parameter("hardware_profile", "config/hardware_profile.mock.yaml")
        self.declare_parameter("algorithm_profile", "mapping-v2-workbench-mock")
        self.declare_parameter("git_commit", "UNKNOWN")
        self.declare_parameter("preview_version", "")
        self.declare_parameter("minimum_free_disk_gb", 2.0)
        self.declare_parameter("bag_compression_mode", "file")
        self.declare_parameter("bag_compression_format", "zstd")
        self.declare_parameter("map_cloud_gui_rate_hz", 1.0)
        self.declare_parameter("map_export_timeout_sec", 300.0)
        if self.get_parameter("mode").value != "mock":
            raise RuntimeError("operator workbench backend permits mode:=mock only")
        if bool(self.get_parameter("hardware_enabled").value):
            raise RuntimeError("operator workbench backend refuses hardware_enabled:=true")

        self._backend = str(self.get_parameter("mapping_backend").value)
        if self._backend not in ("rtabmap", "slam_toolbox"):
            raise ValueError("mapping_backend must be rtabmap or slam_toolbox")
        self._map_root = Path(str(self.get_parameter("map_root").value)).expanduser().resolve()
        self._experiment_root = Path(str(self.get_parameter("experiment_root").value)).expanduser().resolve()
        self._map_root.mkdir(parents=True, exist_ok=True)
        self._experiment_root.mkdir(parents=True, exist_ok=True)
        self._session = WorkbenchSession()
        self._session_lock = threading.Lock()
        self._map_name = "workbench_map"
        self._started_monotonic = time.monotonic()
        self._active_operation = ""
        self._quality_result = "NOT RUN"
        self._version_directory = ""
        self._bag_path = ""
        self._experiment_directory = None
        self._bag_process = None
        self._bag_size_bytes = 0
        self._bag_write_rate_bytes_per_sec = 0.0
        self._last_bag_measure_monotonic = time.monotonic()
        self._last_bag_measure_bytes = 0
        self._hardware_status = None
        self._mapping_status = None
        self._source_diagnostics = {}
        self._latest_map = None
        self._latest_cloud = None
        self._latest_path = None
        self._selected_version = ""
        self._trajectory_length = 0.0
        self._keyframes = 0
        self._loops = set()
        self._map_points = 0
        cloud_rate = float(self.get_parameter("map_cloud_gui_rate_hz").value)
        if cloud_rate <= 0.0:
            raise ValueError("map_cloud_gui_rate_hz must be positive")
        self._cloud_publish_period = 1.0 / cloud_rate
        self._map_export_timeout = float(self.get_parameter("map_export_timeout_sec").value)
        if self._map_export_timeout <= 0.0:
            raise ValueError("map_export_timeout_sec must be positive")
        self._last_cloud_publish_monotonic = float("-inf")
        self._counts = {}
        self._receive_times = {}
        self._last_stamps = {}
        self._last_receive_monotonic = {}
        self._stamp_regressions = 0
        self._last_diagnostic_monotonic = time.monotonic()
        self._last_process_cpu_sec = time.process_time()
        self._callback_group = ReentrantCallbackGroup()
        self._observation_callback_group = ReentrantCallbackGroup()
        self._observation_subscriptions = []

        latched = _latched_qos()
        self._status_pub = self.create_publisher(WorkbenchStatus, "/workbench/status", latched)
        self._map_pub = self.create_publisher(OccupancyGrid, "/map", latched)
        self._cloud_pub = self.create_publisher(PointCloud2, "/map_cloud", latched)
        self._path_pub = self.create_publisher(PathMessage, "/mapping/optimized_path", latched)
        self._teleop_stop_pub = self.create_publisher(Twist, "/teleop/cmd_vel", 1)
        self._diagnostics_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 20)

        self.create_service(
            WorkbenchCommand,
            "/workbench/command",
            self._on_command,
            callback_group=self._callback_group,
        )
        self.create_service(
            MapProductTask,
            "/map_products/task",
            self._on_product_task,
            callback_group=self._callback_group,
        )
        self._mapping_client = self.create_client(
            MapTask, "/mapping/task", callback_group=self._callback_group
        )
        if self._backend == "rtabmap":
            self._pause_client = self.create_client(
                Empty, "/rtabmap/pause", callback_group=self._callback_group
            )
            self._resume_client = self.create_client(
                Empty, "/rtabmap/resume", callback_group=self._callback_group
            )
        else:
            self._pause_client = self.create_client(
                Pause, "/slam_toolbox/pause_new_measurements", callback_group=self._callback_group
            )
            self._resume_client = self._pause_client

        self.create_subscription(
            HardwareStatus, "/hardware/status", self._on_hardware, latched
        )
        self.create_subscription(
            MappingStatus, "/mapping/status", self._on_mapping, latched
        )
        self._observation_subscriptions.append(self.create_subscription(
            DiagnosticArray,
            "/diagnostics",
            self._on_diagnostics,
            20,
            callback_group=self._observation_callback_group,
        ))
        self.create_subscription(
            PointCloud2,
            str(self.get_parameter("map_cloud_source").value),
            self._on_cloud,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            OccupancyGrid,
            str(self.get_parameter("map_source").value),
            self._on_map,
            latched,
        )
        self._observation_subscriptions.append(self.create_subscription(
            PathMessage,
            str(self.get_parameter("path_source").value),
            self._on_path,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
            callback_group=self._observation_callback_group,
        ))
        self._observation_subscriptions.append(self.create_subscription(
            Info,
            "/rtabmap/info",
            self._on_rtabmap_info,
            20,
            callback_group=self._observation_callback_group,
        ))
        self.create_timer(0.5, self._publish_status)
        self.create_timer(1.0, self._publish_diagnostics)
        preview_version = str(self.get_parameter("preview_version").value).strip()
        self._preview_timer = None
        if preview_version:
            self._selected_version = str(Path(preview_version).expanduser().resolve())
            self._preview_timer = self.create_timer(1.0, self._publish_initial_preview)
        self._publish_status()

    def _publish_initial_preview(self) -> None:
        try:
            loaded = load_map_version(self._selected_version)
            self._publish_loaded_map(loaded)
            self._version_directory = str(loaded.directory)
            self._quality_result = loaded.quality_summary
            self._session.state = "READY"
        except Exception as error:
            self._session.fail(f"initial map preview failed: {error}")
        finally:
            if self._preview_timer is not None:
                self._preview_timer.cancel()
            self._publish_status()

    def _observe(self, topic: str, message) -> None:
        self._counts[topic] = self._counts.get(topic, 0) + 1
        received = time.monotonic()
        self._last_receive_monotonic[topic] = received
        self._receive_times.setdefault(topic, deque(maxlen=500)).append(received)
        if not hasattr(message, "header"):
            return
        stamp = _stamp_seconds(message.header.stamp)
        previous = self._last_stamps.get(topic)
        if previous is not None and stamp < previous:
            self._stamp_regressions += 1
        self._last_stamps[topic] = stamp

    def _on_hardware(self, message: HardwareStatus) -> None:
        self._hardware_status = message

    def _on_mapping(self, message: MappingStatus) -> None:
        self._mapping_status = message
        if message.state == "FAILED":
            with self._session_lock:
                self._session.fail(f"mapping manager: {message.failure_reason}")

    def _on_diagnostics(self, message: DiagnosticArray) -> None:
        for status in message.status:
            if status.name.startswith("smartwheel_sim/"):
                self._source_diagnostics[status.name] = status

    def _on_cloud(self, message: PointCloud2) -> None:
        self._observe("/map_cloud_source", message)
        if message.header.frame_id != "map":
            return
        self._latest_cloud = message
        self._map_points = int(message.width) * int(message.height)
        now = time.monotonic()
        if now - self._last_cloud_publish_monotonic < self._cloud_publish_period:
            return
        try:
            xyz, colors = normalize_cloud(message)
            self._cloud_pub.publish(xyzrgb_message(message.header, xyz, colors))
            self._last_cloud_publish_monotonic = now
        except (AssertionError, ValueError) as error:
            self.get_logger().error(f"FAILED to normalize /map_cloud source: {error}")

    def _on_map(self, message: OccupancyGrid) -> None:
        self._observe("/map_source", message)
        if message.info.width == 0 or message.info.height == 0 or not message.data:
            return
        self._latest_map = message
        self._map_pub.publish(message)

    def _on_path(self, message: PathMessage) -> None:
        self._observe("/mapping/optimized_path_source", message)
        self._trajectory_length = _path_length(message.poses)
        self._latest_path = message
        self._path_pub.publish(message)

    def _on_rtabmap_info(self, message: Info) -> None:
        self._keyframes = max(self._keyframes, len(message.wm_state))
        if message.loop_closure_id > 0:
            self._loops.add((message.ref_id, message.loop_closure_id))
        if message.proximity_detection_id > 0:
            self._loops.add((message.ref_id, message.proximity_detection_id))

    def _preflight(self) -> tuple[bool, str]:
        if self._hardware_status is None:
            return False, "no /hardware/status received"
        if self._hardware_status.hardware_enabled or self._hardware_status.mode != "mock":
            return False, "hardware status is not mock with hardware_enabled=false"
        availability = {
            "left LiDAR": self._hardware_status.lidar_left_available,
            "right LiDAR": self._hardware_status.lidar_right_available,
            "H30 IMU": self._hardware_status.imu_available,
            "wheel encoder": self._hardware_status.wheel_available,
            "camera array": self._hardware_status.cameras_available,
        }
        unavailable = [name for name, available in availability.items() if not available]
        if unavailable:
            return False, "mock hardware status unavailable: " + ", ".join(unavailable)
        if self._mapping_status is None:
            return False, "no /mapping/status received"
        if self._mapping_status.state == "FAILED":
            return False, f"mapping manager failed: {self._mapping_status.failure_reason}"
        mapping_checks = {
            name: result == "PASS"
            for entry in self._mapping_status.checks
            if "=" in entry
            for name, result in (entry.rsplit("=", 1),)
        }
        required_mapping_checks = (
            "topic_left", "topic_right", "topic_imu", "topic_wheel",
            "rate_left", "rate_right", "rate_imu", "rate_wheel",
            "point_units_left", "point_units_right",
            "dual_lidar_time_delta", "lidar_imu_time_delta",
            "tf_static_complete", "backend_tf_dynamic", "disk_space",
        )
        failed_checks = [
            name for name in required_mapping_checks if not mapping_checks.get(name, False)
        ]
        if failed_checks:
            return False, "mapping preflight checks not passing: " + ", ".join(failed_checks)
        camera_diagnostics = [f"smartwheel_sim/{name}_camera" for name in CAMERAS]
        unavailable_cameras = [
            name for name in camera_diagnostics
            if name not in self._source_diagnostics
            or self._source_diagnostics[name].level != DiagnosticStatus.OK
            or "MOCK" not in self._source_diagnostics[name].hardware_id.upper()
        ]
        if unavailable_cameras:
            return False, "camera source diagnostics not healthy: " + ", ".join(unavailable_cameras)
        if self._stamp_regressions:
            return False, f"timestamp regression count={self._stamp_regressions}"
        free_gb = shutil.disk_usage(self._map_root).free / 1024**3
        if free_gb < float(self.get_parameter("minimum_free_disk_gb").value):
            return False, f"free disk {free_gb:.2f} GB below minimum"
        return True, "authoritative mock source, mapping, TF, timestamp and disk checks passed"

    @staticmethod
    def _call(client, request, timeout_sec: float = 5.0):
        if not client.wait_for_service(timeout_sec=1.0):
            raise RuntimeError(f"service unavailable: {client.srv_name}")
        future = client.call_async(request)
        deadline = time.monotonic() + timeout_sec
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.01)
        if not future.done():
            raise RuntimeError(f"service timeout: {client.srv_name}")
        result = future.result()
        if result is None:
            raise RuntimeError(f"service returned no response: {client.srv_name}")
        return result

    def _start_bag(self) -> str:
        if self._bag_process is not None and self._bag_process.poll() is None:
            raise RuntimeError("rosbag is already recording")
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        experiment = self._experiment_root / datetime.now().strftime("%Y-%m-%d") / f"{self._map_name}_{stamp}"
        experiment.mkdir(parents=True, exist_ok=False)
        (experiment / "logs").mkdir()
        bag = experiment / "rosbag"
        compression_mode = str(self.get_parameter("bag_compression_mode").value)
        compression_format = str(self.get_parameter("bag_compression_format").value)
        if compression_mode not in ("file", "message"):
            raise RuntimeError("bag_compression_mode must be file or message")
        if compression_format != "zstd":
            raise RuntimeError("bag_compression_format must be zstd")
        self._experiment_directory = experiment
        self._write_initial_experiment_metadata(experiment)
        command = [
            "ros2",
            "bag",
            "record",
            "-o",
            str(bag),
            "--compression-mode",
            compression_mode,
            "--compression-format",
            compression_format,
            *BAG_TOPICS,
        ]
        self._bag_process = subprocess.Popen(command, start_new_session=True)
        self._bag_path = str(bag)
        self._bag_size_bytes = 0
        self._bag_write_rate_bytes_per_sec = 0.0
        self._last_bag_measure_bytes = 0
        self._last_bag_measure_monotonic = time.monotonic()
        return self._bag_path

    @staticmethod
    def _command_output(command) -> str:
        try:
            result = subprocess.run(
                command, check=False, capture_output=True, text=True, timeout=5.0
            )
            output = result.stdout.strip()
            if result.stderr.strip():
                output += ("\n" if output else "") + result.stderr.strip()
            return output or f"command returned {result.returncode} without output"
        except (OSError, subprocess.TimeoutExpired) as error:
            return f"command failed: {error}"

    def _write_initial_experiment_metadata(self, experiment: Path) -> None:
        commit = str(self.get_parameter("git_commit").value)
        (experiment / "git_commit.txt").write_text(commit + "\n", encoding="utf-8")
        profile = Path(str(self.get_parameter("hardware_profile").value)).expanduser()
        if profile.is_file():
            shutil.copyfile(profile, experiment / "hardware_profile.yaml")
        else:
            (experiment / "hardware_profile.yaml").write_text(
                json.dumps({"mode": "mock", "hardware_enabled": False, "source": str(profile)}, indent=2) + "\n",
                encoding="utf-8",
            )
        algorithm = {
            "profile": str(self.get_parameter("algorithm_profile").value),
            "mapping_backend": self._backend,
            "map_cloud_source": str(self.get_parameter("map_cloud_source").value),
            "map_source": str(self.get_parameter("map_source").value),
            "path_source": str(self.get_parameter("path_source").value),
            "map_cloud_gui_rate_hz": float(self.get_parameter("map_cloud_gui_rate_hz").value),
            "bag_compression": {
                "mode": str(self.get_parameter("bag_compression_mode").value),
                "format": str(self.get_parameter("bag_compression_format").value),
            },
        }
        (experiment / "algorithm_profile.yaml").write_text(
            json.dumps(algorithm, indent=2) + "\n", encoding="utf-8"
        )
        (experiment / "topics.txt").write_text("\n".join(BAG_TOPICS) + "\n", encoding="utf-8")
        (experiment / "nodes.txt").write_text(
            self._command_output(["ros2", "node", "list"]) + "\n", encoding="utf-8"
        )
        (experiment / "notes.md").write_text(
            "# Experiment Notes\n\n"
            "Mock-only RViz workbench run. No physical transport or motor interface was enabled.\n",
            encoding="utf-8",
        )
        self._write_experiment_runtime_metadata(experiment)

    def _write_experiment_runtime_metadata(self, experiment: Path) -> None:
        checks = list(self._mapping_status.checks) if self._mapping_status else []
        tf_checks = [entry for entry in checks if "tf" in entry]
        (experiment / "tf_snapshot.yaml").write_text(
            json.dumps(
                {
                    "source": "mapping manager TF buffer",
                    "checks": tf_checks,
                    "note": "Static transforms are published from the mock robot description.",
                },
                indent=2,
            ) + "\n",
            encoding="utf-8",
        )
        diagnostic_lines = []
        for name, status in sorted(self._source_diagnostics.items()):
            diagnostic_lines.append(
                f"{name} level={int.from_bytes(status.level, 'little') if isinstance(status.level, bytes) else status.level} "
                f"hardware_id={status.hardware_id} message={status.message}"
            )
            diagnostic_lines.extend(f"  {item.key}={item.value}" for item in status.values)
        diagnostic_lines.extend(f"mapping_check {entry}" for entry in checks)
        (experiment / "diagnostics.txt").write_text(
            "\n".join(diagnostic_lines) + "\n", encoding="utf-8"
        )
        trajectory_lines = []
        if self._latest_path is not None:
            for pose in self._latest_path.poses:
                stamp = _stamp_seconds(pose.header.stamp)
                point = pose.pose.position
                orientation = pose.pose.orientation
                trajectory_lines.append(
                    f"{stamp:.9f} {point.x:.9f} {point.y:.9f} {point.z:.9f} "
                    f"{orientation.x:.9f} {orientation.y:.9f} {orientation.z:.9f} {orientation.w:.9f}"
                )
        (experiment / "trajectory.tum").write_text(
            "\n".join(trajectory_lines) + ("\n" if trajectory_lines else ""), encoding="utf-8"
        )
        metrics = {
            "state": self._session.state,
            "map_name": self._map_name,
            "elapsed_sec": time.monotonic() - self._started_monotonic,
            "trajectory_length_m": self._trajectory_length,
            "keyframe_count": self._keyframes,
            "loop_closure_count": len(self._loops),
            "map_point_count": self._map_points,
            "timestamp_regressions": self._stamp_regressions,
            "bag_size_bytes": _directory_size_bytes(self._bag_path),
            "bag_write_rate_bytes_per_sec": self._bag_write_rate_bytes_per_sec,
            "quality_result": self._quality_result,
            "failure_reason": self._session.failure_reason,
        }
        (experiment / "metrics.json").write_text(
            json.dumps(metrics, indent=2) + "\n", encoding="utf-8"
        )

    def _measure_bag(self) -> None:
        now = time.monotonic()
        size = _directory_size_bytes(self._bag_path)
        elapsed = now - self._last_bag_measure_monotonic
        if elapsed > 0.0:
            self._bag_write_rate_bytes_per_sec = max(
                0.0, (size - self._last_bag_measure_bytes) / elapsed
            )
        self._bag_size_bytes = size
        self._last_bag_measure_bytes = size
        self._last_bag_measure_monotonic = now

    def _stop_bag(self) -> None:
        if self._bag_process is None:
            return
        if self._bag_process.poll() is None:
            os.killpg(self._bag_process.pid, signal.SIGINT)
            try:
                self._bag_process.wait(timeout=10.0)
            except subprocess.TimeoutExpired:
                os.killpg(self._bag_process.pid, signal.SIGTERM)
                self._bag_process.wait(timeout=5.0)
        if self._experiment_directory is not None:
            self._write_experiment_runtime_metadata(self._experiment_directory)
        self._bag_process = None

    def _request_mapping(self, command: int):
        request = MapTask.Request()
        request.command = command
        request.map_name = self._map_name
        response = self._call(self._mapping_client, request)
        if not response.accepted:
            raise RuntimeError(f"mapping manager rejected command: {response.reason}")
        return response.reason

    def _export(self) -> str:
        self._request_mapping(MapTask.Request.EXPORT)
        deadline = time.monotonic() + self._map_export_timeout
        while time.monotonic() < deadline:
            status = self._mapping_status
            if status is not None and status.state == "FAILED":
                raise RuntimeError(f"mapping manager export failed: {status.failure_reason}")
            if status is not None and status.state == "READY":
                break
            time.sleep(0.05)
        else:
            raise RuntimeError(
                f"mapping manager export did not reach READY within "
                f"{self._map_export_timeout:.0f} seconds"
            )
        latest = self._map_root / "latest_path.txt"
        if not latest.is_file():
            raise RuntimeError("mapping manager reached READY without latest_path.txt")
        directory = Path(latest.read_text(encoding="utf-8").strip()).resolve()
        loaded = load_map_version(directory)
        self._version_directory = str(loaded.directory)
        self._selected_version = self._version_directory
        self._quality_result = loaded.quality_summary
        self._publish_loaded_map(loaded)
        return self._version_directory

    def _on_command(self, request: WorkbenchCommand.Request, response: WorkbenchCommand.Response):
        command = int(request.command)
        if request.map_name.strip():
            self._map_name = request.map_name.strip()
        with self._session_lock:
            was_paused = self._session.paused
            accepted, reason = self._session.transition(command)
        if not accepted:
            response.accepted = False
            response.state = self._session.state
            response.reason = reason
            response.version_directory = self._version_directory
            return response
        try:
            self._active_operation = reason
            if command == WorkbenchCommand.Request.PREFLIGHT:
                passed, detail = self._preflight()
                if not passed:
                    raise RuntimeError(detail)
                reason = detail
            elif command == WorkbenchCommand.Request.START_RECORDING:
                reason = f"recording started: {self._start_bag()}"
            elif command == WorkbenchCommand.Request.START_MAPPING:
                if self._mapping_status is None:
                    raise RuntimeError("mapping manager status unavailable")
                if self._mapping_status.state == "IDLE":
                    reason = self._request_mapping(MapTask.Request.START)
                elif self._mapping_status.state not in ("MAPPING", "RECORDING", "CHECKING"):
                    raise RuntimeError(f"mapping manager is {self._mapping_status.state}")
                else:
                    reason = f"mapping manager confirmed active state {self._mapping_status.state}"
            elif command == WorkbenchCommand.Request.PAUSE_MAPPING:
                request_message = Empty.Request() if self._backend == "rtabmap" else Pause.Request()
                result = self._call(self._pause_client, request_message)
                if self._backend == "slam_toolbox" and not result.status:
                    raise RuntimeError("slam_toolbox did not enter paused state")
                reason = f"{self._backend} pause service confirmed"
            elif command == WorkbenchCommand.Request.RESUME_MAPPING:
                request_message = Empty.Request() if self._backend == "rtabmap" else Pause.Request()
                result = self._call(self._resume_client, request_message)
                if self._backend == "slam_toolbox" and result.status:
                    raise RuntimeError("slam_toolbox remained paused")
                reason = f"{self._backend} resume service confirmed"
            elif command == WorkbenchCommand.Request.FINISH_MAPPING:
                if was_paused:
                    request_message = Empty.Request() if self._backend == "rtabmap" else Pause.Request()
                    self._call(self._resume_client, request_message)
                self._stop_bag()
                self._session.recording = False
                reason = self._request_mapping(MapTask.Request.STOP)
            elif command == WorkbenchCommand.Request.OPTIMIZE:
                if self._backend == "rtabmap" and self._keyframes == 0:
                    raise RuntimeError("RTAB-Map has no keyframe evidence to optimize")
                if self._backend == "slam_toolbox" and self._latest_map is None:
                    raise RuntimeError("slam_toolbox has no map evidence to optimize")
                reason = "backend optimization evidence confirmed"
            elif command in (
                WorkbenchCommand.Request.SAVE_MAP,
                WorkbenchCommand.Request.EXPORT_PRODUCTS,
            ):
                directory = self._export()
                self._session.state = "QUALITY_CHECK"
                self._session.map_saved = True
                self._session.state = "READY"
                reason = f"map products validated and published: {directory}"
            elif command == WorkbenchCommand.Request.PREVIEW_SAVED_MAP:
                loaded = load_map_version(self._selected_version or self._version_directory)
                self._publish_loaded_map(loaded)
                reason = f"preview published: {loaded.directory}"
            elif command == WorkbenchCommand.Request.CANCEL:
                self._stop_bag()
                self._teleop_stop_pub.publish(Twist())
                reason = "mock session cancelled and zero velocity published"
            elif command == WorkbenchCommand.Request.RESET_SESSION:
                self._stop_bag()
                if self._mapping_client.service_is_ready():
                    self._request_mapping(MapTask.Request.RESET)
                self._trajectory_length = 0.0
                self._quality_result = "NOT RUN"
                reason = "mock workbench session reset"
            response.accepted = True
        except Exception as error:  # service boundary must return a specific failure
            with self._session_lock:
                self._session.fail(str(error))
            response.accepted = False
            reason = str(error)
        self._active_operation = ""
        response.state = self._session.state
        response.reason = reason
        response.version_directory = self._version_directory
        self._publish_status()
        return response

    def _resolve_version(self, request_directory: str) -> Path:
        candidate = request_directory.strip() or self._selected_version
        if candidate:
            path = Path(candidate).expanduser().resolve()
            if self._map_root not in path.parents:
                raise ValueError("selected map must be inside configured map_root")
            return path
        versions = list_map_versions(self._map_root)
        if not versions:
            raise ValueError("no complete map versions found")
        return versions[0]

    def _on_product_task(self, request: MapProductTask.Request, response: MapProductTask.Response):
        try:
            versions = list_map_versions(self._map_root)
            response.versions = [str(path) for path in versions]
            if request.command == MapProductTask.Request.LIST:
                response.accepted = True
                response.reason = f"found {len(versions)} complete map versions"
                if versions:
                    loaded = load_map_version(versions[0])
                    response.selected_version = str(loaded.directory)
                    response.available_files = loaded.files
                    response.quality_summary = loaded.quality_summary
                return response
            directory = self._resolve_version(request.version_directory)
            loaded = load_map_version(directory)
            self._selected_version = str(loaded.directory)
            if request.command == MapProductTask.Request.SELECT:
                reason = "map version selected"
            elif request.command == MapProductTask.Request.REPUBLISH_CLOUD:
                self._publish_loaded_cloud(loaded)
                reason = "saved 3D cloud republished on /map_cloud"
            elif request.command == MapProductTask.Request.REPUBLISH_MAP:
                self._publish_loaded_occupancy(loaded)
                reason = "saved occupancy grid republished on /map"
            elif request.command == MapProductTask.Request.PREVIEW:
                self._publish_loaded_map(loaded)
                reason = "saved 3D cloud, 2D map and trajectory republished"
            else:
                raise ValueError(f"unknown map product command {request.command}")
            response.accepted = True
            response.reason = reason
            response.selected_version = str(loaded.directory)
            response.available_files = loaded.files
            response.quality_summary = loaded.quality_summary
        except Exception as error:
            response.accepted = False
            response.reason = str(error)
        return response

    def _publish_loaded_cloud(self, loaded) -> None:
        header = Header(stamp=self.get_clock().now().to_msg(), frame_id="map")
        if loaded.colors is not None:
            colors = pack_rgb(loaded.colors[:, 0], loaded.colors[:, 1], loaded.colors[:, 2])
        else:
            colors = colors_for_cloud(loaded.points[:, 2])
        message = xyzrgb_message(header, loaded.points, colors)
        self._latest_cloud = message
        self._map_points = int(loaded.points.shape[0])
        self._cloud_pub.publish(message)

    def _publish_loaded_occupancy(self, loaded) -> None:
        message = OccupancyGrid()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "map"
        message.info.map_load_time = message.header.stamp
        message.info.resolution = loaded.resolution
        message.info.width = loaded.cells.shape[1]
        message.info.height = loaded.cells.shape[0]
        message.info.origin.position.x = loaded.origin_x
        message.info.origin.position.y = loaded.origin_y
        message.info.origin.orientation.z = math.sin(loaded.origin_yaw * 0.5)
        message.info.origin.orientation.w = math.cos(loaded.origin_yaw * 0.5)
        message.data = loaded.cells.ravel().tolist()
        self._latest_map = message
        self._map_pub.publish(message)

    def _publish_loaded_path(self, loaded) -> None:
        message = PathMessage()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "map"
        for stamp, x, y, yaw in loaded.trajectory:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp.sec = int(stamp)
            pose.header.stamp.nanosec = int((stamp - int(stamp)) * 1e9)
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.z = math.sin(yaw * 0.5)
            pose.pose.orientation.w = math.cos(yaw * 0.5)
            message.poses.append(pose)
        self._latest_path = message
        self._path_pub.publish(message)

    def _publish_loaded_map(self, loaded) -> None:
        self._publish_loaded_cloud(loaded)
        self._publish_loaded_occupancy(loaded)
        self._publish_loaded_path(loaded)

    def _publish_status(self) -> None:
        if self._bag_process is not None and self._bag_process.poll() is None:
            free_gb = shutil.disk_usage(self._map_root).free / 1024**3
            minimum_free_gb = float(self.get_parameter("minimum_free_disk_gb").value)
            if free_gb < minimum_free_gb:
                reason = (
                    f"free disk {free_gb:.2f} GB below runtime reserve "
                    f"{minimum_free_gb:.2f} GB; recording stopped"
                )
                self.get_logger().error(reason)
                self._stop_bag()
                with self._session_lock:
                    self._session.recording = False
                    self._session.fail(reason)
        message = WorkbenchStatus()
        message.stamp = self.get_clock().now().to_msg()
        message.state = self._session.state
        message.map_name = self._map_name
        message.mapping_backend = self._backend
        message.bag_path = self._bag_path
        message.version_directory = self._version_directory
        message.elapsed_sec = time.monotonic() - self._started_monotonic
        message.trajectory_length_m = self._trajectory_length
        message.keyframe_count = self._keyframes
        message.loop_closure_count = len(self._loops)
        message.map_point_count = self._map_points
        message.recording = self._bag_process is not None and self._bag_process.poll() is None
        message.paused = self._session.paused
        message.map_saved = self._session.map_saved
        message.quality_result = self._quality_result
        message.failure_reason = self._session.failure_reason
        message.active_operation = self._active_operation
        self._status_pub.publish(message)

    def _publish_diagnostics(self) -> None:
        now = time.monotonic()
        free_gb = shutil.disk_usage(self._map_root).free / 1024**3
        self._measure_bag()
        rss_mb = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024.0
        process_cpu_sec = time.process_time()
        diagnostic_period = max(now - self._last_diagnostic_monotonic, 1e-6)
        cpu_percent = 100.0 * (process_cpu_sec - self._last_process_cpu_sec) / diagnostic_period
        self._last_diagnostic_monotonic = now
        self._last_process_cpu_sec = process_cpu_sec
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        runtime = DiagnosticStatus()
        runtime.name = "smartwheel_workbench/runtime"
        runtime.hardware_id = "MOCK_ONLY"
        runtime.level = DiagnosticStatus.ERROR if self._session.state == "FAILED" else DiagnosticStatus.OK
        runtime.message = self._session.failure_reason or "mock workbench operational"
        runtime.values = [
            KeyValue(key="cpu_process_percent", value=f"{cpu_percent:.1f}"),
            KeyValue(key="memory_rss_mb", value=f"{rss_mb:.1f}"),
            KeyValue(key="disk_free_gb", value=f"{free_gb:.2f}"),
            KeyValue(
                key="bag_write_rate",
                value=f"{self._bag_write_rate_bytes_per_sec / 1024**2:.2f} MB/s",
            ),
            KeyValue(key="bag_size_gb", value=f"{self._bag_size_bytes / 1024**3:.2f}"),
            KeyValue(
                key="bag_compression",
                value=(
                    f"{self.get_parameter('bag_compression_mode').value}/"
                    f"{self.get_parameter('bag_compression_format').value}"
                ),
            ),
            KeyValue(key="timestamp_regressions", value=str(self._stamp_regressions)),
            KeyValue(key="git_commit", value=str(self.get_parameter("git_commit").value)),
            KeyValue(key="hardware_profile", value=str(self.get_parameter("hardware_profile").value)),
            KeyValue(key="algorithm_profile", value=str(self.get_parameter("algorithm_profile").value)),
            KeyValue(key="dual_lidar_time_delta", value="validated by mapping manager"),
            KeyValue(key="point_cloud_units", value="validated by mapping manager"),
        ]
        for topic in sorted(self._counts):
            samples = [
                stamp
                for stamp in self._receive_times.get(topic, ())
                if now - stamp <= 5.0
            ]
            sample_elapsed = samples[-1] - samples[0] if len(samples) >= 2 else 0.0
            frequency = (
                (len(samples) - 1) / sample_elapsed if sample_elapsed > 0.0 else 0.0
            )
            runtime.values.append(
                KeyValue(key=f"frequency:{topic}", value=f"{frequency:.2f} Hz")
            )
            age = now - self._last_receive_monotonic.get(topic, now)
            runtime.values.append(
                KeyValue(key=f"last_message_age:{topic}", value=f"{age:.3f} s")
            )
            stamp = self._last_stamps.get(topic)
            if stamp is not None:
                latency_ms = (self.get_clock().now().nanoseconds * 1e-9 - stamp) * 1000.0
                runtime.values.append(
                    KeyValue(
                        key=f"timestamp_latency:{topic}", value=f"{latency_ms:.1f} ms"
                    )
                )
        motor = DiagnosticStatus(
            level=DiagnosticStatus.OK,
            name="smartwheel_workbench/motor_driver",
            message="DISABLED; no motor command publisher exists in workbench",
            hardware_id="DISABLED",
        )
        estop = DiagnosticStatus(
            level=DiagnosticStatus.OK,
            name="smartwheel_workbench/emergency_stop",
            message="mock software status clear; no physical E-stop claimed",
            hardware_id="MOCK_ONLY",
        )
        disk = DiagnosticStatus(
            level=(DiagnosticStatus.WARN if free_gb < 5.0 else DiagnosticStatus.OK),
            name="smartwheel_workbench/disk",
            message=f"{free_gb:.2f} GB free",
            hardware_id="MOCK_ONLY",
        )
        bag_active = self._bag_process is not None and self._bag_process.poll() is None
        bag = DiagnosticStatus(
            level=DiagnosticStatus.OK,
            name="smartwheel_workbench/rosbag",
            message="recording" if bag_active else "idle",
            hardware_id="MOCK_ONLY",
        )
        mapping_checks = {
            name: result == "PASS"
            for entry in (self._mapping_status.checks if self._mapping_status else ())
            if "=" in entry
            for name, result in (entry.rsplit("=", 1),)
        }
        static_tf_available = mapping_checks.get("tf_static_complete", False)
        dynamic_tf_available = mapping_checks.get("backend_tf_dynamic", False)
        tf_available = static_tf_available and dynamic_tf_available
        tf_status = DiagnosticStatus(
            level=DiagnosticStatus.OK if tf_available else DiagnosticStatus.WARN,
            name="smartwheel_workbench/tf",
            message=(
                f"static={'PASS' if static_tf_available else 'FAIL'} "
                f"map_to_base={'PASS' if dynamic_tf_available else 'FAIL'}; "
                "validated by mapping manager TF buffer"
            ),
            hardware_id="MOCK_ONLY",
        )
        array.status = [runtime, motor, estop, disk, bag, tf_status]
        self._diagnostics_pub.publish(array)

    def shutdown(self) -> None:
        if rclpy.ok(context=self.context):
            try:
                self._teleop_stop_pub.publish(Twist())
            except rclpy.executors.ExternalShutdownException:
                pass
        self._stop_bag()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WorkbenchNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        executor.shutdown(timeout_sec=3.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
