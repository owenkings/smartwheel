import ctypes
import math
import os
import platform
import sys
import threading
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, List, Optional, Sequence, Tuple

try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import PointCloud2, PointField
    from sensor_msgs_py import point_cloud2
    from std_msgs.msg import Header, String
except ImportError:
    rclpy = None
    Node = object
    PointCloud2 = None
    PointField = None
    point_cloud2 = None
    Header = None
    String = None


XYZI = Tuple[float, float, float, float]


@dataclass
class XTM60SdkConfig:
    sdk_root: str = ""
    connection_mode: str = "ethernet"
    ip_address: str = "192.168.0.101"
    serial_port: str = ""
    image_type: int = 4
    frame_id: str = "laser_link"
    point_unit_scale: float = 1.0
    range_min: float = 0.05
    range_max: float = 20.0
    publish_intensity: bool = True
    enable_sdk_filters: bool = True
    kalman_factor: int = 300
    kalman_threshold: int = 200
    kalman_range: int = 2000
    median_size: int = 3
    edge_threshold: int = 150
    reconnect_interval_sec: float = 3.0


def _candidate_sdk_roots(configured_root: str) -> List[Path]:
    candidates: List[Path] = []
    for raw in (configured_root, os.environ.get("XTSDK_PY_ROOT", "")):
        if raw:
            candidates.append(Path(raw).expanduser())

    here = Path(__file__).resolve()
    for parent in here.parents:
        candidates.append(parent / "xtsdk_py-main")
        candidates.append(parent / "xtsdk_py")
        candidates.append(parent / "third_party" / "xtsdk_py-main")
        candidates.append(parent / "third_party" / "xtsdk_py")
        candidates.append(parent / "reference" / "xtsdk_py_main" / "xtsdk_py-main")
    return candidates


def find_xtsdk_root(configured_root: str = "") -> Path:
    for candidate in _candidate_sdk_roots(configured_root):
        if (candidate / "cfg").is_dir() and (candidate / "lib").is_dir():
            return candidate
    searched = "\n  ".join(str(p) for p in _candidate_sdk_roots(configured_root))
    raise FileNotFoundError(
        "xtsdk_py root not found. Set XTSDK_PY_ROOT or sdk_root. Searched:\n  "
        + searched
    )


def configure_xtsdk_import_path(sdk_root: Path) -> Path:
    system_name = platform.system()
    if system_name == "Windows":
        lib_dir = sdk_root / "lib" / "win32"
        if hasattr(os, "add_dll_directory"):
            os.add_dll_directory(str(lib_dir))
        os.environ["PATH"] = str(lib_dir) + os.pathsep + os.environ.get("PATH", "")
    elif system_name == "Linux":
        arch = platform.machine()
        if arch not in ("x86_64", "aarch64"):
            raise RuntimeError(f"unsupported Linux architecture for xtsdk_py: {arch}")
        lib_dir = sdk_root / "lib" / "linux" / arch
        shared = lib_dir / "libxtsdk_shared.so"
        if shared.exists():
            ctypes.CDLL(str(shared), mode=ctypes.RTLD_GLOBAL)
    else:
        raise RuntimeError(f"unsupported OS for xtsdk_py: {system_name}")

    if not lib_dir.is_dir():
        raise FileNotFoundError(f"xtsdk library directory not found: {lib_dir}")

    for path in (lib_dir, sdk_root / "cfg"):
        text = str(path)
        if text not in sys.path:
            sys.path.insert(0, text)
    return lib_dir


def extract_xyzi_points(
    frame,
    unit_scale: float = 1.0,
    range_min: float = 0.05,
    range_max: float = 20.0,
) -> List[XYZI]:
    """Extract finite XYZI points from an XT SDK frame.

    The SDK frame points are treated as the lidar-frame coordinates. Unit scale
    is configurable because deployments should verify whether a given SDK build
    returns meters or millimeters.
    """
    if not getattr(frame, "hasPointcloud", False):
        return []
    raw_points = getattr(frame, "points", None)
    if not raw_points:
        return []

    amplitudes: Optional[Sequence] = getattr(frame, "amplData", None)
    scaled_min = max(0.0, float(range_min))
    scaled_max = max(scaled_min, float(range_max))
    scale = float(unit_scale)
    result: List[XYZI] = []

    for index, point in enumerate(raw_points):
        x = float(getattr(point, "x", math.nan)) * scale
        y = float(getattr(point, "y", math.nan)) * scale
        z = float(getattr(point, "z", math.nan)) * scale
        if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
            continue
        distance = math.sqrt(x * x + y * y + z * z)
        if distance < scaled_min or distance > scaled_max:
            continue
        intensity = float(getattr(point, "i", 0.0))
        if amplitudes is not None and index < len(amplitudes):
            try:
                intensity = float(amplitudes[index])
            except (TypeError, ValueError):
                intensity = 0.0
        result.append((x, y, z, intensity))
    return result


class XTM60SdkAdapter:
    """Small XT-M60 SDK wrapper for ROS2 publishing.

    The SDK owns the network/USB receiving thread and invokes callbacks. This
    wrapper keeps callbacks lightweight: copy the latest point frame under a
    lock, then let the ROS node publish it from a timer.
    """

    def __init__(self, config: XTM60SdkConfig, logger):
        self.config = config
        self.logger = logger
        self._sdk = None
        self._xintan_sdk = None
        self._lock = threading.Lock()
        self._latest_points: Optional[List[XYZI]] = None
        self._latest_frame_id: Optional[int] = None
        self._latest_sdk_stamp: Optional[Tuple[int, int]] = None
        self._connected = False
        self._measurement_started = False
        self._last_connect_attempt = 0.0
        self._last_error = ""

    @property
    def connected(self) -> bool:
        return self._connected

    @property
    def measurement_started(self) -> bool:
        return self._measurement_started

    @property
    def last_error(self) -> str:
        return self._last_error

    def start(self) -> None:
        sdk_root = find_xtsdk_root(self.config.sdk_root)
        lib_dir = configure_xtsdk_import_path(sdk_root)
        self.logger.info(f"XT-M60 SDK root: {sdk_root}")
        self.logger.info(f"XT-M60 SDK lib: {lib_dir}")

        try:
            import xintan_sdk
        except ImportError as exc:
            self._last_error = str(exc)
            raise RuntimeError(
                "failed to import xintan_sdk; check Python version and SDK binary"
            ) from exc

        self._xintan_sdk = xintan_sdk
        self._sdk = xintan_sdk.XtSdk()
        self._sdk.setCallback(self._on_event, self._on_frame)
        self._apply_optional_filters()
        self._configure_connection()
        self._sdk.startup()
        self._last_connect_attempt = self._now()

    def stop(self) -> None:
        self._measurement_started = False
        self._connected = False
        if self._sdk is None:
            return
        try:
            self._sdk.stop()
        except Exception as exc:
            self.logger.warning(f"XT-M60 SDK stop failed: {exc}")
        try:
            self._sdk.shutdown()
        except Exception as exc:
            self.logger.warning(f"XT-M60 SDK shutdown failed: {exc}")
        self._sdk = None

    def poll(self) -> None:
        if self._sdk is None:
            return
        try:
            self._connected = bool(self._sdk.isconnect())
            if self._connected and not self._measurement_started:
                self._start_measurement()
        except Exception as exc:
            self._last_error = str(exc)
            self.logger.warning(f"XT-M60 SDK poll failed: {exc}")
            self._connected = False

    def take_latest_points(self) -> Tuple[Optional[List[XYZI]], Optional[Tuple[int, int]]]:
        with self._lock:
            points = self._latest_points
            stamp = self._latest_sdk_stamp
            self._latest_points = None
            self._latest_sdk_stamp = None
        return points, stamp

    def _configure_connection(self) -> None:
        mode = self.config.connection_mode.lower().strip()
        if mode == "ethernet":
            ok = self._sdk.setConnectIpaddress(self.config.ip_address)
            if not ok:
                self.logger.warning(f"XT-M60 setConnectIpaddress returned false: {self.config.ip_address}")
        elif mode == "usb":
            if not self.config.serial_port:
                raise ValueError("serial_port is required when connection_mode is usb")
            ok = self._sdk.setConnectSerialportName(self.config.serial_port)
            if not ok:
                self.logger.warning(f"XT-M60 setConnectSerialportName returned false: {self.config.serial_port}")
        else:
            raise ValueError("connection_mode must be ethernet or usb")

    def _apply_optional_filters(self) -> None:
        if not self.config.enable_sdk_filters:
            return
        calls = [
            ("setSdkMedianFilter", (int(self.config.median_size),)),
            ("setSdkEdgeFilter", (int(self.config.edge_threshold),)),
        ]
        for name, args in calls:
            func = getattr(self._sdk, name, None)
            if func is None:
                continue
            try:
                func(*args)
            except Exception as exc:
                self.logger.warning(f"XT-M60 optional filter {name} failed: {exc}")

        kalman = getattr(self._sdk, "setSdkKalmanFilter", None)
        if kalman is not None:
            try:
                kalman(
                    int(self.config.kalman_factor),
                    int(self.config.kalman_threshold),
                    int(self.config.kalman_range),
                )
            except TypeError:
                try:
                    kalman(int(self.config.kalman_factor), int(self.config.kalman_threshold))
                except Exception as exc:
                    self.logger.warning(f"XT-M60 optional filter setSdkKalmanFilter failed: {exc}")
            except Exception as exc:
                self.logger.warning(f"XT-M60 optional filter setSdkKalmanFilter failed: {exc}")

    def _start_measurement(self) -> None:
        image_type = self._xintan_sdk.ImageType(int(self.config.image_type))
        try:
            try:
                self._sdk.start(image_type, False)
            except TypeError:
                self._sdk.start(image_type)
            self._measurement_started = True
            self.logger.info(f"XT-M60 measurement started with ImageType({self.config.image_type})")
        except Exception as exc:
            self._last_error = str(exc)
            self.logger.error(f"XT-M60 measurement start failed: {exc}")

    def _on_event(self, event) -> None:
        event_name = str(getattr(event, "eventstr", ""))
        cmd_id = getattr(event, "cmdid", None)
        if event_name == "sdkState":
            try:
                self._connected = bool(self._sdk is not None and self._sdk.isconnect())
            except Exception:
                self._connected = False
            if self._connected:
                self.logger.info(f"XT-M60 SDK connected event cmdid={cmd_id}")

    def _on_frame(self, frame) -> None:
        points = extract_xyzi_points(
            frame,
            unit_scale=self.config.point_unit_scale,
            range_min=self.config.range_min,
            range_max=self.config.range_max,
        )
        if not points:
            return
        sdk_stamp = None
        sec = getattr(frame, "timeStampS", None)
        nsec = getattr(frame, "timeStampNS", None)
        if isinstance(sec, int) and isinstance(nsec, int):
            sdk_stamp = (sec, nsec)
        frame_id = getattr(frame, "frame_id", None)
        with self._lock:
            self._latest_points = points
            self._latest_sdk_stamp = sdk_stamp
            self._latest_frame_id = frame_id

    @staticmethod
    def _now() -> float:
        try:
            import time

            return time.monotonic()
        except Exception:
            return 0.0


class XTM60AdapterNode(Node):
    def __init__(self):
        super().__init__("xtm60_adapter_node")
        self.declare_parameter("mode", "real")
        self.declare_parameter("frame_id", "laser_link")
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("sdk_root", "")
        self.declare_parameter("connection_mode", "ethernet")
        self.declare_parameter("ip_address", "192.168.0.101")
        self.declare_parameter("serial_port", "")
        self.declare_parameter("image_type", 4)
        self.declare_parameter("point_unit_scale", 1.0)
        self.declare_parameter("range_min", 0.05)
        self.declare_parameter("range_max", 20.0)
        self.declare_parameter("publish_intensity", True)
        self.declare_parameter("enable_sdk_filters", True)
        self.declare_parameter("kalman_factor", 300)
        self.declare_parameter("kalman_threshold", 200)
        self.declare_parameter("kalman_range", 2000)
        self.declare_parameter("median_size", 3)
        self.declare_parameter("edge_threshold", 150)

        self.mode = self.get_parameter("mode").value
        self.frame_id = self.get_parameter("frame_id").value
        self.pub = self.create_publisher(PointCloud2, "/xtm60/points", 10)
        self.status_pub = self.create_publisher(String, "/xtm60/status", 10)
        self.adapter: Optional[XTM60SdkAdapter] = None
        self.sdk_start_failed = False

        if self.mode == "real":
            self.adapter = XTM60SdkAdapter(self._read_config(), self.get_logger())
            try:
                self.adapter.start()
            except Exception as exc:
                self.sdk_start_failed = True
                self.get_logger().error(f"XT-M60 SDK startup failed: {exc}")

        rate = max(1.0, float(self.get_parameter("publish_rate_hz").value))
        self.timer = self.create_timer(1.0 / rate, self.tick)

    def destroy_node(self):
        if self.adapter is not None:
            self.adapter.stop()
        super().destroy_node()

    def tick(self):
        if self.mode == "mock":
            self.pub.publish(self.make_mock_cloud())
            self._publish_status("mock publishing /xtm60/points")
            return

        if self.adapter is None or self.sdk_start_failed:
            self._publish_status("error: XT-M60 SDK not running")
            return

        self.adapter.poll()
        points, sdk_stamp = self.adapter.take_latest_points()
        if points:
            self.pub.publish(self._make_cloud(points, sdk_stamp))
        state = "connected" if self.adapter.connected else "disconnected"
        started = "measuring" if self.adapter.measurement_started else "waiting_measurement"
        self._publish_status(f"{state}; {started}; {self.adapter.last_error}")

    def _read_config(self) -> XTM60SdkConfig:
        return XTM60SdkConfig(
            sdk_root=str(self.get_parameter("sdk_root").value),
            connection_mode=str(self.get_parameter("connection_mode").value),
            ip_address=str(self.get_parameter("ip_address").value),
            serial_port=str(self.get_parameter("serial_port").value),
            image_type=int(self.get_parameter("image_type").value),
            frame_id=str(self.get_parameter("frame_id").value),
            point_unit_scale=float(self.get_parameter("point_unit_scale").value),
            range_min=float(self.get_parameter("range_min").value),
            range_max=float(self.get_parameter("range_max").value),
            publish_intensity=bool(self.get_parameter("publish_intensity").value),
            enable_sdk_filters=bool(self.get_parameter("enable_sdk_filters").value),
            kalman_factor=int(self.get_parameter("kalman_factor").value),
            kalman_threshold=int(self.get_parameter("kalman_threshold").value),
            kalman_range=int(self.get_parameter("kalman_range").value),
            median_size=int(self.get_parameter("median_size").value),
            edge_threshold=int(self.get_parameter("edge_threshold").value),
        )

    def _make_cloud(self, points: Sequence[XYZI], sdk_stamp: Optional[Tuple[int, int]]):
        header = Header()
        header.frame_id = self.frame_id
        if sdk_stamp is not None and sdk_stamp[0] > 0:
            header.stamp.sec = int(sdk_stamp[0])
            header.stamp.nanosec = int(sdk_stamp[1])
        else:
            header.stamp = self.get_clock().now().to_msg()

        if bool(self.get_parameter("publish_intensity").value):
            fields = [
                PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
                PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
            ]
            return point_cloud2.create_cloud(header, fields, list(points))
        return point_cloud2.create_cloud_xyz32(header, [(x, y, z) for x, y, z, _i in points])

    def _publish_status(self, text: str):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)

    def make_mock_cloud(self):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.frame_id
        points = []
        for deg in range(-60, 61, 3):
            angle = math.radians(deg)
            points.append([2.0 * math.cos(angle), 2.0 * math.sin(angle), 0.0])
        return point_cloud2.create_cloud_xyz32(header, points)


def main(args=None):
    if rclpy is None:
        raise RuntimeError("ROS2 Python packages are required to run this node")
    rclpy.init(args=args)
    node = XTM60AdapterNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
