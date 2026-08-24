from dataclasses import dataclass
from functools import partial
import threading
import time
from typing import Callable, Dict, Optional, Sequence

import numpy as np

try:
    import cv2
except ImportError:
    cv2 = None

try:
    import gi

    gi.require_version("Gst", "1.0")
    from gi.repository import Gst

    Gst.init(None)
except (ImportError, ValueError):
    Gst = None

try:
    import rclpy
    from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
    from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import (
        DurabilityPolicy,
        HistoryPolicy,
        QoSProfile,
        ReliabilityPolicy,
    )
    from sensor_msgs.msg import CameraInfo, CompressedImage, Image
except ImportError:
    rclpy = None
    ExternalShutdownException = RuntimeError
    MutuallyExclusiveCallbackGroup = None
    MultiThreadedExecutor = None
    QoSProfile = None
    Node = object
    Image = None
    CompressedImage = None
    CameraInfo = None


@dataclass
class CameraConfig:
    name: str
    topic: str
    frame_id: str
    device: str
    enabled: bool = True
    rotate_deg: int = 0


class CameraAdapter:
    def __init__(
        self,
        camera: CameraConfig,
        width: int,
        height: int,
        fps: float,
        fourcc: str = "MJPG",
        reconnect_interval_sec: float = 3.0,
    ):
        self.camera = camera
        self.width = width
        self.height = height
        self.fps = fps
        self.fourcc = fourcc
        self.reconnect_interval_sec = max(0.0, float(reconnect_interval_sec))
        self.capture = None
        self.last_open_attempt = 0.0

    def open(self):
        if self.capture is not None:
            return True
        now = time.monotonic()
        if now - self.last_open_attempt < self.reconnect_interval_sec:
            return False
        self.last_open_attempt = now
        if cv2 is None:
            raise RuntimeError("opencv-python is required for camera real mode")
        device = int(self.camera.device) if str(self.camera.device).isdigit() else self.camera.device
        use_v4l2 = isinstance(device, int) or str(device).startswith("/dev/video")
        backend = getattr(cv2, "CAP_V4L2", 0) if use_v4l2 else 0
        self.capture = cv2.VideoCapture(device, backend) if backend else cv2.VideoCapture(device)
        if self.fourcc:
            # Compressed (MJPG) uses ~1/7-1/10 the USB bandwidth of YUYV, which
            # is what lets multiple USB cameras share bus/hub bandwidth.
            self.capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*self.fourcc))
        if self.width > 0:
            self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        if self.height > 0:
            self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        if self.fps > 0:
            self.capture.set(cv2.CAP_PROP_FPS, self.fps)
        self.capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if not self.capture.isOpened():
            self.capture.release()
            self.capture = None
            return False
        return True

    def close(self):
        if self.capture is not None:
            self.capture.release()
            self.capture = None

    def read_image(self) -> Optional[bytes]:
        if not self.open():
            return None
        ok, frame = self.capture.read()
        if not ok or frame is None:
            self.close()
            return None
        return frame


def gstreamer_mjpeg_pipeline(
    device: str,
    width: int,
    height: int,
    fps: float,
    appsink_name: str,
    repair_jpeg: bool = False,
    jpeg_quality: int = 85,
) -> str:
    """Build a bounded-latency UVC MJPEG passthrough pipeline."""
    safe_device = str(device).replace("\\", "\\\\").replace('"', '\\"')
    fps_numerator = max(1, int(round(float(fps))))
    quality = max(1, min(100, int(jpeg_quality)))
    repair = f"jpegdec ! jpegenc quality={quality} ! " if repair_jpeg else ""
    return (
        f'v4l2src device="{safe_device}" ! '
        f'image/jpeg,width={int(width)},height={int(height)},'
        f'framerate={fps_numerator}/1 ! '
        f'{repair}'
        f'appsink name={appsink_name} sync=false max-buffers=1 drop=true'
    )


class GStreamerMjpegAdapter:
    """Read the camera's original MJPEG payload without decoding it first."""

    def __init__(
        self,
        camera: CameraConfig,
        width: int,
        height: int,
        fps: float,
        reconnect_interval_sec: float = 3.0,
        sample_timeout_sec: float = 0.25,
        repair_jpeg: bool = False,
        jpeg_quality: int = 85,
    ):
        self.camera = camera
        self.width = width
        self.height = height
        self.fps = fps
        self.reconnect_interval_sec = max(0.0, float(reconnect_interval_sec))
        self.sample_timeout_ns = max(1, int(float(sample_timeout_sec) * 1e9))
        self.repair_jpeg = bool(repair_jpeg)
        self.jpeg_quality = max(1, min(100, int(jpeg_quality)))
        self.pipeline = None
        self.sink = None
        self.last_open_attempt = 0.0
        self._lock = threading.Lock()
        self._frame_condition = threading.Condition(self._lock)
        self._stop_event = threading.Event()
        self._capture_thread = None
        self._dispatch_thread = None
        self._latest_jpeg = None
        self._latest_sequence = 0
        self._last_read_sequence = 0
        self._frame_callback: Optional[Callable[[bytes], None]] = None

    def open(self):
        if self.pipeline is not None:
            return True
        now = time.monotonic()
        if now - self.last_open_attempt < self.reconnect_interval_sec:
            return False
        self.last_open_attempt = now
        if Gst is None:
            raise RuntimeError("GStreamer Python bindings are required for MJPEG passthrough")
        name = f"smartwheel_{self.camera.name}_jpeg"
        try:
            pipeline = Gst.parse_launch(
                gstreamer_mjpeg_pipeline(
                    self.camera.device,
                    self.width,
                    self.height,
                    self.fps,
                    name,
                    self.repair_jpeg,
                    self.jpeg_quality,
                )
            )
            sink = pipeline.get_by_name(name)
            if sink is None:
                raise RuntimeError("GStreamer appsink was not created")
            if pipeline.set_state(Gst.State.PLAYING) == Gst.StateChangeReturn.FAILURE:
                raise RuntimeError("GStreamer pipeline failed to enter PLAYING")
        except Exception:
            if 'pipeline' in locals():
                pipeline.set_state(Gst.State.NULL)
            raise
        self.pipeline = pipeline
        self.sink = sink
        self._stop_event.clear()
        self._capture_thread = threading.Thread(
            target=self._capture_loop,
            args=(pipeline, sink),
            name=f"{self.camera.name}_mjpeg_capture",
            daemon=True,
        )
        self._capture_thread.start()
        if self._frame_callback is not None:
            self._dispatch_thread = threading.Thread(
                target=self._dispatch_loop,
                name=f"{self.camera.name}_mjpeg_dispatch",
                daemon=True,
            )
            self._dispatch_thread.start()
        return True

    def close(self):
        self._stop_event.set()
        with self._frame_condition:
            self._frame_condition.notify_all()
        if self.pipeline is not None:
            self.pipeline.set_state(Gst.State.NULL)
        if (
            self._capture_thread is not None
            and self._capture_thread is not threading.current_thread()
        ):
            self._capture_thread.join(timeout=1.0)
        if (
            self._dispatch_thread is not None
            and self._dispatch_thread is not threading.current_thread()
        ):
            self._dispatch_thread.join(timeout=1.0)
        self.pipeline = None
        self.sink = None
        self._capture_thread = None
        self._dispatch_thread = None
        with self._lock:
            self._latest_jpeg = None
            self._latest_sequence = 0
            self._last_read_sequence = 0

    def _capture_loop(self, pipeline, sink):
        """Continuously drain appsink and retain only the newest complete JPEG."""
        while not self._stop_event.is_set():
            sample = sink.emit("try-pull-sample", self.sample_timeout_ns)
            if sample is None:
                bus = pipeline.get_bus()
                message = bus.pop_filtered(
                    Gst.MessageType.ERROR | Gst.MessageType.EOS
                )
                if message is not None:
                    break
                continue
            buffer = sample.get_buffer()
            ok, mapping = buffer.map(Gst.MapFlags.READ)
            if not ok:
                continue
            try:
                data = bytes(mapping.data)
            finally:
                buffer.unmap(mapping)
            if not (data.startswith(b"\xff\xd8") and data.endswith(b"\xff\xd9")):
                continue
            with self._lock:
                self._latest_jpeg = data
                self._latest_sequence += 1
                self._frame_condition.notify()

    def _dispatch_loop(self):
        """Publish the newest frame without ever blocking the capture thread."""
        last_sequence = 0
        while not self._stop_event.is_set():
            with self._frame_condition:
                self._frame_condition.wait_for(
                    lambda: self._stop_event.is_set()
                    or self._latest_sequence > last_sequence,
                    timeout=0.5,
                )
                if self._stop_event.is_set():
                    return
                data = self._latest_jpeg
                last_sequence = self._latest_sequence
                callback = self._frame_callback
            if data is None or callback is None:
                continue
            try:
                callback(data)
            except Exception:
                # The node callback handles and logs normal publish/decode
                # errors. Keep draining the latest-frame cache if shutdown or
                # another transient ROS exception crosses this boundary.
                continue

    def read_jpeg(self) -> Optional[bytes]:
        if not self.open():
            return None
        if self._capture_thread is None or not self._capture_thread.is_alive():
            self.close()
            return None
        with self._lock:
            if self._latest_sequence == self._last_read_sequence:
                return None
            self._last_read_sequence = self._latest_sequence
            return self._latest_jpeg

    def is_running(self) -> bool:
        """Return whether the asynchronous camera pipeline is still alive."""
        capture_running = bool(
            self.pipeline is not None
            and self._capture_thread is not None
            and self._capture_thread.is_alive()
        )
        if self._frame_callback is None:
            return capture_running
        return bool(
            capture_running
            and self._dispatch_thread is not None
            and self._dispatch_thread.is_alive()
        )

    def set_frame_callback(self, callback: Optional[Callable[[bytes], None]]):
        """Publish from the capture thread instead of polling its one-frame cache."""
        with self._lock:
            self._frame_callback = callback


def rotate_frame(frame, deg: int):
    """Rotate a frame by 0/90/180/270 deg for a physically mis-mounted camera."""
    if not deg or cv2 is None:
        return frame
    codes = {90: cv2.ROTATE_90_CLOCKWISE, 180: cv2.ROTATE_180, 270: cv2.ROTATE_90_COUNTERCLOCKWISE}
    code = codes.get(int(deg) % 360)
    return cv2.rotate(frame, code) if code is not None else frame


def cv_frame_to_image(frame, stamp, frame_id: str):
    msg = Image()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height = int(frame.shape[0])
    msg.width = int(frame.shape[1])
    msg.encoding = "bgr8"
    msg.is_bigendian = False
    msg.step = int(frame.strides[0])
    msg.data = frame.tobytes()
    return msg


def cv_frame_to_compressed(frame, stamp, frame_id: str, jpeg_quality: int = 85):
    quality = max(1, min(100, int(jpeg_quality)))
    ok, encoded = cv2.imencode(
        ".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), quality]
    )
    if not ok:
        raise RuntimeError("OpenCV failed to encode camera frame as JPEG")
    msg = CompressedImage()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.format = "jpeg"
    msg.data = encoded.tobytes()
    return msg


def jpeg_bytes_to_compressed(data: bytes, stamp, frame_id: str):
    msg = CompressedImage()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.format = "jpeg"
    msg.data = data
    return msg


def jpeg_bytes_to_cv_frame(data: bytes):
    if cv2 is None:
        raise RuntimeError("opencv-python is required to decode MJPEG for raw output")
    frame = cv2.imdecode(np.frombuffer(data, dtype=np.uint8), cv2.IMREAD_COLOR)
    if frame is None:
        raise RuntimeError("OpenCV failed to decode camera MJPEG frame")
    return frame


def load_camera_info(path: str):
    """Load a ROS camera-calibration YAML (from cameracalibrator) into CameraInfo."""
    import yaml
    with open(path) as f:
        d = yaml.safe_load(f) or {}
    info = CameraInfo()
    info.width = int(d["image_width"])
    info.height = int(d["image_height"])
    info.distortion_model = str(d.get("distortion_model", "plumb_bob"))
    info.k = [float(x) for x in d["camera_matrix"]["data"]]
    info.d = [float(x) for x in d["distortion_coefficients"]["data"]]
    info.r = [float(x) for x in d["rectification_matrix"]["data"]]
    info.p = [float(x) for x in d["projection_matrix"]["data"]]
    return info


class CameraAdapterNode(Node):
    def __init__(self):
        super().__init__("camera_adapter_node")
        self.declare_parameter("mode", "real")
        self.declare_parameter("publish_rate_hz", 15.0)
        self.declare_parameter("width", 640)
        self.declare_parameter("height", 480)
        self.declare_parameter("fps", 30.0)
        self.declare_parameter("fourcc", "MJPG")
        self.declare_parameter("publish_raw", True)
        self.declare_parameter("publish_compressed", False)
        self.declare_parameter("compressed_passthrough", False)
        # Humble infers an untyped [] as BYTE_ARRAY, so use an empty string
        # sentinel to keep YAML overrides such as [right] a STRING_ARRAY.
        self.declare_parameter("compressed_repair_cameras", [""])
        self.declare_parameter("jpeg_quality", 85)
        self.declare_parameter("camera_reconnect_interval_sec", 3.0)
        self.declare_parameter("enabled_cameras", ["front"])
        self.declare_parameter("front_device", "0")
        self.declare_parameter("left_device", "1")
        self.declare_parameter("right_device", "2")
        self.declare_parameter("rear_device", "3")
        self.declare_parameter("front_rotate_deg", 0)
        self.declare_parameter("left_rotate_deg", 0)
        self.declare_parameter("right_rotate_deg", 0)
        self.declare_parameter("rear_rotate_deg", 0)
        self.declare_parameter("front_camera_info_url", "")
        self.declare_parameter("left_camera_info_url", "")
        self.declare_parameter("right_camera_info_url", "")
        self.declare_parameter("rear_camera_info_url", "")

        self.mode = self.get_parameter("mode").value
        enabled = set(self.get_parameter("enabled_cameras").value)
        self.camera_configs = self._make_camera_configs(enabled)
        width = int(self.get_parameter("width").value)
        height = int(self.get_parameter("height").value)
        fps = float(self.get_parameter("fps").value)
        fourcc = str(self.get_parameter("fourcc").value)
        self.publish_raw = bool(self.get_parameter("publish_raw").value)
        self.publish_compressed = bool(
            self.get_parameter("publish_compressed").value
        )
        requested_passthrough = bool(
            self.get_parameter("compressed_passthrough").value
        )
        self.compressed_passthrough_requested = requested_passthrough
        passthrough_available = bool(
            requested_passthrough
            and self.mode == "real"
            and self.publish_compressed
            and fourcc.upper() in ("MJPG", "MJPEG")
            and Gst is not None
        )
        if requested_passthrough and not passthrough_available:
            self.get_logger().warning(
                "compressed_passthrough requested but unavailable; falling back to OpenCV"
            )
        self.jpeg_quality = int(self.get_parameter("jpeg_quality").value)
        if not self.publish_raw and not self.publish_compressed:
            raise ValueError("at least one of publish_raw/publish_compressed must be true")
        reconnect_interval_sec = float(
            self.get_parameter("camera_reconnect_interval_sec").value
        )
        compressed_repair_cameras = {
            str(name)
            for name in self.get_parameter("compressed_repair_cameras").value
            if str(name)
        }
        self.passthrough_cameras = {
            cfg.name
            for cfg in self.camera_configs
            if cfg.enabled and passthrough_available
        }
        self.adapters = {
            cfg.name: (
                GStreamerMjpegAdapter(
                    cfg,
                    width,
                    height,
                    fps,
                    reconnect_interval_sec,
                    repair_jpeg=cfg.name in compressed_repair_cameras,
                    jpeg_quality=self.jpeg_quality,
                )
                if cfg.name in self.passthrough_cameras
                else CameraAdapter(
                    cfg, width, height, fps, fourcc, reconnect_interval_sec
                )
            )
            for cfg in self.camera_configs
            if cfg.enabled
        }
        image_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.pubs = {
            cfg.name: self.create_publisher(Image, cfg.topic, image_qos)
            for cfg in self.camera_configs
            if cfg.enabled and self.publish_raw
        }
        self.compressed_pubs = {
            cfg.name: self.create_publisher(
                CompressedImage, f"{cfg.topic}/compressed", image_qos
            )
            for cfg in self.camera_configs
            if cfg.enabled and self.publish_compressed
        }
        self.info_pubs = {}
        self.cam_infos = {}
        for cfg in self.camera_configs:
            if not cfg.enabled:
                continue
            self.info_pubs[cfg.name] = self.create_publisher(CameraInfo, f"/camera/{cfg.name}/camera_info", 10)
            url = str(self.get_parameter(f"{cfg.name}_camera_info_url").value).strip()
            if url:
                try:
                    self.cam_infos[cfg.name] = load_camera_info(url)
                    self.get_logger().info(f"{cfg.name} camera_info loaded from {url}")
                except Exception as exc:
                    self.get_logger().warning(f"{cfg.name} camera_info_url failed ({url}): {exc}")
        self.warned: Dict[str, bool] = {}
        for config in self.camera_configs:
            if config.enabled and config.name in self.passthrough_cameras:
                adapter = self.adapters[config.name]
                adapter.set_frame_callback(
                    partial(self._publish_passthrough_jpeg, config)
                )
                try:
                    adapter.open()
                except Exception as exc:
                    self.get_logger().warning(
                        f"{config.name} camera MJPEG startup failed: {exc}"
                    )
                    self.warned[config.name] = True
        period = 1.0 / float(self.get_parameter("publish_rate_hz").value)
        self._camera_callback_groups = {}
        self._camera_timers = {}
        for config in self.camera_configs:
            if not config.enabled:
                continue
            group = MutuallyExclusiveCallbackGroup()
            self._camera_callback_groups[config.name] = group
            callback = (
                partial(self._ensure_passthrough_running, config)
                if config.name in self.passthrough_cameras
                else partial(self._tick_camera, config)
            )
            timer_period = (
                min(1.0, max(0.25, reconnect_interval_sec))
                if config.name in self.passthrough_cameras
                else period
            )
            self._camera_timers[config.name] = self.create_timer(
                timer_period,
                callback,
                callback_group=group,
            )

    def _make_camera_configs(self, enabled: Sequence[str]):
        return [
            CameraConfig("front", "/camera/front/image_raw", "camera_front_link", self.get_parameter("front_device").value, "front" in enabled, int(self.get_parameter("front_rotate_deg").value)),
            CameraConfig("left", "/camera/left/image_raw", "camera_left_link", self.get_parameter("left_device").value, "left" in enabled, int(self.get_parameter("left_rotate_deg").value)),
            CameraConfig("right", "/camera/right/image_raw", "camera_right_link", self.get_parameter("right_device").value, "right" in enabled, int(self.get_parameter("right_rotate_deg").value)),
            CameraConfig("rear", "/camera/rear/image_raw", "camera_rear_link", self.get_parameter("rear_device").value, "rear" in enabled, int(self.get_parameter("rear_rotate_deg").value)),
        ]

    def tick(self):
        # Retained as a deterministic test/manual hook. Production timers call
        # each camera independently in a MultiThreadedExecutor, so a missing or
        # slow USB camera cannot throttle the other streams.
        for config in self.camera_configs:
            if not config.enabled:
                continue
            self._tick_camera(config)

    def _tick_camera(self, config: CameraConfig):
        if self.mode == "mock":
            if not rclpy.ok():
                return
            stamp = self.get_clock().now().to_msg()
            if self._raw_has_subscriber(config.name):
                self._publish_safely(
                    self.pubs[config.name],
                    self._mock_image(stamp, config.frame_id),
                    config.name,
                )
            self._publish_info(config.name, config.frame_id, stamp)
            return
        if config.name in self.passthrough_cameras:
            self._tick_camera_passthrough(config)
            return
        try:
            frame = self.adapters[config.name].read_image()
        except Exception as exc:
            if not self.warned.get(config.name):
                self.get_logger().warning(f"{config.name} camera read failed: {exc}")
                self.warned[config.name] = True
            return
        if frame is not None:
            if not rclpy.ok():
                return
            stamp = self.get_clock().now().to_msg()
            native_frame = frame
            frame = rotate_frame(frame, config.rotate_deg)
            if self.publish_compressed:
                try:
                    # When passthrough mode is enabled, every compressed topic
                    # keeps sensor-native orientation. A camera excluded from
                    # passthrough may be decoded/re-encoded to repair its MJPEG,
                    # but its display rotation still belongs in the RViz panel.
                    compressed_frame = (
                        native_frame
                        if self.compressed_passthrough_requested
                        else frame
                    )
                    compressed = cv_frame_to_compressed(
                        compressed_frame, stamp, config.frame_id, self.jpeg_quality
                    )
                    self._publish_safely(
                        self.compressed_pubs[config.name],
                        compressed,
                        config.name,
                    )
                except Exception as exc:
                    if rclpy.ok() and not self.warned.get(config.name):
                        self.get_logger().warning(
                            f"{config.name} camera JPEG encode failed: {exc}"
                        )
                        self.warned[config.name] = True
            if self._raw_has_subscriber(config.name):
                self._publish_safely(
                    self.pubs[config.name],
                    cv_frame_to_image(frame, stamp, config.frame_id),
                    config.name,
                )
            self._publish_info(config.name, config.frame_id, stamp)
            self.warned[config.name] = False
        elif not self.warned.get(config.name):
            self.get_logger().warning(
                f"{config.name} camera unavailable or returned no frame; "
                f"retrying every {self.adapters[config.name].reconnect_interval_sec:.1f}s"
            )
            self.warned[config.name] = True

    def _tick_camera_passthrough(self, config: CameraConfig):
        try:
            jpeg = self.adapters[config.name].read_jpeg()
        except Exception as exc:
            if not self.warned.get(config.name):
                self.get_logger().warning(
                    f"{config.name} camera MJPEG passthrough failed: {exc}"
                )
                self.warned[config.name] = True
            return
        if jpeg is None:
            # The timer intentionally polls faster than the camera. No new
            # sequence on an individual tick is normal; warn only if the
            # capture pipeline itself stopped.
            if (
                not self.adapters[config.name].is_running()
                and not self.warned.get(config.name)
            ):
                self.get_logger().warning(
                    f"{config.name} camera MJPEG pipeline stopped; "
                    f"retrying every {self.adapters[config.name].reconnect_interval_sec:.1f}s"
                )
                self.warned[config.name] = True
            return
        if not rclpy.ok():
            return
        self._publish_passthrough_jpeg(config, jpeg)

    def _publish_passthrough_jpeg(self, config: CameraConfig, jpeg: bytes):
        if not rclpy.ok():
            return
        stamp = self.get_clock().now().to_msg()
        self._publish_safely(
            self.compressed_pubs[config.name],
            jpeg_bytes_to_compressed(jpeg, stamp, config.frame_id),
            config.name,
        )
        if self._raw_has_subscriber(config.name):
            try:
                frame = rotate_frame(
                    jpeg_bytes_to_cv_frame(jpeg), config.rotate_deg
                )
                self._publish_safely(
                    self.pubs[config.name],
                    cv_frame_to_image(frame, stamp, config.frame_id),
                    config.name,
                )
            except Exception as exc:
                if rclpy.ok() and not self.warned.get(config.name):
                    self.get_logger().warning(
                        f"{config.name} raw-on-demand JPEG decode failed: {exc}"
                    )
                    self.warned[config.name] = True
        self._publish_info(config.name, config.frame_id, stamp)
        self.warned[config.name] = False

    def _ensure_passthrough_running(self, config: CameraConfig):
        if not rclpy.ok():
            return
        adapter = self.adapters[config.name]
        if adapter.is_running():
            return
        adapter.close()
        try:
            adapter.open()
            self.warned[config.name] = False
        except Exception as exc:
            if not self.warned.get(config.name):
                self.get_logger().warning(
                    f"{config.name} camera MJPEG reconnect failed: {exc}"
                )
                self.warned[config.name] = True

    def _publish_info(self, name, frame_id, stamp):
        info = self.cam_infos.get(name)
        if info is None:
            return
        info.header.stamp = stamp
        info.header.frame_id = frame_id
        self._publish_safely(self.info_pubs[name], info, name)

    def _publish_safely(self, publisher, message, camera_name):
        if not rclpy.ok():
            return False
        try:
            publisher.publish(message)
            return True
        except Exception as exc:
            # SIGINT invalidates the ROS context before all executor workers
            # have returned. Do not turn a normal bounded diagnostic shutdown
            # into an unhandled timer-future exception.
            if rclpy.ok() and not self.warned.get(camera_name):
                self.get_logger().warning(
                    f"{camera_name} camera publish failed: {exc}"
                )
                self.warned[camera_name] = True
            return False

    def _raw_has_subscriber(self, camera_name):
        if not self.publish_raw or not rclpy.ok():
            return False
        try:
            return self.pubs[camera_name].get_subscription_count() > 0
        except Exception:
            # The default SIGINT handler may invalidate the context while an
            # executor worker is between its rclpy.ok() check and this call.
            return False

    @staticmethod
    def _mock_image(stamp, frame_id: str):
        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.height = 120
        msg.width = 160
        msg.encoding = "rgb8"
        msg.step = msg.width * 3
        msg.data = bytes([50, 60, 90]) * (msg.width * msg.height)
        return msg

    def destroy_node(self):
        for adapter in self.adapters.values():
            adapter.close()
        super().destroy_node()


def main(args=None):
    if rclpy is None:
        raise RuntimeError("ROS2 Python packages are required to run this node")
    rclpy.init(args=args)
    node = CameraAdapterNode()
    executor = MultiThreadedExecutor(
        num_threads=max(2, len(node.adapters) + 1)
    )
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        pass
    except Exception:
        if rclpy.ok():
            raise
    finally:
        try:
            executor.shutdown()
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
