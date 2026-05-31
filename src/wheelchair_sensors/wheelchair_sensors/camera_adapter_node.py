from dataclasses import dataclass
from typing import Dict, Optional, Sequence

try:
    import cv2
except ImportError:
    cv2 = None

try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import CameraInfo, Image
except ImportError:
    rclpy = None
    Node = object
    Image = None
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
    def __init__(self, camera: CameraConfig, width: int, height: int, fps: float, fourcc: str = "MJPG"):
        self.camera = camera
        self.width = width
        self.height = height
        self.fps = fps
        self.fourcc = fourcc
        self.capture = None

    def open(self):
        if self.capture is not None:
            return
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
            raise RuntimeError(f"failed to open camera {self.camera.name} at {self.camera.device}")

    def close(self):
        if self.capture is not None:
            self.capture.release()
            self.capture = None

    def read_image(self) -> Optional[bytes]:
        self.open()
        ok, frame = self.capture.read()
        if not ok or frame is None:
            self.close()
            return None
        return frame


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
        self.adapters = {
            cfg.name: CameraAdapter(cfg, width, height, fps, fourcc)
            for cfg in self.camera_configs
            if cfg.enabled
        }
        self.pubs = {
            cfg.name: self.create_publisher(Image, cfg.topic, 10)
            for cfg in self.camera_configs
            if cfg.enabled
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
        self.timer = self.create_timer(
            1.0 / float(self.get_parameter("publish_rate_hz").value), self.tick
        )

    def _make_camera_configs(self, enabled: Sequence[str]):
        return [
            CameraConfig("front", "/camera/front/image_raw", "camera_front_link", self.get_parameter("front_device").value, "front" in enabled, int(self.get_parameter("front_rotate_deg").value)),
            CameraConfig("left", "/camera/left/image_raw", "camera_left_link", self.get_parameter("left_device").value, "left" in enabled, int(self.get_parameter("left_rotate_deg").value)),
            CameraConfig("right", "/camera/right/image_raw", "camera_right_link", self.get_parameter("right_device").value, "right" in enabled, int(self.get_parameter("right_rotate_deg").value)),
            CameraConfig("rear", "/camera/rear/image_raw", "camera_rear_link", self.get_parameter("rear_device").value, "rear" in enabled, int(self.get_parameter("rear_rotate_deg").value)),
        ]

    def tick(self):
        stamp = self.get_clock().now().to_msg()
        for config in self.camera_configs:
            if not config.enabled:
                continue
            if self.mode == "mock":
                msg = self._mock_image(stamp, config.frame_id)
                self.pubs[config.name].publish(msg)
                self._publish_info(config.name, config.frame_id, stamp)
                continue
            try:
                frame = self.adapters[config.name].read_image()
            except Exception as exc:
                if not self.warned.get(config.name):
                    self.get_logger().warning(f"{config.name} camera read failed: {exc}")
                    self.warned[config.name] = True
                continue
            if frame is not None:
                frame = rotate_frame(frame, config.rotate_deg)
                self.pubs[config.name].publish(cv_frame_to_image(frame, stamp, config.frame_id))
                self._publish_info(config.name, config.frame_id, stamp)
            elif not self.warned.get(config.name):
                self.get_logger().warning(f"{config.name} camera opened but returned no frame")
                self.warned[config.name] = True

    def _publish_info(self, name, frame_id, stamp):
        info = self.cam_infos.get(name)
        if info is None:
            return
        info.header.stamp = stamp
        info.header.frame_id = frame_id
        self.info_pubs[name].publish(info)

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
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception:
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
