from dataclasses import dataclass
from typing import Dict, Optional, Sequence

try:
    import cv2
except ImportError:
    cv2 = None

try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image
except ImportError:
    rclpy = None
    Node = object
    Image = None


@dataclass
class CameraConfig:
    name: str
    topic: str
    frame_id: str
    device: str
    enabled: bool = True


class CameraAdapter:
    def __init__(self, camera: CameraConfig, width: int, height: int, fps: float):
        self.camera = camera
        self.width = width
        self.height = height
        self.fps = fps
        self.capture = None

    def open(self):
        if self.capture is not None:
            return
        if cv2 is None:
            raise RuntimeError("opencv-python is required for camera real mode")
        device = int(self.camera.device) if str(self.camera.device).isdigit() else self.camera.device
        self.capture = cv2.VideoCapture(device)
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
            return None
        return frame


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


class CameraAdapterNode(Node):
    def __init__(self):
        super().__init__("camera_adapter_node")
        self.declare_parameter("mode", "real")
        self.declare_parameter("publish_rate_hz", 15.0)
        self.declare_parameter("width", 640)
        self.declare_parameter("height", 480)
        self.declare_parameter("fps", 30.0)
        self.declare_parameter("enabled_cameras", ["front", "left"])
        self.declare_parameter("front_device", "0")
        self.declare_parameter("left_device", "1")
        self.declare_parameter("right_device", "2")
        self.declare_parameter("rear_device", "3")

        self.mode = self.get_parameter("mode").value
        enabled = set(self.get_parameter("enabled_cameras").value)
        self.camera_configs = self._make_camera_configs(enabled)
        width = int(self.get_parameter("width").value)
        height = int(self.get_parameter("height").value)
        fps = float(self.get_parameter("fps").value)
        self.adapters = {
            cfg.name: CameraAdapter(cfg, width, height, fps)
            for cfg in self.camera_configs
            if cfg.enabled
        }
        self.pubs = {
            cfg.name: self.create_publisher(Image, cfg.topic, 10)
            for cfg in self.camera_configs
            if cfg.enabled
        }
        self.warned: Dict[str, bool] = {}
        self.timer = self.create_timer(
            1.0 / float(self.get_parameter("publish_rate_hz").value), self.tick
        )

    def _make_camera_configs(self, enabled: Sequence[str]):
        return [
            CameraConfig("front", "/camera/front/image_raw", "camera_front_link", self.get_parameter("front_device").value, "front" in enabled),
            CameraConfig("left", "/camera/left/image_raw", "camera_left_link", self.get_parameter("left_device").value, "left" in enabled),
            CameraConfig("right", "/camera/right/image_raw", "camera_right_link", self.get_parameter("right_device").value, "right" in enabled),
            CameraConfig("rear", "/camera/rear/image_raw", "camera_rear_link", self.get_parameter("rear_device").value, "rear" in enabled),
        ]

    def tick(self):
        stamp = self.get_clock().now().to_msg()
        for config in self.camera_configs:
            if not config.enabled:
                continue
            if self.mode == "mock":
                msg = self._mock_image(stamp, config.frame_id)
                self.pubs[config.name].publish(msg)
                continue
            try:
                frame = self.adapters[config.name].read_image()
            except Exception as exc:
                if not self.warned.get(config.name):
                    self.get_logger().warning(f"{config.name} camera read failed: {exc}")
                    self.warned[config.name] = True
                continue
            if frame is not None:
                self.pubs[config.name].publish(cv_frame_to_image(frame, stamp, config.frame_id))

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
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
