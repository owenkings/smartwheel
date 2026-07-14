import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

from camera_array_driver import CAMERA_NAMES
from smartwheel_sensor_api.contracts import guard_real_backend


class CameraArrayNode(Node):
    def __init__(self) -> None:
        super().__init__("camera_array_driver")
        self.declare_parameter("mode", "mock")
        self.declare_parameter("hardware_enabled", False)
        self.declare_parameter("width", 160)
        self.declare_parameter("height", 120)
        self.declare_parameter("fps", 5.0)
        self.declare_parameter("drop_every_n", 0)
        self.declare_parameter("blur_every_n", 0)
        guard_real_backend(
            str(self.get_parameter("mode").value),
            bool(self.get_parameter("hardware_enabled").value),
            adapter_ready=False,
        )
        self._width = int(self.get_parameter("width").value)
        self._height = int(self.get_parameter("height").value)
        self._drop_every = int(self.get_parameter("drop_every_n").value)
        self._blur_every = int(self.get_parameter("blur_every_n").value)
        info_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._image_pubs = {}
        self._info_pubs = {}
        for name in CAMERA_NAMES:
            self._image_pubs[name] = self.create_publisher(
                Image, f"/camera/{name}/image_raw", qos_profile_sensor_data
            )
            self._info_pubs[name] = self.create_publisher(
                CameraInfo, f"/camera/{name}/camera_info", info_qos
            )
        self._frame = 0
        self.create_timer(1.0 / max(0.1, float(self.get_parameter("fps").value)), self._tick)

    def _tick(self) -> None:
        self._frame += 1
        if self._drop_every > 0 and self._frame % self._drop_every == 0:
            return
        now = self.get_clock().now().to_msg()
        x = np.arange(self._width, dtype=np.uint8)[None, :]
        y = np.arange(self._height, dtype=np.uint8)[:, None]
        for index, name in enumerate(CAMERA_NAMES):
            image = np.empty((self._height, self._width, 3), dtype=np.uint8)
            image[..., 0] = (x + index * 53 + self._frame) % 255
            image[..., 1] = (y + index * 37) % 255
            image[..., 2] = (x // 2 + y // 2 + index * 29) % 255
            if self._blur_every > 0 and self._frame % self._blur_every == 0:
                image[:] = image.mean(axis=(0, 1), dtype=np.float64).astype(np.uint8)
            msg = Image()
            msg.header.stamp = now
            msg.header.frame_id = f"camera_{name}_link"
            msg.height = self._height
            msg.width = self._width
            msg.encoding = "rgb8"
            msg.is_bigendian = False
            msg.step = self._width * 3
            msg.data = image.tobytes()
            self._image_pubs[name].publish(msg)
            info = CameraInfo()
            info.header = msg.header
            info.height = self._height
            info.width = self._width
            focal = float(self._width) * 0.8
            info.k = [focal, 0.0, self._width / 2.0, 0.0, focal, self._height / 2.0, 0.0, 0.0, 1.0]
            info.p = [focal, 0.0, self._width / 2.0, 0.0, 0.0, focal, self._height / 2.0, 0.0, 0.0, 0.0, 1.0, 0.0]
            info.distortion_model = "plumb_bob"
            info.d = [0.0] * 5
            self._info_pubs[name].publish(info)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CameraArrayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
