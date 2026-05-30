import math
import struct
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

try:
    import serial
except ImportError:
    serial = None

try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Imu
except ImportError:
    rclpy = None
    Node = object
    Imu = None


ACCEL_ID = 0x10
GYRO_ID = 0x20
EULER_ID = 0x40
QUATERNION_ID = 0x41
SAMPLE_TIMESTAMP_ID = 0x51


@dataclass
class YesenseSample:
    accel_mps2: Optional[Tuple[float, float, float]] = None
    gyro_rps: Optional[Tuple[float, float, float]] = None
    euler_rad: Optional[Tuple[float, float, float]] = None
    quat_xyzw: Optional[Tuple[float, float, float, float]] = None
    sample_timestamp_us: Optional[int] = None


def yesense_checksum(data: bytes) -> Tuple[int, int]:
    """Yesense checksum over TID, payload length and payload bytes."""
    check_a = 0
    check_b = 0
    for value in data:
        check_a = (check_a + value) & 0xFF
        check_b = (check_b + check_a) & 0xFF
    return check_a, check_b


def _unpack_i32_triplet(data: bytes) -> Tuple[int, int, int]:
    return struct.unpack("<iii", data[:12])


def euler_to_quaternion(pitch: float, roll: float, yaw: float) -> Tuple[float, float, float, float]:
    """Convert H30 pitch/roll/yaw radians to ROS xyzw quaternion."""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return x, y, z, w


class YesenseParser:
    """Parser for WHEELTEC H30 / Yesense standard binary output frames.

    Frame: 0x59 0x53 + TID(2) + payload_len(1) + TLV payload + checksum(2).
    Values from the vendor examples use little-endian signed integers scaled by
    1e-6. Gyro and Euler outputs are in degrees and are converted to radians.
    """

    def __init__(self):
        self._buffer = bytearray()

    def feed(self, data: bytes) -> List[YesenseSample]:
        if data:
            self._buffer.extend(data)
        frames: List[YesenseSample] = []
        while True:
            start = self._find_header()
            if start < 0:
                if self._buffer[-1:] == b"\x59":
                    self._buffer[:] = b"\x59"
                else:
                    self._buffer.clear()
                return frames
            if start:
                del self._buffer[:start]
            if len(self._buffer) < 7:
                return frames
            payload_len = self._buffer[4]
            frame_len = 7 + payload_len
            if len(self._buffer) < frame_len:
                return frames
            frame = bytes(self._buffer[:frame_len])
            del self._buffer[:frame_len]
            if not self._valid_checksum(frame):
                continue
            sample = self._parse_payload(frame[5 : 5 + payload_len])
            if sample is not None:
                frames.append(sample)

    def _find_header(self) -> int:
        for index in range(max(0, len(self._buffer) - 1)):
            if self._buffer[index] == 0x59 and self._buffer[index + 1] == 0x53:
                return index
        return -1

    @staticmethod
    def _valid_checksum(frame: bytes) -> bool:
        check_a, check_b = yesense_checksum(frame[2:-2])
        # Vendor examples are inconsistent about byte order. Accept both, but
        # emitted hardware frames normally use low byte then high byte on x86.
        return frame[-2:] in (bytes([check_a, check_b]), bytes([check_b, check_a]))

    @staticmethod
    def _parse_payload(payload: bytes) -> Optional[YesenseSample]:
        pos = 0
        sample = YesenseSample()
        parsed_any = False
        while pos + 2 <= len(payload):
            data_id = payload[pos]
            data_len = payload[pos + 1]
            data_start = pos + 2
            data_end = data_start + data_len
            if data_len <= 0 or data_end > len(payload):
                pos += 1
                continue
            data = payload[data_start:data_end]
            if data_id == ACCEL_ID and data_len == 12:
                sample.accel_mps2 = tuple(v * 1e-6 for v in _unpack_i32_triplet(data))
                parsed_any = True
            elif data_id == GYRO_ID and data_len == 12:
                sample.gyro_rps = tuple(math.radians(v * 1e-6) for v in _unpack_i32_triplet(data))
                parsed_any = True
            elif data_id == EULER_ID and data_len == 12:
                pitch, roll, yaw = (math.radians(v * 1e-6) for v in _unpack_i32_triplet(data))
                sample.euler_rad = (pitch, roll, yaw)
                sample.quat_xyzw = euler_to_quaternion(pitch, roll, yaw)
                parsed_any = True
            elif data_id == QUATERNION_ID and data_len == 16:
                q0, q1, q2, q3 = struct.unpack("<iiii", data)
                # Yesense order is q0,q1,q2,q3 = w,x,y,z; ROS uses x,y,z,w.
                sample.quat_xyzw = (q1 * 1e-6, q2 * 1e-6, q3 * 1e-6, q0 * 1e-6)
                parsed_any = True
            elif data_id == SAMPLE_TIMESTAMP_ID and data_len == 4:
                sample.sample_timestamp_us = struct.unpack("<I", data)[0]
                parsed_any = True
            pos = data_end
        return sample if parsed_any else None


@dataclass
class H30ImuAdapter:
    port: str = "/dev/smartwheel_h30_imu"
    baud_rate: int = 460800
    timeout_sec: float = 0.01
    parser: YesenseParser = field(default_factory=YesenseParser)
    _serial: object = None

    def open(self):
        if self._serial is not None:
            return
        if serial is None:
            raise RuntimeError("pyserial is required for H30 real mode")
        self._serial = serial.Serial(
            self.port,
            self.baud_rate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=self.timeout_sec,
        )

    def close(self):
        if self._serial is not None:
            self._serial.close()
            self._serial = None

    def read_samples(self) -> List[YesenseSample]:
        self.open()
        waiting = getattr(self._serial, "in_waiting", 0) or 1
        data = self._serial.read(waiting)
        return self.parser.feed(data)

    def read_sample(self) -> Optional[YesenseSample]:
        samples = self.read_samples()
        return samples[-1] if samples else None


class ImuAdapterNode(Node):
    def __init__(self):
        super().__init__("imu_adapter_node")
        self.declare_parameter("mode", "real")
        self.declare_parameter("frame_id", "imu_link")
        self.declare_parameter("publish_rate_hz", 100.0)
        self.declare_parameter("serial_port", "/dev/smartwheel_h30_imu")
        self.declare_parameter("baud_rate", 460800)
        self.declare_parameter("serial_timeout_sec", 0.01)
        self.declare_parameter("orientation_covariance", [0.05, 0.05, 0.10])
        self.declare_parameter("angular_velocity_covariance", [0.02, 0.02, 0.02])
        self.declare_parameter("linear_acceleration_covariance", [0.05, 0.05, 0.08])
        self.declare_parameter("use_device_timestamp", False)

        self.mode = self.get_parameter("mode").value
        self.frame_id = self.get_parameter("frame_id").value
        self.use_device_timestamp = bool(self.get_parameter("use_device_timestamp").value)
        self._clock_offset_ns = None
        self.adapter = H30ImuAdapter(
            port=self.get_parameter("serial_port").value,
            baud_rate=int(self.get_parameter("baud_rate").value),
            timeout_sec=float(self.get_parameter("serial_timeout_sec").value),
        )
        self.pub = self.create_publisher(Imu, "/imu/data", 10)
        self.warned = False
        self.timer = self.create_timer(
            1.0 / float(self.get_parameter("publish_rate_hz").value), self.tick
        )

    def tick(self):
        if self.mode == "mock":
            msg = Imu()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.frame_id
            msg.orientation.w = 1.0
            msg.linear_acceleration.z = 9.81
            self.pub.publish(msg)
            return
        try:
            samples = self.adapter.read_samples()
        except Exception as exc:
            if not self.warned:
                self.get_logger().warning(f"H30 IMU serial read failed: {exc}")
                self.warned = True
            return
        for sample in samples:
            self.pub.publish(self._sample_to_msg(sample))

    def _sample_to_msg(self, sample: YesenseSample):
        msg = Imu()
        msg.header.stamp = self._stamp(sample)
        msg.header.frame_id = self.frame_id
        if sample.quat_xyzw is not None:
            msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = sample.quat_xyzw
        else:
            msg.orientation.w = 1.0
            msg.orientation_covariance[0] = -1.0
        if sample.gyro_rps is not None:
            msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = sample.gyro_rps
        if sample.accel_mps2 is not None:
            msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = sample.accel_mps2
        self._set_diag_covariance(msg.orientation_covariance, self.get_parameter("orientation_covariance").value)
        self._set_diag_covariance(msg.angular_velocity_covariance, self.get_parameter("angular_velocity_covariance").value)
        self._set_diag_covariance(msg.linear_acceleration_covariance, self.get_parameter("linear_acceleration_covariance").value)
        return msg

    def _stamp(self, sample):
        now = self.get_clock().now().to_msg()
        if not self.use_device_timestamp or getattr(sample, "sample_timestamp_us", None) is None:
            return now
        # Map device uptime (us) to ROS time using a running-min offset (the
        # sample with least transport delay). Opt-in; assumes small long-run
        # drift. Without this, host publish time adds jitter that hurts LIVO/EKF.
        now_ns = now.sec * 1_000_000_000 + now.nanosec
        dev_ns = int(sample.sample_timestamp_us) * 1000
        offset = now_ns - dev_ns
        if self._clock_offset_ns is None or offset < self._clock_offset_ns:
            self._clock_offset_ns = offset
        t_ns = dev_ns + self._clock_offset_ns
        now.sec = int(t_ns // 1_000_000_000)
        now.nanosec = int(t_ns % 1_000_000_000)
        return now

    @staticmethod
    def _set_diag_covariance(covariance, diagonal):
        if covariance[0] == -1.0:
            return
        covariance[0] = float(diagonal[0])
        covariance[4] = float(diagonal[1])
        covariance[8] = float(diagonal[2])

    def destroy_node(self):
        try:
            self.adapter.close()
        finally:
            super().destroy_node()


def main(args=None):
    if rclpy is None:
        raise RuntimeError("ROS2 Python packages are required to run this node")
    rclpy.init(args=args)
    node = ImuAdapterNode()
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
