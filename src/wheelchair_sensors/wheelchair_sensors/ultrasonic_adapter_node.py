import struct
import time
from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

try:
    import serial
except ImportError:
    serial = None

try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Range
except ImportError:
    rclpy = None
    Node = object
    Range = None


def modbus_crc16(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


def append_modbus_crc(frame_without_crc: bytes) -> bytes:
    crc = modbus_crc16(frame_without_crc)
    return frame_without_crc + struct.pack("<H", crc)


def build_read_holding_registers(address: int, register: int = 0x0001, count: int = 1) -> bytes:
    if not 1 <= address <= 247:
        raise ValueError("Modbus address must be 1..247")
    request = struct.pack(">BBHH", address, 0x03, register & 0xFFFF, count & 0xFFFF)
    return append_modbus_crc(request)


def parse_read_holding_registers_response(
    frame: bytes, expected_address: Optional[int] = None
) -> Tuple[int, List[int]]:
    if len(frame) < 5:
        raise ValueError("Modbus response too short")
    body = frame[:-2]
    expected_crc = struct.unpack("<H", frame[-2:])[0]
    actual_crc = modbus_crc16(body)
    if actual_crc != expected_crc:
        raise ValueError(f"bad Modbus CRC: got 0x{expected_crc:04x}, expected 0x{actual_crc:04x}")
    address = frame[0]
    if expected_address is not None and address != expected_address:
        raise ValueError(f"unexpected Modbus address {address}, expected {expected_address}")
    function_code = frame[1]
    if function_code & 0x80:
        raise ValueError(f"Modbus exception response 0x{frame[2]:02x}")
    if function_code != 0x03:
        raise ValueError(f"unexpected Modbus function 0x{function_code:02x}")
    byte_count = frame[2]
    if len(frame) != 3 + byte_count + 2:
        raise ValueError("Modbus byte count does not match frame length")
    if byte_count % 2:
        raise ValueError("Modbus register payload must contain whole uint16 values")
    values = [
        struct.unpack(">H", frame[3 + i : 5 + i])[0]
        for i in range(0, byte_count, 2)
    ]
    return address, values


@dataclass
class UltrasonicSensor:
    index: int
    address: int
    frame_id: str
    enabled: bool = True


class UltrasonicArrayAdapter:
    def __init__(
        self,
        port: str = "/dev/smartwheel_ultrasonic",
        baud_rate: int = 9600,
        timeout_sec: float = 0.08,
        register: int = 0x0001,
        sensors: Optional[Sequence[UltrasonicSensor]] = None,
    ):
        self.port = port
        self.baud_rate = baud_rate
        self.timeout_sec = timeout_sec
        self.register = register
        self.sensors = list(sensors or [])
        self._serial = None

    def open(self):
        if self._serial is not None:
            return
        if serial is None:
            raise RuntimeError("pyserial is required for ultrasonic real mode")
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

    def read_ranges(self) -> Dict[int, float]:
        self.open()
        values: Dict[int, float] = {}
        for sensor in self.sensors:
            if not sensor.enabled:
                continue
            try:
                request = build_read_holding_registers(sensor.address, self.register, 1)
                self._serial.reset_input_buffer()
                self._serial.write(request)
                response = self._serial.read(7)
                _, registers = parse_read_holding_registers_response(response, sensor.address)
                distance_mm = registers[0]
                values[sensor.index] = max(0.0, distance_mm / 1000.0)
                time.sleep(0.01)
            except Exception:
                continue
        return values


def make_sensor_list(addresses: Iterable[int], indices: Iterable[int], enabled_count: int) -> List[UltrasonicSensor]:
    sensors: List[UltrasonicSensor] = []
    for offset, (address, index) in enumerate(zip(addresses, indices)):
        sensors.append(
            UltrasonicSensor(
                index=int(index),
                address=int(address),
                frame_id=f"ultrasonic_{int(index)}_link",
                enabled=offset < enabled_count,
            )
        )
    return sensors


class UltrasonicAdapterNode(Node):
    def __init__(self):
        super().__init__("ultrasonic_adapter_node")
        self.declare_parameter("mode", "real")
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("serial_port", "/dev/smartwheel_ultrasonic")
        self.declare_parameter("baud_rate", 9600)
        self.declare_parameter("serial_timeout_sec", 0.08)
        self.declare_parameter("register", 0x0001)
        self.declare_parameter("sensor_addresses", [1])
        self.declare_parameter("sensor_indices", [0])
        self.declare_parameter("enabled_count", 1)
        self.declare_parameter("min_range", 0.03)
        self.declare_parameter("max_range", 3.0)
        self.declare_parameter("field_of_view", 0.45)

        self.mode = self.get_parameter("mode").value
        self.max_range = float(self.get_parameter("max_range").value)
        self.min_range = float(self.get_parameter("min_range").value)
        self.field_of_view = float(self.get_parameter("field_of_view").value)
        self.sensors = make_sensor_list(
            self.get_parameter("sensor_addresses").value,
            self.get_parameter("sensor_indices").value,
            int(self.get_parameter("enabled_count").value),
        )
        self.adapter = UltrasonicArrayAdapter(
            port=self.get_parameter("serial_port").value,
            baud_rate=int(self.get_parameter("baud_rate").value),
            timeout_sec=float(self.get_parameter("serial_timeout_sec").value),
            register=int(self.get_parameter("register").value),
            sensors=self.sensors,
        )
        self.enabled_sensors = [sensor for sensor in self.sensors if sensor.enabled]
        self.pubs = {
            sensor.index: self.create_publisher(Range, f"/ultrasonic/range_{sensor.index}", 10)
            for sensor in self.enabled_sensors
        }
        self.warned = False
        self.get_logger().info(
            "ultrasonic adapter using "
            + ", ".join(
                f"range_{sensor.index}@addr{sensor.address}" for sensor in self.enabled_sensors
            )
        )
        self.timer = self.create_timer(
            1.0 / float(self.get_parameter("publish_rate_hz").value), self.tick
        )

    def tick(self):
        if self.mode == "mock":
            values = {sensor.index: 2.5 for sensor in self.enabled_sensors}
        else:
            try:
                values = self.adapter.read_ranges()
            except Exception as exc:
                if not self.warned:
                    self.get_logger().warning(f"ultrasonic serial read failed: {exc}")
                    self.warned = True
                values = {}
        self._publish_ranges(values)

    def _publish_ranges(self, values: Dict[int, float]):
        now = self.get_clock().now().to_msg()
        for sensor in self.enabled_sensors:
            if sensor.index not in values:
                # Do NOT fabricate max_range for a failed/missing read: a dead
                # sensor must never look like "clear". Skipping lets the
                # safety_supervisor detect staleness instead of masking an obstacle.
                continue
            msg = Range()
            msg.header.stamp = now
            msg.header.frame_id = sensor.frame_id
            msg.radiation_type = Range.ULTRASOUND
            msg.field_of_view = self.field_of_view
            msg.min_range = self.min_range
            msg.max_range = self.max_range
            msg.range = float(values[sensor.index])
            self.pubs[sensor.index].publish(msg)

    def destroy_node(self):
        self.adapter.close()
        super().destroy_node()


def main(args=None):
    if rclpy is None:
        raise RuntimeError("ROS2 Python packages are required to run this node")
    rclpy.init(args=args)
    node = UltrasonicAdapterNode()
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
