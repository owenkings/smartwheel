import json
from typing import List

try:
    import rclpy
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
    from rclpy.node import Node
    from std_msgs.msg import String
except ImportError:
    rclpy = None
    Node = object
    DiagnosticArray = None
    DiagnosticStatus = None
    KeyValue = None
    String = None

from wheelchair_diagnostics.hardware_probe import (
    ProbeResult,
    available_serial_ports,
    probe_camera_device,
    probe_h30_port,
    probe_ultrasonic_port,
    probe_xtm60_sdk,
    probe_zlac_read_register,
)


def result_to_status(result: ProbeResult):
    status = DiagnosticStatus()
    status.name = f"hardware_self_check/{result.name}"
    status.hardware_id = result.name
    status.message = result.message
    if result.level == "OK":
        status.level = DiagnosticStatus.OK
    elif result.level == "WARN":
        status.level = DiagnosticStatus.WARN
    else:
        status.level = DiagnosticStatus.ERROR
    status.values = [KeyValue(key=str(k), value=json.dumps(v, ensure_ascii=False)) for k, v in result.details.items()]
    return status


class HardwareSelfCheckNode(Node):
    def __init__(self):
        super().__init__("hardware_self_check_node")
        self.declare_parameter("run_once", True)
        self.declare_parameter("period_sec", 10.0)
        self.declare_parameter("scan_serial_ports", False)
        self.declare_parameter("h30_port", "/dev/smartwheel_h30_imu")
        self.declare_parameter("h30_baud_rate", 460800)
        self.declare_parameter("ultrasonic_port", "/dev/smartwheel_ultrasonic")
        self.declare_parameter("ultrasonic_baud_rate", 9600)
        self.declare_parameter("ultrasonic_addresses", [1])
        self.declare_parameter("ultrasonic_register", 1)
        self.declare_parameter("camera_devices", ["0"])
        self.declare_parameter("xtm60_sdk_root", "")
        self.declare_parameter("xtm60_ip_address", "192.168.1.101")
        self.declare_parameter("xtm60_ip_addresses", [])
        self.declare_parameter("xtm60_tcp_port", 0)
        self.declare_parameter("zlac_port", "/dev/ttyUSB2")
        self.declare_parameter("zlac_baud_rate", 115200)
        self.declare_parameter("zlac_slave_id", 1)
        self.declare_parameter("zlac_probe_register", -1)

        self.status_pub = self.create_publisher(String, "/hardware/self_check", 10)
        self.diag_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self.has_run = False
        self.timer = self.create_timer(float(self.get_parameter("period_sec").value), self.run_check)
        self.run_check()

    def run_check(self):
        if self.has_run and bool(self.get_parameter("run_once").value):
            return
        self.has_run = True
        results = self._collect_results()
        payload = {
            "state": "OK" if all(r.ok or r.level == "WARN" for r in results) else "ERROR",
            "results": [
                {
                    "name": r.name,
                    "ok": r.ok,
                    "level": r.level,
                    "message": r.message,
                    "details": r.details,
                }
                for r in results
            ],
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.status_pub.publish(msg)

        diag = DiagnosticArray()
        diag.header.stamp = self.get_clock().now().to_msg()
        diag.status = [result_to_status(result) for result in results]
        self.diag_pub.publish(diag)

    def _collect_results(self) -> List[ProbeResult]:
        ports = available_serial_ports() if bool(self.get_parameter("scan_serial_ports").value) else []
        h30_port = self._pick_port(self.get_parameter("h30_port").value, ports)
        ultra_port = self._pick_port(self.get_parameter("ultrasonic_port").value, ports)
        zlac_port = self._pick_port(self.get_parameter("zlac_port").value, ports)
        xtm60_ips = [str(item) for item in self.get_parameter("xtm60_ip_addresses").value]
        if not xtm60_ips:
            xtm60_ips = [str(self.get_parameter("xtm60_ip_address").value)]

        results = [
            ProbeResult("serial_ports", bool(ports) or not bool(self.get_parameter("scan_serial_ports").value), "OK", "serial scan complete", {"ports": ports}),
            probe_h30_port(h30_port, int(self.get_parameter("h30_baud_rate").value)),
            probe_ultrasonic_port(
                ultra_port,
                int(self.get_parameter("ultrasonic_baud_rate").value),
                self.get_parameter("ultrasonic_addresses").value,
                int(self.get_parameter("ultrasonic_register").value),
            ),
            probe_zlac_read_register(
                zlac_port,
                int(self.get_parameter("zlac_baud_rate").value),
                int(self.get_parameter("zlac_slave_id").value),
                int(self.get_parameter("zlac_probe_register").value),
            ),
        ]
        for index, ip_address in enumerate(xtm60_ips):
            result = probe_xtm60_sdk(
                self.get_parameter("xtm60_sdk_root").value,
                ip_address,
                int(self.get_parameter("xtm60_tcp_port").value),
            )
            result.name = f"xtm60_{index}_{ip_address}"
            results.append(result)
        for device in self.get_parameter("camera_devices").value:
            result = probe_camera_device(str(device))
            result.name = f"camera_{device}"
            results.append(result)
        return results

    @staticmethod
    def _pick_port(configured: str, ports: List[str]) -> str:
        return configured if configured else (ports[0] if ports else "")


def main(args=None):
    if rclpy is None:
        raise RuntimeError("ROS2 Python packages are required to run this node")
    rclpy.init(args=args)
    node = HardwareSelfCheckNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
