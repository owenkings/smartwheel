import socket
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Optional

try:
    import cv2
except ImportError:
    cv2 = None

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    serial = None
    list_ports = None

from wheelchair_sensors.imu_adapter_node import YesenseParser
from wheelchair_sensors.ultrasonic_adapter_node import (
    build_read_holding_registers,
    parse_read_holding_registers_response,
)


@dataclass
class ProbeResult:
    name: str
    ok: bool
    level: str
    message: str
    details: Dict


def available_serial_ports() -> List[str]:
    if list_ports is None:
        return []
    return [port.device for port in list_ports.comports()]


def probe_h30_port(port: str, baud_rate: int = 460800, duration_sec: float = 0.5) -> ProbeResult:
    if serial is None:
        return ProbeResult("h30_imu", False, "ERROR", "pyserial not installed", {"port": port})
    parser = YesenseParser()
    deadline = time.monotonic() + duration_sec
    try:
        with serial.Serial(port, baud_rate, timeout=0.02) as handle:
            while time.monotonic() < deadline:
                data = handle.read(getattr(handle, "in_waiting", 0) or 1)
                if parser.feed(data):
                    return ProbeResult("h30_imu", True, "OK", "Yesense frame decoded", {"port": port})
    except Exception as exc:
        return ProbeResult("h30_imu", False, "ERROR", str(exc), {"port": port})
    return ProbeResult("h30_imu", False, "WARN", "no Yesense frame decoded", {"port": port})


def probe_ultrasonic_port(
    port: str,
    baud_rate: int = 9600,
    addresses: Iterable[int] = (1, 2),
    register: int = 0x0001,
    timeout_sec: float = 0.2,
) -> ProbeResult:
    if serial is None:
        return ProbeResult("ultrasonic", False, "ERROR", "pyserial not installed", {"port": port})
    found = []
    try:
        with serial.Serial(port, baud_rate, timeout=timeout_sec) as handle:
            for address in addresses:
                handle.reset_input_buffer()
                handle.write(build_read_holding_registers(int(address), register, 1))
                response = handle.read(7)
                try:
                    _, values = parse_read_holding_registers_response(response, int(address))
                    found.append({"address": int(address), "value": values[0]})
                except Exception:
                    continue
    except Exception as exc:
        return ProbeResult("ultrasonic", False, "ERROR", str(exc), {"port": port})
    return ProbeResult(
        "ultrasonic",
        bool(found),
        "OK" if found else "WARN",
        f"found {len(found)} Modbus sensor(s)" if found else "no Modbus ultrasonic response",
        {"port": port, "sensors": found},
    )


def probe_camera_device(device: str) -> ProbeResult:
    if cv2 is None:
        return ProbeResult("camera", False, "ERROR", "opencv-python not installed", {"device": device})
    capture = None
    try:
        source = int(device) if str(device).isdigit() else device
        capture = cv2.VideoCapture(source)
        if not capture.isOpened():
            return ProbeResult("camera", False, "WARN", "camera open failed", {"device": device})
        ok, frame = capture.read()
        if ok and frame is not None:
            return ProbeResult(
                "camera",
                True,
                "OK",
                "camera frame read",
                {"device": device, "width": int(frame.shape[1]), "height": int(frame.shape[0])},
            )
        return ProbeResult("camera", False, "WARN", "camera opened but frame read failed", {"device": device})
    except Exception as exc:
        return ProbeResult("camera", False, "ERROR", str(exc), {"device": device})
    finally:
        if capture is not None:
            capture.release()


def probe_xtm60_sdk(sdk_root: str, ip_address: str = "", tcp_port: int = 0) -> ProbeResult:
    root = Path(sdk_root).expanduser() if sdk_root else None
    details = {"sdk_root": str(root) if root else "", "ip_address": ip_address, "tcp_port": tcp_port}
    if not root or not root.exists():
        return ProbeResult("xtm60", False, "WARN", "XTSDK root not found", details)
    if tcp_port > 0 and ip_address:
        try:
            with socket.create_connection((ip_address, tcp_port), timeout=0.5):
                return ProbeResult("xtm60", True, "OK", "SDK root exists and TCP port reachable", details)
        except Exception as exc:
            return ProbeResult("xtm60", False, "WARN", f"SDK root exists but TCP probe failed: {exc}", details)
    return ProbeResult("xtm60", True, "OK", "SDK root exists; runtime topic watchdog verifies live points", details)


def probe_zlac_read_register(
    port: str,
    baud_rate: int,
    slave_id: int,
    register: int,
    timeout_sec: float = 0.05,
) -> ProbeResult:
    if register < 0:
        return ProbeResult(
            "zlac8030",
            False,
            "WARN",
            "ZLAC8030 probe register disabled; fill zlac8030_base.yaml after confirming manual",
            {"port": port, "slave_id": slave_id, "register": register},
        )
    from wheelchair_base.modbus_rtu import ModbusRtuClient, ModbusSerialConfig

    client = ModbusRtuClient(ModbusSerialConfig(port=port, baud_rate=baud_rate, timeout_sec=timeout_sec))
    try:
        values = client.read_holding_registers(slave_id, register, 1)
        return ProbeResult(
            "zlac8030",
            True,
            "OK",
            "read holding register succeeded",
            {"port": port, "slave_id": slave_id, "register": register, "value": values[0]},
        )
    except Exception as exc:
        return ProbeResult(
            "zlac8030",
            False,
            "ERROR",
            str(exc),
            {"port": port, "slave_id": slave_id, "register": register},
        )
    finally:
        client.close()
