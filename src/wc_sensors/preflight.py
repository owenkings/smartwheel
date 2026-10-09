"""Offline endpoint policy; no sockets or device commands are issued."""

from dataclasses import dataclass
from ipaddress import IPv4Address


@dataclass(frozen=True)
class SensorEndpoint:
    side: str
    sensor_id: str
    device_ip: str
    receive_ip: str
    receive_port: int
    bind_ip: str | None = None
    bind_source_verified: bool = False
    source_filter_verified: bool = False


def validate_endpoints(endpoints: list[SensorEndpoint], *, source_mode="dual",
                       allow_same_port_verified_bind=False) -> dict:
    """Fail closed for duplicate devices or unresolved same-port socket routing.

    Default dual policy requires distinct numeric receive ports per task 02.
    Optional shared-port policy requires explicit reviewed evidence of separate
    concrete local-IP binds plus per-source filtering. It establishes static
    eligibility only; it never claims concurrent hardware traffic was tested.
    """
    if source_mode not in ("dual", "single_left", "single_right"):
        raise ValueError("invalid source_mode")
    if not isinstance(allow_same_port_verified_bind, bool):
        raise TypeError("allow_same_port_verified_bind must be bool")
    required = {"left", "right"} if source_mode == "dual" else {source_mode[7:]}
    if len(endpoints) != len(required) or {item.side for item in endpoints} != required:
        raise ValueError("source_mode does not match supplied endpoint sides")
    serials = set()
    device_ips = set()
    for item in endpoints:
        if not isinstance(item.bind_source_verified, bool) or not isinstance(item.source_filter_verified, bool):
            raise TypeError("verification flags must be bool")
        if not isinstance(item.sensor_id, str) or not item.sensor_id:
            raise ValueError("sensor identity required")
        if item.sensor_id in serials:
            raise ValueError("duplicate sensor identity")
        serials.add(item.sensor_id)
        address = IPv4Address(item.device_ip)
        receive = IPv4Address(item.receive_ip)
        if address.is_unspecified or address.is_multicast or receive.is_unspecified or receive.is_multicast:
            raise ValueError("device and receive IP must be concrete unicast")
        if address in device_ips:
            raise ValueError("duplicate device IP")
        device_ips.add(address)
        if isinstance(item.receive_port, bool) or not isinstance(item.receive_port, int) or not 1 <= item.receive_port <= 65535:
            raise ValueError("receive_port must be in [1,65535]")
        if item.bind_ip is not None:
            IPv4Address(item.bind_ip)
    same_port_exception = False
    if len(endpoints) == 2 and endpoints[0].receive_port == endpoints[1].receive_port:
        if not allow_same_port_verified_bind:
            raise ValueError("same numeric UDP port requires separate approved source review")
        if any(not p.bind_source_verified or not p.source_filter_verified or p.bind_ip != p.receive_ip for p in endpoints):
            raise ValueError("same-port isolation is UNKNOWN; need verified concrete binds and source filters")
        if endpoints[0].bind_ip == endpoints[1].bind_ip:
            raise ValueError("duplicate UDP bind endpoint")
        same_port_exception = True
    return {"status": "STATIC_ELIGIBLE", "source_mode": source_mode,
            "same_port_exception": same_port_exception,
            "hardware_validation": "NOT_RUN", "identity_readback": "UNKNOWN"}
