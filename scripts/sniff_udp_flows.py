#!/usr/bin/env python3
"""Count IPv4 UDP flows on an interface without storing packet payloads."""

import collections
import socket
import struct
import sys
import time


interface = sys.argv[1]
seconds = float(sys.argv[2]) if len(sys.argv) > 2 else 5.0
port_min = int(sys.argv[3]) if len(sys.argv) > 3 else 7686
port_max = int(sys.argv[4]) if len(sys.argv) > 4 else 7688

sock = socket.socket(socket.AF_PACKET, socket.SOCK_RAW, socket.ntohs(3))
sock.bind((interface, 0))
sock.settimeout(0.25)

flows = collections.Counter()
byte_counts = collections.Counter()
deadline = time.monotonic() + seconds
while time.monotonic() < deadline:
    try:
        packet = sock.recv(65535)
    except socket.timeout:
        continue
    if len(packet) < 42:
        continue
    ethertype = struct.unpack_from("!H", packet, 12)[0]
    ip_offset = 14
    if ethertype == 0x8100 and len(packet) >= 46:
        ethertype = struct.unpack_from("!H", packet, 16)[0]
        ip_offset = 18
    if ethertype != 0x0800 or len(packet) < ip_offset + 20:
        continue
    ihl = (packet[ip_offset] & 0x0F) * 4
    if ihl < 20 or packet[ip_offset + 9] != socket.IPPROTO_UDP:
        continue
    udp_offset = ip_offset + ihl
    if len(packet) < udp_offset + 8:
        continue
    src_port, dst_port = struct.unpack_from("!HH", packet, udp_offset)
    if not (port_min <= src_port <= port_max or port_min <= dst_port <= port_max):
        continue
    src_ip = socket.inet_ntoa(packet[ip_offset + 12 : ip_offset + 16])
    dst_ip = socket.inet_ntoa(packet[ip_offset + 16 : ip_offset + 20])
    flow = (src_ip, src_port, dst_ip, dst_port)
    flows[flow] += 1
    byte_counts[flow] += len(packet)

for flow, packets in flows.most_common():
    src_ip, src_port, dst_ip, dst_port = flow
    print(
        f"{src_ip}:{src_port} -> {dst_ip}:{dst_port} "
        f"packets={packets} bytes={byte_counts[flow]}"
    )
