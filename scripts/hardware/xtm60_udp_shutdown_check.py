#!/usr/bin/env python3
"""Verify that both XT-M60 host UDP destinations are silent after shutdown."""

import argparse
import json
import selectors
import socket
import time
from pathlib import Path


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration-sec", type=float, default=3.0)
    parser.add_argument("--port", type=int, default=7687)
    parser.add_argument(
        "--addresses",
        nargs="+",
        default=["192.168.0.100", "192.168.1.100"],
    )
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()

    selector = selectors.DefaultSelector()
    sockets = {}
    bind_errors = {}
    for address in args.addresses:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setblocking(False)
        try:
            sock.bind((address, args.port))
        except OSError as exc:
            bind_errors[address] = str(exc)
            sock.close()
            continue
        sockets[address] = {"socket": sock, "packets": 0, "bytes": 0}
        selector.register(sock, selectors.EVENT_READ, data=address)

    started = time.monotonic()
    try:
        while time.monotonic() - started < max(0.0, args.duration_sec):
            remaining = args.duration_sec - (time.monotonic() - started)
            for key, _mask in selector.select(timeout=max(0.0, min(0.2, remaining))):
                address = key.data
                while True:
                    try:
                        payload, _peer = key.fileobj.recvfrom(65535)
                    except BlockingIOError:
                        break
                    sockets[address]["packets"] += 1
                    sockets[address]["bytes"] += len(payload)
    finally:
        for entry in sockets.values():
            selector.unregister(entry["socket"])
            entry["socket"].close()
        selector.close()

    result = {
        "schema": "smartwheel.xtm60_udp_shutdown_check.v1",
        "duration_sec": time.monotonic() - started,
        "port": args.port,
        "addresses": {
            address: {
                "packets": entry["packets"],
                "bytes": entry["bytes"],
            }
            for address, entry in sockets.items()
        },
        "bind_errors": bind_errors,
    }
    result["pass"] = (
        not bind_errors
        and set(result["addresses"]) == set(args.addresses)
        and all(entry["packets"] == 0 for entry in result["addresses"].values())
    )
    rendered = json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True)
    print(rendered)
    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered + "\n", encoding="utf-8")
    return 0 if result["pass"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
