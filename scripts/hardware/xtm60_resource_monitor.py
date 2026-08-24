#!/usr/bin/env python3
"""Sample XT-M60 adapter CPU/RSS and host load during a bounded soak."""

import argparse
import json
import math
import os
import statistics
import time
from pathlib import Path


def _summary(values):
    if not values:
        return {"count": 0}
    ordered = sorted(float(value) for value in values)
    p95_index = max(0, min(len(ordered) - 1, math.ceil(0.95 * len(ordered)) - 1))
    return {
        "count": len(ordered),
        "min": ordered[0],
        "mean": statistics.fmean(ordered),
        "median": statistics.median(ordered),
        "p95": ordered[p95_index],
        "max": ordered[-1],
    }


def _matching_processes():
    result = {}
    for name in os.listdir("/proc"):
        if not name.isdigit():
            continue
        path = Path("/proc") / name
        try:
            command = (path / "cmdline").read_bytes().replace(b"\0", b" ").decode(
                "utf-8", errors="replace"
            )
            if "xtm60_adapter_node" not in command:
                continue
            fields = (path / "stat").read_text(encoding="utf-8").split()
            statm = (path / "statm").read_text(encoding="utf-8").split()
            result[int(name)] = {
                "command": command.strip(),
                "ticks": int(fields[13]) + int(fields[14]),
                "rss_bytes": int(statm[1]) * os.sysconf("SC_PAGE_SIZE"),
            }
        except (FileNotFoundError, PermissionError, ProcessLookupError, ValueError):
            continue
    return result


def _host_memory():
    values = {}
    for line in Path("/proc/meminfo").read_text(encoding="utf-8").splitlines():
        key, value = line.split(":", 1)
        values[key] = int(value.strip().split()[0]) * 1024
    total = values.get("MemTotal", 0)
    available = values.get("MemAvailable", 0)
    return {
        "total_bytes": total,
        "available_bytes": available,
        "used_fraction": ((total - available) / total) if total else None,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration-sec", type=float, required=True)
    parser.add_argument("--interval-sec", type=float, default=5.0)
    parser.add_argument("--min-process-count", type=int, default=2)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    clock_ticks = os.sysconf("SC_CLK_TCK")
    started = time.monotonic()
    previous_time = started
    previous_ticks = {}
    samples = []

    while time.monotonic() - started < args.duration_sec:
        now = time.monotonic()
        processes = _matching_processes()
        elapsed = now - previous_time
        cpu_percent = 0.0
        if elapsed > 0.0:
            for pid, entry in processes.items():
                prior = previous_ticks.get(pid)
                if prior is not None:
                    cpu_percent += (
                        (entry["ticks"] - prior) / clock_ticks / elapsed * 100.0
                    )
        load1, load5, load15 = os.getloadavg()
        memory = _host_memory()
        samples.append(
            {
                "elapsed_sec": now - started,
                "process_count": len(processes),
                "pids": sorted(processes),
                "adapter_cpu_percent": cpu_percent,
                "adapter_rss_bytes": sum(
                    entry["rss_bytes"] for entry in processes.values()
                ),
                "load1": load1,
                "load5": load5,
                "load15": load15,
                "host_memory_used_fraction": memory["used_fraction"],
            }
        )
        previous_ticks = {
            pid: entry["ticks"] for pid, entry in processes.items()
        }
        previous_time = now
        remaining = args.duration_sec - (time.monotonic() - started)
        if remaining <= 0.0:
            break
        time.sleep(min(max(0.1, args.interval_sec), remaining))

    result = {
        "schema": "smartwheel.xtm60_resource_monitor.v1",
        "requested_duration_sec": args.duration_sec,
        "elapsed_sec": time.monotonic() - started,
        "logical_cpu_count": os.cpu_count(),
        "samples": samples,
        "summary": {
            "process_count": _summary(
                sample["process_count"] for sample in samples
            ),
            "adapter_cpu_percent": _summary(
                sample["adapter_cpu_percent"] for sample in samples[1:]
            ),
            "adapter_rss_bytes": _summary(
                sample["adapter_rss_bytes"] for sample in samples
            ),
            "load1": _summary(sample["load1"] for sample in samples),
            "host_memory_used_fraction": _summary(
                sample["host_memory_used_fraction"]
                for sample in samples
                if sample["host_memory_used_fraction"] is not None
            ),
        },
    }
    rendered = json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True)
    print(rendered)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(rendered + "\n", encoding="utf-8")
    return (
        0
        if any(
            sample["process_count"] >= args.min_process_count
            for sample in samples
        )
        else 2
    )


if __name__ == "__main__":
    raise SystemExit(main())
