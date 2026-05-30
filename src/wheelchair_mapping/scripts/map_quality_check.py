#!/usr/bin/env python3
"""Heuristic quality check for saved Nav2 occupancy-grid maps.

This is not a safety certification. It is an early gate that catches maps that
are obviously too small, mostly unknown, fragmented, or missing useful wall
structure before they are used for localization and navigation.
"""

import argparse
import json
from collections import deque
from pathlib import Path
from typing import Dict, List, Tuple

import yaml


def _next_token(data: bytes, index: int) -> Tuple[str, int]:
    while index < len(data):
        byte = data[index]
        if byte == ord("#"):
            while index < len(data) and data[index] not in b"\r\n":
                index += 1
        elif chr(byte).isspace():
            index += 1
        else:
            break
    start = index
    while index < len(data) and not chr(data[index]).isspace():
        index += 1
    return data[start:index].decode("ascii"), index


def read_pgm(path: Path) -> Tuple[int, int, List[int]]:
    data = path.read_bytes()
    magic, index = _next_token(data, 0)
    if magic not in ("P5", "P2"):
        raise ValueError(f"unsupported PGM format {magic}")
    width_text, index = _next_token(data, index)
    height_text, index = _next_token(data, index)
    maxval_text, index = _next_token(data, index)
    width = int(width_text)
    height = int(height_text)
    maxval = int(maxval_text)
    if maxval <= 0 or maxval > 255:
        raise ValueError("only 8-bit PGM maps are supported")
    if magic == "P2":
        values: List[int] = []
        while len(values) < width * height:
            token, index = _next_token(data, index)
            if not token:
                break
            values.append(int(token))
        if len(values) != width * height:
            raise ValueError("PGM payload size does not match width*height")
        return width, height, values
    while index < len(data) and chr(data[index]).isspace():
        index += 1
    pixels = list(data[index : index + width * height])
    if len(pixels) != width * height:
        raise ValueError("PGM payload size does not match width*height")
    return width, height, pixels


def load_map(map_yaml: Path) -> Tuple[Dict, int, int, List[int]]:
    meta = yaml.safe_load(map_yaml.read_text(encoding="utf-8")) or {}
    required = ["image", "resolution", "origin", "occupied_thresh", "free_thresh"]
    missing = [key for key in required if key not in meta]
    if missing:
        raise ValueError(f"missing required map keys: {', '.join(missing)}")
    image_path = Path(meta["image"])
    if not image_path.is_absolute():
        image_path = map_yaml.parent / image_path
    width, height, pixels = read_pgm(image_path)
    return meta, width, height, pixels


def classify_cells(meta: Dict, pixels: List[int]) -> List[int]:
    occupied_thresh = float(meta.get("occupied_thresh", 0.65))
    free_thresh = float(meta.get("free_thresh", 0.196))
    negate = int(meta.get("negate", 0))
    cells = []
    for pixel in pixels:
        occupancy = pixel / 255.0 if negate else 1.0 - pixel / 255.0
        if occupancy > occupied_thresh:
            cells.append(100)
        elif occupancy < free_thresh:
            cells.append(0)
        else:
            cells.append(-1)
    return cells


def free_components(cells: List[int], width: int, height: int) -> Tuple[int, int]:
    visited = bytearray(width * height)
    component_count = 0
    largest = 0
    for start, value in enumerate(cells):
        if value != 0 or visited[start]:
            continue
        component_count += 1
        size = 0
        queue = deque([start])
        visited[start] = 1
        while queue:
            index = queue.popleft()
            size += 1
            row, col = divmod(index, width)
            neighbors = []
            if col > 0:
                neighbors.append(index - 1)
            if col < width - 1:
                neighbors.append(index + 1)
            if row > 0:
                neighbors.append(index - width)
            if row < height - 1:
                neighbors.append(index + width)
            for neighbor in neighbors:
                if cells[neighbor] == 0 and not visited[neighbor]:
                    visited[neighbor] = 1
                    queue.append(neighbor)
        largest = max(largest, size)
    return component_count, largest


def check_quality(map_yaml: Path, args) -> Dict:
    meta, width, height, pixels = load_map(map_yaml)
    cells = classify_cells(meta, pixels)
    total = max(1, width * height)
    free = sum(1 for cell in cells if cell == 0)
    occupied = sum(1 for cell in cells if cell == 100)
    unknown = total - free - occupied
    known = free + occupied
    component_count, largest_free = free_components(cells, width, height)
    resolution = float(meta["resolution"])
    area_m2 = total * resolution * resolution
    free_component_ratio = largest_free / max(1, free)
    metrics = {
        "width": width,
        "height": height,
        "resolution_m": resolution,
        "area_m2": round(area_m2, 3),
        "known_ratio": round(known / total, 4),
        "free_ratio": round(free / total, 4),
        "occupied_ratio": round(occupied / total, 4),
        "unknown_ratio": round(unknown / total, 4),
        "free_component_count": component_count,
        "largest_free_component_ratio": round(free_component_ratio, 4),
    }

    reasons = []
    score = 100

    def penalize(points: int, severity: str, message: str):
        nonlocal score
        score -= points
        reasons.append({"severity": severity, "message": message})

    if total < args.min_cells:
        penalize(45, "bad", f"地图像素过少：{total} < {args.min_cells}")
    if area_m2 < args.min_area_m2:
        penalize(35, "bad", f"地图面积过小：{area_m2:.2f} m^2")
    if metrics["known_ratio"] < args.min_known_ratio:
        penalize(40, "bad", f"已知区域比例过低：{metrics['known_ratio']:.1%}")
    elif metrics["known_ratio"] < args.warn_known_ratio:
        penalize(18, "warning", f"已知区域偏少：{metrics['known_ratio']:.1%}")
    if metrics["free_ratio"] < args.min_free_ratio:
        penalize(35, "bad", f"可通行区域过少：{metrics['free_ratio']:.1%}")
    if metrics["occupied_ratio"] < args.min_occupied_ratio:
        penalize(18, "warning", f"障碍/墙体比例偏低：{metrics['occupied_ratio']:.1%}")
    if metrics["occupied_ratio"] > args.max_occupied_ratio:
        penalize(25, "warning", f"障碍/墙体比例偏高：{metrics['occupied_ratio']:.1%}")
    if free > 0 and free_component_ratio < args.min_largest_free_component_ratio:
        penalize(22, "warning", f"可通行区域碎片化：最大连通区 {free_component_ratio:.1%}")
    if component_count > args.max_free_components:
        penalize(12, "warning", f"可通行连通区数量偏多：{component_count}")

    score = max(0, min(100, score))
    if any(reason["severity"] == "bad" for reason in reasons) or score < 40:
        verdict = "BAD"
    elif reasons or score < 70:
        verdict = "WARNING"
    else:
        verdict = "GOOD"
        reasons.append({"severity": "ok", "message": "地图基础质量检查通过"})

    return {
        "map_yaml": str(map_yaml),
        "verdict": verdict,
        "score": score,
        "metrics": metrics,
        "reasons": reasons,
    }


def print_text(report: Dict):
    print(f"map: {report['map_yaml']}")
    print(f"verdict: {report['verdict']}  score: {report['score']}")
    for key, value in report["metrics"].items():
        print(f"{key}: {value}")
    print("reasons:")
    for reason in report["reasons"]:
        print(f"- {reason['severity']}: {reason['message']}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("map_yaml", help="Path to map YAML produced by map_saver_cli")
    parser.add_argument("--json", action="store_true", help="Print machine-readable JSON")
    parser.add_argument("--report", help="Optional JSON report path to write")
    parser.add_argument("--fail-on-bad", action="store_true", help="Exit non-zero for BAD maps")
    parser.add_argument("--min-cells", type=int, default=1000)
    parser.add_argument("--min-area-m2", type=float, default=4.0)
    parser.add_argument("--min-known-ratio", type=float, default=0.08)
    parser.add_argument("--warn-known-ratio", type=float, default=0.18)
    parser.add_argument("--min-free-ratio", type=float, default=0.03)
    parser.add_argument("--min-occupied-ratio", type=float, default=0.002)
    parser.add_argument("--max-occupied-ratio", type=float, default=0.45)
    parser.add_argument("--min-largest-free-component-ratio", type=float, default=0.50)
    parser.add_argument("--max-free-components", type=int, default=30)
    args = parser.parse_args()

    report = check_quality(Path(args.map_yaml), args)
    if args.report:
        Path(args.report).write_text(
            json.dumps(report, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
    if args.json:
        print(json.dumps(report, ensure_ascii=False))
    else:
        print_text(report)
    if args.fail_on_bad and report["verdict"] == "BAD":
        raise SystemExit(2)


if __name__ == "__main__":
    main()
