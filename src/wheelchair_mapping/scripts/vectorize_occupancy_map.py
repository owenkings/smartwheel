#!/usr/bin/env python3
"""Convert a saved Nav2 map PGM into a simple vector wall layer.

This is an engineering map overlay for the Web UI. It does not replace the
static occupancy grid used by AMCL/Nav2, and it must not be used to bypass
local costmaps or safety supervision.
"""

import argparse
from pathlib import Path
from typing import Dict, Iterable, List, Tuple

import yaml


Point = Tuple[float, float]
Segment = Tuple[Point, Point]


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
        return width, height, values
    while index < len(data) and chr(data[index]).isspace():
        index += 1
    pixels = list(data[index : index + width * height])
    if len(pixels) != width * height:
        raise ValueError("PGM payload size does not match width*height")
    return width, height, pixels


def occupied_mask(pixels: List[int], width: int, height: int, occupied_thresh: float, negate: int) -> List[List[bool]]:
    grid: List[List[bool]] = []
    for row in range(height):
        values = []
        for col in range(width):
            pixel = pixels[row * width + col]
            occ = (pixel / 255.0) if negate else (1.0 - pixel / 255.0)
            values.append(occ >= occupied_thresh)
        grid.append(values)
    return grid


def cell_edges(mask: List[List[bool]], resolution: float, origin: List[float]) -> List[Segment]:
    height = len(mask)
    width = len(mask[0]) if height else 0
    ox, oy = float(origin[0]), float(origin[1])
    segments: List[Segment] = []

    def world(col: int, row_from_top: int) -> Point:
        return ox + col * resolution, oy + (height - row_from_top) * resolution

    for row in range(height):
        for col in range(width):
            if not mask[row][col]:
                continue
            neighbors = {
                "top": row == 0 or not mask[row - 1][col],
                "bottom": row == height - 1 or not mask[row + 1][col],
                "left": col == 0 or not mask[row][col - 1],
                "right": col == width - 1 or not mask[row][col + 1],
            }
            if neighbors["top"]:
                segments.append((world(col, row), world(col + 1, row)))
            if neighbors["bottom"]:
                segments.append((world(col, row + 1), world(col + 1, row + 1)))
            if neighbors["left"]:
                segments.append((world(col, row), world(col, row + 1)))
            if neighbors["right"]:
                segments.append((world(col + 1, row), world(col + 1, row + 1)))
    return segments


def merge_segments(segments: Iterable[Segment], precision: int = 4) -> List[Segment]:
    horizontal: Dict[float, List[Tuple[float, float]]] = {}
    vertical: Dict[float, List[Tuple[float, float]]] = {}
    for (x1, y1), (x2, y2) in segments:
        x1, y1, x2, y2 = round(x1, precision), round(y1, precision), round(x2, precision), round(y2, precision)
        if y1 == y2:
            lo, hi = sorted((x1, x2))
            horizontal.setdefault(y1, []).append((lo, hi))
        elif x1 == x2:
            lo, hi = sorted((y1, y2))
            vertical.setdefault(x1, []).append((lo, hi))

    merged: List[Segment] = []
    for y, spans in horizontal.items():
        for lo, hi in _merge_spans(spans):
            merged.append(((lo, y), (hi, y)))
    for x, spans in vertical.items():
        for lo, hi in _merge_spans(spans):
            merged.append(((x, lo), (x, hi)))
    return merged


def _merge_spans(spans: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
    spans = sorted(spans)
    if not spans:
        return []
    merged = [spans[0]]
    for lo, hi in spans[1:]:
        prev_lo, prev_hi = merged[-1]
        if lo <= prev_hi + 1e-4:
            merged[-1] = (prev_lo, max(prev_hi, hi))
        else:
            merged.append((lo, hi))
    return merged


def vectorize(map_yaml: Path, output: Path, max_segments: int):
    map_data = yaml.safe_load(map_yaml.read_text(encoding="utf-8")) or {}
    image_path = (map_yaml.parent / map_data["image"]).resolve()
    width, height, pixels = read_pgm(image_path)
    mask = occupied_mask(
        pixels,
        width,
        height,
        float(map_data.get("occupied_thresh", 0.65)),
        int(map_data.get("negate", 0)),
    )
    segments = merge_segments(cell_edges(mask, float(map_data["resolution"]), map_data["origin"]))
    if len(segments) > max_segments:
        step = max(1, len(segments) // max_segments)
        segments = segments[::step][:max_segments]
    semantic = {}
    if output.exists():
        semantic = yaml.safe_load(output.read_text(encoding="utf-8")) or {}
    semantic.setdefault("version", 1)
    semantic.setdefault("frame_id", "map")
    semantic.setdefault("rooms", [])
    semantic.setdefault("points_of_interest", [])
    semantic.setdefault("no_go_zones", [])
    semantic.setdefault("preferred_routes", [])
    semantic["walls"] = [
        {"type": "occupied_boundary", "polyline": [[x1, y1], [x2, y2]]}
        for (x1, y1), (x2, y2) in segments
    ]
    output.write_text(yaml.safe_dump(semantic, allow_unicode=True, sort_keys=False), encoding="utf-8")
    return len(segments)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("map_yaml", help="Path to map YAML produced by map_saver_cli")
    parser.add_argument("--output", required=True, help="semantic_map.yaml to update")
    parser.add_argument("--max-segments", type=int, default=2000)
    args = parser.parse_args()
    count = vectorize(Path(args.map_yaml), Path(args.output), args.max_segments)
    print(f"wrote {count} vector wall segments to {args.output}")


if __name__ == "__main__":
    main()
