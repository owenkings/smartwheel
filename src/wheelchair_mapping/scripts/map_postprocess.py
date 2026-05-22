#!/usr/bin/env python3
"""Map post-processing placeholder for static map maintenance.

Version 0.1 does not merge temporary obstacles into the static map. This script
only validates the saved map YAML and records where user-confirmed maintenance
updates should be implemented later.
"""

import argparse
from pathlib import Path

import yaml


def validate_map_yaml(path: Path) -> dict:
    data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    required = ["image", "resolution", "origin", "occupied_thresh", "free_thresh"]
    missing = [key for key in required if key not in data]
    if missing:
        raise ValueError(f"missing required map keys: {', '.join(missing)}")
    image_path = (path.parent / data["image"]).resolve()
    if not image_path.exists():
        raise FileNotFoundError(f"map image not found: {image_path}")
    return data


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("map_yaml", help="Path to map YAML produced by map_saver_cli")
    args = parser.parse_args()
    data = validate_map_yaml(Path(args.map_yaml))
    print("map yaml valid")
    print(f"image: {data['image']}")
    print(f"resolution: {data['resolution']}")
    print("TODO: apply only user-confirmed long-term obstacle updates here")


if __name__ == "__main__":
    main()
