import argparse
import json
from pathlib import Path
from typing import Any, Dict, Optional

import yaml


def validate_nav_map(path: Path) -> bool:
    try:
        metadata = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
        image_value = metadata["image"]
        resolution = float(metadata["resolution"])
        origin = metadata["origin"]
    except (KeyError, OSError, TypeError, ValueError, yaml.YAMLError):
        return False
    if not isinstance(image_value, str) or not image_value.strip():
        return False
    if resolution <= 0.0 or not isinstance(origin, list) or len(origin) < 3:
        return False
    image_path = Path(image_value).expanduser()
    if not image_path.is_absolute():
        image_path = path.parent / image_path
    return image_path.is_file()


def _entry_map_path(entry: Dict[str, Any]) -> Optional[Path]:
    value = entry.get("current_yaml") or entry.get("version_yaml")
    if not value:
        return None
    return Path(str(value)).expanduser()


def resolve_active_nav_map(workspace_root: Path) -> Optional[Path]:
    manifest_path = workspace_root / "maps" / "map_versions.json"
    try:
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    except (OSError, TypeError, ValueError):
        return None
    if not isinstance(manifest, dict):
        return None
    versions = manifest.get("versions") or []
    if not isinstance(versions, list):
        return None
    entries = {
        entry.get("version_id"): entry
        for entry in versions
        if isinstance(entry, dict) and entry.get("version_id")
    }

    active_id = manifest.get("active_nav")
    candidate_ids = [active_id, manifest.get("latest")]
    candidate_ids.extend(
        entry.get("version_id")
        for entry in reversed(versions)
        if isinstance(entry, dict)
    )
    checked = set()
    for version_id in candidate_ids:
        if not version_id or version_id in checked:
            continue
        checked.add(version_id)
        entry = entries.get(version_id)
        if not entry:
            continue
        verdict = str((entry.get("quality") or {}).get("verdict", "")).upper()
        if version_id != active_id and verdict == "BAD":
            continue
        map_path = _entry_map_path(entry)
        if map_path is not None and validate_nav_map(map_path):
            return map_path.resolve()
    return None


def main(args=None) -> int:
    parser = argparse.ArgumentParser(description="Resolve the active Nav2 map")
    parser.add_argument("--workspace", type=Path, required=True)
    parsed = parser.parse_args(args)
    selected = resolve_active_nav_map(parsed.workspace.expanduser().resolve())
    if selected is None:
        return 1
    print(selected)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
