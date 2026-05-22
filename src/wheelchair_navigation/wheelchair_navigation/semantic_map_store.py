from pathlib import Path
from typing import Dict, List, Optional

import yaml

try:
    from ament_index_python.packages import get_package_share_directory
except ImportError:
    get_package_share_directory = None


DEFAULT_SEMANTIC_MAP = {
    "version": 1,
    "frame_id": "map",
    "rooms": [],
    "points_of_interest": [],
    "no_go_zones": [],
    "preferred_routes": [],
    "walls": [],
}


class SemanticMapStore:
    """YAML-backed semantic/vector layer over the occupancy grid.

    This is the first "Gaode-like" layer: named rooms, POIs, no-go polygons and
    route annotations. The occupancy grid remains the planner map; this layer is
    for UI display, goal selection and later map maintenance workflows.
    """

    def __init__(self, path: str):
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        if not self.path.exists():
            self.save(DEFAULT_SEMANTIC_MAP.copy())

    def load(self) -> Dict:
        if not self.path.exists():
            return DEFAULT_SEMANTIC_MAP.copy()
        with self.path.open("r", encoding="utf-8") as handle:
            data = yaml.safe_load(handle) or {}
        merged = DEFAULT_SEMANTIC_MAP.copy()
        merged.update(data)
        return merged

    def save(self, data: Dict) -> Dict:
        normalized = DEFAULT_SEMANTIC_MAP.copy()
        normalized.update(data or {})
        with self.path.open("w", encoding="utf-8") as handle:
            yaml.safe_dump(normalized, handle, allow_unicode=True, sort_keys=False)
        return normalized

    def list_rooms(self) -> List[Dict]:
        return list(self.load().get("rooms", []))

    def upsert_room(self, name: str, polygon: List[List[float]], color: str = "#6aa6ff") -> Dict:
        data = self.load()
        rooms = [room for room in data.get("rooms", []) if room.get("name") != name]
        rooms.append({"name": name, "polygon": polygon, "color": color})
        data["rooms"] = rooms
        return self.save(data)

    def delete_room(self, name: str) -> bool:
        data = self.load()
        rooms = data.get("rooms", [])
        filtered = [room for room in rooms if room.get("name") != name]
        data["rooms"] = filtered
        self.save(data)
        return len(filtered) != len(rooms)

    def upsert_no_go_zone(self, name: str, polygon: List[List[float]]) -> Dict:
        data = self.load()
        zones = [zone for zone in data.get("no_go_zones", []) if zone.get("name") != name]
        zones.append({"name": name, "polygon": polygon})
        data["no_go_zones"] = zones
        return self.save(data)

    def delete_no_go_zone(self, name: str) -> bool:
        data = self.load()
        zones = data.get("no_go_zones", [])
        filtered = [zone for zone in zones if zone.get("name") != name]
        data["no_go_zones"] = filtered
        self.save(data)
        return len(filtered) != len(zones)


def default_semantic_map_path() -> str:
    source_candidate = Path(__file__).resolve().parents[1] / "config" / "semantic_map.yaml"
    if source_candidate.exists():
        return str(source_candidate)
    if get_package_share_directory is not None:
        return str(Path(get_package_share_directory("wheelchair_navigation")) / "config" / "semantic_map.yaml")
    return "semantic_map.yaml"
