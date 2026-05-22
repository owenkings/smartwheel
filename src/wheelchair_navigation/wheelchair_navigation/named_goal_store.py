import math
from pathlib import Path
from typing import Dict, Optional

import yaml


def normalize_goal_key(name: str) -> str:
    return name.strip().lower().replace(" ", "_")


def yaw_to_quaternion(yaw: float) -> Dict[str, float]:
    half = yaw * 0.5
    return {"x": 0.0, "y": 0.0, "z": math.sin(half), "w": math.cos(half)}


def quaternion_to_yaw(z: float, w: float) -> float:
    return math.atan2(2.0 * w * z, 1.0 - 2.0 * z * z)


class NamedGoalStore:
    def __init__(self, path: str):
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        if not self.path.exists():
            self.save({"goals": {}})

    def load(self) -> Dict:
        with self.path.open("r", encoding="utf-8") as stream:
            data = yaml.safe_load(stream) or {}
        data.setdefault("goals", {})
        return data

    def save(self, data: Dict) -> None:
        with self.path.open("w", encoding="utf-8") as stream:
            yaml.safe_dump(data, stream, allow_unicode=True, sort_keys=True)

    def list_goals(self) -> Dict:
        return self.load()["goals"]

    def get_goal(self, name: str) -> Optional[Dict]:
        goals = self.list_goals()
        key = normalize_goal_key(name)
        if key in goals:
            return goals[key]
        for value in goals.values():
            if value.get("label") == name:
                return value
        return None

    def upsert_goal(
        self,
        name: str,
        x: float,
        y: float,
        yaw: float = 0.0,
        frame_id: str = "map",
        label: Optional[str] = None,
    ) -> str:
        data = self.load()
        key = normalize_goal_key(name)
        data["goals"][key] = {
            "label": label or name,
            "frame_id": frame_id,
            "position": [float(x), float(y), 0.0],
            "yaw": float(yaw),
        }
        self.save(data)
        return key

    def delete_goal(self, name: str) -> bool:
        data = self.load()
        key = normalize_goal_key(name)
        if key in data["goals"]:
            del data["goals"][key]
            self.save(data)
            return True
        for candidate_key, value in list(data["goals"].items()):
            if value.get("label") == name:
                del data["goals"][candidate_key]
                self.save(data)
                return True
        return False
