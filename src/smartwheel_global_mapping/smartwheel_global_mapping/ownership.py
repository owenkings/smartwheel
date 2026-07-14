from enum import Enum


class MapBackend(str, Enum):
    RTABMAP = "rtabmap"
    SLAM_TOOLBOX = "slam_toolbox"


def map_tf_owner(backend: str) -> str:
    try:
        selected = MapBackend(backend)
    except ValueError as exc:
        raise ValueError(f"unsupported mapping backend: {backend}") from exc
    return "rtabmap" if selected is MapBackend.RTABMAP else "slam_toolbox"


def validate_tf_owners(owners: list[str]) -> None:
    normalized = [owner for owner in owners if owner]
    if len(normalized) != 1:
        raise ValueError(f"map->odom requires exactly one publisher, got {normalized}")

