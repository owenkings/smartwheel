import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from wheelchair_navigation.semantic_map_store import SemanticMapStore  # noqa: E402


def test_semantic_map_room_and_no_go_zone_roundtrip(tmp_path):
    store = SemanticMapStore(str(tmp_path / "semantic_map.yaml"))

    store.upsert_room("餐厅", [[0, 0], [1, 0], [1, 1]], "#123456")
    store.upsert_no_go_zone("楼梯口", [[2, 2], [3, 2], [3, 3]])
    data = store.load()

    assert data["rooms"][0]["name"] == "餐厅"
    assert data["no_go_zones"][0]["name"] == "楼梯口"
    assert store.delete_room("餐厅") is True
    assert store.delete_no_go_zone("楼梯口") is True


def test_semantic_map_save_leaves_no_temporary_file(tmp_path):
    path = tmp_path / "semantic_map.yaml"
    store = SemanticMapStore(str(path))

    store.save({"rooms": [{"name": "客厅", "polygon": []}]})

    assert path.exists()
    assert list(tmp_path.glob(".semantic_map.yaml.*.tmp")) == []
