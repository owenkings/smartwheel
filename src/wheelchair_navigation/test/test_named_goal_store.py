import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from wheelchair_navigation.named_goal_store import NamedGoalStore  # noqa: E402


def test_named_goals_yaml_read_write(tmp_path):
    path = tmp_path / "named_goals.yaml"
    store = NamedGoalStore(str(path))

    key = store.upsert_goal("卫生间", 1.2, -0.3, 1.57)
    assert key
    goal = store.get_goal("卫生间")
    assert goal["position"][:2] == [1.2, -0.3]
    assert goal["yaw"] == 1.57

    goals = store.list_goals()
    assert key in goals
    assert store.delete_goal("卫生间") is True
    assert store.get_goal("卫生间") is None
