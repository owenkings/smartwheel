# wheelchair_navigation

命名目标和导航状态包。

- `named_goal_store.py`：读写 `config/named_goals.yaml`
- `goal_manager_node`：把目标名称映射成 `/goal_pose`
- `navigation_status_node`：发布简化导航状态

目标不存在时返回错误状态，不自动生成坐标。
