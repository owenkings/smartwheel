# 阶段 3 — 定位导航 + 上层应用

> 状态：spec（占位，阶段2 稳定后细化）。前置：阶段 2 通过。

## 1. 目标
有了可靠地图后，实现"用户指定目标点 → 轮椅自主导航过去"，并逐步加回上层应用（GUI/Web、语音、用户地图）。这是最后阶段，不提前做。

## 2. 范围（分子阶段，逐个验证）
- 3a. **定位**：已有地图 + 开机/手动重定位（先手动设初始位姿，再考虑 AprilTag/UWB/全局匹配）。
- 3b. **导航**：Nav2 路径规划 + 安全层 + 底盘闭环，RViz 设目标点（`/goal_pose`）。
- 3c. **POI / 用户地图**：命名目标点、语义图层、禁行区（先显示存储，再接 Nav2 keepout）。
- 3d. **GUI/Web**：恢复 `wheelchair_ui` Web 2D 用户地图（复用 git 历史资产）。
- 3e. **语音**：恢复语音意图链路（仅输出意图，不直接控速）。

## 3. 复用资产（git 历史）
Nav2 launch/params、AMCL/定位健康、goal_manager、semantic_map_store、
wheelchair_ui（FastAPI Web）、wheelchair_voice_agent。这些都已存在，按子阶段逐个接回并验证。

## 4. 安全红线（全程不变）
- Web/语音/模型**不得**直接发 `/cmd_vel` 或 `/cmd_vel_safe`。
- 一切运动经 `/cmd_vel_nav → safety_supervisor → /cmd_vel_safe`。
- 电机默认不动，载人测试前需完整安全验证。

## 5. 验收标准（待细化）
- 能在已知地图中重定位并稳定显示当前位置。
- 能在 RViz/界面点选目标点，轮椅经 Nav2 + safety 自主到达。
- POI 保存/调用、禁行区生效。
- 上层应用不绕过安全链路。

## 6. 待阶段2 完成后补充
各子阶段的入口、参数、验证脚本与逐项验收清单。
