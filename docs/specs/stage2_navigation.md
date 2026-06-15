# 阶段 2 — 定位 + 导航（用手动建的图）

> 状态：spec。前置：阶段 1（手动建图）通过、已保存地图。后续：阶段 3（自主建图）。
> 顺序调整（2026-06）：导航提前到自主建图之前，"手动建图 + 导航"作为可用主线打底。

## 1. 目标
用阶段1 手动建好的地图（`maps/<name>/`），实现：
1. **定位**：在已知地图中确定轮椅当前位置（先手动设初始位姿）。
2. **导航**：在 RViz 点选目标点（或命名 POI），Nav2 规划路径，轮椅经 safety 自主到达。

形成最小可用闭环：人工建图 → 自主导航到目标点。

## 2. 范围
**做：** 加载已存地图 + 定位 + Nav2 规划/控制 + safety + 底盘 + RViz 目标点。
**不做（后置）：** 自主探索建图（阶段3）、frontier、GUI/Web、语音、自动重定位（先手动初始位姿）。

## 3. 定位方案（分步，从简到全）
- 2a. **手动初始位姿**：RViz 的 "2D Pose Estimate" 给初始位姿，定位用 RTAB-Map localization 模式或 AMCL。
- 2b. 之后再考虑开机自动重定位（AprilTag/充电标记/全局匹配）——本阶段不做。

定位用哪套要确认：
- **RTAB-Map localization 模式**（`Mem/IncrementalMemory:false`，加载 db）：和建图同一套，激光重定位，推荐。
- 或 **AMCL + 2D pgm 地图**：传统 2D 定位，需要 `/scan`。
两者都可行，倾向 RTAB-Map localization（与建图一致，复用 db）。

## 4. 导航数据流（安全不绕过）
```
RViz 目标点 /goal_pose (或 POI /named_goal_command)
  -> Nav2 (全局规划 + 局部控制)
  -> /cmd_vel_nav
  -> safety_supervisor   (这里恢复严格避障，不再 manual_bypass)
  -> /cmd_vel_safe
  -> 底盘
```
- **重要**：导航是自主运动，safety **必须用严格 profile**（`safety_params.yaml`，启用障碍避障），
  不能用手动建图的 `manual_bypass`。
- 地图：全局规划用 `/rtabmap/grid_map`（或加载的 pgm）；动态避障用实时 `/scan` + costmap。

## 5. 点云预处理 / 障碍判定（在本阶段落地）
自主运动需要可靠的实时障碍信息，这里实现之前讨论的点云筛选：
1. **统计离群点滤波（Statistical Outlier Removal）**：去掉孤立飞点（手动观察到的"波动噪点"）。
2. **多帧/时间一致性**：稳定出现的点才算障碍，压制瞬时噪点。
3. **密度/高度判定**：成片 + 离地一定高度的点 = 障碍（RTAB-Map `Grid/MaxGroundHeight/MaxObstacleHeight` 已做静态投影；实时避障在 costmap）。
4. **距离判定**：对确认障碍算最近距离 → costmap 膨胀 + safety 减速/停。
> 这部分是从"点云"到"可信障碍"的关键，手动建图阶段不需要，导航阶段必须有。

## 6. 复用资产（git 历史）
Nav2 launch/params（`navigation.launch.py`、`nav2_params.yaml`）、AMCL/定位健康、
goal_manager、named_goal_store、semantic_map_store、`pointcloud_to_laserscan`、costmap 配置。
按子阶段逐个接回并验证，不一次性全开。

## 7. 入口与文件（计划）
| 类型 | 文件 |
|---|---|
| Launch | `rviz_navigation_left.launch.py`（待建：加载地图 + 定位 + Nav2 + safety严格 + RViz） |
| 点云预处理 | fusion 节点加统计离群点滤波，或独立 cloud_filter 节点（待定） |
| 脚本 | `run_rviz_navigation_left.sh`、`check_rviz_navigation_left.sh`（待建） |
| 地图 | 阶段1 保存的 `maps/<name>/`（db + pgm + ply） |

## 8. 验收标准（待细化）
1. 加载已存地图，RViz 显示地图 + 轮椅当前位置（手动初始位姿后稳定）。
2. RViz 点选目标点，Nav2 出全局路径。
3. 轮椅经 safety（严格 profile）自主到达目标点，遇障减速/绕行/停。
4. 飞点不再误判为障碍（离群点滤波生效）。
5. 急停随时可介入。

## 9. 风险与门控
- 自主运动高风险：离地测试 → 空载清场低速 → 逐步放开。
- safety 必须严格 profile（非 manual_bypass）。
- 速度上限收紧，物理急停常备。

## 10. 待阶段1 收尾后细化
- 定位方案最终选型（RTAB-Map localization vs AMCL）
- Nav2 参数（costmap、规划器、速度）针对单 120° 雷达调参
- 点云预处理具体实现位置与参数
- 各子阶段验证脚本
