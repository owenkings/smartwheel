# 阶段 2 — 自主建图（reactive）

> 状态：spec（占位，阶段1 稳定后细化）。前置：阶段 1 通过。后续：阶段 3。

## 1. 目标
在阶段1（手动建图链路稳定）的基础上，让轮椅**自主低速探索**并建图。先做 **reactive**（类扫地机初级模式：前方空就走、近了就转），**不做 frontier**。

## 2. 为什么先 reactive 不做 frontier
- frontier 依赖较稳定的 `/rtabmap/grid_map` 和 Nav2 路径规划，调试成本高。
- 当前单左雷达（120° FOV）、右雷达返修中，reactive 对地图质量要求低、更鲁棒、风险更可控。

## 3. 范围
**做：** 阶段1 全部 + reactive 探索节点（读 `/scan`，发 `/cmd_vel_nav`，经 safety）。
**不做：** frontier、Nav2 全局规划、定位重定位、POI 导航、GUI/语音。

## 4. 控制链路
```
reactive_explorer → /cmd_vel_nav → safety_supervisor → /cmd_vel_safe → 底盘
```
- 复用已有 `reactive_explorer_node`（git 历史中存在，发布 `/cmd_vel_nav`，不绕过 safety）。
- 双重门控：电机使能 + 显式 arm（`/autonomy/enable`）后才动。

## 5. 入口与文件（计划）
| 类型 | 文件 |
|---|---|
| Launch | `rviz_autonomous_mapping_left.launch.py`（待建，单雷达 reactive） |
| 探索节点 | `reactive_explorer_node`（复用） |
| 脚本 | `run_rviz_autonomous_mapping_left.sh`、`check_autonomous_mapping_left.sh`（待建） |

## 6. 验收标准（待细化）
1. arm 后轮椅自主低速前进，遇近障转向，不撞墙。
2. safety 始终可介入（减速/停/急停）。
3. 自主走动时 RTAB-Map 地图持续增长，能闭合实验室一圈。
4. 可保存地图。
5. disarm/急停立即停。

## 7. 风险与门控
- 高风险：轮椅自主运动。必须离地测试 → 空载清场低速 → 逐步放开。
- 速度上限收紧（如 ≤0.05 m/s 线速、≤0.18 rad/s 角速）。
- 物理急停常备。

## 8. 待阶段1 完成后补充
- reactive 参数（前向扇区角度、触发转向距离、速度）
- arm/disarm 的 RViz 面板按钮（可在 TeleopPanel 扩展）
- 具体验证脚本阈值
