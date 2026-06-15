# 阶段 3 — 自主建图（reactive）

> 状态：spec（占位，阶段2 导航稳定后细化）。前置：阶段 2（定位+导航）通过。
> 顺序调整（2026-06）：自主建图后置。即使本阶段难度大或暂时做不出，
> "手动建图（阶段1）+ 导航（阶段2）"已是可用主线，不受影响。

## 1. 目标
在导航链路稳定的基础上，让轮椅**自主低速探索**未知环境并建图。先做 **reactive**
（类扫地机初级模式：前方空就走、近了就转），**不做 frontier**。

## 2. 为什么 reactive 先于 frontier
- frontier 依赖稳定的 `/rtabmap/grid_map` 和 Nav2 全局规划，调试成本高。
- 单左雷达（120° FOV），reactive 对地图质量要求低、更鲁棒、风险可控。

## 3. 范围
**做：** 建图链路（阶段1）+ reactive 探索节点（读 `/scan`，发 `/cmd_vel_nav`，经严格 safety）。
**不做：** frontier、GUI/语音。

## 4. 依赖阶段2 的成果
本阶段直接复用阶段2 落地的**点云预处理 / 障碍判定**（统计离群点滤波 + 多帧一致性 +
距离判定）。自主探索的避障质量取决于这套管线，所以必须在导航阶段先做好。

## 5. 控制链路
```
reactive_explorer → /cmd_vel_nav → safety_supervisor(严格) → /cmd_vel_safe → 底盘
```
- 复用 `reactive_explorer_node`（git 历史，发布 `/cmd_vel_nav`，不绕过 safety）。
- 双重门控：电机使能 + 显式 arm（`/autonomy/enable`）。

## 6. 入口与文件（计划）
| 类型 | 文件 |
|---|---|
| Launch | `rviz_autonomous_mapping_left.launch.py`（待建，单雷达 reactive） |
| 探索节点 | `reactive_explorer_node`（复用） |
| 脚本 | `run_rviz_autonomous_mapping_left.sh`、`check_autonomous_mapping_left.sh`（待建） |

## 7. 验收标准（待细化）
1. arm 后轮椅自主低速前进，遇近障转向，不撞墙。
2. safety 始终可介入（减速/停/急停）。
3. 自主走动时 RTAB-Map 地图持续增长，能闭合实验室一圈。
4. 可保存地图（复用阶段1 保存脚本）。
5. disarm/急停立即停。

## 8. 风险与门控
- 最高风险：轮椅自主运动。离地测试 → 空载清场低速 → 逐步放开。
- 速度上限收紧（≤0.05 m/s 线速、≤0.18 rad/s 角速）。
- 物理急停常备。

## 9. 待阶段2 完成后补充
- reactive 参数（前向扇区角度、触发转向距离、速度）
- arm/disarm 的 RViz 面板按钮
- 具体验证脚本阈值
