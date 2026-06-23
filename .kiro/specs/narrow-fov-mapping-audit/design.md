# Design Document — narrow-fov-mapping-audit

## 概述

本 spec 是对已实现的 FAST-LIO narrow-FOV 建图系统的**审查与补全**，而非从头设计。
已有实现（`26927ea`）作为基线，本 spec 找出并修复 gap，使系统完全达到 requirements.md 所述目标。

---

## 现状分析（基线 `26927ea`）

### 已实现 ✅

| 组件 | 状态 | 文件 |
|---|---|---|
| FAST-LIO2 vendored（commit `2fffc57`） | 已落地，gitignored，文档化 clone 步骤 | `patches/`, `docs/fastlio_mapping.md` |
| `lio_cloud_adapter_node` | 已实现 + 单测（19 pass） | `src/wheelchair_3d_mapping/` |
| 右雷达配置 `xtm60_right_lio.yaml` | 已实现，外参 51cm/pitch −1.04°/roll −9.1° | `config/xtm60_right_lio.yaml` |
| `fast_lio_mapping.launch.py` `radar:=left|right` | 已实现 | `launch/fast_lio_mapping.launch.py` |
| 顶层 `manual_mapping_lio_right.launch.py` | 已实现（EKF off，FAST-LIO 拥有位姿） | `launch/manual_mapping_lio_right.launch.py` |
| LASER_POINT_COV=100 patch | 已 patch + 幂等脚本 | `patches/` |
| TF 桥接（map→camera_init，body→base_link） | 已实现 | `fast_lio_mapping.launch.py` |
| `cloud_to_occupancy_grid_node` 接入 `/cloud_registered` | 已实现 | 顶层 launch |
| RViz 版式 `manual_mapping_lio_left.rviz` | 已有（左雷达版） | `rviz/` |
| 死代码清理（KISS-ICP、ground_plane、livo placeholder） | 已删 | `git status` D 标记 |
| 旧 RTAB-Map 文档归档 | 已归档 | `docs/archive/` |
| Orin brownout 防护 | cpu_safe.sh + 持久 journald + steering | `.kiro/steering/orin-host-ops.md` |

### Gap / 未完成 ❌

| Gap | 对应需求 | 影响 |
|---|---|---|
| **RViz 版式未更新为右雷达 + 目标 layout** | R2 | 无法对标示例截图（电池面板、2D 栅格布局、轨迹） |
| **`/scan` LaserScan 时间戳用墙钟**（D024） | R4、R9 | scan 与 TF 时刻错位，2D 建图 / 避障漂移 |
| **TF lookup 用 `Time()`**（D025） | R4、R9 | 运动期点云投影位置错误 |
| **scan_merger 无跨 frame TF 变换**（D029） | R9 | 双雷达/双 scan 合并时几何错位（当前单雷达不触发） |
| **fusion 节点输出时间戳用墙钟**（D034） | R9 | 点云时间戳与 IMU 不对齐（当前右雷达单用，部分缓解） |
| **`cloud_to_occupancy_grid` 仅存瞬时帧**（D038） | R3 | 2D 栅格无历史，已驶过区域障碍被擦除 |
| **Task 15 现场验收未做** | R1 | 链路端到端未在真实运动场景确认 |
| **RTAB-Map 回环后端 TF 对齐缺失** | R6 | `enable_loop_backend:=true` 时帧名不匹配 |
| **右雷达 RViz 配置文件** | R2 | 当前只有 `manual_mapping_lio_left.rviz` |

---

## 修复设计

### F1 — RViz 版式（R2）

新建 `src/wheelchair_bringup/rviz/manual_mapping_lio_right.rviz`，布局对标示例截图二：

```
┌────────────────────────────────────┬──────────────────┐
│  3D 上色累积点云（主视图，Orbit）    │ Views 面板       │
│  /cloud_registered AxisColor(Z)    │ Battery Status   │
│  + RobotModel + TF + /path 轨迹    │                  │
├──────────────────┬─────────────────┤                  │
│ 地图（2D 占用栅格）│ 激光雷达         │                  │
│ /map_2d_from_3d  │ (LaserScan)     │                  │
│ Map + 比例尺      │ /scan 极坐标    │                  │
└──────────────────┴─────────────────┴──────────────────┘
```

关键参数：
- 3D 云：`Color Transformer: AxisColor`，`Axis: Z`，`Decay Time: 0`（不无限堆叠）
- 2D 栅格：`rviz_default_plugins/Map`，话题 `/map_2d_from_3d`，独立浮动面板
- 轨迹：`rviz_default_plugins/Path`，话题 `/path`，颜色蓝色
- LaserScan：`rviz_default_plugins/LaserScan`，话题 `/scan`，Size 0.02
- Battery：`wheelchair_bringup/BatteryPanel`（若已有）或 `rviz_default_plugins/StringDisplay`（话题 `/battery_state`）
- Fixed Frame：`map`

### F2 — 2D 栅格累积（R3，修复 D038）

`cloud_to_occupancy_grid_node` 目前每帧 `np.full(unknown)` 重建，不累积历史。

**方案**：在节点内部维护一个累积栅格 `_acc_grid`：
- 新帧 obstacle 点 → 标记 occupied（不可逆，除非显式 reset）
- 新帧 ground 点 → 标记 free（如果原来是 unknown）
- 定时器周期发布 `_acc_grid` 而非从零重建

同时修复 D039（`rolling` 模式下 origin 漂移）：`rolling` 模式用固定步进 anchor 而非每帧 min/max。

### F3 — LaserScan 时间戳修复（R4，修复 D024/D025）

`pointcloud_to_laserscan_node`：
- `restamp_output` 默认改为 `False`（保留源帧 stamp）
- TF lookup 改为 `lookup_transform(target_frame, frame_id, msg.header.stamp, timeout=Duration(seconds=0.1))`
  TF 不可用时跳帧，不用 `Time()` 强投影

### F4 — scan_merger TF 变换（R9，修复 D029）

单右雷达场景下 `/scan_left`/`/scan_right` 来自同一雷达投影，当前不触发几何错位。
**当前为低优先级**，记录在案；双雷达恢复时再实现。

### F5 — RTAB-Map TF 对齐（R6）

FAST-LIO2 的位姿在 `camera_init→body→base_link` 链上，而 RTAB-Map external-odom 期望
`odom→base_link`。需发布一个 `odom`↔`camera_init` 等价的 TF。

**方案**：在顶层 launch 中，`enable_loop_backend:=true` 时增加 static TF
`odom → camera_init`（identity）；FAST-LIO 的 `map→camera_init` 已有，链路即完整。

### F6 — LASER_POINT_COV 提为 ROS 参数（可选，R5）

将 `LASER_POINT_COV` 从 `#define` 提为 yaml 参数，免改第三方源码。
**实现复杂度较高**（需修改 C++ 类成员），列为后续优化；当前 patch 机制已足够。

---

## 文件改动一览（预计）

| 文件 | 类型 | 原因 |
|---|---|---|
| `src/wheelchair_bringup/rviz/manual_mapping_lio_right.rviz` | 新增 | 对标示例截图的右雷达版式 |
| `src/wheelchair_bringup/launch/manual_mapping_lio_right.launch.py` | 更新 | 引用新 rviz 文件 |
| `src/wheelchair_3d_mapping/wheelchair_3d_mapping/cloud_to_occupancy_grid_node.py` | 修改 | D038 累积栅格 |
| `src/wheelchair_perception/wheelchair_perception/pointcloud_to_laserscan_node.py` | 修改 | D024/D025 时间戳与 TF |
| `src/wheelchair_3d_mapping/launch/fast_lio_mapping.launch.py` | 可选更新 | D029 注释 + RTAB-Map 帧对齐 |
| `.kiro/specs/fastlio-narrow-fov-mapping/tasks.md` | 更新 [x] | Task 15/16 标记与进度同步 |
| `docs/fastlio_mapping.md` | 补充 | Task 16 文档收尾 |

---

## 验证策略

每项修复在 `auto_test/` 下建时间戳目录 + `report.md`，包含：
- `ros2 topic hz` 输出（链路活性）
- RViz 截图（版式面板）
- 地面 z 检查（`cloud_to_occupancy_grid` 累积正确性）
- `colcon test` 输出（单测全绿）

**雷达上电约束**：仍遵守 ≤20s 上电 + watchdog + cpu_safe 包装，防 brownout。
