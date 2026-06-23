# Requirements Document — narrow-fov-mapping-audit

## 简介

本 spec 对 SmartWheel 项目的**当前实现**（`feature/fastlio-narrow-fov-mapping` 分支，提交 `26927ea`）
进行全项目代码审查与修复，目标是交付一个完整可用的 **120°×45° 窄视场雷达建图系统**，在 RViz 中呈现：

- **3D 上色累积点云**（按高度 Z 渐变，对标示例截图一的全场彩色点云）
- **2D 占据栅格**（俯视室内平面图，对标示例截图二左下角的"地图"面板）
- **机器人轨迹**（蓝色路径线 + 编号航点，对标截图二主视图）
- **LaserScan 极坐标视图**（截图二右下角的"激光雷达"面板）
- **电池状态面板**（截图二右侧的 Battery Status）

### 硬件基线（已实测，不重测）

| 设备 | 地址 / 接口 | 状态 |
|---|---|---|
| **右 XT-M60** | 192.168.1.100，`/xtm60/right/points` | **主用**（左雷达入盒遮挡） |
| 左 XT-M60 | 192.168.0.100，`/xtm60/left/points` | 暂不可用（盒内） |
| H30 IMU | 串口，`/imu/data` ~200 Hz | 正常 |
| ZLAC8030D 底盘 | RS485 | 正常，`motion_control_enabled:=false` |
| Orin 主机 | 12 核，MODE_50W | 供电 brownout 风险，须限核运行 |

### 传感器特性（XT-M60）

- **视场角**：水平 120°，垂直 **45°**（比典型旋转雷达纵向窄得多）
- **帧率**：~10 Hz
- **格式**：标准 `sensor_msgs/PointCloud2`，字段 x/y/z/intensity，organized cloud
- **无逐点时间戳、无 ring 字段** → deskew 必须关闭
- 外参（右雷达，20260623 标定）：高度 51.0 cm，pitch −1.04°，roll −9.1°，平面残差 4.3 mm

### 算法架构（已确立，本 spec 不变更）

```
XT-M60 点云 → lio_cloud_adapter_node → /lio/cloud_in
                                              ↓
IMU /imu/data ──────────────────→ FAST-LIO2 (fast_lio)
                                   iEKF: IMU 主导 yaw + LiDAR scan-to-map 修正
                                   LASER_POINT_COV=100 (让 IMU 主导)
                                              ↓
    /Odometry  /cloud_registered  /path  TF odom→base_link
         ↓              ↓
  2D 投影栅格     RViz 3D 上色图
  /map_2d_from_3d

可选后端（默认 off）：
RTAB-Map (external-odom 模式) → 回环 / 位姿图 / 地图导出
```

**关键工程决策**（已确认）：
- FAST-LIO2 是唯一建图主线与实时位姿来源。
- RTAB-Map 降级为**可选回环/导出后端**，消费 FAST-LIO2 位姿，不抢前端里程计。
- 45° 纵向 FOV 下沿墙平移仍可能漂移（窄 FOV 通病），可由回环或轮速约束缓解（后续阶段）。

---

## Glossary

- **LIO_Pipeline**：FAST-LIO2 + adapter + 外参 + TF 桥接的完整建图链路。
- **LASER_POINT_COV**：vendored FAST-LIO2 源码中每点测量协方差，已 patch 为 100（IMU 主导）。
- **RViz_Layout**：目标 RViz 版式，含 3D 上色云 + 2D 栅格 + 轨迹 + LaserScan + 电池面板。
- **Right_Radar**：右 XT-M60，当前唯一可用雷达；话题 `/xtm60/right/points`。
- **Extrinsic_Right**：已标定，`xtm60_right_lio.yaml` 中 `extrinsic_T/R`，高度 51 cm。
- **Loop_Backend**：RTAB-Map 可选回环后端，`enable_loop_backend:=true` 时启动。
- **cpu_safe**：`auto_test/20260623_dual_radar_calib/cpu_safe.sh`，限核限优先级运行，防 brownout。

---

## Requirements

> 优先级：P0 = 阻断（当前无法运行）；P1 = 高（影响核心功能）；P2 = 中（质量/完整性）

---

### 需求 R1：右雷达 FAST-LIO 链路端到端可用

**优先级：P0**

**用户故事：** 作为操作者，我希望用一条命令启动右雷达建图，FAST-LIO2 正常运行并输出
`/Odometry`、`/cloud_registered`、`/path`，以便系统能够实时建图。

#### 验收标准

1. `scripts/run_rviz_manual_mapping_left.sh`（或等价脚本）可在 Orin 上成功启动，
   FAST-LIO2 初始化完成，`/Odometry` 以 ~10 Hz 稳定输出。
2. 启动时自动应用 `patches/apply_fastlio_patches.sh`（`LASER_POINT_COV=100`），
   或在文档中提示需要先应用。
3. 偏航跟踪正常：原地转圈后 yaw 累积接近真实角度（≥70% 真实值），不被 LiDAR 扫描拉回零。
4. `xtm60_right_lio.yaml` 外参已正确填入（高度 51 cm，pitch −1.04°，roll −9.1°）。
5. 启动后 `pgrep` 无残留进程（teardown 干净）。

---

### 需求 R2：RViz 版式对标示例图

**优先级：P0**

**用户故事：** 作为操作者，我希望在 RViz 中同时看到示例截图所示的 3D 上色点云、
2D 占据栅格、机器人轨迹和 LaserScan，以便直观监控建图进度。

#### 验收标准

1. **3D 上色点云面板**：显示 `/cloud_registered` 累积点云，按高度（AxisColor Z）或强度上色；
   对标截图一的全场彩色点云效果（深色背景、高度渐变色）。
2. **2D 占据栅格面板**：显示 `/map_2d_from_3d`（`cloud_to_occupancy_grid_node` 输出），
   为独立的 Map 显示项，对标截图二左下"地图（2D 占用栅格）"面板，含比例尺。
3. **轨迹面板**：显示 `/path`（机器人行进路径），对标截图二主视图中的蓝色路径线 + 航点。
4. **LaserScan 极坐标面板**：显示 `/scan`（`pointcloud_to_laserscan_node` 投影输出），
   对标截图二右下"激光雷达（LaserScan）"面板的红色极坐标扇形。
5. **电池状态面板**（Battery Status）：显示电量百分比、电压、电流、剩余时间，
   对标截图二右侧栏。数据不可用时面板留空不报错破版。
6. Fixed Frame 为 `map`（或 `odom`，视 TF 链路），RobotModel 正确显示。
7. RViz 配置文件 `manual_mapping_lio_right.rviz`（或已有的 `manual_mapping_lio_left.rviz`
   更新为 right）已就位，一键加载上述版式。

---

### 需求 R3：2D 栅格累积正确，供 Nav2 使用

**优先级：P1**

**用户故事：** 作为开发者，我希望 2D 栅格反映累积的建图结果（而非瞬时视野），
以便 Nav2 click-to-goal 路径规划能看到历史障碍。

#### 验收标准

1. `cloud_to_occupancy_grid_node` 订阅的是 FAST-LIO2 **累积地图**
   （`/cloud_registered` 多帧累积，或 `/cloud_map`），而非仅当前帧快照。
2. 机器人转头后，已驶过区域的墙体仍保留在 2D 栅格中。
3. 2D 栅格分辨率、range、地图坐标系与 RViz 显示一致（原点不随帧漂移）。

---

### 需求 R4：LaserScan 正确投影（右雷达 120°×45° FOV）

**优先级：P1**

**用户故事：** 作为操作者，我希望 LaserScan 反映右雷达 120°×45° 的真实扇形视场，
以便避障和 RViz 可视化准确。

#### 验收标准

1. `pointcloud_to_laserscan_node` 的 `min_height`/`max_height` 参数按右雷达 51 cm 安装高度
   和 ±45°/2 ≈ ±22.5° 纵向 FOV 正确配置（z 截取合理）。
2. `/scan` 在 RViz 中呈现约 120° 扇形（非 360°），与截图二右下极坐标视图一致。
3. `/scan` 时间戳基于采集时刻而非发布墙钟（修复 D024 — 已在 project_code_audit.md 记录）。

---

### 需求 R5：LASER_POINT_COV patch 可复现（防 re-clone 还原）

**优先级：P1**

**用户故事：** 作为维护者，我希望 FAST-LIO2 的 IMU 主导修复（LASER_POINT_COV=100）
在每次 re-clone/重建后能自动恢复，以免偏航追踪退化复发。

#### 验收标准

1. `patches/fastlio_laser_point_cov.patch` 存在且可正确应用到 commit `2fffc57`。
2. `patches/apply_fastlio_patches.sh` 为幂等脚本，已应用则跳过，未应用则自动打 patch。
3. `docs/fastlio_mapping.md §8` 明确记录 LASER_POINT_COV 脆弱点与重建后的恢复步骤。
4. `colcon build` 后 `LASER_POINT_COV` 在编译产物中为 100.0（可通过 strings/nm 验证）。

---

### 需求 R6：RTAB-Map 可选回环后端可用

**优先级：P2**

**用户故事：** 作为开发者，我希望 `enable_loop_backend:=true` 时 RTAB-Map 以 external-odom
模式启动并消费 FAST-LIO2 位姿，而不抢占 FAST-LIO2 的前端里程计。

#### 验收标准

1. 默认（`enable_loop_backend:=false`）下 RTAB-Map 不启动，FAST-LIO2 独立产图。
2. `enable_loop_backend:=true` 时 RTAB-Map 以 `odom_mode=external`，`odom_topic=/Odometry`
   启动，不发布 `odom→base_link`。
3. 回环后端与 FAST-LIO2 的 TF 帧名对齐（`odom`↔`camera_init` 需要对齐 TF，或文档说明当前未接）。
4. `docs/fastlio_mapping.md` 中有明确的"双角色"说明：FAST-LIO2=前端实时位姿；
   RTAB-Map=可选后端回环/导出。

---

### 需求 R7：Orin 防 brownout 规范落地

**优先级：P1**

**用户故事：** 作为维护者，我希望所有重负载命令都通过 cpu_safe 包装或等价限核手段运行，
以防止 Orin 供电 brownout 重启。

#### 验收标准

1. `auto_test/20260623_dual_radar_calib/cpu_safe.sh` 存在且可执行。
2. 雷达采集脚本（`capture_one.sh`）使用 cpu_safe 包装，并有 20s 硬上限 watchdog。
3. `colcon build` 使用 `--parallel-workers 2` 或等价限核方式。
4. 大点云 RANSAC 前子采样到 ≤4 万点。
5. `/var/log/journal/` 存在（持久化日志已启用），可在下次 brownout 后查看内核日志。
6. `.kiro/steering/orin-host-ops.md` 中记录了 sudo 密码、brownout 根因与防护规则。

---

### 需求 R8：项目文档整洁，反映当前实现

**优先级：P2**

**用户故事：** 作为维护者，我希望项目文档准确、不含误导性旧内容，以便新成员/新会话
能快速理解当前方案。

#### 验收标准

1. 顶层 `README.md` 描述 FAST-LIO2 单右雷达建图，FOV 标注 120°×45°。
2. `docs/fastlio_mapping.md` 完整，包含 vendored 源码获取步骤、外参、启动命令、地图保存。
3. 旧 RTAB-Map 时代文档已归档至 `docs/archive/`（不在主目录）。
4. `.kiro/specs/fastlio-narrow-fov-mapping/` 各 task 的 [x] 标记与实际实现状态一致。
5. `progress_log/` 有最新会话记录，说明未完成的 Task 15（现场建图验收）和 Task 16（文档收尾）。

---

### 需求 R9：全项目代码审查缺陷修复（按严重度）

**优先级：P1（高/阻断）/ P2（中）**

**用户故事：** 作为开发者，我希望 `docs/project_code_audit.md` 中标注为"高"和"阻断"的缺陷
得到修复，以确保里程计精度、安全链路和 RViz 可视化正常工作。

#### 验收标准

1. **D024（高）** `pointcloud_to_laserscan_node` 时间戳改为采集时刻而非墙钟。
2. **D025（高）** TF lookup 使用 `msg.header.stamp` 而非 `Time()`（最新 TF）。
3. **D029（高）** `scan_merger_node` 合并前对各路 scan 做 TF 变换到统一 frame。
4. **D034（高）** `dual_lidar_cloud_fusion_node` 输出时间戳改用源帧 stamp，而非发布墙钟。
5. **D001/D005（中）** 里程计单轮饱和与反馈失败时的零积分问题修复或文档说明。
6. `colcon test --packages-select wheelchair_3d_mapping wheelchair_sensors wheelchair_perception` 全绿。

---

## 已知限制（不在本 spec 修复范围，记录在案）

- 45° 纵向 FOV 导致地面点少、天花板点少，垂直结构信息弱。
- 无回环（默认）下大场景累积漂移不可避免；Loop_Backend 可缓解（需 TF 对齐完善）。
- 沿长走廊平移仍可能漂：IMU 锁住 yaw，但纯 LiDAR 沿墙方向仍弱约束。
- 左雷达入盒期间双雷达融合不可用；解盒后用 `RADAR=left` 恢复。
