# LiDAR-Visual-Inertial-Wheel 3D SLAM 架构

> **状态（2026-06）**：3D 建图主线已改为 **RTAB-Map**（见 `docs/rtabmap_3d_mapping.md`），KISS-ICP 为保底。
> 本文描述的 FAST-LIVO2/R3LIVE 为**暂停**的后续高级 LIVO 方向，不作为当前交付；`backend:=none` 时本链路仍可用。

本文件描述 `wheelchair_3d_mapping` 包与外部 LIVO 后端组成的 3D 建图系统。
该系统在现有 2D `/scan`、Nav2、安全层、ZLAC8030D 之上叠加，不替换它们。

## 数据流

```text
/xtm60/left/points  ┐
/xtm60/right/points ┼─ dual_lidar_cloud_fusion_node ─► /points_merged (base_link, XYZI)
TF base_link<-xtm60_* ┘                                   │
                                                          ▼
/imu/data ───────────────────────────────►  外部 LIVO 后端 (FAST-LIVO2 / R3LIVE)
/main_camera/image_raw  ───────────────────►  (本仓库不实现算法主体)
/main_camera/camera_info ──────────────────►        │
                                                    ├─► /livo/odom
                                                    ├─► /livo/path
                                                    ├─► /livo/cloud_registered
                                                    ├─► /livo/rgb_cloud_map
                                                    └─► /livo/map_cloud
/wheel/odom ─┐
/imu/data  ──┼─ robot_localization EKF ─► /odometry/filtered (+可选 odom->base_link)
/livo/odom ──┘
/wheel/odom ─┐
/livo/odom  ─┴─ wheel_livo_consistency_monitor ─► /livo_wheel/status, /consistency_score

/livo/cloud_registered ─ cloud_to_occupancy_grid_node ─► /map_2d_from_3d ─► Nav2
/points_merged + image ─ rgb_cloud_colorizer_node(可选) ─► /rgb_cloud_map
```

## 传感器角色（谁进 SLAM，谁不进）

| 部件 | 角色 | 进入主估计器? |
| --- | --- | --- |
| 2× XT-M60 | 主几何，融合为 /points_merged | 是 |
| H30 IMU | 紧耦合运动先验（在 LIVO 内） | 是 |
| 1× 前向主摄像头 | 视觉约束 + 彩色地图 | 是（仅一个） |
| 另一前向摄像头 | 识别/显示/纹理 | 否（aux） |
| ZLAC8030D 轮速 | 松耦合 EKF + 一致性监控 | 否（不改 LIVO 源码） |
| 4× 超声波 | 近距离安全/限速/盲区 | 否（仅 safety/costmap） |

两个前向摄像头相距约 60 cm，中间盲区不可默认视为可通行；中间几何由 XT-M60 点云判断，近距离盲区由超声波 + `safety_supervisor` 负责。两摄像头**不是标定双目**，不得当双目用。

## TF 约定

静态外参全部来自 URDF (`wheelchair_description`)：`base_link` -> `xtm60_left_link` / `xtm60_right_link` / `imu_link` / `camera_*_link` / `ultrasonic_*_link`。

动态 TF：
- `odom -> base_link`：**同一时刻只能有一个发布者**，由 `tf_owner` 选择：
  - `ekf`（默认）：robot_localization EKF 发布；ZLAC `publish_tf:=false`；LIVO 不发布。
  - `wheel`：ZLAC 发布；EKF `publish_tf=false`。
  - `livo`：外部 LIVO 发布；EKF 与 ZLAC 都不发布。
- `map -> odom`：第一版由 Nav2/AMCL（使用投影 2D 图）或外部 LIVO 提供，二者择一。

## map 与 map_3d/livo_map 的关系

- LIVO 在自己的全局坐标系（本仓库记为 `livo_map`）输出 3D 位姿与点云。
- Nav2 使用 `map`（2D），来自 `cloud_to_occupancy_grid_node` 投影的 `/map_2d_from_3d` 或已保存的 2D 图。
- **第一版不强行统一 `livo_map` 与 `map`**：3D 图用于展示/结构理解/彩色点云；2D 投影用于轮椅导航。
  若要统一，可在标定后发布一个 `map -> livo_map` 的静态/对齐 TF，属于后续工作（见 calibration_checklist.md）。

## 关键 topic

| Topic | 类型 | 来源 |
| --- | --- | --- |
| `/points_merged` | PointCloud2 | dual_lidar_cloud_fusion_node |
| `/points_merged/status` | String(JSON) | 同上（诊断） |
| `/livo/odom` `/livo/cloud_registered` `/livo/rgb_cloud_map` `/livo/map_cloud` `/livo/path` | 各类 | 外部 LIVO |
| `/odometry/filtered` | Odometry | EKF |
| `/livo_wheel/status` `/livo_wheel/consistency_score` | String / Float32 | consistency monitor |
| `/map_2d_from_3d` | OccupancyGrid | cloud_to_occupancy_grid_node |
| `/rgb_cloud_map` | PointCloud2(RGB) | colorizer 或直接来自 LIVO |

## 安全

- 超声波不参与 SLAM；仅进入 `safety_supervisor`（已订阅 `/ultrasonic/range_0..3`）与 costmap。
- 电机默认不动：`zlac8030_base.yaml` 中 `motion_control_enabled:false`、命令寄存器 `-1`。
- `wheel_livo_consistency_monitor` 只报告不急停；其 `/livo_wheel/status` 可供 `safety_supervisor` 后续消费。
