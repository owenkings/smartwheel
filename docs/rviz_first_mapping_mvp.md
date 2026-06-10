# RViz-first Mapping MVP

围绕 **RViz** 的极简主线：先可视化，再手动建图，再自主建图，最后才做导航/GUI/语音。
不优先做 GUI/Web、语音、3DGS/NeRF、frontier 自主探索、双雷达完整融合、用户 POI 导航。

分支：`feature/rviz-first-mapping-mvp`（从 `feature/livo-wheel-3d-slam` 拉出）。

## 硬件现状（2026-06）
- 左 XT-M60 `192.168.0.101`：正常，~10Hz。
- 右 XT-M60 `192.168.1.101`：**硬件损坏**（TCP 握手失败 / chip `0 0`），本主线**默认只用左雷达**。
- H30 IMU ~200Hz、4 路超声波、左 USB 相机（`/dev/video0`）正常；右相机 `/dev/video2` 设备不存在。

## 主线原则
1. **单左雷达是默认**，不是降级开关。诊断/scan/rviz 都用 left-only 配置。
2. 一切运动必经 `safety_supervisor`：`/cmd_vel_nav -> /cmd_vel_safe -> base`。
3. 每个阶段都有独立、解耦的 launch 入口和验证脚本，**不复用** `autonomous_rviz_mapping.launch.py` 那条大杂烩线。
4. 电机默认 `motion_control_enabled:=false`（只读）；真正驱动需显式开启 + 离地/清场 + 物理急停。

## 主线文件清单（只在这些范围内工作）

### 阶段 0 — RViz 传感器可视化（不让轮椅动）
- `src/wheelchair_bringup/rviz/sensor_view.rviz` — Fixed Frame=odom，含左雷达点云/scan/超声波/里程计/TF/相机
- 传感器入口：`src/wheelchair_bringup/launch/sensors.launch.py`（left-only 参数）
- `scripts/run_rviz_sensors.sh` — 在 DISPLAY=:1 上启动 RViz
- 验证：`scripts/check_rviz_sensors_left.sh`（待建）
- 成功标准：RViz 看到实验室点云轮廓、TF 方向正确、IMU 朝向变化正确、超声波有读数、相机画面显示、无电机运动。

### 阶段 1 — 手动行走建图
- `src/wheelchair_bringup/launch/manual_teleop.launch.py` — 传感器+TF+scan+EKF+单雷达 watchdog+safety+base+RViz（不含 Nav2/RTAB-Map/explorer）
- RViz Teleop 面板插件：
  - `src/wheelchair_bringup/include/wheelchair_bringup/teleop_panel.hpp`
  - `src/wheelchair_bringup/src/teleop_panel.cpp`
  - `src/wheelchair_bringup/rviz_panel_plugin.xml`
- 建图入口（待建，单雷达 RTAB-Map，仅建图，无自动驱动）：`manual_mapping_left.launch.py`
- 建图 RViz 配置（待建）：含 `/rtabmap/cloud_map`、`/rtabmap/grid_map`
- 脚本（待建）：`scripts/run_rviz_manual_mapping_left.sh`、`scripts/save_mapping_result.sh`
- 成功标准：用户按键/面板让轮椅慢速移动；RTAB-Map 用 `/points_merged`+`/odometry/filtered` 边走边建图；RViz 实时显示 3D 点云图和 2D 栅格；走一圈后保存 PCD/PLY/db。

### 阶段 2 — 自主建图（reactive，先不做 frontier）
（阶段1 稳定后再设计；本文件届时补充。）

### 阶段 3 — 定位导航 / GUI / 语音 / 用户地图
（最后做，暂缓。）

## 复用的底座资产（不重写）
XT-M60 左雷达驱动、H30 IMU、超声波、相机驱动、URDF/TF、robot_state_publisher、
wheel odom/base driver、robot_localization EKF、RTAB-Map、pointcloud_to_laserscan、
scan_merger、safety_supervisor、地图保存脚本。

## 暂缓（不在本主线，归档不删除）
GUI/Web（wheelchair_ui）、语音（wheelchair_voice_agent）、frontier explorer、
用户 POI 导航、语义地图/keepout、双雷达完整融合、`autonomous_rviz_mapping.launch.py`
大杂烩线、复杂诊断脚本、多 hardware_profile。

> 清理删除策略：阶段 1/2 验证稳定、依赖关系明确后再做针对性删除。当前用"主线清单 + 隔离"
> 保持干净，不物理删除文件，避免误删可复用资产。
