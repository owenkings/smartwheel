# SmartWheel

室内自主轮椅(ROS 2 Humble,Jetson Orin / aarch64,工作区 `/home/nvidia/smartwheel`)。

## 3D 建图主线:FAST-LIO2(LiDAR-惯性里程计)

当前 3D 建图采用 **FAST-LIO2(LiDAR-Inertial Odometry)**,取代了此前「纯 EKF 位姿 + RTAB-Map 零配准」
链路(后者在窄视场 XT-M60 上产生"漩涡/重影",根因是缺少 IMU 主导的帧间配准)。

- **传感器**:XT-M60 flash ToF(窄视场 120°×45°,横向 120°、纵向 45°,整帧快照、无逐点时间/ring)+ H30 IMU(~200 Hz)。
- **核心**:`fast_lio`(vendored `Ericsii/FAST_LIO_ROS2`)+ 一个 `lio_cloud_adapter` 把 XT-M60 点云
  适配成 FAST-LIO 的 Velodyne 输入(关 de-skew)。
- **位姿**:IMU 主导、LiDAR 辅助(关键调参 `LASER_POINT_COV=100`,见下)。EKF 关闭,FAST-LIO 独占
  `base_link` 位姿。
- **输出**:`/Odometry`、`/cloud_registered`(累积 3D 地图)、`/path`、`/map_2d_from_3d`(供 Nav2)。

### 当前部署:RIGHT 雷达
LEFT 雷达已入盒遮挡,**当前用 RIGHT 雷达**(高度 51.0cm、pitch −1.04°、roll −9.1°)。

```bash
# 一键启动(默认 right;只读,电机不动):
bash scripts/run_rviz_manual_mapping_left.sh
# 含运动(需空旷场地 + 物理急停在手):
RADAR=right MOTION=true bash scripts/run_rviz_manual_mapping_left.sh
# 停止:
bash scripts/stop_mapping.sh --force
# 保存地图(PLY 点云 + 2D 栅格):
bash scripts/save_mapping_result.sh <map_name>
```

LEFT 雷达出盒后:`RADAR=left bash scripts/run_rviz_manual_mapping_left.sh`。

### 关键文档
- `docs/fastlio_mapping.md` — FAST-LIO 获取/编译/外参/启动/保存/已知坑(**必读 §8 LASER_POINT_COV 脆弱点**、§9 供电 brownout)。
- `.kiro/specs/fastlio-narrow-fov-mapping/` — 需求/设计/任务(本次重构的完整 spec)。
- `.kiro/steering/orin-host-ops.md` — Orin 主机运行须知(sudo、供电 brownout 防护、传感器现状)。
- `progress_log/` — 历次对话/会话的进展记录(供其他对话或 AI 快速了解项目状态,见下)。

### ⚠️ 两个必知的坑
1. **LASER_POINT_COV**:Task 7 的 IMU-主导修复(`0.001→100.0`)在 vendored 源码里,re-clone/submodule
   update 会还原它 → 偏航不跟踪、地图变漩涡。修复后若复发,先跑 `bash patches/apply_fastlio_patches.sh`。
2. **Orin 供电 brownout**:重负载命令会触发 PMIC 硬复位(非软件/温度/内存)。重命令用 `taskset -c 0-3`
   限核、`--parallel-workers 2`;采集与离线处理不并行。

## 测试与验证
- `auto_test/` — 可视化自动化测试协议:每次测试一个 `<时间戳>_<主题>/` 目录 + `report.md`(现象/期望/根因/复现命令)。
- 单元测试:`colcon test --packages-select wheelchair_3d_mapping`。

## 进展记录
本仓库用 `progress_log/` 按日期记录每次对话/会话的进展,便于其他对话或 AI 接手时快速了解。
见 `progress_log/README.md` 与该目录的 hook 自动化说明。
