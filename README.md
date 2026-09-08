# SmartWheel Mapping V2

This workspace contains a clean ROS 2 Humble architecture for Stage A indoor mapping
without physical hardware. Stage A uses deterministic synthetic XT-M60-like point
clouds, H30-style IMU data, wheel encoders, four mock cameras, and mock-LIO copied from
ground truth. It does not validate any real sensor, motor, protocol, calibration, or
FAST-LIO2 performance.

Real transports and motor output are disabled by default. Do not set
`hardware_enabled:=true` until the Stage B runbook has been authorized and completed.

## Build and test

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
source install/setup.bash
colcon test
colcon test-result --verbose
```

The full build passes all 28 workspace packages. The retained third-party
`livox_ros_driver2` package has pre-existing copyright/style/uncrustify lint failures.
The mapping-v2 acceptance suite is isolated with:

```bash
colcon test --test-result-base /tmp/smartwheel_stage_a_results \
  --packages-select smartwheel_interfaces smartwheel_description \
  smartwheel_sensor_api xtm60_ros2_driver h30_imu_driver wheel_odom_driver \
  camera_array_driver dual_lidar_fusion smartwheel_state_estimation \
  smartwheel_global_mapping smartwheel_map_products smartwheel_mapping_manager \
  smartwheel_teleop smartwheel_bringup smartwheel_sim smartwheel_tests
colcon test-result --test-result-base /tmp/smartwheel_stage_a_results --verbose
```

## Stage A simulation

```bash
source install/setup.bash
ros2 launch smartwheel_bringup sim_mapping.launch.py \
  map_name:=stage_a_demo record_bag:=false
```

The default RTAB-Map path produces a versioned directory under `maps/versions/` with
PCD/PLY geometry, optional colored PLY, PGM/PNG/YAML occupancy data, trajectory files,
quality reports, manifest, hardware/algorithm profiles, and `rtabmap.db`.

Use the optional 2D baseline with:

```bash
ros2 launch smartwheel_bringup sim_mapping.launch.py mapping_backend:=slam_toolbox
```

## Record and replay

```bash
ros2 launch smartwheel_bringup sim_mapping.launch.py \
  map_name:=recorded_run record_bag:=true

ros2 launch smartwheel_bringup offline_mapping.launch.py \
  bag_path:=/absolute/path/to/raw_bag map_name:=replayed_run replay_rate:=1.0
```

Offline replay namespaces recorded derived TF and maps, reconstructs the selected
`odom -> base_link` edge, and leaves exactly one selected global backend responsible for
`map -> odom`. Replay at `1.0` is the validated rate.

For an externally supplied stream or bag, start the exporter and trigger it explicitly:

```bash
ros2 launch smartwheel_bringup map_export.launch.py
ros2 service call /map_export/export std_srvs/srv/Trigger '{}'
```

Key documents are [the delivery report](docs/STAGE_A_DELIVERY_REPORT.md),
[the frame/topic contract](docs/FRAME_AND_TOPIC_CONTRACT.md),
[implementation status](docs/IMPLEMENTATION_STATUS.md), and
[known limitations](docs/KNOWN_LIMITATIONS.md). Stage B must follow
[the hardware runbook](docs/HARDWARE_BRINGUP_RUNBOOK.md).

## Legacy deployment (not mapping v2)

The material below describes the pre-existing hardware stack. It is retained for audit
and reference and must not be mixed into mapping-v2 launches.

### 3D 建图主线:FAST-LIO2（软件与受限诊断链路）

当前 FAST-LIO2 仅代表已编译、已加固并可离线/受限诊断的前端；真实动态建图、最终外参、时间同步和产品地图仍未验收。

- 传感器：XT-M60 flash ToF（窄视场 120°×45°、整帧快照、无逐点时间/ring）+ H30 IMU（静态约 200 Hz）。
- 核心：fast_lio（vendored Ericsii/FAST_LIO_ROS2）+ lio_cloud_adapter 将 XT-M60 点云适配为标准 PointCloud2 输入（deskew 关闭）。
- 位姿：使用 IMU 预测和 LiDAR 更新门/回滚；正式配置 laser_point_cov=0.001，绝不使用历史 R=100 方案。EKF/FAST-LIO 的运行角色由具体诊断入口决定。
- 输出：/Odometry、/cloud_registered、/path 等当前帧/前端产品；不能把 cloud_registered 或 path 自动解释为累计、回环优化或正式地图。
- 时间语义：XT-M60 默认是 SDK 回调主机接收时间代理，H30 是读取边界之间的主机侧插值；均不等于设备采样时钟。

### 当前部署与诊断通路
左右雷达均保留只读诊断入口；右路曾有较稳定几何回波，左路有效率/链路仍需一变量一项排查。任何一路都不能绕过外参合同直接作为正式地图来源。

⚠️ **RIGHT 雷达的已审核通路是 fail-closed 的**:`right_lidar_stage1_calibration` 契约状态为
`BLOCKED_CONFLICT`(只做过地面拟合,给出高度 51.0cm / pitch −1.04° / roll −9.1°,**纵向 x 与
yaw 从未标定**),因此以下三处会在任何节点启动前抛
`BLOCKED_CONFLICT: right_lidar_stage1_calibration has no runtime-eligible transform`:
`launch/right_lidar_stage1_mapping.launch.py`(`manual_mapping_lio_right.launch.py` 只是它的转发壳)、
`launch/sensors.launch.py::_block_unapproved_right_lidar`、
`wheelchair_3d_mapping/launch/fast_lio_mapping.launch.py`(`radar=right`)。
`RADAR=right bash scripts/run_rviz_manual_mapping_left.sh` **不可用**,会走进上述门禁。

要在右雷达上做实机验证(WASD 驾驶 + FAST-LIO 3D + 2D 栅格),用不触碰门禁的诊断通路:

```bash
# 只读(电机不动),含 RViz + 3D/2D 建图 + 3 路相机:
bash scripts/run_right_diag_mapping.sh
# 含运动:RViz 面板内 W/A/S/D 驾驶,Space 立停(需空旷场地 + 物理急停在手):
MOTION=true bash scripts/run_right_diag_mapping.sh
# 纯建图,不开相机(负载最低):
CAMERAS=false bash scripts/run_right_diag_mapping.sh
# 停止:关闭 RViz 窗口 或 在终端按 Ctrl-C,两者都会关掉整栈。
# 兜底(会话异常残留时):
bash scripts/stop_mapping.sh --force
# 保存地图(PLY 点云 + 2D 栅格):
bash scripts/save_mapping_result.sh <map_name>
```

RViz 节点带 `on_exit=Shutdown`,所以它是这个会话的操作台:关掉它整栈就结束。在那之前终端里
节点日志会一直滚动,这是正常的。另注 **RViz2 不响应 SIGTERM**(优雅关闭卡在 Qt/Ogre 清理),
所以 `stop_mapping.sh` 要靠 KILL 阶段收掉它 —— 这也是强杀 RViz 时会看到
`guard condition` 崩溃转储的原因。

**⚠️ W/A/S/D 只有在显式开启 MOTION=true 且满足硬件安全授权时才会驱动轮椅。** 默认是只读，电机被有意门控：
`/base/status` 会显示 `motion_control_enabled=false`，驱动日志会记录非零命令被阻断。
离地、无人条件下已验证 GUI 按键/方向路由和安全层限幅；这不等于轮椅落地行驶、里程计精度、物理急停或载人安全通过。
在 RViz 里要先点 SmartWheel Teleop 面板获得键盘焦点；方向按钮可用鼠标点击，不依赖焦点。

**相机当前状态：压缩传输可用但右前链路仍受限。** 四路 30 分钟 compressed soak 无 USB/UVC 重置，
左前/左侧/右侧约 8.1–8.2 Hz；右前曾约 4.925 Hz 并产生 libjpeg 警告。该物理链路仍需相机/线缆/接口一变量一项 A/B，不能称为四路无条件通过。

⚠️ **四画面 RViz 必须用 compressed 传输,不能用 raw。** 这是本项目早已验收过的结论
(`docs/hardware/CAMERA_ARRAY_B4_REPORT.md`:raw 每路仅 0.67–0.73 Hz)。所以布局要用
`right_diag_mapping.rviz` 里的 `SmartWheel/Camera` 面板(带 `TransportHint: compressed`),
不要用 `rviz_default_plugins/Image`(那个订阅 raw)。用 raw 的后果实测是:帧率掉到 2.6–9 Hz、
四路里两路起不来,极端情况下还会触发 UVC/USB 链路重置 —— 那是二级现象,不是相机故障。
`manual_mapping_lio_right.rviz` 同时有两个问题(raw 传输 + 缺 `QMainWindow State` 导致
相机 dock 被压成零高度),不要用它跑这条通路。

**`right`(right-front,`2-3.4`)画面比其它三路模糊,是设计取舍不是故障。** 该链路的 MJPEG
流本身带损坏标记(文档记录:两分钟内其它三路 0 条 corrupt-JPEG 警告,right-front 81 条),
所以 `camera_quad.yaml` 里给它单独配了 `compressed_repair_cameras: [right]` +
`jpeg_quality: 40`,走 `jpegdec ! jpegenc quality=40` 重编码修复 —— 质量 40 就是它偏模糊的
直接原因。其它三路走 passthrough,画质原样。想更清晰可以调高 `jpeg_quality`(会掉帧率),或
把 `right` 从 repair 列表移除(画质原样但坏帧直接进 RViz)。根治需要做未完成的 A/B 交换,
定位是相机本体、线缆还是 `2-3.4` 接口。

细节与完整证据见 `auto_test/20260903_173000_right_diag_route/report.md`。

该通路使用 URDF 与 xtm60_right_lio.yaml 中的 provisional 安装值；right_lidar_stage1_calibration 合同仍为 BLOCKED_CONFLICT，
纵向 x、yaw、时间偏移和动态外参未标定。因此它只能用于观察节点/硬件是否运行，不能把输出地图当作标定合格产品。

LEFT 雷达恢复后仍须先完成冷却后的只读链路/质量检查和带 marker 采集；不能仅凭话题存在就进入最终标定或运动测试。

### 关键文档
- `docs/fastlio_mapping.md` — FAST-LIO 获取/编译/外参/启动/保存/已知坑(**必读 §8 LASER_POINT_COV 脆弱点**、§9 供电 brownout)。
- `.kiro/specs/fastlio-narrow-fov-mapping/` — 需求/设计/任务(本次重构的完整 spec)。
- `.kiro/steering/orin-host-ops.md` — Orin 主机运行须知(sudo、供电 brownout 防护、传感器现状)。
- `progress_log/` — 历次对话/会话的进展记录(供其他对话或 AI 快速了解项目状态,见下)。

### ⚠️ 两个必知的坑
1. **LASER_POINT_COV 与持久化**：历史 0.001→100.0 实验已证明不能作为通用根因修复，且可能重现静止发散。当前正式值为 0.001；权威补丁由 patches/apply_fastlio_patches.sh 幂等应用，并校验基线、补丁和修改后源文件 SHA-256。旧 fastlio_laser_point_cov.patch / fastlio_zupt.patch 只是不可应用墓碑。
2. **Orin 供电 brownout**:重负载命令会触发 PMIC 硬复位(非软件/温度/内存)。重命令用 `taskset -c 0-3`
   限核、`--parallel-workers 2`;采集与离线处理不并行。

## 测试与验证
- `auto_test/` — 可视化自动化测试协议:每次测试一个 `<时间戳>_<主题>/` 目录 + `report.md`(现象/期望/根因/复现命令)。
- 单元测试:`colcon test --packages-select wheelchair_3d_mapping`。

## 进展记录
本仓库用 `progress_log/` 按日期记录每次对话/会话的进展,便于其他对话或 AI 接手时快速了解。
见 `progress_log/README.md` 与该目录的 hook 自动化说明。
