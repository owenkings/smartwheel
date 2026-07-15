# SmartWheel RViz2 建图操作台实现报告

## 1. 范围与安全边界

- 分支：`feature/mapping-v2-rviz-workbench`
- 基线：阶段 A-R 审查提交 `27bd875`
- 运行模式：仅 `mode:=mock`、`hardware_enabled:=false`
- 本阶段没有访问 XT-M60 网络、串口、CAN、RS485、`/dev/video*` 或电机驱动。
- Teleop 只发布 `/teleop/cmd_vel`，由 mock safety supervisor 限速、超时并转发到 `/cmd_vel_safe`。
- 启动文件和后端节点同时拒绝 `mode:=real` 或 `hardware_enabled:=true`，不存在静默回退。
- 本实现没有修改、fork 或复制 RViz2 上游源码。

## 2. 架构

中央三维区域继续使用 RViz2 原生 RenderPanel。它不是 Dock Panel，因此不会出现在 Panels 菜单中，也不能像自定义 Panel 一样关闭或拖出。三维交互仍由 RViz 原生 Orbit/MoveCamera 工具完成。

六个自定义插件均继承 `rviz_common::Panel`，通过 pluginlib 注册：

| Panels 菜单名称 | 类 | 主要职责 |
| --- | --- | --- |
| SmartWheel/2D Occupancy Map | `OccupancyGridPanel` | 二维栅格、轨迹、轮椅位置、缩放和平移 |
| SmartWheel/Camera | `CameraPanel` | 可重复实例化的有界缓存图像显示 |
| SmartWheel/Teleop | `TeleopPanel` | WSAD、鼠标、deadman、STOP 和超时停车 |
| SmartWheel/Mapping Control | `MappingControlPanel` | 调用真实 mock service 并显示状态机 |
| SmartWheel/System Status | `SystemStatusPanel` | 显示权威 diagnostics 和安全状态 |
| SmartWheel/Map Products | `MapProductsPanel` | 枚举、选择、预览和重新发布地图产品 |

插件包位于 `src/smartwheel_rviz_plugins`。ROS 回调通过 Qt queued signal 更新 GUI；CameraPanel 只保留最新帧，地图和状态更新也不维护无界 GUI 队列。耗时导出、rosbag 和 service 操作由后端执行，不在 Qt GUI 线程中执行。

## 3. 三维显示树

专用配置 `src/smartwheel_bringup/rviz/smartwheel_operator_workbench.rviz` 使用 `map` 作为 Fixed Frame，并创建 `SmartWheel 3D Mapping` 分组：

- 默认开启：RobotModel、TF、Grid、Dual LiDAR Merged、Optimized Global Map、LIO Path、Optimized Path、Fused Odometry。
- 默认关闭：左右原始点云、Primary LiDAR LIO Input、左右注册点云、Local Registered Cloud、Wheel Odometry。
- `/map_cloud` 使用 RGB8；后端规范化规则为 RGB 优先、其次 intensity、最后 Z 高度。
- `map` 不可用时 RViz 显示原生错误，代码不会切换到其他 Fixed Frame。

## 4. Panel 行为

### 2D Occupancy Map

正确处理 OccupancyGrid 的 width、height、resolution、带旋转的 origin、row-major data 和 occupied/free/unknown。显示 `/odom/fused` 轮椅位置与 `/mapping/optimized_path`，支持滚轮缩放、拖动平移、双击自适应、地图过期和无数据状态。

### Camera

同一插件默认创建 Front、Left、Right、Rear 四个独立实例。每个实例独立保存 image topic、camera_info topic、宽高比、GUI 最大帧率、时间戳/FPS、镜像、旋转和离线阈值。ROS 接收与 GUI 刷新分离，单个实例不会积累帧。

### Teleop

支持 W/A/S/D 大小写、组合键、鼠标按下/释放、Space/STOP、失焦、隐藏、析构和命令超时停车。默认线速度 `0.10 m/s`、最大 `0.15 m/s`，角速度和最大值 `0.25 rad/s`，命令频率 `20 Hz`，超时 `350 ms`。键盘自动重复不会延长危险命令。

### Mapping Control

调用 `/workbench/command`，显示 IDLE、CHECKING、RECORDING、MAPPING、LOOP_CLOSING、OPTIMIZING、EXPORTING、QUALITY_CHECK、READY、FAILED。状态、bag 路径、地图目录、关键帧、回环、点数和质量结果来自后端，不由按钮颜色推断。

### System Status

优先使用 `/diagnostics`、`/mapping/status`、`/hardware/status` 和 `/safety/status`。设备状态不会由 GUI 根据 topic 存在性猜测。mock、disabled、warning 和 error 明确区分，物理急停不会被宣称为已验证。

### Map Products

调用 `/map_products/task`，只列出具有完整 manifest 的版本。支持重新发布 `/map_cloud`、`/map` 和轨迹，显示质量摘要并打开目录；Panel 不提供删除操作。

## 5. 后端与数据产品

新增 `smartwheel_workbench` 后端负责：

- 权威 preflight 检查；
- zstd rosbag 生命周期和磁盘预留；
- mapping manager/service 协调；
- 地图产品保存、清单校验和预览发布；
- `/workbench/status` 与运行诊断；
- 实验元数据目录。

地图导出使用有上限的体素累积器，默认最多 500,000 个体素；超限明确失败，不会静默丢弃后继续宣称成功。正式产品包含 PCD、PLY、XYZRGB PLY、PGM、PNG、YAML、轨迹、poses、质量报告、profiles、bag 路径、RTAB 数据库和 manifest。

最终验证地图：

`maps/versions/rviz_workbench_final_verified_20260715_183757_216366`

其 manifest 标记 `complete: true`，13 个由导出器管理的文件均记录大小和 SHA-256。`rtabmap.db` 被声明为 externally managed，因为 RTAB-Map 在进程退出时仍会完成数据库写入。

## 6. 启动入口

操作台：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch smartwheel_bringup operator_workbench.launch.py \
  mode:=mock hardware_enabled:=false
```

地图预览：

```bash
ros2 launch smartwheel_bringup map_preview_workbench.launch.py \
  map_version:=/home/nvidia/smartwheel/maps/versions/<version_directory>
```

## 7. 主要提交

- `e17b439`：Workbench service/message 合约。
- `60906d7`：六个 RViz Panel 插件。
- `a73f486`：mock workbench session 后端。
- `0432484`：权威 mock diagnostics。
- `702112f`：操作台、地图预览 launch 和默认布局。
- `335affe` 至 `4958acd`：长会话导出、bag 可追溯、质量门和有界累积修复。
- `95534d1`：ROS context 关闭竞态和干净退出修复。

本任务到此停止，不进入 B0/B1，不构成连接任何真实硬件或启动电机的批准。
