# 左雷达实验室自主建图 Demo

## 模式用途

右侧 XT-M60 正在返修，短期不可用。`left_lidar_lab` 是临时实验室 profile，使用左侧 XT-M60 完成低速自主建图演示，同时保留 `dual_lidar` 正式双雷达模式。

## 能力范围

- 左雷达通过单雷达 fallback 生成 `/points_merged`。
- RTAB-Map 生成 `/rtabmap/cloud_map` 三维点云地图和 `/rtabmap/grid_map` 二维投影。
- H30 IMU 与轮速经过 EKF 输出 `/odometry/filtered`。
- reactive explorer 根据 `/scan` 和四个超声波生成低速 `/cmd_vel_nav`。
- safety supervisor 将 `/cmd_vel_nav` 裁剪为底盘唯一接收的 `/cmd_vel_safe`。
- 四个超声波补充近距离盲区保护。
- RViz 显示左雷达、融合点云、地图、扫描、里程计和 TF。

## 不作承诺

- 这不是最终双雷达正式能力。
- 单个 120 度 FOV 雷达覆盖不足，存在明显盲区。
- 不适合狭窄、拥挤、临边、坡道、玻璃或动态人员密集环境。
- 不允许无人值守，不建议且默认禁止载人测试。
- 摄像头当前用于记录、画面或可选上色，不进入几何 SLAM 的硬同步链路。

## 运行步骤

先进行离地测试，清空场地并确认物理急停可用：

```bash
colcon build --symlink-install
source install/setup.bash
bash scripts/run_left_lidar_lab_mapping_demo.sh
```

运行脚本会在启动可写底盘命令的 ROS 栈之前检查左雷达 `192.168.0.101`。如果检查失败，先确认左雷达供电、交换机/网线和链路灯，再执行：

```bash
bash scripts/setup_radar_network.sh --check-only
```

网络检查失败时脚本不会启动 autonomous mapping stack。
若默认数据库已存在，运行脚本会先在同一目录创建带时间戳的
`rtabmap.backup-YYYYmmdd-HHMMSS.db` 备份，再开始新的演示地图。

在另一个终端检查状态：

```bash
source install/setup.bash
bash scripts/check_left_lidar_lab_mapping_status.sh --require-motion-enabled
```

只有输出 `READY_TO_ARM_LEFT_LIDAR` 后才发布使能：

```bash
ros2 topic pub --once /autonomy/enable std_msgs/msg/Bool "{data: true}"
```

直接使用 launch 时等价命令为：

```bash
ros2 launch wheelchair_3d_mapping autonomous_rviz_mapping.launch.py \
  hardware_profile:=left_lidar_lab \
  enable_motion:=true \
  autonomous_exploration:=true \
  exploration_mode:=reactive \
  require_enable_signal:=true \
  max_linear_speed:=0.03 \
  max_angular_speed:=0.18 \
  turn_trigger_distance:=0.60 \
  stop_on_safety_warning:=false \
  use_colorizer:=false \
  rviz:=true
```

## 验收标准

- `/xtm60/left/points` 持续有数据。
- `/points_merged` 持续有数据。
- `/points_merged/status` 显示 `left_fresh=true` 且 `output_points > 0`；`right_fresh=false` 和 `single_lidar_fallback=true` 是预期状态。
- `/scan_left` 和 `/scan` 持续有数据。
- `/odometry/filtered` 持续有数据。
- `/rtabmap/cloud_map` 的点数或消息内容随移动增长。
- `/rtabmap/grid_map` 的有效栅格和已知区域随移动增长。
- `/exploration/status` 出现 `FORWARD`、`TURN` 或 `TURNING` 状态。
- `/cmd_vel_nav` 线速度不高于 0.03 m/s，角速度不高于 0.18 rad/s。
- `/cmd_vel_safe` 输出极低速命令，或被安全层按障碍距离合理减速、置零。
- RViz 中实验室轮廓逐步生成。

可用以下命令观察地图和速度：

```bash
ros2 topic hz /rtabmap/cloud_map
ros2 topic echo /rtabmap/cloud_map --once --field width
ros2 topic hz /rtabmap/grid_map
ros2 topic echo /cmd_vel_nav
ros2 topic echo /cmd_vel_safe
```

RViz 的 `left_lidar_lab_mapping.rviz` 默认显示 TF、`/xtm60/left/points`、`/points_merged`、`/rtabmap/cloud_map`、`/rtabmap/grid_map`、`/scan_left`、`/scan`、`/odometry/filtered` 和由 `/exploration/status` 派生的 `/exploration/status_marker`。两个摄像头图像为可选且默认关闭。`/cmd_vel_nav` 和 `/cmd_vel_safe` 是 RViz 默认插件不能直接显示的 Twist 消息，应在终端中观察。

## 急停

任一异常立即按物理急停，并执行：

```bash
bash scripts/stop_autonomous_mapping.sh
```

也可以在 launch 终端按 `Ctrl+C`。停止后确认：

```bash
ros2 topic echo /cmd_vel_safe
```

其线速度和角速度必须为零。

## 恢复双雷达

右雷达返修回来后，使用默认 `hardware_profile:=dual_lidar`：

```bash
ros2 launch wheelchair_3d_mapping autonomous_rviz_mapping.launch.py \
  hardware_profile:=dual_lidar
```

恢复前重新校验左右雷达外参、`/points_merged/status` 中两侧 freshness、双输入 `scan_merger`、watchdog，以及检查脚本的默认双雷达结果。双雷达自主运动时不允许 single-lidar fallback。
