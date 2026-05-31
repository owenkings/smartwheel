# RTAB-Map 3D 建图（当前主线）

## 一句话（汇报用）

> 当前系统采用 RTAB-Map / KISS-ICP **直接从 XT-M60 三维点云构建 3D 点云地图**，2D grid 仅为
> 导航投影，不是最终建图成果。

## 为什么是 RTAB-Map（暂停 FAST-LIVO2/R3LIVE）

- `/points_merged` 是普通 **XYZI 点云**（已在 base_link、滤波+voxel、~10Hz），XT-M60 是
  深度/Flash ToF 设备，不是 Livox 原始点云；FAST-LIVO2/R3LIVE 对其适配差、且 ROS2 集成成本高。
- **RTAB-Map**：ROS2 Humble 原生可用（apt 直装），直接吃点云+里程计/IMU，**更快跑出真实 3D 图**。
- FAST-LIVO2/R3LIVE 作为**后续高级 LIVO 方向保留**（`livo_*` 接口层仍在，`backend:=none`），
  不作为当前交付阻塞。

当前现实：

```text
主线：RTAB-Map 直接 3D 点云建图（已实测 /rtabmap/cloud_map）
保底：KISS-ICP 直接 3D 点云 odometry/map（已实测 /kiss/map_cloud）
暂停：FAST-LIVO2/R3LIVE（后续高级 LIVO 方向）
主输入：/points_merged + /imu/data 或 /wheel/odom（摄像头为增强）
主输出：/rtabmap/cloud_map 或 /kiss/map_cloud，保存为 PCD/PLY
导航投影：/rtabmap/grid_map（仅供 Nav2，不是建图成果）
```

## 这是原生 3D 建图，不是“先 2D 再渲染 3D”

- **主地图是 3D**：RTAB-Map 把每帧 `/points_merged` 按位姿图拼成 3D 点云地图
  `/rtabmap/cloud_map`，并存进数据库 `~/.ros/rtabmap.db`（位姿图 + 每关键帧点云）。
- **2D 栅格只是投影**：`/rtabmap/grid_map` 由 3D 点云按高度带压平得到，**仅供 Nav2**。
- 轨迹是 6-DoF：`/rtabmap/odom`（`icp_odometry` 对点云 ICP）。
- 超声波**不进 SLAM**（只进 safety/costmap）；摄像头上色/语义是**增强**，不阻塞几何 3D 建图。

## 安装依赖

```bash
sudo apt install ros-humble-rtabmap-ros      # RTAB-Map（主线）
pip install --user kiss-icp                  # KISS-ICP 保底（无 apt 包）
```

## 输入 / 输出

输入：`/points_merged`（必须）、`/imu/data` 或 `/wheel/odom`（至少其一）、`/camera/left/*`（可选增强）。
输出：`/rtabmap/odom`、`/rtabmap/cloud_map`、`/rtabmap/mapData`、`/rtabmap/grid_map`、数据库 `~/.ros/rtabmap.db`。

## 运行

```bash
cd ~/smartwheel && source /opt/ros/humble/setup.bash && source install/setup.bash

# 0) 清运行态（自启服务会抢串口/重复节点；这是文档化的人工步骤，launch 不自动杀服务）
systemctl --user stop smartwheel.service      # reboot 后会自启；如需长期停用再 disable
scripts/check_3d_mapping_runtime.sh           # 确认无重复 imu/ultrasonic 适配器

# 1) 一条命令：自带 sensors + dual-lidar 融合（无 base/EKF，icp_odometry 独占 odom->base_link）
ros2 launch wheelchair_3d_mapping rtabmap_3d_mapping.launch.py bringup_sensors:=true rviz:=true

#    或分步：先 sensors，再融合，再 RTAB-Map（确保只有一个 odom->base_link 发布者）
ros2 launch wheelchair_bringup sensors.launch.py mode:=real enable_xtm60:=false \
     enable_xtm60_left:=true enable_xtm60_right:=true enable_imu:=true enable_ultrasonic:=false enable_camera:=false
ros2 launch wheelchair_3d_mapping dual_lidar_fusion.launch.py
ros2 launch wheelchair_3d_mapping rtabmap_3d_mapping.launch.py        # bringup_sensors 默认 false

# 用外部轮速里程计代替 ICP 里程计（此时由该 odom 的发布者持有 odom->base_link）
ros2 launch wheelchair_3d_mapping rtabmap_3d_mapping.launch.py odom_mode:=external odom_topic:=/wheel/odom

# 加相机（仅纹理/视觉回环，几何不依赖标定）
ros2 launch wheelchair_3d_mapping rtabmap_3d_mapping.launch.py bringup_sensors:=true subscribe_rgb:=true

# 2) 保存 3D 地图
scripts/save_rtabmap_3d_map.sh                       # 数据库导出 PLY 到 maps/
scripts/save_rtabmap_3d_map.sh --live-pcd            # 实时 /rtabmap/cloud_map 落成 PCD
```

建图时**慢速**推动轮椅；XT-M60 仅约 120° 前向 FOV，快速旋转或长时间面对空旷区域会丢失约束。
电机默认不动（`motion_control_enabled:false`、命令寄存器 -1），本 launch 不启用任何运动。

## 保底：KISS-ICP

RTAB-Map 因相机/RGB-D 链路或其它原因阻塞时，用纯点云后端保底：

```bash
ros2 launch wheelchair_3d_mapping kiss_icp_mapping.launch.py bringup_sensors:=true
```

输出 `/kiss/odom`、`/kiss/path`、`/kiss/map_cloud`（`deskew` 默认 false：flash ToF 无 per-point 时间）。

## 验收

```bash
# 雷达 + 融合（点云是 best-effort QoS，ros2 topic hz 默认 reliable 可能收不到 ->
# 优先用 check 脚本，它用 sensor_data QoS 计数）
scripts/check_3d_mapping_runtime.sh
ros2 topic hz /xtm60/left/points
ros2 topic hz /xtm60/right/points
ros2 topic hz /points_merged
# RTAB-Map / KISS-ICP
ros2 topic hz /rtabmap/odom
ros2 topic echo /rtabmap/cloud_map --once   # RViz 里点云 Reliability 选 Best Effort
ros2 topic hz /kiss/odom
ros2 topic echo /kiss/map_cloud --once
ros2 run tf2_ros tf2_echo odom base_link
```

RViz：`Fixed Frame=map`，加 `PointCloud2 /rtabmap/cloud_map`、`Odometry /rtabmap/odom`、
`Map /rtabmap/grid_map`、TF；点云 Reliability=**Best Effort**。

> `rtabmap-export` 需要**移动过的**会话（≥2 关键帧）。纯静止只有 1 帧会报 `no odometry poses`
> 并跳过导出——此时用 `--live-pcd` 直接落盘 `/rtabmap/cloud_map`。

## TF 归属（单一发布者）

- `odom -> base_link`：`odom_mode:=icp`（默认）由 `icp_odometry` 唯一发布；`odom_mode:=external`
  时由外部 odom（如 ZLAC/EKF）发布。**两者不可同时发**。`bringup_sensors:=true` 只起
  传感器+融合，天然单一归属。
- `map -> odom`：`rtabmap` 发布。`base_link -> 传感器`：URDF（`robot_state_publisher`）。

## XT-M60 掉线排查

偶发 `cmdid=0xFF disconnected; waiting for self-reconnect` 会自恢复；若**持续**抖动：

- 供电：M60 需 12V/3A 或 19V/3.42A，电压/电流不足会反复重连；
- 网线 / 千兆交换机：双雷达隔离子网，确认链路与端口；
- 短时间内**反复重启 SDK**（多次 launch）会累积不稳，先彻底停掉再重来；
- 残留进程：`scripts/check_3d_mapping_runtime.sh` 查重复适配器，`systemctl --user stop smartwheel.service`；
- 单雷达保底：`dual_lidar_cloud_fusion` 的 `allow_single_lidar_fallback:true`，掉一台仍出 `/points_merged`。
