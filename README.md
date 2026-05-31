# 室内智能自动轮椅 ROS2 第一版算法系统

本工程面向室内低速自动轮椅，目标是先跑通可构建、可运行、可扩展的第一版算法链路：传感器接入、TF、点云转二维激光、SLAM 建图、定位、Nav2 导航、安全限速、Web UI 命名目标点，以及语音/图像模型接口占位。

> **当前建图主线（2026-06）**：3D 建图主线为 **RTAB-Map**（直接从 XT-M60 三维点云构建 3D 点云地图，
> 输出 `/rtabmap/cloud_map` + 可保存 PCD/PLY），保底为 **KISS-ICP**；详见 `docs/rtabmap_3d_mapping.md`。
> 下文的 **2D `pointcloud_to_laserscan` + `scan_merger` + `slam_toolbox`（`/scan`）** 链路已降级为
> **导航/保底链路**——为 Nav2 提供 2D 定位与代价地图栅格，不再是最终建图成果。
> FAST-LIVO2/R3LIVE 为暂停的后续高级 LIVO 方向。

默认目标环境：Ubuntu 22.04 + ROS2 Humble + NVIDIA AGX Orin。Foxy/Galactic 可参考相同包结构适配，但 Nav2、slam_toolbox、robot_localization 的参数名和插件名可能需要按对应发行版调整。

## 硬件

- 主机：NVIDIA AGX Orin 64G
- 雷达：左右两台 XT-M60 固态 Flash 激光雷达，左侧 `192.168.0.101`（网关 `192.168.0.1`）、右侧 `192.168.1.101`（网关 `192.168.1.1`），两台位于隔离子网，第一版合并为 `/scan`
- IMU：H30，只用于姿态、角速度、上下坡、异常晃动检测，不作为长期定位唯一来源
- 超声波：6 个 topic 预留，当前默认启用 4 个 FD07-34 RS485/Modbus RTU（`range_0`~`range_3`）
- 摄像头：4 路预留，原 4 路中前向/后向已损坏，现仅左右两个前向摄像头可用，默认启用左右前向 USB 摄像头
- 底盘：中菱科技 ZLAC8030 轮毂伺服驱动器 + KeepLINK/RS485 通信设备，软件输出 `/cmd_vel_safe`，底盘节点发布 `/wheel/odom`

## 架构

```text
XT-M60 /xtm60/{left,right}/points --> pointcloud_to_laserscan --> scan_merger --> /scan --> slam_toolbox / Nav2 costmap
IMU /imu/data -------------------------------> robot_localization 预留
Ultrasonic /ultrasonic/range_* -------------> local costmap + safety_supervisor
Nav2 /cmd_vel_nav ---------------------------> safety_supervisor --> /cmd_vel_safe
/cmd_vel_safe -------------------------------> zlac8030_driver --> /wheel/odom + odom->base_link
Web UI / voice intent ------------------------> goal_manager --> /goal_pose
Camera /camera/*/image_raw ------------------> image recognition stub, 不直接控制速度
OccupancyGrid /map + semantic_map.yaml ------> Web UI 栅格底图 + 语义矢量图层
```

临时障碍只进入 local costmap 和安全监督，不写入静态地图。静态地图只在地图维护模式下由用户确认后更新；第一版提供维护入口和 TODO，不自动改图。

安全裁决规则：底盘节点只订阅 `/cmd_vel_safe`。Nav2、UI、语音和模型模块不得发布 `/cmd_vel_safe` 或底盘控制命令。`safety_supervisor_node` 会在物理急停、软件急停、雷达/超声波障碍、关键传感器故障、定位健康异常、通行性阻塞时输出零速。默认禁止自动倒车，默认最大自动导航线速度为 `0.4 m/s`。

## TF

```text
map
└── odom
    └── base_link
        ├── laser_link
        ├── imu_link
        ├── camera_front_link
        ├── camera_left_link
        ├── camera_right_link
        ├── camera_rear_link
        ├── ultrasonic_0_link
        ├── ultrasonic_1_link
        ├── ultrasonic_2_link
        ├── ultrasonic_3_link
        ├── ultrasonic_4_link
        └── ultrasonic_5_link
```

`base_link` 采用 ROS 移动机器人常用约定：X 向前，Y 向左，Z 向上。传感器外参在 `src/wheelchair_bringup/config/sensor_layout.yaml` 和 URDF 中给出，仅为初始估计，真实项目必须机械测量并在 RViz 中验证。

## Topics

| 类型 | Topic | 消息 |
| --- | --- | --- |
| 输入 | `/xtm60/left/points`、`/xtm60/right/points` | `sensor_msgs/PointCloud2` |
| 输入 | `/imu/data` | `sensor_msgs/Imu` |
| 输入 | `/ultrasonic/range_0` 到 `/ultrasonic/range_5` | `sensor_msgs/Range` |
| 输入 | `/camera/front/image_raw` | `sensor_msgs/Image` |
| 输入 | `/camera/left/image_raw` | `sensor_msgs/Image` |
| 输入 | `/wheel/odom` | `nav_msgs/Odometry` |
| 输入 | `/emergency_stop_hw` | `std_msgs/Bool` |
| 中间 | `/scan` | `sensor_msgs/LaserScan` |
| 中间 | `/obstacles` | `visualization_msgs/MarkerArray` |
| 中间 | `/safety_state` | `std_msgs/String` |
| 中间 | `/hardware/status` | `std_msgs/String` JSON |
| 中间 | `/system_stop_required` | `std_msgs/Bool` |
| 中间 | `/localization/health` | `std_msgs/String` JSON |
| 中间 | `/passability/status` | `std_msgs/String` JSON |
| 中间 | `/cmd_vel_nav` | `geometry_msgs/Twist` |
| 输出 | `/cmd_vel_safe` | `geometry_msgs/Twist` |
| 输出 | `/base/status` | `std_msgs/String` |
| 输出 | `/goal_pose` | `geometry_msgs/PoseStamped` |
| 输出 | `/map` | `nav_msgs/OccupancyGrid` |
| 输出 | `/amcl_pose` | `geometry_msgs/PoseWithCovarianceStamped` |

## 安装依赖

```bash
sudo apt update
sudo apt install -y \
  ros-humble-desktop \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-slam-toolbox \
  ros-humble-robot-localization \
  ros-humble-xacro \
  python3-serial \
  python3-opencv \
  python3-pip
pip3 install fastapi uvicorn pyyaml pytest pyserial
```

## AGX Orin 上手流程

以下流程默认代码已经放在 Orin 的某个目录，例如 `~/wheelchair_ws`。这里不包含上传/scp 步骤。

1. 进入工作空间并安装依赖：

```bash
cd ~/wheelchair_ws
source /opt/ros/humble/setup.bash
rosdep update
rosdep install --from-paths src -y --ignore-src
pip3 install fastapi uvicorn pyyaml pytest pyserial
```

2. 配置串口权限，重新登录后生效：

```bash
sudo usermod -a -G dialout $USER
```

3. 配置 XT-M60 网口。推荐使用脚本自动给有线网口追加雷达专用副地址，不修改 WiFi/互联网默认网关：

```bash
sudo scripts/setup_radar_network.sh
ping 192.168.0.101
ping 192.168.1.101
```

当前默认监听双 XT-M60：左侧 `192.168.0.101`（网关 `192.168.0.1`）、右侧 `192.168.1.101`（网关 `192.168.1.1`），两台位于隔离子网，Orin 默认追加 `192.168.0.100/24` 和 `192.168.1.100/24`。只接其中一台时也可直接运行；接入另一台后无需改代码。如果要开机自动配置：

```bash
sudo scripts/setup_radar_network.sh --install-service
```

该脚本不会设置默认网关，因此不会影响 WiFi 或其他网络。若需要手动指定双雷达参数：

```bash
sudo scripts/setup_radar_network.sh \
  --radar-ip 192.168.0.101,192.168.1.101 \
  --host-cidr 192.168.0.100/24,192.168.1.100/24 \
  --gateway 192.168.0.1
```

4. 配置 XTSDK。假设 SDK 放在 `/home/nvidia/smartwheel/xtsdk_py`：

```bash
export XTSDK_PY_ROOT=/home/nvidia/smartwheel/xtsdk_py
```

如需长期生效，可写入 `~/.bashrc`。

5. 构建：

```bash
cd ~/wheelchair_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

6. 先跑无硬件 mock demo，确认 ROS2 包和 UI 正常：

```bash
ros2 launch wheelchair_bringup demo_mock.launch.py
```

浏览器访问：

```text
http://<orin_ip>:8080
```

7. 做真实硬件 preflight 自检。先不要启动全系统，避免主动探测占用串口：

```bash
ros2 launch wheelchair_bringup preflight_check.launch.py
ros2 topic echo /hardware/self_check
ros2 topic echo /diagnostics
```

如果串口号不是默认值，修改：

```text
src/wheelchair_bringup/config/h30_imu.yaml
src/wheelchair_bringup/config/ultrasonic.yaml
src/wheelchair_bringup/config/camera.yaml
src/wheelchair_bringup/config/zlac8030_base.yaml
src/wheelchair_bringup/config/diagnostics.yaml
```

建议后续用 udev 固定设备名，例如 `/dev/h30_imu`、`/dev/ultrasonic_rs485`、`/dev/zlac8030`。

8. 启动真实传感器：

```bash
ros2 launch wheelchair_bringup sensors.launch.py mode:=real
ros2 topic hz /xtm60/right/points
ros2 topic hz /scan
ros2 topic hz /imu/data
ros2 topic echo /ultrasonic/range_0
ros2 topic hz /camera/front/image_raw
```

9. 启动底盘前先用 mock 模式验证 `/cmd_vel_safe -> /wheel/odom`：

```bash
ros2 launch wheelchair_bringup base.launch.py mode:=mock
ros2 topic pub /cmd_vel_safe geometry_msgs/msg/Twist "{linear: {x: 0.1}, angular: {z: 0.0}}"
ros2 topic echo /wheel/odom
```

10. ZLAC8030 真实模式必须先确认寄存器。默认 `command_*_register=-1`，不会给电机写速度。确认手册和 KeepLINK 模式后，再填写 `zlac8030_base.yaml`，并按顺序测试：

```text
离地空转 -> 空载低速 -> 有人旁站低速 -> 载人低速
```

11. 建图：

```bash
ros2 launch wheelchair_bringup mapping.launch.py use_mock:=false
ros2 run nav2_map_server map_saver_cli -f maps/indoor_map
python3 src/wheelchair_mapping/scripts/vectorize_occupancy_map.py \
  maps/indoor_map.yaml \
  --output src/wheelchair_navigation/config/semantic_map.yaml
```

12. 定位、导航和完整系统：

```bash
ros2 launch wheelchair_bringup localization.launch.py map:=maps/indoor_map.yaml
ros2 launch wheelchair_bringup navigation.launch.py
```

或：

```bash
ros2 launch wheelchair_bringup full_system.launch.py map:=maps/indoor_map.yaml
```

13. 运行中重点观察：

```bash
ros2 topic echo /safety_state
ros2 topic echo /hardware/status
ros2 topic echo /system_stop_required
ros2 topic echo /localization/health
ros2 topic echo /passability/status
ros2 topic echo /base/status
```

只要 `/system_stop_required=true`，安全节点会输出零速，不允许继续自动导航。

`navigation.launch.py` 会启动定位健康检查；如果 `/amcl_pose`、`/wheel/odom` 或 `/scan` 缺失，`/localization/is_healthy=false`，安全节点会保持 `/cmd_vel_safe=0`。如果只是调试 Nav2 配置而不运行定位链路，可以先用 mock demo 或完整系统入口验证。

14. 录制现场数据：

```bash
ros2 launch wheelchair_bringup record_bag.launch.py output:=bags/test_001
```

回放：

```bash
ros2 launch wheelchair_bringup replay_bag.launch.py bag:=bags/test_001
```

回放调试定位/可视化时，相关节点需要启用 `use_sim_time` 的后续参数化；第一版先提供标准 rosbag2 录制和回放入口。

## 构建

```bash
cd wheelchair_ws
colcon build
source install/setup.bash
```

## Mock Demo

```bash
cd wheelchair_ws
source install/setup.bash
ros2 launch wheelchair_bringup demo_mock.launch.py
```

验证关键 topic：

```bash
ros2 topic echo /safety_state
ros2 topic echo /cmd_vel_safe
bash scripts/check_topics.sh
```

打开 UI：

```text
http://localhost:8080
```

mock 障碍距离会周期性变化，`/safety_state` 应在 `CLEAR/WARNING/SLOWDOWN/STOP/EMERGENCY_STOP` 间变化。也可固定障碍距离：

```bash
ros2 launch wheelchair_bringup demo_mock.launch.py cycle_obstacle:=false obstacle_distance:=0.25
```

mock demo 中 `mock_sensor_node` 只发布 `map -> odom`，`odom -> base_link` 和 `/wheel/odom` 由 mock 底盘节点发布，避免 TF 重复发布。

## XT-M60 SDK 接入

解压官方 `xtsdk_py` 到机器人本机，例如：

```bash
sudo mkdir -p /home/nvidia/smartwheel/xtsdk_py
sudo cp -r /path/to/xtsdk_py-main/* /home/nvidia/smartwheel/xtsdk_py/
export XTSDK_PY_ROOT=/home/nvidia/smartwheel/xtsdk_py
```

AGX Orin + Ubuntu 22.04 通常使用 ROS2 Humble 的 Python 3.10，SDK 需要存在 `lib/linux/aarch64/xintan_sdk.cpython-310-aarch64-linux-gnu.so`。启动真实传感器：

```bash
ros2 launch wheelchair_bringup sensors.launch.py mode:=real
ros2 topic echo /xtm60/right/status
ros2 topic hz /xtm60/right/points
```

默认完整系统和建图入口会启用左右两个适配器；单独调试传感器时也可显式启用：

```bash
ros2 launch wheelchair_bringup sensors.launch.py mode:=real \
  enable_xtm60:=false \
  enable_xtm60_left:=true \
  enable_xtm60_right:=true
ros2 topic hz /xtm60/left/points
ros2 topic hz /xtm60/right/points
```

如果雷达不是默认 IP，修改 `src/wheelchair_bringup/config/xtm60_sdk.yaml` 或直接传参：

```bash
ros2 run wheelchair_sensors xtm60_adapter_node --ros-args \
  -p sdk_root:=/home/nvidia/smartwheel/xtsdk_py \
  -p connection_mode:=ethernet \
  -p ip_address:=192.168.0.101
```

左右雷达的 frame 分别为 `xtm60_left_link` 和 `xtm60_right_link`。`point_unit_scale` 默认 `1.0`，需要用实测标尺确认 SDK 点云单位；若 SDK 输出毫米，则设为 `0.001`。

## H30 IMU / 超声波 / 摄像头真实读取

H30 使用 Yesense 二进制帧协议，默认串口 `/dev/ttyUSB0`、`460800` baud，发布 `/imu/data`：

```bash
ros2 launch wheelchair_bringup sensors.launch.py mode:=real
ros2 topic hz /imu/data
```

FD07-34 超声波使用 RS485 Modbus RTU，默认串口 `/dev/smartwheel_ultrasonic`、`9600` baud，地址 `[1, 2, 3, 4]`，读取保持寄存器 `0x0001`，单位按 mm 转 m，发布 `/ultrasonic/range_0` 到 `/ultrasonic/range_3`；读取失败的传感器不再发布（交由 `safety_supervisor` 按超时判定故障），不会伪造最大量程，避免坏掉的传感器把障碍误判为通畅。

USB 摄像头使用 OpenCV/V4L2，现仅左右两个前向摄像头可用，默认启用 `left`、`right`，发布 `/camera/left/image_raw` 和 `/camera/right/image_raw`（设备号需按实际 `/dev/video*` 核对，建议用 udev 固定）。所有参数在：

```text
src/wheelchair_bringup/config/h30_imu.yaml
src/wheelchair_bringup/config/ultrasonic.yaml
src/wheelchair_bringup/config/camera.yaml
```

Linux 上需要串口权限：

```bash
sudo usermod -a -G dialout $USER
```

重新登录后生效。建议在 AGX Orin 上用 udev 规则把 H30、超声波、ZLAC8030/KeepLINK 固定成稳定设备名。

### 一键启动真实传感器

当前只接 XT-M60、H30 IMU、2 个超声波和 2 个摄像头时，使用：

```bash
cd ~/smartwheel
chmod +x scripts/run_real_sensors.sh scripts/check_topics.sh
scripts/run_real_sensors.sh
```

该脚本会启动真实传感器、`pointcloud_to_laserscan_node` 和 Web UI，不启动 Nav2、底盘驱动或自动驾驶速度链路。默认启动 H30 IMU；如果临时没有接 IMU：

```bash
scripts/run_real_sensors.sh --no-imu
```

如果已经构建过，跳过 build：

```bash
scripts/run_real_sensors.sh --no-build
```

另开终端验证：

```bash
cd ~/smartwheel
source /opt/ros/humble/setup.bash
source install/setup.bash
bash scripts/check_topics.sh sensors
ros2 topic hz /xtm60/right/points
ros2 topic hz /scan
ros2 topic hz /imu/data
ros2 topic echo /ultrasonic/range_0 --once
ros2 topic hz /camera/front/image_raw
```

## 建图

```bash
cd wheelchair_ws
source install/setup.bash
ros2 launch wheelchair_bringup mapping.launch.py use_mock:=false
```

真实建图入口会同时启动真实传感器、点云转 `/scan`、slam_toolbox 和底盘 odom source。若 ZLAC8030 反馈寄存器还没有配置，`odom -> base_link` 只能保持静止；这种情况下可以先用 mock 验证链路，真实建图需要接入轮速反馈或其他可靠 odom 来源。

流程：

1. 手动低速推动或驾驶轮椅绕室内走一到两圈。
2. 在 RViz 检查 `/scan`、TF 和 `/map`。
3. 保存地图：

```bash
ros2 run nav2_map_server map_saver_cli -f maps/indoor_map
```

生成 UI 用的墙线矢量层：

```bash
python3 src/wheelchair_mapping/scripts/vectorize_occupancy_map.py \
  maps/indoor_map.yaml \
  --output src/wheelchair_navigation/config/semantic_map.yaml
```

XT-M60 不是 360 度雷达，只有前向约 120 度 FOV。建图时必须慢速转向，让墙面和走廊边界被充分观测，避免快速旋转和长时间只面向空旷区域。

## 定位与导航

已有地图后：

```bash
ros2 launch wheelchair_bringup localization.launch.py map:=maps/indoor_map.yaml
ros2 launch wheelchair_bringup navigation.launch.py
```

完整系统入口：

```bash
ros2 launch wheelchair_bringup full_system.launch.py map:=maps/indoor_map.yaml
```

第一版默认自动导航最大线速度 `0.4 m/s`，调试上限 `0.6 m/s`，不建议直接进行 2-3 m/s 载人测试。最终发给控制器的速度必须使用 `/cmd_vel_safe`。

## ZLAC8030 底盘驱动与轮速里程计

底盘节点：

```bash
ros2 launch wheelchair_bringup base.launch.py mode:=mock
ros2 topic pub /cmd_vel_safe geometry_msgs/msg/Twist "{linear: {x: 0.1}, angular: {z: 0.0}}"
ros2 topic echo /wheel/odom
```

真实模式入口：

```bash
ros2 launch wheelchair_bringup base.launch.py mode:=real
```

配置文件：`src/wheelchair_bringup/config/zlac8030_base.yaml`。

当前已实现：

- `/cmd_vel_safe` 转左右轮 RPM
- Modbus RTU 读/写保持寄存器
- `/wheel/odom` 发布和 `odom -> base_link` TF
- 命令超时自动归零
- 开环里程计与反馈里程计两种路径

安全默认值：`command_left_register`、`command_right_register`、`feedback_*_register` 默认都是 `-1`，因此真实模式不会向未知 ZLAC8030 寄存器写速度，也不会把目标速度伪装成真实运动里程计。拿到你手上这套 ZLAC8030 + KeepLINK 的确切寄存器表后，把速度命令寄存器、速度反馈寄存器、缩放系数、左右轮方向填入 YAML，再进行离地低速测试。

## UI 使用

```bash
ros2 run wheelchair_ui wheelchair_ui --host 0.0.0.0 --port 8080
```

页面能力：

- 显示地图、当前位姿、安全状态、传感器在线状态
- 点击地图填入目标点坐标
- 添加/删除命名目标点，保存到 `wheelchair_navigation/config/named_goals.yaml`
- 显示/编辑 `wheelchair_navigation/config/semantic_map.yaml` 中的墙线、房间边界、禁行区、POI 等语义矢量图层
- 点击目标点发送 `/goal_pose`
- 停止/继续按钮发布软件急停

说明：当前不是商业高德地图式完整矢量导航引擎，而是“SLAM 栅格地图 + 墙线矢量化 + 语义矢量标注层”的可扩展第一版。室内导航仍由 Nav2 使用栅格地图和 costmap 完成；UI 叠加墙线、房间、目标点、禁行区等语义对象，后续可扩展为楼层、门、走廊中心线、路径偏好和地图维护审批。

## 语音和模型接口

```bash
ros2 run wheelchair_voice_agent command_parser_node
ros2 topic pub --once /voice/text_command std_msgs/msg/String "{data: '去卫生间'}"
```

输出 `/voice/intent` 示例：

```json
{"intent": "navigate_to", "goal_name": "卫生间", "confidence": 0.93}
```

模型只能输出结构化意图，不允许直接控制 `/cmd_vel_safe`。目标名不存在时由 `goal_manager_node` 返回错误，不生成虚假目标。

## 真实硬件接入

真实读取代码已落在以下节点：

- `wheelchair_sensors/xtm60_adapter_node.py`：官方 `xtsdk_py`，发布 `/xtm60/left/points`、`/xtm60/right/points`
- `wheelchair_sensors/imu_adapter_node.py`：H30 Yesense 串口协议，发布 `/imu/data`
- `wheelchair_sensors/ultrasonic_adapter_node.py`：FD07-34 Modbus RTU，发布 `/ultrasonic/range_*`
- `wheelchair_sensors/camera_adapter_node.py`：OpenCV/V4L2，发布 `/camera/front/image_raw`、`/camera/left/image_raw`
- `wheelchair_base/zlac8030_driver_node.py`：ZLAC8030/KeepLINK Modbus RTU 底盘接口，发布 `/wheel/odom`

物理急停输入仍需由你的硬件 IO 节点发布 `/emergency_stop_hw`。软件急停由 UI 发布 `/emergency_stop_sw`，统一进入 `safety_supervisor_node`。

## 测试

```bash
cd wheelchair_ws
pytest src/wheelchair_sensors/test src/wheelchair_perception/test src/wheelchair_safety/test src/wheelchair_navigation/test src/wheelchair_base/test src/wheelchair_diagnostics/test
```

当前最小回归测试覆盖传感器解析、点云投影、安全急停/倒车禁止/旋转近障停车、命名目标、语义地图、底盘运动学和诊断策略。最近一次本机结果：

```text
33 passed
```

## 已实现

- 标准 ROS2 workspace/package 骨架
- URDF/Xacro 和传感器 TF
- mock 传感器链路
- XT-M60 点云到 `/scan` 投影
- slam_toolbox 建图 launch
- AMCL/Nav2 launch 和参数
- 安全监督、动态停车距离、限速、软硬急停
- 命名目标 YAML 存储和 `/goal_pose` 发布
- FastAPI Web UI
- 语音/文本命令解析 stub
- 图像识别接口 stub
- H30 IMU Yesense 真实串口读取
- FD07-34 超声波 Modbus RTU 真实读取
- 前/左 USB 摄像头 OpenCV 真实读取
- ZLAC8030/KeepLINK 可配置 Modbus RTU 底盘节点、轮速运动学和 `/wheel/odom`
- 栅格地图叠加语义矢量图层的 UI 第一版
- 最小 pytest 与 topic 检查脚本

## TODO

- 用实际 ZLAC8030/KeepLINK 手册确认寄存器地址、速度单位、使能流程和反馈寄存器
- 接入物理急停 IO 节点并做硬件断电链路测试
- 接入电机编码器反馈后启用 EKF 融合 `/wheel/odom` + IMU
- 补齐建图产品化流程：rosbag 录制、离线建图、地图质量评分、地图版本管理和 GUI 验收状态
- 用实车数据标定 Nav2 costmap、DWB 局部规划器和安全阈值
- 完成地图维护模式：长期障碍候选、用户确认、静态地图写入
- 扩展右/后摄像头与行人/门状态/物体识别
- 将 UI 的导航状态升级为 Nav2 action 反馈和失败原因展示

## 3D SLAM (LiDAR-Visual-Inertial-Wheel)

> **状态（2026-06）**：3D 建图主线为 **RTAB-Map**（直接点云 3D 建图），保底 KISS-ICP；详见 `docs/rtabmap_3d_mapping.md`。
> 下述 FAST-LIVO2/R3LIVE 为**暂停**的后续高级方向（`backend:=none` 仍可跑传感器/融合/EKF/投影全链路）。
>
> ```bash
> ros2 launch wheelchair_3d_mapping rtabmap_3d_mapping.launch.py bringup_sensors:=true rviz:=true
> ```

在现有 2D/Nav2/安全链路之上新增了 3D 建图骨架包 `wheelchair_3d_mapping`：双 XT-M60 点云融合、外部 FAST-LIVO2/R3LIVE 后端接口、轮速/LIVO 一致性监控、3D→2D 占据栅格投影、可选彩色点云上色，以及 wheel+IMU+LIVO 松耦合 EKF。超声波不进 SLAM，电机默认不动。外部后端未安装时本仓库仍可 `colcon build`，`backend:=none` 可跑除算法外全链路。

总入口与调试：

```bash
ros2 launch wheelchair_bringup bringup_3d_slam.launch.py backend:=fast_livo2 main_camera:=left tf_owner:=ekf
ros2 launch wheelchair_bringup bringup_3d_slam.launch.py backend:=none   # 无后端调试
```

文档：

- `docs/rtabmap_3d_mapping.md`：**当前 3D 主线**（RTAB-Map 运行/验收/保存、KISS-ICP 保底）
- `docs/3d_slam_livo_wheel_architecture.md`：架构、传感器角色、TF、map/map_3d 关系
- `docs/fast_livo2_r3live_integration.md`：外部后端安装与话题对齐
- `docs/calibration_checklist.md`：双雷达/雷达-IMU/相机-雷达/IMU 速率/轮速标定
- `docs/runtime_commands_3d_slam.md`：构建、启动、验收、排查
