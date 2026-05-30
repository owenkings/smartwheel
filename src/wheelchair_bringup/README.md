# wheelchair_bringup

集中放置 launch 和全局 YAML 参数。

常用入口：

```bash
ros2 launch wheelchair_bringup demo_mock.launch.py
ros2 launch wheelchair_bringup mapping.launch.py use_mock:=false
ros2 launch wheelchair_bringup localization.launch.py map:=maps/indoor_map.yaml
ros2 launch wheelchair_bringup navigation.launch.py
ros2 launch wheelchair_bringup full_system.launch.py map:=maps/indoor_map.yaml
ros2 launch wheelchair_bringup preflight_check.launch.py
ros2 launch wheelchair_bringup diagnostics.launch.py
ros2 launch wheelchair_bringup record_bag.launch.py output:=bags/test_001
ros2 launch wheelchair_bringup replay_bag.launch.py bag:=bags/test_001
```

当前只接 XT-M60、H30 IMU、2 个超声波和 2 个摄像头时，可直接使用一键脚本：

```bash
cd ~/smartwheel
chmod +x scripts/run_real_sensors.sh scripts/check_topics.sh
scripts/run_real_sensors.sh
```

脚本默认启动 IMU，但不启动 Nav2、底盘驱动和速度控制链路；只用于真实传感器接入验证。临时不接 IMU 时使用 `scripts/run_real_sensors.sh --no-imu`。

XT-M60 雷达网络可用以下脚本配置，不修改默认网关，不影响 WiFi。默认会给有线网口追加 `192.168.0.100/24` 和 `192.168.1.100/24`，并监听左右雷达 `192.168.0.101`、`192.168.1.101`：

```bash
sudo scripts/setup_radar_network.sh
sudo scripts/setup_radar_network.sh --install-service
```

双 XT-M60 布局：

- 左侧 XT-M60：`192.168.0.101`（网关 `192.168.0.1`），topic `/xtm60/left/points`
- 右侧 XT-M60：`192.168.1.101`（网关 `192.168.1.1`），topic `/xtm60/right/points`

需要手动指定时：

```bash
sudo scripts/setup_radar_network.sh \
  --radar-ip 192.168.0.101,192.168.1.101 \
  --host-cidr 192.168.0.100/24,192.168.1.100/24 \
  --gateway 192.168.0.1
```

只启动双雷达适配器：

```bash
ros2 launch wheelchair_bringup sensors.launch.py mode:=real \
  enable_xtm60:=false \
  enable_xtm60_left:=true \
  enable_xtm60_right:=true
```

自动运行和硬件收尾：

```bash
scripts/install_autostart.sh
systemctl --user start smartwheel.service
systemctl --user stop smartwheel.service
scripts/hardware_shutdown.sh
```

`smartwheel.service` 默认启动 `full_system.launch.py`，停止服务或脚本退出时会先发布急停和零速命令，再结束 ROS 进程，让各节点释放 XT-M60、串口、摄像头和底盘连接。

关键配置：

- `config/pointcloud_to_scan.yaml`：XT-M60 点云投影参数
- `config/xtm60_left.yaml`、`config/xtm60_right.yaml`：双 XT-M60 预留 IP、frame 和 SDK 参数
- `config/h30_imu.yaml`：H30 Yesense 串口参数
- `config/ultrasonic.yaml`：FD07-34 Modbus RTU 参数
- `config/camera.yaml`：USB 摄像头参数
- `config/zlac8030_base.yaml`：ZLAC8030/KeepLINK 底盘参数
- `config/diagnostics.yaml`：硬件自检、topic watchdog、定位健康阈值
- `config/passability.yaml`：通道宽度/轮椅通行性分析参数
- `config/safety_params.yaml`：安全距离、速度上限、动态停车距离参数
- `config/nav2_params.yaml`：Nav2 controller/planner/costmap
- `config/slam_toolbox_params.yaml`：建图参数
- `config/robot_localization_ekf.yaml`：轮速里程计和 IMU 融合预留

底盘单独启动：

```bash
ros2 launch wheelchair_bringup base.launch.py mode:=mock
```

注意：

- `demo_mock.launch.py` 中 `/wheel/odom` 和 `odom -> base_link` 只由 mock 底盘节点发布，`mock_sensor_node` 只保留 `map -> odom`，避免 TF 重复发布。
- `mapping.launch.py use_mock:=false` 会启动真实 sensors、点云转 scan、slam_toolbox 和 base odom source；真实建图仍需要 ZLAC8030 反馈寄存器或其他可靠 odom 来源。
- `navigation.launch.py` 会启动定位健康检查，并让 `safety_supervisor_node` 要求定位健康；定位/odom/scan 缺失时 `/cmd_vel_safe` 保持零速。
- ZLAC8030 寄存器默认禁用时，真实模式不会向电机写速度，也不会用目标速度伪造真实里程计。
