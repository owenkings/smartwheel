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

关键配置：

- `config/pointcloud_to_scan.yaml`：XT-M60 点云投影参数
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
