# 3D SLAM 运行命令与排查

## 构建

```bash
cd ~/smartwheel
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

外部 LIVO 未安装也能构建本仓库。`backend:=none` 可跑除算法外的全部链路。

## 启动

```bash
# 仅双雷达融合
ros2 launch wheelchair_3d_mapping dual_lidar_fusion.launch.py

# 3D SLAM 总入口（EKF 拥有 odom->base_link，主摄像头=左）
ros2 launch wheelchair_bringup bringup_3d_slam.launch.py backend:=fast_livo2 main_camera:=left tf_owner:=ekf

# 后端未安装时的调试模式（不启动外部算法，明确提示）
ros2 launch wheelchair_bringup bringup_3d_slam.launch.py backend:=none

# 单独 3D->2D 投影
ros2 launch wheelchair_3d_mapping cloud_to_2d_map.launch.py

# 可视化
rviz2 -d install/wheelchair_3d_mapping/share/wheelchair_3d_mapping/rviz/3d_mapping.rviz
```

总入口参数：`use_sim_time backend main_camera tf_owner use_wheel_fusion use_colorizer use_cloud_to_2d enable_sensors provide_main_camera_alias`。

## Topic 验收

```bash
ros2 topic list | grep points
ros2 topic hz /points_merged
ros2 topic echo /points_merged/status --once
ros2 topic hz /wheel/odom
ros2 topic hz /imu/data
ros2 topic hz /main_camera/image_raw
ros2 topic echo /livo_wheel/status --once
ros2 topic echo /odometry/filtered --once
ros2 run tf2_ros tf2_echo odom base_link
bash scripts/check_3d_slam_topics.sh
```

后端运行后还应有：`/livo/odom /livo/cloud_registered /livo/path /map_2d_from_3d`。

## RViz 应显示
`/points_merged`、`/livo/cloud_registered`、`/livo/rgb_cloud_map`、`/livo/path`、`/map_2d_from_3d`、TF 树。

## 保存地图 / 录包

```bash
bash scripts/save_livo_map.sh            # 保存 LIVO 点云图（需后端支持）
bash scripts/record_3d_slam_bag.sh bags/run_001
```

## 排查

| 现象 | 检查 |
| --- | --- |
| `/points_merged` 无数据 | `ros2 topic hz /xtm60/left/points` `/right`；`/points_merged/status` 的 tf_ok；robot_state_publisher 是否在跑 |
| `/points_merged/status` tf_ok=false | URDF/`robot_state_publisher` 未发 `base_link->xtm60_*`；frame_id 是否匹配 |
| 只有一个雷达 | `allow_single_lidar_fallback:true` 会继续发布；查另一台 IP/网络 |
| `/livo/*` 无数据 | 后端未装/包名错 → 看 livo_3d_mapping 的 ERROR；`backend:=none` 时本就无 |
| `odom->base_link` 抖动/重复 | `tf_owner` 是否唯一；`view_frames` 看发布者数量 |
| 彩色点云颜色错位 | `rgb_colorizer.yaml` 外参未标定；`/main_camera/camera_info` 缺失 |
| `/map_2d_from_3d` 空 | 输入点云 frame 与 `map_frame` 不一致且无 TF；调 z-band |
| 电机不动 | **预期**：`motion_control_enabled:false`、命令寄存器 -1，自动运动被门控 |
