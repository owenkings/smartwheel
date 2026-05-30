# 3D SLAM 标定清单

多传感器融合的成败主要取决于标定与时间同步，而非算法选择。上车前逐项完成。

## 1. 双雷达外参（lidar-lidar）
- URDF 当前值是估计：`base_link->xtm60_left_link xyz[0.45,0.24,0.65] rpy[0,-0.0873,0]`，右同位置 y=-0.24。
- 机械实测或用重叠点云 ICP 标定，更新 `wheelchair_description/urdf/wheelchair.urdf.xacro`。
- 验证：`ros2 topic echo /points_merged/status` 看左右点数；RViz 看左右点云墙面是否重合不重影。

## 2. 雷达-IMU 外参（lidar-imu）
- `base_link->imu_link xyz[0,0,0.45]`。LIVO 多数可在线估计外参，但初值要接近真实。
- 在 LIVO 后端参数中填入 `T_imu_lidar`（按其约定）。

## 3. 相机-雷达外参（camera-lidar）—— 点云上色依赖它
- `rgb_colorizer.yaml` 的 `cam_lidar_translation` / `cam_lidar_quaternion` 为 `T_camera_optical_from_lidar`，默认单位阵会导致颜色错位，节点会 warning。
- 用棋盘格 + 点云标定（如 `livox_camera_calib` 一类）得到外参填入。
- 注意：URDF 里 `camera_left/right_link` 当前是**侧向**安装（rpy 含 1.5708）。现在改为**前向**后必须重标 URDF 外参与该 yaml。

## 4. camera_info（内参）
- `camera_adapter_node` 不发 `camera_info`。用 `camera_calibration` 标定，按标定结果发布 `/main_camera/camera_info`。
- 无 `camera_info` 时：colorizer 与依赖视觉的 LIVO 会 warning / 退化，不崩溃。

## 5. IMU 速率与时间戳
- 当前 `imu_adapter` 默认 100Hz，且时间戳用发布时主机时钟。紧耦合 LIVO 建议：
  - 提高发布率到 ~200Hz（`h30_imu.yaml` 的 `publish_rate_hz`，并确认串口能跟上）。
  - 尽量使用硬件采样时间戳（需改 `imu_adapter_node`，本次未改）。

## 6. 轮速里程计（ZLAC8030D）
- `/wheel/odom` 已可用（反馈寄存器已配）。但 `register_to_rpm_scale` 等需现场标定（见 `zlac8030_base.yaml` 注释中的推距记录）。
- 验证：精确推 1.5m，比对 `/wheel/odom` 位移；按 `new_scale = scale * 实际/里程` 校正。

## 7. XT-M60 点云单位
- `xtm60_*.yaml` 的 `point_unit_scale` 默认 1.0；若 SDK 输出毫米改 0.001。用标尺确认。

## 8. TF owner 自检
- 启动后 `ros2 run tf2_tools view_frames`，确认 `odom->base_link` 只有一个发布者（与 `tf_owner` 一致）。
