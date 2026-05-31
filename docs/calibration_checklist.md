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

## 相机内参 + 外参标定（操作命令）

> 前提：每个终端先 `cd ~/smartwheel && source /opt/ros/humble/setup.bash && source install/setup.bash`。
> **同一时刻只能运行一个相机节点**，否则设备被占用，报 `can't open camera by index`。

### 0. 准备（同款双 USB 相机的带宽问题）
```bash
# 同款相机共享 USB 带宽时第二台开不了 -> 加 uvcvideo quirk（已持久化可跳过）
cat /sys/module/uvcvideo/parameters/quirks            # 应为 128
sudo modprobe -r uvcvideo && sudo modprobe uvcvideo quirks=128
echo 'options uvcvideo quirks=128' | sudo tee /etc/modprobe.d/uvcvideo.conf
# 清掉残留相机节点，避免设备被占用
pkill -9 -f '[c]amera_adapter_node'
```

### 1. 内参标定（camera_info，每个相机一次）
```bash
# 终端A：只启动一个相机节点（发布 /camera/left|right/image_raw）
ros2 run wheelchair_sensors camera_adapter_node --ros-args \
  --params-file "$(ros2 pkg prefix wheelchair_bringup)/share/wheelchair_bringup/config/camera.yaml"

# 终端B：cameracalibrator。--size = 内角点数(列x行)，--square = 实测方格边长(米)
ros2 run camera_calibration cameracalibrator --size 8x6 --square 0.024 \
  --no-service-check --ros-args -r image:=/camera/left/image_raw
#   棋盘在画面 左右/上下/远近/倾斜 移动 -> CALIBRATE -> SAVE
#   右相机：把 image:=/camera/right/image_raw 再标一遍
```
保存后**立刻**拷出 `/tmp`（重启会清空 `/tmp`！）：
```bash
cp /tmp/calibrationdata.tar.gz ~/smartwheel/calib/left_cal.tar.gz   # 右相机改名 right_cal.tar.gz
rm -rf /tmp/ost && mkdir -p /tmp/ost && tar xzf ~/smartwheel/calib/left_cal.tar.gz -C /tmp/ost
cp /tmp/ost/ost.yaml ~/smartwheel/calib/left.yaml                   # 右相机 -> right.yaml
```
让系统发布 camera_info：在 `src/wheelchair_bringup/config/camera.yaml` 设
`left_camera_info_url`/`right_camera_info_url` 指向上面的 yaml，然后
`colcon build --packages-select wheelchair_bringup && source install/setup.bash`。验证：
```bash
ros2 topic echo /camera/left/camera_info --once     # k 应有非零 fx,fy,cx,cy
```

### 2. 外参标定 相机↔雷达（PnP，准且轻量）
```bash
# 起传感器+融合，产生 /points_merged(base_link) 和相机图
ros2 launch wheelchair_bringup bringup_3d_slam.launch.py backend:=none use_colorizer:=false use_cloud_to_2d:=false
# 采像素：点击同一批特征（记住顺序）
python3 scripts/pick_image_points.py --topic /camera/left/image_raw --out pts2d_left.txt
# 采 3D：RViz 里 Fixed Frame=base_link，显示 /points_merged，用 Publish Point 点同样特征：
ros2 topic echo /clicked_point          # 读出 x y z
# 写 corr_left.txt：每行 "u v x y z"（>=6，最好 8-12，分布广、远近不同），再求解：
python3 scripts/solve_cam_lidar_extrinsic.py calib/left.yaml corr_left.txt
#   输出的 cam_lidar_translation/quaternion 填入
#   src/wheelchair_3d_mapping/config/rgb_colorizer.yaml：
#     主相机(进LIVO的那台) -> cam_lidar_*；副相机(另一台,用其 calib+对应点重做) -> aux_cam_lidar_*
#   重投影误差 <2px 很好，<5px 够用，>10px 说明对应点配对有误，重挑。
```

### 常见报错
- `can't open camera by index`：设备被占用（已有相机节点/标定器在跑）或索引变了 → `pkill -9 -f '[c]amera_adapter_node'` 后只跑一个。
- 同款双相机只有一台能开 → `uvcvideo quirks=128`（见 0）。
- 重启后 `/tmp/calibrationdata.tar.gz` 丢失 → `/tmp` 被清空，标定后务必拷到 `~/smartwheel/calib/`。
