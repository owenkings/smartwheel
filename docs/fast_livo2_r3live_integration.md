# FAST-LIVO2 / R3LIVE 外部后端集成

> **状态（2026-06）**：FAST-LIVO2/R3LIVE 当前**暂停**，仅作后续高级 LIVO 方向保留。
> 当前 3D 建图主线是 **RTAB-Map**（见 `docs/rtabmap_3d_mapping.md`），保底是 KISS-ICP。
> 下文接口层（`livo_interface.yaml` + `livo_3d_mapping.launch.py`）仍在，未安装后端时 `backend:=none` 可跑全链路。

本仓库**不实现** LIVO 算法主体，而是以外部 ROS2 包的形式集成。`wheelchair_3d_mapping`
通过 `livo_3d_mapping.launch.py` + `config/livo_interface.yaml` 把外部后端接到轮椅话题上。
未安装外部后端时，本仓库仍可 `colcon build`，且 `backend:=none` 可运行除算法外的全部链路。

## 后端需要的输入 / 应产生的输出

输入（本仓库提供）：
- `/points_merged`  (sensor_msgs/PointCloud2, base_link, 10Hz)
- `/imu/data`       (sensor_msgs/Imu)
- `/main_camera/image_raw` (sensor_msgs/Image)
- `/main_camera/camera_info` (sensor_msgs/CameraInfo，需你提供，见下)

期望输出（供 EKF / 投影 / RViz 使用）：
- `/livo/odom` (nav_msgs/Odometry)
- `/livo/path` (nav_msgs/Path)
- `/livo/cloud_registered` (PointCloud2)
- `/livo/rgb_cloud_map` (PointCloud2 带 RGB，可选)
- `/livo/map_cloud` (PointCloud2 全局图，可选)

## 安装外部后端（示例，按实际仓库为准）

FAST-LIVO2 与 R3LIVE 多为 ROS1/ROS2 混合，需按其官方说明在 Orin 上编译。大致步骤：

```bash
cd ~/smartwheel/src
git clone <fast-livo2-or-r3live-ros2-repo>
# 按其 README 安装依赖 (Sophus / livox_ros_driver2 / PCL / OpenCV 等)
cd ~/smartwheel
colcon build --packages-select <backend_pkg>
source install/setup.bash
ros2 pkg list | grep -i livo   # 确认包名
```

确认后端的**包名**和**launch 文件名**，填入 `config/livo_interface.yaml`：

```yaml
backends:
  fast_livo2:
    package: <实际包名>
    launch_file: <实际launch文件>.launch.py
  r3live:
    package: <实际包名>
    launch_file: <实际launch文件>.launch.py
```

> 默认值 `fast_livo`/`r3live` 及 launch 文件名是占位猜测，**必须按你的安装核对**。

## 让后端话题对齐

外部后端内部话题名（如 `/cloud_registered`, `/aft_mapped_to_init`）通常与本仓库期望名不同。
在 `livo_interface.yaml` 的 `external_remappings` 用 `[from, to]` 对齐，launch 会用 `SetRemap` 应用：

```yaml
external_remappings:
  - ["/cloud_registered", "/livo/cloud_registered"]
  - ["/aft_mapped_to_init", "/livo/odom"]
  - ["/path", "/livo/path"]
  - ["/rgb_map", "/livo/rgb_cloud_map"]
```

后端若需要点云/IMU/图像输入名，也在此把它的输入名 remap 到 `/points_merged` `/imu/data` `/main_camera/image_raw`。

## camera_info

`camera_adapter_node` 只发图像，不发 `camera_info`。请用标定产生的内参提供 `/main_camera/camera_info`，例如：

```bash
ros2 run camera_calibration cameracalibrator ...   # 标定得到 yaml
# 然后用 camera_info_manager / image_proc 或自写小节点按标定 yaml 发布 /main_camera/camera_info
```

## 启动

```bash
# 仅后端（需先有 /points_merged /imu/data /main_camera/*）
ros2 launch wheelchair_3d_mapping livo_3d_mapping.launch.py backend:=fast_livo2

# 后端缺失或包名不对时，launch 会打印明确 ERROR 并退出，不会拖垮其它节点。
# 调试无后端：
ros2 launch wheelchair_3d_mapping livo_3d_mapping.launch.py backend:=none
```

## 三种 wheel 使用方式

1. 旁路监控：`wheel_livo_consistency_monitor`（本仓库已实现）。
2. 松耦合 EKF：`robot_localization_livo_wheel_ekf.yaml`（本仓库已实现）。
3. 紧耦合改 LIVO 源码：**本项目不实现**（不改后端源码）。
