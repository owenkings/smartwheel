# 阶段 0 — RViz 传感器可视化

> 状态：spec。前置：无。后续：阶段 1（手动建图）。

## 1. 目标
打开 RViz 后，能在正确的 TF 坐标系下看到所有可用传感器的实时数据。**不让轮椅移动**，不启动 safety/base/Nav2/RTAB-Map/explorer。

这一阶段回答的核心问题：我到底有哪些数据可用？坐标系对不对？传感器是不是活的？

## 2. 范围
**做：** 启动左雷达、IMU、超声波、左相机 + TF + scan 转换 + EKF（提供 `odom→base_link`）+ RViz。
**不做：** 电机控制、安全层、底盘、导航、建图、自主探索、GUI/Web/语音。

## 3. 需要的节点
- `robot_state_publisher`（静态 TF：base_link → 各传感器）
- 左 XT-M60 适配器（`/xtm60/left/points`）
- H30 IMU 适配器（`/imu/data`）
- 超声波适配器（`/ultrasonic/range_0..3`）
- 左相机适配器（`/camera/left/image_raw`）
- `pointcloud_to_laserscan`（左）+ `scan_merger` → `/scan`、`/scan_left`
- `robot_localization` EKF → `odom→base_link` TF + `/odometry/filtered`
  - 说明：EKF 至少有 IMU 输入即可发布 `odom→base_link`；静止时坐标停在原点，正是可视化所需。
- `rviz2`，加载 `sensor_view.rviz`（Fixed Frame = `odom`）

## 4. 入口与文件
| 类型 | 文件 |
|---|---|
| Launch | `src/wheelchair_bringup/launch/rviz_sensors_left.launch.py` |
| RViz 配置 | `src/wheelchair_bringup/rviz/sensor_view.rviz` |
| 启动脚本 | `scripts/run_rviz_sensors.sh`（设 `DISPLAY=:1`） |
| 验证脚本 | `scripts/check_rviz_sensors_left.sh` |

> 这些文件在 git 历史中已存在且阶段0已实测通过，恢复主线时从历史 checkout。

## 5. RViz 显示项（sensor_view.rviz）
- Fixed Frame = `odom`
- Grid、TF、RobotModel
- PointCloud2：`/xtm60/left/points`、`/points_merged`（建图阶段才有数据）
- LaserScan：`/scan_left`、`/scan`
- Range：`/ultrasonic/range_0..3`
- Odometry：`/wheel/odom`（阶段0 无底盘则空）、EKF `/odometry/filtered`
- Image：`/camera/left/image_raw`
- 工具：MoveCamera、SetInitialPose、SetGoal（阶段0 不使用，留作后续）

## 6. 运行方式
```bash
# 终端 A：启动（渲染到 NoMachine 桌面）
DISPLAY=:1 ros2 launch wheelchair_bringup rviz_sensors_left.launch.py rviz:=true
# 或：DISPLAY=:1 bash scripts/run_rviz_sensors.sh

# 终端 B：验证
bash scripts/check_rviz_sensors_left.sh
```

## 7. 验收标准（成功定义）
`check_rviz_sensors_left.sh` 输出 `STAGE0_OK`，且：
1. `/xtm60/left/points` ≥ 4 Hz
2. `/scan` ≥ 5 Hz
3. `/imu/data` ≥ 50 Hz
4. `/camera/left/image_raw` ≥ 3 Hz
5. 超声波至少有读数（range_3 偶尔静默可接受）
6. TF 链完整：`base_link→xtm60_left_link`、`base_link→imu_link`、`odom→base_link`
7. RViz 中能看到实验室点云轮廓、轮椅坐标系方向正确、IMU 朝向变化正确、相机画面显示
8. 无任何电机运动

## 8. 已知坑（来自实测）
- **Fixed Frame 必须是存在的帧**：若设为 `map` 但无定位，则点云/scan 全不显示，只剩相机图。阶段0 用 `odom`。
- **必须有 `odom→base_link`**：纯 `sensors.launch.py` 不发此 TF，需 EKF 或底盘提供。本阶段用 EKF。
- **右雷达已损坏**：launch 默认禁用，避免日志刷屏与误判。
- **进程残留**：重复启动前先彻底杀掉旧适配器/rviz 进程，否则抢占雷达 TCP / `/dev/video*` 导致断流。
- **RViz 的 DISPLAY 与 XAUTHORITY**：NoMachine 桌面通常是 `DISPLAY=:1`，但 X cookie 在
  `/run/user/<uid>/gdm/Xauthority`（**不是** `~/.Xauthority`）。两者都对才能渲染到桌面。
  NoMachine 重连后 display 号可能变，最稳的是在桌面终端 `echo $DISPLAY; echo $XAUTHORITY`
  取当前值。`run_rviz_sensors.sh` 已内置自动探测。
- **双摄像头**：两个相机各自独立显示（左 `/camera/left/image_raw`、右 `/camera/right/image_raw`），
  不合并。设备号：左 `/dev/video0`、右 `/dev/video2`（偶数为 capture，奇数为 metadata）。
  双路 MJPG 同总线时帧率会降到 ~3-4Hz，正常。
- **XT-M60 间歇断流**：左雷达偶发瞬时网络不可达（ping 100% 丢包几秒），适配器会自重连恢复，
  点云会自动回到 ~10Hz。若长时间不可达需查网线/供电。

## 9. 完成产物
- 可重复启动的阶段0 入口与验证脚本
- 验证通过记录（`STAGE0_OK`）
- 进入阶段1 的前置条件满足：传感器数据齐、TF 正确、坐标系确认无误
