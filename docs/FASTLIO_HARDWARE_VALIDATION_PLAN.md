# FAST-LIO2 最终实机闭环计划

更新时间:2026-09-04。本文只安排必须依赖实物、人工观察或受控运动的数据采集。代码、补丁重放、
离线构建和无传感器冒烟应先完成;不要用标定去掩盖软件错误。

## 安全边界

- 任何电机动作前必须再次确认:无乘员、区域清空、物理急停可立即触达、操作者全程在场。
- 首轮时间/外参采集优先手推或电机关闭;需要原地转向或地面行驶时另行明确授权。
- 不同时改变外参、时间偏移、质量门和 iEKF 门限。每轮只改一个变量,保留前后证据。
- XT-M60 实机任务结束必须先向两个精确 adapter 进程发送 SIGINT,等待 `sdk.stop()/shutdown()`,
  再停止服务并确认无 adapter 进程、两路 UDP 均静默。

## H0:软件后静止基线

1. 轮椅保持水平、无人触碰,电机写入关闭。
2. 启动获准的右雷达诊断入口;等待 IMU 连续静止初始化完成(至少 2 秒/400 样本)。
3. 另一个终端运行:

   ```bash
   cd /home/nvidia/smartwheel
   source /opt/ros/humble/setup.bash
   source install/setup.bash
   python3 scripts/hardware/fastlio_acceptance_monitor.py \
     --mode stationary --duration-sec 120 \
     --require-feedback-health \
     --require-stop-window \
     --require-cloud --require-quality \
     --output auto_test/fastlio_h0_stationary.json
   ```

4. 通过条件:有持续 `/Odometry`;最大接收空窗不超过 0.5 秒;单帧平移不超过 5 cm、旋转不超过
   2°;首帧 covariance 非全零;活动半径不超过 5 cm;无反馈健康时 ZUPT 必须暂停而非触发。

## H1:采样时间与固定延迟

> H0 已于 2026-09-04 通过。120.07 秒监控无 failure，最大 Odometry/
> registered-cloud 空窗分别为 0.248/0.247 秒，最大单帧平移 2.41 mm、
> 旋转 0.193°，活动半径 2.98 cm；关停后两路 UDP 均为 0 帧。完整证据见
> `docs/hardware/FASTLIO_H0_STATIONARY_20260904.md`。该结果不替代 H1-H4。

当前 LiDAR 使用 SDK 回调的主机接收时刻；H30 生产配置保留串口读取完成边界，同时默认
按 nominal `200 Hz` 生成逐样本 host-interpolated 时间，避免一个读取批内的重复时间戳。两者
都早于 ROS 发布时刻，但仍不是真实硬件采样时间；原始 H30 边界字段必须保留用于审计。正式
启用设备时间前必须先完成以下只读验证:

1. 同时记录 `/xtm60/right/points`、`/imu/data`、`/wheel/odom`、`/Odometry`、
   `/base/wheel_feedback_healthy`、`/xtm60/right/quality`、`/tf`、`/tf_static`。不要同时录制
   `/cloud_registered_body`、`/cloud_effected` 和完整 `/path`。
2. 人工完成数次清晰、缓慢、方向相反的 yaw 激励,每次前后静止至少 5 秒。
3. 检查 XT-M60 `timeStampS/timeStampNS` 与 H30 timestamp TLV 是否真的是采样时钟:单调性、单位、
   重启归零、H30 32 位微秒回绕、10 分钟频率漂移都必须实测,不能从字段名猜测。
4. 用角速度/姿态变化做双向互相关,分别估计 LiDAR↔IMU 固定延迟及置信区间。若正反方向所得偏移
   不一致,先查外参和传输抖动,不得直接写 `time_offset_lidar_to_imu`。
5. 只有设备时钟行为和映射算法均通过后,才考虑从主机时间线切到设备采样时刻;否则保留原始
   `host_receive` 与明确标注的 `host_interpolated` 估计，并记录残余延迟。

## H2:最终 LiDAR/IMU/车体外参

需要操作者提供或配合测量:

- `base_link` 原点的物理定义,右雷达光学中心相对其 x/y/z,以及 IMU 中心相对其 x/y/z;
- 雷达安装面的水平 yaw 参考;当前纵向 x 和 yaw 不允许由地图“看起来顺”反推;
- 水平地面上的静止多姿态数据,以及含两面不平行竖墙的缓慢直行/转向数据。

处理顺序:

1. 先固定平移的实测先验和 H30 已知 roll/pitch,再拟合雷达相对 IMU 的 roll/pitch。
2. 用非平行竖直平面与受控 yaw 激励求 yaw;只用地面不能观测 yaw。
3. 在独立数据上检查地面高度/倾角、墙厚、正反转弯平移误差和 LiDAR/IMU 角速度残差。
4. 只有独立验证一致,才把 `right_lidar_stage1_calibration_contract.json` 从
   `BLOCKED_CONFLICT` 更新为可运行合同;同时更新 URDF、FAST-LIO extrinsic 和说明文档,禁止三处各写
   一套数值。

相机内参、相机外参不属于本轮 FAST-LIO 前端的阻塞项,应在 3D 彩色地图需要相机投影时单独标定。

## H3:门禁、ZUPT 与退化方向动态闭环

在 H1/H2 固定后,按顺序做四个短段,每段之间 STOP 静止至少 10 秒:

1. 低速直行;
2. 左/右各一次 90° 转弯;
3. 获准后原地左/右转;
4. 人工制造一次 1～3 秒点云消费者空窗,观察恢复。

运行动态监视器:

```bash
python3 scripts/hardware/fastlio_acceptance_monitor.py \
  --mode dynamic --duration-sec 180 \
  --require-feedback-health \
  --require-stop-window \
  --require-cloud --require-quality \
  --output auto_test/fastlio_h3_dynamic.json
```

同时保存 FAST-LIO 日志中的 `accepted/rejected/projected/adapted/zupt` 计数。验收要求:

- 正常运动不因跨帧像素变化被质量门丢弃;`temporal_jump`/`relative_drop` 可以报告但驾驶入口不据此
  hard reject;
- 长走廊/单地面中 `projected` 可增加,但可观秩不少于 3 时不应整帧拒绝;
- `adapted` 可在重捕获时增加,最终每次写入地图的修正仍小于 5 cm/2°/0.5 m/s;
- STOP 后只有时间对齐轮速、实时健康反馈和连续静止 IMU 窗口同时成立才执行 ZUPT;
- 任何反馈读取失败都应看到 `healthy=false` 和 `/wheel/odom` 空窗,不能出现伪零速样本。

## H4:高负载 A/B 压力测试

使用同一路径、相近速度和相同时长做三轮,不要一上来录三份大点云:

| 轮次 | 记录内容 | 目的 |
|---|---|---|
| A | 不录 bag,只运行监视器 | 算法/发布基线 |
| B | IMU、原始右点云、轮速、Odometry、质量/健康、TF | 推荐的可复现最小证据 |
| C | 在 B 上仅增加 `/cloud_registered` | 隔离世界系大点云序列化/磁盘负担 |

对比三个 JSON 的 Odometry 最大/P95 空窗、registered cloud 空窗和单帧跳变。若 A 已有长空窗,继续查
FAST-LIO 计算/ikd-tree;若只在 C 恶化,优先降低 registered cloud 发布/录制频率,不要放宽状态门禁。

## 最终交付证据

- H0～H4 的 JSON、精简 bag、启动/FAST-LIO 日志及确切 Git SHA;
- 最终时间源、固定偏移及测量方法;
- 最终 `base↔imu↔lidar` 数值、坐标约定、协方差/不确定度和独立验证结果;
- 三种压力轮次的对照表;
- 实机结束后的 inactive/no-process/no-UDP 关机证据。
