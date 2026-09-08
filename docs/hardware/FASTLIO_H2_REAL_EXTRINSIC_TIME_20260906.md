# FAST-LIO2 H2 实机外参与时间语义采集（2026-09-06）

## 范围与安全

用户明确确认“无人、清场、急停可达、操作者在场”。本轮只读采集，入口为
`right_lidar_diag_mapping.launch.py`，固定 `motion_control_enabled=false`、
`rviz=false`、关闭相机和超声波；没有电机控制写入、设备配置写入或最终外参写入。

## 证据

- 目录：`/home/nvidia/smartwheel/auto_test/h2_real_extrinsic_time_20260906_124524`
- bag：`bag/bag_0.db3`，约 `1127.928 s`、`1.7 GiB`、`316186` 条消息。
- `bag_info.txt`、`bag_sha256.txt`、`launch.log` 和 `bag_record.log` 同目录保存。
- 采集序列：初始静止 → 手动左侧姿态 → 回中静止 → 手动右侧姿态 → 面向两面
  不平行墙的墙角静止 → 约 1 m 缓慢直行 → 约 45° 手动左转 → 收尾静止。

## 原始通道结果

- 右 XT-M60 `/points`、`/quality`、`/timing` 各 `11266`；H30 `/imu/data`
  `225443`；`/Odometry` `10350`；`/tf` `10354`；健康反馈 `false=13704`；
  `/wheel/odom=0`。
- 右点云质量消息全部 `accepted`；valid fraction 中位数 `0.99427`，范围
  `0.68552..0.99490`，启动重连和低值段需短采复核。

## 时间语义观察

- XT-M60 `sdk_timestamp` 在 `1429.818..2556.818 s` 单调，帧间隔 `98..200 ms`
  （中位 `100 ms`，无非正间隔）；但 `timestamp_source` 仍是 `host_receive`。
- 点云 header 被定义为主机接收时刻，无法由本 bag 证明设备采样时钟或 LiDAR–IMU
  固定延迟。timing 中 publish 相对 host 的中位约 `86 ms`、最大约 `235 ms`，
  更像 ROS 发布排队，不直接作为传感器延迟。
- H30 `/imu/data` header 无负跳变但有 `2318` 个重复时间戳；其余多数约 `5 ms`，
  另见约 `10/20 ms` 间隔。严格单调采样时间验收未通过。

## 外参与动态结果

- 当前 bag 没有机器可读的动作标记，且使用临时外参、轮速反馈不健康；长录制中的
  FAST-LIO `/Odometry` 明显发散，不能作为几何真值或标定结果。
- 仍缺少可追溯的 `base_link` 原点与 IMU xyz 实测，以及不依赖 FAST-LIO 位姿的
  独立地面/墙面拟合。因此本轮是原始数据采集，不是最终外参通过。

## 关停

先停止录包，再 SIGINT 精确 XT-M60 适配器，最后停止 launch；结束时无雷达、IMU、
FAST-LIO、底盘或录包进程，两路 XT-M60 UDP 7687 静默。

## 结论

`H2_REAL_COLLECTION_CAPTURED_NOT_ACCEPTED`。保留 `host_receive`、
`time_sync_en=false`、`time_offset_lidar_to_imu=0.0` 和 `BLOCKED_CONFLICT`。
下一轮应采用更短且带动作标记的原始采集，先解决或明确 H30 重复时间戳处理规则，
并补齐 base/IMU 实测与独立墙面/地面拟合后再讨论最终外参。

## 后续软件修订（2026-09-07）

H30 适配器已加入主机侧逐样本时间线：保留原始 `host_receive_*` 读取边界，并在生产默认
`200 Hz` 下生成 `host_interpolated_*`；同一读取批中的多个样本不再默认共用 ROS header 时间。
`h2_direct_dual_capture.py` 的既有时间字段使用该估计时间，同时单独记录原始读取边界。

这项修订只处理主机批处理造成的重复时间戳，不声称恢复设备采样时刻、硬件同步或固定延迟。
本报告中的历史 bag 仍按 `host_receive` 分析，原结论
`H2_REAL_COLLECTION_CAPTURED_NOT_ACCEPTED` 不变。后续必须用新代码重新做带 marker 的短采集，
并在 H30、左雷达链路和温度状态稳定后，才可继续 H2 外参与时间验证。
