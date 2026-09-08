# FAST-LIO2 H0 静止基线（2026-09-04）

## 范围与安全边界

- 主仓库分支：`feature/mapping-v2-rviz-workbench`
- 主仓库 HEAD/origin：`19188466ff48d9874bcd148a14da1e714b14732b`
- 嵌套 FAST-LIO2 HEAD：`2fffc570a25d0df172720bac034fbdb6a13d2162`
- 用户确认 H0 就绪：水平地面、无人乘坐且无人触碰。
- 启动参数固定为 `motion_control_enabled:=false`、`rviz:=false`、
  `enable_camera:=false`、`enable_ultrasonic:=false`。
- 本轮只验证静止前端、输出连续性、ZUPT 前提和关停；不验证真实采样时间、
  最终外参、动态门禁、地图精度、地面电动行驶或载人安全。

证据目录：
`/home/nvidia/smartwheel/auto_test/fastlio_h0_20260904_152735`

## 启动与初始化

- 右 XT-M60 序列号/成像配置只读核对通过，使用 ImageType 4。
- FAST-LIO 启动参数确认：`R=0.001`、至少 50 个有效特征、残差上限
  `0.25 m`、可观秩至少 3、状态增量上限 `0.05 m / 2 deg / 0.5 m/s`，
  自适应重算最多 3 次、每次 covariance ×4。
- H30 初始化在 `401` 样本、`2.00021 s` 后完成；gyro 标准差
  `0.000960867 rad/s`，accel 标准差 `0.00919983 m/s²`。
- 健康轮速、连续 IMU 静止窗和时间对齐样本均成立，正规零速伪测量开始工作。

## 120 秒验收结果

监控时长 `120.0729 s`，`passed=true`，无 failure：

| 指标 | 结果 | 限值 |
|---|---:|---:|
| `/Odometry` 样本/平均率 | `1177 / 9.802 Hz` | 至少 2 样本 |
| 最大接收空窗 | `0.247693 s` | `0.5 s` |
| 最大 header 空窗 | `0.143955 s` | 记录项 |
| header 非单调 | `0` | `0` |
| 最大单帧平移 | `0.002409 m` | `0.05 m` |
| P95 单帧平移 | `0.001487 m` | 记录项 |
| 最大单帧旋转 | `0.193468 deg` | `2 deg` |
| 相对首帧最大活动半径 | `0.029841 m` | `0.05 m` |
| 健康 STOP 最大漂移 | `0.029841 m` | `0.05 m` |
| 全零 pose covariance 帧 | `0` | `0` |
| registered cloud 最大空窗 | `0.247495 s` | `0.5 s` |
| 轮速最大值 | `0 m/s, 0 rad/s` | 静止 |
| 健康反馈 | `5997 true / 0 false` | 当前健康且不陈旧 |
| 点云质量 | `1199` 条，拒绝/temporal/relative 均 `0` | 有消息、无拒绝 |

会话末尾 FAST-LIO 统计为：`accepted=1626`、`rejected=0`、
`projected=0`、`adapted=0`、`zupt=1626`；最后一帧 `features=832`、
`residual=0.0286 m`、`observability=6.032e-03`、`rank=6`。

本次 `2.98 cm` 活动半径低于正式 `5 cm` 门限，但高于历史描述的约
`1.12 cm`；两者时长和统计工具不完全相同，不能直接宣称性能变好或变差。
应在 H3 前保留该差异，并用同一监控器做重复静止对照。

## 关停证据

- 通过前台 launch 发送 SIGINT；随后未发现 `xtm60_adapter_node`、
  `fastlio_mapping`、`imu_adapter_node`、`zlac8030_driver_node` 或 launch 进程。
- 用户态 UDP socket 检查在 `192.168.0.100:7687` 与
  `192.168.1.100:7687` 分别监听 `3.0017 s`，两路均为
  `0 packet / 0 byte`，`pass=true`。
- 本轮为直接 launch，不存在需要额外停止的 transient service。

主要文件 SHA-256：

- `fastlio_h0_stationary.json`：
  `a26d3f8ccd38b77426704a83b3b97579c5bbf5c68d8f6a34d7d6f4ba30f1f03c`
- `launch_console.log`：
  `2c4c9e577bf6129c5b6d23c1eb813fe1d2a04affb54a481e3ecb9da007a1c099`
- `udp_shutdown_check.json`：
  `a54cf0722b2819e25cbf7984264a482befa4cf87e467e0817b636f2cf1469344`

## 结论

H0 静止基线通过。下一阶段仍是 H1 采样时钟与固定延迟验证；不得因 H0
通过而解除 `BLOCKED_CONFLICT`，也不得跳过 H2 外参和 H3/H4 动态、负载
闭环。
