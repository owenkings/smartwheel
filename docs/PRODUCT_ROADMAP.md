# 室内自动轮椅产品化待办清单

本文档按必要性排序，记录当前 ROS2 主线后续应做的功能、取舍、执行方案和代码落点。目标是：后续看到本清单即可继续实现，而不需要回看 Claude/Kiro 原项目代码。Claude/Kiro 只作为需求和思路参考，不直接复用其代码文件。

## P0 必要，已开始实现

### 1. 设备自检与自动探测

必要性：最高。没有自检，产品上电后无法判断雷达、IMU、超声波、摄像头、底盘和急停链路是否可用。

优点：
- 上车前能明确“可导航/不可导航/降级运行”。
- 串口号、摄像头索引变化时更容易定位问题。
- Web UI 可以给用户清晰错误原因。

代价：
- 主动探测会占用串口/摄像头，因此应放在启动前的 preflight 流程，运行中只做 watchdog。

执行方案：
- 包：`wheelchair_diagnostics`
- 节点：`hardware_self_check_node`
- 输出：`/hardware/self_check` JSON，`/diagnostics` DiagnosticArray
- 探测：
  - H30：扫描串口，按 `460800` baud 读取 Yesense 帧头 `0x59 0x53`，解析成功则通过。
  - 超声波：扫描 RS485 串口，向地址 `[1,2]` 发送 Modbus `0x03` 读寄存器 `0x0001`，CRC 正确则通过。
  - 摄像头：OpenCV `VideoCapture(index)` 打开并读取一帧。
  - XT-M60：检查 SDK 根目录和可选 TCP 端口；真实点云在线性由运行期 watchdog 判断。
  - ZLAC8030：只读探测寄存器，默认关闭，避免误写。

### 2. 传感器容错与系统门控

必要性：最高。载人系统不能在雷达、里程计、底盘通信丢失时继续自动导航。

优点：
- 把“设备在线状态”和“是否允许导航”解耦。
- 安全节点可统一接收 `/system_stop_required`，避免各节点各自停车。

代价：
- 需要明确每个传感器的故障等级，配置不当会过于保守。

执行方案：
- 节点：`sensor_watchdog_node`
- 输入：`/scan`、`/wheel/odom`、`/base/status`、`/imu/data`、`/ultrasonic/range_*`、`/camera/*/image_raw`
- 输出：
  - `/hardware/status` JSON
  - `/system_stop_required` Bool
  - `/system_stop_reason` String
  - `/diagnostics` DiagnosticArray
- 策略：
  - `/scan` 超时：停止自动导航。
  - `/wheel/odom` 超时：停止自动导航。
  - `/base/status` 超时：停止自动导航。
  - IMU 超时：降级，提示姿态/坡度保护不可用。
  - 超声波超时：降级，雷达正常时可低速运行。
  - 摄像头超时：降级，不影响基础导航。

### 3. 数据录制与回放

必要性：高。研发阶段需要复现急停、定位漂移、传感器丢帧、规划失败等问题。

优点：
- 现场问题可离线复盘。
- 可以在无硬件环境下回放调试算法。
- 便于形成测试集。

代价：
- 摄像头和点云占用磁盘较大。
- 回放时需要注意 `/clock` 和 `use_sim_time`。

执行方案：
- 使用 ROS2 标准 `rosbag2`，不自研 HDF5 主链路。
- Launch：
  - `wheelchair_bringup/launch/record_bag.launch.py`
  - `wheelchair_bringup/launch/replay_bag.launch.py`
- 默认录制 topics：
  - `/xtm60/points`、`/scan`、`/imu/data`、`/ultrasonic/range_*`
  - `/camera/front/image_raw`、`/camera/left/image_raw`
  - `/wheel/odom`、`/cmd_vel_nav`、`/cmd_vel_safe`
  - `/safety_state`、`/hardware/status`、`/localization/health`
  - `/tf`、`/tf_static`

### 4. 通行性分析

必要性：高。轮椅不是点机器人，门宽、走廊宽度、转弯空间直接影响可达性和安全。

优点：
- UI 可解释“目标不可达”的原因。
- 在窄门/狭窄通道前提前降级或阻止规划。
- 可补充 Nav2 footprint/costmap 的产品级解释能力。

代价：
- 第一版只能做几何估计，复杂家具环境仍需实车标定。

执行方案：
- 节点：`passability_analyzer_node`
- 输入：`/scan`
- 输出：`/passability/status` JSON
- 算法：
  - 将 LaserScan 投影为 `(x,y)`。
  - 在 `0 < x < lookahead_distance` 区间统计左右最近边界。
  - 估计通道宽度 `width = left_boundary - right_boundary`。
  - 要求宽度 `required_width = wheelchair_width + 2 * clearance_margin`。
  - `width < required_width` => `BLOCKED`
  - `required_width <= width < required_width + warning_margin` => `NARROW`
  - 否则 `CLEAR`
- 安全节点只对 `BLOCKED` 强制停车，`NARROW` 先作为 UI/诊断提示。

### 5. 定位健康度监控

必要性：高，但不替代 AMCL。真实部署时必须知道当前定位是否可信。

优点：
- 定位不可靠时可停车或要求重新定位。
- UI 能显示明确状态，而不是“车突然不动”。

代价：
- AMCL covariance 阈值需要实车调参。

执行方案：
- 节点：`localization_health_node`
- 输入：`/amcl_pose`、`/wheel/odom`、`/scan`
- 输出：`/localization/health` JSON，`/localization/is_healthy` Bool
- 算法：
  - 检查 AMCL 位姿是否超时。
  - 检查 `cov_x/cov_y/cov_yaw` 是否超过阈值。
  - 检查 `/wheel/odom` 和 `/scan` 是否新鲜。
  - 输出 `GOOD / DEGRADED / LOST`。
  - 后续可加入 scan-to-map residual、ICP inlier ratio 作为辅助指标。

### 6. 测试覆盖增强

必要性：高。不是追求测试数量，而是覆盖安全链路。

执行方案：
- 增加纯函数测试：
  - watchdog 策略
  - passability 宽度判断
  - localization health covariance 判断
  - Modbus/Yesense 异常解析
- 后续增加 launch smoke test 和 rosbag replay smoke test。

## P1 必要，但不是第一优先级

### 7. XT-M60 多数据输出

必要性：中高。导航主链路只需要点云/scan，但调试、识别和故障定位需要深度图、红外图、置信图。

优点：
- 可以做摄像头/雷达融合、低矮障碍识别、画面质量判断。
- UI 和 rosbag 数据更完整。

代价：
- 带宽和磁盘占用增加。
- SDK 输出结构需要继续实机确认。

执行方案：
- 不复用 Claude 代码文件。
- 在 `xtm60_adapter_node.py` 内重新实现 SDK frame 到 ROS topic 的转换。
- 目标 topics：
  - `/xtm60/depth_image`
  - `/xtm60/infrared_image`
  - `/xtm60/confidence_image`
  - `/xtm60/status`

### 8. Orin 部署工程化

必要性：中高。产品化阶段必须自启动、自动恢复和留日志。

执行方案：
- 增加 `deploy/systemd/*.service` 模板。
- 增加 `deploy/udev/99-wheelchair.rules` 模板。
- 配置 `journalctl` 日志查看流程。
- 崩溃自动重启后默认进入安全停车状态。

### 9. 标定流程

必要性：中高。未经标定的轮速里程计、外参和速度比例不能用于载人测试。

执行方案：
- 文档化：
  - 传感器外参测量
  - H30 安装方向确认
  - 轮径、轮距、左右轮方向
  - ZLAC8030 速度单位和寄存器比例
  - `/cmd_vel_safe` 到实际速度实测表

## P2 可考虑

### 10. Web UI 视频预览

优点：远程查看方便。

缺点：HTTP/MJPEG/WebSocket 视频有 100-500ms 级延迟，不能进入安全闭环。

执行方案：
- Orin 本地识别节点订阅 ROS Image。
- Web UI 只预览压缩图像。
- 低延迟调试仍使用 RViz/rqt_image_view。

### 11. 自研 ICP 作为定位辅助

优点：可提供 scan-to-map 匹配质量、退化检测、重定位辅助。

缺点：不应替代 AMCL/Nav2 主链路，否则产品风险上升。

执行方案：
- 独立节点订阅 `/scan`、`/map`、`/amcl_pose`。
- 输出匹配 residual、inlier ratio、degenerate flag。
- 只进入 `/localization/health`，不直接控制速度。

### 12. 完整高德式矢量地图引擎

优点：显示效果好，语义丰富。

缺点：短期对导航闭环帮助有限，成本高。

执行方案：
- 先保持 `semantic_map.yaml`：墙线、房间、禁行区、POI、路径偏好。
- 后续再升级为楼层/门/走廊中心线/拓扑图。

## 暂不建议

### 13. 把 PyQt GUI 做最终产品 UI

原因：Orin 无头部署、远程访问、系统服务化都更适合 Web UI。PyQt 可作为开发调试工具，但不是最终产品主 UI。

### 14. 用自研 SLAM/Nav 替代 slam_toolbox/Nav2

原因：产品路线优先稳定、可调试、可维护。自研算法可做对比实验或辅助指标，不应放进第一版主链路。

### 15. 浏览器直接参与避障或速度控制

原因：Web UI 延迟不可控。安全闭环必须在 Orin 本地 ROS2 节点内完成。
