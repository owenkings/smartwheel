# SmartWheel RViz2 建图操作台已知限制

## 1. 硬件边界

- 本操作台只在 `mode=mock`、`hardware_enabled=false` 下实现和验证。
- XT-M60、H30、编码器、真实相机、FD07-34R、驱动器和物理急停均未验证。
- System Status 中的 MOCK/DISABLED 不能替代真实设备 bringup、标定或安全验收。
- 完成本 GUI 不允许跳过 B0/B1，不允许连接双雷达、四相机或启动电机。

## 2. 建图后端

- 当前 RTAB-Map mock 基线使用外部 LIO odometry 和 ICP/空间邻近回环；在线视觉回环默认关闭，因此 RTAB-Map 会输出 missing visual features warning。这不是硬件相机能力结论。
- `slam_toolbox` 是可选二维后端，但本次主要实际截图、长时运行和最终产品验收使用 RTAB-Map。
- RTAB-Map 的 RSS 随持续增加的关键帧、图和数据库工作集增长。30 分钟 mock 运行从约 271 MiB 增至 626 MiB，最终数据库约 219 MiB。B1 以后必须按真实路线长度建立内存预算和关键帧策略。
- `rtabmap.db` 在 RTAB-Map 关闭时完成写入，因此 manifest 将其列为 externally managed，不对仍可能变化的数据库记录 SHA-256；其余 13 个导出文件均有 hash。

## 3. 图像和显示

- 最终运行验证了四路独立模拟图像；没有在 30 分钟正式运行中单独中断其中一路。CameraPanel 的实例隔离、离线阈值和有界最新帧由单元测试覆盖，但真实 USB 断流必须在 B4 单相机阶段重新验证。
- `/map` 或 `/map_cloud` 的单独断流未作为最终 30 分钟运行中的故障注入项；Panel 会显示 NO MAP DATA/STALE，RViz PointCloud2 会显示 topic/frame 错误。
- 实际截图和视频分辨率为 1920x1080。布局配置包含滚动区和可停靠 Panel，但 2560x1440 及更小物理显示器没有分别录制验收视频。
- 预览模式不会启动相机源，因此四个 CameraPanel 显示 OFFLINE 是正确状态。

## 4. 操作和安全

- Teleop 的 acceleration 参数在 Panel 状态中保存，但当前 mock safety supervisor 主要执行绝对速度限幅和 command timeout；真实驱动加减速度约束必须在 E1 安全链中实现并实测。
- GUI 失焦、隐藏和析构停车属于软件安全层，不能替代物理急停和驱动器 watchdog。
- System Status 不从 GUI 猜测设备正常；如果后端诊断不提供某项，Panel 会显示缺失/禁用，而不是伪造 ONLINE。

## 5. 测试工具链

- 完整 `colcon test` 仍报告未修改的第三方 `livox_ros_driver2` 上游 lint 失败。项目自有 28 包的 225 个测试全部通过。第三方例外不能被解释为真实 XT-M60 驱动已验证。
- 早期长运行保存了失败地图/实验目录，用于保留导出超时和质量门失败证据；Map Products 只列出 manifest 完整的版本。

## 6. 结果解释

- 最终质量报告中的 ground-truth RMSE 只代表 Stage A 合成模拟，不代表真实轮椅定位或建图精度。
- 当前地图的 `hardware_validated=false` 是强制事实，不得删除或改成 true。
- RVIZ WORKBENCH PROCEED 只表示 mock 操作台门禁通过，不表示 B0、B1、导航或载人安全门禁通过。
