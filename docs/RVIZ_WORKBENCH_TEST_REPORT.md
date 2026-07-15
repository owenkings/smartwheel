# SmartWheel RViz2 建图操作台测试报告

## 1. 测试范围

- 日期：2026-07-15
- 平台：NVIDIA Jetson AGX Orin 64GB、Ubuntu 22.04、ROS 2 Humble
- 分支：`feature/mapping-v2-rviz-workbench`
- 模式：`mock`，`hardware_enabled=false`
- 使用设备：Jetson 显示器、键盘和鼠标。
- 未使用设备：两台 XT-M60、H30 IMU、编码器、电机、四台真实相机、FD07-34R、串口、CAN、RS485。

## 2. 构建与自动测试

执行了要求的干净构建：

```bash
rm -rf build install log
source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

结果：30 个包构建成功，用时约 4 分 33 秒。`fast_lio` 有编译 warning，RViz 插件有 GCC ABI note，均不是构建失败。

执行了完整命令：

```bash
colcon test
colcon test-result --verbose
```

完整结果保留了未修改的第三方 `src/third_party/livox_ros_driver2` 上游 lint 失败：1 error、730 failures、73 skipped。该例外已在阶段 A-R 报告中登记；没有禁用测试，也没有修改第三方代码制造全绿。所有项目自有结果 XML 均为零 failure/error。

最终提交前再次执行项目自有测试：

```bash
colcon test --packages-skip fast_lio livox_ros_driver2 \
  --event-handlers console_cohesion+
```

结果：28 个包完成，225 tests，0 errors，0 failures，0 skipped。其中：

- `smartwheel_rviz_plugins`：23/23；
- `smartwheel_workbench`：17/17；
- `smartwheel_map_products`：11/11；
- `smartwheel_global_mapping`：7/7；
- `smartwheel_bringup`：5/5；
- `smartwheel_mapping_manager`：4/4。

覆盖内容包括 OccupancyGrid 旋转 origin/颜色/投影、Camera 配置/FPS/离线、Teleop 多键/STOP/超时/失焦/释放、安全限幅、插件发现、状态机、导出清单、ray casting 和有界累积。

## 3. 运行验收

| # | 验收项 | 结果 | 证据 |
| --- | --- | --- | --- |
| 1 | 中央 3D 持续更新 | PASS | 运行截图、视频、`/map_cloud` 数据 |
| 2 | 中央视图不能关闭 | PASS | 使用 RViz 原生 RenderPanel，不是 Panel 插件 |
| 3 | 2D 地图持续更新 | PASS | `/map` 约 3.91 Hz，实际截图 |
| 4 | 2D 缩放和平移 | PASS | 实际交互视频 |
| 5 | 2D Panel 可拖动 | PASS | 实际交互视频 |
| 6 | 2D Panel 可浮动 | PASS | floating 截图和视频 |
| 7 | 2D Panel 可关闭 | PASS | 实际交互视频 |
| 8 | Panels 菜单重新添加 2D | PASS | 实际交互视频 |
| 9 | 四路不同模拟图像 | PASS | 四个图像内容 hash 均不同 |
| 10 | Camera 可移动、关闭、添加 | PASS | 四相机截图和视频 |
| 11 | 四 Camera 标签页 | PASS | 默认布局和实际视频 |
| 12 | 四 Camera 可布置 2x2 | PASS | 实际四相机布局截图 |
| 13 | WSAD 键盘控制 mock base | PASS | 视频及 `/cmd_vel_safe` |
| 14 | 鼠标控制 mock base | PASS | 视频及 `/cmd_vel_safe` |
| 15 | 键盘松开停车 | PASS | 实测和 TeleopModel 测试 |
| 16 | 鼠标松开停车 | PASS | 实测和 TeleopModel 测试 |
| 17 | Space 立即停车 | PASS | 实测和 STOP 优先级测试 |
| 18 | RViz 失焦停车 | PASS | 实测和 focus-lost 测试 |
| 19 | Teleop Panel 关闭停车 | PASS | 实测析构/隐藏路径 |
| 20 | command timeout 停车 | PASS | 实测和超时测试 |
| 21 | Mapping Control 调用服务 | PASS | 最终 READY 导出，不是 UI 假状态 |
| 22 | System Status 权威诊断 | PASS | `/diagnostics`、`/mapping/status`、`/hardware/status` |
| 23 | RViz 配置保存/恢复 | PASS | 保存后重启恢复实例 topic 和布局 |
| 24 | 地图重载显示 3D/2D | PASS | map preview 实际启动和截图 |
| 25 | 退出无残留进程 | PASS | 关闭修复后 14 个子进程全部 clean exit |

四路模拟图像 SHA-256 截断值分别为：front `1ec341244bc3eb49`、left `2a012aeb7e7715c7`、right `51ac52b0ef84ee1a`、rear `fb8eba4e08b6baf8`。

## 4. 最终保存和重载

最终通过质量门的地图目录：

`maps/versions/rviz_workbench_final_verified_20260715_183757_216366`

关键结果：

| 指标 | 数值 |
| --- | ---: |
| 地图点数 | 58,738 |
| 已着色点数 | 55,950 |
| 轨迹姿态数 | 3,001 |
| 起终点平移误差 | 0.3522 m |
| 起终点偏航误差 | 0.0570 rad |
| 轨迹 RMSE（仅模拟评价） | 0.1750 m |
| 轨迹 yaw RMSE（仅模拟评价） | 0.0312 rad |
| occupied/free/unknown | 4,411 / 35,642 / 40,223 |
| 错配姿态 | 0 |

`hardware_validated` 明确为 `false`。Ground truth 只用于离线质量评价，不是 RTAB-Map、里程计或地图生成输入。

对应 rosbag：

`experiments/2026-07-15/rviz_workbench_final_verified_20260715_183817/rosbag`

- zstd，52.2 MiB；
- 87.811 s；
- 24,401 条消息；
- 四路 image topic 各 225 帧；
- 包含双雷达、IMU、轮速、LIO、TF、状态、teleop 和 safe cmd。

地图预览实际启动后，`/map` 和 `/map_cloud` 均有可靠 transient-local 发布者和 RViz 订阅者，三维彩色地图、二维栅格和轨迹同时可见。

## 5. 十分钟与三十分钟运行

当前最终代码进行了 30 分钟持续活动建图，覆盖了“至少运行 10 分钟”的要求。采样证据：`docs/rviz_workbench_stability_30m.tsv`。

- 61 组采样，最后采样时间 1,827 s；实际会话状态在约 1,876 s 时仍为 MAPPING。
- 最终轨迹约 37.16 m，378 个关键帧，255 个回环事件，约 173,055 个地图点。
- 所有受监控进程的文件描述符在 30 分钟内固定：workbench 19、RViz 50、sim 19、RTAB-Map 20、map products 19、mapping manager 19、safety 19。
- sim、mapping manager 和 safety RSS 基本稳定。
- map products 在最后 304 s 增长 5.35 MiB；workbench 增长 9.21 MiB。两者均使用有界缓存/累积器。
- RTAB-Map RSS 从 271 MiB 增至 626 MiB，随持续增加的关键帧、图和数据库工作集增长；最终数据库约 219 MiB。此项属于后端地图规模成本，不作为“固定内存”结果，已列入已知限制。

首次较早的 10 分钟验收曾暴露导出超时，保存为失败证据，并据此修复导出复杂度、同步、bag 关联和完成通知。最终代码上的 30 分钟活动运行未出现累积器 overflow 或设备访问。

## 6. 关闭验证

稳定性运行退出时发现 mapping manager 的 ROS context 关闭竞态，修复后重新启动完整操作台并 SIGINT：

- 14 个子进程全部 `process has finished cleanly`；
- 无 `process has died`；
- 无 traceback；
- 无 `exception was never retrieved`；
- 无残留 workbench、RTAB-Map 或 RViz 进程。

## 7. 截图和视频

- `docs/images/rviz_workbench_default.png`
- `docs/images/rviz_workbench_3d_mapping.png`
- `docs/images/rviz_workbench_2d_map.png`
- `docs/images/rviz_workbench_four_cameras.png`
- `docs/images/rviz_workbench_teleop.png`
- `docs/images/rviz_workbench_panels_floating.png`
- `docs/images/rviz_workbench_map_preview.png`
- `docs/images/rviz_workbench_demo.webm`

所有截图均为实际 RViz2 运行画面，分辨率 1920x1080。视频为 VP8/WebM、1920x1080、2 FPS、时长 5 分 03.985 秒，包含拖动、浮动、关闭、重新添加、WSAD、建图和预览操作。

## 8. 门禁结论

RVIZ WORKBENCH PROCEED

此结论只批准结束本 RViz mock 操作台任务。它不批准连接全部硬件、启动电机、自动导航或载人测试；下一步仍必须等待用户明确批准，并从 B0 资料审计开始。
