# SmartWheel RViz2 建图操作台使用指南

## 1. 启动

```bash
cd /home/nvidia/smartwheel
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch smartwheel_bringup operator_workbench.launch.py \
  mode:=mock \
  hardware_enabled:=false
```

当前启动入口只接受 mock。不要将本指南中的命令改成 `mode:=real` 或 `hardware_enabled:=true`。

## 2. 默认布局

- 中央：RViz 原生三维 RenderPanel，显示实时双雷达、优化地图和轨迹。
- 左侧：Displays、Mapping Control、Teleop 和 Views。
- 右侧：SmartWheel 2D Map。
- 底部/右下：四路 Camera、System Status 和 Map Products，以标签页组织。

中央 RenderPanel 始终存在。自定义 Panel 可以从标题栏拖动、停靠、浮动、调整大小或关闭。

关闭 Panel 后，从 `Panels -> Add New Panel` 选择以下任一项重新打开：

- `SmartWheel/2D Occupancy Map`
- `SmartWheel/Camera`
- `SmartWheel/Teleop`
- `SmartWheel/Mapping Control`
- `SmartWheel/System Status`
- `SmartWheel/Map Products`

Camera 是通用插件。重新添加后在 Panel 内配置 image topic、camera_info topic，再通过 RViz 保存配置。使用 `File -> Save Config As` 保存当前布局；下次用 `rviz2 -d <config>` 加载。

## 3. 三维和二维查看

中央三维视图使用 RViz 原生鼠标操作：拖动旋转/平移，滚轮缩放。原始点云默认关闭，可在 Displays 的 `SmartWheel 3D Mapping` 分组中开启。

二维地图 Panel：

- 滚轮：缩放；
- 鼠标拖动：平移；
- 双击：恢复自适应；
- 状态栏：显示缩放比例、地图尺寸、分辨率和 MAP ONLINE/STALE/NO MAP DATA。

`NO MAP DATA` 或 `INVALID MAP` 是失败状态，不表示一张正常的空白地图。

## 4. WSAD 模拟遥控

先点击 Teleop Panel 使其获得键盘焦点。

- `W`：前进；
- `S`：后退；
- `A`：左转；
- `D`：右转；
- `W+A`、`W+D`、`S+A`、`S+D`：组合运动；
- `Space` 或 STOP：立即发布零速度；
- 松开键盘或鼠标按钮：立即停止；
- RViz 失焦、Panel 关闭/隐藏或命令超时：停止。

鼠标可按住 W/A/S/D 按钮控制，释放即停。Panel 发布链路固定为：

```text
/teleop/cmd_vel -> mock_safety_supervisor -> /cmd_vel_safe -> mock base
```

`hardware_enabled=false` 时只表示模拟底盘收到命令，不表示真实轮椅移动。

## 5. 建图流程

在 Mapping Control 中设置地图名称，按以下顺序操作：

1. `Preflight Check`：验证 mock 来源、TF、频率、时间戳、点云单位和磁盘空间。
2. 可选 `Start Recording`：创建 zstd rosbag 和实验目录。
3. `Start Mapping`：开始本次建图会话。
4. 使用 WSAD 观察实时三维、二维地图和四路相机。
5. 可用 `Pause Mapping` / `Resume Mapping` 暂停或恢复后端。
6. `Finish Mapping`：停止记录并请求 mapping manager 结束。
7. `Optimize`：确认后端已有可优化的关键帧/地图证据。
8. `Export Products` 或 `Save Map`：执行真实导出和质量门。
9. 状态变成 `READY` 且质量结果为 PASS 后，地图才算保存成功。

`FAILED` 时先阅读 FAILED 原因。不要把导出超时、质量失败或缺少服务视为成功。Cancel 和 Reset Session 会弹出确认对话框。

## 6. 地图浏览和预览

Map Products Panel 中：

1. 按 `Refresh` 获取完整地图版本；
2. 选择一个版本并按 `Select`；
3. `Republish 3D` 重新发布 `/map_cloud`；
4. `Republish 2D` 重新发布 `/map`；
5. `Preview` 同时发布三维、二维和轨迹；
6. `Open Directory` 打开产品目录。

也可以单独启动预览工作台：

```bash
ros2 launch smartwheel_bringup map_preview_workbench.launch.py \
  map_version:=/home/nvidia/smartwheel/maps/versions/rviz_workbench_final_verified_20260715_183757_216366
```

预览模式不会启动模拟传感器或建图后端。相机显示 OFFLINE 是预期行为。

## 7. 结束

在启动终端按 `Ctrl+C`。所有子进程应显示 `process has finished cleanly`。若仍有 ROS 进程，先保存日志并停止继续操作，不要直接进入真实硬件阶段。
