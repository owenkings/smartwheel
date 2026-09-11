# 右雷达：手推 / WASD 共用建图会话

日期：2026-09-11。仅修改 `/home/nvidia/smartwheel`，基准 HEAD `103b569`。未操作另一人的 `smartwheel-rtab-old`，未提交或暂存既有改动。

## 使用范围

`scripts/run_wheel_imu_mapping.sh` 使用 **右 XT-M60**：`/xtm60/right/points`，设备 `192.168.1.101`。左雷达不在此入口启动。两种移动方式都沿用轮速 vx＋H30 yaw rate 的平面位姿、RTAB-Map 三维 XYZI 地图与几何回环；FAST-LIO 默认关闭。模式切换不重建地图数据库，不重置 EKF 或里程计。

## 使用方法

先在旧建图窗口按 STOP、确认实际停车，再在原启动终端 Ctrl+C，等待保存退出。已运行的旧 Python 节点和 RViz 不会自动加载新功能。

在 NoMachine 的 Orin 图形终端执行：

```bash
cd /home/nvidia/smartwheel
source /opt/ros/humble/setup.bash
source install/setup.bash
CAMERAS=true ULTRASONIC=true MOTION=true FASTLIO_SHADOW=false \
  bash scripts/run_wheel_imu_mapping.sh
```

SmartWheel Teleop 面板增加 **手推建图**、**WASD 建图**两个按钮：

1. 默认为驾驶模式。等待底盘模式显示 `drive`，可按 WASD 或方向按钮驾驶。
2. 要手推：先 STOP、确认实际停稳，再点“手推建图”。等待控制器确认且模式显示 `push`，再手推；WASD 方向命令在该模式被忽略，轮速与 IMU 继续读取。
3. 要恢复电动：停止推动、确认停稳，点“WASD 建图”。成功后松开旧按键，再重新按方向键。切回动作本身不会使能或发送非零目标速度。
4. 模式为 `*_no_feedback` 表示缺少有效轮速；不能当作位移记录正常。切换失败或超时不表示已释放车轮；界面保持运动命令关闭，需停车后确认并重新选择模式。
5. 正常结束仍为 STOP → 确认实际停车 → 启动终端 Ctrl+C → 等待自动保存。自动录包额外记录 `/base/mapping_drive_mode` 和 `/base/status`，便于核对模式与反馈。

`MOTION=false` 仍不允许该服务写电机控制器，不能把它当成解除制动命令。普通旧入口和共享底盘配置默认不启用手推切换。

## 代码与约束

- `wheelchair_base/zlac8030_driver_node.py` 新增可选 `allow_manual_push_mode`；仅 wheel-primary launch 开启。`/base/set_mapping_push_mode` 是 SetBool 服务，true=手推，false=驾驶；`/base/mapping_drive_mode` 为模式/轮速健康心跳。
- 切换要求原运动命令为零、最近反馈不超过 0.25 秒，且左右实测转速均在 ±0.5 rpm 内保持至少 0.30 秒。这是软件切换阈值，不是保证轮椅物理静止的认证。
- 进入手推先清零左右目标，再发送当前已配置的 stop 控制字 7；不经过 drive-enable 初始化。控制器不确认时进入 blocked，尝试既有 emergency-stop 写入，并明确返回失败，不伪装成功。
- 手推/blocked 时，常规速度写入被阻止，但 tick 继续读取真实轮速、发布健康和实测里程计；仍保留反馈失败不发布伪造零速度里程计的逻辑。
- 切回驾驶清空旧命令，并要求切换之后收到零命令，再接受新的非零命令。原 WASD → safety_supervisor → `/cmd_vel_safe` 路径不变，未新增绕过安全监督的速度发布。
- RViz 等待服务回复；切换中、手推、缺轮速、模式心跳超时等状态禁止方向输入。STOP 立即发布零命令，应用失活、面板关闭/隐藏也清空方向。新模式心跳限时 1 秒，为兼容既有电机初始化的两次 200 ms 等待；没有延长现有底盘速度超时或安全监督阈值。
- 保留 `release_motion_after_zero_sec=-1.0`，没有做“松键后自动释放”。需要手推时明确点击模式按钮。

## 验证与未验证事项

测试使用假 Modbus 控制器，无真实串口访问。ROS 服务联调在 localhost/domain 93 中运行，验证了服务请求、反馈定时器、手推非零轮速仍生成位移、手推时命令不写入、移动中拒绝恢复驾驶、旧方向不直接执行、新零命令后新方向才在模拟控制器中使能。Qt 离屏测试覆盖模式/心跳/服务缺失阻止方向、STOP、失活及控件渲染。构建和最终测试输出保存在远端 `auto_test/push_mode_20260911/`。

最终结果：base＋bringup 构建安装通过；`pytest src/wheelchair_base/test src/wheelchair_3d_mapping/test -q` 为 **225 passed in 37.68s**；`ctest --test-dir build/wheelchair_bringup -R teleop_mapping_mode --output-on-failure` 为 **1/1 测试程序通过**。离屏截图 `build/wheelchair_bringup/push_mode_panel.png` 已查看，两个按钮及状态文字可见；这不是现场实车切换截图。相关代码的 `git diff --check` 通过。

**控制器确认 stop 不等于机械自由推动已验证。** 本轮没有远程驱动车轮、释放真实控制器或替用户按切换按钮。仍需现场确认该控制字是否允许手推，以及手推时驱动轮是否带动编码器并有非零反馈。若显示 push 但仍推不动，不要硬推；若机械离合使编码器脱离车轮，当前轮速主位姿无法记录真实距离，不能用 IMU 或命令速度伪造补偿。

轮径比例、滑移、IMU 轴向、雷达外参、实际地图精度和成功回环仍需验证。此功能不把平面位姿升级为六自由度坡道跟踪，也不构成导航或载人安全验收。
