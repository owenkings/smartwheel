# 阶段 1 — 手动行走建图

> 状态：spec。前置：阶段 0 通过（传感器可视化、TF 正确）。后续：阶段 2（自主建图）。

## 1. 目标
用户在电脑端（RViz Teleop 面板或键盘）控制轮椅**低速手动行走**，RTAB-Map 边走边构建 3D 点云地图和 2D 投影地图，RViz 实时显示，走完一圈后能保存地图。

不让系统自己决定往哪走（那是阶段2）。

## 2. 范围
**做：** 阶段0 全部 + safety_supervisor + 底盘（可选电机使能）+ RTAB-Map 建图 + 手动遥控（RViz 面板/键盘）+ 地图保存。
**不做：** 自主探索、frontier、Nav2 自动导航、定位重定位、GUI/Web/语音。

## 3. 控制链路（安全不绕过）
```
RViz TeleopPanel / 键盘 → /cmd_vel_nav → safety_supervisor → /cmd_vel_safe → zlac8030 底盘
```
- 手动指令也必须经 safety_supervisor（障碍减速/停车、急停生效）。
- 电机默认 `motion_control_enabled:=false`（只读验证链路）；真正驱动显式置 true。

## 4. 建图数据流（沿用 RTAB-Map 主线）
```
/points_merged（左雷达单路 fallback）+ /odometry/filtered（EKF）
   → RTAB-Map → /rtabmap/cloud_map（3D 主地图）
              → /rtabmap/grid_map（2D 投影）
```
- 相机默认**不**进入 RTAB-Map 几何同步链路（USB 时间戳抖动），仅作显示。

## 5. 需要的节点（在阶段0 基础上新增）
- `safety_supervisor`（manned 安全 profile）
- `sensor_watchdog`（**单雷达诊断配置**，右雷达不列 critical，否则触发 fail-closed 停车）
- `emergency_stop_node`（软急停 `/emergency_stop_sw`）
- `zlac8030_driver`（`motion_control_enabled` 控制是否真写电机；EKF 拥有 `odom→base_link`，底盘 `publish_tf:=false`）
- RTAB-Map 建图节点（`subscribe_rgb:=false`，`odom_mode:=external` 用 `/odometry/filtered`）
- RViz Teleop 面板（已实现的 `wheelchair_bringup/TeleopPanel`，发布 `/cmd_vel_nav`）

## 6. 入口与文件
| 类型 | 文件 | 备注 |
|---|---|---|
| 手动遥控+看传感器 | `src/wheelchair_bringup/launch/manual_teleop.launch.py` | 已实现，含 safety+base+RViz 面板，无建图 |
| 手动建图 | `manual_mapping_left.launch.py` | **待建**：manual_teleop + RTAB-Map |
| 建图 RViz 配置 | 复用/扩展 `sensor_view.rviz` 加 `/rtabmap/cloud_map`、`/rtabmap/grid_map` | 待定 |
| Teleop 面板插件 | `teleop_panel.hpp/.cpp` + `rviz_panel_plugin.xml` | 已实现并编译通过 |
| 启动脚本 | `scripts/run_rviz_manual_mapping_left.sh` | 待建 |
| 保存脚本 | `scripts/save_mapping_result.sh`（PCD/PLY/db） | 待建 |
| 验证脚本 | `scripts/check_rviz_mapping_left.sh` | 待建 |

## 7. 运行方式（计划）
```bash
# 只读验证链路（不动电机）
DISPLAY=:1 ros2 launch wheelchair_bringup manual_mapping_left.launch.py motion_control_enabled:=false
# 确认无误、离地/清场后，真正驱动建图
DISPLAY=:1 ros2 launch wheelchair_bringup manual_mapping_left.launch.py motion_control_enabled:=true
# 保存地图
bash scripts/save_mapping_result.sh <map_name>
```

## 8. 验收标准
1. RViz Teleop 面板/键盘可控制轮椅前进/后退/左转/右转/停（电机使能时真动）。
2. 指令确实经过 safety：周围无障碍时前进指令到达 `/cmd_vel_safe`；近距离障碍时被减速/零速。
3. `system_stop_required=false`、`/safety_state` 不是 `SENSOR_FAULT`（右雷达不再卡停车）。
4. 行走时 RViz 实时显示 `/rtabmap/cloud_map`（3D）和 `/rtabmap/grid_map`（2D）增长。
5. 走完实验室一圈后能保存 `rtabmap.db` + PCD/PLY，文件非空。
6. 急停（空格/软急停）能立即零速。

## 9. 已知坑
- 右雷达若被 watchdog 当 critical → 全局停车，手动指令也被零速。**必须用单雷达诊断配置。**
- 近距离障碍（超声波/scan < 安全阈值）会正常阻止前进，这是避障不是 bug；测试需清场 >0.5m。
- 多套 launch 同时跑会抢雷达/相机/话题，启动前先清残留进程。
- EKF 与底盘不能同时发 `odom→base_link`：底盘 `publish_tf:=false`，EKF 唯一拥有。

## 10. 完成产物
- `manual_mapping_left.launch.py` + 保存/验证脚本
- 一份实验室手动建图结果（db + PCD/PLY）
- 验证记录，满足进入阶段2 的前置（建图链路稳定、手动可控、地图可保存）
