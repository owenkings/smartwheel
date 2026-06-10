# SmartWheel — RViz-first 分阶段路线图

围绕 **RViz** 的极简主线，一个阶段一个阶段实现，每阶段完成即验证，验证通过才进下一阶段。
不在早期堆 GUI/Web、语音、3DGS、frontier 自主探索、双雷达完整融合、用户 POI 导航。

- 分支：`feature/rviz-first-mapping-mvp`
- 恢复来源：所有历史代码在 `.git` 中（旧分支 `feature/livo-wheel-3d-slam`），可随时 checkout 复用。

## 硬件基线（2026-06 实测）
| 设备 | 地址/接口 | 状态 |
|---|---|---|
| 左 XT-M60 | `192.168.0.101` | 正常 ~10Hz |
| 右 XT-M60 | `192.168.1.101` | **硬件损坏**，本主线默认不使用 |
| H30 IMU | 串口 | 正常 ~200Hz |
| 超声波 ×4 | RS485 | range_0..3，正常 |
| 左相机 | `/dev/video0` | 正常 |
| 右相机 | `/dev/video2` | 设备不存在，禁用 |
| 底盘 | ZLAC8030/KeepLINK | 可用，默认 `motion_control_enabled:=false` |
| 远程桌面 | NoMachine, `DISPLAY=:1` | RViz 须渲染到此 display |

## 核心原则
1. **单左雷达为默认**，不是降级开关。诊断/scan/rviz 全用 left-only。
2. 一切运动必经 `safety_supervisor`：`/cmd_vel_nav → /cmd_vel_safe → base`。
3. 每阶段有独立、解耦的 launch 入口 + 验证脚本，不复用旧的大杂烩 launch。
4. 电机默认只读；真正驱动需显式 `motion_control_enabled:=true` + 离地/清场 + 物理急停。
5. 每阶段先写 spec（本目录 `specs/`），再实现，再验证。

## 阶段总览
| 阶段 | 名称 | 目标 | 状态 |
|---|---|---|---|
| 0 | RViz 传感器可视化 | 在 RViz 看到点云/scan/IMU/超声波/相机/TF，不动电机 | spec: `specs/stage0_sensor_visualization.md` |
| 1 | 手动行走建图 | 用户按键/面板控制低速行走，RTAB-Map 边走边建图，保存地图 | spec: `specs/stage1_manual_mapping.md` |
| 2 | 自主建图 (reactive) | 轮椅自主低速探索建图（先 reactive，不做 frontier） | spec: `specs/stage2_autonomous_mapping.md` |
| 3 | 定位导航 + 上层 | 已有地图重定位、POI 导航、GUI/语音/用户地图 | spec: `specs/stage3_navigation_and_apps.md` |

## 复用的底座资产（不重写，从 git 历史 checkout）
XT-M60 左雷达驱动、H30 IMU、超声波、相机驱动、URDF/TF、robot_state_publisher、
wheel odom / base driver、robot_localization EKF、RTAB-Map、pointcloud_to_laserscan、
scan_merger、safety_supervisor、地图保存脚本、已验证的 `sensor_view.rviz` 和 RViz Teleop 面板插件。

## 暂缓（归档在 git，不在早期主线）
GUI/Web、语音大模型、frontier exploration、用户 POI 导航、语义地图/keepout、
双雷达完整融合、`autonomous_rviz_mapping.launch.py` 大杂烩线、多 hardware_profile。
