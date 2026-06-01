# 自动探索 3D 建图 + RViz 可视化（autonomous_rviz_mapping）

## 1. 这是什么
类似扫地机器人的**自主探索建图**：轮椅低速自己走、自动选未探索区域、边走边建 3D 地图，RViz 实时显示。

分工：
- **RTAB-Map** 建图：`/rtabmap/cloud_map`（3D 点云主地图）、`/rtabmap/grid_map`（2D 导航投影）。
- **Nav2** 规划+控制（低速专用参数 `nav2_autonomous_mapping_params.yaml`）。
- **safety_supervisor** 安全裁决：`/cmd_vel_nav` → `/cmd_vel_safe`（绝不绕过）。
- **frontier_explorer_node** 选未知区域边界，经 Nav2 `NavigateToPose` 发目标（自己不发 `/cmd_vel`）。
- **RViz** 只负责可视化。

> TF 所有权：本模式由 **RTAB-Map ICP（icp_odometry）作为唯一 `odom->base_link` TF 发布者**；底盘节点的 TF 被禁用（`base.launch.py publish_tf:=false`），仅发布 `/wheel/odom`、`/base/status`，避免 TF 双发布/抖动。（`full_system` 普通导航模式仍由底盘发布 TF，不受影响。）

> 安全：默认不动。**只有同时 `enable_motion:=true` 且 `autonomous_exploration:=true` 才会自动行驶。**

## 2. 怎么运行
```bash
bash scripts/run_autonomous_rviz_mapping.sh   # 需输入 I_UNDERSTAND_AUTONOMOUS_MAPPING_RISK
```
或手动（可控开关）：
```bash
ros2 launch wheelchair_3d_mapping autonomous_rviz_mapping.launch.py \
  enable_motion:=true autonomous_exploration:=true rviz:=true
```
只看可视化、不让它动：`enable_motion:=false`（会打印提示，底盘不写电机）。

状态自检：`bash scripts/check_autonomous_mapping_status.sh` → 输出 `READY_TO_EXPLORE` 或 `BLOCKED` + 原因。

## 3. RViz 里应看到
`/points_merged`(融合点云)、`/rtabmap/cloud_map`(3D 主图，按高度上色)、`/rtabmap/grid_map`(2D 投影)、`/rtabmap/odom`(位姿)、`/scan`(障碍输入)、`/exploration/frontiers`(绿球=边界)、`/exploration/selected_goal`(绿箭头)、`/goal_pose`(黄)、`/plan`(Nav2 路径)、可选 `/global_costmap/costmap` `/local_costmap/costmap`。默认 TopDownOrtho 俯视；切 3D 用 Views 面板选 `Orbit3D`（或鼠标拖动）。

## 4. 成功标准
轮椅低速移动；frontier 绿球出现；selected goal 周期更新；`/rtabmap/cloud_map` 逐渐变大；`/rtabmap/grid_map` 逐渐扩展；`/plan` 有路径；`/safety_state` 多为 CLEAR/WARNING；`/cmd_vel_nav` 与 `/cmd_vel_safe` 有合理低速；软急停立即生效。

## 5. 失败排查
| 现象 | 原因 |
|---|---|
| 无 `/points_merged` | 雷达/融合没起，查 `/xtm60/left|right/points` |
| 无 `/rtabmap/cloud_map` | RTAB-Map 没起或未移动 |
| 无 `/rtabmap/grid_map` | 投影没出，慢推一段再看 |
| 无 map->odom | RTAB-Map 未跟踪（icp_odometry）|
| 无 odom->base_link | 底盘驱动/odom 没起 |
| `/navigate_to_pose` 不在线 | Nav2 未 active，看 lifecycle |
| frontier 为空 | 已探索完 / `min_frontier_size` 太大 |
| `/cmd_vel_nav` 有但 `/cmd_vel_safe`=0 | 安全层拦停：看 `/safety_state`（passability/scan/估计急停）|
| `/cmd_vel_safe` 有但底盘不动 | `enable_motion` 未开 / 电机寄存器 / 抱闸 |
| `/safety_state`=EMERGENCY | 急停被触发，复位后再试 |

## 6. 风险
首次必须**离地**测；其次空旷区低速；**不得载人**；必须有**物理急停**与看护人；不得在狭窄、有人、坡道、玻璃门、楼梯附近测试。

## 7. 保存地图
```bash
bash scripts/save_rtabmap_3d_map.sh                 # 从数据库导出 3D 点云 PLY（--scan）
bash scripts/save_rtabmap_3d_map.sh --live-pcd      # 直接录实时点云话题
```

## 8. 当前限制
- 不是 Matterport/NeRF/3DGS 那种照片级房间；输出是激光彩色/强度点云。
- 摄像头**默认不参与几何建图**（`subscribe_rgb:=false`），仅 `use_colorizer:=true` 时上色。
- 彩色/纹理质量需相机内参 + 相机-雷达外参 + 时间同步标定。
- 自动探索质量依赖 `/rtabmap/grid_map`；XT-M60 是 120° FOV，需慢速转向扫全。
- **Nav2 速度上限写在 `nav2_autonomous_mapping_params.yaml`**（launch 的 `max_*_speed` 为记录值）。
- 全局代价地图静态层订阅 `/rtabmap/grid_map`，其 QoS（transient_local）需在 Orin 上实测确认；如代价地图不更新，调 `static_layer.map_subscribe_transient_local`。
- 底盘真实运动、探索闭环必须在 AGX Orin 实机上离地→空旷逐步验证。
