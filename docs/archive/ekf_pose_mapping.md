# 单雷达纯 EKF 位姿建图说明

> 适用范围:SmartWheel(ROS 2 Humble,工作区 `/home/nvidia/smartwheel`)3D 建图链路。
> 关联 spec:`.kiro/specs/ekf-pose-mapping/`(requirements / design / tasks)。
> 本文档对应 spec 需求 4.3、4.4(分阶段)与 6.1、6.4(审查修复严重度标注)。

## 1. 建图模式概述

本链路采用 **「纯 EKF 位姿、关闭 ICP」(pure EKF pose, ICP disabled)** 模式:

- **位姿唯一来源**:`robot_localization` EKF 输出的 `/odometry/filtered`(H30 IMU 主导航向 + 轮式里程提供平移),
  EKF 拥有 `odom → base_link` TF。
- **关闭所有几何配准**:RTAB-Map 工作于「外部里程计」模式,逐字采信 EKF 位姿,不做 ICP/邻接精修/空间回环修正。
- **理由**:XT-M60 是 120°×60° 窄视场 flash ToF 雷达。窄视场 ICP 面对平墙时沿墙方向无约束,会滑移并旋转 yaw,
  把本应平行的墙线拧成三角形(Triangle_Distortion)。ICP 在本传感器上是负作用,必须关闭。

数据流:左雷达点云 → 地平面自动标定(得 z/pitch)→ 动态 TF → 融合节点变换到 `base_link` 并裁剪/降采样 →
`/points_merged` → RTAB-Map(按 EKF 位姿直接堆叠)→ `/rtabmap/cloud_map`(3D)与 `/rtabmap/grid_map`(2D 投影)。

## 2. 分阶段约束(req 4.3、4.4)

| 阶段 | 内容 | 状态 |
|------|------|------|
| **阶段 1(当前 · 必须成功)** | 单 **左** 雷达(IP `192.168.0.101`)纯 EKF 位姿建图 + 自动地平面标定 + 相关审查修复 | 本特性范围 |
| **阶段 2(后续 · 范围外)** | 双雷达:引入 **右** 雷达(IP `192.168.1.101`) | 推迟 |

阶段 2 推迟的审查条目(本特性**不实现**):

- **D035(中)**:左右两路点云无时间对齐,运动时被当作同时刻叠加(墙体加宽/双影)。属右雷达接入后的左右同步问题。
- **D133(中)**:右雷达 joint origin `xyz=[0.45,-0.24,0.65]` 为 placeholder,未标定。
- **D165(中)**:右雷达相对外参缺失(`calib/` 只有相机内参,无左右雷达相对外参)。

阶段 1 不要求右雷达在线:融合节点 `allow_single_lidar_fallback=true`、`enable_xtm60_right=false`,
仅左雷达新鲜即持续发布 `/points_merged`(req 3.3、4.1、4.2)。

启动路径:`ros2 launch wheelchair_bringup manual_mapping_left.launch.py`(始终传
`odom_mode=external` + `odom_topic=/odometry/filtered`)。

## 3. 已实施的审查修复(req 6.1、6.4)

严重度沿用审查报告 `docs/project_code_audit.md` 的分级:**阻断 / 高 / 中 / 低**。

| D 编号 | 严重度 | 文件 / 位置 | 修复内容 |
|--------|--------|-------------|----------|
| **D034 / D168** | **高**(ICP 关闭后降级验证) | `dual_lidar_cloud_fusion_node.py` · `_publish_merged` | `/points_merged` 的 `header.stamp` 改用参与合并的**源帧采集时刻**(`state.stamp`,左/右取较新者,单雷达即左帧),而非发布时刻墙钟 `get_clock().now()`;仅当源 stamp 全 0(sec 与 nanosec 均 0)时回退墙钟。修复点云与 IMU/odom 的对齐时刻。 |
| **D178** | **中** | `dual_lidar_cloud_fusion_node.py` · `_lookup` / `_on_cloud` | 融合 TF 改用点云 `msg.header.stamp` **对应时刻**查询(`lookup_transform(target, source, stamp, timeout)`),而非 `rclpy.time.Time()` 最新可用变换;TF 不可用(`ExtrapolationException` 等)时**跳过该帧**(返回 None),不退回最新 TF 强变换历史点云。 |
| **D179** | **低** | `cloud_utils.py` · `filter_by_range` | 显式剔除无效/占位点:在范数计算后加 `r > 0` 与全分量 `np.isfinite` 校验,剔除 `(0,0,0)`/NaN 点,不再依赖 `min_range` 过滤的副作用;`min_range` 语义保持不变。 |
| **D041** | **(参数双真值源)** | `rtabmap_params.yaml` ↔ `rtabmap_3d_mapping.launch.py` · `essential` dict | 关闭 ICP 时同步修改 yaml 与 launch `essential` dict 两处真值源,并在 yaml 顶部注释标注「**essential dict 为准**」,消除双真值源隐患。 |

每项修复在对应代码处以 `# audit Dxxx (severity)` 注释标注(req 3.5)。

## 4. 阶段 1 核心交付

### 4.1 关闭 ICP(任务 1)

在 `rtabmap_params.yaml` 与 launch `essential` dict 两处同步修改(`essential` dict 为实际生效源):

| 参数 | 原值 | 新值 | 原因 |
|------|------|------|------|
| `Reg/Strategy` | `1`(ICP) | `0` | 关闭配准,位姿不被 ICP 改写 |
| `RGBD/NeighborLinkRefining` | `true` | `false` | 关闭相邻关键帧 ICP 精修 |
| `RGBD/ProximityBySpace` | `true` | `false` | 关闭空间邻近回环(同样用 ICP 移动位姿) |
| `RGBD/ProximityPathMaxNeighbors` | `10` | `0` | 禁用邻近搜索 |

保留 `Reg/Force3DoF=true`(平面轮椅,锁 z/roll/pitch 防地图倾斜)、`Mem/IncrementalMemory=true`
(维持数据库 / cloud_map / grid_map,req 1.9)、`Grid/*` 投影参数(继续产 2D grid_map,req 1.5);
`Icp/*` 保留但 Strategy=0 后不生效,加注释「留作回退」。

### 4.2 地平面自动标定(需求 2)

`ground_plane_calibrator_node`(`wheelchair_3d_mapping` 包)从左雷达点云用纯 numpy RANSAC 拟合主导「向上法向」
地平面,自动得出雷达安装高度(z = 原点到地平面垂距)与俯仰(pitch = 平面法向偏离理想竖直的角度),
用 `StaticTransformBroadcaster` 发布完整 `base_link → xtm60_left_link`(唯一所有者,URDF 不再发布该边),
雷达被移动/重装时无需编辑 URDF/static_transforms 即可保持建图正确(req 2.7)。

失败兜底:地面不可见或内点率低于阈值时,保留上次有效 TF 并节流告警,绝不发布无效外参(req 2.5)。
诊断:把 height/pitch/inlier_ratio/valid 以 JSON 发布到 `/calibration/ground_plane`,供 auto_test 记录(req 2.6)。

#### Task-14 修复结果(右雷达过冲)

为修正右雷达标定值过冲(实测 0.50–0.60 m,曾报告 0.749 m),引入四项约束:

1. **垂直法向约束(vertical-normal constraint)**:在 RANSAC 循环内只接受近竖直法向的候选平面(排除墙面/倾斜面);
   近竖直容差由 25° 收紧到 **15°**。
2. **最低优先先验(farthest-down prior)**:在合格候选中选 `|d|` 最大(离传感器最远 / 最低)的平面作为地面,
   而非内点最多的平面(常是更近的座椅/桌面/平台);辅以内点率下限(`ground_min_inlier_frac`)防稀疏噪声面胜出。
3. **外推一致性门(extrapolation-consistency gate)**:候选仅当其 `|d|` 与其内点的实际竖直深度一致
   (`|abs_d - drop| ≤ ground_extrapolation_tol`)才能凭 `|d|` 胜出,拒绝因倾斜外推而虚高 `|d|` 的稀疏倾斜碎片。
4. **400 次 RANSAC 迭代**:稀疏单帧下 100 次易锁定坏值;400 次使单帧 `|d|` 中位数稳定在量程带内,使
   single-frame settle-lock 可靠。

结果:右雷达标定到约 **0.53 m**,落在 0.50–0.60 m 实测带内(req 2.2/2.3,Property 3)。

#### 关键可调参数

| 参数 | 默认值 | 含义 |
|------|--------|------|
| `vertical_normal_tol_deg` | `15.0` | 近竖直法向容差(度);越小越严格地排除倾斜面 |
| `ground_extrapolation_tol` | `0.10` | 外推一致性门容差(米);`|d|` 与内点实际竖直深度的允许偏差 |
| `ransac_iterations` | `400` | RANSAC 迭代次数;稀疏单帧需较大值才稳定 |
| `min_inlier_ratio` | `0.30` | 选定地平面在其竞争的近水平点子集中的最小内点占比;低于则判 invalid |
| `recalibrate_period_sec` | `0.0` | `0` = 仅启动用前 `settle_frames` 帧标定一次并锁定;`>0` = 按周期重估 |

(辅助参数:`prefer_farthest_ground`、`ground_min_inlier_frac`、`settle_frames`、`ground_min_drop`、
`ransac_distance_thresh`。)

## 5. 待办(已推迟,本轮文档不覆盖)

- **EKF 缺失兜底现场验证(req 1.8)**:验证 `/odometry/filtered` 中断 >0.5s 时不用过期位姿堆点。
  **仍待完成**(需在任务 15 之后做现场验证)。
- **本地 git 提交(req 6.2)**:所有改动仅本地提交、不推 GitHub。**仍待完成**(deferred)。
