# Design Document

## Overview

本设计把 SmartWheel 单(左)雷达 3D 建图从「纯 EKF 位姿 + RTAB-Map 零配准」(产生漩涡)替换为
**FAST-LIO2(LiDAR-惯性里程计)前端**,实现真正的帧间紧耦合拼接;RTAB-Map 降级为**可选回环/导出后端**
(路线 2)。本阶段为**方案 B(纯 LiDAR + IMU,不接相机进估计器)**,但保留并完善 RViz 版式(含相机画面、
3D 上色地图、2D 栅格、可扩展 4 路相机面板)。

设计的核心工程事实(已核实,非假设):

- **平台**:aarch64 / Jetson Orin,JetPack 6(Tegra R36.4),ROS 2 **Humble**,gcc 11.4,cmake 3.22。
- **FAST-LIO2 仓库选型**:`Ericsii/FAST_LIO_ROS2`(`--recursive`,含 `ikd-Tree`)。理由:① 明确推荐
  ROS Humble;② FAST-LIO2 直接在原始点上做 scan-to-map,**支持 Velodyne/Ouster 标准 PointCloud2 与
  外部 IMU**,不限于 Livox 自定义消息;③ 明确支持 ARM 平台(TX2/Khadas/RPi),apt 默认 PCL/Eigen 即可。
  仓库 URL 与锁定 commit 在实现时记入文档(req 1.5)。
- **窄视场可行先例**:Livox Avia FoV ~70°,比 XT-M60(120°×45°,横向 120°、纵向 45°)纵向更窄,FAST-LIO2 工作良好。
- **XT-M60 点云**:`sensor_msgs/PointCloud2`,字段 x/y/z/intensity,**无逐点 `time`、无 `ring`**,
  organized cloud,~10 Hz,在 `xtm60_left_link` 传感器系。
- **IMU**:`/imu/data`,H30(YIS106),100 Hz。
- **既有外参**(`static_transforms.yaml` / URDF):`base_link→xtm60_left_link` 与 `base_link→imu_link`
  已定义(见 §4,外参从**实时 TF** 取值而非硬编码旧 yaml,以防陈旧)。

### 关键设计判断

1. **deskew 必须关闭**:FAST-LIO2 的运动去畸变依赖逐点 `time`。XT-M60 整帧快照无逐点时间。喂入时
   每点时间设 0、`point_filter_num` 合理,使前向/后向传播退化为「整帧单一时间戳」——这正是我们要的,
   不是缺陷。FAST-LIO 对缺失 time 字段会打印告警 `Failed to find match for field 'time'`,属预期。
2. **不引入 Livox 驱动依赖**:用 FAST-LIO2 的 Velodyne/Ouster(通用 XYZI)输入路径 + 一个轻量
   `Cloud_Adapter`,而非 livox_ros_driver2。避免为一个非 Livox 传感器背上 Livox 工具链。
3. **外参以实时 TF 为准**:`xtm60_left_link`↔`imu_link` 的相对位姿从运行时 TF 静态链解析,折算成
   FAST-LIO 要求的「LiDAR 在 IMU 体系下的位姿」(extrinsic_T/R),不手抄可能陈旧的 yaml 数字。
4. **单一 TF 所有权**:建图时 FAST-LIO2 作为 `odom→base_link` 的唯一发布者(TF_Owner=lio),EKF 与
   ZLAC 的 TF 发布关闭。EKF 仍可运行供其他消费者,但不发 `odom→base_link`。
5. **RTAB-Map 仅作可选后端**:默认不启动;启用时以 external-odom 消费 FAST-LIO2 位姿做回环/导出。

---

## Architecture

### 数据流(本阶段:方案 B)

```text
/xtm60/left/points (PointCloud2 XYZI, sensor frame, ~10Hz)
        │
        ▼
 Cloud_Adapter (lio_cloud_adapter_node)
   - 剔除 NaN / (0,0,0) 占位点 (复用 cloud_utils)
   - 规整为 FAST-LIO 通用 XYZI 输入 (每点 time=0; 无 ring 需求)
   - 统一 header.stamp 时间基准, frame_id=lidar_frame
        │  /lio/cloud_in
        ▼
/imu/data (100Hz) ──►  FAST-LIO2 (fast_lio)  ◄── extrinsic_T/R (lidar w.r.t. imu)
        │                    │  iEKF: IMU 主导姿态(yaw 锁定) + LiDAR scan-to-map 修正
        │                    │  ikd-Tree 增量地图; deskew 关闭
        │                    ├─► /Odometry        (LIO 位姿, body in odom/global)
        │                    ├─► /path
        │                    ├─► /cloud_registered (当前帧已配准点云)
        │                    ├─► /cloud_map 或等价累积地图 (可选/周期)
        │                    └─► TF: odom→base_link (TF_Owner=lio; 经 base_link↔imu 外参折算)
        │
        ├─────────────► cloud_to_occupancy_grid_node ─► /map_2d_from_3d (2D 栅格, 供 Nav2)
        │
        └─(可选, req10)─► RTAB_Map_Backend (external odom=/Odometry) ─► 回环/位姿图/导出 .db

相机(本阶段只显示, 不进估计器):
  /camera/{front,left,right,rear}/image_raw ──► RViz Image 面板 (Camera_Panels)

安全/控制(不变):
  RViz TeleopPanel ─► /cmd_vel_nav ─► safety_supervisor ─► /cmd_vel_safe ─► base (MOTION 默认关闭)
```

### TF 树

```text
map ─(可选: RTAB-Map 回环 / 否则 identity 或不发)→ odom ─(FAST-LIO2, TF_Owner=lio)→ base_link
  base_link → xtm60_left_link / imu_link / camera_*_link / ... (URDF 静态)
```

- FAST-LIO2 内部以 IMU 体系做估计,输出 `odom→base_link` 需用 `base_link↔imu_link` 外参折算
  (见 §4.3)。本阶段 `map→odom` 若不启用 RTAB-Map,可由一个 identity static TF 提供(让 RViz Fixed
  Frame=map 可用),或直接用 `odom` 作为 Fixed Frame;具体见 §6 RViz。

### 包与节点落点

| 组件 | 位置 | 说明 |
|---|---|---|
| FAST-LIO2 (`fast_lio`, `ikd-Tree`) | `src/third_party/FAST_LIO_ROS2/`(vendored, `--recursive`) | 外部算法本体,锁 commit |
| `lio_cloud_adapter_node` | `wheelchair_3d_mapping/` 新增 | XT-M60 → FAST-LIO 输入适配 |
| `fast_lio_mapping.launch.py` | `wheelchair_3d_mapping/launch/` 新增 | 启 adapter + fast_lio + 外参注入 |
| `xtm60_left_lio.yaml` | `wheelchair_3d_mapping/config/` 新增 | FAST-LIO 参数(lidar_type/extrinsic/deskew off) |
| `manual_mapping_lio_left.launch.py` | `wheelchair_bringup/launch/` 新增 | 顶层:teleop 基座 + LIO + RViz |
| `manual_mapping_lio_left.rviz` | `wheelchair_bringup/rviz/` 新增 | 新版式(req 8) |
| `run_rviz_manual_mapping_left.sh` | `scripts/` 改写 | 指向 LIO 链路(替换旧 RTAB-Map 入口) |
| 清理 | KISS-ICP / ground_plane / 旧 LIVO 包装 | req 9 |

---

## Components and Interfaces

### 4.1 Cloud_Adapter — `lio_cloud_adapter_node`

**职责**:把 `/xtm60/left/points` 转成 FAST-LIO2 能干净消费的 PointCloud2。

**输入**:`/xtm60/left/points`(`PointCloud2`,XYZI,sensor frame,QoS sensor_data/BEST_EFFORT)。
**输出**:`/lio/cloud_in`(`PointCloud2`,XYZI,frame_id 保持 `xtm60_left_link`)。

**处理**:
1. `cloud_utils.read_xyz_intensity`(已 NaN-skip)+ 显式剔除 (0,0,0)(复用既有 `filter_by_range` 的
   `r>0` 逻辑;req 2.5)。
2. 不新增 `ring`;**不依赖逐点 time**。若所选 FAST-LIO lidar_type 在缺 time 时报错而非告警,则在 adapter
   里补一个值为 0 的 `time`(float32)字段使其满足解析,但语义上整帧同一时刻(req 2.3)。
3. `header.stamp`:沿用源帧 stamp;若与 IMU 时间基准不一致导致 FAST-LIO 拒绝,提供参数
   `restamp_to_now`(默认 false)对齐(req 2.6)。
4. QoS:输出用 FAST-LIO 期望的 RELIABLE/默认深度;参数化便于调。

> 备选:若实测证明直接把 `/xtm60/left/points` remap 给 FAST-LIO(配 lidar_type=Velodyne、关 deskew)即可
> 工作,则 adapter 退化为「仅清理无效点」甚至不需要。Adapter 存在的价值是**显式控制无效点与时间字段**,
> 降低对 FAST-LIO 内部解析容错的依赖。设计保留 adapter 作为稳健默认。

### 4.2 FAST-LIO2 配置 — `xtm60_left_lio.yaml`

关键参数(具体键名以 `Ericsii/FAST_LIO_ROS2` 的 config 模板为准,实现时对齐):

| 参数 | 值 | 理由 |
|---|---|---|
| `lid_topic` | `/lio/cloud_in` | adapter 输出 |
| `imu_topic` | `/imu/data` | H30 100Hz |
| `lidar_type` | 通用 XYZI(Velodyne/Ouster 路径) | 非 Livox |
| `scan_line` | 按 XT-M60 行数(organized height) | 通用路径需要 |
| `timestamp_unit` / per-point time | 关闭/无 | **deskew off**(req 2.2) |
| `blind` | ~0.1 m | 近盲区(雷达近表面噪声) |
| `extrinsic_T` | LiDAR 在 IMU 系平移(§4.3 计算) | req 3.1 |
| `extrinsic_R` | LiDAR 在 IMU 系旋转矩阵(§4.3) | req 3.1 |
| `extrinsic_est_en` | false | 外参已知,不在线估计(README C 建议) |
| `filter_size_surf/map` | 室内调小(~0.2/0.3) | 室内尺度 |
| `pcd_save_en` | 可配 | 地图保存(req 11) |
| `publish.path/scan_publish` | on | RViz 轨迹/当前帧 |

### 4.3 外参折算(LiDAR w.r.t. IMU)

FAST-LIO 的 `extrinsic_T/R` 定义为 **LiDAR 在 IMU 体系下的位姿**(IMU 为 base)。
本设计**从运行时 TF 静态链解析** `imu_link → xtm60_left_link`(而非抄 yaml),记为 `T_il`:

```
T_il = inv(TF(base_link→imu_link)) · TF(base_link→xtm60_left_link)
extrinsic_T = T_il.translation
extrinsic_R = T_il.rotation (3x3)
```

实现时提供一个一次性小工具(launch 内或脚本)在 TF 就绪后读出 `T_il` 写入/校验 yaml,并在 auto_test
报告记录数值(req 3.1)。

> **风险点(诚实标注)**:`xtm60_left_link` 的朝向涉及传感器内部坐标约定(历史上 XT-M60 有过绕前向轴
> 180°/90° 的约定修正,见 ekf-pose-mapping 备注)。因此外参**不能只靠 yaml 数字**,必须用实时 TF +
> 一帧地面/已知物体在 LIO 输出中的方向核对(地面在 base_link 下 z≈0、墙竖直)。这是本特性最易错处,
> 设计为「TF 取值 → 录包离线核对 → 必要时修正约定」的闭环,而非一次写死。

### 4.4 TF 所有权切换

- 新增 launch 参数 `tf_owner`(本链路默认 `lio`)。
- `tf_owner=lio`:FAST-LIO2 发 `odom→base_link`;`manual_teleop.launch.py` 的 EKF 以 `publish_tf:=false`
  启动(EKF 仍算 `/odometry/filtered` 供诊断/一致性,但不发 TF);ZLAC `publish_tf:=false`。
- 复用既有 `bringup_3d_slam.launch.py` 的 `tf_owner` 单一所有权理念(已存在 ekf/wheel/livo 三态),
  扩展/接入 `lio`。

### 4.5 2D 投影(保留)

`cloud_to_occupancy_grid_node` 订阅 LIO 的累积/配准点云(`/cloud_registered` 或累积云),输出
`/map_2d_from_3d` 供 Nav2。参数沿用 `cloud_to_occupancy_grid.yaml`,仅改输入话题。

### 4.6 RTAB_Map_Backend(可选,req 10)

- 默认 **不启动**。
- `enable_loop_backend:=true` 时:以 external-odom(`odom_topic=/Odometry`,Reg/Strategy=0)消费 LIO 位姿,
  仅做回环检测/位姿图优化/数据库与导出,不做前端里程计。复用既有 `rtabmap_3d_mapping.launch.py`,
  改 odom 源为 `/Odometry`。
- 文档明确:**FAST-LIO2=前端实时位姿;RTAB-Map=可选后端回环/导出**,无双真值源(req 10.3)。

---

## RViz 版式设计(req 8) — `manual_mapping_lio_left.rviz`

对标用户两张参考截图(3D 上色轨迹图 + 左下 2D 栅格 + 右下 LaserScan 极坐标;及全屏上色点云)。

显示项(Displays):
- **Grid**、**TF**、**RobotModel**(保留)。
- **LIO 3D Cloud Map**:订阅 LIO 累积云(`/cloud_registered` 累积 或 `/cloud_map`),`Color Transformer=AxisColor(Z)` 或 `Intensity`,**实时帧 Decay Time=0**(不复现漩涡;req 8.1)。累积地图由 LIO 自身维护,不靠 RViz decay 堆叠。
- **2D Occupancy Grid**:`rviz_default_plugins/Map`,订阅 `/map_2d_from_3d`(req 8.2)。
- **LaserScan**:订阅 `/scan`(req 8.3,对标"激光雷达"面板)。
- **LIO Odometry / Path**:订阅 `/Odometry` + `/path`(req 8.4,行进轨迹)。
- **Camera_Panels**:4 个 `rviz_default_plugins/Image`,分别订阅
  `/camera/front|left|right|rear/image_raw`(req 8.5/8.6)。当前仅左/右有数据,front/rear 预置但留空
  (无数据不报错破版)。
- **TeleopPanel**:`wheelchair_bringup/TeleopPanel`(req 8.7)。
- **Battery Status**(若有对应话题/面板;req 8.9,数据缺失不阻塞)。

视图:Orbit(主)+ TopDownOrtho(保存视图,对标 2D 俯视)。Fixed Frame:`map`(RTAB-Map 关时由 identity
`map→odom` static TF 提供)或 `odom`。

> Window Geometry 预置成参考截图式布局:主 3D 视图居中,底部 2D 栅格 + LaserScan,侧栏相机 Image。
> 具体 dock 布局在实现时落 rviz 文件。

---

## Build & Dependencies(req 1)

```bash
# 依赖(apt 默认 PCL/Eigen 即可; Humble)
sudo apt update
sudo apt install -y libpcl-dev libeigen3-dev ros-humble-pcl-ros \
                    ros-humble-pcl-conversions
rosdep install --from-paths src --ignore-src -y   # 解析 FAST_LIO 依赖

# 获取 FAST-LIO2 (Humble fork, 含子模块)
cd src/third_party
git clone https://github.com/Ericsii/FAST_LIO_ROS2.git --recursive
# 记录并锁定 commit hash 写入文档 (req 1.5)

# 编译 (aarch64)
cd /home/nvidia/smartwheel
colcon build --symlink-install --packages-up-to fast_lio
colcon build --symlink-install --packages-select wheelchair_3d_mapping wheelchair_bringup
```

> 不安装 livox_ros_driver2(非 Livox 传感器)。若 FAST_LIO 的 CMake 硬依赖 livox 消息,采用
> Ericsii 提供的 standard-unit 分支或在 adapter 侧绕过,实现时确认并记录。

---

## Data Models

### LIO_Config(`xtm60_left_lio.yaml`)
- `lid_topic: string` = `/lio/cloud_in`
- `imu_topic: string` = `/imu/data`
- `lidar_type: enum` = 通用 XYZI(Velodyne/Ouster 路径)
- `scan_line: int` = XT-M60 organized height
- `blind: float` = 近盲区(m)
- `extrinsic_T: float[3]` = LiDAR 在 IMU 系平移(§4.3 解析)
- `extrinsic_R: float[3][3]` = LiDAR 在 IMU 系旋转矩阵
- `extrinsic_est_en: bool` = false
- `deskew/per-point-time: off`
- `filter_size_surf/map: float`
- `pcd_save_en: bool`

### LiDAR_IMU_Extrinsic(`T_il`,运行时从 TF 解析)
- `translation: float[3]`
- `rotation: float[3][3]`(由 `inv(base→imu)·(base→lidar)` 得到)

### Cloud_Adapter_IO
- 输入 `/xtm60/left/points`:`PointCloud2{x,y,z,intensity}`,frame=`xtm60_left_link`,BEST_EFFORT。
- 输出 `/lio/cloud_in`:`PointCloud2{x,y,z,intensity[,time=0]}`,frame=`xtm60_left_link`。

### LIO_Outputs
- `/Odometry`:`nav_msgs/Odometry`(body in odom/global)。
- `/path`:`nav_msgs/Path`。
- `/cloud_registered`:`PointCloud2`(当前帧已配准)。
- `/cloud_map`(可选/周期):`PointCloud2`(累积地图)。
- TF `odom→base_link`(TF_Owner=lio)。
- 派生 `/map_2d_from_3d`:`nav_msgs/OccupancyGrid`(2D 投影)。

### Map_Save_Artifacts(req 11)
- `<map_name>.pcd` / `.ply`:FAST-LIO 累积点云。
- `<map_name>.pgm` + `.yaml`:2D 栅格(Nav2)。
- 可选 `<map_name>.db`:RTAB_Map_Backend 启用时的回环数据库。

---

## Error Handling

| 情况 | 处理 |
|---|---|
| 点云含 NaN/(0,0,0) 占位点 | Cloud_Adapter 喂入前显式剔除(复用 `cloud_utils`,req 2.5) |
| FAST-LIO 报 `Failed to find match for field 'time'` | 预期告警(整帧无逐点 time);若上游硬要求,adapter 补 time=0 字段(§4.1) |
| 点云与 IMU 时间基准不一致被拒 | `restamp_to_now` 参数对齐时间基准(req 2.6) |
| 外参折算错误(地面不水平/墙不竖直) | 录包离线核对 → 修正传感器约定/外参,不一次写死(§4.3) |
| TF `odom→base_link` 双发布冲突 | `tf_owner=lio` 时关闭 EKF/ZLAC 的 TF 发布(§4.4,Property 3) |
| FAST-LIO 编译缺依赖(Sophus/TBB 等) | 文档补 apt 命令;`rosdep install` 兜底(req 1.4) |
| LIO 初始化失败/IMU 静止不足 | 启动时保持静止数秒让 iEKF 初始化;失败则报警并提示重启 |
| 雷达过热风险 | 测试单次上电 ≤30s 即停;录包→离线复跑;每次后 `pgrep` 确认无残留(req 5) |

---

## Correctness Properties

### Property 1: 无漩涡拼接
遥控小回环后,LIO 累积地图中同一面墙不应呈同心环/重影;
本应平行的墙/桌边夹角偏差 ≤ 5°(对照旧 RTAB-Map 漩涡图,req 11.3)。
**Validates: Requirements 4.4, 11.3**

### Property 2: yaw 漂移被 IMU 锁定
原地旋转 360° 回到起点,LIO yaw 闭合误差显著小于旧纯 EKF 链路(定量记录)。
**Validates: Requirements 2.4, 6.4**

### Property 3: 单一 TF 所有权
`odom→base_link` 仅 FAST-LIO2 发布;无 `TF_REPEATED`/authority 冲突;`icp_odometry` 不启动。
**Validates: Requirements 3.2, 3.3**

### Property 4: 外参一致
LIO 输出中地面在 base_link 下 z≈0、墙面竖直(外参折算正确)。
**Validates: Requirements 3.1, 3.4**

### Property 5: 默认不依赖 RTAB-Map
`enable_loop_backend:=false` 时仍产出清晰 3D 图 + 2D 栅格。
**Validates: Requirements 10.1, 10.4**

---

## Testing Strategy

遵循 auto_test 框架 + **雷达过热硬约束(单次上电 ≤30s 即停;5 min 硬上限)** + **录包→离线复跑** 工作流:

1. **录包(一次上电 ≤30s)**:`ros2 bag record /xtm60/left/points /imu/data /tf /tf_static`,操作者手动
   慢速转一圈 + 走小回环,立即停雷达。该 bag 为后续所有离线调参/回归的唯一数据源(req 5.5/6.2)。
2. **离线编译 + 跑 FAST-LIO2**:`ros2 bag play` 喂 adapter+fast_lio,反复调参不再上电雷达。
3. **外参核对**:从 bag 的 TF 解析 `T_il`,核对地面 z≈0/墙竖直(Property 4)。
4. **before/after 回归**:同一 bag 分别跑「旧 RTAB-Map 零配准」与「FAST-LIO2」,对比墙厚度/平行边夹角/
   回环闭合(req 11.3,Property 1/2)。
5. **RViz 截图**:版式各面板(3D 上色、2D 栅格、LaserScan、相机、轨迹)齐全(req 8)。
6. **TF/话题层**:`ros2 topic hz /Odometry /cloud_registered`,TF 单一所有权检查(Property 3)。
7. **单元测试**:`lio_cloud_adapter_node` 的无效点剔除 / 时间字段 / restamp 逻辑;删除组件的测试同步移除
   (req 9.5),`colcon test` 通过。
8. **安全**:运动相关步骤离地/清场、`motion_control_enabled` 默认 false、每次测试后 `pgrep` 确认无残留。

---

## 迁移与清理(req 9 / 7)

- **删除**:`kiss_icp_mapping_node.py` + `kiss_icp_mapping.launch.py`(及 setup.py 注册、测试)。
- **删除/归档**:`ground_plane_calibrator_node.py`(LIO 自估姿态;及其在 launch 的启动与测试)。
- **重构/移除**:`livo_3d_mapping.launch.py` + `livo_interface.yaml` 旧占位后端包装(避免误导入口)。
- **改写**:`manual_mapping_left.launch.py`(RTAB-Map 零配准硬编码)→ 由 `manual_mapping_lio_left.launch.py`
  取代为默认;`scripts/run_rviz_manual_mapping_left.sh`、`stop_mapping.sh`、`save_mapping_result.sh`
  更新为 LIO 链路。
- **保留**:`dual_lidar_cloud_fusion_node`、`cloud_to_occupancy_grid_node`、`cloud_utils`、RTAB-Map(后端角色)。
- **文档**:顶层 README + `docs/`(新增 `docs/fastlio_mapping.md`;更新/弃用 `rtabmap_3d_mapping.md` 主线表述)。

---

## 风险与已知限制

1. **窄视场沿墙平移退化**:IMU 锁住 yaw 可消除漩涡,但正对长平墙时沿墙平移仍可能缓慢漂移
   (纯 LIO 通病)。本阶段接受,后续由视觉(方案 A)或轮速约束(方案 C)缓解(req 7.6)。
2. **无回环(默认)**:FAST-LIO2 是里程计无回环;大场景累积漂移由可选 RTAB_Map_Backend 缓解(req 10)。
3. **外参/传感器约定**:最易错点,以实时 TF + 离线核对闭环处理(§4.3)。
4. **FAST_LIO 对缺 time 字段的容错**:若上游严格要求 time,adapter 补 0 值 time 字段兜底(§4.1)。
5. **aarch64 编译**:apt 默认 PCL/Eigen 应足够;若遇 Sophus/TBB 等缺失,文档补 apt 命令(req 1.4)。
