# Requirements Document

## Introduction

本特性将 SmartWheel(室内自主轮椅,ROS 2 Humble,工作区 `/home/nvidia/smartwheel`)的 3D 建图链路重构为
**「纯 EKF 位姿、ICP 关闭」(pure EKF pose, ICP disabled)** 模式,并在此基础上修复既有代码审查报告
(`docs/project_code_audit.md`,缺陷表 D001..D198)中**与单雷达 EKF 位姿建图相关、且未被废弃**的优先子集。

核心工程判断(已与用户在前期对话中确立,本特性据此实现):

- **传感器是窄视场 flash ToF 雷达,不是 360° 激光**:XT-M60 视场约 120°×90°。在面对一面平墙的 120° 视野中,
  ICP/几何配准在沿墙方向缺少约束、几何退化,会滑移并旋转 yaw,把本应平行的边变成三角形。因此本特性
  **关闭 ICP/几何配准**。
- **位姿来源是 EKF(robot_localization)**:融合 H30 IMU(Yesense YIS106,提供姿态/yaw,室内无绝对位移)
  与轮式里程计(ZLAC8030 电机编码器,提供距离)。EKF 已验证良好(90° 转向 EKF==IMU;直线距离误差 0.2%),
  其配置 `robot_localization_ekf.yaml` 已修正为「IMU 主导航向、轮速仅提供平移」。EKF 输出
  `/odometry/filtered` 作为建图的唯一位姿来源。
- **不替换 RTAB-Map**:将 RTAB-Map 重配为「纯外部里程计」模式(消费 EKF 的 `/odometry/filtered`、不做几何配准),
  或采用等价的「已知位姿点云累积」。RTAB-Map 保留数据库管理、点云累积、地图导出与未来的视觉回环能力。
- **拒绝 Fast-LIO / LIO-SAM / KISS-ICP / LIVO**:它们都是扫描匹配(类 ICP),需要宽/360° 视场,在本窄视场传感器上更差。
- **雷达安装高度/俯仰不确定且随时可能变化**:因此**不手填外参**(URDF/static_transforms 不写死高度与俯仰),
  改为从点云用 RANSAC **自动标定**雷达高度(z)与俯仰(pitch):雷达到主导向上法向地平面的垂距=安装高度,
  平面倾角=俯仰。需要视场内可见地面。
- **分阶段**:**先单雷达(左,192.168.0.101)必须成功**;双雷达(右,192.168.1.101)为后续独立阶段,本特性范围之外。

本特性遵循项目既有约定:**硬件优先**(位姿来自 IMU+轮速 EKF,而非 ICP);**仅本地、不推送 GitHub**;
引用审查修复时沿用 `project_code_audit.md` 的严重度分级(阻断/高/中/低);每项改动通过 `auto_test` 框架验证。

### 已在本会话完成、不在本特性范围内重做的事项

- XT-M60 点云绕前向轴旋转 180° 的问题已修复:URDF 中 `xtm60_left_fixed_joint` 与 `xtm60_right_fixed_joint`
  的 rpy 已由 `(-1.5708, 0, -1.5708)` 改为 `(1.5708, 0, 1.5708)`(原始传感器约定经硬件验证为 z 前、y 上、x 左)。
  这是先前建图变差的主因,已解决,本特性**不再重复处理**。

### 显式排除项(不作为本特性需求)

以下审查条目已被用户判定为废弃或与单雷达 EKF 建图无关,**不纳入本特性**:
LIVO/Fast-LIO/KISS-ICP 几何配准分支(D042、D119、`kiss_icp_mapping_node` 等);D113/D114(缺失 UI/voice 包依赖声明);
D078/D079(导航/语义占位数据);D090–D092、D097、D098(`velocity_limiter_node` 孤儿节点);
所有「测试覆盖不足」类(D008/D009/D023/D033/D051/D080/D099/D107);手填硬件位置/高度/外参类
(D130/D131/D132/D133/D163/D164/D165/D122/sensor_layout —— 被需求 2 的自动地平面标定取代,不手填数字);
双雷达专属项(D035 左右同步、D133/D165 右雷达外参 —— 推迟到双雷达阶段);多数「墙钟时间戳」项
(D012/D024/D028 等 —— 关闭 ICP 后重要性下降,降级处理,仅在仍痛时再议)。

## Glossary

- **EKF**:robot_localization 的扩展卡尔曼滤波节点,融合 IMU 与轮式里程计,发布 `/odometry/filtered` 与 `odom→base_link` TF。
- **EKF_Pose_Mapping_Pipeline**:本特性交付的单雷达建图链路,以 EKF 位姿为唯一位姿源、关闭几何配准。
- **RTAB_Map**:rtabmap_slam 节点;本特性中工作于「纯外部里程计、几何配准关闭」模式。
- **ICP**:迭代最近点几何配准;本特性中在建图链路中**禁用**。
- **Geometric_Registration**:任何基于点云几何对齐推断或修正位姿的过程(含 ICP、`icp_odometry`、RTAB-Map 的 `Reg/Strategy=1`、邻接链精修)。
- **External_Odometry**:由 EKF 提供、不依赖几何配准的外部位姿输入(`/odometry/filtered`)。
- **Left_Radar**:左侧 XT-M60 flash ToF 雷达(IP 192.168.0.101),帧约 120°×90° 视场。
- **Cloud_Map**:累积的 3D 点云地图(RTAB-Map 数据库及导出云)。
- **Occupancy_Grid_2D**:由 3D 点云投影得到的 2D 占据栅格(供 Nav2 使用)。
- **Ground_Plane_Calibrator**:从点云用 RANSAC 拟合主导地平面、自动估计雷达安装高度(z)与俯仰(pitch)的组件。
- **Radar_To_Base_Transform**:`xtm60_left_link`(或等价雷达帧)到 `base_link` 的坐标变换。
- **Auto_Test_Framework**:项目既有的 `auto_test/` 可视化自动化测试协议(每次测试一个 `auto_test/<时间戳>_<主题>/` 目录,含 `report.md`)。
- **Audit_Report**:`docs/project_code_audit.md` 缺陷表(D001..D198),严重度 ∈ {阻断, 高, 中, 低}。
- **Triangle_Distortion**:窄视场下 ICP 沿墙方向无约束、滑移/旋转 yaw,使本应平行的墙线收敛成三角形的失真现象。
- **Fusion_Node**:`dual_lidar_cloud_fusion_node`,在单雷达模式下负责将左雷达点云裁剪/降采样/变换到 `base_link` 并发布 `/points_merged`。

## Requirements

> 按优先级组织:需求 1–2 为本特性核心(建图模式重构 + 自动标定);需求 3 为相关审查修复;
> 需求 4 为分阶段约束;需求 5 为验证协议;需求 6 为审查修复的通用约束。

---

### 需求 1:单雷达纯 EKF 位姿建图(关闭 ICP)

**优先级:阻断级核心(P0)**

**用户故事(User Story):** 作为 SmartWheel 的开发者,我希望单(左)雷达建图链路只用 EKF 位姿累积点云、
完全不做几何配准,以便在窄视场传感器上消除 ICP 引起的 yaw 滑移与三角形失真,得到平行墙线正确的地图。

#### 验收标准(Acceptance Criteria)

1. THE EKF_Pose_Mapping_Pipeline SHALL 使用 `/odometry/filtered` 作为唯一位姿来源累积点云。
2. THE EKF_Pose_Mapping_Pipeline SHALL 禁用所有 Geometric_Registration(包含 `icp_odometry` 节点、RTAB_Map 的 `Reg/Strategy=1` 与 `RGBD/NeighborLinkRefining`)。
3. WHERE RTAB_Map 用于建图,THE RTAB_Map SHALL 以「外部里程计」模式运行,消费 External_Odometry 且不修改其传入位姿。
4. THE EKF_Pose_Mapping_Pipeline SHALL 发布累积的 3D Cloud_Map。
5. THE EKF_Pose_Mapping_Pipeline SHALL 发布由 3D 点云投影得到的 Occupancy_Grid_2D。
6. THE EKF_Pose_Mapping_Pipeline SHALL 提供一个可用的启动路径(launch 或脚本)以单条命令启动该链路。
7. WHEN 操作者绕一张桌子驱动一个小回环,THE EKF_Pose_Mapping_Pipeline SHALL 产出一张地图,其中本应平行的墙/桌边在地图中夹角偏差不超过 5°(即 Triangle_Distortion 消失)。
8. IF `/odometry/filtered` 在超过 0.5 秒内不可用,THEN THE EKF_Pose_Mapping_Pipeline SHALL 暂停点云累积并记录一条告警,而非用过期位姿累积。
9. WHILE 该链路运行,THE RTAB_Map SHALL 保留其数据库管理、点云累积与地图导出能力。

---

### 需求 2:雷达高度与俯仰的自动地平面标定

**优先级:阻断级核心(P0)**

**用户故事(User Story):** 作为 SmartWheel 的开发者,我希望系统从点云自动标定雷达的安装高度与俯仰,
以便雷达被移动或重新安装时无需编辑 URDF/static_transforms 即可保持建图正确。

#### 验收标准(Acceptance Criteria)

1. WHEN 建图链路启动且地面在 Left_Radar 视场内可见,THE Ground_Plane_Calibrator SHALL 用 RANSAC 从点云拟合主导的「向上法向」地平面。
2. THE Ground_Plane_Calibrator SHALL 取雷达到该地平面的垂直距离作为安装高度(z)。
3. THE Ground_Plane_Calibrator SHALL 取该地平面相对水平面的倾角作为俯仰(pitch)。
4. WHEN 标定完成,THE Ground_Plane_Calibrator SHALL 自动发布或更新 Radar_To_Base_Transform,使后续点云无需手填外参即可正确变换到 `base_link`。
5. IF 地面在视场内不可见或 RANSAC 内点比例低于可配置阈值,THEN THE Ground_Plane_Calibrator SHALL 保留上一次有效的 Radar_To_Base_Transform 并记录一条告警,而非发布无效外参。
6. THE Ground_Plane_Calibrator SHALL 在 `auto_test` 报告中输出本次估计的高度与俯仰数值,供人工核对。
7. THE EKF_Pose_Mapping_Pipeline SHALL NOT 依赖 URDF 或 `static_transforms.yaml` 中手填的雷达高度/俯仰数值(这些被自动标定取代)。

---

### 需求 3:与单雷达 EKF 位姿建图相关的优先审查修复

**优先级:高(P1)**

**用户故事(User Story):** 作为 SmartWheel 的开发者,我希望修复审查报告中影响单雷达 EKF 位姿建图质量的缺陷,
以便在 ICP 关闭后,剩余的时间戳/TF/正确性问题不再污染累积地图。每项修复引用其审查 D 编号与严重度。

#### 验收标准(Acceptance Criteria)

1. THE Fusion_Node SHALL 使用参与合并的源帧 `msg.header.stamp`(而非发布时刻墙钟)作为 `/points_merged` 的 `header.stamp`;仅当源 stamp 全为 0 时回退墙钟。(D034/D168,严重度:高;关闭 ICP 后对配准的重要性下降,但仍影响位姿插值时刻,故保留修复并允许降级为低风险验证)
2. THE Fusion_Node SHALL 使用点云 `msg.header.stamp` 对应时刻的 TF(而非 `rclpy.time.Time()` 最新可用变换)将点云变换到 `base_link`;TF 不可用时跳过该帧而非用最新 TF 强变换。(D178,严重度:中)
3. WHERE 单雷达模式启用,THE Fusion_Node SHALL 在仅左雷达新鲜时持续从左雷达发布点云(`allow_single_lidar_fallback`),不因右雷达缺席而停止。
4. IF 点云中存在无效/占位点(NaN 或全 0 坐标),THEN THE Fusion_Node SHALL 显式剔除该类点,而非依赖 `min_range` 过滤的副作用。(D179,严重度:低)
5. THE EKF_Pose_Mapping_Pipeline SHALL 对每项实施的审查修复在其代码注释或 `auto_test` 报告中标注对应的 D 编号与严重度。
6. WHERE 某审查条目属于显式排除集,THE EKF_Pose_Mapping_Pipeline SHALL NOT 将其纳入实现范围。

---

### 需求 4:分阶段约束(单雷达优先,双雷达推迟)

**优先级:高(P1)**

**用户故事(User Story):** 作为 SmartWheel 的开发者,我希望本特性明确限定在单(左)雷达,
以便先把单雷达建图做对,再在后续独立阶段引入双雷达,而不让双雷达的复杂度污染当前阶段。

#### 验收标准(Acceptance Criteria)

1. THE EKF_Pose_Mapping_Pipeline SHALL 仅使用 Left_Radar(192.168.0.101)作为点云来源。
2. THE EKF_Pose_Mapping_Pipeline SHALL NOT 要求右雷达(192.168.1.101)在线即可建图成功。
3. WHERE 右雷达相关的同步/外参问题(D035、D133、D165)被提及,THE EKF_Pose_Mapping_Pipeline SHALL 将其推迟到后续双雷达阶段,不在本特性中实现。
4. THE EKF_Pose_Mapping_Pipeline SHALL 在文档中记录单雷达为当前必须成功的阶段、双雷达为后续阶段。

---

### 需求 5:auto_test 框架验证

**优先级:高(P1)**

**用户故事(User Story):** 作为 SmartWheel 的开发者,我希望每项改动都经 `auto_test` 框架验证并留下可复现报告,
以便每个问题都有「现象 + 期望 + 根因 + 修复 + 复现命令」的闭环记录,符合项目既有测试协议。

#### 验收标准(Acceptance Criteria)

1. WHEN 完成一项改动的验证,THE Auto_Test_Framework SHALL 在 `auto_test/` 下创建一个 `<YYYYMMDD_HHMMSS>_<主题>/` 时间戳目录。
2. THE Auto_Test_Framework SHALL 在该目录内生成 `report.md`,包含现象、期望、初步根因(指向具体文件/参数/节点)、建议修复与复现命令。
3. WHEN 验证涉及 RViz 可视化,THE Auto_Test_Framework SHALL 采集截图(`NN_<步骤名>.png`)与数据层证据(如 `ros2 topic hz`、TF 输出)交叉印证。
4. WHERE 验证步骤会触发轮椅运动,THE Auto_Test_Framework SHALL 在报告中将该步骤标注为高风险并要求离地/清场前置条件。
5. THE Auto_Test_Framework SHALL 复用 `auto_test/templates/report_template.md` 作为报告结构。

---

### 需求 6:审查修复的通用约束

**优先级:中(P2)**

**用户故事(User Story):** 作为 SmartWheel 的维护者,我希望本特性的所有改动遵循项目既有约定,
以便改动可被一致地追溯、评审与回滚。

#### 验收标准(Acceptance Criteria)

1. THE EKF_Pose_Mapping_Pipeline SHALL 引用 Audit_Report 的严重度分级(阻断/高/中/低)来描述每项修复的优先级。
2. THE EKF_Pose_Mapping_Pipeline SHALL 将所有改动保留在本地版本控制中,不推送至 GitHub。
3. WHERE 改动影响安全相关链路(底盘运动、急停),THE EKF_Pose_Mapping_Pipeline SHALL 在 `auto_test` 报告中明确说明已验证项与未能验证项。
4. THE EKF_Pose_Mapping_Pipeline SHALL 优先修复直接影响建图质量的条目,再处理健壮性/规范类条目。
