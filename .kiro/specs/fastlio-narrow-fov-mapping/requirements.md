# Requirements Document

## Introduction

本特性将 SmartWheel(室内自主轮椅,ROS 2 Humble,aarch64/Jetson Orin JetPack 6 (Tegra R36.4),
工作区 `/home/nvidia/smartwheel`)的 3D 建图链路从**「纯 EKF 位姿、关闭几何配准」**重构为
**「LiDAR-惯性里程计(LiDAR-Inertial Odometry, LIO)」**模式,采用并适配开源 **FAST-LIO2**
(HKU-MARS,https://github.com/hku-mars/FAST_LIO)作为位姿估计与点云拼接的核心。

这是上一阶段(`ekf-pose-mapping` spec)的**根因性替换**。核心工程判断(已与用户在前期对话中确立):

- **漩涡/重影的根因是缺少帧间拼接**:上一阶段 `Reg/Strategy=0` 关闭了所有几何配准,点云仅按 EKF
  位姿"原地堆叠",不做帧间重合/过滤/后处理。EKF 的 yaw 一旦漂移,帧与帧对不齐,墙被堆成同心环
  (漩涡),物体出现重影。修复方向是补上"拼接"这一步。
- **拼接的正确做法是 IMU 主导 + LiDAR 紧耦合修正(不是纯 ICP)**:纯 ICP 在窄视场(XT-M60 约
  120°×45°(横向 120°、纵向 45°)flash ToF)正对平墙时几何退化,沿墙方向无约束 → yaw 滑移(上一阶段已踩坑,故当时关闭)。
  FAST-LIO2 用 IMU 高频角速度(H30 YIS106,100 Hz)预测姿态把 **yaw 锁住**,再用 LiDAR 点云在
  迭代卡尔曼滤波(iEKF)中做修正,即"ICP 做辅助"。窄视场有先例:Livox Avia(~70°×77°,比 XT-M60 还窄)
  在 FAST-LIO2 上工作良好。
- **本阶段为方案 B(纯 LiDAR + IMU)**:先不接相机进估计器。验证 LIO 能否消除 yaw 漂移与漩涡。视觉融合
  (方案 A / FAST-LIVO2)与轮速平移约束(方案 C)为后续独立阶段,本特性范围之外,但接口需为其预留。
  注意:相机**不进 LIO 估计器**,但其**画面仍在 RViz 中显示**(见需求 8),二者不冲突。
- **RTAB-Map 角色:路线 2(降级保留,不删除)**(已与用户确立)。FAST-LIO2 成为**唯一建图主线与默认入口**,
  彻底取代产生漩涡的「纯 EKF 零配准」链路;但 RTAB-Map **不删除**,而是**降级为可选的回环/位姿图优化与
  地图导出后端**(消费 FAST-LIO2 位姿),以保留 FAST-LIO2 本身不具备的回环闭合能力。真正的死代码
  (KISS-ICP、旧 LIVO 占位接口、ground_plane 标定)则清理。
- **XT-M60 是整帧快照式 flash ToF,无逐点时间戳**:因此必须**关闭运动去畸变(deskew)**,
  并提供一个把 `/xtm60/left/points`(标准 PointCloud2 XYZI)喂给 FAST-LIO2 的输入适配,
  而非其默认的 Livox 自定义消息或带逐点时间的扫描雷达格式。
- **本阶段先单(左)雷达**:左雷达 192.168.0.101。双雷达为后续阶段。
- **彻底替换、含文档**:用户已将旧版本备份并上传 GitHub,授权对本项目做彻底更改,包括 README 与文档。
  旧的纯 EKF/RTAB-Map 零配准建图链路被 LIO 链路取代;相关脚本、launch、文档同步更新。

本特性遵循项目既有约定:**硬件优先**;**仅本地改动,推送由用户自行决定**;每项改动通过 `auto_test`
框架验证(时间戳目录 + `report.md` + 截图/数据交叉印证);**雷达过热硬约束:任何单次雷达上电
≤ 5 分钟**,自动化测试中以更短时限(≤ 30 s 流式抓取/录包)并在捕获后立即停止。

### 已确认的平台事实(本 spec 据此实现,无需重测)

- 架构 `aarch64`;`/etc/nv_tegra_release` = R36(JetPack 6 系);ROS 2 `humble`;gcc 11.4;cmake 3.22.1。
- GitHub 可达:`git ls-remote https://github.com/hku-mars/FAST_LIO HEAD` 成功解析。
- 左雷达 192.168.0.101 在线(ping 0% 丢包)。
- H30 IMU(YIS106)100 Hz,`/imu/data`,EKF 配置中已为「IMU 主导 yaw」。
- 现存脚手架:`livo_interface.yaml`、`bringup_3d_slam.launch.py backend:=fast_livo2|r3live|none`、
  `livo_3d_mapping.launch.py`(外部后端 launch 包装器,缺包时优雅降级)。

### 显式排除项(不在本特性范围)

- 相机/视觉**进入 LIO 估计器**(FAST-LIVO2、雷达-相机外参标定、`rgb_cloud_colorizer` 融合上色)—— 方案 A,后续阶段。
  (注:相机画面在 RViz **显示**不在此排除之列,见需求 8。)
- 右雷达 / 双雷达 LIO —— 后续阶段。
- 轮速平移约束并入 LIO 状态 —— 方案 C,后续阶段(本阶段轮速仅保留在既有 EKF / 一致性监控中,不进 LIO)。
- Nav2 自主导航行为变更(本阶段只保证 3D→2D 投影接口不被破坏,供 Nav2 继续可用)。
- R3LIVE 后端。

## Glossary

- **FAST_LIO2**:HKU-MARS 开源 LiDAR-惯性里程计(iEKF + ikd-Tree 增量地图),本特性的位姿与拼接核心。
- **LIO_Pipeline**:本特性交付的单左雷达 LiDAR-惯性建图链路。
- **iEKF**:迭代扩展卡尔曼滤波,FAST-LIO2 紧耦合 IMU 与 LiDAR 的估计器。
- **Deskew**:运动去畸变,依赖逐点时间戳;XT-M60 为整帧快照,本特性中**关闭**。
- **Left_Radar**:左 XT-M60 flash ToF(192.168.0.101),约 120°×45° 视场,~10 Hz,XYZI PointCloud2。
- **Cloud_Adapter**:把 `/xtm60/left/points` 转换为 FAST-LIO2 可消费输入(点格式/时间/QoS)的适配组件。
- **Registered_Cloud**:FAST-LIO2 输出的、已配准到全局坐标系的累积点云(对应其 `/cloud_registered` / 地图)。
- **LIO_Odometry**:FAST-LIO2 输出的位姿/里程计(全局 `lidar`/`body` 位姿)。
- **Extrinsic_LiDAR_IMU**:LiDAR 坐标系到 IMU 坐标系的固定外参(平移 + 旋转),FAST-LIO2 必需参数。
- **TF_Owner**:`odom→base_link` 的唯一发布者(单一所有权,防 TF 争用)。
- **Map_Frame_Chain**:`map`/全局 → `odom` → `base_link` → 传感器帧 的 TF 链。
- **Auto_Test_Framework**:项目既有 `auto_test/` 可视化自动化测试协议(时间戳目录 + `report.md`)。
- **Radar_Thermal_Limit**:雷达过热硬约束,单次上电 ≤ 5 分钟;自动化测试用 ≤ 30 s 时限并即停。
- **Legacy_EKF_Mapping**:上一阶段的纯 EKF 位姿 + RTAB-Map 零配准建图链路(被本特性取代)。
- **RTAB_Map_Backend**:本特性中 RTAB-Map 的**新角色**——可选的回环/位姿图优化与地图导出后端,
  消费 FAST-LIO2 位姿;不再是建图主线(路线 2)。
- **Mapping_RViz_Layout**:建图用 RViz 版式,含 3D 上色累积地图 + 2D 占据栅格 + LaserScan 极坐标 +
  多路相机画面面板 + 电池/视图等面板(对标用户提供的参考截图)。
- **Camera_Panels**:RViz 中显示各相机 `image_raw` 的 Image 显示项;需支持未来扩展到 4 路相机
  (`/camera/{front,left,right,rear}/image_raw`)。
- **Legacy_Dead_Code**:与新方案无关且已废弃的组件:`kiss_icp_mapping_node` 及其 launch、
  旧 LIVO 占位接口(`livo_3d_mapping.launch.py` / `livo_interface.yaml` 的后端包装)、
  `ground_plane_calibrator_node`(LIO 自估姿态后不再需要)。

## Requirements

> 优先级:需求 1(获取并编译 FAST-LIO2)与需求 2(窄视场 ToF 输入适配)为阻断级核心;
> 需求 3(TF/坐标与单一所有权)、需求 4(启动链与建图产出)、需求 8(RViz 版式)为高;
> 需求 5(过热与安全)、需求 6(auto_test 验证)、需求 9(遗留代码处置)、需求 10(RTAB-Map 后端角色)、
> 需求 11(地图保存与回归对比)、需求 7(文档与彻底替换)为高/中。

---

### 需求 1:获取、编译并集成 FAST-LIO2(aarch64 / Humble)

**优先级:阻断级核心(P0)**

**用户故事:** 作为 SmartWheel 开发者,我希望在本机(Jetson Orin / Humble)成功获取并编译 FAST-LIO2
及其依赖,以便有一个可运行的 LiDAR-惯性里程计核心。

#### 验收标准

1. THE LIO_Pipeline SHALL 从 `https://github.com/hku-mars/FAST_LIO`(含子模块,如 `livox_ros_driver2`/ikd-Tree)
   获取 FAST-LIO2 的 ROS 2 Humble 兼容版本到工作区源码树(作为 vendored 第三方包或 `src/` 下独立包)。
2. THE LIO_Pipeline SHALL 在 aarch64 / ROS 2 Humble 上用 `colcon build` 成功编译 FAST-LIO2 及其依赖
   (PCL、Eigen 等),`BUILD_EXIT=0`。
3. IF FAST-LIO2 的上游主分支不原生支持 ROS 2 Humble,THEN THE LIO_Pipeline SHALL 采用其受支持的
   ROS 2 分支/fork(在 `auto_test` 报告与文档中记录所用仓库 URL 与 commit hash)。
4. WHERE 编译需要系统依赖(libpcl、libeigen3、Sophus 等),THE LIO_Pipeline SHALL 在文档中列出
   精确的 `apt`/`rosdep` 安装命令,并使其可复现。
5. THE LIO_Pipeline SHALL 将所用 FAST-LIO2 仓库 URL 与锁定的 commit hash 记录在文档中以便复现与回滚。

---

### 需求 2:窄视场 flash ToF 输入适配(关闭 deskew)

**优先级:阻断级核心(P0)**

**用户故事:** 作为 SmartWheel 开发者,我希望把 XT-M60 的标准 XYZI 点云正确喂给 FAST-LIO2 并关闭
运动去畸变,以便整帧快照式 ToF 数据能被 LIO 正确消费而不产生时间畸变假象。

#### 验收标准

1. THE Cloud_Adapter SHALL 使 FAST-LIO2 消费 `/xtm60/left/points`(`sensor_msgs/PointCloud2`,XYZI,~10 Hz),
   无需 Livox 自定义消息。
2. THE LIO_Pipeline SHALL 关闭运动去畸变(deskew / point-time 补偿),因为 XT-M60 整帧无逐点时间戳。
3. WHERE FAST-LIO2 需要每点时间或 ring/tag 字段,THE Cloud_Adapter SHALL 提供安全默认(整帧单一时间戳、
   time=0),不依赖不存在的字段。
4. THE LIO_Pipeline SHALL 消费 `/imu/data`(H30,100 Hz)作为 iEKF 的 IMU 输入。
5. THE Cloud_Adapter SHALL 正确处理 XT-M60 点云中的无效/占位点(NaN 或 (0,0,0)),在喂入前剔除。
6. IF 点云与 IMU 的时间戳基准不一致(墙钟 vs steady)导致 FAST-LIO2 拒绝数据,THEN THE LIO_Pipeline
   SHALL 对齐时间基准(或记录并提供配置项)使二者可被同步消费。

---

### 需求 3:坐标系、外参与单一 TF 所有权

**优先级:高(P1)**

**用户故事:** 作为 SmartWheel 开发者,我希望 LIO 的坐标系与外参正确接入既有 TF 树且不产生 TF 争用,
以便地图、机器人模型、传感器在 RViz 中空间一致。

#### 验收标准

1. THE LIO_Pipeline SHALL 配置 Extrinsic_LiDAR_IMU(`xtm60_left_link` 与 `imu_link` 间的平移+旋转),
   其值来源于 URDF/`static_transforms.yaml` 的既有外参,并在文档中记录。
2. THE LIO_Pipeline SHALL 保证 `odom→base_link` 在任一时刻只有一个发布者(TF_Owner)。
3. WHEN FAST-LIO2 作为 TF_Owner 发布位姿,THE LIO_Pipeline SHALL 禁用 EKF 与 ZLAC 的 `odom→base_link`
   发布,避免 TF 争用;反之亦然。
4. THE LIO_Pipeline SHALL 把 FAST-LIO2 的全局/里程计帧正确桥接到既有 `map`/`odom`/`base_link` 约定
   (必要时发布一个对齐 TF),使 RViz 的 Fixed Frame 下点云、RobotModel、传感器帧空间一致。
5. THE Registered_Cloud SHALL 在固定全局帧下累积且不随机器人移动而整体漂移(拼接正确)。

---

### 需求 4:启动链、建图产出与 RViz 可视化

**优先级:高(P1)**

**用户故事:** 作为 SmartWheel 开发者,我希望用一条命令启动单左雷达 LIO 建图并在 RViz 看到清晰、
可拼接的累积地图,以便手动遥控建图时实时看到正确结果。

#### 验收标准

1. THE LIO_Pipeline SHALL 提供一条单命令启动路径(launch + 脚本)启动:左雷达驱动 + IMU + Cloud_Adapter +
   FAST-LIO2 + RViz(+ 安全/底盘只读链路)。
2. THE LIO_Pipeline SHALL 发布累积的 Registered_Cloud(3D 地图)与 LIO_Odometry。
3. THE LIO_Pipeline SHALL 提供一个 RViz 配置,默认显示 Registered_Cloud(累积地图)与当前帧,
   且不含上一阶段导致"漩涡"的无限累积实时显示项(实时帧 `Decay Time=0`)。
4. WHEN 操作者手动遥控绕一个小回环驱动,THE Registered_Cloud SHALL 呈现可辨认的墙/物体结构而非
   同心环漩涡或显著重影。
5. THE LIO_Pipeline SHALL 保留把 3D 点云投影为 2D 占据栅格的能力(供 Nav2),或在文档中说明其接入方式。
6. THE LIO_Pipeline SHALL 提供保存最终地图(点云 PLY/PCD 及位姿)的方法。
7. WHERE 运动控制涉及(`MOTION=true`),THE LIO_Pipeline SHALL 默认只读(电机不动),运动需显式开启,
   并保留既有安全监督链(safety_supervisor / emergency_stop)。

---

### 需求 5:雷达过热与运动安全约束

**优先级:高(P1)**

**用户故事:** 作为 SmartWheel 维护者,我希望所有涉及雷达的开发与测试都受过热与运动安全约束保护,
以免损坏硬件或造成意外移动。

#### 验收标准

1. THE LIO_Pipeline SHALL 在任何自动化测试中将单次雷达上电时间限制在 ≤ 30 s(远低于 5 分钟硬上限),
   并在数据捕获/录包完成后立即停止雷达。
2. THE Auto_Test_Framework SHALL 在每次雷达测试后用 `pgrep` 确认无残留雷达/建图进程。
3. WHERE 测试会触发底盘运动,THE Auto_Test_Framework SHALL 将该步骤标注为高风险并要求离地/清场前置条件,
   物理急停在手边。
4. THE LIO_Pipeline SHALL 默认 `motion_control_enabled=false`(只读),不写电机指令,除非显式开启。
5. THE LIO_Pipeline SHALL 优先使用「一次性录包 → 离线反复调参」的工作流,把雷达上电次数降到最少。

---

### 需求 6:auto_test 框架验证

**优先级:高(P1)**

**用户故事:** 作为 SmartWheel 开发者,我希望每项改动都经 `auto_test` 验证并留下可复现报告,
以便每个问题有「现象 + 期望 + 根因 + 修复 + 复现命令」的闭环记录。

#### 验收标准

1. WHEN 完成一项改动的验证,THE Auto_Test_Framework SHALL 在 `auto_test/<YYYYMMDD_HHMMSS>_<主题>/`
   创建时间戳目录并生成 `report.md`(复用 `templates/report_template.md`)。
2. THE Auto_Test_Framework SHALL 录制一段「点云 + IMU」rosbag(≤ 30 s 雷达上电)作为离线调参与回归的数据源,
   并保留该 bag 以便无需再次上电雷达即可复测。
3. WHEN 验证涉及 RViz 可视化,THE Auto_Test_Framework SHALL 采集截图与数据层证据(`ros2 topic hz`、
   TF 输出、位姿轨迹)交叉印证。
4. THE Auto_Test_Framework SHALL 用离线 bag 回放定量评估拼接质量(如同一平面/墙在累积地图中的厚度、
   平行边夹角、回到原点的位姿闭合误差),给出 LIO 相对 Legacy_EKF_Mapping 的改善证据。
5. THE Auto_Test_Framework SHALL 记录所用 FAST-LIO2 仓库 URL 与 commit hash。

---

### 需求 7:彻底替换与文档更新

**优先级:中(P2)**

**用户故事:** 作为 SmartWheel 维护者,我希望本次重构彻底且文档同步,以便项目对外呈现一致的 LIO 建图方案,
不残留误导性的旧描述。

#### 验收标准

1. THE LIO_Pipeline SHALL 更新顶层 `README` 与相关 `docs/`,把建图方案描述为 LiDAR-惯性(FAST-LIO2)
   单左雷达模式,并标注双雷达/视觉融合/轮速约束为后续阶段。
2. THE LIO_Pipeline SHALL 更新或弃用上一阶段专用于「纯 EKF 零配准」的脚本/launch/配置中与新方案冲突的部分,
   不残留会把用户引向旧漩涡链路的默认入口。
3. WHERE 旧的 RTAB-Map 零配准配置被保留(用于 2D 投影或回退),THE LIO_Pipeline SHALL 在文档中明确其
   角色与新 LIO 链路的关系,消除"双真值源"歧义。
4. THE LIO_Pipeline SHALL 在文档中提供完整复现步骤:依赖安装、获取/编译 FAST-LIO2、启动建图、录包、保存地图。
5. THE LIO_Pipeline SHALL 把所有改动保留在本地版本控制中;是否推送 GitHub 由用户决定。
6. THE LIO_Pipeline SHALL 在文档中记录已知限制:窄视场沿墙平移退化仍可能残留少量漂移,后续可由视觉(方案 A)
   或轮速约束(方案 C)缓解。

---

### 需求 8:建图 RViz 版式(3D 上色地图 + 2D 栅格 + LaserScan + 多路相机面板)

**优先级:高(P1)**

**用户故事:** 作为 SmartWheel 操作者,我希望保留并完善当前建图的 RViz 版式,在一个窗口里同时看到
摄像头画面、建图实时上色的 3D 点云、建图生成的 2D 占据栅格、激光极坐标视图与机器人轨迹,
以便像参考截图那样直观地监控建图过程;并希望相机面板可随相机扩展到 4 路而无需重做版式。

#### 验收标准

1. THE Mapping_RViz_Layout SHALL 显示 FAST-LIO2 输出的累积 3D 点云(Registered_Cloud),
   默认按高度(AxisColor / Z)或强度上色,呈现参考截图那种彩色 3D 结构(实时帧 `Decay Time=0`,
   不复现上一阶段的"漩涡"无限累积)。
2. THE Mapping_RViz_Layout SHALL 显示由 3D 点云投影得到的 2D 占据栅格(Map 显示项),作为独立面板/显示项。
3. THE Mapping_RViz_Layout SHALL 显示 `/scan`(或等价 LaserScan)的视图,对标参考截图中的"激光雷达(LaserScan)"面板。
4. THE Mapping_RViz_Layout SHALL 显示机器人模型(RobotModel)、TF、以及 LIO/EKF 里程计轨迹(Odometry / Path),
   使操作者能看到行进路线(对标参考截图中的轨迹折线)。
5. THE Camera_Panels SHALL 显示当前可用相机的 `image_raw` 画面;WHERE 当前为左/右两路前向相机,
   THE Camera_Panels SHALL 至少显示这两路。
6. WHERE 相机未来扩展为 4 路(`/camera/{front,left,right,rear}/image_raw`),THE Camera_Panels SHALL
   提供对应的 4 个 Image 显示项/面板(可预置为 4 路,缺失的相机话题在无数据时留空而不报错破版)。
7. THE Mapping_RViz_Layout SHALL 保留既有的嵌入式遥控面板(`wheelchair_bringup/TeleopPanel`),
   使操作者可在同一窗口手动驱动建图。
8. THE Mapping_RViz_Layout SHALL 作为新建图启动链的默认 RViz 配置被引用,替换指向旧 `/rtabmap/*`
   漩涡显示的默认入口(显示话题改为 FAST-LIO2 的累积云与投影 2D 栅格)。
9. WHERE 参考截图含电池状态等诊断面板,THE Mapping_RViz_Layout SHOULD 在相应数据可用时提供等价显示项
   (数据不可用时不阻塞版式)。

---

### 需求 9:遗留建图组件的清理处置

**优先级:高(P1)**

**用户故事:** 作为 SmartWheel 维护者,我希望明确清理与新 LIO 方案无关的死代码,
以便仓库不残留会把人引向旧链路或造成混淆的组件。

#### 验收标准

1. THE LIO_Pipeline SHALL 移除 Legacy_Dead_Code 中的 KISS-ICP 组件(`kiss_icp_mapping_node.py`、
   `kiss_icp_mapping.launch.py`)及其在 `setup.py`/文档中的注册与引用。
2. THE LIO_Pipeline SHALL 移除或归档上一阶段的地平面标定组件(`ground_plane_calibrator_node.py` 及其
   在 launch 中的启动),因为 FAST-LIO2 自行估计姿态,不再需要手动地平面标定。
3. WHERE 旧 LIVO 占位接口(`livo_3d_mapping.launch.py`、`livo_interface.yaml` 的外部后端包装)与新的
   FAST-LIO2 集成方式冲突或冗余,THE LIO_Pipeline SHALL 将其移除或重构为指向 FAST-LIO2 的真实集成,
   不残留缺包即降级为"什么都不做"的误导性入口。
4. THE LIO_Pipeline SHALL 移除/重写专用于"纯 EKF 零配准"的启动入口(如 `manual_mapping_left.launch.py`
   对 RTAB-Map 零配准链路的硬编码调用),使默认建图入口指向 FAST-LIO2。
5. WHERE 删除组件涉及单元测试,THE LIO_Pipeline SHALL 同步移除/更新相关测试,保持 `colcon test` 通过。
6. THE LIO_Pipeline SHALL 保留共用组件:`dual_lidar_cloud_fusion_node`(点云入口)、
   `cloud_to_occupancy_grid_node`(3D→2D 投影),并确认其与新链路兼容。

---

### 需求 10:RTAB-Map 降级为可选回环/导出后端(路线 2)

**优先级:中(P2)**

**用户故事:** 作为 SmartWheel 开发者,我希望 RTAB-Map 不再是建图主线,但作为可选的回环优化与地图导出
后端被保留,以便保留 FAST-LIO2 本身不具备的回环闭合能力,供大场景或后续阶段启用。

#### 验收标准

1. THE LIO_Pipeline SHALL NOT 默认启动 RTAB-Map 的几何配准里程计;FAST-LIO2 为默认且唯一的建图主线与
   位姿来源。
2. WHERE 用户显式开启 RTAB_Map_Backend,THE RTAB_Map_Backend SHALL 以"外部里程计"模式消费 FAST-LIO2 的
   位姿,提供回环检测/位姿图优化、数据库管理与地图导出,而不替代 FAST-LIO2 做前端里程计。
3. THE LIO_Pipeline SHALL 在文档中明确 RTAB_Map_Backend 的新角色及其与 FAST-LIO2 的关系,消除"双位姿
   真值源"歧义(FAST-LIO2 = 前端实时位姿;RTAB-Map = 可选后端回环/导出)。
4. THE LIO_Pipeline SHALL 保证默认建图路径在**不启动** RTAB-Map 的情况下即可产出清晰可拼接的 3D 地图与
   2D 投影(即回环为可选增强,而非必需依赖)。

---

### 需求 11:地图保存/导出适配与 before/after 回归对比

**优先级:中(P2)**

**用户故事:** 作为 SmartWheel 开发者,我希望地图保存链路适配 FAST-LIO2,并用同一段数据定量对比新旧地图,
以便证明漩涡问题确实被解决并可复现保存成果。

#### 验收标准

1. THE LIO_Pipeline SHALL 提供从 FAST-LIO2 保存最终地图的方法(PCD/PLY 点云 + 轨迹/位姿),不再依赖
   `~/.ros/rtabmap.db`。
2. THE LIO_Pipeline SHALL 更新或替换 `scripts/save_mapping_result.sh`,使其保存 FAST-LIO2 的累积点云
   与 2D 投影栅格(供 Nav2);WHERE RTAB_Map_Backend 启用,其 `.db`/导出仍可选保留。
3. THE Auto_Test_Framework SHALL 用同一段离线 bag 分别产出"旧 RTAB-Map 零配准地图"与"FAST-LIO2 地图",
   做 before/after 对比(同一平面厚度、平行边夹角、回原点位姿闭合误差),作为漩涡消除的硬证据。
4. THE LIO_Pipeline SHALL 在文档中记录保存的地图文件格式、位置与再次加载/查看方法。
