# SmartWheel 当前项目交接说明

更新时间：2026-09-08（Asia/Shanghai）

适用对象：新 Codex 会话、其他 AI 工具、后续开发者

当前状态：阶段 B 的硬件基础链路与 RViz 实机工作台已建立，FAST-LIO2 软件加固、H2 离线回放和正式 3D 产品的软件 fail-closed 管线已完成；真实设备时间、最终外参、动态/实时性、回环重复性、正式实机累计三维建图、导航与载人安全仍未验收。

## 0. 2026-07-29 历史停止点（后续章节已补充最新状态）

用户要求暂不处理 LiDAR–IMU 外参和载人测试，先解决左雷达有效点比例低。
当前不得载人。只读诊断证明历史 `60.54%` 不是永久设备上限；当前左单机约
`90.7%`、右单机约 `97.5%`。决定性“左单机→双机→左单机”A/B 中，左侧
逐帧距离变化中位数/P95 从 `23/85 mm` 恶化为 `143/1646 mm`，停止右侧
后恢复，根因是两台未同步、同成像配置 Flash ToF 雷达的光学串扰。降低左侧
`miniAmp 70→50` 只有约 `+1.3` 个百分点收益且略增抖动，已经恢复为 `70`。

生产配置新增主机时钟网格错相：左/右 SDK 分别按共同 `100 ms` 网格的
`0/50 ms` offset 启动，并每 `30 s` 重新对齐；不写设备配置，
`enable_sdk_filters=false`、`apply_device_config=false` 不变。最终 65 秒
双路结果为左 `9.867 Hz/91.72%/30/131 mm`、右
`9.839 Hz/97.11%/16/67 mm`（频率/平均有效点/距离变化中位数/P95）。
左侧各持续 10 秒分段稳定在约 `91.5%～92.0%`，但有一次瞬时 `63.0%`
坏帧。右侧已接近单机质量，左侧仍比单机更抖。

这不是硬件同步；每 30 秒约有 1～2 帧空窗。C2 首轮 FAST-LIO2 应优先仅用
右主雷达，双雷达累计建图前增加坏帧门禁，并继续向厂家确认正式多机同步、
触发或安全调制频率方案。完整报告：
`docs/hardware/XT_M60_DUAL_INTERFERENCE_MITIGATION_20260729.md`。
测试和构建通过，最终 transient service inactive、无 adapter 进程、两路
无 UDP，雷达已安全停止。

## 1. 使用方式与事实优先级

本文件是便于快速接手的摘要，不替代测试证据。发生冲突时按以下顺序判断：

1. 用户最新明确指令；
2. `docs/PROJECT_MEMORY_CURRENT.md`；
3. Orin 当前仓库、已暂存报告与原始证据；
4. 用户提供的厂家手册、SDK、Windows 上位机配置；
5. 本文件；
6. 旧 README、旧分支和历史 AI 文档。

新会话必须先读取工作区 `AGENTS.md`、本文件和
`docs/PROJECT_MEMORY_CURRENT.md`，再运行 Windows 工作区中的只读检查：

```powershell
powershell -ExecutionPolicy Bypass -File .\tools\project_startup_check.ps1
```

不能只根据本文件的时间戳假定远端仍未变化。

## 2. 项目目标

SmartWheel 是一台低速室内 ROS 2 自动导航与安全辅助轮椅原型，不是可直接载人的通用全自动车辆。核心目标是让操作者通过安全的 WSAD 控制或合规的人工推动完成室内一圈采集，最终得到：

- 清晰、无严重双层重影的三维几何点云地图；
- 保留 XT-M60 反射幅值的 PointCloud+Amp/XYZI 累计点云产品；
- 可选 PCD、PLY 及由四相机离线着色的 XYZRGB PLY；
- 可供 Nav2 后续使用的二维占据栅格地图；
- RTAB-Map 数据库或等价的可重定位地图数据库；
- 完整轨迹、TF、rosbag、硬件/算法配置与质量报告。

第一版算法主线：

1. 主 XT-M60 + H30 IMU 进入 FAST-LIO2，生成局部 LIO；
2. 轮速里程计独立生成、标定，用作辅助约束和退化检测；
3. 副 XT-M60 经外参转换后参与全局三维/二维建图；
4. RTAB-Map 消费外部里程计和点云，负责关键帧、回环、全局位姿图和数据库；
5. `slam_toolbox` 只作为可切换的二维对照后端，不能与 RTAB-Map 同时发布 `map -> odom`；
6. 四相机全部录包，第一版最多选一台前向相机参与在线回环，其余用于离线着色。

不把 FAST-LIVO2、LIO-SAM 或自研多传感器滤波器作为第一版默认主线。

## 3. 当前代码与连接

- Orin SSH：`orin`，当前地址 `nvidia@192.168.5.10`；
- 远端仓库：`/home/nvidia/smartwheel`；
- GitHub：`https://github.com/owenkings/smartwheel`；
- 当前主开发分支：`feature/mapping-v2-rviz-workbench`；
- 2026-09-08 启动检查确认当前 HEAD 与 GitHub origin 同为
  `19188466ff48d9874bcd148a14da1e714b14732b`；
- GitHub 默认 `main` 较旧，不能自动切换；
- Windows 目录 `C:\Users\admin\Desktop\智能轮椅` 不是 Git 仓库，只保存文档、工具、厂家资料和本地记忆；
- 远端已有大量按步骤暂存的改动，尤其要保留用户的 `docs/goal.md`，不得自动 unstage、覆盖、reset、stash、clean、checkout、pull 或 push。

当前实机入口：

```bash
cd /home/nvidia/smartwheel
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch wheelchair_bringup manual_teleop.launch.py motion_control_enabled:=false
```

必须保持 `motion_control_enabled=false`，除非用户对新的运动试验再次明确授权并重新确认全部安全条件。

## 4. 已确认硬件与固定身份

### 双 XT-M60

- 左配置实例：`192.168.0.101`，ROS `/xtm60/left/points`，frame `xtm60_left_link`；
- 右配置实例：`192.168.1.101`，ROS `/xtm60/right/points`，frame `xtm60_right_link`；
- 当前启动日志对应的型号/序列串：左 `.0.101` =
  `XTM60B20250324000151`，右 `.1.101` = `XTM60B20250324000134`；
- 两台分别用独立 SDK 实例，并发约 10 Hz；
- 两雷达水平间距约 `0.60 m`，镜片中心离地约 `0.735 m`；
- SDK 轴：`+z` 前、`+x` 左、`+y` 上；
- 两个 Windows 导出配置均为 `imgType=4`（`IMG_POINTCLOUDAMP`）和 `renderType=2`；
- ROS 发布有组织 `160x60` XYZI，`intensity` 优先取 `frame.amplData`；
- 当前固定 TF 仅是临时显示/方向修正，最终 yaw、xyz、双雷达外参和硬件同步都未验收。

临时 TF：

- 左 xyz `[0.45, 0.30, 0.735] m`，RPY
  `[1.5515153364, -0.0136154207, 1.5709313394] rad`；
- 右 xyz `[0.45, -0.30, 0.735] m`，RPY
  `[1.7071165220, 0.0265268546, 1.5744345976] rad`；
- 两侧 `x=0.45 m` 均是旧估计，不是最终测量。

### 四相机

物理 USB3 路径已由用户确认：

- `2-3.1 = 左前`；
- `2-3.2 = 左侧`；
- `2-3.3 = 右侧`；
- `2-3.4 = 右前`。

为兼容现有 ROS 槽位，生产配置映射为：

- `/camera/left` = 左前；
- `/camera/front` = 左侧；
- `/camera/rear` = 右侧；
- `/camera/right` = 右前。

四路当前采用独立进程、MJPEG、compressed 订阅和 RViz 解码前 10 Hz 限流。右前相机/线缆/`2-3.4` 安装链曾稳定在约 4.925 Hz 并产生 libjpeg 警告，具体坏点仍需单变量 A/B 交换定位。暂定旋转 `180/180/270/180°` 仍需目视确认，四相机内外参未标定。

### 其他硬件

- H30 IMU：静态约 200 Hz；水平放置下 roll/pitch 安装修正已写入 TF；动态轴向、yaw、xyz 和设备时间戳仍未完成；
- FD07-34R：地址 1～4，按厂家间隔轮询，四路约 2 Hz；它属于近距安全层，不是 SLAM 主传感器；
- ZLAC8030：USB-RS485 静止只读通信已恢复；用户曾在双轮架空、无人、场地清空、急停可用条件下完成 RViz W/A/S/D/STOP 测试并确认方向正确；
- 上述电机测试不等于轮径、轮距、编码器比例、ground drive、物理急停闭环或载人安全验收。

## 5. 用户确定的 RViz 工作台要求

当前布局必须保持：

- 左侧从上到下：左前相机、左侧相机、SmartWheel Teleop；
- 右侧从上到下：右前相机、右侧相机、SmartWheel 2D Map；
- Displays、Views、SmartWheel System Status 与 Teleop 标签化合并；
- 中央主区域显示三维点云；
- 相机和 2D 地图遵循“图像优先”：正常状态只显示画面和一个紧凑 `Details`；
- 话题、路径、帧率、尺寸、状态和选项在点击 `Details` 后才显示，并放在可滚动区域；
- 相机保持原比例，允许黑边，不能默认拉伸或裁掉安全相关画面；
- 2D 地图默认 Auto-fit，优先铺满地图；关闭 Auto-fit 后才进行手动缩放和平移；
- 点云必须显示 PointCloud+Amp，即按 `intensity/amplData` 着色，不再用坐标轴颜色冒充。

当前工作台已完成上述六区布局、图像优先折叠面板和 intensity 着色。插件测试为 27/27 通过。

## 6. 已完成与通过范围

- Mock 阶段 A/A-R 和原 RViz workbench 已完成；
- B0 文档/SDK/协议审计完成；
- B1 左单雷达基础链路曾完成：约 10 Hz、XYZ 米制、无效强度哨兵过滤；
- B2 H30 静态链路与水平安装 roll/pitch 修正完成；
- 双雷达独立 SDK 实例并发、话题分离、基础 TF 显示完成；
- FD07-34R 地址 1～4 的只读轮询约 2 Hz；
- 四相机同时读取、30 分钟 compressed soak 和当前四画面显示完成，但有右前链路限制；
- ZLAC 静止读取和受控架空 RViz 手动方向测试完成；
- WSAD 远程桌面长按的重复 press/release 问题已加 120 ms generation-guarded release debounce；
- 双点云投影、合并 `/scan`、`slam_toolbox` 小型实时 2D 预览已经同屏；
- PointCloud+Amp 的数据语义、上位机二进制可见证据、ROS XYZI 保留路径已经审计；
- 四相机与 2D 地图的 `Details` 展开/收起已在实机桌面目视验证。

“完成”仅覆盖以上链路，不代表真实建图、导航或安全认证。

## 7. 当前阶段与最新结论

当前阶段是：**B 阶段硬件工作台收口 + C 阶段建图前的数据正确性准备**。

2026-07-26 的失败现象与 2026-07-28 的修复结论：

- 失败时只有物理右雷达贡献可见场景；左 ROS 话题虽约 10 Hz，距离中位数却为 `36.356 m`，用户遮挡左雷达时画面没有变化；
- 2026-07-28 使用厂家 SDK、ImageType 4、各自网卡绑定且不启用额外 SDK 滤波链，左右原始距离中位数分别恢复为 `1.278 m`、`1.922 m`；SDK `points` 与 `distData` 均满足 `0.001 m/mm`；
- 因此左硬件、测距数据和 XYZ 转换都能正常工作，故障位于 ROS 适配器额外调用的 median/edge/Kalman/dust/postprocess/reflective SDK 滤波链；
- 生产左右 YAML 现统一设为 `enable_sdk_filters: false`，没有写入设备保存配置，`apply_device_config` 仍为 `false`；
- 完整双路 ROS 复测：左 `10.069 Hz`、距离中位数 `1.356 m`、P05～P95 `0.484～4.125 m`；右 `10.066 Hz`、中位数 `1.945 m`、P05～P95 `1.060～5.549 m`；
- RViz 分别关闭左右显示后，两路均能独立留下可识别近场几何；恢复双显示后两路 PointCloud+Amp 同时出现，双雷达 RViz 软件显示问题已修复。

尚未单独定位六类可选滤波中的具体责任项，不能整体重新启用。只读设备回读还发现左右保存配置当前存在 `isFilterOn=55/125` 和第三曝光槽 `20/30 us` 的差异；由于关闭额外 SDK 滤波后两路原始数据已正常，本次没有进行设备写入。详情见 `docs/hardware/XT_M60_DUAL_RVIZ_RECOVERY_20260728.md`。

### 雷达任务结束的强制停机要求

两台 XT-M60 长时间运行温度很高。任何真实雷达任务完成、暂停或交接前，都必须停止测量，不能把 RViz/bring-up 留在后台。

推荐顺序：

1. 对两个 `xtm60_adapter_node` 发送 SIGINT，让节点执行 SDK `stop()` 和 `shutdown()`；
2. 等待适配器进程退出，再停止其余工作台服务；
3. 确认服务 `inactive` 且不存在 `xtm60_adapter_node`；
4. 必要时分别监听 `192.168.0.100:7687`、`192.168.1.100:7687`，确认不再收到 UDP 帧。

不能只依赖普通 `systemctl stop`：当前 transient service 在总进程组退出过慢时会进入 SIGKILL，可能跳过厂家 SDK 的正常清理。2026-07-26 最终已分别完成两台 SDK 的干净停止；工作台为 `inactive`、无雷达进程，两块主机网卡地址各监听 3 秒均无 UDP 数据。

## 8. 当前阶段尚未完成的效果

按优先级：

1. **额外 SDK 滤波链的具体故障项尚未定位。** 当前生产配置已安全关闭整条可选滤波链并恢复双路显示；后续若要恢复滤波，必须逐项、有界 A/B，不能整体打开；
2. **中央三维仍是当前帧原始双雷达点云，不是累计建图。** 尚不能得到参考视频那种随运动持续增长、回环优化后的三维地图；
3. **2D Map 只是静止实时预览。** 已能增长和显示，但质量、闭环、保存、重载和重复一致性未验收；
4. **FAST-LIO2 未做真实硬件验收。** 未完成静止、直线、慢转、小闭环；
5. **双雷达最终外参/时间同步/重叠区融合未完成。** 临时 TF 不能用于地图质量结论；
6. **离线 PointCloud+Amp 累计产品已实现，正式实机产品未验收。** H2 full replay 已导出并校验 4,983,459 点 XYZI；新 RTAB 正式链会从优化图同快照生成几何、强度和完整 6DoF 轨迹，但仍需真实硬件动态/外参/时间/回环验收；
7. **右雷达曾出现 amplitude `>2039`。** 本次未复现，但仍要向厂家确认语义，存储数据不能静默截断；
8. **相机标定和右前物理链路未完成。** 四相机真实内参、畸变、外参和离线着色均未验收；
9. **IMU 动态轴向/yaw/xyz 未完成。**
10. **轮速/安全仍有限。** 轮径、轮距、极性、编码器比例、watchdog、地面运动、物理急停闭环和载人安全均未验收。

## 9. 后续阶段计划与门禁

### C0：双雷达数据正确性收口

- 核对物理序列号、IP、左右贴标和上位机显示；
- 解决左路 11～49 m 的异常距离分布；
- 用相同静态场景记录左右原始 XYZI、幅值分位数、有效点比例、帧率和时间戳；
- 确认两路量程、坐标轴和 PointCloud+Amp 都可信后才能进入 LIO。

### C1：标定和可复现数据集

- 完成 H30 动态轴向与 LiDAR-IMU 外参；
- 测量双雷达精确 xyz/rpy，验证重叠区；
- 验证时间同步/时间偏差；
- 在电机默认禁用下录制静态、人工缓慢移动或重新授权的低速 rosbag；
- 每个关键数据集保存硬件配置、TF 和诊断摘要。

### C2：单雷达 FAST-LIO2

- 固定并记录 FAST-LIO2 ROS 2 依赖 commit；
- 先用主雷达 + H30 离线 bag；
- 再按静止、直线、慢转、小闭环顺序做实机只读算法验收；
- 检查漂移、姿态跳变、墙面重影、时间戳、CPU/GPU、丢帧；
- 输出 LIO odom/path 和保 intensity 的 registered cloud。

### C3：双雷达 map-only 与累计三维

- 主雷达只进入 FAST-LIO2，副雷达按时间门限和最终外参投到已估计位姿；
- 不允许无时间检查地拼“最新两帧”；
- 输出几何累计地图和独立的 PointCloud+Amp 累计地图；
- 验证重叠墙面不出现明显双层。

### C4：RTAB-Map、2D 产品与回环

- RTAB-Map 使用外部 LIO 里程计，完成关键帧、回环、全局优化与数据库；
- 生成 PCD/PLY、二维 PGM/PNG/YAML、轨迹、manifest 和质量报告；
- `slam_toolbox` 仅作二维对照，确保只有一个 `map -> odom` 发布者；
- 地图必须能保存、重载，并在同路线重复三次具有基本一致性。

### C5：相机着色

- 标定四相机内外参；
- 使用优化后位姿、深度/可见性检查和多相机择优完成离线 XYZRGB PLY；
- 先保证几何地图正确，不能让着色问题阻塞 LIO 主线。

### D：导航与地面安全测试

只有前述地图与里程计通过后，且用户对每个新运动阶段重新授权，才进行：

- 轮径/轮距/编码器比例标定；
- 地面低速运动与制动；
- 物理急停闭环；
- Nav2 定位、规划、避障；
- 受控环境下的安全验收。

载人测试不属于当前授权范围。

## 10. 当前阶段完成定义

当前收口阶段至少满足以下条件才可宣布完成：

- 两路 XT-M60 在同一近场环境都输出尺度可信的 XYZI、约 10 Hz、无时间倒退；
- 两路 PointCloud+Amp 单独显示均有清楚、可识别的场景结构；
- 左右物理身份、IP、serial、topic、frame 一一对应；
- 临时 TF 被最终测量/标定外参替换，重叠区无明显双层墙；
- 四相机角色与旋转目视确认，右前链路风险有明确 A/B 结论；
- H30 动态轴向与时间语义完成；
- 形成可供 FAST-LIO2 离线复现的静态和低速 rosbag；
- 电机写入保持默认关闭，所有报告和证据暂存且工作树用户改动被保留。

## 11. 安全边界

- 不能猜测 IP、数据包、单位、时间戳、外参、串口参数、电机寄存器或安全行为；
- 设备“能显示”不代表 TF、尺度、时间戳或同步正确；
- 不自动重复雷达配置/标定写入；
- 不允许在任务结束后继续让雷达测量；必须执行并核验上一节的雷达停机流程；
- 不自动开始 FAST-LIO2 实机验收、完整外参标定、累计建图、导航或新的电机运动；
- 任何真实运动前必须再次由用户确认：双轮架空、无人乘坐、场地清空、物理急停可用；
- 地面运动、物理急停闭环、导航和载人安全需要更高等级独立授权；
- 暂存不等于提交；除非用户明确要求，不 commit、不 push。

## 12. 关键文件与证据入口

- 长期记忆：`docs/PROJECT_MEMORY_CURRENT.md`；
- 原始项目目标：`docs/goal.md`（其中阶段 A 的旧限制已被后续用户明确授权部分取代）；
- 当前交接：`docs/PROJECT_HANDOFF_CURRENT.md`；
- 工作台报告：`docs/hardware/HARDWARE_WORKBENCH_REPORT_20260726.md`；
- PointCloud+Amp 审计：
  `docs/hardware/XT_M60_POINTCLOUD_AMP_ANALYSIS_20260726.md`；
- 双雷达报告：`docs/hardware/XT_M60_DUAL_BRINGUP_REPORT.md`；
- 非电机硬件验收：
  `docs/hardware/NON_MOTOR_HARDWARE_ACCEPTANCE_20260722.md`；
- 左右最新诊断：
  `docs/hardware/evidence/XT_M60_AMP_LEFT_20260726.json`、
  `XT_M60_AMP_RIGHT_20260726.json`、
  `XT_M60_AMP_LEFT_RIGHT_STOPPED_20260726.json`；
- 左右隔离画面：
  `docs/hardware/evidence/XT_M60_AMP_LEFT_ONLY_20260726.png`、
  `XT_M60_AMP_RIGHT_ONLY_20260726.png`；
- 实机启动：
  `src/wheelchair_bringup/launch/manual_teleop.launch.py`；
- 最终 RViz 配置：
  `src/wheelchair_bringup/rviz/hardware_operator_workbench.rviz`。

接手后的第一件事不是继续写算法，而是刷新远端状态并处理第 7 节的左路距离异常。

## 13. 2026-07-29 最新长测纠正（覆盖前文短测判断）

- `65 s` 双路左/右 `91.72%/97.11%` 只能保留为短时历史结果。
  后续 `30 min` 长测左路平均/中位仅 `46.38%/45.56%`，质量门接受
  `21.21%`、主点云 `2.084 Hz`；右路为 `97.25%/97.33%`、接受
  `98.01%`。正式双路门禁失败。
- 左独立 `10 min` 仍约 `46.16%`，所以当前主因不是仅双路光学串扰；
  温升期间有效点略增，也不是简单热衰减。
- 同场景直接 SDK 左/右独立 60 帧为 `48.03%/96.90%`，幅值中位数
  `179/511`。左侧原始距离已出现大量厂家失效码，问题跟随左硬件链。
- 只读快照发现左第三 HDR 曝光回退 `30 -> 20 us`。已按序列号门禁
  恢复 `30 us`，断开、测量、再次断开后的回读都保持；左路仍约 `48%`，
  所以配置漂移不是剩余主因。
- 新增 fail-closed 保护：质量门、拒绝帧话题、质量 JSON、`75%` 绝对
  有效点下限，以及启动前 serial/曝光/HDR/minAmp/fps 只读核对。漂移
  时保持停测，不自动改写。单测 `21/21`，两包构建和真实 `45 s` 启动
  冒烟通过。
- 下一步必须由用户做单变量物理 A/B：先完整断电左雷达并检查/清洁光学
  窗口，再依次隔离左供电、线缆/接口或设备本体。不要同时交换多个变量。
- 映射软件回归全部通过，但现有 bag 没有同时包含真实 XT-M60+H30 与
  实际车体路线；FAST-LIO2 静止/直线/转弯/小闭环、RTAB-Map 回环和
  保存/重载没有因此完成。
- 用户明确暂缓 LiDAR–IMU 外参和载人测试；H30 动态轴向/时间偏差与
  双雷达最终外参仍需人工配合。没有新的地面电动运动或物理急停授权。
- 当前雷达均已停止：服务 `inactive`、无适配器进程、两路 UDP 均静默。

## 14. 历史右雷达单路 Stage 1 候选（2026-07-29，非当前正式入口产品）

本节结果不是由 `formal_3d_mapping.launch.py` 产生，也不是正式双雷达 3D 产品；
仅作为历史 Stage 1 candidate 证据保留。

Orin 连接已恢复，右路 Stage 1 已同步、build 并完成静态实机验收：

- 左适配器关闭且左话题不存在；H30、FAST-LIO2、电机写入均关闭；
- 四个改动包 build 通过，相关测试 `54/54` 通过；
- 最终 25.04 秒诊断：右点云 227 帧、约 `9.804 Hz`、有效点
  `96.969%`，`160x60` XYZI、尺度和时间检查全通过；
- 最终 bag：
  `/home/nvidia/smartwheel/bags/hardware/right_lidar_stage1_20260729_185019`
  （54.49 秒，12,713 条消息）；
- live map 为 `59x120 @ 0.05 m/cell`；PGM/YAML 实际落盘，pose graph
  序列化和重载成功；
- 版本地图包：
  `/home/nvidia/smartwheel/maps/versions/right_lidar_stage1_20260729_185022`，
  manifest 14 文件完整；PointCloud+Amp PCD/PLY 为 18,521 点并保留
  intensity；
- 证据：
  `docs/hardware/evidence/right_lidar_stage1_20260729_185017`；
- 最终 service `inactive`、无适配器进程、两路 `:7687` UDP 均静默。

保存脚本不再调用会返回 `255` 的 Humble `slam_toolbox SaveMap` 回调，
而是用独立 Nav2 map saver，并硬校验 occupancy、pose graph 和 export
文件。RTAB-Map 的 guarded ICP loop switch 仍默认关闭。

下一步不是继续修左雷达，而是在无人、人工监督且获得新的运动授权后，用
右雷达完成一条低速短路线：静止起点、直线、转弯、小闭环、回到起点，
再检查地图增长、里程计漂移和假回环。当前
`RIGHT_LIDAR_STAGE1_PRELIMINARY` 的 `hardware_validated=false` 必须保留；
静态通过不等于路线/小闭环、LiDAR–IMU/FAST-LIO2、地面行驶、物理急停、
导航或载人通过。用户仍明确暂缓 LiDAR–IMU 外参和载人测试。

## 15. 2026-09-04 FAST-LIO2 软件修复交接

当前主仓库 HEAD/origin 均为 `19188466ff48d9874bcd148a14da1e714b14732b`，
嵌套 FAST-LIO2 HEAD 为 `2fffc570a25d0df172720bac034fbdb6a13d2162`。
用户已有 staged/dirty 内容全部保留；本轮未提交或推送。

软件已闭环的项目：R 恢复 `0.001`、完整更新门与回滚、有界自适应重试、
退化可观子空间投影、时间对齐且健康/IMU 双门控的正规 ZUPT、400 样本/
2 秒静止初始化、当前帧 covariance 发布、有限 `/path`、小队列点云、单雷达
phase realign 默认关闭、左右质量话题隔离、动态 temporal/relative 只报告、
ZLAC 反馈失败不再伪造零速、完整 `body→base_link` 逆变换，以及固定基线
可重放补丁。旧 R=100/ZUPT 补丁已退役为不可应用 tombstone。

验证：FAST-LIO2 完整编译通过；四个相关主包 build 通过；105 项相关测试
全部通过；验收监控器 py_compile/flake8/help 与无输入 fail-closed 冒烟通过；
最终补丁在干净克隆可应用、与当前核心源码逐字节一致、二次运行幂等。未启动
任何硬件。

仍需用户配合，且应按顺序最后完成：

1. H0：水平地面、无人触碰、电机写入关闭的 120 秒静止基线；
2. H1：人工缓慢双向 yaw 激励，确认两设备时钟语义并估计固定延迟；
3. H2：实测 `base_link` 原点、雷达/IMU xyz 与 yaw，完成独立数据验证后才
   解除 `BLOCKED_CONFLICT`；
4. H3：获准的直行、左右转、STOP 与短点云空窗动态闭环；
5. H4：不录 bag、最小 bag、再加 registered cloud 的负载 A/B。

执行命令、阈值和关机证据见 `docs/FASTLIO_HARDWARE_VALIDATION_PLAN.md`。
当前不能把主机接收时间称为真实采样时间，也不能把动态、压力或外参标定写成
已完成。

## 16. H0 静止基线结果（2026-09-04）

H0 已通过。入口固定 `motion_control_enabled=false`，关闭 RViz/相机/超声波；
H30 以 401 样本/2.00021 秒完成初始化。120.07 秒监控得到：1177 个
Odometry 样本、9.802 Hz，最大 Odometry/cloud 空窗 0.248/0.247 秒，最大
单帧平移 2.41 mm、旋转 0.193°，活动半径 2.98 cm，全零 covariance 0；
反馈健康 5997 true/0 false，点云质量 1199 条且无拒绝。末尾更新计数为
`1626/0/0/0`（accepted/rejected/projected/adapted），ZUPT 1626 次、秩 6。

关停后无相关进程，两路 UDP 均为 0 packet/0 byte。证据位于
`auto_test/fastlio_h0_20260904_152735`，报告为
`docs/hardware/FASTLIO_H0_STATIONARY_20260904.md`。下一步只能进入 H1
时间语义/固定延迟采集；H0 不代表外参、动态门禁、负载或产品地图已通过。

## 17. H1 formal conditional close and H2 offline replay status

- 用户已现场完成正式左右手动 yaw；正式 bag 为 `/home/nvidia/smartwheel/auto_test/h1_formal_20260904_213140/bag`，约 537.554 s / 185501 条消息。IMU 核心积分约 `+1.60/-1.96 rad`，timing/quality/点云/IMU/TF/Odometry 可复核。
- 用户明确要求跳过独立单设备/scan-to-scan 检查以推进 H1；因此 H1 为 `CONDITIONAL_PASS_BY_USER_WAIVER`。不把融合 Odometry 间接 lag 写入 offset，仍保持 `host_receive`、`time_sync_en=false`、`time_offset_lidar_to_imu=0.0`。
- H2 目前只做离线准备：未启动真实雷达、IMU、FAST-LIO、底盘或电机，未写设备配置，最终外参仍未批准。左 FAST-LIO YAML 历史外参与当前 URDF 临时安装值不一致，保持审查。
- `offline_mapping.launch.py` 已补齐 `bag_cloud_topic`、`bag_odom_topic`、`bag_tf_topic`、`bag_storage`、`bag_start_offset_sec` 参数，正式嵌套 bag 使用 `--storage sqlite3`；`hardware_enabled=true` 在节点构造前拒绝，`state_mode` 透传正确。离线 SLAM 参数采用 1.0 s TF 等待、30 s buffer、50 帧 scan 缓冲。
- 启动验证可读打开 bag 且发现点云、scan、fused odom、TF，但 `slam_toolbox` 仍报告 `Failed to compute odom pose`。进一步读取确认 `/tf_static` 包含 `body→base_link` 与 `base_link→xtm60_right_link`；问题仍在离线 `camera_init→body` 到 SLAM 消费链的时间/后端契约，不得宣称离线建图通过。
- 已新增离线专用归一化节点：隔离原始动态 `/tf`，按 scan 时间插值并独占发布 `odom→body`，scan 输出使用可靠 QoS；19 项聚焦测试和三包构建通过，短回放能对齐 115 条 odom/scan，但 slam_toolbox 仍失败。因此它是待验收候选，不是 H2 后端通过。
- 下一步：做更小的 `getOdomPose` 隔离复现或明确切换后端契约，完成小段回放验收后，才能讨论 H2 实机授权。当前所有测试结束后无离线/硬件残留进程，双 UDP 7687 静默。


## 18. 2026-09-06 H2 离线短回放通过（完整回放仍待完成）

H2 离线入口的主要阻塞已收口：正式 bag 的 `/tf_static` 是 volatile，且非零 start offset 会跳过 bag 起点静态 TF；此外离线节点重命名使 `slam_toolbox_params.yaml` 的 `slam_toolbox:` 顶层键未自动应用。当前入口已增加静态 TF transient-local relay、非零 offset fail-closed、原始纳秒时间戳保留以及 `base_link` 等关键参数显式注入。

独立 ROS domain 的只读短回放结果：`32` scans、`31` normalized odom、`1` map（`28x56 @ 0.05 m/cell`），无 `Failed to compute odom pose`、无 Message Filter queue drop；相关三包构建通过，定向测试 `13 passed`，量程提示已按 `min_laser_range=0.2`/`max_laser_range=20.0` 修正。当前可记录为 **H2_OFFLINE_SHORT_REPLAY_PASS**，但必须完成完整约 537 s 回放、地图导出/重载和报告后，才能关闭 H2 离线阶段。

不得把该短回放结果解释为 FAST-LIO2、最终外参、真实硬件建图、地面运动或载人安全通过。所有真实 H2 操作仍需新的用户授权；当前硬件保持停止。

## 19. 2026-09-06 H2 离线完整回放通过，等待实机阶段授权

H2 离线软件准备已完成一轮完整验收：

- 正式 H1 bag `/home/nvidia/smartwheel/auto_test/h1_formal_20260904_213140/bag` 在独立 ROS domain 以 `replay_rate=0.5` 完整回放；入口保持 `hardware_enabled=false`、`bag_start_offset_sec=0.0`，全程未启动真实传感器、FAST-LIO、底盘或电机。
- `slam_toolbox` 主链无 `Failed to compute odom pose`、无 Message Filter 丢弃、无 ERROR；5,000,000 有界体素上限未触发。
- map-products 的有界 pending cloud 队列消除了回调先后竞态：session `STOPPED/complete=true`，4777 帧全部接受，`rejected_frame_count=0`，4,983,459 个体素；质量报告另记 174 个超出 0.15 s 时间窗的关联候选。`map_pointcloud_amp` 保留 4,983,459 点和 intensity。
- 输出 `/home/nvidia/smartwheel/auto_test/h2_offline_full_20260906_021135/offline_map_20260906_021135_222732` 的 14 个 manifest 文件 bytes/SHA256 全通过；2D PGM/YAML 由 `nav2_map_server` configure/activate 成功重载并发布 `/map`（8510x6569，0.05 m/cell）。
- 启动器已改为 playback→stop session→export，33.8 s 子包自动回归通过；相关构建通过，定向测试 `54 passed`。所有离线/地图重载进程已停止，XT-M60 两路 UDP 7687 均静默。

当前结论：**H2_OFFLINE_FULL_REPLAY_PASS_WITH_TIME_WINDOW_NOTE**。仍未完成/未授权：最终 LiDAR/IMU/base 外参、H30 动态轴向与时间偏差、FAST-LIO2 实机或累计 3D 建图、双雷达融合、RTAB-Map 回环一致性、地面运动、物理急停闭环、导航和载人安全。下一步需要用户明确是否进入 H2 实机外参/动态采集；在新授权前不启动任何硬件。


## 20. 2026-09-06 H2 实机外参与时间语义采集

- 用户确认无人、清场、急停可达、操作者在场；采用 `right_lidar_diag_mapping.launch.py` 只读入口，`motion_control_enabled=false`、无相机/超声波、无电机控制写入。
- 证据目录 `/home/nvidia/smartwheel/auto_test/h2_real_extrinsic_time_20260906_124524`，bag 约 1127.928 s、1.7 GiB、316186 条消息；包含初始/左右静止姿态、墙角静止、约 1 m 直行和约 45° 左转。
- 右点云/quality/timing 各 11266，IMU 225443，Odometry 10350；右 quality 全部 accepted，valid fraction 中位 0.99427、范围 0.68552..0.99490。反馈健康全程 false=13704，wheel/odom=0。
- XT-M60 SDK timestamp 单调、98..200 ms（中位 100 ms），但时间源仍为 `host_receive`；H30 IMU header 出现 2318 个重复时间戳，严格单调时间语义尚未通过。
- 当前临时外参与不健康轮速下 FAST-LIO Odometry 长录制发散，不能做几何真值；尚缺机器可读动作标记、base 原点/IMU xyz 实测及独立墙面/地面拟合。
- 结论为 **H2_REAL_COLLECTION_CAPTURED_NOT_ACCEPTED**；保留 `time_offset_lidar_to_imu=0.0`、`host_receive` 与 `BLOCKED_CONFLICT`。详情见 `docs/hardware/FASTLIO_H2_REAL_EXTRINSIC_TIME_20260906.md`。


## 21. 2026-09-06 H2 双雷达加速尝试

- 为加快 H2，启动只读双路原始入口（左右 XT-M60 + H30，独立 ROS domain），不启动 FAST-LIO、底盘或电机；证据 `/home/nvidia/smartwheel/auto_test/h2_real_dual_raw_20260906_132843`。
- bag 时长 165.283 s、约 256.6 MiB、44043 条消息；右 points/timing/quality 各 1640，IMU 33054。
- 左路仅有 2772 条 status：`waiting: XT-M60 SDK not running; waiting for ping 192.168.0.101`；左 points/timing/quality 均为 0，因此没有进行现场转动或双路外参采集。
- 录包、左右适配器与 IMU 已停止；无相关进程、无 UDP 7687。
- 结论：此前只用右雷达是因为右路稳定且已有 Stage 1 证据，而左路当前未 ping；不能用右路替代左路。下一步先做左路电源/链路/IP 单变量只读排查，恢复有效数据后再进行带 marker 的双路短采集。详情见 `docs/hardware/FASTLIO_H2_DUAL_RAW_20260906.md`。

## 22. 2026-09-07 H30 时间戳软件收口与最新 H2 阻塞

本轮恢复连接后只做了软件修改和离线验证，没有启动雷达、IMU、FAST-LIO、底盘或电机。

- H30 `YesenseSample` 现在同时保留原始串口读取边界（`host_receive_time_ns`/
  `host_receive_monotonic_ns`）和按配置 nominal rate 估计的逐样本时间
  （`host_interpolated_time_ns`/
  `host_interpolated_monotonic_ns`）。一个串口读取批次包含多个样本时，默认按
  `200 Hz` 插值，避免未来 `/imu/data` 在批内重复时间戳；严重落后时在上一批与当前读取边界之间压缩间隔，不把时间推到未来。
- 生产 `h30_imu.yaml` 默认 `host_timestamp_mode: interpolated`、
  `nominal_sample_rate_hz: 200.0`；`receive` 模式仍可用于对照。直接 H2 采集脚本把既有
  `host_monotonic_ns`/`host_wall_ns` 字段改为估计逐样本时间，同时新增原始读取边界字段，保持分析器兼容。
- 这只是主机侧时间线估计，不是 H30 设备采样时刻，也没有证明 LiDAR–IMU 固定延迟。
  2026-09-06 历史 bag 仍按当时的 `host_receive` 结论解释，不能回写成已通过。
- H30 适配器全套测试为 `43 passed`；相关 Python 语法检查通过。现有远端工作树仍有
  大量 staged/unstaged/untracked 改动，未执行提交、推送、清理或重置。
- H2 最新直接证据仍未接受：`h2_formal_direct_dual_20260906` 曾两路各 257 帧且子进程正常，
  但没有带墙角/慢速 yaw 标记、绝对 yaw 锚点和经验证的 H30 时间语义；相位 A/B 的短场景中
  `50 ms` 是候选最佳值，不是硬件同步。之后的 marked 运行出现左路有效率约 `0.41`、较高温度/
  连接超时和 H30 串口错误，而右路仍约 `0.995`，所以左雷达/链路/热状态仍需一变量一项排查。

当前下一步顺序：先在不改设备配置的前提下完成左雷达与 H30 冷却后的只读健康检查；再做带
marker 的双路短采集；只有数据稳定后，才进入最终外参、时间偏移、动态 H3 和负载 H4。外参
标定、任何电机运动和载人测试继续放在最后，并需重新得到用户明确授权。

## 23. 2026-09-07 软件时间戳链与 FAST-LIO 持久化收口

- patches/apply_fastlio_patches.sh 现在锁定基线、权威补丁以及两个修改后 FAST-LIO 源文件的
  SHA-256；在当前嵌套树上幂等通过，并已在临时干净基线验证补丁可应用且哈希一致。
- scan_merger_node 使用主机 monotonic 时钟判断输入新鲜度，合并输出保留最新非零源 LaserScan
  时间戳；三个 pointcloud-to-scan 配置关闭 restamp_output，避免用回调时刻覆盖采集时刻。
- 相关包构建通过（wheelchair_sensors、wheelchair_perception、wheelchair_bringup、fast_lio）；
  22 个测试目录逐目录隔离运行共 331 项通过。一次跨目录合并 pytest 会触发 rclpy 环境级
  segfault，隔离运行无失败，不作为代码回归。
- 本轮只做软件与离线验证；硬件保持停止，未启动雷达/IMU/FAST-LIO/底盘/电机，未写设备配置。
- 文档 docs/fastlio_mapping.md 已将旧的“任务 5 外参完成”和单雷达默认部署改为历史/临时状态；
  最终 LiDAR/IMU/base 外参、真实设备时间/固定延迟、动态门禁和 H3/H4 仍需用户授权后实机完成。

## 24. 2026-09-08 正式 3D 地图软件闭环

本轮只修改代码、文档并做离线验证；未启动 XT-M60、H30、FAST-LIO 实机、
底盘或电机，也未写设备配置。正式产品入口仍因默认
`BLOCKED_CONFLICT` 合同和真实设备时间缺失而 fail-closed。

- 新增合同驱动的 `formal_fast_lio.launch.py`：从 canonical
  `dual_lidar_imu` 合同计算 `T_imu_lidar` 和完整 `body->base_link` 逆变换，
  禁止 FAST-LIO 在线估外参；正式默认 `odom_mode=contract_fastlio`，右主雷达
  负责 LIO，左右雷达仍以严格同步方式共同进入 RTAB-Map。
- 正式传感器自启路径强制 XT-M60 `sdk_epoch` 和 H30 必需设备 timestamp TLV；
  uptime 形态 SDK 时间、缺失接收边界或缺少 H30 设备时间时直接丢帧，不再以
  ROS `now()` 冒充采样时间。生产配置继续如实标记 host time，因此当前时间门
  仍未通过。
- RTAB 优化节点从同一 pose-graph 快照发布
  `/rtabmap/optimized_cloud` 与 `/rtabmap/optimized_path`；导出器要求二者在
  session stop 后更新、stamp 相同、轨迹覆盖采集尾部，并输出真实 XYZ/XYZI、
  完整 6DoF TUM/CSV 和实际使用的校准合同。
- 正式 TF schema 固定为 `map->camera_init`、`camera_init->body`、
  `body->base_link`，启动器、algorithm profile、runtime evidence 和最终
  validator 使用相同边名并要求每边一个 publisher。传感器安装 TF 由同一
  canonical 合同唯一发布。
- validator 逐行检查 ASCII PCD/PLY 的有限/非零几何和有限、非负、非零
  intensity，核对完整轨迹、manifest、profile/合同 installation epoch、合同
  SHA、证据 provenance、动态/实时性/回环/重复性；任何降级 CLI 选项都保持
  `formal_ready=false`。
- `scripts/save_mapping_result.sh` 现在有界暂停 RTAB、停止 session、重试等待
  stop 后新 backend 快照、调用导出，并核对 transient-local completion 路径、
  `.incomplete` 和 manifest。导出瞬间会重读 evidence，并重新核对固定的
  profile/合同/现场身份；writer 再校验 bundle 内实际复制的三份来源及合同 SHA，
  通过后才生成 manifest。错配只留下 `.incomplete`，不会发布完整产品。
- 正式入口要求显式、绝对且尚不存在的 `database_path`，固定
  `delete_db_on_start=false`，同时检查绝对可写 `output_root` 和安全 map name；
  不再覆盖或追加旧 RTAB 数据库。
- 正式 profile、canonical calibration contract 与 hardware evidence 现在执行
  三方硬件身份强绑定：左右 XT-M60 必须以各自厂家 serial、H30 必须以稳定安装
  资产号绑定到同一 installation epoch；evidence 必须逐 role 给出 observed
  ID/model/kind/frame、identity source、`match=true`，雷达另需 observed IP。
  左右调换、重复/缺失 ID/运行 frame、合同摘要改变或仅写 PASS 均拒绝。
  IP 只是辅助配置，不作为唯一身份；当前 H30 资产号仍需操作者最后确认。
- FAST-LIO 自动补丁链已复核：`laser_point_cov=0.001`、正规 ZUPT、退化方向
  投影、初始化、covariance 顺序和有界 path 均在锁定补丁/源码哈希内，旧
  `R=100`/旧 ZUPT 补丁为不可应用 tombstone。

最新身份绑定后在 Orin 重新构建并通过完整包级回归：
wheelchair_3d_mapping `111 passed`、smartwheel_map_products `76 passed`；此前
smartwheel_global_mapping `21 passed`、wheelchair_sensors `46 passed` 继续作为
本轮未改包的最近回归记录，五个相关包此前构建通过。默认正式 launch 冒烟在
构造任何节点前以 `BLOCKED_CONFLICT` 退出，随后进程审计为空。提交、推送和
工作树清理均未执行；用户 staged 的 `docs/goal.md` 保持不动。

余下工作全部需要新的真实证据：H30 稳定资产号/三设备身份现场确认、
XT-M60/H30 设备时钟语义、最终
base/IMU/双雷达外参、直行/转弯/原地转向/STOP 恢复、负载 A/B、至少一个
真实闭环及三次重复路线。详情与顺序以
`docs/FORMAL_3D_MAP_ACCEPTANCE_PLAN.md` 为准。
