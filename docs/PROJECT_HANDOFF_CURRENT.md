# SmartWheel 当前项目交接说明

更新时间：2026-07-29（Asia/Shanghai）

适用对象：新 Codex 会话、其他 AI 工具、后续开发者

当前状态：阶段 B 的硬件基础链路与 RViz 实机工作台已建立，左 XT-M60 低有效点的主要动态根因已定位并完成主机侧串扰缓解；尚未完成真实 FAST-LIO2、累计三维建图或导航验收。

## 0. 2026-07-29 最新停止点

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
- 最近确认 HEAD：`b50c187fc9fb`；
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
6. **PointCloud+Amp 累计产品未实现。** 必须确保 fusion、LIO、registered cloud、map cloud 每一级继续保留 XYZI；若 RTAB-Map 丢 intensity，需另发保强度累计点云；
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

## 14. 右雷达单路 Stage 1 当前交接（2026-07-29）

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
