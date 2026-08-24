# 智能轮椅项目当前记忆

> **最高优先级覆盖状态：`BLOCKED_CONFLICT`。** 本节晚于本文其余所有 Stage 1 可运行描述；发生冲突时必须以本节为准。

## 0. 最新安全覆盖事实：右雷达 Stage 1 永久失败关闭

- 当前设备无人看守、不能移动；本轮不允许启动、探测或清理真实雷达、H30、相机、电机及任何 ROS 硬件链路。
- 左 XT-M60 当前不使用。仅有右侧部署身份（设备 `192.168.1.101`、主机绑定 `192.168.1.100`、UDP `7687`、`eno1`）被记录；该身份只说明部署端点，不是外参。
- `base_link -> xtm60_right_link` 存在跨安装 epoch 且 claim scope 不同的冲突证据：2026-06-23 的 `0.510 m/-1.04°/-9.10°/4.3 mm` 只是部分地平面摘要；初版完整矩阵已由验证结果判为 `REJECTED_BAD`；同 bag 的 `0.499 m/8.2 mm` 结果只完成 ground leveling；2026-07-24 的 `z=0.735 m` 与 provisional RPY 属于不同安装 epoch。它们均 `runtime_eligible=false`，不得拼接或选取为运行时外参。
- 严格合同 `src/wheelchair_bringup/config/right_lidar_stage1_calibration_contract.json` 当前为 `status=BLOCKED_CONFLICT` 且 `runtime_transform=null`。将状态改成 `APPROVED` 或填入 transform 会被启动入口判为 `CONTRACT_TAMPERED`，不是批准流程。
- `right_lidar_stage1_mapping.launch.py` 现在只有一个失败关闭的 `OpaqueFunction`，所有合同分支均失败，不再构造任何硬件或建图 action。`run_right_lidar_stage1_static_acceptance.sh` 固定打印 `BLOCKED_CONFLICT` 并以 `78` 退出。
- 2026-07-29 的静态右雷达 evidence、bag、保存/重载与 PointCloud+Amp 产品继续作为历史事实保留，但它们不授权当前硬件运行，也不证明当前外参、动态建图、回环、FAST-LIO2、导航、地面运动、物理急停闭环或载人能力通过。

更新时间：2026-07-29（Asia/Shanghai）
状态：当前项目事实与工作边界
最新版需求来源：`C:\Users\admin\Downloads\自动轮椅项目概述.md`
来源导出时间：2026-07-22 18:22:51
来源 SHA-256：`3B7F7AE12A10C3FFD9D7CCF5005F3920828C31B3E1DA36E8F0E826BA361CC87E`

## 1. 如何使用这份记忆

这份文件用于替代项目创建初期遗留的默认认知。根目录 README、四五月份的 `.kiro/specs`、旧执行方案及历史 AI 输出可以作为背景资料，但若与本文件或用户后续指令冲突，不得作为当前事实。

原始概述是聊天历史导出，内部也包含前后版本。解析原则是：较晚的用户确认覆盖较早描述；用户提供的设备型号和阶段决定覆盖助手猜测；远端仓库的运行证据用于补充实际实施状态，但不能把 mock 结果解释为硬件验证。

新对话必须先读取根目录 `AGENTS.md` 和本文件，然后运行 `tools/project_startup_check.ps1`，自动通过 SSH 别名 `orin` 做只读状态刷新。若安全沙箱要求网络授权，应立即申请；若连接失败，应明确说明只能使用本文件的时间戳快照。启动检查不得执行 `fetch`、`pull`、`checkout`、`reset`、`clean`、`stash`、提交或推送。

### 1.1 2026-07-29 左 XT-M60 有效点问题与双机串扰缓解

用户明确要求先定位并尝试解决左 XT-M60 有效点比例偏低的问题，同时将
LiDAR–IMU 外参和载人测试留到后续单独处理；当前不允许载人。新的只读
`distData/rawdistData/amplData/PointCloud2` 诊断表明，历史写入第三 HDR
曝光 `30 us` 后记录的左侧 `60.54%` 并不是当前环境下的永久上限：单机当前
约 `90.7%`，右侧约 `97.5%`。左侧原始深度和处理后深度的有效比例近似，
所以剩余差距不是 ROS 转换或额外 SDK 滤波造成的。

同场景“左单机→双机→左单机”A/B 找到了主要动态根因：双机同时测量时，
左侧逐帧距离变化中位数由 `23 mm` 升到 `143 mm`、P95 由约 `85 mm`
升到 `1646 mm`，停止右侧后立即恢复；右侧单机 ROS 对照也由
`11/51 mm`（中位数/P95）恶化到未正确错峰时的 `38/793 mm`。这证明两台
未硬件同步、使用相同成像/调制配置的 Flash ToF 雷达存在明显光学串扰，
弱信号的左设备更容易受影响。左侧 `miniAmp 70→50→70` 的序列号门禁
A/B 只提高约 `1.3` 个百分点且略增抖动，因此已否决并恢复 `70`。

当前生产缓解不写设备配置：两路 SDK `start()` 对齐到共同的 `100 ms`
主机单调时钟网格，左 offset `0 ms`、右 offset `50 ms`，每 `30 s`
短暂停止并重新按网格启动以限制独立设备时钟漂移。
`enable_sdk_filters=false`、`apply_device_config=false` 保持不变。最终
65 秒双路 ROS 验证为：

- 左 `9.867 Hz`，平均有效点 `91.72%`，逐帧距离变化中位数/P95
  `30/131 mm`；
- 右 `9.839 Hz`，平均有效点 `97.11%`，逐帧距离变化中位数/P95
  `16/67 mm`；
- 左侧各持续 10 秒分段均约 `91.5%～92.0%`，但出现过一次瞬时
  `63.0%` 帧；
- 两路仍发布有组织 `160×60` XYZI，幅值未在存储点云中裁剪；
- `test_xtm60_adapter.py` `13/13` 通过，`wheelchair_sensors` 和
  `wheelchair_bringup` 构建通过。

这是主机侧串扰缓解，不等于硬件触发同步或时间戳同源；每 30 秒重对齐约有
1～2 帧空窗，左侧仍比单机更抖。C2 首轮 FAST-LIO2 应优先只使用右主雷达；
双雷达累计建图前还应增加坏帧质量门禁，并向厂家确认正式多机同步/触发或
安全调制频率分配方案。完整报告为
`docs/hardware/XT_M60_DUAL_INTERFERENCE_MITIGATION_20260729.md`。
最终已验证 transient service `inactive`、无 `xtm60_adapter_node`、两接口
三秒均无 UDP 帧，雷达没有留在测量状态。

## 2. 项目定位与一期目标

项目定位是基于 NVIDIA Jetson AGX Orin 64GB、Ubuntu 22.04 和 ROS 2 Humble 的室内低速自主导航与安全辅助轮椅原型。

目标工作流：

1. 人工推动轮椅，或通过 RViz 中的 WSAD 面板低速遥控轮椅绕室内环境行驶。
2. 融合窄视场雷达、IMU、轮式里程计及后续相机信息，建立清晰的三维点云地图。
3. 同时生成可用于后续导航的二维占据栅格地图。
4. 保存、版本化、重新加载并预览地图及轨迹。
5. 后续阶段再完成地图定位、目标点导航、局部避障、故障停车、人工接管和物理急停闭环。

一期边界是固定或半固定室内环境、低速、可接管、少量目标点。不得把它描述成已具备任意环境完全自主载人能力。

## 3. 用户确认的硬件现状

- 车载计算：NVIDIA Jetson AGX Orin 64GB。
- 激光雷达：2 台 XT-M60 窄视场三维雷达，左/右 IP 为 `192.168.0.101`、`192.168.1.101`，网关分别为 `192.168.0.1`、`192.168.1.1`，掩码均为 `/24`。两台已按 Windows 上位机导出文件 `xintan_windows_export_20260722.xtcfg`（SHA-256 `1611F653AA13B35AEA38DE844A130840342A73CA8AA1DB47910B7C8588E0D9F4`）应用同一组受 SDK 支持的成像/滤波参数；导出中的积分时间为 `1600/200/20/1600 us`，HDR 开启、最小幅值 70、最大帧率 10 Hz。双路后测左约 `9.976 Hz`、右约 `10.036 Hz`，话题/frame 分离正确。用户转述厂家意见为“说把SDK实例化一份就可以实现双雷达同步进行”；当前实现为每台雷达各自独立的 SDK 实例，只证明并发共存，不等同于硬件时间同步。右雷达 SN `XTM60B20250324000134` 的强度仍可达到 `2157` 左右，超过资料明确的 `0～2039`；同配置后仍存在，不能归因于旧积分时间。SIGINT 段错误已定位为 ROS Humble `ExternalShutdownException` 未进入 SDK 硬退出保护，现已修复，左右节点最终退出码均为 0。
- 左 XT-M60 B1：SN `XTM60B20250324000151`，固件 `XTFW-RT-V2.34.2`，boot `BT-N1066-V0.28`；原始帧 `160×60`、9600 点、约 10 Hz。最终 ROS 有序 XYZI 点云与同一 bag 回放均通过，无效强度 `>=64000` 已转为 NaN；B1 只证明左单雷达基础网络/SDK/原始点云链路，不代表双雷达、外参、FAST-LIO2 或建图效果通过。
- 雷达方向/TF：2026-07-24 核查发现现分支注释所称的 `ground_plane_calibrator_node` 并不存在，运行时也没有该可执行文件，导致原本预期由它发布的 `base_link -> xtm60_*_link` 缺失。XT-M60 SDK 实测/代码坐标为 `+z` 前、`+x` 左、`+y` 上。右雷达在水平、静止现场的 30 帧重复地面拟合得到法向 `[-0.026524, 0.990374, -0.135851]`、原点到地面约 `0.754 m`、残差均值约 `12.17 mm`，对应约 `7.81°` 俯仰和 `1.53°` 横滚安装偏差；已生成并写入可恢复的右侧固定 TF，RPY 为 `[1.7071165220, 0.0265268546, 1.5744345976] rad`。水平 yaw 没有由地面确定，当前明确约束为 0；右侧 xyz `[0.45,-0.30,0.735] m` 中横向/高度来自用户测量，前后向 `0.45 m` 仍是旧估计。左侧当前地面拟合两次都选中非地面上方表面，不能用于新标定，因此暂时采用旧分支 `45b1163` 的历史可靠旋转 `[1.5515153364,-0.0136154207,1.5709313394] rad`，xyz `[0.45,0.30,0.735] m` 同样含未核实前后向估计。两条 TF 已通过 `robot_state_publisher` 运行验证且没有重复发布；这只纠正显示/坐标方向，不是最终双雷达外参或融合验收。
- IMU：H30，`/dev/smartwheel_h30_imu -> /dev/ttyACM0`、460800 baud、约 200 Hz，没有设备时间戳 TLV。用户随后确认底盘水平、设备静止且附近无人干扰；新的 60 秒/11999 帧静态测量得到设备 roll/pitch `+0.571160/+0.064734 deg`，由原始重力独立反算得到 `+0.567121/+0.063040 deg`，两者高度一致。现以可恢复的 `base_link -> imu_link` 静态 TF 应用 RPY `[0.0098981264, 0.0011002597, 0] rad`；H30 样本仍保持物理传感器坐标，未发送持久姿态复位。陀螺均值约 `[-0.000540,-0.000681,+0.000069] rad/s`，姿态标准差约 `0.010/0.007 deg`，不符合官方手册所说“波动较大”才需陀螺零偏写入的条件，因此未写零偏。磁校准需要受控转动，也未进行。动态轴向、yaw 与 xyz 外参仍未标定。
- 轮式信息：电机编码器/里程计链路是建图输入之一。2026-07-25 用户明确表示电机已接入，并随后确认控制器适配器已接好并上电；这覆盖了先前“控制器/电机侧未接入或未上电”的历史状态。预期 KeepLINK/USB-RS485 适配器已正常枚举为 `1a86:55d3`、序列号 `5C66036979`，`/dev/smartwheel_zlac8030 -> /dev/ttyACM1`，端口无人占用且 udev 规则正确。严格只读 Modbus `0x03` 在历史设置 115200 baud、slave 1、`0x20AB/0x20AC` 上 50 次和留档复测 25 次仍全部只有本地请求回显、控制器返回 0 字节；另对厂家支持范围内的 `9600/19200/38400/57600/115200/128000` baud 与 slave 1～4 做 24 组有界只读扫描，也全部无控制器响应。用户再次确信接线无误后，又以 1 Hz、1.0 秒超时重复 6 次，结果仍相同，排除了原 0.2 秒超时过短。旧分支中没有找到可归档的 ZLAC 原始返回帧或轮速采样，只有代码、配置注释和历史校准声明，不能替代当前有效回包。当前阻塞已收窄到控制器逻辑电源、RS485 A/B/公共地、误接 CAN/485 接口、连接器接触或其他物理通信链；不能据此判定编码器故障。正常 ZLAC 节点关闭路径会写控制字，因此仍未启动，且未发送任何使能、速度、停止或急停写指令。
- 2026-07-26 再次复测时，四相机均以 `5000M` 在线；ZLAC 稳定别名改为当前枚举的 `/dev/ttyACM0`，但 `ID_PATH` 仍为 `platform-3610000.usb-usb-0:1.1:1.0`，位于外置 Genesys 拓展坞下，所以这不是直连 Orin 的 A/B 测试。10 秒、20 次、115200/slave 1/0.5 秒超时的只读 `0x03` 复测仍全部只有本地回显，控制器响应为 0；期间没有新增 USB/UVC 内核事件。下一项有区分力的测试仍是把 `1a86:55d3` 适配器移到 Orin 直连接口，先确认 `ID_PATH` 改变，再重复同一只读请求。
- 随后用户因 RS485 适配器外壳较宽而拔掉右侧和右前相机，将该适配器改插 Orin 直连接口；`ID_PATH` 已从外置拓展坞的 `...0:1.1:1.0` 明确变为直连接口 USB2 companion 路径 `...0:4.3:1.0`。左前/左侧相机 `2-3.1/2-3.2` 仍以 `5000M` 在线，右侧/右前 `2-3.3/2-3.4` 为用户主动拔除。直连条件下同样进行 10 秒、20 次、115200/slave 1/0.5 秒超时的只读 `0x03` 测试，仍全部只有精确本地回显、控制器返回 0 字节，且没有新增 USB/UVC 内核事件。这一 A/B 结果排除了外置拓展坞是控制器静默的唯一原因，故障域仍在适配器之后的 RS485/控制器物理链路；在出现有效读帧前不得发送控制写入。
- 相机：用户确认四台物理相机均已插入，2026-07-23 最终角色为 `2-3.1=左前`、`2-3.2=左侧`、`2-3.3=右侧`、`2-3.4=右前`。2026-07-22 经外置 USB hub 时系统最初只枚举到 3 台，负载后最终只剩两台。2026-07-23 移除外置 hub、四台直插 Orin 后，早期会话仍一度只有三台，并出现损坏 JPEG、UVC `-71/-110`、USB3 reset/断开/重连以及 `2-3.3` 回退为 USB2 `1-4.3/480M`。用户随后按物理接口重新插拔，13:34:59 成功同时枚举四个 SuperSpeed `5000M` 路径：`2-3.1→video6`、`2-3.2→video4`、`2-3.3→video2`、`2-3.4→video0`。按用户明确指定的上一分支 `feature/fastlio-narrow-fov-mapping` 最新提交 `45b1163` 核对，其四机配置只是普通 `cv2.VideoCapture`，使用 `/dev/video0/2/4/6`、MJPG、`640×480@30`，与当前提交基线的读取实现相同，没有特殊枚举逻辑。以同样参数复测时四台逐台和 15 秒并发均能读取不同真实画面、无读帧失败；实际并发率约 `14.664/18.541/14.797/14.801 Hz`，因此“请求 30 fps”指标未通过，但四机读取本身通过。生产 `camera_quad.yaml` 已固定上述四个 `/dev/v4l/by-path`，同时为兼容现有软件保留话题槽位映射：`left=左前`、`front=左侧`、`rear=右侧`、`right=右前`；这里的 `front/rear` 只是旧槽位名，不代表实体正前/正后相机。根据已保存画面暂定物理旋转为左前 `180°`、左侧 `180°`、右侧 `270°`、右前 `180°`，仍需用户在带标签画面中目视确认，不能当成标定结果。
  四路 raw RViz 画面可以同时显示，但 15 秒诊断只有每路 8～9 帧，约 `0.67～0.73 Hz`，正常操作台应选 compressed transport。最终 30 分钟四路 compressed soak 全部通过：左前 `/camera/left` 16762 条、`9.312 Hz`；左侧 `/camera/front` 17001 条、`9.444 Hz`；右侧 `/camera/rear` 16888 条、`9.382 Hz`；右前 `/camera/right` 16029 条、`8.912 Hz`。四路均非空、分辨率/旋转后形状正确、时间戳严格递增、覆盖完整测试窗口；测试后四台仍为 `5000M`，内核没有新增 USB reset/disconnect。节点日志仍有 1493 行未标注来源的 libjpeg 警告，其中 359 行为 premature-end、2 行为 bad-Huffman；警告行数不能直接等同于坏帧数。随后以每台独立进程、独立 stderr 且四台并发约 2 分钟归因，左前/左侧/右侧/右前告警数为 `0/0/0/81`；右前单机 71 秒仍有 20 行告警（4 行 premature-end），同时无内核 USB/UVC reset/disconnect。因此告警不依赖四机并发，已定位到“右前相机 + 线缆 + `2-3.4` 接口”整条安装链，但尚不能判定具体是相机本体、线缆还是接口，需单变量交换复测。当前结论为“四机生产路径、真实画面、并发读取和 30 分钟 ROS compressed 稳定性通过，但右前 MJPEG 质量告警和早期链路不稳定风险仍需保留”。四台序列号相同导致 `/dev/v4l/by-id` 冲突。系统全局 `uvcvideo quirks=128` 和 USB autosuspend `2 s/auto` 本轮未修改，不能认定为原因。前向相机位于同侧雷达外侧约 5 cm、参考高度约 73.5 cm 仍只是用户测量，光心、内参和完整外参未标定。
  为改善 RViz 卡顿，生产路径现改为 `320×240` MJPEG、每台独立进程、GStreamer 采集线程与“最新帧”发布线程解耦，操作台 Camera Panel 直接订阅 compressed 并在 JPEG 解码前限到 10 Hz；CameraInfo QoS 改为 best-effort/volatile。右前链因损坏 JPEG 单独增加 `jpegdec -> jpegenc quality=40`。最终约 2 分钟长测中左前/左侧/右侧约 `8.096～8.175 Hz`，右前仅 `4.925 Hz`，进程均存活、四路仍为 `5000M` 且无新增 USB/UVC 复位；因此软件显示链已减负，但右前安装链仍不满足平滑四画面目标，不能把约 5 Hz 解释成软件已经解决。
  2026-07-25 电机接入复查初期，`2-3.2` 左侧相机未枚举；随后四个 USB3 设备都重新出现，但共享 USB2 hub 记录了 `disabled by hub (EMI?)` 并重枚举 H30，`2-3.1` 又发生断开、UVC `-110/-71` 和 SuperSpeed reset。最终检查时 `2-3.1` 的 USB 设备可见但视频驱动/by-path 尚未恢复。不能据此证明电机控制器造成 USB 故障，但全系统上电稳定性未通过；恢复测试前必须重新确认 H30 和四相机节点、by-path、`5000M` 及内核错误计数。
- 近距传感器：4 个 FD07-34R，地址 1～4 已完成真实 RS485 轮询与 ROS 发布。原实现约 `1.406 Hz`，原因是约 94 ms 响应后又额外等待 110 ms；现已改为严格按请求起点调度，响应时间计入间隔并覆盖轮询周期边界。20 秒复测 160/160 次只读响应有效，最短请求起点间隔 `110.143 ms`；ROS 四路约 `1.99969～1.99985 Hz`，时间戳严格递增，达到配置 2 Hz。当前未见串口丢包或 CRC 错误，但未测供电电压、电流及全系统负载余量，不能把结果解释成电源适配器认证。
- 底盘：电动轮椅双电机底盘及电机驱动/通信链路；2026-07-22 用户明确电机暂未接入，所有测试必须继续避开电机。
- 安全：需要物理急停、驱动器 watchdog、安全监督、通信异常停车和人工接管，但尚未形成真实硬件验收证据。

硬件数量以“两雷达、四 FD07-34R、四相机”为用户确认的物理配置；四相机现已直插 Orin，并以 `2-3.1`～`2-3.4` 四个 SuperSpeed by-path 完成 30 分钟四路 ROS compressed 测试。仍不得把这解释为相机标定、视觉定位、同步或安全功能通过。早期“只剩两台相机”、单雷达降配、六个传感器或右雷达送修等描述均为历史状态。

用户测得双雷达参考点横向间距约 60 cm；雷达高度使用上下两光窗总长约 37 mm 的中点作为参考，约 73.5 cm。XT-M60 手册确认下窗为 VCSEL 发射、上窗为接收，但未明确 SDK 坐标原点。一次左侧 B1 地面拟合估计 SDK 原点到地面约 78.8 cm，2026-07-24 右侧重复拟合约 75.4 cm，均与用户参考点口径不同。当前 TF 采用用户测量的 y/z 和明确标注的前后向旧估计，只作为方向显示与后续标定起点；最终结构固定后仍须做几何/平面/靶标外参标定，不能视为最终 TF。

## 4. FD07-34R 与机械设计注意事项

FD07-34R 不属于第一阶段建图主输入，但已完成四地址只读实机检测。`docs\传感器说明文档.pdf`（SHA-256 `76BB13744956417F7A9C42A3D2FB00434E96B3D291A58E766E18BC0C876A1CBD`）是当前官方资料来源。

手册确认 FD07-34R 是大角度 RS485 输出型号：3～300 cm、盲区 3 cm、9600 bps、地址 `0x01～0xFC`、广播 `0xFD`，大角度选项约 `80°±10°`。实际值寄存器 `0x0001` 的相邻操作必须严格大于 100 ms，处理值 `0x0002` 必须大于 300 ms。四地址 1～4 的初次 30 秒实测完成 60 轮、240/240 次 CRC/地址/功能/载荷均有效；修正请求起点调度后的 20 秒复测又完成 40 轮、160/160 次有效事务，最短间隔 `0.110142974 s`。ROS 四话题现稳定约 2 Hz；无目标编码、串扰与角度口径仍待验证。旧代码的约 0.45 rad 不能再当官方规格。

已知实测现象：

- 官方商品资料标称测距约 3～300 cm。
- 传感器装入旧盒体时，在前方无明显障碍物的情况下约返回 700 mm。
- 取出并在空旷处向上测量时可达到约 3000 mm。
- 用户确认两个探头完整露出，盒体定位在传感器第二级较厚八边形底座处。

不能据此直接认定传感器故障或单位错误。后续应使用官方资料和隔离实验核查声束、近场/盲区、盒体边缘或腔体反射、结构传振、多径、安装姿态、周围地面/扶手反射及多传感器串扰。不要把代码中曾出现的约 0.45 rad 视场角当作官方规格。

旧传感器盒可能影响雷达或 FD07-34R 数据，机械结构正在重新设计。真实外参必须在最终安装结构固定后测量，不得沿用旧盒体的猜测参数。

## 5. 当前算法与数据流基线

计划中的工程组合为：

- FAST-LIO2：窄视场雷达与 H30 IMU 的主要 LIO/里程计候选；
- RTAB-Map：全局三维图、回环优化、地图产品与后续彩色点云基线；
- `slam_toolbox`：可选二维栅格后端；
- 轮式里程计：用于低速运动约束和冗余，但需要真实时间同步、尺度和协方差验证；
- 相机：四路显示已纳入操作台，真实视觉回环、着色或语义功能留待单相机接入后评估。

期望链路为：

```text
双 XT-M60 + H30 + 轮式里程计
  -> 时间同步、单位与坐标校验
  -> LIO / 融合里程计
  -> RTAB-Map 全局优化和三维地图
  -> 二维占据栅格
  -> 地图版本、轨迹、质量报告和 rosbag
```

FAST-LIO2、真实彩色建图和上述融合效果尚未经过硬件验证。不得用 Stage A 的模拟 RMSE 推断真实精度。

## 6. RViz-first 操作台需求

主界面是 RViz2：

- 中央原生三维 RenderPanel 始终存在，不作为可关闭 Dock Panel。
- 二维 OccupancyGrid Panel 默认位于右侧，可缩放、平移、调整大小、浮动、关闭并从 Panels 菜单重新添加。
- 4 个可独立配置的 Camera Panel，默认标签化，可移动、关闭、重开或排成 2×2。
- Teleop Panel 支持键盘和鼠标 WSAD、组合键、Space/STOP、释放停车、失焦停车、关闭停车和命令超时。
- Mapping Control Panel 必须调用真实 service/action 状态，不能只改变按钮颜色。
- System Status Panel 使用 diagnostics 和后端权威状态，不能由 GUI 猜测设备在线。
- Map Products Panel 用于选择、预览和重新发布三维、二维、轨迹与质量报告，不直接删除地图。

安全命令链保持：

```text
/teleop/cmd_vel -> safety_supervisor -> /cmd_vel_safe -> mock/real base
```

真实电机接入后也不得绕过 safety supervisor。

## 7. 已完成的软件状态

### Stage A 与 A-R

- Stage A 完成无硬件建图架构。
- A-R 审查分支为 `review/mapping-v2-stage-a`，验收提交 `27bd875`，审查基线 `1db393a`。
- A-R 修复了 mock LIO 直接使用 ground truth、非优化地图冒充全局地图、`/map` 竞争、TF 时间戳、WSAD watchdog、离线导出和门禁不足等问题。
- A-R 门禁为 `PROCEED`，含义仅为在用户明确批准后可进入 B0 文档审计。

### RViz workbench

远端已完成需求中描述的 mock RViz 操作台，不能再把它记为“尚未开始”。

- 分支：`feature/mapping-v2-rviz-workbench`。
- 2026-07-22 核对 HEAD：`b50c187fc9fb`（`style(workbench): normalize file endings`）。
- 基线：A-R 的 `27bd875`。
- 30 个包清洁构建成功。
- 项目自有 28 个包共 225 项测试：0 error、0 failure、0 skip。
- 完整 `colcon test` 仍有未修改第三方 `livox_ros_driver2` 的上游 lint 失败；不得通过修改/禁用第三方测试伪造全绿。
- 25 项实际运行验收全部 PASS，包括中央三维、二维地图、四路不同模拟相机、Panel 浮动/关闭/重开、WSAD 停车语义、真实 mock service、保存/重载及干净退出。
- 完成 30 分钟 mock 活动建图稳定性运行；RTAB-Map 内存随关键帧和数据库增长，是后续真实路线需要建立预算的已知限制。
- 最终门禁：`RVIZ WORKBENCH PROCEED`，只表示 mock 操作台任务通过。
- 最终验证地图：`maps/versions/rviz_workbench_final_verified_20260715_183757_216366`。
- 当前所有产品必须保留 `hardware_validated=false`。

远端权威报告：

- `docs/STAGE_A_REVIEW_REPORT.md`
- `docs/STAGE_A_ACCEPTANCE.md`
- `docs/RVIZ_WORKBENCH_IMPLEMENTATION.md`
- `docs/RVIZ_WORKBENCH_TEST_REPORT.md`
- `docs/RVIZ_WORKBENCH_KNOWN_LIMITATIONS.md`
- `docs/RVIZ_WORKBENCH_USER_GUIDE.md`

## 8. 远端代码位置与工作树注意事项

- SSH 别名：`orin`。
- 地址：`nvidia@192.168.5.10:22`。
- 仓库：`/home/nvidia/smartwheel`。
- Git remote：`https://github.com/owenkings/smartwheel.git`。
- 当前跟踪：`feature/mapping-v2-rviz-workbench...origin/feature/mapping-v2-rviz-workbench`。
- 2026-07-22 工作树存在用户原有的暂存新增文件 `docs/goal.md`。除非用户明确要求，必须保留，不编辑、不取消暂存、不提交、不丢弃。
- 同日已按用户要求把 B0、B1、B2 以及随后所有非电机检测的每组结果分别暂存，未提交；包括左右/双雷达、H30 复测、FD07-34R、相机和严格只读的编码器链路证据。新对话必须先用 `git status --short` 复核，不得把这些结果与 `docs/goal.md` 擅自提交。

GitHub 地址：

- 仓库：<https://github.com/owenkings/smartwheel>
- 分支页：<https://github.com/owenkings/smartwheel/branches>

2026-07-22 通过 `git ls-remote --heads origin` 核对到 5 个实时分支头：

| 最近提交时间 | 分支 | 提交 | 当前用途判断 |
| --- | --- | --- | --- |
| 2026-07-15 | `feature/mapping-v2-rviz-workbench` | `b50c187fc9fb` | 当前最新、主工作分支 |
| 2026-06-24 | `feature/fastlio-narrow-fov-mapping` | `45b1163fbf35` | 旧 FAST-LIO 窄视场实验线 |
| 2026-06-22 | `feature/rviz-first-mapping-mvp` | `83c0e9a41358` | 旧 RViz-first MVP |
| 2026-06-10 | `feature/livo-wheel-3d-slam` | `066b7160c053` | 更早的 3D SLAM 保存分支 |
| 2026-05-30 | `main` | `13bdb1f33cad` | GitHub 默认分支，但不是最新开发线 |

“以最新分支为主”表示优先使用最近且目的与当前阶段一致、证据完整的开发分支，并不等于永远使用 `main`，也不允许新对话看到一个新名字后自动切换。启动时先核对实时 heads；若出现更新的提交或分支，先检查其用途、报告和工作树，再更新本记忆。

本 Windows 工作区 `C:\Users\admin\Desktop\智能轮椅` 不是 Git 仓库，主要包含旧文档、工具、传感器资料和机械设计资产。代码实施以 Orin 仓库为准。

## 9. 当前阶段门禁与下一步

用户已于 2026-07-22 先授权 B0、B1、B2，随后明确要求检测所有剩余设备但排除电机。执行结果如下：

1. **B0 已完成并暂存。** 已审计 XT-M60 SDK/网络资料、2025-09 XT-M60 产品手册、Windows 上位机导出、H30 官方协议/标定资料和 FD07-34R 手册；随后 B1/B2 及非电机实机调试均已开展，B0 中“仅文档、尚未打开设备”的限制已被后续受控实测取代。
2. **B1 左单雷达基础链路通过并已暂存。** 第一次网络失败的历史保留；恢复后左雷达 20/20 ping，设备身份/保存配置读取成功，20 帧原始 SDK 数据和 20 秒 ROS 数据均约 10 Hz。SDK XYZ 已是米，raw distance 表现为毫米单位；SDK 时间约 5802 秒且非 Unix epoch，因此 ROS 验收使用 host timestamp。最终过滤后实采 200 帧约 9.95 Hz、同一 bag 回放 213 帧约 10.02 Hz，九项自动检查全通过，强度范围不再包含 `>=64000` 无效哨兵。最终 bag 为 `/home/nvidia/smartwheel/bags/hardware/b1_xtm60_left_filtered_20260722_200403`。常规左雷达配置默认 `apply_device_config=false`，只在显式操作时才允许写成像参数。
3. **B2 H30 静态链路与水平安装角校准通过并已暂存。** 初始两次 30 秒链路约 200 Hz；用户确认底盘水平、静止且无附近干扰后，新 60 秒读取 11999 帧、199.966 Hz。重力反算的 `base_link -> imu_link` roll/pitch 为 `+0.567121/+0.063040 deg`，已以静态 TF 写入当前实机 URDF，并由设备 Euler 值交叉确认。Xacro 和 `check_urdf` 通过。未做 H30 持久姿态复位、陀螺零偏或磁校准：前者串口帧未由厂家公开，后两者在当前低波动/无受控转动条件下不适用。动态轴向、yaw、xyz 和设备时间戳仍待处理。
4. **右雷达、双雷达基础共存与临时方向 TF 通过但有限制，结果已暂存。** 两台均以各自独立 SDK 实例按对应上位机导出配置并发运行，双路约 10 Hz，话题/frame 分离正确；未因这个方案通过而再测单实例管理双雷达。右侧同配置后仍出现约 `2157` 的未解释幅值，超过资料明确的 `0～2039`。SIGINT 段错误已修复，最终左右均退出码 0。右雷达地面法向给出的约 `7.81°` 俯仰/`1.53°` 横滚已写入固定 TF，运行时 TF 验证通过；yaw 与 x 仍是约束/估计，左侧沿用旧分支可靠旋转。尚未验证硬件同步、最终外参、融合、FAST-LIO2 或建图。
5. **四个 FD07-34R 实机只读检测与 2 Hz 修正通过并已暂存。** 初次 240/240、修正后复测 160/160 次响应有效，全程只有功能码 `0x03`；最短请求起点间隔 `110.143 ms`。轮询现按请求起点调度，不再把约 94 ms 响应和 110 ms 休眠重复相加；ROS 四话题约 `1.99969～1.99985 Hz`。
6. **相机软件链路、角色固定、四机 30 分钟稳定性与 RViz 减负已完成但有限制，结果已分组暂存。** 四台现可同时以 `5000M` 枚举，角色为 `2-3.1=左前`、`2-3.2=左侧`、`2-3.3=右侧`、`2-3.4=右前`；生产 YAML 固定四个 by-path。旧分支同参数的四路真实并发读取通过，但 30 fps 请求没有达到。较早四路 ROS compressed 30 分钟稳定在 `8.912～9.444 Hz`，全部持续窗口检查通过且无 USB reset/disconnect。当前操作台已直接使用 compressed、解码前 10 Hz 限流和独立采集线程；最终长测三路约 `8.1 Hz`，右前仍只有 `4.925 Hz`。旋转值仍需用户目视确认；libjpeg 告警和低帧率都定位在右前相机/线缆/`2-3.4` 安装链，仍需一次只改变相机本体、线缆或接口中的一个变量做 A/B 交换。
7. **电机已接入，ZLAC 静止只读通信门槛现已通过，结果已暂存。** 此前 115200/slave 1 的多轮复测和波特率/地址扫描均只有本地回显；外置拓展坞与 Orin 直连 A/B 也同样失败，排除了拓展坞是唯一原因。用户随后再次进行物理调整，但未说明唯一改变的电气变量；适配器回到拓展坞路径、四相机恢复 `5000M` 后，10 秒慢速复测获得 5/5 组有效左右反馈，15 秒较高频确认获得 38/38 组，均无失败、无回显，左右静止值始终为 `0 raw / 0.0 rpm`，且无新增 USB/UVC 内核事件。没有启动正常驱动，也没有发送任何写寄存器或运动命令。该结果只验证静止通信和 `0x20AB/0x20AC` 读取；动态方向、非零缩放、轮径/轮距、极性、watchdog 与运动安全均未验证。任何控制写入前仍必须由用户明确确认双轮架空、无人乘坐、场地清空且物理急停可用。
8. **用户已明确确认运动安全门槛，最低档左轮点动已安全结束但没有可测转速。** 用户确认双轮架空、无人乘坐、场地清空且物理急停可用后，先写双轮零目标与软件急停，控制器返回 `mode=3/control=5/left_rpm=0/right_rpm=0`。随后按项目既有受控脚本初始化 `async=0`、清故障、使能，只向左目标寄存器写 `+5`、右侧写 `0`，持续 0.75 秒；左转矩从 `0` 变为 `3`，但左右速度反馈均保持 `0`。退出路径已写双轮零目标并锁存软件急停 `control=5`；事后只读复核 5/5 组反馈均为零，且无新增 USB/UVC 内核事件。不能由转矩变化推断物理转动，也不能自动升速；下一试次若继续，应仍保持左轮单变量和有界目标，获得动态反馈后再测右轮。
9. **用户报告 `+5` 无可见变化并要求更明显后，左轮 raw `+20`/1.0 秒试验获得首次非零动态反馈。** 该目标仍低于历史 raw `30`/2 秒试验，右轮保持 `0`；使能后左转矩为 `2`，指令期间左速度 raw `+1`、左转矩 `16`，右速度/转矩均为 `0`。退出时双轮速度已回零且 `control=5`；事后只读复核 5/5 组全为零，无新增 USB/UVC 内核事件。物理是否转动及方向仍待用户目视确认，单次 raw `+1` 反馈也不能完成 `0.112 rpm/raw` 动态缩放标定。
10. **用户仍未看出变化并要求扩大观察范围后，左轮 raw `+30`/2.0 秒历史边界试验已安全结束。** 右轮目标仍为 `0`，未超过旧硬件反馈测试使用过的 raw `30`/2 秒。左转矩升到 `26`，软件急停刚锁存时捕获左速度 raw `+2`，右速度保持 `0`；随后 5/5 组只读反馈均回零，无新增 USB/UVC 内核事件。该瞬时非零反馈与采样/惯性窗口一致，但物理是否转动及方向仍必须由用户目视确认。
11. **RViz 实机手动控制验收完成，用户确认各按钮和方向均正常。** 在用户再次确认双轮架空、无人乘坐、场地清空且物理急停可用后，真实命令链 `/teleop/cmd_vel -> safety_supervisor -> /cmd_vel_safe -> ZLAC` 运行约 254 秒；bag 为 `/home/nvidia/smartwheel/bags/hardware/zlac_rviz_manual_20260726`，含 43130 条消息。输入峰值 `0.15 m/s`、`0.25 rad/s`，安全输出限为 `0.08 m/s`、`0.20 rad/s`，底盘反馈出现非零速度。用户明确反馈“可正常使用，测试完成，并且各个按钮均无误”。测试后所有运动相关单元停机，最后严格只读 3/3 组左右反馈均为零、无 Modbus 写入。
12. **实机 RViz 的长按、四相机、三维和二维显示问题已修复并分步暂存。** 录包证明长按碎片在 `/teleop/cmd_vel` 输入端已存在，原因是远程桌面把持键合成为重复 press/release；现以 120 ms、带 generation guard 的释放去抖保持连续命令，STOP、失焦、隐藏/关闭和安全超时仍独立立即归零，Qt 回归测试通过。`manual_teleop.launch.py` 现默认双 XT-M60、四相机 `camera_quad.yaml` 分进程、compressed Camera Panel、H30、四 FD07-34R、双点云投影、合并 `/scan`、安全链和新的 `hardware_operator_workbench.rviz`；安全默认仍是 `motion_control_enabled=false`。最终布局为左侧“左前、左侧、Teleop”，右侧“右前、右侧、2D Map”；Displays、Views、System Status 与 Teleop 合并为标签页。二维面板把画布置于首位，状态、尺寸、缩放、映射状态和“未保存的实时预览”置于底部，话题设置默认折叠；默认 Auto-fit 会在地图扩展时保持全图可见，关闭后可手动滚轮缩放和拖拽平移，Dock 仍可缩放、浮动和关闭。12 秒四路 compressed 验收全部通过，约 `21.91～23.44 Hz`，RViz 解码仍限 10 Hz。插件现为 27 项测试全部通过。
13. **二维预览链修复并通过，但不等于建图验收。** 点云投影和 scan merger 原用 `ceil(span/increment)+1`，使 `/scan` 对声明角范围输出 363 束而 `slam_toolbox` 期望 362 束；现采用不越过最大角的完整增量数，并发布实际最后一束角度。感知包 10 项测试通过；运行时 `/scan` 约 `9～10 Hz`，`angle_min/max/increment=-1.5707999468/+1.5699000359/0.0087000001 rad`。为避免传感器/EKF TF 尚未稳定时先塞满消息过滤队列，`slam_toolbox` 在工作台中延后 8 秒启动；最终重启后二维 `/map` 从 `47×69` 增长到 `50×89、0.05 m/cell`，并在额外 45 秒渲染检查后仍与四相机和双雷达点云同屏。这是静止、未保存的 UI/管线预览；FAST-LIO2、双雷达融合、累积三维地图、二维地图质量、回环、持久化和导航均未验收。中央三维仍是两路当前帧原始点云，不是参考视频中的累计建图画面。此前发黄是无有效编码器反馈时 Wheel Odometry 巨大协方差的 RViz 可视化，不是点云或地图颜色；该无效显示现默认关闭，模型、TF、网格和双点云保留。
14. **已确认并采用 XT-M60 PointCloud+Amp 数据语义。** 用户提供的 XT-Toffuture V2.10.6、两个 685 字节导出配置和 SDK 证据一致：`imgType=4` 即 `IMG_POINTCLOUDAMP`，回调同时给出有组织的 undistorted `XYZI points` 与逐像素 `uint16 amplData`；`renderType=2` 是上位机显示选择。上位机二进制/PDB 显示其用 PCL/VTK 及自定义 `PointCloudColorHandlerXt<PointXYZI>::getColor`，但未附 C++ 源码，因此不能臆测其精确 LUT。Orin 当前双实例已用 `image_type: 4`、`publish_intensity: true`、`organized_cloud: true`，并优先把 `frame.amplData` 写入 ROS `intensity`；双雷达融合和 LIO adapter 会继续保留强度字段。工作台使用 `intensity`、rainbow、Flat Squares；2026-07-28 根据实测幅值分布仅在 RViz 中改为左 `50～700`、右 `50～1600` 的独立稳定显示范围，原始 XYZI 不裁剪、不归一化、不改写。强度是反射幅度，不是 RGB，也不会自动提高几何建图精度；后续每个融合/LIO/累计地图话题都必须核验 XYZI。若 RTAB-Map 输出丢弃 intensity，应另建保强度的累计点云，不得静默丢失。右雷达实测最高约 2529 超出文档 2039，仍是厂家问题，显示可饱和但原始值不得截断。
15. **操作台现统一为“图像内容优先”。** 四个 Camera Panel 默认只显示相机画面与一行 `Details`，物理标签、图像/CameraInfo 话题、transport、分辨率、FPS、时间戳、延迟、在线状态和旋转/镜像等选项仅在点击 `Details` 后出现，并在独立滚动区内浏览。SmartWheel 2D Map 同样默认只显示地图画布与一行 `Details`，尺寸、缩放、mapping 状态、文件路径、话题和 Auto-fit/原点选项全部收进展开区。相机默认保持原比例；竖向相机允许黑边，不能为了铺满而默认拉伸变形或裁掉障碍物视野。实机已验证 camera/map Details 可展开和收起，插件仍为 27 项测试全通过；最终只读工作台保持 `motion_control_enabled=false`。
16. **PointCloud+Amp 左右隔离及用户遮挡确认：当前只有物理右雷达贡献有效画面。** 逐一关闭 RViz 点云显示后，密集、可识别的幅值着色室内场景来自 `/xtm60/right/points`（当前配置 IP `192.168.1.101`）；`/xtm60/left/points`（当前配置 IP `192.168.0.101`）单独显示时在轮椅附近几乎没有主体点云。同步 12 秒只读诊断显示两路都有组织 `160×60` XYZI 且约 10 Hz，强度分别为 `52～2034`、`64～1937`，但左路距离中位数 `36.356 m`、P05～P95 `11.848～49.100 m`，右路为 `2.506 m`、`1.170～5.700 m`。随后让右 SDK 进程干净退出并关闭 TCP/UDP，左路再读 12 秒仍为中位数 `36.366 m`、P05～P95 `11.852～49.098 m`，排除了第二 ROS SDK 实例或当时活跃的右测量是直接原因。用户再用手遮挡：遮住物理右雷达后点云完全消失，遮住左雷达无变化。因此左节点的 connected/measuring 与约 10 Hz 不能视为雷达有效工作，它没有输出会随近场遮挡变化的可用场景数据。当前日志对应左 `.0.101` 序列串 `XTM60B20250324000151`、右 `.1.101` 为 `XTM60B20250324000134`。不得用颜色缩放掩盖，也不得自动重写设备配置；先在官方上位机同场景核对 `.0.101` 距离、设备序列号和左右贴标。已新增 `docs/PROJECT_HANDOFF_CURRENT.md` 作为快速交接摘要。
17. **每次真实雷达任务结束必须停止两台 XT-M60 测量。** 用户明确反馈雷达运行温度很高，不允许工作台/RViz 在任务结束后继续后台测量。普通 transient service 停止曾因进程组退出超时进入 SIGKILL，不能单凭 `inactive` 推断厂家 SDK 已执行 `stop()`；正确顺序是先对两个适配器发 SIGINT并等待各自 `sdk.stop()/shutdown()`，再停止余下服务，最后确认无 `xtm60_adapter_node` 和无 UDP 帧。2026-07-26 最终已分别用绑定各自网卡的实例完成干净停止，服务为 `inactive`、无适配器进程，并在 `192.168.0.100:7687` 与 `.1.100:7687` 各监听 3 秒均未收到 UDP 数据。
18. **2026-07-28 已修复 RViz 双雷达显示，根因是适配器可选 SDK 滤波链破坏左路，而不是左雷达未开启。** 在不调用 median/edge/Kalman/dust/postprocess/reflective 额外滤波时，厂家 SDK ImageType 4 原始左/右距离中位数分别为 `1.278/1.922 m`，P05～P95 为 `0.619～4.046/1.042～5.546 m`，点向量范数与 `distData` 均为 `0.001 m/mm`，证明两台硬件和 XYZ 转换都能输出近场数据。生产 `xtm60_left.yaml`、`xtm60_right.yaml` 已统一改为 `enable_sdk_filters: false`，没有设备配置写入。完整双路 ROS 复测左 `10.069 Hz`、中位数 `1.356 m`、P05～P95 `0.484～4.125 m`，右 `10.066 Hz`、中位数 `1.945 m`、P05～P95 `1.060～5.549 m`；RViz 左单独、右单独和双路恢复画面均已截图。具体责任滤波项仍未逐项定位，不得整体重新开启。设备只读回读还发现当前左右保存配置 `isFilterOn=55/125`、第三曝光槽 `20/30 us` 不同，但本次未写设备，因为关闭主机 SDK 滤波后原始数据已正常。最终使用 `KillSignal=SIGINT` 停止，服务 inactive、无雷达进程、两网卡 UDP 各 3 秒无数据。
19. **2026-07-28 已解释并修正左右 PointCloud+Amp 的视觉差异。** 两路都已设置为 ImageType 4 并把 `amplData` 写入 PointCloud2 `intensity`；两个 Windows `.xtcfg` 各自对应 IP，但文件内容和 SHA256 完全相同。左/右 ROS 幅值中位数与 P95 分别为 `158/625`、`583/1445`，原先共用 `0～2039` 色阶使左路长期落在低色段，看起来不像右路的多色 PointCloud+Amp。RViz 逐帧自动范围在本机测试时中央渲染为空，已撤回；最终仅将显示范围设为左 `50～700`、右 `50～1600`，封闭式只读实机截图出现完整多色双云。空间几何若仍不重合，仍属于临时 TF、最终外参、时间同步和视角遮挡问题，不能用颜色范围掩盖。测试结束再次通过 `inactive`、无适配器进程、双网口 3 秒无 UDP 的停机检查。
20. **用户换位试验确认左右雷达并非单纯场景差异，且两雷达相对前后间距为零。** 用户把物理左雷达移动到右雷达先前的位置并旋转轮椅，左路仍不能复现右路的密集平面；同时一个经过行人在两路最远点中对应出现，说明两路观察到共同目标但投影没有正确重合。用户明确两光学中心仅横向相隔约 `0.60 m`，没有相对前后间距；当前 TF 也为同一 `x=0.45 m`、`y=±0.30 m`，其中共同绝对 x 仍是未实测的旧 base 原点估计。当前 TF 将左/右光学前向轴置于约上扬 `1.1°`/下俯 `7.8°`，相差约 `8.9°`，5 m 处可造成约 `0.78 m` 距离相关表观错位。另有独立的成像密度不对称：左/右有效像素约 `45.6%/99.1%`，换位后差异仍跟随设备；正常启动虽强制 ImageType 4，但 `apply_device_config=false` 不会全量重写 Windows 配置，设备保存参数又存在 `isFilterOn=55/125`、第三曝光槽 `20/30 us` 漂移。后续必须先隔离设备保存配置/光学/处理链，再标定最终外参；不得把相对 x 改成非零来补偿角度或成像问题。
21. **用户已授权并完成“右雷达稳定成像配置复制到左雷达”。** 三轮不启动测量的回读确认右/左唯一稳定成像差异为第三 HDR 曝光 `30/20 us`；`isFilterOn` 在无写入的连续连接中随机变化（左 `65→13→130`、右 `60→192→179`），因此被判定为不可安全复制的非稳定字段。专用脚本校验右源序列号 `XTM60B20250324000134`、左目标 `XTM60B20250324000151`，不改网络/身份/TF、不启动测量，只向左侧调用 `setIntTimesus(2000,1600,200,30,1600,0)` 并重申双方已相同的 HDR=1、minAmp=70、maxfps=10；所有调用和立即回读通过，新连接持久化复核仍为 `[1600,200,30,1600] us`。生产左右 YAML 均改记 `int_time_3: 30`，但日常 `apply_device_config=false` 不变。写入后左有效点由 `45.63%` 升至 `60.54%`，幅值中位数/P95 由 `158/625` 升至 `280/966`；右侧同期为 `99.49%`、`511/1451`。配置复制有明显改善但未消除左侧设备差异，后续仍需光学/设备处理诊断与最终角度外参；相对前后间距保持用户确认的零。最终服务 inactive、无适配器进程、双网口 3 秒无 UDP。

远端权威新增证据：

- `docs/hardware/B0_DOCUMENT_AUDIT.md`
- `docs/hardware/XT_M60_SINGLE_BRINGUP_REPORT.md`
- `docs/hardware/H30_IMU_BRINGUP_REPORT.md`
- `docs/hardware/XT_M60_DUAL_BRINGUP_REPORT.md`
- `docs/hardware/XT_M60_RIGHT_ORIENTATION_REPORT_20260724.md`
- `docs/hardware/FD07_ARRAY_REPORT.md`
- `docs/hardware/CAMERA_ARRAY_B4_REPORT.md`
- `docs/hardware/ZLAC_ENCODER_READONLY_REPORT.md`
- `docs/hardware/HARDWARE_WORKBENCH_REPORT_20260726.md`
- `docs/hardware/NON_MOTOR_HARDWARE_ACCEPTANCE_20260722.md`
- `docs/hardware/evidence/H30_B2_STATIC.json`
- `docs/hardware/evidence/H30_B7_STATIC_REPEAT.json`
- `docs/hardware/evidence/H30_LEVEL_STATIC_PRE_CAL_60S.json`
- `docs/hardware/evidence/H30_LEVEL_MOUNT_CALIBRATION.json`
- `scripts/hardware/h30_static_diagnostic.py`
- `docs/hardware/evidence/XT_M60_LEFT_DEVICE_INFO.json`
- `docs/hardware/evidence/XT_M60_LEFT_B1_SDK_FRAMES.json`
- `docs/hardware/evidence/XT_M60_LEFT_B1_CLOUD.json`
- `docs/hardware/evidence/XT_M60_LEFT_B1_BAG_REPLAY.json`
- `docs/hardware/evidence/XT_M60_LEFT_B1_GROUND_PLANE.json`
- `docs/hardware/INSTALLATION_MEASUREMENTS_20260722.md`
- `docs/hardware/evidence/XT_M60_LEFT_EXPORT_CONFIG_APPLIED.json`
- `docs/hardware/evidence/XT_M60_RIGHT_EXPORT_CONFIG_APPLIED.json`
- `docs/hardware/evidence/XT_M60_DUAL_POST_EXPORT_LEFT_CLOUD.json`
- `docs/hardware/evidence/XT_M60_DUAL_POST_EXPORT_RIGHT_CLOUD.json`
- `docs/hardware/evidence/XT_M60_RIGHT_GROUND_CURRENT_REPEAT_20260724.json`
- `docs/hardware/evidence/XT_M60_RIGHT_ORIENTATION_PROVISIONAL_20260724.json`
- `docs/hardware/evidence/XT_M60_TF_RUNTIME_20260724.txt`
- `scripts/hardware/xtm60_orientation_diagnostic.py`
- `docs/hardware/evidence/FD07_ARRAY_POST_PACING_RAW.json`
- `docs/hardware/evidence/FD07_ARRAY_POST_PACING_ROS.json`
- `docs/hardware/evidence/CAMERA_TWO_COMPRESSED_FRONT.json`
- `docs/hardware/evidence/CAMERA_TWO_COMPRESSED_LEFT.json`
- `docs/hardware/evidence/CAMERA_DIRECT_USB3_20260723.json`
- `docs/hardware/evidence/CAMERA_DIRECT_USB3_10HZ_20260723.json`
- `docs/hardware/evidence/CAMERA_DIRECT_USB3_PORT33_RECOVERY_20260723.json`
- `docs/hardware/evidence/CAMERA_DIRECT_USB3_THREE_COMPRESSED_20260723.json`
- `docs/hardware/evidence/CAMERA_DIRECT_USB3_TWO_COMPRESSED_20260723.json`
- `docs/hardware/evidence/CAMERA_FOUR_45B_COMPAT_20260723.json`
- `docs/hardware/evidence/CAMERA_FOUR_ROS_COMPRESSED_20260723.json`
- `docs/hardware/evidence/CAMERA_FOUR_RVIZ_RAW_20260723.json`
- `docs/hardware/evidence/CAMERA_SOAK30_LEFT_FRONT_20260723.json`
- `docs/hardware/evidence/CAMERA_SOAK30_LEFT_SIDE_20260723.json`
- `docs/hardware/evidence/CAMERA_SOAK30_RIGHT_FRONT_20260723.json`
- `docs/hardware/evidence/CAMERA_SOAK30_RIGHT_SIDE_20260723.json`
- `docs/hardware/evidence/CAMERA_MJPEG_WARNING_ISOLATION_20260723.json`
- `docs/hardware/evidence/ZLAC_ENCODER_POWERED_READONLY_20260725.json`
- `docs/hardware/evidence/ZLAC_ENCODER_POWERED_RETRY_LONG_TIMEOUT_20260725.json`
- `docs/hardware/evidence/ZLAC_ENCODER_HUB_RETRY_20260726.json`
- `docs/hardware/evidence/ZLAC_ENCODER_DIRECT_USB_READONLY_20260726.json`
- `docs/hardware/evidence/ZLAC_ENCODER_CURRENT_RETRY_20260726.json`
- `docs/hardware/evidence/ZLAC_ENCODER_STATIONARY_CONFIRM_20260726.json`
- `docs/hardware/evidence/ZLAC_ENCODER_POST_LEFT5_READONLY_20260726.json`
- `docs/hardware/evidence/ZLAC_LEFT20_GUARDED_MOTION_20260726.json`
- `docs/hardware/evidence/ZLAC_ENCODER_POST_LEFT20_READONLY_20260726.json`
- `docs/hardware/evidence/ZLAC_LEFT30_GUARDED_MOTION_20260726.json`
- `docs/hardware/evidence/ZLAC_ENCODER_POST_LEFT30_READONLY_20260726.json`
- `docs/hardware/evidence/ZLAC_ENCODER_POST_RVIZ_READONLY_20260726.json`
- `docs/hardware/evidence/HARDWARE_WORKBENCH_CAMERAS_20260726.json`
- `docs/hardware/evidence/HARDWARE_WORKBENCH_LEFT_CLOUD_20260726.json`
- `docs/hardware/evidence/HARDWARE_WORKBENCH_RIGHT_CLOUD_20260726.json`
- `docs/hardware/evidence/HARDWARE_WORKBENCH_RVIZ_20260726.png`
- `docs/hardware/evidence/HARDWARE_WORKBENCH_LAYOUT_20260726.png`

当前停止点是“雷达、FD07-34R、H30 基础链路已处理；双雷达独立 SDK 实例并发约 10 Hz；四相机实机操作台链路和物理标签已恢复；ZLAC 静止通信已恢复；左轮三档试验已安全结束；用户随后完成 RViz W/A/S/D/STOP 实机验收并确认全部按钮/方向正常；远程桌面长按、相机 Dock、实时双雷达三维与二维地图预览问题已修复；最终六区布局和地图优先面板已实机显示”。最终集成工作台以 `motion_control_enabled=false` 验证，四相机、双点云、约 10 Hz `/scan` 和最终增长到 `50×89` 的小型 `/map` 同屏可见；交接时该只读工作台可能仍在 Orin 桌面运行，新对话须先检查进程状态，绝不能据此假设电机写入已授权。通信恢复原因未能按单一物理变量归因，USB/供电/接地风险仍保留。下一算法阶段仍是 FAST-LIO2、双雷达融合/最终外参、真实累积三维地图及二维地图质量验证；当前小型 `/map` 仅为静止预览，不是建图验收。导航、真实地面运动、物理急停闭环和载人安全测试仍未完成。

B0 后仍需用户/厂家材料或实测确认：

- XT-M60 SDK 坐标原点、时间戳 epoch/reset 语义、双雷达硬件同步、最终外参和右雷达 `2040～2165` 幅值语义；
- H30 受控动态轴向、yaw/xyz 外参、设备时间戳和安装结构变更后的复核；
- ZLAC 有效静止应答已恢复但原因未单独归因；仍需确认编码器分辨率、减速比、轮径、轮距、方向、极性、watchdog 和受控运动安全；
- 右前安装链 libjpeg 告警的具体元件来源、暂定旋转的目视确认、内外参标定和精确安装位置；需一次只交换相机本体、线缆或接口中的一个变量；
- FD07-34R 的测量触发模式、角度口径、无目标返回、多机串扰建议和全系统负载下供电余量；
- 最终机械安装尺寸、精确雷达/相机参考点、俯仰/偏航/横滚和结构稳定性。

## 10. 不得遗忘的工程原则

- 先建立可解释、可单测、可复现的链路，再增加功能。
- 传感器可显示不代表 TF、时间戳、单位和里程计正确。
- UI 在线状态不代表硬件安全可用。
- mock 结果、合成 ground truth 指标和模拟地图不能作为真实轮椅精度证明。
- 超声波/FD07-34R 主要属于近距安全层，不作为一期 SLAM 主传感器。
- 相机一期先做单路接入和诊断，再决定视觉回环、着色或语义功能。
- 所有真实设备接入必须单设备、低风险、可回退、有原始数据留档，并通过阶段门禁后再扩大范围。

## 11. 2026-07-29 长时纠正与当前最新停止点

本节晚于前文所有 `65 s` 双路结论，并明确覆盖“左雷达已经长期恢复”的
旧判断。

- 主机单调时钟 `100 ms` 网格、左 `0 ms`/右 `50 ms` 和每 `30 s`
  重对齐确实能缓解未硬件同步的 Flash ToF 光学串扰，但不是硬件同步，
  也不是左雷达低有效点的完整修复。
- 早先 `65 s` 双路左/右平均有效点 `91.72%/97.11%` 只是短时好窗口。
  随后的 `30 min` 双路长测中，左原始帧平均/中位有效点仅
  `46.38%/45.56%`，质量门接受率 `21.21%`、主点云仅 `2.084 Hz`；
  右路为 `97.25%/97.33%`、接受率 `98.01%`。长测门禁失败。
- 左雷达关闭右路独立运行 `10 min` 仍只有约 `46.16%` 有效点，且有效点
  随温度上升略增，否决“仅双机串扰”和简单“越热越差”解释。
- 直接 SDK 同场景独立对照：左路 60 帧有效点 `48.03%`、幅值中位数
  `179`；右路 `96.90%`、幅值中位数 `511`。左路大量
  `rawdistData` 已是厂家失效码 `964009/964001`，故障不在 ROS、TF、
  PointCloud2 或质量门。
- 严格只读配置快照发现左侧第三 HDR 曝光从先前确认的 `30 us` 回退到
  `20 us`，右侧仍为 `30 us`。按用户已授权的序列号门禁流程恢复后，
  断开重连和测量后再次重连均确认左侧保持 `[1600,200,30,1600] us`；
  但直接 SDK 左路仍约 `48%`，因此配置回退是真问题但不是剩余主因。
- 当前剩余故障域是左雷达光学窗口、VCSEL/接收成像链、线缆/供电或需要
  完整断电复位的设备状态。下一步必须由用户进行单变量物理 A/B；不能再
  通过猜测软件门限或外参掩盖。
- 适配器已增加质量门和只读启动保护。生产主点云只发布接受帧，拒绝帧
  进入 `points_rejected`，质量 JSON 单独发布；正式绝对有效点下限为
  `75%`。启动前核对 serial、四档曝光、HDR、minAmp 和 fps，漂移时
  保持停测而不自动写设备。单测 `21/21`，传感器/bringup 构建通过。
- 真实 `45 s` 启动冒烟中两路只读配置保护通过，左/右平均有效点
  `54.28%/97.44%`；保护生效，但左硬件链问题未消失。
- 映射软件侧回归通过：global mapping `7`、mapping manager `4`、
  map products `11`、occupancy/start `8`、architecture/integration
  `9` 项；`map_saver_cli`、RTAB-Map、FAST-LIO2 可执行文件存在。这只
  证明接口与软件安装，不等于真实静止/直线/转弯/小闭环、回环或保存/
  重载验收。现有真实 bags 没有同时包含 XT-M60+H30 和实际车体路线，
  不能补做该验收。
- 用户明确暂缓 LiDAR–IMU 外参和载人测试。H30 当前输出配置没有设备
  timestamp TLV，只能使用主机接收时间；动态轴向和时间偏差仍需人工
  受控动作。双雷达最终外参同样暂缓。不得把这些项目写成已修复。
- 当前没有授权新的地面电动运动或物理急停闭环；不能用“修复全部”扩大
  安全授权。
- 最终停止检查：transient service `inactive`，无
  `xtm60_adapter_node`，两块主机网卡 `:7687` 均为 `0` UDP 帧。

权威报告：
`docs/hardware/XT_M60_DUAL_INTERFERENCE_MITIGATION_20260729.md`。

## 12. 2026-07-29 右雷达单路 Stage 1 已完成静态验收

- 用户明确决定后续暂不使用左 XT-M60。远端连接恢复后，右路 Stage 1
  补丁已完整同步到 Orin；分支仍为
  `feature/mapping-v2-rviz-workbench`，HEAD `b50c187fc9fb`，用户原有
  dirty/staged 内容（特别是 `docs/goal.md`）保持不动。
- 通用传感器启动改为雷达默认全关；手动工作台、旧二维建图、全系统和
  单雷达映射默认侧改为右路。新增右路专用 scan merger、diagnostics 和
  `right_lidar_stage1_mapping.launch.py`。该入口硬编码左雷达 false、
  H30 false、FAST-LIO2 不启动、电机写入 false；右路同时进入 `/scan`、
  `slam_toolbox` 和累计地图产品。
- Orin 上四个改动包 build 通过，相关测试结果 `54/54` 无失败。全仓库
  聚合结果中仍有 7 月 15 日第三方 Livox/RapidJSON 历史 lint 失败，
  不能误归因于本次改动。
- 最终有界静态实机证据目录为
  `docs/hardware/evidence/right_lidar_stage1_20260729_185017`。25.04 秒
  点云诊断收到 227 帧，接收/头时间率约 `9.804 Hz`，有效点
  `96.969%`；组织形式 `160x60`，XYZI、米尺度、时间递增等全部检查通过，
  幅值 min/median/P95/max 为 `71/539/1423/1600`。
- 左话题不存在、左适配器进程证据为空。最终 bag 为
  `/home/nvidia/smartwheel/bags/hardware/right_lidar_stage1_20260729_185019`，
  `54.49 s`、12,713 条消息，含右原始点云 480、融合点云 487、`/scan`
  487、`/map` 23 条。
- 实测发现 Humble `slam_toolbox` 内置 `SaveMap` 回调会等待自身 `/map`
  订阅超时，返回 `255` 且不落盘。控制脚本已改为独立
  `nav2_map_server/map_saver_cli`，并对 PGM/YAML、posegraph/data 与服务
  响应做硬校验。离线 bag 保存测试通过，最终实机保存得到
  `59x120 @ 0.05 m/cell` 栅格文件；pose graph 序列化和重载成功。
- 版本化导出目录
  `/home/nvidia/smartwheel/maps/versions/right_lidar_stage1_20260729_185022`
  的 14 文件 manifest 完整。`map_pointcloud_amp.pcd/.ply` 保留 intensity；
  PCD 为 18,521 点，幅值 min/median/P95/max 为
  `71/357/1350/1600`。质量报告 stage 为
  `RIGHT_LIDAR_STAGE1_PRELIMINARY`，`hardware_validated=false`，这是
  正确状态，不得改成最终实机验收完成。
- 最终清理通过：service `inactive`、无 adapter process、两路主机
  `:7687` 均为 0 UDP 帧。
- 这只完成“静止链路、保存/重载、PointCloud+Amp 产品”门槛。未执行
  实际路线、地图随运动增长、直线/转弯/小闭环、RTAB-Map ICP 回环、
  轮式里程计精度、LiDAR–IMU/FAST-LIO2、地面电动行驶、物理急停闭环、
  导航或载人测试。用户明确暂缓 LiDAR–IMU 外参和载人；没有新的地面
  电动运动授权。
- RTAB-Map 参数纠正保留：`Reg/Strategy=0` 是视觉配准，ICP 是 `1`；
  `enable_loop_closure` 默认关闭，必须等真实右雷达小闭环 bag 再验收。
- 当前设计和实测报告：
  `docs/mapping/RIGHT_LIDAR_STAGE1_MAPPING_20260729.md`。
