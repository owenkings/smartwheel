# 算法选型、已有结果与对照实验

本页按 2026-10-07 的源码和已保存实验核查。当前已有阶段性结果，尚没有足够证据删除某个估计器。公开仓库保留实现、参数模板和实验入口；原始录包、地图、个人室内图像及完整实验报告留在配置的数据目录。

## 1. 当前结论

| 部分 | 当前实现及默认 | 已有结果 | 下一步怎样决定 |
|---|---|---|---|
| 五状态 EKF / robot_localization | 实时平地主线保留五状态；`compare` 可在同录包中运行两者 | 已完成较短录包的同输入、同平面约束、同源时刻对照；单条路线不足以宣布优胜或等效 | 先完成两种实现共用新版运动修正适配，再固定候选及权重，在未参与拟合的多条录包验证 |
| IMU 零偏 / 轮反馈修正 | `refine` 自动估计或显式加载该会话候选；包含独立时间窗验证 | 一条较长录包的闭合一致性改善；轮速与 IMU 的验证残差下降 | 对不同录包分别验证；永久机械参数不能由单次传感器间一致性直接替代 |
| 原过程噪声 / 连续白加速度 PSD | `refine` 默认 `legacy`，`white_acceleration` 显式开启 | 同一较长录包的 PSD 候选闭合数字较小；整图仍有残留问题 | 固定训练、验证数据，分别调参后在相同未见录包比较，不能因两种模型都填 0.25 就称噪声强度相同 |
| SDK raw / filtered | `compare`、`refine` 默认 raw；当前实时 `mapping_live.json` 是 filtered | 较短录包上 filtered 的 ICP 残差更低、接受对更多，但轨迹并未因此变化 | 固定运动估计、应用层过滤及帧身份，比较墙厚、边缘、拖影、有效点与匹配失败 |
| 应用层过滤开 / 关 | `compare --filter off on`；实时平地主线开启 | 同录包能比较；它改变点云，不应被解释为直接修复轮速 / IMU 漂移 | 固定同一墙面和距离段；检查异常减少与细小障碍保留的取舍 |
| 5 Hz / 10 Hz / 全部有效配对 | `compare --input-rate-hz 5 10 0`；`refine` 支持 (0,10]，不支持 0 | 较短录包前端分别转发 515 / 796 / 1440 帧；共同时间戳的位姿完全一致 | 增加帧数不等于提高定位精度；另测实际吞吐、排队延迟和完整原生地图 |
| 几何纠偏 | `refine --geometry on` 才开启，默认 off | 较长录包保守几何接受 106 对、拒绝 1252 对，没有稳定额外地图收益 | 先解决单帧异常、覆盖及退化，达到预先约定收益后再考虑默认启用 |
| ICP 影子评估 / 真正几何反馈 | `compare --icp-shadow point_to_plane` 只记录匹配；`refine --geometry on` 会生成几何纠偏结果 | 影子评估结果不能当成已经用于建图的轨迹 | 分开评估，禁止把“ICP 能运行”当成“地图已改进” |
| 空间回环 | 当前平地基线未开启正式自动回环 | 旧录包首尾局部匹配候选未通过；没有确认闭环证据 | 要同时验证正确接受和错误拒绝，不能因用户大致返起点便强制闭环 |
| 平面 / 三维诊断 | 平地 `planar_ekf`；`mapping_3d_diagnostic.json` 保留 `se3_gyro` | `compare` 当前强制平面；尚无同条件坡道验收 | 三维诊断不等于正式坡道定位；比较时须额外统一当前不同的过滤设置 |
| 渲染后端 | 属于显示性能和稳定性选择 | 与估计器优劣是两个问题 | 同一保存地图检查加载、帧率和故障；禁止用显示流畅度代替地图几何准确性 |

raw 是主机过滤前的 SDK XYZ，已经经过设备处理和 SDK 投影，不是原始 UDP 或原始光学深度。应用层过滤与 SDK filtered 是两层独立因素。

### 1.1 最初建议中的全部对照类别，现在到哪一步

最初的表同时包含工程整合、实验选项和现场验证建议，不能把它们全部说成“已保留两个完整实现并完成比较”。下表覆盖具有选型含义的类别；**有代码、能选择、做过对照、确认优胜**是四个不同状态。

| 类别 | 当前采用 / 默认 | 候选或开关是否实际存在 | 已有证据 | 结论状态 | 下一步 |
|---|---|---|---|---|---|
| 工程主体、驱动 | 新版工程及 C++ 驱动 | 旧版可供只读历史参考；没有完整旧 SDK 网络包回放接口 | 新版有逐侧身份、来源及录包对账 | 已定工程主体；未证明新版设备测距更准 | 保留旧证据；不要为“清理选项”删除历史录包 |
| 五状态 / 官方 EKF | 五状态平地主线 | `compare --estimators five_state robot_localization` 已实现 | 515 帧同基线，五状态闭合数字略小 | 有结果，未最终选型 | 共用新版修正适配后在独立路线比较 |
| 启动零偏流程 | 已有 10 秒预热、10 秒估计、10 秒独立验证模块及确认入口；`refine` 另有会话内独立时间窗提取 | `confirm_gyro_bias`、`mapping_bias.py`、`offline_motion_calibration.py` | 长录包修正改善；实时配置的 `gyro_bias_config` 仍为 null | 流程已实现，不等于每次实时启动都已应用已验证零偏 | 从当前录包生成并验证；分别记录实时是否加载、离线是否应用 |
| 在线自动学习零偏 | 候选不自动生效 | 观察、筛选、确认机制存在 | 已避免把零轮速直接当物理静止；没有“自动学习更准”的完整对照 | 已定使用边界 | 保留候选与独立确认，不为比较取消证据要求 |
| 异常创新门 / 权重 | 有创新门，保留现有基线权重 | 诊断计数和实验参数存在；不是已经选出通用最优门限 | 长录包拒绝大多对应尖峰；关闭门限仅作分析 | 门控机制保留，门限及权重未定型 | 固定正常转弯与异常样本，比较误拒绝和误接受 |
| 轮反馈尺度与转向修正 | 实时配置保留轮径、轮距和原反馈换算；`refine` 有会话 `w'=a*w+b*v` | 完整协方差 `J R Jᵀ` 已实现；当前未共用到 RL 新对照 | 独立时间段轮速 / IMU 残差下降 | 会话内有效，永久标定未完成 | 独立录包确认，不将传感器互相拟合当独立尺度真值 |
| 原过程噪声 / 连续 PSD | `refine` 默认 legacy | `--process-noise white_acceleration` 已实现 | 1,359 帧，PSD 候选首尾间距更小 | 有结果，未替换默认 | 统一调参预算及独立验证集；两类参数量纲不同 |
| 主机到达时间 / 估计测量时间 | 离线统一主机单调时钟映射，保存原时间与序号 | `source_time.py` 单一策略；**尚无已验证的设备共同曝光时间模型对照** | 同戳按 wheel、imu、source 及源序号确定顺序 | 基线已定，物理同步未确认 | 先核实设备时间含义与延迟，不能凭标称 200 Hz 造时间 |
| IMU 轮询周期 | 独立 `record` 支持 10 / 5 / 2.5 ms，默认 10 ms | 入口存在；不能据此称所有 `capture` 流程都有同一切换参数 | 有历史减少批次同戳的观察，未建立当前负载下正式选型 | 可选，未定 | 同负载检查批次、丢包、CPU 与延迟 |
| 轮反馈读取率 | `capture` / 现有手动流程使用 10 Hz | 独立 `encoder --rate-hz` 支持 10 / 20 / 50 | 无完整当前动态比较证明 50 Hz 更好 | 可选，未定 | 在事务容量允许时比较超时、控制竞争及转弯时间分辨率 |
| SDK raw / filtered | 实时 filtered；离线 compare/refine raw | 两类消息可分别读取 | 短录包 filtered 的影子匹配更稳定 | 有结果，未最终选型 | 固定帧、运动估计和应用筛选，核验墙面与边缘 |
| SDK 各级滤波组合 | 仍按每侧 `.xtcfg` 应用主机滤波；preserve_current 保留设备成像设置 | 中值、Kalman、空间等开关在配置 / 驱动；**没有完整同原始网络包逐滤波离线复现** | 驱动明确记录部分 `.xtcfg` 字段不支持，例如 Windows `pclFilterOn` 无已验证 Linux 等价项 | 不能说已实现“最小滤波组合胜出” | 单项配置、实际读回与原始 / filtered 一起记录；设备变化另作实验 |
| 整帧质量门 / 跨帧跳变 | 有身份、格式、点数、有限点和空侧等检查 | **没有可直接选择的旧 75% / 65% 有效率门与新门公平对照，也未建立动态自适应跨帧硬门** | 基础防护和拒绝原因可追溯 | 基础检查保留；质量门候选尚未完成 | 从固定好帧 / 坏帧样本确定阈值，避免删掉正常遮挡变化 |
| 距离、高度、邻域过滤 | 实时开启；离线按参数控制 | `--filter off on` 整体切换；各规则配置存在 | 已有整体开关实验；保留原始行、无效 XYZ 及源身份 | 整体有对照，单项阈值未定 | 固定场景逐项检查细物体、边缘与孤点 |
| 体素降采样 | 诊断前端未新增可选体素化 | **当前 compare/refine 没有 voxel 因子接口** | 无该因素实测对照 | 尚未实现独立对照 | 仅在负载确有需要时添加，不能靠删点掩盖问题 |
| 光学互扰 | 保留单侧 / 双侧明确采集和来源 | 可分别采集；**没有已验证的自动互扰判别器** | 时间配对不能证明发光不互扰 | 未确认 | 固定静止场景单开 / 双开，保留设备设置与逐侧点云 |
| 双雷达配对及失效行为 | 显式消费来源，50 ms 配对上限；all 模式不静默改单侧 | 单侧模式独立选择；没有旧 0.5 秒缓存与新配对的统一 CLI 因子 | 来源对账与覆盖机制存在 | 已定工程行为；50 ms 最佳值未定 | 按实际运动和延迟评估更小阈值及缺帧成本 |
| 双帧运动补偿 | 使用同一位姿提供者补偿两侧到达时差 | 已有补偿和几何纠偏后重新补偿；**compare/refine 没有独立补偿 on/off 因子** | 20,728,532 点重新补偿数学一致性检查通过 | 实现核验通过；真实收益未完成单因子对照 | 增加关闭对照，保持相同有效帧；不冒称逐点去畸变 |
| 点云消费率 | 实时 5 Hz；compare 可 5 / 10 / 0 | 0 表示全部有效配对；refine 仅 (0,10] | 短录包 515 / 796 / 1440 帧，共同时刻位姿一致 | 帧密度结果已知，精度优胜未定 | 原生消费率同步单独测试，检查 ACK、队列与墙厚 |
| RTAB-Map / 更换框架 | 继续 RTAB-Map | 没有为本轮实现另一套 SLAM 框架 | 问题未定位为框架必须替换 | 已定继续现后端 | 先处理输入和约束质量 |
| ICP 影子 / 真几何反馈 | 默认不让影子 ICP 更改位姿；几何纠偏默认 off | 两个不同入口已实现 | 短录包影子对照；长录包几何接受 106、拒绝 1252 | 有结果，几何尚无稳定额外收益 | 先查退化和单帧形状；不取消拒绝门求“更直” |
| 邻边精配准 / 空间回环 | wheel_imu 分支关闭两者 | launch 有底层参数；**compare/refine 没有已验收开关矩阵** | 首尾影子候选未通过；未证明真实闭环 | 关闭为当前基线，实验未完成 | 分开验证，增加错配拒绝和多解检查 |
| 图优化器 / 栅格分辨率 | 保留现后端；平地配置 5 cm | 有参数，未提供完整选型矩阵 | 无多优化器 / 多分辨率公平结论 | 未对照 | 输入与约束稳定后单独做；5 cm 栅格不是 5 cm 精度 |
| 平面 / 三维运动模型 | 平地主线 planar_ekf；se3_gyro 诊断保留 | 配置存在；compare 固定平面 | 三维诊断同时关闭过滤，不能直接当单因子结果 | 未完成同条件动态对照 | 固定其余参数再比较；坡道另验收 |
| 三维点云 / 二维栅格输出 | 两者都保留 | 是同源的不同输出用途，不是只保留一个的竞争算法 | 保存 / 查看已验证 | 已定同时保留 | 分别检查几何和通行表达 |
| 地面 / 障碍分类 | 有高度、法线、聚类及轮轴地面参考 | 参数可改；尚无固定实物标签验收集 | 机械初值与运行参数有来源，非地面真值 | 未定型 | 同一已知地面和物体评价漏检 / 误检 |
| 单 / 双雷达射线清空 | 单侧按真实原点；双雷达 app 关闭射线 | `wc_maps` 有多观测原点重建；当前原生主线标记 dual_ray_tracing_implemented=false | 多原点软件实现存在，不代表主线已打通并胜出 | 主线选择已定，双源替代验收未完成 | 固定遮挡场景验证，不用公共虚构原点 |
| 导出清噪 / 原始导出 | 保留未额外清噪的原生导出 | **未找到当前 compare/refine 的可选独立清噪副本入口**；后端分类里的 noise 参数不是同一功能 | 无导出清噪公平结果 | 原始保留已定；副本功能待补 | 如增加则另存并保留同处理比较 |
| system / software 渲染 | 实时配置 software | `rviz_renderer` 支持两者；有特定 GL 初始化故障的一次重试 | 为已观察软件兼容问题提供路径；无同负载正式吞吐对照 | 可选，未改成 system 优先 | 同保存地图测试稳定性、资源和可见帧率 |
| periodic / on_close 持久化 | 当前 mapping_live 为 queued_realtime + on_close；录制有内存暂存、转存及最终核验 | 两种 checkpoint 策略实现存在；不能将映射输入 checkpoint 等同全链数据库断电恢复 | 正常收尾、文件和哈希验证通过；当前无现场断电恢复验收 | 有机制，长期策略未定型 | 固定负载比较写盘延迟、可恢复前缀；运行中掉电不能宣称已保存 |

以上“尚未实现独立对照”并不表示基础功能完全不存在，而是目前没有一套已接通、冻结同输入且可据结果选型的实验入口。纯工程修复（源身份、停止期限、队列对账、配置快照、存储路径、手动控制事件保护）采用修复后的统一路径，无需永久保留带缺陷的旧路径作为日常选项。

外参文件、已保存尺寸及安全编辑 / 校验入口见 [外参与设备安装配置](calibration_configuration.md)。无需为了整理这些选项再次测量已经记录的尺寸。

## 2. 已有数值能说明什么

### 2.1 较短录包：两种估计器

条件：同一录包、相同 raw 来源、应用过滤关闭、前端 5 Hz 上限，515 个共同输出帧，源时间跨度约 143.69 秒。两者均完成原生建图导出。

| 指标 | 五状态 EKF | robot_localization |
|---|---:|---:|
| 首尾位置间距 | 0.749 m | 0.846 m |
| 首尾航向差 | +5.684° | +7.087° |
| 累计轨迹长度 | 40.537 m | 40.530 m |

五状态在这次闭合数字上较小。没有独立轨迹真值，结束点也只是接近起点，不能把这些数值称作绝对误差，更不能据此淘汰 robot_localization。该对照使用当时冻结的参数，不包含后续新版会话运动修正。

同一较短录包的固定 143 对 ICP 影子评估：raw 下五状态接受 75 对、robot_localization 接受 77 对；filtered 下两者均接受 88 对，平均最终残差约从 4.51 cm 降至 4.07 cm。这只是匹配诊断，不是测距精度、ATE 或已纠偏地图的精度；首尾候选均未通过。

### 2.2 较长录包：五状态内部修正

条件：同一批 1,359 帧，均使用五状态 EKF。

| 方案 | 首尾位置间距 | 首尾航向差 |
|---|---:|---:|
| 原始融合 | 1.780 m | +27.217° |
| 零偏及轮反馈运动修正 | 0.986 m | +8.862° |
| 运动修正 + 连续噪声候选 | 0.538 m | −6.596° |
| 运动修正 + 保守几何约束 | 0.966 m | +9.022° |

这些方案不是五状态与 robot_localization 的新对照。下半部分地图有所改善，上方弧形点仍然存在；部分弧形点在单雷达单帧中已经出现，不能把全部弯曲解释为累计漂移。

原噪声模型与白加速度 PSD 的参数量纲不同。这次 PSD 实验同时改变模型及候选扰动强度，没有完成各模型等预算调参后在独立数据上的最终选型。

## 3. 实验准备与路径

以下命令在已部署 ROS 2 和工程的受支持 Linux 主机运行，计算不启动真实设备。把 `PROJECT` 设为实际 clone 目录；示例与 README 的获取位置一致，不限制用户名、主机名或目录。每次使用新的输出目录。

```bash
PROJECT="$HOME/projects/wheelchair"
cd "$PROJECT"
source "${WHEELCHAIR_ROS_SETUP:-/opt/ros/humble/setup.bash}"
source install/main/setup.bash

# 改为本机已配置的数据根目录；它可以是内部目录，也可以是已验证挂载的外置盘。
ARCHIVE="$HOME/wheelchair-data"
DATASET="$ARCHIVE/data/experiments/session_001"
RUN="$(date +%Y%m%d_%H%M%S)"
```

`ARCHIVE` 必须与 `configure_storage` 选定的数据根一致。当前机器使用外置盘时，应填写它的实际挂载路径；这里不绑定任何固定盘符或 UUID。

如果历史录包是 PARTIAL，经查看其缺段、原因和有效区间后，命令可显式追加 `--allow-partial`。它允许诊断处理，不把原录包改成完整。若确实选择 V7 机械初值做离线试验，可追加 `--mechanical-initial`；这不会覆盖实时标定。不要在未知原因下盲目添加这两个选项。

## 4. 现有命令能直接运行的对照

### 4.1 两个 EKF 的现有基线

```bash
python3 scripts/wc_phase1 compare \
  --dataset "$DATASET" \
  --output "$ARCHIVE/data/analysis/ekf_$RUN" \
  --estimators five_state robot_localization \
  --cloud raw --filter on --input-rate-hz 5 \
  --native-map --native-rate-hz 5 --native-wall-interval-s 0.2
```

此命令为两者冻结同一输入、外参、零偏配置、时间策略、点云筛选和原生地图取帧。估计器改变后，双帧运动补偿结果可以随之改变；“同输入”指相同原始源帧和观测，不能要求派生点云字节相同。两种状态维数及过程模型并不相同，同数值参数也不自动代表同一统计假设。

可用 `--gyro-bias /绝对路径/bias.json` 为两者提供同一已验证零偏；旁边必须存在对应 `bias.evidence.json`。不要将 `refine` 的 `motion_candidate.json` 冒充这个文件。

**当前限制：`compare` 没有 `--motion-candidate` 或 `--process-noise`，`refine` 只支持五状态。** 因此目前不能用一个命令把新版轮反馈修正及 PSD 对等应用到 robot_localization。把修正版五状态与旧输入适配的 robot_localization 放在一起，不能据此归因于估计器。最终选型前应先补齐共用测量适配层与回归，再运行新的同条件比较。

### 4.2 raw 与 SDK filtered

只改变 `--cloud`，固定其余参数：

```bash
for CLOUD in raw filtered; do
  python3 scripts/wc_phase1 compare   \
    --dataset "$DATASET"   \
    --output "$ARCHIVE/data/analysis/cloud_${CLOUD}_$RUN"   \
    --estimators five_state --cloud "$CLOUD"   \
    --filter off --input-rate-hz 5   \
    --native-map --native-rate-hz 5 --native-wall-interval-s 0.2
done
```

这两个命令各自有结果清单。比较前仍需核对跨目录的共同原始帧身份、时间窗及配置哈希；不能仅比较两张截图。

### 4.3 应用层过滤

```bash
python3 scripts/wc_phase1 compare \
  --dataset "$DATASET" \
  --output "$ARCHIVE/data/analysis/application_filter_$RUN" \
  --estimators five_state --cloud raw \
  --filter off on --input-rate-hz 5 \
  --native-map --native-rate-hz 5 --native-wall-interval-s 0.2
```

### 4.4 前端帧率

```bash
python3 scripts/wc_phase1 compare \
  --dataset "$DATASET" \
  --output "$ARCHIVE/data/analysis/frontend_rate_$RUN" \
  --estimators five_state --cloud raw --filter on \
  --input-rate-hz 5 10 0
```

`0` 表示取消离线前端人为限频，仍然保留配对、有效性与覆盖检查，不会提高设备原始采样率。所有真实轮反馈 / IMU 事件都保留。该命令只比较前端；不加 `--native-map` 就不会生成新的 RTAB-Map 地图。

同一次多速率 `compare` 的原生地图使用各单元共同帧交集，以便公平比较算法。要研究“更多帧是否改善地图”，必须分别运行单速率单元，明确令原生消费率对应 5 或 10 Hz，并核验实际确认帧数；不能把共同帧地图当成高密度地图对照。`--native-rate-hz` 必须大于 0，不能填 0。`--native-wall-interval-s` 只影响回放墙钟节奏，不修改录制时间戳。

### 4.5 运动修正与连续噪声候选

先用该录包独立提取并验证候选：

```bash
MOTION="$ARCHIVE/data/analysis/motion_$RUN"
python3 scripts/wc_phase1 refine \
  --dataset "$DATASET" --output "$MOTION" \
  --process-noise legacy --geometry off --native-map

python3 scripts/wc_phase1 refine \
  --dataset "$DATASET" \
  --output "$ARCHIVE/data/analysis/psd_$RUN" \
  --motion-candidate "$MOTION/motion_candidate.json" \
  --process-noise white_acceleration \
  --linear-acceleration-psd 0.25 \
  --angular-acceleration-psd 0.25 \
  --geometry off --native-map
```

第二次复用同一录包的已验证候选，以保持零偏及轮反馈修正相同。候选绑定会话及原始证据，不能复制给另一条录包使用。自动提取依赖足够的静止和运动数据；失败时应查看候选验证原因，不能用零值强行继续。PSD 两参数是实验起点，单位分别为 m²/s³ 和 rad²/s³。

查看对应地图：

```bash
python3 scripts/view_map "$MOTION/native/calibrated_motion/export" --max-view-points 300000
```

查看器限制只影响显示点数，不删除保存的点。

### 4.6 几何约束与 ICP 影子诊断

真正生成几何约束对照：

```bash
python3 scripts/wc_phase1 refine \
  --dataset "$DATASET" \
  --output "$ARCHIVE/data/analysis/geometry_$RUN" \
  --motion-candidate "$MOTION/motion_candidate.json" \
  --process-noise legacy --geometry on \
  --native-map --native-cells calibrated_motion geometric_motion
```

只分析匹配而不改写轨迹：

```bash
python3 scripts/wc_phase1 compare \
  --dataset "$DATASET" \
  --output "$ARCHIVE/data/analysis/icp_shadow_$RUN" \
  --estimators five_state robot_localization \
  --cloud raw --filter on --input-rate-hz 5 \
  --icp-shadow point_to_plane
```

## 5. 从“保留选项”走向“确定方案”的验收顺序

1. **固定输入质量。** 核验归档、双雷达身份、时间窗、断流及原始帧。把明显单帧异常与融合漂移分开。原始录包完整性、算法运行成功、地图几何准确分别记录。
2. **补齐公平输入适配。** 新轮反馈修正及 `J R Jᵀ`、原生 IMU 零偏扣除一次、安装旋转、事件顺序和缺段策略应对两个 EKF 一致。用已知转向、异常观测及位姿查询不改状态等回归验证。这一步目前尚未接通到新的双估计器修正对照。
3. **冻结试验方案。** 预先指定训练录包、独立验证录包、主要指标、可接受退化和资源预算；每种方法获得相同调参预算。不要在最终验证图上反复调参然后宣称验证通过。
4. **先比估计器，再比其余因素。** 首轮只比较现有 EKF；新版共同适配完成后再对比修正后的 EKF。随后依次比较过程噪声、过滤、消费频率、几何约束和回环。不能把多个开关同时切换后的差异归给其中一个。
5. **覆盖最少几类实际运动。** 建议独立录制静止、直线及往返、左右转弯、停走和不同方向的闭合路线，每类重复至少三次是试验起点。保留静止开头和结尾有助于零偏估计及独立验证；并非要求再次测量所有机械外参。
6. **使用可解释指标。** 同帧位姿差、断流恢复、异常接受 / 拒绝、稳定段漂移、固定墙段厚度与直线残差、重复经过的重影、失败率、延迟及资源开销一起看。墙段须事先固定，不能只挑更直的一段。重复路线属于覆盖和重现性测试，不自动提供独立真值。
7. **有参考再谈精度。** 独立测量的若干位置 / 朝向检查点可以先做检查点误差；只有时间和坐标明确对应的独立轨迹才能做轨迹真值指标。当前 `compare --truth-json` 要求 `independent: true`、已确认时间对应、准确帧名和严格相同源时间戳，不内插、不自动拟合对齐。最低共同样本数应在运行前通过 `--truth-min-common-samples` 设置，不能把默认一个样本作为完整精度验收。
8. **最后收敛配置与代码。** 某方案在预先规定的主指标上改善、其余指标不超出允许退化，并在独立路线重复通过，才更换默认。先保留带 Git 标签的回退版本；移除日常配置中的实验开关后观察一轮，再删除已无调用者的旧实现。回归测试、原始证据及参数来源仍保留。

项目目前没有一组经用户用途及独立参考共同确定的动态精度阈值，因此本页不临时编造“低于某厘米即通过”。没有真值也可以做相对方案筛选，但结论应写为测试场景内更稳定或更一致。

## 6. 结果在哪里看

`compare` 输出 `result.json`、`summary_zh.md`、冻结配置、原事件索引、逐单元前端 / 轨迹；开启原生建图时还有 `native/<cell>/result.json` 和 `export/`。`refine` 输出候选及验证证据、冻结配置、阶段产物、原生地图与完成清单。完整目录应保存在配置的数据根，不能将原始地图或录包加入公开 Git 仓库。

源码入口：`src/wc_runtime/mapping_compare.py`、`mapping_refine.py`、`compare_replay.py`、`robot_localization_provider.py`、`offline_motion_adapter.py`、`offline_planar_process.py`。以 `python3 scripts/wc_phase1 compare --help` 和 `refine --help` 的当前部署输出为参数核对入口。
