# XT-M60 双机光学串扰定位与缓解报告

时间：2026-07-29（Asia/Shanghai）

## 结论

**本节已按同日后续 30 分钟长测修正。** 早先 65 秒结果只能证明某个
短时窗口良好，不能作为长期双路验收。两台未硬件同步的 Flash ToF 雷达
确有可复现光学串扰，`50 ms` 主机侧错峰能显著降低部分逐帧跳变，但长测
证明它不是左雷达低有效点的唯一根因。

当前生产缓解方案不自动写设备配置：

- 两个 SDK 进程的 `start()` 对齐到共同的 `100 ms` 主机单调时钟网格；
- 左雷达相位 offset 为 `0 ms`；
- 右雷达相位 offset 为 `50 ms`；
- 两路每 `30 s` 短暂停止并重新按网格启动，限制独立设备时钟漂移；
- `enable_sdk_filters=false`、`apply_device_config=false` 保持不变；
- 左右 `miniAmp` 最终仍为 `70`。

早先 65 秒双路验证如下，保留为短时历史证据：

| 指标 | 左 XT-M60 | 右 XT-M60 |
| --- | ---: | ---: |
| ROS 频率 | `9.867 Hz` | `9.839 Hz` |
| 平均有效点比例 | `91.72%` | `97.11%` |
| 距离逐帧变化中位数 | `30.0 mm` | `16.0 mm` |
| 距离逐帧变化 P95 | `131.0 mm` | `67.0 mm` |
| 最大接收间隔 | `0.214 s` | `0.289 s` |
| PointCloud2 | 有组织 `160x60` XYZI | 有组织 `160x60` XYZI |

右侧单机 ROS 对照为 `97.07%` 有效率、`11.0 mm` 中位变化和
`51.0 mm` P95；错峰策略能把右主雷达恢复到接近单机质量。**但不能由此
推断左路长期恢复，更不代表硬件同步已经完成。**

后续 30 分钟双路质量门长测得到：

| 指标 | 左 XT-M60 | 右 XT-M60 |
| --- | ---: | ---: |
| 质量话题频率 | `9.822 Hz` | `9.429 Hz` |
| 全部原始帧平均/中位有效点 | `46.38% / 45.56%` | `97.25% / 97.33%` |
| 质量门接受率 | `21.21%` | `98.01%` |
| 接受后主点云频率 | `2.084 Hz` | `9.241 Hz` |
| 最大接受帧空窗 | `221.85 s` | 无同等级空窗 |

因此 30 分钟双路验收**失败**。质量门正确地隔离了大部分左路坏帧，但
不能把坏数据修成好数据；正式建图不得把该长测判为通过。

进一步排查证明当前低有效点跟随左设备：

- 左雷达关闭右路、独立运行 10 分钟：平均/中位有效点
  `46.16% / 46.14%`，接受率仅 `0.42%`；
- 左路温度约 `57.2 -> 64.9 C` 时有效点反而缓慢上升，不能用“越热越差”
  解释当前故障；
- 直接 SDK 原始 ImageType-4 左路 60 帧：有效点 `48.03%`，幅值中位数
  `179`；
- 同场景右路 60 帧：有效点 `96.90%`，幅值中位数 `511`；
- 左路大量原始距离直接为厂家失效码 `964009/964001`，不是 ROS、TF、
  PointCloud2 或质量门造成的丢点。

只读配置对比还发现左路第三 HDR 曝光曾从已验证的 `30 us` 回退为
`20 us`。按已授权的序列号门禁流程恢复为 `30 us` 后，断开重连和一次
测量后的再次重连均确认保持；然而左路直接 SDK 有效点仍只有约 `48%`。
配置回退是真实缺陷，但不是剩余故障的主因。当前故障域收敛为左设备
光学窗口/VCSEL 发射与接收链、线缆供电或需要完整断电复位的设备状态。

## 根因证据

### 当前状态可复现性

新的只读 SDK 信号诊断同时分析 `distData`、`rawdistData`、点云、幅值失效码
和固定像素区域。2026-07-29 当前环境下：

- 左侧约 `90.7%` 有效点；
- 右侧约 `97.5%` 有效点；
- 左侧无论原始深度还是处理后深度，有效率都约 `91%`；
- 无效区域主要集中在特定边缘/右半视场块，而不是 ROS 随机丢包。

这说明历史 `60.54%` 不是左设备在所有场景下的永久上限。此前将左侧第三
HDR 曝光从 `20 us` 匹配到右侧 `30 us` 的写入仍是有效改善，但不是完整根因。

### 幅值门限 A/B 被否决

序列号门禁脚本仅对左设备执行 `miniAmp 70 -> 50 -> 70`，三段各取 60 帧，
并在正常路径和 `finally` 中双重恢复：

| 阶段 | 有效点比例 | 距离变化中位数 |
| --- | ---: | ---: |
| 控制 `70` | `91.24%` | `21 mm` |
| 测试 `50` | `92.51%` | `22 mm` |
| 恢复 `70` | `91.18%` | `20 mm` |

降低门限只增加约 `1.3` 个百分点，并略增抖动，不能解释历史差距，也不能
把左侧追平右侧。最终读回 `miniAmp=70`，没有保留该改动。

### 左单机、双机、左单机 A/B

同一位置连续测试：

| 阶段 | 左有效点 | 左距离变化中位数 | 左 P95 |
| --- | ---: | ---: | ---: |
| 左单机（前） | `90.07%` | `23 mm` | `85 mm` |
| 双机同时 | `90.67%` | `143 mm` | `1646 mm` |
| 左单机（后） | `90.67%` | `23 mm` | `81 mm` |

双机阶段左侧还有约 `2.5%` 点被推到 `12 m` 建图范围外。停止右侧后立即
恢复，排除了持续环境变化、ROS 转换和左设备永久故障。

右雷达的 ROS 单机对照同样明确：

- 单机距离变化中位数/P95：`11/51 mm`；
- 未正确错峰的双机生产测试：`38/793 mm`。

### 启动 offset 扫描

两个只读诊断进程连接完成后，使用共同主机单调时钟计划 `sdk.start()`：

| 右相对左启动 offset | 左中位变化 | 右中位变化 |
| ---: | ---: | ---: |
| `0 ms` | `451 mm` | `360 mm` |
| `25 ms` | `24 mm` | `26 mm` |
| `50 ms` | `29 mm` | `13 mm` |
| `75 ms` | `39 mm` | `13 mm` |

`50 ms` 为当前综合最优，并保护质量更高、计划作为单雷达 LIO 主输入的右侧。
300 帧直接 SDK 长测仍为左/右 `32/20 mm`，但 30 秒内接收相位漂移约
`7.4 ms`，所以生产实现增加了每 30 秒一次的共同网格重对齐。

## 软件改动

- `xtm60_adapter_node.py`
  - 增加可关闭的 `phase_mode=grid`；
  - 增加 `phase_period_sec`、`phase_offset_sec`、
    `phase_realign_interval_sec`；
  - 修复迟到的 SDK `0xFE` 连接事件误清除已开始测量状态、造成二次启动的竞态；
  - 网格模式只控制主机调用 `start()/stop()` 的时机，不调用设备配置写 API。
- `xtm60_left.yaml`
  - `phase_mode=grid`、周期 `0.1 s`、offset `0.0 s`、重对齐 `30 s`。
- `xtm60_right.yaml`
  - `phase_mode=grid`、周期 `0.1 s`、offset `0.05 s`、重对齐 `30 s`。
- `sensors.launch.py`
  - 保留左右 phase 话题重映射，供后续诊断/可选 follower 模式使用。
- 新增或扩展诊断：
  - `xtm60_signal_diagnostic.py`
  - `xtm60_min_amplitude_ab.py`
  - `xtm60_dual_phase_ab.py`
  - `xtm60_phase_runtime_analysis.py`
  - `xtm60_sdk_capabilities.py`
  - `xtm60_cloud_diagnostic.py`

后续又增加坏帧质量门、温度/VCSEL 温度诊断、只读序列号与关键成像配置
启动保护。关键行为：

- `/xtm60/<side>/points` 只发布质量门接受帧；
- 完整拒绝帧仅发布到 `/xtm60/<side>/points_rejected`；
- `/xtm60/<side>/quality` 发布轻量逐帧 JSON；
- 启动前只读核对 serial、四档曝光、HDR、minAmp 和 fps；
- 任一稳定字段漂移时保持停测，不自动改写设备；
- 正式配置把绝对有效点下限提高到 `75%`，禁止把左路当前低质量状态
  学成“正常基线”。

`test_xtm60_adapter.py` 最终 `21/21` 通过；
`wheelchair_sensors`、`wheelchair_bringup` 清洁增量构建通过。

真实 45 秒启动冒烟确认两路 serial/配置核对均通过、双路按
`0/50 ms` 启动，结束后服务 `inactive`、无适配器进程、双 UDP 静默。
该次左路平均有效点仍只有 `54.28%`，右路为 `97.44%`，再次证明保护
生效但硬件链问题未消失。

## 限制与建图使用建议

- 这是主机侧串扰缓解，不是硬件触发同步，也不证明两台设备时间戳同源。
- 质量门已经实现并在真实双路运行中生效；当前 `75%` 绝对下限会把左路
  大多数帧隔离。它是安全保护，不是成像质量修复。
- 65 秒平均良好但 30 分钟失败，阶段结论必须以长测为准。
- 每 30 秒重对齐产生约 1～2 帧空窗。两路整体仍约 `9.8 Hz`，但 FAST-LIO2
  尚未验证对该空窗的行为。
- C2 单雷达 FAST-LIO2 首轮只可考虑右 XT-M60；在用户已明确暂缓
  LiDAR–IMU 外参的条件下，本报告不把 FAST-LIO2 实机验收列为已完成。
- 双雷达 map-only 阶段可以使用当前网格缓解，但最终产品仍应向厂家确认
  正式的多机硬件同步、触发或安全调制频率分配方案。
- 本任务没有改变 LiDAR–IMU 外参、双雷达最终外参、轮椅运动许可或电机状态。

## 关键证据

- `docs/hardware/evidence/XT_M60_LEFT_MIN_AMP_70_50_70_AB_20260729.json`
- `docs/hardware/evidence/XT_M60_LEFT_INTERFERENCE_SINGLE_BEFORE_20260729.json`
- `docs/hardware/evidence/XT_M60_LEFT_INTERFERENCE_DUAL_20260729.json`
- `docs/hardware/evidence/XT_M60_LEFT_INTERFERENCE_SINGLE_AFTER_20260729.json`
- `docs/hardware/evidence/XT_M60_DUAL_PHASE_AB_20260729.json`
- `docs/hardware/evidence/XT_M60_DUAL_PHASE_50MS_LONG_RETRY_20260729.json`
- `docs/hardware/evidence/XT_M60_RIGHT_ONLY_ROS_CONTROL_30S_20260729.json`
- `docs/hardware/evidence/XT_M60_LEFT_PHASE_GRID_REALIGN_65S_20260729.json`
- `docs/hardware/evidence/XT_M60_RIGHT_PHASE_GRID_REALIGN_65S_20260729.json`
- `docs/hardware/evidence/XT_M60_PHASE_GRID_REALIGN_ANALYSIS_20260729.json`
- `docs/hardware/evidence/XT_M60_PHASE_GRID_REALIGN_LOG_20260729.txt`
- `docs/hardware/evidence/XT_M60_QUALITY_SOAK30M_20260729_LEFT_QUALITY.json`
- `docs/hardware/evidence/XT_M60_QUALITY_SOAK30M_20260729_RIGHT_QUALITY.json`
- `docs/hardware/evidence/XT_M60_QUALITY_LEFT_THERMAL_10M_20260729_LEFT_QUALITY.json`
- `docs/hardware/evidence/XT_M60_LEFT_SIGNAL_LOW_STATE_20260729.json`
- `docs/hardware/evidence/XT_M60_READONLY_CONFIG_SNAPSHOT_20260729.json`
- `docs/hardware/evidence/XT_M60_LEFT_HDR_RESTORE_20260729.json`
- `docs/hardware/evidence/XT_M60_READONLY_CONFIG_AFTER_SIGNAL_20260729.json`
- `docs/hardware/evidence/XT_M60_LEFT_HDR_RESTORE_SIGNAL_20260729.json`
- `docs/hardware/evidence/XT_M60_RIGHT_CURRENT_CONTROL_20260729.json`
- `docs/hardware/evidence/XT_M60_QUALITY_CONFIG_GUARD_SMOKE_20260729_SERVICE_LOG.txt`

## 停机

最终任务结束检查：

- transient service `inactive`；
- 无 `xtm60_adapter_node`；
- 左右 `192.168.0.100:7687`、`192.168.1.100:7687` 三秒监听均无 UDP 帧；
- `shutdown_verification=PASS`。
