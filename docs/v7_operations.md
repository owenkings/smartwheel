# V7 新版操作与问题定位

适用工程：`/home/nvidia/wheelchair`。五状态平面 EKF 保持默认。当前机械资料是标称布局，默认配置中的未知数据外参不补零；因此允许原生预览、采集和标定，阻止依赖它们的运动融合。安装版本、相机内参、测量同步和地图精度的验证状态彼此独立。

## 每次开始前

```bash
cd /home/nvidia/wheelchair
python3 scripts/wc_phase1 doctor
python3 scripts/map left --check-config
```

`doctor` 只读检查设备身份、占用、空间和外参能力。`map --check-config` 不打开任何传感器。修改 `config/hardware_setup.json` 后，下一次会话才读取；当前会话继续使用保存的配置快照。请先按 `docs/calibration/V7_geometry.md` 核实实物安装和变换证据。

原生预览使用 `python3 scripts/map left`、`right` 或 `all`。缺外参时不发布推测的 TF，RViz 的 Fixed Frame 选择 `lidar_left` 或 `lidar_right` 分别查看；无法同时物理叠加是明确的几何边界。2026-10-05 用户选择持久 `manual_controls.interaction_policy=hybrid_manual`：原生预览可由用户手推或使用 WASD，运动融合仍独立受外参门控。软件不会为允许驾驶而补填外参。

混合手动会话不需要每次点击允许或转换按钮：窗口前台、新鲜心跳、连续新鲜零轮速反馈满足后，收到释放 ACK 才显示手推；按住 WASD 自动接管；正常全部松键先发零命令，连续零反馈确认后恢复手推。失焦、断联、意图过期、停止按钮、退出和故障不按正常松键处理，不自动请求释放。已经释放时不能宣称制动已保持。独立采集不带 `--manual-drive` 仍只读；带该选项才启动同一轮接口下的键盘/手推 UI，不启动 SLAM 或运动估计。真实制动、远程断线和坡道行为仍待现场验收。

## 采一遍、保存证据

必须显式选择采集范围和时间，名称不得与既有档案重复：

```bash
python3 scripts/wc_phase1 capture --profile mapping_core --session v7_trial_001 --duration 60 --dry-run
python3 scripts/wc_phase1 capture --profile mapping_core --session v7_trial_001 --duration 60
python3 scripts/wc_phase1 capture --profile all_sensors --session v7_all_001 --duration 60
```

需要边采边手推/键盘驾驶时，在 Orin 桌面会话运行：

```bash
python3 scripts/wc_phase1 capture --profile mapping_core --session v7_manual_001 --duration 60 --manual-drive --dry-run
python3 scripts/wc_phase1 capture --profile mapping_core --session v7_manual_001 --duration 60 --manual-drive
```

该窗口只做采集状态与人工操纵。关闭窗口或 Ctrl+C 结束采集；先关闭控制窗口及唯一轮接口，再停其他源，最后排空录制器。手动窗口的计时从轮接口准备好后开始，初始化数据也保留。`--manual-drive` 不允许 `--diagnostic` 绕过轮接口、显示或控制配置缺失。不要另开第二套 WASD/轮反馈进程，设备锁仍然独占。最终清单保存真实控制发送次数；缺收尾证据时标未知，不写假零。

- `mapping_core`：双雷达 SDK 主机滤波前后点云、IMU 原包与串口读取批次、轮反馈事务、设备读回与配置快照。主机滤波前 XYZ 仍经过 SDK 深度转换，不等同于原始光学深度或 UDP 字节。
- `all_sensors`：额外四相机、四超声波地址。缺设备不会退化成较小配置。仅显式 `--diagnostic` 允许带缺项继续采集，结果保持不完整。
- 普通采集时长包含各源启动阶段；手动采集从轮接口就绪后计时，并额外预算 20 秒初始化容量。各源实际时间与计数在档案中，不能把请求 60 秒等同于每个设备均有 60 秒有效数据。
- 相机保存程序实际读到的、预览旋转之前的 BGR8 无损分块字节；8 Hz 只是预览。设备曝光是否丢帧未知，文件中不声称保存了 UVC 原始流。
- 超声波保留地址、请求、响应、CRC、超时。型号/寄存器单位未确认前只保留数值寄存器，不从无回复或未知单位生成米制距离。
- 开始前容量估算包含相机全部捕获帧；运行保留 2 GiB 收尾空间，触及阈值停止并标记不完整，不删除旧实验。

档案位于 `data/experiments/<session>/`。`capture_manifest.json` 的 `recording_complete=true` 才表示独立源端、索引和 SQLite 字节对账通过；不代表设备物理完整性、外参正确或定位准确。异常退出未写完收尾清单时不能被判断为完整。采集端使用自己的进程树包装器，父进程丢失时收尾设备；事后仍以档案和关闭证据判断结果。

采集专用点云使用 RELIABLE QoS 和有界录制队列。可靠 QoS 仍不能代替源端与落盘字节对账，超出队列或持久化失败仍必须标记不完整。2026-10-04 留存的两次超声波实测中，四地址每轮各 33 个事务均为 `VALID_RESPONSE`，这是历史通信证据；寄存器单位和探头物理角色仍未确认，本轮未重新驱动硬件检查。

重新逐字节核查留存档案：`python3 scripts/wc_phase1 doctor --session-root data/experiments/v7_trial_001 --verify-archive`。此操作只读原档案，复核源端、索引、袋内字节和配置文件哈希，不把最初不完整的采集事后升级为完整。

## 原生 IMU 零偏：10 + 10 + 10 秒

先由本人确认车体在整个源时间段内物理静止，采集一个有完整 IMU 与轮反馈的 `mapping_core` 档案。预热 10 秒、估计 10 秒、独立留出 10 秒；实际有效源时间必须至少覆盖 30 秒。请求时长还包括启动，宜按上面的 60 秒入口采集并核对实际源时间。已有 15 秒现场档案不能完成该协议，不能补写时间或静止声明使它通过。轮寄存器为零与 IMU 筛选只能排除部分异常，不能替代本人的物理静止观察。

下面只读提取待声明的时间范围，不打开传感器。需先建立 ROS 环境，以读取已保存的消息：

```bash
source /opt/ros/humble/setup.bash
source install/main/setup.bash
export PYTHONPATH="$PWD/src${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONNOUSERSITE=1
python3 - <<'PY'
import json
from wc_runtime.mapping_bias_confirm import capture_calibration_rows
rows, provenance = capture_calibration_rows('data/experiments/v7_trial_001')
if not rows:
    raise ValueError('没有可用的原生 IMU/轮反馈校准事件')
print(json.dumps({'start_stamp_ns': rows[0]['stamp_ns'],
                  'required_end_stamp_ns': rows[0]['stamp_ns'] + 30_000_000_000,
                  'recorded_end_stamp_ns': rows[-1]['stamp_ns'],
                  'source_time_span_s': (rows[-1]['stamp_ns'] - rows[0]['stamp_ns']) / 1e9}, indent=2))
PY
```

本人核对独立观察覆盖了整个区间后，才创建 `reports/v7_stationary_001.json`。必需字段为：`physically_stationary`（仅确认后填布尔 `true`）、`source`（`operator_visual_confirmation` 或 `external_stationarity_calibration`）、非空字符串 `evidence_id`、整数 `start_stamp_ns` 与 `end_stamp_ns`。声明开始不得晚于上述开始时间，结束不得早于所需结束时间；这些是档案映射后的源事件时间，不能填写当前墙钟或随意数值。可另存观察人、方法和备注。`evidence_sha256` 由入口对实际声明文件计算，无需手填。

```bash
python3 -m wc_runtime.mapping_bias_confirm \
  --capture-dataset data/experiments/v7_trial_001 \
  --stationary-evidence reports/v7_stationary_001.json \
  --output config/calibration/v7_gyro_bias_001.json
```

此入口不依赖安装外参，不启动硬件，也不修改当前会话。输出文件名必须尚不存在，父目录必须已有。仅留出筛选通过才保存零偏与同名 `.evidence.json`；门限仍是实验候选，`HOLDOUT_PASSED` 不表示绝对精度已验收。后续会话需显式使用该零偏文件，不能只因文件存在便视为已经应用。

## 用同一数据对照

先补齐所需变换证据，再运行：

```bash
source /opt/ros/humble/setup.bash
source install/main/setup.bash
python3 scripts/wc_phase1 compare --dataset data/experiments/v7_trial_001 --output reports/v7_compare_001 --native-map
```

默认只改变估计器：`five_state` 与 `robot_localization`。轮/IMU 测量、零偏、外参、时间映射、点云和地图参数一致；轨迹、创新、拒绝计数、缺段、原始输入与输出哈希全部保存。`--native-map` 运行隔离 ROS 域下的 RTAB-Map 并关闭、导出实际地图，未加此开关时只执行前端/轨迹比较，报告明确写 `NOT_RUN`。

后续单因素实验使用 `--estimators five_state --input-rate-hz 5 10`、`--filter off on`、`--icp-shadow point_to_plane`。ICP 默认关闭，只给离线约束质量和拒绝原因，不改变正在运行的车体估计器。不要把多因素同时改变后的效果归因于单个参数。该链路仅覆盖同一 SDK 输出之后的算法，未实现完整旧/新 SDK 原始网络包回放。

同一主机单调时钟映射为事件时间，原始设备时间、主机墙钟和序号保留。序列乱序、缺段和新源 epoch 都应在报告中检查。这不证明设备曝光、激光测量与反馈真实时间同步。没有独立真值时，绝对精度和两种 EKF 等效性都是“待评估”，不根据地图看起来更顺自动切换默认。

提供 `--truth-json` 时，各组使用“真值与所有对照组共同有效时刻”评价；同时记录每组缺失、剔除及无效样本，不允许各挑不同片段。`--truth-min-common-samples` 是预声明的最少共同样本数；不足时对照失败，但轨迹、真值诊断和失败清单仍保留。`rotation_error_deg` 为完整 SO(3) 旋转误差，航向误差另列，二者不混称。

地图导出附带 `map_quality.json`：数据库完整性、移动量、连通图、逐节点扫描、实验门限和数据库哈希分别记录。候选移动地图门限不代表独立几何精度；单节点静态图可以保存，同时显示“不满足移动地图实验判据”。当前点云比较使用的默认原生地图消费频率与设备输出频率分别记录。

## 问题 → 证据 → 修改位置 → 复测

| 观察到的问题 | 检查证据 | 应检查或修改的位置 | 复测 |
|---|---|---|---|
| 左摄像头未枚举 | doctor 枚举、预期 USB 3.1/3.2 路径、相机日志 | 先交换右侧正常线缆/端口，核对 `config/cameras.json`；不能按相同序列号替代绑定 | `all_sensors --dry-run`，再诊断采集查看每路 BGR 归档 |
| 超声波无回复 | `sources/ultrasonic/events.jsonl` 中请求、响应、CRC/超时和地址 | `config/ultrasonic_capture.json` 与实际型号、线路、总线地址；不能直接认定探头损坏 | 核实硬件后显式诊断采集，四地址各有协议有效响应 |
| 外参缺失 | `capability_assessment.json`、health 原因和参数路径 | `config/hardware_setup.json` 的对应 transform 与证据文档 | 配置检查，再静态双云/已知转向验证；不以外形中心替代真实坐标系 |
| 地图仍在但数据过期 | `health.json` 中 source age、区间与 lifecycle；源端统计 | 按受影响来源先检查连接，再查处理负载/队列 | 相同档案回放区分硬件采集故障和算法问题 |
| 源端有数据，录包缺失 | `records.jsonl`、源端独立序号、recorder_summary 与清单 issues | `source_recorder.py` 队列、DDS 订阅就绪、写盘/剩余空间 | 合成源尾帧测试，现场再次采集对账；不要忽略 PARTIAL |
| EKF 拒绝或漂移 | 轨迹、残差/NIS、接受/拒绝计数、轮寄存器原事务、IMU 原包 | `mapping_planar.py`、`measurement.py`、零偏证据、`planar_ekf` 参数 | 固定同一输入，仅改一个因素；5.0 是待验门限，非已标定结论 |
| 雷达“稀疏” | 源端 raw/filtered 次数与点数、配对次数、建图消费及显示统计 | 设备实际读回、`.xtcfg` disposition、SDK 主机过滤、应用过滤分别查 | 同场景固定参数分别测设备输出与5/10 Hz消费，不能用显示帧率代替设备频率 |
| 改代码后行为没变 | configuration/software_hashes、启动命令、实际模块路径 | `cli.ros_command` 在 ROS 环境建立后优先项目 src；原生组件必须 rebuild | 重新会话并核对源码/安装产物哈希 |

每次会话的 `health.json`、`diagnosis_zh.md` 给出观察、影响功能、待查假设、证据和复测方法。高频原始字节在档案，UI 只读取摘要。

## 留存、回退与验收边界

地图保存默认 `--retention-profile experiment`：保存地图并以哈希引用原会话的 bag/input，不复制大文件、不自动清理。引用指向原始档案，手动移动/删除它会破坏重放。显式 `--retention-profile map_only` 才清理原始档案并记录不可重放。

运行 `python3 scripts/map all --mapping true` 才实际建图（仍要求必要外参齐全）。关闭窗口或 Ctrl+C 后先停设备，再显示保存对话框；无可用桌面则在终端询问。选择“保存”才导出，选择“废弃本次结果”删除本次地图和原始大数据、保留轻量诊断，不碰历史实验；关闭询问窗口、稍后决定、EOF 或中断都记 `SAVE_PENDING`，不是默许保存或删除。可用 `python3 scripts/save_map <本次工作目录>` 恢复待决定流程。已经保存并被地图引用的原始档案不会再作为未保存结果废弃。

2026-10-05 手动采集升级备份位于 `reports/v7_manual_capture_20261005/`，此前 V7 基础升级仍保留在 `reports/v7_upgrade_20261004T131319Z/`。各自 `baseline/source.tar.gz` 包含该轮修改前实际源码，`deployed_state.json` 记录该轮增改文件。查看本轮精确回退：

```bash
python3 scripts/v7_rollback.py --report reports/v7_manual_capture_20261005
```

先通过项目正常停止入口结束预览、采集、控制、回放、离线查看、比较和构建，不在回退期间启动新的任务；核对后加 `--apply` 恢复。脚本占用项目实际源、处理、控制、编码器、相机、轮反馈、离线查看、构建锁与已存在的会话/串口/端口锁，忙锁即拒绝，绝不强杀进程。前端离线比较本身没有统一运行锁，新域首次创建锁也存在并发边界，因此不能把获得文件锁等同于所有进程已停止。

回退只处理 `deployed_state.json` 实际列出的文件，数量随本轮增补动态变化；部署后的其他修改会使其拒绝覆盖。新增文件移入 `rollback_retained_<时间>/` 保留，采集档案不动，不执行 git reset。旧文件先完整写入并同步暂存，再保留当前文件、原子替换；空间不足的准备阶段不移动源码。保留区的 `rollback_status.json` 记录进度。若状态是 `INTERRUPTED_DO_NOT_START_PROJECT`，先依据日志和保留文件核对并恢复一致性，不启动混合版本，也不要直接重新运行覆盖已有变更。

源码回退完成状态是 `SOURCE_RESTORED_REBUILD_REQUIRED`。按项目 build 命令重新构建 `wc_bringup wc_xt_driver wc_camera_panel`，再使用旧程序；源码回退本身不把当前安装产物变成旧版本。

验收记录要分别写“软件检查”“录制完整性”“现场采集”“动态几何/控制”。直线、左右转弯、启停、闭环路线与手推/制动只由用户现场实施。没有参考测量时保留“待评估”；软件编译和合成数据通过均不能替代这些现场结论。

