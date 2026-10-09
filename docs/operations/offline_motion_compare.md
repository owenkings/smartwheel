# 官方 EKF 的同会话运动修正对照

本文对应 `wc_phase1 compare --motion-candidate`。这是离线实验入口，不改变实时默认估计器、设备配置或原始录包。

## 1. 本次增加的能力

`compare` 可以让 `five_state` 与 `robot_localization` 共用同一份已经验证的 `motion_candidate.json`。省略 `--motion-candidate` 时保留原来的观测处理。

| 内容 | 处理方式 |
|---|---|
| IMU | 原生轴减候选零偏一次，再按冻结安装旋转转换到轮轴坐标；使用目标 z 轴角速度 |
| 轮反馈 | 先按原配置转换左右实际反馈，再令 `v'=v`、`w'=a*w+b*v` |
| 轮观测协方差 | `J=[[1,0],[b,a]]`，完整执行 `R'=J R Jᵀ`；保留速度与角速度的交叉项 |
| 官方库 | 仍使用实际安装的 robot_localization C++ Ekf；新增版本化完整轮协方差协议 |
| 五状态 | 原来的运动修正路径保留；数值基线不变 |
| 过程噪声 | `compare` 不改各估计器自身的过程噪声。五状态连续噪声只在 `refine` 独立比较 |
| 点云几何反馈 | 此参数不启用几何反馈；`--icp-shadow` 是诊断，不改变轨迹 |

官方 worker 的旧 `wheel` 协议保留。新适配器首先查询能力，再使用 `wheel_cov_v1` 传入两个方差和一个交叉协方差；旧 worker 不会静默忽略交叉项。

## 2. 构建与运行

在实际 clone 的工程根目录执行；不要求固定用户名或 U 盘。

```bash
source /opt/ros/humble/setup.bash
python3 scripts/wc_phase1 build
source install/main/setup.bash

python3 scripts/wc_phase1 compare --help
```

`--dataset` 指原录制的会话目录；`--motion-candidate` 指该会话自己的候选文件，不能换成另一段录制的候选。`data/` 路径按本机 `config/storage.local.json` 或项目存储配置解析。

下面选择今日同一份录包和候选；输出目录必须尚不存在。

```bash
DATASET="data/experiments/session_001"
CANDIDATE="data/analysis/fusion_session_001/refine_raw/motion_candidate.json"
RUN="$(date +%Y%m%d_%H%M%S)"

# 官方基线：不加 --motion-candidate。
python3 scripts/wc_phase1 compare \
  --dataset "$DATASET" \
  --output "data/analysis/official_baseline_$RUN" \
  --allow-partial --mechanical-initial --estimators robot_localization \
  --input-rate-hz 5 --filter on --cloud raw

# 同一输入的官方运动修正。
python3 scripts/wc_phase1 compare \
  --dataset "$DATASET" \
  --output "data/analysis/official_corrected_$RUN" \
  --allow-partial --mechanical-initial --estimators robot_localization \
  --input-rate-hz 5 --filter on --cloud raw \
  --motion-candidate "$CANDIDATE"
```

上面输出离线轨迹和可供原生建图使用的派生缓存。需要同步生成 RTAB-Map 时，在各命令中加上：

```bash
--native-map --native-rate-hz 5 --native-wall-interval-s 0.2 --domain 197
```

`--domain` 须选择当前空闲的独立值；禁止使用生产域 83。`--native-wall-interval-s` 只调整离线发送节奏，仍等待每帧确认，不更改原始测量时间。`--input-rate-hz 5` 是点云消费上限，不是雷达录制频率。

如需 filtered，仅把两条命令的 `--cloud raw` 都改为 `--cloud filtered`，并使用新的输出目录。只有输入、候选、外参、点云筛选和建图参数相同，才能将结果视为估计器或运动修正的对照。

长录包的派生缓存和地图会占用数 GB，运行前应确保所选数据盘有相应空间。若使用 `--memory-work`，输出必须是 `/dev/shm/wc_compare_$(id -u)/<新目录>` 的直接子目录；这是易失暂存，重启会丢失，须在核验归档后才清理。

## 3. 参数边界

- `--motion-candidate` 不会估计新候选；只验证并应用已经保存的同会话候选。
- 禁止同时使用 `--gyro-bias`，也禁止给已有确认零偏的运行配置再叠加候选。
- 候选必须通过内容摘要、设备身份、会话、IMU 安装轴、轮反馈转换、时钟映射和原始 IMU／轮反馈消息哈希检查。
- `--allow-partial` 仅显式允许研究已保留的完整短段，不会把原始会话改成 COMPLETE。
- 今日示例使用 `--mechanical-initial`，显式允许采用当前冻结的 机械安装初值进行离线实验。该选项不修改外参、不代表机械初值已获得独立几何精度验收；比较各组应保持该选项一致。
- 官方估计器不接受五状态连续噪声参数。当前也没有官方 EKF 加几何反馈的实现，不能把 `--icp-shadow` 当作几何反馈。
- 零偏和轮转向比例参考本会话静止段及转向数据，仍不是独立物理真值或自动生效的实车标定。

## 4. 输出与定位问题

| 文件 | 用途 |
|---|---|
| `motion_candidate.json` | 候选原字节快照，保留其 SHA-256 |
| `candidate_verification.json` | 原候选路径、归档快照路径、会话及逐项来源哈希验证 |
| `frozen_runtime_config.json` | 本次冻结配置；`offline_refinement` 指明候选、原噪声和无几何反馈 |
| `result.json` | 输入与算法哈希、实际官方库／worker 身份、各实验单元统计和失败原因 |
| `<单元>/trajectory.csv` | 独立轨迹，不依赖显示地图来查看路线 |
| `<单元>/resolved_estimator_config.json` | 实际展开的滤波参数和候选应用声明 |
| `<单元>/prior_initialization.json` | 坐标变换、初始化和零偏应用信息 |
| `<单元>/frontend/index.jsonl` | 每帧源关联、位姿、协方差、观测统计；轮观测包含交叉协方差与协议版本 |
| `native/<单元>/export/` | 显式启用原生建图后导出的二维图和三维点云 |

`OFFLINE_CANDIDATE_APPLIED` 表示候选确实进入观测处理。文件生成成功不代表地图精度验收通过。首尾距离和角度应与墙面直度、重访重影一起判断；没有真值时不能称为绝对定位精度。

## 5. 已执行软件验证（2026-10-07）

- Orin 上增量编译 `wc_estimation` 成功。
- 84 项适配器、候选来源、连续噪声、消费频率和官方库回归通过；追加的 15 项目标测试包含新增非单位轴旋转与候选冻结检查，全部通过（两轮测试存在重叠）。
- 原二进制与新二进制执行 1,688 条旧协议命令，输出逐字节相同。
- 含完整轮交叉协方差的官方 `ekf_node` 一致性检查：40 次轮观测和 40 次 IMU 观测均通过；包含启动、正常转向、冲突和异常拒绝。
- 软件测试没有启动设备或执行控制，真实录包和地图对照结果由对应实验目录另外记录。

源码改动：`mapping_compare.py`、`offline_motion_adapter.py`、`robot_localization_provider.py`、`wc_estimation/src/rl_worker.cpp`。本次没有切换实时默认估计器或删除五状态实现。

## 6. 单雷达离线建图（2026-10-08）


`compare` 和 `refine` 均支持 `--lidar left|right|all`（2026-10-08）。省略时沿用录包模式，无需重新录制或拆分原 bag。

| 参数 | 地图点云来源 | 其他输入 |
|---|---|---|
| `--lidar left` | 左雷达 | 完整 IMU、轮反馈 |
| `--lidar right` | 右雷达 | 完整 IMU、轮反馈 |
| `--lidar all` | 双雷达 | 完整 IMU、轮反馈 |

在工程根目录执行。此示例的运动修正候选仅属于这份 10 月 7 日录包；更换录包，应生成它自己的候选，或省略 `--motion-candidate` 跑未修正基线。

```bash
source /opt/ros/humble/setup.bash
source install/main/setup.bash
DATASET="data/experiments/session_001"
CANDIDATE="data/analysis/fusion_session_001/refine_raw/motion_candidate.json"
SIDE=left                     # 仅右侧改为 right；双雷达改为 all
CLOUD=raw                     # 也可选 filtered
OUT="data/analysis/single_${SIDE}_${CLOUD}_$(date +%Y%m%d_%H%M%S)"

python3 scripts/wc_phase1 compare \
  --dataset "$DATASET" --output "$OUT" --lidar "$SIDE" --cloud "$CLOUD" \
  --allow-partial --mechanical-initial \
  --estimators robot_localization --motion-candidate "$CANDIDATE" \
  --input-rate-hz 5 --filter on \
  --native-map --native-rate-hz 5 --native-wall-interval-s 0.2 --domain 198

# 完成后，在图形桌面终端查看2D/3D地图
python3 scripts/view_map "$OUT/native/robot_localization_hz5_filter_on/export"
```

逐次执行，每次使用新 `OUT`，不要在同一 ROS domain 同时运行两个原生建图。`data/analysis/` 按本机存储配置解析；当前 Orin 对应 `/media/USER/PROJECT_DISK/wheelchair/data/analysis/`，换机可使用自选普通文件夹。5 Hz 是离线消费上限，不改变原录包频率。完整回放会生成数 GB 的派生缓存，须保留足够空间。

`result.json` 的 `lidar_selection` 及 `frozen_runtime_config.json` 的 `offline_lidar_selection` 保存原模式、所选侧、排除侧和身份；`original_event_index.jsonl`、`<单元>/input/pairs.jsonl` 可核对实际来源，单侧 pairs 只有一个来源。另一侧的时间元数据用于保持原录包公共时间原点，其点云不进入地图。缺少所选侧时明确报错。原始档案不变。

左右单侧仍使用各自到车体的外参。当前左单侧和双雷达以左雷达为参考，右单侧以右雷达为参考；对比时先对齐参考原点并选共同有效时段，不能把固定平移当作漂移。单侧能帮助隔离双雷达拼接影响，但不保证建图更好。

`refine ... --lidar left` / `right` 同样支持单侧，可继续比较五状态运动修正、连续噪声和几何纠偏；官方 EKF + 运动修正用上面的 `compare`。官方 EKF 暂未接入几何反馈。

