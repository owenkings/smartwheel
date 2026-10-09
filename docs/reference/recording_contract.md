# 完整记录与离线反复建图

本手册对应 2026-10-06 当前源码的独立 `capture`、归档审计、`compare` 和历史压缩恢复入口。下面的命令是操作步骤，不是本次现场通过报告；每次采集都以该会话自己的清单、源端记录、磁盘同步和重新审计结果为准。

独立采集不运行 EKF、ICP 或 SLAM，也不要求雷达安装外参已齐全。先保留完整记录，之后可用同一份数据多次运行离线融合、估计器比较和 RTAB-Map 建图；运动融合仍要求相应外参、来源证据和配置检查通过。

原始记录强制保留。后续地图保存选择只决定派生地图的去留，不删除本次录制归档。

## 1. 在哪里运行

以下都是 **Orin 的 Bash 命令**。带 `--preview` 或 `--manual-drive` 的采集在 nvidia 图形桌面终端运行，需要可用的显示环境。不要在 Windows PowerShell 直接复制执行。

```bash
cd /home/nvidia/wheelchair
source /opt/ros/humble/setup.bash
source /home/nvidia/wheelchair/install/main/setup.bash
export PYTHONPATH="/home/nvidia/wheelchair/src${PYTHONPATH:+:$PYTHONPATH}"
```

同一时间仅运行一份实际采集/驾驶程序。设备占用或身份检查阻断时，先确认现有程序和本会话日志；不要手工删设备锁，也不要同时打开旧工程、另一个相机程序或第二个轮串口读取者。

## 2. 直接请求录制 300 秒

当前使用 `mapping_core`：只录左右雷达、IMU 和左右轮寄存器反馈，**不启动、不录制相机或超声波**。先前包含这些来源的档案仍保留。以下命令直接录制，无需先运行 `--dry-run`，也无需以预测容量 `sufficient` 为条件再启动。

带 `--manual-drive` 时，本会话授权用户 WASD／手推；`--preview` 只显示所选双雷达。使用新会话名：

```bash
SESSION="core_300_$(date -u +%Y%m%dT%H%M%SZ)"
DATASET="/home/nvidia/wheelchair/data/experiments/$SESSION"

python3 scripts/wc_phase1 capture \
  --profile mapping_core --session "$SESSION" --duration 300 \
  --manual-drive --preview
```

设备身份、占用、配置以及真实可用空间／内存保护仍会自动执行。按请求时长预测的容量仅作 **advisory（参考估计）**，不因估计“录满 300 秒不够”而阻断启动；运行中根据实际积累的数据和剩余资源决定是否需要提前收尾。**300 秒是请求的共同窗口，不保证实际一定录满，也不是无限录制。**

想提前结束时，按 **一次 Ctrl+C**，等待停止来源、排空、转存和审计完成；不要连续中断或强杀进程。不足请求窗口时结果仍为 `PARTIAL`，原始数据保留。

## 3. 可选：包含相机和超声波的采集

只有希望记录这些来源时才使用 `all_sensors`。它请求左右雷达、IMU、左右轮寄存器反馈、四相机和超声波地址 1–4。相机角色为 `left_front`、`right_front`、`left_side`、`right_side`；超声波地址是逻辑身份，探头方位和量纲仍需独立确认。可直接请求 30 秒，例如：

```bash
SESSION="all_30_$(date -u +%Y%m%dT%H%M%SZ)"
DATASET="/home/nvidia/wheelchair/data/experiments/$SESSION"
python3 scripts/wc_phase1 capture \
  --profile all_sensors --session "$SESSION" --duration 30 \
  --manual-drive --preview
```

需要较长窗口时，使用另一个新会话名直接请求 180 秒：

```bash
SESSION="all_180_$(date -u +%Y%m%dT%H%M%SZ)"
DATASET="/home/nvidia/wheelchair/data/experiments/$SESSION"

python3 scripts/wc_phase1 capture \
  --profile all_sensors --session "$SESSION" --duration 180 \
  --manual-drive --preview
```

需要排查身份、占用、配置或显示时，`--dry-run` 可作为额外诊断选项，不是录制前的必经步骤。它不打开采集设备、不查询超声波、不发送轮控制、不创建采集档案；预测容量只是参考。超声波诊断的 `probe_response=NOT_QUERIED` 表示还没有响应证据，不能推断四个探头正常。

**如果超声波任一地址没有有效响应，`all_sensors` 不会自动改成较少来源的采集。** 启动阶段可能无法进入共同就绪窗，运行中也可能因来源退出而失败；已产生的数据保留并标为 `PARTIAL`。不要用空记录、零距离或旧响应填补缺失。`--diagnostic` 只用于明确的不完整诊断，不能让缺失来源通过完整性验收。30／180 秒都只是请求长度，运行资源可能要求提前结束；不依赖相机压缩必然节省多少空间，不降低完整性要求。

## 4. 驾驶、手推与预览

| 参数/操作 | 当前行为 |
|---|---|
| 不加 `--manual-drive` | 只读采集；轮读取仍有固定反馈查询，但不会启用 WASD 控制。 |
| 加 `--manual-drive` | 本会话明确授权用户 WASD/手推，启动独立 Qt 界面；唯一轮串口 owner 同时读反馈、发送控制并记录真实发送证据。 |
| 窗口前台正常操作 | 服务就绪后无需另按连接/启用按钮；按住 WASD 操作。启动空键状态及正常松键后的手推释放须等待连续新鲜零反馈和真实协议回执。 |
| 空格、失焦、断线或故障 | 进入停控/解除用户意图路径；空格暂停自动释放，失焦/断线不触发自动手推释放。界面的软件状态不证明物理制动或断线停车已验收。 |
| 提前关闭手动驾驶窗口 | 停止本次采集并进入收尾；不足请求的共同时间窗时保留 `PARTIAL`，不冒充完整 300 秒或其他请求时长。 |
| `--preview` | 只订阅已有采集源，不另开 SDK、USB 相机或轮串口；默认约 3 Hz 显示，两侧原生坐标各一个 RViz 视图并显示所选相机。 |
| 关闭预览窗口 | 退出预览；采集继续。驾驶窗口和预览窗口的关闭语义不同。 |

双雷达预览分别使用 `lidar_left`、`lidar_right` 原生 frame，没有借未知外参把两幅点云叠到一个坐标系。显示降频只影响预览；录制继续保留来源实际收到的帧。只有请求相机的 profile 才使用 `monitor_320` 录制：320×240、请求 30 fps；相机原有 ROS 预览发布约 8 Hz，独立显示桥再降到约 3 Hz，二者都不是原始相机归档帧数。当前 `mapping_core` 没有相机图像录制或预览。

预览新鲜度写入 `capture_preview_status.json`，本次预览实例另有 `preview/<preview_epoch>/status.json`。`source_status` 按 `lidar_left/right` 和所选 `camera_<role>` 分开记录 `last_received_monotonic_ns`、`age_ns/age_s` 和状态：

| 预览来源状态 | 含义 |
|---|---|
| `WAITING` | 尚未接受本会话该来源的有效预览消息；时间/年龄为null。 |
| `FRESH` | 最近1秒内收到符合本会话身份/原生frame检查的新消息；不代表测量时间已同步、点云几何准确或磁盘录制完整。 |
| `STALE` | 曾收到消息，但超过1秒未更新；相机身份诊断失效也会停止显示新帧并标过期。 |

状态变化会在预览进程日志打印 `PREVIEW_SOURCE_STATE_CHANGED`。过期雷达发送一次空的**派生预览**点云来清除旧画面，保留原生frame和点字段布局；清屏单独计入 `stale_clear_counts`，不算新数据帧，也不改bag/原始消息/录制结果。恢复后只有收到新帧才发布，旧帧不会被循环发送来伪造显示频率。相机Image面板可能保留最后图像，应同时看该路状态；这些预览状态完全独立于capture完整性。预览实例整体为`CLOSED`时，文件是退出记录，不是仍在运行的设备健康显示。

`capture --preview` 默认让两个 RViz 子进程使用 Mesa 软件渲染（`LIBGL_ALWAYS_SOFTWARE=1`、`GALLIUM_DRIVER=llvmpipe`、GLX vendor=`mesa`）；采集源和桌面环境不受这些覆盖影响。`rendering.selected_backend` 及 `environment` 记录所选后端，`rendering.windows.left/right` 保存逐侧日志证据与错误。只有日志提供实际 renderer 身份时才记录可验证的 `observed_backend`；仅打印 OpenGL 版本时实际后端仍为 `UNKNOWN`。

整体 `SUBSCRIBING` 和 `capture_preview_ready.json` 的 `ready_scope=SUBSCRIPTION_BRIDGE` 只表示订阅桥已启动，不能证明窗口画面正常。上下文失败、`GLXBadDrawable` 等渲染错误会打印 `PREVIEW_RENDER_STATE_CHANGED` 并将预览标为 `FAILED`，即使来源仍为 `FRESH`；停止时新出现的渲染错误也保留。两侧实际画面仍需观察验收，`visible_pixels_verified=false` 不会因订阅或 GL 版本输出自动变成成功。

仅需静止采集时，省略 `--manual-drive`。若无需图形显示，再省略 `--preview`。例如：

```bash
SESSION="core_static_$(date -u +%Y%m%dT%H%M%SZ)"
DATASET="/home/nvidia/wheelchair/data/experiments/$SESSION"
python3 scripts/wc_phase1 capture \
  --profile mapping_core --session "$SESSION" --duration 300
```

## 5. 时间窗、2 GiB 边界和收尾

流程为：冻结配置/源码 → recorder 就绪 → 启动来源 → 全部请求逻辑来源至少收到有效数据 → 记录完整共同时间窗 → 停控/停止来源 → 等待发布及记录排空 → 转存、同步、审计、提交最终清单。

- 来源共同就绪的当前等待上限为 30 秒；手动轮 owner/UI 的单独就绪上限为 20 秒。`--duration` 从全部请求来源就绪之后开始计时，准备期间已有数据也保留；停止、转存和审计还需时间，进程运行总时间可能超过请求时长。
- 结束处保留各来源到达目标终点或之后的覆盖样本；请求窗口、首末有效到达和最大间隔会被重新核验。共同窗口使用原始主机 monotonic 到达时间，不能据此宣称设备曝光、雷达测量或轮反馈已实现物理同步。
- 请求时长对应的磁盘／内存容量估计仅为 advisory，不以“预计整段放不下”作为启动门。真实空间保护保留 **2 GiB = 2,147,483,648 bytes** 的磁盘／tmpfs边界；转存时也逐文件检查余量。这是应用检查，不是操作系统配额，也不保证可以无限录制。
- 当前 CLI 默认 `--staging memory`：先录 `/dev/shm/wc_capture/$SESSION`。运行时按已积累的 RAM 记录、待复制的雷达来源日志及真实磁盘剩余量核算收尾空间，同时检查 tmpfs和主存可用量；接近保留边界时停止采集并进入排空／转存，可能录不满 300 秒。主存保留至少 4 GiB或总内存的10%，有界队列仍有自身预算。停止来源后复制到 `$DATASET`，逐文件检查哈希并 fsync；最终磁盘审计通过之前不会从 RAM 发布 `COMPLETE`。
- 完成且磁盘审计通过后，只清除已核验的 RAM 临时副本，原始归档仍在磁盘保留。失败时 RAM/磁盘已有结果尽量保留；tmpfs断电或重启不持久，不能把仅存在 RAM 的数据当成永久档案。
- 可显式用 `--staging disk` 直接写磁盘，身份、队列、时间窗及严格失败门不变；磁盘写入性能不足仍会留下 `PARTIAL`。

来源窗口的当前代码间隔硬门为雷达 0.5 秒、IMU 0.1 秒、轮反馈 0.5 秒、相机 0.2 秒、超声波 3 秒。这些是完整性检查参数，不是传感器精度或产品性能标准。最终 `COMPLETE` 还要求身份、序号、源端/录包对账、关闭、同步与保留文件哈希全部通过。

不要因为预览还在更新就认定磁盘已完整。用户按一次 Ctrl+C或资源边界触发停止后，等待终端完成排空、`TRANSFERRING`、`VERIFYING` 并给出最终 `COMPLETE`／`PARTIAL`，再使用归档；不要连续中断或强杀来源来加快收尾。提前停止仍按请求时长验收，缺少窗口的数据保留并标为 `PARTIAL`。

## 6. 去哪里找每个来源

最终目录是 `/home/nvidia/wheelchair/data/experiments/$SESSION`。直接查看当前/最终阶段：

```bash
python3 -m json.tool "$DATASET/capture_manifest.json"
python3 -m json.tool "$DATASET/progress.json"
```

默认内存暂存期间，进度文件在 `/dev/shm/wc_capture/$SESSION/progress.json`。转存和最终审计后以磁盘目录为准。

| 路径（相对会话目录） | 用途 |
|---|---|
| `capture_manifest.json`、`summary_zh.md` | 请求范围、状态、共同时间窗、进程退出、真实控制计数、问题及保留文件哈希。 |
| `configuration/capture_contract.json` | 冻结的逻辑来源、sensor/device身份、寄存器/地址及窗口检查参数。 |
| `configuration/source_selection.json` | 本次 selected/excluded 范围，避免把部分 profile 认作全部传感器。 |
| `configuration/data_capabilities.json` | 当前采集边界：各来源实际记录的字节/字段与无法获得的内容，避免把SDK XYZ称为原始UDP或把BGR称为原始UVC；也记录未确认超声波方位/单位等限制。 |
| `configuration/` | 本次运行、硬件、轮、雷达配置及标定证据快照；相机／超声波配置仅在对应 profile 请求时存在，手动会话另有 `manual_runtime.json`。 |
| `software/source_and_install.tar.gz`、`software/files.json` | 实际源码/脚本/配置/测试/文档/SDK/安装文件快照及逐文件身份；Git状态和依赖另存于 `software/`。 |
| `bag/*.db3`、`bag/metadata.yaml`、`records.jsonl` | 原始ROS消息及序号/时间/CDR哈希索引。 |
| `sources/lidar/{left,right}/<epoch>/` | 分侧设备/配置/原始与SDK处理点云来源记录；各侧 `bag_index.jsonl` 指向原始bag。 |
| `sources/imu/` | H30来源事件/摘要及 `bag_index.jsonl`。 |
| `sources/wheel_feedback.jsonl`、`sources/wheel_summary.json` | 唯一owner真实查询、反馈和控制证据；`sources/wheel/{left,right}/index.jsonl` 区分两个寄存器。左右物理/比例验收状态仍保留原值。 |
| `sources/cameras/<role>/frames.jsonl`、chunk及summary | 仅请求相机时存在：每路成功捕获、解码后且预览旋转前的BGR8帧，逐帧身份、序号、时间和哈希。 |
| `sources/ultrasonic/identity.json`、`events.jsonl`及summary | 仅请求超声波时存在：实际打开的USB总线身份、各地址真实请求/响应及退出证据；`address_1`–`address_4/index.jsonl` 分开索引。 |
| `ready/`；manifest的`acquisition_window`、`window_diagnostics` | 来源就绪及共同时间窗证据；当前窗口诊断嵌入清单，不另假定存在独立window JSON。 |
| `progress.json`、`events/lifecycle.jsonl`、进程日志、`health.json`、`diagnosis_zh.md` | 准备、采集、停止、转存、审计阶段及异常定位。 |
| `capture_preview_status.json`、`preview/<preview_epoch>/status.json` | 所选雷达/相机的WAITING/FRESH/STALE、最新接受消息年龄、真实帧发布数及单独清屏数；仅是预览证据。 |

“原始保留”指本链路实际获得的消息和帧字节：雷达为 SDK 解码后的 raw XYZ 以及SDK处理输出，保留原消息；相机为解码后的 BGR8。它不等于完整雷达 UDP 包或原始 UVC/MJPEG 包。外参和后续滤波只在派生结果中应用，不回写原始归档。

## 7. 重新核验归档和排查 PARTIAL

```bash
python3 scripts/wc_phase1 doctor \
  --session-root "$DATASET" --verify-archive
```

该命令只读取既有档案，重新核对来源/索引/bag、相机解压哈希、共同窗口和保留文件哈希；不用打开设备。查看返回 `status`、`recording_complete` 和 `issues`，不能只看 doctor 命令是否启动。原始清单已标不完整时，重新审计不能把它洗成完整。

| 现象/问题码 | 先看哪里与处理方法 |
|---|---|
| `ALL_SOURCES_NOT_READY` | `missing`/`invalid`、`ready/`、来源退出码。超声波需四地址有效响应，串口身份正确不等于探头已响应。 |
| `SOURCE_EXITED`、`RECORDER_EXITED` | 对应源/recorder日志、summary、队列和publish/ACK证据；不要用增加队列掩盖持续吞吐不足。 |
| `MAX_SOURCE_GAP_EXCEEDED`、`CAPTURE_TAIL_WINDOW_NOT_COVERED` | manifest的`window_diagnostics`、原始到达时间和源端计数；查断流、长间隔或提前停止。 |
| `STORAGE_RESERVE_REACHED`、`CAPTURE_MEMORY_RESERVE_REACHED` | `preflight`、磁盘/tmpfs/内存容量及进度阶段。保留现有结果，另取新会话名重试。 |
| `PARTIAL` / `FINALIZING` / 转存失败 | 最终manifest的`issues`、转存证据、RAM临时目录和源端summary。排空/同步/写清单失败都不能宣称完整。 |
| 文件哈希或身份不一致 | 保留现场文件和原快照，核查路径、复制及配置版本；不要重写清单使哈希“匹配”。 |

若只是预览进程退出，录制可继续；若手动驾驶窗口提前关闭，请求窗口可能不足。换会话名不会修复设备故障，但可防止覆盖前次证据。

## 8. 读取相机：旧帧与无损压缩帧兼容

当前相机归档使用 zlib level 1 无损压缩。`frames.jsonl` 记录存储块和解码后帧的SHA256、chunk相对路径、offset/length、宽高及压缩类型。不要把 `length` 当作 `width*height*3`，也不要把压缩块直接reshape。

统一接口 `read_camera_frame(directory, row)` 返回经有界解压、长度和哈希核验的 **BGR8 bytes**，兼容旧 schema1 未压缩归档和 schema2 zlib归档：

```bash
export DATASET
python3 - <<'PY'
import json
import os
from pathlib import Path
import numpy as np
from wc_runtime.source_archive import read_camera_frame

directory = Path(os.environ['DATASET']) / 'sources/cameras/left_front'
with (directory / 'frames.jsonl').open(encoding='utf-8') as stream:
    row = json.loads(next(stream))
frame = np.frombuffer(read_camera_frame(directory, row), dtype=np.uint8)
frame = frame.reshape(row['height'], row['width'], 3)
print({'shape': frame.shape, 'sequence': row.get('sequence'),
       'compression': row.get('compression', 'none')})
PY
```

此例只读第一帧；完整逐帧读取时遍历 `frames.jsonl` 并对每一行调用同一接口。原始BGR到RGB显示需交换颜色通道，不能把颜色转换或显示旋转写回归档。

## 9. 同一份数据反复离线融合与建图

先通过上面的归档审计。离线比较不打开雷达、相机、IMU、超声波或轮串口，也不重放历史控制指令；需要本工程的 Python/ROS消息依赖，RL/原生建图另需对应已构建的 `wc_estimation` / RTAB-Map。缺少设备不阻断回放，缺少软件依赖或必要外参仍会明确失败。

### 先做估计器/滤波对照

每次指定一个 **不存在的新输出目录**，并放在原始数据目录之外：

```bash
OUT="/home/nvidia/wheelchair/reports/replays/${SESSION}_compare_$(date -u +%Y%m%dT%H%M%SZ)"
python3 scripts/wc_phase1 compare \
  --dataset "$DATASET" --output "$OUT" \
  --project-root /home/nvidia/wheelchair \
  --estimators five_state robot_localization \
  --input-rate-hz 5 10 --filter off on --cloud raw
```

这会运行同一原始数据上的因子组合。5/10 Hz只改变点云消费频率，不改变硬件采集频率；所有真实轮/IMU源事件仍被消费。`--filter off/on` 控制当前软件前端滤波，`--cloud raw/filtered` 选择冻结的SDK原始/处理输出，不能关闭采集时已在设备或SDK中发生的处理，也不能复现旧SDK。

采集时冻结的几何不足时，先完成所需标定/确认配置。之后可用显式覆盖反复处理原始数据，而不改旧档案：

```bash
OUT="/home/nvidia/wheelchair/reports/replays/${SESSION}_new_geometry_$(date -u +%Y%m%dT%H%M%SZ)"
python3 scripts/wc_phase1 compare \
  --dataset "$DATASET" --output "$OUT" \
  --project-root /home/nvidia/wheelchair \
  --hardware-setup /home/nvidia/wheelchair/config/hardware_setup.json \
  --estimators five_state --input-rate-hz 5 --filter on --cloud raw
```

覆盖文件及其来源会按实际字节冻结/哈希；若其必要外参仍未知或证据验证失败，此命令仍会阻断。IMU转轴正确、采集窗口完整，都不能替代雷达数据原点及相应运动融合外参。

### 再生成原生地图

```bash
OUT="/home/nvidia/wheelchair/reports/replays/${SESSION}_maps_$(date -u +%Y%m%dT%H%M%SZ)"
python3 scripts/wc_phase1 compare \
  --dataset "$DATASET" --output "$OUT" \
  --project-root /home/nvidia/wheelchair \
  --estimators five_state robot_localization \
  --input-rate-hz 5 --filter on --cloud raw \
  --native-map --domain 89 --native-rate-hz 1
```

原生建图在独立ROS domain中使用各对照单元的共同输出样本，关闭数据库后导出。这里的1 Hz为原生地图共同子集选择，不是完整10 Hz实时建图已通过的声明。domain必须1–232且不能是生产domain83，并避免与其他离线作业冲突。

若需采用新确认外参，在该地图命令中追加同样的 `--hardware-setup`。`--native-limit 0` 和 `--source-limit 0` 默认不限制条数；非零只用于诊断前缀。普通比较不会把相机/超声波用于当前轮-IMU-雷达运动融合，其原始记录、身份和索引继续留存。

### 当前真实参数

参数由 `src/wc_runtime/mapping_compare.py:main` 核对，完整帮助可运行：

```bash
python3 scripts/wc_phase1 compare --help
```

| 参数 | 当前取值/作用 |
|---|---|
| `--dataset`、`--output` | 必填；原始会话目录和独立新输出目录。 |
| `--project-root` | 本工程软件/安装根；上述命令显式使用`/home/nvidia/wheelchair`。 |
| `--estimators` | `five_state` / `robot_localization`，可列多个；默认两者。 |
| `--input-rate-hz` | 多个 `(0,10]` Hz 值；默认5，仅控制点云消费。 |
| `--filter`、`--cloud` | 滤波 `off/on`（默认on，可多个）；点云 `raw/filtered`（默认raw）。 |
| `--hardware-setup`、`--gyro-bias` | 显式新几何/带同名证据的零偏覆盖，保留哈希；不篡改原会话。 |
| `--allow-partial` | 明确允许不完整输入诊断，不能视作完整实验。 |
| `--source-limit` | 默认0消费全部源事件；非零为诊断前缀。 |
| `--truth-json`、`--truth-min-common-samples` | 独立声明真值与所有单元共同精确时间样本的预声明最小数；默认最小1不是精度合格线。 |
| `--map-quality-policy` | 显式冻结的移动地图实验判据，不是自动获得产品标准。 |
| `--icp-shadow` | `off/point_to_plane`；影子匹配不改权威轨迹。 |
| `--native-map` | 额外运行隔离原生RTAB-Map并关闭/导出地图。 |
| `--domain`、`--native-rate-hz`、`--native-limit` | 默认89、1 Hz、0；原生频率不得超过任何前端单元频率。 |

输出至少查看 `result.json`、`summary_zh.md` 和 `artifact_manifest.json`；逐单元配置、原始事件关联、前端索引、轨迹及 `native/` 结果用于追溯。`OFFLINE_COMPARISON_COMPLETE` 表示流程完成，不能单独证明地图准确、旧新SDK等价或动态验收通过。

## 10. 历史压缩会话先按receipt恢复

当前比较直接读取原始 `.db3`，不会把历史 `.xz` 自动当作bag。历史整理仅允许经审查的单会话明确文件清单；不得广扫 `--apply`，也不要手动删除原始文件来强行腾空间。

如果某文件已经验证压缩并替换，其逐文件receipt位于：

```text
/home/nvidia/wheelchair/reports/recording_example/storage/receipts/*.json
```

查看receipt的 `path`、`archive_path` 和 `status`，选择准确的那一个。receipt文件名由原相对路径的SHA256生成；不要猜编号。将下面的占位路径替换成实际receipt：

```bash
python3 -m wc_runtime.history_archive \
  --root /home/nvidia/wheelchair \
  --restore-receipt /home/nvidia/wheelchair/reports/recording_example/storage/receipts/实际receipt文件名.json
```

此入口校验压缩文件和恢复字节SHA256/长度，fsync后恢复原路径、模式和mtime，保留压缩归档，且不会覆盖已存在原文件。恢复要求原始文件大小之外另留2 GiB；空间不足会拒绝。若receipt为 `VERIFIED_ORIGINAL_RETAINED`，原文件已在，无需恢复。

恢复一个分块不等于整个bag已恢复。开始比较前，根据各receipt检查该会话所有必要bag分块及索引都已回到原路径，并保留原有metadata、配置、CDR和数据库。历史会话没有新capture_manifest时，其完整性仍是 `LEGACY_COMPLETENESS_UNKNOWN`，不能自动升级为全来源完整记录。

## 源码依据与验收边界

本手册核对的入口为 `capture.py`（profile、预算、共同窗口、收尾）、`capture_contract.py` / `capture_audit.py`（身份与完整性）、`capture_support.py`（源码快照与分源索引）、`source_archive.py:read_camera_frame`（兼容解码）、`capture_preview.py` / `manual_capture_ui.cpp`（订阅预览和用户驾驶）、`cli.py`（doctor转发）、`mapping_compare.py`（真实参数和离线门控）、`history_archive.py`（逐receipt恢复）。

软件实现、软件测试、旧录包审计和本次现场完整采集是不同证据。单次采集只有自身最终 `COMPLETE`、`recording_complete=true`、共同窗口和重新归档审计都通过，才可称该请求范围完整；超声波物理方位、单位、真实外参、物理同步、制动安全和地图精度各自按独立证据验收。
