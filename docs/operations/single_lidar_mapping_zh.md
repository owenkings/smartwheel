# 单雷达三维建图实验操作

本文对应 2026-09-14 新增的 `map_single_lidar.sh`、输入守卫、原生 RTAB-Map 启动链和监视页。今天优先使用右雷达。本文说明代码已有的操作能力，不代表已经实机建图成功；每次结果必须按该会话的记录判定。

这是 `SINGLE_LIDAR_EXPERIMENT`：只使用选定的一台雷达，以原生六自由度 ICP 里程计驱动 RTAB-Map，保存三维几何、来源数据和原生数据库。没有 IMU 重力约束、轮速输入、左右雷达外参、双雷达融合或导航验收。初始参考来自所选雷达的初始姿态，地图没有经过验证的水平地面或车体坐标关系；画面中的高度和倾斜不能直接用作地面、障碍物或可通行区域判断。

## 运行前

- 在目标 Orin 的 `nvidia@ubuntu` 环境中运行，工程位于 `/home/nvidia/wheelchair`，已部署当前代码并具备 ROS 2 Humble、工程安装环境和原生 `rtabmap_odom`、`rtabmap_slam`、`rtabmap-export`。
- 核对 `config/live_unvalidated.json` 中所选侧的传感器身份；输入守卫会逐帧核对会话、侧别、传感器身份、米制 FLU 坐标、帧名、时间和点布局。该配置名及实验记录都不表示标定已通过。
- 先正常关闭占用雷达的配对、录制或其他采集会话。本入口检查设备占用并持有会话锁；遇到占用失败应查明拥有者，不删除锁或强杀不明进程。
- 为每次尝试使用新的会话名。建议仅用字母、数字和下划线，例如 `single_right_20260914_run01`。入口拒绝覆盖已有会话或数据库，也不提供续写旧地图。
- 用户先让设备静止，在周围保留有重叠的墙角、门框、箱子等三维结构。移动必须由用户手动或使用既有操纵方式完成，智能体不得控制电机。

## 启动与静止检查

在 Orin 终端执行：

```bash
cd /home/nvidia/wheelchair
bash scripts/map_single_lidar.sh start \
  --session single_right_20260914_run01 \
  --side right --duration 300 --port 8770
```

`--side` 可选 `right` 或 `left`，默认 `right`；`--duration` 默认 300 秒，当前代码接受 40 至 1800 秒，达到时限后监督进程关闭本次链路；`--port` 默认 8770，仅监听本机回环地址。这些是本次实验的软件参数，不是雷达性能、精度或导航承诺。当前输入转发策略为按主机单调时间限流至 5 Hz，并保留原始消息时间戳。

启动会运行所选单雷达、来源录包、输入校验与归档、原生 ICP/RTAB-Map 和监视页。驱动保留当前设备配置。返回 `STARTED_NOT_YET_VERIFIED` 仅表示监督进程启动，还没有证明来源有效、跟踪成立或地图生成。

在另一个 Orin 终端反复检查同一会话：

```bash
cd /home/nvidia/wheelchair
bash scripts/map_single_lidar.sh inspect --session single_right_20260914_run01
```

保持静止，核对下列证据后再考虑移动：

- `runtime.state` 为 `RUNNING`。
- `input.state` 为 `RUNNING`，`failure_reason` 为空，`archived_frames` 和 `published_frames` 随来源持续增加。两者分别代表落盘和发布；若发布前后发生失败，不能把已归档数直接当成已送入算法的帧数。
- `monitor.status` 为 `RUNNING`，`tracking_lost` 为 `false`；`counts` 中 `odom`、`odom_info`、`cloud_map` 均已有消息，`map.available` 为 `true`，里程计消息持续新鲜。`UNKNOWN` 或 `INITIALIZING` 表示尚未完成这一检查。
- 静止时查看点云、位姿和故障状态，确认没有明显跳变或失跟。此项观察不构成精度标定。静止时 `cloud_map` 可以保持不变，单独出现 `cloud_map_stale` 不能据此断定整条链失败。

监视页地址为启动输出的 `http://127.0.0.1:8770/`，这是 Orin 本机地址；其他电脑只有在已有、已核对的转发连接下才能使用对应地址。页面只读，可旋转、缩放、平移及选择视角。网页点云是原生地图的抽样预览；地图与里程计轨迹使用不同帧，页面不会无依据地把二者叠加成同一坐标下的轨迹。

## 用户移动与失跟处理

静止检查通过后，由用户决定并执行短距离、轻缓的移动和转弯。尽量让连续视野保留墙角、门框、箱子等重叠结构；只有一面平墙时约束可能不足，快速转弯或重叠骤减容易失跟。这里没有给出已验证的最大速度或转弯速率，也没有 IMU 运动补偿、重力调平或轮速补偿。

移动期间关注 `tracking_lost`、`consecutive_lost`、`total_lost`、位姿变化和 `failure`。出现失跟、跳变、来源中断或 `FAILED` 时，用户先用现有操纵方式停止实际移动，再检查会话。输入无效、超时、归档失败或监视器判定失败会以非零退出触发监督进程停链；连续失跟也有软件停止门限。门限用于保留明确的失败边界，不说明门限以内的地图一定正确。

失跟记录和轨迹分段会被保留。不要因后续重新出现点云、旧画面仍在或某个文件存在就宣布路线完成；不要把失败记录改成成功。排查并确认原会话退出后，使用新的会话名重试。

## 正常停止

用户先停止实际移动，再执行：

```bash
cd /home/nvidia/wheelchair
bash scripts/map_single_lidar.sh stop --session single_right_20260914_run01
bash scripts/map_single_lidar.sh inspect --session single_right_20260914_run01
```

`stop` 只关闭本会话登记的采集、算法、录包和监视进程，等待清理；它不是车辆制动或紧急停车命令。运行达到 `--duration` 后也会自动收尾，但不会控制车辆。监视页随所属会话关闭。

正常关闭要求 `runtime.state` 为 `STOPPED`、`runtime.exit_code` 为 `0` 且没有 `cleanup_errors`。仍为 `RUNNING`、出现 `FAILED` 或清理错误时，保留现场记录继续排查，不能跳过关闭检查导出或重用该目录。

## 导出实验地图

正常关闭后执行：

```bash
cd /home/nvidia/wheelchair
bash scripts/map_single_lidar.sh export --session single_right_20260914_run01
```

导出入口还会要求原生数据库存在、没有残留 WAL、SQLite 完整性检查通过且保留了地图节点。它以只读方式读取原数据库，将副本写入新的 `export/` 后调用原生 `rtabmap-export` 请求三维点云和位姿导出；成功检查包括产生 PLY、命令正常退出，以及原数据库哈希未改变。实际文件清单和哈希见 `export/result.json`。

`EXPORTED_EXPERIMENTAL_MAP` 只表示这一导出检查通过。它仍标记几何精度、导航和动态路线完成情况未验证，不能代替现场路线、重叠、漂移和闭环质量审查。仅静止采集也可能有可导出的节点。

已有 `export/` 不会被覆盖；导出失败可能留下副本和日志。保留这些证据，不删除原数据库、WAL 或失败目录来绕过检查，也不手工把失败地图标成完成。

## 数据与诊断位置

以下 `<SESSION>` 替换为实际会话名。会话结果根目录为 `/home/nvidia/wheelchair/reports/single_mapping/<SESSION>/`。

| 相对结果根目录的路径 | 内容与用途 |
| --- | --- |
| `experiment.json` | 会话、所选传感器、实验限制、实际录包话题和代码哈希。 |
| `bag/` | SQLite rosbag，包含所选雷达滤波前/后的 authoritative `SourceFrame`、诊断，以及本次输入、里程计、RTAB-Map 地图相关话题和私有 TF。实际话题白名单见 `experiment.json`；不录另一侧雷达、IMU 或编码器。 |
| `input/frames/00000001.cdr` 等 | 每个已归档转发候选的完整 `wc_interfaces/msg/SourceFrame` CDR，保留来源元数据及滤波后原坐标点云字节；每帧和索引写完后才发布。它不是滤波前原始来源的替代品，滤波前来源见 bag。 |
| `input/frames.jsonl` | CDR 文件索引、来源标识、序列、点数和哈希，供回溯输入。 |
| `input/status.json` | 输入守卫的最新状态、来源/归档/发布计数及明确失败原因。 |
| `slam/rtabmap.db` | 本次原生 RTAB-Map 数据库；保留原件，正常关闭后才能导出。 |
| `monitor/status.json`、`monitor/status.jsonl` | 最新监视状态及状态历史。 |
| `monitor/events.jsonl`、`monitor/odometry.jsonl` | 故障、失跟等事件和接收到的里程计记录。 |
| `monitor/trajectory.jsonl` | 经同时间戳跟踪状态确认的有效位姿轨迹，保留失跟后的分段。 |
| `monitor/latest_map.npz` | 最近一次原生地图点云快照和坐标帧信息，供预览/离线核对；不是原始雷达帧归档，也不等于最终验收地图。 |
| `export/` | 原生导出副本 `native_export_copy.db`、`export.log`、生成的点云/位姿文件，以及成功时的 `result.json`。 |

监督进程证据位于 `/home/nvidia/wheelchair/.phase1_runtime/sessions/<SESSION>/single_mapping/`：`plan.json` 是实际命令和环境，`manifest.json` 是进程身份、退出状态和清理结果，`supervisor.log` 与 `process-*.log` 用于诊断各组件。

停止后如需只读查看已有监视快照，可使用受管理的查看入口（最长运行三小时）：

```bash
cd /home/nvidia/wheelchair
bash scripts/map_single_lidar.sh preview --session single_right_20260914_run01 --port 8770
```

输出 `viewer_session` 是本次查看进程的会话名。下一次录制要使用相同端口前，先关闭此查看会话：

```bash
python3 -s scripts/wc_phase1 stop --session <上一步输出的viewer_session>
```

也可在已加载工程 Python 环境的 Orin 终端前台运行：

```bash
cd /home/nvidia/wheelchair
source /opt/ros/humble/setup.bash
source install/main/setup.bash
PYTHONNOUSERSITE=1 PYTHONPATH="$PWD/src:$PYTHONPATH" python3 -s \
  -m wc_runtime.single_mapping_monitor \
  --offline reports/single_mapping/single_right_20260914_run01/monitor --port 8770
```

离线页标记 `OFFLINE_SNAPSHOT`，只读已保存状态和地图，不启动雷达或建图；旧快照不能表明设备仍在运行。用 `Ctrl+C` 关闭该离线查看进程。

## 为后续双雷达处理保留依据

本次来源使用 `arrival_only` 主机接收时间；它不是经验证的测量同步。输入点云保持所选雷达的原始米制 FLU 坐标，不猜测安装外参、不缩放、不进行跨雷达坐标转换。算法产生的是本次实验内部的里程计/地图关系和私有 TF，没有据此得到 `base_link`、IMU 或另一雷达的标定关系。

后续做左右雷达标定、时间关系核对或融合时，应保留 bag、逐帧 CDR、来源索引和原生数据库，依据验证过的转换关系重新处理；仅有一份导出的合并点云不够恢复全部来源依据。今天单雷达实验的成功与否，均不能宣布双雷达配对、调平、融合或正式导航已完成。
