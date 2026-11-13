# 超声波现场核对（约 20 分钟）

四个 FD07-34R 超声波探头要接成一道刹车（总管 2026-11-12 A3 第 3 条，用户 11-12 22:05 同意）。地址和探头怎么对应、读数的单位、盲区、波束、没有回波时读到什么，都还没量过。这一页写现场怎么量、怎么算"过"、量完写到哪里。

在量完以前，[ultrasonic_live.py](../../src/wc_runtime/ultrasonic_live.py) 的状态话题一直报 `"verified": false`，C 不能拿这些距离去刹车。

## 准备

- 主检出里要有 A 的超声波提交（`wc_runtime.ultrasonic_live`）。
- 面板不在建图。雷达、轮控都不用开，车架空或落地都行。
- 工具：卷尺；约 30 × 30 cm 的平木板；直径 2–3 cm、1 m 以上的细杆；纸胶带。
- 探头位置：每个盒子正面雷达窗下方一对圆孔是"前"探头，盒子外侧面一对圆孔是"侧"探头。距离都从孔的外表面量起（探头面在盒壁里面 2 mm，可以忽略）。
- 环境：`cd ~/wheelchair && source /opt/ros/humble/setup.bash && source install/main/setup.bash`。

## 第 0 步：USB 绑定和供电（2 分钟）

```
python3 -s -B -m wc_runtime.ultrasonic_live --check-binding
```

- 超声波的 CH340 没有序列号，只能按 USB 口认。`ultrasonic.matches` 为 true 才能往下做。
- 为 false，并且 `adapters_found` 里只有一个，执行：
  ```
  python3 -s -B -m wc_runtime.ultrasonic_live --check-binding --write-local-binding
  ```
  它把现在的口写进 `config/local/project/ultrasonic_capture.json`（本机覆盖；仓库文件不改），之后录制和面板会话都按它认。
- 轮控按序列号认，换到 Orin 本机口不用改。`wheel.matches` 为 true 就行。
- 供电：2026-11-13 03:45 在 Orin 上读了 40 s，四个地址每次都超时（`RESPONSE_TIMEOUT`）；10-12 那次四个都回复。先确认四个探头有电、RS-485 线接好。

## 读数怎么看

另开一个终端运行，每一轮在屏幕上打一行。原始事务记进 `--output` 目录，Ctrl-C 结束：

```
T=$(date +%Y%m%dT%H%M%S)
python3 -s -B -m wc_runtime.ultrasonic_live --allow-read-queries --no-ros \
  --output /media/nvidia/tiany-USB/wheelchair/reports/ultrasonic_field/${T}_check \
  --run-root /tmp/wc_ultrasonic_run --session ultrasonic_check_${T}
```

- 每行的 `raw` 是四个地址的原始读数，按现在配置里的占位顺序标成 left_front、right_front、left_side、right_side。顺序在第 1 步改正。
- `cycle_s` 是一轮的秒数。
- 读数为 null 表示那次没回复。

## 核对项和合格标准

| # | 项目 | 做法 | 算过的标准 | 写进配置 |
|---|---|---|---|---|
| 1 | 地址对应（3 分钟） | 木板贴住一个探头（离孔 5–10 cm）停 3 轮，再换下一个，四个都做 | 每次恰好一个地址的读数明显变小，四个探头对应四个不同地址 | `probes[].address`；`address_mapping_status: VERIFIED` |
| 2 | 单位和精度（6 分钟） | 木板正对左前探头、和盒子正面平行，距孔 0.50 / 1.00 / 1.50 m，各读 5 轮；其余三个探头各在 1.00 m 读 5 轮 | 每处 5 轮的中位数 × `raw_unit_m` 和卷尺差不超过 ±3 cm，5 轮极差不超过 2 cm | `raw_unit_m`（读数约 500 / 1000 / 1500 就是毫米，0.001）；`raw_unit_status: VERIFIED` |
| 3 | 盲区（2 分钟） | 左前探头前，木板从 0.50 m 慢慢移到 0.05 m，每 5 cm 停 2 轮 | 找到一个距离 d：比 d 远时读数跟着变，比 d 近时读数不再变小、跳变或没回复 | `min_range_m` = d + 0.05 |
| 4 | 没有回波（2 分钟） | 探头前 3 m 内清空，或朝空旷处，读 10 轮；四个探头都做 | 10 轮读到同一个可以重复的表示（固定值，或每次都没回复） | 固定值写进 `no_echo_raw`；`no_echo_status: MEASURED` |
| 5 | 波束宽度（3 分钟） | 细杆竖直立在左前探头正前 1.00 m，横向每次移 5 cm，停 2 轮，左右各做一次 | 记下读数还在 1.00 ± 0.05 m 的最大横向偏移 a（左右各一个）；左右差不超过 10 cm | `field_of_view_rad` = 2 × atan(左右平均 a / 1.00) |
| 6 | 地面回波（1 分钟） | 车停在空地，前方 2.5 m 内没有东西，读 10 轮 | 两个前探头都没有 2 m 以内的稳定读数 | 不过：记下读数；C 要按地面回波处理（不能当障碍） |
| 7 | 串扰（2 分钟） | 只在左前探头前 1.00 m 放木板，其他探头前空着，读 10 轮；再换右前做一次 | 其他探头不出现约 1.00 m 的读数 | 不过：记下是哪个探头，把 `inter_sensor_delay_s` 加大到 0.2 再测一次 |
| 8 | 更新率（自动） | 看第 2 到第 7 项时屏幕上的 `cycle_s` | 一轮不超过 1.0 s（C 的 RANGE_MAX_AGE_S）；10-12 的记录是每个地址约 0.21 s、一轮约 0.84 s | 超过时记下；一个探头不回复，每次多等 0.4 s |
| 9 | 用卷尺核几何（2 分钟） | 量前探头两孔中点到同侧雷达窗中心：横向、竖向；量侧探头两孔中点到雷达窗所在的盒子正面的前后距离；再量左右两个盒子正面是否齐平 | 和 CAD 差不超过 5 mm：前探头在雷达窗外侧 30.75 mm、下方 57.15 mm；侧探头在盒子正面后 50 mm、离雷达窗中心横向 80.75 mm、下方 57.15 mm；两盒正面齐平 | `geometry_tape_check: DONE`；不符时把量到的数写进 `evidence`，由 A 改几何 |
| 10 | 坐人（1 分钟，记下就行） | 人坐上车，脚放踏板，读 5 轮 | 只记录：前探头的读数有没有变成膝盖或脚的距离 | 写进 `evidence`；被挡住时，前探头的刹车要另想办法 |

- 第 9 项的数来自 `config/hardware_setup.json` 的 CAD 面中心。
- 右雷达的现场外参比对称外壳的位置靠前 8.6 cm。所以发布的几何不挂在右雷达的外参上，用的是 CAD（[ultrasonic_live.py](../../src/wc_runtime/ultrasonic_live.py) 开头有说明）。两盒正面齐平的话，CAD 是对的。

## 量完以后

1. 把 `config/ultrasonic_live.json` 复制成 `config/local/project/ultrasonic_live.json`，填上量到的值和状态。
2. 在 `evidence` 里写日期和每项的读数。
3. 检查格式：
   ```
   python3 -s -B -m wc_runtime.ultrasonic_live --print-transforms
   ```
4. 要在面板会话里一起发布，把 `config/local/project/mapping_live.json` 里的 `"ultrasonic_enabled"` 设为 true（默认是 false）。会话目录会多出 `ultrasonic/events.jsonl`、`ultrasonic_capture.json`、`ultrasonic_live.json`，录包里多出 `/wc_mapping/ultrasonic/` 下的六个话题。
5. 状态话题的 `not_verified` 变成空列表、`verified` 变成 true 以后，C 才能接进刹车。

## 话题（面板会话，域 83）

| 话题 | 类型 | 内容 |
|---|---|---|
| `/wc_mapping/ultrasonic/<探头>/range` | sensor_msgs/Range | `<探头>` 是 left_front、right_front、left_side、right_side。frame 是 `ultrasonic_<探头>`；ULTRASOUND；没回波或比 max_range 远是 +inf，比 min_range 近是 -inf；时间戳是收到回复的时间；没回复的那次不发 |
| `/wc_mapping/ultrasonic/status` | std_msgs/String（JSON，latched） | 每轮一次：每个探头的最后状态、原始值、距离、年龄、计数，一轮的时间、CPU、几何、`verified` 和 `not_verified` |
| `/wc_mapping/ultrasonic/feedback_raw` | std_msgs/String（JSON） | 每次事务一条，字段和录制时的超声波事务相同 |

几何：base_link（车轴）到 `ultrasonic_<探头>` 的静态变换，标记为 CAD_NOMINAL。
- 面板会话把它发到域 83 的 `/tf_static`。建图程序和它的 RViz 用自己的 `/wc_mapping/app/tf_static`，看不到这些坐标系，也不会冲突。
- 录包不录 `/tf_static`。几何另外记在状态话题的 `geometry` 里，会话目录里也有冻结的配置。
- 单独运行时加 `--publish-tf base_link` 发同样的变换；`--publish-tf lidar` 发的是相对同侧雷达的变换。

2026-11-13 在 Orin 上量的数（MAXN，带 ROS 发布，四个探头都超时）：
- CPU：稳定时占一个核的 2.3 %（50 s 内 1.13 s CPU），内存 80 MB。
- 一轮 2.05 s：每个地址等满 0.4 s 超时，再加 0.11 s 间隔。
- 10-12 有回复时，每个地址从发请求到收齐回复约 94 ms，一轮约 0.84 s。C 的年龄门限是 1.0 s，余量只有约 0.16 s；有一个探头不回复，一轮就变成约 1.15 s。
