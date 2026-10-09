# Orin 常用命令

更新日期：2026-09-15。以下是本次版本的命令接口；实际部署和测试结果以本轮报告为准。在 Orin 的桌面终端执行，无须先切目录或手动 source ROS。

## 看实时单帧

```bash
python3 /home/nvidia/wheelchair/scripts/map right
```

默认只预览实时输入，不生成累计地图、不录包；关闭窗口或 Ctrl+C 后停止本次设备并清理临时数据。`right` 可替换为 `left` 或 `all`，每次只运行一种模式。这里的实时单帧会随新数据替换，不是保存地图回放。

## 开始建图

```bash
python3 /home/nvidia/wheelchair/scripts/map right --mapping true
```

`--mapping true` 才启用建图。关窗或 Ctrl+C 后，程序先停止设备，再在原终端询问是否保存。

- 输入 `y`：选择一个尚不存在的保存目录，按回车采用提示的 `~/maps/<会话名>`。目录可以在工程外；已有目录不会被覆盖。
- 输入 `n` 或直接回车：不保存本次地图，清理临时数据。
- 终端断开、输入 EOF 或中断选择：保留为待决定，不当成拒绝保存。

`--output` 指定的是工程内临时工作会话目录，最终保存路径在停止后询问。连续建图没有总时长或 12,000 帧上限；`--duration` 兼容保留但不再计时结束。磁盘、真实设备错误、内存队列与保存完整性限制仍有效。

## 重新处理待决定的地图

终端中断保存选择、没有交互终端，或保存目标出错后，可以用终端结果中的 `session_output` 工作目录重新处理：

```bash
python3 /home/nvidia/wheelchair/scripts/save_map /home/nvidia/wheelchair/reports/maps/实际会话名
```

不传目的地会再次询问 `[y/N]`，选 `y` 后输入新的保存目录；选 `n` 或回车则清理本次临时地图和原始录制，保留轻量诊断。也可直接指定新目的地：

```bash
python3 /home/nvidia/wheelchair/scripts/save_map /home/nvidia/wheelchair/reports/maps/实际会话名 ~/maps/room_a
```

此命令不重新启动设备。工作目录必须是工程内、带本次临时保留标记且已正常停止的建图会话，不能用来删除历史地图。目的地可在工程外但必须尚不存在；保存 DB、三维/二维成品和轻量元数据，不长期保留原始 bag/CDR。保存失败保留工作数据供重试；EOF、非交互终端或中断选择仍保持待决定，不默认丢弃。

## 查看已经保存的地图

把下面 `room_a` 换成实际保存目录：

```bash
python3 /home/nvidia/wheelchair/scripts/view_map ~/maps/room_a
```

也可直接指定导出目录或数据库：

```bash
python3 /home/nvidia/wheelchair/scripts/view_map ~/maps/room_a/export
python3 /home/nvidia/wheelchair/scripts/view_map ~/maps/room_a/slam/rtabmap.db
```

含空格的路径用双引号包住。查看器打开离线 RViz，显示保存的三维点云和二维地图；不会启动雷达、IMU、相机或电机，也不会回放控制话题。初始视角按地图范围自动居中、缩放，可以继续旋转、缩放，或在 `Views` 选择 `2D top`。关窗或 Ctrl+C 结束，不设查看倒计时。

目录查看直接使用其中一个 PLY 与一个地图 YAML/PGM，不要求保留原始 bag。直接指定 DB 或目录仅剩 DB 时，先只读检查原库，再复制到本次临时目录，用现有原生导出程序生成查看文件。原生导出最多等待 180 秒，临时空间须至少有数据库大小加 512 MiB；退出清理本次临时文件，原库不改写。损坏或仍有 WAL/journal 的数据库会被拒绝。

查看既有部分地图不会把原失败会话改成成功。地图可打开只说明文件可解析，不是完整路线、精度或导航验收。

## 只读取文件与数据库摘要

```bash
python3 /home/nvidia/wheelchair/scripts/view_map ~/maps/room_a --check
python3 /home/nvidia/wheelchair/scripts/view_map ~/maps/room_a/slam/rtabmap.db --check
```

`--check` 只读输出 JSON，不启动 ROS/窗口，也不从 DB 创建导出副本。数据库摘要包含 SQLite 完整性检查后的 Node 数量、首末 ID 和 SHA-256；已有地图文件还显示点数、XYZ 范围、二维尺寸和分辨率。独立 DB 会显示 `requires_temporary_export=true`，表示打开图形窗口时才需要临时导出。

查看最多显示 1,000,000 个实际保存点的确定性抽样，不改原点云。可用 `--max-view-points 2000000` 调到 2,000,000；这只是显示容量。默认会话内软件渲染，如需系统渲染加 `--renderer system`。同一离线 domain 84 一次只开一份查看，已有窗口未关闭时会明确提示占用。

## 对照主机滤波前后的点云

默认使用 `filtered`。`--cloud` 同时选择当前帧预览和实际建图输入，并写入会话配置；它不写雷达的设备参数。

```bash
# 两种实时预览；一次只运行一条，关闭窗口后再换另一种
python3 /home/nvidia/wheelchair/scripts/map right --cloud raw
python3 /home/nvidia/wheelchair/scripts/map right --cloud filtered

# 选择滤波前输入建图；停止后仍询问是否保存和保存位置
python3 /home/nvidia/wheelchair/scripts/map right --mapping true --cloud raw

# 无设备检查：确认最终输入选择
python3 /home/nvidia/wheelchair/scripts/map right --mapping true --cloud raw --check-config
```

`raw` 指 SDK 解码、投影得到的主机滤波前 XYZ，并非原始光学深度或 HDR 子曝光数据。这个选项选择下游使用哪一路，驱动仍产生滤波前后两路；建图期间也同时临时录制两路，便于按同帧身份核对。昨晚录包对照显示滤波会移动边缘点，但部分墙候选会更薄；因此 `raw` 只是明确的对照选项，不代表精度更高。

连续建图现在在后台收集陀螺零偏候选，结果在工作会话的 `prior/status.json` 的 `gyro_bias` 中；`latest_candidate` 是待验证估计，`accepted_version` 才是已接受偏置。当前没有自动静止确认来源，默认不应用这些候选、不等待静止，也不会因此停止会话。`prior/guesses.jsonl` 每帧记录实际使用的 `gyro_bias_applied_native_rad_s`。时间偏移、外参和默认里程计不因这次诊断自动改写。

## 帮助与配置检查

```bash
python3 /home/nvidia/wheelchair/scripts/map --help
python3 /home/nvidia/wheelchair/scripts/view_map --help
python3 /home/nvidia/wheelchair/scripts/save_map --help
python3 /home/nvidia/wheelchair/scripts/map right --check-config
python3 /home/nvidia/wheelchair/scripts/map right --mapping true --check-config
```

配置检查不打开设备或窗口，不能代替实际连接、地图显示或驾驶验证。

独立旧雷达查看器仍可使用，但它保留自己的倒计时，且参数名与统一入口不同：

```bash
bash /home/nvidia/wheelchair/scripts/view_lidars.sh single_right 60
```

该命令只看右雷达，60 秒后结束；左侧用 `single_left`，双路独立视图用 `dual`。日常预览优先用上面的 `scripts/map right`。
