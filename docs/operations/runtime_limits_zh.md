# 当前运行限制、超时与质量门

核对日期：2026-09-14。本文依据当前源码和配置，重点说明 `scripts/map left|right|all`；独立诊断和严格工具另列。源码位置相对工程根目录 `/home/nvidia/wheelchair`。这是源码行为说明，不是本轮部署、真实断流恢复或动态地图精度验收。

## 当前主入口已取消的限制

`mapping_controller.configuration()` 对主入口强制写入 `continuous_mapping=true`，配置缺省值或显式 false 都不会把 `scripts/map` 改回严格模式。独立调用组件时缺省仍为 false。

| 原限制 | 当前 `scripts/map` 行为 | 代码接线 |
| --- | --- | --- |
| 总运行时长 | 没有运行倒计时。`--duration` 仍接受非负整数以兼容旧命令，但所有值均忽略并记录 `requested_duration_s`，实际 `duration_s=0`；`--duration 60` 也不会在 60 秒停下 | `mapping_app.prepare_request()`；`mapping_controller.plan()`、`start()` |
| 12,000 帧/输出上限 | 主输入、每源归档及最终审计不再设 12,000 总量门；原约 40 分钟限制已取消 | `single_mapping_input.SourceGate`、`FrameArchive`；`mapping_input.py`；`mapping_controller.input_archive_summary()` |
| 启动静止窗口及重力质量门 | 使用固定安装参考，不要求双轮零速、连续 2 秒静止或重力窗口；输入配置立即生成。先验等到真实 IMU 和轮反馈各有样本后初始化，无 15/30 秒静止启动截止 | `mapping_input.GravityBootstrap`；`mapping_prior.MotionPrior._initialize_from_mount()`；主计划 `--wait-config-seconds 0` |
| 传感器/里程计静默期限 | 没有首份数据或数据沉默导致整次建图失败的计时门；XT 真正连接等待 20 秒仍保留。雷达、IMU、输入和监视器等待恢复；主计划给 XT `source_stale_seconds:=0`、H30 `--stale-timeout 0` | `mapping_controller.plan()`；`wc_xt_driver`；`wc_imu.ros_node`；`mapping_input`；`mapping_monitor.Health` |
| IMU/轮反馈历史缺段 | 正向时间/序号缺口记录覆盖缺失；保留位姿并从真实新数据重新接续，不用末次速度无限积分缺失时段。无法覆盖的旧云可以丢弃并记数 | `history_preview.HistoryPreview(continuous=True)`；`mapping_prior.MotionPrior`、`next_prepared_cloud()` |

固定参考采用配置的轮轴轴向及所选雷达原点，**没有本次静止重力实测，也没有本次陀螺零偏估计**。先验明确记录 `gravity_estimated=false`、`gyro_bias_status=NOT_ESTIMATED_ZERO_ASSUMPTION`。停车地面倾斜不会自动重新找平；缺段期间真实发生的运动也不会被补回。等待恢复只适用于数据沉默/覆盖缺失，不吞掉设备断开、串口应答失败、CRC/身份错误或磁盘错误。

当前默认 `wheel_imu` 直接采用轮反馈和 IMU 位姿，不使用 ICP 失跟门，并关闭扫描修正、邻近闭环和回环约束。显式 `icp` 分支仍有连续五次失跟停止。持续运行仍受磁盘、内存队列、真实 I/O、格式与保存检查约束。

## WASD 与手推：本次修改后的控制边界

本次取消界面“启用 WASD”按钮、控制区域必须聚焦、连接 10 秒后永久放弃、连续静止轮反馈才能激活等交互限制。有效按键持续控制没有总时长上限；会话空闲后再按键、连接恢复后重新按键，都不需要重新点击激活。

| 条件或参数 | 当前行为 | 配置 / 实现 |
| --- | --- | --- |
| 默认手动许可 | 当前 `manual_controls.arm_allowed=true`；这是配置许可，不是每次操作的界面激活步骤。设为 false 仍会禁用发送手动控制 | `config/hardware_setup.json` → `manual_controls`；`mapping_wheel.validate_manual_controls()` |
| 首次 WASD 前 | 保持反馈读取；不因建图命令启动而初始化电机或抱闸。首次非零方向键才请求旧驱动模式/控制字初始化 | `src/wc_runtime/mapping_wheel.py` |
| 初始化等待 | 旧控制流程保留两段 0.2 秒间隔；初始化与串口应答所需时间意味着首键不是零延迟 | `mapping_wheel.py` → 初始化序列 |
| 手推切换 | 松键/停车先发零速，空闲 0.75 秒后使用旧实现的 `0x200E=7` 释放分支；释放后的下次方向键重新初始化。首次用 WASD 前可直接手推 | `mapping_wheel.py` → 空闲释放流程；这里的寄存器 ACK 不等同于独立测得的机械制动状态 |
| 操作范围 | RViz 本应用内接收 WASD，Space 清键停车；不再限定特定面板区域。键盘仍需由操作系统/远程桌面投递给此应用，不是全系统键盘监听；切出应用、隐藏面板或关闭窗口仍清键停车，返回后新键可继续 | `src/wc_bringup/src/mapping_teleop.cpp`、`mapping_rviz.cpp` |
| 按键意图新鲜度 | 250 ms 未得到有效更新时清除旧方向、停车；后续新键可恢复使用，不永久撤销许可 | `mapping_wheel.py` → `KEY_TIMEOUT_S=0.25` |
| UI 状态新鲜度 | 500 ms 状态失鲜时清键；本地控制连接恢复后可重新按键，无 10 秒放弃限制 | `mapping_teleop.cpp` |
| UI 连接重试 | 一次连接或首次状态等待超过 2 秒就放弃本次连接；每 500 ms 继续重试。后端接受连接后 hello 握手限 1 秒 | `mapping_teleop.cpp` → `tick()`；`mapping_wheel.ManualSocket.pump()`；只清键/断开当前客户端，反馈继续，不是整次建图 2 秒或 1 秒时限 |
| 软件指令速度 | 线速度 0.10 m/s，角速度 0.20 rad/s；配置允许区间分别 0.001–0.15 m/s、0.001–0.30 rad/s。轮指令另有 30 rpm 与有符号 16 位寄存器范围检查 | `hardware_setup.json` → `manual_controls`；`mapping_wheel.velocity_request()` |
| 反馈/控制频率 | 轮反馈目标 10 Hz，控制发送间隔 0.05 秒；发送控制前轮反馈须在 0.25 秒内 | `mapping_wheel.py` → `FEEDBACK_PERIOD_S`、`CONTROL_PERIOD_S`、`FEEDBACK_FRESH_S` |
| 串口错误 | 建图统一轮适配器实际使用 `min(feedback_timeout_s, 0.08)`，当前 **80 ms**；单独的反馈工具读取配置时是 0.2 秒。CRC/回包不符、断线、超时等真错误进入 `FAULT`，不能靠再次按键消除真实硬件通信故障 | `config/wheel_feedback_current.json` → `timeout_s`；`mapping_wheel.MappingWheel.__init__()` → `self.timeout`、通道校验 |
| 控制连接边界 | 本机 Unix socket、同 UID、单客户端、会话身份与递增按键序号校验；输入缓冲 8,192 B、单行 4,096 B、每轮最多 8 行、待发送状态 16,384 B。协议错误/溢出拒绝客户端并清键，反馈继续 | `mapping_wheel.py` → `ManualSocket`；不是开放网络驾驶接口 |
| 控制退出收尾 | 已使用 WASD 后退出，最多约 0.9 秒等待已请求的零速/释放过程；无法完成记录 `shutdown servo release did not complete` 并进入失败 | `mapping_wheel.MappingWheel.close()`；不是会话运行时长 |

释放使用的是旧程序中已有但旧 YAML 设为 `-1` 而未启用的分支；本次按“WASD/手推交替”的需求启用 0.75 秒延迟。固件身份、硬件看门狗时长、USB/RS485 物理断线后的实际停车仍未独立验证；软件心跳也不能证明远程桌面的真实操作者仍在线。以上是现有证据范围，不是新增软件激活条件。

## `scripts/map` 的启动阻止条件

| 检查 | 当前要求 / 数值 | 触发结果 | 位置 |
| --- | --- | --- | --- |
| 执行主机和目录 | `aarch64`、用户 `nvidia`、主机名 `ubuntu`；工程根 `/home/nvidia/wheelchair`；要求在该工程运行 | 启动前报错 | `src/wc_runtime/cli.py` → `target()`；`mapping_controller.preflight()` |
| 图形桌面 | 当前 UID 可访问的 GNOME / X 显示会话；多桌面时须能确定目标；缺少窗口、原生组件或安装文件也拒绝启动 | 启动前报错，不具备无头建图入口 | `sensor_viewer.desktop_environment()`、`confirm_display()`；`mapping_controller.preflight()` |
| 输出与会话 | 名称为简单 ASCII 标识；输出目录和运行会话名称不能已存在；路径必须在工程内且无 `..`、符号链接等重定向 | 启动前报错，防止覆盖已有数据 | `mapping_app.prepare_request()`；`mapping_controller.preflight()`；`prepare_picker_input.project_path()` |
| 硬件身份和独占 | 雷达、H30、轮反馈身份匹配；串口别名/by-id/USB 身份一致；不得已被其他进程占用；持有共享设备/会话锁 | 无法通过就不启动或立即失败 | `mapping_controller.preflight()`、`cli.device_preflight()`、`imu_preflight()`、`feedback_transport.SerialLease`、`supervisor.py` |
| 配置分类 | `schema_version=1`、`status=EXPERIMENT`、`source_mode=real`；`formal_acceptance`、`navigation_validated`、`common_measurement_time_validated` 必须为 false | 配置不符报错；不要求伪造正式验收 | `mapping_controller.configuration()`；`mapping_input.validate_config()` |
| 安装模型 | 安装和轮参数只从 `hardware_setup_config` 读；旋转须为合法 3×3 正交矩阵；不能同时放旧重复安装参数；硬件/相机配置各有 128 KiB 文件界限 | 启动前报错；参数使用会话快照，运行中改文件不热生效 | `mapping_controller.configuration()`；`hardware_setup.resolve_hardware_setup()` |
| 输入频率/配对配置 | 输入频率必须 `0 < input_rate_hz ≤ 5`，当前 5 Hz；双源时间差设定必须在 0–50 ms，当前 50 ms | 配置越界拒绝启动 | `config/mapping_live.json`；`mapping_controller.configuration()` |
| 里程计选择 | 仅 `wheel_imu` 或 `icp`；不会因一种失效自动切换另一种 | 无效值拒绝；运行故障按当前路径处理 | `mapping_controller.configuration()`；`mapping_app.launch.py` |
| 启动受管进程 | 等待运行清单最多 10 秒 | 没有确认启动则收尾并报错 | `mapping_controller.start()` |

持续模式仅提示当前磁盘余量，不按预计帧数或指定 duration 限制会话。空间读取失败仍会报错。

## 运行中保留的输入、覆盖与质量检查

表内“失败”表示核心组件记录错误并退出；主监督器将收尾整个会话。主程序只对确认正常关闭且没有错误的会话执行导出。“等待/丢弃”会保留诊断，不能当作对应区间已经获得完整数据。

| 检查 | 主入口当前数值 / 条件 | 触发结果 | 位置 |
| --- | --- | --- | --- |
| 来源和消息真实性 | session/sensor/side/frame/单位/轴一致；时间诚实标为 `arrival_only`；设备 epoch、配置 hash 不变；原始包、CRC、嵌套时间戳匹配；无未来时间、重复/倒退序号或倒退时间 | 不符失败；正向缺口单独记数，不能用重置身份掩盖断流 | `single_mapping_input.SourceGate`；`mapping_input.GravityBootstrap`；`mapping_prior.py`；`history_preview.py` |
| 时钟关系 | ROS 到达时钟与 monotonic 的相对间隔差以 0.05 秒判断覆盖缺口 | 连续模式正向偏差超限记 gap，不结束会话；倒退等非法时序仍失败 | `mapping_prior._clock_interval()` |
| 运动值上界 | 各轮候选速度 ≤1.0 m/s；IMU 陀螺范数 ≤3 rad/s、加速度范数 ≤30 m/s² | 运行失败；不是启动静止判据 | `config/hardware_setup.json`；`mapping_prior.DEFAULT_POLICY`；`history_preview.py` |
| 缺段判定 | IMU 0.25 秒、轮反馈 0.5 秒保留为历史覆盖判定尺度；轮预览正向跳号或超过 0.5 秒的间隔也记录 gap | 不因年龄结束会话；缺失区间保持位姿、不积分末速度，记录 `motion_coverage_complete`、gap/未覆盖时长 | `mapping_prior.MotionPrior`；`history_preview.HistoryPreview` |
| 历史和等待云容量 | 历史窗口 2 秒，IMU 16,000、轮样本 4,096，云队列 8；连续模式按容量/历史范围退旧数据 | 旧历史被退役，云队列满时丢最旧云并计数；不再用 0.75 秒等待期限结束建图。历史覆盖不可能补齐的云丢弃 | `mapping_prior._retire_history()`、`enqueue_cloud()`、`next_prepared_cloud()` |
| 双源配对 | 来源到达时间差 ≤50 ms；每侧未配对缓冲最多 32 帧 | 没配上或缓冲退旧时丢弃并记数；持续缺少一侧时等待，不用旧帧冒充新配对 | `mapping_input._members()`；`config/mapping_live.json` |
| 发布频率 | 当前输入目标 5 Hz，配置允许 `0 < input_rate_hz ≤5` | 节流，不是累计总量上限，也不保证实际有 5 Hz 新数据 | `mapping_controller.configuration()`；`single_mapping_input.SourceGate` |
| 云/地图体积 | 单源云 1–20,000 点、data ≤4,000,000 B、CDR ≤8,000,000 B；先验云 ≤100,000 点、≤16,000,000 B；健康地图云 ≤2,000,000 点、二维网格 ≤10,000,000 格 | 越界失败；网格还检查分辨率、原点旋转和 [-1,100] 占据值 | `single_mapping_input.py`；`mapping_prior.DEFAULT_POLICY`；`mapping_monitor.py` |
| 真正设备连接与事务 | XT 默认连接等待 20 秒，SDK 明确断开/重连失败仍报错；H30 EOF/读取错误仍报错；轮反馈和控制应答仍须在实际 80 ms 事务期限内符合协议 | 核心组件失败。单纯没有新帧和明确设备 I/O 失败是两种结果；不自动重开整条建图链 | `wc_xt_driver`；`wc_imu/ros_node.py`；`mapping_wheel.py` |
| 输出健康监视 | 持续记录消息计数、年龄与 `waiting_outputs`；主入口取消 30 秒首次输出和 3 秒里程计沉默失败 | 等待恢复；空白或停止更新的画面不代表新数据正在到达。最终导出仍要求必要输出存在 | `mapping_monitor.Health.tick()`；`mapping_controller.export()` |
| ICP 特有门 | 显式 `icp`：连续 5 次 `lost`；对应比例 0.15、对应距离 0.25 m、最多 30 次迭代、3 cm voxel、点到面近邻 20、TF 等待 0.2 秒 | 注册参数决定匹配接受；连续失跟仍失败。默认 `wheel_imu` 不使用这些 ICP 门 | `mapping_monitor.receive_info()`；`mapping_app.launch.py` |

数据恢复继续接收的前提是来源身份和时间序列仍合法。设备重启后改变 epoch、收到坏包或消息时间倒退仍是明确错误；此次没有实现跨设备重启重新标定/接管。

## 存储、运行结束与保存门

| 检查 | 当前默认值 / 条件 | 触发结果 | 位置 |
| --- | --- | --- | --- |
| 主磁盘保护 | 每约 1 秒检查，剩余 ≤2 GiB | 正常提前停止并尝试保存；空间检查出错则失败。余量不是保存保证 | `mapping_app.run_session()`；`mapping_controller.STORAGE_STOP_FREE_BYTES` |
| 归档底层空间 | 初始化 ≥1,000,000,000 B；逐帧写入还须保留本帧 CDR 大小 +65,536 B | 不够即归档失败；2 GiB 主保护通常先触发，但不构成原子保证 | `single_mapping_input.FrameArchive` |
| 异步归档容量 | 当前 `queued_realtime` 最多 100 个待完成输出、128 MiB 预留内存；另一种 `written_before_publish` 最多 10 个 | `ARCHIVE_QUEUE_FULL` 失败，不丢弃已承诺归档的输出；这些是同时积压上限，不是累计帧数 | `mapping_input.py` |
| checkpoint | 当前 `on_close`，停止后集中同步；显式 `periodic` 才有 1 秒计划周期和 256 个未确认输出上限 | 当前不启用 256 门；可选 periodic 积压仍会失败。无总帧数门不代表无限内存或磁盘 | `config/mapping_live.json`；`mapping_input.py` |
| 其他日志队列 | 轮日志 1,024 条 /4 MiB，单条 65,536 B；先验日志 128 条；ICP 健康跟踪日志 4,096 条 | 轮/健康日志超预算失败；连续先验日志满时暂停取云形成回压，云容量按上述规则退旧；真实写入/关闭失败仍失败 | `feedback_transport.AsyncJournal`；`mapping_prior.AsyncPriorJournal`、`main()`；`mapping_monitor.AsyncHealthArchive` |
| bag 分包 | 单 bag SQLite 512 MiB | 自动分新文件，不是总录包上限 | `mapping_controller.plan()` |
| 用户/界面结束 | 关 RViz、Ctrl+C、正常停止信号 | 停止采集、排空并尝试保存；未完成清理则失败 | `mapping_app.run_session()` |
| 核心组件自行退出 | `allow_component_exit=false`，包括未经请求提前返回 0 | 整组收尾并记失败；关闭新鲜度门不允许核心进程无故消失 | `mapping_controller.plan()`；`supervisor.py` |
| 前台所有者消失 | 每约 0.5 秒检查 PID/start ticks；主入口 duration=0，无 `duration+60` watcher 终点 | 停止该会话，不自动重新启动 | `mapping_controller.watch()` |
| RViz 启动重试 | 仅 software 模式、启动 10 秒内、exit 245、指定 GL drawable 签名且无就绪证据时重开一次 | 不满足或第二次失败则失败；该策略不改变传感器链路 | `mapping_controller.rviz_startup_retry()` |
| 停止持久化预算 | 主入口给输入/先验/轮/健康/相机状态写入 90 秒 close budget | 超时不能声明持久化成功；不是运行倒计时或电机停车响应时间 | `mapping_shutdown.py`；`mapping_controller.plan()` |
| 进程收尾预算 | 原生 SIGTERM 等待 105 秒、SIGKILL 5 秒；组件 SIGINT grace 120 秒，然后 TERM 3 秒/KILL 2 秒；监督 CLI 总预算 166 秒 | 逾期升级终止并记录失败；不是常规运行时间限制 | `mapping_shutdown.py`；`component.py`；`supervisor.shutdown_policy()`；`mapping_app.launch.py` |
| 导出空间与期限 | 剩余空间 ≥原库大小 +512 MiB；`rtabmap-export` 最多 180 秒 | 不够或超时拒绝导出，原库保留 | `mapping_controller.export()` |
| 归档审计 | input 正常 STOPPED；final checkpoint；接收/归档/发布/丢弃计数、索引和 CDR 文件尺寸一致 | 缺证据拒绝导出；索引大小按实际记录数约束，无旧 12,000 帧界限 | `mapping_controller.input_archive_summary()` |
| 保存完整性 | 必需输出存在且 health 无失败；原生正常关闭；无未闭合 WAL/journal；SQLite integrity_check=ok、Node>0；原库 hash 不变；PLY 非空、二维 YAML/图像有效；bag 关闭且必需话题非空/数据库完整 | 任一失败不标 `SAVED`。这些是完整性/非空检查，不是几何精度或覆盖率验收 | `mapping_controller.export()` |

RTAB-Map 当前 `DbSqlite3/InMemory=true`，原始归档采用 `on_close` checkpoint。异常断电/杀进程可能丢失未持久化数据和 RAM 中的地图数据库。正常关窗也必须等最终状态；只看到窗口消失不等于已经保存。

## 相机和显示的特殊情况

相机默认开启，但仅预览，不参与当前里程计/建图，也不录制图像。当前配置是每路 320×240、采集目标 30 fps、发布 8 Hz；这些是配置目标，不能由此推断现场实测帧率。

| 条件 | 当前值 / 行为 | 对主建图影响 | 位置 |
| --- | --- | --- | --- |
| 相机打开/读取/图像旧化 | 打开 15 秒、读取 3 秒、旧帧显示 1 秒 | 单路缺失/失效通常标记降级，健康槽继续，不因没有相机图像否定核心地图 | `config/cameras.json`；`wc_cameras/node.py`；`mapping_cameras.py` |
| 相机重启 | 仅已识别读取截止错误且确认正常释放后；每槽最多 2 次，间隔 1 秒 | 不无限重开、不换配置兜底；其他失败保持降级 | `mapping_cameras.MAX_RESTARTS_PER_SLOT`、`RESTART_DELAY_S` |
| 相机进程清理 | companion 共享 7.5+0.75+0.75 秒；内部 capture 自己还有 5+1+1 秒清理 | 无法正常清理/升级终止会使 companion 返回非零，从而影响整组正常关闭；单纯预览状态日志持久化失败另记，不因此拒绝核心地图 | `mapping_cameras.STOP_PHASES`、`close()`；`wc_cameras/capture.py` |
| 栅格分类 | 3 cm 格、地面角 20°、法线近邻 20、聚类半径 0.1 m、最小簇 10；关闭高度截断、最大距离截断和 ray tracing | 影响地面/障碍物显示和成品内容，不是超时自动停机门；这些参数不是测量精度 | `mapping_app.launch.py` → `Grid/*` |

## 其他独立入口：不要套用到 `scripts/map`

| 入口 / 子系统 | 默认与限制 | 触发行为 / 区别 | 位置 |
| --- | --- | --- | --- |
| 直接调用输入/先验/轮预览组件（未给 continuous） | `continuous_mapping` / `HistoryPreview(continuous=...)` 缺省 false：输入首帧/重力配置 15 秒、静止窗口 2 秒、3 秒断流、每源/输出 12,000 帧；先验启动 30 秒、静止 2 秒、IMU 0.25 秒/轮 0.5 秒年龄、云等待 0.75 秒、队列 8 | 严格默认门保留，不能拿这些常量断言 `scripts/map` 仍在执行；严格先验还有重力样本/离散度质量门 | `single_mapping_input.py`；`mapping_input.py`；`mapping_prior.DEFAULT_POLICY`；`history_preview.py` |
| 直接调用驱动/监视器 | H30 `--stale-timeout` 默认 2 秒；XT `source_stale_seconds` 默认 3 秒；Health 默认首输出 30 秒、里程计 3 秒 | 主入口分别传 0 / continuous=true；独立缺省仍会按时失败 | `wc_imu/ros_node.py`；`wc_xt_driver`；`mapping_monitor.py` |
| `wc_runtime.cli map` 正式融合入口 | 实机配置必须有已验证外参、时间、质量 profile；`quality_profile` 还需 evidence | 目前实验标定不能冒充 VALIDATED；会在启动前阻止。这不是 `scripts/map` 实验入口的前置门 | `src/wc_runtime/cli.py` 的 `map` 分支；`src/wc_fusion/core.py` |
| 正式双源融合质量 | 配对 20 ms、队列每源 8、年龄 300 ms、每源有效点至少 20、范围 0.1–10 m；速度/角速度预算为 0.2 m/s、0.2 rad/s，时间误差预算 0.02 m、预测误差 0.003 m、预测最长 150 ms，运动硬界限为 0.5 m/s、0.5 rad/s | 范围外点先过滤；每源不足 20 个有效点、无效时间/身份等锁存暂停；配对窗不符/队列溢出先丢旧帧，持续无合格源或配对超时也锁存；恢复需 explicit/stable/continuity_verified 且标定与时钟身份相同 | `config/quality_experimental.json`；`wc_fusion.core.Policy`、`DualFusion` |
| 正式里程计结果接收 | 待处理结果 8、结果时限 500 ms；速度 0.5 m/s、角速度 0.5 rad/s | 超时/坏位姿等锁存状态，不应误认为默认 wheel_imu 路径的 ICP 质量门 | `wc_fusion.core.Gate` |
| `map_single_lidar.sh` / `wc_runtime.single_mapping start` | 默认 300 秒，允许 40–1,800 秒；其查看会话 10,800 秒；导出 180 秒 | 独立单雷达诊断/建图入口的时限；`map left/right` 不继承 1,800 秒上限 | `src/wc_runtime/single_mapping.py` |
| `cli drivers` / `record` | 默认 30 秒，允许 1–3,600 秒；监督会话额外留 3 秒 | 到时结束；`scripts/map` 直接构建自己的 duration 0 子进程计划 | `src/wc_runtime/cli.py` |
| `view_lidars.sh` | 默认 900 秒，允许 1–3,600 秒 | 独立雷达查看器到时结束 | `src/wc_runtime/sensor_viewer.py` |
| `cli cameras` | 默认 3,600 秒，允许 1–43,200 秒；独立 camera node 默认 60 秒并允许显式 0 持续 | 不代表建图相机会在 1 小时自动停止，建图已传 0 | `cli.py`；`wc_cameras/node.py`；`mapping_controller.plan()` |
| `cli encoder` | 默认 30 秒，允许 1–300 秒 | 不代表建图轮反馈会在 300 秒结束 | `src/wc_runtime/cli.py` |
| 独立 `manual_teleop` / `manual_hardware` | 默认 60 秒，允许 0.1–300 秒；`wheel_manual_unvalidated.json` 的 dry-run；实机路径另需绑定的控制/停车/硬件 watchdog 证据、现场确认、用户激活 | 这些独立手动工具保留其严格证据/激活门；建图使用 `mapping_wheel`，不继承 300 秒上限或其 E 激活流程 | `src/wc_motion/manual_teleop.py`、`manual_hardware.py`；`config/wheel_manual_hardware.json` |

## 当前未完成的测量与验收

- 安装、轮径、轮距和轮速比例仍为实验候选；轮/IMU 协方差是模型权重。固定参考免静止不等于完成重力找平、航向或零偏标定。
- 到达时间不是已验证共同测量时钟；没有已验证逐点时间，原生 `deskewing=false`。
- 手推依赖驱动轮滚动反馈；抬轮、滑移、轮子未反映真实车体运动和数据缺口中的运动不会自动修正。
- 默认关闭扫描修正和回环；没有地图真值 ATE、完整覆盖、长期漂移或导航可通行性的自动验收。`SAVED` 只表示本次保存/导出完整性检查完成。
- 连续模式的软件行为不证明已完成真实断流恢复、长期持续运行或真车 WASD/释放/物理断线停车验收。本轮进展见[连续建图更改与退出诊断](../../reports/continuous_mapping_20260914/README_zh.md)。

排查提前结束时，先看终端最后结果 JSON、`.phase1_runtime/sessions/<session>/mapping_app/manifest.json` 和地图会话内 `input/status.json`、`prior/status.json`、`health/status.json`、`wheel_status.json`。区分低空间正常收尾、等待新源，以及明确组件/协议/写盘失败。最终以主程序 `status`、`errors` 和 `export/result.json` 为准；主程序不自动写同名顶层结果文件。
