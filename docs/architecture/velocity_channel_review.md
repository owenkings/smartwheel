# 速度通道第 0 级：给用户审的代码说明
审的是分支 `feature/velocity-channel-20261109`（worktree `~/wheelchair-wt/motion`）上的三个提交：b46193c7（第 0 级）、9f8b772e（按 D 的审查改接口）和其后一个提交（按总管 2026-11-10 的代码审查修改，并按用户 11-09 23:50 的决定加"开关未安装"模式）。总管的审查全文在 `~/wheelchair_coord/reviews/level0_review_20261110.md`，设计见[速度通道设计](velocity_channel.md)。全部只用假串口测过：没开过真串口，没连驱动器，没发过速度。

## 改了哪些文件
| 文件 | 内容 |
|---|---|
| [mapping_wheel.py](../../src/wc_runtime/mapping_wheel.py) | 唯一改动的旧文件，只加挂接（+27/−3 行），见下一节 |
| [mapping_auto.py](../../src/wc_runtime/mapping_auto.py)（新） | 轮控所有者里的自动控制源 `AutoDrive`、本机 socket `AutoSocket`、入口 `attach_auto` |
| [auto_protocol.py](../../src/wc_motion/auto_protocol.py)（新） | 闸门决定 `wc_gate_command_v1`、桥和轮控所有者之间的消息，只做校验 |
| [auto_bridge.py](../../src/wc_motion/auto_bridge.py)（新） | 底盘桥：域 100 里收闸门决定，经 `auto.sock` 交给轮控所有者；发 `base_status`、使能、急停 |
| [auto_evidence.py](../../src/wc_motion/auto_evidence.py)（新）和[证据模板](../../config/wheel_auto_drive_evidence.template.json) | 接通前的证据文件格式；按档提速；模板总被拒绝 |
| 4 个测试文件（新） | 见"假串口测了什么" |
| README、本设计文档 | 文件列表和说明 |

## 11-10 为第一次空车闭环加的（总管 14:40 的顺序）
- 9e5d79dc：没装开关时桥照样发 `/wc_nav/estop`。3f575fd9：只读回读小工具。3631f11f：接口约定按你同意的那句改。
- 977c69ec：给 D 的闭环干跑，真的轮控所有者配模拟驱动器。52065582：0.10 m/s 测试指令源和停车报告。
- 其后一个提交：采集从本机硬件配置抄 `auto_controls`；轮控所有者在会话开始读一次证据列的驱动器参数；证据按减过的现场计划改（3 次、只做 0.10 m/s、拔 USB 驱动器不停时记下决定）。见设计 8.1 和第 11 节。

## 默认路径为什么不变
1. `MappingWheel` 里新增的属性 `auto` 默认是 `None`。四处挂接（`command`、`fault`、`step`、`status`）都写成 `if self.auto is not None`，为 `None` 时一行旧代码都没跳过。
2. `main()` 只在会话冻结配置里有 `auto_controls` 时才导入 `mapping_auto`、调用 `attach_auto`。采集只在本机硬件配置有 `auto_controls` 时才把它抄进会话配置（11-10 加），建图的配置生成器不写；仓库和车上现在的 `hardware_setup.json` 里都没有。所以现在的面板会话里自动控制源根本不挂，也不导入这个模块。这次在 `main()` 的收尾里给 `auto_socket.close()` 加了 try（审查第 4 条），也在 `auto_socket is not None` 的分支里。
3. 即使写了 `auto_controls`，也要 `allowed`、当天的 `WC_AUTO_DRIVE_APPROVAL=APPROVED_BY_USER_<YYYYMMDD>`、审过的证据文件（其中的 `hold_to_run_switch` 和配置一致）、装了开关时的开关读入、证据列的驱动器参数回读成功，才建 `auto.sock`。回读在前面几条都过了以后、会话开始时读一次（11-10 加，只发白名单里的 FC03）。
4. 主循环原来是 `control_socket.pump(); owner.step()`，现在拆成两行，中间加一行 `if auto_socket is not None: auto_socket.pump()`，顺序不变。
5. 现场证据：11-09 面板跑的就是 b46193c7（录包 ZL1108shadow_2026_1109_203823 的 `software_hashes.json` 里 mapping_wheel.py 的 sha256 是 681827af…）。会话配置里没有 `auto_controls`，事务日志里没有任何 `auto_*` 事件。那次的轮速超时是 USB 转 485 从 USB 上掉线，出在没改过的 `WheelChannel.exchange`（设计第 7.1 节）。总管审查也确认了这一条。
6. 回归：原有的 `tests/operations/test_mapping_wheel.py`（110 个）不改一字照样通过；另有测试专门核对"不挂接时状态里没有 `auto`"和"`main()` 只在有 `auto_controls` 时挂接、关自动 socket 出错也照样写零"。

## 这次按审查改的
- 第 1 条：减速停车中出现急停、开关松开或过期、闸门急停锁存、后退，下一帧就写零；写前守卫在减速时也查这些。
- 第 2 条：闸门决定的新鲜度从闸门的 `mono_ns` 算起，端到端 0.3 s；超前 50 ms 以上的时间戳断开桥。端到端测试改用同一个假时钟。
- 第 4 条：会话退出时写日志失败不再跳过零速。
- 第 5–8 条：闸门急停锁存挡进入；拒绝原因单独放在 `refusal`，不盖掉停车原因；初始化写了使能就不再报手推；证据文件要求失联三项的停车时间，并且不超过通讯中断时间加 0.5 s。
- 第 9 条：任一轮反馈超过 27 rpm 的 1.2 倍持续 0.2 s，立即停。
- D 2026-11-10 13:15（总管定）：没装开关时桥照样是 `/wc_nav/estop` 唯一的发布者，来源是面板的软件急停（手动 socket 的 `estop` 包）和轮速判出的急停（至少报 0.5 s），闸门照样锁存。
- 用户的决定：`hold_to_run_switch: NOT_INSTALLED` 时状态里开关和急停报 null，使能来自"开始自动"；车上急停从轮速反馈看出来（有指令、两轮为 0 超过 0.3 s 就立即停，下次先重新初始化）；证据按档提速（第一次闭环 0.10 m/s、0.30 rad/s）。

## 假串口测了什么
- 假串口和 `WheelChannel.exchange` 的约定一样：写之前先调守卫函数，守卫抛出就不发；轮子默认跟着最后一次速度指令转，测试也可以把轮子"卡住"、"一直转"或"往后走"。测试检查写出的每一帧。
- `tests/operations/test_mapping_auto.py`（96 个），按设计的节：
  - 第 4 节：进入条件逐条拒绝（含过期的闸门时间戳、闸门急停锁存、`allow_reverse`）；伺服已释放时先重新初始化再爬升；空格、关界面立即停，"停止自动"、接管键、桥断开、决定过期按减速度停；减速中按急停、松开、开关过期、闸门急停锁存都改立即停；初始化途中退出；导航不能经桥请求自动；"开始自动"包的序号和字段。
  - 第 5 节：超限、负线速度、超过 `linear_ceiling`、非 ACTIVE 带速度、闸门重启、`seq` 不增都退出；容差两侧的边界；角加速度斜坡；27 rpm、上限不自洽、寄存器溢出；轮速超速。
  - 第 6 节：后退、指令为零还在转、反馈过期、停不下来进 FAULT；串口故障后关闭时补零；会话取消、`close()`、关 socket 时日志失败，都写出零速。
  - 开关未安装：状态报 null；"停止自动"、空格、WASD 让使能变 false；面板软件急停立即停、按着时不能开始、手动驾驶也停；轮速判出的急停报 0.5 s；闸门锁存时新的开始先解锁再进，1 s 内没解锁就拒绝；轮速判急停后要重新开始、先重新初始化；起步时轮子晚一点转不会误判。
- `tests/motion/test_auto_protocol.py`、`test_auto_evidence.py`、`test_auto_bridge.py`：消息格式的每种错误、证据文件的每种缺项和按档提速、桥的故障退出、两种使能来源的发法、急停周期 true、`base_status`，桥的 `main()`（QoS、定时器、出错退出，用替身 rclpy），以及桥到假轮控所有者的端到端。

## 还没做的
- RViz 面板的"开始自动""停止自动"和状态显示：要改 C++（`src/wc_bringup/src/mapping_teleop.cpp`）并重编面板，等你审完这部分、同意重编再做。
- 2003h 输入状态、故障码的低频读取：放到第一次闭环以后（总管 11-10 14:40）。驱动器参数在会话开始读一次（11-10 加）。只发 FC03 的现场小工具：[driver_readback.py](../../src/wc_motion/driver_readback.py)（设计 7.3 第 7 步），总管审过、你当天同意再用。
- 现场：按设计第 7.3 节修 USB 转换器，架空看急停怎么停、复位后会不会自己走，拔 USB 看驱动器停不停。
- 别人的部分：D 按 9.2 改闸门（`gate_command`、`require_base`、使能来源检查、单调钟，以及这次登记的"开始自动"使能）；B 看平滑器放行跳变和 `velocity_timeout`；C 用实测停车距离核对碰撞监控余量。
- [接口约定](../reference/interface_contract.md)：你 11-10 同意的改法已写进去（只有轮控所有者的自动控制源能向驱动器写速度，而且只在用户当天同意、证据文件通过时；其他进程照旧不发电机或导航命令）。

## 建议的审法
1. `git -C ~/wheelchair-wt/motion diff be96b7f7 HEAD -- src/wc_runtime/mapping_wheel.py`：先确认挂接只有这些。
2. `mapping_auto.py` 从 `AutoDrive.step`、`_step_active`、`_immediate_reason` 和 `_exit_reason` 看起，再看 `_drive`、`_guard`（每次非零写之前的复查）。
3. `auto_bridge.py` 的 `on_gate_command`、`enable_decision`、`switch_outputs`、`base_status`。
4. 测试名就是规则清单，可以对着设计第 4–6 节看。

返回[系统设计](README.md)。
