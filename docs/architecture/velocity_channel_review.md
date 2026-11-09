# 速度通道第 0 级：给用户审的代码说明
审的是分支 `feature/velocity-channel-20261109`（worktree `~/wheelchair-wt/motion`）上的两个提交：b46193c7（第 0 级）和它后面一个按 D 的审查改接口的提交。设计见[速度通道设计](velocity_channel.md)。全部只用假串口测过：没开过真串口，没连驱动器，没发过速度。

## 改了哪些文件
| 文件 | 内容 |
|---|---|
| [mapping_wheel.py](../../src/wc_runtime/mapping_wheel.py) | 唯一改动的旧文件，只加挂接（+24/−3 行），见下一节 |
| [mapping_auto.py](../../src/wc_runtime/mapping_auto.py)（新） | 轮控所有者里的自动控制源 `AutoDrive`、本机 socket `AutoSocket`、入口 `attach_auto` |
| [auto_protocol.py](../../src/wc_motion/auto_protocol.py)（新） | 闸门决定 `wc_gate_command_v1`、桥和轮控所有者之间的消息，只做校验 |
| [auto_bridge.py](../../src/wc_motion/auto_bridge.py)（新） | 底盘桥：域 100 里收闸门决定，经 `auto.sock` 交给轮控所有者；发 `base_status`、使能、急停 |
| [auto_evidence.py](../../src/wc_motion/auto_evidence.py)（新）和[证据模板](../../config/wheel_auto_drive_evidence.template.json) | 接通前的证据文件格式；模板总被拒绝 |
| 4 个测试文件（新） | 见"假串口测了什么" |
| README、本设计文档 | 文件列表和说明 |

## 默认路径为什么不变
1. `MappingWheel` 里新增的属性 `auto` 默认是 `None`。四处挂接（`command`、`fault`、`step`、`status`）都写成 `if self.auto is not None`，为 `None` 时一行旧代码都没跳过。
2. `main()` 只在会话冻结配置里有 `auto_controls` 时才导入 `mapping_auto`、调用 `attach_auto`。采集和建图的配置生成器都不写这个键，`hardware_setup.json` 里也没有。所以面板会话里自动控制源根本不挂，也不导入这个模块。
3. 即使写了 `auto_controls`，也要 `allowed`、当天的 `WC_AUTO_DRIVE_APPROVAL=APPROVED_BY_USER_<YYYYMMDD>`、审过的证据文件、使能开关和急停的读入、驱动器参数回读全有，才建 `auto.sock`。后两样现在不存在，所以车上一定是 `UNAVAILABLE`。
4. 主循环原来是 `control_socket.pump(); owner.step()`，现在拆成两行，中间加一行 `if auto_socket is not None: auto_socket.pump()`，顺序不变。
5. 现场证据：11-09 面板跑的就是 b46193c7（录包 ZL1108shadow_2026_1109_203823 的 `software_hashes.json` 里 mapping_wheel.py 的 sha256 是 681827af…，和 b46193c7 一致）。会话配置里没有 `auto_controls`，事务日志里没有任何 `auto_*` 事件。那次的轮速超时是 USB 转 485 从 USB 上掉线，出在没改过的 `WheelChannel.exchange`（设计第 7.1 节）。
6. 回归：原有的 `tests/operations/test_mapping_wheel.py`（110 个）不改一字照样通过；另有两个测试专门核对"不挂接时状态里没有 `auto`"和"`main()` 只在有 `auto_controls` 时挂接"。

## 假串口测了什么
- 假串口和 `WheelChannel.exchange` 的约定一样：写之前先调守卫函数，守卫抛出就不发。测试检查写出的每一帧。
- `tests/operations/test_mapping_auto.py`（49 个）：
  - 进入条件逐条拒绝：桥、闸门决定新鲜度、上限、`allow_reverse`、使能开关、急停、开关过期、按着键、车在动、参数回读不符、界面、代次；
  - 伺服已释放时先按旧顺序重新初始化，再按 0.30 m/s² 爬升，写出的速度字从不超过上限；
  - 超限、负线速度、超过 `linear_ceiling`、非 ACTIVE 带非零速度、闸门重启、`seq` 不增，都按 0.60 m/s² 减速停并退出自动；
  - 开关松开、急停、开关过期时立即写零；任何键接管只停车，松开再按才手动；
  - 桥断开、决定过期按减速停；界面断开经手动层立即停；停不下来进 FAULT；闸门保持时 20 Hz 写零；后退立即停；
  - 退出后不自动松轴；手推请求只停不松；串口故障锁存；状态 JSON 带反馈时间戳；
  - `attach_auto` 缺任何一个前提都不建 socket；socket 只接一个客户端、1 s 内要 hello、序号要递增（同 UID 的检查在代码里，测试只跑了同一个用户）。
- `tests/motion/test_auto_protocol.py`、`test_auto_evidence.py`、`test_auto_bridge.py`：消息格式的每种错误、证据文件的每种缺项、桥的故障退出、使能心跳和"松手立即发一次 false"、急停周期 true、`base_status` 的 `base_auto` 和原因，以及桥到假轮控所有者的端到端。

## 还没做的
- RViz 面板的"开始自动""停止自动"和状态显示：要改 C++（`src/wc_bringup/src/mapping_teleop.cpp`）并重编面板，等你审完这部分、同意重编再做。
- 第 1 级：使能开关和急停的读入（打算经驱动器 2003h 只读读出，见设计第 7.4 节）、驱动器参数只读回读、一个只发 FC03 的现场小工具。都要先审再用。
- 硬件：松手即停开关还没有；急停的接法下次现场按设计第 7.3 节确认。
- 别人的部分：D 按 9.2 改闸门（`gate_command`、`require_base`、使能来源检查、单调钟）；B 看平滑器放行跳变和 `velocity_timeout`；C 用实测停车距离核对碰撞监控余量。
- 要你定的：[接口约定](../reference/interface_contract.md)"不发送任何电机或导航命令"那一条（第 2 级架空之前）；证据文件要不要按 0.10、0.20、0.30 m/s 分档卡。

## 建议的审法
1. `git -C ~/wheelchair-wt/motion show b46193c7 -- src/wc_runtime/mapping_wheel.py`：先确认挂接只有这些。
2. `mapping_auto.py` 从 `AutoDrive.step` 和 `_exit_reason` 看起，再看 `_drive`、`_guard`（每次非零写之前的复查）。
3. `auto_bridge.py` 的 `on_gate_command`、`switch_outputs`、`base_status`。
4. 测试名就是规则清单，可以对着设计第 4–6 节看。

返回[系统设计](README.md)。
