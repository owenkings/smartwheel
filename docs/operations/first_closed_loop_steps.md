# 第一次空车闭环：现场步骤（0.10 m/s）
总管 2026-11-10 14:40 定的顺序，一次现场做完：修 USB → 架空用 WASD 试急停和拔 USB → 只读回读 → 架空跑测试指令源、核对速度换算 → 落地四种停法各 3 次 → 第一次闭环。空车、封闭走廊、人在车旁、手能按到急停，速度只有 0.10 m/s、0.30 rad/s。设计和通过线见[速度通道设计](../architecture/velocity_channel.md) 8.1，接口见 9.2。每一步写了命令、预期和必须停下的情况。

## 必须停下的情况
总管定的三种，出现就不往下做，发给总管：
- 急停按了轮子不停：马上关绿色电机开关。
- 急停复位后轮子自己走（复位时指令已经是零：松开了 W，或者自动已经退出）：按急停，关绿色开关。
- 速度换算不对：测试指令源的停车报告里，反馈和指令之比不在 90–110 %（容差是 A 提的，总管审）。

拔 USB 时驱动器不停不算，照样往下做：0.10 m/s 下掉线靠人按急停，驱动器参数以后再改（总管定）。这种情况下，复位急停之前一定先关绿色开关，否则驱动器可能接着按原来的速度走。

另外这些也先停下发给总管：轮子反转或只转一边；录制一开始就失败（例如报 `AUTO_STARTUP_READBACK_FAILED`）；停车距离超过通过线（软件零速、杀测试指令源 0.18 m，急停、拔 USB 0.30 m）或航向变化超过 5°。

## 通用
- 绿色电机开关是最后一道。凡是轮子会转的步骤，人都站在车旁、手放在急停上。
- 每个新终端先运行（`<当天>` 写成当天日期 YYYYMMDD，是你当天的同意）：
```bash
cd ~/wheelchair
export PYTHONPATH=$HOME/wheelchair/src PYTHONDONTWRITEBYTECODE=1
export WC_AUTO_DRIVE_APPROVAL=APPROVED_BY_USER_<当天>
R=/media/nvidia/tiany-USB/wheelchair
```
- 录制都用面板"数据录制"的常规录制（双雷达），前缀写在各步里，后缀照常勾上。
- "开始自动"：在终端运行 `python3 -m wc_runtime.auto_operator --project-root ~/wheelchair start`，然后 10 s 内点进 RViz 窗口，之后不要再点别的窗口。自动只在 RViz 窗口有焦点时跑：点出 RViz（包括点回终端）自动立即停。命令会一直打印状态，到 `auto AUTO` 或者"refused"为止。`... status` 看当前状态。
- 急停和拔 USB 每次都用手机录视频（架空拍轮子，落地拍整车和地上的胶带）。
- 每一步做完，把终端输出、观察到的情况和视频发给总管。

## 0. 开始前（不动车）
1. 代码：总管审过 velocity 分支，合进面板运行的 `~/wheelchair`。第 9 步还要 D 按设计 9.2 改好的闸门和 D 的现场步骤。
2. 关掉面板，带当天同意重新打开：`WC_AUTO_DRIVE_APPROVAL=APPROVED_BY_USER_<当天> bash ~/wheelchair/scripts/panel`。当天不用再重启；本机硬件配置里没有 `auto_controls` 时它不起任何作用。
3. `sync`。

## 1. 修 USB（绿色开关关着，约 15 分钟）
1. USB 转 485 插牢、线固定，能直接插 Orin 更好；J5 上的杜邦线换成拧紧的端子。
2. 命令：`ls -l /dev/serial/by-id/ | grep 5C66036979`。预期：有一行。
3. 来回推车、动一动线 1 分钟，然后 `journalctl -k --since -10min | grep -i 'usb disconnect'`。预期：没有转换器的掉线行。
4. 还在掉：换一个口或直接插 Orin 再试；还掉就停下，发给总管（掉线会让整段录制失败）。

## 2. 架空，用现有 WASD（约 25 分钟）
车垫高，两个驱动轮离地 ≥ 2 cm，垫块不会滑；轮子周围没人没东西。开绿色开关。面板开常规录制，前缀 `ZLauto_stand_wasd`。
1. 急停，3 次：RViz 窗口里按住 W（0.10 m/s），按下急停。预期：轮子停。记下是减速停住、立刻停住还是自由滑转，ALM、PWR 灯亮没亮。松开 W，再旋转复位急停。预期：轮子不动。必须停下：按了不停；复位后自己转。
2. 第 3 次以后加一次：按着 W 复位急停，记下轮子会不会接着转。会转是驱动器照着还在发的速度走，这正是自动要防的情况；记下来就行，不算"自己走"。
3. 拔 USB，3 次：按住 W，拔掉 USB 转 485。预期：面板的录制报错结束（`SOURCE_EXITED: wheel`，轮控所有者没了串口）。看视频记下驱动器停不停、几秒停。
   - 自己停了：记下时间。
   - 不停：按急停。轮子还在空中，复位急停看一次：轮子又转起来就是"复位后自己走"，必须停下。看完按急停，关绿色开关。不停本身照样往下做。
   - 每次之后：关绿色开关 → 插回 USB → `ls -l /dev/serial/by-id/ | grep 5C66036979` → 开绿色开关 → 面板重新开常规录制（前缀 `ZLauto_stand_usb`）。
4. 结束录制。把每次怎么停、几秒、复位后动没动、拔 USB 停没停发给总管。

## 3. 只读回读（不开录制，约 5 分钟）
面板上没有录制在跑时才能读（轮控占着串口时工具会拒绝）。
```bash
python3 -m wc_motion.driver_readback --project-root ~/wheelchair --label stand_all --dry-run
WC_DRIVER_READBACK_APPROVAL=APPROVED_BY_USER_<当天> python3 -m wc_motion.driver_readback --project-root ~/wheelchair --label stand_all
```
- 预期：第一条只列出要发的 FC03 请求；第二条每个寄存器一行值或 `MODBUS_EXCEPTION`，结果在 `$R/reports/driver_readback/<UTC>_stand_all/result.json`。
- 停下：出现 `TRANSPORT_FAILED`（超时、校验错）就先查 USB 线和 J5，不往下。
- 把输出发给总管。

## 4. 写架空阶段的证据（A 写、总管核对，约 15 分钟，车不动）
A 写两处（都是本机文件，不进 git），总管核对以后才开第 5 步的录制：
- 本机硬件配置 `config/local/project/hardware_setup.json` 加：
```json
"auto_controls": {"allowed": true, "evidence": "config/local/project/wheel_auto_drive_evidence.json",
  "hold_to_run_switch": "NOT_INSTALLED", "max_linear_m_s": 0.10, "max_angular_rad_s": 0.30, "max_wheel_rpm": 27.0,
  "max_linear_accel_m_s2": 0.30, "max_linear_decel_m_s2": 0.60, "max_angular_accel_rad_s2": 0.80,
  "note": "第一次空车闭环，总管 2026-11-10 14:40 的计划"}
```
- 证据文件 `config/local/project/wheel_auto_drive_evidence.json`，`stage` 为 `STAND_TEST`：

| 字段 | 内容和来源 |
|---|---|
| `device_id` | `ZLAC8030D-5C66036979`（本机 `wheel_feedback_current.json`） |
| `firmware_id` | 第 3 步 20A0h 的原始值 |
| `hold_to_run_switch` | `installed: false`，用户 11-09 23:50 的决定 |
| `approved_limits` | 和上面 `auto_controls` 一样的六个值（用户当天同意） |
| `driver_readback` | 第 3 步读到的配置类寄存器，每个字一条（`mask` 65535，`equals` 读数）：2000h、2007h、2008h、2011h、2016h–2018h、2084h、2085h、20A0h。2003h 输入状态和 20A5h、20A6h 故障码会变，只记录不比对。会话开始时轮控所有者再读一次，值变了就不让开始自动 |
| `communication_timeout_s` | 2000h 读得出、能按手册换算成秒的：填秒数，加 `communication_timeout` 一条回读，驱动器自己停的停车时间要在它加 0.5 s 以内；不知道的：`null`（只在 0.10 m/s 以内可以） |
| `stop_tests.estop` | 第 2 步：`trials` 3，`moved_after_reset: false` |
| `stop_tests.usb_unplugged` | 第 2 步：自己停的 `driver_stopped: true` 和最长停车时间；不停的 `driver_stopped: false`，`decision` 写总管的决定，`decided_by: "coordinator"` |
| 来源 | 结束后数据盘上各段录制的 `sources/wheel_feedback.jsonl` 和回读的 `result.json`（路径和 sha256）；视频拷到数据盘以后可以补进来 |
| `reviewer`、`reviewed_at` | 总管、当天 |

检查（预期 `PASS: stage STAND_TEST, device ZLAC8030D-5C66036979, limits 0.10 m/s 0.30 rad/s ...`）：
```bash
python3 -m wc_motion.auto_evidence --project-root ~/wheelchair --evidence config/local/project/wheel_auto_drive_evidence.json
```

## 5. 架空跑测试指令源（约 20 分钟）
还是架空。面板开常规录制，前缀 `ZLauto_stand_auto`。开两个终端（都按"通用"准备好）。
1. 终端 1：`python3 -m wc_runtime.auto_operator --project-root ~/wheelchair status`。预期：`auto NONE (IDLE) ... limits 0.1 m/s 0.3 rad/s, evidence STAND_TEST`。报 `found 0` 时后面会带上轮控所有者自己说的原因（例如 `EVIDENCE_REJECTED: ...`），发给总管。
2. 软件零速，3 次。终端 1：`python3 -m wc_motion.auto_test_source --project-root ~/wheelchair --stop software_zero`（它最多等 60 s；下面的 `... --stop external` 等都是这条命令换 `--stop` 的值）；终端 2："开始自动"。预期：轮子朝前（和按 W 同向）转 3 s 后停住，终端 1 最后打印一行 `1. software_zero  指令 0.100 m/s，反馈 0.0xx m/s（xx%），... 停车 x.xx s、0.0xx m、航向变 x.x°`。必须停下：比例不在 90–110 %；轮子反转或只转一边。
3. 自动中按急停，1 次。终端 1：`... --stop external`；终端 2："开始自动"。轮子转起来 2 s 后按急停。预期：轮子停，终端 1 报 `estop`，`status` 是 `auto NONE (AUTO_EXITED: WHEELS_STOPPED_WHILE_COMMANDED)`。旋转复位急停：轮子不动。必须停下：按了不停；复位后自己转。
4. 结束录制以前：`python3 -m wc_motion.auto_stop_report --project-root ~/wheelchair --label stand_auto`（打印这段录制的全部试次，并存到 `$R/reports/auto_stop_tests/`）。然后结束录制，把输出发给总管。

## 6. 写落地阶段的证据（A 写、总管核对，约 5 分钟）
A 把 `stage` 改成 `GROUND_TEST`，加 `stop_tests.auto_stand_checks`：`trials` 3，`speed_ratios` 是第 5 步三次软件零速的反馈/指令比，来源是第 5 步的停车报告和录制。检查命令同第 4 步，预期 `PASS: stage GROUND_TEST`。

## 7. 落地，四种停法各 3 次（约 60 分钟）
车放在封闭走廊的地上，车头朝走廊的长方向，车前 3 m 内没人没东西；地上用胶带贴起点。测试指令源不看障碍、不看定位，只直走：人站在车侧、手放在急停上，跟着车走。面板开常规录制，前缀 `ZLauto_ground`。每次试完把车弄回起点：RViz 里按住 S 倒回去，或者点一下 W 松开、车进了手推以后推回去。车停稳再开下一次。
1. 软件零速，3 次：终端 1 `... --stop software_zero`，终端 2"开始自动"。预期：走约 0.3 m 停下，报告 `software_zero`，停车距离几厘米。
2. 杀测试指令源，3 次：终端 1 `... --stop kill`，终端 2"开始自动"。它在自动开始 3 s 后用 `kill -9` 杀掉自己的子进程，不用你回终端。预期：终端 1 打印 `KILLED: kill -9 ...`，车减速停，报告 `source_lost`（轮控所有者：`BRIDGE_DISCONNECTED`）。
3. 急停，3 次：终端 1 `... --stop external`，终端 2"开始自动"，走起来 2 s 后按急停。预期：车停，报告 `estop`；复位急停，车不动。必须停下：按了不停；复位后车自己走。
4. 前三种做完、结束录制以前：`python3 -m wc_motion.auto_stop_report --project-root ~/wheelchair --label ground`。然后结束录制。
5. 拔 USB，3 次，最后做（每次录制都会报错结束）：每次新开常规录制（前缀 `ZLauto_ground_usb`），终端 1 `... --stop external`，终端 2"开始自动"，走起来 2 s 后拔 USB。预期：驱动器自己停，或者不停、人按急停。车停稳以后：先关绿色开关，再复位急停、插回 USB、开绿色开关。停车时间和距离看视频（拔 USB 那一刻到车停）；拔之前的速度用 `python3 -m wc_motion.auto_stop_report --session $R/data/experiments/<这段录制的文件夹名>` 看（`feedback_lost` 一行）。
6. 把输出、视频和拔 USB 三次的时间、距离发给总管。

## 8. 写闭环阶段的证据（A 写、总管核对，约 10 分钟）
A 把 `stage` 改成 `SUPERVISED_CLOSED_LOOP`，加 `braking`：软件零速（`software_zero`）、杀测试指令源（`source_killed`）、急停（`estop`）各 3 个停车距离和最大航向变化，来自第 7 步的停车报告；拔 USB（`communication_loss`）只在驱动器自己停时要，距离来自视频。通过线：前两种 ≤ 0.18 m，后两种 ≤ 0.30 m；3 次的离散不超过均值的 20 % 或 2 cm；航向变化 ≤ 5°。检查命令同第 4 步，预期 `PASS: stage SUPERVISED_CLOSED_LOOP`。

## 9. 第一次闭环
按 D 的现场步骤开闸门（实车闭环配置，设计 9.2）、Nav2 和目标。面板开常规录制，前缀 `ZLauto_loop`。A 这边只有底盘桥和"开始自动"：
```bash
source /opt/ros/humble/setup.bash && source install/main/setup.bash
export PYTHONPATH=$HOME/wheelchair/src:$PYTHONPATH ROS_DOMAIN_ID=100 ROS_LOCALHOST_ONLY=1
python3 -m wc_motion.auto_bridge --project-root ~/wheelchair $(python3 -m wc_runtime.auto_operator --project-root ~/wheelchair session)
```
桥起来以后在另一个终端"开始自动"。停车：RViz 里按空格，或者按急停；点出 RViz 也会立即停。必须停下的情况同上，另外闸门以外的任何东西让车动了也停下。

返回[功能操作手册](README.md)。
