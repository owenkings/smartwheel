# 第一次空车闭环：现场步骤（0.10 m/s）
总管 2026-11-10 14:40 定的顺序，一次现场做完：修 USB → 架空用 WASD 试急停和拔 USB → 只读回读 → 架空跑测试指令源、核对速度换算 → 落地四种停法各 3 次 → 第一次闭环。11-10 23:15 按第二次审查（`~/wheelchair_coord/reviews/level0_review2_20261110.md`）补了复位前的确认、远程画面断开时的做法、低速检查和几处证据字段。空车、封闭走廊、人在车旁、手能按到急停，速度只有 0.10 m/s、0.30 rad/s。设计和通过线见[速度通道设计](../architecture/velocity_channel.md) 8.1，接口见 9.2。每一步写了命令、预期和必须停下的情况。

## 必须停下的情况
下面四种出现就不往下做，发给总管（前三种总管 14:40 定，第四种来自第二次审查第 1 条）：
- 急停按了轮子不停：马上关绿色电机开关。
- 急停复位后轮子自己走（复位时指令已经是零：松开了 W，或者自动已经退出）：按急停，关绿色开关。
- 速度换算不对：测试指令源的停车报告里，车轴、左轮、右轮三个反馈/指令之比有一个不在 90–110 %（容差是 A 提的，总管审）。
- 自动中按了急停，轮子停住以后 1 s 内终端 1 没打出 `OWNER_STOPPED: the owner left AUTO: WHEELS_STOPPED_WHILE_COMMANDED`：轮控所有者没认出急停，还在写原来的速度。先在 RViz 按空格，再复位急停，然后停下。

拔 USB 时驱动器不停不算，照样往下做：0.10 m/s 下掉线靠人按急停，驱动器参数以后再改（总管定）。这次不知道驱动器的通讯中断时间，所以拔了以后 1.5 s 内（数到二）没停就按"不停"处理，按急停。这种情况下，复位急停之前一定先关绿色开关：USB 拔了，没人能给驱动器写零，它可能接着按原来的速度走。

另外这些也先停下发给总管：轮子反转或只转一边；录制一开始就失败（例如报 `AUTO_STARTUP_READBACK_FAILED`）；停车距离超过通过线（软件零速、杀测试指令源 0.18 m，急停、拔 USB 0.30 m）或航向变化超过 5°。

## 通用
- 远程画面卡住或断开：旁边的人马上按急停。面板跑在 Orin 本机的显示（DISPLAY :1）上，NoMachine 只是镜像它：NoMachine 断了，车上的 RViz 照样有焦点、照样发心跳，自动不会停。
- 用户的备用有线键盘插在 Orin 空着的 USB 口上（别碰 USB 转 485 的线），固定在车上、陪同的人伸手够得到。RViz 窗口有焦点时，在它上面按空格不经网络也能停。
- 绿色电机开关是最后一道。凡是轮子会转的步骤，人都站在车旁、手放在急停上。
- 每个新终端先运行（`<当天>` 写成当天日期 YYYYMMDD，是你当天的同意）：
```bash
cd ~/wheelchair
export PYTHONPATH=$HOME/wheelchair/src PYTHONDONTWRITEBYTECODE=1
export WC_AUTO_DRIVE_APPROVAL=APPROVED_BY_USER_<当天>
R=/media/nvidia/tiany-USB/wheelchair
```
- 录制都用面板"数据录制"的常规录制（双雷达），前缀写在各步里，后缀照常勾上。第 4 步写了 `auto_controls` 以后不开实时建图：建图会话也会带上它，一开始就读驱动器。
- "开始自动"：在终端运行 `python3 -m wc_runtime.auto_operator --project-root ~/wheelchair start`（`bash ~/wheelchair_coord/field/auto_start.sh` 是同一个命令），然后 10 s 内点进 RViz 窗口，之后不要再点别的窗口。自动只在 RViz 窗口有焦点时跑：点出 RViz（包括点回终端）自动立即停。命令会一直打印状态，到 `auto AUTO` 或者"refused"为止。`... status` 看当前状态。
- 自动中按了急停：第 5、7 步等终端 1 打出 `OWNER_STOPPED: the owner left AUTO: WHEELS_STOPPED_WHILE_COMMANDED` 再复位；第 9 步没有这行输出，一律先在 RViz 按空格再复位。
- 急停和拔 USB 每次都用手机录视频（架空拍轮子，落地拍整车和地上的胶带）。视频存着、记下文件名，不作证据文件的来源（见第 4 步）。
- 每一步做完，把终端输出、观察到的情况和视频发给总管。

## 0. 开始前（不动车）
1. 代码：主检出 `~/wheelchair` 11-10 23:15 已快进到 e54c8860，第 1–5 步用它就行。第二次审查的修改（轮速判急停、证据新字段）总管复看后再快进一次，第 6 步以前一定要进来；快进只在两段录制之间做。第 9 步还要 `~/wheelchair_coord/field/run_closed_loop.sh`（总管按 D 的闭环准备）。
2. 关掉面板，在 Orin 的桌面上（NoMachine 看到的就是它）开终端，带当天同意重新打开：`WC_AUTO_DRIVE_APPROVAL=APPROVED_BY_USER_<当天> bash ~/wheelchair/scripts/panel`。当天不用再重启；本机硬件配置里没有 `auto_controls` 时它不起任何作用。
   - 重新打开面板以前，先核对两台雷达的成像参数（11-11 晚 A 试改过，两台断电前停在 int1 = int4 = 800 µs）：`bash $R/reports/lidar_frequency/20261111T1445Z_room/restore/restore_s0.sh`。两台都显示"和原设置一致（含原始配置 78 字节）"才往下做。它不是原设置时会自动改回再核对；第一次读回是什么（800 还是 1600）记下来发给总管，这能说明设置断电后还在不在。详见同目录上一级的 `README.md`。
3. 有线键盘插到 Orin、固定在车上（见"通用"）。
4. `sync`。

## 1. 修 USB（绿色开关关着，约 15 分钟）
1. USB 转 485 插牢、线固定，能直接插 Orin 更好；J5 上的杜邦线换成拧紧的端子。
2. 命令：`ls -l /dev/serial/by-id/ | grep 5C66036979`。预期：有一行。
3. 来回推车、动一动线 1 分钟，然后 `journalctl -k --since -10min | grep -i 'usb disconnect'`。预期：没有转换器的掉线行。
4. 还在掉：换一个口或直接插 Orin 再试；还掉就停下，发给总管（掉线会让整段录制失败）。

## 2. 架空，用现有 WASD（约 35 分钟）
车垫高，两个驱动轮离地 ≥ 2 cm，垫块不会滑；轮子周围没人没东西。开绿色开关。面板开常规录制，前缀 `ZLauto_stand_wasd`。
1. 有线键盘：点一下 RViz 窗口，在车上的有线键盘上按住 W 1 s。预期：两只轮朝前转（和车往前走的方向一致），松开就停。再按住 A、D 各 1 s。预期：A 时左轮朝后、右轮朝前（原地向左转），D 反过来。必须停下：轮子反转或只转一边。后面几步用哪个键盘都行。
2. 急停，3 次：按住 W（0.10 m/s），按下急停。预期：轮子停。记下是减速停住、立刻停住还是自由滑转，ALM、PWR 灯亮没亮。松开 W，再旋转复位急停。预期：轮子不动。必须停下：按了不停；复位后自己转。
3. 第 3 次以后加一次：按着 W 复位急停，记下轮子会不会接着转。会转是驱动器照着还在发的速度走，这正是自动要防的情况；记下来就行，不算"自己走"。
4. 拔 USB，3 次：按住 W，拔掉 USB 转 485。预期：面板的录制报错结束（`SOURCE_EXITED: wheel`，轮控所有者没了串口）。看视频记下驱动器停不停、几秒停。
   - 1.5 s 内自己停了：记下时间。
   - 1.5 s 内没停（数到二还在转）：按急停，按"不停"记。轮子在空中，可以复位急停看一次驱动器会不会接着按原来的速度转；会转就记下来。这是复位时驱动器手里还是原来的速度，不算"复位后自己走"，但说明落地拔 USB 以后复位之前必须先关绿色开关。看完按急停，关绿色开关。
   - 每次之后：关绿色开关 → 插回 USB → `ls -l /dev/serial/by-id/ | grep 5C66036979` → 开绿色开关 → 面板重新开常规录制（前缀 `ZLauto_stand_usb`）。
5. 结束录制。把每次怎么停、几秒、复位后动没动、拔 USB 停没停发给总管。
6. 低速（B 问的，约 10 分钟）：闭环里最慢的速度是到点前 0.02–0.05 m/s 和减速框里原地转约 0.05 rad/s（每只轮 0.015 m/s，约 0.9 rpm，指令 7 个字），看轮子在这个速度转不转得起来。A 把本机硬件配置 `manual_controls` 的 `max_linear_m_s` 改成 0.02、`max_angular_rad_s` 改成 0.05（只把这两个上限调低；用户同意的是 0.10、0.20）。面板重开常规录制（前缀 `ZLauto_stand_slow`），按住 W 5 s、A 5 s、D 5 s，记下轮子转不转、转得匀不匀。结束录制，A 改回 0.10、0.20。改以前、改以后都打印出来给总管看：
```bash
python3 -c "import json; c = json.load(open('config/local/project/hardware_setup.json'))['manual_controls']; print(c['max_linear_m_s'], c['max_angular_rad_s'])"
```

## 3. 只读回读（不开录制，约 5 分钟）
面板上没有录制在跑时才能读（轮控占着串口时工具会拒绝）。
```bash
python3 -m wc_motion.driver_readback --project-root ~/wheelchair --label stand_all --dry-run
WC_DRIVER_READBACK_APPROVAL=APPROVED_BY_USER_<当天> python3 -m wc_motion.driver_readback --project-root ~/wheelchair --label stand_all
```
- 预期：第一条只列出要发的 FC03 请求；第二条每个寄存器一行值或 `exception`，行尾是这次读用了多少毫秒，结果在 `$R/reports/driver_readback/<UTC>_stand_all/result.json`。
- 耗时：轮控所有者在每段录制开始时自己再读一遍，每次只等 100 ms（这个工具等 200 ms）。每行都应远小于 100 ms；有超过 50 ms 的，停下发给总管，否则第 5 步的录制可能一开始就失败。
- 停下：出现 `TRANSPORT_FAILED`（超时、校验错）就先查 USB 线和 J5，不往下。
- 把输出发给总管。

## 4. 写架空阶段的证据（A 写、总管核对，约 15 分钟，车不动）
A 写两处（都是本机文件，不进 git），总管核对以后才开第 5 步的录制：
- 本机硬件配置 `config/local/project/hardware_setup.json` 加：
```json
"auto_controls": {"allowed": true, "evidence": "config/local/project/wheel_auto_drive_evidence.json",
  "hold_to_run_switch": "NOT_INSTALLED", "max_linear_m_s": 0.10, "max_angular_rad_s": 0.30, "max_wheel_rpm": 12.0,
  "max_linear_accel_m_s2": 0.30, "max_linear_decel_m_s2": 0.60, "max_angular_accel_rad_s2": 0.80,
  "note": "第一次空车闭环，总管 2026-11-10 14:40 的计划"}
```
  `max_wheel_rpm` 12：0.10 m/s 加 0.30 rad/s 时外侧轮 11.0 rpm，上限不能低于它；轮速反馈超过它的 1.2 倍（14.4 rpm，约 0.25 m/s）持续 0.2 s 就立即停（第二次审查第 3 条）。
- 证据文件 `config/local/project/wheel_auto_drive_evidence.json`，`stage` 为 `STAND_TEST`：

| 字段 | 内容和来源 |
|---|---|
| `device_id` | `ZLAC8030D-5C66036979`（本机 `wheel_feedback_current.json`） |
| `firmware_id` | 第 3 步 20A0h 的原始值 |
| `hold_to_run_switch` | `installed: false`，`decision` 写用户 11-09 23:50 的决定，`decided_by: "user"`，`decided_at: "2026-11-09 23:50"` |
| `approved_limits` | 和上面 `auto_controls` 一样的六个值（用户当天同意），`max_wheel_rpm` 是 12 |
| `driver_readback` | 第 3 步读到的配置类寄存器，每个字一条（`mask` 65535，`equals` 读数）：2000h、2007h、2008h、2011h、2016h–2018h、2084h、2085h、20A0h。2003h 输入状态和 20A5h、20A6h 故障码会变，只记录不比对。会话开始时轮控所有者再读一次，值变了就不让开始自动 |
| `communication_timeout_s` | `null`：2000h 的单位还没有 8030D 手册核对。这时拔 USB 驱动器自己停的，停车时间要在 1.5 s 以内 |
| `stop_tests.estop` | 第 2 步：`verified: true`，`trials: 3`，`moved_after_reset: false`，`sources` |
| `stop_tests.usb_unplugged` | 第 2 步，`verified: true`，`trials: 3`，`sources`。都在 1.5 s 内自己停的：`driver_stopped: true`，`max_stop_time_s` 填最长一次。有一次没停或晚于 1.5 s：`driver_stopped: false`，`max_stop_time_s: null`，`decision` 写总管的决定，`decided_by: "coordinator"`，`decided_at: "2026-11-10 14:40"` |
| 来源（`sources`） | 结束后数据盘上各段录制的 `sources/wheel_feedback.jsonl` 和回读的 `result.json`（路径和 sha256）。视频不作来源：轮控所有者每段录制开始都把每个来源整份读一遍算 sha256，采集只等轮速反馈 20 s；视频的文件名写进 `note` |
| `reviewer`、`reviewed_at` | 总管、当天 |

检查（预期 `PASS: stage STAND_TEST, device ZLAC8030D-5C66036979, limits 0.10 m/s 0.30 rad/s ...`）：
```bash
python3 -m wc_motion.auto_evidence --project-root ~/wheelchair --evidence config/local/project/wheel_auto_drive_evidence.json
```

## 5. 架空跑测试指令源（约 20 分钟）
还是架空。面板开常规录制，前缀 `ZLauto_stand_auto`。开两个终端（都按"通用"准备好）。
1. 终端 1：`python3 -m wc_runtime.auto_operator --project-root ~/wheelchair status`。预期：`auto NONE (IDLE) ... limits 0.1 m/s 0.3 rad/s, evidence STAND_TEST`。报 `found 0` 时后面会带上轮控所有者自己说的原因（例如 `EVIDENCE_REJECTED: ...`），发给总管。
2. 软件零速，3 次。终端 1：`python3 -m wc_motion.auto_test_source --project-root ~/wheelchair --stop software_zero`（它最多等 60 s；下面的 `... --stop external` 等都是这条命令换 `--stop` 的值）；终端 2："开始自动"。预期：轮子朝前（和按 W 同向）转 3 s 后停住，终端 1 最后打印一行 `1. software_zero  指令 0.100 m/s，反馈 0.0xx m/s（xx%，左轮 xx%、右轮 xx%），... 停车 x.xx s、0.0xx m、航向变 x.x°`。必须停下：三个比有一个不在 90–110 %；轮子反转或只转一边。
3. 自动中按急停，至少 1 次。终端 1：`... --stop external`；终端 2："开始自动"。轮子转起来 2 s 后按急停。预期：轮子停，停住后 1 s 内终端 1 打出 `OWNER_STOPPED: the owner left AUTO: WHEELS_STOPPED_WHILE_COMMANDED`，这以后才旋转复位急停：轮子不动。终端 1 最后报 `estop`，`status` 是 `auto NONE (AUTO_EXITED: WHEELS_STOPPED_WHILE_COMMANDED)`。必须停下：按了不停；没打出那一行（先按空格再复位）；复位后自己转。
   - 录制要是在急停以后紧接着报错结束（`wheel.log` 里有 `NOT_STATIONARY_AFTER_STOP`）：急停以后有一只轮一直读到 3 个字以上，轮子停住后 1 s 内确认不了停稳（11-11 起两轮都不超过 3 个字就算停稳）。零速已经写了，复位急停时轮子应该不动。记下来发给总管，重开录制再做。
4. 结束录制以前：`python3 -m wc_motion.auto_stop_report --project-root ~/wheelchair --label stand_auto`（打印这段录制的全部试次，并存到 `$R/reports/auto_stop_tests/`）。然后结束录制，把输出、急停复位后轮子动没动发给总管。

## 6. 写落地阶段的证据（A 写、总管核对，约 5 分钟）
A 把 `stage` 改成 `GROUND_TEST`，加 `stop_tests.auto_stand_checks`：`verified: true`；`trials: 3`；`speed_ratios` 是第 5 步三次软件零速的车轴反馈/指令比（每轮的比第 5 步已经核对过）；`estop_trials` 是 5.3 自动中按急停的次数（至少 1）；`moved_after_reset: false`，来自 5.3 的视频和复位时的观察；`sources` 是第 5 步录制的 `sources/wheel_feedback.jsonl` 和停车报告的 `report.json`。检查命令同第 4 步，预期 `PASS: stage GROUND_TEST`。

## 7. 落地（约 75 分钟）
车放在封闭走廊的地上，车头朝走廊的长方向，车前 3 m 内没人没东西；地上用胶带贴起点。测试指令源不看障碍、不看定位，只直走：人站在车侧、手放在急停上，跟着车走。面板开常规录制，前缀 `ZLauto_ground`。每次试完把车弄回起点：RViz 里按住 S 倒回去，或者点一下 W 松开、车进了手推以后推回去。车停稳再开下一次。
1. 软件零速，3 次：终端 1 `... --stop software_zero`，终端 2"开始自动"。预期：走约 0.3 m 停下，报告 `software_zero`，停车距离几厘米。
2. 杀测试指令源，3 次：终端 1 `... --stop kill`，终端 2"开始自动"。它在自动开始 3 s 后用 `kill -9` 杀掉自己的子进程，不用你回终端。预期：终端 1 打印 `KILLED: kill -9 ...`，车减速停，报告 `source_lost`（轮控所有者：`BRIDGE_DISCONNECTED`）。
3. 急停，3 次：终端 1 `... --stop external`，终端 2"开始自动"，走起来 2 s 后按急停。预期：车停，停住后 1 s 内终端 1 打出 `OWNER_STOPPED: the owner left AUTO: WHEELS_STOPPED_WHILE_COMMANDED`，这以后才复位：车不动；报告 `estop`。必须停下：按了不停；没打出那一行（先按空格再复位）；复位后车自己走。录制紧接着报错结束（`NOT_STATIONARY_AFTER_STOP`）的，同 5.3。
4. 前三种做完、结束录制以前：`python3 -m wc_motion.auto_stop_report --project-root ~/wheelchair --label ground`。然后结束录制。
5. 拔 USB，3 次（每次录制都会报错结束）：每次新开常规录制（前缀 `ZLauto_ground_usb`），终端 1 `... --stop external`，终端 2"开始自动"，走起来 2 s 后拔 USB。预期：驱动器 1.5 s 内自己停；数到二还在走就按急停，按"不停"记。车停稳以后：先关绿色开关，再复位急停、插回 USB、开绿色开关。停车时间和距离看视频（拔 USB 那一刻到车停）；拔之前的速度用 `python3 -m wc_motion.auto_stop_report --session $R/data/experiments/<这段录制的文件夹名>` 看（`feedback_lost` 一行）。
6. 低速（B 问的，约 10 分钟）：和 2.6 一样，A 把两个手动上限改成 0.02 m/s、0.05 rad/s；面板开常规录制（前缀 `ZLauto_ground_slow`），按住 W 5 s（约走 0.1 m）、A 5 s、D 5 s（各转约 14°），记下车动不动、动得匀不匀。结束录制，A 改回 0.10、0.20，打印出来给总管看。
7. 把输出、视频、急停复位后车动没动、拔 USB 三次的时间和距离，以及低速时车动没动，发给总管。

## 8. 写闭环阶段的证据（A 写、总管核对，约 10 分钟）
A 把 `stage` 改成 `SUPERVISED_CLOSED_LOOP`，加 `braking`：软件零速（`software_zero`）、杀测试指令源（`source_killed`）、急停（`estop`）各一行，每行 3 个停车距离和最大航向变化，来自第 7 步的停车报告；`speed_m_s` 写批准的 0.10（不写报告里量到的 0.098 之类，否则检查会说没在批准的最高速度下测）；急停那一行加 `moved_after_reset: false`，来自 7.3 的视频和复位时的观察；`sources` 是第 7 步录制的 `sources/wheel_feedback.jsonl` 和 `report.json`。拔 USB（`communication_loss`）只在驱动器三次都 1.5 s 内自己停时要，距离来自视频。通过线：前两种 ≤ 0.18 m，后两种 ≤ 0.30 m；3 次的离散不超过均值的 20 % 或 2 cm；航向变化 ≤ 5°。检查命令同第 4 步，预期 `PASS: stage SUPERVISED_CLOSED_LOOP`。

## 9. 第一次闭环
前提：第 0–8 步做完，第 8 步的检查 PASS；面板已开常规录制（前缀 `ZLauto_loop`，双雷达），设备都就绪。车放在 11-07 建图起点的胶带处，朝向按脚本里说的放。
1. 终端 1：`bash ~/wheelchair_coord/field/run_closed_loop.sh`。它找面板的录制会话、冻结面板当前的配置并核对、问你同不同意（输入 `yes`），然后起闸门、Nav2 和底盘桥。不要另起 `wc_motion.auto_bridge`：两个桥会让闸门停在 `ENABLE_SOURCE_AMBIGUOUS`。
2. 站着看每秒一行的"闭环状态"约 30 s。预期：闸门 `DISABLED(OPERATOR_ENABLE_OFF)`、底盘不在 AUTO、急停否、目标 `NAVIGATING`，"影子判断"是"会放行 0.10 m/s …"。起点检查没通过、定位不可信、影子判断方向不对：不要开始自动，按 Ctrl-C。
3. 都对：终端 2 `bash ~/wheelchair_coord/field/auto_start.sh`，然后 10 s 内点进 RViz 窗口。车直走约 4 m 到目标。
4. 停车：RViz 里按空格（车上的有线键盘也行），或者按急停；点出 RViz 也会立即停。按了急停以后，一律先在 RViz 按空格，再复位急停。必须停下的情况同上；另外闸门以外的任何东西让车动了，也停下。
5. 低速（B 问的）：看车到点前（0.05 m/s 以下）和原地转（0.30 rad/s，减速框里约 0.05 rad/s）时动不动。A 事后从录制的轮控日志里查这几段的指令和轮速，写进结果。
6. 结束：按 Ctrl-C（先停桥，轮控所有者写零，再停其他进程），然后在面板结束录制。

## 10. 现场结束
1. A 把本机硬件配置里的 `auto_controls` 删掉，或者改成 `"allowed": false`（第二次审查第 5 条：留着它，以后的录制和实时建图一开始都会读驱动器、建 `auto.sock`）。打印出来给总管看：
```bash
python3 -c "import json; print(json.load(open('config/local/project/hardware_setup.json')).get('auto_controls'))"
```
   预期：`None`，或者 `allowed` 是 `False`。
2. 关绿色开关，`sync`。

返回[功能操作手册](README.md)。
