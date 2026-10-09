本诊断仅检查 RViz 在私有模拟手动状态连接后的布局和渲染，不启动传感器、串口或控制进程。地图来自第 04 轮已保存文件；四路图像是已有脚本生成的 8 Hz 人工色条。结果不能作为实时四相机或手动驾驶验收。

构建工具读取当前 `mapping_rviz.cpp`，在新报告目录生成诊断副本。它保持原生 RViz、相机插件、窗口布局和关闭策略，只把手动面板的会话来源换成直接构造的 `synthetic_manual_dashboard_left_20260914_01`。不调用生产会话解析器，不创建 `source_mode=real` 的模拟身份，不覆盖原始源码、构建对象或安装文件。当前 `mapping_teleop.cpp` 会单独编译到诊断目录，链接参数沿用 `build/main/wc_bringup/CMakeFiles/mapping_rviz.dir/{flags.make,link.txt}`。

在目标已准备好的 ROS 环境执行；下面只展示本诊断命令，不负责连接目标或设备。两个输出目录都必须是新的，重复检查请换后缀。

```bash
cd /home/nvidia/wheelchair
python3 -s tests/integration/synthetic_manual_render.py build \
  --output-root reports/map_six_issues_20260914/synthetic_manual_build_02

python3 -s tests/integration/synthetic_manual_render.py run \
  --build-root reports/map_six_issues_20260914/synthetic_manual_build_02 \
  --session-root reports/maps/map_dashboard_left_20260914_04 \
  --output-root reports/map_six_issues_20260914/synthetic_manual_render_02
```

`run` 生成的流程复用现有 `check_dashboard_saved_render.py`：持有 domain 84 离线锁、使用 localhost、只读验原图、启动受管的地图发布和四路合成图、抓取本次窗口后正常关闭。生产 `scripts/map`、`mapping_wheel`、硬件驱动都不会启动。私有 socket 由进程内 `QLocalServer` 创建于新 `/tmp/wc_synthetic_manual_XXXXXX`，当前用户持有且权限为 0600，结束后临时目录删除。不会查找或连接真实 `manual.sock`。

模拟服务始终发送 `DISARMED`、`arm_allowed=false`、`arm_generation=0`；只接受一次匹配身份的 `hello`，收到 `arm`、`keys` 或其他消息即失败。面板的确认回调也固定拒绝。短状态持续至少 3 秒，然后每 100 ms 发送实际长度的中文禁用原因，持续至少 12 秒。中文取自当前 `mapping_wheel.py` 的 `DISABLED_REASON` 字面量，仅 AST 读取文字，不导入该模块；没有添加超长压力文本。构建记录和运行身份文件均保留文字来源。

首包的 socket `write/flush` 与面板 `readyRead` 更新标签是异步步骤。诊断仅在首状态确认前允许精确的 `WAITING_STATUS` 最多 1000 ms，并继续发送状态；实际看见本次 `DISARMED` 和短状态文本后才开始计 3 秒。按钮可激活、身份或其他状态异常、断开连接都不进入等待；确认后也绝不重新使用此等待。生产首状态 2 秒和后续 500 ms 期限均保持原样。`final.json` 记录首状态是否确认、确认耗时和等待次数。01 的首包时序失败证据保留，重跑使用独立 build02/run02。

每阶段记录 frame、central、手动 dock、各标签的 geometry/sizeHint/最小尺寸及可见性；另外记录原生渲染 `QWindow` 的 geometry、`isVisible`、`isExposed`。不会调用 `renderNow`、`windowMovedOrResized` 或修改相机状态以修正画面。

主要输出：

- 构建目录：`build.log`、`build.json`（源码、输入、命令和二进制 hash）、独立 `synthetic_manual_rviz`。
- 运行目录：已有离线验收的 `result.json`、`saved_dashboard.png`、地图审计和四路合成图状态。
- `manual_native/identity.json`：明确 synthetic 身份和临时 socket 来源。
- `manual_native/short_ogre.png`、`long_ogre.png`：直接调用公共 `getRenderWindow()->captureScreenShot(std::string)` 保存的 Ogre 画面。
- `manual_native/short_qt_screen.png`、`long_qt_screen.png`：仅本进程窗口的 `QScreen::grabWindow` 截图，用于和 Ogre 画面比较。
- `manual_native/final.json`：全部尺寸观察、客户端消息、状态计数、最大定时器间隔、截图耗时与最终结果。`phases_complete.json` 只是阶段就绪信号；最终关闭仍须检查 `final.json` 和外层 `result.json`。

`PASS` 仅表示两阶段完成、面板持续未激活、没有控制意图且窗口正常关闭。截图仍标记 `visual_review_status=PENDING`；必须目视比较原生 Ogre 与外窗图像，不能用进程退出码认定没有黑屏。若 Ogre 正常而外窗黑，才有进一步检查窗口系统呈现的依据；若二者都黑，仍需看显示内容、相机和坐标状态，不能立即归因于 Qt 文本。

截图与 PNG 保存是同步诊断操作，会暂停同线程的模拟状态定时器。若截图写盘耗时接近或超过面板原有 500 ms 状态期限，诊断可能因自身截图而失败；须结合 `last_capture_ms`、阶段的 `both_captures_ms` 和 `maximum_timer_gap_ms` 解释，不能把这种失败归因于 live 数据或长文本。没有放宽生产超时。窗口若未被外层正常关闭，35 秒定时检查会令诊断失败退出；外层还有原有受管组件退出期限。同步 OS 写盘阻塞本身不承诺被 Qt 定时器抢占。

本地可先执行纯构建计划测试，不需要 ROS、编译器或 GUI：

```bash
python3 -m unittest discover -s tests/integration -p test_synthetic_manual_render.py -v
```

实际 C++ 编译和原生截图须由主任务在目标执行；本地准备不代表目标构建或渲染通过。
