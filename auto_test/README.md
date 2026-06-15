# SmartWheel 自动化可视化测试协议

用 **ubuntu-desktop-control MCP** 截取 RViz / 桌面运行界面，对 SmartWheel 的运行状态做
**可视化自动化测试**，并生成可直接用于修复的 Markdown 报告。

本协议是**通用流程**，不绑定具体测试内容（测什么由你每次发起时指定）。协议只约定
**怎么测（步骤）** 和 **报告怎么写**。

> 核心目标：报告必须**明确指出问题 + 初步根因**，下次能照报告直接定位修复，
> 而不是只说"失败了"。

---

## 1. 使用的工具

| 工具 | 用途 |
|---|---|
| `ubuntu-desktop-control` MCP（`take_screenshot`） | 截取 RViz / 桌面界面；返回值含分辨率/缩放，兼作环境检查 |
| `ubuntu-desktop-control` MCP（`get_display_diagnostics`） | 确认缩放/坐标系正常（`get_screen_info` 多屏下有 bug，勿用） |
| 终端命令（`ros2 topic hz/echo`、`ros2 node list`、`tf2_echo` 等） | 采集"数据层"证据，与截图交叉印证 |
| 其他按需命令（如 `rtabmap-info`、查日志） | 采集运行时状态文本 |

显示环境（SSH 远程 + NoMachine 桌面）：
- `DISPLAY=:1`，`XAUTHORITY=/run/user/1000/gdm/Xauthority`，NoMachine 重连后可能变。
- 截图全黑/失败多半是 DISPLAY/XAUTHORITY 变了，更新 `.kiro/settings/mcp.json` 的 env。
- **已知坑**：多屏下 `get_screen_info` 会因 monitor 元数据（如 `DP-1`）解析失败报错，**勿用**；
  改用 `take_screenshot` 返回值确认分辨率/缩放。

---

## 2. 目录结构与保存位置

**每次测试 = 一个独立的日期时间+主题文件夹**，里面装本次的报告和截图：

```
auto_test/
├── README.md                          # 本协议（通用流程）
├── templates/
│   └── report_template.md             # 报告模板（每次复制填充）
└── <YYYYMMDD_HHMMSS>_<主题>/           # 一次测试一个文件夹
    ├── report.md                      # 本次测试报告
    ├── 01_<步骤名>.png                 # 截图（两位序号开头，便于排序）
    ├── 02_<步骤名>.png
    └── ...
```

**命名约定（强制）**：
- 文件夹：`auto_test/<YYYYMMDD_HHMMSS>_<主题>/`，例如 `20260615_193700_manual_mapping`。
- 报告：该文件夹内固定叫 `report.md`。
- 截图：`NN_<步骤名>.png`（NN 两位序号）。
- 报告里引用截图用**同目录相对路径**：`./01_xxx.png`。

---

## 3. 通用测试步骤

每次测试都按这个顺序走，不论测什么对象：

1. **建文件夹**：用当前时间戳 + 本次主题建 `auto_test/<YYYYMMDD_HHMMSS>_<主题>/`。
2. **环境检查**：`take_screenshot`（确认分辨率/缩放）+ `get_display_diagnostics`，
   确认显示可用、被测栈在运行（`ros2 node list`）。
3. **运行**：确保被测目标处于待测状态（启动/已在跑）。
4. **截图**：对每个要看的界面状态 `take_screenshot`，存到本次文件夹，按 `NN_步骤名.png` 命名。
5. **采数据**：同步用终端命令采集对应的数据层证据（话题频率、TF、状态文本等）。
6. **分析**：把"截图 + 数据"对照"期望"，判定每步 PASS / FAIL / WARN。
7. **写报告**：复制 `templates/report_template.md` 到本次文件夹的 `report.md`，填充结果、
   截图引用、问题与根因。

> 步骤粒度就到"运行 / 截图 / 采数据 / 分析 / 写报告"这一级；**具体测哪些界面、看哪些话题，
> 由本次测试主题决定，不写死在协议里。**

---

## 4. 测试报告要求（关键）

报告**必须**对每个 FAIL/WARN 写清楚：

- **现象**：看到/测到什么（附截图 + 数据）。
- **期望**：本应是什么。
- **初步根因**：最可能的原因，**指到具体文件/参数/节点/硬件**，不是泛泛"有问题"。
- **建议修复**：下次可直接照做的具体动作（改哪个文件的哪个参数、查哪个话题）。
- **复现命令**：触发该现象的命令，便于复测。

报告结构见 `templates/report_template.md`。

---

## 5. 安全约束

- 可视化测试默认**只读 + 截图**，不通过 MCP 点击会引发轮椅运动的按钮。
- 若测试需要轮椅运动：离地/清场 + 物理急停在手，并在报告里把该步标注为高风险。
- MCP 的点击/输入类操作不自动批准（`.kiro/settings/mcp.json` 只自动批准截图/诊断类只读工具）。
