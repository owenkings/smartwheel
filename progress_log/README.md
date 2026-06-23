# Progress Log（项目进展记录）

本目录按**日期**记录每次对话/会话的进展,供其他对话或 AI 接手时快速了解项目当前状态、
已完成/未完成项、关键决策与坑。**每个对话/会话一个 markdown**(或按日期合并当天多次会话),
而非把所有内容堆进一个文件。

## 命名约定
- 单次会话:`YYYYMMDD_<主题简述>.md`(例:`20260623_fastlio_right_radar.md`)。
- 若同一天多次会话,可各自一个文件,或在当天文件内用 `## 会话 N` 分节。

## 每个记录应包含
1. **日期 / 主题**
2. **本次目标**
3. **做了什么**(关键改动:文件、配置、决策)
4. **结果 / 验证**(数据、测试、auto_test 目录链接)
5. **未完成 / 待办**(交接给下次)
6. **关键坑 / 注意事项**

## 自动化
- Hook `save-progress-log`(`.kiro/hooks/`):在每次 agent 执行结束(agentStop)时提示/生成当日进展条目,
  追加到 `progress_log/<当天日期>_session.md`。
- 也可手动新建文件按上面约定记录。

## 索引(最新在上)
- `20260623_narrow_fov_audit.md` — narrow-fov-mapping-audit spec 执行(Task 1–11,除 Task5 需硬件):
  D024/D025/D038/D039/D034 修复、右雷达 RViz 版式、回环后端 TF 对齐、65/65 测试。
- `20260623_fastlio_right_radar.md` — FAST-LIO 右雷达路径 + LASER_POINT_COV 加固 + 外参标定 + 文档收尾。
- (更早的 FAST-LIO 重构过程见 `auto_test/2026062*` 各 report.md 与 `.kiro/specs/fastlio-narrow-fov-mapping/`。)
