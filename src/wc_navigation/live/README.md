# 定位与实车影子链路

验证阶梯第 4 级（实车影子模式）需要的定位和实时链路：已存地图上的定位、把实时传感器转成 Nav2 输入、只记录不下发的影子启动器。本目录的程序都不打开设备、不连接轮控、不发布 `/cmd_vel`。

| 文件 | 职责 |
|---|---|
| [amcl_cross.py](amcl_cross.py) | 跨录包定位检查：验证录包的冻结扫描和里程计，在另一段建图录包的地图上跑 map_server + AMCL（隔离域 95，经 `amcl_replay`）。初值来自两段里程计坐标系的刚体对齐和刻意扰动；与固定对齐（纯里程计）比较扫描端点到地图的距离，统计协方差、收敛与失败、沿车头方向的歧义，以及闸门条件在真实 AMCL 输出上的表现。起点和终点在地图空白区（门内）的帧单独统计。 |

命令为 `bash scripts/navigation_amcl_replay cross ...`，用法见[实车影子操作说明](../../../docs/operations/navigation_live_shadow.md)，设计见[实车影子链路](../../../docs/architecture/live_shadow.md)。测试为 `tests/navigation/test_amcl_cross.py` 等 `test_live_*.py`、`test_amcl_*.py`。返回[离线导航开发](../README.md)。
