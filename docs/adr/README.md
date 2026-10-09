# 当前设计决策

本目录记录仍然约束实现的工程决策及其理由，供修改运行链路、数据契约和部署边界时查阅。

- [轮反馈与惯性里程计](0001-wheel-imu-mapping-odometry.md)：里程计输入、估计和消息边界。
- [地图归档关闭同步](0002-mapping-archive-close-sync.md)：停止、关闭、保存与完整性边界。

新增影响多个模块的长期设计决策放在此处，沿用顺序编号，并列出受影响的接口、兼容规则与验证依据。模块结构说明放入[系统设计](../architecture/README.md)，用户步骤放入[操作手册](../operations/README.md)，临时计划和阶段报告进入数据根开发归档。

这里没有可执行入口，依赖相应源码模块与[接口契约](../reference/README.md)。修改决策后核对受影响模块的实现，并运行[运行管理](../../tests/runtime/README.md)、[操作入口](../../tests/operations/README.md)或对应功能测试。返回[文档导航](../README.md)。
