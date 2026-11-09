# 系统设计

维护当前模块职责、数据流和依赖；不混入阶段计划和验收流水。

## 内容与入口

- [algorithm_selection.md](algorithm_selection.md)
- [navigation-and-obstacles.md](navigation-and-obstacles.md)：定位、地图朝向、超声波与激光雷达的职责及后续导航接入条件。
- [autonomy_framework.md](autonomy_framework.md)：自主建图、路径规划、自动驾驶的分层结构、安全约束及仿真与实车的对应。
- [velocity_channel.md](velocity_channel.md)：导航到电机的速度通道：手动和自动的切换、各层速度上限、出事时怎么停、急停硬件和接通前的证据（设计，用户 2026-11-09 同意）。

## 添加与验证

新增功能优先扩展现有模块；新增独立子目录时同时创建 README 并登记父级导航。运行状态、原始录包、临时任务和测试结果不得混入此目录。

修改后运行[对应功能测试](../../tests/README.md)；涉及入口、配置、资源或路径时同时检查安装后的调用与旧数据读取。

依赖与构建方法见[项目入口](../../README.md)；返回[上级目录](../README.md)。
