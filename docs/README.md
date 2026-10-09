# 项目说明

入门放 getting_started，使用步骤放 operations，模块设计放 architecture，接口契约放 reference，维护规则放 development。

## 内容与入口

- [adr](adr/README.md)：当前长期设计决策。
- [architecture](architecture/README.md)：模块职责、数据流与依赖。
- [calibration](calibration/README.md)：标定主题导航。
- [development](development/README.md)：新增内容、验证与维护规则。
- [getting_started](getting_started/README.md)：环境准备与首次运行。
- [operations](operations/README.md)：按功能组织的操作步骤。
- [reference](reference/README.md)：配置、接口与数据契约。

## 添加与验证

新增功能优先扩展现有模块；新增独立子目录时同时创建 README 并登记父级导航。运行状态、原始录包、临时任务和测试结果不得混入此目录。

修改后运行[对应功能测试](../tests/README.md)；涉及入口、配置、资源或路径时同时检查安装后的调用与旧数据读取。

依赖与构建方法见[项目入口](../README.md)；返回[上级目录](../README.md)。
