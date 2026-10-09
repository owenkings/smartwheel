# 标定与点云配对

几何求解、配对输入准备、交互页面及坐标校验放这里；原始测量依据放 config/calibration。

## 内容与入口

- [picker_assets](picker_assets/README.md)：下一级职责说明。
- [__init__.py](__init__.py)
- [__main__.py](__main__.py)
- [alignment.py](alignment.py)
- [alignment_workspace.py](alignment_workspace.py)
- [assessment.py](assessment.py)
- [cli.py](cli.py)
- [core.py](core.py)
- [exploratory.py](exploratory.py)
- [gravity_level.py](gravity_level.py)
- [importers.py](importers.py)
- [level_reference.py](level_reference.py)
- [organized_input.py](organized_input.py)
- 其余同类文件遵守本目录的职责和命名规则。

## 添加与验证

新增功能优先扩展现有模块；新增独立子目录时同时创建 README 并登记父级导航。运行状态、原始录包、临时任务和测试结果不得混入此目录。

修改后运行[对应功能测试](../../tests/README.md)；涉及入口、配置、资源或路径时同时检查安装后的调用与旧数据读取。

依赖与构建方法见[项目入口](../../README.md)；返回[上级目录](../README.md)。
