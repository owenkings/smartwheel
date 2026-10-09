# 用户面板

窗口、用户参数、结果浏览及后台服务接口放这里；计算算法通过所属功能包调用。

## 内容与入口

- [__init__.py](__init__.py)
- [app.py](app.py)
- [backend.py](backend.py)
- [calibration_tools.py](calibration_tools.py)
- [catalog.py](catalog.py)
- [config_profiles.py](config_profiles.py)
- [installation_parameters.py](installation_parameters.py)
- [job_worker.py](job_worker.py)
- [jobs.py](jobs.py)
- [live_calibration.py](live_calibration.py)
- [offline_worker.py](offline_worker.py)
- [pairing_capture.py](pairing_capture.py)
- 其余同类文件遵守本目录的职责和命名规则。

## 添加与验证

新增功能优先扩展现有模块；新增独立子目录时同时创建 README 并登记父级导航。运行状态、原始录包、临时任务和测试结果不得混入此目录。

修改后运行[对应功能测试](../../tests/README.md)；涉及入口、配置、资源或路径时同时检查安装后的调用与旧数据读取。

依赖与构建方法见[项目入口](../../README.md)；返回[上级目录](../README.md)。

## 当前配置的本机覆盖

面板编辑器通过 `wc_runtime.project_config.selected_config_path` 选择完整配置。`hardware_setup.json`、`mapping_live.json`、`cameras.json` 优先读取和写回 `config/local/project/` 下的同名文件；不存在本机覆盖时，仍使用 `config/` 下的公共文件。存在损坏、非普通文件或链接形式的本机配置会直接报错，不回退到公共模板。真实设备首次使用前，应先创建完整的本机配置副本；面板不会自动初始化这些文件。

界面路径、修订身份、原始字节备份、保存和失败恢复均对应实际选中的文件。修订身份包含相对路径，文件字节相同而来源从 base 切到 local（或反向）也会使旧编辑授权失效。设备身份依赖也纳入修订检查。既有录包中的冻结配置继续按原路径读取。

命名配置集仍存放于 `config/panel_profiles/`。新配置集与切换事务使用 schema 2，记录 `source_paths`；每次写入和回滚均检查该绑定。旧 schema 1 只在当前仍使用完整 base 路径且依赖验证通过时兼容。出现本机覆盖、未知格式/状态、缺失哈希或外部修改时，保留原件、备份和事务记录并拒绝恢复，避免写入另一个来源。
