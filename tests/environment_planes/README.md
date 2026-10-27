# 环境平面与木板标定测试

用解析射线合成场景回归环境平面分割、木板内部区域、木板诊断和多姿态配准。这个方案已暂停，见[说明](../../docs/environment_plane_calibration.md)。

- [scene_fixture.py](scene_fixture.py)：合成场景。
- [test_environment_pipeline.py](test_environment_pipeline.py)、[test_small_board_planes.py](test_small_board_planes.py)、[test_small_board_registration.py](test_small_board_registration.py)、[test_small_board_pipeline.py](test_small_board_pipeline.py)、[test_small_board_diagnostic.py](test_small_board_diagnostic.py)、[test_board_interior_patch.py](test_board_interior_patch.py)、[test_board_patch_pipeline.py](test_board_patch_pipeline.py)

运行方法：`bash tests/run_target_tests.sh tests/environment_planes`。返回[上级目录](../README.md)。
