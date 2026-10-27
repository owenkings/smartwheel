# 现场外参标定测试

合成场景回归，覆盖：由墙面求竖直方向、找平旋转、固定倾角的左右 4 自由度联合配准、平面手眼求偏航、轮速静止段与距离积分、推墙一致性和离群停顿剔除。

- [test_field_geometry.py](test_field_geometry.py)

不读真实录包，不打开设备。被测实现见 [wc_calibration/field](../../src/wc_calibration/field/README.md)。运行方法：`bash tests/run_target_tests.sh tests/field_calibration`。返回[上级目录](../README.md)。
