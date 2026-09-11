"""Regressions for the patched FAST-LIO state updates; never starts ROS/hardware."""
from pathlib import Path
import shutil
import subprocess

import pytest

ROOT = Path(__file__).resolve().parents[3]
FAST = ROOT / 'src/third_party/FAST_LIO_ROS2'


@pytest.fixture
def source():
    return (FAST / 'src/laserMapping.cpp').read_text()


def test_zero_velocity_aid_cannot_write_pose_or_attitude(source):
    body = source.split('void apply_zupt_if_stationary', 1)[1].split('void apply_wheel_forward_speed', 1)[0]
    assert 'update_zero_velocity' in body
    assert 's.vel +=' in body
    assert '.boxplus(' not in body


def test_rejected_lidar_state_and_imu_time_are_coherent(source):
    assert 'restore_last_valid_lidar_state' not in source
    assert 'rollback_imu_timeline' not in source
    body = source.split('if (!lidar_update_accepted)', 1)[1].split('else\n            {', 1)[0]
    assert 'kf.change_x(state_before_update);' in body
    assert 'covariance_restore = covariance_before_update' in body


def test_final_guard_is_before_tf_cloud_and_map_insertion(source):
    guard = source.index('validate_post_aid<state_ikfom::DOF>')
    publish = source.index('publish_odometry(pubOdomAftMapped_', guard)
    assert guard < publish < source.index('map_incremental();', publish)
    assert 'kf.change_x(state_after_lidar);' in source[guard:publish]
    assert 'if (tf_publish_en)' in source
    assert 'odomAftMapped.twist.covariance' in source


def test_pure_cpp_covariance_guard_and_recorded_innovation(tmp_path):
    compiler = shutil.which('g++')
    if compiler is None or not Path('/usr/include/eigen3/Eigen/Core').exists():
        pytest.skip('C++/Eigen runtime unavailable; source-only tests are not math acceptance')
    binary = tmp_path / 'state_aiding_test'
    subprocess.run([compiler, '-std=c++17', '-O1', '-I/usr/include/eigen3',
                    '-I', str(FAST), str(FAST / 'tests/test_state_aiding_safety.cpp'),
                    '-o', str(binary)], check=True, timeout=180)
    subprocess.run([str(binary)], check=True, timeout=10)
