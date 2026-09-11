"""Fake controller only. Never instantiate a serial/real ROS driver."""
import time
from types import SimpleNamespace as NS
import pytest
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool
from wheelchair_base.kinematics import DifferentialDriveModel, OdometryState
from test_zlac_shutdown import make_node, FakeClock, FakeTime


def drive():
    n = make_node(single_slave_dual_axis=True)
    n.mode = 'real'
    n.allow_manual_push_mode = True
    n.mapping_drive_mode = 'drive'
    n.mapping_rearm_required = False
    n.feedback_healthy = True
    n.last_feedback_monotonic = time.monotonic()
    n.stationary_since = time.monotonic() - 1
    n.last_cmd = Twist()
    n.last_cmd_time = FakeTime(0)
    n.last_odom_time = FakeTime(0)
    n.get_clock = lambda: FakeClock(20_000_000)
    n.registers.control_word_register = 30
    return n


def switch(n, push=True):
    return n.set_mapping_push_mode(SetBool.Request(data=push), SetBool.Response())


def test_push_clears_targets_and_stops_without_enable():
    n = drive()
    assert switch(n).success
    assert n.mapping_drive_mode == 'push'
    assert n.modbus.writes == [(1, 10, 0), (1, 11, 0), (1, 30, 7)]
    assert not n._write_wheel_commands(20, 20)
    assert len(n.modbus.writes) == 3


@pytest.mark.parametrize('fault', ['disabled', 'readonly', 'fake', 'unhealthy', 'stale', 'moving', 'command', 'brief'])
def test_switch_preconditions_never_write(fault):
    n = drive()
    if fault == 'disabled': n.allow_manual_push_mode = False
    if fault == 'readonly': n.motion_control_enabled = False
    if fault == 'fake': n.mode = 'mock'
    if fault == 'unhealthy': n.feedback_healthy = False
    if fault == 'stale': n.last_feedback_monotonic -= 1
    if fault == 'moving': n.stationary_since = None
    if fault == 'brief': n.stationary_since = time.monotonic()
    if fault == 'command': n.last_cmd.linear.x = .1
    assert not switch(n).success
    assert n.modbus.writes == []


def test_failed_release_latches_blocked_no_drive_writes():
    n = drive()
    calls = []
    n._write_control_stop = lambda emergency: calls.append(emergency) or False
    assert not switch(n).success
    assert calls == [False, True]
    assert n.mapping_drive_mode == 'blocked'
    assert not n._write_wheel_commands(10, 10)
    assert n.modbus.writes == [(1, 10, 0), (1, 11, 0)]


def test_return_to_drive_needs_fresh_zero_then_new_command():
    n = drive()
    assert switch(n).success
    cmd = Twist(); cmd.linear.x = .1
    n.on_cmd_vel(cmd)
    assert n.last_cmd.linear.x == 0
    assert switch(n, False).success
    assert n.mapping_rearm_required
    assert not n._write_wheel_commands(10, 10)
    n.on_cmd_vel(cmd)
    assert n.last_cmd.linear.x == 0
    n.on_cmd_vel(Twist())
    assert not n.mapping_rearm_required
    assert n.last_cmd.linear.x == 0
    n.on_cmd_vel(cmd)
    assert n.last_cmd.linear.x == .1
    assert len(n.modbus.writes) == 3  # switching back never enables the servo


def test_push_tick_records_real_motion_without_any_command_writes():
    n = drive(); assert switch(n).success
    n.modbus.writes.clear()
    n.command_timeout_sec = .5
    n.registers.feedback_left_register = 100
    n.registers.feedback_right_register = 101
    n.model = DifferentialDriveModel(.165, .58, .25, .6)
    n.odom_state = OdometryState()
    n._read_feedback = lambda: (3., 3.)
    health = []; poses = []
    n._publish_feedback_health = lambda: health.append(n.feedback_healthy)
    n._publish_status = lambda age: None
    n._publish_odom = lambda v,w,stamp: poses.append((v,w))
    n.tick()
    assert health == [True]
    assert len(poses) == 1 and poses[0][0] > 0
    assert n.odom_state.x > 0
    assert n.modbus.writes == []
    assert n.stationary_since is None


def test_push_feedback_failure_still_blocks_fake_odometry():
    n = drive(); assert switch(n).success
    n.modbus.writes.clear()
    n.command_timeout_sec = .5
    n.registers.feedback_left_register = 100
    n.registers.feedback_right_register = 101
    n.model = DifferentialDriveModel(.165, .58, .25, .6)
    n._read_feedback = lambda: None
    health = []; poses = []
    n._publish_feedback_health = lambda: health.append(n.feedback_healthy)
    n._publish_status = lambda age: None
    n._publish_odom = lambda *args: poses.append(args)
    n.tick()
    assert health == [False] and poses == [] and n.modbus.writes == []
