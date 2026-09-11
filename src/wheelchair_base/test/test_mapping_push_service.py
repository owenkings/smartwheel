"""Exercise the real ROS service/timer wiring using a fake Modbus transport.

Localhost domain 93 only. No serial port is opened and no hardware is started.
"""
import time
import rclpy
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_srvs.srv import SetBool
import wheelchair_base.zlac8030_driver_node as driver


def test_ros_push_drive_switch_keeps_measured_odometry(monkeypatch):
    monkeypatch.setenv('ROS_DOMAIN_ID', '93')
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    class Transport:
        def __init__(self, config): self.writes = []; self.speed = 0
        def write_single_register(self, slave, register, value): self.writes.append((register, value))
        def read_holding_registers(self, slave, register, count): return [self.speed]
        def close(self): pass
    monkeypatch.setattr(driver, 'ModbusRtuClient', Transport)
    rclpy.init(args=['--ros-args',
        '-p', 'allow_manual_push_mode:=true', '-p', 'motion_control_enabled:=true',
        '-p', 'single_slave_dual_axis:=true', '-p', 'invert_right:=false',
        '-p', 'command_left_register:=10', '-p', 'command_right_register:=11',
        '-p', 'feedback_left_register:=100', '-p', 'feedback_right_register:=101',
        '-p', 'control_mode_register:=28', '-p', 'async_mode_register:=29',
        '-p', 'control_word_register:=30', '-p', 'release_motion_after_zero_sec:=-1.0'])
    n = driver.Zlac8030DriverNode()
    observer = rclpy.create_node('push_mode_fake_controller_test', use_global_arguments=False)
    executor = SingleThreadedExecutor(); executor.add_node(n); executor.add_node(observer)
    poses = []
    sub = observer.create_subscription(Odometry, '/wheel/odom', poses.append, 20)
    commands = observer.create_publisher(Twist, '/cmd_vel_safe', 10)
    client = observer.create_client(SetBool, '/base/set_mapping_push_mode')
    def spin(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end: executor.spin_once(timeout_sec=.01)
    def switch(push):
        future = client.call_async(SetBool.Request(data=push))
        end = time.monotonic() + 3
        while not future.done() and time.monotonic() < end: executor.spin_once(timeout_sec=.01)
        assert future.done()
        return future.result()
    try:
        spin(.8)
        assert n.feedback_healthy and poses
        assert switch(True).success
        assert n.modbus.writes == [(10, 0), (11, 0), (30, 7)]
        n.modbus.writes.clear(); n.modbus.speed = 3
        cmd = Twist(); cmd.linear.x = .1; commands.publish(cmd)
        spin(.3)
        assert n.modbus.writes == []
        assert poses[-1].twist.twist.linear.x > 0
        assert not switch(False).success  # cannot re-enable while hand-pushing
        n.modbus.speed = 0; commands.publish(Twist()); spin(.4)
        assert switch(False).success
        commands.publish(cmd); spin(.1)
        assert n.modbus.writes == []  # stale/held direction before fresh zero
        commands.publish(Twist()); spin(.1)
        commands.publish(cmd); spin(.15)
        assert (30, 8) in n.modbus.writes  # fake enable, followed by fake target
        assert any(reg == 10 and value != 0 for reg, value in n.modbus.writes)
    finally:
        executor.remove_node(n); executor.remove_node(observer)
        n.destroy_node(); observer.destroy_node(); executor.shutdown(); rclpy.shutdown()
