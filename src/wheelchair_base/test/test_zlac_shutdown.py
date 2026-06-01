from wheelchair_base.zlac8030_driver_node import Zlac8030DriverNode, ZlacRegisterMap


class FakeModbus:
    def __init__(self):
        self.writes = []

    def write_single_register(self, slave_id, register, value):
        self.writes.append((slave_id, register, value))

    def write_multiple_registers(self, slave_id, register, values):
        self.writes.append((slave_id, register, list(values)))


class FakeLogger:
    def __init__(self):
        self.messages = []

    def warning(self, message):
        self.messages.append(message)


def make_node(single_slave_dual_axis=False):
    node = object.__new__(Zlac8030DriverNode)
    node.mode = "real"
    node.left_slave_id = 1
    node.right_slave_id = 2
    node.single_slave_dual_axis = single_slave_dual_axis
    node.rpm_to_register_scale = 1.0
    node.motion_control_enabled = True
    node.write_dual_axis_command_together = False
    node.initialize_motion_on_first_command = False
    node.hold_zero_before_motion_init = False
    node.release_motion_after_zero_sec = -1.0
    node.motion_initialized = False
    node.last_nonzero_command_monotonic = 0.0
    node.warned_motion_disabled = False
    node.warned_zero_before_motion_init = False
    node.registers = ZlacRegisterMap(
        command_left_register=10,
        command_right_register=11,
        enable_register=20,
        enable_value=1,
        disable_value=0,
    )
    node.modbus = FakeModbus()
    node.warned_no_registers = False
    node.get_logger = lambda: FakeLogger()
    return node


def test_shutdown_writes_zero_speed_and_disables_both_slaves():
    node = make_node()

    node._shutdown_hardware()

    assert node.modbus.writes == [
        (1, 10, 0),
        (2, 11, 0),
        (1, 20, 0),
        (2, 20, 0),
    ]


def test_shutdown_single_slave_disables_once():
    node = make_node(single_slave_dual_axis=True)

    node._shutdown_hardware()

    assert node.modbus.writes == [
        (1, 10, 0),
        (1, 11, 0),
        (1, 20, 0),
    ]


def test_motion_control_disabled_blocks_nonzero_commands():
    node = make_node(single_slave_dual_axis=True)
    node.motion_control_enabled = False

    assert node._write_wheel_commands(10.0, 10.0) is False
    assert node.modbus.writes == []


def test_single_slave_can_write_dual_axis_command_together():
    node = make_node(single_slave_dual_axis=True)
    node.write_dual_axis_command_together = True

    assert node._write_wheel_commands(10.0, -10.0) is True
    assert node.modbus.writes == [(1, 10, [10, -10])]


def test_zero_command_before_motion_init_does_not_drive_enable():
    node = make_node(single_slave_dual_axis=True)
    node.initialize_motion_on_first_command = True
    node.registers.control_word_register = 30

    assert node._write_wheel_commands(0.0, 0.0) is True

    assert node.motion_initialized is False
    assert node.modbus.writes == []


def test_zero_command_before_motion_init_can_hold_for_navigation_mode():
    node = make_node(single_slave_dual_axis=True)
    node.initialize_motion_on_first_command = True
    node.hold_zero_before_motion_init = True
    node.registers.control_mode_register = 28
    node.registers.async_mode_register = 29
    node.registers.control_word_register = 30

    assert node._write_wheel_commands(0.0, 0.0) is True

    assert node.motion_initialized is True
    assert node.modbus.writes == [
        (1, 28, 3),
        (1, 29, 0),
        (1, 30, 6),
        (1, 30, 8),
        (1, 10, 0),
        (1, 11, 0),
    ]


def test_nonzero_command_initializes_motion_then_writes_speed():
    node = make_node(single_slave_dual_axis=True)
    node.initialize_motion_on_first_command = True
    node.registers.control_mode_register = 28
    node.registers.async_mode_register = 29
    node.registers.control_word_register = 30

    assert node._write_wheel_commands(10.0, -10.0) is True

    assert node.motion_initialized is True
    assert node.modbus.writes == [
        (1, 28, 3),
        (1, 29, 0),
        (1, 30, 6),
        (1, 30, 8),
        (1, 10, 10),
        (1, 11, -10),
    ]


def test_zero_command_after_idle_releases_motion_enable():
    node = make_node(single_slave_dual_axis=True)
    node.motion_initialized = True
    node.release_motion_after_zero_sec = 0.0
    node.registers.control_word_register = 30

    assert node._write_wheel_commands(0.0, 0.0) is True

    assert node.motion_initialized is False
    assert node.modbus.writes == [(1, 30, 7)]


def test_zero_command_holds_servo_when_release_disabled():
    # release disabled (-1) => autonomous stop must HOLD the wheels (抱死), i.e.
    # keep the servo ENABLED at zero speed, never send a stop/disable control word.
    node = make_node(single_slave_dual_axis=True)
    node.motion_initialized = True
    node.release_motion_after_zero_sec = -1.0
    node.registers.control_word_register = 30

    assert node._write_wheel_commands(0.0, 0.0) is True

    assert node.motion_initialized is True
    assert node.modbus.writes == [(1, 10, 0), (1, 11, 0)]
