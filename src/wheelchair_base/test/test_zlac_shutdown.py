from wheelchair_base.zlac8030_driver_node import Zlac8030DriverNode, ZlacRegisterMap


class FakeModbus:
    def __init__(self):
        self.writes = []

    def write_single_register(self, slave_id, register, value):
        self.writes.append((slave_id, register, value))

<<<<<<< HEAD
    def write_multiple_registers(self, slave_id, register, values):
        self.writes.append((slave_id, register, list(values)))


class FakeLogger:
    def __init__(self):
        self.messages = []

    def warning(self, message):
        self.messages.append(message)

=======
>>>>>>> 8a8e91d227314564f506195666f0b3386fa7353b

def make_node(single_slave_dual_axis=False):
    node = object.__new__(Zlac8030DriverNode)
    node.mode = "real"
    node.left_slave_id = 1
    node.right_slave_id = 2
    node.single_slave_dual_axis = single_slave_dual_axis
    node.rpm_to_register_scale = 1.0
<<<<<<< HEAD
    node.motion_control_enabled = True
    node.write_dual_axis_command_together = False
    node.initialize_motion_on_first_command = False
    node.motion_initialized = False
    node.warned_motion_disabled = False
=======
>>>>>>> 8a8e91d227314564f506195666f0b3386fa7353b
    node.registers = ZlacRegisterMap(
        command_left_register=10,
        command_right_register=11,
        enable_register=20,
        enable_value=1,
        disable_value=0,
    )
    node.modbus = FakeModbus()
    node.warned_no_registers = False
<<<<<<< HEAD
    node.get_logger = lambda: FakeLogger()
=======
>>>>>>> 8a8e91d227314564f506195666f0b3386fa7353b
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
<<<<<<< HEAD


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
=======
>>>>>>> 8a8e91d227314564f506195666f0b3386fa7353b
