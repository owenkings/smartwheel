from wheelchair_base.zlac8030_driver_node import Zlac8030DriverNode, ZlacRegisterMap


class FakeModbus:
    def __init__(self):
        self.writes = []

    def write_single_register(self, slave_id, register, value):
        self.writes.append((slave_id, register, value))


def make_node(single_slave_dual_axis=False):
    node = object.__new__(Zlac8030DriverNode)
    node.mode = "real"
    node.left_slave_id = 1
    node.right_slave_id = 2
    node.single_slave_dual_axis = single_slave_dual_axis
    node.rpm_to_register_scale = 1.0
    node.registers = ZlacRegisterMap(
        command_left_register=10,
        command_right_register=11,
        enable_register=20,
        enable_value=1,
        disable_value=0,
    )
    node.modbus = FakeModbus()
    node.warned_no_registers = False
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
