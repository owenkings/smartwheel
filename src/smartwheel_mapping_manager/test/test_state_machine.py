import pytest

from smartwheel_mapping_manager.state_machine import ORDER, MappingState, MappingStateMachine


def test_state_machine_uses_required_order():
    machine = MappingStateMachine()
    visited = [machine.state]
    while machine.state is not MappingState.READY:
        visited.append(machine.advance())
    assert tuple(visited) == ORDER
    assert machine.progress == 1.0


def test_failure_records_specific_reason_and_is_terminal():
    machine = MappingStateMachine()
    machine.advance()
    machine.fail("LiDAR timestamp regressed")
    assert machine.state is MappingState.FAILED
    assert machine.failure_reason == "LiDAR timestamp regressed"
    with pytest.raises(RuntimeError):
        machine.advance()
