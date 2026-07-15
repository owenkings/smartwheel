import pytest

from smartwheel_interfaces.srv import WorkbenchCommand
from smartwheel_workbench.state import WorkbenchSession


def test_valid_mapping_lifecycle():
    session = WorkbenchSession()
    commands = (
        WorkbenchCommand.Request.PREFLIGHT,
        WorkbenchCommand.Request.START_RECORDING,
        WorkbenchCommand.Request.START_MAPPING,
        WorkbenchCommand.Request.PAUSE_MAPPING,
        WorkbenchCommand.Request.RESUME_MAPPING,
        WorkbenchCommand.Request.FINISH_MAPPING,
        WorkbenchCommand.Request.OPTIMIZE,
        WorkbenchCommand.Request.SAVE_MAP,
    )
    states = []
    for command in commands:
        accepted, _ = session.transition(command)
        assert accepted
        states.append(session.state)
    assert states == [
        "CHECKING",
        "RECORDING",
        "MAPPING",
        "MAPPING",
        "MAPPING",
        "LOOP_CLOSING",
        "OPTIMIZING",
        "EXPORTING",
    ]


def test_pause_is_a_mapping_flag_not_an_extra_protocol_state():
    session = WorkbenchSession(state="MAPPING")
    accepted, _ = session.transition(WorkbenchCommand.Request.PAUSE_MAPPING)
    assert accepted and session.state == "MAPPING" and session.paused
    accepted, _ = session.transition(WorkbenchCommand.Request.RESUME_MAPPING)
    assert accepted and session.state == "MAPPING" and not session.paused


def test_invalid_transition_is_rejected_without_mutation():
    session = WorkbenchSession()
    accepted, reason = session.transition(WorkbenchCommand.Request.START_MAPPING)
    assert not accepted
    assert session.state == "IDLE"
    assert "not valid" in reason


def test_failure_requires_specific_reason():
    session = WorkbenchSession()
    with pytest.raises(ValueError):
        session.fail(" ")
    session.fail("map service unavailable")
    assert session.state == "FAILED"
    assert session.failure_reason == "map service unavailable"
