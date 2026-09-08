from smartwheel_mapping_manager.finalization import (
    FinalizationAction,
    finalization_action,
    should_shutdown_on_terminal_state,
)
from smartwheel_mapping_manager.state_machine import MappingState


def test_ready_backend_requests_export():
    assert finalization_action(
        MappingState.OPTIMIZING,
        backend_ready=True,
        export_service_ready=True,
        timed_out=False,
    ) is FinalizationAction.REQUEST_EXPORT


def test_finalization_failures_are_specific():
    assert finalization_action(
        MappingState.OPTIMIZING,
        backend_ready=False,
        export_service_ready=True,
        timed_out=True,
    ) is FinalizationAction.FAIL_BACKEND
    assert finalization_action(
        MappingState.OPTIMIZING,
        backend_ready=True,
        export_service_ready=False,
        timed_out=True,
    ) is FinalizationAction.FAIL_EXPORT_SERVICE


def test_only_enabled_terminal_states_request_shutdown():
    assert should_shutdown_on_terminal_state(MappingState.READY, True)
    assert should_shutdown_on_terminal_state(MappingState.FAILED, True)
    assert not should_shutdown_on_terminal_state(MappingState.MAPPING, True)
    assert not should_shutdown_on_terminal_state(MappingState.READY, False)
