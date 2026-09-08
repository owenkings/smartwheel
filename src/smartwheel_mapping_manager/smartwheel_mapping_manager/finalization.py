from enum import Enum

from smartwheel_mapping_manager.state_machine import MappingState


class FinalizationAction(str, Enum):
    WAIT = "WAIT"
    REQUEST_EXPORT = "REQUEST_EXPORT"
    FAIL_BACKEND = "FAIL_BACKEND"
    FAIL_EXPORT_SERVICE = "FAIL_EXPORT_SERVICE"


def finalization_action(
    state: MappingState,
    backend_ready: bool,
    export_service_ready: bool,
    timed_out: bool,
) -> FinalizationAction:
    if state is not MappingState.OPTIMIZING:
        return FinalizationAction.WAIT
    if not backend_ready:
        return FinalizationAction.FAIL_BACKEND if timed_out else FinalizationAction.WAIT
    if export_service_ready:
        return FinalizationAction.REQUEST_EXPORT
    return FinalizationAction.FAIL_EXPORT_SERVICE if timed_out else FinalizationAction.WAIT


def should_shutdown_on_terminal_state(state: MappingState, enabled: bool) -> bool:
    return enabled and state in (MappingState.READY, MappingState.FAILED)
