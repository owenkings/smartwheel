from enum import Enum


class MappingState(str, Enum):
    IDLE = "IDLE"
    CHECKING = "CHECKING"
    RECORDING = "RECORDING"
    MAPPING = "MAPPING"
    LOOP_CLOSING = "LOOP_CLOSING"
    OPTIMIZING = "OPTIMIZING"
    EXPORTING = "EXPORTING"
    QUALITY_CHECK = "QUALITY_CHECK"
    READY = "READY"
    FAILED = "FAILED"


ORDER = (
    MappingState.IDLE,
    MappingState.CHECKING,
    MappingState.RECORDING,
    MappingState.MAPPING,
    MappingState.LOOP_CLOSING,
    MappingState.OPTIMIZING,
    MappingState.EXPORTING,
    MappingState.QUALITY_CHECK,
    MappingState.READY,
)


class MappingStateMachine:
    def __init__(self) -> None:
        self.state = MappingState.IDLE
        self.failure_reason = ""

    @property
    def progress(self) -> float:
        if self.state is MappingState.FAILED:
            return 0.0
        return ORDER.index(self.state) / (len(ORDER) - 1)

    def advance(self) -> MappingState:
        if self.state in (MappingState.FAILED, MappingState.READY):
            raise RuntimeError(f"cannot advance terminal state {self.state.value}")
        self.state = ORDER[ORDER.index(self.state) + 1]
        return self.state

    def fail(self, reason: str) -> None:
        if not reason.strip():
            raise ValueError("failure reason must be specific")
        self.state = MappingState.FAILED
        self.failure_reason = reason.strip()

    def reset(self) -> None:
        self.state = MappingState.IDLE
        self.failure_reason = ""

