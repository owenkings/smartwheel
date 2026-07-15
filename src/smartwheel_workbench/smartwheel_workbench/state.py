from dataclasses import dataclass


VALID_STATES = {
    "IDLE",
    "CHECKING",
    "RECORDING",
    "MAPPING",
    "LOOP_CLOSING",
    "OPTIMIZING",
    "EXPORTING",
    "QUALITY_CHECK",
    "READY",
    "FAILED",
}


@dataclass
class WorkbenchSession:
    state: str = "IDLE"
    failure_reason: str = ""
    paused: bool = False
    recording: bool = False
    map_saved: bool = False

    def transition(self, command: int) -> tuple[bool, str]:
        from smartwheel_interfaces.srv import WorkbenchCommand

        if command == WorkbenchCommand.Request.PAUSE_MAPPING:
            if self.state != "MAPPING" or self.paused:
                return False, f"command {command} is not valid in {self.state} paused={self.paused}"
            self.paused = True
            return True, "MAPPING paused"
        if command == WorkbenchCommand.Request.RESUME_MAPPING:
            if self.state != "MAPPING" or not self.paused:
                return False, f"command {command} is not valid in {self.state} paused={self.paused}"
            self.paused = False
            return True, "MAPPING resumed"

        transitions = {
            WorkbenchCommand.Request.PREFLIGHT: ({"IDLE", "FAILED"}, "CHECKING"),
            WorkbenchCommand.Request.START_RECORDING: ({"CHECKING"}, "RECORDING"),
            WorkbenchCommand.Request.START_MAPPING: ({"CHECKING", "RECORDING"}, "MAPPING"),
            WorkbenchCommand.Request.FINISH_MAPPING: ({"MAPPING"}, "LOOP_CLOSING"),
            WorkbenchCommand.Request.OPTIMIZE: ({"LOOP_CLOSING"}, "OPTIMIZING"),
            WorkbenchCommand.Request.SAVE_MAP: ({"OPTIMIZING"}, "EXPORTING"),
            WorkbenchCommand.Request.EXPORT_PRODUCTS: (
                {"OPTIMIZING", "EXPORTING", "READY"},
                "EXPORTING",
            ),
            WorkbenchCommand.Request.PREVIEW_SAVED_MAP: ({"READY"}, "READY"),
            WorkbenchCommand.Request.CANCEL: (
                {"CHECKING", "RECORDING", "MAPPING", "LOOP_CLOSING", "OPTIMIZING", "EXPORTING"},
                "IDLE",
            ),
            WorkbenchCommand.Request.RESET_SESSION: ({"IDLE", "FAILED", "READY"}, "IDLE"),
        }
        if command not in transitions:
            return False, f"unknown workbench command {command}"
        allowed, target = transitions[command]
        if self.state not in allowed:
            return False, f"command {command} is not valid in {self.state}"
        self.state = target
        self.paused = False
        if command == WorkbenchCommand.Request.START_RECORDING:
            self.recording = True
        if command in (WorkbenchCommand.Request.CANCEL, WorkbenchCommand.Request.RESET_SESSION):
            self.recording = False
            self.map_saved = False
            self.failure_reason = ""
        return True, target

    def fail(self, reason: str) -> None:
        if not reason.strip():
            raise ValueError("failure reason must not be empty")
        self.state = "FAILED"
        self.failure_reason = reason.strip()
        self.paused = False
