from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from wheelchair_voice_agent.audio_io_bridge_node import make_audio_status  # noqa: E402


def test_audio_devices_are_reserved_when_backends_are_disabled():
    status = make_audio_status("hw:2,0", "default", False, False, "none", "none")

    assert status["mode"] == "reserved"
    assert status["microphone"]["device"] == "hw:2,0"
    assert status["microphone"]["ready"] is False
    assert status["speaker"]["ready"] is False


def test_audio_backend_reports_ready_only_when_enabled():
    status = make_audio_status("default", "default", True, True, "whisper", "piper")

    assert status["mode"] == "ready"
    assert status["microphone"]["ready"] is True
    assert status["speaker"]["ready"] is True
