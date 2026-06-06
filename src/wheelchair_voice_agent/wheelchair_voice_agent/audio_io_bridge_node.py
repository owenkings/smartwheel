import json

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
except ImportError:
    rclpy = None
    Node = object
    String = None


def make_audio_status(
    microphone_device: str,
    speaker_device: str,
    microphone_enabled: bool,
    speaker_enabled: bool,
    stt_backend: str,
    tts_backend: str,
) -> dict:
    stt_ready = microphone_enabled and stt_backend.lower() != "none"
    tts_ready = speaker_enabled and tts_backend.lower() != "none"
    return {
        "microphone": {
            "device": microphone_device,
            "enabled": microphone_enabled,
            "backend": stt_backend,
            "ready": stt_ready,
        },
        "speaker": {
            "device": speaker_device,
            "enabled": speaker_enabled,
            "backend": tts_backend,
            "ready": tts_ready,
        },
        "mode": "ready" if stt_ready or tts_ready else "reserved",
    }


class AudioIoBridgeNode(Node):
    """Hardware-neutral microphone/speaker integration contract.

    Actual ALSA/PulseAudio capture, STT, and TTS backends can be attached later.
    This node only reports configuration and forwards trusted STT transcripts to
    the existing text-command parser. It never publishes velocity commands.
    """

    def __init__(self):
        super().__init__("audio_io_bridge_node")
        p = self.declare_parameter
        p("microphone_device", "default")
        p("speaker_device", "default")
        p("microphone_enabled", False)
        p("speaker_enabled", False)
        p("stt_backend", "none")
        p("tts_backend", "none")
        p("transcript_input_topic", "/voice/audio_transcript")
        p("text_command_topic", "/voice/text_command")
        p("tts_request_topic", "/voice/tts_text")
        p("status_topic", "/voice/audio_status")
        p("publish_rate_hz", 1.0)

        self.microphone_enabled = bool(self.get_parameter("microphone_enabled").value)
        self.stt_backend = str(self.get_parameter("stt_backend").value)
        self.text_pub = self.create_publisher(
            String, str(self.get_parameter("text_command_topic").value), 10
        )
        self.status_pub = self.create_publisher(
            String, str(self.get_parameter("status_topic").value), 10
        )
        self.create_subscription(
            String,
            str(self.get_parameter("transcript_input_topic").value),
            self.on_transcript,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("tts_request_topic").value),
            self.on_tts_request,
            10,
        )
        self.last_tts_request = ""
        rate = max(0.2, float(self.get_parameter("publish_rate_hz").value))
        self.create_timer(1.0 / rate, self.publish_status)

    def on_transcript(self, msg):
        if not self.microphone_enabled or self.stt_backend.lower() == "none":
            self.get_logger().warning("ignoring audio transcript: microphone/STT is reserved but disabled")
            return
        self.text_pub.publish(String(data=msg.data))

    def on_tts_request(self, msg):
        self.last_tts_request = msg.data

    def publish_status(self):
        status = make_audio_status(
            str(self.get_parameter("microphone_device").value),
            str(self.get_parameter("speaker_device").value),
            self.microphone_enabled,
            bool(self.get_parameter("speaker_enabled").value),
            self.stt_backend,
            str(self.get_parameter("tts_backend").value),
        )
        status["last_tts_request_pending"] = bool(self.last_tts_request)
        self.status_pub.publish(String(data=json.dumps(status, ensure_ascii=False)))


def main(args=None):
    if rclpy is None:
        raise RuntimeError("ROS2 Python packages are required to run this node")
    rclpy.init(args=args)
    node = AudioIoBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
