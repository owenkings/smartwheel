from dataclasses import dataclass
from typing import Dict, List


@dataclass
class ModelIntent:
    intent: str
    confidence: float
    goal_name: str | None = None

    def to_dict(self) -> Dict:
        data = {"intent": self.intent, "confidence": self.confidence}
        if self.goal_name is not None:
            data["goal_name"] = self.goal_name
        return data


class TextCommandModelStub:
    """Replace this class with a local LLM/VLM or cloud API later.

    The model output is only an intent. It must not publish velocity commands or
    write /cmd_vel_safe. The goal manager and safety supervisor remain in charge.
    """

    def parse(self, text: str, known_goal_names: List[str] | None = None) -> ModelIntent:
        command = text.strip()
        known_goal_names = known_goal_names or []
        if command in ("停止", "停下", "急停", "暂停"):
            return ModelIntent(intent="stop", confidence=0.98)
        if command in ("继续", "恢复", "接着走"):
            return ModelIntent(intent="continue", confidence=0.95)
        if command in ("当前位置", "我在哪", "我现在在哪", "现在位置", "这是哪里"):
            return ModelIntent(intent="query_pose", confidence=0.9)
        if command in (
            "回去充电",
            "回充",
            "回充电位",
            "返回充电位",
            "回到充电位",
            "去充电",
        ):
            return ModelIntent(
                intent="navigate_to", goal_name="充电点", confidence=0.97
            )

        prefixes = ("去", "到", "导航到", "带我去")
        for prefix in prefixes:
            if command.startswith(prefix) and len(command) > len(prefix):
                goal_name = command[len(prefix) :].strip()
                confidence = 0.93 if goal_name in known_goal_names or not known_goal_names else 0.78
                return ModelIntent(
                    intent="navigate_to",
                    goal_name=goal_name,
                    confidence=confidence,
                )

        for goal_name in known_goal_names:
            if goal_name and goal_name in command:
                return ModelIntent(
                    intent="navigate_to", goal_name=goal_name, confidence=0.75
                )
        return ModelIntent(intent="unknown", confidence=0.2)


class ImageRecognitionStub:
    """Reserved visual perception interface.

    Future pedestrian, door-state, and object recognition results should be
    published to perception/semantic topics. They must not bypass the safety
    supervisor or directly control /cmd_vel_safe.
    """

    def recognize_front_image(self, _image) -> Dict:
        return {
            "objects": [],
            "door_state": "unknown",
            "confidence": 0.0,
            "note": "image recognition model is not connected in version 0.1",
        }
