from wheelchair_voice_agent.model_api_stub import TextCommandModelStub


def test_charge_commands_map_to_named_charging_goal():
    model = TextCommandModelStub()

    for command in ("回去充电", "回充", "返回充电位"):
        intent = model.parse(command, ["充电点"])
        assert intent.intent == "navigate_to"
        assert intent.goal_name == "充电点"
        assert intent.confidence >= 0.75


def test_pose_queries_are_recognized():
    intent = TextCommandModelStub().parse("我现在在哪")

    assert intent.intent == "query_pose"
    assert intent.confidence >= 0.75
