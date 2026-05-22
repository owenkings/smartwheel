# wheelchair_voice_agent

语音/文本命令和模型接口占位包。

```bash
ros2 run wheelchair_voice_agent command_parser_node
ros2 topic pub --once /voice/text_command std_msgs/msg/String "{data: '去卫生间'}"
```

输出 `/voice/intent`。模型只输出结构化意图，不能直接控制速度。
