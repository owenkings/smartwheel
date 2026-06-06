# wheelchair_voice_agent

语音/文本命令和模型接口占位包。

```bash
ros2 run wheelchair_voice_agent command_parser_node
ros2 topic pub --once /voice/text_command std_msgs/msg/String "{data: '去卫生间'}"
```

输出 `/voice/intent`。模型只输出结构化意图，不能直接控制速度。
支持“去客厅/回去充电/停下/继续/我现在在哪”等基础意图。导航结果和
当前位置回答通过 `/voice/tts_text` 输出，等待后续扬声器/TTS 后端接入。

`voice_agent.launch.py` 同时启动麦克风/扬声器接口占位节点。默认配置不打开
音频设备，仅发布 `/voice/audio_status`；后续 STT 可向
`/voice/audio_transcript` 发布文本，TTS 从 `/voice/tts_text` 接收请求。
