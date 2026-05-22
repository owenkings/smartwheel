# wheelchair_ui

FastAPI Web UI。

```bash
ros2 run wheelchair_ui wheelchair_ui --host 0.0.0.0 --port 8080
```

浏览器打开 `http://localhost:8080`。第一版支持查看状态、目标点列表、添加/删除目标点、发送目标点、软件急停，并在 SLAM 栅格地图上叠加墙线、房间边界、禁行区、POI 等语义矢量图层。
