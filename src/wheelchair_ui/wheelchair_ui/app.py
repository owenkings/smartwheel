import argparse
from pathlib import Path

from fastapi import FastAPI, HTTPException
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel
import uvicorn

from wheelchair_navigation.semantic_map_store import default_semantic_map_path
from wheelchair_ui.mapping_manager import MappingManager
from wheelchair_ui.ros_bridge import RosBridge, default_named_goals_path


class GoalPayload(BaseModel):
    name: str
    x: float
    y: float
    yaw: float = 0.0
    frame_id: str = "map"
    label: str | None = None


class RoomPayload(BaseModel):
    name: str
    polygon: list[list[float]]
    color: str = "#6aa6ff"


class ZonePayload(BaseModel):
    name: str
    polygon: list[list[float]]


class MappingStartPayload(BaseModel):
    map_name: str | None = None
    force: bool = False
    backend: str | None = None  # "rtabmap" (3D, default) | "slam_toolbox" (2D fallback)


class MappingFinishPayload(BaseModel):
    map_name: str | None = None


class MappingVersionPayload(BaseModel):
    version_id: str


def create_app(named_goals_path: str, semantic_map_path: str | None = None) -> FastAPI:
    app = FastAPI(title="Wheelchair Control UI", version="0.1.0")
    bridge = RosBridge(named_goals_path, semantic_map_path)
    mapping = MappingManager()
    static_dir = Path(__file__).resolve().parent / "static"
    app.mount("/static", StaticFiles(directory=str(static_dir)), name="static")

    @app.on_event("shutdown")
    def shutdown_bridge():
        mapping.shutdown()
        bridge.shutdown()

    @app.get("/")
    def index():
        return FileResponse(static_dir / "index.html")

    @app.get("/api/status")
    def status():
        return bridge.node.status()

    @app.get("/api/map")
    def map_snapshot():
        return bridge.node.map_snapshot() or mapping.map_snapshot() or {
            "frame_id": "map",
            "width": 80,
            "height": 60,
            "resolution": 0.05,
            "origin": {"x": -2.0, "y": -1.5},
            "data": [-1] * (80 * 60),
        }

    @app.get("/api/mapping/status")
    def mapping_status():
        return mapping.status(bridge.node.status())

    @app.get("/api/mapping/preflight")
    def mapping_preflight():
        return mapping.preflight(bridge.node.status())

    @app.post("/api/mapping/start")
    def mapping_start(payload: MappingStartPayload):
        return mapping.start(bridge.node.status(), payload.map_name, payload.force, payload.backend)

    @app.post("/api/mapping/finish")
    def mapping_finish(payload: MappingFinishPayload):
        return mapping.finish(payload.map_name)

    @app.post("/api/mapping/cancel")
    def mapping_cancel():
        return mapping.cancel()

    @app.post("/api/mapping/version/activate")
    def mapping_activate_version(payload: MappingVersionPayload):
        return mapping.activate_version(payload.version_id)

    @app.get("/api/goals")
    def goals():
        return bridge.node.list_goals()

    @app.post("/api/goals")
    def add_goal(payload: GoalPayload):
        return bridge.node.add_goal(payload.dict())

    @app.delete("/api/goals/{name}")
    def delete_goal(name: str):
        if not bridge.node.delete_goal(name):
            raise HTTPException(status_code=404, detail="goal not found")
        return {"ok": True}

    @app.post("/api/navigate/{name}")
    def navigate(name: str):
        if not bridge.node.send_named_goal(name):
            detail = bridge.node.last_goal_error or "goal not found"
            status_code = 404 if "不存在" in detail or "not found" in detail else 400
            raise HTTPException(status_code=status_code, detail=detail)
        return {"ok": True}

    @app.post("/api/stop")
    def stop():
        bridge.node.set_software_stop(True)
        return {"ok": True}

    @app.post("/api/resume")
    def resume():
        bridge.node.set_software_stop(False)
        return {"ok": True}

    @app.post("/api/hardware/zero")
    def zero_velocity():
        bridge.node.publish_zero_velocity()
        return {"ok": True}

    @app.post("/api/hardware/shutdown")
    def hardware_shutdown():
        return bridge.node.request_hardware_shutdown()

    @app.get("/api/semantic-map")
    def semantic_map():
        return bridge.node.semantic_map()

    @app.put("/api/semantic-map")
    def save_semantic_map(payload: dict):
        return bridge.node.save_semantic_map(payload)

    @app.post("/api/rooms")
    def upsert_room(payload: RoomPayload):
        return bridge.node.upsert_room(payload.dict())

    @app.delete("/api/rooms/{name}")
    def delete_room(name: str):
        if not bridge.node.delete_room(name):
            raise HTTPException(status_code=404, detail="room not found")
        return {"ok": True}

    @app.post("/api/no-go-zones")
    def upsert_no_go_zone(payload: ZonePayload):
        return bridge.node.upsert_no_go_zone(payload.dict())

    @app.delete("/api/no-go-zones/{name}")
    def delete_no_go_zone(name: str):
        if not bridge.node.delete_no_go_zone(name):
            raise HTTPException(status_code=404, detail="zone not found")
        return {"ok": True}

    return app


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", default=8080, type=int)
    parser.add_argument("--named-goals-path", default=default_named_goals_path())
    parser.add_argument("--semantic-map-path", default=default_semantic_map_path())
    args, _ = parser.parse_known_args()
    uvicorn.run(
        create_app(args.named_goals_path, args.semantic_map_path),
        host=args.host,
        port=args.port,
    )


if __name__ == "__main__":
    main()
