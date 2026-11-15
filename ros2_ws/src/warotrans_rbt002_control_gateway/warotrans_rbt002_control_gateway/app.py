"""FastAPI app: WebSocket control + telemetry for the RBT-002 mobile UI.

The ROS side is behind the `Bridge` protocol so the app can be tested with a fake.

WebSocket /ws?token=...   (token required when WARO_UI_TOKEN is set)
  server -> client  {"type":"telemetry", ...}  10 Hz
                    {"type":"ack", "req": <type>, "ok": bool, "message": str}
  client -> server  {"type":"claim"} | {"type":"release"}
                    {"type":"drive", "x": -1..1, "y": -1..1}         ~20 Hz while the stick is held
                    {"type":"stop"}                                   anyone, always
                    {"type":"set_mode", "mode": "SAFE"|"ENABLED"|"CALIBRATION"}
                    {"type":"set_speed_limit", "scale": 0..1}
                    {"type":"calibrate", "motor": 1..4, "rps": float, "duration_ms": int}
                    {"type":"save_motor_map", "ports": {"FL":1,"FR":2,"RL":3,"RR":4}, "signs": [1,-1,1,-1]}
"""

from __future__ import annotations

import asyncio
import json
import logging
import math
import os
import secrets
import time
import uuid
from contextlib import asynccontextmanager
from typing import Any, Optional, Protocol

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import JSONResponse
from fastapi.staticfiles import StaticFiles

from .session import ControlSession

log = logging.getLogger("warotrans_rbt002_control_gateway")
MODES = {"SAFE": 0, "ENABLED": 1, "CALIBRATION": 2}


class Bridge(Protocol):
    def publish_twist(self, vx: float, wz: float) -> None: ...

    def set_mode(self, mode: int) -> tuple[bool, str]: ...

    def calibrate(self, motor: int, rps: float, duration_ms: int) -> tuple[bool, str]: ...

    def set_speed_limit(self, scale: float) -> tuple[bool, str]: ...

    def save_motor_map(self, ports: dict, signs: list) -> tuple[bool, str]: ...

    def snapshot(self) -> dict[str, Any]: ...

    def limits(self) -> tuple[float, float]: ...  # current (max_linear_mps, max_angular_radps)


def sanitize(obj: Any) -> Any:
    """JSON for browsers: NaN / inf are not valid JSON, send null instead."""
    if isinstance(obj, float):
        return obj if math.isfinite(obj) else None
    if isinstance(obj, dict):
        return {k: sanitize(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [sanitize(v) for v in obj]
    if hasattr(obj, "tolist"):  # numpy scalar / array from rclpy
        return sanitize(obj.tolist())
    return obj


def create_app(bridge: Bridge, web_dist: Optional[str] = None, token: Optional[str] = None,
               deadman_s: float = 0.3, cmd_rate_hz: float = 20.0, telemetry_rate_hz: float = 10.0,
               clock=time.monotonic) -> FastAPI:
    session = ControlSession(deadman_s=deadman_s)
    clients: dict[str, WebSocket] = {}
    tasks: list[asyncio.Task] = []

    async def cmd_loop() -> None:
        period = 1.0 / cmd_rate_hz
        while True:
            try:
                max_v, max_w = bridge.limits()
                vx, wz = session.twist(clock(), max_v, max_w)
                bridge.publish_twist(vx, wz)
            except Exception:
                log.exception("cmd loop")
            await asyncio.sleep(period)

    async def telemetry_loop() -> None:
        period = 1.0 / telemetry_rate_hz
        while True:
            await asyncio.sleep(period)
            if not clients:
                continue
            try:
                snap = bridge.snapshot()
            except Exception:
                log.exception("snapshot")
                continue
            now = clock()
            cur = session.current(now)
            for cid, ws in list(clients.items()):
                msg = dict(snap)
                msg["type"] = "telemetry"
                msg["gateway"] = {"client_id": cid, "has_control": session.has_control(cid, now),
                                  "controlled_by_other": session.holder not in (None, cid),
                                  "drive": cur.direction, "drive_magnitude": cur.magnitude,
                                  "auth": token is not None, "clients": len(clients)}
                try:
                    text = json.dumps(sanitize(msg))
                except (TypeError, ValueError):
                    log.exception("telemetry not serializable")
                    continue
                try:
                    await ws.send_text(text)
                except Exception:
                    clients.pop(cid, None)

    @asynccontextmanager
    async def lifespan(_app: FastAPI):
        tasks.append(asyncio.create_task(cmd_loop()))
        tasks.append(asyncio.create_task(telemetry_loop()))
        try:
            yield
        finally:
            session.stop()
            try:
                bridge.publish_twist(0.0, 0.0)
            except Exception:
                pass
            for t in tasks:
                t.cancel()

    app = FastAPI(title="WaroTrans RBT-002 control gateway", lifespan=lifespan)
    app.state.session = session

    @app.get("/api/health")
    async def health() -> JSONResponse:
        return JSONResponse({"ok": True, "auth": token is not None, "clients": len(clients)})

    async def call(fn, *args):
        return await asyncio.get_running_loop().run_in_executor(None, fn, *args)

    async def ack(ws: WebSocket, req: str, ok: bool, message: str) -> None:
        await ws.send_text(json.dumps({"type": "ack", "req": req, "ok": ok, "message": message}))

    @app.websocket("/ws")
    async def ws_endpoint(ws: WebSocket) -> None:
        if token is not None and not secrets.compare_digest(ws.query_params.get("token", ""), token):
            await ws.close(code=4401)
            return
        await ws.accept()
        cid = uuid.uuid4().hex[:8]
        clients[cid] = ws
        log.info("client %s connected (%d total)", cid, len(clients))
        try:
            while True:
                raw = await ws.receive_text()
                now = clock()
                try:
                    msg = json.loads(raw)
                    kind = str(msg.get("type", ""))
                except (ValueError, AttributeError):
                    await ack(ws, "?", False, "invalid JSON")
                    continue
                session.touch(cid, now)
                if kind == "stop":  # always allowed, from anyone
                    session.stop()
                    bridge.publish_twist(0.0, 0.0)
                    ok, m = await call(bridge.set_mode, MODES["SAFE"])
                    await ack(ws, kind, ok, m)
                elif kind == "claim":
                    ok, m = session.claim(cid, now)
                    await ack(ws, kind, ok, m)
                elif kind == "release":
                    session.release(cid)
                    await ack(ws, kind, True, "released")
                elif kind == "drive":
                    try:
                        x, y = float(msg.get("x", 0.0)), float(msg.get("y", 0.0))
                    except (TypeError, ValueError):
                        x = y = math.nan
                    ok, m, _ = session.drive(cid, x, y, now)
                    if not ok:
                        await ack(ws, kind, ok, m)
                elif kind == "set_mode":
                    mode = MODES.get(str(msg.get("mode", "")))
                    if mode is None:
                        await ack(ws, kind, False, "mode must be SAFE|ENABLED|CALIBRATION")
                    elif mode != MODES["SAFE"] and not session.has_control(cid, now):
                        await ack(ws, kind, False, "take control first")
                    else:
                        session.stop()
                        ok, m = await call(bridge.set_mode, mode)
                        await ack(ws, kind, ok, m)
                elif kind == "save_motor_map":
                    if not session.has_control(cid, now):
                        await ack(ws, kind, False, "take control first")
                        continue
                    ok, m = await call(bridge.save_motor_map, msg.get("ports") or {}, msg.get("signs") or [])
                    await ack(ws, kind, ok, m)
                elif kind in ("calibrate", "set_speed_limit"):
                    if not session.has_control(cid, now):
                        await ack(ws, kind, False, "take control first")
                        continue
                    try:
                        if kind == "calibrate":
                            ok, m = await call(bridge.calibrate, int(msg["motor"]), float(msg["rps"]),
                                               int(msg.get("duration_ms", 300)))
                        else:
                            ok, m = await call(bridge.set_speed_limit, float(msg["scale"]))
                    except (KeyError, TypeError, ValueError):
                        ok, m = False, "bad arguments"
                    await ack(ws, kind, ok, m)
                else:
                    await ack(ws, kind, False, "unknown message type")
        except WebSocketDisconnect:
            pass
        finally:
            clients.pop(cid, None)
            if session.release(cid):
                bridge.publish_twist(0.0, 0.0)
                log.info("client %s released control on disconnect", cid)
            log.info("client %s disconnected", cid)

    if web_dist and os.path.isdir(web_dist):
        # colcon --symlink-install installs the UI as symlinks into src/; older Starlette (0.27 on Ubuntu 24.04)
        # refuses files whose real path is outside `directory`, so serve the real directory.
        index = os.path.join(web_dist, "index.html")
        real = os.path.dirname(os.path.realpath(index)) if os.path.exists(index) else web_dist
        app.mount("/", StaticFiles(directory=real, html=True), name="web")
    return app
