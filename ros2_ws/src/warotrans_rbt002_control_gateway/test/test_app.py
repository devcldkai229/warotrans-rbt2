import json
import time

import pytest

fastapi = pytest.importorskip("fastapi")
pytest.importorskip("httpx")
from fastapi.testclient import TestClient  # noqa: E402

from warotrans_rbt002_control_gateway.app import create_app, sanitize  # noqa: E402


class FakeBridge:
    def __init__(self):
        self.twists = []
        self.modes = []
        self.calls = []

    def publish_twist(self, vx, wz):
        self.twists.append((vx, wz))

    def set_mode(self, mode):
        self.modes.append(mode)
        return True, "ok"

    def calibrate(self, motor, rps, duration_ms):
        self.calls.append(("cal", motor, rps, duration_ms))
        return True, "ok"

    def set_speed_limit(self, scale):
        self.calls.append(("speed", scale))
        return True, "ok"

    def save_motor_map(self, ports, signs):
        self.calls.append(("map", dict(ports), list(signs)))
        return True, "saved"

    def snapshot(self):
        return {"safety": {"mode": "SAFE", "battery_voltage": float("nan")}}

    def limits(self):
        return 0.2, 0.8


def recv_ack(ws, req):
    for _ in range(50):
        msg = json.loads(ws.receive_text())
        if msg.get("type") == "ack" and msg.get("req") == req:
            return msg
    raise AssertionError("no ack")


def test_sanitize_nan():
    assert sanitize({"a": float("nan"), "b": [float("inf"), 1.0]}) == {"a": None, "b": [None, 1.0]}


def test_token_required():
    app = create_app(FakeBridge(), token="secret")
    with TestClient(app) as c:
        with pytest.raises(Exception):
            with c.websocket_connect("/ws?token=wrong") as ws:
                ws.receive_text()
        with c.websocket_connect("/ws?token=secret") as ws:
            ws.send_text(json.dumps({"type": "claim"}))
            assert recv_ack(ws, "claim")["ok"]


def test_drive_requires_lease_and_publishes_quantized_twist():
    b = FakeBridge()
    app = create_app(b, cmd_rate_hz=100.0)
    with TestClient(app) as c:
        with c.websocket_connect("/ws") as ws:
            ws.send_text(json.dumps({"type": "drive", "x": 0.0, "y": 1.0}))
            assert not recv_ack(ws, "drive")["ok"]
            ws.send_text(json.dumps({"type": "claim"}))
            recv_ack(ws, "claim")
            for _ in range(5):
                ws.send_text(json.dumps({"type": "drive", "x": 0.3, "y": 0.9}))
                time.sleep(0.02)
            time.sleep(0.05)
            assert (pytest.approx(0.2 * (0.9 - 0.15) / 0.85), 0.0) in [(v, w) for v, w in b.twists if v > 0] or \
                any(v > 0 and w == 0.0 for v, w in b.twists)
            assert all(v == 0.0 or w == 0.0 for v, w in b.twists)


def test_disconnect_stops_and_deadman():
    b = FakeBridge()
    app = create_app(b, cmd_rate_hz=100.0, deadman_s=0.1)
    with TestClient(app) as c:
        with c.websocket_connect("/ws") as ws:
            ws.send_text(json.dumps({"type": "claim"}))
            recv_ack(ws, "claim")
            ws.send_text(json.dumps({"type": "drive", "x": 0.0, "y": 1.0}))
            time.sleep(0.3)  # no refresh -> deadman
            b.twists.clear()
            time.sleep(0.05)
            assert b.twists and all(t == (0.0, 0.0) for t in b.twists)
        time.sleep(0.05)
        assert b.twists[-1] == (0.0, 0.0)


def test_stop_allowed_without_lease_and_sets_safe():
    b = FakeBridge()
    app = create_app(b)
    with TestClient(app) as c:
        with c.websocket_connect("/ws") as holder, c.websocket_connect("/ws") as other:
            holder.send_text(json.dumps({"type": "claim"}))
            recv_ack(holder, "claim")
            other.send_text(json.dumps({"type": "set_mode", "mode": "ENABLED"}))
            assert not recv_ack(other, "set_mode")["ok"]
            other.send_text(json.dumps({"type": "stop"}))
            assert recv_ack(other, "stop")["ok"]
            assert b.modes[-1] == 0


def test_calibrate_and_speed_limit_need_lease():
    b = FakeBridge()
    app = create_app(b)
    with TestClient(app) as c:
        with c.websocket_connect("/ws") as ws:
            ws.send_text(json.dumps({"type": "calibrate", "motor": 1, "rps": 0.1, "duration_ms": 300}))
            assert not recv_ack(ws, "calibrate")["ok"]
            ws.send_text(json.dumps({"type": "claim"}))
            recv_ack(ws, "claim")
            ws.send_text(json.dumps({"type": "calibrate", "motor": 1, "rps": 0.1, "duration_ms": 300}))
            assert recv_ack(ws, "calibrate")["ok"]
            ws.send_text(json.dumps({"type": "set_speed_limit", "scale": 0.4}))
            assert recv_ack(ws, "set_speed_limit")["ok"]
            ws.send_text(json.dumps({"type": "calibrate", "motor": "x"}))
            assert not recv_ack(ws, "calibrate")["ok"]
    assert ("cal", 1, 0.1, 300) in b.calls and ("speed", 0.4) in b.calls


def test_telemetry_is_valid_json_without_nan():
    app = create_app(FakeBridge(), telemetry_rate_hz=50.0)
    with TestClient(app) as c:
        with c.websocket_connect("/ws") as ws:
            for _ in range(20):
                raw = ws.receive_text()
                if '"telemetry"' in raw:
                    assert "NaN" not in raw
                    msg = json.loads(raw)
                    assert msg["safety"]["battery_voltage"] is None
                    assert msg["gateway"]["has_control"] is False
                    return
    raise AssertionError("no telemetry")


def test_save_motor_map_needs_lease():
    b = FakeBridge()
    app = create_app(b)
    msg = {"type": "save_motor_map", "ports": {"FL": 1, "FR": 2, "RL": 3, "RR": 4}, "signs": [1, -1, 1, -1]}
    with TestClient(app) as c:
        with c.websocket_connect("/ws") as ws:
            ws.send_text(json.dumps(msg))
            assert not recv_ack(ws, "save_motor_map")["ok"]
            ws.send_text(json.dumps({"type": "claim"}))
            recv_ack(ws, "claim")
            ws.send_text(json.dumps(msg))
            assert recv_ack(ws, "save_motor_map")["ok"]
    assert b.calls[-1] == ("map", {"FL": 1, "FR": 2, "RL": 3, "RR": 4}, [1, -1, 1, -1])


def test_static_ui_served_through_symlinks(tmp_path):
    real = tmp_path / "src_web"
    real.mkdir()
    (real / "index.html").write_text("<html>ok</html>")
    inst = tmp_path / "share_web"
    inst.mkdir()
    try:
        (inst / "index.html").symlink_to(real / "index.html")
    except OSError:
        pytest.skip("symlinks not permitted on this machine")
    app = create_app(FakeBridge(), web_dist=str(inst))
    with TestClient(app) as c:
        r = c.get("/")
        assert r.status_code == 200 and "ok" in r.text
