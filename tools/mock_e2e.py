#!/usr/bin/env python3
"""End-to-end check of the whole stack against the MOCK board, through the real WebSocket gateway.

Start the stack first, e.g.:
    ros2 launch warotrans_rbt002_bringup bringup.launch.py mock:=true motors_enabled:=true mock_profile:=patched
Then:
    python3 tools/mock_e2e.py --expect-profile patched

Never point this at the real robot: it drives the motors. It refuses unless the link reports mock://.
"""

from __future__ import annotations

import argparse
import asyncio
import json
import sys
import time

import websockets

failures: list[str] = []


def check(cond: bool, what: str) -> None:
    print(("PASS " if cond else "FAIL ") + what)
    if not cond:
        failures.append(what)


class Client:
    def __init__(self, ws):
        self.ws = ws
        self.telemetry: dict = {}
        self.acks: list[dict] = []
        self._task = asyncio.create_task(self._reader())

    async def _reader(self):
        try:
            async for raw in self.ws:
                m = json.loads(raw)
                if m.get("type") == "telemetry":
                    self.telemetry = m
                elif m.get("type") == "ack":
                    self.acks.append(m)
        except websockets.ConnectionClosed:
            pass

    async def send(self, **msg):
        await self.ws.send(json.dumps(msg))

    async def request(self, timeout=2.0, **msg) -> dict:
        n = len(self.acks)
        await self.send(**msg)
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            for a in self.acks[n:]:
                if a["req"] == msg["type"]:
                    return a
            await asyncio.sleep(0.02)
        return {"ok": False, "message": "no ack"}

    async def wait_for(self, pred, timeout=3.0) -> bool:
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            try:
                if pred(self.telemetry):
                    return True
            except (KeyError, TypeError, IndexError):
                pass
            await asyncio.sleep(0.05)
        return False

    async def drive(self, x, y, seconds):
        t0 = time.monotonic()
        while time.monotonic() - t0 < seconds:
            await self.send(type="drive", x=x, y=y)
            await asyncio.sleep(0.05)


def sp(t):
    return t["motors"]["setpoint_rps"]


async def main(url: str, profile: str) -> int:
    async with websockets.connect(url) as ws:
        c = Client(ws)
        check(await c.wait_for(lambda t: t["link"]["link_ok"], 5.0), "link_ok via telemetry")
        port = c.telemetry.get("link", {}).get("port", "")
        if not port.startswith("mock://"):
            print(f"REFUSING: link port is {port!r}, not the mock board")
            return 2
        check(await c.wait_for(lambda t: t["link"]["firmware_profile"] == profile, 4.0), f"firmware profile {profile}")
        check(c.telemetry["safety"]["mode"] == "SAFE", "starts in SAFE")

        a = await c.request(type="set_mode", mode="ENABLED")
        check(not a["ok"] and "control" in a["message"], "ENABLE refused without control lease")
        check((await c.request(type="claim"))["ok"], "claim lease")
        check((await c.request(type="set_mode", mode="ENABLED"))["ok"], "ENABLE accepted")

        await c.drive(0.0, 1.0, 1.0)
        t = c.telemetry
        s = sp(t)
        check(s[0] > 0 and s[1] < 0 and abs(s[0] + s[1]) < 1e-6, f"forward: mock map signs, setpoints {s}")
        check(abs(t["safety"]["cmd_angular_radps"]) < 1e-6, "forward has no rotation")
        check(t["link"]["last_sent_rps"][0] > 0, "setpoints reach the board")

        await c.drive(0.4, 0.9, 0.6)  # diagonal stick -> must quantise to pure forward
        check(abs(c.telemetry["safety"]["cmd_angular_radps"]) < 1e-6, "diagonal stick quantised to forward only")

        await c.drive(-1.0, 0.0, 1.0)
        s = sp(c.telemetry)
        check(c.telemetry["safety"]["cmd_angular_radps"] > 0 and abs(c.telemetry["safety"]["cmd_linear_mps"]) < 1e-3,
              f"rotate left: +wz, no linear ({s})")

        # deadman: stop sending
        t_stop = time.monotonic()
        ok = await c.wait_for(lambda t: all(abs(v) < 1e-6 for v in sp(t)), 2.0)
        check(ok, f"deadman ramps setpoints to 0 ({time.monotonic() - t_stop:.2f}s)")
        check(c.telemetry["safety"]["mode"] == "ENABLED", "deadman keeps ENABLED (no fault)")

        if profile == "patched":
            check(c.telemetry["motors"]["feedback_valid"], "patched: encoder feedback valid")
            check(c.telemetry["link"]["ultrasonic_available"], "patched: ultrasonic available")
        else:
            check(not c.telemetry["motors"]["feedback_valid"], "stock: no encoder feedback")
            check(not c.telemetry["link"]["ultrasonic_available"], "stock: no ultrasonic")

        # speed limit
        check((await c.request(type="set_speed_limit", scale=0.25))["ok"], "speed limit 25%")
        await c.drive(0.0, 1.0, 1.2)
        v = c.telemetry["safety"]["cmd_linear_mps"]
        check(abs(v - 0.25 * 0.2) < 0.005, f"speed capped at 25% of 0.2 m/s (got {v:.3f})")

        # STOP from a second client without lease
        async with websockets.connect(url) as ws2:
            c2 = Client(ws2)
            await c2.wait_for(lambda t: "safety" in t, 2.0)
            await c.send(type="drive", x=0.0, y=1.0)
            check((await c2.request(type="stop"))["ok"], "STOP accepted from a client without the lease")
            check(await c.wait_for(lambda t: t["safety"]["mode"] == "SAFE", 1.0), "STOP -> SAFE")
            check(await c.wait_for(lambda t: all(v == 0 for v in sp(t)), 1.0), "STOP -> setpoints 0")
            a = await c2.request(type="claim")
            check(not a["ok"], "second client cannot take an active lease")

        # calibration: one motor, bounded
        check((await c.request(type="set_mode", mode="CALIBRATION"))["ok"], "CALIBRATION accepted")
        a = await c.request(type="calibrate", motor=3, rps=0.5, duration_ms=300)
        check(not a["ok"], "calibration rps above limit refused")
        for _ in range(6):
            await c.send(type="calibrate", motor=3, rps=-0.2, duration_ms=300)
            await asyncio.sleep(0.1)
        s = sp(c.telemetry)
        check(abs(s[2] + 0.2) < 1e-6 and s[0] == s[1] == s[3] == 0, f"calibration drives only M3 ({s})")
        check(await c.wait_for(lambda t: all(v == 0 for v in sp(t)), 1.0), "calibration expires without refresh")

        # motor map from the UI: applied at runtime + persisted; refused while ENABLED; invalid refused
        good = {"FL": 1, "FR": 2, "RL": 3, "RR": 4}
        a = await c.request(type="save_motor_map", ports=good, signs=[1, -1, 1, -1])
        check(a["ok"], f"save_motor_map applied in CALIBRATION ({a['message']})")
        check(await c.wait_for(lambda t: t["calibration"]["saved"]["port_rr"] == 4, 2.0), "saved map visible in telemetry")
        a = await c.request(type="save_motor_map", ports={"FL": 1, "FR": 1, "RL": 3, "RR": 4}, signs=[1, -1, 1, -1])
        check(not a["ok"], "duplicate port refused")
        await c.request(type="set_mode", mode="ENABLED")
        a = await c.request(type="save_motor_map", ports=good, signs=[1, -1, 1, -1])
        check(not a["ok"], f"map change refused while ENABLED ({a['message'][:50]})")
        await c.request(type="set_mode", mode="CALIBRATION")

        # socket close -> stop
        await c.request(type="set_mode", mode="ENABLED")
        await c.drive(0.0, 1.0, 0.5)
    print("closed the controlling socket")
    async with websockets.connect(url) as ws3:
        c3 = Client(ws3)
        ok = await c3.wait_for(lambda t: all(abs(v) < 1e-6 for v in sp(t)), 2.0)
        check(ok, "closing the controlling socket stops the motors")
        check(not c3.telemetry["gateway"]["controlled_by_other"], "lease released on disconnect")
        await c3.request(type="stop")
    print(f"\n{len(failures)} failure(s)")
    return 1 if failures else 0


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--url", default="ws://127.0.0.1:8080/ws")
    ap.add_argument("--expect-profile", default="stock", choices=["stock", "patched"])
    args = ap.parse_args()
    sys.exit(asyncio.run(main(args.url, args.expect_profile)))
