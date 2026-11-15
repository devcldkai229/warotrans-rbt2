"""Byte-level simulation of the RRC Lite firmware, for running the whole stack without hardware.

Profiles:
  stock   — behaves like the audited stock firmware: IMU (g, deg/s) about 50 Hz, battery mV about 1 Hz,
            closed-loop motor setpoints with NO command timeout and NO encoder report.
  patched — adds the proposed patch: version reply, 250 ms motor watchdog, encoder feedback 50 Hz,
            ultrasonic 10 Hz.

Fault injection: unplug/replug, corrupt bytes, battery voltage, all-zero IMU frames (seen on the real board).
"""

from __future__ import annotations

import math
import random
import threading
import time
from dataclasses import dataclass, field
from typing import Callable, Optional

from . import protocol as p


@dataclass
class MockConfig:
    profile: str = "stock"  # stock | patched
    battery_mv: int = 12000
    imu_period_s: float = 0.02
    battery_period_s: float = 1.0
    feedback_period_s: float = 0.02
    ultrasonic_period_s: float = 0.1
    ultrasonic_mm: int = 850
    motor_tau_s: float = 0.08
    counts_per_rev: int = 1980
    rps_limit: float = 3.0  # what the patched firmware clamps to; stock does not clamp the serial path
    patched_watchdog_s: float = 0.25
    corrupt_probability: float = 0.0
    zero_imu_probability: float = 0.0
    seed: int = 0


@dataclass
class MockStats:
    motor_commands: int = 0
    stop_commands: int = 0
    type_commands: int = 0
    version_queries: int = 0
    out_of_range_ids: int = 0
    watchdog_trips: int = 0
    rx_frames: int = 0
    rx_crc_errors: int = 0
    history: list = field(default_factory=list)  # (t, setpoints) after every motor command


class MockBoard:
    def __init__(self, config: Optional[MockConfig] = None, clock: Callable[[], float] = time.monotonic,
                 realtime: bool = True) -> None:
        self.cfg = config or MockConfig()
        if self.cfg.profile not in ("stock", "patched"):
            raise ValueError("profile must be stock|patched")
        self._clock = clock
        self._realtime = realtime
        self._rng = random.Random(self.cfg.seed)
        self._lock = threading.Lock()
        self._out = bytearray()
        self._parser = p.FrameParser()
        self.connected = True
        self.motor_type: Optional[int] = None
        self.setpoints = [0.0] * 4
        self.rps = [0.0] * 4
        self._counts = [0.0] * 4
        self.stats = MockStats()
        t = clock()
        self._t_sim = t
        self._t0 = t
        self._next = {"imu": t, "bat": t + 0.3, "fb": t, "us": t}
        self._last_motor_cmd = t

    # ------------------------------------------------------------ host side (called by MockTransport)

    def write_host(self, data: bytes) -> None:
        with self._lock:
            self._advance(self._clock())
            for frame in self._parser.feed(data):
                self.stats.rx_frames += 1
                self._handle(frame)
            self.stats.rx_crc_errors = self._parser.stats.crc_errors

    def read_host(self, max_bytes: int, timeout_s: float) -> bytes:
        deadline = self._clock() + timeout_s
        while True:
            with self._lock:
                self._advance(self._clock())
                if self._out:
                    chunk = bytes(self._out[:max_bytes])
                    del self._out[:max_bytes]
                    return chunk
            if not self._realtime or self._clock() >= deadline:
                return b""
            time.sleep(min(0.005, max(0.0, deadline - self._clock())))

    # ------------------------------------------------------------ test / fault controls

    def unplug(self) -> None:
        with self._lock:
            self.connected = False
            self._out.clear()
            self._parser.reset()

    def replug(self, reset_mcu: bool = False) -> None:
        with self._lock:
            self.connected = True
            if reset_mcu:
                self.setpoints = [0.0] * 4
                self.rps = [0.0] * 4
                self.motor_type = None

    def set_battery_mv(self, mv: int) -> None:
        with self._lock:
            self.cfg.battery_mv = int(mv)

    def counts(self) -> list[int]:
        return [int(c) for c in self._counts]

    # ------------------------------------------------------------ simulation

    def _handle(self, frame: p.Frame) -> None:
        if frame.func == p.Func.MOTOR:
            cmd = p.decode_motor_command(frame)
            self._last_motor_cmd = self._t_sim
            if cmd.sub in (p.MOTOR_SET_MANY, p.MOTOR_SET_ONE):
                self.stats.motor_commands += 1
                for idx, rps in cmd.speeds:
                    if idx >= 4:  # real firmware: out-of-bounds write (memory corruption)
                        self.stats.out_of_range_ids += 1
                        continue
                    if self.cfg.profile == "patched":
                        rps = max(-self.cfg.rps_limit, min(self.cfg.rps_limit, rps))
                    self.setpoints[idx] = float(rps)
            elif cmd.sub in (p.MOTOR_STOP_MASK, p.MOTOR_STOP_ONE):
                self.stats.stop_commands += 1
                for i in range(4):
                    if cmd.mask & (1 << i):
                        self.setpoints[i] = 0.0
            elif cmd.sub == p.MOTOR_SET_TYPE:
                self.stats.type_commands += 1
                self.motor_type = cmd.motor_type
            self.stats.history.append((self._t_sim, tuple(self.setpoints)))
        elif frame.func == p.Func.SYS and frame.payload[:1] == bytes([p.SYS_VERSION]):
            self.stats.version_queries += 1
            if self.cfg.profile == "patched":
                self._emit(p.encode_version_report(1, 0, 0x0007, 20261115))

    def _advance(self, now: float) -> None:
        if not self.connected:
            self._t_sim = now
            return
        # integrate in <= 10 ms steps (the firmware PID period)
        while self._t_sim < now:
            dt = min(0.01, now - self._t_sim)
            self._t_sim += dt
            t = self._t_sim
            if self.cfg.profile == "patched" and t - self._last_motor_cmd > self.cfg.patched_watchdog_s:
                if any(self.setpoints):
                    self.stats.watchdog_trips += 1
                self.setpoints = [0.0] * 4
            a = 1.0 - math.exp(-dt / self.cfg.motor_tau_s)
            for i in range(4):
                self.rps[i] += a * (self.setpoints[i] - self.rps[i])
                self._counts[i] += self.rps[i] * self.cfg.counts_per_rev * dt
            if t >= self._next["imu"]:
                self._next["imu"] += self.cfg.imu_period_s
                wz_dps = math.degrees(self._yaw_rate())
                if self._rng.random() < self.cfg.zero_imu_probability:
                    self._emit(p.encode_imu_report((0.0, 0.0, 0.0), (0.0, 0.0, 0.0)))
                else:
                    self._emit(p.encode_imu_report((0.001, 0.05, 0.995), (0.2, -0.5, 0.2 + wz_dps)))
            if t >= self._next["bat"]:
                self._next["bat"] += self.cfg.battery_period_s
                self._emit(p.encode_battery_report(self.cfg.battery_mv))
            if self.cfg.profile == "patched":
                if t >= self._next["fb"]:
                    self._next["fb"] += self.cfg.feedback_period_s
                    ms = int((t - self._t0) * 1000)
                    self._emit(p.encode_feedback_report(ms, [int(c) for c in self._counts], self.rps))
                if t >= self._next["us"]:
                    self._next["us"] += self.cfg.ultrasonic_period_s
                    self._emit(p.encode_ultrasonic_report(self.cfg.ultrasonic_mm))

    def _yaw_rate(self) -> float:
        # crude: assumes ports 1,3 left and 2,4 right; only used to make the mock IMU respond
        left = (self.rps[0] + self.rps[2]) / 2
        right = (self.rps[1] + self.rps[3]) / 2
        return (right - left) * 0.5

    def _emit(self, frame: bytes) -> None:
        if self.cfg.corrupt_probability and self._rng.random() < self.cfg.corrupt_probability:
            b = bytearray(frame)
            b[self._rng.randrange(2, len(b))] ^= 0x5A
            frame = bytes(b)
        self._out += frame
