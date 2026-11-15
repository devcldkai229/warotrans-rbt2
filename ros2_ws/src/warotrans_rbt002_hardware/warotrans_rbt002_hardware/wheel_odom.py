"""4-wheel encoder odometry for RBT-002. Pure Python.

Adapted from RBT-001 `warotrans_hardware/wheel_odom_node.py` (midpoint integration, MCU-clock deltas,
wraparound and MCU-reset handling, refuse non-positive physical parameters). Differences: four encoder
channels indexed by motor port, mecanum forward kinematics, int32 count wrap.

Only usable with the PATCHED firmware: the stock RRC Lite firmware sends no encoder data, so this
integrator is never fed and /odom is never published (no dead-reckoning from commanded speeds).
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Sequence

from .kinematics import Geometry, MotorMap, body_velocity, from_ports

U32 = 1 << 32
I32 = 1 << 32


def _wrap_i32(delta: int) -> int:
    delta %= I32
    return delta - I32 if delta >= I32 // 2 else delta


@dataclass
class OdomState:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0


class WheelOdometry:
    def __init__(self, geometry: Geometry, motor_map: MotorMap, counts_per_rev: float,
                 max_step_s: float = 0.5) -> None:
        if not (math.isfinite(counts_per_rev) and counts_per_rev > 0):
            raise ValueError("counts_per_rev must be > 0")
        self.g = geometry
        self.m = motor_map
        self.cpr = float(counts_per_rev)
        self.max_step_s = max_step_s
        self.state = OdomState()
        self._last_counts: Optional[list[int]] = None
        self._last_ms: Optional[int] = None
        self.resets = 0

    def update(self, mcu_ms: int, counts: Sequence[int]) -> bool:
        """Feed one encoder report. Returns True if the pose advanced."""
        counts = [int(c) for c in counts]
        if self._last_counts is None or self._last_ms is None:
            self._last_counts, self._last_ms = counts, mcu_ms
            return False
        dms = (mcu_ms - self._last_ms) % U32
        if dms == 0:
            return False
        if dms > U32 // 2 or dms / 1000.0 > self.max_step_s:
            # MCU clock went backwards (reset) or a gap too long to integrate safely: re-baseline.
            self.resets += 1
            self._last_counts, self._last_ms = counts, mcu_ms
            self.state.vx = self.state.vy = self.state.wz = 0.0
            return False
        dt = dms / 1000.0
        deltas_rev = [_wrap_i32(c - p) / self.cpr for c, p in zip(counts, self._last_counts)]
        self._last_counts, self._last_ms = counts, mcu_ms
        dx, dy, dyaw = body_velocity(from_ports(deltas_rev, self.m), self.g)
        s = self.state
        mid = s.yaw + dyaw / 2.0
        s.x += dx * math.cos(mid) - dy * math.sin(mid)
        s.y += dx * math.sin(mid) + dy * math.cos(mid)
        s.yaw = math.atan2(math.sin(s.yaw + dyaw), math.cos(s.yaw + dyaw))
        s.vx, s.vy, s.wz = dx / dt, dy / dt, dyaw / dt
        return True
