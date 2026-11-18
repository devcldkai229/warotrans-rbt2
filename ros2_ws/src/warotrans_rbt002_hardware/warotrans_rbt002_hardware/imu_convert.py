"""RRC Lite IMU sample -> SI units, with validity checks. Pure Python.

The real board occasionally emits an all-zero IMU frame (seen in the 2026-11-15 probe, docs/calibration.md);
such frames are dropped instead of being published as "zero gravity".
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Sequence

G = 9.80665
UNITS = ("g_dps", "si")


@dataclass(frozen=True)
class ImuSI:
    accel: tuple[float, float, float]  # m/s^2
    gyro: tuple[float, float, float]  # rad/s


@dataclass
class ImuFilterStats:
    accepted: int = 0
    zero_frames: int = 0
    non_finite: int = 0
    implausible: int = 0


class ImuConverter:
    def __init__(self, units: str, max_accel_mps2: float = 8 * G, max_gyro_radps: float = math.radians(2000)) -> None:
        if units not in UNITS:
            raise ValueError(f"imu_units must be one of {UNITS}")
        self.units = units
        self._max_a = max_accel_mps2
        self._max_g = max_gyro_radps
        self.stats = ImuFilterStats()

    def convert(self, accel: Sequence[float], gyro: Sequence[float]) -> Optional[ImuSI]:
        values = list(accel) + list(gyro)
        if len(values) != 6 or not all(math.isfinite(v) for v in values):
            self.stats.non_finite += 1
            return None
        if all(v == 0.0 for v in values):
            self.stats.zero_frames += 1
            return None
        if self.units == "g_dps":
            a = tuple(v * G for v in accel)
            w = tuple(math.radians(v) for v in gyro)
        else:
            a = tuple(float(v) for v in accel)
            w = tuple(float(v) for v in gyro)
        if any(abs(v) > self._max_a for v in a) or any(abs(v) > self._max_g for v in w):
            self.stats.implausible += 1
            return None
        self.stats.accepted += 1
        return ImuSI(a, w)  # type: ignore[arg-type]


def gravity_check(accel_mps2: Sequence[float], tolerance: float = 0.15) -> bool:
    """True if |a| is within tolerance of 1 g (useful at rest to catch a wrong imu_units setting)."""
    mag = math.sqrt(sum(v * v for v in accel_mps2))
    return abs(mag - G) <= tolerance * G


class GyroBiasEstimator:
    """Gyro bias learned while the wheels are still (encoder rps ~ 0), subtracted always.

    2026-11-18 on RBT-002: the QMI8658 z bias is ~0.0034 rad/s (0.2 deg/s, 12 deg/min). The EKF integrated it raw;
    while the robot stood still AMCL does not update, so the heading in the map drifted 38 deg and RPP saw
    "collision ahead" next to a wall that was beside the robot.
    """

    def __init__(self, still_rps: float = 0.005, settle_s: float = 0.5, tau_s: float = 4.0,
                 max_bias_radps: float = 0.05) -> None:
        self.still_rps = still_rps
        self.settle_s = settle_s
        self.tau_s = tau_s
        self.max_bias = max_bias_radps
        self.bias = [0.0, 0.0, 0.0]
        self.samples = 0
        self._still_since: Optional[float] = None
        self._last_t: Optional[float] = None

    def on_wheels(self, rps: Sequence[float], now: float) -> None:
        if all(abs(v) < self.still_rps for v in rps):
            if self._still_since is None:
                self._still_since = now
        else:
            self._still_since = None

    def still(self, now: float) -> bool:
        return self._still_since is not None and now - self._still_since >= self.settle_s

    def apply(self, gyro: Sequence[float], now: float) -> tuple[float, float, float]:
        if self.still(now) and all(abs(g) < self.max_bias for g in gyro):
            dt = 0.02 if self._last_t is None else min(0.2, max(0.0, now - self._last_t))
            # fast start (plain mean of the first ~2 s), then an EMA with tau_s
            k = 1.0 / (self.samples + 1) if self.samples < 100 else dt / self.tau_s
            self.bias = [b + k * (g - b) for b, g in zip(self.bias, gyro)]
            self.samples += 1
        self._last_t = now
        return tuple(g - b for g, b in zip(gyro, self.bias))
