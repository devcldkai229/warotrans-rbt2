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
