"""Mecanum kinematics restricted to forward / reverse / rotate (no lateral motion). Pure Python.

Wheel layout (REP-103, x forward, y left, viewed from above):

        FL  ---  FR        lx = half wheelbase (front-rear axle distance / 2)
         |   x^  |         ly = half track     (left-right wheel distance / 2)
         | y<-+  |
        RL  ---  RR

Standard mecanum inverse kinematics (rollers in "X" when seen from above):
    w_FL = (vx - vy - k*wz) / r     w_FR = (vx + vy + k*wz) / r
    w_RL = (vx + vy - k*wz) / r     w_RR = (vx - vy + k*wz) / r      k = lx + ly
RBT-002 forces vy = 0, so left wheels share one speed and right wheels another (skid-steer pattern).
Wheel speeds here are rev/s of the wheel; the gear output shaft drives the wheel directly.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Mapping, Optional, Sequence

WHEELS = ("FL", "FR", "RL", "RR")
TWO_PI = 2.0 * math.pi


@dataclass(frozen=True)
class Geometry:
    wheel_radius_m: float
    half_wheelbase_m: float
    half_track_m: float

    def __post_init__(self) -> None:
        for name in ("wheel_radius_m", "half_wheelbase_m", "half_track_m"):
            v = getattr(self, name)
            if not (isinstance(v, (int, float)) and math.isfinite(v) and v > 0):
                raise ValueError(f"{name} must be a positive number, got {v!r}")

    @property
    def k(self) -> float:
        return self.half_wheelbase_m + self.half_track_m


@dataclass(frozen=True)
class MotorMap:
    """Which motor port (1..4) drives each wheel, and the sign that makes +rps roll the robot forward."""

    port_of: Mapping[str, int]  # wheel -> port 1..4
    sign_of_port: Sequence[int]  # index port-1 -> +1 / -1

    def __post_init__(self) -> None:
        if set(self.port_of) != set(WHEELS):
            raise ValueError(f"motor map must name exactly {WHEELS}")
        ports = list(self.port_of.values())
        if sorted(ports) != [1, 2, 3, 4]:
            raise ValueError(f"each port 1..4 must be used once, got {ports}")
        if len(self.sign_of_port) != 4 or any(s not in (1, -1) for s in self.sign_of_port):
            raise ValueError("sign_of_port must be four values of +1 / -1")

    def wheel_of_port(self) -> list[str]:
        inv = {p: w for w, p in self.port_of.items()}
        return [inv[i] for i in (1, 2, 3, 4)]


def wheel_rps(vx: float, wz: float, g: Geometry) -> dict[str, float]:
    """Body velocity (m/s, rad/s) -> wheel rev/s. Lateral velocity is structurally impossible here."""
    left = (vx - g.k * wz) / g.wheel_radius_m / TWO_PI
    right = (vx + g.k * wz) / g.wheel_radius_m / TWO_PI
    return {"FL": left, "RL": left, "FR": right, "RR": right}


def to_ports(wheels: Mapping[str, float], m: MotorMap) -> list[float]:
    out = [0.0] * 4
    for w, rps in wheels.items():
        port = m.port_of[w]
        out[port - 1] = m.sign_of_port[port - 1] * rps
    return out


def from_ports(port_values: Sequence[float], m: MotorMap) -> dict[str, float]:
    return {w: m.sign_of_port[p - 1] * port_values[p - 1] for w, p in m.port_of.items()}


def saturate(values: Sequence[float], limit: float) -> tuple[list[float], float]:
    """Scale all values by one factor so none exceeds `limit` (keeps the turn curvature)."""
    peak = max((abs(v) for v in values), default=0.0)
    if peak <= limit or peak == 0.0:
        return list(values), 1.0
    f = limit / peak
    return [v * f for v in values], f


def body_velocity(wheels: Mapping[str, float], g: Geometry) -> tuple[float, float, float]:
    """Wheel rev/s (or revolutions) -> (vx, vy, wz) (or displacement). vy is what the encoders imply."""
    w = {k: v * TWO_PI for k, v in wheels.items()}
    r = g.wheel_radius_m
    vx = r / 4.0 * (w["FL"] + w["FR"] + w["RL"] + w["RR"])
    vy = r / 4.0 * (-w["FL"] + w["FR"] + w["RL"] - w["RR"])
    wz = r / (4.0 * g.k) * (-w["FL"] + w["FR"] - w["RL"] + w["RR"])
    return vx, vy, wz


def parse_motor_map(port_of: Mapping[str, Optional[int]], signs: Sequence[Optional[int]]) -> Optional[MotorMap]:
    """None while any value is still unconfigured (CHƯA XÁC NHẬN)."""
    if any(v in (None, 0) for v in port_of.values()) or any(s in (None, 0) for s in signs):
        return None
    return MotorMap({k: int(v) for k, v in port_of.items()}, [int(s) for s in signs])


def parse_geometry(r: Optional[float], lx: Optional[float], ly: Optional[float]) -> Optional[Geometry]:
    if any(v is None or (isinstance(v, (int, float)) and v <= 0) for v in (r, lx, ly)):
        return None
    return Geometry(float(r), float(lx), float(ly))
