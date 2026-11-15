"""Shared parameter handling. Convention: 0 / 0.0 means "CHƯA XÁC NHẬN" (not measured yet).

ROS 2 parameter files cannot carry null, so unmeasured physical values are 0 and every consumer treats a
non-positive value as missing. Nothing is ever defaulted to a guessed number.
"""

from __future__ import annotations

from typing import Optional

from .kinematics import Geometry, MotorMap, WHEELS, parse_geometry, parse_motor_map

GEOMETRY_PARAMS = {"wheel_radius_m": 0.0, "half_wheelbase_m": 0.0, "half_track_m": 0.0}
MAP_PARAMS = {"port_fl": 0, "port_fr": 0, "port_rl": 0, "port_rr": 0, "port_signs": [0, 0, 0, 0]}


def declare_geometry_and_map(node) -> None:
    for k, v in {**GEOMETRY_PARAMS, **MAP_PARAMS}.items():
        node.declare_parameter(k, v)


def read_geometry(node) -> Optional[Geometry]:
    g = lambda n: float(node.get_parameter(n).value)  # noqa: E731
    return parse_geometry(g("wheel_radius_m"), g("half_wheelbase_m"), g("half_track_m"))


def read_motor_map(node) -> Optional[MotorMap]:
    ports = {w: int(node.get_parameter(f"port_{w.lower()}").value) for w in WHEELS}
    signs = [int(s) for s in node.get_parameter("port_signs").value]
    if len(signs) != 4:
        raise ValueError("port_signs must have 4 entries")
    return parse_motor_map(ports, signs)


def positive_or_none(value: float) -> Optional[float]:
    value = float(value)
    return value if value > 0 else None
