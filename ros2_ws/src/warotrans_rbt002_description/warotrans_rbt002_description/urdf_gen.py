"""Build the RBT-002 URDF from config/geometry.yaml (no xacro: it is not installed on the Pi).

Only fixed joints. An edge is emitted only when every value it needs is measured (not null). Adapted from
RBT-001 warotrans_description (geometry.yaml + "fail loudly on null" rule); the template is new.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Mapping, Optional

POSE_XYZ_YAW = ("x", "y", "z", "yaw")
POSE_6 = ("x", "y", "z", "roll", "pitch", "yaw")
CHILD_EDGES = {  # yaml section -> (child frame, required keys)
    "laser": ("laser", POSE_XYZ_YAW),
    "imu": ("imu_link", POSE_6),
    "ultrasonic_front": ("ultrasonic_front", POSE_XYZ_YAW),
    "camera": ("camera_link", POSE_6),
}


@dataclass
class UrdfResult:
    urdf: Optional[str]  # None when base_footprint -> base_link is not measured
    published: list[str] = field(default_factory=list)
    missing: list[str] = field(default_factory=list)
    mock: bool = False


def _complete(section: Optional[Mapping[str, Any]], keys: tuple[str, ...]) -> bool:
    return isinstance(section, Mapping) and all(isinstance(section.get(k), (int, float)) for k in keys)


def _origin(s: Mapping[str, Any]) -> str:
    r, p = float(s.get("roll", 0.0) or 0.0), float(s.get("pitch", 0.0) or 0.0)
    return (f'<origin xyz="{float(s["x"])} {float(s["y"])} {float(s["z"])}" '
            f'rpy="{r} {p} {float(s["yaw"])}"/>')


def build_urdf(geo: Mapping[str, Any], require_complete: bool = False) -> UrdfResult:
    res = UrdfResult(urdf=None, mock=bool(geo.get("mock", False)))
    base = geo.get("base_footprint_to_base_link")
    if not _complete(base, POSE_XYZ_YAW):
        res.missing.append("base_footprint_to_base_link")
    for section, (_, keys) in CHILD_EDGES.items():
        if not _complete(geo.get(section), keys):
            res.missing.append(section)
    if require_complete and res.missing:
        raise ValueError(f"geometry not measured (CHƯA XÁC NHẬN): {', '.join(res.missing)}")
    if "base_footprint_to_base_link" in res.missing:
        return res

    ch = geo.get("chassis") or {}
    box = ""
    if _complete(ch, ("length", "width", "height")):
        box = (f'<visual><geometry><box size="{ch["length"]} {ch["width"]} {ch["height"]}"/></geometry>'
               f'<material name="grey"><color rgba="0.6 0.6 0.6 1"/></material></visual>')
    parts = ['<?xml version="1.0"?>', '<robot name="warotrans_rbt002">',
             '<link name="base_footprint"/>', f'<link name="base_link">{box}</link>',
             f'<joint name="base_footprint_to_base_link" type="fixed"><parent link="base_footprint"/>'
             f'<child link="base_link"/>{_origin(base)}</joint>']
    res.published.append("base_footprint->base_link")
    for section, (child, keys) in CHILD_EDGES.items():
        s = geo.get(section)
        if not _complete(s, keys):
            continue
        parts.append(f'<link name="{child}"/>')
        parts.append(f'<joint name="base_link_to_{child}" type="fixed"><parent link="base_link"/>'
                     f'<child link="{child}"/>{_origin(s)}</joint>')
        res.published.append(f"base_link->{child}")
    parts.append("</robot>")
    res.urdf = "\n".join(parts)
    return res
