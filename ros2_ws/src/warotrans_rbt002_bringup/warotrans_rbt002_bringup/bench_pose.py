"""Bench mode pose math (no ROS): the robot stands on a stand with its wheels in the air.

The lidar then sees the same room whatever the wheels do, so AMCL would pull the pose back to the stand and Nav2
would never reach a goal. On the bench the wheel odometry is the truth: map->odom is fixed at the moment an
operator (or the backend's LOCALIZE) places the robot, and the robot "drives" through the map by odometry alone.

Poses are (x, y, yaw) in 2D.
"""

from __future__ import annotations

import math
from typing import Tuple

Pose2D = Tuple[float, float, float]


def normalize(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def compose(a: Pose2D, b: Pose2D) -> Pose2D:
    """a ∘ b: pose b expressed in a's frame, moved into a's parent frame."""
    ax, ay, ayaw = a
    bx, by, byaw = b
    c, s = math.cos(ayaw), math.sin(ayaw)
    return (ax + c * bx - s * by, ay + s * bx + c * by, normalize(ayaw + byaw))


def inverse(a: Pose2D) -> Pose2D:
    ax, ay, ayaw = a
    c, s = math.cos(ayaw), math.sin(ayaw)
    return (-(c * ax + s * ay), -(-s * ax + c * ay), normalize(-ayaw))


def map_to_odom(map_base: Pose2D, odom_base: Pose2D) -> Pose2D:
    """map->odom such that map->odom ∘ odom->base == map->base (the pose the operator gave)."""
    return compose(map_base, inverse(odom_base))


def map_pose(map_odom: Pose2D, odom_base: Pose2D) -> Pose2D:
    """Where the robot is in the map: map->odom ∘ odom->base."""
    return compose(map_odom, odom_base)
