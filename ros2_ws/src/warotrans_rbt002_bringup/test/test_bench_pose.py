import math

import pytest

from warotrans_rbt002_bringup.bench_pose import compose, inverse, map_pose, map_to_odom


def _close(a, b):
    assert a[0] == pytest.approx(b[0], abs=1e-9)
    assert a[1] == pytest.approx(b[1], abs=1e-9)
    assert math.cos(a[2] - b[2]) == pytest.approx(1.0, abs=1e-9)


def test_inverse_round_trip():
    p = (1.2, -0.4, 2.5)
    _close(compose(p, inverse(p)), (0.0, 0.0, 0.0))


def test_initial_pose_is_reproduced_now():
    operator = (1.30, -0.81, math.pi / 2)
    odom_base = (3.7, 2.1, -1.0)  # the wheels already spun before the operator placed the robot
    mo = map_to_odom(operator, odom_base)
    _close(map_pose(mo, odom_base), operator)


def test_driving_forward_follows_the_map_heading():
    operator = (0.0, 0.0, math.pi / 2)  # facing +y in the map
    odom_start = (5.0, 5.0, 0.0)        # facing +x in odom
    mo = map_to_odom(operator, odom_start)
    moved = (5.5, 5.0, 0.0)             # 0.5 m forward in odom
    _close(map_pose(mo, moved), (0.0, 0.5, math.pi / 2))
