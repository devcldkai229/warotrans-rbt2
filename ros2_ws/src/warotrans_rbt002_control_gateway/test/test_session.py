import math

import pytest

from warotrans_rbt002_control_gateway.session import (FORWARD, REVERSE, ROTATE_LEFT, ROTATE_RIGHT, STOP,
                                                      ControlSession, quantize, to_twist)


@pytest.mark.parametrize("x,y,direction", [
    (0.0, 1.0, FORWARD), (0.0, -1.0, REVERSE), (-1.0, 0.0, ROTATE_LEFT), (1.0, 0.0, ROTATE_RIGHT),
    (0.4, 0.6, FORWARD), (0.7, -0.5, ROTATE_RIGHT), (0.05, 0.05, STOP), (math.nan, 1.0, STOP),
])
def test_quantize_four_directions(x, y, direction):
    assert quantize(x, y)[0] == direction


def test_quantize_never_mixes_and_magnitude_bounded():
    for x in [i / 10 - 1 for i in range(21)]:
        for y in [j / 10 - 1 for j in range(21)]:
            d, m = quantize(x * 3, y * 3)
            assert 0.0 <= m <= 1.0
            vx, wz = to_twist(d, m, 0.2, 0.8)
            assert vx == 0.0 or wz == 0.0


def test_to_twist_signs():
    assert to_twist(FORWARD, 1.0, 0.2, 0.8) == (0.2, 0.0)
    assert to_twist(REVERSE, 0.5, 0.2, 0.8) == (-0.1, 0.0)
    assert to_twist(ROTATE_LEFT, 1.0, 0.2, 0.8) == (0.0, 0.8)
    assert to_twist(ROTATE_RIGHT, 1.0, 0.2, 0.8) == (0.0, -0.8)
    assert to_twist(STOP, 1.0, 0.2, 0.8) == (0.0, 0.0)


def test_single_lease_and_takeover_after_timeout():
    s = ControlSession(lease_timeout_s=3.0)
    assert s.claim("a", 0.0)[0]
    assert not s.claim("b", 1.0)[0]
    assert not s.drive("b", 0.0, 1.0, 1.0)[0]
    assert s.claim("b", 3.5)[0]  # a went silent
    assert s.holder == "b"


def test_deadman():
    s = ControlSession(deadman_s=0.3)
    s.claim("a", 0.0)
    s.drive("a", 0.0, 1.0, 0.0)
    assert s.twist(0.1, 0.2, 0.8) == (0.2, 0.0)
    assert s.twist(0.31, 0.2, 0.8) == (0.0, 0.0)


def test_release_and_stop_cancel_motion():
    s = ControlSession()
    s.claim("a", 0.0)
    s.drive("a", 0.0, 1.0, 0.0)
    s.stop()
    assert s.twist(0.01, 0.2, 0.8) == (0.0, 0.0)
    s.drive("a", 0.0, 1.0, 0.02)
    s.release("a")
    assert s.twist(0.03, 0.2, 0.8) == (0.0, 0.0)


def test_new_holder_does_not_inherit_motion():
    s = ControlSession(lease_timeout_s=1.0, deadman_s=10.0)
    s.claim("a", 0.0)
    s.drive("a", 0.0, 1.0, 0.0)
    s.claim("b", 2.0)
    assert s.twist(2.0, 0.2, 0.8) == (0.0, 0.0)
