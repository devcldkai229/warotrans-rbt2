import math

import pytest

from warotrans_rbt002_hardware.imu_convert import G, ImuConverter, gravity_check
from warotrans_rbt002_hardware.kinematics import (Geometry, MotorMap, body_velocity, from_ports, parse_geometry,
                                                  parse_motor_map, saturate, to_ports, wheel_rps)
from warotrans_rbt002_hardware.wheel_odom import WheelOdometry

# Test-only numbers: NOT RBT-002 measurements.
GEO = Geometry(0.04, 0.10, 0.12)
MAP = MotorMap({"FL": 1, "FR": 2, "RL": 3, "RR": 4}, [1, -1, 1, -1])


def test_forward_all_wheels_same_speed():
    w = wheel_rps(0.2, 0.0, GEO)
    expected = 0.2 / 0.04 / (2 * math.pi)
    assert all(v == pytest.approx(expected) for v in w.values())


def test_rotate_left_wheels_opposite():
    w = wheel_rps(0.0, 1.0, GEO)
    assert w["FL"] == w["RL"] == pytest.approx(-w["FR"]) and w["FL"] < 0


def test_never_lateral():
    for vx, wz in [(0.2, 0.0), (0.0, 1.0), (-0.1, 0.5), (0.3, -0.7)]:
        _, vy, _ = body_velocity(wheel_rps(vx, wz, GEO), GEO)
        assert vy == pytest.approx(0.0, abs=1e-12)


def test_inverse_forward_roundtrip_through_ports():
    ports = to_ports(wheel_rps(0.15, -0.6, GEO), MAP)
    vx, vy, wz = body_velocity(from_ports(ports, MAP), GEO)
    assert (vx, vy, wz) == pytest.approx((0.15, 0.0, -0.6))


def test_port_signs_applied():
    ports = to_ports(wheel_rps(0.2, 0.0, GEO), MAP)
    assert ports[0] > 0 and ports[1] < 0 and ports[2] > 0 and ports[3] < 0


def test_saturate_keeps_ratio():
    out, f = saturate([1.0, -2.0, 0.5, 2.0], 1.0)
    assert f == 0.5 and out == [0.5, -1.0, 0.25, 1.0]
    assert saturate([0.1, 0.2], 1.0) == ([0.1, 0.2], 1.0)


@pytest.mark.parametrize("ports,signs", [
    ({"FL": 1, "FR": 1, "RL": 3, "RR": 4}, [1, 1, 1, 1]),
    ({"FL": 1, "FR": 2, "RL": 3, "RR": 5}, [1, 1, 1, 1]),
    ({"FL": 1, "FR": 2, "RL": 3, "RR": 4}, [1, 2, 1, 1]),
])
def test_motor_map_validation(ports, signs):
    with pytest.raises(ValueError):
        MotorMap(ports, signs)


def test_unconfigured_values_give_none():
    assert parse_motor_map({"FL": None, "FR": 2, "RL": 3, "RR": 4}, [1, 1, 1, 1]) is None
    assert parse_motor_map({"FL": 1, "FR": 2, "RL": 3, "RR": 4}, [1, None, 1, 1]) is None
    assert parse_geometry(None, 0.1, 0.1) is None
    assert parse_geometry(0.04, 0.0, 0.1) is None
    assert parse_geometry(0.04, 0.1, 0.1) == Geometry(0.04, 0.1, 0.1)


def test_imu_g_dps_conversion_and_zero_frame_rejected():
    c = ImuConverter("g_dps")
    s = c.convert((0.0, 0.05, 0.94), (0.0, 0.0, 90.0))
    assert s.accel[2] == pytest.approx(0.94 * G) and s.gyro[2] == pytest.approx(math.pi / 2)
    assert c.convert((0, 0, 0), (0, 0, 0)) is None and c.stats.zero_frames == 1
    assert c.convert((math.nan, 0, 1), (0, 0, 0)) is None and c.stats.non_finite == 1
    assert c.convert((100.0, 0, 1), (0, 0, 0)) is None and c.stats.implausible == 1
    with pytest.raises(ValueError):
        ImuConverter("mg")


def test_gravity_check():
    assert gravity_check((0, 0.5, 9.3))
    assert not gravity_check((0, 0, 0.95))  # g-valued data mistakenly treated as SI


def test_odom_straight_line():
    cpr = 1980
    odo = WheelOdometry(GEO, MAP, cpr, max_step_s=2.0)
    rev = 1.0  # each wheel one revolution forward
    signs = MAP.sign_of_port
    odo.update(0, [0, 0, 0, 0])
    assert odo.update(1000, [int(s * rev * cpr) for s in signs])
    assert odo.state.x == pytest.approx(2 * math.pi * 0.04, rel=1e-3)
    assert odo.state.y == pytest.approx(0.0, abs=1e-9)
    assert odo.state.vx == pytest.approx(2 * math.pi * 0.04, rel=1e-3)


def test_odom_rotation_in_place():
    cpr = 1000
    odo = WheelOdometry(GEO, MAP, cpr, max_step_s=2.0)
    wheels = wheel_rps(0.0, 0.5, GEO)  # rev/s for 1 s -> revolutions
    ports = to_ports(wheels, MAP)
    odo.update(0, [0] * 4)
    odo.update(1000, [round(v * cpr) for v in ports])
    assert odo.state.yaw == pytest.approx(0.5, rel=2e-3)
    assert math.hypot(odo.state.x, odo.state.y) == pytest.approx(0.0, abs=1e-3)


def test_odom_mcu_reset_and_wrap():
    odo = WheelOdometry(GEO, MAP, 1000)
    odo.update(5000, [10, 10, 10, 10])
    assert not odo.update(10, [0, 0, 0, 0])  # clock went backwards: re-baseline
    assert odo.resets == 1
    odo = WheelOdometry(GEO, MAP, 1000)
    odo.update(2**32 - 10, [2**31 - 5, 0, 0, 0])
    assert odo.update(10, [-(2**31) + 5, 0, 0, 0])  # both u32 time and i32 counts wrap
    with pytest.raises(ValueError):
        WheelOdometry(GEO, MAP, 0)


def test_odom_long_gap_rebaselines_instead_of_integrating():
    odo = WheelOdometry(GEO, MAP, 1000, max_step_s=0.5)
    odo.update(0, [0] * 4)
    assert not odo.update(1000, [1000, -1000, 1000, -1000])
    assert odo.state.x == 0.0 and odo.resets == 1
