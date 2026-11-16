import math

import pytest

from warotrans_rbt002_hardware.controller import CALIBRATION, ENABLED, SAFE, BaseController, ControllerConfig
from warotrans_rbt002_hardware.kinematics import Geometry, MotorMap

# Test-only numbers: NOT RBT-002 measurements.
GEO = Geometry(0.04, 0.10, 0.12)
MAP = MotorMap({"FL": 1, "FR": 2, "RL": 3, "RR": 4}, [1, -1, 1, -1])


def cfg(**kw):
    base = dict(geometry=GEO, motor_map=MAP, max_wheel_rps=1.0, motors_enabled=True, battery_min_v=10.5,
                battery_max_v=12.8, default_speed_scale=1.0)
    base.update(kw)
    return ControllerConfig(**base)


def healthy(c, now):
    c.on_link(True, now)
    c.on_battery(12.0, now)


def tick(c, t0, seconds, dt=0.02, feed=True, cmd=None):
    t = t0
    out = None
    for _ in range(int(round(seconds / dt))):
        t += dt
        if feed:
            healthy(c, t)
        if cmd is not None:
            c.on_cmd_vel(*cmd, t)
        out = c.step(t)
    return t, out


def test_starts_safe_and_publishes_nothing_but_one_zero():
    c = BaseController(cfg())
    assert c.mode == SAFE
    assert c.step(0.0) == [0.0] * 4
    assert c.step(0.02) is None


@pytest.mark.parametrize("kw,needle", [
    (dict(motors_enabled=False), "motors_enabled"),
    (dict(geometry=None, percent_mode_without_geometry=False), "geometry"),
    (dict(motor_map=None), "motor port"),
    (dict(battery_min_v=None), "thresholds"),
])
def test_enable_blockers(kw, needle):
    c = BaseController(cfg(**kw))
    healthy(c, 0.0)
    ok, msg = c.request_mode(ENABLED, 0.0)
    assert not ok and needle in msg and c.mode == SAFE


def test_enable_refused_without_link_or_with_low_battery():
    c = BaseController(cfg())
    c.on_battery(12.0, 0.0)
    assert not c.request_mode(ENABLED, 0.0)[0]
    c.on_link(True, 0.0)
    c.on_battery(9.0, 0.0)
    ok, msg = c.request_mode(ENABLED, 0.0)
    assert not ok and "outside" in msg


def test_calibration_does_not_need_geometry_or_map():
    c = BaseController(cfg(geometry=None, motor_map=None))
    healthy(c, 0.0)
    assert c.request_mode(CALIBRATION, 0.0)[0]


def test_drive_forward_with_accel_limit_and_no_lateral():
    c = BaseController(cfg(max_linear_accel=0.5))
    healthy(c, 0.0)
    assert c.request_mode(ENABLED, 0.0)[0]
    t, out = tick(c, 0.0, 0.1, cmd=(0.2, 0.3, 0.0))  # linear.y = 0.3 must be dropped
    v_after = c.state(t).cmd_linear_mps
    assert v_after == pytest.approx(0.05, abs=0.011)  # 0.5 m/s^2 * 0.1 s
    assert c.lateral_rejected > 0
    t, out = tick(c, t, 1.0, cmd=(0.2, 0.0, 0.0))
    assert c.state(t).cmd_linear_mps == pytest.approx(0.2)
    assert out[0] > 0 and out[1] < 0  # signs from the motor map
    assert abs(out[0]) == pytest.approx(abs(out[1]))


def test_speed_limits_and_scale():
    c = BaseController(cfg(max_linear_mps=0.2, max_linear_accel=10.0))
    healthy(c, 0.0)
    c.request_mode(ENABLED, 0.0)
    assert c.set_speed_scale(0.5)[0]
    t, _ = tick(c, 0.0, 0.5, cmd=(1.0, 0.0, 0.0))
    assert c.state(t).cmd_linear_mps == pytest.approx(0.1)
    assert not c.set_speed_scale(1.5)[0]
    assert not c.set_speed_scale(math.nan)[0]


def test_wheel_rps_saturation():
    c = BaseController(cfg(max_linear_mps=5.0, max_linear_accel=100.0, max_wheel_rps=0.5))
    healthy(c, 0.0)
    c.request_mode(ENABLED, 0.0)
    _, out = tick(c, 0.0, 0.5, cmd=(5.0, 0.0, 0.0))
    assert max(abs(v) for v in out) == pytest.approx(0.5)


def test_deadman_ramps_to_zero():
    c = BaseController(cfg(max_linear_accel=10.0))
    healthy(c, 0.0)
    c.request_mode(ENABLED, 0.0)
    t, _ = tick(c, 0.0, 0.2, cmd=(0.2, 0.0, 0.0))
    t, out = tick(c, t, 0.5)  # no more commands
    assert out == pytest.approx([0.0] * 4)
    assert c.mode == ENABLED


def test_link_loss_forces_safe_and_latches_fault_no_auto_reenable():
    c = BaseController(cfg())
    healthy(c, 0.0)
    c.request_mode(ENABLED, 0.0)
    t, _ = tick(c, 0.0, 0.1, cmd=(0.2, 0.0, 0.0))
    c.on_link(False, t)
    out = c.step(t + 0.02)
    assert c.mode == SAFE and out == [0.0] * 4
    assert any("link" in f for f in c.state(t).faults)
    t, out = tick(c, t + 0.02, 1.0, cmd=(0.2, 0.0, 0.0))  # healthy again
    assert c.mode == SAFE and out is None


def test_battery_stale_forces_safe():
    c = BaseController(cfg(battery_stale_s=3.0))
    healthy(c, 0.0)
    c.request_mode(ENABLED, 0.0)
    t = 0.0
    for _ in range(200):  # link fresh, battery never refreshed
        t += 0.02
        c.on_link(True, t)
        c.step(t)
    assert c.mode == SAFE and any("stale" in f for f in c.faults)


def test_non_finite_cmd_treated_as_stop():
    c = BaseController(cfg(max_linear_accel=100.0))
    healthy(c, 0.0)
    c.request_mode(ENABLED, 0.0)
    t, _ = tick(c, 0.0, 0.2, cmd=(0.2, 0.0, 0.0))
    t, out = tick(c, t, 0.2, cmd=(math.nan, 0.0, 0.0))
    assert out == pytest.approx([0.0] * 4) and c.invalid_cmds > 0


def test_foreign_cmd_publisher_blocks_enabled():
    c = BaseController(cfg())
    healthy(c, 0.0)
    c.set_cmd_publishers_ok(False)
    assert not c.request_mode(ENABLED, 0.0)[0]


def test_calibration_one_motor_bounded():
    c = BaseController(cfg(calibration_max_rps=0.3, calibration_max_duration_ms=500))
    healthy(c, 0.0)
    assert not c.calibrate(1, 0.1, 100, 0.0)[0]  # not in calibration mode
    c.request_mode(CALIBRATION, 0.0)
    assert not c.calibrate(5, 0.1, 100, 0.0)[0]
    assert not c.calibrate(1, 0.5, 100, 0.0)[0]
    assert not c.calibrate(1, 0.1, 1000, 0.0)[0]
    assert c.calibrate(2, -0.2, 300, 0.0)[0]
    assert not c.calibrate(3, 0.1, 300, 0.01)[0]  # another motor running
    healthy(c, 0.02)
    assert c.step(0.02) == [0.0, -0.2, 0.0, 0.0]
    t, out = tick(c, 0.02, 0.4)
    assert out == [0.0] * 4  # expired
    assert c.calibrate(3, 0.1, 300, t)[0]


def test_cmd_vel_ignored_in_calibration_and_safe():
    c = BaseController(cfg())
    healthy(c, 0.0)
    c.request_mode(CALIBRATION, 0.0)
    _, out = tick(c, 0.0, 0.2, cmd=(0.2, 0.0, 0.0))
    assert out == [0.0] * 4


def test_request_safe_clears_faults_and_stops():
    c = BaseController(cfg())
    healthy(c, 0.0)
    c.request_mode(ENABLED, 0.0)
    tick(c, 0.0, 0.1, cmd=(0.2, 0.0, 0.0))
    assert c.request_mode(SAFE, 0.2)[0]
    assert c.step(0.22) == [0.0] * 4 and c.step(0.24) is None


def test_config_validation():
    with pytest.raises(ValueError):
        cfg(max_wheel_rps=0.0)
    with pytest.raises(ValueError):
        cfg(calibration_max_rps=2.0)


def test_percent_mode_without_geometry_like_rbt001():
    c = BaseController(cfg(geometry=None, max_wheel_rps=0.8, percent_accel_per_s=100.0))
    healthy(c, 0.0)
    assert c.units == "percent"
    assert c.request_mode(ENABLED, 0.0)[0]
    c.set_speed_scale(0.5)
    t, out = tick(c, 0.0, 0.2, cmd=(1.0, 0.0, 0.0))  # full stick -> 50 % of max_wheel_rps
    assert out == pytest.approx([0.4, -0.4, 0.4, -0.4])  # MAP signs: ports 2 and 4 inverted
    t, out = tick(c, t, 0.2, cmd=(0.0, 0.0, 1.0))  # rotate left: left wheels back, right forward
    assert out[0] == pytest.approx(-0.4) and out[1] == pytest.approx(-0.4)  # FL -0.4, FR +0.4*(-1)
    st = c.state(t)
    assert st.units == "percent" and st.max_linear_mps == pytest.approx(0.5)


def test_percent_mode_saturates_combined_command():
    c = BaseController(cfg(geometry=None, max_wheel_rps=0.5, percent_accel_per_s=100.0))
    healthy(c, 0.0)
    c.request_mode(ENABLED, 0.0)
    _, out = tick(c, 0.0, 0.2, cmd=(1.0, 0.0, 1.0))
    assert max(abs(v) for v in out) == pytest.approx(0.5)


def test_runtime_calibration_update_refused_while_enabled():
    c = BaseController(cfg(motor_map=None))
    healthy(c, 0.0)
    assert not c.request_mode(ENABLED, 0.0)[0]  # map missing
    assert c.update_calibration(GEO, MAP)[0]
    assert c.request_mode(ENABLED, 0.0)[0]
    assert not c.update_calibration(GEO, None)[0]


def test_auto_enable_after_stable_clear_and_after_fault():
    c = BaseController(cfg(auto_enable=True, auto_enable_stable_s=1.0))
    t, _ = tick(c, 0.0, 0.5)
    assert c.mode == SAFE  # not yet stable for 1 s
    t, _ = tick(c, t, 0.7)
    assert c.mode == ENABLED
    c.on_link(False, t)
    c.step(t + 0.02)
    assert c.mode == SAFE and c.faults
    t, _ = tick(c, t + 0.02, 0.5)  # healthy again but not long enough
    assert c.mode == SAFE
    t, _ = tick(c, t, 0.7)
    assert c.mode == ENABLED


def test_auto_enable_off_never_enables():
    c = BaseController(cfg())
    tick(c, 0.0, 5.0)
    assert c.mode == SAFE
