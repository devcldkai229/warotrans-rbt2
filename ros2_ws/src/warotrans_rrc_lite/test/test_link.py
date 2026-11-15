import math

import pytest

from warotrans_rrc_lite import protocol as p
from warotrans_rrc_lite.link import PROFILE_PATCHED, PROFILE_STOCK, LinkConfig, RrcLiteLink
from warotrans_rrc_lite.mock_board import MockBoard, MockConfig
from warotrans_rrc_lite.transport import MockTransport, TransportError, resolve_port


class FakeClock:
    def __init__(self):
        self.t = 100.0

    def __call__(self):
        return self.t

    def advance(self, dt):
        self.t += dt


def make(profile="stock", motors_allowed=True, **cfg):
    cfg.setdefault("motor_type", int(p.MotorType.JGB37))
    clock = FakeClock()
    board = MockBoard(MockConfig(profile=profile), clock=clock, realtime=False)
    reports = []
    link = RrcLiteLink(MockTransport(board), LinkConfig(rps_limit=1.0, motors_allowed=motors_allowed, **cfg),
                       on_report=lambda r, t: reports.append(r), clock=clock)
    return clock, board, link, reports


def run(link, clock, seconds, dt=0.01):
    for _ in range(int(round(seconds / dt))):
        clock.advance(dt)
        link.step()


def test_connect_sends_stop_but_no_motion():
    clock, board, link, _ = make()
    run(link, clock, 0.1)
    assert link.status().connected
    assert board.stats.stop_commands >= 1
    assert board.stats.motor_commands == 0
    assert board.setpoints == [0.0] * 4


def test_stock_profile_detected_and_imu_battery_reported():
    clock, board, link, reports = make()
    run(link, clock, 2.5)
    st = link.status()
    assert st.profile == PROFILE_STOCK and st.link_ok
    assert any(isinstance(r, p.ImuSample) for r in reports)
    assert any(isinstance(r, p.BatterySample) for r in reports)
    assert not any(isinstance(r, p.MotorFeedback) for r in reports)


def test_patched_profile_detected_with_feedback():
    clock, board, link, reports = make(profile="patched")
    run(link, clock, 0.3)
    assert link.status().profile == PROFILE_PATCHED
    assert link.status().firmware_version.startswith("1.0")
    assert any(isinstance(r, p.MotorFeedback) for r in reports)


def test_setpoints_reach_board_and_watchdog_stops_them():
    clock, board, link, _ = make(setpoint_timeout_s=0.2)
    run(link, clock, 0.1)
    assert link.set_setpoints([0.5, -0.5, 0.5, -0.5])
    run(link, clock, 0.05)
    assert board.setpoints == pytest.approx([0.5, -0.5, 0.5, -0.5])
    run(link, clock, 0.3)  # no new setpoint
    assert board.setpoints == [0.0] * 4
    st = link.status()
    assert st.watchdog_trips == 1 and st.watchdog_active


def test_streaming_setpoints_keeps_running():
    clock, board, link, _ = make(setpoint_timeout_s=0.2)
    run(link, clock, 0.1)
    for _ in range(30):
        link.set_setpoints([0.2] * 4)
        run(link, clock, 0.05)
    assert board.setpoints == pytest.approx([0.2] * 4)
    assert link.status().watchdog_trips == 0


def test_motors_not_allowed_never_sends_motion():
    clock, board, link, _ = make(motors_allowed=False)
    run(link, clock, 0.1)
    assert not link.set_setpoints([0.5] * 4)
    run(link, clock, 0.1)
    assert board.stats.motor_commands == 0
    assert link.status().rejected_setpoints == 1


@pytest.mark.parametrize("bad", [[2.0, 0, 0, 0], [math.nan, 0, 0, 0], [0.1, 0.1, 0.1], ["x", 0, 0, 0]])
def test_invalid_setpoint_rejected_and_stops(bad):
    clock, board, link, _ = make()
    run(link, clock, 0.1)
    link.set_setpoints([0.3] * 4)
    run(link, clock, 0.02)
    assert not link.set_setpoints(bad)
    run(link, clock, 0.02)
    assert board.setpoints == [0.0] * 4


def test_unplug_reconnect_and_stop_on_reconnect():
    clock, board, link, _ = make()
    run(link, clock, 0.1)
    link.set_setpoints([0.3] * 4)
    board.unplug()
    run(link, clock, 0.2)
    assert not link.status().connected
    assert not link.set_setpoints([0.3] * 4)  # dropped while disconnected
    stops_before = board.stats.stop_commands
    board.replug()
    run(link, clock, 2.0)
    st = link.status()
    assert st.connected and st.reconnects == 1
    assert board.stats.stop_commands > stops_before
    assert board.setpoints == [0.0] * 4  # stale setpoint from before the unplug cleared


def test_silent_board_triggers_reconnect():
    clock, board, link, _ = make(rx_dead_s=1.0)
    run(link, clock, 0.2)
    board.cfg.imu_period_s = 1e9
    board.cfg.battery_period_s = 1e9
    board._next = {k: 1e12 for k in board._next}
    run(link, clock, 1.5)
    assert link.status().reconnects >= 1


def test_corrupt_stream_counts_crc_errors_and_keeps_link():
    clock = FakeClock()
    board = MockBoard(MockConfig(corrupt_probability=0.2, seed=3), clock=clock, realtime=False)
    link = RrcLiteLink(MockTransport(board), LinkConfig(rps_limit=1.0), clock=clock)
    run(link, clock, 2.0)
    st = link.status()
    assert st.crc_errors + st.malformed > 0 and st.link_ok


def test_motor_type_sent_on_connect_before_any_stop():
    clock, board, link, _ = make(motor_type=int(p.MotorType.JGB520))
    run(link, clock, 0.05)
    assert board.motor_type == p.MotorType.JGB520
    # first recorded MOTOR command must be the type (history logs every motor command in order)
    assert board.stats.type_commands == 1
    assert board.stats.history[0][1] == (0.0, 0.0, 0.0, 0.0)


def test_set_motors_allowed_false_stops():
    clock, board, link, _ = make()
    run(link, clock, 0.1)
    link.set_setpoints([0.4] * 4)
    run(link, clock, 0.02)
    link.set_motors_allowed(False)
    run(link, clock, 0.02)
    assert board.setpoints == [0.0] * 4


def test_shutdown_sends_stop():
    clock, board, link, _ = make()
    run(link, clock, 0.1)
    link.set_setpoints([0.4] * 4)
    run(link, clock, 0.02)
    link.shutdown()
    assert board.setpoints == [0.0] * 4


def test_resolve_port_glob_must_match_one(tmp_path):
    (tmp_path / "usb-1a86_a").touch()
    assert resolve_port(str(tmp_path / "usb-1a86_*")).endswith("usb-1a86_a")
    (tmp_path / "usb-1a86_b").touch()
    with pytest.raises(TransportError):
        resolve_port(str(tmp_path / "usb-1a86_*"))
    with pytest.raises(TransportError):
        resolve_port("")


def test_config_validation():
    with pytest.raises(ValueError):
        LinkConfig(rps_limit=0)
    with pytest.raises(ValueError):
        LinkConfig(rps_limit=1.0, motor_type=9)


def test_no_reconnect_after_shutdown():
    clock, board, link, _ = make()
    run(link, clock, 0.1)
    link.shutdown()
    run(link, clock, 10.0)  # a racing run() loop keeps stepping
    assert not link.status().connected


def test_no_motor_packet_at_all_without_motor_type():
    """Regression 2026-11-15: a STOP packet started the firmware PID (JGA27 gains) -> rear wheels ran away."""
    clock, board, link, _ = make(motor_type=None)
    run(link, clock, 3.0)
    assert not link.set_setpoints([0.1] * 4)
    link.stop("test")
    link.set_motors_allowed(False)
    run(link, clock, 1.0)
    link.shutdown()
    assert board.stats.stop_commands == 0 and board.stats.motor_commands == 0 and board.stats.type_commands == 0
    assert board.stats.version_queries == 1  # SYS packets are fine
