import math
import random
import struct

import pytest

from warotrans_rrc_lite import protocol as p

# First 32 entries of crc8_table in Hiwonder's official SDK (LanderPi ros_robot_controller_sdk.py).
SDK_TABLE_HEAD = [0, 94, 188, 226, 97, 63, 221, 131, 194, 156, 126, 32, 163, 253, 31, 65,
                  157, 195, 33, 127, 252, 162, 64, 30, 95, 1, 227, 189, 62, 96, 130, 220]
SDK_TABLE_TAIL = [116, 42, 200, 150, 21, 75, 169, 247, 182, 232, 10, 84, 215, 137, 107, 53]


def sdk_buf_write(func, data):
    """Re-implementation of the SDK's Board.buf_write framing, for cross-checking."""
    buf = [0xAA, 0x55, int(func), len(data)] + list(data)
    check = 0
    for b in bytes(buf[2:]):
        check = p.CRC8_TABLE[check ^ b]
    return bytes(buf + [check])


def test_crc_table_matches_sdk():
    assert list(p.CRC8_TABLE[:32]) == SDK_TABLE_HEAD
    assert list(p.CRC8_TABLE[-16:]) == SDK_TABLE_TAIL


def test_crc8_maxim_check_value():
    # CRC-8/MAXIM catalogue check value for "123456789"
    assert p.crc8(b"123456789") == 0xA1


def test_motor_frame_matches_sdk_layout():
    frame = p.encode_motor_speeds([(0, 0.5), (3, -0.25)], rps_limit=1.0)
    data = [0x01, 2] + list(struct.pack("<Bf", 0, 0.5)) + list(struct.pack("<Bf", 3, -0.25))
    assert frame == sdk_buf_write(p.Func.MOTOR, data)


def test_stop_and_type_frames():
    assert p.encode_motor_stop_mask() == sdk_buf_write(3, [0x03, 0x0F])
    assert p.encode_set_motor_type(p.MotorType.JGB37) == sdk_buf_write(3, [0x05, 0x01])
    with pytest.raises(ValueError):
        p.encode_set_motor_type(7)


@pytest.mark.parametrize("speeds", [
    [(4, 0.1)], [(-1, 0.1)], [(True, 0.1)], [(0, float("nan"))], [(0, float("inf"))], [(0, 1.01)],
    [(0, 0.1), (0, 0.2)], [],
])
def test_motor_command_validation(speeds):
    with pytest.raises(p.ProtocolError):
        p.encode_motor_speeds(speeds, rps_limit=1.0)


def test_parse_reports_roundtrip():
    stream = (p.encode_imu_report((0.0, 0.05, 0.94), (0.25, -0.5, 0.2)) + p.encode_battery_report(11870)
              + p.encode_feedback_report(1234, [1, -2, 3, -4], [0.1, 0.2, 0.3, 0.4])
              + p.encode_ultrasonic_report(850) + p.encode_version_report(1, 0, 7, 20261115))
    parser = p.FrameParser()
    reports = [p.decode(f) for f in parser.feed(stream)]
    assert isinstance(reports[0], p.ImuSample) and reports[0].accel[2] == pytest.approx(0.94)
    assert reports[1] == p.BatterySample(11870)
    assert reports[2].counts == (1, -2, 3, -4) and reports[2].mcu_ms == 1234
    assert reports[3] == p.UltrasonicSample(850)
    assert reports[4] == p.FirmwareVersion(1, 0, 7, 20261115)
    assert parser.stats.crc_errors == 0


def test_parser_split_byte_by_byte():
    frame = p.encode_battery_report(12000)
    parser = p.FrameParser()
    out = []
    for b in frame:
        out += parser.feed(bytes([b]))
    assert len(out) == 1 and p.decode(out[0]) == p.BatterySample(12000)


def test_parser_skips_garbage_bad_crc_and_finds_next_frame():
    good = p.encode_battery_report(12000)
    bad = bytearray(p.encode_battery_report(11000))
    bad[-1] ^= 0xFF
    stream = b"\x00\xAA\x13\xAA" + bytes(bad) + b"\xAA" + good
    parser = p.FrameParser()
    frames = parser.feed(stream)
    assert [p.decode(f) for f in frames] == [p.BatterySample(12000)]
    assert parser.stats.crc_errors == 1


def test_parser_rejects_oversize_and_bad_function_then_resyncs():
    good = p.encode_battery_report(12000)
    stream = b"\xAA\x55\x07\xFF" + b"\xAA\x55\x30\x02\x00\x00\x00" + good
    parser = p.FrameParser()
    assert [p.decode(f) for f in parser.feed(stream)] == [p.BatterySample(12000)]
    assert parser.stats.oversize == 1 and parser.stats.bad_function == 1


def test_frame_hidden_inside_corrupt_length():
    # A false start marker with a large length must not swallow the real frame behind it.
    good = p.encode_battery_report(12000)
    stream = b"\xAA\x55\x07\x30" + good
    parser = p.FrameParser()
    frames = parser.feed(stream)
    frames += parser.feed(b"\x00" * 60)
    assert p.BatterySample(12000) in [p.decode(f) for f in frames]


def test_fuzz_never_raises_and_recovers():
    rng = random.Random(1)
    parser = p.FrameParser()
    for _ in range(200):
        parser.feed(bytes(rng.randrange(256) for _ in range(rng.randrange(1, 64))))
    frames = parser.feed(p.encode_battery_report(9999) * 2)
    assert p.BatterySample(9999) in [p.decode(f) for f in frames]


def test_decode_malformed_payloads():
    with pytest.raises(p.ProtocolError):
        p.decode(p.Frame(p.Func.IMU, b"\x00" * 23))
    with pytest.raises(p.ProtocolError):
        p.decode(p.Frame(p.Func.SYS, b"\x04\x00"))
    assert p.decode(p.Frame(p.Func.PWM_SERVO, b"\x01")) is None


def test_decode_motor_command():
    frame = p.FrameParser().feed(p.encode_motor_speeds([(1, 0.3)], 1.0))[0]
    cmd = p.decode_motor_command(frame)
    assert cmd.sub == 1 and cmd.speeds[0][0] == 1 and math.isclose(cmd.speeds[0][1], 0.3, rel_tol=1e-6)
