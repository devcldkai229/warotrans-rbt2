"""Hiwonder RRC Lite serial protocol (pure Python, no ROS imports).

Verified against Hiwonder's official host SDK (LanderPi `ros_robot_controller_sdk.py`) and the RRC Lite
firmware source `RosRobotControllerLite_ros_250811` (see docs/audit/rrc_lite_protocol.md):

    0xAA 0x55 | function u8 | length u8 | data[length] | crc8
    crc8 = CRC-8/MAXIM (reflected poly 0x8C, init 0, no xorout) over function, length and data.
    Multi-byte fields are little-endian. Baud 1 000 000 8N1. There is no protocol version field.

Items tagged PATCH exist only in the proposed firmware patch (docs/firmware/patch_proposal.md). The stock
firmware never sends them, so the driver only reports encoder / ultrasonic data after it has received one.
"""

from __future__ import annotations

import math
import struct
from dataclasses import dataclass
from enum import IntEnum
from typing import Iterable, Optional, Sequence

BAUD_RATE = 1_000_000
START1 = 0xAA
START2 = 0x55
MOTOR_COUNT = 4
# Longest legitimate payload we accept. Stock reports are <= 24 bytes; host commands we send are <= 22.
MAX_PAYLOAD = 64


class Func(IntEnum):
    SYS = 0
    LED = 1
    BUZZER = 2
    MOTOR = 3
    PWM_SERVO = 4
    BUS_SERVO = 5
    KEY = 6
    IMU = 7
    GAMEPAD = 8  # removed on RRC Lite
    SBUS = 9  # removed on RRC Lite
    OLED = 10
    RGB = 11
    ULTRASONIC = 12  # PATCH
    NONE = 13  # stock firmware: NONE = 12 (rejects func >= 12)


# SYS sub-commands
SYS_SET_BATTERY_ALARM = 0x01  # host -> board <BH> mV (low-voltage buzzer only, does not stop motors)
SYS_BATTERY_REPORT = 0x04  # board -> host <BH> mV, about once per second
SYS_VERSION = 0x10  # PATCH: host -> board query (no args); board -> host <BBBHI> sub, major, minor, flags, build

# MOTOR sub-commands
MOTOR_SET_ONE = 0x00  # <BBf> sub, id, rps
MOTOR_SET_MANY = 0x01  # [sub, n, (id u8, rps f32) * n]
MOTOR_STOP_ONE = 0x02  # [sub, id]
MOTOR_STOP_MASK = 0x03  # [sub, mask] bit i = motor i
MOTOR_SET_TYPE = 0x05  # [sub, type] applies to all 4 motors, RAM only
MOTOR_FEEDBACK = 0x80  # PATCH: board -> host <BI4i4f> sub, mcu_ms, counts[4], rps[4]

STOP_ALL_MASK = 0x0F


class MotorType(IntEnum):
    """Firmware motor presets (motors_param.h). counts = encoder counts per output-shaft revolution."""

    JGB520 = 0  # 3960 counts, rps_limit 1.5
    JGB37 = 1  # 1980 counts, rps_limit 3.0
    JGA27 = 2  # 1040 counts, rps_limit 6.0 (power-on default of FW-25)
    JGB528 = 3  # 5764 counts, rps_limit 1.1


MOTOR_TYPE_RPS_LIMIT = {MotorType.JGB520: 1.5, MotorType.JGB37: 3.0, MotorType.JGA27: 6.0, MotorType.JGB528: 1.1}
MOTOR_TYPE_COUNTS_PER_REV = {MotorType.JGB520: 3960, MotorType.JGB37: 1980, MotorType.JGA27: 1040,
                             MotorType.JGB528: 5764}


def _crc8_table() -> tuple[int, ...]:
    table = []
    for i in range(256):
        c = i
        for _ in range(8):
            c = (c >> 1) ^ 0x8C if c & 1 else c >> 1
        table.append(c)
    return tuple(table)


CRC8_TABLE = _crc8_table()


def crc8(data: bytes | bytearray | Iterable[int]) -> int:
    c = 0
    for b in data:
        c = CRC8_TABLE[c ^ b]
    return c


class ProtocolError(ValueError):
    pass


def build_frame(func: int, payload: bytes) -> bytes:
    if not 0 <= int(func) < 256:
        raise ProtocolError(f"function {func} out of range")
    if len(payload) > 255:
        raise ProtocolError("payload longer than 255 bytes")
    body = bytes([int(func), len(payload)]) + bytes(payload)
    return bytes([START1, START2]) + body + bytes([crc8(body)])


@dataclass(frozen=True)
class Frame:
    func: int
    payload: bytes


@dataclass
class ParserStats:
    frames: int = 0
    crc_errors: int = 0
    bad_function: int = 0
    oversize: int = 0
    discarded_bytes: int = 0


class FrameParser:
    """Incremental parser. Survives garbage, split frames, bad CRC and bogus lengths.

    On any error it restarts the search one byte after the failed start marker, so a valid frame that
    begins inside a corrupt one is still found.
    """

    def __init__(self, max_payload: int = MAX_PAYLOAD, max_func: int = Func.NONE) -> None:
        self._buf = bytearray()
        self._max_payload = max_payload
        self._max_func = int(max_func)
        self.stats = ParserStats()

    def reset(self) -> None:
        self._buf.clear()

    def feed(self, data: bytes) -> list[Frame]:
        self._buf.extend(data)
        out: list[Frame] = []
        buf = self._buf
        while True:
            start = buf.find(b"\xAA\x55")
            if start < 0:
                # keep a trailing 0xAA: it may be the first half of a marker
                keep = 1 if buf and buf[-1] == START1 else 0
                self.stats.discarded_bytes += len(buf) - keep
                del buf[: len(buf) - keep]
                break
            if start:
                self.stats.discarded_bytes += start
                del buf[:start]
            if len(buf) < 4:
                break
            func, length = buf[2], buf[3]
            if func >= self._max_func:
                self.stats.bad_function += 1
                self._skip_marker()
                continue
            if length > self._max_payload:
                self.stats.oversize += 1
                self._skip_marker()
                continue
            total = 5 + length
            if len(buf) < total:
                break
            if crc8(buf[2 : 4 + length]) != buf[4 + length]:
                self.stats.crc_errors += 1
                self._skip_marker()
                continue
            out.append(Frame(func, bytes(buf[4 : 4 + length])))
            self.stats.frames += 1
            del buf[:total]
        return out

    def _skip_marker(self) -> None:
        self.stats.discarded_bytes += 1
        del self._buf[:1]


# ---------------------------------------------------------------- host -> board commands


def _check_motor_index(index: int) -> int:
    # The firmware indexes motors[id] with no bounds check: an id >= 4 writes past the array.
    if not isinstance(index, int) or isinstance(index, bool) or not 0 <= index < MOTOR_COUNT:
        raise ProtocolError(f"motor index {index!r} not in 0..3")
    return index


def _check_rps(rps: float, limit: float) -> float:
    value = float(rps)
    if not math.isfinite(value):
        raise ProtocolError("non-finite motor speed")
    if limit <= 0 or not math.isfinite(limit):
        raise ProtocolError("rps limit must be a positive finite number")
    if abs(value) > limit:
        raise ProtocolError(f"motor speed {value} exceeds limit {limit}")
    return value


def encode_motor_speeds(speeds: Sequence[tuple[int, float]], rps_limit: float) -> bytes:
    """MOTOR sub 1. `speeds` = [(index 0..3, rps)], rps signed output-shaft rev/s, closed loop on the MCU.

    The firmware's serial path bypasses its own rps_limit, so the caller's limit is enforced here.
    """
    if not speeds:
        raise ProtocolError("no motor speeds")
    seen: set[int] = set()
    payload = bytearray([MOTOR_SET_MANY, len(speeds)])
    for index, rps in speeds:
        idx = _check_motor_index(index)
        if idx in seen:
            raise ProtocolError(f"motor index {idx} repeated")
        seen.add(idx)
        payload += struct.pack("<Bf", idx, _check_rps(rps, rps_limit))
    return build_frame(Func.MOTOR, bytes(payload))


def encode_motor_stop_mask(mask: int = STOP_ALL_MASK) -> bytes:
    if not 0 < mask <= STOP_ALL_MASK:
        raise ProtocolError("stop mask must be in 1..0x0F")
    return build_frame(Func.MOTOR, bytes([MOTOR_STOP_MASK, mask]))


def encode_set_motor_type(motor_type: int) -> bytes:
    mt = MotorType(int(motor_type))  # raises ValueError for unknown types (firmware would silently pick JGB520)
    return build_frame(Func.MOTOR, bytes([MOTOR_SET_TYPE, int(mt)]))


def encode_battery_alarm(millivolts: int) -> bytes:
    if not 0 < int(millivolts) < 65536:
        raise ProtocolError("battery alarm mV out of range")
    return build_frame(Func.SYS, struct.pack("<BH", SYS_SET_BATTERY_ALARM, int(millivolts)))


def encode_version_query() -> bytes:
    """PATCH query. The stock firmware ignores unknown SYS sub-commands (verified in packet_handle.c)."""
    return build_frame(Func.SYS, bytes([SYS_VERSION]))


# ---------------------------------------------------------------- board -> host reports


@dataclass(frozen=True)
class ImuSample:
    accel: tuple[float, float, float]  # firmware units (FW-25: g)
    gyro: tuple[float, float, float]  # firmware units (FW-25: deg/s)


@dataclass(frozen=True)
class BatterySample:
    millivolts: int


@dataclass(frozen=True)
class KeyEvent:
    key_id: int
    event: int


@dataclass(frozen=True)
class MotorFeedback:  # PATCH
    mcu_ms: int
    counts: tuple[int, int, int, int]
    rps: tuple[float, float, float, float]


@dataclass(frozen=True)
class UltrasonicSample:  # PATCH
    distance_mm: int  # 0 = no echo / invalid


@dataclass(frozen=True)
class FirmwareVersion:  # PATCH
    major: int
    minor: int
    flags: int
    build: int


Report = ImuSample | BatterySample | KeyEvent | MotorFeedback | UltrasonicSample | FirmwareVersion

_FEEDBACK = struct.Struct("<BI4i4f")
_VERSION = struct.Struct("<BBBHI")


def decode(frame: Frame) -> Optional[Report]:
    """Decode a board report. Returns None for frames we do not use; raises ProtocolError if malformed."""
    f, p = frame.func, frame.payload
    if f == Func.IMU:
        if len(p) != 24:
            raise ProtocolError(f"IMU payload {len(p)} != 24")
        v = struct.unpack("<6f", p)
        return ImuSample(v[0:3], v[3:6])
    if f == Func.SYS:
        if p and p[0] == SYS_BATTERY_REPORT:
            if len(p) != 3:
                raise ProtocolError("battery payload length")
            return BatterySample(struct.unpack_from("<H", p, 1)[0])
        if p and p[0] == SYS_VERSION:
            if len(p) != _VERSION.size:
                raise ProtocolError("version payload length")
            _, major, minor, flags, build = _VERSION.unpack(p)
            return FirmwareVersion(major, minor, flags, build)
        return None
    if f == Func.KEY:
        if len(p) != 2:
            raise ProtocolError("key payload length")
        return KeyEvent(p[0], p[1])
    if f == Func.MOTOR:
        if p and p[0] == MOTOR_FEEDBACK:
            if len(p) != _FEEDBACK.size:
                raise ProtocolError("motor feedback payload length")
            v = _FEEDBACK.unpack(p)
            return MotorFeedback(v[1], tuple(v[2:6]), tuple(v[6:10]))
        return None
    if f == Func.ULTRASONIC:
        if len(p) != 2:
            raise ProtocolError("ultrasonic payload length")
        return UltrasonicSample(struct.unpack("<H", p)[0])
    return None


# Encoders for board-side reports: used by the mock board and tests.


def encode_imu_report(accel: Sequence[float], gyro: Sequence[float]) -> bytes:
    return build_frame(Func.IMU, struct.pack("<6f", *accel, *gyro))


def encode_battery_report(millivolts: int) -> bytes:
    return build_frame(Func.SYS, struct.pack("<BH", SYS_BATTERY_REPORT, millivolts))


def encode_feedback_report(mcu_ms: int, counts: Sequence[int], rps: Sequence[float]) -> bytes:
    return build_frame(Func.MOTOR, _FEEDBACK.pack(MOTOR_FEEDBACK, mcu_ms & 0xFFFFFFFF, *counts, *rps))


def encode_ultrasonic_report(distance_mm: int) -> bytes:
    return build_frame(Func.ULTRASONIC, struct.pack("<H", distance_mm))


def encode_version_report(major: int, minor: int, flags: int = 0, build: int = 0) -> bytes:
    return build_frame(Func.SYS, _VERSION.pack(SYS_VERSION, major, minor, flags, build))


@dataclass(frozen=True)
class MotorCommand:
    """Decoded host -> board MOTOR command (mock board / tests)."""

    sub: int
    speeds: tuple[tuple[int, float], ...] = ()
    mask: int = 0
    motor_type: int = -1


def decode_motor_command(frame: Frame) -> MotorCommand:
    p = frame.payload
    if frame.func != Func.MOTOR or not p:
        raise ProtocolError("not a motor command")
    sub = p[0]
    if sub == MOTOR_SET_MANY:
        n = p[1]
        if len(p) != 2 + 5 * n:
            raise ProtocolError("motor set-many length")
        speeds = tuple(struct.unpack_from("<Bf", p, 2 + 5 * i) for i in range(n))
        return MotorCommand(sub, speeds=speeds)
    if sub == MOTOR_SET_ONE:
        idx, rps = struct.unpack_from("<Bf", p, 1)
        return MotorCommand(sub, speeds=((idx, rps),))
    if sub == MOTOR_STOP_ONE:
        return MotorCommand(sub, mask=1 << p[1])
    if sub == MOTOR_STOP_MASK:
        return MotorCommand(sub, mask=p[1])
    if sub == MOTOR_SET_TYPE:
        return MotorCommand(sub, motor_type=p[1])
    raise ProtocolError(f"unknown motor sub {sub}")
