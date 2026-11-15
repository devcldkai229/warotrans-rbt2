#!/usr/bin/env python3
"""Passive, read-only probe of the Hiwonder RRC Lite serial stream.

NEVER writes a byte to the port. Counts frames per function id, CRC errors, report periods, and prints
IMU / battery statistics so the firmware build (IMU units) can be identified.

Self-contained on purpose so it can run on the Pi without installing anything:
    ssh waro@robot 'python3 - --seconds 10' < tools/pi_probe.py

Opening a USB-CDC port may toggle DTR/RTS. Like Hiwonder's own SDK, DTR and RTS are set low before open.
Stop the ROS driver first: the port has a single owner.
"""

from __future__ import annotations

import argparse
import glob
import math
import statistics
import struct
import sys
import time

DEFAULT_GLOB = "/dev/serial/by-id/usb-1a86_*"
FUNC_NAMES = {0: "SYS", 1: "LED", 2: "BUZZER", 3: "MOTOR", 4: "PWM_SERVO", 5: "BUS_SERVO", 6: "KEY",
              7: "IMU", 8: "GAMEPAD", 9: "SBUS", 10: "OLED", 11: "RGB", 12: "ULTRASONIC(patch)"}


def crc8_table() -> list[int]:
    table = []
    for i in range(256):
        c = i
        for _ in range(8):
            c = (c >> 1) ^ 0x8C if c & 1 else c >> 1
        table.append(c)
    return table


TABLE = crc8_table()


def crc8(data: bytes) -> int:
    c = 0
    for b in data:
        c = TABLE[c ^ b]
    return c


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="")
    ap.add_argument("--seconds", type=float, default=10.0)
    ap.add_argument("--baud", type=int, default=1_000_000)
    args = ap.parse_args()

    import serial  # pyserial

    port = args.port
    if not port:
        matches = sorted(glob.glob(DEFAULT_GLOB))
        if len(matches) != 1:
            print(f"expected exactly one {DEFAULT_GLOB}, found {matches}")
            return 2
        port = matches[0]

    s = serial.Serial(None, args.baud, timeout=0.1)
    s.rts = False
    s.dtr = False
    s.port = port
    s.exclusive = True
    s.open()
    print(f"port={port} baud={args.baud} listening {args.seconds}s (read-only)")

    counts: dict[int, int] = {}
    times: dict[int, list[float]] = {}
    crc_err = 0
    raw_bytes = 0
    imu: list[tuple[float, ...]] = []
    batt: list[int] = []
    other: list[str] = []
    buf = bytearray()
    t_end = time.monotonic() + args.seconds
    while time.monotonic() < t_end:
        chunk = s.read(512)
        raw_bytes += len(chunk)
        buf.extend(chunk)
        while True:
            i = buf.find(b"\xAA\x55")
            if i < 0:
                del buf[:-1]
                break
            if i:
                del buf[:i]
            if len(buf) < 5:
                break
            func, length = buf[2], buf[3]
            if len(buf) < 5 + length:
                break
            frame = bytes(buf[:5 + length])
            if crc8(frame[2:4 + length]) != frame[4 + length]:
                crc_err += 1
                del buf[:2]
                continue
            del buf[:5 + length]
            data = frame[4:4 + length]
            now = time.monotonic()
            counts[func] = counts.get(func, 0) + 1
            times.setdefault(func, []).append(now)
            if func == 7 and length == 24:
                imu.append(struct.unpack("<6f", data))
            elif func == 0 and length == 3 and data[0] == 0x04:
                batt.append(struct.unpack("<H", data[1:])[0])
            elif len(other) < 20:
                other.append(f"func={func} len={length} data={data.hex()}")
    s.close()

    print(f"raw_bytes={raw_bytes} crc_errors={crc_err}")
    for func in sorted(counts):
        ts = times[func]
        period = statistics.median([b - a for a, b in zip(ts, ts[1:])]) if len(ts) > 2 else float("nan")
        print(f"  func {func:2d} {FUNC_NAMES.get(func, '?'):18s} n={counts[func]:5d} median_period={period * 1000:.1f} ms")
    if batt:
        print(f"battery_mV min={min(batt)} max={max(batt)} last={batt[-1]}")
    if imu:
        cols = list(zip(*imu))
        names = ["ax", "ay", "az", "gx", "gy", "gz"]
        for n, c in zip(names, cols):
            print(f"  {n}: mean={statistics.fmean(c):+.4f} stdev={statistics.pstdev(c):.4f} min={min(c):+.4f} max={max(c):+.4f}")
        amag = statistics.fmean(math.sqrt(a * a + b * b + c * c) for a, b, c, *_ in imu)
        unit = "g (FW-25 style: g, deg/s)" if 0.8 < amag < 1.2 else ("m/s^2 (SI build)" if 8.0 < amag < 11.5 else "UNKNOWN")
        print(f"accel |a| mean={amag:.4f} -> units look like {unit}")
    for line in other:
        print("  other:", line)
    return 0


if __name__ == "__main__":
    sys.exit(main())
