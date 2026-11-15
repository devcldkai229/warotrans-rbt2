"""Control session rules for the mobile UI. Pure Python (no ROS, no FastAPI).

  * One control lease: only the holder may drive, calibrate, enable or change the speed limit.
    Anyone connected may press STOP (it always wins).
  * Joystick input is quantised to exactly one of FORWARD / REVERSE / ROTATE_LEFT / ROTATE_RIGHT / STOP.
    Forward and rotation are never mixed and there is no lateral direction. This is enforced here, on the
    server, not only in the browser.
  * Deadman: a drive command older than `deadman_s` becomes STOP. Losing the lease or the socket stops.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

STOP, FORWARD, REVERSE, ROTATE_LEFT, ROTATE_RIGHT = "STOP", "FORWARD", "REVERSE", "ROTATE_LEFT", "ROTATE_RIGHT"
DIRECTIONS = (STOP, FORWARD, REVERSE, ROTATE_LEFT, ROTATE_RIGHT)


def quantize(x: float, y: float, deadzone: float = 0.15) -> tuple[str, float]:
    """Joystick (x right+, y up+, each -1..1) -> (direction, magnitude 0..1)."""
    if not (math.isfinite(x) and math.isfinite(y)):
        return STOP, 0.0
    x, y = max(-1.0, min(1.0, x)), max(-1.0, min(1.0, y))
    if math.hypot(x, y) < deadzone:
        return STOP, 0.0
    if abs(y) >= abs(x):
        mag = (abs(y) - deadzone) / (1.0 - deadzone)
        return (FORWARD if y > 0 else REVERSE), max(0.0, min(1.0, mag))
    mag = (abs(x) - deadzone) / (1.0 - deadzone)
    return (ROTATE_RIGHT if x > 0 else ROTATE_LEFT), max(0.0, min(1.0, mag))


def to_twist(direction: str, magnitude: float, max_linear: float, max_angular: float) -> tuple[float, float]:
    """(direction, magnitude) -> (vx m/s, wz rad/s). REP-103: rotate left = +wz."""
    m = max(0.0, min(1.0, magnitude)) if math.isfinite(magnitude) else 0.0
    if direction == FORWARD:
        return m * max_linear, 0.0
    if direction == REVERSE:
        return -m * max_linear, 0.0
    if direction == ROTATE_LEFT:
        return 0.0, m * max_angular
    if direction == ROTATE_RIGHT:
        return 0.0, -m * max_angular
    return 0.0, 0.0


@dataclass
class DriveCommand:
    direction: str = STOP
    magnitude: float = 0.0
    t: float = -math.inf


class ControlSession:
    def __init__(self, deadman_s: float = 0.3, lease_timeout_s: float = 3.0, deadzone: float = 0.15) -> None:
        if deadman_s <= 0 or lease_timeout_s <= 0:
            raise ValueError("timeouts must be > 0")
        self.deadman_s = deadman_s
        self.lease_timeout_s = lease_timeout_s
        self.deadzone = deadzone
        self.holder: Optional[str] = None
        self._holder_seen = -math.inf
        self._cmd = DriveCommand()

    # ---------------------------------------------------------------- lease

    def claim(self, client: str, now: float) -> tuple[bool, str]:
        if self.holder in (None, client) or now - self._holder_seen > self.lease_timeout_s:
            if self.holder not in (None, client):
                self._cmd = DriveCommand()  # previous holder went silent: never inherit its motion
            self.holder = client
            self._holder_seen = now
            return True, "control granted"
        return False, "another device has control"

    def release(self, client: str) -> bool:
        if self.holder == client:
            self.holder = None
            self._cmd = DriveCommand()
            return True
        return False

    def touch(self, client: str, now: float) -> None:
        if self.holder == client:
            self._holder_seen = now

    def has_control(self, client: str, now: float) -> bool:
        return self.holder == client and now - self._holder_seen <= self.lease_timeout_s

    # ---------------------------------------------------------------- driving

    def drive(self, client: str, x: float, y: float, now: float) -> tuple[bool, str, str]:
        if not self.has_control(client, now):
            self._cmd = DriveCommand() if self.holder == client else self._cmd
            return False, "no control lease", STOP
        self.touch(client, now)
        direction, mag = quantize(x, y, self.deadzone)
        self._cmd = DriveCommand(direction, mag, now)
        return True, "ok", direction

    def stop(self) -> None:
        self._cmd = DriveCommand()

    def current(self, now: float) -> DriveCommand:
        c = self._cmd
        if c.direction == STOP or now - c.t > self.deadman_s:
            return DriveCommand(STOP, 0.0, c.t)
        if self.holder is None or now - self._holder_seen > self.lease_timeout_s:
            return DriveCommand(STOP, 0.0, c.t)
        return c

    def twist(self, now: float, max_linear: float, max_angular: float) -> tuple[float, float]:
        c = self.current(now)
        return to_twist(c.direction, c.magnitude, max_linear, max_angular)
