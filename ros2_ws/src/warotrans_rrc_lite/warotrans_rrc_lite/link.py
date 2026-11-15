"""RRC Lite link: connection management, last-line motor watchdog, firmware capability detection.

Pure Python (no ROS). The ROS node runs `run()` on a thread; tests call `step()` with a fake clock.

Safety rules implemented here (the stock firmware has NO command timeout, see docs/safety.md):
  * on connect: set_motor_type FIRST, then STOP. The stock firmware leaves the motor outputs uninitialised (and
    its PID on JGA27 defaults, negative gains) until the first MOTOR packet; on RBT-002 the rear wheels ran by
    themselves in that state and stopped as soon as a set_motor_type(JGB37) packet arrived (2026-11-15).
    Without a motor_type the link sends no MOTOR packet at all, and the ROS node refuses to start on hardware;
  * nothing but STOP is ever sent unless `motors_allowed` is true;
  * no motion setpoint is sent at connect; a STOP-all is sent on connect (clears a stale setpoint left
    by a previous session) and periodically while idle;
  * setpoints older than `setpoint_timeout_s` are replaced by STOP;
  * any invalid setpoint (NaN, over the limit, wrong length) is rejected and answered with STOP;
  * close() sends STOP before releasing the port.
"""

from __future__ import annotations

import logging
import math
import threading
import time
from dataclasses import dataclass, field, replace
from typing import Callable, Optional, Sequence

from . import protocol as p
from .transport import Transport, TransportError

log = logging.getLogger("warotrans_rrc_lite.link")

PROFILE_UNKNOWN = "unknown"
PROFILE_STOCK = "stock"
PROFILE_PATCHED = "patched"


@dataclass
class LinkConfig:
    rps_limit: float  # host-side clamp; must be <= the motor's real capability (firmware does not clamp)
    motors_allowed: bool = False
    motor_type: Optional[int] = None  # sent on every (re)connect; None = never send ANY motor packet
    setpoint_timeout_s: float = 0.2
    idle_stop_period_s: float = 0.5
    stop_on_connect: bool = True
    query_version: bool = True
    profile_detect_s: float = 2.0
    rx_stale_s: float = 0.5  # link_ok false after this long without a valid frame
    rx_dead_s: float = 3.0  # reconnect after this long without a valid frame
    reconnect_min_s: float = 0.5
    reconnect_max_s: float = 5.0
    read_chunk: int = 512
    read_timeout_s: float = 0.02

    def __post_init__(self) -> None:
        if not (math.isfinite(self.rps_limit) and self.rps_limit > 0):
            raise ValueError("rps_limit must be > 0")
        if self.motor_type is not None:
            p.MotorType(int(self.motor_type))
        for name in ("setpoint_timeout_s", "rx_stale_s", "rx_dead_s", "reconnect_min_s", "reconnect_max_s"):
            if getattr(self, name) <= 0:
                raise ValueError(f"{name} must be > 0")


@dataclass
class LinkStatus:
    connected: bool = False
    link_ok: bool = False  # connected and frames arriving
    port: str = ""
    profile: str = PROFILE_UNKNOWN
    firmware_version: str = ""
    rx_frames: int = 0
    crc_errors: int = 0
    malformed: int = 0
    discarded_bytes: int = 0
    rx_age_s: float = math.inf
    setpoint_age_s: float = math.inf
    watchdog_active: bool = True  # True = motors are being held at STOP by the host watchdog
    watchdog_trips: int = 0
    rejected_setpoints: int = 0
    reconnects: int = 0
    motors_allowed: bool = False
    last_sent_rps: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 0.0)
    last_error: str = ""
    errors: list[str] = field(default_factory=list)


ReportCallback = Callable[[p.Report, float], None]


class RrcLiteLink:
    def __init__(self, transport: Transport, config: LinkConfig, on_report: Optional[ReportCallback] = None,
                 clock: Callable[[], float] = time.monotonic) -> None:
        self._t = transport
        self.cfg = config
        self._on_report = on_report
        self._clock = clock
        self._lock = threading.RLock()  # guards the transport and the state below
        self._parser = p.FrameParser()
        self._connected = False
        self._connected_at = 0.0
        self._next_connect = 0.0
        self._backoff = config.reconnect_min_s
        self._last_rx: Optional[float] = None
        self._last_setpoint_t: Optional[float] = None
        self._setpoint_nonzero = False
        self._stale_handled = True
        self._last_idle_stop = -math.inf
        self._profile = PROFILE_UNKNOWN
        self._status = LinkStatus(motors_allowed=config.motors_allowed, port=transport.description)
        self._stop_event = threading.Event()

    # ------------------------------------------------------------------ public API (thread-safe)

    def set_setpoints(self, rps: Sequence[float]) -> bool:
        """Command all four motors (index = port M1..M4). Returns True if written to the board."""
        now = self._clock()
        with self._lock:
            if not self.cfg.motors_allowed or self.cfg.motor_type is None:
                self._status.rejected_setpoints += 1
                if self.cfg.motor_type is None:
                    self._error("setpoint rejected: motor_type not configured (motor PID never started)")
                return False
            try:
                if len(rps) != p.MOTOR_COUNT:
                    raise p.ProtocolError(f"expected 4 setpoints, got {len(rps)}")
                frame = p.encode_motor_speeds(list(enumerate(float(v) for v in rps)), self.cfg.rps_limit)
            except (p.ProtocolError, TypeError, ValueError) as exc:
                self._status.rejected_setpoints += 1
                self._error(f"setpoint rejected: {exc}")
                self._send_stop("invalid setpoint")
                return False
            if not self._connected:
                return False
            if not self._write(frame):
                return False
            self._last_setpoint_t = now
            self._setpoint_nonzero = any(v != 0.0 for v in rps)
            self._stale_handled = False
            self._status.last_sent_rps = tuple(float(v) for v in rps)  # type: ignore[assignment]
            self._status.watchdog_active = False
            return True

    def stop(self, reason: str = "requested") -> None:
        with self._lock:
            self._last_setpoint_t = None
            self._stale_handled = True
            self._send_stop(reason)

    def set_motors_allowed(self, allowed: bool) -> None:
        with self._lock:
            self.cfg = replace(self.cfg, motors_allowed=bool(allowed))
            self._status.motors_allowed = bool(allowed)
            if not allowed:
                self.stop("motors disallowed")

    def status(self) -> LinkStatus:
        now = self._clock()
        with self._lock:
            s = replace(self._status, errors=list(self._status.errors))
            s.connected = self._connected
            s.rx_age_s = math.inf if self._last_rx is None else now - self._last_rx
            s.link_ok = self._connected and s.rx_age_s <= self.cfg.rx_stale_s
            s.setpoint_age_s = math.inf if self._last_setpoint_t is None else now - self._last_setpoint_t
            s.profile = self._profile
            st = self._parser.stats
            s.crc_errors, s.discarded_bytes = st.crc_errors, st.discarded_bytes
            s.rx_frames = st.frames
            return s

    def run(self) -> None:
        while not self._stop_event.is_set():
            self.step()
            if not self._connected:
                self._stop_event.wait(0.05)

    def shutdown(self) -> None:
        self._stop_event.set()
        with self._lock:
            if self._connected:
                self._send_stop("shutdown")
            self._disconnect("shutdown", count=False)

    # ------------------------------------------------------------------ loop

    def step(self) -> None:
        now = self._clock()
        if not self._connected:
            if now >= self._next_connect:
                self._connect(now)
            return
        try:
            data = self._t.read(self.cfg.read_chunk, self.cfg.read_timeout_s)
        except TransportError as exc:
            with self._lock:
                self._disconnect(f"read: {exc}")
            return
        now = self._clock()
        if data:
            self._process(data, now)
        with self._lock:
            self._watchdog(now)
            self._detect_profile(now)
            if self._last_rx is not None and now - self._last_rx > self.cfg.rx_dead_s:
                self._disconnect(f"no valid frame for {now - self._last_rx:.1f}s")
            elif self._last_rx is None and now - self._connected_at > self.cfg.rx_dead_s:
                self._disconnect("board silent after connect")

    def _connect(self, now: float) -> None:
        with self._lock:
            if self._stop_event.is_set():  # never reopen the port after shutdown()
                return
            try:
                self._t.open()
            except (TransportError, OSError) as exc:
                self._error(f"connect: {exc}")
                self._next_connect = now + self._backoff
                self._backoff = min(self._backoff * 2, self.cfg.reconnect_max_s)
                return
            self._connected = True
            self._connected_at = now
            self._status.port = self._t.description
            self._parser.reset()
            self._last_rx = None
            self._profile = PROFILE_UNKNOWN
            self._status.firmware_version = ""
            self._last_setpoint_t = None
            self._stale_handled = True
            log.info("connected to %s", self._t.description)
            # Motor type FIRST: the first MOTOR packet starts the firmware motor PID; sending the type in that
            # packet means it never runs with the JGA27 defaults (negative gains), then STOP clears setpoints.
            if self.cfg.motor_type is not None:
                if self._write(p.encode_set_motor_type(int(self.cfg.motor_type))):
                    log.info("motor type %s sent (initialises the firmware motor outputs at 0)",
                             p.MotorType(int(self.cfg.motor_type)).name)
            if self.cfg.stop_on_connect:
                self._send_stop("connect")
            if self.cfg.query_version:
                self._write(p.encode_version_query())

    def _disconnect(self, reason: str, count: bool = True) -> None:
        if self._connected:
            log.warning("disconnected: %s", reason)
            self._error(f"disconnected: {reason}")
            if count:
                self._status.reconnects += 1
        self._connected = False
        self._t.close()
        self._status.watchdog_active = True
        self._status.last_sent_rps = (0.0, 0.0, 0.0, 0.0)
        self._last_setpoint_t = None
        self._next_connect = self._clock() + self._backoff
        self._backoff = min(self._backoff * 2, self.cfg.reconnect_max_s)

    def _process(self, data: bytes, now: float) -> None:
        with self._lock:
            frames = self._parser.feed(data)
        for frame in frames:
            try:
                report = p.decode(frame)
            except p.ProtocolError as exc:
                with self._lock:
                    self._status.malformed += 1
                    self._error(f"malformed frame: {exc}")
                continue
            with self._lock:
                self._last_rx = now
                self._backoff = self.cfg.reconnect_min_s
                if isinstance(report, p.FirmwareVersion):
                    self._profile = PROFILE_PATCHED
                    self._status.firmware_version = f"{report.major}.{report.minor} build {report.build}"
                elif isinstance(report, (p.MotorFeedback, p.UltrasonicSample)):
                    self._profile = PROFILE_PATCHED
            if report is not None and self._on_report is not None:
                try:
                    self._on_report(report, now)
                except Exception as exc:  # a consumer bug must not kill the link
                    log.exception("report callback failed")
                    self._error(f"callback: {exc}")

    def _watchdog(self, now: float) -> None:
        if self._last_setpoint_t is not None and now - self._last_setpoint_t > self.cfg.setpoint_timeout_s:
            if not self._stale_handled:
                if self._setpoint_nonzero:
                    self._status.watchdog_trips += 1
                    self._error(f"watchdog: setpoint older than {self.cfg.setpoint_timeout_s:.2f}s, STOP")
                self._stale_handled = True
                self._send_stop("watchdog")
                self._last_setpoint_t = None
            return
        if self._last_setpoint_t is None and self.cfg.idle_stop_period_s > 0:
            if now - self._last_idle_stop >= self.cfg.idle_stop_period_s:
                self._send_stop("idle", quiet=True)

    def _detect_profile(self, now: float) -> None:
        if self._profile == PROFILE_UNKNOWN and self._last_rx is not None:
            if now - self._connected_at >= self.cfg.profile_detect_s:
                self._profile = PROFILE_STOCK

    # ------------------------------------------------------------------ helpers

    def _send_stop(self, reason: str, quiet: bool = False) -> None:
        if not self._connected or self.cfg.motor_type is None:
            return  # any MOTOR packet (even STOP) would start the firmware PID with default gains
        if self._write(p.encode_motor_stop_mask(p.STOP_ALL_MASK)):
            self._last_idle_stop = self._clock()
            self._status.watchdog_active = True
            self._status.last_sent_rps = (0.0, 0.0, 0.0, 0.0)
            if not quiet:
                log.info("STOP sent (%s)", reason)

    def _write(self, frame: bytes) -> bool:
        try:
            self._t.write(frame)
            return True
        except TransportError as exc:
            self._disconnect(f"write: {exc}")
            return False

    def _error(self, message: str) -> None:
        self._status.last_error = message
        self._status.errors.append(message)
        del self._status.errors[:-20]
