"""Drive safety state machine + velocity/acceleration limits. Pure Python, deterministic (caller gives time).

Modes:
  SAFE         default at start and after any fault. No motion setpoints are produced.
  ENABLED      /cmd_vel drives the wheels (forward / reverse / rotate; linear.y is dropped).
  CALIBRATION  one motor port at a time, small speed, bounded duration (hold-to-run).

Units: with measured wheel geometry /cmd_vel is in m/s and rad/s ("mps"). Without geometry (and with
percent_mode_without_geometry) it is a fraction of the wheel speed limit ("percent", like the RBT-001 mobile
control): linear.x and angular.z in -1..1, left wheels = (x - z) * max_wheel_rps, right = (x + z) * max_wheel_rps.

ENABLED / CALIBRATION are entered only by an explicit request and only when there is no blocker.
If a blocker appears while active, the mode drops to SAFE and the reason is latched in `faults`.
Leaving SAFE again requires a new request: nothing re-enables by itself.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Optional

from .kinematics import Geometry, MotorMap, saturate, to_ports, wheel_rps

SAFE, ENABLED, CALIBRATION = 0, 1, 2
MODE_NAMES = {SAFE: "SAFE", ENABLED: "ENABLED", CALIBRATION: "CALIBRATION"}


@dataclass
class ControllerConfig:
    geometry: Optional[Geometry]
    motor_map: Optional[MotorMap]
    max_wheel_rps: float  # must be <= the RRC Lite driver rps_limit
    motors_enabled: bool = False
    max_linear_mps: float = 0.2
    max_angular_radps: float = 0.8
    max_linear_accel: float = 0.4  # m/s^2, both speeding up and slowing down
    max_angular_accel: float = 1.5  # rad/s^2
    cmd_timeout_s: float = 0.3
    battery_min_v: Optional[float] = None
    battery_max_v: Optional[float] = None
    battery_stale_s: float = 3.0
    link_stale_s: float = 1.0
    calibration_max_rps: float = 0.3
    calibration_max_duration_ms: int = 3000
    default_speed_scale: float = 0.5
    percent_mode_without_geometry: bool = True
    percent_accel_per_s: float = 1.5  # fraction of full speed per second (percent mode)

    def __post_init__(self) -> None:
        for n in ("max_wheel_rps", "max_linear_mps", "max_angular_radps", "max_linear_accel",
                  "max_angular_accel", "cmd_timeout_s", "calibration_max_rps"):
            v = getattr(self, n)
            if not (math.isfinite(v) and v > 0):
                raise ValueError(f"{n} must be > 0")
        if self.calibration_max_rps > self.max_wheel_rps:
            raise ValueError("calibration_max_rps must be <= max_wheel_rps")
        if not 0.0 <= self.default_speed_scale <= 1.0:
            raise ValueError("default_speed_scale must be in 0..1")


@dataclass
class LinkView:
    link_ok: bool = False
    t: float = -math.inf  # when the status was received


@dataclass
class State:
    units: str
    mode: int
    mode_name: str
    blockers: list[str]
    faults: list[str]
    link_ok: bool
    battery_ok: bool
    battery_voltage: float
    battery_age_s: float
    motors_enabled_param: bool
    cmd_age_s: float
    max_linear_mps: float
    max_angular_radps: float
    speed_scale: float
    lateral_rejected: int
    cmd_linear_mps: float
    cmd_angular_radps: float
    calibration_motor: int
    calibration_rps: float
    setpoints: list[float] = field(default_factory=lambda: [0.0] * 4)


class BaseController:
    def __init__(self, config: ControllerConfig) -> None:
        self.cfg = config
        self.mode = SAFE
        self.faults: list[str] = []
        self.speed_scale = config.default_speed_scale
        self.lateral_rejected = 0
        self.invalid_cmds = 0
        self._cmd: tuple[float, float] = (0.0, 0.0)
        self._cmd_t = -math.inf
        self._out = (0.0, 0.0)  # rate-limited body command
        self._last_step: Optional[float] = None
        self._link = LinkView()
        self._battery_v: Optional[float] = None
        self._battery_t = -math.inf
        self._cmd_publishers_ok = True
        self._calib: Optional[tuple[int, float, float]] = None  # port, rps, until
        self._setpoints = [0.0] * 4
        self._sent_stop_after_safe = False

    # ------------------------------------------------------------------ inputs

    def on_cmd_vel(self, vx: float, vy: float, wz: float, now: float) -> None:
        if not all(math.isfinite(v) for v in (vx, vy, wz)):
            self.invalid_cmds += 1
            self._cmd, self._cmd_t = (0.0, 0.0), now
            return
        if abs(vy) > 1e-6:
            self.lateral_rejected += 1  # mecanum could strafe; RBT-002 must not
        self._cmd, self._cmd_t = (vx, wz), now

    def on_link(self, link_ok: bool, now: float) -> None:
        self._link = LinkView(link_ok, now)

    def on_battery(self, volts: float, now: float) -> None:
        if math.isfinite(volts):
            self._battery_v, self._battery_t = volts, now

    def set_cmd_publishers_ok(self, ok: bool) -> None:
        self._cmd_publishers_ok = ok

    def set_speed_scale(self, scale: float) -> tuple[bool, str]:
        if not (math.isfinite(scale) and 0.0 <= scale <= 1.0):
            return False, "scale must be within 0..1"
        self.speed_scale = scale
        return True, f"speed scale {scale:.2f}"

    def request_mode(self, mode: int, now: float) -> tuple[bool, str]:
        if mode == SAFE:
            self._enter_safe()
            self.faults = []
            return True, "SAFE"
        if mode not in (ENABLED, CALIBRATION):
            return False, f"unknown mode {mode}"
        blockers = self.blockers(mode, now)
        if blockers:
            return False, "refused: " + "; ".join(blockers)
        self._enter_safe()  # reset outputs and calibration before switching
        self.mode = mode
        self.faults = []
        self._sent_stop_after_safe = False
        return True, MODE_NAMES[mode]

    def calibrate(self, port: int, rps: float, duration_ms: int, now: float) -> tuple[bool, str]:
        if self.mode != CALIBRATION:
            return False, "not in CALIBRATION mode"
        if port not in (1, 2, 3, 4):
            return False, "motor must be 1..4"
        if not math.isfinite(rps) or abs(rps) > self.cfg.calibration_max_rps:
            return False, f"|rps| must be <= {self.cfg.calibration_max_rps}"
        if not 0 < duration_ms <= self.cfg.calibration_max_duration_ms:
            return False, f"duration_ms must be 1..{self.cfg.calibration_max_duration_ms}"
        if rps == 0.0:
            self._calib = None
            return True, "stopped"
        if self._calib is not None and self._calib[0] != port and now < self._calib[2]:
            return False, f"motor {self._calib[0]} is running; stop it first"
        self._calib = (port, float(rps), now + duration_ms / 1000.0)
        return True, f"motor {port} at {rps:+.3f} rps for {duration_ms} ms"

    # ------------------------------------------------------------------ evaluation

    def battery_ok(self, now: float) -> tuple[bool, str]:
        c = self.cfg
        if c.battery_min_v is None or c.battery_max_v is None:
            return False, "battery thresholds not configured (CHƯA XÁC NHẬN)"
        if self._battery_v is None:
            return False, "no battery report"
        if now - self._battery_t > c.battery_stale_s:
            return False, f"battery report stale ({now - self._battery_t:.1f}s)"
        if not c.battery_min_v <= self._battery_v <= c.battery_max_v:
            return False, f"battery {self._battery_v:.2f} V outside {c.battery_min_v}..{c.battery_max_v} V"
        return True, ""

    @property
    def units(self) -> str:
        if self.cfg.geometry is not None:
            return "mps"
        return "percent" if self.cfg.percent_mode_without_geometry else "none"

    def _max_speeds(self) -> tuple[float, float]:
        if self.units == "percent":
            return 1.0, 1.0
        return self.cfg.max_linear_mps, self.cfg.max_angular_radps

    def update_calibration(self, geometry: Optional[Geometry], motor_map: Optional[MotorMap]) -> tuple[bool, str]:
        """Runtime update of geometry / motor map (from the UI). Refused while ENABLED."""
        if self.mode == ENABLED:
            return False, "switch to SAFE or CALIBRATION first"
        from dataclasses import replace as _replace
        self.cfg = _replace(self.cfg, geometry=geometry, motor_map=motor_map)
        return True, "calibration updated"

    def link_ok(self, now: float) -> bool:
        return self._link.link_ok and now - self._link.t <= self.cfg.link_stale_s

    def blockers(self, mode: int, now: float) -> list[str]:
        b: list[str] = []
        if not self.cfg.motors_enabled:
            b.append("motors_enabled=false")
        if not self.link_ok(now):
            b.append("RRC Lite link down")
        ok, why = self.battery_ok(now)
        if not ok:
            b.append(why)
        if mode == ENABLED:
            if self.units == "none":
                b.append("wheel geometry not configured (CHƯA XÁC NHẬN)")
            if self.cfg.motor_map is None:
                b.append("motor port/sign map not calibrated (CHƯA XÁC NHẬN)")
            if not self._cmd_publishers_ok:
                b.append("foreign /cmd_vel publisher")
        return b

    def step(self, now: float) -> Optional[list[float]]:
        """Advance one control tick. Returns setpoints to publish (index = port M1..M4) or None for none."""
        dt = 0.0 if self._last_step is None else max(0.0, min(0.2, now - self._last_step))
        self._last_step = now

        if self.mode != SAFE:
            blockers = self.blockers(self.mode, now)
            if blockers:
                self._enter_safe()
                self.faults = ["forced SAFE"] + blockers

        if self.mode == SAFE:
            if not self._sent_stop_after_safe:
                self._sent_stop_after_safe = True
                return [0.0] * 4  # one explicit zero; the driver watchdog then holds STOP
            return None

        if self.mode == CALIBRATION:
            out = [0.0] * 4
            if self._calib is not None:
                port, rps, until = self._calib
                if now < until:
                    out[port - 1] = rps
                else:
                    self._calib = None
            self._setpoints = out
            return out

        # ENABLED
        c = self.cfg
        vx, wz = self._cmd if now - self._cmd_t <= c.cmd_timeout_s else (0.0, 0.0)
        max_v, max_w = self._max_speeds()
        lim_v = max_v * self.speed_scale
        lim_w = max_w * self.speed_scale
        vx = max(-lim_v, min(lim_v, vx))
        wz = max(-lim_w, min(lim_w, wz))
        percent = self.units == "percent"
        acc_v = c.percent_accel_per_s if percent else c.max_linear_accel
        acc_w = c.percent_accel_per_s if percent else c.max_angular_accel
        ov, ow = self._out
        ov += max(-acc_v * dt, min(acc_v * dt, vx - ov))
        ow += max(-acc_w * dt, min(acc_w * dt, wz - ow))
        assert c.motor_map is not None
        if percent:
            left, right = (ov - ow) * c.max_wheel_rps, (ov + ow) * c.max_wheel_rps
            wheels = {"FL": left, "RL": left, "FR": right, "RR": right}
        else:
            assert c.geometry is not None
            wheels = wheel_rps(ov, ow, c.geometry)
        ports = to_ports(wheels, c.motor_map)
        ports, factor = saturate(ports, c.max_wheel_rps)
        self._out = (ov * factor, ow * factor)
        self._setpoints = ports
        return ports

    def state(self, now: float) -> State:
        mode_for_blockers = self.mode if self.mode != SAFE else ENABLED
        ok, _ = self.battery_ok(now)
        cal_port, cal_rps = (self._calib[0], self._calib[1]) if self._calib and now < self._calib[2] else (0, 0.0)
        max_v, max_w = self._max_speeds()
        return State(
            units=self.units, mode=self.mode, mode_name=MODE_NAMES[self.mode], blockers=self.blockers(mode_for_blockers, now),
            faults=list(self.faults), link_ok=self.link_ok(now), battery_ok=ok,
            battery_voltage=self._battery_v if self._battery_v is not None else math.nan,
            battery_age_s=now - self._battery_t, motors_enabled_param=self.cfg.motors_enabled,
            cmd_age_s=now - self._cmd_t, max_linear_mps=max_v * self.speed_scale,
            max_angular_radps=max_w * self.speed_scale, speed_scale=self.speed_scale,
            lateral_rejected=self.lateral_rejected, cmd_linear_mps=self._out[0], cmd_angular_radps=self._out[1],
            calibration_motor=cal_port, calibration_rps=cal_rps,
            setpoints=list(self._setpoints) if self.mode != SAFE else [0.0] * 4)

    # ------------------------------------------------------------------ internals

    def _enter_safe(self) -> None:
        self.mode = SAFE
        self._out = (0.0, 0.0)
        self._calib = None
        self._setpoints = [0.0] * 4
        self._sent_stop_after_safe = False
