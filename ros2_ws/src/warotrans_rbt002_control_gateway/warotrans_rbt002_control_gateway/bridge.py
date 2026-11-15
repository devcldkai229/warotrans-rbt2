"""ROS 2 side of the control gateway (node name `control_gateway`).

Publishes /cmd_vel (base_controller only accepts it from this node). Calls base_controller services.
Aggregates telemetry for the UI from /safety_state, /motor_state, /rrc_lite/status, /rrc_lite/battery,
/battery/estimate, /imu/data, /ultrasonic/front, /diagnostics.
"""

from __future__ import annotations

import math
import os
import threading
import time
from typing import Any, Optional

import yaml
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters

from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import BatteryState, Imu, Range
from warotrans_msgs.msg import BatteryEstimate
from warotrans_rbt002_msgs.msg import MotorState, RrcLiteStatus, SafetyState
from warotrans_rbt002_msgs.srv import CalibrateMotor, SetMode, SetSpeedLimit

LEVELS = {0: "OK", 1: "WARN", 2: "ERROR", 3: "STALE"}


class RosBridge(Node):
    def __init__(self) -> None:
        super().__init__("control_gateway")
        self.declare_parameter("base_controller", "/base_controller")
        self.declare_parameter("service_timeout_s", 1.0)
        self.declare_parameter("calibration_file", "~/.config/warotrans/rbt002_calibration.yaml")
        base = str(self.get_parameter("base_controller").value).rstrip("/")
        self._cal_file = os.path.expanduser(str(self.get_parameter("calibration_file").value))
        self._timeout = float(self.get_parameter("service_timeout_s").value)
        self._lock = threading.Lock()
        self._data: dict[str, Any] = {}
        self._stamp: dict[str, float] = {}
        self._diag: dict[str, dict[str, Any]] = {}
        self._imu_times: list[float] = []

        self._pub = self.create_publisher(Twist, "/cmd_vel", 5)
        self._cli_mode = self.create_client(SetMode, f"{base}/set_mode")
        self._cli_cal = self.create_client(CalibrateMotor, f"{base}/calibrate_motor")
        self._cli_speed = self.create_client(SetSpeedLimit, f"{base}/set_speed_limit")
        self._cli_params = self.create_client(SetParameters, f"{base}/set_parameters")
        self._calibration = self._load_calibration()
        self.create_subscription(SafetyState, "/safety_state", self._on_safety, 5)
        self.create_subscription(MotorState, "/motor_state", self._on_motor, 5)
        self.create_subscription(RrcLiteStatus, "/rrc_lite/status", self._on_link, 5)
        self.create_subscription(BatteryState, "/rrc_lite/battery", self._on_batt_raw, 5)
        self.create_subscription(BatteryEstimate, "/battery/estimate", self._on_batt_est, 5)
        self.create_subscription(Imu, "/imu/data", self._on_imu, 20)
        self.create_subscription(Range, "/ultrasonic/front", self._on_us, 10)
        self.create_subscription(DiagnosticArray, "/diagnostics", self._on_diag, 20)

    # ------------------------------------------------------------------ Bridge protocol

    def publish_twist(self, vx: float, wz: float) -> None:
        t = Twist()
        t.linear.x, t.angular.z = float(vx), float(wz)
        self._pub.publish(t)

    def set_mode(self, mode: int) -> tuple[bool, str]:
        req = SetMode.Request()
        req.mode = int(mode)
        return self._call(self._cli_mode, req)

    def calibrate(self, motor: int, rps: float, duration_ms: int) -> tuple[bool, str]:
        req = CalibrateMotor.Request()
        req.motor, req.rps, req.duration_ms = int(motor), float(rps), int(duration_ms)
        return self._call(self._cli_cal, req)

    def set_speed_limit(self, scale: float) -> tuple[bool, str]:
        req = SetSpeedLimit.Request()
        req.scale = float(scale)
        return self._call(self._cli_speed, req)

    def save_motor_map(self, ports: dict[str, int], signs: list[int]) -> tuple[bool, str]:
        """Apply the motor port/sign map to base_controller at runtime, then persist it for the next launch."""
        wheels = ("FL", "FR", "RL", "RR")
        try:
            ports = {w: int(ports[w]) for w in wheels}
            signs = [int(s) for s in signs]
        except (KeyError, TypeError, ValueError):
            return False, "need ports FL/FR/RL/RR and 4 signs"
        if sorted(ports.values()) != [1, 2, 3, 4] or len(signs) != 4 or any(s not in (1, -1) for s in signs):
            return False, "each port 1..4 exactly once, signs +1/-1"
        req = SetParameters.Request()
        for w in wheels:
            req.parameters.append(Parameter(name=f"port_{w.lower()}", value=ParameterValue(
                type=ParameterType.PARAMETER_INTEGER, integer_value=ports[w])))
        req.parameters.append(Parameter(name="port_signs", value=ParameterValue(
            type=ParameterType.PARAMETER_INTEGER_ARRAY, integer_array_value=signs)))
        if not self._cli_params.wait_for_service(timeout_sec=self._timeout):
            return False, "base_controller parameter service not available"
        future = self._cli_params.call_async(req)
        done = threading.Event()
        future.add_done_callback(lambda _f: done.set())
        if not done.wait(self._timeout):
            return False, "set_parameters timed out"
        results = future.result().results
        bad = [r.reason for r in results if not r.successful]
        if bad:
            return False, "; ".join(bad)
        data = {"/**": {"ros__parameters": {**{f"port_{w.lower()}": ports[w] for w in wheels}, "port_signs": signs}}}
        try:
            os.makedirs(os.path.dirname(self._cal_file), exist_ok=True)
            tmp = self._cal_file + ".tmp"
            with open(tmp, "w", encoding="utf-8") as f:
                f.write("# Written by the RBT-002 control UI " + time.strftime("%Y-%m-%d %H:%M:%S") + "\n")
                yaml.safe_dump(data, f, sort_keys=False)
                f.flush()
                os.fsync(f.fileno())
            os.replace(tmp, self._cal_file)
        except OSError as exc:
            return False, f"applied, but not saved: {exc}"
        self._calibration = self._load_calibration()
        return True, f"motor map saved to {self._cal_file}"

    def _load_calibration(self) -> Optional[dict[str, Any]]:
        try:
            with open(self._cal_file, encoding="utf-8") as f:
                return (yaml.safe_load(f) or {}).get("/**", {}).get("ros__parameters")
        except (OSError, yaml.YAMLError, AttributeError):
            return None

    def limits(self) -> tuple[float, float]:
        with self._lock:
            s = self._data.get("safety")
            fresh = s is not None and time.monotonic() - self._stamp.get("safety", -math.inf) < 1.0
        if not fresh:
            return 0.0, 0.0  # no safety state -> no motion request
        return float(s["max_linear_mps"]), float(s["max_angular_radps"])

    def snapshot(self) -> dict[str, Any]:
        now = time.monotonic()
        with self._lock:
            out = {k: v for k, v in self._data.items()}
            out["age_s"] = {k: now - t for k, t in self._stamp.items()}
            out["calibration"] = {"file": self._cal_file, "saved": self._calibration}
            out["diagnostics"] = sorted(self._diag.values(), key=lambda d: (-d["level_num"], d["name"]))
            recent = [t for t in self._imu_times if now - t < 2.0]
            if "imu" in out:
                out["imu"] = dict(out["imu"], rate_hz=len(recent) / 2.0)
        return out

    # ------------------------------------------------------------------ internals

    def _call(self, client, req) -> tuple[bool, str]:
        if not client.wait_for_service(timeout_sec=self._timeout):
            return False, f"service {client.srv_name} not available"
        future = client.call_async(req)
        done = threading.Event()
        future.add_done_callback(lambda _f: done.set())
        if not done.wait(self._timeout):
            return False, f"service {client.srv_name} timed out"
        res = future.result()
        return bool(res.success), str(res.message)

    def _put(self, key: str, value: dict[str, Any]) -> None:
        with self._lock:
            self._data[key] = value
            self._stamp[key] = time.monotonic()

    def _on_safety(self, m: SafetyState) -> None:
        self._put("safety", {
            "units": m.units, "mode": m.mode_name, "blockers": list(m.blockers), "faults": list(m.faults), "link_ok": m.link_ok,
            "battery_ok": m.battery_ok, "battery_voltage": m.battery_voltage, "motors_enabled_param": m.motors_enabled_param,
            "cmd_age_s": m.cmd_age_s, "max_linear_mps": m.max_linear_mps, "max_angular_radps": m.max_angular_radps,
            "speed_scale": m.speed_scale, "lateral_rejected": m.lateral_rejected, "cmd_linear_mps": m.cmd_linear_mps,
            "cmd_angular_radps": m.cmd_angular_radps, "calibration_motor": m.calibration_motor,
            "calibration_rps": m.calibration_rps})

    def _on_motor(self, m: MotorState) -> None:
        self._put("motors", {
            "wheel": list(m.wheel), "setpoint_rps": _floats(m.setpoint_rps), "feedback_valid": m.feedback_valid,
            "measured_rps": _floats(m.measured_rps), "encoder_counts": [int(c) for c in m.encoder_counts],
            "feedback_age_s": float(m.feedback_age_s)})

    def _on_link(self, m: RrcLiteStatus) -> None:
        self._put("link", {
            "connected": m.connected, "link_ok": m.link_ok, "port": m.port, "firmware_profile": m.firmware_profile,
            "firmware_version": m.firmware_version, "rx_frames": int(m.rx_frames), "crc_errors": int(m.crc_errors),
            "malformed_frames": int(m.malformed_frames), "rx_age_s": m.rx_age_s, "imu_rate_hz": m.imu_rate_hz,
            "setpoint_age_s": m.setpoint_age_s, "watchdog_active": m.watchdog_active,
            "watchdog_trips": int(m.watchdog_trips), "rejected_setpoints": int(m.rejected_setpoints),
            "reconnects": int(m.reconnects), "motors_allowed": m.motors_allowed,
            "encoder_feedback": m.encoder_feedback, "ultrasonic_available": m.ultrasonic_available,
            "last_sent_rps": _floats(m.last_sent_rps), "last_error": m.last_error})

    def _on_batt_raw(self, m: BatteryState) -> None:
        cur = dict(self._data.get("battery", {}))
        cur["raw_voltage"] = m.voltage
        self._put("battery", cur)

    def _on_batt_est(self, m: BatteryEstimate) -> None:
        cur = dict(self._data.get("battery", {}))
        cur.update({"filtered_voltage": m.voltage, "percentage": m.percentage, "estimate_mode": m.mode,
                    "estimate_fault": m.fault})
        self._put("battery", cur)

    def _on_imu(self, m: Imu) -> None:
        now = time.monotonic()
        with self._lock:
            self._imu_times.append(now)
            self._imu_times = [t for t in self._imu_times if now - t < 2.0]
        a, w = m.linear_acceleration, m.angular_velocity
        self._put("imu", {"accel": [a.x, a.y, a.z], "gyro": [w.x, w.y, w.z]})

    def _on_us(self, m: Range) -> None:
        self._put("ultrasonic", {"range_m": m.range, "min_range_m": m.min_range, "max_range_m": m.max_range})

    def _on_diag(self, m: DiagnosticArray) -> None:
        with self._lock:
            for s in m.status:
                level = _level(s.level)
                self._diag[s.name] = {"name": s.name, "level": LEVELS.get(level, "?"), "level_num": level,
                                      "message": s.message}


def _floats(values) -> list[float]:
    """Fixed-size ROS arrays arrive as numpy arrays; JSON needs plain Python floats."""
    return [float(v) for v in values]


def _level(value) -> int:
    """diagnostic_msgs level is a ROS `byte`: bytes of length 1 in rclpy."""
    return int.from_bytes(value, "little") if isinstance(value, (bytes, bytearray)) else int(value)
