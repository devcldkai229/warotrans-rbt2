"""Base controller: /cmd_vel -> limited, mecanum-mapped wheel setpoints for the RRC Lite driver.

Subscribes  /cmd_vel (geometry_msgs/Twist), /rrc_lite/status, /rrc_lite/battery, /rrc_lite/encoders
Publishes   /rrc_lite/motor_setpoints (only in ENABLED / CALIBRATION), /safety_state, /motor_state, /diagnostics
Services    ~/set_mode, ~/calibrate_motor, ~/set_speed_limit
"""

from __future__ import annotations

import math
import time
from typing import Optional

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import SetParametersResult
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from warotrans_rbt002_msgs.msg import EncoderFeedback, MotorSetpoints, MotorState, RrcLiteStatus, SafetyState
from warotrans_rbt002_msgs.srv import CalibrateMotor, SetMode, SetSpeedLimit

from .controller import BaseController, ControllerConfig
from .kinematics import WHEELS, parse_geometry, parse_motor_map
from .params import (GEOMETRY_PARAMS, MAP_PARAMS, declare_geometry_and_map, positive_or_none, read_geometry,
                     read_motor_map)


def _f(x: float) -> float:
    return float(x) if math.isfinite(x) else float("inf")


class BaseControllerNode(Node):
    def __init__(self) -> None:
        super().__init__("base_controller")
        declare_geometry_and_map(self)
        d = self.declare_parameter
        d("motors_enabled", False)
        d("max_wheel_rps", 0.5)
        d("max_linear_mps", 0.2)
        d("max_angular_radps", 0.8)
        d("max_linear_accel", 0.4)
        d("max_angular_accel", 1.5)
        d("cmd_timeout_s", 0.3)
        d("battery_min_v", 0.0)
        d("battery_max_v", 0.0)
        d("battery_stale_s", 3.0)
        d("calibration_max_rps", 0.3)
        d("calibration_max_duration_ms", 3000)
        d("default_speed_scale", 0.5)
        d("control_rate_hz", 50.0)
        d("cmd_vel_topic", "/cmd_vel")
        d("required_cmd_vel_publisher", "")
        d("feedback_timeout_s", 0.2)
        d("percent_mode_without_geometry", True)
        d("percent_accel_per_s", 1.5)
        d("auto_enable", False)
        d("auto_enable_stable_s", 2.0)
        g = lambda n: self.get_parameter(n).value  # noqa: E731

        geometry = read_geometry(self)
        motor_map = read_motor_map(self)
        self._ctl = BaseController(ControllerConfig(
            geometry=geometry, motor_map=motor_map, max_wheel_rps=float(g("max_wheel_rps")),
            motors_enabled=bool(g("motors_enabled")), max_linear_mps=float(g("max_linear_mps")),
            max_angular_radps=float(g("max_angular_radps")), max_linear_accel=float(g("max_linear_accel")),
            max_angular_accel=float(g("max_angular_accel")), cmd_timeout_s=float(g("cmd_timeout_s")),
            battery_min_v=positive_or_none(g("battery_min_v")), battery_max_v=positive_or_none(g("battery_max_v")),
            battery_stale_s=float(g("battery_stale_s")), calibration_max_rps=float(g("calibration_max_rps")),
            calibration_max_duration_ms=int(g("calibration_max_duration_ms")),
            default_speed_scale=float(g("default_speed_scale")),
            percent_mode_without_geometry=bool(g("percent_mode_without_geometry")),
            percent_accel_per_s=float(g("percent_accel_per_s")),
            auto_enable=bool(g("auto_enable")), auto_enable_stable_s=float(g("auto_enable_stable_s"))))
        self._wheel_names = motor_map.wheel_of_port() if motor_map else ["?"] * 4
        if geometry is None:
            self.get_logger().warning(f"wheel geometry not configured (CHƯA XÁC NHẬN): drive units = {self._ctl.units}")
        if motor_map is None:
            self.get_logger().warning("motor port/sign map not calibrated (CHƯA XÁC NHẬN): ENABLED is blocked")
        self._feedback_timeout = float(g("feedback_timeout_s"))
        self._feedback: Optional[EncoderFeedback] = None
        self._feedback_t = -math.inf

        self._pub_sp = self.create_publisher(MotorSetpoints, "/rrc_lite/motor_setpoints", 1)
        self._pub_safety = self.create_publisher(SafetyState, "/safety_state", 5)
        self._pub_motor = self.create_publisher(MotorState, "/motor_state", 5)
        self._pub_diag = self.create_publisher(DiagnosticArray, "/diagnostics", 5)

        self._cmd_topic = self.resolve_topic_name(str(g("cmd_vel_topic")))
        self._required_pub = str(g("required_cmd_vel_publisher")).strip()
        self.create_subscription(Twist, self._cmd_topic, self._on_cmd, 5)
        self.create_subscription(RrcLiteStatus, "/rrc_lite/status", self._on_status, 5)
        self.create_subscription(BatteryState, "/rrc_lite/battery", self._on_battery, 5)
        self.create_subscription(EncoderFeedback, "/rrc_lite/encoders", self._on_feedback, 10)
        self.create_service(SetMode, "~/set_mode", self._srv_mode)
        self.create_service(CalibrateMotor, "~/calibrate_motor", self._srv_calibrate)
        self.create_service(SetSpeedLimit, "~/set_speed_limit", self._srv_speed)
        if self._required_pub:
            self._ctl.set_cmd_publishers_ok(False)
            self.create_timer(0.5, self._check_publishers)
        self.add_on_set_parameters_callback(self._on_params)
        self.create_timer(1.0 / float(g("control_rate_hz")), self._tick)
        self.create_timer(0.1, self._publish_state)
        self.create_timer(1.0, self._publish_diag)

    def _on_params(self, params) -> SetParametersResult:
        """Runtime update of motor map / geometry (UI calibration). Refused while ENABLED."""
        watched = set(GEOMETRY_PARAMS) | set(MAP_PARAMS)
        changed = {p.name: p.value for p in params if p.name in watched}
        if not changed:
            return SetParametersResult(successful=True)
        cur = {n: self.get_parameter(n).value for n in watched}
        cur.update(changed)
        try:
            signs = [int(s) for s in cur["port_signs"]]
            if len(signs) != 4:
                raise ValueError("port_signs must have 4 entries")
            motor_map = parse_motor_map({w: int(cur[f"port_{w.lower()}"]) for w in WHEELS}, signs)
            geometry = parse_geometry(float(cur["wheel_radius_m"]), float(cur["half_wheelbase_m"]),
                                      float(cur["half_track_m"]))
        except (ValueError, TypeError) as exc:
            return SetParametersResult(successful=False, reason=str(exc))
        ok, msg = self._ctl.update_calibration(geometry, motor_map)
        if ok:
            self._wheel_names = motor_map.wheel_of_port() if motor_map else ["?"] * 4
            self.get_logger().info(f"calibration updated: map={motor_map} geometry={geometry}")
        return SetParametersResult(successful=ok, reason=msg)

    @staticmethod
    def _now() -> float:
        return time.monotonic()

    def _check_publishers(self) -> None:
        names = [i.node_name for i in self.get_publishers_info_by_topic(self._cmd_topic)]
        if "_NODE_NAME_UNKNOWN_" in names:
            return  # discovery not finished; decide on the next check
        ok = all(n == self._required_pub for n in names)
        self._ctl.set_cmd_publishers_ok(ok)

    def _on_cmd(self, m: Twist) -> None:
        self._ctl.on_cmd_vel(m.linear.x, m.linear.y, m.angular.z, self._now())

    def _on_status(self, m: RrcLiteStatus) -> None:
        self._ctl.on_link(bool(m.link_ok), self._now())

    def _on_battery(self, m: BatteryState) -> None:
        self._ctl.on_battery(float(m.voltage), self._now())

    def _on_feedback(self, m: EncoderFeedback) -> None:
        self._feedback, self._feedback_t = m, self._now()

    def _tick(self) -> None:
        out = self._ctl.step(self._now())
        if out is not None:
            m = MotorSetpoints()
            m.header.stamp = self.get_clock().now().to_msg()
            m.rps = [float(v) for v in out]
            self._pub_sp.publish(m)

    def _srv_mode(self, req, res):
        ok, msg = self._ctl.request_mode(int(req.mode), self._now())
        res.success, res.message = ok, msg
        # rclpy forbids one call site from logging at different severities: keep two separate calls.
        if ok:
            self.get_logger().info(f"set_mode {req.mode}: {msg}")
        else:
            self.get_logger().warning(f"set_mode {req.mode}: {msg}")
        return res

    def _srv_calibrate(self, req, res):
        res.success, res.message = self._ctl.calibrate(int(req.motor), float(req.rps), int(req.duration_ms),
                                                       self._now())
        return res

    def _srv_speed(self, req, res):
        res.success, res.message = self._ctl.set_speed_scale(float(req.scale))
        res.scale = float(self._ctl.speed_scale)
        return res

    def _publish_state(self) -> None:
        now = self._now()
        s = self._ctl.state(now)
        stamp = self.get_clock().now().to_msg()
        m = SafetyState()
        m.header.stamp = stamp
        m.units = s.units
        m.mode, m.mode_name, m.blockers, m.faults = s.mode, s.mode_name, s.blockers, s.faults
        m.link_ok, m.battery_ok, m.battery_voltage = s.link_ok, s.battery_ok, float(s.battery_voltage)
        m.battery_age_s, m.motors_enabled_param, m.cmd_age_s = _f(s.battery_age_s), s.motors_enabled_param, _f(s.cmd_age_s)
        m.max_linear_mps, m.max_angular_radps, m.speed_scale = s.max_linear_mps, s.max_angular_radps, s.speed_scale
        m.lateral_rejected = s.lateral_rejected
        m.cmd_linear_mps, m.cmd_angular_radps = s.cmd_linear_mps, s.cmd_angular_radps
        m.calibration_motor, m.calibration_rps = s.calibration_motor, s.calibration_rps
        self._pub_safety.publish(m)

        ms = MotorState()
        ms.header.stamp = stamp
        ms.wheel = self._wheel_names
        ms.setpoint_rps = [float(v) for v in s.setpoints]
        age = now - self._feedback_t
        valid = self._feedback is not None and age <= self._feedback_timeout
        ms.feedback_valid = valid
        ms.feedback_age_s = _f(age)
        if valid:
            ms.measured_rps = [float(v) for v in self._feedback.rps]
            ms.encoder_counts = [int(v) for v in self._feedback.counts]
        else:
            ms.measured_rps = [float("nan")] * 4
            ms.encoder_counts = [0] * 4
        self._pub_motor.publish(ms)

    def _publish_diag(self) -> None:
        s = self._ctl.state(self._now())
        d = DiagnosticStatus(name="base_controller: mode", hardware_id="RBT-002")
        d.level = DiagnosticStatus.WARN if s.faults else DiagnosticStatus.OK
        d.message = s.mode_name + (f" (faults: {'; '.join(s.faults)})" if s.faults else "")
        d.values = [KeyValue(key="blockers", value="; ".join(s.blockers) or "none"),
                    KeyValue(key="speed_scale", value=f"{s.speed_scale:.2f}"),
                    KeyValue(key="lateral_rejected", value=str(s.lateral_rejected))]
        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        arr.status = [d]
        self._pub_diag.publish(arr)

    def destroy_node(self) -> bool:
        try:
            self._ctl.request_mode(0, self._now())
            m = MotorSetpoints()
            m.rps = [0.0] * 4
            self._pub_sp.publish(m)
        except Exception:
            pass
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = BaseControllerNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
