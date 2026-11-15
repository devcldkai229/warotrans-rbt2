"""ROS 2 node that owns the RRC Lite serial port. The ONLY process allowed to talk to the board.

Subscribes  /rrc_lite/motor_setpoints   warotrans_rbt002_msgs/MotorSetpoints  (from base_controller only)
Publishes   /rrc_lite/imu_raw           warotrans_rbt002_msgs/ImuRaw          (firmware units)
            /rrc_lite/battery           sensor_msgs/BatteryState              (pack voltage only)
            /rrc_lite/encoders          warotrans_rbt002_msgs/EncoderFeedback (patched firmware only)
            /rrc_lite/ultrasonic        warotrans_rbt002_msgs/UltrasonicRaw   (patched firmware only)
            /rrc_lite/status            warotrans_rbt002_msgs/RrcLiteStatus
            /diagnostics                diagnostic_msgs/DiagnosticArray
"""

from __future__ import annotations

import logging
import math
import threading
import time
from collections import deque
from typing import Optional

import rclpy
from rclpy.executors import ExternalShutdownException
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import BatteryState
from warotrans_rbt002_msgs.msg import EncoderFeedback, ImuRaw, MotorSetpoints, RrcLiteStatus, UltrasonicRaw

from . import protocol as p
from .link import PROFILE_PATCHED, PROFILE_STOCK, LinkConfig, RrcLiteLink
from .mock_board import MockBoard, MockConfig
from .transport import DEFAULT_PORT_GLOB, make_transport

OK, WARN, ERROR = DiagnosticStatus.OK, DiagnosticStatus.WARN, DiagnosticStatus.ERROR


def _f(x: float) -> float:
    return float(x) if math.isfinite(x) else float("inf")


class RrcLiteNode(Node):
    def __init__(self) -> None:
        super().__init__("rrc_lite")
        d = self.declare_parameter
        d("transport", "serial")  # serial | mock
        d("port", DEFAULT_PORT_GLOB)
        d("baud", p.BAUD_RATE)
        d("motors_enabled", False)
        d("max_wheel_rps", 0.5)  # host clamp; the firmware serial path does not clamp
        d("motor_type", -1)  # -1 = do not send; 0 JGB520, 1 JGB37, 2 JGA27, 3 JGB528
        d("setpoint_timeout_s", 0.2)
        d("idle_stop_period_s", 0.5)
        d("stop_on_connect", True)
        d("query_version", True)
        d("required_setpoint_publisher", "base_controller")
        d("status_rate_hz", 5.0)
        d("mock_profile", "stock")
        d("mock_battery_mv", 12000)
        d("mock_zero_imu_probability", 0.01)
        g = lambda n: self.get_parameter(n).value  # noqa: E731

        motor_type = int(g("motor_type"))
        if motor_type < 0 and str(g("transport")) == "serial":
            raise ValueError("motor_type must be set (0..3): the stock RRC Lite firmware leaves the motor outputs "
                             "uninitialised until the first MOTOR packet; on RBT-002 the rear wheels ran away in that "
                             "state (docs/safety.md)")
        self._cfg = LinkConfig(
            rps_limit=float(g("max_wheel_rps")),
            motors_allowed=bool(g("motors_enabled")),
            motor_type=None if motor_type < 0 else motor_type,
            setpoint_timeout_s=float(g("setpoint_timeout_s")),
            idle_stop_period_s=float(g("idle_stop_period_s")),
            stop_on_connect=bool(g("stop_on_connect")),
            query_version=bool(g("query_version")),
        )
        kind = str(g("transport"))
        board = None
        if kind == "mock":
            board = MockBoard(MockConfig(profile=str(g("mock_profile")), battery_mv=int(g("mock_battery_mv")),
                                         zero_imu_probability=float(g("mock_zero_imu_probability"))))
            self.get_logger().warning(f"MOCK transport ({g('mock_profile')} firmware): no hardware is used")
        transport = make_transport(kind, str(g("port")), int(g("baud")), board)
        self._board = board

        if self._cfg.motor_type is None:
            self.get_logger().warning("motor_type not configured: NO motor packet is sent (firmware motor PID stays off)")
        elif not self._cfg.motors_allowed:
            self.get_logger().warning("motors_enabled=false: only STOP is ever sent to the board")

        self._pub_imu = self.create_publisher(ImuRaw, "/rrc_lite/imu_raw", 20)
        self._pub_batt = self.create_publisher(BatteryState, "/rrc_lite/battery", 5)
        self._pub_enc = self.create_publisher(EncoderFeedback, "/rrc_lite/encoders", 20)
        self._pub_us = self.create_publisher(UltrasonicRaw, "/rrc_lite/ultrasonic", 10)
        self._pub_status = self.create_publisher(RrcLiteStatus, "/rrc_lite/status", 5)
        self._pub_diag = self.create_publisher(DiagnosticArray, "/diagnostics", 5)

        self._imu_times: deque[float] = deque(maxlen=200)
        self._have_feedback = False
        self._have_ultrasonic = False
        self._last_battery_mv: Optional[int] = None
        self._lock = threading.Lock()

        self._link = RrcLiteLink(transport, self._cfg, on_report=self._on_report)

        self._setpoint_topic = self.resolve_topic_name("/rrc_lite/motor_setpoints")
        self._required_pub = str(g("required_setpoint_publisher")).strip()
        self._publisher_ok = not self._required_pub
        self.create_subscription(MotorSetpoints, self._setpoint_topic, self._on_setpoints,
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))
        if self._required_pub:
            self.create_timer(0.5, self._check_publishers)
        self.create_timer(1.0 / max(0.5, float(g("status_rate_hz"))), self._publish_status)

        # SIGINT/SIGTERM shut the context down before destroy_node(): send STOP first, then stop publishing.
        self.context.on_shutdown(self._link.shutdown)
        self._thread = threading.Thread(target=self._link.run, name="rrc_lite_link", daemon=True)
        self._thread.start()

    # ------------------------------------------------------------------ inputs

    def _check_publishers(self) -> None:
        names = [i.node_name for i in self.get_publishers_info_by_topic(self._setpoint_topic)]
        if "_NODE_NAME_UNKNOWN_" in names:
            return  # discovery not finished; decide on the next check
        ok = all(n == self._required_pub for n in names)  # zero publishers is fine (nothing to obey)
        if ok != self._publisher_ok:
            if ok:
                self.get_logger().info(f"setpoint publishers verified: {names}")
            else:
                self.get_logger().error(
                    f"{self._setpoint_topic} publishers {names} != only '{self._required_pub}': motors held at STOP")
                self._link.stop("foreign setpoint publisher")
        self._publisher_ok = ok

    def _on_setpoints(self, msg: MotorSetpoints) -> None:
        if not self._publisher_ok:
            self._link.stop("foreign setpoint publisher")
            return
        self._link.set_setpoints(list(msg.rps))

    def _on_report(self, report: p.Report, t: float) -> None:
        if not self.context.ok():
            return
        stamp = self.get_clock().now().to_msg()
        if isinstance(report, p.ImuSample):
            self._imu_times.append(t)
            m = ImuRaw()
            m.header.stamp = stamp
            m.header.frame_id = "imu_link"
            m.accel = [float(v) for v in report.accel]
            m.gyro = [float(v) for v in report.gyro]
            self._pub_imu.publish(m)
        elif isinstance(report, p.BatterySample):
            self._last_battery_mv = report.millivolts
            b = BatteryState()
            b.header.stamp = stamp
            nan = float("nan")
            b.voltage = report.millivolts / 1000.0
            b.current = b.charge = b.capacity = b.design_capacity = b.percentage = b.temperature = nan
            b.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_UNKNOWN
            b.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN
            b.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_UNKNOWN
            b.present = report.millivolts > 0
            self._pub_batt.publish(b)
        elif isinstance(report, p.MotorFeedback):
            self._have_feedback = True
            e = EncoderFeedback()
            e.header.stamp = stamp
            e.mcu_ms = report.mcu_ms
            e.counts = list(report.counts)
            e.rps = [float(v) for v in report.rps]
            self._pub_enc.publish(e)
        elif isinstance(report, p.UltrasonicSample):
            self._have_ultrasonic = True
            u = UltrasonicRaw()
            u.header.stamp = stamp
            u.header.frame_id = "ultrasonic_front"
            u.distance_mm = report.distance_mm
            self._pub_us.publish(u)
        elif isinstance(report, p.KeyEvent):
            self.get_logger().info(f"board key {report.key_id} event 0x{report.event:02x}")

    # ------------------------------------------------------------------ outputs

    def _imu_rate(self) -> float:
        now = time.monotonic()
        recent = [t for t in self._imu_times if now - t <= 2.0]
        return len(recent) / 2.0

    def _publish_status(self) -> None:
        s = self._link.status()
        m = RrcLiteStatus()
        m.header.stamp = self.get_clock().now().to_msg()
        m.connected, m.link_ok, m.port = s.connected, s.link_ok, s.port
        m.firmware_profile, m.firmware_version = s.profile, s.firmware_version
        m.rx_frames, m.crc_errors, m.malformed_frames, m.discarded_bytes = (
            s.rx_frames, s.crc_errors, s.malformed, s.discarded_bytes)
        m.rx_age_s, m.setpoint_age_s = _f(s.rx_age_s), _f(s.setpoint_age_s)
        m.imu_rate_hz = self._imu_rate()
        m.watchdog_active, m.watchdog_trips = s.watchdog_active, s.watchdog_trips
        m.rejected_setpoints, m.reconnects, m.motors_allowed = s.rejected_setpoints, s.reconnects, s.motors_allowed
        m.encoder_feedback, m.ultrasonic_available = self._have_feedback, self._have_ultrasonic
        m.last_sent_rps = [float(v) for v in s.last_sent_rps]
        m.last_error = s.last_error
        self._pub_status.publish(m)
        self._publish_diagnostics(s, m.imu_rate_hz)

    def _publish_diagnostics(self, s, imu_rate: float) -> None:
        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()

        def status(name, level, message, **values):
            d = DiagnosticStatus(name=f"rrc_lite: {name}", hardware_id="RRC Lite", level=level, message=message)
            d.values = [KeyValue(key=k, value=str(v)) for k, v in values.items()]
            arr.status.append(d)

        if not s.connected:
            status("link", ERROR, "serial port not connected", port=s.port, last_error=s.last_error,
                   reconnects=s.reconnects)
        elif not s.link_ok:
            status("link", ERROR, f"no valid frame for {s.rx_age_s:.2f}s", port=s.port)
        else:
            lvl = WARN if s.crc_errors or s.malformed else OK
            status("link", lvl, "connected", port=s.port, crc_errors=s.crc_errors, malformed=s.malformed,
                   reconnects=s.reconnects, imu_rate_hz=f"{imu_rate:.1f}")
        if s.profile == PROFILE_STOCK:
            status("firmware", WARN, "stock firmware: no encoder feedback, no MCU command timeout",
                   profile=s.profile)
        elif s.profile == PROFILE_PATCHED:
            status("firmware", OK, f"patched firmware {s.firmware_version}", profile=s.profile)
        else:
            status("firmware", WARN, "firmware profile not yet detected", profile=s.profile)
        if self._cfg.motor_type is None:
            status("motor_type", WARN, "not configured: motor packets disabled (firmware PID never started)")
        status("watchdog", WARN if s.watchdog_trips else OK,
               "holding STOP" if s.watchdog_active else "streaming setpoints",
               trips=s.watchdog_trips, setpoint_age_s=f"{s.setpoint_age_s:.3f}",
               motors_allowed=s.motors_allowed, rejected=s.rejected_setpoints)
        if self._last_battery_mv is None:
            status("battery", WARN, "no battery report yet")
        else:
            status("battery", OK, f"{self._last_battery_mv / 1000:.2f} V (raw report)")
        status("ultrasonic", OK if self._have_ultrasonic else WARN,
               "reporting" if self._have_ultrasonic else "UNSUPPORTED_BY_FIRMWARE (stock RRC Lite does not read I2C sonar)")
        status("encoders", OK if self._have_feedback else WARN,
               "reporting" if self._have_feedback else "UNSUPPORTED_BY_FIRMWARE (stock RRC Lite sends no encoder data)")
        self._pub_diag.publish(arr)

    def destroy_node(self) -> bool:
        self._link.shutdown()
        self._thread.join(timeout=1.0)
        return super().destroy_node()


def main(args: Optional[list[str]] = None) -> None:
    logging.basicConfig(level=logging.INFO, format="[rrc_lite.link] %(levelname)s %(message)s")
    rclpy.init(args=args)
    node: Optional[RrcLiteNode] = None
    try:
        node = RrcLiteNode()
        rclpy.spin(node)
    except ValueError as exc:
        print(f"[rrc_lite] configuration error: {exc}")
        raise SystemExit(2)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
