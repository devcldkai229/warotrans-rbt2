"""RRC Lite raw sensor data -> standard ROS messages.

/rrc_lite/imu_raw    -> /imu/data           sensor_msgs/Imu (SI, frame imu_link, no orientation)
/rrc_lite/ultrasonic -> /ultrasonic/front   sensor_msgs/Range (patched firmware only)
/rrc_lite/battery    -> /battery_state      sensor_msgs/BatteryState
                        /battery/estimate   warotrans_msgs/BatteryEstimate (RBT-001 contract)
"""

from __future__ import annotations

import math
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import BatteryState, Imu, Range
from warotrans_msgs.msg import BatteryEstimate
from warotrans_rbt002_msgs.msg import ImuRaw, UltrasonicRaw

from .battery_soc import MODE_UNAVAILABLE, BatteryConfig, VoltageSocEstimator
from .imu_convert import ImuConverter, gravity_check


class SensorAdapterNode(Node):
    def __init__(self) -> None:
        super().__init__("sensor_adapter")
        d = self.declare_parameter
        d("imu_units", "g_dps")  # verified 2026-11-15 on RBT-002 (docs/calibration.md)
        d("imu_frame", "imu_link")
        d("imu_accel_variance", 0.0)  # 0 = not measured yet (CHƯA XÁC NHẬN)
        d("imu_gyro_variance", 0.0)
        d("ultrasonic_frame", "ultrasonic_front")
        d("ultrasonic_min_range_m", 0.0)  # 0 = CHƯA XÁC NHẬN, the Range message then carries 0
        d("ultrasonic_max_range_m", 0.0)
        d("ultrasonic_fov_rad", 0.0)
        d("battery_soc_enabled", False)  # chemistry / OCV table not measured for RBT-002
        d("battery_cells_in_series", 3)
        d("battery_plausible_min_v", 5.0)
        d("battery_plausible_max_v", 30.0)
        d("battery_design_capacity_ah", 0.0)
        g = lambda n: self.get_parameter(n).value  # noqa: E731

        self._imu = ImuConverter(str(g("imu_units")))
        self._imu_frame = str(g("imu_frame"))
        self._acc_var = float(g("imu_accel_variance"))
        self._gyr_var = float(g("imu_gyro_variance"))
        self._us_frame = str(g("ultrasonic_frame"))
        self._us_min = float(g("ultrasonic_min_range_m"))
        self._us_max = float(g("ultrasonic_max_range_m"))
        self._us_fov = float(g("ultrasonic_fov_rad"))
        self._soc_enabled = bool(g("battery_soc_enabled"))
        self._bcfg = BatteryConfig(
            cells_in_series=int(g("battery_cells_in_series")),
            design_capacity_ah=float(g("battery_design_capacity_ah")),
            divider_gain=1.0, offset_v=0.0,  # the RRC Lite already reports pack millivolts
            adc_disconnected_pin_mv=1000.0, adc_saturated_pin_mv=60000.0,
            plausible_min_v=float(g("battery_plausible_min_v")), plausible_max_v=float(g("battery_plausible_max_v")),
            stale_after_s=3.0, median_window=3, ema_tau_s=3.0)
        self._batt = VoltageSocEstimator(self._bcfg)
        self._gravity_warned = False

        self._pub_imu = self.create_publisher(Imu, "/imu/data", 20)
        self._pub_us = self.create_publisher(Range, "/ultrasonic/front", 10)
        self._pub_bs = self.create_publisher(BatteryState, "/battery_state", 5)
        self._pub_be = self.create_publisher(BatteryEstimate, "/battery/estimate", 5)
        self.create_subscription(ImuRaw, "/rrc_lite/imu_raw", self._on_imu, 20)
        self.create_subscription(UltrasonicRaw, "/rrc_lite/ultrasonic", self._on_us, 10)
        self.create_subscription(BatteryState, "/rrc_lite/battery", self._on_batt, 5)
        self.create_timer(1.0, self._publish_battery)
        self.create_timer(10.0, self._log_imu_stats)

    def _on_imu(self, m: ImuRaw) -> None:
        s = self._imu.convert(m.accel, m.gyro)
        if s is None:
            return
        if not self._gravity_warned and not gravity_check(s.accel, tolerance=0.5):
            self._gravity_warned = True
            self.get_logger().warning(
                f"|accel| = {math.sqrt(sum(a * a for a in s.accel)):.2f} m/s^2 is far from 1 g: check imu_units")
        out = Imu()
        out.header.stamp = m.header.stamp
        out.header.frame_id = self._imu_frame
        out.orientation_covariance[0] = -1.0  # no orientation estimate
        out.linear_acceleration.x, out.linear_acceleration.y, out.linear_acceleration.z = s.accel
        out.angular_velocity.x, out.angular_velocity.y, out.angular_velocity.z = s.gyro
        for i in (0, 4, 8):
            out.linear_acceleration_covariance[i] = self._acc_var
            out.angular_velocity_covariance[i] = self._gyr_var
        self._pub_imu.publish(out)

    def _on_us(self, m: UltrasonicRaw) -> None:
        r = Range()
        r.header.stamp = m.header.stamp
        r.header.frame_id = self._us_frame
        r.radiation_type = Range.ULTRASOUND
        r.field_of_view = self._us_fov
        r.min_range, r.max_range = self._us_min, self._us_max
        r.range = m.distance_mm / 1000.0 if m.distance_mm > 0 else float("inf")
        self._pub_us.publish(r)

    def _on_batt(self, m: BatteryState) -> None:
        if math.isfinite(m.voltage):
            self._batt.add_sample(time.monotonic(), m.voltage * 1000.0)

    def _publish_battery(self) -> None:
        r = self._batt.estimate(time.monotonic())
        stamp = self.get_clock().now().to_msg()
        nan = float("nan")
        pct = r.percentage if (self._soc_enabled and r.percentage is not None) else None
        fault = r.fault or ("" if self._soc_enabled else "soc_not_calibrated")
        e = BatteryEstimate()
        e.header.stamp = stamp
        e.mode = r.mode if self._soc_enabled else MODE_UNAVAILABLE
        e.confidence = r.confidence if self._soc_enabled else "NONE"
        e.charging_state = r.charging_state
        e.percentage = pct if pct is not None else nan
        e.voltage = r.voltage if r.voltage is not None else nan
        e.ocv_estimate = r.ocv_estimate if (self._soc_enabled and r.ocv_estimate is not None) else nan
        e.rested = r.rested
        e.fault = fault
        e.sample_age_s = r.sample_age_s if r.sample_age_s is not None else nan
        self._pub_be.publish(e)
        b = BatteryState()
        b.header.stamp = stamp
        b.voltage = e.voltage
        b.percentage = e.percentage
        b.current = b.charge = b.capacity = b.temperature = nan
        b.design_capacity = self._bcfg.design_capacity_ah if self._bcfg.design_capacity_ah > 0 else nan
        b.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_UNKNOWN
        b.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN
        b.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_UNKNOWN
        b.present = r.voltage is not None
        self._pub_bs.publish(b)

    def _log_imu_stats(self) -> None:
        st = self._imu.stats
        if st.zero_frames or st.non_finite or st.implausible:
            self.get_logger().info(f"IMU dropped: zero={st.zero_frames} non_finite={st.non_finite} "
                                   f"implausible={st.implausible} accepted={st.accepted}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = SensorAdapterNode()
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
