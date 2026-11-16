#!/usr/bin/env python3
"""Read-only health probe of the RBT-002 fleet stack: message rates and TF chain, from ONE rclpy node
(repeated `ros2 topic hz` processes miss data on a loaded Pi).

    python3 tools/fleet_probe.py [--seconds 10]
"""

import argparse
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

TOPICS = {
    "/rrc_lite/status": ("warotrans_rbt002_msgs.msg", "RrcLiteStatus"),
    "/rrc_lite/imu_raw": ("warotrans_rbt002_msgs.msg", "ImuRaw"),
    "/rrc_lite/encoders": ("warotrans_rbt002_msgs.msg", "EncoderFeedback"),
    "/odom": ("nav_msgs.msg", "Odometry"),
    "/imu/data": ("sensor_msgs.msg", "Imu"),
    "/odometry/filtered": ("nav_msgs.msg", "Odometry"),
    "/scan": ("sensor_msgs.msg", "LaserScan"),
    "/amcl_pose": ("geometry_msgs.msg", "PoseWithCovarianceStamped"),
    "/battery/estimate": ("warotrans_msgs.msg", "BatteryEstimate"),
    "/safety_state": ("warotrans_rbt002_msgs.msg", "SafetyState"),
}
TF_PAIRS = [("odom", "base_footprint"), ("map", "odom"), ("base_footprint", "laser"), ("base_footprint", "imu_link")]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=10.0)
    a = ap.parse_args()
    rclpy.init()
    node = Node("fleet_probe")
    counts = {t: 0 for t in TOPICS}
    last = {}
    for topic, (mod, name) in TOPICS.items():
        cls = getattr(__import__(mod, fromlist=[name]), name)

        def cb(msg, t=topic):
            counts[t] += 1
            last[t] = msg
        node.create_subscription(cls, topic, cb, qos_profile_sensor_data if topic == "/scan" else 10)
    buf = Buffer()
    TransformListener(buf, node)
    t_end = time.monotonic() + a.seconds
    while time.monotonic() < t_end:
        rclpy.spin_once(node, timeout_sec=0.05)
    print(f"--- rates over {a.seconds:.0f} s")
    for t, n in counts.items():
        print(f"{t:22s} {n / a.seconds:6.1f} Hz")
    print("--- TF")
    for parent, child in TF_PAIRS:
        try:
            tr = buf.lookup_transform(parent, child, Time())
            p = tr.transform.translation
            print(f"{parent:>15s} -> {child:15s} OK  ({p.x:+.3f}, {p.y:+.3f}, {p.z:+.3f})")
        except Exception as exc:  # noqa: BLE001
            print(f"{parent:>15s} -> {child:15s} MISSING ({type(exc).__name__})")
    s = last.get("/safety_state")
    if s is not None:
        print(f"--- base_controller: mode={s.mode_name} units={s.units} blockers={list(s.blockers)}")
    b = last.get("/battery/estimate")
    if b is not None:
        print(f"--- battery: mode={b.mode} pct={b.percentage:.3f} V={b.voltage:.2f} fault={b.fault!r}")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
