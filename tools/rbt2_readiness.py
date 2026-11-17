#!/usr/bin/env python3
"""Is RBT-002's fleet stack ready to take Jobs from the backend? Read-only: it subscribes, lists the graph, opens one
TCP connection to the broker and one HTTP GET to the backend. It never publishes anything.

Run ON the Pi with the stack in MODE=fleet (same ROS_DOMAIN_ID as the service, 31 by default):

    ROS_DOMAIN_ID=31 python3 tools/rbt2_readiness.py --broker 192.168.137.1 [--backend http://192.168.137.1:5090]

Exit code 0 = every REQUIRED check passed. Checks (RBT-001's real_robot_readiness, for RBT-002's base):
  nodes       base (rrc_lite, base_controller, sensor_adapter), EKF, AMCL, Nav2, gate, recovery_guard, gateway
  cmd_vel     /cmd_vel_motor has exactly one publisher: cmd_vel_authority_gate (nothing can bypass traffic)
  tf          map->odom and odom->base_footprint resolve
  amcl        /amcl_pose seen (the gateway still needs an operator pose / web LOCALIZE to report LOCALIZED)
  battery     /battery/estimate has a percentage, or the gateway runs with a SIMULATED battery (dry run)
  broker      TCP connect to the MQTT broker
  clock       |Pi clock - backend HTTP Date| <= 2 s (when --backend is given)
"""

from __future__ import annotations

import argparse
import email.utils
import math
import os
import socket
import sys
import time
import urllib.request
from dataclasses import dataclass, field

REQUIRED_NODES = (
    "rrc_lite", "base_controller", "sensor_adapter", "ekf_filter_node", "amcl", "map_server",
    "bt_navigator", "controller_server", "planner_server", "collision_monitor", "velocity_smoother",
    "cmd_vel_authority_gate", "recovery_guard", "robot_gateway",
)
GATE = "cmd_vel_authority_gate"
MAX_CLOCK_SKEW_S = 2.0


@dataclass
class Facts:
    """What was observed; the verdict is computed from this alone (tested without ROS)."""

    nodes: set[str] = field(default_factory=set)
    cmd_vel_motor_publishers: list[str] = field(default_factory=list)
    tf_ok: dict[str, bool] = field(default_factory=dict)
    amcl_pose_seen: bool = False
    battery_percentage: float | None = None   # 0..1 from /battery/estimate
    battery_mode: str | None = None
    simulated_battery: str | None = None      # SIM_BATTERY_PERCENT of the service env, if set
    broker_ok: bool | None = None
    clock_skew_s: float | None = None
    motors_enabled: str | None = None


@dataclass
class Check:
    name: str
    ok: bool
    required: bool
    detail: str


def evaluate(f: Facts) -> list[Check]:
    checks: list[Check] = []
    missing = [n for n in REQUIRED_NODES if n not in f.nodes]
    checks.append(Check("nodes", not missing, True, "all running" if not missing else "missing: " + ", ".join(missing)))

    pubs = sorted(set(f.cmd_vel_motor_publishers))
    checks.append(Check(
        "cmd_vel", pubs == [GATE], True,
        f"/cmd_vel_motor publishers: {pubs or 'none'} (must be only {GATE})"))

    bad_tf = [k for k, ok in f.tf_ok.items() if not ok]
    checks.append(Check("tf", bool(f.tf_ok) and not bad_tf, True,
                        "map->odom->base_footprint OK" if not bad_tf and f.tf_ok else "missing: " + ", ".join(bad_tf or ["(none checked)"])))

    checks.append(Check("amcl", f.amcl_pose_seen, True,
                        "/amcl_pose seen" if f.amcl_pose_seen else "no /amcl_pose yet (set the pose: web LOCALIZE)"))

    pct = f.battery_percentage
    real = pct is not None and math.isfinite(pct) and 0.0 <= pct <= 1.0
    if f.simulated_battery:
        checks.append(Check("battery", True, True,
                            f"SIMULATED {f.simulated_battery} % (dry run only; real estimate: "
                            f"{f'{pct * 100:.0f} %' if real else 'none'})"))
    else:
        checks.append(Check("battery", real, True,
                            f"{pct * 100:.0f} % ({f.battery_mode})" if real
                            else "no percentage on /battery/estimate: the backend refuses dispatch (battery UNKNOWN)"))

    if f.broker_ok is not None:
        checks.append(Check("broker", f.broker_ok, True, "MQTT broker reachable" if f.broker_ok else "cannot connect to the broker"))
    if f.clock_skew_s is not None:
        ok = abs(f.clock_skew_s) <= MAX_CLOCK_SKEW_S
        checks.append(Check("clock", ok, True, f"Pi - backend = {f.clock_skew_s:+.1f} s (limit {MAX_CLOCK_SKEW_S:.0f} s)"))
    checks.append(Check("motors", True, False, f"base_controller motors_enabled={f.motors_enabled or '?'}"))
    return checks


def broker_reachable(host: str, port: int = 1883, timeout: float = 3.0) -> bool:
    try:
        with socket.create_connection((host, port), timeout=timeout):
            return True
    except OSError:
        return False


def clock_skew(backend_url: str, timeout: float = 3.0) -> float | None:
    """Pi clock minus the backend's HTTP Date (1 s resolution; half the round trip added back)."""
    sent = time.time()
    try:
        with urllib.request.urlopen(backend_url.rstrip("/") + "/health", timeout=timeout) as r:
            date = r.headers.get("Date")
    except Exception as exc:  # noqa: BLE001 — an HTTP error still carries a Date header
        date = getattr(getattr(exc, "headers", None), "get", lambda _k: None)("Date")
    received = time.time()
    if not date:
        return None
    server = email.utils.parsedate_to_datetime(date).timestamp() + 0.5  # Date truncates to the second
    return (sent + received) / 2.0 - server


def collect(seconds: float) -> Facts:  # pragma: no cover - needs rclpy on the Pi
    import rclpy
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from rclpy.node import Node
    from rclpy.time import Time
    from tf2_ros import Buffer, TransformListener
    from warotrans_msgs.msg import BatteryEstimate

    rclpy.init()
    node = Node("rbt2_readiness")
    f = Facts()
    last_battery: list = []

    def on_amcl(_msg):
        f.amcl_pose_seen = True

    node.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", on_amcl, 1)
    node.create_subscription(BatteryEstimate, "/battery/estimate", lambda m: last_battery.append(m), 10)
    buf = Buffer()
    TransformListener(buf, node)
    # Discovery of ~30 nodes takes 10-20 s on a loaded Pi 5 (load 6-9): wait until the graph is complete (node names
    # and the /cmd_vel_motor publisher resolved) or the time is up, but never less than 3 s for TF and AMCL.
    start = time.monotonic()
    end = start + seconds
    next_look = start + 3.0
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
        if time.monotonic() >= next_look:
            next_look += 1.0
            names = {name for name, _ns in node.get_node_names_and_namespaces()}
            pubs = [i.node_name for i in node.get_publishers_info_by_topic("/cmd_vel_motor")]
            if set(REQUIRED_NODES) <= names and pubs and "_NODE_NAME_UNKNOWN_" not in pubs:
                break

    f.nodes = {name for name, _ns in node.get_node_names_and_namespaces()}
    f.cmd_vel_motor_publishers = [i.node_name for i in node.get_publishers_info_by_topic("/cmd_vel_motor")]
    for parent, child in (("map", "odom"), ("odom", "base_footprint")):
        try:
            buf.lookup_transform(parent, child, Time())
            f.tf_ok[f"{parent}->{child}"] = True
        except Exception:  # noqa: BLE001
            f.tf_ok[f"{parent}->{child}"] = False
    if last_battery:
        f.battery_percentage = float(last_battery[-1].percentage)
        f.battery_mode = str(last_battery[-1].mode)
    node.destroy_node()
    rclpy.shutdown()
    return f


def main() -> int:  # pragma: no cover - needs rclpy on the Pi
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--broker", required=True, help="MQTT broker host (the backend laptop)")
    ap.add_argument("--backend", default="", help="backend base URL for the clock check, e.g. http://192.168.137.1:5090")
    ap.add_argument("--seconds", type=float, default=25.0, help="longest wait for discovery")
    a = ap.parse_args()

    f = collect(a.seconds)
    env_file = os.path.expanduser("~/.config/warotrans/rbt002.env")
    if os.path.isfile(env_file):
        for line in open(env_file, encoding="utf-8"):
            key, _, value = line.strip().partition("=")
            if key == "SIM_BATTERY_PERCENT" and value:
                f.simulated_battery = value
            elif key == "MOTORS_ENABLED":
                f.motors_enabled = value
    f.broker_ok = broker_reachable(a.broker)
    if a.backend:
        f.clock_skew_s = clock_skew(a.backend)

    checks = evaluate(f)
    print(f"RBT-002 readiness (ROS_DOMAIN_ID={os.environ.get('ROS_DOMAIN_ID', '0')})")
    for c in checks:
        mark = "PASS" if c.ok else ("FAIL" if c.required else "WARN")
        print(f"  [{mark}] {c.name:9s} {c.detail}")
    failed = [c.name for c in checks if c.required and not c.ok]
    print("READY" if not failed else "NOT READY: " + ", ".join(failed))
    return 0 if not failed else 1


if __name__ == "__main__":
    sys.exit(main())
