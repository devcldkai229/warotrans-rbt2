"""The readiness verdict, without ROS: python -m pytest tools/test_rbt2_readiness.py"""

import os
import sys

sys.path.insert(0, os.path.dirname(__file__))

from rbt2_readiness import GATE, REQUIRED_NODES, Facts, evaluate  # noqa: E402


def ready() -> Facts:
    return Facts(
        nodes=set(REQUIRED_NODES),
        cmd_vel_motor_publishers=[GATE],
        tf_ok={"map->odom": True, "odom->base_footprint": True},
        amcl_pose_seen=True,
        battery_percentage=0.8,
        battery_mode="VOLTAGE_ONLY",
        broker_ok=True,
        clock_skew_s=0.4,
        motors_enabled="false",
    )


def failed(f: Facts) -> list[str]:
    return [c.name for c in evaluate(f) if c.required and not c.ok]


def test_a_complete_stack_is_ready():
    assert failed(ready()) == []


def test_anything_but_the_gate_on_cmd_vel_motor_is_not_ready():
    f = ready()
    f.cmd_vel_motor_publishers = [GATE, "control_gateway"]
    assert failed(f) == ["cmd_vel"]
    f.cmd_vel_motor_publishers = []
    assert failed(f) == ["cmd_vel"]


def test_a_missing_node_is_named():
    f = ready()
    f.nodes.discard("robot_gateway")
    check = next(c for c in evaluate(f) if c.name == "nodes")
    assert not check.ok and "robot_gateway" in check.detail


def test_no_battery_percentage_is_not_ready_unless_the_dry_run_simulates_one():
    f = ready()
    f.battery_percentage = None
    assert failed(f) == ["battery"]
    f.simulated_battery = "80"
    assert failed(f) == []
    assert "SIMULATED" in next(c for c in evaluate(f) if c.name == "battery").detail


def test_clock_skew_over_two_seconds_and_no_broker_are_not_ready():
    f = ready()
    f.clock_skew_s = -3.1
    f.broker_ok = False
    assert sorted(failed(f)) == ["broker", "clock"]


def test_no_pose_and_no_tf_are_not_ready():
    f = ready()
    f.amcl_pose_seen = False
    f.tf_ok = {"map->odom": False, "odom->base_footprint": True}
    assert sorted(failed(f)) == ["amcl", "tf"]
