"""Static checks on config/rbt002.yaml: no guessed physical constants, consistent limits."""
import os

import yaml

HERE = os.path.dirname(__file__)


def load(name):
    with open(os.path.join(HERE, "..", "config", name), encoding="utf-8") as f:
        return yaml.safe_load(f)


def test_real_config_has_no_unmeasured_guesses():
    shared = load("rbt002.yaml")["/**"]["ros__parameters"]
    # measured 2026-11-15 (docs/calibration.md §1); a change here must come with a new calibration entry
    assert (shared["wheel_radius_m"], shared["half_wheelbase_m"], shared["half_track_m"]) == (0.0485, 0.0975, 0.1075)
    # the motor map lives in ~/.config/warotrans/rbt002_calibration.yaml on the robot, not in the repo
    for k in ("port_fl", "port_fr", "port_rl", "port_rr"):
        assert shared[k] == 0
    assert shared["port_signs"] == [0, 0, 0, 0]
    assert shared["motors_enabled"] is False


def test_real_config_uses_by_id_paths():
    cfg = load("rbt002.yaml")
    assert cfg["rrc_lite"]["ros__parameters"]["port"].startswith("/dev/serial/by-id/")
    assert cfg["sllidar_node"]["ros__parameters"]["serial_port"].startswith("/dev/serial/by-id/")
    assert cfg["rrc_lite"]["ros__parameters"]["transport"] == "serial"


def test_calibration_limit_below_wheel_limit():
    cfg = load("rbt002.yaml")
    assert cfg["base_controller"]["ros__parameters"]["calibration_max_rps"] <= cfg["/**"]["ros__parameters"]["max_wheel_rps"]


def test_mock_overlay_only_for_mock():
    assert load("mock.yaml")["rrc_lite"]["ros__parameters"]["transport"] == "mock"


def test_fleet_overlays_present_and_consistent():
    import os
    base = os.path.join(HERE, "..", "config", "rbt002")
    for f in ("nav2_rbt002.yaml", "ekf_rbt002.yaml", "amcl_rbt002.yaml", "robot_gateway_rbt002.yaml", "fleet_base.yaml"):
        assert os.path.isfile(os.path.join(base, f)), f
    ekf = load("rbt002/ekf_rbt002.yaml")["ekf_filter_node"]["ros__parameters"]
    assert ekf["imu0_config"][5] is False and ekf["imu0_config"][11] is True  # no IMU yaw, yaw rate only
    gw = load("rbt002/robot_gateway_rbt002.yaml")["robot_gateway"]["ros__parameters"]
    assert gw["robot_code"] == "RBT-002" and gw["human_confirm_enabled"] is False
    fb = load("rbt002/fleet_base.yaml")
    assert fb["base_controller"]["ros__parameters"]["required_cmd_vel_publisher"] == "cmd_vel_authority_gate"
    assert fb["wheel_odom"]["ros__parameters"]["publish_tf"] is False
    nav = load("rbt002/nav2_rbt002.yaml")
    assert nav["velocity_smoother"]["ros__parameters"]["max_velocity"][0] <= fb["base_controller"]["ros__parameters"]["max_linear_mps"]
