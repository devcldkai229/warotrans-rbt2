"""Static checks on config/rbt002.yaml: no guessed physical constants, consistent limits."""
import os

import yaml

HERE = os.path.dirname(__file__)


def load(name):
    with open(os.path.join(HERE, "..", "config", name), encoding="utf-8") as f:
        return yaml.safe_load(f)


def test_real_config_has_no_unmeasured_guesses():
    shared = load("rbt002.yaml")["/**"]["ros__parameters"]
    for k in ("wheel_radius_m", "half_wheelbase_m", "half_track_m", "port_fl", "port_fr", "port_rl", "port_rr"):
        assert shared[k] == 0, f"{k} must stay 0 (CHƯA XÁC NHẬN) until measured and recorded in docs/calibration.md"
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
