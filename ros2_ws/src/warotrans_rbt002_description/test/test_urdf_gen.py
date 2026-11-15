import os
import xml.etree.ElementTree as ET

import pytest
import yaml

from warotrans_rbt002_description.urdf_gen import build_urdf

HERE = os.path.dirname(__file__)


def load(name):
    with open(os.path.join(HERE, "..", "config", name), encoding="utf-8") as f:
        return yaml.safe_load(f)


def test_real_geometry_is_unmeasured_so_no_urdf():
    res = build_urdf(load("geometry.yaml"))
    assert res.urdf is None and "base_footprint_to_base_link" in res.missing and not res.mock
    with pytest.raises(ValueError):
        build_urdf(load("geometry.yaml"), require_complete=True)


def test_mock_geometry_builds_all_edges():
    res = build_urdf(load("geometry_mock.yaml"), require_complete=True)
    assert res.mock and not res.missing
    root = ET.fromstring(res.urdf.split("\n", 1)[1])
    children = {j.find("child").get("link") for j in root.iter("joint")}
    assert children == {"base_link", "laser", "imu_link", "ultrasonic_front", "camera_link"}


def test_partial_geometry_publishes_only_measured_edges():
    geo = load("geometry_mock.yaml")
    geo["imu"]["roll"] = None
    res = build_urdf(geo)
    assert "imu" in res.missing and "base_link->imu_link" not in res.published
    assert "base_link->laser" in res.published
