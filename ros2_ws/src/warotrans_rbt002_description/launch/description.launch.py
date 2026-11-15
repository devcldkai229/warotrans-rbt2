"""robot_state_publisher for RBT-002, from config/geometry.yaml (or geometry_mock.yaml).

Arguments:
  geometry_file  path to the geometry YAML (default: package config/geometry.yaml)
  require        true = fail if anything is unmeasured; false = publish only measured edges
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from warotrans_rbt002_description.urdf_gen import build_urdf


def _setup(context):
    path = LaunchConfiguration("geometry_file").perform(context)
    require = LaunchConfiguration("require").perform(context).lower() == "true"
    with open(path, encoding="utf-8") as f:
        geo = yaml.safe_load(f) or {}
    res = build_urdf(geo, require_complete=require)
    actions = []
    if res.mock:
        actions.append(LogInfo(msg="[description] MOCK geometry: not RBT-002 measurements"))
    if res.missing:
        actions.append(LogInfo(msg=f"[description] not measured (CHƯA XÁC NHẬN), no TF: {', '.join(res.missing)}"))
    if res.urdf is None:
        actions.append(LogInfo(msg="[description] base_footprint->base_link unmeasured: robot_state_publisher NOT started"))
        return actions
    actions.append(LogInfo(msg=f"[description] TF edges: {', '.join(res.published)}"))
    actions.append(Node(package="robot_state_publisher", executable="robot_state_publisher",
                        name="robot_state_publisher", output="screen",
                        parameters=[{"robot_description": res.urdf}]))
    return actions


def generate_launch_description():
    share = get_package_share_directory("warotrans_rbt002_description")
    return LaunchDescription([
        DeclareLaunchArgument("geometry_file", default_value=os.path.join(share, "config", "geometry.yaml")),
        DeclareLaunchArgument("require", default_value="false"),
        OpaqueFunction(function=_setup),
    ])
