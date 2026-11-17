"""RBT-002 bringup.

  ros2 launch warotrans_rbt002_bringup bringup.launch.py                       # real robot, motors held at STOP
  ros2 launch warotrans_rbt002_bringup bringup.launch.py motors_enabled:=true  # real robot, motion possible
  ros2 launch warotrans_rbt002_bringup bringup.launch.py mock:=true motors_enabled:=true mock_profile:=patched

Arguments
  mock            true = in-process simulated RRC Lite (no serial port), mock geometry, no lidar
  mock_profile    stock | patched
  motors_enabled  false (default) = only STOP ever reaches the board, ENABLED/CALIBRATION refused
  lidar           RPLIDAR C1 via sllidar_ros2 (default: true on the robot, false in mock)
  gateway         mobile control UI on :8080 (default true)
  camera          MJPEG camera server on :8081/stream (default: true on the robot, false in mock)
  description     robot_state_publisher from geometry.yaml (only measured edges)
  calibration_file  motor map saved from the UI, loaded after rbt002.yaml (real robot only)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context):
    lc = lambda n: LaunchConfiguration(n).perform(context)  # noqa: E731
    mock = lc("mock").lower() == "true"
    motors = lc("motors_enabled").lower() == "true"
    lidar = lc("lidar").lower()
    lidar = (not mock) if lidar == "auto" else lidar == "true"
    gateway = lc("gateway").lower() == "true"
    camera = lc("camera").lower()
    camera = (not mock) if camera == "auto" else camera == "true"
    description = lc("description").lower() == "true"

    bringup = get_package_share_directory("warotrans_rbt002_bringup")
    params = [os.path.join(bringup, "config", "rbt002.yaml")]
    cal_file = os.path.expanduser(lc("calibration_file"))
    if mock:
        params.append(os.path.join(bringup, "config", "mock.yaml"))
    elif os.path.isfile(cal_file):
        params.append(cal_file)  # motor map saved from the control UI (overrides rbt002.yaml)
    override = {"motors_enabled": motors}
    rrc_override = dict(override)
    if mock:
        rrc_override["mock_profile"] = lc("mock_profile")

    actions = [LogInfo(msg=f"[bringup] mock={mock} motors_enabled={motors} lidar={lidar} gateway={gateway} "
                           f"calibration={'loaded ' + cal_file if (not mock and os.path.isfile(cal_file)) else 'none'}")]
    if motors and not mock:
        actions.append(LogInfo(msg="[bringup] MOTORS ENABLED ON THE REAL ROBOT. Stock RRC Lite firmware has no command "
                                   "timeout: keep the wheels off the ground or a hand on the battery switch."))

    # No respawn: on Ctrl+C the nodes get SIGINT before launch and would be restarted after shutdown
    # (seen in the mock E2E), leaving an orphan driver holding the serial port. USB loss is handled
    # inside rrc_lite (reconnect), so respawn is not needed.
    actions += [
        Node(package="warotrans_rrc_lite", executable="rrc_lite_node", name="rrc_lite", output="screen",
             parameters=params + [rrc_override]),
        Node(package="warotrans_rbt002_hardware", executable="base_controller_node", name="base_controller",
             output="screen", parameters=params + [override]),
        Node(package="warotrans_rbt002_hardware", executable="sensor_adapter_node", name="sensor_adapter",
             output="screen", parameters=params),
        Node(package="warotrans_rbt002_hardware", executable="odom_node", name="wheel_odom", output="screen",
             parameters=params),
    ]
    if lidar:
        actions.append(Node(package="sllidar_ros2", executable="sllidar_node", name="sllidar_node", output="screen",
                            parameters=params))
    if camera:
        actions.append(Node(package="warotrans_camera", executable="camera_stream", name="camera_stream",
                            output="screen", parameters=params))
    if gateway:
        actions.append(Node(package="warotrans_rbt002_control_gateway", executable="control_gateway",
                            name="control_gateway", output="screen", parameters=params))
    if description:
        desc = get_package_share_directory("warotrans_rbt002_description")
        geo = os.path.join(desc, "config", "geometry_mock.yaml" if mock else "geometry.yaml")
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(desc, "launch", "description.launch.py")),
            launch_arguments={"geometry_file": geo, "require": "false"}.items()))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("mock", default_value="false"),
        DeclareLaunchArgument("mock_profile", default_value="stock"),
        DeclareLaunchArgument("motors_enabled", default_value="false"),
        DeclareLaunchArgument("lidar", default_value="auto"),
        DeclareLaunchArgument("gateway", default_value="true"),
        DeclareLaunchArgument("camera", default_value="auto"),
        DeclareLaunchArgument("description", default_value="true"),
        DeclareLaunchArgument("calibration_file", default_value="~/.config/warotrans/rbt002_calibration.yaml"),
        OpaqueFunction(function=_setup),
    ])
