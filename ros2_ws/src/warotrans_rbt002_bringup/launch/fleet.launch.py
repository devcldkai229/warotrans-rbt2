"""RBT-002 FLEET mode: same functional stack as RBT-001's semantic traffic operation
(warotrans/tools/pi_run_semantic.sh -> warotrans_bringup navigation.launch.py), on RBT-002's base.

  ros2 launch warotrans_rbt002_bringup fleet.launch.py bundle:=<.../map_one_v013> motors_enabled:=true

Graph:
  base (rrc_lite, base_controller on /cmd_vel_motor from cmd_vel_authority_gate only, sensor_adapter,
        wheel_odom without TF), description, RPLIDAR C1, camera
  + EKF (warotrans_localization/ekf.launch.py, config/rbt002/ekf_rbt002.yaml: gyro-z only)
  + AMCL + map_server (warotrans_localization/localization.launch.py, config/rbt002/amcl_rbt002.yaml)
  + after nav_start_delay_sec: Nav2 core (warotrans_navigation/navigation.launch.py) with the lane-aware
    planner, keepout/speed filters, collision monitor, recovery_guard and cmd_vel_authority_gate
    (extra params: core nav2_params_lane_aware.yaml then config/rbt002/nav2_rbt002.yaml)
  + robot_gateway (warotrans_fleet/fleet.launch.py, config/rbt002/robot_gateway_rbt002.yaml) when
    mqtt_host is given; robot_code defaults to RBT-002.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def gateway_params_with_simulated_battery(params_file: str, percent: float, out_dir: str | None = None) -> str:
    """A copy of the gateway YAML with simulated_battery_percent set (0..100). Returns the copy's path."""
    import tempfile

    import yaml

    if not 0.0 <= percent <= 100.0:
        raise RuntimeError(f"simulated_battery_percent must be 0..100, got {percent}")
    with open(params_file, encoding="utf-8") as f:
        doc = yaml.safe_load(f)
    doc["robot_gateway"]["ros__parameters"]["simulated_battery_percent"] = float(percent)
    fd, path = tempfile.mkstemp(prefix="robot_gateway_rbt002_sim_battery_", suffix=".yaml", dir=out_dir)
    with os.fdopen(fd, "w", encoding="utf-8") as f:
        yaml.safe_dump(doc, f, sort_keys=False)
    return path


def _setup(context):
    lc = lambda n: LaunchConfiguration(n).perform(context)  # noqa: E731
    bundle = os.path.expanduser(lc("bundle"))
    motors = lc("motors_enabled").lower() == "true"
    bench = lc("bench").lower() == "true"
    # Bench (robot on a stand, wheels in the air): the gyro never turns, so EKF yaw would stay 0 -> wheel odom owns TF.
    ekf = lc("enable_ekf").lower() == "true" and not bench
    for f in ("map.yaml", "lanes.json", "keepout.yaml", "speed.yaml", "semantic_manifest.json"):
        if not os.path.isfile(os.path.join(bundle, f)):
            raise RuntimeError(f"bundle file missing: {os.path.join(bundle, f)}")

    me = get_package_share_directory("warotrans_rbt002_bringup")
    nav = get_package_share_directory("warotrans_navigation")
    loc = get_package_share_directory("warotrans_localization")
    desc = get_package_share_directory("warotrans_rbt002_description")
    cfg = lambda n: os.path.join(me, "config", "rbt002", n)  # noqa: E731

    params = [os.path.join(me, "config", "rbt002.yaml")]
    cal = os.path.expanduser(lc("calibration_file"))
    if os.path.isfile(cal):
        params.append(cal)
    params.append(cfg("fleet_base.yaml"))
    base_override = {"motors_enabled": motors}
    odom_override = {"publish_tf": not ekf}  # exactly one owner of odom->base_footprint
    if not ekf:
        # A plain dict loses to the node-keyed "wheel_odom:" section of fleet_base.yaml (publish_tf: false), so the
        # wheel odometry would publish no TF at all: give it as a node-keyed file, last.
        import tempfile

        import yaml

        fd, odom_yaml = tempfile.mkstemp(prefix="wheel_odom_tf_", suffix=".yaml")
        with os.fdopen(fd, "w", encoding="utf-8") as f:
            yaml.safe_dump({"wheel_odom": {"ros__parameters": {"publish_tf": True}}}, f)
        odom_override = odom_yaml

    actions = [
        LogInfo(msg=f"[rbt002 fleet] bundle={bundle} motors_enabled={motors} ekf={ekf} calibration="
                    f"{'loaded' if os.path.isfile(cal) else 'MISSING (no motor map: ENABLED blocked)'}"),
        Node(package="warotrans_rrc_lite", executable="rrc_lite_node", name="rrc_lite", output="screen",
             parameters=params + [base_override]),
        Node(package="warotrans_rbt002_hardware", executable="base_controller_node", name="base_controller",
             output="screen", parameters=params + [base_override]),
        Node(package="warotrans_rbt002_hardware", executable="sensor_adapter_node", name="sensor_adapter",
             output="screen", parameters=params),
        Node(package="warotrans_rbt002_hardware", executable="odom_node", name="wheel_odom", output="screen",
             parameters=params + [odom_override]),
        Node(package="sllidar_ros2", executable="sllidar_node", name="sllidar_node", output="screen",
             parameters=params),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(desc, "launch", "description.launch.py")),
            launch_arguments={"geometry_file": os.path.join(desc, "config", "geometry.yaml"),
                              "require": "false"}.items()),
    ]
    if bench:
        # map_server only + bench_localizer in place of AMCL (pose = wheel odometry from the last /initialpose).
        actions += [
            LogInfo(msg="[rbt002 fleet] BENCH MODE: AMCL off, EKF off, pose from wheel odometry (robot on a stand)"),
            Node(package="nav2_map_server", executable="map_server", name="map_server", output="screen",
                 respawn=True, respawn_delay=2.0,
                 parameters=[cfg("amcl_rbt002.yaml"), {"yaml_filename": os.path.join(bundle, "map.yaml")}]),
            Node(package="nav2_lifecycle_manager", executable="lifecycle_manager",
                 name="lifecycle_manager_localization", output="screen",
                 parameters=[{"autostart": True}, {"node_names": ["map_server"]}]),
            Node(package="warotrans_rbt002_bringup", executable="bench_localizer", name="bench_localizer",
                 output="screen",
                 parameters=[{"initial_x": float(lc("bench_initial_x")), "initial_y": float(lc("bench_initial_y")),
                              "initial_yaw": float(lc("bench_initial_yaw"))}]),
        ]
    else:
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(loc, "launch", "localization.launch.py")),
            launch_arguments={"map": os.path.join(bundle, "map.yaml"), "autostart": "true",
                              "amcl_params_file": cfg("amcl_rbt002.yaml")}.items()))
    if lc("camera").lower() == "true":
        actions.append(Node(package="warotrans_camera", executable="camera_stream", name="camera_stream",
                            output="screen", parameters=params))
    if ekf:
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(loc, "launch", "ekf.launch.py")),
            launch_arguments={"ekf_params_file": cfg("ekf_rbt002.yaml")}.items()))

    nav_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav, "launch", "navigation.launch.py")),
        launch_arguments={
            "autostart": "true",
            "enable_costmap_filters": "true",
            "keepout_mask": os.path.join(bundle, "keepout.yaml"),
            "speed_mask": os.path.join(bundle, "speed.yaml"),
            "lanes_json": os.path.join(bundle, "lanes.json"),
            "extra_params_file": os.path.join(nav, "config", "nav2_params_lane_aware.yaml") + ";" +
                                 cfg("nav2_rbt002.yaml") + (";" + cfg("nav2_bench.yaml") if bench else ""),
            "enable_ekf": "true" if ekf else "false",
            "enable_cmd_vel_pipeline": "true",
            "enable_authority_gate": "true",
            "gate_output_topic": "cmd_vel_motor",
            "required_authority_publisher": lc("authority_publisher"),
            # Lane-graph mode: Nav2 Route Server on this graph (empty: the semantic costmap planner as before).
            "route_graph": lc("route_graph"),
        }.items())
    actions.append(TimerAction(period=float(lc("nav_start_delay_sec")), actions=[nav_include]))

    mqtt_host = lc("mqtt_host").strip()
    if mqtt_host:
        fleet = get_package_share_directory("warotrans_fleet")
        gateway_params = cfg("robot_gateway_rbt002.yaml")
        simulated = lc("simulated_battery_percent").strip()
        if simulated:
            # Dry runs without the motor battery only: telemetry then says batteryEstimationMode=SIMULATED.
            gateway_params = gateway_params_with_simulated_battery(gateway_params, float(simulated))
            actions.append(LogInfo(msg=f"[rbt002 fleet] WARNING battery is SIMULATED at {float(simulated):.0f} % "
                                       "(dry run only; unset SIM_BATTERY_PERCENT before real work)"))
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(fleet, "launch", "fleet.launch.py")),
            launch_arguments={"params_file": gateway_params,
                              "robot_code": lc("robot_code"), "mqtt_host": mqtt_host,
                              "semantic_manifest": os.path.join(bundle, "semantic_manifest.json"),
                              "route_graph": lc("route_graph")}.items()))
        if lc("camera").lower() == "true" and lc("evidence").lower() == "true":
            # Evidence: records the camera during trips and uploads to the Recorder on the backend laptop. The robot's
            # credential is WARO_EVIDENCE_TOKEN in ~/.config/warotrans/rbt002.env (never in this repository).
            actions.append(Node(package="warotrans_evidence_agent", executable="evidence_agent", name="evidence_agent",
                                output="screen",
                                parameters=[cfg("evidence_agent_rbt002.yaml"),
                                            {"recorder_url": lc("recorder_url").strip() or f"http://{mqtt_host}:5095",
                                             "robot_code": lc("robot_code")}]))
    else:
        actions.append(LogInfo(msg="[rbt002 fleet] mqtt_host empty: robot_gateway NOT started (no backend link)"))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("bundle", description="semantic map bundle dir (same as the backend's)"),
        DeclareLaunchArgument("motors_enabled", default_value="false"),
        DeclareLaunchArgument("route_graph", default_value="",
                              description="lane graph GeoJSON: drive the backend's graph edges (traffic_mode=active)"),
        DeclareLaunchArgument("enable_ekf", default_value="true"),
        DeclareLaunchArgument("bench", default_value="false",
                              description="robot on a stand (wheels in the air): no AMCL/EKF, pose = wheel odometry"),
        DeclareLaunchArgument("bench_initial_x", default_value="0.0"),
        DeclareLaunchArgument("bench_initial_y", default_value="0.0"),
        DeclareLaunchArgument("bench_initial_yaw", default_value="0.0"),
        DeclareLaunchArgument("camera", default_value="true"),
        DeclareLaunchArgument("evidence", default_value="true"),
        DeclareLaunchArgument("nav_start_delay_sec", default_value="12.0"),
        DeclareLaunchArgument("calibration_file", default_value="~/.config/warotrans/rbt002_calibration.yaml"),
        DeclareLaunchArgument("mqtt_host", default_value="", description="MQTT broker (backend laptop)"),
        DeclareLaunchArgument("robot_code", default_value="RBT-002"),
        DeclareLaunchArgument("recorder_url", default_value="",
                              description="Evidence Recorder (empty: http://<mqtt_host>:5095, the laptop)"),
        DeclareLaunchArgument("authority_publisher", default_value="robot_gateway",
                              description="only node allowed to open the cmd_vel gate (nav tests: the test runner)"),
        DeclareLaunchArgument("simulated_battery_percent", default_value="",
                              description="dry run without the motor battery: report this battery % (SIMULATED)"),
        OpaqueFunction(function=_setup),
    ])
