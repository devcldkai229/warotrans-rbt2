#!/usr/bin/env bash
# ExecStart of warotrans-rbt002.service. One stack at a time, chosen by MODE in ~/.config/warotrans/rbt002.env:
#   MODE=manual  (default) mobile control UI on :8080, base_controller takes /cmd_vel from control_gateway
#   MODE=fleet   RBT-001-equivalent semantic traffic stack: EKF + AMCL + Nav2 + authority gate + robot_gateway
#                (needs BROKER=<backend laptop IP>; BUNDLE defaults to the installed map_one_v013)
set -eo pipefail
source /opt/ros/jazzy/setup.bash
source "$HOME/ros2_ws/install/setup.bash"          # sllidar_ros2
source "$HOME/warotrans_rbt2_ws/install/setup.bash"
MODE=${MODE:-manual}
MOTORS_ENABLED=${MOTORS_ENABLED:-false}
if [ "$MODE" = fleet ]; then
  BUNDLE=${BUNDLE:-$(ros2 pkg prefix warotrans_navigation)/share/warotrans_navigation/maps/map_one_v013}
  echo "[rbt2_start] FLEET mode bundle=$BUNDLE broker=${BROKER:-<none>} motors=$MOTORS_ENABLED"
  exec ros2 launch warotrans_rbt002_bringup fleet.launch.py bundle:="$BUNDLE" motors_enabled:="$MOTORS_ENABLED" \
       mqtt_host:="${BROKER:-}" robot_code:="${ROBOT_CODE:-RBT-002}"
fi
echo "[rbt2_start] MANUAL mode motors=$MOTORS_ENABLED"
exec ros2 launch warotrans_rbt002_bringup bringup.launch.py motors_enabled:="$MOTORS_ENABLED"
