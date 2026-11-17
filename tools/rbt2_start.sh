#!/usr/bin/env bash
# ExecStart of warotrans-rbt002.service. One stack at a time, chosen by MODE in ~/.config/warotrans/rbt002.env:
#   MODE=manual  (default) mobile control UI on :8080, base_controller takes /cmd_vel from control_gateway
#   MODE=fleet   RBT-001-equivalent semantic traffic stack: EKF + AMCL + Nav2 + authority gate + robot_gateway
#                BROKER=auto (default) finds the backend laptop by name / ICS address / LAN scan
#                (tools/broker_discovery.py); BROKER=<ip|name> pins it. BUNDLE defaults to map_one_v014.
set -eo pipefail
source /opt/ros/jazzy/setup.bash
source "$HOME/ros2_ws/install/setup.bash"          # sllidar_ros2
source "$HOME/warotrans_rbt2_ws/install/setup.bash"
MODE=${MODE:-manual}
MOTORS_ENABLED=${MOTORS_ENABLED:-false}
# Own DDS domain: RBT-001 uses 0 on the same LAN (laptop RViz / ros2 CLI on 0 would see RBT-001 only).
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-31}
STATE="$HOME/.cache/warotrans/broker"     # what warotrans-rbt002-broker-watch compares against
mkdir -p "$(dirname "$STATE")"
if [ "$MODE" = fleet ]; then
  # The laptop's address changes with the network: resolve it at every start (60 s, then start without MQTT;
  # the watcher restarts the stack once the backend shows up or moves).
  if BROKER=$(BROKER="${BROKER:-auto}" python3 "$(dirname "$0")/broker_discovery.py" resolve --retry-s 60); then
    echo "$BROKER" > "$STATE"
  else
    BROKER=""; echo none > "$STATE"
  fi
  BUNDLE=${BUNDLE:-$(ros2 pkg prefix warotrans_navigation)/share/warotrans_navigation/maps/map_one_v014}
  echo "[rbt2_start] FLEET mode bundle=$BUNDLE broker=${BROKER:-<none>} motors=$MOTORS_ENABLED domain=$ROS_DOMAIN_ID sim_battery=${SIM_BATTERY_PERCENT:-off}"
  # ros2 launch rejects an empty "name:=" (2026-11-17: the unit restarted 2770 times on simulated_battery_percent:=),
  # so optional arguments are passed only when set.
  ARGS=(bundle:="$BUNDLE" motors_enabled:="$MOTORS_ENABLED" robot_code:="${ROBOT_CODE:-RBT-002}")
  if [ -n "${BROKER:-}" ]; then ARGS+=(mqtt_host:="$BROKER"); fi
  if [ -n "${SIM_BATTERY_PERCENT:-}" ]; then ARGS+=(simulated_battery_percent:="$SIM_BATTERY_PERCENT"); fi
  exec ros2 launch warotrans_rbt002_bringup fleet.launch.py "${ARGS[@]}"
fi
rm -f "$STATE"
echo "[rbt2_start] MANUAL mode motors=$MOTORS_ENABLED"
exec ros2 launch warotrans_rbt002_bringup bringup.launch.py motors_enabled:="$MOTORS_ENABLED"
