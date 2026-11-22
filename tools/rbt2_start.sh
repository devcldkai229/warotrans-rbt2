#!/usr/bin/env bash
# ExecStart of warotrans-rbt002.service. One stack at a time, chosen by MODE in ~/.config/warotrans/rbt002.env:
#   MODE=manual  (default) mobile control UI on :8080, base_controller takes /cmd_vel from control_gateway
#   MODE=fleet   RBT-001-equivalent semantic traffic stack: EKF + AMCL + Nav2 + authority gate + robot_gateway
#                BROKER=auto (default) finds the backend laptop by name / ICS address / LAN scan
#                (tools/broker_discovery.py); BROKER=<ip|name> pins it. BUNDLE defaults to map_one_v014.
#   MODE=cloud   the fleet stack against AWS: MQTT over TLS to AWS IoT Core with this robot's thing certificate,
#                evidence to the cloud Recorder; settings in ~/.config/warotrans/cloud.env (tools/cloud.env.example).
set -eo pipefail
source /opt/ros/jazzy/setup.bash
source "$HOME/ros2_ws/install/setup.bash"          # sllidar_ros2
source "$HOME/warotrans_rbt2_ws/install/setup.bash"
MODE=${MODE:-manual}
# Fast DDS shared-memory segments of killed processes stay in /dev/shm; 2026-11-18 ~120 of them (after SIGKILLed
# calibration stacks) made robot_gateway receive nothing (telemetry pose/battery null). "shm clean" removes only
# segments no live process holds.
timeout 20 fastdds shm clean >/dev/null 2>&1 || true
MOTORS_ENABLED=${MOTORS_ENABLED:-false}
# Own DDS domain: RBT-001 uses 0 on the same LAN (laptop RViz / ros2 CLI on 0 would see RBT-001 only).
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-31}
STATE="$HOME/.cache/warotrans/broker"     # what warotrans-rbt002-broker-watch compares against
mkdir -p "$(dirname "$STATE")"
RECORDER_URL=""
if [ "$MODE" = cloud ]; then
  CLOUD_ENV="$HOME/.config/warotrans/cloud.env"
  [ -f "$CLOUD_ENV" ] || { echo "[rbt2_start] MODE=cloud needs $CLOUD_ENV"; exit 2; }
  set -a; source "$CLOUD_ENV"; set +a
  IOT_DIR="${WARO_IOT_DIR:-$HOME/.config/warotrans/iot}"
  CODE="${ROBOT_CODE:-RBT-002}"
  export WARO_MQTT_TLS=true WARO_MQTT_PORT="${WARO_MQTT_PORT:-8883}"
  export WARO_MQTT_CERT_FILE="${WARO_MQTT_CERT_FILE:-$IOT_DIR/$CODE.cert.pem}"
  export WARO_MQTT_KEY_FILE="${WARO_MQTT_KEY_FILE:-$IOT_DIR/$CODE.private.key}"
  export WARO_MQTT_CA_FILE="${WARO_MQTT_CA_FILE:-$IOT_DIR/AmazonRootCA1.pem}"
  for f in "$WARO_MQTT_CERT_FILE" "$WARO_MQTT_KEY_FILE" "$WARO_MQTT_CA_FILE"; do
    [ -f "$f" ] || { echo "[rbt2_start] MODE=cloud: missing $f"; exit 2; }
  done
  [ -n "${WARO_MQTT_HOST:-}" ] && [ -n "${WARO_RECORDER_URL:-}" ] \
    || { echo "[rbt2_start] $CLOUD_ENV must set WARO_MQTT_HOST and WARO_RECORDER_URL"; exit 2; }
  BROKER="$WARO_MQTT_HOST"; RECORDER_URL="$WARO_RECORDER_URL"
  rm -f "$STATE"    # IoT Core has a fixed name: the broker watcher has nothing to follow
fi
if [ "$MODE" = fleet ]; then
  # The laptop's address changes with the network: resolve it at every start (60 s, then start without MQTT;
  # the watcher restarts the stack once the backend shows up or moves).
  if BROKER=$(BROKER="${BROKER:-auto}" python3 "$(dirname "$0")/broker_discovery.py" resolve --retry-s 60); then
    echo "$BROKER" > "$STATE"
  else
    BROKER=""; echo none > "$STATE"
  fi
fi
if [ "$MODE" = fleet ] || [ "$MODE" = cloud ]; then
  BUNDLE=${BUNDLE:-$(ros2 pkg prefix warotrans_navigation)/share/warotrans_navigation/maps/map_one_v014}
  echo "[rbt2_start] $MODE mode bundle=$BUNDLE broker=${BROKER:-<none>} motors=$MOTORS_ENABLED domain=$ROS_DOMAIN_ID sim_battery=${SIM_BATTERY_PERCENT:-off}"
  # ros2 launch rejects an empty "name:=" (2026-11-17: the unit restarted 2770 times on simulated_battery_percent:=),
  # so optional arguments are passed only when set.
  ARGS=(bundle:="$BUNDLE" motors_enabled:="$MOTORS_ENABLED" robot_code:="${ROBOT_CODE:-RBT-002}")
  if [ -n "${BROKER:-}" ]; then ARGS+=(mqtt_host:="$BROKER"); fi
  if [ -n "$RECORDER_URL" ]; then ARGS+=(recorder_url:="$RECORDER_URL"); fi
  if [ -n "${SIM_BATTERY_PERCENT:-}" ]; then ARGS+=(simulated_battery_percent:="$SIM_BATTERY_PERCENT"); fi
  exec ros2 launch warotrans_rbt002_bringup fleet.launch.py "${ARGS[@]}"
fi
rm -f "$STATE"
echo "[rbt2_start] MANUAL mode motors=$MOTORS_ENABLED"
exec ros2 launch warotrans_rbt002_bringup bringup.launch.py motors_enabled:="$MOTORS_ENABLED"
