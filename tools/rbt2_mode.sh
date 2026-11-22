#!/usr/bin/env bash
# Switch RBT-002 between the manual UI stack and the fleet stack (run ON the Pi), then restart the service.
#   tools/rbt2_mode.sh manual
#   tools/rbt2_mode.sh fleet [auto|<broker-ip-or-name>] [bundle-dir]     auto (default): found at every start
#   tools/rbt2_mode.sh cloud [bundle-dir]          AWS IoT Core + cloud Recorder (~/.config/warotrans/cloud.env)
#   tools/rbt2_mode.sh sim-battery <0..100>|off   dry run without the motor battery (telemetry says SIMULATED)
#   tools/rbt2_mode.sh motors true|false          only with someone at the robot (no hardware E-stop)
set -euo pipefail
ENV=$HOME/.config/warotrans/rbt002.env
mode=${1:?manual|fleet|sim-battery|motors}
set_kv() { grep -v "^$1=" "$ENV" > "$ENV.tmp" || true; echo "$1=$2" >> "$ENV.tmp"; mv "$ENV.tmp" "$ENV"; chmod 600 "$ENV"; }
unset_kv() { grep -v "^$1=" "$ENV" > "$ENV.tmp" || true; mv "$ENV.tmp" "$ENV"; chmod 600 "$ENV"; }
case "$mode" in
  manual) set_kv MODE manual ;;
  fleet)  set_kv MODE fleet; set_kv BROKER "${2:-auto}"; [ -n "${3:-}" ] && set_kv BUNDLE "$3" ;;
  cloud)  set_kv MODE cloud; [ -n "${2:-}" ] && set_kv BUNDLE "$2" ;;
  sim-battery)
    v=${2:?percent or off}
    if [ "$v" = off ]; then unset_kv SIM_BATTERY_PERCENT
    elif [[ "$v" =~ ^[0-9]+([.][0-9]+)?$ ]] && awk "BEGIN{exit !($v <= 100)}"; then set_kv SIM_BATTERY_PERCENT "$v"
    else echo "sim-battery needs 0..100 or off"; exit 2; fi ;;
  motors)
    case "${2:-}" in true|false) set_kv MOTORS_ENABLED "$2" ;; *) echo "motors true|false"; exit 2 ;; esac ;;
  *) echo "usage: $0 manual | fleet [auto|<broker>] [bundle] | cloud [bundle] | sim-battery <pct>|off | motors true|false"; exit 2 ;;
esac
grep -E '^(MODE|BROKER|WARO_BACKEND_HOST|BUNDLE|MOTORS_ENABLED|SIM_BATTERY_PERCENT|ROS_DOMAIN_ID)=' "$ENV" || true
sudo systemctl restart warotrans-rbt002
echo "mode=$mode; follow: journalctl -u warotrans-rbt002 -f"
