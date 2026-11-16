#!/usr/bin/env bash
# Switch RBT-002 between the manual UI stack and the fleet stack (run ON the Pi).
#   tools/rbt2_mode.sh manual
#   tools/rbt2_mode.sh fleet <broker-ip> [bundle-dir]
set -euo pipefail
ENV=$HOME/.config/warotrans/rbt002.env
mode=${1:?manual|fleet}
set_kv() { grep -v "^$1=" "$ENV" > "$ENV.tmp" || true; echo "$1=$2" >> "$ENV.tmp"; mv "$ENV.tmp" "$ENV"; chmod 600 "$ENV"; }
case "$mode" in
  manual) set_kv MODE manual ;;
  fleet)  set_kv MODE fleet; set_kv BROKER "${2:?broker ip}"; [ -n "${3:-}" ] && set_kv BUNDLE "$3" ;;
  *) echo "usage: $0 manual | fleet <broker-ip> [bundle]"; exit 2 ;;
esac
sudo systemctl restart warotrans-rbt002
echo "mode=$mode; follow: journalctl -u warotrans-rbt002 -f"
