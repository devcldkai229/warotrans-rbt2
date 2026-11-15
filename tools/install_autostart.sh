#!/usr/bin/env bash
# Run ON the Pi:  tools/install_autostart.sh   (needs sudo). Creates the env file with a random UI token once.
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
ENV=/home/waro/.config/warotrans/rbt002.env
mkdir -p "$(dirname "$ENV")"
if [ ! -f "$ENV" ]; then
  umask 077
  printf 'WARO_UI_TOKEN=%s\nMOTORS_ENABLED=true\n' "$(head -c 12 /dev/urandom | base64 | tr -d '/+=' | head -c 16)" > "$ENV"
fi
sudo install -m 644 "$HERE/warotrans-rbt002.service" /etc/systemd/system/warotrans-rbt002.service
sudo systemctl daemon-reload
sudo systemctl enable --now warotrans-rbt002.service
echo "UI: http://$(hostname -I | awk '{print $1}'):8080/?token=$(grep WARO_UI_TOKEN "$ENV" | cut -d= -f2)"
