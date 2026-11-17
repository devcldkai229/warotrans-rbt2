#!/usr/bin/env bash
# Run ON the Pi:  tools/install_autostart.sh   (needs sudo). Creates the env file with a random UI token once.
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
ENV=/home/waro/.config/warotrans/rbt002.env
mkdir -p "$(dirname "$ENV")"
if [ ! -f "$ENV" ]; then
  umask 077
  # Motors stay off until someone turns them on deliberately (tools/rbt2_mode.sh / the env file); own DDS domain.
  printf 'WARO_UI_TOKEN=%s\nMOTORS_ENABLED=false\nROS_DOMAIN_ID=31\n' "$(head -c 12 /dev/urandom | base64 | tr -d '/+=' | head -c 16)" > "$ENV"
fi
# logind's default RemoveIPC=yes deletes every /dev/shm file of user waro when its last SSH session ends. The stack (also
# user waro) keeps running, but Fast DDS shared memory of its nodes is gone: any process started later (ros2 topic echo,
# tools/rbt2_readiness.py, fleet_probe.py) discovers the topics and never gets a message (seen 2026-11-17).
sudo mkdir -p /etc/systemd/logind.conf.d
printf '[Login]\n# ROS 2 shared memory of the warotrans-rbt002 service must survive SSH logouts.\nRemoveIPC=no\n' |
  sudo tee /etc/systemd/logind.conf.d/warotrans-keep-ipc.conf >/dev/null
sudo systemctl restart systemd-logind
sudo install -m 644 "$HERE/warotrans-rbt002.service" /etc/systemd/system/warotrans-rbt002.service
sudo systemctl daemon-reload
sudo systemctl enable --now warotrans-rbt002.service
echo "UI: http://$(hostname -I | awk '{print $1}'):8080/?token=$(grep WARO_UI_TOKEN "$ENV" | cut -d= -f2)"
