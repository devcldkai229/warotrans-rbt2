#!/usr/bin/env bash
# Restart warotrans-rbt002 when the wall clock jumps (no RTC battery: NTP may sync long after boot).
# 2026-11-15: nodes started before a ~4 h jump lost DDS contact ~5 min later (rrc_lite fine, base_controller
# saw link/battery stale, UI service calls timed out). A restart after the jump cured it (9 min clean run).
SERVICE=warotrans-rbt002
THRESH=30
offset() { echo $(( $(date +%s) - $(cut -d. -f1 /proc/uptime) )); }
base=""
since=""
while true; do
  started=$(systemctl show -p ActiveEnterTimestampMonotonic --value "$SERVICE")
  if [ "$(systemctl is-active "$SERVICE")" = active ]; then
    if [ "$started" != "$since" ]; then since="$started"; base=$(offset); fi
    now=$(offset)
    drift=$(( now - base )); drift=${drift#-}
    if [ "$drift" -gt "$THRESH" ]; then
      echo "wall clock jumped ${drift}s since $SERVICE started: restarting it"
      systemctl restart "$SERVICE"
      since=""
    fi
  fi
  sleep 5
done
