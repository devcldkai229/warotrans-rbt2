#!/usr/bin/env bash
# Read-only health check, run ON the Pi. Sends nothing to the RRC Lite.
set -u
ok() { echo "  [ OK ] $*"; }; bad() { echo "  [FAIL] $*"; }; warn() { echo "  [WARN] $*"; }
echo "== devices"
RRC=$(ls /dev/serial/by-id/usb-1a86_* 2>/dev/null | head -1); [ -n "$RRC" ] && ok "RRC Lite $RRC" || bad "RRC Lite (1a86) not found"
LID=$(ls /dev/serial/by-id/usb-Silicon_Labs_CP2102N* 2>/dev/null | head -1); [ -n "$LID" ] && ok "LiDAR $LID" || bad "LiDAR (CP2102N) not found"
id -nG | grep -qw dialout && ok "user in dialout" || bad "user not in dialout"
if [ -n "$RRC" ]; then
  OWN=$(fuser "$RRC" 2>/dev/null); [ -z "$OWN" ] && ok "RRC port free" || warn "RRC port held by pid(s):$OWN (driver running?)"
fi
echo "== software"
source /opt/ros/jazzy/setup.bash 2>/dev/null && ok "ROS 2 Jazzy" || bad "ROS 2 Jazzy"
[ -f ~/ros2_ws/install/setup.bash ] && source ~/ros2_ws/install/setup.bash
ros2 pkg prefix sllidar_ros2 >/dev/null 2>&1 && ok "sllidar_ros2" || warn "sllidar_ros2 not found (source ~/ros2_ws)"
[ -f ~/warotrans_rbt2_ws/install/setup.bash ] && source ~/warotrans_rbt2_ws/install/setup.bash && ok "warotrans_rbt2_ws built" || warn "warotrans_rbt2_ws not built"
python3 -c "import serial" 2>/dev/null && ok "pyserial" || bad "pyserial (apt: python3-serial)"
python3 -c "import fastapi, uvicorn, websockets" 2>/dev/null && ok "fastapi/uvicorn/websockets" || warn "gateway deps missing (apt: python3-fastapi python3-uvicorn python3-websockets)"
echo "== power / clock"
command -v vcgencmd >/dev/null && echo "  throttled: $(vcgencmd get_throttled 2>/dev/null)"
timedatectl 2>/dev/null | grep -E "synchronized" | sed 's/^/  /'
echo "== running graph (if launched)"
timeout 4 ros2 topic echo --once /rrc_lite/status --field link_ok 2>/dev/null | head -1 | sed 's/^/  rrc link_ok: /' || true
timeout 4 ros2 topic echo --once /safety_state --field mode_name 2>/dev/null | head -1 | sed 's/^/  mode: /' || true
