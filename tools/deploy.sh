#!/usr/bin/env bash
# Sync ros2_ws/src (+ built UI) to the Pi and build there. Run from Linux/WSL.
#   tools/deploy.sh [--pi waro@192.168.137.200] [--no-build]
# Writes ONLY to ~/warotrans_rbt2_ws on the Pi (previous src kept in ~/warotrans_rbt2_backups).
set -euo pipefail
PI=${PI:-waro@192.168.137.200}
BUILD=1
while [ $# -gt 0 ]; do
  case "$1" in
    --pi) PI=$2; shift 2 ;;
    --no-build) BUILD=0; shift ;;
    *) echo "unknown arg $1"; exit 2 ;;
  esac
done
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
[ -d "$ROOT/ros2_ws/src/warotrans_rbt002_control_gateway/web" ] || "$ROOT/tools/build_web.sh"
STAMP=$(date +%Y%m%d-%H%M%S)
echo "deploying to $PI:~/warotrans_rbt2_ws (backup $STAMP)"
tar -C "$ROOT/ros2_ws" --exclude=__pycache__ --exclude=.pytest_cache -czf - src | ssh "$PI" "
  set -e
  mkdir -p ~/warotrans_rbt2_ws ~/warotrans_rbt2_backups
  if [ -d ~/warotrans_rbt2_ws/src ]; then mv ~/warotrans_rbt2_ws/src ~/warotrans_rbt2_backups/src-$STAMP; fi
  tar -C ~/warotrans_rbt2_ws -xzf -
  # the UI asset names change every build; stale symlinks from --symlink-install break colcon
  rm -rf ~/warotrans_rbt2_ws/build/warotrans_rbt002_control_gateway ~/warotrans_rbt2_ws/install/warotrans_rbt002_control_gateway
"
if [ "$BUILD" = 1 ]; then
  ssh "$PI" "source /opt/ros/jazzy/setup.bash && { [ -f ~/ros2_ws/install/setup.bash ] && source ~/ros2_ws/install/setup.bash || true; } && cd ~/warotrans_rbt2_ws && colcon build --symlink-install 2>&1 | tail -5"
fi
echo "done. On the Pi: source ~/warotrans_rbt2_ws/install/setup.bash && ros2 launch warotrans_rbt002_bringup bringup.launch.py"
