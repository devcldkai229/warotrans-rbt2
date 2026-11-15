#!/usr/bin/env bash
# Offline quality gate: pure-python tests, config guards, web tests. colcon build/test if ROS is available.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"
echo "== no hard-coded /dev/ttyACM / ttyUSB in code or config"
if grep -rnE "/dev/tty(ACM|USB)[0-9]" ros2_ws/src --include=*.py --include=*.yaml | grep -v "^.*#"; then echo "FAIL"; exit 1; fi
echo "== base_frame must be base_footprint"
if grep -rnE "base_frame[\"']?\s*[:=,]\s*[\"']?base_link" ros2_ws/src; then echo "FAIL"; exit 1; fi
echo "== every Python file compiles (ROS nodes are not imported by the pure tests)"
python3 -m compileall -q ros2_ws/src tools >/dev/null
echo "== pytest (no ROS needed)"
for p in warotrans_rrc_lite warotrans_rbt002_hardware warotrans_rbt002_description warotrans_rbt002_bringup warotrans_rbt002_control_gateway; do
  (cd "ros2_ws/src/$p" && python3 -m pytest -q test) || exit 1
done
echo "== web"
(cd web && npm test && npm run build >/dev/null)
if [ -f /opt/ros/jazzy/setup.bash ]; then
  echo "== colcon"
  # shellcheck disable=SC1091
  source /opt/ros/jazzy/setup.bash
  (cd ros2_ws && colcon build && colcon test && colcon test-result)
fi
echo "ALL CHECKS PASSED"
