#!/usr/bin/env bash
# Build the React UI and stage it into the gateway package so colcon installs it.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT/web"
[ -d node_modules ] || npm ci --no-audit --no-fund
npm run build
DEST="$ROOT/ros2_ws/src/warotrans_rbt002_control_gateway/web"
rm -rf "$DEST"
cp -r dist "$DEST"
echo "UI staged at $DEST"
