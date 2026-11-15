#!/usr/bin/env bash
# Build the patched RRC Lite firmware with arm-none-eabi-gcc (Linux). Does NOT flash.
#   firmware/rrc_lite_patch/build.sh [workdir]        (default ~/rrc_lite_fw)
# Needs: git, python3, gcc-arm-none-eabi (apt), make.
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
WORK="${1:-$HOME/rrc_lite_fw}"
REPO=https://github.com/OPEN-SOURCE-UPIITA/C-ROS.git
COMMIT=9396c71e8032941f181178ebced60a598c774f0f
SUB=AutoModelCar/stm/RosRobotControllerLite_ros_250811

if [ ! -d "$WORK/src/.git" ]; then
  mkdir -p "$WORK"
  git clone -q --filter=blob:none --no-checkout "$REPO" "$WORK/src"
  git -C "$WORK/src" sparse-checkout set "$SUB"
fi
git -C "$WORK/src" checkout -q -f "$COMMIT"
git -C "$WORK/src" clean -qfdx "$SUB"
FW="$WORK/src/$SUB"

python3 "$HERE/apply_patch.py" "$FW"
python3 "$HERE/gen_sources.py" "$FW" > "$FW/sources.mk"
make -C "$FW" -f "$HERE/Makefile" PATCH_DIR="$HERE/" -j"$(nproc)" 2>&1 | grep -E "error|Error|warning: implicit|FLASH|RAM|text" || true
BIN="$FW/build_gcc/rrc_lite_warotrans.bin"
[ -f "$BIN" ] || { echo "BUILD FAILED"; exit 1; }
cp "$BIN" "$WORK/"
echo "built $WORK/rrc_lite_warotrans.bin  $(stat -c %s "$BIN") bytes  sha256 $(sha256sum "$BIN" | cut -d' ' -f1)"
