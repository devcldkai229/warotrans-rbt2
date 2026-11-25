#!/usr/bin/env python3
"""Copy RBT-001's hardware-independent ROS packages into this repo, unchanged.

    python tools/sync_from_rbt1.py [--src ../warotrans] [--dry-run]

Rules (docs/adr/0002-core-copied-from-rbt1.md):
  * the copied packages are NEVER edited here; RBT-002 differences live in
    ros2_ws/src/warotrans_rbt002_bringup/config/rbt002/*.yaml (overlays) and launch arguments;
  * a value that RBT-002 needs but RBT-001 hardcodes is made a parameter in the warotrans repo, then synced;
  * every sync appends the source commit (and whether its tree was dirty) to UPSTREAM.md.
"""

from __future__ import annotations

import argparse
import datetime as dt
import filecmp
import os
import shutil
import subprocess
import sys

CORE = ["warotrans_navigation", "warotrans_localization", "warotrans_slam", "warotrans_lane_planner", "warotrans_fleet", "warotrans_msgs",
        "warotrans_fleet_layer",
        # Evidence recording (camera owner + agent), same code on both robots; RBT-002 differences are parameters.
        "warotrans_camera", "warotrans_evidence_agent"]
# Map bundles to carry (the current operational map). Raw SLAM maps and old bundles stay in RBT-001's repo.
MAP_BUNDLES = ["map_one_v014", "map_one_v019", "map_one_v021", "map_one_v022"]
# Single files outside the core packages that core code imports at runtime/test time:
# path_policy_validator and export_semantic_bundle.py load semantic_map_adapter.py from
# <src>/warotrans_simulation/scripts (not a package here: COLCON_IGNORE).
EXTRA_FILES = ["warotrans_simulation/scripts/semantic_map_adapter.py"]
# Robot-side tools shared by both robots (network, clock, backend discovery): copied into tools/ unchanged.
TOOL_FILES = ["broker_discovery.py", "test_broker_discovery.py", "pi_network_setup.sh", "install_chrony.sh"]
SKIP_DIRS = {"__pycache__", ".pytest_cache", "build", "install", "log"}
SKIP_SUFFIX = (".pyc",)

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
DST_SRC = os.path.join(ROOT, "ros2_ws", "src")


def wanted(pkg: str, rel: str) -> bool:
    parts = rel.replace("\\", "/").split("/")
    if any(p in SKIP_DIRS for p in parts) or rel.endswith(SKIP_SUFFIX):
        return False
    if pkg == "warotrans_navigation" and parts[0] == "maps":
        return len(parts) > 1 and parts[1] in MAP_BUNDLES
    return True


def files_of(pkg_root: str, pkg: str) -> set[str]:
    out = set()
    for d, dirs, files in os.walk(pkg_root):
        dirs[:] = [x for x in dirs if x not in SKIP_DIRS]
        for f in files:
            rel = os.path.relpath(os.path.join(d, f), pkg_root)
            if wanted(pkg, rel):
                out.add(rel)
    return out


def git(src_repo: str, *args: str) -> str:
    try:
        return subprocess.run(["git", "-C", src_repo, *args], capture_output=True, text=True, check=True).stdout.strip()
    except (OSError, subprocess.CalledProcessError):
        return "?"


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--src", default=os.path.join(os.path.dirname(ROOT), "warotrans"))
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()
    src_src = os.path.join(args.src, "ros2_ws", "src")
    if not os.path.isdir(src_src):
        print(f"not found: {src_src}")
        return 2

    added = changed = removed = 0
    for pkg in CORE:
        s_root, d_root = os.path.join(src_src, pkg), os.path.join(DST_SRC, pkg)
        s_files = files_of(s_root, pkg)
        d_files = files_of(d_root, pkg) if os.path.isdir(d_root) else set()
        for rel in sorted(s_files):
            sp, dp = os.path.join(s_root, rel), os.path.join(d_root, rel)
            if rel not in d_files:
                added += 1
                print(f"  + {pkg}/{rel}")
            elif not filecmp.cmp(sp, dp, shallow=False):
                changed += 1
                print(f"  ~ {pkg}/{rel}")
            else:
                continue
            if not args.dry_run:
                os.makedirs(os.path.dirname(dp), exist_ok=True)
                shutil.copy2(sp, dp)
        for rel in sorted(d_files - s_files):
            removed += 1
            print(f"  - {pkg}/{rel}")
            if not args.dry_run:
                os.remove(os.path.join(d_root, rel))

    for rel in EXTRA_FILES:
        sp, dp = os.path.join(src_src, rel), os.path.join(DST_SRC, rel)
        if not os.path.isfile(dp) or not filecmp.cmp(sp, dp, shallow=False):
            changed += 1
            print(f"  ~ {rel}")
            if not args.dry_run:
                os.makedirs(os.path.dirname(dp), exist_ok=True)
                shutil.copy2(sp, dp)
                ignore = os.path.join(DST_SRC, rel.split("/")[0], "COLCON_IGNORE")
                open(ignore, "a").close()

    for name in TOOL_FILES:
        sp, dp = os.path.join(args.src, "tools", name), os.path.join(HERE, name)
        if not os.path.isfile(dp) or not filecmp.cmp(sp, dp, shallow=False):
            changed += 1
            print(f"  ~ tools/{name}")
            if not args.dry_run:
                shutil.copy2(sp, dp)

    commit = git(args.src, "rev-parse", "--short", "HEAD")
    branch = git(args.src, "branch", "--show-current")
    dirty = git(args.src, "status", "--porcelain", "--", *[f"ros2_ws/src/{p}" for p in CORE])
    print(f"\n{added} added, {changed} changed, {removed} removed (source {branch}@{commit}"
          f"{', DIRTY working tree' if dirty and dirty != '?' else ''})")
    if not args.dry_run and (added or changed or removed):
        with open(os.path.join(ROOT, "UPSTREAM.md"), "a", encoding="utf-8") as f:
            f.write(f"| {dt.date.today()} | core sync | `{branch}@{commit}`"
                    f"{' + working tree chưa commit' if dirty and dirty != '?' else ''}"
                    f" | +{added} ~{changed} -{removed} | {', '.join(CORE)}; maps {', '.join(MAP_BUNDLES)} |\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
