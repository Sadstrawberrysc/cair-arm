#!/usr/bin/env bash
# Offline C++ planner. Defaults are a synthetic demo, never a hardware target.
set -euo pipefail
collision_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
planner_bin="${COLLISION_PLANNER_BIN:-$collision_dir/build-monitor/collision_plan}"
deps_prefix="${COLLISION_DEPS_PREFIX:-/tmp/uspilot-collision-deps/root/usr}"
if [[ ! -x "$planner_bin" ]]; then
    printf 'Planner executable missing: %s\nBuild instructions: %s/PLANNING.md\n' "$planner_bin" "$collision_dir" >&2
    exit 1
fi
if [[ -d "$deps_prefix/lib/x86_64-linux-gnu" ]]; then
    export LD_LIBRARY_PATH="$deps_prefix/lib/x86_64-linux-gnu${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
fi
printf '%s\n' 'Offline planning only. Default inputs: synthetic seven-joint demo, not RM75 or camera data.' >&2
exec "$planner_bin" --urdf "$collision_dir/config/planning.demo.urdf" \
    --request "$collision_dir/config/planning.demo.json" \
    --mesh-dir "$collision_dir/mesh" --drop-zero-area-triangles "$@"
