#!/usr/bin/env bash
# Start the C++ read-only monitor using this checkout's local resources.
set -euo pipefail

collision_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
monitor_bin="${COLLISION_MONITOR_BIN:-$collision_dir/build-monitor/collision_monitor}"
deps_prefix="${COLLISION_DEPS_PREFIX:-/tmp/uspilot-collision-deps/root/usr}"

if [[ ! -x "$monitor_bin" ]]; then
    printf 'Monitor executable missing: %s\nBuild instructions: %s/MONITOR.md\n' \
        "$monitor_bin" "$collision_dir" >&2
    exit 1
fi

# Prepend only existing local runtime directories; preserve the caller's paths.
for lib_dir in "$deps_prefix/lib/x86_64-linux-gnu" \
    "$collision_dir/.deps/realsense/usr/lib/x86_64-linux-gnu"; do
    if [[ -d "$lib_dir" ]]; then
        export LD_LIBRARY_PATH="$lib_dir${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
    fi
done

# Parse option positions so a value containing '--replay' is not a mode switch.
replay=false
information=false
help=false
coarse=auto
forwarded=()
remaining=("$@")
while ((${#remaining[@]})); do
    option="${remaining[0]}"
    remaining=("${remaining[@]:1}")
    if [[ "$option" == --coarse ]]; then coarse=true; continue; fi
    if [[ "$option" == --monitor-only ]]; then coarse=false; continue; fi
    forwarded+=("$option")
    case "$option" in
        --help) information=true; help=true ;;
        --sdk-info) information=true ;;
        --live|--headless|--drop-zero-area-triangles|--plan-once|--verbose) ;;
        --urdf|--calibration|--config|--replay|--record|--mesh-dir|--robot-ip|--robot-port|--max-frames|--pixel-stride|--goal)
            if ((${#remaining[@]} == 0)); then
                printf 'Missing value for %s\n' "$option" >&2
                exit 1
            fi
            if [[ "$option" == --replay ]]; then replay=true; fi
            forwarded+=("${remaining[0]}")
            remaining=("${remaining[@]:1}")
            ;;
        *) printf 'Unknown option: %s (see --help)\n' "$option" >&2; exit 1 ;;
    esac
done

if "$information"; then
    if "$help"; then
        printf '%s\n' \
            'Usage: start_monitor.sh [monitor options]' \
            'No arguments: live read-only carotid coarse planning (press P) using local resources.' \
            'Default robot: 192.168.50.254:8080; override with --robot-ip / --robot-port.' \
            'Use --replay FILE for offline playback; --sdk-info only checks SDK linkage.' \
            'Use --monitor-only for monitoring without planning; --coarse remains a compatible alias.' \
            'Replay defaults to monitoring; add --coarse or --goal to enable replay planning.' \
            'Explicit options override defaults. Native monitor options follow:'
    fi
    exec "$monitor_bin" "${forwarded[@]}"
fi

defaults=(
    --urdf "$collision_dir/rm_75.urdf"
    --mesh-dir "$collision_dir/mesh"
    --drop-zero-area-triangles
    --calibration "$collision_dir/../hand_eye_calibration-main/d455_to_rm75_base.json"
    --config "$collision_dir/config/monitor.example.json"
)
if ! "$replay"; then
    defaults+=(--live --robot-ip "${COLLISION_ROBOT_IP:-192.168.50.254}"
        --robot-port "${COLLISION_ROBOT_PORT:-8080}")
fi
if [[ "$coarse" == true ]] || { [[ "$coarse" == auto ]] && ! "$replay"; }; then
    defaults+=(--goal "$collision_dir/config/coarse.goal.example.json")
fi

# The C++ parser uses the last value, so explicit arguments override defaults.
exec "$monitor_bin" "${defaults[@]}" "${forwarded[@]}"
