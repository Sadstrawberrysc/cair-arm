#!/usr/bin/env bash
# One C++ camera owner; the existing anatomy model consumes shared RGB-D.
set -euo pipefail
collision_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
camera_dir="$collision_dir/../Camera_RT"
vision_python="${COLLISION_VISION_PYTHON:-/home/cair-jacen/anaconda3/envs/camera/bin/python}"
[[ -x "$vision_python" ]] || { echo 'Camera 环境 Python 不存在。' >&2; exit 1; }
(cd -- "$camera_dir" && "$vision_python" -B - <<'PY'
import sys
from camera_access import ensure_global_camera_free
try:
    ensure_global_camera_free()
except ValueError as error:
    print('BLOCKED: {}'.format(error), file=sys.stderr)
    raise SystemExit(2)
PY
)
session_dir=$(mktemp -d /tmp/uspilot-carotid.XXXXXXXX)
monitor_pid=''
vision_pid=''
interrupted=false
stop_children() {
    [[ -z "$monitor_pid" ]] || kill -TERM "$monitor_pid" 2>/dev/null || true
    [[ -z "$vision_pid" ]] || kill -TERM "$vision_pid" 2>/dev/null || true
}
cleanup() {
    stop_children
    [[ -z "$monitor_pid" ]] || wait "$monitor_pid" 2>/dev/null || true
    [[ -z "$vision_pid" ]] || wait "$vision_pid" 2>/dev/null || true
    rm -rf -- "$session_dir"
}
on_signal() { interrupted=true; stop_children; }
trap on_signal INT TERM HUP
trap cleanup EXIT
(
    cd -- "$camera_dir"
    exec "$vision_python" -B -u cliff_demo.py --shared-frames "$session_dir"
) &
vision_pid=$!
bash "$collision_dir/start_monitor.sh" --shared-frames "$session_dir" "$@" &
monitor_pid=$!
set +e
wait -n "$monitor_pid" "$vision_pid"
first_status=$?
if kill -0 "$monitor_pid" 2>/dev/null; then
    kill -TERM "$monitor_pid" 2>/dev/null
fi
wait "$monitor_pid"
monitor_status=$?
monitor_pid=''
if "$interrupted"; then exit 130; fi
if (( first_status != 0 && monitor_status == 0 )); then exit 1; fi
exit "$monitor_status"
