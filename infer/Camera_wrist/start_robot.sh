#!/usr/bin/env bash
# Real hardware wrist experiment. Operator selects the point and presses b/r.
set -euo pipefail
if (( $# != 0 )); then
  echo 'Usage: bash infer/Camera_wrist/start_robot.sh (no arguments)' >&2
  exit 2
fi
wrist_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
project_dir=$(cd -- "$wrist_dir/../.." && pwd)
robot_binary="$project_dir/infer/Robot/build/main_rm75"
if [[ ! -x "$robot_binary" ]]; then
  echo 'Build infer/Robot/build/main_rm75 first.' >&2
  exit 2
fi
mkdir -p -- "$wrist_dir/log"
run_dir=$(mktemp -d "$wrist_dir/log/wrist_continuous_XXXXXXXX")
echo "Real wrist execution: no force sensor, candidate extrinsic, no total excursion limits."
echo "Continuous until Ctrl+C or fault; click and b/r still required. Logs: $run_dir"
# Export after normal exit or a controller fault, preserving its exit status.
export_simple_log() {
  if [[ -f "$run_dir/runtime.csv" ]]; then
    python3 "$wrist_dir/export_runtime_simple.py" "$run_dir/runtime.csv" ||
      echo 'Simple CSV export failed; original runtime.csv is preserved.' >&2
  fi
}
trap export_simple_log EXIT
robot_pid=''
forward_stop() {
  if [[ -n "$robot_pid" ]]; then kill -INT "$robot_pid" 2>/dev/null || true; fi
}
trap forward_stop INT TERM
"$robot_binary" \
  --execute-wrist-follow --confirm-wrist-follow \
  --wrist-no-force --wrist-candidate-trial --wrist-unlimited-excursion \
  --calibration "$project_dir/infer/Robot/build/rm75_force_calibration.json" \
  --probe-model "$project_dir/infer/Robot/model/Lprobe-IFS.STL" \
  --wrist-follow "$wrist_dir/gemini305_to_rm75_armtip.json" \
  --publish-every 1 --duration-sec 0 --runtime-log "$run_dir/runtime.csv" &
robot_pid=$!
set +e
while true; do
  wait "$robot_pid"
  robot_status=$?
  # A signal can interrupt wait before the controller has flushed its logs.
  kill -0 "$robot_pid" 2>/dev/null || break
done
exit "$robot_status"
