#!/usr/bin/env bash
# Visible, detached terminal for the Runme wrist Robot entry.
set -uo pipefail
project_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
log_dir="$project_dir/infer/Camera_wrist/log"
mkdir -p -- "$log_dir"
terminal_log=$(mktemp "$log_dir/wrist_robot_terminal_$(date +%Y%m%d_%H%M%S)_XXXXXX.log")
exec > >(tee -a "$terminal_log") 2>&1
finish() {
  local status=$?
  trap - EXIT
  printf 'Camera_wrist Robot 结束于 %s；退出码：%s；日志：%s\n' "$(date --iso-8601=seconds)" "$status" "$terminal_log"
  read -r -p '按 Enter 关闭此机器人终端...' _ || true
  exit "$status"
}
trap finish EXIT
cd -- "$project_dir" || exit 2
if pgrep -af '^(/[^ ]*/|\./)?main_rm75( |$)'; then
  echo '已有 RM75 控制进程运行，请先在原终端正常停止。' >&2
  exit 1
fi
printf 'Camera_wrist Robot 启动于 %s\n终端日志：%s\n' "$(date --iso-8601=seconds)" "$terminal_log"
bash infer/Camera_wrist/start_robot.sh
exit $?
