#!/usr/bin/env bash
# Open one wrist component in an independent, visible terminal from Runme or a shell.
set -euo pipefail

if (( $# != 1 )); then
  echo '用法：bash infer/Camera_wrist/launch_wrist_terminal.sh robot|camera' >&2
  exit 2
fi

wrist_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
project_dir=$(cd -- "$wrist_dir/../.." && pwd)
log_dir="$wrist_dir/log"
mkdir -p -- "$log_dir"

case "$1" in
  robot)
    title='Camera_wrist Robot'
    runner="$wrist_dir/start_robot_terminal.sh"
    launcher_log="$log_dir/wrist_robot_launcher.log"
    process_pattern='^(/[^ ]*/|\./)?main_rm75( |$)'
    duplicate_message='已有 RM75 控制进程运行，请先在原终端正常停止。'
    ;;
  camera)
    title='Camera_wrist'
    runner="$wrist_dir/start_camera.sh"
    launcher_log="$log_dir/wrist_camera_launcher.log"
    process_pattern='^(/[^ ]*/)?python(3)?( -u)? infer/Camera_wrist/click_follow\.py( |$)'
    duplicate_message='Camera_wrist 相机进程已运行，请先在原终端按 q 或 Ctrl+C。'
    ;;
  *)
    echo '用法：bash infer/Camera_wrist/launch_wrist_terminal.sh robot|camera' >&2
    exit 2
    ;;
esac

if pgrep -af "$process_pattern"; then
  echo "$duplicate_message" >&2
  exit 1
fi
command -v terminator >/dev/null || { echo '需要安装 Terminator。' >&2; exit 2; }
command -v setsid >/dev/null || { echo '需要安装 setsid。' >&2; exit 2; }

nohup setsid -f terminator --no-dbus --title="$title" \
  --working-directory="$project_dir" --command="bash $runner" \
  >"$launcher_log" 2>&1 </dev/null
echo "已打开 $title 独立终端；日志位于 $log_dir。"
