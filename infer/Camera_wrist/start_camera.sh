#!/usr/bin/env bash
# Launch the wrist camera in a dedicated desktop terminal. Never starts robot motion.
set -uo pipefail

project_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
wrist_dir="$project_dir/infer/Camera_wrist"
log_dir="$wrist_dir/log"
mkdir -p -- "$log_dir"
terminal_log=$(mktemp "$log_dir/wrist_camera_terminal_$(date +%Y%m%d_%H%M%S)_XXXXXX.log")
exec > >(tee -a "$terminal_log") 2>&1
finish() {
  local status=$?
  trap - EXIT
  printf 'Camera_wrist 结束于 %s；退出码：%s；日志：%s\n' "$(date --iso-8601=seconds)" "$status" "$terminal_log"
  if (( status != 0 )); then
    echo '相机异常退出。请查看上面的错误和终端日志。'
  fi
  read -r -p '按 Enter 关闭此相机终端...' _ || true
  exit "$status"
}
trap finish EXIT

if [[ -z "${DISPLAY:-}" ]]; then
  if [[ ! -S /tmp/.X11-unix/X0 ]]; then
    echo '未找到桌面 X11 显示，无法启动相机窗口。' >&2
    exit 2
  fi
  export DISPLAY=:0
fi
if [[ -z "${XAUTHORITY:-}" && -r "/run/user/$(id -u)/gdm/Xauthority" ]]; then
  export XAUTHORITY="/run/user/$(id -u)/gdm/Xauthority"
fi

source /home/cair-jacen/anaconda3/etc/profile.d/conda.sh
conda activate camera || exit 2
# Match the working ordinary terminal; inherited Qt plugin paths can load a mismatched plugin.
unset QT_QPA_PLATFORM_PLUGIN_PATH QT_QPA_FONTDIR
cd -- "$project_dir" || exit 2

if pgrep -af '^(/[^ ]*/)?python(3)?( -u)? infer/Camera_wrist/click_follow\.py( |$)'; then
  echo 'Camera_wrist 相机进程已运行，请先在原终端按 q 或 Ctrl+C。' >&2
  exit 1
fi

printf 'Camera_wrist 启动于 %s\n终端日志：%s\n' "$(date --iso-8601=seconds)" "$terminal_log"
model_id=${CAROTID_MODEL_ID:-songjiatong/carotid-detect-c9gi9-9-rfdetr-small-t1}
python -u infer/Camera_wrist/click_follow.py --model-id "$model_id" --model-class carotid
camera_status=$?
if (( camera_status == 7 )); then
  echo '首次启动未取得完整 RGB-D；关闭旧 SDK 会话后只重新初始化一次相机。'
  sleep 1
  python -u infer/Camera_wrist/click_follow.py --model-id "$model_id" --model-class carotid
  camera_status=$?
fi
exit "$camera_status"
