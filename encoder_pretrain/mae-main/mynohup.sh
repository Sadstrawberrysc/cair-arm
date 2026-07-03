#!/usr/bin/env bash
log_dir=/home/jiuan_chen/workspace/carotid_base_log/log
mkdir -p "$log_dir"

nohup bash train.sh >"$log_dir/train.log" 2>&1 &

echo "Training started. Log: $log_dir/train.log"
echo "PID: $!"
